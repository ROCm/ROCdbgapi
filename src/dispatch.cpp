/* Copyright (c) 2019-2020 Advanced Micro Devices, Inc.

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 THE SOFTWARE. */

#include "defs.h"

#include "agent.h"
#include "architecture.h"
#include "debug.h"
#include "dispatch.h"
#include "logging.h"
#include "process.h"
#include "queue.h"
#include "utils.h"

#include <hsa/amd_hsa_kernel_code.h>

#include <cstring>

namespace amd
{
namespace dbgapi
{

dispatch_t::dispatch_t (amd_dbgapi_dispatch_id_t dispatch_id, queue_t &queue,
                        amd_dbgapi_queue_packet_id_t queue_packet_id,
                        amd_dbgapi_global_address_t packet_address)
    : handle_object (dispatch_id), m_queue_packet_id (queue_packet_id),
      m_queue (queue)
{
  if (process ().read_global_memory (packet_address, &m_packet,
                                     sizeof (m_packet))
      != AMD_DBGAPI_STATUS_SUCCESS)
    error ("Could not read the dispatch's packet");

  if (process ().read_global_memory (m_packet.kernel_object,
                                     &m_kernel_descriptor,
                                     sizeof (m_kernel_descriptor))
      != AMD_DBGAPI_STATUS_SUCCESS)
    error ("Could not read the dispatch's kernel descriptor");
}

amd_dbgapi_global_address_t
dispatch_t::kernel_entry_address () const
{
  return m_packet.kernel_object
         + m_kernel_descriptor.kernel_code_entry_byte_offset;
}

bool
dispatch_t::scratch_enabled () const
{
  return !!(
      m_kernel_descriptor.compute_pgm_rsrc2
      & AMD_COMPUTE_PGM_RSRC_TWO_ENABLE_SGPR_PRIVATE_SEGMENT_WAVE_BYTE_OFFSET);
}

amd_dbgapi_status_t
dispatch_t::get_info (amd_dbgapi_dispatch_info_t query, size_t value_size,
                      void *value) const
{
  switch (query)
    {
    case AMD_DBGAPI_DISPATCH_INFO_QUEUE:
      return utils::get_info (value_size, value, queue ().id ());

    case AMD_DBGAPI_DISPATCH_INFO_AGENT:
      return utils::get_info (value_size, value, agent ().id ());

    case AMD_DBGAPI_DISPATCH_INFO_ARCHITECTURE:
      return utils::get_info (value_size, value, architecture ().id ());

    case AMD_DBGAPI_DISPATCH_INFO_KERNEL_ENTRY_ADDRESS:
      return utils::get_info (value_size, value, kernel_entry_address ());

    case AMD_DBGAPI_DISPATCH_INFO_WORK_GROUP_SIZES:
      return utils::get_info (value_size, value,
                              (uint16_t[3]){ m_packet.workgroup_size_x,
                                             m_packet.workgroup_size_y,
                                             m_packet.workgroup_size_z });

    case AMD_DBGAPI_DISPATCH_INFO_GRID_SIZES:
      return utils::get_info (value_size, value, m_packet.grid_size_x);

    default:
      return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;
    }

  return AMD_DBGAPI_STATUS_SUCCESS;
}

} /* namespace dbgapi */
} /* namespace amd */

using namespace amd::dbgapi;

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_dispatch_get_info (amd_dbgapi_process_id_t process_id,
                              amd_dbgapi_dispatch_id_t dispatch_id,
                              amd_dbgapi_dispatch_info_t query,
                              size_t value_size, void *value)
{
  TRY;
  TRACE (process_id, dispatch_id, query, value_size, value);

  process_t *process = process_t::find (process_id);

  if (!process)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_PROCESS_ID;

  dispatch_t *dispatch = process->find (dispatch_id);

  if (!dispatch)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_DISPATCH_ID;

  return dispatch->get_info (query, value_size, value);
  CATCH;
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_dispatch_list (amd_dbgapi_process_id_t process_id,
                          size_t *dispatch_count,
                          amd_dbgapi_dispatch_id_t **dispatches,
                          amd_dbgapi_changed_t *changed)
{
  TRY;
  TRACE (process_id);

  if (!amd::dbgapi::is_initialized)
    return AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED;

  return utils::get_handle_list<dispatch_t> (process_id, dispatch_count,
                                             dispatches, changed);
  CATCH;
}
