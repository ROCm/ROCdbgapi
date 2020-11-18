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

#include "dispatch.h"
#include "agent.h"
#include "architecture.h"
#include "debug.h"
#include "initialization.h"
#include "logging.h"
#include "process.h"
#include "queue.h"
#include "utils.h"

#include <hsa/amd_hsa_kernel_code.h>

#include <utility>
#include <vector>

namespace amd::dbgapi
{

dispatch_t::dispatch_t (amd_dbgapi_dispatch_id_t dispatch_id, queue_t &queue,
                        amd_dbgapi_os_queue_packet_id_t os_queue_packet_id,
                        amd_dbgapi_global_address_t packet_address)
    : handle_object (dispatch_id), m_os_queue_packet_id (os_queue_packet_id),
      m_queue (queue)
{
  amd_dbgapi_status_t status;

  status = process ().read_global_memory (packet_address, &m_packet,
                                          sizeof (m_packet));
  if (status != AMD_DBGAPI_STATUS_SUCCESS)
    error ("Could not read the dispatch's packet (rc=%d)", status);

  status = process ().read_global_memory (m_packet.kernel_object,
                                          &m_kernel_descriptor,
                                          sizeof (m_kernel_descriptor));
  if (status != AMD_DBGAPI_STATUS_SUCCESS)
    error ("Could not read the dispatch's kernel descriptor (rc=%d)", status);
}

amd_dbgapi_global_address_t
dispatch_t::kernel_descriptor_address () const
{
  return m_packet.kernel_object;
}

amd_dbgapi_global_address_t
dispatch_t::kernel_code_entry_address () const
{
  return m_packet.kernel_object
         + m_kernel_descriptor.kernel_code_entry_byte_offset;
}

bool
dispatch_t::is_scratch_enabled () const
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

    case AMD_DBGAPI_DISPATCH_INFO_PROCESS:
      return utils::get_info (value_size, value, process ().id ());

    case AMD_DBGAPI_DISPATCH_INFO_ARCHITECTURE:
      return utils::get_info (value_size, value, architecture ().id ());

    case AMD_DBGAPI_DISPATCH_INFO_OS_QUEUE_PACKET_ID:
      return utils::get_info (value_size, value, m_os_queue_packet_id);

    case AMD_DBGAPI_DISPATCH_INFO_BARRIER:
      return utils::get_info (value_size, value,
                              utils::bit_extract (m_packet.header,
                                                  HSA_PACKET_HEADER_BARRIER,
                                                  HSA_PACKET_HEADER_BARRIER)
                                  ? AMD_DBGAPI_DISPATCH_BARRIER_PRESENT
                                  : AMD_DBGAPI_DISPATCH_BARRIER_NONE);

    case AMD_DBGAPI_DISPATCH_INFO_ACQUIRE_FENCE:
      static_assert ((int)AMD_DBGAPI_DISPATCH_FENCE_SCOPE_NONE
                         == (int)HSA_FENCE_SCOPE_NONE,
                     "amd_dbgapi_dispatch_fence_scope_t != hsa_fence_scope_t");
      static_assert ((int)AMD_DBGAPI_DISPATCH_FENCE_SCOPE_AGENT
                         == (int)HSA_FENCE_SCOPE_AGENT,
                     "amd_dbgapi_dispatch_fence_scope_t != hsa_fence_scope_t");
      static_assert ((int)AMD_DBGAPI_DISPATCH_FENCE_SCOPE_SYSTEM
                         == (int)HSA_FENCE_SCOPE_SYSTEM,
                     "amd_dbgapi_dispatch_fence_scope_t != hsa_fence_scope_t");

      return utils::get_info (
          value_size, value,
          static_cast<amd_dbgapi_dispatch_fence_scope_t> (utils::bit_extract (
              m_packet.header, HSA_PACKET_HEADER_SCACQUIRE_FENCE_SCOPE,
              HSA_PACKET_HEADER_SCACQUIRE_FENCE_SCOPE
                  + HSA_PACKET_HEADER_WIDTH_SCACQUIRE_FENCE_SCOPE - 1)));

    case AMD_DBGAPI_DISPATCH_INFO_RELEASE_FENCE:
      return utils::get_info (
          value_size, value,
          static_cast<amd_dbgapi_dispatch_fence_scope_t> (utils::bit_extract (
              m_packet.header, HSA_PACKET_HEADER_SCRELEASE_FENCE_SCOPE,
              HSA_PACKET_HEADER_SCRELEASE_FENCE_SCOPE
                  + HSA_PACKET_HEADER_WIDTH_SCRELEASE_FENCE_SCOPE - 1)));

    case AMD_DBGAPI_DISPATCH_INFO_GRID_DIMENSIONS:
      return utils::get_info (
          value_size, value,
          static_cast<uint32_t> (utils::bit_extract (
              m_packet.setup, HSA_KERNEL_DISPATCH_PACKET_SETUP_DIMENSIONS,
              HSA_KERNEL_DISPATCH_PACKET_SETUP_DIMENSIONS
                  + HSA_KERNEL_DISPATCH_PACKET_SETUP_WIDTH_DIMENSIONS - 1)));

    case AMD_DBGAPI_DISPATCH_INFO_WORK_GROUP_SIZES:
      return utils::get_info (value_size, value,
                              (uint16_t[3]){ m_packet.workgroup_size_x,
                                             m_packet.workgroup_size_y,
                                             m_packet.workgroup_size_z });

    case AMD_DBGAPI_DISPATCH_INFO_GRID_SIZES:
      return utils::get_info (value_size, value,
                              (uint32_t[3]){ m_packet.grid_size_x,
                                             m_packet.grid_size_y,
                                             m_packet.grid_size_z });

    case AMD_DBGAPI_DISPATCH_INFO_PRIVATE_SEGMENT_SIZE:
      return utils::get_info (
          value_size, value,
          static_cast<amd_dbgapi_size_t> (m_packet.private_segment_size));

    case AMD_DBGAPI_DISPATCH_INFO_GROUP_SEGMENT_SIZE:
      return utils::get_info (
          value_size, value,
          static_cast<amd_dbgapi_size_t> (m_packet.group_segment_size));

    case AMD_DBGAPI_DISPATCH_INFO_KERNEL_ARGUMENT_SEGMENT_ADDRESS:
      return utils::get_info (value_size, value, m_packet.kernarg_address);

    case AMD_DBGAPI_DISPATCH_INFO_KERNEL_DESCRIPTOR_ADDRESS:
      return utils::get_info (value_size, value, kernel_descriptor_address ());

    case AMD_DBGAPI_DISPATCH_INFO_KERNEL_CODE_ENTRY_ADDRESS:
      return utils::get_info (value_size, value, kernel_code_entry_address ());

    case AMD_DBGAPI_DISPATCH_INFO_KERNEL_COMPLETION_ADDRESS:
      return utils::get_info (value_size, value, m_packet.completion_signal);
    }
  return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;
}

} /* namespace amd::dbgapi */

using namespace amd::dbgapi;

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_dispatch_get_info (amd_dbgapi_dispatch_id_t dispatch_id,
                              amd_dbgapi_dispatch_info_t query,
                              size_t value_size, void *value)
{
  TRY;
  TRACE (dispatch_id, query, value_size, value);

  if (!detail::is_initialized)
    return AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED;

  dispatch_t *dispatch = find (dispatch_id);

  if (!dispatch)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_DISPATCH_ID;

  return dispatch->get_info (query, value_size, value);
  CATCH;
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_process_dispatch_list (amd_dbgapi_process_id_t process_id,
                                  size_t *dispatch_count,
                                  amd_dbgapi_dispatch_id_t **dispatches,
                                  amd_dbgapi_changed_t *changed)
{
  TRY;
  TRACE (process_id);

  if (!detail::is_initialized)
    return AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED;

  std::vector<process_t *> processes;
  if (process_id != AMD_DBGAPI_PROCESS_NONE)
    {
      process_t *process = process_t::find (process_id);

      if (!process)
        return AMD_DBGAPI_STATUS_ERROR_INVALID_PROCESS_ID;

      if (amd_dbgapi_status_t status = process->update_queues ();
          status != AMD_DBGAPI_STATUS_SUCCESS)
        error ("process_t::update_agents failed (rc=%d)", status);

      processes.emplace_back (process);
    }
  else
    {
      for (auto &&process : process_t::all ())
        {
          if (amd_dbgapi_status_t status = process.update_queues ();
              status != AMD_DBGAPI_STATUS_SUCCESS)
            error ("process_t::update_agents failed (rc=%d)", status);

          processes.emplace_back (&process);
        }
    }

  std::vector<std::pair<process_t *, std::vector<queue_t *>>>
      queues_needing_resume;

  for (auto &&process : processes)
    {
      std::vector<queue_t *> queues;

      for (auto &&queue : process->range<queue_t> ())
        if (!queue.is_suspended ())
          queues.emplace_back (&queue);

      process->suspend_queues (queues, "refresh dispatch list");

      if (process->forward_progress_needed ())
        queues_needing_resume.emplace_back (process, std::move (queues));
    }

  amd_dbgapi_status_t status = utils::get_handle_list<dispatch_t> (
      processes, dispatch_count, dispatches, changed);

  for (auto &&[process, queues] : queues_needing_resume)
    process->resume_queues (queues, "refresh dispatch list");

  return status;
  CATCH;
}
