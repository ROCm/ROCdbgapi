/* Copyright (c) 2019-2021 Advanced Micro Devices, Inc.

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
#include "exception.h"
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
  /* If this is a dummy dispatch, we don't have a packet to read from.  */
  if (dispatch_id == AMD_DBGAPI_DISPATCH_NONE)
    return;

  /* Read the dispatch packet and kernel descriptor.  */
  process ().read_global_memory (packet_address, &m_packet);
  process ().read_global_memory (m_packet.kernel_object, &m_kernel_descriptor);
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

void
dispatch_t::get_info (amd_dbgapi_dispatch_info_t query, size_t value_size,
                      void *value) const
{
  switch (query)
    {
    case AMD_DBGAPI_DISPATCH_INFO_QUEUE:
      utils::get_info (value_size, value, queue ().id ());
      return;

    case AMD_DBGAPI_DISPATCH_INFO_AGENT:
      utils::get_info (value_size, value, agent ().id ());
      return;

    case AMD_DBGAPI_DISPATCH_INFO_PROCESS:
      utils::get_info (value_size, value, process ().id ());
      return;

    case AMD_DBGAPI_DISPATCH_INFO_ARCHITECTURE:
      utils::get_info (value_size, value, architecture ().id ());
      return;

    case AMD_DBGAPI_DISPATCH_INFO_OS_QUEUE_PACKET_ID:
      utils::get_info (value_size, value, m_os_queue_packet_id);
      return;

    case AMD_DBGAPI_DISPATCH_INFO_BARRIER:
      utils::get_info (value_size, value,
                       utils::bit_extract (m_packet.header,
                                           HSA_PACKET_HEADER_BARRIER,
                                           HSA_PACKET_HEADER_BARRIER)
                         ? AMD_DBGAPI_DISPATCH_BARRIER_PRESENT
                         : AMD_DBGAPI_DISPATCH_BARRIER_NONE);
      return;

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

      utils::get_info (
        value_size, value,
        static_cast<amd_dbgapi_dispatch_fence_scope_t> (utils::bit_extract (
          m_packet.header, HSA_PACKET_HEADER_SCACQUIRE_FENCE_SCOPE,
          HSA_PACKET_HEADER_SCACQUIRE_FENCE_SCOPE
            + HSA_PACKET_HEADER_WIDTH_SCACQUIRE_FENCE_SCOPE - 1)));
      return;

    case AMD_DBGAPI_DISPATCH_INFO_RELEASE_FENCE:
      utils::get_info (
        value_size, value,
        static_cast<amd_dbgapi_dispatch_fence_scope_t> (utils::bit_extract (
          m_packet.header, HSA_PACKET_HEADER_SCRELEASE_FENCE_SCOPE,
          HSA_PACKET_HEADER_SCRELEASE_FENCE_SCOPE
            + HSA_PACKET_HEADER_WIDTH_SCRELEASE_FENCE_SCOPE - 1)));
      return;

    case AMD_DBGAPI_DISPATCH_INFO_GRID_DIMENSIONS:
      utils::get_info (
        value_size, value,
        static_cast<uint32_t> (utils::bit_extract (
          m_packet.setup, HSA_KERNEL_DISPATCH_PACKET_SETUP_DIMENSIONS,
          HSA_KERNEL_DISPATCH_PACKET_SETUP_DIMENSIONS
            + HSA_KERNEL_DISPATCH_PACKET_SETUP_WIDTH_DIMENSIONS - 1)));
      return;

    case AMD_DBGAPI_DISPATCH_INFO_WORK_GROUP_SIZES:
      {
        uint16_t workgroup_sizes[3]
          = { m_packet.workgroup_size_x, m_packet.workgroup_size_y,
              m_packet.workgroup_size_z };
        utils::get_info (value_size, value, workgroup_sizes);
        return;
      }
    case AMD_DBGAPI_DISPATCH_INFO_GRID_SIZES:
      {
        uint32_t grid_sizes[3] = { m_packet.grid_size_x, m_packet.grid_size_y,
                                   m_packet.grid_size_z };
        utils::get_info (value_size, value, grid_sizes);
        return;
      }
    case AMD_DBGAPI_DISPATCH_INFO_PRIVATE_SEGMENT_SIZE:
      utils::get_info (
        value_size, value,
        static_cast<amd_dbgapi_size_t> (m_packet.private_segment_size));
      return;

    case AMD_DBGAPI_DISPATCH_INFO_GROUP_SEGMENT_SIZE:
      utils::get_info (
        value_size, value,
        static_cast<amd_dbgapi_size_t> (m_packet.group_segment_size));
      return;

    case AMD_DBGAPI_DISPATCH_INFO_KERNEL_ARGUMENT_SEGMENT_ADDRESS:
      utils::get_info (value_size, value, m_packet.kernarg_address);
      return;

    case AMD_DBGAPI_DISPATCH_INFO_KERNEL_DESCRIPTOR_ADDRESS:
      utils::get_info (value_size, value, kernel_descriptor_address ());
      return;

    case AMD_DBGAPI_DISPATCH_INFO_KERNEL_CODE_ENTRY_ADDRESS:
      utils::get_info (value_size, value, kernel_code_entry_address ());
      return;

    case AMD_DBGAPI_DISPATCH_INFO_KERNEL_COMPLETION_ADDRESS:
      utils::get_info (value_size, value, m_packet.completion_signal);
      return;
    }

  throw api_error_t (AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT);
}

} /* namespace amd::dbgapi */

using namespace amd::dbgapi;

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_dispatch_get_info (amd_dbgapi_dispatch_id_t dispatch_id,
                              amd_dbgapi_dispatch_info_t query,
                              size_t value_size, void *value)
{
  TRACE_BEGIN (param_in (dispatch_id), param_in (query), param_in (value_size),
               param_in (value));
  TRY;

  if (!detail::is_initialized)
    THROW (AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED);

  dispatch_t *dispatch = find (dispatch_id);

  if (!dispatch)
    THROW (AMD_DBGAPI_STATUS_ERROR_INVALID_DISPATCH_ID);

  dispatch->get_info (query, value_size, value);

  return AMD_DBGAPI_STATUS_SUCCESS;

  CATCH (AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED,
         AMD_DBGAPI_STATUS_ERROR_INVALID_DISPATCH_ID,
         AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT,
         AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT_COMPATIBILITY,
         AMD_DBGAPI_STATUS_ERROR_CLIENT_CALLBACK);
  TRACE_END (make_query_ref (query, param_out (value)));
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_process_dispatch_list (amd_dbgapi_process_id_t process_id,
                                  size_t *dispatch_count,
                                  amd_dbgapi_dispatch_id_t **dispatches,
                                  amd_dbgapi_changed_t *changed)
{
  TRACE_BEGIN (param_in (process_id), param_in (dispatch_count),
               param_in (dispatches), param_in (changed));
  TRY;

  if (!detail::is_initialized)
    THROW (AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED);

  std::vector<process_t *> processes = process_t::match (process_id);

  if (!dispatches || !dispatch_count)
    THROW (AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT);

  std::vector<std::pair<process_t *, std::vector<queue_t *>>>
    queues_needing_resume;

  for (auto &&process : processes)
    {
      process->update_queues ();

      std::vector<queue_t *> queues;
      for (auto &&queue : process->range<queue_t> ())
        if (!queue.is_suspended ())
          queues.emplace_back (&queue);

      process->suspend_queues (queues, "refresh dispatch list");

      if (process->forward_progress_needed ())
        queues_needing_resume.emplace_back (process, std::move (queues));
    }

  amd_dbgapi_changed_t dispatch_list_changed;
  auto dispatch_list = utils::get_handle_list<dispatch_t> (
    processes, changed ? &dispatch_list_changed : nullptr);

  auto deallocate_dispatch_list = utils::make_scope_fail (
    [&] () { amd::dbgapi::deallocate_memory (dispatches); });

  for (auto &&[process, queues] : queues_needing_resume)
    process->resume_queues (queues, "refresh dispatch list");

  std::tie (*dispatches, *dispatch_count) = dispatch_list;
  if (changed)
    *changed = dispatch_list_changed;

  return AMD_DBGAPI_STATUS_SUCCESS;

  CATCH (AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED,
         AMD_DBGAPI_STATUS_ERROR_INVALID_PROCESS_ID,
         AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT,
         AMD_DBGAPI_STATUS_ERROR_CLIENT_CALLBACK);
  TRACE_END (make_ref (param_out (dispatch_count)),
             make_ref (make_ref (param_out (dispatches)), *dispatch_count),
             make_ref (param_out (changed)));
}
