/* Copyright (c) 2022 Advanced Micro Devices, Inc.

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

#include "workgroup.h"
#include "agent.h"
#include "dispatch.h"
#include "process.h"
#include "queue.h"

namespace amd::dbgapi
{

compute_queue_t &
workgroup_t::queue () const
{
  return dispatch ().queue ();
}

const agent_t &
workgroup_t::agent () const
{
  return queue ().agent ();
}

process_t &
workgroup_t::process () const
{
  return agent ().process ();
}

const architecture_t &
workgroup_t::architecture () const
{
  return queue ().architecture ();
}

void
workgroup_t::get_info (amd_dbgapi_workgroup_info_t query, size_t value_size,
                       void *value) const
{
  switch (query)
    {
    case AMD_DBGAPI_WORKGROUP_INFO_DISPATCH:
      utils::get_info (value_size, value, dispatch ().id ());
      return;

    case AMD_DBGAPI_WORKGROUP_INFO_QUEUE:
      utils::get_info (value_size, value, queue ().id ());
      return;

    case AMD_DBGAPI_WORKGROUP_INFO_AGENT:
      utils::get_info (value_size, value, agent ().id ());
      return;

    case AMD_DBGAPI_WORKGROUP_INFO_PROCESS:
      utils::get_info (value_size, value, process ().id ());
      return;

    case AMD_DBGAPI_WORKGROUP_INFO_ARCHITECTURE:
      utils::get_info (value_size, value, architecture ().id ());
      return;

    case AMD_DBGAPI_WORKGROUP_INFO_WORKGROUP_COORD:
      if (!group_ids ())
        throw api_error_t (AMD_DBGAPI_STATUS_ERROR_NOT_AVAILABLE);
      utils::get_info (value_size, value, *group_ids ());
      return;
    }

  throw api_error_t (AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT);
}

} /* namespace amd::dbgapi */

using namespace amd::dbgapi;

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_workgroup_get_info (amd_dbgapi_workgroup_id_t workgroup_id,
                               amd_dbgapi_workgroup_info_t query,
                               size_t value_size, void *value)
{
  TRACE_BEGIN (param_in (workgroup_id), param_in (query),
               param_in (value_size), param_in (value));
  TRY
  {
    if (!detail::is_initialized)
      THROW (AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED);

    workgroup_t *workgroup = find (workgroup_id);

    if (!workgroup)
      THROW (AMD_DBGAPI_STATUS_ERROR_INVALID_WORKGROUP_ID);

    workgroup->get_info (query, value_size, value);
  }
  CATCH (AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED,
         AMD_DBGAPI_STATUS_ERROR_INVALID_WORKGROUP_ID,
         AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT,
         AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT_COMPATIBILITY,
         AMD_DBGAPI_STATUS_ERROR_NOT_AVAILABLE,
         AMD_DBGAPI_STATUS_ERROR_CLIENT_CALLBACK);
  TRACE_END (make_query_ref (query, param_out (value)));
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_process_workgroup_list (amd_dbgapi_process_id_t process_id,
                                   size_t *workgroup_count,
                                   amd_dbgapi_workgroup_id_t **workgroups,
                                   amd_dbgapi_changed_t *changed)
{
  TRACE_BEGIN (param_in (process_id), param_in (workgroup_count),
               param_in (workgroups), param_in (changed));
  TRY
  {
    if (!detail::is_initialized)
      THROW (AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED);

    std::vector<process_t *> processes = process_t::match (process_id);

    if (!workgroups || !workgroup_count)
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

        process->suspend_queues (queues, "refresh workgroup list");

        if (process->forward_progress_needed ())
          queues_needing_resume.emplace_back (process, std::move (queues));
      }

    amd_dbgapi_changed_t workgroup_list_changed;
    auto workgroup_list = utils::get_handle_list<workgroup_t> (
      processes, changed ? &workgroup_list_changed : nullptr);

    auto deallocate_workgroup_list = utils::make_scope_fail (
      [&] () { amd::dbgapi::deallocate_memory (workgroups); });

    for (auto &&[process, queues] : queues_needing_resume)
      process->resume_queues (queues, "refresh workgroup list");

    std::tie (*workgroups, *workgroup_count) = workgroup_list;
    if (changed)
      *changed = workgroup_list_changed;
  }
  CATCH (AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED,
         AMD_DBGAPI_STATUS_ERROR_INVALID_PROCESS_ID,
         AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT,
         AMD_DBGAPI_STATUS_ERROR_CLIENT_CALLBACK);
  TRACE_END (make_ref (param_out (workgroup_count)),
             make_ref (make_ref (param_out (workgroups)), *workgroup_count),
             make_ref (param_out (changed)));
}
