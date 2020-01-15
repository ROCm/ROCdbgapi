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
#include "debug.h"
#include "event.h"
#include "handle_object.h"
#include "logging.h"
#include "process.h"
#include "queue.h"
#include "utils.h"
#include "wave.h"

#include <atomic>
#include <string>
#include <vector>

namespace amd
{
namespace dbgapi
{

/* Breakpoint resume event.  */
event_t::event_t (amd_dbgapi_event_id_t event_id, process_t &process,
                  amd_dbgapi_event_kind_t event_kind,
                  amd_dbgapi_breakpoint_id_t breakpoint_id,
                  amd_dbgapi_client_thread_id_t client_thread_id)
    : handle_object (event_id),
      m_event_kind (event_kind), m_data{ .breakpoint_resume_event
                                         = { breakpoint_id,
                                             client_thread_id } },
      m_process (process)
{
  dbgapi_assert (event_kind == AMD_DBGAPI_EVENT_KIND_BREAKPOINT_RESUME
                 && "check event kind");
}

/* Code object list updated event.  */
event_t::event_t (amd_dbgapi_event_id_t event_id, process_t &process,
                  amd_dbgapi_event_kind_t event_kind,
                  amd_dbgapi_event_id_t breakpoint_resume_event_id)
    : handle_object (event_id),
      m_event_kind (event_kind), m_data{ .code_object_list_updated_event
                                         = { breakpoint_resume_event_id } },
      m_process (process)
{
  dbgapi_assert (event_kind == AMD_DBGAPI_EVENT_KIND_CODE_OBJECT_LIST_UPDATED
                 && "check event kind");
}

/* Runtime event. */
event_t::event_t (amd_dbgapi_event_id_t event_id, process_t &process,
                  amd_dbgapi_event_kind_t event_kind,
                  amd_dbgapi_runtime_state_t runtime_state)
    : handle_object (event_id),
      m_event_kind (event_kind), m_data{ .runtime_event = { runtime_state } },
      m_process (process)
{
  dbgapi_assert (event_kind == AMD_DBGAPI_EVENT_KIND_RUNTIME
                 && "check event kind");
}

/* Wave stop / command terminated event.  */
event_t::event_t (amd_dbgapi_event_id_t event_id, process_t &process,
                  amd_dbgapi_event_kind_t event_kind,
                  amd_dbgapi_wave_id_t wave_id)
    : handle_object (event_id),
      m_event_kind (event_kind), m_data{ .wave_event = { wave_id } },
      m_process (process)
{
  dbgapi_assert (
      (event_kind == AMD_DBGAPI_EVENT_KIND_WAVE_STOP
       || event_kind == AMD_DBGAPI_EVENT_KIND_WAVE_COMMAND_TERMINATED)
      && "check event kind");
}

std::string
event_t::pretty_printer_string () const
{
  switch (kind ())
    {
    case AMD_DBGAPI_EVENT_KIND_NONE:
      return "null event";

    case AMD_DBGAPI_EVENT_KIND_WAVE_STOP:
      {
        wave_t *wave = process ().find (m_data.wave_event.wave_id);
        if (!wave)
          return string_printf (
              "WAVE_STOP for terminated %s",
              to_string (m_data.wave_event.wave_id).c_str ());
        else
          return string_printf ("WAVE_STOP for %s (pc=%#lx, stop_reason=%s)",
                                to_string (wave->id ()).c_str (), wave->pc (),
                                to_string (wave->stop_reason ()).c_str ());
      }

    case AMD_DBGAPI_EVENT_KIND_WAVE_COMMAND_TERMINATED:
      return string_printf ("WAVE_COMMAND_TERMINATED for terminated %s",
                            to_string (m_data.wave_event.wave_id).c_str ());

    case AMD_DBGAPI_EVENT_KIND_CODE_OBJECT_LIST_UPDATED:
      return "CODE_OBJECT_LIST_UPDATED";

    case AMD_DBGAPI_EVENT_KIND_BREAKPOINT_RESUME:
      return string_printf (
          "BREAKPOINT_RESUME for %s",
          to_string (m_data.breakpoint_resume_event.breakpoint_id).c_str ());

    case AMD_DBGAPI_EVENT_KIND_RUNTIME:
      return string_printf (
          "RUNTIME state=%s",
          to_string (m_data.runtime_event.runtime_state).c_str ());

    case AMD_DBGAPI_EVENT_KIND_QUEUE_ERROR:
    default:
      return "unhandled event";
    }
}

void
event_t::processed ()
{
  if (kind () == AMD_DBGAPI_EVENT_KIND_CODE_OBJECT_LIST_UPDATED)
    {
      amd_dbgapi_event_id_t event_id
          = m_data.code_object_list_updated_event.breakpoint_resume_event_id;

      /* Code object updates that are not initiated by the r_brk breakpoint
         callback do not have a breakpoint resume event.  */
      if (event_id != AMD_DBGAPI_EVENT_NONE)
        {
          event_t *breakpoint_resume_event = process ().find (event_id);
          dbgapi_assert (breakpoint_resume_event);
          process ().enqueue_event (*breakpoint_resume_event);
        }
    }
}

amd_dbgapi_status_t
event_t::get_info (amd_dbgapi_event_info_t query, size_t value_size,
                   void *value) const
{
  switch (query)
    {
    case AMD_DBGAPI_EVENT_INFO_KIND:
      return utils::get_info (value_size, value, m_event_kind);

    case AMD_DBGAPI_EVENT_INFO_WAVE:
      if (kind () != AMD_DBGAPI_EVENT_KIND_WAVE_STOP
          && kind () != AMD_DBGAPI_EVENT_KIND_WAVE_COMMAND_TERMINATED)
        return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;

      return utils::get_info (value_size, value, m_data.wave_event.wave_id);

    case AMD_DBGAPI_EVENT_INFO_BREAKPOINT:
      if (kind () != AMD_DBGAPI_EVENT_KIND_BREAKPOINT_RESUME)
        return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;

      return utils::get_info (value_size, value,
                              m_data.breakpoint_resume_event.breakpoint_id);

    case AMD_DBGAPI_EVENT_INFO_CLIENT_THREAD:
      if (kind () != AMD_DBGAPI_EVENT_KIND_BREAKPOINT_RESUME)
        return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;

      return utils::get_info (value_size, value,
                              m_data.breakpoint_resume_event.client_thread_id);

    case AMD_DBGAPI_EVENT_INFO_RUNTIME_STATE:
      if (kind () != AMD_DBGAPI_EVENT_KIND_RUNTIME)
        return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;

      return utils::get_info (value_size, value,
                              m_data.runtime_event.runtime_state);

    default:
      return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;
    }

  return AMD_DBGAPI_STATUS_SUCCESS;
}

} /* namespace dbgapi */
} /* namespace amd */

using namespace amd::dbgapi;

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_next_pending_event (amd_dbgapi_process_id_t process_id,
                               amd_dbgapi_event_id_t *event_id,
                               amd_dbgapi_event_kind_t *kind)
{
  TRY;
  TRACE (process_id);

  if (!amd::dbgapi::is_initialized)
    return AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED;

  if (!event_id || !kind)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;

  process_t *process = process_t::find (process_id);

  if (!process)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_PROCESS_ID;

  event_t *event = process->dequeue_event ();

  /* If we don't have any events left, we have to suspend the queues with
     pending events and process their context save area to add events for
     the waves that have reported events.  */
  if (!event)
    {
      std::vector<queue_t *> suspended_queues;
      suspended_queues.reserve (process->count<agent_t> ());

      /* We get our event notifications from the process event thread, so
         make sure it is still running, an exception may have caused it to
         exit.  */
      process->check_event_thread ();

      /* It is possible for new events to be generated between the time we
         query the agents for pending queue events and the time we actually
         suspend the queues.  To make sure all events associated with the
         suspended queues are consumed, we loop until no new queue events are
         reported.
         This is guaranteed to terminate as once a queue is suspended it cannot
         create any new events. A queue will require at most 2 iterations; and
         if any new events occur on additional queues those queues will be
         suspended by additional iterations, and since there are a finite
         number of queues N, this can at most result in 2*N iterations.  */
      while (true)
        {
          std::vector<queue_t *> queues;

          for (auto &&agent : process->range<agent_t> ())
            {
              /* Use an atomic exchange here since the value is written by the
                 event thread.  */
              if (!agent.kfd_event_notifier ().exchange (
                      false, std::memory_order_relaxed))
                continue;

              while (true)
                {
                  amd_dbgapi_queue_id_t queue_id;
                  uint32_t queue_status;

                  amd_dbgapi_status_t status
                      = agent.next_kfd_event (&queue_id, &queue_status);
                  if (status != AMD_DBGAPI_STATUS_SUCCESS)
                    return status;

                  if (!queue_id.handle)
                    break;

                  queue_t *queue = process->find (queue_id);

                  if (!queue)
                    {
                      /* Events that are reported for a queue that is deleted
                         before the events are retrieved are ignored.  This
                         includes events that are retrieved for a new queue
                         that is deleted before information about the queue can
                         be retrieved.  */
                      continue;
                    }

                  dbgapi_log (AMD_DBGAPI_LOG_LEVEL_INFO,
                              "%s has pending events",
                              to_string (queue_id).c_str ());

                  /* The queue may already be suspended. This can happen if an
                     event occurs after requesting the queue to be suspended
                     and before that request has completed, or if the act of
                     suspending a wave generates a new event (such as for the
                     single step work-around in the CWSR handler).  */
                  if (!queue->suspended ())
                    queues.emplace_back (queue);
                }
            }

          /* Exit the loop if we did not add any new queues to suspend in
             this iteration.  */
          if (queues.empty ())
            break;

          /* Suspend the queues that have pending events which will cause all
             events to be created for any waves that have pending events.  */
          process->suspend_queues (queues);

          /* Append queues into suspended_queues.  */
          suspended_queues.insert (suspended_queues.end (), queues.begin (),
                                   queues.end ());
        }

      event = process->dequeue_event ();

      /* If forward progress is needed, resume the queues we've just
         suspended.  */
      if (process->forward_progress_needed ())
        process->resume_queues (suspended_queues);
    }

  if (event)
    {
      *event_id = amd_dbgapi_event_id_t{ event->id () };
      *kind = event->kind ();
    }
  else
    *event_id = AMD_DBGAPI_EVENT_NONE;

  return AMD_DBGAPI_STATUS_SUCCESS;
  CATCH;
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_event_get_info (amd_dbgapi_process_id_t process_id,
                           amd_dbgapi_event_id_t event_id,
                           amd_dbgapi_event_info_t query, size_t value_size,
                           void *value)
{
  TRY;
  TRACE (process_id, event_id, query, value_size);

  if (!amd::dbgapi::is_initialized)
    return AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED;

  process_t *process = process_t::find (process_id);

  if (!process)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_PROCESS_ID;

  event_t *event = process->find (event_id);

  if (!event)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_EVENT_ID;

  return event->get_info (query, value_size, value);
  CATCH;
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_event_processed (amd_dbgapi_process_id_t process_id,
                            amd_dbgapi_event_id_t event_id)
{
  TRY;
  TRACE (process_id, event_id);

  if (!amd::dbgapi::is_initialized)
    return AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED;

  process_t *process = process_t::find (process_id);

  if (!process)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_PROCESS_ID;

  event_t *event = process->find (event_id);

  if (!event)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_EVENT_ID;

  event->processed ();

  /* We are done with this event, remove it from the map.  */
  process->destroy (event);

  return AMD_DBGAPI_STATUS_SUCCESS;
  CATCH;
}
