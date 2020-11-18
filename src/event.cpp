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

#include "event.h"
#include "debug.h"
#include "handle_object.h"
#include "initialization.h"
#include "logging.h"
#include "process.h"
#include "queue.h"
#include "utils.h"
#include "wave.h"

#include <string>

namespace amd::dbgapi
{

/* Breakpoint resume event.  */
event_t::event_t (amd_dbgapi_event_id_t event_id, process_t &process,
                  amd_dbgapi_event_kind_t event_kind,
                  amd_dbgapi_breakpoint_id_t breakpoint_id,
                  amd_dbgapi_client_thread_id_t client_thread_id)
    : handle_object (event_id), m_event_kind (event_kind),
      m_data (breakpoint_resume_event_t{ breakpoint_id, client_thread_id }),
      m_process (process)
{
  dbgapi_assert (event_kind == AMD_DBGAPI_EVENT_KIND_BREAKPOINT_RESUME
                 && "check event kind");
}

/* Code object list updated event.  */
event_t::event_t (amd_dbgapi_event_id_t event_id, process_t &process,
                  amd_dbgapi_event_kind_t event_kind,
                  amd_dbgapi_event_id_t breakpoint_resume_event_id)
    : handle_object (event_id), m_event_kind (event_kind),
      m_data (code_object_list_updated_event_t{ breakpoint_resume_event_id }),
      m_process (process)
{
  dbgapi_assert (event_kind == AMD_DBGAPI_EVENT_KIND_CODE_OBJECT_LIST_UPDATED
                 && "check event kind");
}

/* Runtime event. */
event_t::event_t (amd_dbgapi_event_id_t event_id, process_t &process,
                  amd_dbgapi_event_kind_t event_kind,
                  amd_dbgapi_runtime_state_t runtime_state)
    : handle_object (event_id), m_event_kind (event_kind),
      m_data (runtime_event_t{ runtime_state }), m_process (process)
{
  dbgapi_assert (event_kind == AMD_DBGAPI_EVENT_KIND_RUNTIME
                 && "check event kind");
}

/* Wave stop / command terminated event.  */
event_t::event_t (amd_dbgapi_event_id_t event_id, process_t &process,
                  amd_dbgapi_event_kind_t event_kind,
                  amd_dbgapi_wave_id_t wave_id)
    : handle_object (event_id), m_event_kind (event_kind),
      m_data (wave_event_t{ wave_id }), m_process (process)
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
        wave_t *wave
            = process ().find (std::get<wave_event_t> (m_data).wave_id);
        if (!wave)
          return string_printf (
              "EVENT_KIND_WAVE_STOP for terminated %s",
              to_string (std::get<wave_event_t> (m_data).wave_id).c_str ());
        else
          return string_printf (
              "EVENT_KIND_WAVE_STOP for %s on %s "
              "(%spc=%#lx, stop_reason=%s)",
              to_string (wave->id ()).c_str (),
              to_string (wave->queue ().id ()).c_str (),
              wave->displaced_stepping () ? "displaced_" : "", wave->pc (),
              to_string (wave->stop_reason ()).c_str ());
      }

    case AMD_DBGAPI_EVENT_KIND_WAVE_COMMAND_TERMINATED:
      return string_printf (
          "EVENT_KIND_WAVE_COMMAND_TERMINATED for terminated %s",
          to_string (std::get<wave_event_t> (m_data).wave_id).c_str ());

    case AMD_DBGAPI_EVENT_KIND_CODE_OBJECT_LIST_UPDATED:
      return "EVENT_KIND_CODE_OBJECT_LIST_UPDATED";

    case AMD_DBGAPI_EVENT_KIND_BREAKPOINT_RESUME:
      return string_printf (
          "EVENT_KIND_BREAKPOINT_RESUME for %s",
          to_string (
              std::get<breakpoint_resume_event_t> (m_data).breakpoint_id)
              .c_str ());

    case AMD_DBGAPI_EVENT_KIND_RUNTIME:
      return string_printf (
          "EVENT_KIND_RUNTIME state=%s",
          to_string (std::get<runtime_event_t> (m_data).runtime_state)
              .c_str ());

    case AMD_DBGAPI_EVENT_KIND_QUEUE_ERROR:
    default:
      return "unhandled event";
    }
}

void
event_t::set_processed ()
{
  if (kind () == AMD_DBGAPI_EVENT_KIND_CODE_OBJECT_LIST_UPDATED)
    {
      amd_dbgapi_event_id_t event_id
          = std::get<code_object_list_updated_event_t> (m_data)
                .breakpoint_resume_event_id;

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
    case AMD_DBGAPI_EVENT_INFO_PROCESS:
      return utils::get_info (value_size, value, process ().id ());

    case AMD_DBGAPI_EVENT_INFO_KIND:
      return utils::get_info (value_size, value, m_event_kind);

    case AMD_DBGAPI_EVENT_INFO_WAVE:
      if (kind () != AMD_DBGAPI_EVENT_KIND_WAVE_STOP
          && kind () != AMD_DBGAPI_EVENT_KIND_WAVE_COMMAND_TERMINATED)
        return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;

      return utils::get_info (value_size, value,
                              std::get<wave_event_t> (m_data).wave_id);

    case AMD_DBGAPI_EVENT_INFO_BREAKPOINT:
      if (kind () != AMD_DBGAPI_EVENT_KIND_BREAKPOINT_RESUME)
        return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;

      return utils::get_info (
          value_size, value,
          std::get<breakpoint_resume_event_t> (m_data).breakpoint_id);

    case AMD_DBGAPI_EVENT_INFO_CLIENT_THREAD:
      if (kind () != AMD_DBGAPI_EVENT_KIND_BREAKPOINT_RESUME)
        return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;

      return utils::get_info (
          value_size, value,
          std::get<breakpoint_resume_event_t> (m_data).client_thread_id);

    case AMD_DBGAPI_EVENT_INFO_RUNTIME_STATE:
      if (kind () != AMD_DBGAPI_EVENT_KIND_RUNTIME)
        return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;

      return utils::get_info (
          value_size, value, std::get<runtime_event_t> (m_data).runtime_state);
    }
  return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;
}

} /* namespace amd::dbgapi */

using namespace amd::dbgapi;

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_process_next_pending_event (amd_dbgapi_process_id_t process_id,
                                       amd_dbgapi_event_id_t *event_id,
                                       amd_dbgapi_event_kind_t *kind)
{
  TRY;
  TRACE (process_id);

  if (!detail::is_initialized)
    return AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED;

  if (!event_id || !kind)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;

  const event_t *event = nullptr;

  if (process_id != AMD_DBGAPI_PROCESS_NONE)
    {
      process_t *process = process_t::find (process_id);

      if (!process)
        return AMD_DBGAPI_STATUS_ERROR_INVALID_PROCESS_ID;

      event = process->next_pending_event ();
    }
  else
    {
      for (auto &&process : process_t::all ())
        if ((event = process.next_pending_event ()))
          break;
    }

  *event_id = event ? event->id () : AMD_DBGAPI_EVENT_NONE;
  *kind = event ? event->kind () : AMD_DBGAPI_EVENT_KIND_NONE;

  return AMD_DBGAPI_STATUS_SUCCESS;
  CATCH;
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_event_get_info (amd_dbgapi_event_id_t event_id,
                           amd_dbgapi_event_info_t query, size_t value_size,
                           void *value)
{
  TRY;
  TRACE (event_id, query, value_size);

  if (!detail::is_initialized)
    return AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED;

  event_t *event = find (event_id);

  if (!event)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_EVENT_ID;

  return event->get_info (query, value_size, value);
  CATCH;
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_event_processed (amd_dbgapi_event_id_t event_id)
{
  TRY;
  TRACE (event_id);

  if (!detail::is_initialized)
    return AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED;

  event_t *event = find (event_id);

  if (!event)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_EVENT_ID;

  event->set_processed ();

  /* We are done with this event, remove it from the map.  */
  event->process ().destroy (event);

  return AMD_DBGAPI_STATUS_SUCCESS;
  CATCH;
}
