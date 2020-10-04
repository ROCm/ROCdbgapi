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

#include "callbacks.h"
#include "debug.h"
#include "initialization.h"
#include "logging.h"
#include "process.h"
#include "utils.h"

#include <functional>
#include <string>
#include <utility>

namespace amd::dbgapi
{

shared_library_t::shared_library_t (amd_dbgapi_shared_library_id_t library_id,
                                    process_t &process, std::string name,
                                    notify_callback_t on_load,
                                    notify_callback_t on_unload)
    : handle_object (library_id), m_name (std::move (name)),
      m_on_load (on_load), m_on_unload (on_unload),
      m_state (AMD_DBGAPI_SHARED_LIBRARY_STATE_UNLOADED), m_process (process)
{
  amd_dbgapi_shared_library_state_t state;

  if (m_process.enable_notify_shared_library (m_name.c_str (), library_id,
                                              &state)
      != AMD_DBGAPI_STATUS_SUCCESS)
    return;

  m_is_valid = true;
  set_state (state);
}

shared_library_t::~shared_library_t ()
{
  if (m_is_valid)
    m_process.disable_notify_shared_library (id ());
}

amd_dbgapi_status_t
shared_library_t::get_info (amd_dbgapi_shared_library_info_t query,
                            size_t value_size, void *value) const
{
  switch (query)
    {
    case AMD_DBGAPI_SHARED_LIBRARY_INFO_PROCESS:
      return utils::get_info (value_size, value, process ().id ());
    }

  return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;
}

void
shared_library_t::set_state (amd_dbgapi_shared_library_state_t state)
{
  dbgapi_assert (state == AMD_DBGAPI_SHARED_LIBRARY_STATE_LOADED
                 || state == AMD_DBGAPI_SHARED_LIBRARY_STATE_UNLOADED);

  if (m_state != state)
    {
      m_state = state;
      /* Call the notifier since the state has changed.  */
      (state == AMD_DBGAPI_SHARED_LIBRARY_STATE_LOADED ? m_on_load
                                                       : m_on_unload) (*this);
    }
}

breakpoint_t::breakpoint_t (amd_dbgapi_breakpoint_id_t breakpoint_id,
                            const shared_library_t &shared_library,
                            amd_dbgapi_global_address_t address,
                            action_callback_t action)
    : handle_object (breakpoint_id), m_address (address), m_action (action),
      m_shared_library (shared_library)
{
  m_inserted = process ().insert_breakpoint (shared_library.id (), address,
                                             breakpoint_id)
               == AMD_DBGAPI_STATUS_SUCCESS;
}

breakpoint_t::~breakpoint_t ()
{
  if (m_inserted)
    {
      amd_dbgapi_status_t status = process ().remove_breakpoint (id ());
      if (status != AMD_DBGAPI_STATUS_SUCCESS)
        warning ("remove_breakpoint failed (rc=%d)", status);
    }
}

amd_dbgapi_status_t
breakpoint_t::get_info (amd_dbgapi_breakpoint_info_t query, size_t value_size,
                        void *value) const
{
  switch (query)
    {
    case AMD_DBGAPI_BREAKPOINT_INFO_SHARED_LIBRARY:
      return utils::get_info (value_size, value, shared_library ().id ());

    case AMD_DBGAPI_BREAKPOINT_INFO_PROCESS:
      return utils::get_info (value_size, value, process ().id ());
    }

  return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;
}

} /* namespace amd::dbgapi */

using namespace amd::dbgapi;

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_report_shared_library (
    amd_dbgapi_shared_library_id_t shared_library_id,
    amd_dbgapi_shared_library_state_t shared_library_state)
{
  TRY;
  TRACE (shared_library_id, shared_library_state);

  if (!detail::is_initialized)
    return AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED;

  if (shared_library_state != AMD_DBGAPI_SHARED_LIBRARY_STATE_LOADED
      && shared_library_state != AMD_DBGAPI_SHARED_LIBRARY_STATE_UNLOADED)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;

  shared_library_t *shared_library = find (shared_library_id);

  if (!shared_library)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_SHARED_LIBRARY_ID;

  shared_library->set_state (shared_library_state);

  return AMD_DBGAPI_STATUS_SUCCESS;
  CATCH;
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_code_shared_library_get_info (
    amd_dbgapi_shared_library_id_t shared_library_id,
    amd_dbgapi_shared_library_info_t query, size_t value_size, void *value)
{
  TRY;
  TRACE (shared_library_id, query, value_size, value);

  if (!detail::is_initialized)
    return AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED;

  shared_library_t *shared_library = find (shared_library_id);

  if (!shared_library)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_SHARED_LIBRARY_ID;

  return shared_library->get_info (query, value_size, value);
  CATCH;
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_report_breakpoint_hit (
    amd_dbgapi_breakpoint_id_t breakpoint_id,
    amd_dbgapi_client_thread_id_t client_thread_id,
    amd_dbgapi_breakpoint_action_t *breakpoint_action)
{
  TRY;
  TRACE (breakpoint_id, client_thread_id);

  if (!detail::is_initialized)
    return AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED;

  if (!breakpoint_action)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;

  breakpoint_t *breakpoint = find (breakpoint_id);

  if (!breakpoint)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_BREAKPOINT_ID;

  return breakpoint->action () (*breakpoint, client_thread_id,
                                breakpoint_action);
  CATCH;
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_breakpoint_get_info (amd_dbgapi_breakpoint_id_t breakpoint_id,
                                amd_dbgapi_breakpoint_info_t query,
                                size_t value_size, void *value)
{
  TRY;
  TRACE (breakpoint_id, query, value_size, value);

  if (!detail::is_initialized)
    return AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED;

  breakpoint_t *breakpoint = find (breakpoint_id);

  if (!breakpoint)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_BREAKPOINT_ID;

  return breakpoint->get_info (query, value_size, value);
  CATCH;
}
