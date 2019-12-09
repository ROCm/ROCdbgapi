/* Copyright (c) 2019 Advanced Micro Devices, Inc.

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

#include "callbacks.h"
#include "debug.h"
#include "logging.h"
#include "process.h"
#include "utils.h"

#include <functional>
#include <string>

namespace amd
{
namespace dbgapi
{

shared_library_t::shared_library_t (amd_dbgapi_shared_library_id_t library_id,
                                    process_t *process, std::string name,
                                    notify_callback_t on_load,
                                    notify_callback_t on_unload)
    : handle_object (library_id), m_on_load (on_load), m_on_unload (on_unload),
      m_process (process)
{
  amd_dbgapi_shared_library_state_t library_state;

  if (m_process->enable_notify_shared_library (name.c_str (), library_id,
                                               &library_state)
      != AMD_DBGAPI_STATUS_SUCCESS)
    return;

  m_is_valid = true;

  if (library_state == AMD_DBGAPI_SHARED_LIBRARY_STATE_LOADED)
    on_load (this);
}

shared_library_t::~shared_library_t ()
{
  if (m_is_valid)
    m_process->disable_notify_shared_library (id ());
}

breakpoint_t::breakpoint_t (amd_dbgapi_breakpoint_id_t breakpoint_id,
                            const shared_library_t *shared_library,
                            amd_dbgapi_global_address_t address,
                            action_callback_t action)
    : handle_object (breakpoint_id), m_address (address), m_action (action),
      m_shared_library (shared_library)
{
  m_inserted = process ()->add_breakpoint (shared_library->id (), address,
                                           breakpoint_id)
               == AMD_DBGAPI_STATUS_SUCCESS;
}

breakpoint_t::~breakpoint_t ()
{
  if (m_inserted)
    {
      amd_dbgapi_status_t status = process ()->remove_breakpoint (id ());
      if (status != AMD_DBGAPI_STATUS_SUCCESS)
        warning ("remove_breakpoint failed (rc=%d)", status);
    }
}

void
breakpoint_t::set_state (amd_dbgapi_breakpoint_state_t state)
{
  amd_dbgapi_status_t status = process ()->set_breakpoint_state (id (), state);
  if (status != AMD_DBGAPI_STATUS_SUCCESS)
    error ("set_breakpoint_state failed (rc=%d)", status);
}

} /* namespace dbgapi */
} /* namespace amd */

using namespace amd::dbgapi;

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_report_shared_library (
    amd_dbgapi_process_id_t process_id,
    amd_dbgapi_shared_library_id_t shared_library_id,
    amd_dbgapi_shared_library_state_t shared_library_state)
{
  TRY;
  TRACE (process_id, shared_library_id, shared_library_state);

  if (!amd::dbgapi::is_initialized)
    return AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED;

  if (shared_library_state != AMD_DBGAPI_SHARED_LIBRARY_STATE_LOADED
      && shared_library_state != AMD_DBGAPI_SHARED_LIBRARY_STATE_UNLOADED)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;

  process_t *process = process_t::find (process_id);

  if (!process)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_PROCESS_ID;

  shared_library_t *shared_library = process->find (shared_library_id);

  if (!shared_library)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_SHARED_LIBRARY_ID;

  shared_library->callback (shared_library_state) (shared_library);

  return AMD_DBGAPI_STATUS_SUCCESS;
  CATCH;
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_report_breakpoint_hit (
    amd_dbgapi_process_id_t process_id,
    amd_dbgapi_breakpoint_id_t breakpoint_id,
    amd_dbgapi_client_thread_id_t client_thread_id,
    amd_dbgapi_breakpoint_action_t *breakpoint_action)
{
  TRY;
  TRACE (process_id, breakpoint_id, client_thread_id);

  if (!amd::dbgapi::is_initialized)
    return AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED;

  if (!breakpoint_action)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;

  process_t *process = process_t::find (process_id);

  if (!process)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_PROCESS_ID;

  breakpoint_t *breakpoint = process->find (breakpoint_id);

  if (!breakpoint)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_BREAKPOINT_ID;

  return breakpoint->action () (breakpoint, client_thread_id,
                                breakpoint_action);
  CATCH;
}
