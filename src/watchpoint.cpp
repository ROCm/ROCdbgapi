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

#include "watchpoint.h"
#include "amd-dbgapi.h"
#include "debug.h"
#include "initialization.h"
#include "logging.h"
#include "process.h"
#include "utils.h"

namespace amd::dbgapi
{

amd_dbgapi_status_t
watchpoint_t::get_info (amd_dbgapi_watchpoint_info_t query, size_t value_size,
                        void *value) const
{
  switch (query)
    {
    case AMD_DBGAPI_WATCHPOINT_INFO_PROCESS:
      return utils::get_info (value_size, value, process ().id ());
    }

  return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;
}

} /* namespace amd::dbgapi */

using namespace amd::dbgapi;

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_set_watchpoint (amd_dbgapi_process_id_t process_id,
                           amd_dbgapi_global_address_t address,
                           amd_dbgapi_size_t size,
                           amd_dbgapi_watchpoint_kind_t kind,
                           amd_dbgapi_watchpoint_id_t *watchpoint_id,
                           amd_dbgapi_global_address_t *watchpoint_address,
                           amd_dbgapi_size_t *watchpoint_size)
{
  TRY;
  TRACE (process_id, address, size, kind);

  if (!detail::is_initialized)
    return AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED;

  process_t *process = process_t::find (process_id);

  if (!process)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_PROCESS_ID;

  if (!size || !watchpoint_id || !watchpoint_address || !watchpoint_size)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;

  switch (kind)
    {
    case AMD_DBGAPI_WATCHPOINT_KIND_LOAD:
    case AMD_DBGAPI_WATCHPOINT_KIND_STORE_AND_RMW:
    case AMD_DBGAPI_WATCHPOINT_KIND_RMW:
    case AMD_DBGAPI_WATCHPOINT_KIND_ALL:
      break;
    default:
      return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;
    }

  watchpoint_t &watchpoint
      = process->create<watchpoint_t> (*process, address, size, kind);

  amd_dbgapi_status_t status = process->insert_watchpoint (
      watchpoint, watchpoint_address, watchpoint_size);
  if (status != AMD_DBGAPI_STATUS_SUCCESS
      && status != AMD_DBGAPI_STATUS_ERROR_PROCESS_EXITED)
    {
      process->destroy (&watchpoint);
      return status;
    }

  *watchpoint_id = watchpoint.id ();
  return AMD_DBGAPI_STATUS_SUCCESS;
  CATCH;
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_remove_watchpoint (amd_dbgapi_process_id_t process_id,
                              amd_dbgapi_watchpoint_id_t watchpoint_id)
{
  TRY;
  TRACE (process_id, watchpoint_id);

  if (!detail::is_initialized)
    return AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED;

  process_t *process = process_t::find (process_id);

  if (!process)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_PROCESS_ID;

  watchpoint_t *watchpoint = process->find (watchpoint_id);

  if (!watchpoint)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_WATCHPOINT_ID;

  process->remove_watchpoint (*watchpoint);
  process->destroy (watchpoint);

  return AMD_DBGAPI_STATUS_SUCCESS;
  CATCH;
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_watchpoint_get_info (amd_dbgapi_watchpoint_id_t watchpoint_id,
                                amd_dbgapi_watchpoint_info_t query,
                                size_t value_size, void *value)
{
  TRY;
  TRACE (watchpoint_id, query, value_size, value);

  if (!detail::is_initialized)
    return AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED;

  watchpoint_t *watchpoint = find (watchpoint_id);

  if (!watchpoint)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_WATCHPOINT_ID;

  return watchpoint->get_info (query, value_size, value);
  CATCH;
}
