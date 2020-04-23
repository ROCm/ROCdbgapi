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

#include "logging.h"
#include "process.h"
#include "utils.h"

namespace amd
{
namespace dbgapi
{

amd_dbgapi_callbacks_s process_callbacks;
bool is_initialized = false;

} /* namespace dbgapi */
} /* namespace amd */

using namespace amd::dbgapi;

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_initialize (struct amd_dbgapi_callbacks_s *callbacks)
{
  TRY;

  if (amd::dbgapi::is_initialized)
    return AMD_DBGAPI_STATUS_ERROR_ALREADY_INITIALIZED;

  /* check that all the callback functions are defined.  */
  for (size_t i = 0; i < sizeof (*callbacks) / sizeof (void (*) (void)); ++i)
    if (!callbacks || !reinterpret_cast<void (**) (void)> (callbacks)[i])
      return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;

  process_callbacks = *callbacks;

  TRACE (callbacks);

  amd::dbgapi::is_initialized = true;

  return AMD_DBGAPI_STATUS_SUCCESS;
  CATCH;
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_finalize ()
{
  TRY;
  TRACE ();

  if (!process_callbacks.get_symbol_address)
    return AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED;

  if (!amd::dbgapi::is_initialized)
    return AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED;

  /* Detach all remaining processes.  */
  for (process_t *process : process_list)
    amd_dbgapi_process_detach (amd_dbgapi_process_id_t{ process->id () });

  process_list.clear ();

  process_callbacks = { 0 };
  amd::dbgapi::is_initialized = false;

  return AMD_DBGAPI_STATUS_SUCCESS;
  CATCH;
}