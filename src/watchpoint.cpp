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

#include "debug.h"
#include "logging.h"
#include "utils.h"

namespace amd
{
namespace dbgapi
{

} /* namespace dbgapi */
} /* namespace amd */

using namespace amd::dbgapi;

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_set_watchpoint (amd_dbgapi_process_id_t process_id,
                           amd_dbgapi_agent_id_t agent_id,
                           amd_dbgapi_global_address_t address,
                           amd_dbgapi_size_t size,
                           amd_dbgapi_watchpoint_kind_t kind,
                           amd_dbgapi_watchpoint_id_t *watchpoint_id,
                           amd_dbgapi_global_address_t *watchpoint_address,
                           amd_dbgapi_size_t *watchpoint_size)
{
  TRY;
  TRACE (process_id, agent_id, address, size, kind);

  warning ("amd_dbgapi_set_watchpoint is not yet implemented");
  return AMD_DBGAPI_STATUS_ERROR_UNIMPLEMENTED;
  CATCH;
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_remove_watchpoint (amd_dbgapi_process_id_t process_id,
                              amd_dbgapi_agent_id_t agent_id,
                              amd_dbgapi_watchpoint_id_t watchpoint_id)
{
  TRY;
  TRACE (process_id, agent_id, watchpoint_id);

  warning ("amd_dbgapi_remove_watchpoint is not yet implemented");
  return AMD_DBGAPI_STATUS_ERROR_UNIMPLEMENTED;
  CATCH;
}
