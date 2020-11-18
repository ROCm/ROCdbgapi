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

#include "code_object.h"
#include "debug.h"
#include "initialization.h"
#include "logging.h"
#include "process.h"
#include "utils.h"

#include <vector>

namespace amd::dbgapi
{

amd_dbgapi_status_t
code_object_t::get_info (amd_dbgapi_code_object_info_t query,
                         size_t value_size, void *value) const
{
  switch (query)
    {
    case AMD_DBGAPI_CODE_OBJECT_INFO_PROCESS:
      return utils::get_info (value_size, value, process ().id ());

    case AMD_DBGAPI_CODE_OBJECT_INFO_URI_NAME:
      return utils::get_info (value_size, value, m_uri);

    case AMD_DBGAPI_CODE_OBJECT_INFO_LOAD_ADDRESS:
      return utils::get_info (value_size, value, m_load_address);
    }
  return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;
}

} /* namespace amd::dbgapi */

using namespace amd::dbgapi;

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_code_object_get_info (amd_dbgapi_code_object_id_t code_object_id,
                                 amd_dbgapi_code_object_info_t query,
                                 size_t value_size, void *value)
{
  TRY;
  TRACE (code_object_id, query, value_size, value);

  if (!detail::is_initialized)
    return AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED;

  code_object_t *code_object = find (code_object_id);

  if (!code_object)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_CODE_OBJECT_ID;

  return code_object->get_info (query, value_size, value);
  CATCH;
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_process_code_object_list (
    amd_dbgapi_process_id_t process_id, size_t *code_object_count,
    amd_dbgapi_code_object_id_t **code_objects, amd_dbgapi_changed_t *changed)
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

      processes.emplace_back (process);
    }
  else
    {
      for (auto &&process : process_t::all ())
        processes.emplace_back (&process);
    }

  return utils::get_handle_list<code_object_t> (processes, code_object_count,
                                                code_objects, changed);
  CATCH;
}
