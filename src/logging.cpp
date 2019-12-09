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

#include "logging.h"
#include "utils.h"

#include <string>

namespace amd
{
namespace dbgapi
{

amd_dbgapi_log_level_t log_level = AMD_DBGAPI_LOG_LEVEL_NONE;

size_t tracer::s_call_depth = 0;

void
vlog (amd_dbgapi_log_level_t level, const char *format, va_list va)
{
  if (level > log_level)
    return;

  std::string message;

  if (level == AMD_DBGAPI_LOG_LEVEL_FATAL_ERROR)
    message.append ("fatal error: ");
  else if (level == AMD_DBGAPI_LOG_LEVEL_WARNING)
    message.append ("warning: ");

  message.append (string_vprintf (format, va));

  (*process_callbacks.log_message) (level, message.c_str ());
}

} /* namespace dbgapi */
} /* namespace amd */

void AMD_DBGAPI
amd_dbgapi_set_log_level (amd_dbgapi_log_level_t level)
{
  TRACE (level);
  amd::dbgapi::log_level = level;
}
