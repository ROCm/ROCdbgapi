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

#ifndef AMD_DBGAPI_DEBUG_H
#define AMD_DBGAPI_DEBUG_H 1

#include "amd-dbgapi.h"

#include <stdexcept>
#include <string>

#if defined(NDEBUG)
#define dbgapi_assert(expr) ((void)0)
#else /* !defined (NDEBUG) */
#define dbgapi_assert(expr)                                                   \
  ((void)((expr) ? 0 : (dbgapi_assert_fail (#expr, __FILE__, __LINE__), 0)))
#endif /* !defined(NDEBUG) */

#define dbgapi_assert_fail(assertion, file, line)                             \
  amd::dbgapi::error ("%s:%d: Assertion `%s' failed.", file, line, assertion)

#define dbgapi_assert_not_reached(message)                                    \
  amd::dbgapi::error ("%s:%d: Should not reach here: " message, __FILE__,     \
                      __LINE__)

namespace amd::dbgapi
{

/* AMD Debugger API exception.  */

class exception_t : public std::runtime_error
{
private:
  /* The error code for this exception.  */
  amd_dbgapi_status_t m_error_code;

public:
  exception_t (amd_dbgapi_status_t error_code, std::string message = {})
      : std::runtime_error (std::move (message)), m_error_code (error_code)
  {
  }

  void print_message () const noexcept;

  amd_dbgapi_status_t error_code () const noexcept { return m_error_code; }
};

extern void warning (const char *format, ...)
#if defined(__GNUC__)
    __attribute__ ((format (printf, 1, 2)))
#endif /* defined (__GNUC__) */
    ;

extern void error [[noreturn]] (const char *format, ...)
#if defined(__GNUC__)
__attribute__ ((format (printf, 1, 2)))
#endif /* defined (__GNUC__) */
;

} /* namespace amd::dbgapi */

#endif /* AMD_DBGAPI_DEBUG_H */
