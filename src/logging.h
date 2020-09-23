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

#ifndef AMD_DBGAPI_LOGGING_H
#define AMD_DBGAPI_LOGGING_H 1

#include "amd-dbgapi.h"

#include <cstdarg>
#include <cstddef>
#include <sstream>
#include <string>
#include <utility>

#define dbgapi_log(level, format, ...)                                        \
  do                                                                          \
    {                                                                         \
      if (level <= amd::dbgapi::log_level)                                    \
        amd::dbgapi::detail::log (level, format, ##__VA_ARGS__);              \
    }                                                                         \
  while (0)

namespace amd::dbgapi
{
namespace detail
{

extern void log (amd_dbgapi_log_level_t level, const char *format, ...)
#if defined(__GNUC__)
    __attribute__ ((format (printf, 2, 3)))
#endif /* defined (__GNUC__) */
    ;

} /* namespace detail */

extern void vlog (amd_dbgapi_log_level_t level, const char *format,
                  va_list va);

extern amd_dbgapi_log_level_t log_level;

class tracer
{
public:
  template <typename... Args>
  tracer (const char *prefix, const char *function, Args &&... args);
  ~tracer () { --s_call_depth; }

private:
  static size_t s_call_depth;
};

#define TRACE(...)                                                            \
  amd::dbgapi::tracer __tracer__##__COUNTER__ { "", __FUNCTION__, __VA_ARGS__ }

template <typename T>
inline std::string
to_string (T v)
{
  std::ostringstream ss;
  ss << v;
  return ss.str ();
};

namespace detail
{

template <typename T> struct hex
{
  T value;
};

} /* namespace detail  */

template <typename T>
detail::hex<T>
make_hex (T value)
{
  return detail::hex<T>{ value };
}

template <typename T>
std::string
to_string (detail::hex<T> v)
{
  std::ostringstream ss;
  ss << "0x" << std::hex << v.value;
  return ss.str ();
}

#define AMD_DBGAPI_TYPES_DO(F)                                                \
  F (amd_dbgapi_address_class_id_t)                                           \
  F (amd_dbgapi_address_class_info_t)                                         \
  F (amd_dbgapi_address_class_state_t)                                        \
  F (amd_dbgapi_address_space_access_t)                                       \
  F (amd_dbgapi_address_space_alias_t)                                        \
  F (amd_dbgapi_address_space_id_t)                                           \
  F (amd_dbgapi_address_space_info_t)                                         \
  F (amd_dbgapi_agent_id_t)                                                   \
  F (amd_dbgapi_agent_info_t)                                                 \
  F (amd_dbgapi_architecture_id_t)                                            \
  F (amd_dbgapi_architecture_info_t)                                          \
  F (amd_dbgapi_breakpoint_action_t)                                          \
  F (amd_dbgapi_breakpoint_id_t)                                              \
  F (amd_dbgapi_breakpoint_info_t)                                            \
  F (amd_dbgapi_changed_t)                                                    \
  F (amd_dbgapi_code_object_id_t)                                             \
  F (amd_dbgapi_code_object_info_t)                                           \
  F (amd_dbgapi_dispatch_barrier_t)                                           \
  F (amd_dbgapi_dispatch_fence_scope_t)                                       \
  F (amd_dbgapi_dispatch_id_t)                                                \
  F (amd_dbgapi_dispatch_info_t)                                              \
  F (amd_dbgapi_displaced_stepping_id_t)                                      \
  F (amd_dbgapi_displaced_stepping_info_t)                                    \
  F (amd_dbgapi_event_id_t)                                                   \
  F (amd_dbgapi_event_info_t)                                                 \
  F (amd_dbgapi_event_kind_t)                                                 \
  F (amd_dbgapi_instruction_kind_t)                                           \
  F (amd_dbgapi_log_level_t)                                                  \
  F (amd_dbgapi_memory_precision_t)                                           \
  F (amd_dbgapi_os_queue_type_t)                                              \
  F (amd_dbgapi_process_id_t)                                                 \
  F (amd_dbgapi_process_info_t)                                               \
  F (amd_dbgapi_progress_t)                                                   \
  F (amd_dbgapi_queue_error_reason_t)                                         \
  F (amd_dbgapi_queue_id_t)                                                   \
  F (amd_dbgapi_queue_info_t)                                                 \
  F (amd_dbgapi_queue_state_t)                                                \
  F (amd_dbgapi_register_class_id_t)                                          \
  F (amd_dbgapi_register_class_info_t)                                        \
  F (amd_dbgapi_register_class_state_t)                                       \
  F (amd_dbgapi_register_exists_t)                                            \
  F (amd_dbgapi_register_id_t)                                                \
  F (amd_dbgapi_register_info_t)                                              \
  F (amd_dbgapi_resume_mode_t)                                                \
  F (amd_dbgapi_runtime_state_t)                                              \
  F (amd_dbgapi_shared_library_id_t)                                          \
  F (amd_dbgapi_shared_library_info_t)                                        \
  F (amd_dbgapi_shared_library_state_t)                                       \
  F (amd_dbgapi_status_t)                                                     \
  F (amd_dbgapi_wave_creation_t)                                              \
  F (amd_dbgapi_wave_id_t)                                                    \
  F (amd_dbgapi_wave_info_t)                                                  \
  F (amd_dbgapi_wave_state_t)                                                 \
  F (amd_dbgapi_wave_stop_reason_t)                                           \
  F (amd_dbgapi_watchpoint_id_t)                                              \
  F (amd_dbgapi_watchpoint_info_t)                                            \
  F (amd_dbgapi_watchpoint_kind_t)                                            \
  F (amd_dbgapi_watchpoint_share_kind_t)

#define EXPLICIT_SPECIALIZATION(T) template <> std::string to_string (T);
AMD_DBGAPI_TYPES_DO (EXPLICIT_SPECIALIZATION);

#undef EXPLICIT_SPECIALIZATION
#undef AMD_DBGAPI_TYPES_DO

inline std::string
to_string ()
{
  return "";
}

template <typename T, typename... Args>
inline std::string
to_string (T first, Args &&... args)
{
  return to_string (first) + ", " + to_string (std::forward<Args> (args)...);
}

template <typename... Args>
tracer::tracer (const char *prefix, const char *function, Args &&... args)
{
  const size_t indent_length = s_call_depth++ * 3;

  if (log_level < AMD_DBGAPI_LOG_LEVEL_VERBOSE)
    return;

  std::string prefix_string (prefix);
  if (!prefix_string.empty ())
    prefix_string.append (" ");

  std::string indent_string (indent_length + 1, ' ');
  indent_string[indent_length] = '>';

  dbgapi_log (AMD_DBGAPI_LOG_LEVEL_VERBOSE, "%s %s%s (%s)",
              indent_string.c_str (), prefix_string.c_str (), function,
              to_string (args...).c_str ());
}

} /* namespace amd::dbgapi */

#endif /* AMD_DBGAPI_LOGGING_H */
