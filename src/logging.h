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

#ifndef _AMD_DBGAPI_LOGGING_H
#define _AMD_DBGAPI_LOGGING_H 1

#include "defs.h"

#include "utils.h"

#include <cstdarg>
#include <sstream>
#include <stdexcept>
#include <string>

#define dbgapi_log(level, format, ...)                                        \
  do                                                                          \
    {                                                                         \
      if (level <= amd::dbgapi::log_level)                                    \
        {                                                                     \
          [] (amd_dbgapi_log_level_t _level, const char *_format, ...) {      \
            va_list va;                                                       \
            va_start (va, _format);                                           \
            amd::dbgapi::vlog (_level, _format, va);                          \
            va_end (va);                                                      \
          }(level, format, ##__VA_ARGS__);                                    \
        }                                                                     \
    }                                                                         \
  while (0)

namespace amd
{
namespace dbgapi
{

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

template <>
inline std::string
to_string (amd_dbgapi_register_class_id_t register_class_id)
{
  return string_printf ("register_class_%ld", register_class_id.handle);
}

template <>
inline std::string
to_string (amd_dbgapi_register_id_t register_id)
{
  return string_printf ("register_%ld", register_id.handle);
}

template <>
inline std::string
to_string (amd_dbgapi_architecture_id_t architecture_id)
{
  return string_printf ("architecture_%ld", architecture_id.handle);
}

template <>
inline std::string
to_string (amd_dbgapi_process_id_t process_id)
{
  return string_printf ("process_%ld", process_id.handle);
}

template <>
inline std::string
to_string (amd_dbgapi_wave_id_t wave_id)
{
  return string_printf ("wave_%ld", wave_id.handle);
}

template <>
inline std::string
to_string (amd_dbgapi_dispatch_id_t dispatch_id)
{
  return string_printf ("dipatch_%ld", dispatch_id.handle);
}

template <>
inline std::string
to_string (amd_dbgapi_queue_id_t queue_id)
{
  return string_printf ("queue_%ld", queue_id.handle);
}

template <>
inline std::string
to_string (amd_dbgapi_agent_id_t agent_id)
{
  return string_printf ("agent_%ld", agent_id.handle);
}

template <>
inline std::string
to_string (amd_dbgapi_code_object_id_t code_object_id)
{
  return string_printf ("code_object_%ld", code_object_id.handle);
}

template <>
inline std::string
to_string (amd_dbgapi_shared_library_id_t shared_library_id)
{
  return string_printf ("shared_library_%ld", shared_library_id.handle);
}

template <>
inline std::string
to_string (amd_dbgapi_event_id_t event_id)
{
  return string_printf ("event_%ld", event_id.handle);
}

template <>
inline std::string
to_string (amd_dbgapi_breakpoint_id_t breakpoint_id)
{
  return string_printf ("breakpoint_%ld", breakpoint_id.handle);
}

template <>
inline std::string
to_string (amd_dbgapi_displaced_stepping_id_t displaced_stepping_id)
{
  return string_printf ("displaced_stepping_%ld",
                        displaced_stepping_id.handle);
}

template <>
inline std::string
to_string (amd_dbgapi_address_class_id_t address_class_id)
{
  return string_printf ("address_class_%ld", address_class_id.handle);
}

template <>
inline std::string
to_string (amd_dbgapi_address_space_id_t address_space_id)
{
  return string_printf ("address_space_%ld", address_space_id.handle);
}

template <typename T> struct hex
{
  T value;
};

template <typename T>
static inline hex<T>
make_hex (T value)
{
  return hex<T>{ value };
}

template <typename T>
inline std::string
to_string (hex<T> v)
{
  std::ostringstream ss;
  ss << std::showbase << std::hex << v.value;
  return ss.str ();
}

#define CASE(x)                                                               \
  case AMD_DBGAPI_##x:                                                        \
    return #x

template <>
inline std::string
to_string (amd_dbgapi_register_class_info_t register_class_info)
{
  switch (register_class_info)
    {
      CASE (REGISTER_CLASS_INFO_NAME);
    }
  return to_string (make_hex (register_class_info));
}

template <>
inline std::string
to_string (amd_dbgapi_register_info_t register_info)
{
  switch (register_info)
    {
      CASE (REGISTER_INFO_NAME);
      CASE (REGISTER_INFO_SIZE);
      CASE (REGISTER_INFO_TYPE);
    }
  return to_string (make_hex (register_info));
}

template <>
inline std::string
to_string (amd_dbgapi_code_object_info_t code_object_info)
{
  switch (code_object_info)
    {
      CASE (CODE_OBJECT_INFO_URI_NAME);
      CASE (CODE_OBJECT_INFO_LOAD_ADDRESS);
    }
  return to_string (make_hex (code_object_info));
}

template <>
inline std::string
to_string (amd_dbgapi_architecture_info_t architecture_info)
{
  switch (architecture_info)
    {
      CASE (ARCHITECTURE_INFO_NAME);
      CASE (ARCHITECTURE_INFO_ELF_AMDGPU_MACHINE);
      CASE (ARCHITECTURE_INFO_LARGEST_INSTRUCTION_SIZE);
      CASE (ARCHITECTURE_INFO_MINIMUM_INSTRUCTION_ALIGNMENT);
      CASE (ARCHITECTURE_INFO_BREAKPOINT_INSTRUCTION_SIZE);
      CASE (ARCHITECTURE_INFO_BREAKPOINT_INSTRUCTION);
      CASE (ARCHITECTURE_INFO_BREAKPOINT_INSTRUCTION_PC_ADJUST);
      CASE (ARCHITECTURE_INFO_PC_REGISTER);
      CASE (ARCHITECTURE_INFO_EXECUTION_MASK_REGISTER);
      CASE (ARCHITECTURE_INFO_WATCHPOINT_COUNT);
      CASE (ARCHITECTURE_INFO_WATCHPOINT_SHARE);
      CASE (ARCHITECTURE_INFO_DEFAULT_GLOBAL_ADDRESS_SPACE);
      CASE (ARCHITECTURE_INFO_PRECISE_MEMORY_SUPPORTED);
    }
  return to_string (make_hex (architecture_info));
}

template <>
inline std::string
to_string (amd_dbgapi_wave_info_t wave_info)
{
  switch (wave_info)
    {
      CASE (WAVE_INFO_STATE);
      CASE (WAVE_INFO_STOP_REASON);
      CASE (WAVE_INFO_WATCHPOINTS);
      CASE (WAVE_INFO_DISPATCH);
      CASE (WAVE_INFO_QUEUE);
      CASE (WAVE_INFO_AGENT);
      CASE (WAVE_INFO_ARCHITECTURE);
      CASE (WAVE_INFO_PC);
      CASE (WAVE_INFO_EXEC_MASK);
      CASE (WAVE_INFO_WORK_GROUP_COORD);
      CASE (WAVE_INFO_WAVE_NUMBER_IN_WORK_GROUP);
      CASE (WAVE_INFO_LANE_COUNT);
    }
  return to_string (make_hex (wave_info));
}

template <>
inline std::string
to_string (amd_dbgapi_dispatch_info_t dispatch_info)
{
  switch (dispatch_info)
    {
      CASE (DISPATCH_INFO_QUEUE);
      CASE (DISPATCH_INFO_AGENT);
      CASE (DISPATCH_INFO_ARCHITECTURE);
      CASE (DISPATCH_INFO_PACKET_ID);
      CASE (DISPATCH_INFO_BARRIER);
      CASE (DISPATCH_INFO_ACQUIRE_FENCE);
      CASE (DISPATCH_INFO_RELEASE_FENCE);
      CASE (DISPATCH_INFO_GRID_DIMENSIONS);
      CASE (DISPATCH_INFO_WORK_GROUP_SIZES);
      CASE (DISPATCH_INFO_GRID_SIZES);
      CASE (DISPATCH_INFO_PRIVATE_SEGMENT_SIZE);
      CASE (DISPATCH_INFO_GROUP_SEGMENT_SIZE);
      CASE (DISPATCH_INFO_KERNEL_ARGUMENT_SEGMENT_ADDRESS);
      CASE (DISPATCH_INFO_KERNEL_ENTRY_ADDRESS);
    }
  return to_string (make_hex (dispatch_info));
}

template <>
inline std::string
to_string (amd_dbgapi_queue_info_t queue_info)
{
  switch (queue_info)
    {
      CASE (QUEUE_INFO_AGENT);
      CASE (QUEUE_INFO_ARCHITECTURE);
      CASE (QUEUE_TYPE);
      CASE (QUEUE_INFO_STATE);
      CASE (QUEUE_INFO_ERROR_REASON);
    }
  return to_string (make_hex (queue_info));
}

template <>
inline std::string
to_string (amd_dbgapi_agent_info_t agent_info)
{
  switch (agent_info)
    {
      CASE (AGENT_INFO_NAME);
      CASE (AGENT_INFO_ARCHITECTURE);
      CASE (AGENT_INFO_PCIE_SLOT);
      CASE (AGENT_INFO_PCIE_VENDOR_ID);
      CASE (AGENT_INFO_PCIE_DEVICE_ID);
      CASE (AGENT_INFO_SHADER_ENGINE_COUNT);
      CASE (AGENT_INFO_COMPUTE_UNIT_COUNT);
      CASE (AGENT_INFO_NUM_SIMD_PER_COMPUTE_UNIT);
      CASE (AGENT_INFO_MAX_WAVES_PER_SIMD);
    }
  return to_string (make_hex (agent_info));
}

template <>
inline std::string
to_string (amd_dbgapi_progress_t progress)
{
  switch (progress)
    {
      CASE (PROGRESS_NORMAL);
      CASE (PROGRESS_NO_FORWARD);
    }
  return to_string (make_hex (progress));
}

template <>
inline std::string
to_string (amd_dbgapi_shared_library_state_t library_state)
{
  switch (library_state)
    {
      CASE (SHARED_LIBRARY_STATE_LOADED);
      CASE (SHARED_LIBRARY_STATE_UNLOADED);
    }
  return to_string (make_hex (library_state));
}

template <>
inline std::string
to_string (amd_dbgapi_address_space_info_t address_space_info)
{
  switch (address_space_info)
    {
      CASE (ADDRESS_SPACE_INFO_NAME);
      CASE (ADDRESS_SPACE_INFO_ADDRESS_SIZE);
      CASE (ADDRESS_SPACE_INFO_NULL_ADDRESS);
      CASE (ADDRESS_SPACE_INFO_ACCESS);
    }
  return to_string (make_hex (address_space_info));
}

template <>
inline std::string
to_string (amd_dbgapi_memory_precision_t memory_precision)
{
  switch (memory_precision)
    {
      CASE (MEMORY_PRECISION_NONE);
      CASE (MEMORY_PRECISION_PRECISE);
    }
  return to_string (make_hex (memory_precision));
}

template <>
inline std::string
to_string (amd_dbgapi_process_info_t process_info)
{
  switch (process_info)
    {
      CASE (PROCESS_INFO_NOTIFIER);
    }
  return to_string (make_hex (process_info));
}

template <>
inline std::string
to_string (amd_dbgapi_event_kind_t event_kind)
{
  switch (event_kind)
    {
      CASE (EVENT_KIND_NONE);
      CASE (EVENT_KIND_WAVE_STOP);
      CASE (EVENT_KIND_WAVE_COMMAND_TERMINATED);
      CASE (EVENT_KIND_CODE_OBJECT_LIST_UPDATED);
      CASE (EVENT_KIND_BREAKPOINT_RESUME);
      CASE (EVENT_KIND_RUNTIME);
      CASE (EVENT_KIND_QUEUE_ERROR);
    }
  return to_string (make_hex (event_kind));
}

template <>
inline std::string
to_string (amd_dbgapi_wave_state_t wave_state)
{
  switch (wave_state)
    {
      CASE (WAVE_STATE_RUN);
      CASE (WAVE_STATE_SINGLE_STEP);
      CASE (WAVE_STATE_STOP);
    }
  return to_string (make_hex (wave_state));
}

template <>
inline std::string
to_string (amd_dbgapi_event_info_t event_info)
{
  switch (event_info)
    {
      CASE (EVENT_INFO_KIND);
      CASE (EVENT_INFO_WAVE);
      CASE (EVENT_INFO_BREAKPOINT);
      CASE (EVENT_INFO_CLIENT_THREAD);
      CASE (EVENT_INFO_RUNTIME_STATE);
      CASE (EVENT_INFO_RUNTIME_VERSION);
    }
  return to_string (make_hex (event_info));
}

static inline std::string
one_stop_reason_to_string (amd_dbgapi_wave_stop_reason_t stop_reason)
{
  switch (stop_reason)
    {
      CASE (WAVE_STOP_REASON_NONE);
      CASE (WAVE_STOP_REASON_BREAKPOINT);
      CASE (WAVE_STOP_REASON_WATCHPOINT);
      CASE (WAVE_STOP_REASON_SINGLE_STEP);
      CASE (WAVE_STOP_REASON_QUEUE_ERROR);
      CASE (WAVE_STOP_REASON_FP_INPUT_DENORMAL);
      CASE (WAVE_STOP_REASON_FP_DIVIDE_BY_0);
      CASE (WAVE_STOP_REASON_FP_OVERFLOW);
      CASE (WAVE_STOP_REASON_FP_UNDERFLOW);
      CASE (WAVE_STOP_REASON_FP_INEXACT);
      CASE (WAVE_STOP_REASON_FP_INVALID_OPERATION);
      CASE (WAVE_STOP_REASON_INT_DIVIDE_BY_0);
      CASE (WAVE_STOP_REASON_DEBUG_TRAP);
      CASE (WAVE_STOP_REASON_ASSERT_TRAP);
      CASE (WAVE_STOP_REASON_TRAP);
      CASE (WAVE_STOP_REASON_MEMORY_VIOLATION);
      CASE (WAVE_STOP_REASON_ILLEGAL_INSTRUCTION);
      CASE (WAVE_STOP_REASON_ECC_ERROR);
      CASE (WAVE_STOP_REASON_FATAL_HALT);
      CASE (WAVE_STOP_REASON_XNACK_ERROR);
    }
  return to_string (make_hex (stop_reason));
}

template <>
inline std::string
to_string (amd_dbgapi_resume_mode_t resume_mode)
{
  switch (resume_mode)
    {
      CASE (RESUME_MODE_NORMAL);
      CASE (RESUME_MODE_SINGLE_STEP);
    }
  return to_string (make_hex (resume_mode));
}

template <>
inline std::string
to_string (amd_dbgapi_wave_stop_reason_t stop_reason)
{
  std::ostringstream ss;
  bool first = true;

  if (!stop_reason)
    return one_stop_reason_to_string (stop_reason);

  while (stop_reason)
    {
      if (!first)
        ss << " | ";

      amd_dbgapi_wave_stop_reason_t one_bit
          = static_cast<amd_dbgapi_wave_stop_reason_t> (
              stop_reason ^ (stop_reason & (stop_reason - 1)));
      ss << one_stop_reason_to_string (one_bit);

      stop_reason
          = static_cast<amd_dbgapi_wave_stop_reason_t> (stop_reason ^ one_bit);

      first = false;
    }

  return ss.str ();
}

#undef CASE

inline std::string
to_string ()
{
  return "";
}

template <typename T, typename... Args>
inline std::string
to_string (T first, Args &&... args)
{
  return to_string (first) + ", " + to_string (args...);
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

} /* namespace dbgapi */
} /* namespace amd */

#endif /* _AMD_DBGAPI_LOGGING_H */
