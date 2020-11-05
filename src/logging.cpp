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

#include "logging.h"
#include "process.h"
#include "utils.h"

#include <cstdarg>
#include <string>

namespace amd::dbgapi
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

  (*detail::process_callbacks.log_message) (level, message.c_str ());
}

namespace detail
{

void
log (amd_dbgapi_log_level_t level, const char *format, ...)
{
  va_list va;
  va_start (va, format);
  vlog (level, format, va);
  va_end (va);
}

} /* namespace detail */

template <>
std::string
to_string (amd_dbgapi_architecture_id_t architecture_id)
{
  return string_printf ("architecture_%ld", architecture_id.handle);
}

template <>
std::string
to_string (amd_dbgapi_process_id_t process_id)
{
  return string_printf ("process_%ld", process_id.handle);
}

template <>
std::string
to_string (amd_dbgapi_code_object_id_t code_object_id)
{
  return string_printf ("code_object_%ld", code_object_id.handle);
}

template <>
std::string
to_string (amd_dbgapi_agent_id_t agent_id)
{
  return string_printf ("agent_%ld", agent_id.handle);
}

template <>
std::string
to_string (amd_dbgapi_queue_id_t queue_id)
{
  return string_printf ("queue_%ld", queue_id.handle);
}

template <>
std::string
to_string (amd_dbgapi_dispatch_id_t dispatch_id)
{
  return string_printf ("dispatch_%ld", dispatch_id.handle);
}

template <>
std::string
to_string (amd_dbgapi_wave_id_t wave_id)
{
  return string_printf ("wave_%ld", wave_id.handle);
}

template <>
std::string
to_string (amd_dbgapi_displaced_stepping_id_t displaced_stepping_id)
{
  return string_printf ("displaced_stepping_%ld",
                        displaced_stepping_id.handle);
}

template <>
std::string
to_string (amd_dbgapi_register_class_id_t register_class_id)
{
  return string_printf ("register_class_%ld", register_class_id.handle);
}

template <>
std::string
to_string (amd_dbgapi_register_id_t register_id)
{
  return string_printf ("register_%ld", register_id.handle);
}

template <>
std::string
to_string (amd_dbgapi_address_class_id_t address_class_id)
{
  return string_printf ("address_class_%ld", address_class_id.handle);
}

template <>
std::string
to_string (amd_dbgapi_address_space_id_t address_space_id)
{
  return string_printf ("address_space_%ld", address_space_id.handle);
}

template <>
std::string
to_string (amd_dbgapi_event_id_t event_id)
{
  return string_printf ("event_%ld", event_id.handle);
}

template <>
std::string
to_string (amd_dbgapi_shared_library_id_t shared_library_id)
{
  return string_printf ("shared_library_%ld", shared_library_id.handle);
}

template <>
std::string
to_string (amd_dbgapi_breakpoint_id_t breakpoint_id)
{
  return string_printf ("breakpoint_%ld", breakpoint_id.handle);
}

#define CASE(x)                                                               \
  case AMD_DBGAPI_##x:                                                        \
    return #x

template <>
std::string
to_string (amd_dbgapi_changed_t changed)
{
  switch (changed)
    {
      CASE (CHANGED_NO);
      CASE (CHANGED_YES);
    }
  return to_string (make_hex (changed));
}

template <>
std::string
to_string (amd_dbgapi_status_t status)
{
  switch (status)
    {
      CASE (STATUS_SUCCESS);
      CASE (STATUS_ERROR);
      CASE (STATUS_FATAL);
      CASE (STATUS_ERROR_UNIMPLEMENTED);
      CASE (STATUS_ERROR_NOT_SUPPORTED);
      CASE (STATUS_ERROR_INVALID_ARGUMENT);
      CASE (STATUS_ERROR_INVALID_ARGUMENT_COMPATIBILITY);
      CASE (STATUS_ERROR_ALREADY_INITIALIZED);
      CASE (STATUS_ERROR_NOT_INITIALIZED);
      CASE (STATUS_ERROR_RESTRICTION);
      CASE (STATUS_ERROR_ALREADY_ATTACHED);
      CASE (STATUS_ERROR_INVALID_ARCHITECTURE_ID);
      CASE (STATUS_ERROR_ILLEGAL_INSTRUCTION);
      CASE (STATUS_ERROR_INVALID_CODE_OBJECT_ID);
      CASE (STATUS_ERROR_INVALID_ELF_AMDGPU_MACHINE);
      CASE (STATUS_ERROR_INVALID_PROCESS_ID);
      CASE (STATUS_ERROR_INVALID_AGENT_ID);
      CASE (STATUS_ERROR_INVALID_QUEUE_ID);
      CASE (STATUS_ERROR_INVALID_DISPATCH_ID);
      CASE (STATUS_ERROR_INVALID_WAVE_ID);
      CASE (STATUS_ERROR_WAVE_NOT_STOPPED);
      CASE (STATUS_ERROR_WAVE_STOPPED);
      CASE (STATUS_ERROR_WAVE_OUTSTANDING_STOP);
      CASE (STATUS_ERROR_WAVE_NOT_RESUMABLE);
      CASE (STATUS_ERROR_INVALID_DISPLACED_STEPPING_ID);
      CASE (STATUS_ERROR_DISPLACED_STEPPING_BUFFER_UNAVAILABLE);
      CASE (STATUS_ERROR_INVALID_WATCHPOINT_ID);
      CASE (STATUS_ERROR_NO_WATCHPOINT_AVAILABLE);
      CASE (STATUS_ERROR_INVALID_REGISTER_CLASS_ID);
      CASE (STATUS_ERROR_INVALID_REGISTER_ID);
      CASE (STATUS_ERROR_INVALID_LANE_ID);
      CASE (STATUS_ERROR_INVALID_ADDRESS_CLASS_ID);
      CASE (STATUS_ERROR_INVALID_ADDRESS_SPACE_ID);
      CASE (STATUS_ERROR_MEMORY_ACCESS);
      CASE (STATUS_ERROR_INVALID_ADDRESS_SPACE_CONVERSION);
      CASE (STATUS_ERROR_INVALID_EVENT_ID);
      CASE (STATUS_ERROR_INVALID_SHARED_LIBRARY_ID);
      CASE (STATUS_ERROR_INVALID_BREAKPOINT_ID);
      CASE (STATUS_ERROR_CLIENT_CALLBACK);
      CASE (STATUS_ERROR_INVALID_CLIENT_PROCESS_ID);
      CASE (STATUS_ERROR_PROCESS_EXITED);
      CASE (STATUS_ERROR_LIBRARY_NOT_LOADED);
      CASE (STATUS_ERROR_SYMBOL_NOT_FOUND);
      CASE (STATUS_ERROR_INVALID_ADDRESS);
      CASE (STATUS_ERROR_DISPLACED_STEPPING_ACTIVE);
    }
  return to_string (make_hex (status));
}

template <>
std::string
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
    }
  return to_string (make_hex (architecture_info));
}

template <>
std::string
to_string (amd_dbgapi_instruction_kind_t instruction_kind)
{
  switch (instruction_kind)
    {
      CASE (INSTRUCTION_KIND_UNKNOWN);
      CASE (INSTRUCTION_KIND_SEQUENTIAL);
      CASE (INSTRUCTION_KIND_DIRECT_BRANCH);
      CASE (INSTRUCTION_KIND_DIRECT_BRANCH_CONDITIONAL);
      CASE (INSTRUCTION_KIND_INDIRECT_BRANCH_REGISTER_PAIR);
      CASE (INSTRUCTION_KIND_DIRECT_CALL_REGISTER_PAIR);
      CASE (INSTRUCTION_KIND_INDIRECT_CALL_REGISTER_PAIRS);
      CASE (INSTRUCTION_KIND_TERMINATE);
      CASE (INSTRUCTION_KIND_TRAP);
      CASE (INSTRUCTION_KIND_HALT);
      CASE (INSTRUCTION_KIND_BARRIER);
      CASE (INSTRUCTION_KIND_SLEEP);
      CASE (INSTRUCTION_KIND_SPECIAL);
    }
  return to_string (make_hex (instruction_kind));
}

template <>
std::string
to_string (amd_dbgapi_process_info_t process_info)
{
  switch (process_info)
    {
      CASE (PROCESS_INFO_NOTIFIER);
      CASE (PROCESS_INFO_WATCHPOINT_COUNT);
      CASE (PROCESS_INFO_WATCHPOINT_SHARE);
      CASE (PROCESS_INFO_PRECISE_MEMORY_SUPPORTED);
      CASE (PROCESS_INFO_OS_ID);
    }
  return to_string (make_hex (process_info));
}

template <>
std::string
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
std::string
to_string (amd_dbgapi_wave_creation_t wave_creation)
{
  switch (wave_creation)
    {
      CASE (WAVE_CREATION_NORMAL);
      CASE (WAVE_CREATION_STOP);
    }
  return to_string (make_hex (wave_creation));
}

template <>
std::string
to_string (amd_dbgapi_watchpoint_info_t watchpoint_info)
{
  switch (watchpoint_info)
    {
      CASE (WATCHPOINT_INFO_PROCESS);
    }
  return to_string (make_hex (watchpoint_info));
}

template <>
std::string
to_string (amd_dbgapi_code_object_info_t code_object_info)
{
  switch (code_object_info)
    {
      CASE (CODE_OBJECT_INFO_PROCESS);
      CASE (CODE_OBJECT_INFO_URI_NAME);
      CASE (CODE_OBJECT_INFO_LOAD_ADDRESS);
    }
  return to_string (make_hex (code_object_info));
}

template <>
std::string
to_string (amd_dbgapi_breakpoint_info_t breakpoint_info)
{
  switch (breakpoint_info)
    {
      CASE (BREAKPOINT_INFO_SHARED_LIBRARY);
      CASE (BREAKPOINT_INFO_PROCESS);
    }
  return to_string (make_hex (breakpoint_info));
}

template <>
std::string
to_string (amd_dbgapi_shared_library_info_t shared_library_info)
{
  switch (shared_library_info)
    {
      CASE (SHARED_LIBRARY_INFO_PROCESS);
    }
  return to_string (make_hex (shared_library_info));
}

template <>
std::string
to_string (amd_dbgapi_displaced_stepping_info_t displaced_stepping_info)
{
  switch (displaced_stepping_info)
    {
      CASE (DISPLACED_STEPPING_INFO_PROCESS);
    }
  return to_string (make_hex (displaced_stepping_info));
}

template <>
std::string
to_string (amd_dbgapi_agent_info_t agent_info)
{
  switch (agent_info)
    {
      CASE (AGENT_INFO_PROCESS);
      CASE (AGENT_INFO_NAME);
      CASE (AGENT_INFO_ARCHITECTURE);
      CASE (AGENT_INFO_PCI_SLOT);
      CASE (AGENT_INFO_PCI_VENDOR_ID);
      CASE (AGENT_INFO_PCI_DEVICE_ID);
      CASE (AGENT_INFO_EXECUTION_UNIT_COUNT);
      CASE (AGENT_INFO_MAX_WAVES_PER_EXECUTION_UNIT);
      CASE (AGENT_INFO_OS_ID);
    }
  return to_string (make_hex (agent_info));
}

template <>
std::string
to_string (amd_dbgapi_queue_info_t queue_info)
{
  switch (queue_info)
    {
      CASE (QUEUE_INFO_AGENT);
      CASE (QUEUE_INFO_PROCESS);
      CASE (QUEUE_INFO_ARCHITECTURE);
      CASE (QUEUE_INFO_TYPE);
      CASE (QUEUE_INFO_STATE);
      CASE (QUEUE_INFO_ERROR_REASON);
      CASE (QUEUE_INFO_ADDRESS);
      CASE (QUEUE_INFO_SIZE);
      CASE (QUEUE_INFO_OS_ID);
    }
  return to_string (make_hex (queue_info));
}

template <>
std::string
to_string (amd_dbgapi_os_queue_type_t queue_type)
{
  switch (queue_type)
    {
      CASE (OS_QUEUE_TYPE_UNKNOWN);
      CASE (OS_QUEUE_TYPE_HSA_KERNEL_DISPATCH_MULTIPLE_PRODUCER);
      CASE (OS_QUEUE_TYPE_HSA_KERNEL_DISPATCH_SINGLE_PRODUCER);
      CASE (OS_QUEUE_TYPE_HSA_KERNEL_DISPATCH_COOPERATIVE);
      CASE (OS_QUEUE_TYPE_AMD_PM4);
      CASE (OS_QUEUE_TYPE_AMD_SDMA);
      CASE (OS_QUEUE_TYPE_AMD_SDMA_XGMI);
    }
  return to_string (make_hex (queue_type));
}

template <>
std::string
to_string (amd_dbgapi_queue_state_t queue_state)
{
  switch (queue_state)
    {
      CASE (QUEUE_STATE_VALID);
      CASE (QUEUE_STATE_ERROR);
    }
  return to_string (make_hex (queue_state));
}

namespace
{

inline std::string
one_queue_error_reason_to_string (
    amd_dbgapi_queue_error_reason_t queue_error_reason)
{
  switch (queue_error_reason)
    {
      CASE (QUEUE_ERROR_REASON_NONE);
      CASE (QUEUE_ERROR_REASON_INVALID_PACKET);
      CASE (QUEUE_ERROR_REASON_MEMORY_VIOLATION);
      CASE (QUEUE_ERROR_REASON_ASSERT_TRAP);
      CASE (QUEUE_ERROR_REASON_WAVE_ERROR);
      CASE (QUEUE_ERROR_REASON_RESERVED);
    }
  return to_string (make_hex (queue_error_reason));
}

} /* namespace */

template <>
std::string
to_string (amd_dbgapi_queue_error_reason_t queue_error_reason)
{
  std::string str;

  if (!queue_error_reason)
    return one_queue_error_reason_to_string (queue_error_reason);

  while (queue_error_reason)
    {
      amd_dbgapi_queue_error_reason_t one_bit
          = queue_error_reason
            ^ (queue_error_reason & (queue_error_reason - 1));

      if (!str.empty ())
        str += " | ";
      str += one_queue_error_reason_to_string (one_bit);

      queue_error_reason ^= one_bit;
    }

  return str;
}

template <>
std::string
to_string (amd_dbgapi_dispatch_info_t dispatch_info)
{
  switch (dispatch_info)
    {
      CASE (DISPATCH_INFO_QUEUE);
      CASE (DISPATCH_INFO_AGENT);
      CASE (DISPATCH_INFO_PROCESS);
      CASE (DISPATCH_INFO_ARCHITECTURE);
      CASE (DISPATCH_INFO_OS_QUEUE_PACKET_ID);
      CASE (DISPATCH_INFO_BARRIER);
      CASE (DISPATCH_INFO_ACQUIRE_FENCE);
      CASE (DISPATCH_INFO_RELEASE_FENCE);
      CASE (DISPATCH_INFO_GRID_DIMENSIONS);
      CASE (DISPATCH_INFO_WORK_GROUP_SIZES);
      CASE (DISPATCH_INFO_GRID_SIZES);
      CASE (DISPATCH_INFO_PRIVATE_SEGMENT_SIZE);
      CASE (DISPATCH_INFO_GROUP_SEGMENT_SIZE);
      CASE (DISPATCH_INFO_KERNEL_ARGUMENT_SEGMENT_ADDRESS);
      CASE (DISPATCH_INFO_KERNEL_DESCRIPTOR_ADDRESS);
      CASE (DISPATCH_INFO_KERNEL_CODE_ENTRY_ADDRESS);
      CASE (DISPATCH_INFO_KERNEL_COMPLETION_ADDRESS);
    }
  return to_string (make_hex (dispatch_info));
}

template <>
std::string
to_string (amd_dbgapi_dispatch_barrier_t dispatch_barrier)
{
  switch (dispatch_barrier)
    {
      CASE (DISPATCH_BARRIER_NONE);
      CASE (DISPATCH_BARRIER_PRESENT);
    }
  return to_string (make_hex (dispatch_barrier));
}

template <>
std::string
to_string (amd_dbgapi_dispatch_fence_scope_t dispatch_fence_scope)
{
  switch (dispatch_fence_scope)
    {
      CASE (DISPATCH_FENCE_SCOPE_NONE);
      CASE (DISPATCH_FENCE_SCOPE_AGENT);
      CASE (DISPATCH_FENCE_SCOPE_SYSTEM);
    }
  return to_string (make_hex (dispatch_fence_scope));
}

template <>
std::string
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
      CASE (WAVE_INFO_PROCESS);
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
std::string
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

namespace
{

inline std::string
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
      CASE (WAVE_STOP_REASON_RESERVED);
    }
  return to_string (make_hex (stop_reason));
}

} /* namespace */

template <>
std::string
to_string (amd_dbgapi_wave_stop_reason_t stop_reason)
{
  std::string str;

  if (!stop_reason)
    return one_stop_reason_to_string (stop_reason);

  while (stop_reason)
    {
      amd_dbgapi_wave_stop_reason_t one_bit
          = stop_reason ^ (stop_reason & (stop_reason - 1));

      if (!str.empty ())
        str += " | ";
      str += one_stop_reason_to_string (one_bit);

      stop_reason ^= one_bit;
    }

  return str;
}

template <>
std::string
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
std::string
to_string (amd_dbgapi_watchpoint_id_t watchpoint_id)
{
  return string_printf ("watchpoint_%ld", watchpoint_id.handle);
}

template <>
std::string
to_string (amd_dbgapi_watchpoint_share_kind_t watchpoint_share_kind)
{
  switch (watchpoint_share_kind)
    {
      CASE (WATCHPOINT_SHARE_KIND_UNSUPPORTED);
      CASE (WATCHPOINT_SHARE_KIND_UNSHARED);
      CASE (WATCHPOINT_SHARE_KIND_SHARED);
    }
  return to_string (make_hex (watchpoint_share_kind));
}

template <>
std::string
to_string (amd_dbgapi_watchpoint_kind_t watchpoint_kind)
{
  switch (watchpoint_kind)
    {
      CASE (WATCHPOINT_KIND_LOAD);
      CASE (WATCHPOINT_KIND_STORE_AND_RMW);
      CASE (WATCHPOINT_KIND_RMW);
      CASE (WATCHPOINT_KIND_ALL);
    }
  return to_string (make_hex (watchpoint_kind));
}

template <>
std::string
to_string (amd_dbgapi_register_class_info_t register_class_info)
{
  switch (register_class_info)
    {
      CASE (REGISTER_CLASS_INFO_ARCHITECTURE);
      CASE (REGISTER_CLASS_INFO_NAME);
    }
  return to_string (make_hex (register_class_info));
}

template <>
std::string
to_string (amd_dbgapi_register_info_t register_info)
{
  switch (register_info)
    {
      CASE (REGISTER_INFO_ARCHITECTURE);
      CASE (REGISTER_INFO_NAME);
      CASE (REGISTER_INFO_SIZE);
      CASE (REGISTER_INFO_TYPE);
    }
  return to_string (make_hex (register_info));
}

template <>
std::string
to_string (amd_dbgapi_register_exists_t register_exists)
{
  switch (register_exists)
    {
      CASE (REGISTER_ABSENT);
      CASE (REGISTER_PRESENT);
    }
  return to_string (make_hex (register_exists));
}

template <>
std::string
to_string (amd_dbgapi_register_class_state_t register_class_state)
{
  switch (register_class_state)
    {
      CASE (REGISTER_CLASS_STATE_NOT_MEMBER);
      CASE (REGISTER_CLASS_STATE_MEMBER);
    }
  return to_string (make_hex (register_class_state));
}

template <>
std::string
to_string (amd_dbgapi_address_class_info_t address_class_info)
{
  switch (address_class_info)
    {
      CASE (ADDRESS_CLASS_INFO_ARCHITECTURE);
      CASE (ADDRESS_CLASS_INFO_NAME);
      CASE (ADDRESS_CLASS_INFO_ADDRESS_SPACE);
    }
  return to_string (make_hex (address_class_info));
}

template <>
std::string
to_string (amd_dbgapi_address_space_access_t address_space_access)
{
  switch (address_space_access)
    {
      CASE (ADDRESS_SPACE_ACCESS_ALL);
      CASE (ADDRESS_SPACE_ACCESS_PROGRAM_CONSTANT);
      CASE (ADDRESS_SPACE_ACCESS_DISPATCH_CONSTANT);
    }
  return to_string (make_hex (address_space_access));
}

template <>
std::string
to_string (amd_dbgapi_address_space_info_t address_space_info)
{
  switch (address_space_info)
    {
      CASE (ADDRESS_SPACE_INFO_ARCHITECTURE);
      CASE (ADDRESS_SPACE_INFO_NAME);
      CASE (ADDRESS_SPACE_INFO_ADDRESS_SIZE);
      CASE (ADDRESS_SPACE_INFO_NULL_ADDRESS);
      CASE (ADDRESS_SPACE_INFO_ACCESS);
    }
  return to_string (make_hex (address_space_info));
}

template <>
std::string
to_string (amd_dbgapi_address_space_alias_t address_space_alias)
{
  switch (address_space_alias)
    {
      CASE (ADDRESS_SPACE_ALIAS_NONE);
      CASE (ADDRESS_SPACE_ALIAS_MAY);
    }
  return to_string (make_hex (address_space_alias));
}

template <>
std::string
to_string (amd_dbgapi_address_class_state_t address_class_state)
{
  switch (address_class_state)
    {
      CASE (ADDRESS_CLASS_STATE_NOT_MEMBER);
      CASE (ADDRESS_CLASS_STATE_MEMBER);
    }
  return to_string (make_hex (address_class_state));
}

template <>
std::string
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
std::string
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
std::string
to_string (amd_dbgapi_runtime_state_t runtime_state)
{
  switch (runtime_state)
    {
      CASE (RUNTIME_STATE_LOADED_SUCCESS);
      CASE (RUNTIME_STATE_UNLOADED);
      CASE (RUNTIME_STATE_LOADED_ERROR_RESTRICTION);
    }
  return to_string (make_hex (runtime_state));
}

template <>
std::string
to_string (amd_dbgapi_event_info_t event_info)
{
  switch (event_info)
    {
      CASE (EVENT_INFO_PROCESS);
      CASE (EVENT_INFO_KIND);
      CASE (EVENT_INFO_WAVE);
      CASE (EVENT_INFO_BREAKPOINT);
      CASE (EVENT_INFO_CLIENT_THREAD);
      CASE (EVENT_INFO_RUNTIME_STATE);
    }
  return to_string (make_hex (event_info));
}

template <>
std::string
to_string (amd_dbgapi_log_level_t log_level)
{
  switch (log_level)
    {
      CASE (LOG_LEVEL_NONE);
      CASE (LOG_LEVEL_FATAL_ERROR);
      CASE (LOG_LEVEL_WARNING);
      CASE (LOG_LEVEL_INFO);
      CASE (LOG_LEVEL_VERBOSE);
    }
  return to_string (make_hex (log_level));
}

template <>
std::string
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
std::string
to_string (amd_dbgapi_breakpoint_action_t breakpoint_action)
{
  switch (breakpoint_action)
    {
      CASE (BREAKPOINT_ACTION_RESUME);
      CASE (BREAKPOINT_ACTION_HALT);
    }
  return to_string (make_hex (breakpoint_action));
}

#undef CASE

} /* namespace amd::dbgapi */

void AMD_DBGAPI
amd_dbgapi_set_log_level (amd_dbgapi_log_level_t level)
{
  TRACE (level);
  amd::dbgapi::log_level = level;
}
