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

#include "amd-dbgapi.h"
#include "logging.h"
#include "utils.h"

#include <cstdint>

const char AMD_DBGAPI *
amd_dbgapi_get_build_name ()
{
  TRACE ();

  return AMD_DBGAPI_BUILD_INFO;
}

void AMD_DBGAPI
amd_dbgapi_get_version (uint32_t *major, uint32_t *minor, uint32_t *patch)
{
  TRACE ();

  if (major)
    *major = AMD_DBGAPI_VERSION_MAJOR;
  if (minor)
    *minor = AMD_DBGAPI_VERSION_MINOR;
  if (patch)
    *patch = AMD_DBGAPI_VERSION_PATCH;
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_get_status_string (amd_dbgapi_status_t status,
                              const char **status_string)
{
  TRY;
  TRACE (status);

  const char *string = nullptr;
  switch (status)
    {
    case AMD_DBGAPI_STATUS_SUCCESS:
      string = "The function has executed successfully";
      break;
    case AMD_DBGAPI_STATUS_ERROR:
      string = "A generic error has occurred";
      break;
    case AMD_DBGAPI_STATUS_FATAL:
      string = "A fatal error has occurred";
      break;
    case AMD_DBGAPI_STATUS_ERROR_UNIMPLEMENTED:
      string = "The operation is not currently implemented";
      break;
    case AMD_DBGAPI_STATUS_ERROR_NOT_SUPPORTED:
      string = "The operation is not supported";
      break;
    case AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT:
      string = "An invalid argument was given to the function";
      break;
    case AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT_COMPATIBILITY:
      string = "An invalid combination of arguments was given to the function";
      break;
    case AMD_DBGAPI_STATUS_ERROR_ALREADY_INITIALIZED:
      string = "The library is already initialized";
      break;
    case AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED:
      string = "The library is not initialized";
      break;
    case AMD_DBGAPI_STATUS_ERROR_RESTRICTION:
      string = "There is a restriction that prevents debugging";
      break;
    case AMD_DBGAPI_STATUS_ERROR_ALREADY_ATTACHED:
      string = "The process is already attached to the given inferior process";
      break;
    case AMD_DBGAPI_STATUS_ERROR_INVALID_ARCHITECTURE_ID:
      string = "The architecture handle is invalid";
      break;
    case AMD_DBGAPI_STATUS_ERROR_ILLEGAL_INSTRUCTION:
      string = "The bytes being disassembled are not a legal instruction";
      break;
    case AMD_DBGAPI_STATUS_ERROR_INVALID_CODE_OBJECT_ID:
      string = "The code object handle is invalid";
      break;
    case AMD_DBGAPI_STATUS_ERROR_INVALID_ELF_AMDGPU_MACHINE:
      string = "The ELF AMD GPU machine value is invalid or unsupported";
      break;
    case AMD_DBGAPI_STATUS_ERROR_INVALID_PROCESS_ID:
      string = "The process handle is invalid";
      break;
    case AMD_DBGAPI_STATUS_ERROR_INVALID_AGENT_ID:
      string = "The agent handle is invalid";
      break;
    case AMD_DBGAPI_STATUS_ERROR_INVALID_QUEUE_ID:
      string = "The queue handle is invalid";
      break;
    case AMD_DBGAPI_STATUS_ERROR_INVALID_DISPATCH_ID:
      string = "The dispatch handle is invalid";
      break;
    case AMD_DBGAPI_STATUS_ERROR_INVALID_WAVE_ID:
      string = "The wave handle is invalid";
      break;
    case AMD_DBGAPI_STATUS_ERROR_WAVE_NOT_STOPPED:
      string = "The wave is not stopped";
      break;
    case AMD_DBGAPI_STATUS_ERROR_WAVE_STOPPED:
      string = "The wave is stopped";
      break;
    case AMD_DBGAPI_STATUS_ERROR_WAVE_OUTSTANDING_STOP:
      string = "The wave has an outstanding stop request";
      break;
    case AMD_DBGAPI_STATUS_ERROR_WAVE_NOT_RESUMABLE:
      string = "The wave cannot be resumed";
      break;
    case AMD_DBGAPI_STATUS_ERROR_INVALID_DISPLACED_STEPPING_ID:
      string = "The displaced stepping handle is invalid";
      break;
    case AMD_DBGAPI_STATUS_ERROR_DISPLACED_STEPPING_BUFFER_UNAVAILABLE:
      string = "No more displaced stepping buffers are available that "
               "are suitable for the requested wave";
      break;
    case AMD_DBGAPI_STATUS_ERROR_INVALID_WATCHPOINT_ID:
      string = "The watchpoint handle is invalid";
      break;
    case AMD_DBGAPI_STATUS_ERROR_NO_WATCHPOINT_AVAILABLE:
      string = "No more watchpoints available";
      break;
    case AMD_DBGAPI_STATUS_ERROR_INVALID_REGISTER_CLASS_ID:
      string = "The register class handle is invalid";
      break;
    case AMD_DBGAPI_STATUS_ERROR_INVALID_REGISTER_ID:
      string = "The register handle is invalid";
      break;
    case AMD_DBGAPI_STATUS_ERROR_INVALID_LANE_ID:
      string = "The lane handle is invalid";
      break;
    case AMD_DBGAPI_STATUS_ERROR_INVALID_ADDRESS_CLASS_ID:
      string = "The address class handle is invalid";
      break;
    case AMD_DBGAPI_STATUS_ERROR_INVALID_ADDRESS_SPACE_ID:
      string = "The address space handle is invalid";
      break;
    case AMD_DBGAPI_STATUS_ERROR_MEMORY_ACCESS:
      string
          = "An error occurred while trying to access memory in the inferior";
      break;
    case AMD_DBGAPI_STATUS_ERROR_INVALID_ADDRESS_SPACE_CONVERSION:
      string = "The segment address cannot be converted to the "
               "requested address space";
      break;
    case AMD_DBGAPI_STATUS_ERROR_INVALID_EVENT_ID:
      string = "The event handle is invalid";
      break;
    case AMD_DBGAPI_STATUS_ERROR_INVALID_SHARED_LIBRARY_ID:
      string = "The shared library handle is invalid";
      break;
    case AMD_DBGAPI_STATUS_ERROR_INVALID_BREAKPOINT_ID:
      string = "The breakpoint handle is invalid";
      break;
    case AMD_DBGAPI_STATUS_ERROR_CLIENT_CALLBACK:
      string = "A callback to the client reported an error";
      break;
    case AMD_DBGAPI_STATUS_ERROR_INVALID_CLIENT_PROCESS_ID:
      string = "The client process handle is invalid";
      break;
    case AMD_DBGAPI_STATUS_ERROR_PROCESS_EXITED:
      string = "The native operating system process associated with a "
               "client process has exited";
      break;
    case AMD_DBGAPI_STATUS_ERROR_LIBRARY_NOT_LOADED:
      string = "The shared library is not currently loaded";
      break;
    case AMD_DBGAPI_STATUS_ERROR_SYMBOL_NOT_FOUND:
      string = "The symbol was not found";
      break;
    case AMD_DBGAPI_STATUS_ERROR_INVALID_ADDRESS:
      string = "The address is not within the shared library";
      break;
    case AMD_DBGAPI_STATUS_ERROR_DISPLACED_STEPPING_ACTIVE:
      string = "The wave has an active displaced stepping buffer";
      break;
      /* Don't add a default here, so that we can catch at compile time when an
          enum value is missing.  */
    }

  if (!string)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;

  *status_string = string;
  return AMD_DBGAPI_STATUS_SUCCESS;
  CATCH;
}
