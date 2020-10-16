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

#include "displaced_stepping.h"
#include "architecture.h"
#include "debug.h"
#include "initialization.h"
#include "logging.h"
#include "process.h"
#include "queue.h"
#include "register.h"
#include "utils.h"
#include "wave.h"

#include <cstdint>
#include <cstring>
#include <optional>

namespace amd::dbgapi
{

displaced_stepping_t::displaced_stepping_t (
    amd_dbgapi_displaced_stepping_id_t displaced_stepping_id, queue_t &queue,
    amd_dbgapi_global_address_t from, const void *saved_instruction_bytes)
    : handle_object (displaced_stepping_id), m_from (from), m_queue (queue)
{
  const architecture_t &architecture = this->architecture ();
  const std::vector<uint8_t> &breakpoint_instruction
      = architecture.breakpoint_instruction ();

  /* Keep a copy of the original instruction bytes, we may need it to
     complete the displaced stepping.  */

  m_original_instruction.resize (architecture.largest_instruction_size ());
  memcpy (m_original_instruction.data (), saved_instruction_bytes,
          breakpoint_instruction.size ());

  size_t offset = breakpoint_instruction.size ();
  size_t remaining_size = m_original_instruction.size () - offset;

  if (auto status = process ().read_global_memory_partial (
          from + offset, m_original_instruction.data () + offset,
          &remaining_size);
      status != AMD_DBGAPI_STATUS_SUCCESS)
    throw exception_t (status);

  /* Trim unread bytes.  */
  m_original_instruction.resize (offset + remaining_size);

  /* Trim to size of instruction.  */
  size_t instruction_size
      = architecture.instruction_size (m_original_instruction);

  if (!instruction_size)
    {
      /* If instruction_size is 0, the disassembler did not recognize the
         instruction.  This instruction may be non-sequencial, and we won't be
         able to tell if the jump is relative or absolute.  */
      throw exception_t (AMD_DBGAPI_STATUS_ERROR_ILLEGAL_INSTRUCTION);
    }

  m_original_instruction.resize (instruction_size);

  /* Copy a single instruction to the displaced stepping buffer.  */
  amd_dbgapi_status_t status
      = architecture.displaced_stepping_copy (*this, &m_is_simulated);
  if (status != AMD_DBGAPI_STATUS_SUCCESS)
    throw exception_t (status);

  m_is_valid = true;
}

amd_dbgapi_status_t
displaced_stepping_t::get_info (amd_dbgapi_displaced_stepping_info_t query,
                                size_t value_size, void *value) const
{
  switch (query)
    {
    case AMD_DBGAPI_DISPLACED_STEPPING_INFO_PROCESS:
      return utils::get_info (value_size, value, process ().id ());
    }

  return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;
}

} /* namespace amd::dbgapi */

using namespace amd::dbgapi;

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_displaced_stepping_start (
    amd_dbgapi_wave_id_t wave_id, const void *saved_instruction_bytes,
    amd_dbgapi_displaced_stepping_id_t *displaced_stepping_id)
{
  TRY;
  TRACE (wave_id);

  if (!detail::is_initialized)
    return AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED;

  if (!saved_instruction_bytes || !displaced_stepping_id)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;

  wave_t *wave = find (wave_id);

  if (!wave)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_WAVE_ID;

  if (wave->state () != AMD_DBGAPI_WAVE_STATE_STOP)
    return AMD_DBGAPI_STATUS_ERROR_WAVE_NOT_STOPPED;

  /* Already displaced stepping?  */
  if (wave->displaced_stepping ())
    return AMD_DBGAPI_STATUS_ERROR; /* FIXME: Check error code.  */

  /* wave_t::displaced_stepping_start writes registers, so we need the queue
     to be suspended.  (FIXME: Can we check if the instruction is
     simulated?)  */
  scoped_queue_suspend_t suspend (wave->queue (), "displaced stepping start");

  /* Find the wave again, after suspending the queue, to determine if the wave
     has terminated.  */
  if (!(wave = find (wave_id)))
    return AMD_DBGAPI_STATUS_ERROR_INVALID_WAVE_ID;

  amd_dbgapi_status_t status
      = wave->displaced_stepping_start (saved_instruction_bytes);
  if (status != AMD_DBGAPI_STATUS_SUCCESS)
    return status;

  /* TODO: We could handle trivial step-overs (e.g. branches) and return
     AMD_DBGAPI_DISPLACED_STEPPING_NONE.  In that case, the wave does not
     need to be single-stepped to step over the breakpoint.  */
  *displaced_stepping_id = wave->displaced_stepping ()->id ();

  return status;
  CATCH;
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_displaced_stepping_complete (
    amd_dbgapi_wave_id_t wave_id,
    amd_dbgapi_displaced_stepping_id_t displaced_stepping_id)
{
  TRY;
  TRACE (wave_id, displaced_stepping_id);

  if (!detail::is_initialized)
    return AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED;

  wave_t *wave = find (wave_id);

  if (!wave)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_WAVE_ID;

  /* Not displaced stepping?  */
  if (!wave->displaced_stepping ())
    return AMD_DBGAPI_STATUS_ERROR; /* FIXME: Check error code.  */

  if (wave->state () != AMD_DBGAPI_WAVE_STATE_STOP)
    return AMD_DBGAPI_STATUS_ERROR_WAVE_NOT_STOPPED;

  displaced_stepping_t *displaced_stepping = find (displaced_stepping_id);

  if (!displaced_stepping)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_DISPLACED_STEPPING_ID;

  if (wave->displaced_stepping () != displaced_stepping)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT_COMPATIBILITY;

  /* displaced_stepping_t::complete may write uncached registers, so we need
     the queue to be suspended.  (FIXME: Can we check if the instruction is
     simulated?)  */
  scoped_queue_suspend_t suspend (wave->queue (),
                                  "displaced stepping complete");

  /* Find the wave again, after suspending the queue, to determine if the wave
     has terminated.  */
  if (!(wave = find (wave_id)))
    return AMD_DBGAPI_STATUS_ERROR_INVALID_WAVE_ID;

  return wave->displaced_stepping_complete ();
  CATCH;
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_code_displaced_stepping_get_info (
    amd_dbgapi_displaced_stepping_id_t displaced_stepping_id,
    amd_dbgapi_displaced_stepping_info_t query, size_t value_size, void *value)
{
  TRY;
  TRACE (displaced_stepping_id, query, value_size, value);

  if (!detail::is_initialized)
    return AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED;

  displaced_stepping_t *displaced_stepping = find (displaced_stepping_id);

  if (!displaced_stepping)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_DISPLACED_STEPPING_ID;

  return displaced_stepping->get_info (query, value_size, value);
  CATCH;
}
