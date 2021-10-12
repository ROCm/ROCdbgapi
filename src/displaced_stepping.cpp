/* Copyright (c) 2019-2021 Advanced Micro Devices, Inc.

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
#include "debug.h"
#include "exception.h"
#include "initialization.h"
#include "logging.h"
#include "process.h"
#include "queue.h"
#include "utils.h"
#include "wave.h"

#include <utility>

namespace amd::dbgapi
{

displaced_stepping_t::displaced_stepping_t (
  amd_dbgapi_displaced_stepping_id_t displaced_stepping_id, queue_t &queue,
  amd_dbgapi_global_address_t original_pc, instruction_t original_instruction,
  bool simulate, instruction_buffer_t instruction_buffer)
  : handle_object (displaced_stepping_id), m_is_simulated (simulate),
    m_from (original_pc),
    m_instruction_buffer (std::move (instruction_buffer)),
    m_original_instruction (std::move (original_instruction)), m_queue (queue)
{
  dbgapi_assert (m_original_instruction.is_valid ());

  dbgapi_log (AMD_DBGAPI_LOG_LEVEL_INFO, "created new %s (from=%#lx, %s)",
              to_string (id ()).c_str (), from (),
              m_is_simulated ? "simulated"
                             : string_printf ("to=[%#lx..%#lx[", to (),
                                              m_instruction_buffer->end ())
                                 .c_str ());
}

displaced_stepping_t::~displaced_stepping_t ()
{
  dbgapi_assert (m_reference_count == 0
                 && "all displaced stepping operations should have completed");

  dbgapi_log (AMD_DBGAPI_LOG_LEVEL_INFO, "destructed %s",
              to_string (id ()).c_str ());
}

void
displaced_stepping_t::get_info (amd_dbgapi_displaced_stepping_info_t query,
                                size_t value_size, void *value) const
{
  switch (query)
    {
    case AMD_DBGAPI_DISPLACED_STEPPING_INFO_PROCESS:
      utils::get_info (value_size, value, process ().id ());
      return;
    }

  throw api_error_t (AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT);
}

void
displaced_stepping_t::retain (displaced_stepping_t *displaced_stepping)
{
  dbgapi_assert (displaced_stepping);

  [[maybe_unused]] auto prev_count = displaced_stepping->m_reference_count++;
  dbgapi_assert (displaced_stepping->m_reference_count > prev_count);
}

void
displaced_stepping_t::release (displaced_stepping_t *displaced_stepping)
{
  dbgapi_assert (displaced_stepping
                 && displaced_stepping->m_reference_count > 0);

  if (--displaced_stepping->m_reference_count == 0)
    displaced_stepping->process ().destroy (displaced_stepping);
}

} /* namespace amd::dbgapi */

using namespace amd::dbgapi;

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_displaced_stepping_start (
  amd_dbgapi_wave_id_t wave_id, const void *saved_instruction_bytes,
  amd_dbgapi_displaced_stepping_id_t *displaced_stepping_id)
{
  TRACE_BEGIN (param_in (wave_id),
               make_hex (make_ref (param_in (saved_instruction_bytes), 4)),
               param_in (displaced_stepping_id));
  TRY
  {
  if (!detail::is_initialized)
      THROW (AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED);

  if (!saved_instruction_bytes || !displaced_stepping_id)
      THROW (AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT);

  wave_t *wave = find (wave_id);

  if (!wave)
      THROW (AMD_DBGAPI_STATUS_ERROR_INVALID_WAVE_ID);

  if (wave->state () != AMD_DBGAPI_WAVE_STATE_STOP)
      THROW (AMD_DBGAPI_STATUS_ERROR_WAVE_NOT_STOPPED);

  /* Already displaced stepping?  */
  if (wave->displaced_stepping ())
      THROW (AMD_DBGAPI_STATUS_ERROR_DISPLACED_STEPPING_ACTIVE);

  /* wave_t::displaced_stepping_start writes registers, so we need the queue
     to be suspended.  (FIXME: Can we check if the instruction is
     simulated?)  */
    scoped_queue_suspend_t suspend (wave->queue (),
                                    "displaced stepping start");

    /* Find the wave again, after suspending the queue, to determine if the
       wave has terminated.  */
  if (!(wave = find (wave_id)))
      THROW (AMD_DBGAPI_STATUS_ERROR_INVALID_WAVE_ID);

  wave->displaced_stepping_start (saved_instruction_bytes);

  *displaced_stepping_id = wave->displaced_stepping ()->id ();

  return AMD_DBGAPI_STATUS_SUCCESS;
  }
  CATCH (AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED,
         AMD_DBGAPI_STATUS_ERROR_INVALID_WAVE_ID,
         AMD_DBGAPI_STATUS_ERROR_WAVE_NOT_STOPPED,
         AMD_DBGAPI_STATUS_ERROR_DISPLACED_STEPPING_ACTIVE,
         AMD_DBGAPI_STATUS_ERROR_DISPLACED_STEPPING_BUFFER_NOT_AVAILABLE,
         AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT,
         AMD_DBGAPI_STATUS_ERROR_MEMORY_ACCESS,
         AMD_DBGAPI_STATUS_ERROR_ILLEGAL_INSTRUCTION);
  TRACE_END (make_ref (param_out (displaced_stepping_id)));
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_displaced_stepping_complete (
  amd_dbgapi_wave_id_t wave_id,
  amd_dbgapi_displaced_stepping_id_t displaced_stepping_id)
{
  TRACE_BEGIN (param_in (wave_id), param_in (displaced_stepping_id));
  TRY
  {
  if (!detail::is_initialized)
      THROW (AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED);

  wave_t *wave = find (wave_id);

  if (!wave)
      THROW (AMD_DBGAPI_STATUS_ERROR_INVALID_WAVE_ID);

  if (wave->state () != AMD_DBGAPI_WAVE_STATE_STOP)
      THROW (AMD_DBGAPI_STATUS_ERROR_WAVE_NOT_STOPPED);

  displaced_stepping_t *displaced_stepping = find (displaced_stepping_id);

  if (!displaced_stepping)
      THROW (AMD_DBGAPI_STATUS_ERROR_INVALID_DISPLACED_STEPPING_ID);

  /* Not displaced stepping or stepping with a different displaced stepping
     buffer?  */
  if (wave->displaced_stepping () != displaced_stepping)
      THROW (AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT_COMPATIBILITY);

  /* displaced_stepping_t::complete may write uncached registers, so we need
     the queue to be suspended.  (FIXME: Can we check if the instruction is
     simulated?)  */
  scoped_queue_suspend_t suspend (wave->queue (),
                                  "displaced stepping complete");

    /* Find the wave again, after suspending the queue, to determine if the
       wave has terminated.  */
  if (!(wave = find (wave_id)))
      THROW (AMD_DBGAPI_STATUS_ERROR_INVALID_WAVE_ID);

  wave->displaced_stepping_complete ();

  return AMD_DBGAPI_STATUS_SUCCESS;
  }
  CATCH (AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED,
         AMD_DBGAPI_STATUS_ERROR_INVALID_WAVE_ID,
         AMD_DBGAPI_STATUS_ERROR_INVALID_DISPLACED_STEPPING_ID,
         AMD_DBGAPI_STATUS_ERROR_WAVE_NOT_STOPPED,
         AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT_COMPATIBILITY);
  TRACE_END ();
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_displaced_stepping_get_info (
  amd_dbgapi_displaced_stepping_id_t displaced_stepping_id,
  amd_dbgapi_displaced_stepping_info_t query, size_t value_size, void *value)
{
  TRACE_BEGIN (param_in (displaced_stepping_id), param_in (query),
               param_in (value_size), param_in (value));
  TRY
  {
  if (!detail::is_initialized)
    THROW (AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED);

  displaced_stepping_t *displaced_stepping = find (displaced_stepping_id);

  if (!displaced_stepping)
    THROW (AMD_DBGAPI_STATUS_ERROR_INVALID_DISPLACED_STEPPING_ID);

  displaced_stepping->get_info (query, value_size, value);

  return AMD_DBGAPI_STATUS_SUCCESS;
  }
  CATCH (AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED,
         AMD_DBGAPI_STATUS_ERROR_INVALID_DISPLACED_STEPPING_ID,
         AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT,
         AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT_COMPATIBILITY,
         AMD_DBGAPI_STATUS_ERROR_CLIENT_CALLBACK);
  TRACE_END (make_query_ref (query, param_out (value)));
}
