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

#include "wave.h"
#include "architecture.h"
#include "dispatch.h"
#include "displaced_stepping.h"
#include "event.h"
#include "exception.h"
#include "initialization.h"
#include "logging.h"
#include "memory.h"
#include "os_driver.h"
#include "process.h"
#include "queue.h"
#include "register.h"
#include "utils.h"
#include "watchpoint.h"

#include <algorithm>
#include <cstring>
#include <optional>
#include <string>
#include <utility>

namespace amd::dbgapi
{

wave_t::wave_t (amd_dbgapi_wave_id_t wave_id, const dispatch_t &dispatch)
  : handle_object (wave_id), m_dispatch (dispatch)
{
}

wave_t::~wave_t ()
{
  if (displaced_stepping ())
    {
      /* displaced step operations are cancelled by the process on detach,
         unless the process has exited and the queue is invalid, in which case,
         we simply release the displaced stepping buffer.  */
      dbgapi_assert (!queue ().is_valid ());
      displaced_stepping_t::release (m_displaced_stepping);
    }
}

void
wave_t::set_visibility (visibility_t visibility)
{
  if (m_visibility == visibility)
    return;

  dbgapi_log (AMD_DBGAPI_LOG_LEVEL_INFO, "changing %s's visibility to %s",
              to_string (id ()).c_str (),
              visibility == visibility_t::visible ? "visible" : "hidden");

  m_visibility = visibility;

  /* Since the visibility of this wave has changed, the list of waves returned
     by the process has also changed.  */
  process ().set_changed<wave_t> (true);
}

uint64_t
wave_t::exec_mask () const
{

  if (lane_count () == 32)
    {
      uint32_t exec;
      read_register (amdgpu_regnum_t::exec_32, &exec);
      return exec;
    }
  else if (lane_count () == 64)
    {
      uint64_t exec;
      read_register (amdgpu_regnum_t::exec_64, &exec);
      return exec;
    }
  fatal_error ("Not a valid lane_count for EXEC mask: %zu", lane_count ());
}

amd_dbgapi_global_address_t
wave_t::pc () const
{
  amd_dbgapi_global_address_t pc;
  read_register (amdgpu_regnum_t::pc, &pc);
  return pc;
}

std::optional<instruction_t>
wave_t::instruction_at_pc (size_t pc_adjust) const
{
  size_t size = architecture ().largest_instruction_size ();
  std::vector<std::byte> instruction_bytes (size);

  try
    {
      size = process ().read_global_memory_partial (
        pc () + pc_adjust, instruction_bytes.data (), size);
    }
  catch (...)
    {
      return std::nullopt;
    }

  /* Trim partial and unread words.  */
  instruction_bytes.resize (size);

  return instruction_t (architecture (), std::move (instruction_bytes));
}

void
wave_t::park ()
{
  dbgapi_assert (state () == AMD_DBGAPI_WAVE_STATE_STOP
                 && "Cannot park a running wave");

  dbgapi_assert (!m_is_parked && "already parked");

  /* On architectures that do not support halting at certain instructions when
     a wave is stopped, for example a terminating instruction, we change its pc
     to point to an immutable trap instruction.  This guarantees that the
     wave will never be halted at such instructions.  */
  m_parked_pc = pc ();

  amd_dbgapi_global_address_t parked_pc = queue ().park_instruction_address ();
  write_register (amdgpu_regnum_t::pc, &parked_pc);

  m_is_parked = true;
  /* From now on, every read/write to the pc register will be from/to
     m_parked_pc.  The real pc in the context save area will be untouched.  */

  dbgapi_log (AMD_DBGAPI_LOG_LEVEL_VERBOSE, "parked %s (pc=%#lx)",
              to_string (id ()).c_str (), m_parked_pc);
}

void
wave_t::unpark ()
{
  dbgapi_assert (state () != AMD_DBGAPI_WAVE_STATE_STOP
                 && "Cannot unpark a stopped wave");

  dbgapi_assert (m_is_parked && "not parked");

  amd_dbgapi_global_address_t saved_pc = pc ();

  m_is_parked = false;
  /* From now on, every read/write to the pc register will be from/to the
     context save area.  */

  write_register (amdgpu_regnum_t::pc, &saved_pc);

  dbgapi_log (AMD_DBGAPI_LOG_LEVEL_VERBOSE, "unparked %s (pc=%#lx)",
              to_string (id ()).c_str (), pc ());
}

void
wave_t::terminate ()
{
  if (m_displaced_stepping)
    {
      displaced_stepping_t::release (m_displaced_stepping);
      m_displaced_stepping = nullptr;
    }

  /* Mark the wave as invalid and un-halt it at a terminating instruction. This
     allows the hardware to terminate the wave, while ensuring that the wave is
     never reported to the client as existing.  */

  amd_dbgapi_global_address_t terminate_pc
    = queue ().terminating_instruction_address ();

  /* Make the PC point to an immutable terminating instruction.  */
  write_register (amdgpu_regnum_t::pc, &terminate_pc);

  /* Hide this wave so that it isn't reported to the client.  */
  set_visibility (wave_t::visibility_t::hidden_at_terminating_instruction);

  set_state (AMD_DBGAPI_WAVE_STATE_RUN);
}

void
wave_t::displaced_stepping_start (const void *saved_instruction_bytes)
{
  dbgapi_assert (!m_displaced_stepping && "already displaced stepping");
  dbgapi_assert (state () == AMD_DBGAPI_WAVE_STATE_STOP && "not stopped");

  /* Check if we already have a displaced stepping buffer for this pc
     that can be shared between waves associated with the same queue.
   */
  displaced_stepping_t *displaced_stepping = process ().find_if (
    [&] (const displaced_stepping_t &other)
    { return other.queue () == queue () && other.from () == pc (); });

  if (displaced_stepping)
    {
      displaced_stepping_t::retain (displaced_stepping);
    }
  else
    {
      /* If we can't share a displaced stepping operation with another
         wave, create a new one.  */

      /* Reconstitute the original instruction bytes.  */
      std::vector<std::byte> original_instruction_bytes (
        architecture ().largest_instruction_size ());

      memcpy (original_instruction_bytes.data (), saved_instruction_bytes,
              architecture ().breakpoint_instruction ().size ());

      size_t offset = architecture ().breakpoint_instruction ().size ();
      size_t remaining = original_instruction_bytes.size () - offset;

      remaining = process ().read_global_memory_partial (
        pc () + offset, &original_instruction_bytes[offset], remaining);

      /* Trim partial/unread bytes.  */
      original_instruction_bytes.resize (offset + remaining);

      instruction_t original_instruction (
        architecture (), std::move (original_instruction_bytes));

      bool simulate
        = architecture ().can_simulate (*this, original_instruction);

      if (!architecture ().can_execute_displaced (*this, original_instruction)
          && !simulate)
        {
          /* If this instruction cannot be displaced-stepped nor simulated,
             then it must be inline-stepped.  */
          throw api_error_t (AMD_DBGAPI_STATUS_ERROR_ILLEGAL_INSTRUCTION);
        }

      instruction_buffer_t instruction_buffer{};

      if (!simulate)
        {
          instruction_buffer = queue ().allocate_instruction_buffer ();
          instruction_buffer->resize (original_instruction.size ());
          amd_dbgapi_global_address_t instruction_addr
            = instruction_buffer->begin ();

          process ().write_global_memory (instruction_addr,
                                          original_instruction.data (),
                                          original_instruction.size ());
        }

      displaced_stepping = &process ().create<displaced_stepping_t> (
        queue (), pc (), std::move (original_instruction), simulate,
        std::move (instruction_buffer));
    }

  if (!displaced_stepping->is_simulated ())
    {
      amd_dbgapi_global_address_t displaced_pc = displaced_stepping->to ();
      dbgapi_assert (displaced_pc != amd_dbgapi_global_address_t{});

      write_register (amdgpu_regnum_t::pc, &displaced_pc);

      dbgapi_log (AMD_DBGAPI_LOG_LEVEL_INFO,
                  "changing %s's pc from %#lx to %#lx (started %s)",
                  to_string (id ()).c_str (), displaced_stepping->from (),
                  displaced_stepping->to (),
                  to_string (displaced_stepping->id ()).c_str ());
    }

  m_displaced_stepping = displaced_stepping;
}

void
wave_t::displaced_stepping_complete ()
{
  dbgapi_assert (!!m_displaced_stepping && "not displaced stepping");
  dbgapi_assert (state () == AMD_DBGAPI_WAVE_STATE_STOP && "not stopped");

  if (!m_displaced_stepping->is_simulated ())
    {
      amd_dbgapi_global_address_t displaced_pc = pc ();
      amd_dbgapi_global_address_t restored_pc = displaced_pc
                                                + m_displaced_stepping->from ()
                                                - m_displaced_stepping->to ();
      write_register (amdgpu_regnum_t::pc, &restored_pc);

      dbgapi_log (AMD_DBGAPI_LOG_LEVEL_INFO,
                  "changing %s's pc from %#lx to %#lx (%s %s)",
                  to_string (id ()).c_str (), displaced_pc, pc (),
                  displaced_pc == m_displaced_stepping->to () ? "aborted"
                                                              : "completed",
                  to_string (m_displaced_stepping->id ()).c_str ());
    }

  displaced_stepping_t::release (m_displaced_stepping);
  m_displaced_stepping = nullptr;
}

void
wave_t::update (const wave_t &group_leader,
                std::unique_ptr<architecture_t::cwsr_record_t> cwsr_record)
{
  dbgapi_assert (queue ().is_suspended ());
  const bool first_update = !m_mark;

  dbgapi_assert (cwsr_record != nullptr);
  m_cwsr_record = std::move (cwsr_record);
  m_group_leader = &group_leader;

  /* Update the wave's state if this is a new wave, or if the wave was running
     the last time the queue it belongs to was resumed.  */
  amd_dbgapi_wave_state_t prev_state = m_state;
  if (prev_state != AMD_DBGAPI_WAVE_STATE_STOP)
    {
      /* Zero-initialize the ttmp registers if they weren't set up by the
         hardware.  Some ttmp registers are used to determine if the wave was
         stopped by the trap handler because of an exception or a trap.  */
      bool ttmps_initialized
        = process ().is_flag_set (process_t::flag_t::ttmps_setup_enabled)
          || agent ().os_info ().ttmps_always_initialized;

      if (first_update && !ttmps_initialized)
        {
          const uint32_t zero = 0;
          for (auto regnum = amdgpu_regnum_t::first_ttmp;
               regnum <= amdgpu_regnum_t::last_ttmp; ++regnum)
            write_register (regnum, &zero);
        }

      std::tie (m_state, m_stop_reason)
        = architecture ().wave_get_state (*this);
    }

  dbgapi_log (AMD_DBGAPI_LOG_LEVEL_VERBOSE,
              "%s %s%s (pc=%#lx, state=%s) "
              "context_save:[%#lx..%#lx[",
              first_update ? "created" : "updated",
              visibility () != visibility_t::visible ? "invisible " : "",
              to_string (id ()).c_str (), pc (), to_string (m_state).c_str (),
              m_cwsr_record->begin (), m_cwsr_record->end ());

  /* The wave was running, and it is now stopped.  */
  if (prev_state != AMD_DBGAPI_WAVE_STATE_STOP
      && m_state == AMD_DBGAPI_WAVE_STATE_STOP)
    {
      /* Park the wave if the architecture does not support halting at certain
         instructions.  */
      if (architecture ().park_stopped_waves ())
        park ();

      if (visibility () == visibility_t::visible
          && m_stop_reason != AMD_DBGAPI_WAVE_STOP_REASON_NONE)
        raise_event (AMD_DBGAPI_EVENT_KIND_WAVE_STOP);
    }

  /* If this is the first time we update this wave, store the wave_id, and load
     the immutable state from the ttmp registers (group_ids, wave_in_group,
     scratch_offset).  */
  if (first_update)
    {
      /* Write the wave_id register.  */
      amd_dbgapi_wave_id_t wave_id = id ();
      write_register (amdgpu_regnum_t::wave_id, &wave_id);

      /* Read group_ids[0:3].  */
      read_register (amdgpu_regnum_t::dispatch_grid, 0, sizeof (m_group_ids),
                     &m_group_ids[0]);

      /* Read the wave's position in the thread group.  */
      read_register (amdgpu_regnum_t::wave_in_group, &m_wave_in_group);
    }
}

void
wave_t::set_state (amd_dbgapi_wave_state_t state,
                   amd_dbgapi_exceptions_t exceptions)
{
  dbgapi_assert ((exceptions == AMD_DBGAPI_EXCEPTION_NONE
                  || state != AMD_DBGAPI_WAVE_STATE_STOP)
                 && "raising an exception requires the wave to be resumed");

  const architecture_t &architecture = this->architecture ();
  amd_dbgapi_wave_state_t prev_state = m_state;

  if (state == prev_state)
    return;

  dbgapi_assert (
    (!m_displaced_stepping || state != AMD_DBGAPI_WAVE_STATE_RUN)
    && "displaced-stepping waves can only be stopped or single-stepped");

  m_stop_requested = state == AMD_DBGAPI_WAVE_STATE_STOP;

  std::optional<instruction_t> instruction;
  if (state == AMD_DBGAPI_WAVE_STATE_SINGLE_STEP)
    instruction = instruction_at_pc ();

  /* A wave single-stepping a terminating instruction does not generate a trap
     exception upon executing the instruction, so we need to immediately
     terminate the wave and enqueue an aborted command event.  */
  if (state == AMD_DBGAPI_WAVE_STATE_SINGLE_STEP
      && exceptions == AMD_DBGAPI_EXCEPTION_NONE &&
      [&] ()
      {
        if (m_displaced_stepping)
          /* The displaced instruction is a terminating instruction.  */
          return architecture.is_terminating_instruction (
            m_displaced_stepping->original_instruction ());

        /* The current instruction at pc is a terminating instruction.  */
        return instruction
               && architecture.is_terminating_instruction (*instruction);
      }())
    {
      terminate ();
      raise_event (AMD_DBGAPI_EVENT_KIND_WAVE_COMMAND_TERMINATED);
      return;
    }

  dbgapi_log (AMD_DBGAPI_LOG_LEVEL_INFO,
              "changing %s%s's state from %s to %s %s(pc=%#lx)",
              visibility () != visibility_t::visible ? "invisible " : "",
              to_string (id ()).c_str (), to_string (prev_state).c_str (),
              to_string (state).c_str (),
              exceptions != AMD_DBGAPI_EXCEPTION_NONE
                ? ("with " + to_string (exceptions) + " ").c_str ()
                : "",
              pc ());

  architecture.wave_set_state (*this, state, exceptions);
  m_state = state;
  queue ().wave_state_changed (*this);

  if (architecture.park_stopped_waves ())
    {
      if (state == AMD_DBGAPI_WAVE_STATE_STOP)
        park ();
      else
        unpark ();
    }

  if (state != AMD_DBGAPI_WAVE_STATE_STOP)
    {
      dbgapi_assert (prev_state == AMD_DBGAPI_WAVE_STATE_STOP
                     && "cannot resume an already running wave");

      /* m_last_stopped_pc is used to detect spurious single-step events
         (entered the trap handler with mode.debug_en=1 but pc ==
         m_last_stopped_pc).  Save the pc here as this is the last known
         pc before the wave is unhalted.  */
      m_last_stopped_pc = pc ();

      /* Clear the stop reason.  */
      m_stop_reason = AMD_DBGAPI_WAVE_STOP_REASON_NONE;
    }
  else if (prev_state != AMD_DBGAPI_WAVE_STATE_STOP)
    {
      /* We requested the wave be stopped, and the wave wasn't already stopped,
         report an event to acknowledge that the wave has stopped.  */

      m_stop_reason = AMD_DBGAPI_WAVE_STOP_REASON_NONE;

      if (visibility () == visibility_t::visible)
        raise_event (prev_state == AMD_DBGAPI_WAVE_STATE_SINGLE_STEP
                       ? AMD_DBGAPI_EVENT_KIND_WAVE_COMMAND_TERMINATED
                       : AMD_DBGAPI_EVENT_KIND_WAVE_STOP);
    }

  if (state == AMD_DBGAPI_WAVE_STATE_SINGLE_STEP
      && exceptions == AMD_DBGAPI_EXCEPTION_NONE &&
      [&] ()
      {
        /* Simulate the instruction if the wave is displaced-stepping and the
           instruction requires simulation (for example, instruction that
           manipulate the program counter). */
        if (m_displaced_stepping)
          return m_displaced_stepping->is_simulated ()
                 && architecture.simulate (
                   *this, m_displaced_stepping->from (),
                   m_displaced_stepping->original_instruction ());

        /* Simulate all instructions that can be simulated.  */
        return instruction && architecture.can_simulate (*this, *instruction)
               && architecture.simulate (*this, pc (), *instruction);
      }())
    {
      /* The instruction was simulated, get the new wave state and raise a stop
         event.  */
      update (*m_group_leader, std::move (m_cwsr_record));
      queue ().wave_state_changed (*this);
    }

  if (exceptions != AMD_DBGAPI_EXCEPTION_NONE)
    {
      auto convert_one_exception = [&] (amd_dbgapi_exceptions_t one_exception)
      {
        if (one_exception == AMD_DBGAPI_EXCEPTION_WAVE_ABORT)
          return os_exception_mask_t::queue_wave_abort;

        if (one_exception == AMD_DBGAPI_EXCEPTION_WAVE_TRAP)
          return os_exception_mask_t::queue_wave_trap;

        if (one_exception == AMD_DBGAPI_EXCEPTION_WAVE_MATH_ERROR)
          return os_exception_mask_t::queue_wave_math_error;

        if (one_exception == AMD_DBGAPI_EXCEPTION_WAVE_ILLEGAL_INSTRUCTION)
          return os_exception_mask_t::queue_wave_illegal_instruction;

        if (one_exception == AMD_DBGAPI_EXCEPTION_WAVE_MEMORY_VIOLATION)
          return os_exception_mask_t::queue_wave_memory_violation
                 | (agent ().exceptions ()
                    & os_exception_mask_t::device_memory_violation);

        if (one_exception == AMD_DBGAPI_EXCEPTION_WAVE_APERTURE_VIOLATION)
          return os_exception_mask_t::queue_wave_aperture_violation;

        dbgapi_assert_not_reached ("not a valid exception");
      };

      /* Convert an amd_dbgapi_exception_t into an os_exception_mask_t.  */
      os_exception_mask_t os_exceptions = os_exception_mask_t::none;

      while (exceptions)
        {
          auto one_exception = exceptions ^ (exceptions & (exceptions - 1));
          os_exceptions |= convert_one_exception (one_exception);
          exceptions ^= one_exception;
        }

      /* A wave should only send queue exceptions, sometimes combined with a
         device_memory_exception.  */
      dbgapi_assert ((os_exceptions & os_queue_exception_mask) != 0);

      process ().send_exceptions (os_exceptions, &queue ());
    }
}

bool
wave_t::is_register_available (amdgpu_regnum_t regnum) const
{
  if (is_pseudo_register (regnum))
    return architecture ().is_pseudo_register_available (*this, regnum);

  return register_address (regnum).has_value ();
}

void
wave_t::read_register (amdgpu_regnum_t regnum, size_t offset,
                       size_t value_size, void *value) const
{
  if (is_pseudo_register (regnum))
    return architecture ().read_pseudo_register (*this, regnum, offset,
                                                 value_size, value);

  dbgapi_assert (value_size
                 && (offset + value_size)
                      <= architecture ().register_size (regnum)
                 && "read_register is out of bounds");

  auto reg_addr = register_address (regnum);

  /* Out of range sgpr, read s0.  */
  if (!reg_addr
      && (regnum >= amdgpu_regnum_t::first_sgpr
          && regnum <= amdgpu_regnum_t::last_sgpr))
    reg_addr = register_address (amdgpu_regnum_t::s0);

  /* Out of range vgpr, read v0.  */
  if (!reg_addr
      && (regnum >= amdgpu_regnum_t::first_vgpr
          && regnum <= amdgpu_regnum_t::last_vgpr))
    reg_addr = register_address (lane_count () == 32 ? amdgpu_regnum_t::v0_32
                                                     : amdgpu_regnum_t::v0_64);

  dbgapi_assert (reg_addr);

  if (m_is_parked && regnum == amdgpu_regnum_t::pc)
    {
      memcpy (static_cast<char *> (value) + offset,
              reinterpret_cast<const char *> (&m_parked_pc) + offset,
              value_size);
      return;
    }

  std::optional<scoped_queue_suspend_t> suspend;
  if (!queue ().is_suspended ()
      && !process ().memory_cache ().contains_all (*reg_addr + offset,
                                                   value_size))
    {
      suspend.emplace (queue (), "read register");

      /* Look for the wave_id again, the wave may have exited.  */
      wave_t *wave = find (id ());
      if (!wave)
        throw api_error_t (AMD_DBGAPI_STATUS_ERROR_INVALID_WAVE_ID);

      dbgapi_assert (wave == this);

      /* The wave's saved state may have changed location in memory.  */
      reg_addr = register_address (regnum);
    }

  process ().read_global_memory (
    *reg_addr + offset, static_cast<char *> (value) + offset, value_size);
}

void
wave_t::write_register (amdgpu_regnum_t regnum, size_t offset,
                        size_t value_size, const void *value)
{
  if (is_pseudo_register (regnum))
    return architecture ().write_pseudo_register (*this, regnum, offset,
                                                  value_size, value);

  dbgapi_assert (value_size
                 && (offset + value_size)
                      <= architecture ().register_size (regnum)
                 && "write_register is out of bounds");

  auto reg_addr = register_address (regnum);

  if (!reg_addr
      && ((regnum >= amdgpu_regnum_t::first_sgpr
           && regnum <= amdgpu_regnum_t::last_sgpr)
          || (regnum >= amdgpu_regnum_t::first_vgpr
              && regnum <= amdgpu_regnum_t::last_vgpr)))
    /* Out of range sgpr or vgpr, the register write is dropped.  */
    return;

  dbgapi_assert (reg_addr);

  if (m_is_parked && regnum == amdgpu_regnum_t::pc)
    {
      memcpy (reinterpret_cast<char *> (&m_parked_pc) + offset,
              static_cast<const char *> (value) + offset, value_size);
      return;
    }

  std::optional<scoped_queue_suspend_t> suspend;
  if (!queue ().is_suspended ())
    {
      suspend.emplace (queue (), "write register");

      /* Look for the wave_id again, the wave may have exited.  */
      wave_t *wave = find (id ());
      if (!wave)
        throw api_error_t (AMD_DBGAPI_STATUS_ERROR_INVALID_WAVE_ID);

      dbgapi_assert (wave == this);

      /* The wave's saved state may have changed location in memory.  */
      reg_addr = register_address (regnum);
    }

  process ().write_global_memory (*reg_addr + offset,
                                  static_cast<const char *> (value) + offset,
                                  value_size);
}

size_t
wave_t::xfer_private_memory_swizzled (
  amd_dbgapi_segment_address_t segment_address, amd_dbgapi_lane_id_t lane_id,
  void *read, const void *write, size_t size)
{
  dbgapi_assert (lane_id != AMD_DBGAPI_LANE_NONE && lane_id < lane_count ());

  const amd_dbgapi_size_t interleave
    = architecture ().private_swizzled_interleave_size ();
  auto [scratch_base, scratch_size] = scratch_memory_region ();

  size_t bytes = size;
  while (bytes > 0)
    {
      /* Transfer one aligned dword at a time, except for the first (or last)
         read which could read less than a dword if the start (or end) address
         is not aligned.  */

      size_t request_size
        = std::min (interleave - (segment_address % interleave), bytes);
      size_t xfer_size = request_size;

      amd_dbgapi_size_t offset
        = ((segment_address / interleave) * lane_count () * interleave)
          + (lane_id * interleave) + (segment_address % interleave);

      if ((offset + xfer_size) > scratch_size)
        {
          xfer_size = offset < scratch_size ? scratch_size - offset : 0;
          if (xfer_size == 0)
            throw memory_access_error_t (scratch_base + scratch_size);
        }

      amd_dbgapi_global_address_t global_address = scratch_base + offset;

      xfer_size = read ? process ().read_global_memory_partial (
                    global_address, read, xfer_size)
                       : process ().write_global_memory_partial (
                         global_address, write, xfer_size);

      bytes -= xfer_size;
      if (request_size != xfer_size)
        throw memory_access_error_t (global_address + xfer_size);

      if (read)
        read = static_cast<char *> (read) + xfer_size;
      else
        write = static_cast<const char *> (write) + xfer_size;

      segment_address += xfer_size;
    }

  return size - bytes;
}

size_t
wave_t::xfer_private_memory_unswizzled (
  amd_dbgapi_segment_address_t segment_address, void *read, const void *write,
  size_t size)
{
  auto [scratch_base, scratch_size] = scratch_memory_region ();

  if ((segment_address + size) > scratch_size)
    {
      size_t max_size
        = segment_address < scratch_size ? scratch_size - segment_address : 0;
      if (max_size == 0 && size != 0)
        throw memory_access_error_t (scratch_base + scratch_size);
      size = max_size;
    }

  amd_dbgapi_global_address_t global_address = scratch_base + segment_address;

  return read
           ? process ().read_global_memory_partial (global_address, read, size)
           : process ().write_global_memory_partial (global_address, write,
                                                     size);
}

size_t
wave_t::xfer_local_memory (amd_dbgapi_segment_address_t segment_address,
                           void *read, const void *write, size_t size)
{
  /* The LDS is stored in the context save area.  */
  std::optional<scoped_queue_suspend_t> suspend;
  if (!queue ().is_suspended ())
    {
      suspend.emplace (queue (), "xfer local memory");

      /* Look for the wave_id again, the wave may have exited.  */
      wave_t *wave = find (id ());
      if (!wave)
        throw api_error_t (AMD_DBGAPI_STATUS_ERROR_INVALID_WAVE_ID);

      dbgapi_assert (wave == this);
    }

  auto local_memory_base_address
    = group_leader ().m_cwsr_record->register_address (amdgpu_regnum_t::lds_0);

  if (!local_memory_base_address)
    fatal_error ("local memory is not accessible");

  amd_dbgapi_size_t limit = m_cwsr_record->lds_size ();
  amd_dbgapi_size_t offset = segment_address;

  if ((offset + size) > limit)
    {
      size_t max_size = offset < limit ? limit - offset : 0;
      if (max_size == 0 && size != 0)
        throw memory_access_error_t (*local_memory_base_address + limit);
      size = max_size;
    }

  amd_dbgapi_global_address_t global_address
    = *local_memory_base_address + offset;

  return read
           ? process ().read_global_memory_partial (global_address, read, size)
           : process ().write_global_memory_partial (global_address, write,
                                                     size);
}

size_t
wave_t::xfer_segment_memory (const address_space_t &address_space,
                             amd_dbgapi_lane_id_t lane_id,
                             amd_dbgapi_segment_address_t segment_address,
                             void *read, const void *write, size_t size)
{
  dbgapi_assert (state () == AMD_DBGAPI_WAVE_STATE_STOP
                 && "the wave must be stopped to read/write memory");
  dbgapi_assert (!read != !write && "either read or write buffer");

  /* Zero-extend the segment address.  */
  segment_address &= utils::bit_mask (0, address_space.address_size () - 1);

  switch (address_space.kind ())
    {
    case address_space_t::kind_t::private_swizzled:
      return xfer_private_memory_swizzled (segment_address, lane_id, read,
                                           write, size);

    case address_space_t::kind_t::private_unswizzled:
      return xfer_private_memory_unswizzled (segment_address, read, write,
                                             size);

    case address_space_t::kind_t::local:
      return xfer_local_memory (segment_address, read, write, size);

    case address_space_t::kind_t::global:
      return read ? process ().read_global_memory_partial (segment_address,
                                                           read, size)
                  : process ().write_global_memory_partial (segment_address,
                                                            write, size);

    default:
      throw memory_access_error_t (string_printf (
        "xfer_segment_memory from address space `%s' not supported",
        address_space.name ().c_str ()));
    }
}

void
wave_t::raise_event (amd_dbgapi_event_kind_t event_kind)
{
  process_t &process = this->process ();
  event_t &event = process.create<event_t> (process, event_kind, id ());

  if (event_kind == AMD_DBGAPI_EVENT_KIND_WAVE_COMMAND_TERMINATED
      || event_kind == AMD_DBGAPI_EVENT_KIND_WAVE_STOP)
    m_last_stop_event_id = event.id ();

  process.enqueue_event (event);
}

const event_t *
wave_t::last_stop_event () const
{
  dbgapi_assert (state () == AMD_DBGAPI_WAVE_STATE_STOP);
  return process ().find (m_last_stop_event_id);
}

amd_dbgapi_wave_state_t
wave_t::client_visible_state () const
{
  amd_dbgapi_wave_state_t state = this->state ();

  if (state != AMD_DBGAPI_WAVE_STATE_STOP)
    return state;

  if (const event_t *event = last_stop_event ();
      !event || event->state () >= event_t::state_t::reported)
    return AMD_DBGAPI_WAVE_STATE_STOP;

  /* If the wave is stopped, but the wave stop event has not yet been
     reported to the client, return the last resumed state.  */
  return (stop_reason () & AMD_DBGAPI_WAVE_STOP_REASON_SINGLE_STEP)
           ? AMD_DBGAPI_WAVE_STATE_SINGLE_STEP
           : AMD_DBGAPI_WAVE_STATE_RUN;
}

void
wave_t::get_info (amd_dbgapi_wave_info_t query, size_t value_size,
                  void *value) const
{
  switch (query)
    {
    case AMD_DBGAPI_WAVE_INFO_STATE:
      utils::get_info (value_size, value, client_visible_state ());
      return;

    case AMD_DBGAPI_WAVE_INFO_STOP_REASON:
      utils::get_info (value_size, value, stop_reason ());
      return;

    case AMD_DBGAPI_WAVE_INFO_DISPATCH:
      if (dispatch ().id () == AMD_DBGAPI_DISPATCH_NONE)
        throw api_error_t (AMD_DBGAPI_STATUS_ERROR_NOT_AVAILABLE);
      utils::get_info (value_size, value, dispatch ().id ());
      return;

    case AMD_DBGAPI_WAVE_INFO_QUEUE:
      utils::get_info (value_size, value, queue ().id ());
      return;

    case AMD_DBGAPI_WAVE_INFO_AGENT:
      utils::get_info (value_size, value, agent ().id ());
      return;

    case AMD_DBGAPI_WAVE_INFO_PROCESS:
      utils::get_info (value_size, value, process ().id ());
      return;

    case AMD_DBGAPI_WAVE_INFO_ARCHITECTURE:
      utils::get_info (value_size, value, architecture ().id ());
      return;

    case AMD_DBGAPI_WAVE_INFO_PC:
      utils::get_info (value_size, value, pc ());
      return;

    case AMD_DBGAPI_WAVE_INFO_EXEC_MASK:
      utils::get_info (value_size, value, exec_mask ());
      return;

    case AMD_DBGAPI_WAVE_INFO_WORK_GROUP_COORD:
      if (dispatch ().id () == AMD_DBGAPI_DISPATCH_NONE)
        throw api_error_t (AMD_DBGAPI_STATUS_ERROR_NOT_AVAILABLE);
      utils::get_info (value_size, value, m_group_ids);
      return;

    case AMD_DBGAPI_WAVE_INFO_WAVE_NUMBER_IN_WORK_GROUP:
      if (dispatch ().id () == AMD_DBGAPI_DISPATCH_NONE)
        throw api_error_t (AMD_DBGAPI_STATUS_ERROR_NOT_AVAILABLE);
      utils::get_info (value_size, value, m_wave_in_group);
      return;

    case AMD_DBGAPI_WAVE_INFO_WATCHPOINTS:
      {
        auto os_watch_ids = architecture ().triggered_watchpoints (*this);

        auto watchpoint_ids = allocate_memory<amd_dbgapi_watchpoint_id_t[]> (
          os_watch_ids.size () * sizeof (amd_dbgapi_watchpoint_id_t));

        auto watchpoint_id = [this] (os_watch_id_t os_watch_id)
        {
          const watchpoint_t *watchpoint
            = process ().find_watchpoint (os_watch_id);
          if (!watchpoint)
            fatal_error ("kfd_watch_%d not set on %s", os_watch_id,
                         to_string (agent ().id ()).c_str ());
          return watchpoint->id ();
        };

        std::transform (os_watch_ids.begin (), os_watch_ids.end (),
                        watchpoint_ids.get (), watchpoint_id);

        utils::get_info (value_size, value,
                         amd_dbgapi_watchpoint_list_t{
                           os_watch_ids.size (), watchpoint_ids.get () });

        watchpoint_ids.release ();
        return;
      }

    case AMD_DBGAPI_WAVE_INFO_LANE_COUNT:
      utils::get_info (value_size, value, lane_count ());
      return;
    }

  throw api_error_t (AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT);
}

} /* namespace amd::dbgapi */

using namespace amd::dbgapi;

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_wave_stop (amd_dbgapi_wave_id_t wave_id)
{
  TRACE_BEGIN (param_in (wave_id));
  TRY
  {
    if (!detail::is_initialized)
      THROW (AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED);

    wave_t *wave = find (wave_id);

    if (!wave)
      THROW (AMD_DBGAPI_STATUS_ERROR_INVALID_WAVE_ID);

    if (wave->client_visible_state () == AMD_DBGAPI_WAVE_STATE_STOP)
      THROW (AMD_DBGAPI_STATUS_ERROR_WAVE_STOPPED);

    if (wave->stop_requested ())
      THROW (AMD_DBGAPI_STATUS_ERROR_WAVE_OUTSTANDING_STOP);

    scoped_queue_suspend_t suspend (wave->queue (), "stop wave");

    /* Look for the wave_id again, the wave may have exited.  */
    if (!(wave = find (wave_id)))
      THROW (AMD_DBGAPI_STATUS_ERROR_INVALID_WAVE_ID);

    wave->set_state (AMD_DBGAPI_WAVE_STATE_STOP);

    return AMD_DBGAPI_STATUS_SUCCESS;
  }
  CATCH (AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED,
         AMD_DBGAPI_STATUS_ERROR_INVALID_WAVE_ID,
         AMD_DBGAPI_STATUS_ERROR_WAVE_STOPPED,
         AMD_DBGAPI_STATUS_ERROR_WAVE_OUTSTANDING_STOP);
  TRACE_END ();
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_wave_resume (amd_dbgapi_wave_id_t wave_id,
                        amd_dbgapi_resume_mode_t resume_mode,
                        amd_dbgapi_exceptions_t exceptions)
{
  TRACE_BEGIN (param_in (wave_id), param_in (resume_mode),
               param_in (exceptions));
  TRY
  {
    if (!detail::is_initialized)
      THROW (AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED);

    wave_t *wave = find (wave_id);

    if (!wave)
      THROW (AMD_DBGAPI_STATUS_ERROR_INVALID_WAVE_ID);

    if (resume_mode != AMD_DBGAPI_RESUME_MODE_NORMAL
        && resume_mode != AMD_DBGAPI_RESUME_MODE_SINGLE_STEP)
      THROW (AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT);

    if ((exceptions
         & ~(AMD_DBGAPI_EXCEPTION_WAVE_ABORT | AMD_DBGAPI_EXCEPTION_WAVE_TRAP
             | AMD_DBGAPI_EXCEPTION_WAVE_MATH_ERROR
             | AMD_DBGAPI_EXCEPTION_WAVE_ILLEGAL_INSTRUCTION
             | AMD_DBGAPI_EXCEPTION_WAVE_MEMORY_VIOLATION
             | AMD_DBGAPI_EXCEPTION_WAVE_APERTURE_VIOLATION))
        != AMD_DBGAPI_EXCEPTION_NONE)
      THROW (AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT);

    if (wave->client_visible_state () != AMD_DBGAPI_WAVE_STATE_STOP)
      THROW (AMD_DBGAPI_STATUS_ERROR_WAVE_NOT_STOPPED);

    /* The wave is not resumable if the stop event is not yet processed.  */
    if (const event_t *event = wave->last_stop_event ();
        event && event->state () < event_t::state_t::processed)
      THROW (AMD_DBGAPI_STATUS_ERROR_WAVE_NOT_RESUMABLE);

    if (wave->displaced_stepping ()
        && resume_mode != AMD_DBGAPI_RESUME_MODE_SINGLE_STEP)
      THROW (AMD_DBGAPI_STATUS_ERROR_RESUME_DISPLACED_STEPPING);

    scoped_queue_suspend_t suspend (wave->queue (), "resume wave");

    /* Look for the wave_id again, the wave may have exited.  */
    if (!(wave = find (wave_id)))
      THROW (AMD_DBGAPI_STATUS_ERROR_INVALID_WAVE_ID);

    wave->set_state (resume_mode == AMD_DBGAPI_RESUME_MODE_SINGLE_STEP
                       ? AMD_DBGAPI_WAVE_STATE_SINGLE_STEP
                       : AMD_DBGAPI_WAVE_STATE_RUN,
                     exceptions);

    return AMD_DBGAPI_STATUS_SUCCESS;
  }
  CATCH (AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED,
         AMD_DBGAPI_STATUS_ERROR_INVALID_WAVE_ID,
         AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT,
         AMD_DBGAPI_STATUS_ERROR_WAVE_NOT_STOPPED,
         AMD_DBGAPI_STATUS_ERROR_WAVE_NOT_RESUMABLE,
         AMD_DBGAPI_STATUS_ERROR_RESUME_DISPLACED_STEPPING);
  TRACE_END ();
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_wave_get_info (amd_dbgapi_wave_id_t wave_id,
                          amd_dbgapi_wave_info_t query, size_t value_size,
                          void *value)
{
  TRACE_BEGIN (param_in (wave_id), param_in (query), param_in (value_size),
               param_in (value));
  TRY
  {
    if (!detail::is_initialized)
      THROW (AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED);

    wave_t *wave = find (wave_id);

    if (!wave)
      THROW (AMD_DBGAPI_STATUS_ERROR_INVALID_WAVE_ID);

    switch (query)
      {
      case AMD_DBGAPI_WAVE_INFO_STOP_REASON:
      case AMD_DBGAPI_WAVE_INFO_PC:
      case AMD_DBGAPI_WAVE_INFO_EXEC_MASK:
      case AMD_DBGAPI_WAVE_INFO_WATCHPOINTS:
        if (wave->client_visible_state () != AMD_DBGAPI_WAVE_STATE_STOP)
          THROW (AMD_DBGAPI_STATUS_ERROR_WAVE_NOT_STOPPED);
      default:
        break;
      };

    wave->get_info (query, value_size, value);

    return AMD_DBGAPI_STATUS_SUCCESS;
  }
  CATCH (AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED,
         AMD_DBGAPI_STATUS_ERROR_INVALID_WAVE_ID,
         AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT,
         AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT_COMPATIBILITY,
         AMD_DBGAPI_STATUS_ERROR_NOT_AVAILABLE,
         AMD_DBGAPI_STATUS_ERROR_WAVE_NOT_STOPPED,
         AMD_DBGAPI_STATUS_ERROR_CLIENT_CALLBACK);
  TRACE_END (make_query_ref (query, param_out (value)));
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_process_wave_list (amd_dbgapi_process_id_t process_id,
                              size_t *wave_count, amd_dbgapi_wave_id_t **waves,
                              amd_dbgapi_changed_t *changed)
{
  TRACE_BEGIN (param_in (process_id), param_in (wave_count), param_in (waves),
               param_in (changed));
  TRY
  {
    if (!detail::is_initialized)
      THROW (AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED);

    std::vector<process_t *> processes = process_t::match (process_id);

    if (!waves || !wave_count)
      THROW (AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT);

    std::vector<std::pair<process_t *, std::vector<queue_t *>>>
      queues_needing_resume;

    for (auto &&process : processes)
      {
        process->update_queues ();

        std::vector<queue_t *> queues;
        for (auto &&queue : process->range<queue_t> ())
          if (!queue.is_suspended ())
            queues.emplace_back (&queue);

        process->suspend_queues (queues, "refresh wave list");

        if (process->forward_progress_needed ())
          queues_needing_resume.emplace_back (process, std::move (queues));
      }

    amd_dbgapi_changed_t wave_list_changed;
    auto wave_list = utils::get_handle_list<wave_t> (
      processes, changed ? &wave_list_changed : nullptr);

    auto deallocate_wave_list = utils::make_scope_fail (
      [&] () { amd::dbgapi::deallocate_memory (wave_list.first); });

    for (auto &&[process, queues] : queues_needing_resume)
      process->resume_queues (queues, "refresh wave list");

    std::tie (*waves, *wave_count) = wave_list;
    if (changed)
      *changed = wave_list_changed;

    return AMD_DBGAPI_STATUS_SUCCESS;
  }
  CATCH (AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED,
         AMD_DBGAPI_STATUS_ERROR_INVALID_PROCESS_ID,
         AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT,
         AMD_DBGAPI_STATUS_ERROR_CLIENT_CALLBACK);
  TRACE_END (make_ref (param_out (wave_count)),
             make_ref (make_ref (param_out (waves)), *wave_count),
             make_ref (param_out (changed)));
}
