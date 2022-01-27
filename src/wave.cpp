/* Copyright (c) 2019-2022 Advanced Micro Devices, Inc.

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
#include "workgroup.h"

#include <algorithm>
#include <cstring>
#include <optional>
#include <string>
#include <utility>

namespace amd::dbgapi
{

wave_t::wave_t (amd_dbgapi_wave_id_t wave_id, workgroup_t &workgroup,
                std::optional<uint32_t> wave_in_group)
  : handle_object (wave_id), m_wave_in_group (wave_in_group),
    m_workgroup (workgroup)
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

  /* If the wave was single-stepping, the client is expecting either a stop
     event, or a command terminated event.  */
  if (state () == AMD_DBGAPI_WAVE_STATE_SINGLE_STEP)
    raise_event (AMD_DBGAPI_EVENT_KIND_WAVE_COMMAND_TERMINATED);
}

const dispatch_t &
wave_t::dispatch () const
{
  return m_workgroup.dispatch ();
}

compute_queue_t &
wave_t::queue () const
{
  return dispatch ().queue ();
}

const agent_t &
wave_t::agent () const
{
  return queue ().agent ();
}

process_t &
wave_t::process () const
{
  return agent ().process ();
}

const architecture_t &
wave_t::architecture () const
{
  return queue ().architecture ();
}

bool
wave_t::is_halted () const
{
  return architecture ().wave_get_halt (*this);
}

void
wave_t::set_halted (bool halted)
{
  architecture ().wave_set_halt (*this, halted);
}

void
wave_t::set_visibility (visibility_t visibility)
{
  if (m_visibility == visibility)
    return;

  log_info ("changing %s's visibility to %s", to_cstring (id ()),
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
  size_t instruction_size = architecture ().largest_instruction_size ();
  std::vector<std::byte> instruction_bytes (instruction_size);

  amd_dbgapi_global_address_t instruction_pc = pc () + pc_adjust;
  dbgapi_assert (utils::is_aligned (
    instruction_pc, architecture ().minimum_instruction_alignment ()));

  try
    {
      instruction_size = process ().read_global_memory_partial (
        instruction_pc, instruction_bytes.data (), instruction_size);
    }
  catch (...)
    {
      return std::nullopt;
    }

  /* Trim partial and unread bytes.  */
  instruction_bytes.resize (instruction_size);

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

  write_register (amdgpu_regnum_t::pc, queue ().park_instruction_address ());

  m_is_parked = true;
  /* From now on, every read/write to the pc register will be from/to
     m_parked_pc.  The real pc in the context save area will be untouched.  */

  log_verbose ("parked %s (pc=%#lx)", to_cstring (id ()), m_parked_pc);
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

  write_register (amdgpu_regnum_t::pc, saved_pc);

  log_verbose ("unparked %s (pc=%#lx)", to_cstring (id ()), pc ());
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

  /* Make the PC point to an immutable terminating instruction.  */
  write_register (amdgpu_regnum_t::pc,
                  queue ().terminating_instruction_address ());

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

  /* If we can't share a displaced stepping operation with another wave, create
     a new one.  */
  if (!displaced_stepping)
    {
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

      std::optional<compute_queue_t::displaced_instruction_ptr_t>
        displaced_instruction_ptr;

      if (architecture ().can_simulate (*this, original_instruction))
        {
          /* Since this instruction will be simulated when the wave is resumed,
             there is no need to allocate a displaced stepping buffer.  */
        }
      else if (architecture ().can_execute_displaced (*this,
                                                      original_instruction))
        {
          displaced_instruction_ptr.emplace (
            queue ().allocate_displaced_instruction (original_instruction));
        }
      else
        {
          /* If this instruction cannot be simulated nor displaced-stepped,
             then it must be inline-stepped.  */
          throw api_error_t (AMD_DBGAPI_STATUS_ERROR_ILLEGAL_INSTRUCTION);
        }

      displaced_stepping = &process ().create<displaced_stepping_t> (
        queue (), std::move (original_instruction), pc (),
        std::move (displaced_instruction_ptr));
    }

  if (displaced_stepping->to ())
    {
      write_register (amdgpu_regnum_t::pc, *displaced_stepping->to ());

      log_info ("changing %s's pc from %#lx to %#lx (started %s)",
                to_cstring (id ()), displaced_stepping->from (),
                *displaced_stepping->to (),
                to_cstring (displaced_stepping->id ()));
    }

  displaced_stepping_t::retain (displaced_stepping);
  m_displaced_stepping = displaced_stepping;
}

void
wave_t::displaced_stepping_complete ()
{
  dbgapi_assert (m_displaced_stepping && "not displaced stepping");
  dbgapi_assert (state () == AMD_DBGAPI_WAVE_STATE_STOP && "not stopped");

  if (m_displaced_stepping->to ())
    {
      amd_dbgapi_global_address_t displaced_pc = pc ();
      amd_dbgapi_global_address_t restored_pc = displaced_pc
                                                + m_displaced_stepping->from ()
                                                - *m_displaced_stepping->to ();
      write_register (amdgpu_regnum_t::pc, restored_pc);

      log_info ("changing %s's pc from %#lx to %#lx (%s %s)",
                to_cstring (id ()), displaced_pc, pc (),
                displaced_pc == *m_displaced_stepping->to () ? "aborted"
                                                             : "completed",
                to_cstring (m_displaced_stepping->id ()));
    }

  displaced_stepping_t::release (m_displaced_stepping);
  m_displaced_stepping = nullptr;
}

void
wave_t::update (std::unique_ptr<architecture_t::cwsr_record_t> cwsr_record)
{
  dbgapi_assert (queue ().is_suspended ());
  const bool first_update = !m_mark;

  dbgapi_assert (cwsr_record != nullptr);
  m_cwsr_record = std::move (cwsr_record);

  /* Check that the PC in the wave state save area is correctly aligned.  */
  if (!utils::is_aligned (pc (),
                          architecture ().minimum_instruction_alignment ()))
    fatal_error ("corrupted state for %s: misaligned pc: %#lx",
                 to_cstring (id ()), pc ());

  /* Update the wave's state if this is a new wave, or if the wave was running
     the last time the queue it belongs to was resumed.  */
  amd_dbgapi_wave_state_t prev_state = m_state;
  if (prev_state != AMD_DBGAPI_WAVE_STATE_STOP)
    {
      /* Zero-initialize the ttmp registers if they weren't set up by the
         hardware.  Some ttmp registers are used to determine if the wave was
         stopped by the trap handler because of an exception or a trap.  */
      if (first_update && !agent ().ttmps_initialized ())
        {
          for (auto regnum = amdgpu_regnum_t::first_ttmp;
               regnum <= amdgpu_regnum_t::last_ttmp; ++regnum)
            write_register (regnum, uint32_t{ 0 });
        }

      std::tie (m_state, m_stop_reason)
        = architecture ().wave_get_state (*this);
    }

  log_verbose ("%s %s%s in %s (pc=%#lx, state=%s) context_save:[%#lx..%#lx[",
               first_update ? "created" : "updated",
               visibility () != visibility_t::visible ? "invisible " : "",
               to_cstring (id ()), to_cstring (workgroup ().id ()), pc (),
               to_cstring (m_state), m_cwsr_record->begin (),
               m_cwsr_record->end ());

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

  /* If this is the first time we update this wave, store the wave_id.  */
  if (first_update)
    write_register (amdgpu_regnum_t::wave_id, id ());
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

  log_info ("changing %s%s's state from %s to %s %s(pc=%#lx)",
            visibility () != visibility_t::visible ? "invisible " : "",
            to_cstring (id ()), to_cstring (prev_state), to_cstring (state),
            exceptions != AMD_DBGAPI_EXCEPTION_NONE
              ? ("with " + to_string (exceptions) + " ").c_str ()
              : "",
            pc ());

  architecture.wave_set_state (*this, state);
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
        [&] () -> bool /* Return true if the instruction was simulated.  */
      {
        if (m_displaced_stepping)
          {
            /* Simulate displaced instructions that are position sensitive (for
               example, instructions that manipulate the program counter).
               Displaced stepping buffers for such instructions do not have a
               to () address.  */

            return !m_displaced_stepping->to ()
                   && architecture.simulate (
                     *this, m_displaced_stepping->from (),
                     m_displaced_stepping->original_instruction ());
          }

        /* Simulate all instructions that can be simulated.  */
        return instruction && architecture.can_simulate (*this, *instruction)
               && architecture.simulate (*this, pc (), *instruction);
      }())
    {
      /* The instruction was successfully executed, update wave state and
         raise a stop event.  */
      update (std::move (m_cwsr_record));
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

      /* Halt the wave if resuming with exceptions.  */
      architecture.wave_set_halt (*this, true);

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
      memcpy (value, reinterpret_cast<const char *> (&m_parked_pc) + offset,
              value_size);
      return;
    }

  std::optional<scoped_queue_suspend_t> suspend;
  if (!queue ().is_suspended ()
      && !process ().memory_cache ().contains_all (*reg_addr + offset,
                                                   value_size))
    {
      /* Get the wave_id before suspending the queue, as this wave could have
         exited, and queue_t::update_waves may destroy this wave_t.  */
      amd_dbgapi_wave_id_t wave_id = id ();

      suspend.emplace (queue (), "read register");

      /* Look for the wave_id again, the wave may have exited.  */
      wave_t *wave = find (wave_id);
      if (!wave)
        throw api_error_t (AMD_DBGAPI_STATUS_ERROR_INVALID_WAVE_ID);

      dbgapi_assert (wave == this);

      /* The wave's saved state may have changed location in memory.  */
      reg_addr = register_address (regnum);
    }

  process ().read_global_memory (*reg_addr + offset, value, value_size);
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
      if (auto *read_only
          = architecture ().register_read_only_mask (amdgpu_regnum_t::pc);
          read_only != nullptr)
        {
          amd_dbgapi_global_address_t pc = m_parked_pc;
          memcpy (reinterpret_cast<char *> (&pc) + offset, value, value_size);

          amd_dbgapi_global_address_t mask
            = *static_cast<const amd_dbgapi_global_address_t *> (read_only);

          m_parked_pc = (pc & ~mask) | (m_parked_pc & mask);
        }
      else
        {
          memcpy (reinterpret_cast<char *> (&m_parked_pc) + offset, value,
                  value_size);
        }

      return;
    }

  std::optional<scoped_queue_suspend_t> suspend;
  if (!queue ().is_suspended ())
    {
      /* Get the wave_id before suspending the queue, as this wave could have
         exited, and queue_t::update_waves may destroy this wave_t.  */
      amd_dbgapi_wave_id_t wave_id = id ();

      suspend.emplace (queue (), "write register");

      /* Look for the wave_id again, the wave may have exited.  */
      wave_t *wave = find (wave_id);
      if (!wave)
        throw api_error_t (AMD_DBGAPI_STATUS_ERROR_INVALID_WAVE_ID);

      dbgapi_assert (wave == this);

      /* The wave's saved state may have changed location in memory.  */
      reg_addr = register_address (regnum);
    }

  /* This register might not be entirely writable.  Read-only bits in the new
     value must be replaced by the current content of the register.  */
  if (auto *read_only = architecture ().register_read_only_mask (regnum);
      read_only != nullptr)
    {
      void *masked_value = alloca (value_size);
      process ().read_global_memory (*reg_addr + offset, masked_value,
                                     value_size);

      if (offset == 0 && value_size == 8)
        {
          /* Optimize the case when writing an entire 64-bit register.  */
          uint64_t mask = *static_cast<const uint64_t *> (read_only);
          uint64_t &x8 = *static_cast<uint64_t *> (masked_value);
          x8 = (*static_cast<const uint64_t *> (value) & ~mask) | (x8 & mask);
        }
      else if (offset == 0 && value_size == 4)
        {
          /* Optimize the case when writing an entire 32-bit register.  */
          uint32_t mask = *static_cast<const uint32_t *> (read_only);
          uint32_t &x4 = *static_cast<uint32_t *> (masked_value);
          x4 = (*static_cast<const uint32_t *> (value) & ~mask) | (x4 & mask);
        }
      else /* General case, mask one byte at a time.  */
        for (size_t i = 0; i < value_size; ++i)
          {
            char mask = static_cast<const char *> (read_only)[offset + i];
            char &x1 = static_cast<char *> (masked_value)[i];
            x1 = (*static_cast<const char *> (value) & ~mask) | (x1 & mask);
          }

      value = masked_value;
    }

  process ().write_global_memory (*reg_addr + offset, value, value_size);
}

size_t
wave_t::xfer_private_memory_swizzled (
  amd_dbgapi_size_t interleave, amd_dbgapi_segment_address_t segment_address,
  amd_dbgapi_lane_id_t lane_id, void *read, const void *write, size_t size)
{
  if (lane_id == AMD_DBGAPI_LANE_NONE || lane_id >= lane_count ())
    THROW (AMD_DBGAPI_STATUS_ERROR_INVALID_LANE_ID);

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

/* Return the wave's scratch memory region (address and size).  */
std::pair<amd_dbgapi_global_address_t /* address */,
          amd_dbgapi_size_t /* size */>
wave_t::scratch_memory_region () const
{
  return queue ().scratch_memory_region (
    m_cwsr_record->shader_engine_id (),
    m_cwsr_record->scratch_scoreboard_id ());
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

  auto [lowered_address_space, lowered_address]
    = address_space.lower (segment_address);

  switch (lowered_address_space.kind ())
    {
    case address_space_t::kind_t::private_swizzled:
      {
        const auto &private_swizzled_address_space
          = static_cast<const private_swizzled_address_space_t &> (
            lowered_address_space);

        return xfer_private_memory_swizzled (
          private_swizzled_address_space.interleave_size (), lowered_address,
          lane_id, read, write, size);
      }

    case address_space_t::kind_t::private_unswizzled:
      return xfer_private_memory_unswizzled (lowered_address, read, write,
                                             size);

    default:
      throw memory_access_error_t (string_printf (
        "xfer_segment_memory from address space `%s' not supported",
        lowered_address_space.name ().c_str ()));
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

    case AMD_DBGAPI_WAVE_INFO_WORKGROUP:
      if (workgroup ().id () == AMD_DBGAPI_WORKGROUP_NONE)
        throw api_error_t (AMD_DBGAPI_STATUS_ERROR_NOT_AVAILABLE);
      utils::get_info (value_size, value, workgroup ().id ());
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

    case AMD_DBGAPI_WAVE_INFO_WORKGROUP_COORD:
      if (!workgroup ().group_ids ())
        throw api_error_t (AMD_DBGAPI_STATUS_ERROR_NOT_AVAILABLE);
      utils::get_info (value_size, value, *workgroup ().group_ids ());
      return;

    case AMD_DBGAPI_WAVE_INFO_WAVE_NUMBER_IN_WORKGROUP:
      if (!m_wave_in_group)
        throw api_error_t (AMD_DBGAPI_STATUS_ERROR_NOT_AVAILABLE);
      utils::get_info (value_size, value, *m_wave_in_group);
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
                         to_cstring (agent ().id ()));
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
  }
  CATCH (AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED,
         AMD_DBGAPI_STATUS_ERROR_INVALID_PROCESS_ID,
         AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT,
         AMD_DBGAPI_STATUS_ERROR_CLIENT_CALLBACK);
  TRACE_END (make_ref (param_out (wave_count)),
             make_ref (make_ref (param_out (waves)), *wave_count),
             make_ref (param_out (changed)));
}
