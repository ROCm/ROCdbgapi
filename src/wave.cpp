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

wave_t::wave_t (amd_dbgapi_wave_id_t wave_id, dispatch_t &dispatch,
                const callbacks_t &callbacks)
    : handle_object (wave_id), m_callbacks (callbacks), m_dispatch (dispatch)
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
  error ("Not a valid lane_count for EXEC mask: %zu", lane_count ());
}

amd_dbgapi_global_address_t
wave_t::pc () const
{
  amd_dbgapi_global_address_t pc;
  read_register (amdgpu_regnum_t::pc, &pc);
  return pc;
}

std::optional<std::vector<uint8_t>>
wave_t::instruction_at_pc () const
{
  size_t size = architecture ().largest_instruction_size ();
  std::vector<uint8_t> instruction_bytes (size);

  amd_dbgapi_status_t status = process ().read_global_memory_partial (
      pc (), instruction_bytes.data (), &size);
  if (status != AMD_DBGAPI_STATUS_SUCCESS)
    return {};

  /* Trim unread bytes.  */
  instruction_bytes.resize (size);

  return instruction_bytes;
}

void
wave_t::park ()
{
  dbgapi_assert (
      (!m_displaced_stepping || !m_displaced_stepping->instruction_buffer ())
      && "Cannot park a wave with a displaced stepping buffer");
  dbgapi_assert (state () == AMD_DBGAPI_WAVE_STATE_STOP
                 && "Cannot park a running wave");

  if (m_is_parked)
    return;

  /* When a wave hits a breakpoint, we change its pc to point to an immutable
     breakpoint instruction.  This guarantees that the wave will never be
     halted at an s_endpgm if the breakpoint is removed and the original
     instruction restored.  */
  m_saved_pc = pc ();

  amd_dbgapi_global_address_t parked_pc = instruction_buffer ()->end ();
  write_register (amdgpu_regnum_t::pc, &parked_pc);

  m_is_parked = true;
}

void
wave_t::unpark ()
{
  dbgapi_assert (m_is_parked && "not parked");

  /* Restore the original pc if the wave was parked.  */
  amd_dbgapi_global_address_t saved_pc = pc ();

  m_is_parked = false;
  write_register (amdgpu_regnum_t::pc, &saved_pc);
}

void
wave_t::terminate ()
{
  if (m_is_parked)
    unpark ();

  if (m_displaced_stepping)
    {
      displaced_stepping_t::release (m_displaced_stepping);
      m_displaced_stepping = nullptr;
    }

  /* Mark the wave as invalid and un-halt it at an s_endpgm instruction. This
     allows the hardware to terminate the wave, while ensuring that the wave is
     never reported to the client as existing.  */

  auto &endpgm_instruction = architecture ().endpgm_instruction ();

  instruction_buffer ()->resize (endpgm_instruction.size ());
  amd_dbgapi_global_address_t terminate_pc = instruction_buffer ()->begin ();

  if (process ().write_global_memory (terminate_pc, endpgm_instruction.data (),
                                      endpgm_instruction.size ())
      != AMD_DBGAPI_STATUS_SUCCESS)
    error ("Could not write the endpgm instruction");

  /* Make the PC point to an immutable s_endpgm instruction.  */
  write_register (amdgpu_regnum_t::pc, &terminate_pc);

  /* Hide this wave so that it isn't reported to the client.  */
  set_visibility (wave_t::visibility_t::hidden_at_endpgm);

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
  displaced_stepping_t *displaced_stepping
      = process ().find_if ([&] (const displaced_stepping_t &other) {
          return other.queue () == queue () && other.from () == pc ();
        });

  if (displaced_stepping)
    {
      displaced_stepping_t::retain (displaced_stepping);
    }
  else
    {
      /* If we can't share a displaced stepping operation with another
         wave, create a new one.  */

      /* Reconstitute the original instruction bytes.  */
      std::vector<uint8_t> original_instruction (
          architecture ().largest_instruction_size ());

      memcpy (original_instruction.data (), saved_instruction_bytes,
              architecture ().breakpoint_instruction ().size ());

      size_t offset = architecture ().breakpoint_instruction ().size ();
      size_t remaining_size = original_instruction.size () - offset;

      amd_dbgapi_status_t status = process ().read_global_memory_partial (
          pc () + offset, original_instruction.data () + offset,
          &remaining_size);
      if (status != AMD_DBGAPI_STATUS_SUCCESS)
        throw exception_t (status);

      /* Trim unread bytes.  */
      original_instruction.resize (offset + remaining_size);

      /* Trim to size of instruction.  */
      size_t instruction_size
          = architecture ().instruction_size (original_instruction);

      if (!instruction_size
          || !architecture ().can_execute_displaced (original_instruction))
        {
          /* If instruction_size is 0, the disassembler did not recognize the
             instruction.  This instruction may be non-sequencial, and we won't
             be able to tell if the jump is relative or absolute.  */
          throw exception_t (AMD_DBGAPI_STATUS_ERROR_ILLEGAL_INSTRUCTION);
        }

      original_instruction.resize (instruction_size);

      bool simulate = architecture ().can_simulate (original_instruction);

      if (!simulate)
        {
          /* Copy a single instruction to the displaced stepping buffer.  */
          instruction_buffer ()->resize (original_instruction.size ());
          amd_dbgapi_global_address_t instruction_addr
              = instruction_buffer ()->begin ();

          /* Make sure we don't copy an instruction in the displaced stepping
             buffer that would require the wave to be parked.  */
          dbgapi_assert (architecture ().can_halt_at (original_instruction));

          if (process ().write_global_memory (instruction_addr,
                                              original_instruction.data (),
                                              original_instruction.size ())
              != AMD_DBGAPI_STATUS_SUCCESS)
            error ("Could not write the displaced instruction");
        }

      displaced_stepping = &process ().create<displaced_stepping_t> (
          queue (), pc (), std::move (original_instruction), simulate,
          simulate ? std::nullopt
                   : std::make_optional (std::move (instruction_buffer ())));
    }

  if (!displaced_stepping->is_simulated ())
    {
      if (m_is_parked)
        unpark ();

      /* A wave should only hold one instruction buffer reference, either
         through the displaced_stepping_t or its own buffer.  This guarantees
         that all waves can get an instruction buffer.  */
      m_instruction_buffer.reset ();

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

  /* Park after releasing the displaced stepping buffer to ensure a buffer will
     be available for parking.  */
  if (state () == AMD_DBGAPI_WAVE_STATE_STOP
      && !architecture ().can_halt_at (instruction_at_pc ()))
    park ();
}

void
wave_t::update (const wave_t &group_leader,
                std::unique_ptr<architecture_t::cwsr_descriptor_t> descriptor)
{
  dbgapi_assert (queue ().is_suspended ());
  process_t &process = this->process ();

  const bool first_update = !m_descriptor;
  m_descriptor = std::move (descriptor);
  m_group_leader = &group_leader;

  if (first_update)
    {
      /* Write the wave_id register.  */
      amd_dbgapi_wave_id_t wave_id = id ();
      write_register (amdgpu_regnum_t::wave_id, &wave_id);

      architecture ().get_wave_coords (*this, m_group_ids, &m_wave_in_group);

      if (dispatch ().is_scratch_enabled ())
        {
          uint32_t scratch_offset;
          read_register (amdgpu_regnum_t::scratch_offset, &scratch_offset);
          m_scratch_offset = scratch_offset;
        }
    }

  /* Update the wave's state if this is a new wave, or if the wave was running
     the last time the queue it belongs to was resumed.  */
  if (m_state != AMD_DBGAPI_WAVE_STATE_STOP)
    {
      /* Save the previous pc before updating the hwregs cache.  */
      if (!first_update)
        m_saved_pc = pc ();

      /* Reload the HW registers cache.  */
      if (process.read_global_memory (
              register_address (amdgpu_regnum_t::first_hwreg).value (),
              &m_hwregs_cache[0], sizeof (m_hwregs_cache))
          != AMD_DBGAPI_STATUS_SUCCESS)
        error ("Could not reload the hwregs cache");

      architecture ().get_wave_state (*this, &m_state, &m_stop_reason);

      if (m_state == AMD_DBGAPI_WAVE_STATE_STOP && m_stop_reason
          && visibility () == visibility_t::visible)
        {
          process.enqueue_event (process.create<event_t> (
              process, AMD_DBGAPI_EVENT_KIND_WAVE_STOP, id ()));
        }
    }
}

void
wave_t::set_state (amd_dbgapi_wave_state_t state)
{
  amd_dbgapi_wave_state_t prev_state = m_state;

  if (state == prev_state)
    return;

  dbgapi_assert (
      (!m_displaced_stepping || state != AMD_DBGAPI_WAVE_STATE_RUN)
      && "displaced-stepping waves can only be stopped or single-stepped");

  /* A wave single-stepping an s_endpgm instruction does not generate a trap
     exception upon executing the instruction, so we need to immediately
     terminate the wave and enqueue an aborted command event.  */
  if (state == AMD_DBGAPI_WAVE_STATE_SINGLE_STEP)
    {
      auto instruction = instruction_at_pc ();
      bool is_endpgm
          = /* the simulated displaced instruction is s_endpgm */ (
                m_displaced_stepping && m_displaced_stepping->is_simulated ()
                && architecture ().is_endpgm (
                    m_displaced_stepping->original_instruction ()))
            || /* the current instruction at pc is s_endpgm  */ (
                instruction && architecture ().is_endpgm (*instruction));

      if (is_endpgm)
        {
          terminate ();

          process ().enqueue_event (process ().create<event_t> (
              process (), AMD_DBGAPI_EVENT_KIND_WAVE_COMMAND_TERMINATED,
              id ()));

          return;
        }
    }

  if (visibility () == visibility_t::visible)
    dbgapi_log (AMD_DBGAPI_LOG_LEVEL_INFO,
                "changing %s's state from %s to %s (pc=%#lx)",
                to_string (id ()).c_str (), to_string (prev_state).c_str (),
                to_string (state).c_str (), pc ());

  /* Single-stepping a simulated displaced instruction does not require the
     wave to run, simulate resuming of the wave as well.  */
  if (state == AMD_DBGAPI_WAVE_STATE_SINGLE_STEP && m_displaced_stepping
      && m_displaced_stepping->is_simulated ())
    {
      if (m_is_parked)
        unpark ();

      architecture ().simulate_instruction (
          *this, m_displaced_stepping->from (),
          m_displaced_stepping->original_instruction ());

      m_state = AMD_DBGAPI_WAVE_STATE_STOP;
      m_stop_reason = AMD_DBGAPI_WAVE_STOP_REASON_SINGLE_STEP;

      process ().enqueue_event (process ().create<event_t> (
          process (), AMD_DBGAPI_EVENT_KIND_WAVE_STOP, id ()));

      /* We don't know where the PC may have landed after simulating the
         instruction, so park now since the wave is halted.  */
      if (!architecture ().can_halt_at (instruction_at_pc ()))
        park ();

      /* No need to flush the register cache, the wave is still halted.  */
      return;
    }

  architecture ().set_wave_state (*this, state);
  m_state = state;

  if (state != AMD_DBGAPI_WAVE_STATE_STOP)
    {
      dbgapi_assert (prev_state == AMD_DBGAPI_WAVE_STATE_STOP
                     && "cannot resume an already running wave");

      /* Restore the original pc if the wave was parked.  */
      if (m_is_parked)
        unpark ();

      /* Clear the stop reason.  */
      m_stop_reason = AMD_DBGAPI_WAVE_STOP_REASON_NONE;
    }
  else if (prev_state != AMD_DBGAPI_WAVE_STATE_STOP)
    {
      /* We requested the wave be stopped, and the wave wasn't already stopped,
         report an event to acknowledge that the wave has stopped.  */

      /* We have to park the wave if we cannot halt at the current pc.  */
      if (!m_is_parked && !architecture ().can_halt_at (instruction_at_pc ()))
        park ();

      m_stop_reason = AMD_DBGAPI_WAVE_STOP_REASON_NONE;

      dbgapi_assert (visibility () == visibility_t::visible
                     && "cannot request a hidden wave to stop");

      process ().enqueue_event (process ().create<event_t> (
          process (), AMD_DBGAPI_EVENT_KIND_WAVE_STOP, id ()));
    }

  /* If the wave was previously stopped, and is now unhalted, we need to commit
     the hwregs cache to the context save area in order to restore the state
     before the queue is resumed.  If the wave was previously running, and is
     now stopped, we really only need to flush the status hwreg to actually
     halt the wave, but writting the entire cache is just as fast, and easier
     to maintain.  */
  if (register_cache_policy == register_cache_policy_t::write_back)
    {
      /* Write back the register cache in memory.  */
      if (process ().write_global_memory (
              register_address (amdgpu_regnum_t::first_hwreg).value (),
              &m_hwregs_cache[0], sizeof (m_hwregs_cache))
          != AMD_DBGAPI_STATUS_SUCCESS)
        error ("Could not write the hwregs cache back to memory");
    }
}

bool
wave_t::is_register_cached (amdgpu_regnum_t regnum) const
{
  auto reg_addr = register_address (regnum);

  if (!reg_addr)
    return false;

  amd_dbgapi_global_address_t hwregs_addr
      = register_address (amdgpu_regnum_t::first_hwreg).value ();

  return *reg_addr >= hwregs_addr
         && *reg_addr < (hwregs_addr + sizeof (m_hwregs_cache));
}

bool
wave_t::is_register_available (amdgpu_regnum_t regnum) const
{
  return register_address (regnum).has_value ();
}

void
wave_t::read_register (amdgpu_regnum_t regnum, size_t offset,
                       size_t value_size, void *value) const
{
  auto reg_addr = register_address (regnum);

  if (!reg_addr)
    throw exception_t (AMD_DBGAPI_STATUS_ERROR_INVALID_REGISTER_ID);

  if (!value_size
      || (offset + value_size)
             > architecture ().register_size (regnum).value ())
    throw exception_t (AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT_COMPATIBILITY);

  if (regnum == amdgpu_regnum_t::null)
    {
      memset (static_cast<char *> (value) + offset, '\0', value_size);
      return;
    }

  if (m_is_parked && regnum == amdgpu_regnum_t::pc)
    {
      memcpy (static_cast<char *> (value) + offset,
              reinterpret_cast<const char *> (&m_saved_pc) + offset,
              value_size);
      return;
    }

  amd_dbgapi_global_address_t hwregs_addr
      = register_address (amdgpu_regnum_t::first_hwreg).value ();

  /* hwregs are cached, so return the value from the cache.  */
  if (*reg_addr >= hwregs_addr
      && *reg_addr < (hwregs_addr + sizeof (m_hwregs_cache)))
    {
      memcpy (static_cast<char *> (value) + offset,
              reinterpret_cast<const char *> (&m_hwregs_cache[0]) + *reg_addr
                  - hwregs_addr + offset,
              value_size);
      return;
    }

  dbgapi_assert (queue ().is_suspended ());

  if (process ().read_global_memory (
          *reg_addr + offset, static_cast<char *> (value) + offset, value_size)
      != AMD_DBGAPI_STATUS_SUCCESS)
    error ("Could not read the '%s' register",
           architecture ().register_name (regnum)->c_str ());
}

void
wave_t::write_register (amdgpu_regnum_t regnum, size_t offset,
                        size_t value_size, const void *value)
{
  auto reg_addr = register_address (regnum);

  if (!reg_addr)
    throw exception_t (AMD_DBGAPI_STATUS_ERROR_INVALID_REGISTER_ID);

  if (!value_size
      || (offset + value_size)
             > architecture ().register_size (regnum).value ())
    throw exception_t (AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT_COMPATIBILITY);

  if (regnum == amdgpu_regnum_t::null)
    return;

  if (m_is_parked && regnum == amdgpu_regnum_t::pc)
    {
      memcpy (reinterpret_cast<char *> (&m_saved_pc) + offset,
              static_cast<const char *> (value) + offset, value_size);
      return;
    }

  size_t hwregs_addr
      = register_address (amdgpu_regnum_t::first_hwreg).value ();

  /* Update the hwregs cache.  */
  if (*reg_addr >= hwregs_addr
      && *reg_addr < (hwregs_addr + sizeof (m_hwregs_cache)))
    {
      memcpy (reinterpret_cast<char *> (&m_hwregs_cache[0]) + *reg_addr
                  - hwregs_addr + offset,
              static_cast<const char *> (value) + offset, value_size);

      if (register_cache_policy == register_cache_policy_t::write_back)
        return;
    }

  dbgapi_assert (queue ().is_suspended ());

  if (process ().write_global_memory (
          *reg_addr + offset, static_cast<const char *> (value) + offset,
          value_size)
      != AMD_DBGAPI_STATUS_SUCCESS)
    error ("Could not write the '%s' register",
           architecture ().register_name (regnum)->c_str ());
}

amd_dbgapi_status_t
wave_t::xfer_private_memory_swizzled (
    amd_dbgapi_segment_address_t segment_address, amd_dbgapi_lane_id_t lane_id,
    void *read, const void *write, size_t *size)
{
  if (lane_id == AMD_DBGAPI_LANE_NONE || lane_id >= lane_count ())
    return AMD_DBGAPI_STATUS_ERROR_INVALID_LANE_ID;

  if (!dispatch ().is_scratch_enabled ())
    return AMD_DBGAPI_STATUS_ERROR_MEMORY_ACCESS;

  amd_dbgapi_size_t limit = m_callbacks.scratch_memory_size ();
  amd_dbgapi_global_address_t scratch_base
      = m_callbacks.scratch_memory_base ();

  size_t bytes = *size;
  while (bytes > 0)
    {
      /* Transfer one aligned dword at a time, except for the first (or last)
         read which could read less than a dword if the start (or end) address
         is not aligned.  */

      size_t request_size = std::min (4 - (segment_address % 4), bytes);
      size_t xfer_size = request_size;

      amd_dbgapi_size_t offset = m_scratch_offset
                                 + ((segment_address / 4) * lane_count () * 4)
                                 + (lane_id * 4) + (segment_address % 4);

      if ((offset + xfer_size) > limit)
        {
          size_t xfer_size = offset < limit ? limit - offset : 0;
          if (xfer_size == 0)
            return AMD_DBGAPI_STATUS_ERROR_MEMORY_ACCESS;
        }

      amd_dbgapi_global_address_t global_address = scratch_base + offset;

      amd_dbgapi_status_t status;
      if (read)
        status = process ().read_global_memory_partial (global_address, read,
                                                        &xfer_size);
      else
        status = process ().write_global_memory_partial (global_address, write,
                                                         &xfer_size);
      if (status != AMD_DBGAPI_STATUS_SUCCESS)
        return status;

      bytes -= xfer_size;
      if (request_size != xfer_size)
        break;

      if (read)
        read = static_cast<char *> (read) + xfer_size;
      else
        write = static_cast<const char *> (write) + xfer_size;

      segment_address += xfer_size;
    }

  if (bytes && bytes == *size)
    return AMD_DBGAPI_STATUS_ERROR_MEMORY_ACCESS;

  *size -= bytes;
  return AMD_DBGAPI_STATUS_SUCCESS;
}

amd_dbgapi_status_t
wave_t::xfer_private_memory_unswizzled (
    amd_dbgapi_segment_address_t segment_address, void *read,
    const void *write, size_t *size)
{
  if (!dispatch ().is_scratch_enabled ())
    return AMD_DBGAPI_STATUS_ERROR_MEMORY_ACCESS;

  amd_dbgapi_size_t limit = m_callbacks.scratch_memory_size ();
  amd_dbgapi_size_t offset = m_scratch_offset + segment_address;

  if ((offset + *size) > limit)
    {
      size_t max_size = offset < limit ? limit - offset : 0;
      if (max_size == 0 && *size != 0)
        return AMD_DBGAPI_STATUS_ERROR_MEMORY_ACCESS;
      *size = max_size;
    }

  amd_dbgapi_global_address_t global_address
      = m_callbacks.scratch_memory_base () + offset;

  if (read)
    return process ().read_global_memory_partial (global_address, read, size);
  else
    return process ().write_global_memory_partial (global_address, write,
                                                   size);
}

amd_dbgapi_status_t
wave_t::xfer_local_memory (amd_dbgapi_segment_address_t segment_address,
                           void *read, const void *write, size_t *size)
{
  /* The LDS is stored in the context save area.  */
  dbgapi_assert (queue ().is_suspended ());

  amd_dbgapi_size_t limit = architecture ().wave_get_info (
      *m_descriptor, architecture_t::wave_info_t::lds_size);
  amd_dbgapi_size_t offset = segment_address;

  if ((offset + *size) > limit)
    {
      size_t max_size = offset < limit ? limit - offset : 0;
      if (max_size == 0 && *size != 0)
        return AMD_DBGAPI_STATUS_ERROR_MEMORY_ACCESS;
      *size = max_size;
    }

  auto local_memory_base_address = architecture ().register_address (
      *group_leader ().m_descriptor, amdgpu_regnum_t::lds_0);

  if (!local_memory_base_address)
    error ("local memory is not accessible");

  amd_dbgapi_global_address_t global_address
      = *local_memory_base_address + offset;

  if (read)
    return process ().read_global_memory_partial (global_address, read, size);
  else
    return process ().write_global_memory_partial (global_address, write,
                                                   size);
}

amd_dbgapi_status_t
wave_t::xfer_segment_memory (const address_space_t &address_space,
                             amd_dbgapi_lane_id_t lane_id,
                             amd_dbgapi_segment_address_t segment_address,
                             void *read, const void *write, size_t *size)
{
  dbgapi_assert (!read != !write && "either read or write buffer");

  if (state () != AMD_DBGAPI_WAVE_STATE_STOP)
    return AMD_DBGAPI_STATUS_ERROR_WAVE_NOT_STOPPED;

  /* Zero-extend the segment address.  */
  segment_address &= utils::bit_mask (0, address_space.address_size () - 1);

  switch (address_space.kind ())
    {
    case address_space_t::private_swizzled:
      return xfer_private_memory_swizzled (segment_address, lane_id, read,
                                           write, size);

    case address_space_t::private_unswizzled:
      return xfer_private_memory_unswizzled (segment_address, read, write,
                                             size);

    case address_space_t::local:
      return xfer_local_memory (segment_address, read, write, size);

    case address_space_t::global:
      if (read)
        return process ().read_global_memory_partial (segment_address, read,
                                                      size);
      else
        return process ().write_global_memory_partial (segment_address, write,
                                                       size);

    default:
      error ("xfer_segment_memory from address space `%s' not supported",
             address_space.name ().c_str ());
    }
}

amd_dbgapi_status_t
wave_t::get_info (amd_dbgapi_wave_info_t query, size_t value_size,
                  void *value) const
{
  switch (query)
    {
    case AMD_DBGAPI_WAVE_INFO_STATE:
      return utils::get_info (value_size, value, m_state);

    case AMD_DBGAPI_WAVE_INFO_STOP_REASON:
      return utils::get_info (value_size, value, m_stop_reason);

    case AMD_DBGAPI_WAVE_INFO_DISPATCH:
      return utils::get_info (value_size, value, dispatch ().id ());

    case AMD_DBGAPI_WAVE_INFO_QUEUE:
      return utils::get_info (value_size, value, queue ().id ());

    case AMD_DBGAPI_WAVE_INFO_AGENT:
      return utils::get_info (value_size, value, agent ().id ());

    case AMD_DBGAPI_WAVE_INFO_PROCESS:
      return utils::get_info (value_size, value, process ().id ());

    case AMD_DBGAPI_WAVE_INFO_ARCHITECTURE:
      return utils::get_info (value_size, value, architecture ().id ());

    case AMD_DBGAPI_WAVE_INFO_PC:
      return utils::get_info (value_size, value, pc ());

    case AMD_DBGAPI_WAVE_INFO_EXEC_MASK:
      return utils::get_info (value_size, value, exec_mask ());

    case AMD_DBGAPI_WAVE_INFO_WORK_GROUP_COORD:
      return utils::get_info (value_size, value, m_group_ids);

    case AMD_DBGAPI_WAVE_INFO_WAVE_NUMBER_IN_WORK_GROUP:
      return utils::get_info (value_size, value, m_wave_in_group);

    case AMD_DBGAPI_WAVE_INFO_WATCHPOINTS:
      {
        amd_dbgapi_watchpoint_list_t list{};

        auto os_watch_ids = architecture ().triggered_watchpoints (*this);
        list.count = os_watch_ids.size ();

        list.watchpoint_ids = static_cast<amd_dbgapi_watchpoint_id_t *> (
            amd::dbgapi::allocate_memory (
                list.count * sizeof (amd_dbgapi_watchpoint_id_t)));

        if (list.count && !list.watchpoint_ids)
          return AMD_DBGAPI_STATUS_ERROR_CLIENT_CALLBACK;

        auto watchpoint_id = [this] (os_watch_id_t os_watch_id) {
          const watchpoint_t *watchpoint
              = agent ().find_watchpoint (os_watch_id);
          if (!watchpoint)
            error ("kfd_watch_%d not set on %s", os_watch_id,
                   to_string (agent ().id ()).c_str ());
          return watchpoint->id ();
        };

        std::transform (os_watch_ids.begin (), os_watch_ids.end (),
                        list.watchpoint_ids, watchpoint_id);

        amd_dbgapi_status_t status = utils::get_info (value_size, value, list);
        if (status != AMD_DBGAPI_STATUS_SUCCESS)
          amd::dbgapi::deallocate_memory (list.watchpoint_ids);

        return status;
      }

    case AMD_DBGAPI_WAVE_INFO_LANE_COUNT:
      return utils::get_info (value_size, value, lane_count ());
    }
  return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;
}

wave_t::instruction_buffer_ref_t::instruction_buffer_ref_t (
    amd_dbgapi_global_address_t buffer_address, uint32_t capacity,
    deleter_type deleter)
    : m_data{ buffer_address, 0, capacity }, m_deleter (deleter)
{
}
wave_t::instruction_buffer_ref_t::instruction_buffer_ref_t (
    instruction_buffer_ref_t &&other)
    : m_data (other.m_data), m_deleter (other.m_deleter)
{
  other.release ();
}
wave_t::instruction_buffer_ref_t::~instruction_buffer_ref_t ()
{
  if (m_data.m_buffer_address)
    m_deleter (m_data.m_buffer_address);
  m_data = {};
}

wave_t::instruction_buffer_ref_t &
wave_t::instruction_buffer_ref_t::operator= (instruction_buffer_ref_t &&other)
{
  if (m_data.m_buffer_address)
    m_deleter (m_data.m_buffer_address);

  m_data = other.m_data;
  m_deleter = other.m_deleter;

  other.release ();
  return *this;
}

amd_dbgapi_global_address_t
wave_t::instruction_buffer_ref_t::release ()
{
  amd_dbgapi_global_address_t buffer_address = m_data.m_buffer_address;
  m_data = {};
  return buffer_address;
}

} /* namespace amd::dbgapi */

using namespace amd::dbgapi;

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_wave_stop (amd_dbgapi_wave_id_t wave_id)
{
  TRY;
  TRACE (wave_id);

  if (!detail::is_initialized)
    return AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED;

  wave_t *wave = find (wave_id);

  if (!wave)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_WAVE_ID;

  /* FIXME: We can't enable this yet as a trap could set the state to STOP.
     We need the ability to track stop requests.
  if (it->second.state () == AMD_DBGAPI_WAVE_STATE_STOP)
    return AMD_DBGAPI_STATUS_ERROR_WAVE_STOPPED; */

  scoped_queue_suspend_t suspend (wave->queue (), "stop wave");

  /* Look for the wave_id again, the wave may have exited.  */
  if (!(wave = find (wave_id)))
    return AMD_DBGAPI_STATUS_ERROR_INVALID_WAVE_ID;

  wave->set_state (AMD_DBGAPI_WAVE_STATE_STOP);

  return AMD_DBGAPI_STATUS_SUCCESS;
  CATCH;
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_wave_resume (amd_dbgapi_wave_id_t wave_id,
                        amd_dbgapi_resume_mode_t resume_mode)
{
  TRY;
  TRACE (wave_id, resume_mode);

  if (!detail::is_initialized)
    return AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED;

  wave_t *wave = find (wave_id);

  if (!wave)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_WAVE_ID;

  if (wave->state () != AMD_DBGAPI_WAVE_STATE_STOP)
    return AMD_DBGAPI_STATUS_ERROR_WAVE_NOT_STOPPED;

  if (resume_mode != AMD_DBGAPI_RESUME_MODE_NORMAL
      && resume_mode != AMD_DBGAPI_RESUME_MODE_SINGLE_STEP)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;

  if (wave->displaced_stepping ()
      && resume_mode != AMD_DBGAPI_RESUME_MODE_SINGLE_STEP)
    return AMD_DBGAPI_STATUS_ERROR_RESUME_DISPLACED_STEPPING;

  scoped_queue_suspend_t suspend (wave->queue (), "resume wave");

  /* Look for the wave_id again, the wave may have exited.  */
  if (!(wave = find (wave_id)))
    return AMD_DBGAPI_STATUS_ERROR_INVALID_WAVE_ID;

  wave->set_state (resume_mode == AMD_DBGAPI_RESUME_MODE_SINGLE_STEP
                       ? AMD_DBGAPI_WAVE_STATE_SINGLE_STEP
                       : AMD_DBGAPI_WAVE_STATE_RUN);

  return AMD_DBGAPI_STATUS_SUCCESS;
  CATCH;
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_wave_get_info (amd_dbgapi_wave_id_t wave_id,
                          amd_dbgapi_wave_info_t query, size_t value_size,
                          void *value)
{
  TRY;
  TRACE (wave_id, query, value_size, value);

  if (!detail::is_initialized)
    return AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED;

  wave_t *wave = find (wave_id);

  if (!wave)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_WAVE_ID;

  switch (query)
    {
    case AMD_DBGAPI_WAVE_INFO_STOP_REASON:
    case AMD_DBGAPI_WAVE_INFO_PC:
    case AMD_DBGAPI_WAVE_INFO_EXEC_MASK:
    case AMD_DBGAPI_WAVE_INFO_WATCHPOINTS:
      if (wave->state () != AMD_DBGAPI_WAVE_STATE_STOP)
        return AMD_DBGAPI_STATUS_ERROR_WAVE_NOT_STOPPED;
    default:
      break;
    };

  return wave->get_info (query, value_size, value);
  CATCH;
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_process_wave_list (amd_dbgapi_process_id_t process_id,
                              size_t *wave_count, amd_dbgapi_wave_id_t **waves,
                              amd_dbgapi_changed_t *changed)
{
  TRY;
  TRACE (process_id);

  if (!detail::is_initialized)
    return AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED;

  std::vector<process_t *> processes;
  if (process_id != AMD_DBGAPI_PROCESS_NONE)
    {
      process_t *process = process_t::find (process_id);

      if (!process)
        return AMD_DBGAPI_STATUS_ERROR_INVALID_PROCESS_ID;

      if (amd_dbgapi_status_t status = process->update_queues ();
          status != AMD_DBGAPI_STATUS_SUCCESS)
        error ("process_t::update_agents failed (rc=%d)", status);

      processes.emplace_back (process);
    }
  else
    {
      for (auto &&process : process_t::all ())
        {
          if (amd_dbgapi_status_t status = process.update_queues ();
              status != AMD_DBGAPI_STATUS_SUCCESS)
            error ("process_t::update_agents failed (rc=%d)", status);

          processes.emplace_back (&process);
        }
    }

  std::vector<std::pair<process_t *, std::vector<queue_t *>>>
      queues_needing_resume;

  for (auto &&process : processes)
    {
      std::vector<queue_t *> queues;

      for (auto &&queue : process->range<queue_t> ())
        if (!queue.is_suspended ())
          queues.emplace_back (&queue);

      process->suspend_queues (queues, "refresh wave list");

      if (process->forward_progress_needed ())
        queues_needing_resume.emplace_back (process, std::move (queues));
    }

  amd_dbgapi_status_t status
      = utils::get_handle_list<wave_t> (processes, wave_count, waves, changed);

  for (auto &&[process, queues] : queues_needing_resume)
    process->resume_queues (queues, "refresh wave list");

  return status;
  CATCH;
}
