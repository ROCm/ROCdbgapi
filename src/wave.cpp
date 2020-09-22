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

#include "wave.h"
#include "architecture.h"
#include "dispatch.h"
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
  amd_dbgapi_status_t status;

  if (lane_count () == 32)
    {
      uint32_t exec;
      status = read_register (amdgpu_regnum_t::EXEC_32, &exec);
      if (status != AMD_DBGAPI_STATUS_SUCCESS)
        error ("Could not read the EXEC_32 register (rc=%d)", status);
      return exec;
    }
  else if (lane_count () == 64)
    {
      uint64_t exec;
      status = read_register (amdgpu_regnum_t::EXEC_64, &exec);
      if (status != AMD_DBGAPI_STATUS_SUCCESS)
        error ("Could not read the EXEC_64 register (rc=%d)", status);
      return exec;
    }
  error ("Not a valid lane_count for EXEC mask: %zu", lane_count ());
}

amd_dbgapi_global_address_t
wave_t::pc () const
{
  amd_dbgapi_global_address_t pc;
  amd_dbgapi_status_t status = read_register (amdgpu_regnum_t::PC, &pc);
  if (status != AMD_DBGAPI_STATUS_SUCCESS)
    error ("Could not read the PC register (rc=%d)", status);
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

amd_dbgapi_status_t
wave_t::park ()
{
  if (m_parked)
    return AMD_DBGAPI_STATUS_SUCCESS;

  /* When a wave hits a breakpoint, we change its pc to point to an immutable
     breakpoint instruction.  This guarantees that the wave will never be
     halted at an s_endpgm if the breakpoint is removed and the original
     instruction restored.  */
  m_saved_pc = pc ();

  amd_dbgapi_global_address_t parked_pc
      = queue ().parked_wave_buffer_address ();

  amd_dbgapi_status_t status
      = write_register (amdgpu_regnum_t::PC, &parked_pc);
  m_parked = true;

  return status;
}

amd_dbgapi_status_t
wave_t::unpark ()
{
  dbgapi_assert (m_parked && "not parked");

  /* Restore the original pc if the wave was parked.  */
  amd_dbgapi_global_address_t saved_pc = pc ();

  m_parked = false;
  return write_register (amdgpu_regnum_t::PC, &saved_pc);
}

amd_dbgapi_status_t
wave_t::update (const wave_t &group_leader,
                std::unique_ptr<architecture_t::cwsr_descriptor_t> descriptor)
{
  dbgapi_assert (queue ().is_suspended ());
  process_t &process = this->process ();
  amd_dbgapi_status_t status;

  bool first_update = !m_descriptor;
  m_descriptor = std::move (descriptor);
  m_group_leader = &group_leader;

  if (first_update)
    {
      /* Write the wave_id register.  */
      amd_dbgapi_wave_id_t wave_id = id ();
      status = write_register (amdgpu_regnum_t::WAVE_ID, &wave_id);
      if (status != AMD_DBGAPI_STATUS_SUCCESS)
        return status;

      status = architecture ().get_wave_coords (*this, m_group_ids,
                                                &m_wave_in_group);
      if (status != AMD_DBGAPI_STATUS_SUCCESS)
        return status;

      if (dispatch ().is_scratch_enabled ())
        {
          uint32_t scratch_offset;
          status = read_register (amdgpu_regnum_t::SCRATCH_OFFSET,
                                  &scratch_offset);
          if (status != AMD_DBGAPI_STATUS_SUCCESS)
            return status;
          m_scratch_offset = scratch_offset;
        }
    }

  /* Reload the HW registers cache, and update the wave's state, if this is a
     new wave, or if the wave wasn't stopped (mode.halt == 1) the last time the
     queue it belongs to was resumed.  */
  if (m_reload_hwregs_cache)
    {
      /* Save the previous pc before updating the hwregs cache.  */
      if (!first_update)
        m_saved_pc = pc ();

      status = process.read_global_memory (
          register_address (amdgpu_regnum_t::FIRST_HWREG).value (),
          &m_hwregs_cache[0], sizeof (m_hwregs_cache));
      if (status != AMD_DBGAPI_STATUS_SUCCESS)
        return status;

      amd_dbgapi_wave_state_t saved_state = m_state;
      status
          = architecture ().get_wave_state (*this, &m_state, &m_stop_reason);
      if (status != AMD_DBGAPI_STATUS_SUCCESS)
        return status;

      /* If the wave is not stopped, we'll need to reload the hwregs cache
         during the next update.  */
      m_reload_hwregs_cache = (m_state != AMD_DBGAPI_WAVE_STATE_STOP);

      if (m_stop_reason && m_state == AMD_DBGAPI_WAVE_STATE_STOP
          && saved_state != AMD_DBGAPI_WAVE_STATE_STOP)
        {
          if (!!(m_stop_reason & AMD_DBGAPI_WAVE_STOP_REASON_BREAKPOINT)
              && !architecture ().can_halt_at (
                  architecture ().breakpoint_instruction ()))
            {
              /* When a wave hits a breakpoint, we change its pc to point to
                 an immutable breakpoint instruction.  This guarantees that the
                 wave will never be halted at an s_endpgm if the breakpoint is
                 removed and the original instruction restored.  */
              status = park ();
              if (status != AMD_DBGAPI_STATUS_SUCCESS)
                return status;
            }

          if (visibility () == visibility_t::VISIBLE)
            process.enqueue_event (process.create<event_t> (
                process, AMD_DBGAPI_EVENT_KIND_WAVE_STOP, id ()));
        }
    }

  return AMD_DBGAPI_STATUS_SUCCESS;
}

amd_dbgapi_status_t
wave_t::set_state (amd_dbgapi_wave_state_t state)
{
  amd_dbgapi_wave_state_t prev_state = m_state;

  if (state == prev_state)
    return AMD_DBGAPI_STATUS_SUCCESS;

  amd_dbgapi_status_t status = architecture ().set_wave_state (*this, state);
  if (status != AMD_DBGAPI_STATUS_SUCCESS)
    return status;

  m_state = state;

  if (state != AMD_DBGAPI_WAVE_STATE_STOP)
    {
      /* Restore the original pc if the wave was parked.  */
      if (m_parked)
        {
          status = unpark ();
          if (status != AMD_DBGAPI_STATUS_SUCCESS)
            return status;
        }

      /* If we are resuming this wave, we'll need to reload the hwregs cache
        during the next update.  */
      m_reload_hwregs_cache = true;

      /* Clear the stop reason.  */
      m_stop_reason = AMD_DBGAPI_WAVE_STOP_REASON_NONE;
    }

  dbgapi_log (AMD_DBGAPI_LOG_LEVEL_INFO,
              "changing %s's state from %s to %s (pc=%#lx)",
              to_string (id ()).c_str (), to_string (prev_state).c_str (),
              to_string (state).c_str (), pc ());

  /* If we requested the wave be stopped, and the wave wasn't already stopped,
     report an event to acknowledge that the wave has stopped.  */
  if (state == AMD_DBGAPI_WAVE_STATE_STOP
      && prev_state != AMD_DBGAPI_WAVE_STATE_STOP)
    {
      /* We have to park the wave if we cannot halt at the current pc.  */
      if (!m_parked && !architecture ().can_halt_at (instruction_at_pc ()))
        park ();

      m_stop_reason = AMD_DBGAPI_WAVE_STOP_REASON_NONE;

      dbgapi_assert (visibility () == visibility_t::VISIBLE
                     && "cannot set the state of an hidden wave");

      process ().enqueue_event (process ().create<event_t> (
          process (), AMD_DBGAPI_EVENT_KIND_WAVE_STOP, id ()));
    }

  return AMD_DBGAPI_STATUS_SUCCESS;
}

bool
wave_t::is_register_cached (amdgpu_regnum_t regnum) const
{
  auto reg_addr = register_address (regnum);

  if (!reg_addr)
    return false;

  amd_dbgapi_global_address_t hwregs_addr
      = register_address (amdgpu_regnum_t::FIRST_HWREG).value ();

  return *reg_addr >= hwregs_addr
         && *reg_addr < (hwregs_addr + sizeof (m_hwregs_cache));
}

bool
wave_t::is_register_available (amdgpu_regnum_t regnum) const
{
  return register_address (regnum).has_value ();
}

std::optional<std::string>
wave_t::register_name (amdgpu_regnum_t regnum) const
{
  if (is_register_available (regnum))
    return architecture ().register_name (regnum);

  return {};
}

std::optional<std::string>
wave_t::register_type (amdgpu_regnum_t regnum) const
{
  if (is_register_available (regnum))
    return architecture ().register_type (regnum);

  return {};
}

std::optional<amd_dbgapi_size_t>
wave_t::register_size (amdgpu_regnum_t regnum) const
{
  if (is_register_available (regnum))
    return architecture ().register_size (regnum);

  return {};
}

amd_dbgapi_status_t
wave_t::read_register (amdgpu_regnum_t regnum, size_t offset,
                       size_t value_size, void *value) const
{
  auto reg_addr = register_address (regnum);

  if (!reg_addr)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_REGISTER_ID;

  if (!value_size
      || (offset + value_size)
             > architecture ().register_size (regnum).value ())
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT_SIZE;

  if (regnum == amdgpu_regnum_t::NULL_)
    {
      memset (static_cast<char *> (value) + offset, '\0', value_size);
      return AMD_DBGAPI_STATUS_SUCCESS;
    }

  if (m_parked && regnum == amdgpu_regnum_t::PC)
    {
      memcpy (static_cast<char *> (value) + offset,
              reinterpret_cast<const char *> (&m_saved_pc) + offset,
              value_size);
      return AMD_DBGAPI_STATUS_SUCCESS;
    }

  amd_dbgapi_global_address_t hwregs_addr
      = register_address (amdgpu_regnum_t::FIRST_HWREG).value ();

  /* hwregs are cached, so return the value from the cache.  */
  if (*reg_addr >= hwregs_addr
      && *reg_addr < (hwregs_addr + sizeof (m_hwregs_cache)))
    {
      memcpy (static_cast<char *> (value) + offset,
              reinterpret_cast<const char *> (&m_hwregs_cache[0]) + *reg_addr
                  - hwregs_addr + offset,
              value_size);
      return AMD_DBGAPI_STATUS_SUCCESS;
    }

  dbgapi_assert (queue ().is_suspended ());

  return process ().read_global_memory (
      *reg_addr + offset, static_cast<char *> (value) + offset, value_size);
}

amd_dbgapi_status_t
wave_t::write_register (amdgpu_regnum_t regnum, size_t offset,
                        size_t value_size, const void *value)
{
  auto reg_addr = register_address (regnum);

  if (!reg_addr)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_REGISTER_ID;

  if (!value_size
      || (offset + value_size)
             > architecture ().register_size (regnum).value ())
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT_SIZE;

  if (regnum == amdgpu_regnum_t::NULL_)
    return AMD_DBGAPI_STATUS_SUCCESS;

  if (m_parked && regnum == amdgpu_regnum_t::PC)
    {
      memcpy (reinterpret_cast<char *> (&m_saved_pc) + offset,
              static_cast<const char *> (value) + offset, value_size);
      return AMD_DBGAPI_STATUS_SUCCESS;
    }

  size_t hwregs_addr
      = register_address (amdgpu_regnum_t::FIRST_HWREG).value ();

  /* Update the hwregs cache.  */
  if (*reg_addr >= hwregs_addr
      && *reg_addr < (hwregs_addr + sizeof (m_hwregs_cache)))
    {
      memcpy (reinterpret_cast<char *> (&m_hwregs_cache[0]) + *reg_addr
                  - hwregs_addr + offset,
              static_cast<const char *> (value) + offset, value_size);
    }

  dbgapi_assert (queue ().is_suspended ());

  return process ().write_global_memory (
      *reg_addr + offset, static_cast<const char *> (value) + offset,
      value_size);
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

  amd_dbgapi_size_t limit = queue ().scratch_backing_memory_size ();
  amd_dbgapi_global_address_t scratch_base
      = queue ().scratch_backing_memory_address ();

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

  amd_dbgapi_size_t limit = queue ().scratch_backing_memory_size ();
  amd_dbgapi_size_t offset = m_scratch_offset + segment_address;

  if ((offset + *size) > limit)
    {
      size_t max_size = offset < limit ? limit - offset : 0;
      if (max_size == 0 && *size != 0)
        return AMD_DBGAPI_STATUS_ERROR_MEMORY_ACCESS;
      *size = max_size;
    }

  amd_dbgapi_global_address_t global_address
      = queue ().scratch_backing_memory_address () + offset;

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
      *group_leader ().m_descriptor, amdgpu_regnum_t::LDS_0);

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
    case address_space_t::PRIVATE_SWIZZLED:
      return xfer_private_memory_swizzled (segment_address, lane_id, read,
                                           write, size);

    case address_space_t::PRIVATE_UNSWIZZLED:
      return xfer_private_memory_unswizzled (segment_address, read, write,
                                             size);

    case address_space_t::LOCAL:
      return xfer_local_memory (segment_address, read, write, size);

    case address_space_t::GLOBAL:
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

} /* namespace amd::dbgapi */

using namespace amd::dbgapi;

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_wave_stop (amd_dbgapi_wave_id_t wave_id)
{
  TRY;
  TRACE (wave_id);

  if (!amd::dbgapi::is_initialized)
    return AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED;

  wave_t *wave = find (wave_id);

  if (!wave)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_WAVE_ID;

  /* FIXME: We can't enable this yet as a trap could set the state to STOP.
     We need the ability to track stop requests.
  if (it->second.state () == AMD_DBGAPI_WAVE_STATE_STOP)
    return AMD_DBGAPI_STATUS_ERROR_WAVE_STOPPED; */

  {
    scoped_queue_suspend_t suspend (wave->queue ());

    /* Look for the wave_id again, the wave may have exited.  */
    if (!(wave = find (wave_id)))
      return AMD_DBGAPI_STATUS_ERROR_INVALID_WAVE_ID;

    return wave->set_state (AMD_DBGAPI_WAVE_STATE_STOP);
  }
  CATCH;
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_wave_resume (amd_dbgapi_wave_id_t wave_id,
                        amd_dbgapi_resume_mode_t resume_mode)
{
  TRY;
  TRACE (wave_id, resume_mode);

  if (!amd::dbgapi::is_initialized)
    return AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED;

  wave_t *wave = find (wave_id);

  if (!wave)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_WAVE_ID;

  if (wave->state () != AMD_DBGAPI_WAVE_STATE_STOP)
    return AMD_DBGAPI_STATUS_ERROR_WAVE_NOT_STOPPED;

  if (resume_mode != AMD_DBGAPI_RESUME_MODE_NORMAL
      && resume_mode != AMD_DBGAPI_RESUME_MODE_SINGLE_STEP)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;

  {
    scoped_queue_suspend_t suspend (wave->queue ());

    /* Look for the wave_id again, the wave may have exited.  */
    if (!(wave = find (wave_id)))
      return AMD_DBGAPI_STATUS_ERROR_INVALID_WAVE_ID;

    return wave->set_state (resume_mode == AMD_DBGAPI_RESUME_MODE_SINGLE_STEP
                                ? AMD_DBGAPI_WAVE_STATE_SINGLE_STEP
                                : AMD_DBGAPI_WAVE_STATE_RUN);
  }
  CATCH;
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_wave_get_info (amd_dbgapi_wave_id_t wave_id,
                          amd_dbgapi_wave_info_t query, size_t value_size,
                          void *value)
{
  TRY;
  TRACE (wave_id, query, value_size, value);

  if (!amd::dbgapi::is_initialized)
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
amd_dbgapi_wave_list (amd_dbgapi_process_id_t process_id, size_t *wave_count,
                      amd_dbgapi_wave_id_t **waves,
                      amd_dbgapi_changed_t *changed)
{
  TRY;
  TRACE (process_id);

  if (!amd::dbgapi::is_initialized)
    return AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED;

  process_t *process = process_t::find (process_id);

  if (!process)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_PROCESS_ID;

  amd_dbgapi_status_t status = process->update_queues ();
  if (status != AMD_DBGAPI_STATUS_SUCCESS)
    error ("process_t::update_queues failed (rc=%d)", status);

  /* Ensure all queues are suspended so that the wave list is updated.  */
  std::vector<queue_t *> queues;
  for (auto &&queue : process->range<queue_t> ())
    if (!queue.is_suspended ())
      queues.emplace_back (&queue);

  process->suspend_queues (queues);

  status = utils::get_handle_list<wave_t> (process_id, wave_count, waves,
                                           changed);

  if (process->forward_progress_needed ())
    process->resume_queues (queues);

  return status;
  CATCH;
}
