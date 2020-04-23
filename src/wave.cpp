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

#include "defs.h"

#include "architecture.h"
#include "dispatch.h"
#include "event.h"
#include "logging.h"
#include "process.h"
#include "queue.h"
#include "register.h"
#include "utils.h"
#include "wave.h"

#include <algorithm>
#include <cstring>
#include <string>
#include <tuple>
#include <type_traits>

#if defined(__GNUC__)
#define __UNUSED__ __attribute__ ((unused))
#else /* !defined(__GNUC__) */
#define __UNUSED__
#endif /* !defined(__GNUC__) */

namespace amd
{
namespace dbgapi
{

/* ignored_wave_id is ODR-used by the operator==.  */
constexpr amd_dbgapi_wave_id_t wave_t::ignored_wave_id;

wave_t::wave_t (amd_dbgapi_wave_id_t wave_id, dispatch_t &dispatch,
                uint32_t vgpr_count, uint32_t accvgpr_count,
                uint32_t sgpr_count, uint32_t lane_count)
    : handle_object (wave_id), m_vgpr_count (vgpr_count),
      m_accvgpr_count (accvgpr_count), m_sgpr_count (sgpr_count),
      m_lane_count (lane_count), m_dispatch (dispatch)
{
}

amd_dbgapi_global_address_t
wave_t::pc () const
{
  amd_dbgapi_global_address_t pc;
  if (read_register (amdgpu_regnum_t::PC, &pc) != AMD_DBGAPI_STATUS_SUCCESS)
    error ("Could not read the PC register");
  return pc;
}

std::vector<uint8_t>
wave_t::instruction_at_pc () const
{
  size_t size = architecture ().largest_instruction_size ();
  std::vector<uint8_t> instruction_bytes (size);

  if (process ().read_global_memory_partial (pc (), instruction_bytes.data (),
                                             &size)
      != AMD_DBGAPI_STATUS_SUCCESS)
    error ("Could not read the instruction at %#lx", pc ());

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
wave_t::update (amd_dbgapi_global_address_t context_save_address)
{
  dbgapi_assert (queue ().suspended ());
  process_t &process = this->process ();
  amd_dbgapi_status_t status;

  bool first_update = !m_context_save_address;
  m_context_save_address = context_save_address;

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
          m_context_save_address
              + register_offset_and_size (amdgpu_regnum_t::FIRST_HWREG).first,
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
          if (m_stop_reason == AMD_DBGAPI_WAVE_STOP_REASON_BREAKPOINT
              && !architecture ().can_halt_at_endpgm ())
            {
              /* When a wave hits a breakpoint, we change its pc to point to
                 an immutable breakpoint instruction.  This guarantees that the
                 wave will never be halted at an s_endpgm if the breakpoint is
                 removed and the original instruction restored.  */
              status = park ();
              if (status != AMD_DBGAPI_STATUS_SUCCESS)
                return status;
            }

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
              "setting %s's state to %s (pc=%#lx), was %s",
              to_string (id ()).c_str (), to_string (state).c_str (), pc (),
              to_string (prev_state).c_str ());

  /* If we requested the wave be stopped, and the wave wasn't already stopped,
     report an event to acknowledge that the wave has stopped.  */
  if (state == AMD_DBGAPI_WAVE_STATE_STOP
      && prev_state != AMD_DBGAPI_WAVE_STATE_STOP)
    {
      /* We have to park the wave if the instruction at the current pc is an
         ENDPGM and the architecture does not support halting at ENDPGM.  */
      if (!m_parked && !architecture ().can_halt_at_endpgm ()
          && architecture ().is_endpgm (instruction_at_pc ()))
        park ();

      m_stop_reason = AMD_DBGAPI_WAVE_STOP_REASON_NONE;
      process ().enqueue_event (process ().create<event_t> (
          process (), AMD_DBGAPI_EVENT_KIND_WAVE_STOP, id ()));
    }

  return AMD_DBGAPI_STATUS_SUCCESS;
}

bool
wave_t::register_available (amdgpu_regnum_t regnum) const
{
  return register_offset_and_size (regnum, false).second != 0;
}

std::string
wave_t::register_name (amdgpu_regnum_t regnum) const
{
  if (!register_available (regnum))
    return "";

  return architecture ().register_name (regnum);
}

std::string
wave_t::register_type (amdgpu_regnum_t regnum) const
{
  if (!register_available (regnum))
    return "";

  return architecture ().register_type (regnum);
}

std::pair<size_t, size_t>
wave_t::register_offset_and_size (amdgpu_regnum_t regnum,
                                  bool include_aliased_registers) const
{
  if (lane_count () == 32 && regnum >= amdgpu_regnum_t::FIRST_VGPR_32
      && regnum <= amdgpu_regnum_t::LAST_VGPR_32
      && ((regnum - amdgpu_regnum_t::V0_32) < m_vgpr_count))
    {
      size_t vgprs_offset = 0;
      size_t vgpr_size = sizeof (int32_t) * 32;
      size_t vgpr_num = regnum - amdgpu_regnum_t::V0_32;

      return std::make_pair (vgprs_offset + vgpr_num * vgpr_size, vgpr_size);
    }
  else if (lane_count () == 64 && regnum >= amdgpu_regnum_t::FIRST_VGPR_64
           && regnum <= amdgpu_regnum_t::LAST_VGPR_64
           && ((regnum - amdgpu_regnum_t::V0_64) < m_vgpr_count))
    {
      size_t vgprs_offset = 0;
      size_t vgpr_size = sizeof (int32_t) * 64;
      size_t vgpr_num = regnum - amdgpu_regnum_t::V0_64;

      return std::make_pair (vgprs_offset + vgpr_num * vgpr_size, vgpr_size);
    }
  else if (lane_count () == 64 && regnum >= amdgpu_regnum_t::FIRST_ACCVGPR_64
           && regnum <= amdgpu_regnum_t::LAST_ACCVGPR_64
           && ((regnum - amdgpu_regnum_t::ACC0_64) < m_accvgpr_count))
    {
      size_t accvgprs_offset = m_vgpr_count * sizeof (int32_t) * m_lane_count;
      size_t accvgpr_size = sizeof (int32_t) * m_lane_count;
      size_t accvgpr_num = regnum - amdgpu_regnum_t::ACC0_64;

      return std::make_pair (accvgprs_offset + accvgpr_num * accvgpr_size,
                             accvgpr_size);
    }
  else if (regnum >= amdgpu_regnum_t::FIRST_SGPR
           && regnum <= amdgpu_regnum_t::LAST_SGPR
           && (regnum - amdgpu_regnum_t::S0) < std::min (
                  102u, m_sgpr_count -
                            /* TODO: make an architecture query to return the
                               number of special registers.  */
                            (include_aliased_registers ? 0 : 2)))
    {
      size_t sgprs_offset
          = (m_vgpr_count + m_accvgpr_count) * sizeof (int32_t) * m_lane_count;
      size_t sgpr_num = regnum - amdgpu_regnum_t::S0;

      return std::make_pair (sgprs_offset + sgpr_num * sizeof (int32_t),
                             sizeof (int32_t));
    }
  else if (regnum >= amdgpu_regnum_t::FIRST_HWREG
           && regnum <= amdgpu_regnum_t::LAST_HWREG)
    {
      size_t hwregs_offset
          = (m_vgpr_count + m_accvgpr_count) * sizeof (int32_t) * m_lane_count
            + m_sgpr_count * sizeof (int32_t);
      size_t hwreg_size = sizeof (uint32_t);
      size_t hwreg_num = regnum - amdgpu_regnum_t::FIRST_HWREG;

      return std::make_pair (hwregs_offset + hwreg_num * sizeof (uint32_t),
                             hwreg_size);
    }
  else if (regnum >= amdgpu_regnum_t::FIRST_TTMP
           && regnum <= amdgpu_regnum_t::LAST_TTMP)
    {
      size_t ttmps_offset
          = (m_vgpr_count + m_accvgpr_count) * sizeof (int32_t) * m_lane_count
            + m_sgpr_count * sizeof (int32_t) + 16 * sizeof (uint32_t);
      size_t ttmp_num = regnum - amdgpu_regnum_t::FIRST_TTMP;

      return std::make_pair (ttmps_offset + ttmp_num * sizeof (uint32_t),
                             sizeof (uint32_t));
    }
  else if (regnum == amdgpu_regnum_t::PC)
    {
      return std::make_pair (-1, sizeof (void (*) ()));
    }
  else if (regnum == amdgpu_regnum_t::WAVE_ID)
    {
      return std::make_pair (-1, sizeof (uint64_t));
    }
  else if (lane_count () == 32
           && (regnum == amdgpu_regnum_t::EXEC_32
               || regnum == amdgpu_regnum_t::VCC_32))
    {
      return std::make_pair (-1, sizeof (uint32_t));
    }
  else if (lane_count () == 64
           && (regnum == amdgpu_regnum_t::EXEC_64
               || regnum == amdgpu_regnum_t::VCC_64))
    {
      return std::make_pair (-1, sizeof (uint64_t));
    }

  return std::make_pair (-1, 0);
}

amd_dbgapi_status_t
wave_t::read_register (amdgpu_regnum_t regnum, size_t offset,
                       size_t value_size, void *value) const
{
  size_t reg_offset, reg_size;
  std::tie (reg_offset, reg_size) = register_offset_and_size (regnum);

  if (!reg_size)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_REGISTER_ID;

  if (!value_size || (offset + value_size) > reg_size)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT_SIZE;

  if (m_parked && regnum == amdgpu_regnum_t::PC)
    {
      memcpy (static_cast<char *> (value) + offset,
              reinterpret_cast<const char *> (&m_saved_pc) + offset,
              value_size);
      return AMD_DBGAPI_STATUS_SUCCESS;
    }

  if (regnum >= amdgpu_regnum_t::FIRST_PSEUDO
      && regnum <= amdgpu_regnum_t::LAST_PSEUDO)
    {
      return read_pseudo_register (regnum, offset, value_size, value);
    }

  /* hwregs are cached, so return the value from the cache.  */
  if (regnum >= amdgpu_regnum_t::FIRST_HWREG
      && regnum <= amdgpu_regnum_t::LAST_HWREG)
    {
      memcpy (static_cast<char *> (value) + offset,
              reinterpret_cast<const char *> (
                  &m_hwregs_cache[regnum - amdgpu_regnum_t::FIRST_HWREG])
                  + offset,
              value_size);
      return AMD_DBGAPI_STATUS_SUCCESS;
    }

  dbgapi_assert (queue ().suspended ());

  return process ().read_global_memory (
      m_context_save_address + reg_offset + offset,
      static_cast<char *> (value) + offset, value_size);
}

amd_dbgapi_status_t
wave_t::read_pseudo_register (amdgpu_regnum_t regnum, size_t offset,
                              size_t value_size, void *value) const
{
  dbgapi_assert (regnum >= amdgpu_regnum_t::FIRST_PSEUDO
                 && regnum <= amdgpu_regnum_t::LAST_PSEUDO);

  if (lane_count () == 32
      && (regnum == amdgpu_regnum_t::EXEC_32
          || regnum == amdgpu_regnum_t::VCC_32))
    {
      amdgpu_regnum_t regnum_lo;

      switch (regnum)
        {
        case amdgpu_regnum_t::EXEC_32:
          regnum_lo = amdgpu_regnum_t::EXEC_LO;
          break;
        case amdgpu_regnum_t::VCC_32:
          regnum_lo = amdgpu_regnum_t::S0 + sgpr_count () - 2;
          break;
        default:
          return AMD_DBGAPI_STATUS_ERROR_INVALID_REGISTER_ID;
        }

      return read_register (regnum_lo, offset, value_size, value);
    }
  /* Read registers that are lo/hi pairs.  */
  else if (regnum == amdgpu_regnum_t::PC || regnum == amdgpu_regnum_t::WAVE_ID
           || (lane_count () == 64
               && (regnum == amdgpu_regnum_t::EXEC_64
                   || regnum == amdgpu_regnum_t::VCC_64)))
    {
      amdgpu_regnum_t regnum_lo;
      uint32_t reg[2];

      switch (regnum)
        {
        case amdgpu_regnum_t::PC:
          regnum_lo = amdgpu_regnum_t::PC_LO;
          break;
        case amdgpu_regnum_t::WAVE_ID:
          /* FIXME: we should be getting the register numbers ttmp[4:5] from
             the wave->architecture ().  */
          regnum_lo = amdgpu_regnum_t::TTMP4;
          break;
        case amdgpu_regnum_t::EXEC_64:
          regnum_lo = amdgpu_regnum_t::EXEC_LO;
          break;
        case amdgpu_regnum_t::VCC_64:
          regnum_lo = amdgpu_regnum_t::S0 + sgpr_count () - 2;
          break;
        default:
          return AMD_DBGAPI_STATUS_ERROR_INVALID_REGISTER_ID;
        }
      amdgpu_regnum_t regnum_hi = regnum_lo + 1;

      if (!value_size || (offset + value_size) > sizeof (reg))
        return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT_SIZE;

      size_t saved_offset = offset;
      size_t saved_value_size = value_size;
      amd_dbgapi_status_t status;

      /* Read the partial lo register.  */
      if (offset < sizeof (reg[0]))
        {
          size_t size_lo = (offset + value_size) > sizeof (reg[0])
                               ? sizeof (reg[0]) - offset
                               : value_size;

          if ((status = read_register (regnum_lo, offset, size_lo, &reg[0]))
              != AMD_DBGAPI_STATUS_SUCCESS)
            return status;

          value_size -= size_lo;
          offset = sizeof (reg[0]);
        }

      /* Read the partial hi register.  */
      if (value_size)
        {
          if ((status = read_register (regnum_hi, offset - sizeof (reg[0]),
                                       value_size, &reg[1]))
              != AMD_DBGAPI_STATUS_SUCCESS)
            return status;
        }

      memcpy (static_cast<char *> (value) + saved_offset,
              reinterpret_cast<const char *> (&reg[0]) + saved_offset,
              saved_value_size);

      return AMD_DBGAPI_STATUS_SUCCESS;
    }
  else
    {
      return AMD_DBGAPI_STATUS_ERROR_INVALID_REGISTER_ID;
    }
}

amd_dbgapi_status_t
wave_t::write_register (amdgpu_regnum_t regnum, size_t offset,
                        size_t value_size, const void *value)
{
  size_t reg_offset, reg_size;
  std::tie (reg_offset, reg_size) = register_offset_and_size (regnum);

  if (!reg_size)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_REGISTER_ID;

  if (!value_size || (offset + value_size) > reg_size)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT_SIZE;

  if (m_parked && regnum == amdgpu_regnum_t::PC)
    {
      memcpy (reinterpret_cast<char *> (&m_saved_pc) + offset,
              static_cast<const char *> (value) + offset, value_size);
      return AMD_DBGAPI_STATUS_SUCCESS;
    }

  if (regnum >= amdgpu_regnum_t::FIRST_PSEUDO
      && regnum <= amdgpu_regnum_t::LAST_PSEUDO)
    return write_pseudo_register (regnum, offset, value_size, value);

  /* Update the cached hwregs.  */
  if (regnum >= amdgpu_regnum_t::FIRST_HWREG
      && regnum <= amdgpu_regnum_t::LAST_HWREG)
    memcpy (reinterpret_cast<char *> (
                &m_hwregs_cache[regnum - amdgpu_regnum_t::FIRST_HWREG])
                + offset,
            static_cast<const char *> (value) + offset, value_size);

  dbgapi_assert (queue ().suspended ());

  return process ().write_global_memory (
      m_context_save_address + reg_offset + offset,
      static_cast<const char *> (value) + offset, value_size);
}

amd_dbgapi_status_t
wave_t::write_pseudo_register (amdgpu_regnum_t regnum, size_t offset,
                               size_t value_size, const void *value)
{
  dbgapi_assert (regnum >= amdgpu_regnum_t::FIRST_PSEUDO
                 && regnum <= amdgpu_regnum_t::LAST_PSEUDO);

  if (lane_count () == 32
      && (regnum == amdgpu_regnum_t::EXEC_32
          || regnum == amdgpu_regnum_t::VCC_32))
    {
      amdgpu_regnum_t regnum_lo;

      switch (regnum)
        {
        case amdgpu_regnum_t::EXEC_32:
          regnum_lo = amdgpu_regnum_t::EXEC_LO;
          break;
        case amdgpu_regnum_t::VCC_32:
          regnum_lo = amdgpu_regnum_t::S0 + sgpr_count () - 2;
          break;
        default:
          return AMD_DBGAPI_STATUS_ERROR_INVALID_REGISTER_ID;
        }

      return write_register (regnum_lo, offset, value_size, value);
    }
  /* Write registers that are lo/hi pairs.  */
  else if (regnum == amdgpu_regnum_t::PC || regnum == amdgpu_regnum_t::WAVE_ID
           || (lane_count () == 64
               && (regnum == amdgpu_regnum_t::EXEC_64
                   || regnum == amdgpu_regnum_t::VCC_64)))
    {
      amdgpu_regnum_t regnum_lo;
      uint32_t reg[2];

      switch (regnum)
        {
        case amdgpu_regnum_t::PC:
          regnum_lo = amdgpu_regnum_t::PC_LO;
          break;
        case amdgpu_regnum_t::WAVE_ID:
          /* FIXME: we should be getting the register numbers ttmp[4:5] from
             the wave->architecture ().  */
          regnum_lo = amdgpu_regnum_t::TTMP4;
          break;
        case amdgpu_regnum_t::EXEC_64:
          regnum_lo = amdgpu_regnum_t::EXEC_LO;
          break;
        case amdgpu_regnum_t::VCC_64:
          regnum_lo = amdgpu_regnum_t::S0 + sgpr_count () - 2;
          break;
        default:
          return AMD_DBGAPI_STATUS_ERROR_INVALID_REGISTER_ID;
        }
      amdgpu_regnum_t regnum_hi = regnum_lo + 1;

      if (!value_size || (offset + value_size) > sizeof (reg))
        return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT_SIZE;

      memcpy (reinterpret_cast<char *> (&reg[0]) + offset,
              static_cast<const char *> (value) + offset, value_size);

      amd_dbgapi_status_t status;

      /* Write the partial lo register.  */
      if (offset < sizeof (reg[0]))
        {
          size_t size_lo = (offset + value_size) > sizeof (reg[0])
                               ? sizeof (reg[0]) - offset
                               : value_size;

          if ((status = write_register (regnum_lo, offset, size_lo, &reg[0]))
              != AMD_DBGAPI_STATUS_SUCCESS)
            return status;

          value_size -= size_lo;
          offset = sizeof (reg[0]);
        }

      /* Write the partial hi register.  */
      if (value_size)
        {
          if ((status = write_register (regnum_hi, offset - sizeof (reg[0]),
                                        value_size, &reg[1]))
              != AMD_DBGAPI_STATUS_SUCCESS)
            return status;
        }

      return AMD_DBGAPI_STATUS_SUCCESS;
    }
  else
    {
      return AMD_DBGAPI_STATUS_ERROR_INVALID_REGISTER_ID;
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

    case AMD_DBGAPI_WAVE_INFO_DISPATCH:
      return utils::get_info (value_size, value, dispatch ().id ());

    case AMD_DBGAPI_WAVE_INFO_STOP_REASON:
      return utils::get_info (value_size, value, m_stop_reason);

    case AMD_DBGAPI_WAVE_INFO_QUEUE:
      return utils::get_info (value_size, value, queue ().id ());

    case AMD_DBGAPI_WAVE_INFO_AGENT:
      return utils::get_info (value_size, value, agent ().id ());

    case AMD_DBGAPI_WAVE_INFO_ARCHITECTURE:
      return utils::get_info (value_size, value, architecture ().id ());

    case AMD_DBGAPI_WAVE_INFO_WORK_GROUP_COORD:
      return utils::get_info (value_size, value, m_group_ids);

    case AMD_DBGAPI_WAVE_INFO_WAVE_NUMBER_IN_WORK_GROUP:
      return utils::get_info (value_size, value, m_wave_in_group);

    case AMD_DBGAPI_WAVE_INFO_PC:
      return utils::get_info (value_size, value, pc ());

    default:
      return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;
    }

  return AMD_DBGAPI_STATUS_SUCCESS;
}

} /* namespace dbgapi */
} /* namespace amd */

using namespace amd::dbgapi;

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_wave_stop (amd_dbgapi_process_id_t process_id,
                      amd_dbgapi_wave_id_t wave_id)
{
  TRY;
  TRACE (process_id, wave_id);

  if (!amd::dbgapi::is_initialized)
    return AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED;

  process_t *process = process_t::find (process_id);

  if (!process)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_PROCESS_ID;

  wave_t *wave = process->find (wave_id);

  if (!wave)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_WAVE_ID;

  /* FIXME: We can't enable this yet as a trap could set the state to STOP.
     We need the ability to track stop requests.
  if (it->second.state () == AMD_DBGAPI_WAVE_STATE_STOP)
    return AMD_DBGAPI_STATUS_ERROR_WAVE_STOPPED; */

  {
    scoped_queue_suspend_t suspend (wave->queue ());

    /* Look for the wave_id again, the wave may have exited.  */
    if (!(wave = process->find (wave_id)))
      return AMD_DBGAPI_STATUS_ERROR_INVALID_WAVE_ID;

    return wave->set_state (AMD_DBGAPI_WAVE_STATE_STOP);
  }
  CATCH;
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_wave_resume (amd_dbgapi_process_id_t process_id,
                        amd_dbgapi_wave_id_t wave_id,
                        amd_dbgapi_resume_mode_t resume_mode)
{
  TRY;
  TRACE (process_id, wave_id, resume_mode);

  if (!amd::dbgapi::is_initialized)
    return AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED;

  process_t *process = process_t::find (process_id);

  if (!process)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_PROCESS_ID;

  wave_t *wave = process->find (wave_id);

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
    if (!(wave = process->find (wave_id)))
      return AMD_DBGAPI_STATUS_ERROR_INVALID_WAVE_ID;

    return wave->set_state (resume_mode == AMD_DBGAPI_RESUME_MODE_SINGLE_STEP
                                ? AMD_DBGAPI_WAVE_STATE_SINGLE_STEP
                                : AMD_DBGAPI_WAVE_STATE_RUN);
  }
  CATCH;
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_wave_get_info (amd_dbgapi_process_id_t process_id,
                          amd_dbgapi_wave_id_t wave_id,
                          amd_dbgapi_wave_info_t query, size_t value_size,
                          void *value)
{
  TRY;
  TRACE (process_id, wave_id, query, value_size, value);

  if (!amd::dbgapi::is_initialized)
    return AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED;

  process_t *process = process_t::find (process_id);

  if (!process)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_PROCESS_ID;

  wave_t *wave = process->find (wave_id);

  if (!wave)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_WAVE_ID;

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

  return utils::get_handle_list<wave_t> (process_id, wave_count, waves,
                                         changed);
  CATCH;
}