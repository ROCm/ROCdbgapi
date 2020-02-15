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
      /* FIXME: we should be getting the register numbers ttmp[4:5] from the
         wave->architecture ().  */

      /* Write the wave_id into ttmp[4:5].  */
      amd_dbgapi_wave_id_t wave_id = id ();
      status = process.write_global_memory (
          m_context_save_address
              + register_offset_and_size (amdgpu_regnum_t::TTMP4).first,
          &wave_id, sizeof (wave_id));
      if (status != AMD_DBGAPI_STATUS_SUCCESS)
        {
          warning ("Could not write ttmp[4:5]");
          return status;
        }

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
      /* Record the last pc before updating the hwregs cache.  */
      if (!first_update)
        m_prev_pc = pc ();

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
        process.enqueue_event (process.create<event_t> (
            process, AMD_DBGAPI_EVENT_KIND_WAVE_STOP, id ()));
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

  /* If we are resuming this wave, we'll need to reload the hwregs cache during
    the next update.  */
  if (m_state != AMD_DBGAPI_WAVE_STATE_STOP)
    m_reload_hwregs_cache = true;

  /* Clear the stop reason if we are resuming this wave.  */
  if (state != AMD_DBGAPI_WAVE_STATE_STOP)
    m_stop_reason = AMD_DBGAPI_WAVE_STOP_REASON_NONE;

  dbgapi_log (AMD_DBGAPI_LOG_LEVEL_INFO, "setting %s's state to %s, was %s",
              to_string (id ()).c_str (), to_string (state).c_str (),
              to_string (prev_state).c_str ());

  /* If we requested the wave be stopped, and the wave wasn't already stopped,
     report an event to acknowledge that the wave has stopped.  */
  if (state == AMD_DBGAPI_WAVE_STATE_STOP
      && prev_state != AMD_DBGAPI_WAVE_STATE_STOP)
    {
      m_stop_reason = AMD_DBGAPI_WAVE_STOP_REASON_NONE;
      process ().enqueue_event (process ().create<event_t> (
          process (), AMD_DBGAPI_EVENT_KIND_WAVE_STOP, id ()));
    }

  return AMD_DBGAPI_STATUS_SUCCESS;
}

bool
wave_t::register_available (amdgpu_regnum_t regnum) const
{
  if (regnum >= amdgpu_regnum_t::FIRST_VGPR_32
      && regnum <= amdgpu_regnum_t::LAST_VGPR_32)
    {
      return lane_count () == 32
             && ((regnum - amdgpu_regnum_t::V0_32) < m_vgpr_count);
    }
  if (regnum >= amdgpu_regnum_t::FIRST_VGPR_64
      && regnum <= amdgpu_regnum_t::LAST_VGPR_64)
    {
      return lane_count () == 64
             && ((regnum - amdgpu_regnum_t::V0_64) < m_vgpr_count);
    }
  if (regnum >= amdgpu_regnum_t::FIRST_ACCVGPR_64
      && regnum <= amdgpu_regnum_t::LAST_ACCVGPR_64)
    {
      return lane_count () == 64 && architecture ().has_acc_vgprs ()
             && ((regnum - amdgpu_regnum_t::ACC0_64) < m_accvgpr_count);
    }
  if (regnum >= amdgpu_regnum_t::FIRST_SGPR
      && regnum <= amdgpu_regnum_t::LAST_SGPR)
    {
      return ((regnum - amdgpu_regnum_t::S0) < std::min (102u, m_sgpr_count));
    }
  if (regnum >= amdgpu_regnum_t::FIRST_HWREG
      && regnum <= amdgpu_regnum_t::LAST_HWREG)
    {
      return true;
    }
  if (regnum >= amdgpu_regnum_t::FIRST_TTMP
      && regnum <= amdgpu_regnum_t::LAST_TTMP)
    {
      return true;
    }
  if (regnum >= amdgpu_regnum_t::FIRST_PSEUDO
      && regnum <= amdgpu_regnum_t::LAST_PSEUDO)
    {
      return true;
    }

  return false;
}

std::string
wave_t::register_name (amdgpu_regnum_t regnum) const
{
  return register_available (regnum) ? architecture ().register_name (regnum)
                                     : "";
}

std::pair<size_t, size_t>
wave_t::register_offset_and_size (amdgpu_regnum_t regnum) const
{
  if (!register_available (regnum))
    return std::make_pair (0, 0);

  if (lane_count () == 32 && regnum >= amdgpu_regnum_t::FIRST_VGPR_32
      && regnum <= amdgpu_regnum_t::LAST_VGPR_32)
    {
      size_t vgprs_offset = 0;
      size_t vgpr_size = sizeof (int32_t) * 32;
      size_t vgpr_num = regnum - amdgpu_regnum_t::V0_32;

      return std::make_pair (vgprs_offset + vgpr_num * vgpr_size, vgpr_size);
    }
  else if (lane_count () == 64 && regnum >= amdgpu_regnum_t::FIRST_VGPR_64
           && regnum <= amdgpu_regnum_t::LAST_VGPR_64)
    {
      size_t vgprs_offset = 0;
      size_t vgpr_size = sizeof (int32_t) * 64;
      size_t vgpr_num = regnum - amdgpu_regnum_t::V0_64;

      return std::make_pair (vgprs_offset + vgpr_num * vgpr_size, vgpr_size);
    }
  else if (lane_count () == 64 && regnum >= amdgpu_regnum_t::FIRST_ACCVGPR_64
           && regnum <= amdgpu_regnum_t::LAST_ACCVGPR_64)
    {
      size_t accvgprs_offset = m_vgpr_count * sizeof (int32_t) * m_lane_count;
      size_t accvgpr_size = sizeof (int32_t) * m_lane_count;
      size_t accvgpr_num = regnum - amdgpu_regnum_t::ACC0_64;

      return std::make_pair (accvgprs_offset + accvgpr_num * accvgpr_size,
                             accvgpr_size);
    }
  else if (regnum >= amdgpu_regnum_t::FIRST_SGPR
           && regnum <= amdgpu_regnum_t::LAST_SGPR)
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
      size_t hwreg_size
          = (regnum == amdgpu_regnum_t::PC || regnum == amdgpu_regnum_t::EXEC)
                ? sizeof (uint64_t)
                : sizeof (uint32_t);
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

  return std::make_pair (0, 0);
}

/* TODO: Move this to architecture_t.  Needs pseudo regs to handle EXEC_32 and
   EXEC_64.  */
std::string
wave_t::register_type (amdgpu_regnum_t regnum) const
{
  if (!register_available (regnum))
    return "";

  /* Vector registers (arch and acc).  */
  if (lane_count () == 32
      && (regnum >= amdgpu_regnum_t::FIRST_VGPR_32
          && regnum <= amdgpu_regnum_t::LAST_VGPR_32))
    {
      return "int32_t[32]";
    }
  else if (lane_count () == 64
           && ((regnum >= amdgpu_regnum_t::FIRST_VGPR_64
                && regnum <= amdgpu_regnum_t::LAST_VGPR_64)
               || (architecture ().has_acc_vgprs ()
                   && regnum >= amdgpu_regnum_t::FIRST_ACCVGPR_64
                   && regnum <= amdgpu_regnum_t::LAST_ACCVGPR_64)))
    {
      return "int32_t[64]";
    }
  /* Scalar registers.  */
  else if (regnum >= amdgpu_regnum_t::FIRST_SGPR
           && regnum <= amdgpu_regnum_t::LAST_SGPR)
    {
      return "int32_t";
    }
  /* Program counter.  */
  else if (regnum == amdgpu_regnum_t::PC)
    {
      return "void (*)()";
    }
  else if (regnum == amdgpu_regnum_t::EXEC)
    {
      switch (lane_count ())
        {
        case 32:
          return "uint32_t";
        case 64:
          return "uint64_t";
        default:
          error ("wave size = %d is not supported", lane_count ());
        }
    }
  /* Everything else (hwregs, ttmps).  */
  else if ((regnum >= amdgpu_regnum_t::FIRST_HWREG
            && regnum <= amdgpu_regnum_t::LAST_HWREG)
           || (regnum >= amdgpu_regnum_t::FIRST_TTMP
               && regnum <= amdgpu_regnum_t::LAST_TTMP)
           || (regnum >= amdgpu_regnum_t::FIRST_PSEUDO
               && regnum <= amdgpu_regnum_t::LAST_PSEUDO))
    {
      return "uint32_t";
    }

  return "";
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
wave_t::write_register (amdgpu_regnum_t regnum, size_t offset,
                        size_t value_size, const void *value)
{
  size_t reg_offset, reg_size;
  std::tie (reg_offset, reg_size) = register_offset_and_size (regnum);

  if (!reg_size)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_REGISTER_ID;

  if (!value_size || (offset + value_size) > reg_size)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT_SIZE;

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

    amd_dbgapi_status_t status = wave->set_state (AMD_DBGAPI_WAVE_STATE_STOP);

    if (status == AMD_DBGAPI_STATUS_ERROR_INVALID_WAVE_ID)
      process->destroy (wave);

    return status;
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

    amd_dbgapi_status_t status
        = wave->set_state (resume_mode == AMD_DBGAPI_RESUME_MODE_SINGLE_STEP
                               ? AMD_DBGAPI_WAVE_STATE_SINGLE_STEP
                               : AMD_DBGAPI_WAVE_STATE_RUN);

    if (status == AMD_DBGAPI_STATUS_ERROR_INVALID_WAVE_ID)
      process->destroy (wave);

    return status;
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
