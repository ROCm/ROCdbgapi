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

#ifndef _AMD_DBGAPI_WAVE_H
#define _AMD_DBGAPI_WAVE_H 1

#include "agent.h"
#include "amd-dbgapi.h"
#include "debug.h"
#include "dispatch.h"
#include "handle_object.h"
#include "queue.h"
#include "register.h"
#include "utils.h"

#include <cstddef>
#include <cstdint>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace amd
{
namespace dbgapi
{

class architecture_t;
class address_space_t;
class process_t;

/* AMD Debugger API Wave.  */

class wave_t : public detail::handle_object<amd_dbgapi_wave_id_t>
{
public:
  /* New waves are always created by the hardware with an undefined wave_id.
     A wave with an undefined wave_id could be halted at launch if the launch
     mode was set to os_wave_launch_mode_t::HALT when it was  created. Waves
     halted at launch do not have a trap/exception raised by the
     trap handler.  */
  static constexpr amd_dbgapi_wave_id_t undefined = { 0 };

  enum class visibility_t
  {
    VISIBLE,
    /* Waves with HIDDEN_HALTED_AT_LAUNCH visibility are waves that are halted
       at launch because the launch mode was set to
       os_wave_launch_mode_t::HALT when they were created. These waves should
       not be reported to the client until the launch mode is changed to
       os_wave_launch_mode_t::NORMAL.  */
    HIDDEN_HALTED_AT_LAUNCH,
    /* Waves with HIDDEN_AT_ENDPGM visibility are waves that are terminating
       (about to execute a s_endpgm instruction). These waves should never be
       reported to the client and will be destroyed in the next mark and sweep
       when they finally terminate.  */
    HIDDEN_AT_ENDPGM
  };

private:
  amd_dbgapi_status_t
  xfer_private_memory_swizzled (amd_dbgapi_segment_address_t segment_address,
                                amd_dbgapi_lane_id_t lane_id, void *read,
                                const void *write, size_t *size);

  amd_dbgapi_status_t
  xfer_private_memory_unswizzled (amd_dbgapi_segment_address_t segment_address,
                                  void *read, const void *write, size_t *size);

  amd_dbgapi_status_t
  xfer_local_memory (amd_dbgapi_segment_address_t segment_address, void *read,
                     const void *write, size_t *size);

public:
  wave_t (amd_dbgapi_wave_id_t wave_id, dispatch_t &dispatch,
          size_t vgpr_count, size_t accvgpr_count, size_t sgpr_count,
          amd_dbgapi_size_t local_memory_offset,
          amd_dbgapi_size_t local_memory_size, size_t lane_count);

  visibility_t visibility () const { return m_visibility; }
  void set_visibility (visibility_t visibility);

  bool is_valid () const { return visibility () == visibility_t::VISIBLE; }

  const wave_t &group_leader () const
  {
    dbgapi_assert (m_group_leader
                   /* Make sure the group leader truly is a group leader.  */
                   && m_group_leader == m_group_leader->m_group_leader);
    return *m_group_leader;
  }

  size_t vgpr_count () const { return m_vgpr_count; }
  size_t accvgpr_count () const { return m_accvgpr_count; }
  size_t sgpr_count () const { return m_sgpr_count; }
  size_t lane_count () const { return m_lane_count; }

  amd_dbgapi_global_address_t context_save_address () const
  {
    return m_context_save_address;
  }

  amd_dbgapi_global_address_t local_memory_base_address () const
  {
    return (m_group_leader == this)
               ? m_context_save_address + m_local_memory_offset
               : group_leader ().local_memory_base_address ();
  }
  amd_dbgapi_size_t local_memory_size () const { return m_local_memory_size; }

  auto group_ids () const
  {
    return std::tie (m_group_ids[0], m_group_ids[1], m_group_ids[2]);
  }

  uint64_t exec_mask () const;
  amd_dbgapi_global_address_t pc () const;
  amd_dbgapi_global_address_t saved_pc () const { return m_saved_pc; }
  std::vector<uint8_t> instruction_at_pc () const;

  amd_dbgapi_status_t park ();
  amd_dbgapi_status_t unpark ();

  amd_dbgapi_status_t
  update (const wave_t &group_leader,
          amd_dbgapi_global_address_t context_save_address);

  epoch_t mark () const { return m_mark; }
  void set_mark (epoch_t mark) { m_mark = mark; }

  amd_dbgapi_wave_state_t state () const { return m_state; }
  amd_dbgapi_status_t set_state (amd_dbgapi_wave_state_t state);

  amd_dbgapi_wave_stop_reason_t stop_reason () const
  {
    dbgapi_assert (state () == AMD_DBGAPI_WAVE_STATE_STOP);
    return m_stop_reason;
  }

  utils::optional<std::string> register_name (amdgpu_regnum_t regnum) const;
  utils::optional<std::string> register_type (amdgpu_regnum_t regnum) const;

  bool is_register_cached (amdgpu_regnum_t regnum) const
  {
    return m_context_save_address
           && (regnum == amdgpu_regnum_t::PC
               || regnum == amdgpu_regnum_t::EXEC_32
               || regnum == amdgpu_regnum_t::EXEC_64
               || (regnum >= amdgpu_regnum_t::FIRST_HWREG
                   && regnum <= amdgpu_regnum_t::LAST_HWREG));
  }

  bool is_register_available (amdgpu_regnum_t regnum) const;

  std::pair<size_t, size_t>
  register_offset_and_size (amdgpu_regnum_t regnum,
                            bool include_aliased_registers = true) const;

  template <typename T>
  amd_dbgapi_status_t read_register (amdgpu_regnum_t regnum, T *value) const
  {
    return read_register (regnum, 0, sizeof (T), value);
  }

  template <typename T>
  amd_dbgapi_status_t write_register (amdgpu_regnum_t regnum, T *value)
  {
    return write_register (regnum, 0, sizeof (T), value);
  }

  amd_dbgapi_status_t read_register (amdgpu_regnum_t regnum, size_t offset,
                                     size_t value_size, void *value) const;
  amd_dbgapi_status_t write_register (amdgpu_regnum_t regnum, size_t offset,
                                      size_t value_size, const void *value);

  amd_dbgapi_status_t
  xfer_segment_memory (const address_space_t &address_space,
                       amd_dbgapi_lane_id_t lane_id,
                       amd_dbgapi_segment_address_t segment_address,
                       void *read, const void *write, size_t *size);

  amd_dbgapi_status_t get_info (amd_dbgapi_wave_info_t query,
                                size_t value_size, void *value) const;

  dispatch_t &dispatch () const { return m_dispatch; }
  queue_t &queue () const { return dispatch ().queue (); }
  agent_t &agent () const { return queue ().agent (); }
  process_t &process () const { return agent ().process (); }
  const architecture_t &architecture () const
  {
    return agent ().architecture ();
  }

private:
  epoch_t m_mark{ 0 };
  amd_dbgapi_wave_state_t m_state{ AMD_DBGAPI_WAVE_STATE_RUN };
  visibility_t m_visibility{ visibility_t::VISIBLE };
  amd_dbgapi_wave_stop_reason_t m_stop_reason{};

  bool m_reload_hwregs_cache{ true };
  uint32_t m_hwregs_cache[amdgpu_regnum_t::LAST_HWREG
                          - amdgpu_regnum_t::FIRST_HWREG + 1];

  size_t const m_vgpr_count;
  size_t const m_accvgpr_count;
  size_t const m_sgpr_count;
  size_t const m_lane_count;

  amd_dbgapi_size_t m_scratch_offset{ 0 };
  amd_dbgapi_global_address_t m_saved_pc{ 0 };
  bool m_parked{ false };

  uint32_t m_group_ids[3];
  uint32_t m_wave_in_group;

  amd_dbgapi_global_address_t m_context_save_address{ 0 };

  amd_dbgapi_size_t const m_local_memory_offset{ 0 };
  amd_dbgapi_size_t const m_local_memory_size{ 0 };
  const wave_t *m_group_leader{ nullptr };
  dispatch_t &m_dispatch;
};

} /* namespace dbgapi */
} /* namespace amd */

#endif /* _AMD_DBGAPI_WAVE_H */
