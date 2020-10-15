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

#ifndef AMD_DBGAPI_WAVE_H
#define AMD_DBGAPI_WAVE_H 1

#include "agent.h"
#include "amd-dbgapi.h"
#include "architecture.h"
#include "debug.h"
#include "dispatch.h"
#include "handle_object.h"
#include "queue.h"
#include "register.h"
#include "utils.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace amd::dbgapi
{

class architecture_t;
class address_space_t;
class displaced_stepping_t;
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
    visible,
    /* Waves with hidden_halted_at_launch visibility are waves that are halted
       at launch because the launch mode was set to
       os_wave_launch_mode_t::HALT when they were created. These waves should
       not be reported to the client until the launch mode is changed to
       os_wave_launch_mode_t::NORMAL.  */
    hidden_halted_at_launch,
    /* Waves with hidden_at_endpgm visibility are waves that are terminating
       (about to execute a s_endpgm instruction). These waves should never be
       reported to the client and will be destroyed in the next mark and sweep
       when they finally terminate.  */
    hidden_at_endpgm
  };

  enum class register_cache_policy_t
  {
    /* If write-through is used, the cached registers are written immediately
       both in the cache and in memory.  */
    write_through,
    /* If write-back is used, the cached registers are immediately updated in
       the cache, and later updated in memory when the wave is un-halted.  */
    write_back
  };

  static constexpr register_cache_policy_t register_cache_policy
      = register_cache_policy_t::write_back;

  struct callbacks_t
  {
    /* Return the current scratch backing memory address.  */
    std::function<amd_dbgapi_global_address_t ()> scratch_memory_base;
    /* Return the current scratch backing memory size.  */
    std::function<amd_dbgapi_size_t ()> scratch_memory_size;
  };

private:
  epoch_t m_mark{ 0 };
  amd_dbgapi_wave_state_t m_state{ AMD_DBGAPI_WAVE_STATE_RUN };
  visibility_t m_visibility{ visibility_t::visible };
  amd_dbgapi_wave_stop_reason_t m_stop_reason{};

  uint32_t m_hwregs_cache[amdgpu_regnum_t::last_hwreg
                          - amdgpu_regnum_t::first_hwreg + 1];

  std::unique_ptr<architecture_t::cwsr_descriptor_t> m_descriptor;

  amd_dbgapi_size_t m_scratch_offset{ 0 };
  amd_dbgapi_global_address_t m_saved_pc{ 0 };
  bool m_is_parked{ false };

  std::array<uint32_t, 3> m_group_ids;
  uint32_t m_wave_in_group;

  displaced_stepping_t *m_displaced_stepping{ nullptr };
  const wave_t *m_group_leader{ nullptr };
  const callbacks_t &m_callbacks;
  dispatch_t &m_dispatch;

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
          const callbacks_t &callbacks)
      : handle_object (wave_id), m_callbacks (callbacks), m_dispatch (dispatch)
  {
  }

  visibility_t visibility () const { return m_visibility; }
  void set_visibility (visibility_t visibility);

  bool is_valid () const { return visibility () == visibility_t::visible; }

  const wave_t &group_leader () const
  {
    dbgapi_assert (m_group_leader
                   /* Make sure the group leader truly is a group leader.  */
                   && m_group_leader == m_group_leader->m_group_leader);
    return *m_group_leader;
  }

  size_t lane_count () const
  {
    return architecture ().wave_get_info (
        *m_descriptor, architecture_t::wave_info_t::lane_count);
  }

  auto group_ids () const { return m_group_ids; }

  uint64_t exec_mask () const;
  amd_dbgapi_global_address_t pc () const;
  amd_dbgapi_global_address_t saved_pc () const { return m_saved_pc; }
  std::optional<std::vector<uint8_t>> instruction_at_pc () const;

  amd_dbgapi_status_t park ();
  amd_dbgapi_status_t unpark ();

  amd_dbgapi_status_t
  displaced_stepping_start (displaced_stepping_t &displaced_stepping);
  amd_dbgapi_status_t displaced_stepping_complete ();
  const displaced_stepping_t *displaced_stepping () const
  {
    return m_displaced_stepping;
  }

  amd_dbgapi_status_t
  update (const wave_t &group_leader,
          std::unique_ptr<architecture_t::cwsr_descriptor_t> descriptor);

  epoch_t mark () const { return m_mark; }
  void set_mark (epoch_t mark) { m_mark = mark; }

  amd_dbgapi_wave_state_t state () const { return m_state; }
  amd_dbgapi_status_t set_state (amd_dbgapi_wave_state_t state);

  amd_dbgapi_wave_stop_reason_t stop_reason () const
  {
    dbgapi_assert (state () == AMD_DBGAPI_WAVE_STATE_STOP);
    return m_stop_reason;
  }

  bool is_register_cached (amdgpu_regnum_t regnum) const;
  bool is_register_available (amdgpu_regnum_t regnum) const;

  std::optional<amd_dbgapi_global_address_t>
  register_address (amdgpu_regnum_t regnum) const
  {
    return architecture ().register_address (*m_descriptor, regnum);
  }

  amd_dbgapi_status_t read_register (amdgpu_regnum_t regnum, size_t offset,
                                     size_t value_size, void *value) const;
  amd_dbgapi_status_t write_register (amdgpu_regnum_t regnum, size_t offset,
                                      size_t value_size, const void *value);

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
};

} /* namespace amd::dbgapi */

#endif /* AMD_DBGAPI_WAVE_H */
