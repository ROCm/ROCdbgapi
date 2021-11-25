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

#ifndef AMD_DBGAPI_WAVE_H
#define AMD_DBGAPI_WAVE_H 1

#include "agent.h"
#include "amd-dbgapi.h"
#include "architecture.h"
#include "debug.h"
#include "dispatch.h"
#include "exception.h"
#include "handle_object.h"
#include "memory.h"
#include "queue.h"
#include "register.h"
#include "utils.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <vector>

namespace amd::dbgapi
{

class event_t;
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

  /* Mask of stop reasons a wave can have and still be resumed. These stop
     reasons are not fatal, and are cleared when the wave is resumed.  */
  static constexpr amd_dbgapi_wave_stop_reasons_t resumable_stop_reason_mask
    = AMD_DBGAPI_WAVE_STOP_REASON_BREAKPOINT
      | AMD_DBGAPI_WAVE_STOP_REASON_WATCHPOINT
      | AMD_DBGAPI_WAVE_STOP_REASON_SINGLE_STEP
      | AMD_DBGAPI_WAVE_STOP_REASON_DEBUG_TRAP
      | AMD_DBGAPI_WAVE_STOP_REASON_TRAP;

  enum class visibility_t
  {
    visible,
    /* Waves with hidden_halted_at_launch visibility are waves that are halted
       at launch because the launch mode was set to
       os_wave_launch_mode_t::HALT when they were created. These waves should
       not be reported to the client until the launch mode is changed to
       os_wave_launch_mode_t::NORMAL.  */
    hidden_halted_at_launch,
    /* Waves with hidden_at_terminating_instruction visibility are waves that
       are terminating (about to execute a terminating instruction). These
       waves should never be reported to the client and will be destroyed in
       the next mark and sweep when they finally terminate.  */
    hidden_at_terminating_instruction
  };

private:
  amd_dbgapi_wave_state_t m_state{ AMD_DBGAPI_WAVE_STATE_RUN };
  bool m_stop_requested{ false };
  amd_dbgapi_wave_stop_reasons_t m_stop_reason{};
  amd_dbgapi_global_address_t m_parked_pc{ 0 };
  amd_dbgapi_global_address_t m_last_stopped_pc{ 0 };
  epoch_t m_mark{ 0 };

  amd_dbgapi_event_id_t m_last_stop_event_id{ AMD_DBGAPI_EVENT_NONE };
  visibility_t m_visibility{ visibility_t::visible };
  bool m_is_parked{ false };

  std::array<uint32_t, 3> m_group_ids{ 0, 0, 0 };
  uint32_t m_wave_in_group{ 0 };

  std::unique_ptr<architecture_t::cwsr_record_t> m_cwsr_record{};

  displaced_stepping_t *m_displaced_stepping{ nullptr };
  const wave_t *m_group_leader{ nullptr };
  const dispatch_t &m_dispatch;

  [[nodiscard]] size_t
  xfer_private_memory_swizzled (amd_dbgapi_segment_address_t segment_address,
                                amd_dbgapi_lane_id_t lane_id, void *read,
                                const void *write, size_t size);

  [[nodiscard]] size_t
  xfer_private_memory_unswizzled (amd_dbgapi_segment_address_t segment_address,
                                  void *read, const void *write, size_t size);

  [[nodiscard]] size_t
  xfer_local_memory (amd_dbgapi_segment_address_t segment_address, void *read,
                     const void *write, size_t size);

  void raise_event (amd_dbgapi_event_kind_t event_kind);

  void park ();
  void unpark ();

public:
  wave_t (amd_dbgapi_wave_id_t wave_id, const dispatch_t &dispatch);
  ~wave_t ();

  /* Disable copies.  */
  wave_t (const wave_t &) = delete;
  wave_t &operator= (const wave_t &) = delete;

  visibility_t visibility () const { return m_visibility; }
  void set_visibility (visibility_t visibility);

  bool is_valid () const { return visibility () == visibility_t::visible; }

  /* Return the last wave stop event, or nullptr if the event is already
     processed and destroyed.  */
  const event_t *last_stop_event () const;

  const wave_t &group_leader () const
  {
    dbgapi_assert (m_group_leader
                   /* Make sure the group leader truly is a group leader.  */
                   && m_group_leader == m_group_leader->m_group_leader);
    return *m_group_leader;
  }

  size_t lane_count () const { return m_cwsr_record->lane_count (); }

  auto group_ids () const { return m_group_ids; }

  uint64_t exec_mask () const;
  amd_dbgapi_global_address_t pc () const;
  amd_dbgapi_global_address_t last_stopped_pc () const
  {
    /* Return the last known pc before the wave was resumed. It is only valid
       while the wave is running.  */
    dbgapi_assert (state () != AMD_DBGAPI_WAVE_STATE_STOP && "not running");
    return m_last_stopped_pc;
  }
  std::optional<instruction_t> instruction_at_pc (size_t pc_adjust = 0) const;

  void terminate ();
  void displaced_stepping_start (const void *saved_instruction_bytes);
  void displaced_stepping_complete ();
  const displaced_stepping_t *displaced_stepping () const
  {
    return m_displaced_stepping;
  }

  /* Update the wave's status from its saved state in the context save area. */
  void update (const wave_t &group_leader,
               std::unique_ptr<architecture_t::cwsr_record_t> cwsr_record);

  static epoch_t next_mark ()
  {
    static monotonic_counter_t<epoch_t, 1> next_wave_mark{};
    return next_wave_mark ();
  }
  epoch_t mark () const { return m_mark; }
  void set_mark (epoch_t mark) { m_mark = mark; }

  amd_dbgapi_wave_state_t client_visible_state () const;
  amd_dbgapi_wave_state_t state () const { return m_state; }
  void set_state (amd_dbgapi_wave_state_t state,
                  amd_dbgapi_exceptions_t exceptions
                  = AMD_DBGAPI_EXCEPTION_NONE);

  bool stop_requested () const { return m_stop_requested; }
  amd_dbgapi_wave_stop_reasons_t stop_reason () const
  {
    dbgapi_assert (state () == AMD_DBGAPI_WAVE_STATE_STOP);
    return m_stop_reason;
  }

  bool is_register_available (amdgpu_regnum_t regnum) const;

  std::optional<amd_dbgapi_global_address_t>
  register_address (amdgpu_regnum_t regnum) const
  {
    return m_cwsr_record->register_address (regnum);
  }

  void read_register (amdgpu_regnum_t regnum, size_t offset, size_t value_size,
                      void *value) const;

  void write_register (amdgpu_regnum_t regnum, size_t offset,
                       size_t value_size, const void *value);

  template <typename T>
  void read_register (amdgpu_regnum_t regnum, T *value) const
  {
    try
      {
        read_register (regnum, 0, sizeof (T), value);
      }
    catch (const api_error_t &e)
      {
        throw fatal_error_t (string_printf (
          "Could not read the `%s' register: %s",
          architecture ().register_name (regnum).c_str (), e.what ()));
      }
  }

  template <typename T> void write_register (amdgpu_regnum_t regnum, T *value)
  {
    try
      {
        write_register (regnum, 0, sizeof (T), value);
      }
    catch (const api_error_t &e)
      {
        throw fatal_error_t (string_printf (
          "Could not write the `%s' register: %s",
          architecture ().register_name (regnum).c_str (), e.what ()));
      }
  }

  /* Return the wave's scratch memory region (address and size).  */
  std::pair<amd_dbgapi_global_address_t /* address */,
            amd_dbgapi_size_t /* size */>
  scratch_memory_region () const
  {
    return queue ().scratch_memory_region (
      m_cwsr_record->shader_engine_id (),
      m_cwsr_record->scratch_scoreboard_id ());
  }

  [[nodiscard]] size_t
  xfer_segment_memory (const address_space_t &address_space,
                       amd_dbgapi_lane_id_t lane_id,
                       amd_dbgapi_segment_address_t segment_address,
                       void *read, const void *write, size_t size);

  void get_info (amd_dbgapi_wave_info_t query, size_t value_size,
                 void *value) const;

  const dispatch_t &dispatch () const { return m_dispatch; }
  compute_queue_t &queue () const { return dispatch ().queue (); }
  const agent_t &agent () const { return queue ().agent (); }
  process_t &process () const { return agent ().process (); }
  const architecture_t &architecture () const
  {
    return queue ().architecture ();
  }
};

} /* namespace amd::dbgapi */

#endif /* AMD_DBGAPI_WAVE_H */
