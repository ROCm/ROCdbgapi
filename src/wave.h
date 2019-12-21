/* Copyright (c) 2019 Advanced Micro Devices, Inc.

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

#include "defs.h"

#include "agent.h"
#include "debug.h"
#include "dispatch.h"
#include "handle_object.h"
#include "queue.h"
#include "register.h"

#include <string>
#include <utility>

namespace amd
{
namespace dbgapi
{

class architecture_t;
class process_t;

/* AMD Debugger API Wave.  */

class wave_t : public detail::handle_object<amd_dbgapi_wave_id_t>
{
public:
  static constexpr amd_dbgapi_wave_id_t ignored_wave_id
      = mark_id<amd_dbgapi_wave_id_t> ();

  wave_t (amd_dbgapi_wave_id_t wave_id, dispatch_t &dispatch,
          uint32_t vgpr_count, uint32_t accvgpr_count, uint32_t sgpr_count,
          uint32_t lane_count);

  uint32_t vgpr_count () const { return m_vgpr_count; }
  uint32_t accvgpr_count () const { return m_accvgpr_count; }
  uint32_t sgpr_count () const { return m_sgpr_count; }
  uint32_t lane_count () const { return m_lane_count; }

  amd_dbgapi_global_address_t context_save_address () const
  {
    return m_context_save_address;
  }
  amd_dbgapi_global_address_t pc () const { return m_pc; }

  amd_dbgapi_status_t
  update (amd_dbgapi_global_address_t context_save_address);

  epoch_t mark () const { return m_mark; }
  void set_mark (epoch_t mark) { m_mark = mark; }

  amd_dbgapi_wave_state_t state () const { return m_state; }
  amd_dbgapi_status_t set_state (amd_dbgapi_wave_state_t state);

  amd_dbgapi_wave_stop_reason_t stop_reason () const
  {
    dbgapi_assert (state () == AMD_DBGAPI_WAVE_STATE_STOP);
    return m_stop_reason;
  }

  std::string register_name (amdgpu_regnum_t regnum) const;
  std::string register_type (amdgpu_regnum_t regnum) const;

  amd_dbgapi_status_t get_info (amd_dbgapi_wave_info_t query,
                                size_t value_size, void *value) const;

  amd_dbgapi_status_t read_register (amdgpu_regnum_t regnum, size_t offset,
                                     size_t value_size, void *value);
  amd_dbgapi_status_t write_register (amdgpu_regnum_t regnum, size_t offset,
                                      size_t value_size, const void *value);

  template <typename T>
  amd_dbgapi_status_t read_register (amdgpu_regnum_t regnum, T *value)
  {
    return read_register (regnum, 0, sizeof (T), value);
  }

  template <typename T>
  amd_dbgapi_status_t write_register (amdgpu_regnum_t regnum, T *value)
  {
    return write_register (regnum, 0, sizeof (T), value);
  }

  dispatch_t &dispatch () const { return m_dispatch; }
  queue_t &queue () const { return dispatch ().queue (); }
  agent_t &agent () const { return queue ().agent (); }
  process_t &process () const { return agent ().process (); }
  const architecture_t &architecture () const
  {
    return agent ().architecture ();
  }

  bool register_available (amdgpu_regnum_t regnum) const;
  std::pair<size_t, size_t>
  register_offset_and_size (amdgpu_regnum_t regnum) const;

private:
  epoch_t m_mark{ 0 };
  amd_dbgapi_wave_state_t m_state{ AMD_DBGAPI_WAVE_STATE_RUN };
  amd_dbgapi_wave_stop_reason_t m_stop_reason{
    AMD_DBGAPI_WAVE_STOP_REASON_NONE
  };

  uint32_t const m_vgpr_count;
  uint32_t const m_accvgpr_count;
  uint32_t const m_sgpr_count;
  uint32_t const m_lane_count;

  amd_dbgapi_global_address_t m_pc;
  uint32_t m_group_ids[3];
  uint32_t m_wave_in_group;

  amd_dbgapi_global_address_t m_context_save_address{ 0 };
  dispatch_t &m_dispatch;
};

} /* namespace dbgapi */
} /* namespace amd */

#endif /* _AMD_DBGAPI_WAVE_H */
