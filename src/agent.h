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

#ifndef _AMD_DBGAPI_AGENT_H
#define _AMD_DBGAPI_AGENT_H 1

#include "amd-dbgapi.h"
#include "handle_object.h"
#include "os_driver.h"
#include "utils.h"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <string>
#include <unordered_map>

namespace amd
{
namespace dbgapi
{

class architecture_t;
class process_t;
class watchpoint_t;

/* ROCm Debug Agent.  */

class agent_t : public detail::handle_object<amd_dbgapi_agent_id_t>
{
public:
  agent_t (amd_dbgapi_agent_id_t agent_id, process_t &process,
           const architecture_t &architecture,
           const os_agent_snapshot_entry_t &os_agent_info);
  ~agent_t ();

  os_agent_id_t gpu_id () const { return m_os_agent_info.os_agent_id; }
  os_wave_launch_trap_mask_t supported_trap_mask () const
  {
    return m_supported_trap_mask;
  }

  file_desc_t poll_fd () const
  {
    dbgapi_assert (m_poll_fd && "poll_fd is not initialized");
    return *m_poll_fd;
  }

  std::atomic<bool> &os_event_notifier () { return m_os_event_notifier; }
  amd_dbgapi_status_t next_os_event (amd_dbgapi_queue_id_t *queue_id,
                                     os_queue_status_t *queue_status);

  amd_dbgapi_status_t get_info (amd_dbgapi_agent_info_t query,
                                size_t value_size, void *value) const;

  const architecture_t &architecture () const { return m_architecture; }
  process_t &process () const { return m_process; }

  amd_dbgapi_status_t enable_debug_trap ();
  amd_dbgapi_status_t disable_debug_trap ();
  bool debug_trap_enabled () const { return m_poll_fd.has_value (); }

  amd_dbgapi_status_t
  insert_watchpoint (const watchpoint_t &watchpoint,
                     amd_dbgapi_global_address_t adjusted_address,
                     amd_dbgapi_global_address_t adjusted_mask);
  amd_dbgapi_status_t remove_watchpoint (const watchpoint_t &watchpoint);
  const watchpoint_t *find_watchpoint (os_watch_id_t os_watch_id) const;

private:
  os_agent_snapshot_entry_t const m_os_agent_info;
  os_wave_launch_trap_mask_t m_supported_trap_mask{};

  utils::optional<file_desc_t> m_poll_fd;
  std::atomic<bool> m_os_event_notifier{ false };
  std::unordered_map<os_watch_id_t, const watchpoint_t *> m_watchpoint_map;

  const architecture_t &m_architecture;
  process_t &m_process;
};

} /* namespace dbgapi */
} /* namespace amd */

#endif /* _AMD_DBGAPI_AGENT_H */
