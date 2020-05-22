/* Copyright (c) 2020 Advanced Micro Devices, Inc.

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

#ifndef _AMD_DBGAPI_OS_DRIVER_H
#define _AMD_DBGAPI_OS_DRIVER_H 1

#include "amd-dbgapi.h"
#include "linux/kfd_ioctl.h"
#include "logging.h"
#include "utils.h"

#include <cstddef>
#include <cstdint>
#include <string>
#include <type_traits>
#include <utility>

namespace amd
{
namespace dbgapi
{

class agent_t;
class process_t;

constexpr uint32_t OS_DRIVER_MAJOR_VERSION = KFD_IOCTL_DBG_MAJOR_VERSION;
constexpr uint32_t OS_DRIVER_MINOR_VERSION = KFD_IOCTL_DBG_MINOR_VERSION;

using os_agent_id_t = uint32_t;
using os_queue_id_t = uint32_t;
using os_queue_snapshot_entry_t = kfd_queue_snapshot_entry;

enum class os_queue_type_t : uint32_t
{
  COMPUTE = KFD_IOC_QUEUE_TYPE_COMPUTE,
  SDMA = KFD_IOC_QUEUE_TYPE_SDMA,
  COMPUTE_AQL = KFD_IOC_QUEUE_TYPE_COMPUTE_AQL,
  SDMA_XGMI = KFD_IOC_QUEUE_TYPE_SDMA_XGMI
};

enum class os_queue_status_t : uint32_t
{
  TRAP = KFD_DBG_EV_STATUS_TRAP,
  VMFAULT = KFD_DBG_EV_STATUS_VMFAULT,
  SUSPENDED = KFD_DBG_EV_STATUS_SUSPENDED,
  NEW_QUEUE = KFD_DBG_EV_STATUS_NEW_QUEUE
};
template <> struct is_flag<os_queue_status_t> : std::true_type
{
};

inline os_queue_status_t
os_queue_status (os_queue_snapshot_entry_t entry)
{
  return static_cast<os_queue_status_t> (entry.queue_status);
}

inline os_queue_type_t
os_queue_type (os_queue_snapshot_entry_t entry)
{
  return static_cast<os_queue_type_t> (entry.queue_type);
}

constexpr os_queue_id_t OS_INVALID_QUEUEID = KFD_INVALID_QUEUEID;
constexpr os_queue_id_t OS_QUEUE_ERROR_MASK = KFD_DBG_QUEUE_ERROR_MASK;
constexpr os_queue_id_t OS_QUEUE_INVALID_MASK = KFD_DBG_QUEUE_INVALID_MASK;

enum class os_wave_launch_mode_t : uint32_t
{
  NORMAL = 0,      /* Waves launch normally.  */
  HALT = 1,        /* Waves launch in halted mode.  */
  KILL = 2,        /* Waves terminate before executing any instructions.  */
  SINGLE_STEP = 3, /* Waves launch in single-step mode.  */
  DISABLE = 4,     /* Disable launching any new waves.  */
};

class os_driver_t
{
private:
  int kfd_dbg_trap_ioctl (uint32_t action,
                          kfd_ioctl_dbg_trap_args *args) const;

public:
  os_driver_t (process_t &process);
  ~os_driver_t ();

  bool is_valid () const { return m_kfd_fd != -1; }

  amd_dbgapi_status_t get_version (uint32_t *major, uint32_t *minor) const;

  amd_dbgapi_status_t enable_debug_trap (const agent_t &agent,
                                         file_desc_t *poll_fd) const;
  amd_dbgapi_status_t disable_debug_trap (const agent_t &agent) const;

  amd_dbgapi_status_t
  query_debug_event (const agent_t &agent, os_queue_id_t *os_queue_id,
                     os_queue_status_t *os_queue_status) const;

  size_t suspend_queues (os_queue_id_t *queues, size_t queue_count) const;
  size_t resume_queues (os_queue_id_t *queues, size_t queue_count) const;

  amd_dbgapi_status_t queue_snapshot (os_queue_snapshot_entry_t *snapshots,
                                      size_t snapshot_count,
                                      size_t *queue_count) const;

  amd_dbgapi_status_t set_wave_launch_mode (const agent_t &agent,
                                            os_wave_launch_mode_t mode) const;

private:
  file_desc_t m_kfd_fd{ -1 };

  process_t &m_process;
};

template <>
inline std::string
to_string (os_wave_launch_mode_t mode)
{
  switch (mode)
    {
    case os_wave_launch_mode_t::NORMAL:
      return "WAVE_LAUNCH_MODE_NORMAL";
    case os_wave_launch_mode_t::HALT:
      return "WAVE_LAUNCH_MODE_HALT";
    case os_wave_launch_mode_t::KILL:
      return "WAVE_LAUNCH_MODE_KILL";
    case os_wave_launch_mode_t::SINGLE_STEP:
      return "WAVE_LAUNCH_MODE_SINGLE_STEP";
    case os_wave_launch_mode_t::DISABLE:
      return "WAVE_LAUNCH_MODE_DISABLE";
    }
  return to_string (
      make_hex (static_cast<std::underlying_type_t<decltype (mode)>> (mode)));
}

template <>
inline std::string
to_string (os_queue_status_t queue_status)
{
  std::string str
      = (queue_status & os_queue_status_t::TRAP) != 0
            ? "TRAP"
            : (!!(queue_status & os_queue_status_t::VMFAULT) ? "VMFAULT"
                                                             : "UNKNOWN");

  if ((queue_status & os_queue_status_t::NEW_QUEUE) != 0)
    str += "|NEW_QUEUE";
  if ((queue_status & os_queue_status_t::SUSPENDED) != 0)
    str += "|SUSPENDED";

  return str;
}

} /* namespace dbgapi */
} /* namespace amd */

#endif /* _AMD_DBGAPI_OS_DRIVER_H */
