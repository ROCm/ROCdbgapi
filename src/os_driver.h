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
#include <memory>
#include <optional>
#include <string>
#include <type_traits>

namespace amd::dbgapi
{

/* See https://llvm.org/docs/AMDGPUUsage.html#amdgpu-ef-amdgpu-mach-table  */
enum elf_amdgpu_machine_t : uint32_t
{
  EF_AMDGPU_MACH_NONE = 0x000,
  EF_AMDGPU_MACH_AMDGCN_GFX900 = 0x02c,
  EF_AMDGPU_MACH_AMDGCN_GFX902 = 0x02d,
  EF_AMDGPU_MACH_AMDGCN_GFX904 = 0x02e,
  EF_AMDGPU_MACH_AMDGCN_GFX906 = 0x02f,
  EF_AMDGPU_MACH_AMDGCN_GFX908 = 0x030,
  EF_AMDGPU_MACH_AMDGCN_GFX1010 = 0x033,
  EF_AMDGPU_MACH_AMDGCN_GFX1011 = 0x034,
  EF_AMDGPU_MACH_AMDGCN_GFX1012 = 0x035,
};

using os_agent_id_t = uint32_t;

struct os_agent_snapshot_entry_t
{
  /* Agent Id assigned by the driver.  */
  os_agent_id_t os_agent_id;
  /* Public name of the "device".  */
  std::string name;
  /* BDF - Identifies the device location in the overall system.  */
  uint32_t location_id{ 0 };
  /* Number of FCompute cores.  */
  uint32_t simd_count{ 0 };
  /* Number of Shader Banks or Shader Engines.  */
  uint32_t shader_engine_count{ 0 };
  /* Number of SIMD arrays per engine.  */
  uint32_t simd_arrays_per_engine{ 0 };
  /* Number of Compute Units (CU) per SIMD array.  */
  uint32_t cu_per_simd_array{ 0 };
  /* Number of SIMD representing a Compute Unit (CU).  */
  uint32_t simd_per_cu{ 0 };
  /* Maximum number of launched waves per SIMD.  */
  uint32_t max_waves_per_simd{ 0 };
  /* PCI vendor id.  */
  uint32_t vendor_id{ 0 };
  /* PCI device id.  */
  uint32_t device_id{ 0 };
  /* ucode version.  */
  uint32_t fw_version{ 0 };
  /* ucode version required to support debugging.  */
  uint32_t fw_version_required{ 0 };
  /* ELF e_machine.  */
  elf_amdgpu_machine_t e_machine{ EF_AMDGPU_MACH_NONE };
};

using os_queue_id_t = uint32_t;
using os_watch_id_t = uint32_t;

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

enum class os_wave_launch_trap_override_t : uint32_t
{
  OR = KFD_DBG_TRAP_OVERRIDE_OR, /* OR mask with HSA_DBG_TRAP_MASK.  */
  REPLACE = KFD_DBG_TRAP_OVERRIDE_REPLACE, /* Replace mask.  */
};

enum class os_wave_launch_trap_mask_t : uint32_t
{
  NONE = 0,
  FP_INVALID = KFD_DBG_TRAP_MASK_FP_INVALID,
  FP_INPUT_DENORMAL = KFD_DBG_TRAP_MASK_FP_INPUT_DENORMAL,
  FP_DIVIDE_BY_ZERO = KFD_DBG_TRAP_MASK_FP_DIVIDE_BY_ZERO,
  FP_OVERFLOW = KFD_DBG_TRAP_MASK_FP_OVERFLOW,
  FP_UNDERFLOW = KFD_DBG_TRAP_MASK_FP_UNDERFLOW,
  FP_INEXACT = KFD_DBG_TRAP_MASK_FP_INEXACT,
  INT_DIVIDE_BY_ZERO = KFD_DBG_TRAP_MASK_INT_DIVIDE_BY_ZERO,
  ADDRESS_WATCH = KFD_DBG_TRAP_MASK_DBG_ADDRESS_WATCH,
};
template <> struct is_flag<os_wave_launch_trap_mask_t> : std::true_type
{
};

enum class os_watch_mode_t : uint32_t
{
  READ = 0,    /* Read operations only.  */
  NONREAD = 1, /* Write or atomic operations only.  */
  ATOMIC = 2,  /* Atomic operations only.  */
  ALL = 3,     /* Read, write or atomic operations.  */
};

using os_queue_snapshot_entry_t = kfd_queue_snapshot_entry;

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
protected:
  os_driver_t (std::optional<amd_dbgapi_os_pid_t> os_pid);

public:
  virtual ~os_driver_t () = default;

  static std::unique_ptr<const os_driver_t>
  create (std::optional<amd_dbgapi_os_pid_t> os_pid);

  virtual bool is_valid () const { return m_os_pid.has_value (); }

  virtual amd_dbgapi_status_t check_version () const = 0;

  virtual amd_dbgapi_status_t
  agent_snapshot (os_agent_snapshot_entry_t *snapshots, size_t snapshot_count,
                  size_t *agent_count) const = 0;

  virtual amd_dbgapi_status_t
  enable_debug_trap (os_agent_id_t os_agent_id,
                     file_desc_t *poll_fd) const = 0;
  virtual amd_dbgapi_status_t
  disable_debug_trap (os_agent_id_t os_agent_id) const = 0;

  virtual amd_dbgapi_status_t
  query_debug_event (os_agent_id_t os_agent_id, os_queue_id_t *os_queue_id,
                     os_queue_status_t *os_queue_status) const = 0;

  virtual size_t suspend_queues (os_queue_id_t *queues,
                                 size_t queue_count) const = 0;
  virtual size_t resume_queues (os_queue_id_t *queues,
                                size_t queue_count) const = 0;

  virtual amd_dbgapi_status_t
  queue_snapshot (os_queue_snapshot_entry_t *snapshots, size_t snapshot_count,
                  size_t *queue_count) const = 0;

  virtual amd_dbgapi_status_t set_address_watch (
      os_agent_id_t os_agent_id, amd_dbgapi_global_address_t address,
      amd_dbgapi_global_address_t mask, os_watch_mode_t os_watch_mode,
      os_watch_id_t *os_watch_id) const = 0;

  virtual amd_dbgapi_status_t
  clear_address_watch (os_agent_id_t os_agent_id,
                       os_watch_id_t os_watch_id) const = 0;

  virtual amd_dbgapi_status_t
  set_wave_launch_mode (os_agent_id_t os_agent_id,
                        os_wave_launch_mode_t mode) const = 0;

  virtual amd_dbgapi_status_t set_wave_launch_trap_override (
      os_agent_id_t os_agent_id, os_wave_launch_trap_override_t override,
      os_wave_launch_trap_mask_t trap_mask,
      os_wave_launch_trap_mask_t requested_bits,
      os_wave_launch_trap_mask_t *previous_mask,
      os_wave_launch_trap_mask_t *supported_mask) const = 0;

  virtual amd_dbgapi_status_t
  xfer_global_memory_partial (amd_dbgapi_global_address_t address, void *read,
                              const void *write, size_t *size) const = 0;

protected:
  std::optional<amd_dbgapi_os_pid_t> const m_os_pid;
};

template <> std::string to_string (os_wave_launch_mode_t mode);
template <> std::string to_string (os_queue_status_t queue_status);

} /* namespace amd::dbgapi */

#endif /* _AMD_DBGAPI_OS_DRIVER_H */
