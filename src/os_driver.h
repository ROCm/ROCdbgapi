/* Copyright (c) 2021 Advanced Micro Devices, Inc.

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

#ifndef AMD_DBGAPI_OS_DRIVER_H
#define AMD_DBGAPI_OS_DRIVER_H 1

#include "amd-dbgapi.h"
#include "linux/kfd_ioctl.h"
#include "logging.h"
#include "utils.h"

#include <cstddef>
#include <cstdint>
#include <memory>
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
  EF_AMDGPU_MACH_AMDGCN_GFX90A = 0x03f,
  EF_AMDGPU_MACH_AMDGCN_GFX1010 = 0x033,
  EF_AMDGPU_MACH_AMDGCN_GFX1011 = 0x034,
  EF_AMDGPU_MACH_AMDGCN_GFX1012 = 0x035,
  EF_AMDGPU_MACH_AMDGCN_GFX1030 = 0x036,
  EF_AMDGPU_MACH_AMDGCN_GFX1031 = 0x037,
};

using os_agent_id_t = uint32_t;

struct os_agent_snapshot_entry_t
{
  /* Agent Id assigned by the driver.  */
  os_agent_id_t os_agent_id{};
  /* Public name of the "device".  */
  std::string name{};
  /* BDF - Identifies the device location in the overall system.  */
  uint16_t location_id{ 0 };
  /* Number of FCompute cores.  */
  size_t simd_count{ 0 };
  /* Maximum number of launched waves per SIMD.  */
  size_t max_waves_per_simd{ 0 };
  /* PCI vendor id.  */
  uint32_t vendor_id{ 0 };
  /* PCI device id.  */
  uint32_t device_id{ 0 };
  /* ucode version.  */
  uint32_t fw_version{ 0 };
  /* ucode version required to support debugging.  */
  uint32_t fw_version_required{ 0 };
  /* local/shared address space aperture base.  */
  amd_dbgapi_global_address_t local_address_space_aperture{ 0 };
  /* private/scratch address space aperture base.  */
  amd_dbgapi_global_address_t private_address_space_aperture{ 0 };
  /* ELF e_machine.  */
  elf_amdgpu_machine_t e_machine{ EF_AMDGPU_MACH_NONE };
};

using os_queue_id_t = uint32_t;
using os_watch_id_t = uint32_t;

enum class os_queue_type_t : uint32_t
{
  compute = KFD_IOC_QUEUE_TYPE_COMPUTE,
  sdma = KFD_IOC_QUEUE_TYPE_SDMA,
  compute_aql = KFD_IOC_QUEUE_TYPE_COMPUTE_AQL,
  sdma_xgmi = KFD_IOC_QUEUE_TYPE_SDMA_XGMI
};

union os_source_id_t
{
  os_agent_id_t agent;
  os_queue_id_t queue;
  uint32_t raw;
};
static_assert (sizeof (os_source_id_t) == sizeof (uint32_t));

enum class os_exception_mask_t : uint64_t
{
  none = EC_NONE,

  /* per queue exceptions  */
  queue_new = KFD_EC_MASK (EC_QUEUE_NEW),
  trap_handler = KFD_EC_MASK (EC_TRAP_HANDLER),
  cp_packet_error = KFD_EC_MASK (EC_CP_PACKET_ERROR),
  hws_preemption_error = KFD_EC_MASK (EC_HWS_PREEMPTION_ERROR),

  /* per device exceptions  */
  memory_violation = KFD_EC_MASK (EC_MEMORY_VIOLATION),
  ras_error = KFD_EC_MASK (EC_RAS_ERROR),
  fatal_halt = KFD_EC_MASK (EC_FATAL_HALT),
  queue_delete = KFD_EC_MASK (EC_QUEUE_DELETE),
  gpu_add = KFD_EC_MASK (EC_GPU_ADD),

  /* per process exceptions  */
  runtime_enable = KFD_EC_MASK (EC_RUNTIME_ENABLE),
  runtime_disable = KFD_EC_MASK (EC_RUNTIME_DISABLE),
  gpu_remove = KFD_EC_MASK (EC_GPU_REMOVE),
};
template <> struct is_flag<os_exception_mask_t> : std::true_type
{
};

static constexpr os_exception_mask_t os_queue_exceptions_mask
  = static_cast<os_exception_mask_t> (KFD_EC_MASK_QUEUE);

static_assert (os_queue_exceptions_mask
               == (os_exception_mask_t::queue_new
                   | os_exception_mask_t::trap_handler
                   | os_exception_mask_t::cp_packet_error
                   | os_exception_mask_t::hws_preemption_error));

static constexpr os_exception_mask_t os_agent_exceptions_mask
  = static_cast<os_exception_mask_t> (KFD_EC_MASK_DEVICE);

static_assert (os_agent_exceptions_mask
               == (os_exception_mask_t::memory_violation
                   | os_exception_mask_t::ras_error
                   | os_exception_mask_t::fatal_halt
                   | os_exception_mask_t::queue_delete
                   | os_exception_mask_t::gpu_add));

static constexpr os_exception_mask_t os_process_exceptions_mask
  = static_cast<os_exception_mask_t> (KFD_EC_MASK_PROCESS);

static_assert (os_process_exceptions_mask
               == (os_exception_mask_t::runtime_enable
                   | os_exception_mask_t::runtime_disable
                   | os_exception_mask_t::gpu_remove));

enum class os_wave_launch_trap_override_t : uint32_t
{
  apply = KFD_DBG_TRAP_OVERRIDE_OR, /* OR mask with HSA_DBG_TRAP_MASK.  */
  replace = KFD_DBG_TRAP_OVERRIDE_REPLACE, /* Replace mask.  */
};

enum class os_wave_launch_trap_mask_t : uint32_t
{
  none = 0,
  fp_invalid = KFD_DBG_TRAP_MASK_FP_INVALID,
  fp_input_denormal = KFD_DBG_TRAP_MASK_FP_INPUT_DENORMAL,
  fp_divide_by_zero = KFD_DBG_TRAP_MASK_FP_DIVIDE_BY_ZERO,
  fp_overflow = KFD_DBG_TRAP_MASK_FP_OVERFLOW,
  fp_underflow = KFD_DBG_TRAP_MASK_FP_UNDERFLOW,
  fp_inexact = KFD_DBG_TRAP_MASK_FP_INEXACT,
  int_divide_by_zero = KFD_DBG_TRAP_MASK_INT_DIVIDE_BY_ZERO,
  address_watch = KFD_DBG_TRAP_MASK_DBG_ADDRESS_WATCH,
};
template <> struct is_flag<os_wave_launch_trap_mask_t> : std::true_type
{
};

template <> std::string to_string (os_wave_launch_trap_mask_t value);

enum class os_watch_mode_t : uint32_t
{
  read = 0,    /* Read operations only.  */
  nonread = 1, /* Write or atomic operations only.  */
  atomic = 2,  /* Atomic operations only.  */
  all = 3,     /* Read, write or atomic operations.  */
};

using os_queue_snapshot_entry_t = kfd_queue_snapshot_entry;

inline os_exception_mask_t
os_queue_exception_status (os_queue_snapshot_entry_t entry)
{
  return static_cast<os_exception_mask_t> (entry.exception_status);
}

inline os_queue_type_t
os_queue_type (os_queue_snapshot_entry_t entry)
{
  return static_cast<os_queue_type_t> (entry.queue_type);
}

constexpr os_queue_id_t os_invalid_queueid = KFD_INVALID_QUEUEID;
constexpr os_queue_id_t os_queue_error_mask = KFD_DBG_QUEUE_ERROR_MASK;
constexpr os_queue_id_t os_queue_invalid_mask = KFD_DBG_QUEUE_INVALID_MASK;

enum class os_wave_launch_mode_t : uint32_t
{
  normal = 0,      /* Waves launch normally.  */
  halt = 1,        /* Waves launch in halted mode.  */
  kill = 2,        /* Waves terminate before executing any instructions.  */
  single_step = 3, /* Waves launch in single-step mode.  */
  disable = 4,     /* Disable launching any new waves.  */
};

class os_driver_t
{
protected:
  amd_dbgapi_os_process_id_t const m_os_pid;

  os_driver_t (amd_dbgapi_os_process_id_t os_pid) : m_os_pid (os_pid) {}

public:
  virtual ~os_driver_t () = default;

  static std::unique_ptr<os_driver_t>
  create_driver (amd_dbgapi_os_process_id_t os_pid,
                 std::function<void ()> pending_event_notifier);

  virtual bool is_valid () const = 0;

  virtual amd_dbgapi_status_t check_version () const = 0;

  virtual amd_dbgapi_status_t
  agent_snapshot (os_agent_snapshot_entry_t *snapshots, size_t snapshot_count,
                  size_t *agent_count) const = 0;

  virtual amd_dbgapi_status_t
  enable_debug (os_exception_mask_t exceptions_reported)
    = 0;
  virtual amd_dbgapi_status_t disable_debug () = 0;
  virtual bool is_debug_enabled () const = 0;

  virtual amd_dbgapi_status_t
  query_debug_event (os_exception_mask_t *exceptions_present,
                     os_source_id_t *os_source_id,
                     os_exception_mask_t exceptions_cleared)
    = 0;

  virtual size_t
  suspend_queues (os_queue_id_t *queues, size_t queue_count,
                  os_exception_mask_t exceptions_cleared) const = 0;
  virtual size_t resume_queues (os_queue_id_t *queues,
                                size_t queue_count) const = 0;

  virtual amd_dbgapi_status_t
  queue_snapshot (os_queue_snapshot_entry_t *snapshots, size_t snapshot_count,
                  size_t *queue_count,
                  os_exception_mask_t exceptions_cleared) const = 0;

  virtual amd_dbgapi_status_t set_address_watch (
    amd_dbgapi_global_address_t address, amd_dbgapi_global_address_t mask,
    os_watch_mode_t os_watch_mode, os_watch_id_t *os_watch_id) const = 0;

  virtual amd_dbgapi_status_t
  clear_address_watch (os_watch_id_t os_watch_id) const = 0;

  virtual amd_dbgapi_status_t
  set_wave_launch_mode (os_wave_launch_mode_t mode) const = 0;

  virtual amd_dbgapi_status_t set_wave_launch_trap_override (
    os_wave_launch_trap_override_t override, os_wave_launch_trap_mask_t value,
    os_wave_launch_trap_mask_t mask,
    os_wave_launch_trap_mask_t *previous_value = nullptr,
    os_wave_launch_trap_mask_t *supported_mask = nullptr) const = 0;

  virtual amd_dbgapi_status_t set_precise_memory (bool enabled) const = 0;

  virtual amd_dbgapi_status_t
  xfer_global_memory_partial (amd_dbgapi_global_address_t address, void *read,
                              const void *write, size_t *size) const = 0;
};

template <> std::string to_string (os_wave_launch_mode_t mode);
template <> std::string to_string (os_exception_mask_t exception_mask);

} /* namespace amd::dbgapi */

#endif /* AMD_DBGAPI_OS_DRIVER_H */
