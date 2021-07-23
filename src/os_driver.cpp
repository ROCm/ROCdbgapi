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

#include "os_driver.h"
#include "debug.h"
#include "logging.h"
#include "utils.h"

#include <algorithm>
#include <fstream>
#include <optional>
#include <type_traits>
#include <unordered_set>
#include <utility>
#include <vector>

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

using namespace std::string_literals;

namespace amd::dbgapi
{

/* OS driver base class that only implements memory accesses on Linux.  */

class linux_driver_t : public os_driver_t
{
private:
  std::optional<file_desc_t> m_proc_mem_fd{};

public:
  linux_driver_t (amd_dbgapi_os_process_id_t os_pid);
  ~linux_driver_t () override;

  bool is_valid () const override { return m_proc_mem_fd.has_value (); }

  amd_dbgapi_status_t
  xfer_global_memory_partial (amd_dbgapi_global_address_t address, void *read,
                              const void *write, size_t *size) const override;
};

linux_driver_t::linux_driver_t (amd_dbgapi_os_process_id_t os_pid)
  : os_driver_t (os_pid)
{
  /* Open the /proc/pid/mem file for this process.  */
  std::string filename = string_printf ("/proc/%d/mem", os_pid);
  int fd = ::open (filename.c_str (), O_RDWR | O_LARGEFILE | O_CLOEXEC, 0);
  if (fd == -1)
    {
      warning ("Could not open `%s': %s", filename.c_str (), strerror (errno));
      return;
    }

  m_proc_mem_fd.emplace (fd);

  /* See is_valid() for information about how failing to open /proc/pid/mem
     is handled.  */
}

linux_driver_t::~linux_driver_t ()
{
  if (m_proc_mem_fd)
    ::close (*m_proc_mem_fd);
}

amd_dbgapi_status_t
linux_driver_t::xfer_global_memory_partial (
  amd_dbgapi_global_address_t address, void *read, const void *write,
  size_t *size) const
{
  dbgapi_assert (!read != !write && "either read or write buffer");
  dbgapi_assert (is_valid ());

  ssize_t ret = read ? pread (*m_proc_mem_fd, read, *size, address)
                     : pwrite (*m_proc_mem_fd, write, *size, address);

  if (ret < 0 && errno != EIO && errno != EINVAL)
    warning ("linux_driver_t::xfer_memory failed: %s", strerror (errno));

  if (ret < 0 || (ret == 0 && *size != 0))
    return AMD_DBGAPI_STATUS_ERROR_MEMORY_ACCESS;
  else
    {
      *size = ret;
      return AMD_DBGAPI_STATUS_SUCCESS;
    }
}

/* OS Driver implementation for the Linux ROCm stack using KFD.  */

class kfd_driver_t : public linux_driver_t
{
private:
  struct gfxip_lookup_table_t
  {
    const char *gpu_name;           /* Device name reported by KFD.  */
    elf_amdgpu_machine_t e_machine; /* ELF e_machine.  */
    uint16_t fw_version;            /* Minimum required firmware version.  */
  };

  static constexpr gfxip_lookup_table_t s_gfxip_lookup_table[]
    = { { "vega10", EF_AMDGPU_MACH_AMDGCN_GFX900, 455 + 32768 },
        { "vega20", EF_AMDGPU_MACH_AMDGCN_GFX906, 455 },
        { "arcturus", EF_AMDGPU_MACH_AMDGCN_GFX908, 57 },
        { "aldebaran", EF_AMDGPU_MACH_AMDGCN_GFX90A, 47 },
        { "navi10", EF_AMDGPU_MACH_AMDGCN_GFX1010, 143 },
        { "navi12", EF_AMDGPU_MACH_AMDGCN_GFX1011, 143 },
        { "navi14", EF_AMDGPU_MACH_AMDGCN_GFX1012, 143 },
        { "sienna_cichlid", EF_AMDGPU_MACH_AMDGCN_GFX1030, 88 },
        { "navy_flounder", EF_AMDGPU_MACH_AMDGCN_GFX1031, 88 } };

  static size_t s_kfd_open_count;
  static std::optional<file_desc_t> s_kfd_fd;

  static void open_kfd ();
  static void close_kfd ();

  bool m_is_debug_enabled{ false };

  int kfd_dbg_trap_ioctl (uint32_t action,
                          kfd_ioctl_dbg_trap_args *args) const;

public:
  kfd_driver_t (amd_dbgapi_os_process_id_t os_pid) : linux_driver_t (os_pid)
  {
    open_kfd ();

    /* See is_valid() for information about how failing to open /dev/kfd is
       handled.  */
  }

  ~kfd_driver_t () override { close_kfd (); }

  /* Disable copies.  */
  kfd_driver_t (const kfd_driver_t &) = delete;
  kfd_driver_t &operator= (const kfd_driver_t &) = delete;

  bool is_valid () const override
  {
    return linux_driver_t::is_valid () && s_kfd_fd.has_value ();
  }

  amd_dbgapi_status_t check_version () const override;

  amd_dbgapi_status_t
  agent_snapshot (os_agent_snapshot_entry_t *snapshots, size_t snapshot_count,
                  size_t *agent_count,
                  os_exception_mask_t exceptions_cleared) const override;

  amd_dbgapi_status_t enable_debug (os_exception_mask_t exceptions_reported,
                                    file_desc_t notifier) override;
  amd_dbgapi_status_t disable_debug () override;
  bool is_debug_enabled () const override { return m_is_debug_enabled; }

  amd_dbgapi_status_t
  send_exceptions (os_exception_mask_t exceptions,
                   os_source_id_t os_source_id) const override;

  amd_dbgapi_status_t
  query_debug_event (os_exception_mask_t *exceptions_present,
                     os_source_id_t *os_source_id,
                     os_exception_mask_t exceptions_cleared) override;

  amd_dbgapi_status_t
  query_exception_info (os_exception_code_t exception,
                        os_source_id_t os_source_id, void *exception_info,
                        size_t exception_info_size,
                        bool clear_exception) const override;

  size_t
  suspend_queues (os_queue_id_t *queues, size_t queue_count,
                  os_exception_mask_t exceptions_cleared) const override;
  size_t resume_queues (os_queue_id_t *queues,
                        size_t queue_count) const override;

  amd_dbgapi_status_t
  queue_snapshot (os_queue_snapshot_entry_t *snapshots, size_t snapshot_count,
                  size_t *queue_count,
                  os_exception_mask_t exceptions_cleared) const override;

  amd_dbgapi_status_t set_address_watch (
    amd_dbgapi_global_address_t address, amd_dbgapi_global_address_t mask,
    os_watch_mode_t os_watch_mode, os_watch_id_t *os_watch_id) const override;

  amd_dbgapi_status_t
  clear_address_watch (os_watch_id_t os_watch_id) const override;

  amd_dbgapi_status_t
  set_wave_launch_mode (os_wave_launch_mode_t mode) const override;

  amd_dbgapi_status_t set_wave_launch_trap_override (
    os_wave_launch_trap_override_t override,
    os_wave_launch_trap_mask_t trap_mask,
    os_wave_launch_trap_mask_t requested_bits,
    os_wave_launch_trap_mask_t *previous_mask,
    os_wave_launch_trap_mask_t *supported_mask) const override;

  amd_dbgapi_status_t set_precise_memory (bool enabled) const override;
};

size_t kfd_driver_t::s_kfd_open_count{ 0 };
std::optional<file_desc_t> kfd_driver_t::s_kfd_fd;

/* Open the KFD device. The file descriptor is reference counted, multiple
   calls to open_kfd are allowed, as long as the same number of open_kfd and
   close_kfd are called.  The last call to close_kfd closes the device.  */

void
kfd_driver_t::open_kfd ()
{
  if (!s_kfd_open_count++)
    {
      int fd = ::open ("/dev/kfd", O_RDWR | O_CLOEXEC);
      if (fd == -1)
        {
          dbgapi_log (AMD_DBGAPI_LOG_LEVEL_INFO,
                      "Could not open the KFD device: %s", strerror (errno));
          return;
        }

      dbgapi_assert (!s_kfd_fd && "kfd_fd is already open");
      s_kfd_fd.emplace (fd);
    }
}

void
kfd_driver_t::close_kfd ()
{
  dbgapi_assert (s_kfd_open_count > 0 && "kfd_fd is already closed");

  /* The last call to close_kfd closes the KFD device.  */
  if (!--s_kfd_open_count)
    {
      if (s_kfd_fd && ::close (*s_kfd_fd))
        error ("failed to close s_kfd_fd");

      s_kfd_fd.reset ();
    }
}

int
kfd_driver_t::kfd_dbg_trap_ioctl (uint32_t action,
                                  kfd_ioctl_dbg_trap_args *args) const
{
  dbgapi_assert (is_valid ());

  args->pid = m_os_pid;
  args->op = action;

  int ret = ::ioctl (*s_kfd_fd, AMDKFD_IOC_DBG_TRAP, args);
  if (ret < 0 && errno == ESRCH)
    {
      /* TODO: Should we tear down the process now, so that any operation
         executed after this point returns an error?  */
      return -ESRCH;
    }

  return ret < 0 ? -errno : ret;
}

amd_dbgapi_status_t
kfd_driver_t::check_version () const
{
  dbgapi_assert (is_valid ());

  /* Check that the KFD major == IOCTL major, and KFD minor >= IOCTL minor.  */
  kfd_ioctl_get_version_args get_version_args{};
  if (::ioctl (*s_kfd_fd, AMDKFD_IOC_GET_VERSION, &get_version_args)
      || get_version_args.major_version != KFD_IOCTL_MAJOR_VERSION
      || get_version_args.minor_version < KFD_IOCTL_MINOR_VERSION)
    {
      warning (
        "AMD GPU driver version %d.%d does not match %d.%d+ requirement",
        get_version_args.major_version, get_version_args.minor_version,
        KFD_IOCTL_MAJOR_VERSION, KFD_IOCTL_MINOR_VERSION);
      return AMD_DBGAPI_STATUS_ERROR_RESTRICTION;
    }

  /* KFD_IOC_DBG_TRAP_GET_VERSION (#7)
     data1: [out] major version
     data2: [out] minor version */

  kfd_ioctl_dbg_trap_args dbg_trap_args{};
  dbg_trap_args.pid = static_cast<uint32_t> (getpid ());
  dbg_trap_args.op = KFD_IOC_DBG_TRAP_GET_VERSION;

  if (::ioctl (*s_kfd_fd, AMDKFD_IOC_DBG_TRAP, &dbg_trap_args))
    error ("KFD_IOC_DBG_TRAP_GET_VERSION failed");

  int32_t major = dbg_trap_args.data1;
  int32_t minor = dbg_trap_args.data2;

  /* Check that the KFD dbg trap major == IOCTL dbg trap major,
     and KFD dbg trap minor >= IOCTL dbg trap minor.  */
  if (major != KFD_IOCTL_DBG_MAJOR_VERSION
      || minor < KFD_IOCTL_DBG_MINOR_VERSION)
    {
      warning ("AMD GPU driver's debug support version %d.%d does "
               "not match %d.%d+ requirement",
               major, minor, KFD_IOCTL_DBG_MAJOR_VERSION,
               KFD_IOCTL_DBG_MINOR_VERSION);
      return AMD_DBGAPI_STATUS_ERROR_RESTRICTION;
    }

  dbgapi_log (AMD_DBGAPI_LOG_LEVEL_INFO,
              "using AMD GPU driver's debug support version %d.%d", major,
              minor);

  return AMD_DBGAPI_STATUS_SUCCESS;
}

amd_dbgapi_status_t
kfd_driver_t::agent_snapshot (os_agent_snapshot_entry_t *snapshots,
                              size_t snapshot_count, size_t *agent_count,
                              os_exception_mask_t exceptions_cleared) const
{
  dbgapi_assert (snapshots && agent_count && "must not be null");
  dbgapi_assert (snapshot_count <= std::numeric_limits<uint32_t>::max ()
                 && "invalid argument");

  if (!is_debug_enabled ())
    {
      *agent_count = 0;
      return AMD_DBGAPI_STATUS_SUCCESS;
    }

  std::vector<kfd_dbg_device_info_entry> kfd_device_infos (snapshot_count);

  /* KFD_IOC_DBG_TRAP_DEVICE_SNAPSHOT (#12):
     exception_mask: [in] exceptions to clear on snapshot
     data1: [in/out] number of device snapshots
     ptr:   [in] user buffer  */

  kfd_ioctl_dbg_trap_args args{};
  args.exception_mask = static_cast<uint64_t> (exceptions_cleared);
  args.data1 = static_cast<uint32_t> (kfd_device_infos.size ());
  args.ptr = reinterpret_cast<uint64_t> (kfd_device_infos.data ());

  int err = kfd_dbg_trap_ioctl (KFD_IOC_DBG_TRAP_DEVICE_SNAPSHOT, &args);
  if (err == -ESRCH)
    return AMD_DBGAPI_STATUS_ERROR_PROCESS_EXITED;
  else if (err < 0)
    return AMD_DBGAPI_STATUS_ERROR;

  /* KFD writes up to snapshot_count device snapshots, but returns the number
     of devices in the process so that we can check if we have allocated enough
     memory to hold all the snapshots.  */
  *agent_count = args.data1;

  /* Fill in the missing information from the sysfs topology.  */

  static const std::string sysfs_nodes_path (
    "/sys/devices/virtual/kfd/kfd/topology/nodes/");

  std::unique_ptr<DIR, void (*) (DIR *)> dirp (
    opendir (sysfs_nodes_path.c_str ()), [] (DIR *d) { closedir (d); });

  if (!dirp && errno == ENOENT)
    {
      /* The sysfs is not mounted, maybe KFD driver is not installed, or we
         don't have any GPUs installed.  */
      return AMD_DBGAPI_STATUS_SUCCESS;
    }
  else if (!dirp)
    error ("Could not opendir `%s': %s", sysfs_nodes_path.c_str (),
           strerror (errno));

  std::unordered_set<os_agent_id_t> processed_os_agent_ids;
  size_t agents_found = 0;
  struct dirent *dir;

  while ((dir = readdir (dirp.get ())) != nullptr)
    {
      if (dir->d_name == "."s || dir->d_name == ".."s)
        continue;

      std::string node_path (sysfs_nodes_path + dir->d_name);

      /* Retrieve the GPU ID.  */
      std::ifstream gpu_id_ifs (node_path + "/gpu_id");
      if (!gpu_id_ifs.is_open ())
        error ("Could not open %s/gpu_id", node_path.c_str ());

      os_agent_id_t gpu_id;
      gpu_id_ifs >> gpu_id;

      if (gpu_id_ifs.fail () || !gpu_id)
        /* Skip inaccessible nodes and CPU nodes.  */
        continue;

      auto it
        = std::find_if (kfd_device_infos.begin (), kfd_device_infos.end (),
                        [&] (const auto &device_info)
                        { return device_info.gpu_id == gpu_id; });
      if (it == kfd_device_infos.end ())
        /* This sysfs topology node was not reported by the device snapshot
           ioctl, skip it.  */
        continue;

      if (!processed_os_agent_ids.emplace (gpu_id).second)
        error ("More than one os_agent_id %d reported in the sysfs topology",
               gpu_id);

      auto &agent_info = snapshots[agents_found++];
      agent_info.os_agent_id = gpu_id;

      /* Retrieve the GPU name.  */

      std::ifstream gpu_name_ifs (node_path + "/name");
      if (!gpu_name_ifs.is_open ())
        error ("Could not open %s/name", node_path.c_str ());

      gpu_name_ifs >> agent_info.name;
      if (agent_info.name.empty ())
        error ("os_agent_id %d: asic family name not present in the sysfs.",
               agent_info.os_agent_id);

      /* Fill in the apertures for this agent.  */

      agent_info.local_address_space_aperture = it->lds_base;
      agent_info.private_address_space_aperture = it->scratch_base;

      /* Retrieve the GPU node properties.  */

      std::ifstream props_ifs (node_path + "/properties");
      if (!props_ifs.is_open ())
        error ("Could not open %s/properties", node_path.c_str ());

      std::string prop_name;
      uint64_t prop_value;
      while (props_ifs >> prop_name >> prop_value)
        {
          if (prop_name == "location_id")
            agent_info.location_id = static_cast<uint16_t> (prop_value);
          else if (prop_name == "simd_count")
            agent_info.simd_count = static_cast<size_t> (prop_value);
          else if (prop_name == "max_waves_per_simd")
            agent_info.max_waves_per_simd = static_cast<size_t> (prop_value);
          else if (prop_name == "vendor_id")
            agent_info.vendor_id = static_cast<uint32_t> (prop_value);
          else if (prop_name == "device_id")
            agent_info.device_id = static_cast<uint32_t> (prop_value);
          else if (prop_name == "fw_version")
            agent_info.fw_version = static_cast<uint16_t> (prop_value);
        }

      decltype (&s_gfxip_lookup_table[0]) gfxip_info = nullptr;
      constexpr size_t num_elem
        = sizeof (s_gfxip_lookup_table) / sizeof (s_gfxip_lookup_table[0]);

      for (size_t i = 0; i < num_elem; ++i)
        if (agent_info.name == s_gfxip_lookup_table[i].gpu_name)
          {
            gfxip_info = &s_gfxip_lookup_table[i];
            break;
          }

      if (gfxip_info)
        {
          agent_info.e_machine = gfxip_info->e_machine;
          agent_info.fw_version_required = gfxip_info->fw_version;
        }
    }

  /* Make sure we filled the information for all snapshot entries returned by
     the kfd device snapshot ioctl.  */
  if (agents_found != std::min (snapshot_count, *agent_count))
    error ("not all agents found in the sysfs topology");

  return AMD_DBGAPI_STATUS_SUCCESS;
}

amd_dbgapi_status_t
kfd_driver_t::enable_debug (os_exception_mask_t exceptions_reported,
                            file_desc_t notifier)
{
  dbgapi_assert (!is_debug_enabled () && "debug is already enabled");

  /* KFD_IOC_DBG_TRAP_ENABLE (#0):
     exception_mask: [in] exceptions to be reported to the debugger
     data1: [in] 0=disable, 1=enable
     data2: [out] poll_fd
     data3: [out] ttmps setup enabled (0=disabled, 1=enabled)  */

  kfd_ioctl_dbg_trap_args args{};
  args.data1 = 1; /* enable  */
  args.data2 = notifier;
  args.exception_mask = static_cast<uint64_t> (exceptions_reported);

  int err = kfd_dbg_trap_ioctl (KFD_IOC_DBG_TRAP_ENABLE, &args);
  if (err == -ESRCH)
    return AMD_DBGAPI_STATUS_ERROR_PROCESS_EXITED;
  else if (err == -EBUSY)
    {
      /* An agent does not support multi-process debugging and already has
         debug trap enabled by another process.  */
      warning ("At least one agent is busy (debugging may be enabled by "
               "another process)");
      return AMD_DBGAPI_STATUS_ERROR_RESTRICTION;
    }
  else if (err < 0)
    return AMD_DBGAPI_STATUS_ERROR;

  m_is_debug_enabled = true;
  return AMD_DBGAPI_STATUS_SUCCESS;
}

amd_dbgapi_status_t
kfd_driver_t::disable_debug ()
{
  if (!is_debug_enabled ())
    return AMD_DBGAPI_STATUS_SUCCESS;

  /* KFD_IOC_DBG_TRAP_ENABLE (#0):
     data1: [in] 0=disable, 1=enable  */

  kfd_ioctl_dbg_trap_args args{};
  args.data1 = 0; /* disable  */

  int err = kfd_dbg_trap_ioctl (KFD_IOC_DBG_TRAP_ENABLE, &args);
  if (err == -ESRCH)
    return AMD_DBGAPI_STATUS_ERROR_PROCESS_EXITED;
  else if (err < 0)
    return AMD_DBGAPI_STATUS_ERROR;

  m_is_debug_enabled = false;
  return AMD_DBGAPI_STATUS_SUCCESS;
}

amd_dbgapi_status_t
kfd_driver_t::send_exceptions (os_exception_mask_t exceptions,
                               os_source_id_t os_source_id) const
{
  dbgapi_assert (is_debug_enabled () && "debug is not enabled");

  /* KFD_IOC_DBG_TRAP_SEND_RUNTIME_EVENT (#14):
     data1: [in] source id (queue or device)
     data2: [in] event to send  */

  kfd_ioctl_dbg_trap_args args{};
  args.exception_mask = static_cast<uint64_t> (exceptions);
  args.data1 = os_source_id.raw;

  int err = kfd_dbg_trap_ioctl (KFD_IOC_DBG_TRAP_SEND_RUNTIME_EVENT, &args);
  if (err == -ESRCH)
    return AMD_DBGAPI_STATUS_ERROR_PROCESS_EXITED;
  else if (err < 0)
    return AMD_DBGAPI_STATUS_ERROR;

  return AMD_DBGAPI_STATUS_SUCCESS;
}

amd_dbgapi_status_t
kfd_driver_t::query_debug_event (os_exception_mask_t *exceptions_present,
                                 os_source_id_t *os_source_id,
                                 os_exception_mask_t exceptions_cleared)
{
  dbgapi_assert (exceptions_present && os_source_id && "must not be null");

  if (!is_debug_enabled ())
    {
      *exceptions_present = os_exception_mask_t::none;
      os_source_id->raw = 0;
      return AMD_DBGAPI_STATUS_SUCCESS;
    }

  /* KFD_IOC_DBG_TRAP_QUERY_DEBUG_EVENT (#5):

     exception_mask: [in/out] exception to clear on query, and report
     data1: [out] source id  */

  kfd_ioctl_dbg_trap_args args{};
  args.exception_mask = static_cast<uint64_t> (exceptions_cleared);

  int err = kfd_dbg_trap_ioctl (KFD_IOC_DBG_TRAP_QUERY_DEBUG_EVENT, &args);
  if (err == -ESRCH)
    return AMD_DBGAPI_STATUS_ERROR_PROCESS_EXITED;
  else if (err == -EAGAIN)
    {
      /* There are no more events.  */
      *exceptions_present = os_exception_mask_t::none;
      os_source_id->raw = 0;
      return AMD_DBGAPI_STATUS_SUCCESS;
    }
  else if (err < 0)
    return AMD_DBGAPI_STATUS_ERROR;

  *exceptions_present = static_cast<os_exception_mask_t> (args.exception_mask);
  os_source_id->raw = args.data1;

  return AMD_DBGAPI_STATUS_SUCCESS;
}

amd_dbgapi_status_t
kfd_driver_t::query_exception_info (os_exception_code_t exception,
                                    os_source_id_t os_source_id,
                                    void *exception_info,
                                    size_t exception_info_size,
                                    bool clear_exception) const
{
  dbgapi_assert (is_debug_enabled () && "debug is not enabled");

  /* KFD_IOC_DBG_TRAP_QUERY_EXCEPTION_INFO (#11)
     ptr: [in] exception info pointer to copy to
     data1: [in] source_id
     data2: [in] exception_code
     data3: [in] clear_exception (1 == true, 0 == false)
     data4: [in/out] exception info data size  */

  kfd_ioctl_dbg_trap_args args{};
  args.ptr = reinterpret_cast<uintptr_t> (exception_info);
  args.data1 = os_source_id.raw;
  args.data2 = static_cast<uint32_t> (exception);
  args.data3 = clear_exception ? 1 : 0;
  args.data4 = exception_info_size;

  int err = kfd_dbg_trap_ioctl (KFD_IOC_DBG_TRAP_QUERY_EXCEPTION_INFO, &args);
  if (err == -ESRCH)
    return AMD_DBGAPI_STATUS_ERROR_PROCESS_EXITED;
  else if (err < 0)
    return AMD_DBGAPI_STATUS_ERROR;

  return exception_info_size > args.data4
           ? AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT_COMPATIBILITY
           : AMD_DBGAPI_STATUS_SUCCESS;
}

size_t
kfd_driver_t::suspend_queues (os_queue_id_t *queues, size_t queue_count,
                              os_exception_mask_t exceptions_cleared) const
{
  dbgapi_assert (queue_count <= std::numeric_limits<uint32_t>::max ());

  /* KFD_IOC_DBG_TRAP_NODE_SUSPEND (#3):
     exception_mask: [in] exceptions to clear on suspend
     data1: [in] number of queues
     data2: [in] grace period
     ptr:   [in/out] queue ids  */

  kfd_ioctl_dbg_trap_args args{};
  args.exception_mask = static_cast<uint64_t> (exceptions_cleared);
  args.data1 = static_cast<uint32_t> (queue_count);
  args.ptr = reinterpret_cast<uint64_t> (queues);

  int ret = kfd_dbg_trap_ioctl (KFD_IOC_DBG_TRAP_NODE_SUSPEND, &args);
  if (ret == -ESRCH)
    return AMD_DBGAPI_STATUS_ERROR_PROCESS_EXITED;
  else if (ret < 0)
    return AMD_DBGAPI_STATUS_ERROR;

  return ret;
}

size_t
kfd_driver_t::resume_queues (os_queue_id_t *queues, size_t queue_count) const
{
  dbgapi_assert (queue_count <= std::numeric_limits<uint32_t>::max ());

  /* KFD_IOC_DBG_TRAP_NODE_RESUME (#4):
     data1: [in] number of queues
     ptr:   [in/out] queue ids  */

  kfd_ioctl_dbg_trap_args args{};
  args.data1 = static_cast<uint32_t> (queue_count);
  args.ptr = reinterpret_cast<uint64_t> (queues);

  int ret = kfd_dbg_trap_ioctl (KFD_IOC_DBG_TRAP_NODE_RESUME, &args);
  if (ret == -ESRCH)
    return AMD_DBGAPI_STATUS_ERROR_PROCESS_EXITED;
  else if (ret < 0)
    return AMD_DBGAPI_STATUS_ERROR;

  return ret;
}

amd_dbgapi_status_t
kfd_driver_t::queue_snapshot (os_queue_snapshot_entry_t *snapshots,
                              size_t snapshot_count, size_t *queue_count,
                              os_exception_mask_t exceptions_cleared) const
{
  dbgapi_assert (snapshots && queue_count && "must not be null");
  dbgapi_assert (snapshot_count <= std::numeric_limits<uint32_t>::max ()
                 && "invalid argument");

  if (!is_debug_enabled ())
    {
      *queue_count = 0;
      return AMD_DBGAPI_STATUS_SUCCESS;
    }

  /* KFD_IOC_DBG_TRAP_GET_QUEUE_SNAPSHOT (#6):
     exception_mask: [in] exceptions to clear on snapshot
     data1: [in/out] number of queues snapshots
     ptr:   [in] user buffer  */

  kfd_ioctl_dbg_trap_args args{};
  args.exception_mask = static_cast<uint64_t> (exceptions_cleared);
  args.data1 = static_cast<uint32_t> (snapshot_count);
  args.ptr = reinterpret_cast<uint64_t> (snapshots);

  int err = kfd_dbg_trap_ioctl (KFD_IOC_DBG_TRAP_GET_QUEUE_SNAPSHOT, &args);
  if (err == -ESRCH)
    return AMD_DBGAPI_STATUS_ERROR_PROCESS_EXITED;
  else if (err < 0)
    return AMD_DBGAPI_STATUS_ERROR;

  /* KFD writes up to snapshot_count queue snapshots, but returns the number of
     queues in the process so that we can check if we have allocated enough
     memory to hold all the snapshots.  */
  *queue_count = args.data1;

  return AMD_DBGAPI_STATUS_SUCCESS;
}

amd_dbgapi_status_t
kfd_driver_t::set_address_watch (amd_dbgapi_global_address_t address,
                                 amd_dbgapi_global_address_t mask,
                                 os_watch_mode_t os_watch_mode,
                                 os_watch_id_t *os_watch_id) const
{
  dbgapi_assert (os_watch_id && "must not be null");

  /* KFD_IOC_DBG_TRAP_SET_ADDRESS_WATCH (#9)
     ptr:   [in] watch address
     data1: [out] watch ID
     data2: [in] watch_mode: 0=read, 1=nonread, 2=atomic, 3=all
     data3: [in] watch address mask  */

  kfd_ioctl_dbg_trap_args args{};
  args.ptr = address;
  args.data2 = static_cast<std::underlying_type_t<decltype (os_watch_mode)>> (
    os_watch_mode);
  args.data3 = mask;

  int err = kfd_dbg_trap_ioctl (KFD_IOC_DBG_TRAP_SET_ADDRESS_WATCH, &args);
  if (err == -ENOMEM)
    return AMD_DBGAPI_STATUS_ERROR_NO_WATCHPOINT_AVAILABLE;
  else if (err == -ESRCH)
    return AMD_DBGAPI_STATUS_ERROR_PROCESS_EXITED;
  else if (err < 0)
    error ("failed to set address watch: %s", strerror (err));

  *os_watch_id = args.data1;

  return AMD_DBGAPI_STATUS_SUCCESS;
}

amd_dbgapi_status_t
kfd_driver_t::clear_address_watch (os_watch_id_t os_watch_id) const
{
  /* KFD_IOC_DBG_TRAP_CLEAR_ADDRESS_WATCH (#8)
     data1: [in] watch ID  */

  kfd_ioctl_dbg_trap_args args{};
  args.data1 = os_watch_id;

  int err = kfd_dbg_trap_ioctl (KFD_IOC_DBG_TRAP_CLEAR_ADDRESS_WATCH, &args);
  if (err == -ESRCH)
    return AMD_DBGAPI_STATUS_ERROR_PROCESS_EXITED;
  else if (err < 0)
    return AMD_DBGAPI_STATUS_ERROR;

  return AMD_DBGAPI_STATUS_SUCCESS;
}

amd_dbgapi_status_t
kfd_driver_t::set_wave_launch_mode (os_wave_launch_mode_t mode) const
{
  /* KFD_IOC_DBG_TRAP_SET_WAVE_LAUNCH_MODE (#2)
     data1: mode (0=normal, 1=halt, 2=kill, 3=single-step, 4=disable)  */

  kfd_ioctl_dbg_trap_args args{};
  args.data1 = static_cast<std::underlying_type_t<decltype (mode)>> (mode);

  int err = kfd_dbg_trap_ioctl (KFD_IOC_DBG_TRAP_SET_WAVE_LAUNCH_MODE, &args);
  if (err == -ESRCH)
    return AMD_DBGAPI_STATUS_ERROR_PROCESS_EXITED;
  else if (err < 0)
    return AMD_DBGAPI_STATUS_ERROR;

  return AMD_DBGAPI_STATUS_SUCCESS;
}

amd_dbgapi_status_t
kfd_driver_t::set_wave_launch_trap_override (
  os_wave_launch_trap_override_t override, os_wave_launch_trap_mask_t value,
  os_wave_launch_trap_mask_t mask, os_wave_launch_trap_mask_t *previous_value,
  os_wave_launch_trap_mask_t *supported_mask) const
{
  /* KFD_IOC_DBG_TRAP_SET_WAVE_LAUNCH_OVERRIDE (#1)
     data1: [in] override mode (see enum kfd_dbg_trap_override_mode)
     data2: [in/out] trap mask (see enum kfd_dbg_trap_mask)
     data3: [in] requested mask, [out] supported mask  */

  kfd_ioctl_dbg_trap_args args{};
  args.data1
    = static_cast<std::underlying_type_t<decltype (override)>> (override);
  args.data2 = static_cast<std::underlying_type_t<decltype (value)>> (value);
  args.data3 = static_cast<std::underlying_type_t<decltype (mask)>> (mask);

  int err
    = kfd_dbg_trap_ioctl (KFD_IOC_DBG_TRAP_SET_WAVE_LAUNCH_OVERRIDE, &args);
  if (err == -ESRCH)
    return AMD_DBGAPI_STATUS_ERROR_PROCESS_EXITED;
  else if (err == -EPERM || err == -EACCES)
    return AMD_DBGAPI_STATUS_ERROR_NOT_SUPPORTED;
  else if (err < 0)
    return AMD_DBGAPI_STATUS_ERROR;

  if (previous_value)
    *previous_value = static_cast<os_wave_launch_trap_mask_t> (args.data2);
  if (supported_mask)
    *supported_mask = static_cast<os_wave_launch_trap_mask_t> (args.data3);

  return AMD_DBGAPI_STATUS_SUCCESS;
}

amd_dbgapi_status_t
kfd_driver_t::set_precise_memory (bool enabled) const
{
  /* KFD_IOC_DBG_TRAP_SET_PRECISE_MEM_OPS (#10)
     data1: 0=disable, 1=enable  */

  kfd_ioctl_dbg_trap_args args{};
  args.data1 = enabled ? 1 : 0;

  int err = kfd_dbg_trap_ioctl (KFD_IOC_DBG_TRAP_SET_PRECISE_MEM_OPS, &args);
  if (err == -ESRCH)
    return AMD_DBGAPI_STATUS_ERROR_PROCESS_EXITED;
  else if (err < 0)
    return AMD_DBGAPI_STATUS_ERROR;

  return AMD_DBGAPI_STATUS_SUCCESS;
}

class no_agents_driver_t : public linux_driver_t
{
public:
  no_agents_driver_t (amd_dbgapi_os_process_id_t os_pid)
    : linux_driver_t (os_pid)
  {
  }
  ~no_agents_driver_t () override = default;

  amd_dbgapi_status_t check_version () const override
  {
    return AMD_DBGAPI_STATUS_SUCCESS;
  }

  amd_dbgapi_status_t
  agent_snapshot (os_agent_snapshot_entry_t * /* snapshots  */,
                  size_t /* snapshot_count  */, size_t *agent_count,
                  os_exception_mask_t /* exceptions_cleared  */) const override
  {
    *agent_count = 0;
    return AMD_DBGAPI_STATUS_SUCCESS;
  }

  amd_dbgapi_status_t
    enable_debug (os_exception_mask_t /* exceptions_reported  */,
                  file_desc_t /* notifier  */) override
  {
    return AMD_DBGAPI_STATUS_ERROR_RESTRICTION;
  }

  amd_dbgapi_status_t disable_debug () override
  {
    /* Debug is never enabled.  */
    return AMD_DBGAPI_STATUS_ERROR;
  }

  bool is_debug_enabled () const override { return false; }

  amd_dbgapi_status_t
    send_exceptions (os_exception_mask_t /* exceptions  */,
                     os_source_id_t /* os_source_id  */) const override
  {
    return AMD_DBGAPI_STATUS_ERROR;
  }

  amd_dbgapi_status_t
  query_debug_event (os_exception_mask_t *exceptions_present,
                     os_source_id_t *os_source_id,
                     os_exception_mask_t /* exceptions_cleared  */) override
  {
    *exceptions_present = os_exception_mask_t::none;
    os_source_id->raw = 0;
    return AMD_DBGAPI_STATUS_SUCCESS;
  }

  amd_dbgapi_status_t query_exception_info (
    os_exception_code_t /* exception  */, os_source_id_t /* os_source_id  */,
    void * /* exception_info  */, size_t /* exception_info_size  */,
    bool /* clear_exception  */) const override
  {
    return AMD_DBGAPI_STATUS_ERROR;
  }

  size_t
  suspend_queues (os_queue_id_t * /* queues  */, size_t queue_count,
                  os_exception_mask_t /* exceptions_cleared  */) const override
  {
    if (queue_count > 0)
      error ("should not call this, null_driver does not have any queues");
    return AMD_DBGAPI_STATUS_SUCCESS;
  }

  size_t resume_queues (os_queue_id_t * /* queues  */,
                        size_t queue_count) const override
  {
    if (queue_count > 0)
      error ("should not call this, null_driver does not have any queues");
    return AMD_DBGAPI_STATUS_SUCCESS;
  }

  amd_dbgapi_status_t
  queue_snapshot (os_queue_snapshot_entry_t * /* snapshots  */,
                  size_t /* snapshot_count  */, size_t *queue_count,
                  os_exception_mask_t /* exceptions_cleared  */) const override
  {
    *queue_count = 0;
    return AMD_DBGAPI_STATUS_SUCCESS;
  }

  amd_dbgapi_status_t
  set_address_watch (amd_dbgapi_global_address_t /* address  */,
                     amd_dbgapi_global_address_t /* mask  */,
                     os_watch_mode_t /* os_watch_mode  */,
                     os_watch_id_t * /* os_watch_id  */) const override
  {
    return AMD_DBGAPI_STATUS_ERROR_NOT_SUPPORTED;
  }

  amd_dbgapi_status_t
    clear_address_watch (os_watch_id_t /* os_watch_id  */) const override
  {
    error ("should not call this, null_driver does not support watchpoints");
  }

  amd_dbgapi_status_t
    set_wave_launch_mode (os_wave_launch_mode_t /* mode  */) const override
  {
    return AMD_DBGAPI_STATUS_ERROR;
  }

  amd_dbgapi_status_t set_wave_launch_trap_override (
    os_wave_launch_trap_override_t /* override  */,
    os_wave_launch_trap_mask_t /* trap_mask  */,
    os_wave_launch_trap_mask_t /* requested_bits  */,
    os_wave_launch_trap_mask_t * /* previous_mask  */,
    os_wave_launch_trap_mask_t * /* supported_mask  */) const override
  {
    return AMD_DBGAPI_STATUS_ERROR;
  }

  amd_dbgapi_status_t set_precise_memory (bool /* enabled  */) const override
  {
    return AMD_DBGAPI_STATUS_ERROR_NOT_SUPPORTED;
  }
};

std::unique_ptr<os_driver_t>
os_driver_t::create_driver (amd_dbgapi_os_process_id_t os_pid)
{
  std::unique_ptr<os_driver_t> os_driver{ new kfd_driver_t (os_pid) };
  if (os_driver->is_valid ())
    return os_driver;

  /* If we failed to create a kfd_driver_t (kfd is not installed?), then revert
     to a null_driver.  */
  return std::make_unique<no_agents_driver_t> (os_pid);
}

template <>
std::string
to_string (os_wave_launch_mode_t mode)
{
  switch (mode)
    {
    case os_wave_launch_mode_t::normal:
      return "WAVE_LAUNCH_MODE_NORMAL";
    case os_wave_launch_mode_t::halt:
      return "WAVE_LAUNCH_MODE_HALT";
    case os_wave_launch_mode_t::kill:
      return "WAVE_LAUNCH_MODE_KILL";
    case os_wave_launch_mode_t::single_step:
      return "WAVE_LAUNCH_MODE_SINGLE_STEP";
    case os_wave_launch_mode_t::disable:
      return "WAVE_LAUNCH_MODE_DISABLE";
    }
  return to_string (
    make_hex (static_cast<std::underlying_type_t<decltype (mode)>> (mode)));
}

namespace
{

inline std::string
one_os_exception_to_string (os_exception_mask_t exception_mask)
{
  dbgapi_assert (!(exception_mask & (exception_mask - 1)) && "only 1 bit");

  switch (exception_mask)
    {
    case os_exception_mask_t::none:
      return "NONE";
    case os_exception_mask_t::queue_abort:
      return "QUEUE_ABORT";
    case os_exception_mask_t::queue_trap:
      return "QUEUE_TRAP";
    case os_exception_mask_t::queue_math_error:
      return "QUEUE_MATH_ERROR";
    case os_exception_mask_t::queue_illegal_instruction:
      return "QUEUE_ILLEGAL_INSTRUCTION";
    case os_exception_mask_t::queue_memory_violation:
      return "QUEUE_MEMORY_VIOLATION";
    case os_exception_mask_t::queue_aperture_violation:
      return "QUEUE_APERTURE_VIOLATION";
    case os_exception_mask_t::queue_packet_dispatch_dim_invalid:
      return "QUEUE_PACKET_DISPATCH_DIM_INVALID";
    case os_exception_mask_t::queue_packet_dispatch_group_segment_size_invalid:
      return "QUEUE_PACKET_DISPATCH_GROUP_SEGMENT_SIZE_INVALID";
    case os_exception_mask_t::queue_packet_dispatch_code_invalid:
      return "QUEUE_PACKET_DISPATCH_CODE_INVALID";
    case os_exception_mask_t::queue_packet_unsupported:
      return "QUEUE_PACKET_UNSUPPORTED";
    case os_exception_mask_t::queue_packet_dispatch_work_group_size_invalid:
      return "QUEUE_PACKET_DISPATCH_WORK_GROUP_SIZE_INVALID";
    case os_exception_mask_t::queue_packet_dispatch_register_invalid:
      return "QUEUE_PACKET_DISPATCH_REGISTER_INVALID";
    case os_exception_mask_t::queue_packet_dispatch_vendor_unsupported:
      return "QUEUE_PACKET_DISPATCH_VENDOR_UNSUPPORTED";
    case os_exception_mask_t::queue_preemption_error:
      return "QUEUE_PREEMPTION_ERROR";
    case os_exception_mask_t::queue_new:
      return "QUEUE_NEW";
    case os_exception_mask_t::device_queue_delete:
      return "DEVICE_QUEUE_DELETE";
    case os_exception_mask_t::device_memory_violation:
      return "DEVICE_MEMORY_VIOLATION";
    case os_exception_mask_t::device_ras_error:
      return "DEVICE_RAS_ERROR";
    case os_exception_mask_t::device_fatal_halt:
      return "DEVICE_FATAL_HALT";
    case os_exception_mask_t::device_new:
      return "DEVICE_NEW";
    case os_exception_mask_t::process_runtime_enable:
      return "PROCESS_RUNTIME_ENABLE";
    case os_exception_mask_t::process_runtime_disable:
      return "PROCESS_RUNTIME_DISABLE";
    case os_exception_mask_t::process_device_remove:
      return "PROCESS_REMOVE";
    }

  return to_string (
    make_hex (static_cast<std::underlying_type_t<decltype (exception_mask)>> (
      exception_mask)));
}

} /* namespace */

template <>
std::string
to_string (os_exception_mask_t exception_mask)
{
  std::string str;

  if (exception_mask == os_exception_mask_t::none)
    return one_os_exception_to_string (exception_mask);

  while (exception_mask != os_exception_mask_t::none)
    {
      os_exception_mask_t one_bit
        = exception_mask ^ (exception_mask & (exception_mask - 1));

      if (!str.empty ())
        str += " | ";
      str += one_os_exception_to_string (one_bit);

      exception_mask ^= one_bit;
    }

  return str;
}

template <>
std::string
to_string (os_exception_code_t exception_code)
{
  return one_os_exception_to_string (os_exception_mask (exception_code));
}

} /* namespace amd::dbgapi */
