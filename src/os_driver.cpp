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

#include "os_driver.h"
#include "debug.h"
#include "logging.h"

#include <algorithm>
#include <fstream>
#include <limits>
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

namespace
{

/* OS driver base class that only implements memory accesses on Linux.  */

class linux_driver_t : public os_driver_t
{
private:
  std::optional<file_desc_t> m_proc_mem_fd;

public:
  linux_driver_t (std::optional<amd_dbgapi_os_process_id_t> os_pid);
  virtual ~linux_driver_t ();

  virtual bool is_valid () const override
  {
    return os_driver_t::is_valid () && m_proc_mem_fd;
  }

  virtual amd_dbgapi_status_t
  xfer_global_memory_partial (amd_dbgapi_global_address_t address, void *read,
                              const void *write, size_t *size) const override;
};

linux_driver_t::linux_driver_t (
    std::optional<amd_dbgapi_os_process_id_t> os_pid)
    : os_driver_t (std::move (os_pid))
{
  if (!m_os_pid)
    return;

  /* Open the /proc/pid/mem file for this process.  */
  std::string filename = string_printf ("/proc/%d/mem", *m_os_pid);
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
    warning ("process_t::xfer_memory failed: %s", strerror (errno));

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
      = { { "vega10", EF_AMDGPU_MACH_AMDGCN_GFX900, 33206 },
          { "raven", EF_AMDGPU_MACH_AMDGCN_GFX902, 0 },
          { "vega12", EF_AMDGPU_MACH_AMDGCN_GFX904, 0 },
          { "vega20", EF_AMDGPU_MACH_AMDGCN_GFX906, 438 },
          { "arcturus", EF_AMDGPU_MACH_AMDGCN_GFX908, 48 },
          { "navi10", EF_AMDGPU_MACH_AMDGCN_GFX1010, 0 },
          { "navi12", EF_AMDGPU_MACH_AMDGCN_GFX1011, 0 },
          { "navi14", EF_AMDGPU_MACH_AMDGCN_GFX1012, 0 },
          { "sienna_cichlid", EF_AMDGPU_MACH_AMDGCN_GFX1030, 0 },
          { "navy_flounder", EF_AMDGPU_MACH_AMDGCN_GFX1031, 0 } };

  static size_t s_kfd_open_count;
  static std::optional<file_desc_t> s_kfd_fd;

  static void open_kfd ();
  static void close_kfd ();

  int kfd_dbg_trap_ioctl (uint32_t action,
                          kfd_ioctl_dbg_trap_args *args) const;

public:
  kfd_driver_t (std::optional<amd_dbgapi_os_process_id_t> os_pid)
      : linux_driver_t (std::move (os_pid))
  {
    open_kfd ();

    /* See is_valid() for information about how failing to open /dev/kfd is
       handled.  */
  }

  virtual ~kfd_driver_t () { close_kfd (); }

  virtual bool is_valid () const override
  {
    return linux_driver_t::is_valid () && s_kfd_fd;
  }

  virtual amd_dbgapi_status_t check_version () const override;

  virtual amd_dbgapi_status_t
  agent_snapshot (os_agent_snapshot_entry_t *snapshots, size_t snapshot_count,
                  size_t *agent_count) const override;

  virtual amd_dbgapi_status_t
  enable_debug_trap (os_agent_id_t os_agent_id,
                     file_desc_t *poll_fd) const override;
  virtual amd_dbgapi_status_t
  disable_debug_trap (os_agent_id_t os_agent_id) const override;

  virtual amd_dbgapi_status_t
  query_debug_event (os_agent_id_t os_agent_id, os_queue_id_t *os_queue_id,
                     os_queue_status_t *os_queue_status) const override;

  virtual size_t suspend_queues (os_queue_id_t *queues,
                                 size_t queue_count) const override;
  virtual size_t resume_queues (os_queue_id_t *queues,
                                size_t queue_count) const override;

  virtual amd_dbgapi_status_t
  queue_snapshot (os_queue_snapshot_entry_t *snapshots, size_t snapshot_count,
                  size_t *queue_count) const override;

  virtual amd_dbgapi_status_t set_address_watch (
      os_agent_id_t os_agent_id, amd_dbgapi_global_address_t address,
      amd_dbgapi_global_address_t mask, os_watch_mode_t os_watch_mode,
      os_watch_id_t *os_watch_id) const override;

  virtual amd_dbgapi_status_t
  clear_address_watch (os_agent_id_t os_agent_id,
                       os_watch_id_t os_watch_id) const override;

  virtual amd_dbgapi_status_t
  set_wave_launch_mode (os_agent_id_t os_agent_id,
                        os_wave_launch_mode_t mode) const override;

  virtual amd_dbgapi_status_t set_wave_launch_trap_override (
      os_agent_id_t os_agent_id, os_wave_launch_trap_override_t override,
      os_wave_launch_trap_mask_t trap_mask,
      os_wave_launch_trap_mask_t requested_bits,
      os_wave_launch_trap_mask_t *previous_mask,
      os_wave_launch_trap_mask_t *supported_mask) const override;
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

  args->pid = *m_os_pid;
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

  /* KFD_IOC_DBG_TRAP_GET_VERSION (#8)
     data1: [out] major version
     data2: [out] minor version */

  kfd_ioctl_dbg_trap_args dbg_trap_args{};
  dbg_trap_args.pid = static_cast<uint32_t> (getpid ());
  dbg_trap_args.op = KFD_IOC_DBG_TRAP_GET_VERSION;

  if (::ioctl (*s_kfd_fd, AMDKFD_IOC_DBG_TRAP, &dbg_trap_args))
    error ("KFD_IOC_DBG_TRAP_GET_VERSION failed");

  uint32_t major = dbg_trap_args.data1;
  uint32_t minor = dbg_trap_args.data2;

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
                              size_t snapshot_count, size_t *agent_count) const
{
  dbgapi_assert (snapshots && agent_count && "must not be null");

  /* Discover the GPU nodes from the sysfs topology.  */

  static const std::string sysfs_nodes_path (
      "/sys/devices/virtual/kfd/kfd/topology/nodes/");

  std::unique_ptr<DIR, void (*) (DIR *)> dirp (
      opendir (sysfs_nodes_path.c_str ()),
      [] (DIR *dirp) { closedir (dirp); });

  std::unordered_set<os_agent_id_t> processed_os_agent_ids;
  *agent_count = 0;

  if (!dirp && errno == ENOENT)
    {
      /* The sysfs is not mounted, maybe KFD driver is not installed, or we
         don't have any GPUs installed.  */
      return AMD_DBGAPI_STATUS_SUCCESS;
    }
  else if (!dirp)
    error ("Could not opendir `%s': %s", sysfs_nodes_path.c_str (),
           strerror (errno));

  size_t node_count{ 0 };

  struct dirent *dir;
  while ((dir = readdir (dirp.get ())))
    if (dir->d_name != "."s && dir->d_name != ".."s)
      ++node_count;

  std::vector<kfd_process_device_apertures> node_aperture_infos (node_count);

  kfd_ioctl_get_process_apertures_new_args args{};
  args.kfd_process_device_apertures_ptr
      = reinterpret_cast<uintptr_t> (node_aperture_infos.data ());
  args.num_of_nodes = node_aperture_infos.size ();

  /* FIXME: There is no guarantee that the the debugger's apertures are the
     same as the inferior's, so we really should be using a dbgtrap ioctl.  */
  if (::ioctl (*s_kfd_fd, AMDKFD_IOC_GET_PROCESS_APERTURES_NEW, &args))
    error ("AMDKFD_IOC_GET_PROCESS_APERTURES_NEW failed");

  /* KFD returns the actual number of entries copied to the array. */
  dbgapi_assert (args.num_of_nodes <= node_aperture_infos.size ());
  node_aperture_infos.resize (args.num_of_nodes);

  rewinddir (dirp.get ());
  while ((dir = readdir (dirp.get ())))
    {
      if (dir->d_name == "."s || dir->d_name == ".."s)
        continue;

      os_agent_snapshot_entry_t agent_info{};
      std::string node_path (sysfs_nodes_path + dir->d_name);

      /* Retrieve the GPU ID.  */

      std::ifstream gpu_id_ifs (node_path + "/gpu_id");
      if (!gpu_id_ifs.is_open ())
        error ("Could not open %s/gpu_id", node_path.c_str ());

      gpu_id_ifs >> agent_info.os_agent_id;

      if (gpu_id_ifs.fail () || !agent_info.os_agent_id)
        /* Skip inaccessible nodes and CPU nodes.  */
        continue;

      if (!processed_os_agent_ids.emplace (agent_info.os_agent_id).second)
        error ("More than one os_agent_id %d reported in the sysfs topology",
               agent_info.os_agent_id);

      /* Retrieve the GPU name.  */

      std::ifstream gpu_name_ifs (node_path + "/name");
      if (!gpu_name_ifs.is_open ())
        error ("Could not open %s/name", node_path.c_str ());

      gpu_name_ifs >> agent_info.name;
      if (agent_info.name.empty ())
        error ("os_agent_id %d: asic family name not present in the sysfs.",
               agent_info.os_agent_id);

      /* Fill in the apertures for this agent.  */

      auto it = std::find_if (
          node_aperture_infos.begin (), node_aperture_infos.end (),
          [&] (const auto &node_aperture_info) {
            return node_aperture_info.gpu_id == agent_info.os_agent_id;
          });

      if (it == node_aperture_infos.end ())
        error ("Could not get apertures for gpu_id %d",
               agent_info.os_agent_id);

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

      if (snapshot_count)
        {
          *snapshots++ = agent_info;
          --snapshot_count;
        }
      ++*agent_count;
    }

  return AMD_DBGAPI_STATUS_SUCCESS;
}

amd_dbgapi_status_t
kfd_driver_t::enable_debug_trap (os_agent_id_t os_agent_id,
                                 file_desc_t *poll_fd) const
{
  dbgapi_assert (poll_fd && "must not be null");

  /* KFD_IOC_DBG_TRAP_ENABLE (#0):
     data1: [in] enable/disable (1/0)
     data3: [out] poll_fd  */

  kfd_ioctl_dbg_trap_args args{};
  args.gpu_id = os_agent_id;
  args.data1 = 1; /* enable  */

  int err = kfd_dbg_trap_ioctl (KFD_IOC_DBG_TRAP_ENABLE, &args);
  if (err == -ESRCH)
    return AMD_DBGAPI_STATUS_ERROR_PROCESS_EXITED;
  else if (err == -EBUSY)
    {
      /* The agent does not support multi-process debugging and already has
         debug trap enabled by another process.  */
      warning ("os_agent_id %d: device is busy (debugging may be enabled by "
               "another process)",
               os_agent_id);
      return AMD_DBGAPI_STATUS_ERROR_RESTRICTION;
    }
  else if (err < 0)
    return AMD_DBGAPI_STATUS_ERROR;

  *poll_fd = args.data3;
  return AMD_DBGAPI_STATUS_SUCCESS;
}

amd_dbgapi_status_t
kfd_driver_t::disable_debug_trap (os_agent_id_t os_agent_id) const
{
  /* KFD_IOC_DBG_TRAP_ENABLE (#0):
     data1: [in] enable/disable (1/0)  */

  kfd_ioctl_dbg_trap_args args{};
  args.gpu_id = os_agent_id;
  args.data1 = 0; /* disable  */

  int err = kfd_dbg_trap_ioctl (KFD_IOC_DBG_TRAP_ENABLE, &args);
  if (err == -ESRCH)
    return AMD_DBGAPI_STATUS_ERROR_PROCESS_EXITED;
  else if (err < 0)
    return AMD_DBGAPI_STATUS_ERROR;

  return AMD_DBGAPI_STATUS_SUCCESS;
}

amd_dbgapi_status_t
kfd_driver_t::query_debug_event (os_agent_id_t os_agent_id,
                                 os_queue_id_t *os_queue_id,
                                 os_queue_status_t *os_queue_status) const
{
  dbgapi_assert (os_queue_id && os_queue_status && "must not be null");

  /* KFD_IOC_DBG_TRAP_QUERY_DEBUG_EVENT (#6):
     data1: [in/out] queue id
     data2: [in] flags
     data3: [out] new_queue[3:3], suspended[2:2], event_type [1:0]  */

  kfd_ioctl_dbg_trap_args args{};
  args.gpu_id = os_agent_id;
  args.data1 = KFD_INVALID_QUEUEID;
  args.data2 = KFD_DBG_EV_FLAG_CLEAR_STATUS;

  int err = kfd_dbg_trap_ioctl (KFD_IOC_DBG_TRAP_QUERY_DEBUG_EVENT, &args);
  if (err == -ESRCH)
    return AMD_DBGAPI_STATUS_ERROR_PROCESS_EXITED;
  else if (err == -EAGAIN)
    {
      /* There are no more events.  */
      *os_queue_id = os_invalid_queueid;
      *os_queue_status = {};
      return AMD_DBGAPI_STATUS_SUCCESS;
    }
  else if (err < 0)
    return AMD_DBGAPI_STATUS_ERROR;

  *os_queue_id = args.data1;
  *os_queue_status = static_cast<os_queue_status_t> (args.data3);

  return AMD_DBGAPI_STATUS_SUCCESS;
}

size_t
kfd_driver_t::suspend_queues (os_queue_id_t *queues, size_t queue_count) const
{
  dbgapi_assert (queue_count <= std::numeric_limits<uint32_t>::max ());

  /* KFD_IOC_DBG_TRAP_NODE_SUSPEND (#4):
     data1: [in] flags
     data2: [in] number of queues
     data3: [in] grace period
     ptr:   [in] queue ids  */

  kfd_ioctl_dbg_trap_args args{};
  args.data1 = KFD_DBG_EV_FLAG_CLEAR_STATUS;
  args.data2 = static_cast<uint32_t> (queue_count);
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

  /* KFD_IOC_DBG_TRAP_NODE_RESUME (#5):
     data1: [in] flags
     data2: [in] number of queues
     ptr:   [in] queue ids  */

  kfd_ioctl_dbg_trap_args args{};
  args.data1 = 0;
  args.data2 = static_cast<uint32_t> (queue_count);
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
                              size_t snapshot_count, size_t *queue_count) const
{
  dbgapi_assert (snapshots && queue_count && "must not be null");
  dbgapi_assert (snapshot_count <= std::numeric_limits<uint32_t>::max ()
                 && "invalid argument");

  /* KFD_IOC_DBG_TRAP_GET_QUEUE_SNAPSHOT (#7):
     data1: [in] flags
     data2: [in/out] number of queues snapshots
     ptr:   [in] user buffer  */

  kfd_ioctl_dbg_trap_args args{};
  args.data1 = 0;
  args.data2 = static_cast<uint32_t> (snapshot_count);
  args.ptr = reinterpret_cast<uint64_t> (snapshots);

  int err = kfd_dbg_trap_ioctl (KFD_IOC_DBG_TRAP_GET_QUEUE_SNAPSHOT, &args);
  if (err == -ESRCH)
    return AMD_DBGAPI_STATUS_ERROR_PROCESS_EXITED;
  else if (err < 0)
    return AMD_DBGAPI_STATUS_ERROR;

  /* KFD writes up to snapshot_count queue snapshots, but returns the
     number of queues in the process so that we can check if we have
     allocated enough memory to hold all the snapshots.  */
  *queue_count = args.data2;

  return AMD_DBGAPI_STATUS_SUCCESS;
}

amd_dbgapi_status_t
kfd_driver_t::set_address_watch (os_agent_id_t os_agent_id,
                                 amd_dbgapi_global_address_t address,
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
  args.gpu_id = os_agent_id;
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
kfd_driver_t::clear_address_watch (os_agent_id_t os_agent_id,
                                   os_watch_id_t os_watch_id) const
{
  /* KFD_IOC_DBG_TRAP_CLEAR_ADDRESS_WATCH (#8)
     data1: [in] watch ID  */

  kfd_ioctl_dbg_trap_args args{};
  args.gpu_id = os_agent_id;
  args.data1 = os_watch_id;

  int err = kfd_dbg_trap_ioctl (KFD_IOC_DBG_TRAP_CLEAR_ADDRESS_WATCH, &args);
  if (err == -ESRCH)
    return AMD_DBGAPI_STATUS_ERROR_PROCESS_EXITED;
  else if (err < 0)
    return AMD_DBGAPI_STATUS_ERROR;

  return AMD_DBGAPI_STATUS_SUCCESS;
}

amd_dbgapi_status_t
kfd_driver_t::set_wave_launch_mode (os_agent_id_t os_agent_id,
                                    os_wave_launch_mode_t mode) const
{
  /* KFD_IOC_DBG_TRAP_SET_WAVE_LAUNCH_MODE (#2)
     data1: mode (0=normal, 1=halt, 2=kill, 3=single-step, 4=disable)  */

  kfd_ioctl_dbg_trap_args args{};
  args.gpu_id = os_agent_id;
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
    os_agent_id_t os_agent_id, os_wave_launch_trap_override_t override,
    os_wave_launch_trap_mask_t trap_mask,
    os_wave_launch_trap_mask_t requested_bits,
    os_wave_launch_trap_mask_t *previous_mask,
    os_wave_launch_trap_mask_t *supported_mask) const
{
  dbgapi_assert (previous_mask && supported_mask && "must not be null");

  /* KFD_IOC_DBG_TRAP_SET_WAVE_LAUNCH_OVERRIDE (#1)
     data1: [in] override mode (see enum kfd_dbg_trap_override_mode)
     data2: [in/out] trap mask (see enum kfd_dbg_trap_mask)
     data3: [in] requested mask, [out] supported mask  */

  kfd_ioctl_dbg_trap_args args{};
  args.gpu_id = os_agent_id;
  args.data1
      = static_cast<std::underlying_type_t<decltype (override)>> (override);
  args.data2
      = static_cast<std::underlying_type_t<decltype (trap_mask)>> (trap_mask);
  args.data3 = static_cast<std::underlying_type_t<decltype (requested_bits)>> (
      requested_bits);

  int err
      = kfd_dbg_trap_ioctl (KFD_IOC_DBG_TRAP_SET_WAVE_LAUNCH_OVERRIDE, &args);
  if (err == -ESRCH)
    return AMD_DBGAPI_STATUS_ERROR_PROCESS_EXITED;
  else if (err == -EPERM || err == -EACCES)
    return AMD_DBGAPI_STATUS_ERROR_NOT_SUPPORTED;
  else if (err < 0)
    return AMD_DBGAPI_STATUS_ERROR;

  *previous_mask = static_cast<os_wave_launch_trap_mask_t> (args.data2);
  *supported_mask = static_cast<os_wave_launch_trap_mask_t> (args.data3);

  return AMD_DBGAPI_STATUS_SUCCESS;
}

class no_agents_driver_t : public linux_driver_t
{
public:
  no_agents_driver_t (std::optional<amd_dbgapi_os_process_id_t> os_pid)
      : linux_driver_t (std::move (os_pid))
  {
  }
  virtual ~no_agents_driver_t () = default;

  virtual bool is_valid () const override
  {
    return linux_driver_t::is_valid ();
  }

  virtual amd_dbgapi_status_t check_version () const override
  {
    return AMD_DBGAPI_STATUS_SUCCESS;
  }

  virtual amd_dbgapi_status_t
  agent_snapshot (os_agent_snapshot_entry_t *snapshots, size_t snapshot_count,
                  size_t *agent_count) const override
  {
    *agent_count = 0;
    return AMD_DBGAPI_STATUS_SUCCESS;
  }

  virtual amd_dbgapi_status_t
  enable_debug_trap (os_agent_id_t os_agent_id,
                     file_desc_t *poll_fd) const override
  {
    error ("should not call this, null_driver does not have any agents");
  }

  virtual amd_dbgapi_status_t
  disable_debug_trap (os_agent_id_t os_agent_id) const override
  {
    error ("should not call this, null_driver does not have any agents");
  }

  virtual amd_dbgapi_status_t
  query_debug_event (os_agent_id_t os_agent_id, os_queue_id_t *os_queue_id,
                     os_queue_status_t *os_queue_status) const override
  {
    error ("should not call this, null_driver does not have any agents");
  }

  virtual size_t suspend_queues (os_queue_id_t *queues,
                                 size_t queue_count) const override
  {
    error ("should not call this, null_driver does not have any queues");
  }

  virtual size_t resume_queues (os_queue_id_t *queues,
                                size_t queue_count) const override
  {
    error ("should not call this, null_driver does not have any queues");
  }

  virtual amd_dbgapi_status_t
  queue_snapshot (os_queue_snapshot_entry_t *snapshots, size_t snapshot_count,
                  size_t *queue_count) const override
  {
    *queue_count = 0;
    return AMD_DBGAPI_STATUS_SUCCESS;
  }

  virtual amd_dbgapi_status_t set_address_watch (
      os_agent_id_t os_agent_id, amd_dbgapi_global_address_t address,
      amd_dbgapi_global_address_t mask, os_watch_mode_t os_watch_mode,
      os_watch_id_t *os_watch_id) const override
  {
    error ("should not call this, null_driver does not have any agents");
  }

  virtual amd_dbgapi_status_t
  clear_address_watch (os_agent_id_t os_agent_id,
                       os_watch_id_t os_watch_id) const override
  {
    error ("should not call this, null_driver does not have any agents");
  }

  virtual amd_dbgapi_status_t
  set_wave_launch_mode (os_agent_id_t os_agent_id,
                        os_wave_launch_mode_t mode) const override
  {
    error ("should not call this, null_driver does not have any agents");
  }

  virtual amd_dbgapi_status_t set_wave_launch_trap_override (
      os_agent_id_t os_agent_id, os_wave_launch_trap_override_t override,
      os_wave_launch_trap_mask_t trap_mask,
      os_wave_launch_trap_mask_t requested_bits,
      os_wave_launch_trap_mask_t *previous_mask,
      os_wave_launch_trap_mask_t *supported_mask) const override
  {
    error ("should not call this, null_driver does not have any agents");
  }
};

} /* namespace */

os_driver_t::os_driver_t (std::optional<amd_dbgapi_os_process_id_t> os_pid)
    : m_os_pid (std::move (os_pid))
{
}

std::unique_ptr<const os_driver_t>
os_driver_t::create (std::optional<amd_dbgapi_os_process_id_t> os_pid)
{
  std::unique_ptr<const os_driver_t> os_driver{ new kfd_driver_t (os_pid) };

  /* If we failed to create a kfd_driver_t (kfd is not installed?), then revert
     to a null_driver.  */
  if (!os_driver->is_valid ())
    os_driver.reset (new no_agents_driver_t (os_pid));

  return os_driver;
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

template <>
std::string
to_string (os_queue_status_t queue_status)
{
  std::string str
      = (queue_status & os_queue_status_t::trap) != 0
            ? "trap"
            : (!!(queue_status & os_queue_status_t::vmfault) ? "vmfault"
                                                             : "unknown");

  if ((queue_status & os_queue_status_t::new_queue) != 0)
    str += "|new_queue";
  if ((queue_status & os_queue_status_t::suspended) != 0)
    str += "|suspended";

  return str;
}

} /* namespace amd::dbgapi */
