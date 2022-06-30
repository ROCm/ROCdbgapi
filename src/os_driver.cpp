/* Copyright (c) 2021-2023 Advanced Micro Devices, Inc.

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
#include "linux/kfd_sysfs.h"
#include "logging.h"
#include "utils.h"

#include <algorithm>
#include <cinttypes>
#include <fstream>
#include <iomanip>
#include <limits>
#include <optional>
#include <sstream>
#include <type_traits>
#include <utility>
#include <vector>

#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#if defined(WITH_API_TRACING)

#define TRACE_DRIVER_BEGIN(...)                                               \
  TRACE_BEGIN_HELPER (AMD_DBGAPI_LOG_LEVEL_VERBOSE, "driver: ", __VA_ARGS__)

#define TRACE_DRIVER_END(...) TRACE_END_HELPER (__VA_ARGS__)

#else /* !defined (WITH_API_TRACING) */

#define TRACE_DRIVER_BEGIN(...)
#define TRACE_DRIVER_END(...)

#endif /* !defined (WITH_API_TRACING) */

using namespace std::string_literals;

namespace amd::dbgapi
{

/* OS driver class that implements no access that can be used if there is no
   process.  */

class null_driver_t : public os_driver_t
{
public:
  null_driver_t (std::optional<amd_dbgapi_os_process_id_t> os_pid = {})
    : os_driver_t (os_pid)
  {
  }
  ~null_driver_t () override = default;

  /* Disable copies.  */
  null_driver_t (const null_driver_t &) = delete;
  null_driver_t &operator= (const null_driver_t &) = delete;

  bool is_valid () const override { return true; }

  amd_dbgapi_status_t check_version () const override
  {
    return AMD_DBGAPI_STATUS_SUCCESS;
  }

  amd_dbgapi_status_t
  agent_snapshot (os_agent_info_t * /* snapshots  */,
                  size_t /* snapshot_count  */, size_t *agent_count,
                  os_exception_mask_t /* exceptions_cleared  */) const override
  {
    *agent_count = 0;
    return AMD_DBGAPI_STATUS_SUCCESS;
  }

  amd_dbgapi_status_t
  enable_debug (os_exception_mask_t /* exceptions_reported  */,
                file_desc_t /* notifier  */,
                os_runtime_info_t * /* runtime_info  */) override
  {
    return AMD_DBGAPI_STATUS_ERROR_PROCESS_EXITED;
  }

  amd_dbgapi_status_t disable_debug () override
  {
    /* Debug is never enabled.  */
    return AMD_DBGAPI_STATUS_ERROR;
  }

  bool is_debug_enabled () const override { return false; }

  amd_dbgapi_status_t set_exceptions_reported (
    os_exception_mask_t /* exceptions_reported  */) const override
  {
    return AMD_DBGAPI_STATUS_SUCCESS;
  }

  amd_dbgapi_status_t
  send_exceptions (os_exception_mask_t /* exceptions  */,
                   std::optional<os_agent_id_t> /* agent_id  */,
                   std::optional<os_queue_id_t> /* queue_id  */) const override
  {
    return AMD_DBGAPI_STATUS_ERROR;
  }

  amd_dbgapi_status_t
  query_debug_event (os_exception_mask_t *exceptions_present,
                     os_queue_id_t *os_queue_id, os_agent_id_t *os_agent_id,
                     os_exception_mask_t /* exceptions_cleared  */) override
  {
    *exceptions_present = os_exception_mask_t::none;
    *os_queue_id = *os_agent_id = 0;
    return AMD_DBGAPI_STATUS_SUCCESS;
  }

  amd_dbgapi_status_t query_exception_info (
    os_exception_code_t /* exception  */, os_source_id_t /* os_source_id  */,
    void * /* exception_info  */, size_t /* exception_info_size  */,
    bool /* clear_exception  */) const override
  {
    return AMD_DBGAPI_STATUS_ERROR;
  }

  amd_dbgapi_status_t
  suspend_queues (os_queue_id_t * /* queues  */, size_t queue_count,
                  os_exception_mask_t /* exceptions_cleared  */,
                  size_t *suspended_count) const override
  {
    dbgapi_assert (suspended_count != nullptr);

    if (queue_count > 0)
      fatal_error (
        "should not call this, null_driver does not have any queues");

    *suspended_count = 0;
    return AMD_DBGAPI_STATUS_SUCCESS;
  }

  amd_dbgapi_status_t resume_queues (os_queue_id_t * /* queues  */,
                                     size_t queue_count,
                                     size_t *resumed_count) const override
  {
    dbgapi_assert (resumed_count != nullptr);

    if (queue_count > 0)
      fatal_error (
        "should not call this, null_driver does not have any queues");

    *resumed_count = 0;
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
  set_address_watch (os_agent_id_t /* os_agent_id  */,
                     amd_dbgapi_global_address_t /* address  */,
                     amd_dbgapi_global_address_t /* mask  */,
                     os_watch_mode_t /* os_watch_mode  */,
                     os_watch_id_t * /* os_watch_id  */) const override
  {
    return AMD_DBGAPI_STATUS_ERROR_NOT_SUPPORTED;
  }

  amd_dbgapi_status_t
  clear_address_watch (os_agent_id_t /* os_agent_id  */,
                       os_watch_id_t /* os_watch_id  */) const override
  {
    fatal_error (
      "should not call this, null_driver does not support watchpoints");
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

  amd_dbgapi_status_t
  xfer_global_memory_partial (amd_dbgapi_global_address_t /* address  */,
                              void *read, const void *write,
                              size_t * /* size  */) const override
  {
    dbgapi_assert (!read != !write && "either read or write buffer");
    /* Suppress warnings in release builds.  */
    [] (auto &&...) {}(read, write);
    return AMD_DBGAPI_STATUS_ERROR_MEMORY_ACCESS;
  }
};

/* OS driver class that only implements memory accesses on Linux.  */

class linux_driver_t : public null_driver_t
{
private:
  std::optional<file_desc_t> m_proc_mem_fd{};

  mutable size_t m_read_request_count{};
  mutable size_t m_write_request_count{};
  mutable size_t m_bytes_read{};
  mutable size_t m_bytes_written{};

public:
  linux_driver_t (amd_dbgapi_os_process_id_t os_pid);
  ~linux_driver_t () override;

  /* Disable copies.  */
  linux_driver_t (const linux_driver_t &) = delete;
  linux_driver_t &operator= (const linux_driver_t &) = delete;

  bool is_valid () const override { return m_proc_mem_fd.has_value (); }

  amd_dbgapi_status_t
  enable_debug (os_exception_mask_t /* exceptions_reported  */,
                file_desc_t /* notifier  */,
                os_runtime_info_t * /* runtime_info  */) override
  {
    return AMD_DBGAPI_STATUS_ERROR_RESTRICTION;
  }

  amd_dbgapi_status_t
  xfer_global_memory_partial (amd_dbgapi_global_address_t address, void *read,
                              const void *write, size_t *size) const override;

protected:
  static std::string pci_device_name (uint32_t vendor_id, uint32_t device_id);
};

linux_driver_t::linux_driver_t (amd_dbgapi_os_process_id_t os_pid)
  : null_driver_t (os_pid)
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
  log_info ("linux_driver_t statistics (pid %d): "
            "%ld reads (%s), %ld writes (%s)",
            m_os_pid.value (), m_read_request_count,
            utils::human_readable_size (m_bytes_read).c_str (),
            m_write_request_count,
            utils::human_readable_size (m_bytes_written).c_str ());

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

  ++(read != nullptr ? m_read_request_count : m_write_request_count);

  ssize_t ret = read != nullptr
                  ? pread (*m_proc_mem_fd, read, *size, address)
                  : pwrite (*m_proc_mem_fd, write, *size, address);

  if (ret < 0 && errno != EIO && errno != EINVAL)
    warning ("linux_driver_t::xfer_memory failed: %s", strerror (errno));

  if (ret == 0 && *size != 0)
    return AMD_DBGAPI_STATUS_ERROR_PROCESS_EXITED;
  else if (ret < 0)
    return AMD_DBGAPI_STATUS_ERROR_MEMORY_ACCESS;

  (read != nullptr ? m_bytes_read : m_bytes_written) += ret;

  *size = ret;
  return AMD_DBGAPI_STATUS_SUCCESS;
}

/* Find the marketing name for the PCI device VENDOR_ID:DEVICE_ID.

   The information is extracted from the pci.ids database[1] which might or
   might not be up to date, so the result might change from host to host.  The
   location of the pci.ids file is resolved by CMake and available via the
   PCI_IDS_PATH macro.  The file can usually be updated using the update-pciids
   command.

   [1] https://pci-ids.ucw.cz/  */

std::string
linux_driver_t::pci_device_name (uint32_t vendor_id, uint32_t device_id)
{
  auto fallback_name = [vendor_id, device_id] () -> std::string
  {
    std::stringstream name;
    name << "Device " << std::hex << std::setfill ('0') << std::setw (4)
         << vendor_id << ':' << std::setw (4) << device_id;
    return name.str ();
  };

  const char *pciids_file_path = PCI_IDS_PATH;
  std::ifstream pci_ids (pciids_file_path);
  if (!pci_ids.is_open ())
    {
      warning ("Could not open '%s'", pciids_file_path);
      return fallback_name ();
    }

  unsigned int curr_vendor_id = 0, curr_device_id = 0;
  std::string curr_device_name;
  for (std::string line; pci_ids.good (); std::getline (pci_ids, line))
    {
      std::stringstream line_st (line);
      int nextchar = line_st.peek ();
      if (nextchar == std::char_traits<char>::eof () || nextchar == '#')
        continue;
      if (nextchar != '\t')
        {
          /* Format: "vendor_id vendor_name".  */
          line_st >> std::hex >> curr_vendor_id;
          curr_device_id = 0;
          curr_device_name = "";

          /* We reached the end of the list of vendors and did not find
             the one we are looking for.  Break here as the end of the
             file has a different format.  */
          if (curr_vendor_id == 0xffff)
            break;
        }
      else
        {
          /* Skip decoding of the current line if the vendor_id does not match
             the one we are looking for.  */
          if (curr_vendor_id != vendor_id)
            continue;

          /* Consume the leading \t.  */
          line_st.get ();
          nextchar = line_st.peek ();
          if (nextchar != '\t')
            {
              /* Format: "TAB device_id device_name".  */
              line_st >> std::hex >> curr_device_id;
              std::getline (line_st >> std::ws, curr_device_name);
            }

          /* There is a "TAB TAB subvendor subdevice  subsystem_name" format
             we do not care about.  */
        }
      if (curr_vendor_id == vendor_id && curr_device_id == device_id)
        return curr_device_name;
    }

  /* We have not found the device.  */
  return fallback_name ();
}

/* OS Driver implementation for the Linux ROCm stack using KFD.  */

class kfd_driver_base_t : public linux_driver_t
{
public:
  explicit kfd_driver_base_t (amd_dbgapi_os_process_id_t os_pid)
    : linux_driver_t{ os_pid }
  {
  }

  amd_dbgapi_status_t check_version () const override final;

  amd_dbgapi_status_t
  agent_snapshot (os_agent_info_t *snapshots, size_t snapshot_count,
                  size_t *agent_count,
                  os_exception_mask_t exceptions_cleared) const override final;

  amd_dbgapi_status_t
  queue_snapshot (os_queue_snapshot_entry_t *snapshots, size_t snapshot_count,
                  size_t *queue_count,
                  os_exception_mask_t exceptions_cleared) const override final;

protected:
  using version_t = std::pair<uint32_t, uint32_t>;

  /* Query KFD version.  */
  virtual version_t get_kfd_version () const = 0;

  /* Perform the ioctl call, or act as if using core file provided data.  */
  virtual amd_dbgapi_status_t
  kfd_agent_snapshot (kfd_dbg_device_info_entry *agents, size_t snapshot_count,
                      size_t *agent_count,
                      os_exception_mask_t exceptions_cleared) const = 0;

  virtual amd_dbgapi_status_t
  kfd_queue_snapshot (kfd_queue_snapshot_entry *queues, size_t snapshot_count,
                      size_t *queue_cout,
                      os_exception_mask_t exceptions_cleared) const = 0;
};

amd_dbgapi_status_t
kfd_driver_base_t::check_version () const
{
  dbgapi_assert (is_valid ());

  constexpr version_t KFD_IOCTL_VERSION_BEGIN{ 1, 13 };
  constexpr version_t KFD_IOCTL_VERSION_END{ 2, 0 };

  version_t kfd_ioctl_version = get_kfd_version ();

  if (kfd_ioctl_version < KFD_IOCTL_VERSION_BEGIN
      || kfd_ioctl_version >= KFD_IOCTL_VERSION_END)
    {
      warning ("AMD GPU driver's version %u.%u not supported "
               "(version must be >= %u.%u and < %u.%u)",
               kfd_ioctl_version.first, kfd_ioctl_version.second,
               KFD_IOCTL_VERSION_BEGIN.first, KFD_IOCTL_VERSION_BEGIN.second,
               KFD_IOCTL_VERSION_END.first, KFD_IOCTL_VERSION_END.second);
      return AMD_DBGAPI_STATUS_ERROR_RESTRICTION;
    }

  log_info ("using AMD GPU driver version %d.%d", kfd_ioctl_version.first,
            kfd_ioctl_version.second);

  return AMD_DBGAPI_STATUS_SUCCESS;
}

amd_dbgapi_status_t
kfd_driver_base_t::agent_snapshot (
  os_agent_info_t *snapshots, size_t snapshot_count, size_t *agent_count,
  os_exception_mask_t exceptions_cleared) const
{
  TRACE_DRIVER_BEGIN (param_in (snapshots), param_in (snapshot_count),
                      param_in (agent_count), param_in (exceptions_cleared));

  dbgapi_assert (snapshots && agent_count && "must not be null");
  dbgapi_assert (snapshot_count <= std::numeric_limits<uint32_t>::max ()
                 && "invalid argument");

  if (!is_debug_enabled ())
    {
      *agent_count = 0;
      return AMD_DBGAPI_STATUS_SUCCESS;
    }

  std::vector<kfd_dbg_device_info_entry> kfd_device_infos (snapshot_count);
  if (amd_dbgapi_status_t status
      = kfd_agent_snapshot (kfd_device_infos.data (), snapshot_count,
                            agent_count, exceptions_cleared);
      status != AMD_DBGAPI_STATUS_SUCCESS)
    return status;

  for (unsigned int i = 0; i < std::min (snapshot_count, *agent_count); i++)
    {
      os_agent_info_t &agent_info = snapshots[i];
      const kfd_dbg_device_info_entry &entry = kfd_device_infos[i];

      agent_info.os_agent_id = entry.gpu_id;

      agent_info.local_address_aperture_base = entry.lds_base;
      agent_info.local_address_aperture_limit = entry.lds_limit;
      agent_info.private_address_aperture_base = entry.scratch_base;
      agent_info.private_address_aperture_limit = entry.scratch_limit;
      agent_info.location_id = entry.location_id;
      agent_info.simd_count = entry.simd_count * entry.num_xcc;
      agent_info.max_waves_per_simd = entry.max_waves_per_simd;
      agent_info.vendor_id = entry.vendor_id;
      agent_info.device_id = entry.device_id;
      agent_info.revision_id = entry.revision_id;
      agent_info.subsystem_vendor_id = entry.subsystem_vendor_id;
      agent_info.subsystem_device_id = entry.subsystem_device_id;
      agent_info.fw_version = entry.fw_version;
      agent_info.gfxip = { entry.gfx_target_version / 10000,
                           (entry.gfx_target_version / 100) % 100,
                           entry.gfx_target_version % 100 };
      agent_info.debugging_supported
        = entry.capability & HSA_CAP_TRAP_DEBUG_SUPPORT;
      agent_info.address_watch_supported
        = entry.capability & HSA_CAP_WATCH_POINTS_SUPPORTED;
      agent_info.address_watch_register_count
        = 1 << ((entry.capability & HSA_CAP_WATCH_POINTS_TOTALBITS_MASK)
                >> HSA_CAP_WATCH_POINTS_TOTALBITS_SHIFT);
      agent_info.precise_memory_supported
        = (entry.capability
           & HSA_CAP_TRAP_DEBUG_PRECISE_MEMORY_OPERATIONS_SUPPORTED);
      agent_info.firmware_supported
        = entry.capability & HSA_CAP_TRAP_DEBUG_FIRMWARE_SUPPORTED;
      agent_info.address_watch_mask_bits = utils::bit_mask (
        ((entry.debug_prop & HSA_DBG_WATCH_ADDR_MASK_LO_BIT_MASK)
         >> HSA_DBG_WATCH_ADDR_MASK_LO_BIT_SHIFT),
        ((entry.debug_prop & HSA_DBG_WATCH_ADDR_MASK_HI_BIT_MASK)
         >> HSA_DBG_WATCH_ADDR_MASK_HI_BIT_SHIFT));
      agent_info.ttmps_always_initialized
        = entry.debug_prop & HSA_DBG_DISPATCH_INFO_ALWAYS_VALID;
      agent_info.watchpoint_exclusive
        = entry.debug_prop & HSA_DBG_WATCHPOINTS_EXCLUSIVE;
      agent_info.xcc_count = entry.num_xcc;

      if (!agent_info.simd_count || !agent_info.max_waves_per_simd
          || !entry.array_count || !entry.simd_arrays_per_engine)
        fatal_error ("Invalid node properties");

      agent_info.shader_engine_count
        = (entry.array_count * entry.num_xcc) / entry.simd_arrays_per_engine;
      agent_info.name
        = pci_device_name (agent_info.vendor_id, agent_info.device_id);
    }

  return AMD_DBGAPI_STATUS_SUCCESS;

  TRACE_DRIVER_END (
    make_ref (param_out (snapshots), std::min (snapshot_count, *agent_count)),
    make_ref (param_out (agent_count)));
}

amd_dbgapi_status_t
kfd_driver_base_t::queue_snapshot (
  os_queue_snapshot_entry_t *snapshots, size_t snapshot_count,
  size_t *queue_count, os_exception_mask_t exceptions_cleared) const
{
  TRACE_DRIVER_BEGIN (param_in (snapshots), param_in (snapshot_count),
                      param_in (queue_count), param_in (exceptions_cleared));

  return kfd_queue_snapshot (snapshots, snapshot_count, queue_count,
                             exceptions_cleared);

  TRACE_DRIVER_END (
    make_ref (param_out (snapshots), std::min (snapshot_count, *queue_count)),
    make_ref (param_out (queue_count)));
}

class kfd_driver_t final : public kfd_driver_base_t
{
private:
  static size_t s_kfd_open_count;
  static std::optional<file_desc_t> s_kfd_fd;

  static void open_kfd ();
  static void close_kfd ();

  bool m_is_debug_enabled{ false };

  int kfd_dbg_trap_ioctl (uint32_t action,
                          kfd_ioctl_dbg_trap_args *args) const;

public:
  kfd_driver_t (amd_dbgapi_os_process_id_t os_pid) : kfd_driver_base_t (os_pid)
  {
    open_kfd ();

    /* See is_valid() for information about how failing to open /dev/kfd is
       handled.  */
  }

  ~kfd_driver_t () override
  {
    if (is_debug_enabled ())
      disable_debug ();

    close_kfd ();
  }

  /* Disable copies.  */
  kfd_driver_t (const kfd_driver_t &) = delete;
  kfd_driver_t &operator= (const kfd_driver_t &) = delete;

  bool is_valid () const override
  {
    return linux_driver_t::is_valid () && s_kfd_fd.has_value ();
  }

  kfd_driver_base_t::version_t get_kfd_version () const override final;

  amd_dbgapi_status_t
  kfd_agent_snapshot (kfd_dbg_device_info_entry *agents, size_t snapshot_count,
                      size_t *agent_count,
                      os_exception_mask_t exceptions_cleared) const override;

  amd_dbgapi_status_t enable_debug (os_exception_mask_t exceptions_reported,
                                    file_desc_t notifier,
                                    os_runtime_info_t *runtime_info) override;
  amd_dbgapi_status_t disable_debug () override;
  bool is_debug_enabled () const override { return m_is_debug_enabled; }

  amd_dbgapi_status_t
  send_exceptions (os_exception_mask_t exceptions,
                   std::optional<os_agent_id_t> agent_id,
                   std::optional<os_queue_id_t> queue_id) const override;

  amd_dbgapi_status_t set_exceptions_reported (
    os_exception_mask_t exceptions_reported) const override;

  amd_dbgapi_status_t
  query_debug_event (os_exception_mask_t *exceptions_present,
                     os_queue_id_t *os_queue_id, os_agent_id_t *os_agent_id,
                     os_exception_mask_t exceptions_cleared) override;

  amd_dbgapi_status_t
  query_exception_info (os_exception_code_t exception,
                        os_source_id_t os_source_id, void *exception_info,
                        size_t exception_info_size,
                        bool clear_exception) const override;

  amd_dbgapi_status_t suspend_queues (os_queue_id_t *queues,
                                      size_t queue_count,
                                      os_exception_mask_t exceptions_cleared,
                                      size_t *suspended_count) const override;
  amd_dbgapi_status_t resume_queues (os_queue_id_t *queues, size_t queue_count,
                                     size_t *resumed_count) const override;

  amd_dbgapi_status_t
  kfd_queue_snapshot (kfd_queue_snapshot_entry *snapshots,
                      size_t snapshot_count, size_t *queue_count,
                      os_exception_mask_t exceptions_cleared) const override;

  amd_dbgapi_status_t set_address_watch (
    os_agent_id_t os_agent_id, amd_dbgapi_global_address_t address,
    amd_dbgapi_global_address_t mask, os_watch_mode_t os_watch_mode,
    os_watch_id_t *os_watch_id) const override;

  amd_dbgapi_status_t
  clear_address_watch (os_agent_id_t os_agent_id,
                       os_watch_id_t os_watch_id) const override;

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
          log_info ("Could not open the KFD device: %s", strerror (errno));
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
        fatal_error ("failed to close s_kfd_fd");

      s_kfd_fd.reset ();
    }
}

int
kfd_driver_t::kfd_dbg_trap_ioctl (uint32_t action,
                                  kfd_ioctl_dbg_trap_args *args) const
{
  dbgapi_assert (is_valid ());
  dbgapi_assert (m_os_pid);

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

kfd_driver_base_t::version_t
kfd_driver_t::get_kfd_version () const
{
  kfd_ioctl_get_version_args get_version_args{};
  if (::ioctl (*s_kfd_fd, AMDKFD_IOC_GET_VERSION, &get_version_args))
    fatal_error ("AMDKFD_IOC_GET_VERSION failed");

  return { get_version_args.major_version, get_version_args.minor_version };
}

amd_dbgapi_status_t
kfd_driver_t::kfd_agent_snapshot (kfd_dbg_device_info_entry *agents_infos,
                                  size_t agent_info_count, size_t *agent_count,
                                  os_exception_mask_t exceptions_cleared) const
{
  TRACE_DRIVER_BEGIN (param_in (agents_infos), param_in (agent_info_count),
                      param_in (agent_count), param_in (exceptions_cleared));

  dbgapi_assert (agents_infos != nullptr && agent_count != nullptr
                 && "must not be null");
  dbgapi_assert (agent_info_count <= std::numeric_limits<uint32_t>::max ()
                 && "invalid argument");

  dbgapi_assert (is_debug_enabled () && "debug must be enabled");

  kfd_ioctl_dbg_trap_args args{};
  args.device_snapshot.exception_mask
    = static_cast<uint64_t> (exceptions_cleared);
  args.device_snapshot.snapshot_buf_ptr
    = reinterpret_cast<uint64_t> (agents_infos);
  args.device_snapshot.num_devices = static_cast<uint32_t> (*agent_count);
  args.device_snapshot.entry_size
    = static_cast<uint32_t> (sizeof (kfd_dbg_device_info_entry));

  int err = kfd_dbg_trap_ioctl (KFD_IOC_DBG_TRAP_GET_DEVICE_SNAPSHOT, &args);
  if (err == -ESRCH)
    return AMD_DBGAPI_STATUS_ERROR_PROCESS_EXITED;
  else if (args.device_snapshot.entry_size
             != sizeof (kfd_dbg_device_info_entry)
           || err < 0)
    return AMD_DBGAPI_STATUS_ERROR;

  *agent_count = args.device_snapshot.num_devices;

  return AMD_DBGAPI_STATUS_SUCCESS;

  TRACE_DRIVER_END (make_ref (param_out (agents_infos),
                              std::min (agent_info_count, *agent_count)),
                    make_ref (param_out (agent_count)));
}

amd_dbgapi_status_t
kfd_driver_t::enable_debug (os_exception_mask_t exceptions_reported,
                            file_desc_t notifier,
                            os_runtime_info_t *runtime_info)
{
  TRACE_DRIVER_BEGIN (param_in (exceptions_reported), param_in (notifier),
                      param_in (runtime_info));

  dbgapi_assert (!is_debug_enabled () && "debug is already enabled");

  kfd_ioctl_dbg_trap_args args{};
  args.enable.exception_mask = static_cast<uint64_t> (exceptions_reported);
  args.enable.rinfo_ptr = reinterpret_cast<uintptr_t> (runtime_info);
  args.enable.rinfo_size = sizeof (*runtime_info);
  args.enable.dbg_fd = notifier;

  int err = kfd_dbg_trap_ioctl (KFD_IOC_DBG_TRAP_ENABLE, &args);
  if (err == -ESRCH)
    return AMD_DBGAPI_STATUS_ERROR_PROCESS_EXITED;
  else if (err == -EALREADY)
    return AMD_DBGAPI_STATUS_ERROR_RESTRICTION;
  else if (err < 0)
    return AMD_DBGAPI_STATUS_ERROR;

  if (sizeof (*runtime_info) > args.enable.rinfo_size)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT_COMPATIBILITY;

  m_is_debug_enabled = true;
  return AMD_DBGAPI_STATUS_SUCCESS;

  TRACE_DRIVER_END (make_ref (param_out (runtime_info)));
}

amd_dbgapi_status_t
kfd_driver_t::disable_debug ()
{
  TRACE_DRIVER_BEGIN ();

  if (!is_debug_enabled ())
    return AMD_DBGAPI_STATUS_SUCCESS;

  kfd_ioctl_dbg_trap_args args{};

  int err = kfd_dbg_trap_ioctl (KFD_IOC_DBG_TRAP_DISABLE, &args);
  if (err == -ESRCH)
    return AMD_DBGAPI_STATUS_ERROR_PROCESS_EXITED;
  else if (err < 0)
    return AMD_DBGAPI_STATUS_ERROR;

  m_is_debug_enabled = false;
  return AMD_DBGAPI_STATUS_SUCCESS;

  TRACE_DRIVER_END ();
}

amd_dbgapi_status_t
kfd_driver_t::set_exceptions_reported (
  os_exception_mask_t exceptions_reported) const
{
  TRACE_DRIVER_BEGIN (exceptions_reported);

  kfd_ioctl_dbg_trap_args args{};
  args.set_exceptions_enabled.exception_mask
    = static_cast<uint64_t> (exceptions_reported);

  int err
    = kfd_dbg_trap_ioctl (KFD_IOC_DBG_TRAP_SET_EXCEPTIONS_ENABLED, &args);
  if (err == -ESRCH)
    return AMD_DBGAPI_STATUS_ERROR_PROCESS_EXITED;
  else if (err < 0)
    return AMD_DBGAPI_STATUS_ERROR;

  return AMD_DBGAPI_STATUS_SUCCESS;

  TRACE_DRIVER_END ();
}

amd_dbgapi_status_t
kfd_driver_t::send_exceptions (os_exception_mask_t exceptions,
                               std::optional<os_agent_id_t> agent_id,
                               std::optional<os_queue_id_t> queue_id) const
{
  TRACE_DRIVER_BEGIN (param_in (exceptions), param_in (agent_id),
                      param_in (queue_id));

  dbgapi_assert (is_debug_enabled () && "debug is not enabled");

  kfd_ioctl_dbg_trap_args args{};
  args.send_runtime_event.exception_mask = static_cast<uint64_t> (exceptions);
  args.send_runtime_event.gpu_id = agent_id.has_value () ? *agent_id : -1;
  args.send_runtime_event.queue_id = queue_id.has_value () ? *queue_id : -1;

  int err = kfd_dbg_trap_ioctl (KFD_IOC_DBG_TRAP_SEND_RUNTIME_EVENT, &args);
  if (err == -ESRCH)
    return AMD_DBGAPI_STATUS_ERROR_PROCESS_EXITED;
  else if (err < 0)
    return AMD_DBGAPI_STATUS_ERROR;

  return AMD_DBGAPI_STATUS_SUCCESS;

  TRACE_DRIVER_END ();
}

amd_dbgapi_status_t
kfd_driver_t::query_debug_event (os_exception_mask_t *exceptions_present,
                                 os_queue_id_t *os_queue_id,
                                 os_agent_id_t *os_agent_id,
                                 os_exception_mask_t exceptions_cleared)
{
  TRACE_DRIVER_BEGIN (param_in (exceptions_present), param_in (os_queue_id),
                      param_in (os_agent_id), param_in (exceptions_cleared));

  dbgapi_assert (exceptions_present && os_queue_id && os_agent_id
                 && "must not be null");

  if (!is_debug_enabled ())
    {
      *exceptions_present = os_exception_mask_t::none;
      *os_queue_id = *os_agent_id = 0;
      return AMD_DBGAPI_STATUS_SUCCESS;
    }

  kfd_ioctl_dbg_trap_args args{};
  args.query_debug_event.exception_mask
    = static_cast<uint64_t> (exceptions_cleared);

  int err = kfd_dbg_trap_ioctl (KFD_IOC_DBG_TRAP_QUERY_DEBUG_EVENT, &args);
  if (err == -ESRCH)
    return AMD_DBGAPI_STATUS_ERROR_PROCESS_EXITED;
  else if (err == -EAGAIN)
    {
      /* There are no more events.  */
      *exceptions_present = os_exception_mask_t::none;
      *os_queue_id = *os_agent_id = 0;
      return AMD_DBGAPI_STATUS_SUCCESS;
    }
  else if (err < 0)
    return AMD_DBGAPI_STATUS_ERROR;

  *exceptions_present
    = static_cast<os_exception_mask_t> (args.query_debug_event.exception_mask);
  *os_queue_id = args.query_debug_event.queue_id;
  *os_agent_id = args.query_debug_event.gpu_id;

  return AMD_DBGAPI_STATUS_SUCCESS;

  TRACE_DRIVER_END (make_ref (param_out (exceptions_present)),
                    make_ref (param_out (os_queue_id)),
                    make_ref (param_out (os_agent_id)));
}

amd_dbgapi_status_t
kfd_driver_t::query_exception_info (os_exception_code_t exception,
                                    os_source_id_t os_source_id,
                                    void *exception_info,
                                    size_t exception_info_size,
                                    bool clear_exception) const
{
  TRACE_DRIVER_BEGIN (
    param_in (exception), param_in (os_source_id), param_in (exception_info),
    param_in (exception_info_size), param_in (clear_exception));

  dbgapi_assert (is_debug_enabled () && "debug is not enabled");

  kfd_ioctl_dbg_trap_args args{};
  args.query_exception_info.info_ptr
    = reinterpret_cast<uintptr_t> (exception_info);
  args.query_exception_info.info_size = exception_info_size;
  args.query_exception_info.source_id = os_source_id.raw;
  args.query_exception_info.exception_code = static_cast<uint32_t> (exception);
  args.query_exception_info.clear_exception = clear_exception ? 1 : 0;

  int err = kfd_dbg_trap_ioctl (KFD_IOC_DBG_TRAP_QUERY_EXCEPTION_INFO, &args);
  if (err == -ESRCH)
    return AMD_DBGAPI_STATUS_ERROR_PROCESS_EXITED;
  else if (err < 0)
    return AMD_DBGAPI_STATUS_ERROR;

  return exception_info_size > args.query_exception_info.info_size
           ? AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT_COMPATIBILITY
           : AMD_DBGAPI_STATUS_SUCCESS;

  TRACE_DRIVER_END (make_query_ref (exception, param_out (exception_info)));
}

amd_dbgapi_status_t
kfd_driver_t::suspend_queues (os_queue_id_t *queues, size_t queue_count,
                              os_exception_mask_t exceptions_cleared,
                              size_t *suspended_count) const
{
  TRACE_DRIVER_BEGIN (make_ref (param_in (queues), queue_count),
                      param_in (queue_count), param_in (exceptions_cleared),
                      param_in (suspended_count));

  dbgapi_assert (suspended_count != nullptr);
  dbgapi_assert (queue_count <= std::numeric_limits<uint32_t>::max ());

  kfd_ioctl_dbg_trap_args args{};
  args.suspend_queues.exception_mask
    = static_cast<uint64_t> (exceptions_cleared);
  args.suspend_queues.queue_array_ptr = reinterpret_cast<uint64_t> (queues);
  args.suspend_queues.num_queues = static_cast<uint32_t> (queue_count);
  args.suspend_queues.grace_period = 0;

  int ret = kfd_dbg_trap_ioctl (KFD_IOC_DBG_TRAP_SUSPEND_QUEUES, &args);
  if (ret == -ESRCH)
    return AMD_DBGAPI_STATUS_ERROR_PROCESS_EXITED;
  else if (ret < 0)
    return AMD_DBGAPI_STATUS_ERROR;

  *suspended_count = ret;
  return AMD_DBGAPI_STATUS_SUCCESS;

  TRACE_DRIVER_END (
    make_ref (param_out (queues), std::min (queue_count, *suspended_count)),
    make_ref (param_out (suspended_count)));
}

amd_dbgapi_status_t
kfd_driver_t::resume_queues (os_queue_id_t *queues, size_t queue_count,
                             size_t *resumed_count) const
{
  TRACE_DRIVER_BEGIN (make_ref (param_in (queues), queue_count),
                      param_in (queue_count), param_in (resumed_count));

  dbgapi_assert (resumed_count != nullptr);
  dbgapi_assert (queue_count <= std::numeric_limits<uint32_t>::max ());

  kfd_ioctl_dbg_trap_args args{};
  args.resume_queues.queue_array_ptr = reinterpret_cast<uint64_t> (queues);
  args.resume_queues.num_queues = static_cast<uint32_t> (queue_count);

  int ret = kfd_dbg_trap_ioctl (KFD_IOC_DBG_TRAP_RESUME_QUEUES, &args);
  if (ret == -ESRCH)
    return AMD_DBGAPI_STATUS_ERROR_PROCESS_EXITED;
  else if (ret < 0)
    return AMD_DBGAPI_STATUS_ERROR;

  *resumed_count = ret;
  return AMD_DBGAPI_STATUS_SUCCESS;

  TRACE_DRIVER_END (
    make_ref (param_out (queues), std::min (queue_count, *resumed_count)),
    make_ref (param_out (resumed_count)));
}

amd_dbgapi_status_t
kfd_driver_t::kfd_queue_snapshot (kfd_queue_snapshot_entry *snapshots,
                                  size_t snapshot_count, size_t *queue_count,
                                  os_exception_mask_t exceptions_cleared) const
{
  dbgapi_assert (snapshots != nullptr && queue_count != nullptr
                 && "must not be null");
  dbgapi_assert (snapshot_count <= std::numeric_limits<uint32_t>::max ()
                 && "invalid argument");

  if (!is_debug_enabled ())
    {
      *queue_count = 0;
      return AMD_DBGAPI_STATUS_SUCCESS;
    }

  kfd_ioctl_dbg_trap_args args{};
  args.queue_snapshot.exception_mask
    = static_cast<uint64_t> (exceptions_cleared);
  args.queue_snapshot.snapshot_buf_ptr
    = reinterpret_cast<uint64_t> (snapshots);
  args.queue_snapshot.num_queues = static_cast<uint32_t> (snapshot_count);
  args.queue_snapshot.entry_size
    = static_cast<uint32_t> (sizeof (os_queue_snapshot_entry_t));

  int err = kfd_dbg_trap_ioctl (KFD_IOC_DBG_TRAP_GET_QUEUE_SNAPSHOT, &args);
  if (err == -ESRCH)
    return AMD_DBGAPI_STATUS_ERROR_PROCESS_EXITED;
  else if (args.queue_snapshot.entry_size != sizeof (os_queue_snapshot_entry_t)
           || err < 0)
    return AMD_DBGAPI_STATUS_ERROR;

  /* KFD writes up to snapshot_count queue snapshots, but returns the number of
     queues in the process so that we can check if we have allocated enough
     memory to hold all the snapshots.  */
  *queue_count = args.queue_snapshot.num_queues;

  return AMD_DBGAPI_STATUS_SUCCESS;
}

amd_dbgapi_status_t
kfd_driver_t::set_address_watch (os_agent_id_t os_agent_id,
                                 amd_dbgapi_global_address_t address,
                                 amd_dbgapi_global_address_t mask,
                                 os_watch_mode_t os_watch_mode,
                                 os_watch_id_t *os_watch_id) const
{
  TRACE_DRIVER_BEGIN (param_in (address), param_in (mask),
                      param_in (os_watch_mode), param_in (os_watch_id));

  dbgapi_assert (os_watch_id && "must not be null");

  kfd_ioctl_dbg_trap_args args{};
  args.set_node_address_watch.address = address;
  args.set_node_address_watch.mode
    = static_cast<std::underlying_type_t<decltype (os_watch_mode)>> (
      os_watch_mode);
  args.set_node_address_watch.mask = mask;
  args.set_node_address_watch.gpu_id = os_agent_id;

  int err
    = kfd_dbg_trap_ioctl (KFD_IOC_DBG_TRAP_SET_NODE_ADDRESS_WATCH, &args);
  if (err == -ENOMEM)
    return AMD_DBGAPI_STATUS_ERROR_NO_WATCHPOINT_AVAILABLE;
  else if (err == -ESRCH)
    return AMD_DBGAPI_STATUS_ERROR_PROCESS_EXITED;
  else if (err < 0)
    fatal_error ("failed to set address watch: %s", strerror (err));

  *os_watch_id = args.set_node_address_watch.id;

  return AMD_DBGAPI_STATUS_SUCCESS;

  TRACE_DRIVER_END (make_ref (param_out (os_watch_id)));
}

amd_dbgapi_status_t
kfd_driver_t::clear_address_watch (os_agent_id_t os_agent_id,
                                   os_watch_id_t os_watch_id) const
{
  TRACE_DRIVER_BEGIN (param_in (os_watch_id));

  kfd_ioctl_dbg_trap_args args{};
  args.clear_node_address_watch.gpu_id = os_agent_id;
  args.clear_node_address_watch.id = os_watch_id;

  int err
    = kfd_dbg_trap_ioctl (KFD_IOC_DBG_TRAP_CLEAR_NODE_ADDRESS_WATCH, &args);
  if (err == -ESRCH)
    return AMD_DBGAPI_STATUS_ERROR_PROCESS_EXITED;
  else if (err < 0)
    return AMD_DBGAPI_STATUS_ERROR;

  return AMD_DBGAPI_STATUS_SUCCESS;

  TRACE_DRIVER_END ();
}

amd_dbgapi_status_t
kfd_driver_t::set_wave_launch_mode (os_wave_launch_mode_t mode) const
{
  TRACE_DRIVER_BEGIN (param_in (mode));

  kfd_ioctl_dbg_trap_args args{};
  args.launch_mode.launch_mode
    = static_cast<std::underlying_type_t<decltype (mode)>> (mode);

  int err = kfd_dbg_trap_ioctl (KFD_IOC_DBG_TRAP_SET_WAVE_LAUNCH_MODE, &args);
  if (err == -ESRCH)
    return AMD_DBGAPI_STATUS_ERROR_PROCESS_EXITED;
  else if (err < 0)
    return AMD_DBGAPI_STATUS_ERROR;

  return AMD_DBGAPI_STATUS_SUCCESS;

  TRACE_DRIVER_END ();
}

amd_dbgapi_status_t
kfd_driver_t::set_wave_launch_trap_override (
  os_wave_launch_trap_override_t override, os_wave_launch_trap_mask_t value,
  os_wave_launch_trap_mask_t mask, os_wave_launch_trap_mask_t *previous_value,
  os_wave_launch_trap_mask_t *supported_mask) const
{
  TRACE_DRIVER_BEGIN (param_in (override), param_in (value), param_in (mask),
                      param_in (previous_value), param_in (supported_mask));

  kfd_ioctl_dbg_trap_args args{};
  args.launch_override.override_mode
    = static_cast<std::underlying_type_t<decltype (override)>> (override);
  args.launch_override.enable_mask
    = static_cast<std::underlying_type_t<decltype (value)>> (value);
  args.launch_override.support_request_mask
    = static_cast<std::underlying_type_t<decltype (mask)>> (mask);

  int err
    = kfd_dbg_trap_ioctl (KFD_IOC_DBG_TRAP_SET_WAVE_LAUNCH_OVERRIDE, &args);
  if (err == -ESRCH)
    return AMD_DBGAPI_STATUS_ERROR_PROCESS_EXITED;
  else if (err == -EPERM || err == -EACCES)
    return AMD_DBGAPI_STATUS_ERROR_NOT_SUPPORTED;
  else if (err < 0)
    return AMD_DBGAPI_STATUS_ERROR;

  if (previous_value != nullptr)
    *previous_value = static_cast<os_wave_launch_trap_mask_t> (
      args.launch_override.enable_mask);
  if (supported_mask != nullptr)
    *supported_mask = static_cast<os_wave_launch_trap_mask_t> (
      args.launch_override.support_request_mask);

  return AMD_DBGAPI_STATUS_SUCCESS;

  TRACE_DRIVER_END (make_ref (param_out (previous_value)),
                    make_ref (param_out (supported_mask)));
}

amd_dbgapi_status_t
kfd_driver_t::set_precise_memory (bool enabled) const
{
  TRACE_DRIVER_BEGIN (param_in (enabled));

  kfd_ioctl_dbg_trap_args args{};
  args.set_flags.flags = enabled ? KFD_DBG_TRAP_FLAG_SINGLE_MEM_OP /* enable */
                                 : 0 /* disable  */;

  int err = kfd_dbg_trap_ioctl (KFD_IOC_DBG_TRAP_SET_FLAGS, &args);
  if (err == -ESRCH)
    return AMD_DBGAPI_STATUS_ERROR_PROCESS_EXITED;
  else if (err < 0)
    return AMD_DBGAPI_STATUS_ERROR;

  return AMD_DBGAPI_STATUS_SUCCESS;

  TRACE_DRIVER_END ();
}

std::unique_ptr<os_driver_t>
os_driver_t::create_driver (std::optional<amd_dbgapi_os_process_id_t> os_pid)
{
  if (!os_pid)
    return std::make_unique<null_driver_t> ();

  std::unique_ptr<os_driver_t> os_driver{ new kfd_driver_t (*os_pid) };
  if (os_driver->is_valid ())
    return os_driver;

  /* If we failed to create a kfd_driver_t (kfd is not installed?), then revert
     to a plain null driver.  */
  return std::make_unique<null_driver_t> (*os_pid);
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
    case os_exception_mask_t::queue_wave_abort:
      return "QUEUE_WAVE_ABORT";
    case os_exception_mask_t::queue_wave_trap:
      return "QUEUE_WAVE_TRAP";
    case os_exception_mask_t::queue_wave_math_error:
      return "QUEUE_WAVE_MATH_ERROR";
    case os_exception_mask_t::queue_wave_illegal_instruction:
      return "QUEUE_WAVE_ILLEGAL_INSTRUCTION";
    case os_exception_mask_t::queue_wave_memory_violation:
      return "QUEUE_WAVE_MEMORY_VIOLATION";
    case os_exception_mask_t::queue_wave_aperture_violation:
      return "QUEUE_WAVE_APERTURE_VIOLATION";
    case os_exception_mask_t::queue_packet_dispatch_dim_invalid:
      return "QUEUE_PACKET_DISPATCH_DIM_INVALID";
    case os_exception_mask_t::queue_packet_dispatch_group_segment_size_invalid:
      return "QUEUE_PACKET_DISPATCH_GROUP_SEGMENT_SIZE_INVALID";
    case os_exception_mask_t::queue_packet_dispatch_code_invalid:
      return "QUEUE_PACKET_DISPATCH_CODE_INVALID";
    case os_exception_mask_t::queue_packet_unsupported:
      return "QUEUE_PACKET_UNSUPPORTED";
    case os_exception_mask_t::queue_packet_dispatch_work_group_size_invalid:
      return "QUEUE_PACKET_DISPATCH_WORKGROUP_SIZE_INVALID";
    case os_exception_mask_t::queue_packet_dispatch_register_invalid:
      return "QUEUE_PACKET_DISPATCH_REGISTER_INVALID";
    case os_exception_mask_t::queue_packet_vendor_unsupported:
      return "QUEUE_PACKET_VENDOR_UNSUPPORTED";
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
    case os_exception_mask_t::process_runtime:
      return "PROCESS_RUNTIME";
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

template <>
std::string
to_string (os_agent_info_t os_agent_info)
{
  return string_printf (
    "{ .os_agent_id=%d, .name=%s, .domain=%#x, .location_id=%#x, "
    ".gfxip=[%d,%d,%d], .simd_count=%zd, .max_waves_per_simd=%zd, "
    ".shader_engine_count=%zd, .vendor_id=%#x, .device_id=%#x, "
    ".revision_id=%#x, .subsystem_vendor_id=%#x, .subsystem_device_id=%#x, "
    ".fw_version=%d, .local_address_aperture_base=%#" PRIx64 ", "
    ".local_address_aperture_limit=%#" PRIx64 ", "
    ".private_address_aperture_base=%#" PRIx64 ", "
    ".private_address_aperture_limit=%#" PRIx64 ", .debugging_supported=%d, "
    ".address_watch_supported=%d, .address_watch_register_count=%zd, "
    ".address_watch_mask_bits=%#" PRIx64 ", .watchpoint_exclusive=%d, "
    ".precise_memory_supported=%d, .firmware_supported=%d, "
    "ttmps_always_initialized=%d }",
    os_agent_info.os_agent_id, os_agent_info.name.c_str (),
    os_agent_info.domain, os_agent_info.location_id, os_agent_info.gfxip[0],
    os_agent_info.gfxip[1], os_agent_info.gfxip[2], os_agent_info.simd_count,
    os_agent_info.max_waves_per_simd, os_agent_info.shader_engine_count,
    os_agent_info.vendor_id, os_agent_info.device_id,
    os_agent_info.revision_id, os_agent_info.subsystem_vendor_id,
    os_agent_info.subsystem_device_id, os_agent_info.fw_version,
    os_agent_info.local_address_aperture_base,
    os_agent_info.local_address_aperture_limit,
    os_agent_info.private_address_aperture_base,
    os_agent_info.private_address_aperture_limit,
    os_agent_info.debugging_supported, os_agent_info.address_watch_supported,
    os_agent_info.address_watch_register_count,
    os_agent_info.address_watch_mask_bits, os_agent_info.watchpoint_exclusive,
    os_agent_info.precise_memory_supported, os_agent_info.firmware_supported,
    os_agent_info.ttmps_always_initialized);
}

template <>
std::string
to_string (kfd_dbg_device_info_entry entry)
{
  return string_printf (
    "{ .exception_status=%#llx, .lds_base=%#llx, .lds_limit=%#llx, "
    ".scratch_base=%#llx, .scratch_limit=%#llx, .gpuvm_base=%#llx, "
    ".gpuvm_limit=%#llx, .gpu_id=%d, .location_id=%#x, .vendor_id=%#x, "
    ".device_id=%#x, .fw_version=%d, .gfx_target_version=%#x, "
    ".simd_count=%d, .max_waves_per_simd=%d, .array_count=%d, "
    ".simd_arrays_per_engine=%d, .capability=%#x, .debug_prop=%#x }",
    entry.exception_status, entry.lds_base, entry.lds_limit,
    entry.scratch_base, entry.scratch_limit, entry.gpuvm_base,
    entry.gpuvm_limit, entry.gpu_id, entry.location_id, entry.vendor_id,
    entry.device_id, entry.fw_version, entry.gfx_target_version,
    entry.simd_count, entry.max_waves_per_simd, entry.array_count,
    entry.simd_arrays_per_engine, entry.capability, entry.debug_prop);
}

template <>
std::string
to_string (os_runtime_state_t runtime_state)
{
  switch (runtime_state)
    {
    case os_runtime_state_t::disabled:
      return "DISABLED";
    case os_runtime_state_t::enabled:
      return "ENABLED";
    case os_runtime_state_t::enabled_busy:
      return "ENABLED_BUSY";
    case os_runtime_state_t::enabled_error:
      return "ENABLED_ERROR";
    }
  return to_string (
    make_hex (static_cast<std::underlying_type_t<decltype (runtime_state)>> (
      runtime_state)));
}

template <>
std::string
to_string (os_runtime_info_t runtime_info)
{
  return string_printf (
    "{ .r_debug=%#llx, .runtime_state=%s, .ttmp_setup=%d }",
    runtime_info.r_debug,
    to_cstring (static_cast<os_runtime_state_t> (runtime_info.runtime_state)),
    runtime_info.ttmp_setup);
}

template <>
std::string
to_string (os_wave_launch_trap_override_t override)
{
  switch (override)
    {
    case os_wave_launch_trap_override_t::apply:
      return "APPLY";
    case os_wave_launch_trap_override_t::replace:
      return "REPLACED";
    }
  return to_string (make_hex (
    static_cast<std::underlying_type_t<decltype (override)>> (override)));
}

template <>
std::string
to_string (os_source_id_t source_id)
{
  return to_string (source_id.raw);
}

template <>
std::string
to_string (os_queue_snapshot_entry_t snapshot)
{
  return string_printf (
    "{ .exception_status=%#llx, .ring_base_address=%#llx, "
    ".write_pointer_address=%#llx, .read_pointer_address=%#llx, "
    ".ctx_save_restore_address=%#llx, .queue_id=%d, .gpu_id=%d, "
    ".ring_size=%d, .queue_type=%d }",
    snapshot.exception_status, snapshot.ring_base_address,
    snapshot.write_pointer_address, snapshot.read_pointer_address,
    snapshot.ctx_save_restore_address, snapshot.queue_id, snapshot.gpu_id,
    snapshot.ring_size, snapshot.queue_type);
}

template <>
std::string
to_string (os_watch_mode_t watch_mode)
{
  switch (watch_mode)
    {
    case os_watch_mode_t::all:
      return "ALL";
    case os_watch_mode_t::atomic:
      return "ATOMIC";
    case os_watch_mode_t::nonread:
      return "NONREAD";
    case os_watch_mode_t::read:
      return "READ";
    }
  return to_string (make_hex (
    static_cast<std::underlying_type_t<decltype (watch_mode)>> (watch_mode)));
}

template <>
std::string
to_string (detail::query_ref<os_exception_code_t> ref)
{
  auto [query, value] = ref;

  if (query == os_exception_code_t::process_runtime)
    return to_string (
      make_ref (static_cast<const os_runtime_info_t *> (value)));

  return {};
}

} /* namespace amd::dbgapi */
