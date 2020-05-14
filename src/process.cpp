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

#include "defs.h"

#include "architecture.h"
#include "callbacks.h"
#include "code_object.h"
#include "debug.h"
#include "event.h"
#include "linux/kfd_ioctl.h"
#include "logging.h"
#include "process.h"
#include "queue.h"
#include "rocr_rdebug.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstring>
#include <exception>
#include <fstream>
#include <future>
#include <iterator>
#include <list>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <unistd.h>

namespace amd
{
namespace dbgapi
{

std::list<process_t *> process_list;

constexpr struct gfxip_lookup_table
{
  const char *gpu_name; /* Device name reported by KFD.  */
  struct
  {
    uint8_t major;    /* GFXIP Major engine version.  */
    uint8_t minor;    /* GFXIP Minor engine version.  */
    uint8_t stepping; /* GFXIP Stepping info.  */
  } gfxip;
  uint16_t fw_version; /* Minimum required firmware version.  */
} gfxip_lookup_table[]
    = { { "vega10", { 9, 0, 0 }, 432 },  { "raven", { 9, 0, 2 }, 0 },
        { "vega12", { 9, 0, 4 }, 0 },    { "vega20", { 9, 0, 6 }, 432 },
        { "arcturus", { 9, 0, 8 }, 34 }, { "navi10", { 10, 1, 0 }, 0 },
        { "navi12", { 10, 1, 1 }, 0 },   { "navi14", { 10, 1, 2 }, 0 } };

namespace detail
{

static size_t kfd_open_count{ 0 };
static file_desc_t kfd_fd{ -1 };

} /* namespace detail */

/* Open the KFD device. The file descriptor is reference counted, multiple
   calls to open_kfd are allowed, as long as the same number of open_kfd and
   close_kfd are called.  The last call to close_kfd closes the device.  */

static file_desc_t
open_kfd ()
{
  using namespace detail;

  if (!kfd_open_count)
    {
      dbgapi_assert (kfd_fd == -1 && "kfd_fd is already open");
      if ((kfd_fd = ::open ("/dev/kfd", O_RDWR | O_CLOEXEC)) == -1)
        return file_desc_t{ -1 };
    }

  ++kfd_open_count;
  return kfd_fd;
}

static int
close_kfd ()
{
  using namespace detail;

  dbgapi_assert (kfd_open_count > 0 && "kfd_fd is already closed");

  /* The last call to close_kfd closes the KFD device.  */
  if (!--kfd_open_count)
    {
      dbgapi_assert (kfd_fd != -1 && "invalid kfd_fd");

      file_desc_t fd = kfd_fd;
      kfd_fd = file_desc_t{ -1 };

      if (::close (fd))
        return -1;
    }

  return 0;
}

int
process_t::dbg_trap_ioctl (uint32_t action, kfd_ioctl_dbg_trap_args *args)
{
  if (m_process_exited)
    return -ESRCH;

  args->pid = m_os_pid;
  args->op = action;

  int ret = ::ioctl (m_kfd_fd, AMDKFD_IOC_DBG_TRAP, args);
  if (ret < 0 && errno == ESRCH)
    {
      /* The target process does not exist, it must have exited.  */
      m_process_exited = true;
      /* FIXME: We should tear down the process now, so that any operation
         executed after this point returns an error.  */
      return -ESRCH;
    }

  return ret < 0 ? -errno : ret;
}

process_t::process_t (amd_dbgapi_client_process_id_t client_process_id,
                      amd_dbgapi_process_id_t process_id)
    : m_process_id (process_id), m_client_process_id (client_process_id)
{
  if (get_os_pid (&m_os_pid) != AMD_DBGAPI_STATUS_SUCCESS)
    return;

  /* Open the /proc/pid/mem file for this process.  */
  std::string filename = string_printf ("/proc/%d/mem", m_os_pid);
  if ((m_proc_mem_fd
       = open (filename.c_str (), O_RDWR | O_LARGEFILE | O_CLOEXEC, 0))
      == -1)
    warning ("Could not open `%s': %s", filename.c_str (), strerror (errno));

  /* Open the /dev/kfd device.  */
  if ((m_kfd_fd = open_kfd ()) == -1)
    warning ("Could not open the KFD device: %s", strerror (errno));

  /* Create the notifier pipe.  */
  m_client_notifier_pipe.open ();

  /* See is_valid() for information about how failing to open the files or
     the notifier pipe is handled.  */
}

bool
process_t::is_valid () const
{
  /* This process is ready if the /dev/kfd and /proc/pid/mem files are open,
     and the notifier pipe (used to communicate with the client) is ready.
     A process only exists in the not ready state while being created by the
     factory, and if not ready  will be destructed and never be put in the map
     of processes.
   */
  return m_os_pid != -1 && m_kfd_fd != -1 && m_proc_mem_fd != -1
         && m_client_notifier_pipe.is_valid ();
}

void
process_t::detach ()
{
  std::exception_ptr exception;
  std::vector<queue_t *> queues;

  /* The client process could have exited, so refresh its os pid.  */
  amd_dbgapi_status_t status = get_os_pid (&m_os_pid);
  if (status == AMD_DBGAPI_STATUS_ERROR_PROCESS_EXITED)
    m_process_exited = true;
  else if (status != AMD_DBGAPI_STATUS_SUCCESS)
    error ("get_os_pid callback failed (rc=%d)", status);

  /* If an exception is raised while attempting to detach, make sure we still
     destruct the handle objects in the correct order. To achieve this, we
     catch any exception, and rethrow it later.
   */

  if (!m_process_exited)
    try
      {
        /* We don't need to resume the queues until we are done changing the
           state.  */
        set_forward_progress_needed (false);

        /* Resume all the waves halted at launch.  */
        set_wave_launch_mode (wave_launch_mode_t::NORMAL);

        update_queues ();

        /* Suspend the queues that weren't already suspended.  */
        for (auto &&queue : range<queue_t> ())
          if (!queue.suspended ())
            queues.emplace_back (&queue);

        suspend_queues (queues);

        /* Resume the waves that were halted by a debug event (single-step,
           breakpoint, watchpoint), but keep the waves halted because of an
           exception running.  */
        for (auto &&wave : range<wave_t> ())
          {
            /* TODO: Move this to the architecture class.  Not absolutely
               necessary, but restore the DATA0/DATA1 registers to zero for the
               next attach.  */
            uint64_t zero = 0;
            wave.write_register (amdgpu_regnum_t::WAVE_ID, &zero);

            /* Resume the wave if it is single-stepping, or if it is stopped
               because of a debug event (completed single-step, breakpoint,
               watchpoint).  */
            if ((wave.state () == AMD_DBGAPI_WAVE_STATE_SINGLE_STEP)
                || (wave.state () == AMD_DBGAPI_WAVE_STATE_STOP
                    && !(wave.stop_reason ()
                         & ~(AMD_DBGAPI_WAVE_STOP_REASON_SINGLE_STEP
                             | AMD_DBGAPI_WAVE_STOP_REASON_BREAKPOINT
                             | AMD_DBGAPI_WAVE_STOP_REASON_WATCHPOINT))))
              {
                wave.set_state (AMD_DBGAPI_WAVE_STATE_RUN);
              }
          }

        /* Resume all the queues.  */
        set_forward_progress_needed (true);
      }
    catch (...)
      {
        exception = std::current_exception ();
      }

  /* Stop the event thread before destructing the agents.  The event loop polls
     the file descriptors returned by the KFD for each agent.  We need to
     terminate the event loop before the files are closed.  */
  if (stop_event_thread () != AMD_DBGAPI_STATUS_SUCCESS)
    error ("Could not stop the event thread");

  /* Destruct the waves, dispatches, queues, and agents, in this order.  */
  std::get<handle_object_set_t<wave_t>> (m_handle_object_sets).clear ();
  std::get<handle_object_set_t<dispatch_t>> (m_handle_object_sets).clear ();
  std::get<handle_object_set_t<queue_t>> (m_handle_object_sets).clear ();
  std::get<handle_object_set_t<agent_t>> (m_handle_object_sets).clear ();

  /* Destruct the breakpoints before the shared libraries and code objects  */
  std::get<handle_object_set_t<breakpoint_t>> (m_handle_object_sets).clear ();
  std::get<handle_object_set_t<code_object_t>> (m_handle_object_sets).clear ();
  std::get<handle_object_set_t<shared_library_t>> (m_handle_object_sets)
      .clear ();

  if (m_proc_mem_fd != -1)
    {
      ::close (m_proc_mem_fd);
      m_proc_mem_fd = -1;
    }

  m_client_notifier_pipe.close ();

  if (m_kfd_fd != -1 && close_kfd ())
    error ("Could not close the KFD device");

  if (exception)
    std::rethrow_exception (exception);
}

amd_dbgapi_status_t
process_t::read_global_memory_partial (amd_dbgapi_global_address_t address,
                                       void *buffer, size_t *size)
{
  ssize_t ret = pread (m_proc_mem_fd, buffer, *size, address);

  if (ret == -1 && errno != EIO && errno != EINVAL)
    warning ("process_t::read_memory failed: %s", strerror (errno));

  if (ret == -1 || (ret == 0 && *size != 0))
    return AMD_DBGAPI_STATUS_ERROR_MEMORY_ACCESS;
  else
    {
      *size = ret;
      return AMD_DBGAPI_STATUS_SUCCESS;
    }
}

amd_dbgapi_status_t
process_t::read_global_memory (amd_dbgapi_global_address_t address,
                               void *buffer, size_t size)
{
  amd_dbgapi_status_t status;
  size_t requested_size = size;

  status = read_global_memory_partial (address, buffer, &size);
  if (status != AMD_DBGAPI_STATUS_SUCCESS)
    return status;

  return (size == requested_size) ? AMD_DBGAPI_STATUS_SUCCESS
                                  : AMD_DBGAPI_STATUS_ERROR_MEMORY_ACCESS;
}

amd_dbgapi_status_t
process_t::read_string (amd_dbgapi_global_address_t address,
                        std::string *string, size_t size)
{
  constexpr size_t chunk_size = 16;
  static_assert (!(chunk_size & (chunk_size - 1)), "must be a power of 2");

  dbgapi_assert (string && "invalid argument");

  string->clear ();
  while (size > 0)
    {
      char staging_buffer[chunk_size];

      /* Transfer one aligned chunk at a time, except for the first read
         which could read less than a chunk if the start address is not
         aligned.  */

      size_t request_size = chunk_size - (address & (chunk_size - 1));
      size_t xfer_size = request_size;

      amd_dbgapi_status_t status
          = read_global_memory_partial (address, staging_buffer, &xfer_size);

      if (status != AMD_DBGAPI_STATUS_SUCCESS)
        return status;

      size_t length = std::min (size, xfer_size);

      if (!length)
        return AMD_DBGAPI_STATUS_ERROR_MEMORY_ACCESS;

      /* Copy the staging buffer into the string, stop at the '\0'
         terminating char if seen.  */
      for (size_t i = 0; i < length; ++i)
        {
          char c = staging_buffer[i];
          if (c == '\0')
            return AMD_DBGAPI_STATUS_SUCCESS;
          string->push_back (c);
        }

      /* If unable to read full request, then no point trying again as it will
         just fail again.  */
      if (request_size != xfer_size)
        return AMD_DBGAPI_STATUS_ERROR_MEMORY_ACCESS;

      address += length;
      size -= length;
    }

  return AMD_DBGAPI_STATUS_SUCCESS;
}

amd_dbgapi_status_t
process_t::write_global_memory_partial (amd_dbgapi_global_address_t address,
                                        const void *buffer, size_t *size)
{
  ssize_t ret = pwrite (m_proc_mem_fd, buffer, *size, address);

  if (ret == -1 && errno != EIO && errno != EINVAL)
    warning ("process_t::write_memory failed: %s", strerror (errno));

  if (ret == -1 || (ret == 0 && *size != 0))
    return AMD_DBGAPI_STATUS_ERROR_MEMORY_ACCESS;
  else
    {
      *size = ret;
      return AMD_DBGAPI_STATUS_SUCCESS;
    }
}

amd_dbgapi_status_t
process_t::write_global_memory (amd_dbgapi_global_address_t address,
                                const void *buffer, size_t size)
{
  amd_dbgapi_status_t status;
  size_t requested_size = size;

  status = write_global_memory_partial (address, buffer, &size);
  if (status != AMD_DBGAPI_STATUS_SUCCESS)
    return status;

  return (size == requested_size) ? AMD_DBGAPI_STATUS_SUCCESS
                                  : AMD_DBGAPI_STATUS_ERROR_MEMORY_ACCESS;
}

void
process_t::set_forward_progress_needed (bool forward_progress_needed)
{
  m_forward_progress_needed = forward_progress_needed;

  if (forward_progress_needed)
    {
      std::vector<queue_t *> queues;
      queues.reserve (count<queue_t> ());

      for (auto &&queue : range<queue_t> ())
        if (queue.suspended ())
          queues.emplace_back (&queue);

      resume_queues (queues);
    }
}

amd_dbgapi_status_t
process_t::set_wave_launch_mode (wave_launch_mode_t wave_launch_mode)
{
  if (m_wave_launch_mode == wave_launch_mode)
    return AMD_DBGAPI_STATUS_SUCCESS;

  auto &&agent_range = range<agent_t> ();
  for (auto it = agent_range.begin (); it != agent_range.end (); ++it)
    {
      amd_dbgapi_status_t status
          = set_wave_launch_mode (*it, wave_launch_mode);
      if (status != AMD_DBGAPI_STATUS_SUCCESS)
        error ("agent_t::set_wave_launch_mode (%s) failed (rc=%d)",
               to_string (wave_launch_mode).c_str (), status);
    }

  wave_launch_mode_t saved_wave_launch_mode = m_wave_launch_mode;
  m_wave_launch_mode = wave_launch_mode;

  if (saved_wave_launch_mode == wave_launch_mode_t::HALT)
    {
      /* We need to resume the waves that may be halted on launch.  Since we
         don't know anything about these waves, we have to suspend the queues
         and update the waves.  */
      update_queues ();

      std::vector<queue_t *> queues;
      queues.reserve (count<queue_t> ());

      for (auto &&queue : range<queue_t> ())
        {
          if (queue.suspended ())
            /* If the queue is already suspended, just update its waves.  */
            queue.update_waves (
                queue_t::update_waves_flag_t::UNHIDE_WAVES_HALTED_AT_LAUNCH);
          else
            queues.emplace_back (&queue);
        }

      suspend_queues (
          queues, queue_t::update_waves_flag_t::UNHIDE_WAVES_HALTED_AT_LAUNCH);

      /* Suspending the queues causes all the waves to be updated which will
         unhalt waves halted at launch. We now resume them if forward progress
         is needed.  */

      if (forward_progress_needed ())
        resume_queues (queues);
    }

  return AMD_DBGAPI_STATUS_SUCCESS;
}

process_t *
process_t::find (amd_dbgapi_process_id_t process_id, bool flush_cache)
{
  static std::pair<amd_dbgapi_process_id_t, process_t *> cache;

  if (flush_cache)
    cache = { { 0 }, nullptr };

  if (cache.first == process_id)
    return cache.second;

  for (process_t *p : process_list)
    if (p->m_process_id == process_id)
      {
        if (!flush_cache)
          cache = { process_id, p };
        return p;
      }

  return NULL;
}

process_t *
process_t::find (amd_dbgapi_client_process_id_t client_process_id,
                 bool flush_cache)
{
  static std::pair<amd_dbgapi_client_process_id_t, process_t *> cache;

  if (flush_cache)
    cache = { 0, 0 };

  if (cache.first == client_process_id)
    return cache.second;

  for (process_t *p : process_list)
    if (p->m_client_process_id == client_process_id)
      {
        if (!flush_cache)
          cache = { client_process_id, p };
        return p;
      }

  return NULL;
}

amd_dbgapi_status_t
process_t::update_agents (bool enable_debug_trap)
{
  struct sysfs_node_t
  {
    agent_t::kfd_gpu_id_t gpu_id;
    const architecture_t &architecture;
    agent_t::properties_t properties;
  };

  std::vector<sysfs_node_t> sysfs_nodes;

  /* Discover the GPU nodes from the sysfs topology.  */

  static const std::string sysfs_nodes_path (
      "/sys/devices/virtual/kfd/kfd/topology/nodes/");

  auto *dirp = opendir (sysfs_nodes_path.c_str ());
  if (!dirp)
    return AMD_DBGAPI_STATUS_ERROR;

  struct dirent *dir;
  while ((dir = readdir (dirp)) != 0)
    {
      if (!strcmp (dir->d_name, ".") || !strcmp (dir->d_name, ".."))
        continue;

      agent_t::properties_t props;
      std::string node_path (sysfs_nodes_path + dir->d_name);

      /* Retrieve the GPU ID.  */

      std::ifstream gpu_id_ifs (node_path + "/gpu_id");
      if (!gpu_id_ifs.is_open ())
        continue;

      agent_t::kfd_gpu_id_t gpu_id = 0;
      gpu_id_ifs >> gpu_id;

      if (!gpu_id)
        /* Skip CPU nodes.  */
        continue;

      /* Retrieve the GPU name.  */

      std::ifstream gpu_name_ifs (node_path + "/name");
      if (!gpu_name_ifs.is_open ())
        continue;

      gpu_name_ifs >> props.name;
      if (props.name.empty ())
        {
          warning ("gpu_id %d: asic family name not present in the sysfs.",
                   gpu_id);
          continue;
        }

      /* Retrieve the GPU node properties.  */

      std::ifstream props_ifs (node_path + "/properties");
      if (!props_ifs.is_open ())
        continue;

      std::string prop_name;
      uint64_t prop_value;
      while (props_ifs >> prop_name >> prop_value)
        {
          if (prop_name == "location_id")
            props.location_id = static_cast<uint32_t> (prop_value);
          else if (prop_name == "simd_count")
            props.simd_count = static_cast<uint32_t> (prop_value);
          else if (prop_name == "array_count")
            props.shader_engine_count = static_cast<uint32_t> (prop_value);
          else if (prop_name == "simd_arrays_per_engine")
            props.simd_arrays_per_engine = static_cast<uint32_t> (prop_value);
          else if (prop_name == "cu_per_simd_array")
            props.cu_per_simd_array = static_cast<uint32_t> (prop_value);
          else if (prop_name == "simd_per_cu")
            props.simd_per_cu = static_cast<uint32_t> (prop_value);
          else if (prop_name == "max_waves_per_simd")
            props.max_waves_per_simd = static_cast<uint32_t> (prop_value);
          else if (prop_name == "vendor_id")
            props.vendor_id = static_cast<uint32_t> (prop_value);
          else if (prop_name == "device_id")
            props.device_id = static_cast<uint32_t> (prop_value);
          else if (prop_name == "fw_version")
            props.fw_version = static_cast<uint16_t> (prop_value);
        }

      decltype (&gfxip_lookup_table[0]) gfxip_info = nullptr;
      size_t num_elem
          = sizeof (gfxip_lookup_table) / sizeof (gfxip_lookup_table[0]);

      for (size_t i = 0; i < num_elem; ++i)
        if (props.name == gfxip_lookup_table[i].gpu_name)
          {
            gfxip_info = &gfxip_lookup_table[i];
            break;
          }

      /* FIXME: May want to have a state for an agent so it can be listed as
         present, but marked as unsupported.  We would then remove the
         'continue's below and instantiate the agent.  */

      if (!gfxip_info)
        {
          warning ("gpu_id %d: asic family name %s not supported.", gpu_id,
                   props.name.c_str ());
          continue;
        }

      if (props.fw_version < gfxip_info->fw_version)
        {
          warning ("gpu_id %d: firmware version %d is not supported "
                   "(required version is >= %d)",
                   gpu_id, props.fw_version, gfxip_info->fw_version);
          continue;
        }

      const architecture_t *gpu_arch = architecture_t::find (
          gfxip_info->gfxip.major, gfxip_info->gfxip.minor,
          gfxip_info->gfxip.stepping);

      if (!gpu_arch)
        {
          warning ("gpu_id %d: gfx%d%d%d architecture not supported.", gpu_id,
                   gfxip_info->gfxip.major, gfxip_info->gfxip.minor,
                   gfxip_info->gfxip.stepping);
          continue;
        }

      sysfs_nodes.emplace_back (sysfs_node_t{ gpu_id, *gpu_arch, props });
    }

  closedir (dirp);

  /* Add new agents to the process.  */
  for (auto &sysfs_node : sysfs_nodes)
    {
      agent_t *agent = find_if (
          [&] (const agent_t &a) { return a.gpu_id () == sysfs_node.gpu_id; });

      if (!agent)
        agent = &create<agent_t> (*this,                   /* process  */
                                  sysfs_node.gpu_id,       /* gpu_id  */
                                  sysfs_node.architecture, /* architecture  */
                                  sysfs_node.properties);  /* properties  */

      if (enable_debug_trap && !agent->debug_trap_enabled ())
        {
          amd_dbgapi_status_t status = agent->enable_debug_trap ();
          if (status != AMD_DBGAPI_STATUS_SUCCESS)
            /* FIXME: We could not enable the debug mode for this agent.
               Another process may already have enabled debug mode for the
               same agent.  Remove this when KFD supports concurrent
               debugging on the same agent.  */
            error ("Could not enable debugging on gpu_id %d.",
                   sysfs_node.gpu_id);
        }

      if (agent->debug_trap_enabled ())
        {
          amd_dbgapi_status_t status
              = set_wave_launch_mode (*agent, m_wave_launch_mode);
          if (status != AMD_DBGAPI_STATUS_SUCCESS)
            error ("Could not set the wave launch mode for gpu_id %d (rc=%d).",
                   sysfs_node.gpu_id, status);
        }
    }

  /* Delete agents that are no longer present in this process. */
  auto &&agent_range = range<agent_t> ();
  for (auto it = agent_range.begin (); it != agent_range.end ();)
    if (std::find_if (
            sysfs_nodes.begin (), sysfs_nodes.end (),
            [&] (const decltype (sysfs_nodes)::value_type &sysfs_node) {
              return it->gpu_id () == sysfs_node.gpu_id;
            })
        == sysfs_nodes.end ())
      it = destroy (it);
    else
      ++it;

  return AMD_DBGAPI_STATUS_SUCCESS;
}

amd_dbgapi_status_t
process_t::enable_debug_trap (const agent_t &agent, file_desc_t *poll_fd)
{
  dbgapi_assert (poll_fd && "must not be null");

  /* KFD_IOC_DBG_TRAP_ENABLE (#0):
     data1: [in] enable/disable (1/0)
     data3: [out] poll_fd  */

  kfd_ioctl_dbg_trap_args args{};
  args.gpu_id = agent.gpu_id ();
  args.data1 = 1; /* enable  */

  int err = dbg_trap_ioctl (KFD_IOC_DBG_TRAP_ENABLE, &args);
  if (err < 0)
    return AMD_DBGAPI_STATUS_ERROR;

  *poll_fd = args.data3;
  return AMD_DBGAPI_STATUS_SUCCESS;
}

amd_dbgapi_status_t
process_t::disable_debug_trap (const agent_t &agent)
{
  /* KFD_IOC_DBG_TRAP_ENABLE (#0):
     data1: [in] enable/disable (1/0)  */

  kfd_ioctl_dbg_trap_args args{};
  args.gpu_id = agent.gpu_id ();
  args.data1 = 0; /* disable  */

  int err = dbg_trap_ioctl (KFD_IOC_DBG_TRAP_ENABLE, &args);
  if (err < 0)
    return AMD_DBGAPI_STATUS_ERROR;

  return AMD_DBGAPI_STATUS_SUCCESS;
}

amd_dbgapi_status_t
process_t::set_wave_launch_mode (const agent_t &agent, wave_launch_mode_t mode)
{
  /* KFD_IOC_DBG_TRAP_SET_WAVE_LAUNCH_MODE (#2)
     data1: mode (0=normal, 1=halt, 2=kill, 3=single-step, 4=disable)  */

  kfd_ioctl_dbg_trap_args args{};
  args.gpu_id = agent.gpu_id ();
  args.data1 = static_cast<std::underlying_type_t<decltype (mode)>> (mode);

  int err = dbg_trap_ioctl (KFD_IOC_DBG_TRAP_SET_WAVE_LAUNCH_MODE, &args);
  if (err < 0)
    return AMD_DBGAPI_STATUS_ERROR;

  return AMD_DBGAPI_STATUS_SUCCESS;
}

amd_dbgapi_status_t
process_t::query_debug_event (const agent_t &agent,
                              queue_t::kfd_queue_id_t *kfd_queue_id,
                              uint32_t *queue_status)
{
  dbgapi_assert (kfd_queue_id && queue_status && "must not be null");

  /* KFD_IOC_DBG_TRAP_QUERY_DEBUG_EVENT (#6):
     data1: [in/out] queue id
     data2: [in] flags
     data3: [out] new_queue[3:3], suspended[2:2], event_type [1:0]  */

  kfd_ioctl_dbg_trap_args args{};
  args.gpu_id = agent.gpu_id ();
  args.data1 = KFD_INVALID_QUEUEID;
  args.data2 = KFD_DBG_EV_FLAG_CLEAR_STATUS;

  int err = dbg_trap_ioctl (KFD_IOC_DBG_TRAP_QUERY_DEBUG_EVENT, &args);
  if (err == -EAGAIN)
    {
      /* There are no more events.  */
      *kfd_queue_id = KFD_INVALID_QUEUEID;
      *queue_status = 0;
      return AMD_DBGAPI_STATUS_SUCCESS;
    }
  else if (err < 0)
    return AMD_DBGAPI_STATUS_ERROR;

  *kfd_queue_id = args.data1;
  *queue_status = args.data3;

  return AMD_DBGAPI_STATUS_SUCCESS;
}

size_t
process_t::suspend_queues (const std::vector<queue_t *> &queues,
                           queue_t::update_waves_flag_t flags)
{
  if (queues.empty ())
    return 0;

  std::vector<queue_t::kfd_queue_id_t> queue_ids;
  queue_ids.reserve (queues.size ());

  for (auto *queue : queues)
    {
      dbgapi_assert (!queue->suspended () && "already suspended");
      queue_ids.emplace_back (queue->kfd_queue_id ());
      dbgapi_log (AMD_DBGAPI_LOG_LEVEL_INFO, "suspending %s",
                  to_string (queue->id ()).c_str ());
    }

  /* KFD_IOC_DBG_TRAP_NODE_SUSPEND (#4):
     data1: [in] flags
     data2: [in] number of queues
     data3: [in] grace period
     ptr:   [in] queue ids  */

  kfd_ioctl_dbg_trap_args args{};
  args.data1 = KFD_DBG_EV_FLAG_CLEAR_STATUS;
  args.data2 = static_cast<uint32_t> (queue_ids.size ());
  args.ptr = reinterpret_cast<uint64_t> (queue_ids.data ());

  size_t num_suspended_queues
      = dbg_trap_ioctl (KFD_IOC_DBG_TRAP_NODE_SUSPEND, &args);

  if (num_suspended_queues < 0)
    error ("dbg_trap_ioctl (KFD_IOC_DBG_TRAP_NODE_SUSPEND) failed.");

  if (num_suspended_queues != queue_ids.size ())
    {
      /* Some queues may have failed to suspend because they no longer exist so
         check the queue_ids returned by KFD and invalidate queues which have
         been marked as invalid.  */
      bool __maybe_unused__ invalid_queue = false;

      for (size_t i = 0; i < queue_ids.size (); ++i)
        if (queue_ids[i] & KFD_DBG_QUEUE_ERROR_MASK)
          error ("failed to suspend %s",
                 to_string (queues[i]->id ()).c_str ());
        else if (queue_ids[i] & KFD_DBG_QUEUE_INVALID_MASK)
          {
            queues[i]->invalidate ();
            invalid_queue = true;
          }

      dbgapi_assert (invalid_queue && "should have seen an invalid queue");
    }

  /* Update the waves that have been context switched.  */
  for (auto *queue : queues)
    {
      if (!queue->is_valid ())
        continue;

      queue->set_suspended (true);
      if (queue->kfd_queue_type () == KFD_IOC_QUEUE_TYPE_COMPUTE_AQL)
        {
          amd_dbgapi_status_t status = queue->update_waves (flags);
          if (status != AMD_DBGAPI_STATUS_SUCCESS)
            warning ("%s update_waves failed (rc=%d)",
                     to_string (queue->id ()).c_str (), status);
        }
    }

  return num_suspended_queues;
}

size_t
process_t::resume_queues (const std::vector<queue_t *> &queues)
{
  if (queues.empty ())
    return 0;

  std::vector<queue_t::kfd_queue_id_t> queue_ids;
  queue_ids.reserve (queues.size ());

  for (auto *queue : queues)
    {
      dbgapi_assert (queue->suspended () && "queue is not suspended");
      queue_ids.emplace_back (queue->kfd_queue_id ());
      dbgapi_log (AMD_DBGAPI_LOG_LEVEL_INFO, "resuming %s",
                  to_string (queue->id ()).c_str ());
    }

  /* KFD_IOC_DBG_TRAP_NODE_RESUME (#5):
     data1: [in] flags
     data2: [in] number of queues
     ptr:   [in] queue ids  */

  kfd_ioctl_dbg_trap_args args{};
  args.data1 = 0;
  args.data2 = static_cast<uint32_t> (queue_ids.size ());
  args.ptr = reinterpret_cast<uint64_t> (queue_ids.data ());

  size_t num_resumed_queues
      = dbg_trap_ioctl (KFD_IOC_DBG_TRAP_NODE_RESUME, &args);

  if (num_resumed_queues < 0)
    error ("dbg_trap_ioctl (KFD_IOC_DBG_TRAP_NODE_RESUME) failed.");

  if (num_resumed_queues != queue_ids.size ())
    {
      /* Some queues may have failed to resume because they no longer exist so
         check the queue_ids returned by KFD and invalidate queues which have
         been marked as invalid.  */
      bool __maybe_unused__ invalid_queue = false;

      for (size_t i = 0; i < queue_ids.size (); ++i)
        if (queue_ids[i] & KFD_DBG_QUEUE_ERROR_MASK)
          error ("failed to resume %s", to_string (queues[i]->id ()).c_str ());
        else if (queue_ids[i] & KFD_DBG_QUEUE_INVALID_MASK)
          {
            queues[i]->invalidate ();
            invalid_queue = true;
          }

      dbgapi_assert (invalid_queue && "should have seen an invalid queue");
    }

  for (auto *queue : queues)
    if (queue->is_valid ())
      queue->set_suspended (false);

  return num_resumed_queues;
}

amd_dbgapi_status_t
process_t::update_queues ()
{
  epoch_t queue_mark;
  std::unique_ptr<kfd_queue_snapshot_entry[]> snapshots;
  uint32_t snapshot_count;

  /* Prime the queue count with the current number of queues.  */
  uint32_t queue_count = count<queue_t> ();

  do
    {
      /* Until we see all the snapshots, increase the epoch so that we can
         sweep queues that may have been destroyed between iterations.  */
      queue_mark = m_next_queue_mark++;

      /* We should allocate enough memory for the snapshots. Let's start with
         the current number of queues + 16.  */
      snapshot_count = queue_count + 16;
      snapshots.reset (new kfd_queue_snapshot_entry[snapshot_count]);

      /* KFD_IOC_DBG_TRAP_GET_QUEUE_SNAPSHOT (#7):
         data1: [in] flags
         data2: [in/out] number of queues snapshots
         ptr:   [in] user buffer  */

      kfd_ioctl_dbg_trap_args args{};
      args.data1 = 0;
      args.data2 = snapshot_count;
      args.ptr = reinterpret_cast<uint64_t> (snapshots.get ());

      int err = dbg_trap_ioctl (KFD_IOC_DBG_TRAP_GET_QUEUE_SNAPSHOT, &args);
      if (err < 0)
        return AMD_DBGAPI_STATUS_ERROR;

      /* KFD writes up to snapshot_count queue snapshots, but returns the
         number of queues in the process so that we can check if we have
         allocated enough memory to hold all the snapshots.  */
      queue_count = args.data2;

      /* We have to process the snapshots returned by the ioctl now, even
         if the list is incomplete, because we only get notified once that
         a queue is new.  The new_queue bit gets cleared after each query.  */

      for (uint32_t i = 0; i < std::min (queue_count, snapshot_count); ++i)
        {
          const kfd_queue_snapshot_entry &queue_info = snapshots[i];
          amd_dbgapi_queue_id_t reuse_queue_id = AMD_DBGAPI_QUEUE_NONE;

          /* Skip non-AQL queues. In the future, we may want to support
             reporting other queue types.  */
          if (snapshots[i].queue_type != KFD_IOC_QUEUE_TYPE_COMPUTE_AQL)
            continue;

          /* Find the queue by matching its kfd_queue_id with the one
             returned by the ioctl.  */
          queue_t *queue = find_if ([&] (const queue_t &x) {
            return x.kfd_queue_id () == queue_info.queue_id;
          });

          if (queue_info.queue_status & KFD_DBG_EV_STATUS_NEW_QUEUE)
            {
              /* If there is a stale queue with the same kfd_queue_id,
                 destroy it.  */
              if (queue)
                destroy (queue);
            }
          else if (m_initialized)
            {
              /* We should always have a valid queue for a given kfd_queue_id
                 after the process is initialized.  Not finding the queue means
                 that we either did not create a queue when a new queue_id was
                 reported (we consumed the event without action), or KFD did
                 not report the new queue.  */
              if (!queue)
                error (
                    "kfd_queue_id %d should have been reported as a NEW_QUEUE "
                    "before",
                    queue_info.queue_id);

              /* FIXME: If we could select which flags get cleared by the
                 query_debug_event ioctl, we would not need to create a
                 partially initialized queue in agent_t::next_kfd_event, and
                 fix it here with the information contained in the queue
                 snapshots.  */

              /* If the queue mark is null, the queue was created outside of
                 update_queues, and it does not have all the information yet
                 filled in.  */
              if (!queue->mark ())
                {
                  /* This is a partially initialized queue, re-create a fully
                     initialized instance with the same kfd_queue_id.  */
                  reuse_queue_id = amd_dbgapi_queue_id_t{ queue->id () };
                  destroy (queue);
                }
              else
                {
                  dbgapi_assert (
                      !!(queue_info.queue_status & KFD_DBG_EV_STATUS_SUSPENDED)
                          == queue->suspended ()
                      && "queue state does not match queue_info");

                  /* This isn't a new queue, and it is fully initialized.
                     mark it as active, and continue to the next snapshot.  */
                  queue->set_mark (queue_mark);
                  continue;
                }
            }

          /* The queue could be new to us, and not have the new bit set if
             the process was previously attached and detached.  In that case,
             when !m_initialized, we always create a new queue_t for every
             queue_id reported by the snapshot ioctl.  */

          /* Find the agent for this queue. */
          agent_t *agent = find_if ([&] (const agent_t &x) {
            return x.gpu_id () == queue_info.gpu_id;
          });

          /* Skip queues for agents that are not visible in this
             process.  TODO: investigate when this could happen, e.g.
             the debugger cgroups not matching the application cgroups?  */
          if (!agent)
            {
              dbgapi_log (AMD_DBGAPI_LOG_LEVEL_INFO,
                          "could not find an agent (gpu_id=%d)",
                          queue_info.gpu_id);
              continue;
            }

          /* create<queue_t> will allocate a new queue_id if reuse_queue_id
             is {0}.  */
          create<queue_t> (reuse_queue_id, /* queue_id */
                           *agent,         /* agent */
                           queue_info)     /* queue_info */
              .set_mark (queue_mark);
        }
    }
  while (queue_count > snapshot_count);

  /* Iterate all queues belonging to this process, and prune those with a mark
     older than the current mark.  */

  auto &&queue_range = range<queue_t> ();
  for (auto it = queue_range.begin (); it != queue_range.end ();)
    if (it->mark () < queue_mark)
      it = destroy (it);
    else
      ++it;

  return AMD_DBGAPI_STATUS_SUCCESS;
}

amd_dbgapi_status_t
process_t::update_code_objects ()
{
  constexpr size_t URI_NAME_MAX_SIZE = PATH_MAX + sizeof ("file://") - 1
                                       + sizeof ("#offset=0x&size=0x") - 1
                                       + 32;
  epoch_t code_object_mark = m_next_code_object_mark++;

  decltype (r_debug::r_state) state;
  if (read_global_memory (m_r_debug_address + offsetof (r_debug, r_state),
                          &state, sizeof (state))
      != AMD_DBGAPI_STATUS_SUCCESS)
    error ("read_global_memory failed");

  /* If the state is not RT_CONSISTENT then that indicates there is a thread
     actively updating the code object list.  We cannot read the list as it is
     not consistent. But once the thread completes the update it will set state
     back to RT_CONSISTENT and hit the exiting breakpoint in the r_brk function
     which will trigger a read of the code object list.  */
  if (state != r_debug::RT_CONSISTENT)
    return AMD_DBGAPI_STATUS_SUCCESS;

  amd_dbgapi_global_address_t link_map_address;
  if (read_global_memory (m_r_debug_address + offsetof (r_debug, r_map),
                          &link_map_address, sizeof (link_map_address))
      != AMD_DBGAPI_STATUS_SUCCESS)
    error ("read_global_memory failed");

  while (link_map_address)
    {
      amd_dbgapi_global_address_t load_address;
      if (read_global_memory (link_map_address + offsetof (link_map, l_addr),
                              &load_address, sizeof (load_address))
          != AMD_DBGAPI_STATUS_SUCCESS)
        error ("read_global_memory failed");

      amd_dbgapi_global_address_t l_name_address;
      if (read_global_memory (link_map_address + offsetof (link_map, l_name),
                              &l_name_address, sizeof (l_name_address))
          != AMD_DBGAPI_STATUS_SUCCESS)
        error ("read_global_memory failed");

      std::string uri;
      if (read_string (l_name_address, &uri, URI_NAME_MAX_SIZE)
          != AMD_DBGAPI_STATUS_SUCCESS)
        error ("read_string failed");

      /* Check if the code object already exists.  */
      code_object_t *code_object = find_if ([&] (const code_object_t &x) {
        /* FIXME: We have an ABA problem for memory based code objects. A new
           code object of the same size could have been loaded at the same
           address as an old stale code object. We could add a unique
           identifier to the URI.  */
        return x.load_address () == load_address && x.uri () == uri;
      });

      if (!code_object)
        code_object = &create<code_object_t> (*this, uri, load_address);

      code_object->set_mark (code_object_mark);

      if (read_global_memory (link_map_address + offsetof (link_map, l_next),
                              &link_map_address, sizeof (link_map_address))
          != AMD_DBGAPI_STATUS_SUCCESS)
        error ("read_global_memory failed");
    }

  /* Iterate all the code objects in this process, and prune those with a mark
     older than the current mark.  */
  for (auto code_object_it = range<code_object_t> ().begin ();
       code_object_it != range<code_object_t> ().end ();)
    {
      if (code_object_it->mark () < code_object_mark)
        code_object_it = destroy (code_object_it);
      else
        ++code_object_it;
    }

  return AMD_DBGAPI_STATUS_SUCCESS;
}

amd_dbgapi_status_t
process_t::attach ()
{
  /* Check that the KFD major == IOCTL major, and KFD minor >= IOCTL minor.  */
  kfd_ioctl_get_version_args get_version_args{};
  if (ioctl (m_kfd_fd, AMDKFD_IOC_GET_VERSION, &get_version_args)
      || get_version_args.major_version != KFD_IOCTL_MAJOR_VERSION
      || get_version_args.minor_version < KFD_IOCTL_MINOR_VERSION)
    {
      warning ("KFD ioctl version %d.%d does not match %d.%d+ requirement",
               get_version_args.major_version, get_version_args.minor_version,
               KFD_IOCTL_MAJOR_VERSION, KFD_IOCTL_MINOR_VERSION);
      return AMD_DBGAPI_STATUS_ERROR_VERSION_MISMATCH;
    }

  /* KFD_IOC_DBG_TRAP_GET_VERSION (#8)
     data1: [out] major version
     data2: [out] minor version */

  kfd_ioctl_dbg_trap_args dbg_trap_args{};
  dbg_trap_args.pid = static_cast<uint32_t> (getpid ());
  dbg_trap_args.op = KFD_IOC_DBG_TRAP_GET_VERSION;

  /* - Enabling debug mode before the target process opens the KFD device
       requires KFD_IOCTL_DBG >= 1.1
     - Clearing the queue status on queue suspend requires version >= 1.3
     - 1.4 Fixes an issue with kfifo free exposed by a user mode change.
     - 2.0 Returns number of queues suspended/resumed and invalid array slots
   */
  static_assert (KFD_IOCTL_DBG_MAJOR_VERSION > 1
                     || (KFD_IOCTL_DBG_MAJOR_VERSION == 2
                         && KFD_IOCTL_DBG_MINOR_VERSION >= 0),
                 "KFD_IOCTL_DBG >= 1.4 required");

  /* Check that the KFD dbg trap major == IOCTL dbg trap major,
     and KFD dbg trap minor >= IOCTL dbg trap minor.  */
  if (ioctl (m_kfd_fd, AMDKFD_IOC_DBG_TRAP, &dbg_trap_args)
      || dbg_trap_args.data1 != KFD_IOCTL_DBG_MAJOR_VERSION
      || dbg_trap_args.data2 < KFD_IOCTL_DBG_MINOR_VERSION)
    {
      warning (
          "KFD dbg trap ioctl version %d.%d does not match %d.%d+ requirement",
          dbg_trap_args.data1, dbg_trap_args.data2,
          KFD_IOCTL_DBG_MAJOR_VERSION, KFD_IOCTL_DBG_MINOR_VERSION);
      return AMD_DBGAPI_STATUS_ERROR_VERSION_MISMATCH;
    }

  auto on_rocr_load_callback = [this] (const shared_library_t &library) {
    amd_dbgapi_status_t status;

    status = update_agents (true);
    if (status != AMD_DBGAPI_STATUS_SUCCESS)
      error ("update_agents failed (rc=%d)", status);

    status = update_queues ();
    if (status != AMD_DBGAPI_STATUS_SUCCESS)
      error ("update_queues failed (rc=%d)", status);

    std::vector<queue_t *> queues;
    for (auto &&queue : range<queue_t> ())
      queues.emplace_back (&queue);

    /* Suspend the newly create queues to update the waves, then resume them.
       We could have attached to the process while wavefronts were executing.
     */
    suspend_queues (queues,
                    queue_t::update_waves_flag_t::FORCE_ASSIGN_WAVE_IDS);
    resume_queues (queues);

    status = start_event_thread ();
    if (status != AMD_DBGAPI_STATUS_SUCCESS)
      error ("Cannot start the event thread (rc=%d)", status);

    /* Retrieve the address of the rendez-vous structure (_amd_gpu_r_debug)
       used by the ROCm Runtime Loader to communicate details of code objects
       loading to the debugger.  */
    constexpr char amdgpu_r_debug_symbol_name[] = "_amdgpu_r_debug";
    if (get_symbol_address (library.id (), amdgpu_r_debug_symbol_name,
                            &m_r_debug_address)
        != AMD_DBGAPI_STATUS_SUCCESS)
      error ("Cannot find symbol `%s'", amdgpu_r_debug_symbol_name);

    /* Check the r_version.  */
    int r_version;
    if (read_global_memory (m_r_debug_address
                                + offsetof (struct r_debug, r_version),
                            &r_version, sizeof (r_version))
        != AMD_DBGAPI_STATUS_SUCCESS)
      error ("read_global_memory failed");

    if (r_version != ROCR_RDEBUG_VERSION)
      {
        warning ("%s: _amdgpu_r_debug.r_version not supported, "
                 "expected %d got %d.",
                 library.name ().c_str (), ROCR_RDEBUG_VERSION, r_version);
        enqueue_event (
            create<event_t> (*this, AMD_DBGAPI_EVENT_KIND_RUNTIME,
                             AMD_DBGAPI_RUNTIME_STATE_LOADED_UNSUPPORTED));
        return;
      }

    /* Install a breakpoint at _amd_r_debug.r_brk.  The ROCm Runtime calls
       this function before updating the code object list, and after completing
       updating the code object list.  */

    amd_dbgapi_global_address_t r_brk_address;
    if (read_global_memory (m_r_debug_address
                                + offsetof (struct r_debug, r_brk),
                            &r_brk_address, sizeof (r_brk_address))
        != AMD_DBGAPI_STATUS_SUCCESS)
      error ("read_global_memory failed");

    /* This function gets called when the client reports that the breakpoint
       has been hit.  */
    auto r_brk_callback
        = [this] (breakpoint_t &breakpoint,
                  amd_dbgapi_client_thread_id_t client_thread_id,
                  amd_dbgapi_breakpoint_action_t *action) {
            update_code_objects ();

            /* Create a breakpoint resume event that will be enqueued when the
               code object list updated event is reported as processed.  This
               will allow the client thread to resume execution.  */
            event_t &breakpoint_resume_event = create<event_t> (
                *this, AMD_DBGAPI_EVENT_KIND_BREAKPOINT_RESUME,
                breakpoint.id (), client_thread_id);

            /* Enqueue a code object list updated event.  */
            enqueue_event (create<event_t> (
                *this, AMD_DBGAPI_EVENT_KIND_CODE_OBJECT_LIST_UPDATED,
                breakpoint_resume_event.id ()));

            /* Tell the client thread that it cannot resume execution until it
               sees the breakpoint resume event for this breakpoint_id and
               report it as processed.  */
            *action = AMD_DBGAPI_BREAKPOINT_ACTION_HALT;
            return AMD_DBGAPI_STATUS_SUCCESS;
          };

    create<breakpoint_t> (library, r_brk_address, r_brk_callback);

    enqueue_event (
        create<event_t> (*this, AMD_DBGAPI_EVENT_KIND_RUNTIME,
                         AMD_DBGAPI_RUNTIME_STATE_LOADED_SUPPORTED));

    update_code_objects ();

    if (count<code_object_t> ())
      enqueue_event (create<event_t> (
          *this, AMD_DBGAPI_EVENT_KIND_CODE_OBJECT_LIST_UPDATED,
          AMD_DBGAPI_EVENT_NONE));
  };

  auto on_rocr_unload_callback = [this] (const shared_library_t &library) {
    process_t &process = library.process ();

    /* Remove the breakpoints we've inserted when the library was loaded.  */
    auto &&breakpoint_range = process.range<breakpoint_t> ();
    for (auto it = breakpoint_range.begin (); it != breakpoint_range.end ();)
      if (it->shared_library ().id () == library.id ())
        it = process.destroy (it);
      else
        ++it;

    /* Destruct the code objects.  */
    std::get<handle_object_set_t<code_object_t>> (m_handle_object_sets)
        .clear ();

    enqueue_event (
        create<event_t> (*this, AMD_DBGAPI_EVENT_KIND_CODE_OBJECT_LIST_UPDATED,
                         AMD_DBGAPI_EVENT_NONE));

    enqueue_event (create<event_t> (*this, AMD_DBGAPI_EVENT_KIND_RUNTIME,
                                    AMD_DBGAPI_RUNTIME_STATE_UNLOADED));
  };

  /* Set/remove internal breakpoints when the ROCm Runtime is loaded/unloaded.
   */
  const shared_library_t &library = create<shared_library_t> (
      *this, "/libhsa-runtime64.so.1", on_rocr_load_callback,
      on_rocr_unload_callback);

  /* If the ROCm Runtime is not yet loaded, create agents without enabling the
     debug trap.  */
  if (library.state () != AMD_DBGAPI_SHARED_LIBRARY_STATE_LOADED)
    {
      amd_dbgapi_status_t status = update_agents (false);
      if (status != AMD_DBGAPI_STATUS_SUCCESS)
        return status;
    }

  m_initialized = true;
  return AMD_DBGAPI_STATUS_SUCCESS;
}

static void
event_thread_loop (std::vector<file_desc_t> file_descriptors,
                   std::vector<std::atomic<bool> *> notifiers,
                   pipe_t client_notifier_pipe, pipe_t event_thread_exit_pipe,
                   std::promise<void> thread_exception)
{
  std::vector<struct pollfd> poll_fds;

  poll_fds.reserve (file_descriptors.size ());
  std::transform (file_descriptors.begin (), file_descriptors.end (),
                  std::back_inserter (poll_fds), [] (file_desc_t fd) {
                    return pollfd{ fd, POLLIN, 0 };
                  });

  try
    {
      while (true)
        {
          bool process_has_events = false;
          int ret;

          ret = poll (poll_fds.data (), poll_fds.size (), -1);

          if (ret == -1 && errno != EINTR)
            error ("poll: %s", strerror (errno));
          else if (ret <= 0)
            continue;

          /* Check and flush the KFD event pipes.  */
          const size_t count = poll_fds.size ();
          for (size_t i = 0; i < count; ++i)
            {
              struct pollfd &poll_fd = poll_fds[i];

              if (!(poll_fd.revents & POLLIN))
                continue;

              /* If the exit pipe has data, the process is about to be
                 destroyed and we must exit this thread.  */
              if (poll_fd.fd == event_thread_exit_pipe.read_fd ())
                {
                  event_thread_exit_pipe.flush ();
                  return;
                }

              /* Consume all the data in the event pipe, and set
                 the notifier.  */
              do
                {
                  char buf;
                  ret = read (poll_fd.fd, &buf, 1);
                }
              while (ret >= 0 || (ret == -1 && errno == EINTR));

              if (ret == -1 && errno != EAGAIN)
                error ("read: %s", strerror (errno));

              dbgapi_assert (notifiers[i]);
              notifiers[i]->store (true, std::memory_order_relaxed);

              process_has_events = true;
            }

          /* If we read any byte from the KFD event pipes, then notify the
             client application that this process has some pending events.  */
          if (process_has_events)
            client_notifier_pipe.mark ();
        }
    }
  catch (...)
    {
      thread_exception.set_exception (std::current_exception ());
    }
}

amd_dbgapi_status_t
process_t::start_event_thread ()
{
  sigset_t new_mask;
  sigset_t orig_mask;

  /* Make sure another thread is not running.  */
  if (m_event_thread)
    stop_event_thread ();

  /* We want the event thread to inherit an all-signals-blocked mask.  */
  sigfillset (&new_mask);
  if (pthread_sigmask (SIG_SETMASK, &new_mask, &orig_mask))
    {
      warning ("pthread_sigmask failed: %s", strerror (errno));
      return AMD_DBGAPI_STATUS_ERROR;
    }

  /* Create a pipe that we can use to wake up ::poll, and make the event
     thread exit.  */
  if (!m_event_thread_exit_pipe.open ())
    {
      warning ("Could not create exit pipe: %s", strerror (errno));
      return AMD_DBGAPI_STATUS_ERROR;
    }

  /* TODO: To save resources (threads, file descriptors), we could start only
     one event thread for the entire host process, and listen on the file
     descriptors for all the agents in all processes, then individually notify
     the processes with events.  We would need to notify the event thread when
     processes are attached or detached so that the list of polled file
     descriptors is updated.  */

  std::vector<file_desc_t> file_descriptors;
  std::vector<std::atomic<bool> *> notifiers;

  for (auto &&agent : range<agent_t> ())
    {
      file_descriptors.emplace_back (agent.poll_fd ());
      notifiers.emplace_back (&agent.kfd_event_notifier ());
    }

  /* Then add the read end of the exit_pipe.  */
  file_descriptors.emplace_back (m_event_thread_exit_pipe.read_fd ());
  notifiers.emplace_back (nullptr);

  std::promise<void> event_thread_exception;
  m_event_thread_exception = event_thread_exception.get_future ();

  /* Start a new event thread. We capture a snapshot of the file descriptors
     to monitor, and associated atomic boolean notifiers. If agents were to be
     added to or removed from the agent_map, we would stop this thread and
     start a new event thread with a new set of file descriptors,
     and notifiers. */

  m_event_thread = new std::thread (
      event_thread_loop, std::move (file_descriptors), std::move (notifiers),
      m_client_notifier_pipe, m_event_thread_exit_pipe,
      std::move (event_thread_exception));

  if (pthread_sigmask (SIG_SETMASK, &orig_mask, nullptr))
    {
      warning ("pthread_sigmask failed: %s", strerror (errno));
      return AMD_DBGAPI_STATUS_ERROR;
    }

  return AMD_DBGAPI_STATUS_SUCCESS;
}

amd_dbgapi_status_t
process_t::stop_event_thread ()
{
  int ret;

  if (!m_event_thread)
    return AMD_DBGAPI_STATUS_SUCCESS;

  if (!m_event_thread_exit_pipe.is_valid ())
    return AMD_DBGAPI_STATUS_ERROR;

  /* Send a termination request to the event thread.  */
  if ((ret = m_event_thread_exit_pipe.mark ()))
    {
      warning ("exit_pipe mark failed (rc=%d)", ret);
      return AMD_DBGAPI_STATUS_ERROR;
    }

  /* Wait for the event thread to terminate.  */
  m_event_thread->join ();

  delete m_event_thread;
  m_event_thread = nullptr;
  m_event_thread_exit_pipe.close ();

  return AMD_DBGAPI_STATUS_SUCCESS;
}

void
process_t::check_event_thread ()
{
  /* Check for exceptions in the event thread, and rethrow in the
     application thread.  */
  if (m_event_thread_exception.valid ()
      && m_event_thread_exception.wait_for (std::chrono::seconds (0))
             == std::future_status::ready)
    m_event_thread_exception.get ();
}

amd_dbgapi_status_t
process_t::get_info (amd_dbgapi_process_info_t query, size_t value_size,
                     void *value) const
{
  switch (query)
    {
    case AMD_DBGAPI_PROCESS_INFO_NOTIFIER:
      return utils::get_info (value_size, value,
                              m_client_notifier_pipe.read_fd ());

    default:
      return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;
    }

  return AMD_DBGAPI_STATUS_SUCCESS;
}

void
process_t::enqueue_event (event_t &event)
{
  dbgapi_log (AMD_DBGAPI_LOG_LEVEL_INFO, "enqueue %s: %s",
              to_string (event.id ()).c_str (),
              event.pretty_printer_string ().c_str ());

  m_pending_events.emplace (&event);

  /* Notify the client that a new event is available.  */
  client_notifier_pipe ().mark ();
}

event_t *
process_t::dequeue_event ()
{
  /* TODO: If supporting multi-threaded next_pending_event, we should
     synchronize here.  */

  if (m_pending_events.empty ())
    return nullptr;

  event_t *next_event = m_pending_events.front ();
  m_pending_events.pop ();

  return next_event;
}

} /* namespace dbgapi */
} /* namespace amd */

using namespace amd::dbgapi;

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_process_set_progress (amd_dbgapi_process_id_t process_id,
                                 amd_dbgapi_progress_t progress)
{
  TRY;
  TRACE (process_id, progress);

  if (!amd::dbgapi::is_initialized)
    return AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED;

  process_t *process = process_t::find (process_id);

  if (!process)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_PROCESS_ID;

  switch (progress)
    {
    case AMD_DBGAPI_PROGRESS_NORMAL:
      process->set_forward_progress_needed (true);
      break;
    case AMD_DBGAPI_PROGRESS_NO_FORWARD:
      process->set_forward_progress_needed (false);
      break;
    default:
      return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;
    }

  return AMD_DBGAPI_STATUS_SUCCESS;
  CATCH;
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_process_set_wave_creation (amd_dbgapi_process_id_t process_id,
                                      amd_dbgapi_wave_creation_t creation)
{
  TRY;
  TRACE (process_id, creation);

  if (!amd::dbgapi::is_initialized)
    return AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED;

  process_t *process = process_t::find (process_id);

  if (!process)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_PROCESS_ID;

  switch (creation)
    {
    case AMD_DBGAPI_WAVE_CREATION_NORMAL:
      return process->set_wave_launch_mode (
          process_t::wave_launch_mode_t::NORMAL);
    case AMD_DBGAPI_WAVE_CREATION_STOP:
      return process->set_wave_launch_mode (
          process_t::wave_launch_mode_t::HALT);
    default:
      return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;
    }

  CATCH;
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_process_attach (amd_dbgapi_client_process_id_t client_process_id,
                           amd_dbgapi_process_id_t *process_id)
{
  TRY;
  TRACE (client_process_id, process_id);

  /* Start the process_ids at 1, so that 0 is reserved for invalid id.  */
  static monotonic_counter_t<decltype (amd_dbgapi_process_id_t::handle)>
      next_process_id = { 1 };

  if (!amd::dbgapi::is_initialized)
    return AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED;

  if (!client_process_id || !process_id)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;

  /* Return an error if the client_process_id is already attached to another
     process instance.  */
  if (process_t::find (client_process_id))
    return AMD_DBGAPI_STATUS_ERROR_ALREADY_ATTACHED;

  amd_dbgapi_process_id_t id;
  try
    {
      id = amd_dbgapi_process_id_t{ next_process_id++ };
    }
  catch (const exception_t &ex)
    {
      next_process_id = { 1 };
      throw;
    }

  auto process = std::make_unique<process_t> (client_process_id, id);
  if (!process->is_valid ())
    return AMD_DBGAPI_STATUS_ERROR;

  amd_dbgapi_status_t status = process->attach ();
  if (status != AMD_DBGAPI_STATUS_SUCCESS)
    return status;

  *process_id = amd_dbgapi_process_id_t{ process->id () };

  /* Append the new process to the process_list and return.  */
  process_list.push_back (process.release ());

  return AMD_DBGAPI_STATUS_SUCCESS;
  CATCH;
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_process_detach (amd_dbgapi_process_id_t process_id)
{
  TRY;
  TRACE (process_id);

  if (!amd::dbgapi::is_initialized)
    return AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED;

  process_t *process
      = process_t::find (process_id, true); /* Flush the cache.  */

  if (!process)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_PROCESS_ID;

  /* Flush the cache in case the process is re-attached later on. */
  process_t::find (process->client_id (), true);

  process->detach ();

  process_list.remove (process);
  delete process;

  return AMD_DBGAPI_STATUS_SUCCESS;
  CATCH;
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_process_get_info (amd_dbgapi_process_id_t process_id,
                             amd_dbgapi_process_info_t query,
                             size_t value_size, void *value)
{
  TRY;
  TRACE (process_id, query, value_size, value);

  if (!amd::dbgapi::is_initialized)
    return AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED;

  process_t *process = process_t::find (process_id);

  if (!process)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_PROCESS_ID;

  return process->get_info (query, value_size, value);
  CATCH;
}
