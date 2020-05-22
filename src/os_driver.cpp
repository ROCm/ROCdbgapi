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
#include "agent.h"
#include "debug.h"
#include "logging.h"
#include "process.h"

#include <limits>
#include <type_traits>

#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

namespace amd
{
namespace dbgapi
{

namespace
{

size_t kfd_open_count{ 0 };
file_desc_t kfd_fd{ -1 };

/* Open the KFD device. The file descriptor is reference counted, multiple
   calls to open_kfd are allowed, as long as the same number of open_kfd and
   close_kfd are called.  The last call to close_kfd closes the device.  */

file_desc_t
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

int
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

} /* namespace */

os_driver_t::os_driver_t (process_t &process) : m_process (process)
{
  /* Open the /dev/kfd device.  */
  if ((m_kfd_fd = open_kfd ()) == -1)
    warning ("Could not open the KFD device: %s", strerror (errno));

  /* See is_valid() for information about how failing to open /dev/kfd
     is handled.  */
}

os_driver_t::~os_driver_t ()
{
  if (m_kfd_fd != -1 && close_kfd ())
    error ("Could not close the KFD device");
}

int
os_driver_t::kfd_dbg_trap_ioctl (uint32_t action,
                                 kfd_ioctl_dbg_trap_args *args) const
{
  if (m_process.is_flag_set (process_t::flag_t::CLIENT_PROCESS_HAS_EXITED))
    return -ESRCH;

  args->pid = m_process.os_pid ();
  args->op = action;

  int ret = ::ioctl (m_kfd_fd, AMDKFD_IOC_DBG_TRAP, args);
  if (ret < 0 && errno == ESRCH)
    {
      /* The target process does not exist, it must have exited.  */
      m_process.set_flag (process_t::flag_t::CLIENT_PROCESS_HAS_EXITED);
      /* FIXME: We should tear down the process now, so that any operation
         executed after this point returns an error.  */
      return -ESRCH;
    }

  return ret < 0 ? -errno : ret;
}

amd_dbgapi_status_t
os_driver_t::get_version (uint32_t *major, uint32_t *minor) const
{
  dbgapi_assert (major && minor && "must not be null");

  /* Check that the KFD major == IOCTL major, and KFD minor >= IOCTL minor.  */
  kfd_ioctl_get_version_args get_version_args{};
  if (::ioctl (m_kfd_fd, AMDKFD_IOC_GET_VERSION, &get_version_args)
      || get_version_args.major_version != KFD_IOCTL_MAJOR_VERSION
      || get_version_args.minor_version < KFD_IOCTL_MINOR_VERSION)
    {
      warning ("ioctl version %d.%d does not match %d.%d+ requirement",
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

  int err = ::ioctl (m_kfd_fd, AMDKFD_IOC_DBG_TRAP, &dbg_trap_args);
  if (err < 0)
    return AMD_DBGAPI_STATUS_ERROR;

  *major = dbg_trap_args.data1;
  *minor = dbg_trap_args.data2;

  return AMD_DBGAPI_STATUS_SUCCESS;
}

amd_dbgapi_status_t
os_driver_t::enable_debug_trap (const agent_t &agent,
                                file_desc_t *poll_fd) const
{
  dbgapi_assert (poll_fd && "must not be null");

  /* KFD_IOC_DBG_TRAP_ENABLE (#0):
     data1: [in] enable/disable (1/0)
     data3: [out] poll_fd  */

  kfd_ioctl_dbg_trap_args args{};
  args.gpu_id = agent.gpu_id ();
  args.data1 = 1; /* enable  */

  int err = kfd_dbg_trap_ioctl (KFD_IOC_DBG_TRAP_ENABLE, &args);
  if (err < 0)
    return AMD_DBGAPI_STATUS_ERROR;

  *poll_fd = args.data3;
  return AMD_DBGAPI_STATUS_SUCCESS;
}

amd_dbgapi_status_t
os_driver_t::disable_debug_trap (const agent_t &agent) const
{
  /* KFD_IOC_DBG_TRAP_ENABLE (#0):
     data1: [in] enable/disable (1/0)  */

  kfd_ioctl_dbg_trap_args args{};
  args.gpu_id = agent.gpu_id ();
  args.data1 = 0; /* disable  */

  int err = kfd_dbg_trap_ioctl (KFD_IOC_DBG_TRAP_ENABLE, &args);
  if (err < 0)
    return AMD_DBGAPI_STATUS_ERROR;

  return AMD_DBGAPI_STATUS_SUCCESS;
}

amd_dbgapi_status_t
os_driver_t::set_wave_launch_mode (const agent_t &agent,
                                   os_wave_launch_mode_t mode) const
{
  /* KFD_IOC_DBG_TRAP_SET_WAVE_LAUNCH_MODE (#2)
     data1: mode (0=normal, 1=halt, 2=kill, 3=single-step, 4=disable)  */

  kfd_ioctl_dbg_trap_args args{};
  args.gpu_id = agent.gpu_id ();
  args.data1 = static_cast<std::underlying_type_t<decltype (mode)>> (mode);

  int err = kfd_dbg_trap_ioctl (KFD_IOC_DBG_TRAP_SET_WAVE_LAUNCH_MODE, &args);
  if (err < 0)
    return AMD_DBGAPI_STATUS_ERROR;

  return AMD_DBGAPI_STATUS_SUCCESS;
}

amd_dbgapi_status_t
os_driver_t::query_debug_event (const agent_t &agent,
                                os_queue_id_t *os_queue_id,
                                os_queue_status_t *os_queue_status) const
{
  dbgapi_assert (os_queue_id && os_queue_status && "must not be null");

  /* KFD_IOC_DBG_TRAP_QUERY_DEBUG_EVENT (#6):
     data1: [in/out] queue id
     data2: [in] flags
     data3: [out] new_queue[3:3], suspended[2:2], event_type [1:0]  */

  kfd_ioctl_dbg_trap_args args{};
  args.gpu_id = agent.gpu_id ();
  args.data1 = KFD_INVALID_QUEUEID;
  args.data2 = KFD_DBG_EV_FLAG_CLEAR_STATUS;

  int err = kfd_dbg_trap_ioctl (KFD_IOC_DBG_TRAP_QUERY_DEBUG_EVENT, &args);
  if (err == -EAGAIN)
    {
      /* There are no more events.  */
      *os_queue_id = KFD_INVALID_QUEUEID;
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
os_driver_t::suspend_queues (os_queue_id_t *queues, size_t queue_count) const
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

  return kfd_dbg_trap_ioctl (KFD_IOC_DBG_TRAP_NODE_SUSPEND, &args);
}

size_t
os_driver_t::resume_queues (os_queue_id_t *queues, size_t queue_count) const
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

  return kfd_dbg_trap_ioctl (KFD_IOC_DBG_TRAP_NODE_RESUME, &args);
}

amd_dbgapi_status_t
os_driver_t::queue_snapshot (os_queue_snapshot_entry_t *snapshots,
                             size_t snapshot_count, size_t *queue_count) const
{
  dbgapi_assert (queue_count && "must not be null");
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
  if (err < 0)
    return AMD_DBGAPI_STATUS_ERROR;

  /* KFD writes up to snapshot_count queue snapshots, but returns the
     number of queues in the process so that we can check if we have
     allocated enough memory to hold all the snapshots.  */
  *queue_count = args.data2;

  return AMD_DBGAPI_STATUS_SUCCESS;
}

} /* namespace dbgapi */
} /* namespace amd */
