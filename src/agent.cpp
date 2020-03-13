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

#include "agent.h"
#include "architecture.h"
#include "debug.h"
#include "handle_object.h"
#include "linux/kfd_ioctl.h"
#include "logging.h"
#include "process.h"
#include "queue.h"
#include "utils.h"

#include <cstring>

#include <errno.h>
#include <sys/ioctl.h>
#include <unistd.h>

namespace amd
{
namespace dbgapi
{

agent_t::agent_t (amd_dbgapi_agent_id_t agent_id, process_t &process,
                  kfd_gpu_id_t gpu_id, const architecture_t &architecture,
                  const properties_t &properties)
    : handle_object (agent_id), m_gpu_id (gpu_id), m_properties (properties),
      m_architecture (architecture), m_process (process)
{
  enable_debug_trap ();
}

agent_t::~agent_t () { disable_debug_trap (); }

amd_dbgapi_status_t
agent_t::enable_debug_trap (void)
{
  process_t &process = this->process ();

  /* KFD_IOC_DBG_TRAP_ENABLE (#0):
     data1: [in] enable/disable (1/0)
     data3: [out] poll_fd  */

  kfd_ioctl_dbg_trap_args args = {
    .ptr = 0, /* unused  */
    .pid = static_cast<uint32_t> (process.os_pid ()),
    .gpu_id = gpu_id (),
    .op = KFD_IOC_DBG_TRAP_ENABLE,
    .data1 = 1 /* enable  */,
  };

  if (ioctl (process.kfd_fd (), AMDKFD_IOC_DBG_TRAP, &args))
    return AMD_DBGAPI_STATUS_ERROR;

  m_poll_fd = args.data3;

  return AMD_DBGAPI_STATUS_SUCCESS;
}

amd_dbgapi_status_t
agent_t::disable_debug_trap (void)
{
  process_t &process = this->process ();

  if (m_poll_fd != -1)
    {
      ::close (m_poll_fd);
      m_poll_fd = -1;
    }

  /* KFD_IOC_DBG_TRAP_ENABLE (#0):
     data1: [in] enable/disable (1/0)  */

  kfd_ioctl_dbg_trap_args args = {
    .ptr = 0, /* unused  */
    .pid = static_cast<uint32_t> (process.os_pid ()),
    .gpu_id = gpu_id (),
    .op = KFD_IOC_DBG_TRAP_ENABLE,
    .data1 = 0 /* disable  */,
  };

  if (ioctl (process.kfd_fd (), AMDKFD_IOC_DBG_TRAP, &args))
    return AMD_DBGAPI_STATUS_ERROR;

  return AMD_DBGAPI_STATUS_SUCCESS;
}

amd_dbgapi_status_t
agent_t::next_kfd_event (amd_dbgapi_queue_id_t *queue_id,
                         uint32_t *queue_status)
{
  dbgapi_assert (queue_id && queue_status && "must not be null");
  process_t &process = this->process ();

  while (true)
    {
      /* KFD_IOC_DBG_TRAP_QUERY_DEBUG_EVENT (#6):
         data1: [in/out] queue id
         data2: [in] flags
         data3: [out] new_queue[3:3], suspended[2:2], event_type [1:0]  */

      kfd_ioctl_dbg_trap_args args{
        .ptr = 0, /* unused  */
        .pid = static_cast<uint32_t> (process.os_pid ()),
        .gpu_id = gpu_id (),
        .op = KFD_IOC_DBG_TRAP_QUERY_DEBUG_EVENT,
        .data1 = static_cast<uint32_t> (-1),
        .data2 = KFD_DBG_EV_FLAG_CLEAR_STATUS,
      };

      int ret = ioctl (process.kfd_fd (), AMDKFD_IOC_DBG_TRAP, &args);
      if (ret == -1 && errno == EAGAIN)
        {
          /* There are no more events.  */
          *queue_id = { 0 };
          *queue_status = 0;
          return AMD_DBGAPI_STATUS_SUCCESS;
        }
      else if (ret < 0)
        return AMD_DBGAPI_STATUS_ERROR;

      const queue_t::kfd_queue_id_t kfd_queue_id = args.data1;
      const uint32_t status = args.data3;

      /* Find the queue by matching its kfd_queue_id with the one
         returned by the ioctl.  */

      queue_t *queue = process.find_if ([kfd_queue_id] (const queue_t &q) {
        return q.kfd_queue_id () == kfd_queue_id;
      });

      /* If this is a new queue, update the queues to make sure we don't
         return a stale queue with the same kfd_queue_id.  */
      if (status & KFD_DBG_EV_STATUS_NEW_QUEUE)
        {
          /* If there is a stale queue with the same kfd_queue_id, destroy it.
           */
          if (queue)
            process.destroy (queue);

          /* Create a "dummy" queue instance (with a null agent), to reserve
             the unique queue_id.  */
          kfd_queue_snapshot_entry queue_info{ 0 };
          queue_info.queue_id = kfd_queue_id;

          *queue_id = process.create<queue_t> (*this, queue_info).id ();
          *queue_status = status;

          /* Update the queues. This will fill in the queue information for
             the queue we've just created.  */
          process.update_queues ();

          /* Check that the queue still exists, update_queues () may have
             destroyed it if it isn't a supported queue type.  */
          if (process.find (*queue_id))
            return AMD_DBGAPI_STATUS_SUCCESS;
        }
      else if (queue)
        {
          *queue_id = queue->id ();
          *queue_status = status;
          return AMD_DBGAPI_STATUS_SUCCESS;
        }

      dbgapi_log (AMD_DBGAPI_LOG_LEVEL_INFO,
                  "Skipping event for unsupported queue %d", kfd_queue_id);
    }
}

amd_dbgapi_status_t
agent_t::get_info (amd_dbgapi_agent_info_t query, size_t value_size,
                   void *value) const
{
  switch (query)
    {
    case AMD_DBGAPI_AGENT_INFO_PCIE_SLOT:
      return utils::get_info (value_size, value, m_properties.location_id);

    case AMD_DBGAPI_AGENT_INFO_NAME:
      return utils::get_info (value_size, value, m_properties.name);

    case AMD_DBGAPI_AGENT_INFO_ARCHITECTURE:
      return utils::get_info (value_size, value, architecture ().id ());

    case AMD_DBGAPI_AGENT_INFO_SHADER_ENGINE_COUNT:
      return utils::get_info (value_size, value,
                              m_properties.shader_engine_count);

    case AMD_DBGAPI_AGENT_INFO_COMPUTE_UNIT_COUNT:
      return utils::get_info (value_size, value,
                              m_properties.simd_count
                                  / m_properties.simd_per_cu);

    case AMD_DBGAPI_AGENT_INFO_NUM_SIMD_PER_COMPUTE_UNIT:
      return utils::get_info (value_size, value, m_properties.simd_per_cu);

    case AMD_DBGAPI_AGENT_INFO_MAX_WAVES_PER_SIMD:
      return utils::get_info (value_size, value,
                              m_properties.max_waves_per_simd);

    default:
      return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;
    }

  return AMD_DBGAPI_STATUS_SUCCESS;
}

} /* namespace dbgapi */
} /* namespace amd */

using namespace amd::dbgapi;

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_agent_get_info (amd_dbgapi_process_id_t process_id,
                           amd_dbgapi_agent_id_t agent_id,
                           amd_dbgapi_agent_info_t query, size_t value_size,
                           void *value)
{
  TRY;
  TRACE (process_id, agent_id, query, value_size, value);

  process_t *process = process_t::find (process_id);

  if (!process)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_PROCESS_ID;

  agent_t *agent = process->find (agent_id);

  if (!agent)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_AGENT_ID;

  return agent->get_info (query, value_size, value);
  CATCH;
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_agent_list (amd_dbgapi_process_id_t process_id, size_t *agent_count,
                       amd_dbgapi_agent_id_t **agents,
                       amd_dbgapi_changed_t *changed)
{
  TRY;
  TRACE (process_id);

  if (!amd::dbgapi::is_initialized)
    return AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED;

  return utils::get_handle_list<agent_t> (process_id, agent_count, agents,
                                          changed);
  CATCH;
}
