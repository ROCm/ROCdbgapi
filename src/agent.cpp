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

#include "agent.h"
#include "architecture.h"
#include "debug.h"
#include "handle_object.h"
#include "initialization.h"
#include "logging.h"
#include "os_driver.h"
#include "process.h"
#include "queue.h"
#include "utils.h"

#include <cstdint>
#include <string>
#include <unistd.h>

namespace amd
{
namespace dbgapi
{

agent_t::agent_t (amd_dbgapi_agent_id_t agent_id, process_t &process,
                  const architecture_t &architecture,
                  const os_agent_snapshot_entry_t &os_agent_info)
    : handle_object (agent_id), m_os_agent_info (os_agent_info),
      m_architecture (architecture), m_process (process)
{
}

agent_t::~agent_t ()
{
  if (debug_trap_enabled ())
    disable_debug_trap ();
}

amd_dbgapi_status_t
agent_t::enable_debug_trap ()
{
  dbgapi_assert (!debug_trap_enabled () && "debug_trap is already enabled");
  file_desc_t fd;

  amd_dbgapi_status_t status
      = process ().os_driver ().enable_debug_trap (gpu_id (), &fd);
  if (status == AMD_DBGAPI_STATUS_SUCCESS)
    m_poll_fd.emplace (fd);

  return status;
}

amd_dbgapi_status_t
agent_t::disable_debug_trap ()
{
  dbgapi_assert (debug_trap_enabled () && "debug_trap is not enabled");

  amd_dbgapi_status_t status
      = process ().os_driver ().disable_debug_trap (gpu_id ());

  ::close (m_poll_fd.value ());
  m_poll_fd.reset ();

  return status;
}

amd_dbgapi_status_t
agent_t::next_os_event (amd_dbgapi_queue_id_t *queue_id,
                        os_queue_status_t *os_queue_status)
{
  dbgapi_assert (queue_id && os_queue_status && "must not be null");
  process_t &process = this->process ();

  while (true)
    {
      os_queue_id_t os_queue_id;

      amd_dbgapi_status_t status = process.os_driver ().query_debug_event (
          gpu_id (), &os_queue_id, os_queue_status);
      if (status != AMD_DBGAPI_STATUS_SUCCESS)
        return status;

      if (os_queue_id == OS_INVALID_QUEUEID)
        {
          /* There are no more events.  */
          *queue_id = AMD_DBGAPI_QUEUE_NONE;
          return AMD_DBGAPI_STATUS_SUCCESS;
        }

      /* Find the queue by matching its os_queue_id with the one
         returned by the ioctl.  */

      queue_t *queue = process.find_if ([os_queue_id] (const queue_t &q) {
        return q.os_queue_id () == os_queue_id;
      });

      /* If this is a new queue, update the queues to make sure we don't
         return a stale queue with the same os_queue_id.  */
      if ((*os_queue_status & os_queue_status_t::NEW_QUEUE) != 0)
        {
          /* If there is a stale queue with the same os_queue_id, destroy it.
           */
          if (queue)
            {
              amd_dbgapi_queue_id_t queue_id = queue->id ();
              os_queue_id_t os_queue_id = queue->os_queue_id ();

              process.destroy (queue);

              dbgapi_log (AMD_DBGAPI_LOG_LEVEL_INFO,
                          "destroyed stale %s (os_queue_id=%d)",
                          to_string (queue_id).c_str (), os_queue_id);
            }

          /* Create a temporary queue instance to reserve the unique queue_id,
             update_queues with fill in the missing information
             (os_queue_snapshot_entry_t).  */
          *queue_id = process.create<queue_t> (*this, os_queue_id).id ();
          *os_queue_status &= ~os_queue_status_t::NEW_QUEUE;
          process.update_queues ();

          /* Check that the queue still exists, update_queues () may have
             destroyed it if it isn't a supported queue type.  */
          if (process.find (*queue_id))
            return AMD_DBGAPI_STATUS_SUCCESS;

          dbgapi_log (AMD_DBGAPI_LOG_LEVEL_INFO,
                      "skipping event (%s) for deleted os_queue_id %d",
                      to_string (*os_queue_status).c_str (), os_queue_id);
        }
      else if (queue)
        {
          *queue_id = queue->id ();
          return AMD_DBGAPI_STATUS_SUCCESS;
        }
      else
        {
          error ("os_queue_id %d should have been reported as a NEW_QUEUE "
                 "before",
                 os_queue_id);
        }
    }
}

amd_dbgapi_status_t
agent_t::get_info (amd_dbgapi_agent_info_t query, size_t value_size,
                   void *value) const
{
  switch (query)
    {
    case AMD_DBGAPI_AGENT_INFO_NAME:
      return utils::get_info (value_size, value, m_os_agent_info.name);

    case AMD_DBGAPI_AGENT_INFO_ARCHITECTURE:
      return utils::get_info (value_size, value, architecture ().id ());

    case AMD_DBGAPI_AGENT_INFO_PCI_SLOT:
      return utils::get_info (value_size, value, m_os_agent_info.location_id);

    case AMD_DBGAPI_AGENT_INFO_PCI_VENDOR_ID:
      return utils::get_info (value_size, value, m_os_agent_info.vendor_id);

    case AMD_DBGAPI_AGENT_INFO_PCI_DEVICE_ID:
      return utils::get_info (value_size, value, m_os_agent_info.device_id);

    case AMD_DBGAPI_AGENT_INFO_SHADER_ENGINE_COUNT:
      return utils::get_info (value_size, value,
                              m_os_agent_info.shader_engine_count);

    case AMD_DBGAPI_AGENT_INFO_COMPUTE_UNIT_COUNT:
      return utils::get_info (value_size, value,
                              m_os_agent_info.simd_count
                                  / m_os_agent_info.simd_per_cu);

    case AMD_DBGAPI_AGENT_INFO_NUM_SIMD_PER_COMPUTE_UNIT:
      return utils::get_info (value_size, value, m_os_agent_info.simd_per_cu);

    case AMD_DBGAPI_AGENT_INFO_MAX_WAVES_PER_SIMD:
      return utils::get_info (value_size, value,
                              m_os_agent_info.max_waves_per_simd);
    }
  return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;
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

  if (!amd::dbgapi::is_initialized)
    return AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED;

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
