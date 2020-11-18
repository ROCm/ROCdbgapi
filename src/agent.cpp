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
#include "watchpoint.h"

#include <algorithm>
#include <cstdint>
#include <optional>
#include <string>
#include <unistd.h>
#include <utility>
#include <vector>

namespace amd::dbgapi
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
  dbgapi_assert (m_watchpoint_map.empty ()
                 && "there should not be any active watchpoints left");

  if (is_debug_mode_enabled ())
    disable_debug_mode ();
}

amd_dbgapi_status_t
agent_t::enable_debug_mode ()
{
  dbgapi_assert (!is_debug_mode_enabled () && "debug_mode is already enabled");
  file_desc_t fd;

  if (m_os_agent_info.fw_version < m_os_agent_info.fw_version_required)
    {
      warning (
          "os_agent_id %d: firmware version %d does not match %d+ requirement",
          m_os_agent_info.os_agent_id, m_os_agent_info.fw_version,
          m_os_agent_info.fw_version_required);
      return AMD_DBGAPI_STATUS_ERROR_RESTRICTION;
    }

  amd_dbgapi_status_t status
      = process ().os_driver ().enable_debug_trap (os_agent_id (), &fd);
  if (status != AMD_DBGAPI_STATUS_SUCCESS)
    return status;

  m_event_poll_fd.emplace (fd);

  return AMD_DBGAPI_STATUS_SUCCESS;
}

amd_dbgapi_status_t
agent_t::disable_debug_mode ()
{
  dbgapi_assert (is_debug_mode_enabled () && "debug_mode is not enabled");

  amd_dbgapi_status_t status
      = process ().os_driver ().disable_debug_trap (os_agent_id ());

  m_supported_trap_mask.reset ();

  ::close (*m_event_poll_fd);
  m_event_poll_fd.reset ();

  return status;
}

void
agent_t::set_pending_events ()
{
  m_os_event_notifier.store (true, std::memory_order_release);
}

bool
agent_t::has_pending_events ()
{
  /* Use an atomic exchange here since the value may be updated by
     set_pending_events invoked by another thread (e.g. the
     process_event_thread).  */
  return m_os_event_notifier.exchange (false, std::memory_order_acquire);
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
          os_agent_id (), &os_queue_id, os_queue_status);
      if (status != AMD_DBGAPI_STATUS_SUCCESS)
        return status;

      if (os_queue_id == os_invalid_queueid)
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
      if ((*os_queue_status & os_queue_status_t::new_queue) != 0)
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
          *os_queue_status &= ~os_queue_status_t::new_queue;
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
          error ("os_queue_id %d should have been reported as a new_queue "
                 "before",
                 os_queue_id);
        }
    }
}

os_wave_launch_trap_mask_t
agent_t::supported_trap_mask ()
{
  dbgapi_assert (is_debug_mode_enabled () && "debug_mode is not enabled");

  if (!m_supported_trap_mask)
    {
      os_wave_launch_trap_mask_t trap_mask, ignored;
      if (process ().os_driver ().set_wave_launch_trap_override (
              os_agent_id (), os_wave_launch_trap_override_t::apply,
              os_wave_launch_trap_mask_t::none,
              os_wave_launch_trap_mask_t::none, &ignored, &trap_mask)
          != AMD_DBGAPI_STATUS_SUCCESS)
        return {};

      m_supported_trap_mask.emplace (trap_mask);
    }

  return *m_supported_trap_mask;
}

amd_dbgapi_status_t
agent_t::insert_watchpoint (const watchpoint_t &watchpoint,
                            amd_dbgapi_global_address_t adjusted_address,
                            amd_dbgapi_global_address_t adjusted_mask)
{
  std::optional<os_watch_mode_t> os_watch_mode
      = architecture ().watchpoint_mode (watchpoint.kind ());
  if (!os_watch_mode)
    {
      /* This agent does not support the requested watchpoint kind.  */
      return AMD_DBGAPI_STATUS_ERROR_NO_WATCHPOINT_AVAILABLE;
    }

  os_watch_id_t os_watch_id;
  amd_dbgapi_status_t status = process ().os_driver ().set_address_watch (
      os_agent_id (), adjusted_address, adjusted_mask, *os_watch_mode,
      &os_watch_id);
  if (status != AMD_DBGAPI_STATUS_SUCCESS)
    return status;

  dbgapi_log (
      AMD_DBGAPI_LOG_LEVEL_INFO, "%s: set address_watch%d [%#lx-%#lx] (%s)",
      to_string (id ()).c_str (), os_watch_id, adjusted_address,
      adjusted_address + (1 << utils::trailing_zeroes_count (adjusted_mask)),
      to_string (watchpoint.kind ()).c_str ());

  if (!m_watchpoint_map.emplace (os_watch_id, &watchpoint).second)
    error ("os_watch_id %d is already in use", os_watch_id);

  return AMD_DBGAPI_STATUS_SUCCESS;
}

amd_dbgapi_status_t
agent_t::remove_watchpoint (const watchpoint_t &watchpoint)
{
  /* Find watchpoint in the os_watch_id to watchpoint_t map.  The key will be
     the os_watch_id to clear for this agent.  */
  auto it = std::find_if (m_watchpoint_map.begin (), m_watchpoint_map.end (),
                          [&watchpoint] (const auto &value) {
                            return value.second == &watchpoint;
                          });

  if (it == m_watchpoint_map.end ())
    error ("watchpoint is not inserted");

  os_watch_id_t os_watch_id = it->first;

  amd_dbgapi_status_t status = process ().os_driver ().clear_address_watch (
      os_agent_id (), os_watch_id);
  if (status != AMD_DBGAPI_STATUS_SUCCESS
      && status != AMD_DBGAPI_STATUS_ERROR_PROCESS_EXITED)
    return status;

  m_watchpoint_map.erase (it);

  dbgapi_log (AMD_DBGAPI_LOG_LEVEL_INFO, "%s: clear address_watch%d",
              to_string (id ()).c_str (), os_watch_id);

  return status;
}

const watchpoint_t *
agent_t::find_watchpoint (os_watch_id_t os_watch_id) const
{
  auto it = m_watchpoint_map.find (os_watch_id);
  return it != m_watchpoint_map.end () ? it->second : nullptr;
}

amd_dbgapi_status_t
agent_t::get_info (amd_dbgapi_agent_info_t query, size_t value_size,
                   void *value) const
{
  switch (query)
    {
    case AMD_DBGAPI_AGENT_INFO_PROCESS:
      return utils::get_info (value_size, value, process ().id ());

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

    case AMD_DBGAPI_AGENT_INFO_EXECUTION_UNIT_COUNT:
      return utils::get_info (value_size, value, m_os_agent_info.simd_count);

    case AMD_DBGAPI_AGENT_INFO_MAX_WAVES_PER_EXECUTION_UNIT:
      return utils::get_info (value_size, value,
                              m_os_agent_info.max_waves_per_simd);

    case AMD_DBGAPI_AGENT_INFO_OS_ID:
      return utils::get_info (
          value_size, value,
          static_cast<amd_dbgapi_os_agent_id_t> (m_os_agent_info.os_agent_id));
    }
  return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;
}

} /* namespace amd::dbgapi */

using namespace amd::dbgapi;

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_agent_get_info (amd_dbgapi_agent_id_t agent_id,
                           amd_dbgapi_agent_info_t query, size_t value_size,
                           void *value)
{
  TRY;
  TRACE (agent_id, query, value_size, value);

  if (!detail::is_initialized)
    return AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED;

  agent_t *agent = find (agent_id);

  if (!agent)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_AGENT_ID;

  return agent->get_info (query, value_size, value);
  CATCH;
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_process_agent_list (amd_dbgapi_process_id_t process_id,
                               size_t *agent_count,
                               amd_dbgapi_agent_id_t **agents,
                               amd_dbgapi_changed_t *changed)
{
  TRY;
  TRACE (process_id);

  if (!detail::is_initialized)
    return AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED;

  std::vector<process_t *> processes;
  if (process_id != AMD_DBGAPI_PROCESS_NONE)
    {
      process_t *process = process_t::find (process_id);

      if (!process)
        return AMD_DBGAPI_STATUS_ERROR_INVALID_PROCESS_ID;

      if (amd_dbgapi_status_t status = process->update_agents ();
          status != AMD_DBGAPI_STATUS_SUCCESS)
        error ("process_t::update_agents failed (rc=%d)", status);

      processes.emplace_back (process);
    }
  else
    {
      for (auto &&process : process_t::all ())
        {
          if (amd_dbgapi_status_t status = process.update_agents ();
              status != AMD_DBGAPI_STATUS_SUCCESS)
            error ("process_t::update_agents failed (rc=%d)", status);

          processes.emplace_back (&process);
        }
    }

  return utils::get_handle_list<agent_t> (processes, agent_count, agents,
                                          changed);
  CATCH;
}
