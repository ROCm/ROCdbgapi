/* Copyright (c) 2019-2022 Advanced Micro Devices, Inc.

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

#include "process.h"
#include "agent.h"
#include "architecture.h"
#include "callbacks.h"
#include "code_object.h"
#include "debug.h"
#include "event.h"
#include "exception.h"
#include "initialization.h"
#include "logging.h"
#include "os_driver.h"
#include "queue.h"
#include "register.h"
#include "rocr_rdebug.h"
#include "watchpoint.h"
#include "wave.h"

#include <algorithm>
#include <exception>
#include <iterator>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include <link.h>

namespace amd::dbgapi
{

class dispatch_t;

namespace detail
{
process_t *last_found_process = nullptr;
} /* namespace detail */

handle_object_set_t<process_t> process_t::s_process_map;

process_t::process_t (amd_dbgapi_process_id_t process_id,
                      amd_dbgapi_client_process_id_t client_process_id)
  : handle_object (process_id), m_client_process_id (client_process_id),
    m_memory_cache (
      [this] (amd_dbgapi_global_address_t address, void *read,
              const void *write, size_t size)
      {
        amd_dbgapi_status_t status = os_driver ().xfer_global_memory_partial (
          address, read, write, &size);

        if (status == AMD_DBGAPI_STATUS_ERROR_PROCESS_EXITED)
          throw process_exited_exception_t (*this);
        else if (status == AMD_DBGAPI_STATUS_ERROR_MEMORY_ACCESS)
          throw memory_access_error_t (address_space_t::global (), address);
        else if (status != AMD_DBGAPI_STATUS_SUCCESS)
          fatal_error ("xfer_global_memory_partial failed (%s)",
                       to_cstring (status));

        return size;
      }),
    m_dummy_agent (AMD_DBGAPI_AGENT_NONE, *this, nullptr, {})
{
  amd_dbgapi_os_process_id_t os_process_id;
  amd_dbgapi_status_t status = get_os_pid (&os_process_id);
  if (status == AMD_DBGAPI_STATUS_SUCCESS)
    m_os_process_id.emplace (os_process_id);
  else if (status != AMD_DBGAPI_STATUS_ERROR_PROCESS_EXITED)
    fatal_error ("get_os_pid () failed (%s)", to_cstring (status));

  /* Create the notifier pipe.  */
  m_client_notifier_pipe.open ();
  if (!m_client_notifier_pipe.is_valid ())
    fatal_error ("Could not create the client notifier pipe");

  m_os_driver = os_driver_t::create_driver (m_os_process_id);
  if (!m_os_driver->is_valid ())
    fatal_error ("Could not create the OS driver");
}

process_t::~process_t ()
{
  /* Drop all active cache lines.  */
  m_memory_cache.write_back ();
  m_memory_cache.discard ();

  /* Destruct the os_driver before closing the notifier pipe.  */
  m_os_driver.reset ();
  m_client_notifier_pipe.close ();

  dbgapi_assert (m_watchpoint_map.empty ()
                 && "there should not be any active watchpoints left");

  if (this == detail::last_found_process)
    detail::last_found_process = nullptr;
}

namespace detail
{

template <typename Tuple, size_t I = std::tuple_size_v<Tuple> - 1>
void
reset_next_ids ()
{
  std::tuple_element_t<I, Tuple>::reset_next_id ();
  if constexpr (I > 0)
    reset_next_ids<Tuple, I - 1> ();
}

} /* namespace detail */

void
process_t::reset_all_ids ()
{
  dbgapi_assert (s_process_map.size () == 0
                 && "some processes are still attached");
  detail::reset_next_ids<decltype (m_handle_object_sets)> ();
}

void
process_t::detach ()
{
  std::exception_ptr exception;

  /* If an exception is raised while attempting to detach, make sure we still
     destruct the handle objects in the correct order. To achieve this, we
     catch any exception, and rethrow it later.
   */

  try
    {
      std::vector<queue_t *> queues;

      /* We don't need to resume the queues until we are done changing the
         state.  */
      set_forward_progress_needed (false);

      /* Return precise memory reporting to its default off state.  */
      set_precise_memory (false);

      /* Resume all the waves halted at launch.  */
      set_wave_launch_mode (os_wave_launch_mode_t::normal);

      /* Refresh the queues.  New queues may have been created, and waves may
         have hit breakpoints.  */
      update_queues ();

      /* Suspend the queues that weren't already suspended.  */
      for (auto &&queue : range<queue_t> ())
        if (!queue.is_suspended ())
          queues.emplace_back (&queue);

      suspend_queues (queues, "detach from process");

      /* Remove the watchpoints that may still be inserted.  */
      for (auto &&watchpoint : range<watchpoint_t> ())
        remove_watchpoint (watchpoint);

      for (auto &&wave : range<wave_t> ())
        {
          /* If the wave was displaced stepping, cancel the operation now while
             the queue is suspended (it may write registers).  */
          if (wave.displaced_stepping ())
            {
              wave.set_state (AMD_DBGAPI_WAVE_STATE_STOP);
              wave.displaced_stepping_complete ();
            }

          /* Invalidate the wave_id.  */
          wave.write_register (amdgpu_regnum_t::wave_id, wave_t::undefined);

          /* Resume the wave if it is single-stepping, or if it is stopped
             because of a debug event (completed single-step, breakpoint,
             watchpoint).  The wave is not resumed if it is halted because of
             pending exceptions.  */
          if ((wave.state () == AMD_DBGAPI_WAVE_STATE_SINGLE_STEP)
              || (wave.state () == AMD_DBGAPI_WAVE_STATE_STOP
                  && !(wave.stop_reason ()
                       & ~wave_t::resumable_stop_reason_mask)))
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

  if (os_driver ().is_debug_enabled ())
    {
      amd_dbgapi_status_t status = os_driver ().disable_debug ();
      if (status != AMD_DBGAPI_STATUS_SUCCESS
          && status != AMD_DBGAPI_STATUS_ERROR_PROCESS_EXITED)
        fatal_error ("Could not disable debug (%s)", to_cstring (status));

      log_info ("debugging is disabled for %s", to_cstring (id ()));
    }

  /* Destruct the waves, workgroups, dispatches, queues, and agents, in this
     order.  */
  std::get<handle_object_set_t<watchpoint_t>> (m_handle_object_sets).clear ();
  std::get<handle_object_set_t<wave_t>> (m_handle_object_sets).clear ();
  dbgapi_assert (count<displaced_stepping_t> () == 0
                 && "all displaced steppings should have completed");
  std::get<handle_object_set_t<workgroup_t>> (m_handle_object_sets).clear ();
  std::get<handle_object_set_t<dispatch_t>> (m_handle_object_sets).clear ();
  std::get<handle_object_set_t<queue_t>> (m_handle_object_sets).clear ();
  std::get<handle_object_set_t<agent_t>> (m_handle_object_sets).clear ();

  /* Destruct the breakpoints before the shared libraries and code objects  */
  std::get<handle_object_set_t<breakpoint_t>> (m_handle_object_sets).clear ();
  std::get<handle_object_set_t<code_object_t>> (m_handle_object_sets).clear ();

  log_info ("detached %s", to_cstring (id ()));

  if (exception)
    std::rethrow_exception (exception);
}

void
process_t::read_string (amd_dbgapi_global_address_t address,
                        std::string *string, size_t size)
{
  constexpr size_t chunk_size = memory_cache_t::cache_line_size;
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
      size_t xfer_size
        = read_global_memory_partial (address, staging_buffer, request_size);

      size_t length = std::min (size, xfer_size);

      if (!length)
        throw memory_access_error_t (address_space_t::global (), address);

      /* Copy the staging buffer into the string, stop at the '\0'
         terminating char if seen.  */
      for (size_t i = 0; i < length; ++i)
        {
          char c = staging_buffer[i];
          if (c == '\0')
            return;
          string->push_back (c);
        }

      /* If unable to read full request, then no point trying again as it will
         just fail again.  */
      if (request_size != xfer_size)
        throw memory_access_error_t (address_space_t::global (), address);

      address += length;
      size -= length;
    }
}

size_t
process_t::xfer_segment_memory (const address_space_t &address_space,
                                amd_dbgapi_segment_address_t segment_address,
                                void *read, const void *write, size_t size)
{
  auto [lowered_address_space, lowered_address]
    = address_space.lower (segment_address);

  if (lowered_address_space.kind () == address_space_t::kind_t::global)
    return read ? read_global_memory_partial (lowered_address, read, size)
                : write_global_memory_partial (lowered_address, write, size);
  else
    throw memory_access_error_t (address_space, segment_address,
                                 "address is not supported");
}

void
process_t::set_forward_progress_needed (bool forward_progress_needed)
{
  if (m_forward_progress_needed == forward_progress_needed)
    return;

  m_forward_progress_needed = forward_progress_needed;

  if (forward_progress_needed)
    {
      std::vector<queue_t *> queues;
      queues.reserve (count<queue_t> ());

      for (auto &&queue : range<queue_t> ())
        if (queue.is_suspended ())
          queues.emplace_back (&queue);

      resume_queues (queues, "forward progress required");
    }
}

void
process_t::set_wave_launch_mode (os_wave_launch_mode_t wave_launch_mode)
{
  if (m_wave_launch_mode == wave_launch_mode)
    return;

  auto set_wave_launch_mode = utils::make_scope_success (
    [=] () { m_wave_launch_mode = wave_launch_mode; });

  if (!os_driver ().is_debug_enabled ())
    return;

  amd_dbgapi_status_t status
    = os_driver ().set_wave_launch_mode (wave_launch_mode);
  if (status == AMD_DBGAPI_STATUS_ERROR_PROCESS_EXITED)
    throw process_exited_exception_t (*this);
  else if (status != AMD_DBGAPI_STATUS_SUCCESS)
    fatal_error ("os_driver_t::set_wave_launch_mode (%s) failed (%s)",
                 to_cstring (wave_launch_mode), to_cstring (status));

  /* When changing the wave launch mode from WAVE_LAUNCH_MODE_HALT, all
     waves halted at launch need to be resumed and reported to the client.
   */
  if (m_wave_launch_mode == os_wave_launch_mode_t::halt)
    {
      update_queues ();

      std::vector<queue_t *> queues;
      queues.reserve (count<queue_t> ());

      for (auto &&queue : range<queue_t> ())
        if (!queue.is_suspended ())
          queues.emplace_back (&queue);

      suspend_queues (queues, "halt waves at launch");

      /* For all waves in this process, resume the wave if it is halted at
         launch.  */
      for (auto &&wave : range<wave_t> ())
        if (wave.visibility ()
            == wave_t::visibility_t::hidden_halted_at_launch)
          {
            wave.set_state (AMD_DBGAPI_WAVE_STATE_RUN);
            wave.set_visibility (wave_t::visibility_t::visible);
          }

      /* Changing the launch mode before resuming the queues ensures that none
         of them are all-stopped when resuming.  */
      m_wave_launch_mode = wave_launch_mode;
      set_wave_launch_mode.release ();

      if (forward_progress_needed ())
        resume_queues (queues, "halt waves at launch");
    }
}

void
process_t::set_wave_launch_trap_override (os_wave_launch_trap_mask_t value,
                                          os_wave_launch_trap_mask_t mask)
{
  dbgapi_assert ((value & ~mask) == 0 && "invalid value");

  os_wave_launch_trap_mask_t wave_trap_mask
    = (m_wave_trap_mask & ~mask) | (value & mask);

  if (wave_trap_mask == m_wave_trap_mask)
    return;

  auto set_wave_trap_mask = utils::make_scope_success (
    [=] () { m_wave_trap_mask = wave_trap_mask; });

  if (!os_driver ().is_debug_enabled ())
    return;

  amd_dbgapi_status_t status = os_driver ().set_wave_launch_trap_override (
    os_wave_launch_trap_override_t::apply, value, mask);

  if (status == AMD_DBGAPI_STATUS_ERROR_PROCESS_EXITED)
    throw process_exited_exception_t (*this);
  else if (status != AMD_DBGAPI_STATUS_SUCCESS)
    fatal_error ("os_driver::set_wave_launch_trap_override failed (%s)",
                 to_cstring (status));
}

void
process_t::set_precise_memory (bool enabled)
{
  if (m_precise_memory == enabled)
    return;

  if (!m_supports_precise_memory)
    throw api_error_t (AMD_DBGAPI_STATUS_ERROR_NOT_SUPPORTED);

  auto set_precise_memory
    = utils::make_scope_success ([=] () { m_precise_memory = enabled; });

  if (!os_driver ().is_debug_enabled ())
    return;

  amd_dbgapi_status_t status = os_driver ().set_precise_memory (enabled);

  if (status == AMD_DBGAPI_STATUS_ERROR_PROCESS_EXITED)
    throw process_exited_exception_t (*this);
  else if (status != AMD_DBGAPI_STATUS_SUCCESS)
    fatal_error ("os_driver::set_precise_memory failed (%s)",
                 to_cstring (status));
}

std::vector<process_t *>
process_t::match (amd_dbgapi_process_id_t process_id)
{
  std::vector<process_t *> processes;

  if (process_id != AMD_DBGAPI_PROCESS_NONE)
    {
      process_t *process = process_t::find (process_id);

      if (!process)
        throw api_error_t (AMD_DBGAPI_STATUS_ERROR_INVALID_PROCESS_ID);

      processes.emplace_back (process);
    }
  else
    {
      for (auto &&process : process_t::all ())
        {
          process.update_queues ();
          processes.emplace_back (&process);
        }
    }

  return processes;
}

process_t *
process_t::find (amd_dbgapi_process_id_t process_id)
{
  if (detail::last_found_process
      && detail::last_found_process->id () == process_id)
    return detail::last_found_process;

  process_t *process = s_process_map.find (process_id);

  if (process)
    detail::last_found_process = process;

  return process;
}

process_t *
process_t::find (amd_dbgapi_client_process_id_t client_process_id)
{
  if (detail::last_found_process
      && detail::last_found_process->client_id () == client_process_id)
    return detail::last_found_process;

  process_t *process = s_process_map.find_if (
    [=] (auto &v) { return v.client_id () == client_process_id; });

  if (process)
    detail::last_found_process = process;

  return process;
}

void
process_t::update_agents ()
{
  /* Value used to mark agents that are reported by KFD. When sweeping, any
     agent found with a mark less than the current mark will be deleted, as
     these agents are no longer active.  */
  const epoch_t agent_mark = agent_t::next_mark ();

  std::vector<os_agent_info_t> agent_infos;
  size_t prev_agent_count = count<agent_t> ();
  size_t agent_count = prev_agent_count + 16;

  do
    {
      agent_infos.resize (agent_count);

      amd_dbgapi_status_t status = os_driver ().agent_snapshot (
        agent_infos.data (), agent_count, &agent_count,
        os_exception_mask_t::none);
      if (status == AMD_DBGAPI_STATUS_ERROR_PROCESS_EXITED)
        agent_count = 0;
      else if (status != AMD_DBGAPI_STATUS_SUCCESS)
        fatal_error ("os_driver_t::agent_snapshot failed (%s)",
                     to_cstring (status));
    }
  while (agent_infos.size () < agent_count);
  agent_infos.resize (agent_count);

  std::optional<bool> precise_memory_supported;

  /* Add new agents to the process.  */
  for (auto &&agent_info : agent_infos)
    {
      agent_t *agent
        = find_if ([&] (const agent_t &a)
                   { return a.os_agent_id () == agent_info.os_agent_id; });

      if (!agent)
        {
          auto [major, minor, stepping] = agent_info.gfxip;

          const architecture_t *architecture
            = architecture_t::find (string_printf (
              "gfx%d%c%c", major,
              (minor < 10) ? (minor + '0') : (minor - 10 + 'a'),
              (stepping < 10) ? (stepping + '0') : (stepping - 10 + 'a')));

          if (!architecture)
            warning ("os_agent_id %d: `%s' architecture not supported.",
                     agent_info.os_agent_id, agent_info.name.c_str ());

          if (prev_agent_count != 0)
            fatal_error ("gpu hot pluging is not supported");

          agent = &create<agent_t> (*this,        /* process  */
                                    architecture, /* architecture  */
                                    agent_info);  /* os_agent_info  */
        }

      agent->set_mark (agent_mark);

      if (agent->supports_debugging ())
        precise_memory_supported = precise_memory_supported.value_or (true)
                                   & agent_info.precise_memory_supported;
    }

  /* Remove agents that are no longer online.  */
  auto &&agent_range = range<agent_t> ();
  for (auto it = agent_range.begin (); it != agent_range.end ();)
    if (it->mark () < agent_mark)
      {
        amd_dbgapi_agent_id_t agent_id = it->id ();
        os_agent_id_t os_agent_id = it->os_agent_id ();

        it = destroy (it);

        log_info ("destroyed deleted %s (os_agent_id=%d)",
                  to_cstring (agent_id), os_agent_id);
      }
    else
      ++it;

  m_supports_precise_memory = precise_memory_supported.value_or (false);
}

size_t
process_t::watchpoint_count () const
{
  /* Return lowest watchpoint count amongst all the agents.  */

  std::optional<size_t> max_watchpoint_count;

  for (auto &&agent : range<agent_t> ())
    {
      if (!agent.supports_debugging ())
        continue;

      size_t watchpoint_count = agent.os_info ().address_watch_register_count;
      max_watchpoint_count = std::min (
        max_watchpoint_count.value_or (watchpoint_count), watchpoint_count);
    }

  return max_watchpoint_count.value_or (0);
}

amd_dbgapi_watchpoint_share_kind_t
process_t::watchpoint_shared_kind () const
{
  /* Return the lowest capability is this order:
     AMD_DBGAPI_WATCHPOINT_SHARE_KIND_UNSUPPORTED
     AMD_DBGAPI_WATCHPOINT_SHARE_KIND_SHARED
     AMD_DBGAPI_WATCHPOINT_SHARE_KIND_UNSHARED  */

  amd_dbgapi_watchpoint_share_kind_t kind
    = AMD_DBGAPI_WATCHPOINT_SHARE_KIND_UNSUPPORTED;

  for (auto &&agent : range<agent_t> ())
    {
      if (!agent.supports_debugging ())
        continue;

      switch (agent.watchpoint_share_kind ())
        {
        case AMD_DBGAPI_WATCHPOINT_SHARE_KIND_UNSUPPORTED:
          return AMD_DBGAPI_WATCHPOINT_SHARE_KIND_UNSUPPORTED;
        case AMD_DBGAPI_WATCHPOINT_SHARE_KIND_UNSHARED:
          kind = AMD_DBGAPI_WATCHPOINT_SHARE_KIND_UNSHARED;
          continue;
        case AMD_DBGAPI_WATCHPOINT_SHARE_KIND_SHARED:
          kind = AMD_DBGAPI_WATCHPOINT_SHARE_KIND_SHARED;
          break;
        }
    }

  return kind;
}

void
process_t::insert_watchpoint (const watchpoint_t &watchpoint)
{
  /* If this is the first watchpoint we are setting on this device, we need
   to enable the address watch exception trap for all new waves as well as
   update the existing waves.  */
  const bool first_watchpoint = count<watchpoint_t> () == 1;
  if (first_watchpoint)
    {
      set_wave_launch_trap_override (
        os_wave_launch_trap_mask_t::address_watch,
        os_wave_launch_trap_mask_t::address_watch);

      update_queues ();

      std::vector<queue_t *> queues;
      queues.reserve (count<queue_t> ());

      for (auto &&queue : range<queue_t> ())
        if (!queue.is_suspended ())
          queues.emplace_back (&queue);

      suspend_queues (queues, "insert watchpoint");

      for (auto &&wave : range<wave_t> ())
        wave.architecture ().wave_enable_traps (
          wave, os_wave_launch_trap_mask_t::address_watch);

      if (forward_progress_needed ())
        resume_queues (queues, "insert watchpoint");
    }

  os_watch_mode_t watch_mode;
  switch (watchpoint.kind ())
    {
    case AMD_DBGAPI_WATCHPOINT_KIND_LOAD:
      watch_mode = os_watch_mode_t::read;
      break;
    case AMD_DBGAPI_WATCHPOINT_KIND_STORE_AND_RMW:
      watch_mode = os_watch_mode_t::nonread;
      break;
    case AMD_DBGAPI_WATCHPOINT_KIND_RMW:
      watch_mode = os_watch_mode_t::atomic;
      break;
    case AMD_DBGAPI_WATCHPOINT_KIND_ALL:
      watch_mode = os_watch_mode_t::all;
      break;
    default:
      throw api_error_t (AMD_DBGAPI_STATUS_ERROR_NO_WATCHPOINT_AVAILABLE);
    }

  os_watch_id_t os_watch_id;
  amd_dbgapi_status_t status = os_driver ().set_address_watch (
    watchpoint.address (), -watchpoint.size (), watch_mode, &os_watch_id);

  if (status != AMD_DBGAPI_STATUS_SUCCESS
      && status != AMD_DBGAPI_STATUS_ERROR_PROCESS_EXITED)
    fatal_error ("os_driver_t::set_address_watch () failed (%s)",
                 to_cstring (status));

  log_info ("%s: set address_watch%d [%#lx-%#lx] (%s)", to_cstring (id ()),
            os_watch_id, watchpoint.address (),
            watchpoint.address () + watchpoint.size (),
            to_cstring (watchpoint.kind ()));

  if (!m_watchpoint_map.emplace (os_watch_id, &watchpoint).second)
    fatal_error ("os_watch_id %d is already in use", os_watch_id);
}

void
process_t::remove_watchpoint (const watchpoint_t &watchpoint)
{
  /* Find watchpoint in the os_watch_id to watchpoint_t map.  The key will be
     the os_watch_id to clear for this agent.  */
  auto it = std::find_if (m_watchpoint_map.begin (), m_watchpoint_map.end (),
                          [&watchpoint] (const auto &value)
                          { return value.second == &watchpoint; });

  if (it == m_watchpoint_map.end ())
    fatal_error ("watchpoint is not inserted");

  os_watch_id_t os_watch_id = it->first;

  amd_dbgapi_status_t status = os_driver ().clear_address_watch (os_watch_id);
  if (status != AMD_DBGAPI_STATUS_SUCCESS
      && status != AMD_DBGAPI_STATUS_ERROR_PROCESS_EXITED)
    fatal_error ("failed to remove watchpoint (%s)", to_cstring (status));

  m_watchpoint_map.erase (it);

  log_info ("%s: clear address_watch%d", to_cstring (id ()), os_watch_id);

  const bool last_watchpoint = count<watchpoint_t> () == 1;
  if (last_watchpoint)
    {
      set_wave_launch_trap_override (
        os_wave_launch_trap_mask_t::none,
        os_wave_launch_trap_mask_t::address_watch);

      update_queues ();

      std::vector<queue_t *> queues;
      queues.reserve (count<queue_t> ());

      for (auto &&queue : range<queue_t> ())
        if (!queue.is_suspended ())
          queues.emplace_back (&queue);

      suspend_queues (queues, "remove watchpoint");

      for (auto &&wave : range<wave_t> ())
        wave.architecture ().wave_disable_traps (
          wave, os_wave_launch_trap_mask_t::address_watch);

      if (forward_progress_needed ())
        resume_queues (queues, "remove watchpoint");
    }
}

const watchpoint_t *
process_t::find_watchpoint (os_watch_id_t os_watch_id) const
{
  auto it = m_watchpoint_map.find (os_watch_id);
  return it != m_watchpoint_map.end () ? it->second : nullptr;
}

size_t
process_t::suspend_queues (const std::vector<queue_t *> &queues,
                           const char *reason) const
{
  if (queues.empty ())
    return 0;

  size_t num_all_stopped_queues = 0;
  std::vector<os_queue_id_t> queue_ids;
  queue_ids.reserve (queues.size ());

  for (queue_t *queue : queues)
    {
      dbgapi_assert (queue && !queue->is_suspended ()
                     && "queue is null or already suspended");

      if (!queue->is_valid ())
        /* Invalid queues are allowed, but they are simply ignored.  */
        continue;

      if (queue->is_all_stopped ())
        ++num_all_stopped_queues;
      else
        queue_ids.emplace_back (queue->os_queue_id ());
    }

  auto os_queue_id_to_id = [this] (os_queue_id_t os_queue_id)
  {
    const queue_t *queue = find_if (
      [=] (const queue_t &x) { return x.os_queue_id () == os_queue_id; });
    dbgapi_assert (queue != nullptr);
    return queue->id ();
  };

  if (!queue_ids.empty ())
    log_info ("suspending %s (%s)", to_cstring (queue_ids, os_queue_id_to_id),
              reason);

  size_t num_suspended_queues;
  amd_dbgapi_status_t status = os_driver ().suspend_queues (
    queue_ids.data (), queue_ids.size (),
    os_exception_mask_t::queue_wave_abort
      | os_exception_mask_t::queue_wave_trap
      | os_exception_mask_t::queue_wave_math_error
      | os_exception_mask_t::queue_wave_illegal_instruction
      | os_exception_mask_t::queue_wave_memory_violation
      | os_exception_mask_t::queue_wave_aperture_violation,
    &num_suspended_queues);
  if (status == AMD_DBGAPI_STATUS_ERROR_PROCESS_EXITED)
    {
      for (auto &&queue : queues)
        queue->set_state (queue_t::state_t::invalid);
      return 0;
    }
  else if (status != AMD_DBGAPI_STATUS_SUCCESS)
    fatal_error ("os_driver::suspend_queues failed (%s)", to_cstring (status));

  [[maybe_unused]] size_t num_invalid_queues = 0;
  for (os_queue_id_t mask : queue_ids)
    {
      os_queue_id_t queue_id = mask & os_queue_id_mask;

      /* Some queues may have failed to suspend because they are the
         os_invalid_queueid, or no longer exist. Check the queue_ids returned
         by KFD and invalidate those marked as invalid.  It is allowed to
         invalidate a queue that is already invalid.  */

      if (mask & os_queue_error_mask)
        fatal_error ("failed to suspend os_queue_id %d", queue_id);

      if (mask & os_queue_invalid_mask)
        {
          auto it = std::find_if (queues.begin (), queues.end (),
                                  [=] (const queue_t *q)
                                  { return q->os_queue_id () == queue_id; });
          dbgapi_assert (it != queues.end ());

          (*it)->set_state (queue_t::state_t::invalid);
          ++num_invalid_queues;
        }
    }

  /* The queue state is published last so that listeners may
     act on the state right after the queue is unscheduled.  */
  for (queue_t *queue : queues)
    if (queue->is_valid ())
      queue->set_state (queue_t::state_t::suspended);

  dbgapi_assert (
    (num_suspended_queues + num_invalid_queues) == queue_ids.size ()
    && "number of suspended queues does not match number requested queue "
       "less number of invalid queues");

  return num_suspended_queues + num_all_stopped_queues;
}

size_t
process_t::resume_queues (const std::vector<queue_t *> &queues,
                          const char *reason) const
{
  if (queues.empty ())
    return 0;

  size_t num_all_stopped_queues = 0;
  std::vector<os_queue_id_t> queue_ids;
  queue_ids.reserve (queues.size ());

  /* The queue state is published first to give a chance to the listeners to
     act on the state before the hardware is updated.  */
  for (queue_t *queue : queues)
    {
      dbgapi_assert (queue && !queue->is_running ()
                     && "queue is null or already running");

      /* The same queue vector may be passed to suspend_queues () and
         resume_queues (), so ignore queues that may have been invalidated by
         suspend_queues ().  */
      if (!queue->is_valid ())
        continue;

      if (queue->is_all_stopped ())
        ++num_all_stopped_queues;
      else
        queue_ids.emplace_back (queue->os_queue_id ());

      queue->set_state (queue_t::state_t::running);
    }

  auto os_queue_id_to_id = [this] (os_queue_id_t os_queue_id)
  {
    const queue_t *queue = find_if (
      [=] (const queue_t &x) { return x.os_queue_id () == os_queue_id; });
    dbgapi_assert (queue != nullptr);
    return queue->id ();
  };

  if (!queue_ids.empty ())
    log_info ("resuming %s (%s)", to_cstring (queue_ids, os_queue_id_to_id),
              reason);

  size_t num_resumed_queues;
  amd_dbgapi_status_t status = os_driver ().resume_queues (
    queue_ids.data (), queue_ids.size (), &num_resumed_queues);
  if (status == AMD_DBGAPI_STATUS_ERROR_PROCESS_EXITED)
    {
      for (auto &&queue : queues)
        queue->set_state (queue_t::state_t::invalid);
      return 0;
    }
  else if (status != AMD_DBGAPI_STATUS_SUCCESS)
    fatal_error ("os_driver::resume_queues failed (%s)", to_cstring (status));

  [[maybe_unused]] size_t num_invalid_queues = 0;
  for (os_queue_id_t mask : queue_ids)
    {
      os_queue_id_t queue_id = mask & os_queue_id_mask;

      /* Some queues may have failed to resume because they are the
         os_invalid_queueid, or no longer exist. Check the queue_ids returned
         by KFD and invalidate those marked as invalid.  It is allowed to
         invalidate a queue that is already invalid.  */

      if (mask & os_queue_error_mask)
        fatal_error ("failed to resume os_queue_id %d", queue_id);

      if (queue_id & os_queue_invalid_mask)
        {
          auto it = std::find_if (queues.begin (), queues.end (),
                                  [=] (const queue_t *q)
                                  { return q->os_queue_id () == queue_id; });
          dbgapi_assert (it != queues.end ());

          (*it)->set_state (queue_t::state_t::invalid);
          ++num_invalid_queues;
        }
    }

  dbgapi_assert (
    (num_resumed_queues + num_invalid_queues) == queue_ids.size ()
    && "number of resumed queues does not match number requested queues less "
       "number of invalid queues");

  return num_resumed_queues + num_all_stopped_queues;
}

void
process_t::update_waves ()
{
  try
    {
      update_queues ();

      std::vector<queue_t *> queues;
      for (auto &&queue : range<queue_t> ())
        if (!queue.is_suspended ())
          queues.emplace_back (&queue);

      suspend_queues (queues, "process_t::update_waves");
      if (forward_progress_needed ())
        resume_queues (queues, "process_t::update_waves");
    }
  catch (const process_exited_exception_t &)
    {
      auto &&wave_range = range<wave_t> ();
      for (auto it = wave_range.begin (); it != wave_range.end ();)
        it = destroy (it);
    }
}

void
process_t::update_queues ()
{
  /* If the runtime is not loaded, or loaded with restrictions, then we should
     not update the queue list.  */
  if (m_runtime_state != AMD_DBGAPI_RUNTIME_STATE_LOADED_SUCCESS)
    return;

  epoch_t queue_mark;
  std::vector<os_queue_snapshot_entry_t> snapshots;
  size_t snapshot_count;

  /* Prime the queue count with the current number of queues.  */
  size_t queue_count = count<queue_t> ();

  do
    {
      /* Value used to mark queues that are reported by KFD. When sweeping, any
         queue found with a mark less than the current mark will be deleted, as
         these queues are no longer active.

         Update the epoch until we see all snapshots so that we can sweep
         queues that may have been destroyed between iterations.  */
      queue_mark = queue_t::next_mark ();

      /* We should allocate enough memory for the snapshots. Let's start with
         the current number of queues + 16.  */
      snapshot_count = queue_count + 16;
      snapshots.resize (snapshot_count);

      amd_dbgapi_status_t status = os_driver ().queue_snapshot (
        snapshots.data (), snapshot_count, &queue_count,
        os_exception_mask_t::queue_new);

      if (status == AMD_DBGAPI_STATUS_ERROR_PROCESS_EXITED)
        {
          /* The process exited so none of the queues are valid.  */
          for (auto &&queue : range<queue_t> ())
            queue.set_state (queue_t::state_t::invalid);
          queue_count = 0;
        }
      else if (status != AMD_DBGAPI_STATUS_SUCCESS)
        fatal_error ("queue_snapshot failed (%s)", to_cstring (status));

      /* We have to process the snapshots returned by the ioctl now, even
         if the list is incomplete, because we only get notified once that
         a queue is new.  The new_queue bit gets cleared after each query.  */

      snapshots.resize (std::min (queue_count, snapshot_count));
      for (auto &&queue_info : snapshots)
        {
          std::optional<amd_dbgapi_queue_id_t> queue_id;

          /* Find the queue by matching its os_queue_id with the one
             returned by the ioctl.  */
          queue_t *queue
            = find_if ([&] (const queue_t &x)
                       { return x.os_queue_id () == queue_info.queue_id; });

          /* Find the agent for this queue. */
          agent_t *agent
            = find_if ([&] (const agent_t &x)
                       { return x.os_agent_id () == queue_info.gpu_id; });

          /* TODO: investigate when this could happen, e.g. the debugger
             cgroups not matching the application cgroups?  */
          if (!agent)
            fatal_error ("could not find an agent for gpu_id %d",
                         queue_info.gpu_id);
          else if (!agent->supports_debugging ())
            {
              log_info ("ignoring os_queue_id %d due to %s (os_gpu_id=%d) "
                        "not supporting debugging",
                        queue_info.queue_id, to_cstring (agent->id ()),
                        queue_info.gpu_id);
              continue;
            }
          else if ((os_queue_exception_status (queue_info)
                    & os_exception_mask_t::queue_new)
                   != os_exception_mask_t::none)
            {
              /* If there is a stale queue with the same os_queue_id,
                 destroy it.  */
              if (queue)
                {
                  amd_dbgapi_queue_id_t destroyed_queue_id = queue->id ();

                  destroy (queue);

                  log_info ("destroyed stale %s (os_queue_id=%d)",
                            to_cstring (destroyed_queue_id),
                            queue_info.queue_id);
                }
            }
          else if (is_flag_set (flag_t::runtime_enable_during_attach))
            {
              /* When attaching to a process with an already enabled runtime,
                 active queues may not report the queue_new exception as it may
                 have been cleared by another debugger session.  */
              dbgapi_assert (!queue && "no queue should exists before attach");
            }
          else
            {
              /* We should always have a valid queue for a given os_queue_id
                 after the process is attached.  Not finding the queue means
                 that we either did not create a queue when a queue_new
                 exception was reported (we consumed the event without action),
                 or KFD did not report the new queue.  */
              if (!queue)
                fatal_error (
                  "os_queue_id %d should have been reported as a new "
                  "queue before",
                  queue_info.queue_id);

              /* If the queue does not have an agent, it was created outside of
                 update_queues, and it does not have all the information yet
                 filled in.  */
              if (queue->agent () == m_dummy_agent)
                {
                  /* This is a partially initialized queue, re-create a fully
                     initialized instance with the same os_queue_id.  */
                  queue_id.emplace (queue->id ());
                  destroy (queue);
                }
              else
                {
                  /* A queue that is not new should have the same agent it had
                     when created.  */
                  if (queue->agent () != *agent)
                    fatal_error (
                      "%s (os_queue_id %d) has a different %s than the "
                      "%s it had when created",
                      to_cstring (queue->id ()), queue_info.queue_id,
                      to_cstring (agent->id ()),
                      to_cstring (queue->agent ().id ()));

                  /* This isn't a new queue, and it is fully initialized.
                     Mark it as active, and continue to the next snapshot.  */
                  queue->set_mark (queue_mark);
                  continue;
                }
            }

          /* The queue could be new to us, and not have the new bit set if
             the process was previously attached and detached.  In that case,
             we always create a new queue_t for every queue_id reported by the
             snapshot ioctl.  */

          /* queue_t::create requests a new id if queue_id is {0}.  */
          queue = &queue_t::create (queue_id,    /* queue_id */
                                    *agent,      /* agent */
                                    queue_info); /* queue_info */

          queue->set_state (queue_t::state_t::running);
          queue->set_mark (queue_mark);

          log_info ("created new %s (os_queue_id=%d)",
                    to_cstring (queue->id ()), queue->os_queue_id ());
        }
    }
  while (queue_count > snapshot_count);

  /* Iterate all queues belonging to this process, and prune those with a mark
     older than the current mark.  */

  auto &&queue_range = range<queue_t> ();
  for (auto it = queue_range.begin (); it != queue_range.end ();)
    if (it->mark () < queue_mark)
      {
        amd_dbgapi_queue_id_t queue_id = it->id ();
        os_queue_id_t os_queue_id = it->os_queue_id ();

        it = destroy (it);

        log_info ("destroyed deleted %s (os_queue_id=%d)",
                  to_cstring (queue_id), os_queue_id);
      }
    else
      ++it;
}

void
process_t::update_code_objects ()
{
  /* If the runtime is not loaded, or loaded with restrictions, then we should
     not update the code object list.  */
  if (m_runtime_state != AMD_DBGAPI_RUNTIME_STATE_LOADED_SUCCESS)
    return;

  /* Value used to mark code objects that are reported by the ROCR. When
     sweeping, any code object found with a mark less than the current mark
     will be deleted, as these code objects are not longer loaded.  */
  epoch_t code_object_mark = code_object_t::next_mark ();

  try
    {
      decltype (r_debug::r_state) state;
      read_global_memory (m_runtime_info.r_debug + offsetof (r_debug, r_state),
                          &state);

      /* If the state is not RT_CONSISTENT then that indicates there is a
         thread actively updating the code object list.  We cannot read the
         list as it is not consistent. But once the thread completes the update
         it will set state back to RT_CONSISTENT and hit the exiting breakpoint
         in the r_brk function which will trigger a read of the code object
         list.  */
      if (state != r_debug::RT_CONSISTENT)
        return;

      amd_dbgapi_global_address_t link_map_address;
      read_global_memory (m_runtime_info.r_debug + offsetof (r_debug, r_map),
                          &link_map_address);

      while (link_map_address)
        {
          amd_dbgapi_global_address_t load_address;
          read_global_memory (link_map_address + offsetof (link_map, l_addr),
                              &load_address);

          amd_dbgapi_global_address_t l_name_address;
          read_global_memory (link_map_address + offsetof (link_map, l_name),
                              &l_name_address);

          std::string uri;
          read_string (l_name_address, &uri, -1);

          /* Check if the code object already exists.  */
          code_object_t *code_object = find_if (
            [&] (const code_object_t &x)
            {
              /* FIXME: We have an ABA problem for memory based code objects. A
                 new code object of the same size could have been loaded at the
                 same address as an old stale code object. We could add a
                 unique identifier to the URI.  */
              return x.load_address () == load_address && x.uri () == uri;
            });

          if (!code_object)
            code_object = &create<code_object_t> (*this, uri, load_address);

          code_object->set_mark (code_object_mark);

          read_global_memory (link_map_address + offsetof (link_map, l_next),
                              &link_map_address);
        }
    }
  catch (const process_exited_exception_t &)
    {
      /* Prune all code objects */
      code_object_mark = code_object_t::next_mark ();
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
}

void
process_t::runtime_enable (os_runtime_info_t runtime_info)
{
  amd_dbgapi_status_t status;

  m_runtime_state = [&] ()
  {
    /* Only save the runtime info if there are no fatal errors.  */
    auto save_runtime_info
      = utils::make_scope_success ([&] () { m_runtime_info = runtime_info; });

    if (runtime_info.runtime_state == os_runtime_state_t::disabled)
      return AMD_DBGAPI_RUNTIME_STATE_UNLOADED;

    if (runtime_info.runtime_state == os_runtime_state_t::enabled_busy)
      warning ("At least one agent is busy (debugging may be enabled by "
               "another process)");

    if (runtime_info.runtime_state != os_runtime_state_t::enabled)
      return AMD_DBGAPI_RUNTIME_STATE_LOADED_ERROR_RESTRICTION;

    /* Check the r_version.  */
    int r_version;
    read_global_memory (
      runtime_info.r_debug + offsetof (struct r_debug, r_version), &r_version);

    if (r_version != ROCR_RDEBUG_VERSION)
      {
        warning ("AMD GPU runtime's r_debug::r_version %d not supported "
                 "(r_debug::r_version %d required)",
                 r_version, ROCR_RDEBUG_VERSION);
        return AMD_DBGAPI_RUNTIME_STATE_LOADED_ERROR_RESTRICTION;
      }

    return AMD_DBGAPI_RUNTIME_STATE_LOADED_SUCCESS;
  }();

  if (m_runtime_state == AMD_DBGAPI_RUNTIME_STATE_UNLOADED)
    {
      /* From now on, and until the runtime is enabled again, only runtime
         events should be reported.  */
      status = os_driver ().set_exceptions_reported (
        os_exception_mask_t::process_runtime);
      if (status != AMD_DBGAPI_STATUS_SUCCESS)
        fatal_error ("os_driver_t::set_exceptions_reported failed (%s)",
                     to_cstring (status));

      /* Destruct the code objects.  */
      std::get<handle_object_set_t<code_object_t>> (m_handle_object_sets)
        .clear ();

      /* Remove the breakpoints we've inserted when the runtime was loaded.
       */
      auto &&breakpoint_range = range<breakpoint_t> ();
      for (auto it = breakpoint_range.begin (); it != breakpoint_range.end ();)
        it = destroy (it);
    }

  /* If the runtime is loaded but debugging is not available, nothing more
     needs to be done.  The only exception that should be reported is the
     runtime unloaded exception.  */
  if (m_runtime_state != AMD_DBGAPI_RUNTIME_STATE_LOADED_SUCCESS)
    return;

  /* Early exits (exceptions and returns) result in restriction errors.  */
  auto restriction_error = utils::make_scope_exit (
    [this] ()
    {
      os_driver ().set_exceptions_reported (
        os_exception_mask_t::process_runtime);
      m_runtime_state = AMD_DBGAPI_RUNTIME_STATE_LOADED_ERROR_RESTRICTION;
    });

  /* Now that the runtime is enabled, request notifications for all supported
     events.  */
  status = os_driver ().set_exceptions_reported (
    os_exception_mask_t::queue_wave_abort
    | os_exception_mask_t::queue_wave_trap
    | os_exception_mask_t::queue_wave_math_error
    | os_exception_mask_t::queue_wave_illegal_instruction
    | os_exception_mask_t::queue_wave_memory_violation
    | os_exception_mask_t::queue_wave_aperture_violation
    | os_exception_mask_t::device_memory_violation
    | os_exception_mask_t::process_runtime);
  if (status != AMD_DBGAPI_STATUS_SUCCESS)
    fatal_error ("os_driver_t::set_exceptions_reported failed (%s)",
                 to_cstring (status));

  /* Install a breakpoint at _amd_r_debug.r_brk.  The runtime calls this
     function before updating the code object list, and after completing
     updating the code object list.  */

  auto r_brk_callback = [this] (breakpoint_t &breakpoint,
                                amd_dbgapi_client_thread_id_t client_thread_id,
                                amd_dbgapi_breakpoint_action_t *action)
  {
    /* Save the current 'changed' status of code_object_t`s.  */
    bool saved_changed = set_changed<code_object_t> (false);

    update_code_objects ();

    /* If nothing has changed, we don't need to report an event.  */
    if (!set_changed<code_object_t> (changed<code_object_t> ()
                                     | saved_changed))
      {
        *action = AMD_DBGAPI_BREAKPOINT_ACTION_RESUME;
        return;
      }

    /* Create a breakpoint resume event that will be enqueued when the code
       object list updated event is reported as processed.  This will allow the
       client thread to resume execution.  */
    event_t &breakpoint_resume_event
      = create<event_t> (*this, AMD_DBGAPI_EVENT_KIND_BREAKPOINT_RESUME,
                         breakpoint.id (), client_thread_id);

    /* Enqueue a code object list updated event.  */
    enqueue_event (
      create<event_t> (*this, AMD_DBGAPI_EVENT_KIND_CODE_OBJECT_LIST_UPDATED,
                       breakpoint_resume_event.id ()));

    /* Tell the client thread that it cannot resume execution until it sees the
       breakpoint resume event for this breakpoint_id and report it as
       processed.  */
    *action = AMD_DBGAPI_BREAKPOINT_ACTION_HALT;
  };

  amd_dbgapi_global_address_t r_brk_address;
  read_global_memory (m_runtime_info.r_debug + offsetof (r_debug, r_brk),
                      &r_brk_address);

  if (!create<breakpoint_t> (*this, r_brk_address, r_brk_callback)
         .is_inserted ())
    return;

  update_queues ();

  if (!is_flag_set (flag_t::runtime_enable_during_attach) && count<queue_t> ())
    fatal_error ("no queue can exist before the runtime is enabled");

  update_code_objects ();

  status = os_driver ().set_wave_launch_mode (m_wave_launch_mode);
  if (status != AMD_DBGAPI_STATUS_SUCCESS
      && status != AMD_DBGAPI_STATUS_ERROR_PROCESS_EXITED)
    fatal_error ("Could not set the wave launch mode for %s (%s).",
                 to_cstring (id ()), to_cstring (status));

  os_wave_launch_trap_mask_t supported_wave_trap_mask;
  status = os_driver ().set_wave_launch_trap_override (
    os_wave_launch_trap_override_t::apply, os_wave_launch_trap_mask_t::none,
    os_wave_launch_trap_mask_t::none, nullptr, &supported_wave_trap_mask);
  if (status != AMD_DBGAPI_STATUS_SUCCESS
      && status != AMD_DBGAPI_STATUS_ERROR_PROCESS_EXITED)
    fatal_error ("Could not set the wave launch trap override for %s (%s).",
                 to_cstring (id ()), to_cstring (status));

  if ((m_wave_trap_mask & ~supported_wave_trap_mask) != 0)
    fatal_error ("Unsupported wave trap mask (%s) requested for %s",
                 to_cstring (m_wave_trap_mask & ~supported_wave_trap_mask),
                 to_cstring (id ()));

  status = os_driver ().set_wave_launch_trap_override (
    os_wave_launch_trap_override_t::apply, m_wave_trap_mask,
    supported_wave_trap_mask);
  if (status != AMD_DBGAPI_STATUS_SUCCESS
      && status != AMD_DBGAPI_STATUS_ERROR_PROCESS_EXITED)
    fatal_error ("Could not set the wave launch trap override for %s (%s).",
                 to_cstring (id ()), to_cstring (status));

  if (m_supports_precise_memory)
    {
      status = os_driver ().set_precise_memory (m_precise_memory);
      if (status != AMD_DBGAPI_STATUS_SUCCESS
          && status != AMD_DBGAPI_STATUS_ERROR_PROCESS_EXITED)
        fatal_error ("Could not set precise memory for %s (%s).",
                     to_cstring (id ()), to_cstring (status));
    }

  std::vector<queue_t *> queues;
  for (auto &&queue : range<queue_t> ())
    queues.emplace_back (&queue);

  /* Suspend the newly created queues to update the waves, then resume them.
     We could have attached to the process while wavefronts were executing.  */
  suspend_queues (queues, "attach to process");
  resume_queues (queues, "attach to process");

  clear_flag (flag_t::runtime_enable_during_attach);
  set_flag (flag_t::spi_ttmps_setup_enabled);

  restriction_error.release ();
}

void
process_t::attach ()
{
  log_info ("attaching %s to %s", to_cstring (id ()),
            m_os_process_id
              ? string_printf ("OS process %d", *m_os_process_id).c_str ()
              : "exited process");

  if (os_driver ().check_version () != AMD_DBGAPI_STATUS_SUCCESS)
    throw api_error_t (AMD_DBGAPI_STATUS_ERROR_RESTRICTION);

  os_runtime_info_t runtime_info{};
  if (auto status = os_driver ().enable_debug (
        os_exception_mask_t::process_runtime,
        m_client_notifier_pipe.write_fd (), &runtime_info);
      status == AMD_DBGAPI_STATUS_ERROR_RESTRICTION)
    throw api_error_t (AMD_DBGAPI_STATUS_ERROR_RESTRICTION);
  else if (status == AMD_DBGAPI_STATUS_ERROR_PROCESS_EXITED)
    runtime_info.runtime_state
      = static_cast<decltype (runtime_info.runtime_state)> (
        os_runtime_state_t::disabled);
  else if (status != AMD_DBGAPI_STATUS_SUCCESS)
    fatal_error ("enable_debug failed (%s)", to_cstring (status));

  auto disable_debug = utils::make_scope_exit (
    [this] ()
    {
      if (auto status = os_driver ().disable_debug ();
          status != AMD_DBGAPI_STATUS_SUCCESS
          && status != AMD_DBGAPI_STATUS_ERROR_PROCESS_EXITED)
        fatal_error ("disable_debug failed (%s)", to_cstring (status));
    });

  /* Update the agent now, regardless of the runtime state, so that agents can
     be reported as soon as the process is attached.  */
  update_agents ();

  if (runtime_info.runtime_state != os_runtime_state_t::disabled)
    {
      if (runtime_info.ttmp_setup)
        set_flag (flag_t::spi_ttmps_setup_enabled);

      set_flag (flag_t::runtime_enable_during_attach);
      runtime_enable (runtime_info);

      enqueue_event (create<event_t> (*this, AMD_DBGAPI_EVENT_KIND_RUNTIME,
                                      m_runtime_state));
    }

  disable_debug.release ();
  log_info ("debugging is enabled for %s", to_cstring (id ()));
}

void
process_t::get_info (amd_dbgapi_process_info_t query, size_t value_size,
                     void *value) const
{
  switch (query)
    {
    case AMD_DBGAPI_PROCESS_INFO_NOTIFIER:
      utils::get_info (value_size, value, m_client_notifier_pipe.read_fd ());
      return;

    case AMD_DBGAPI_PROCESS_INFO_WATCHPOINT_COUNT:
      utils::get_info (value_size, value, watchpoint_count ());
      return;

    case AMD_DBGAPI_PROCESS_INFO_WATCHPOINT_SHARE:
      utils::get_info (value_size, value, watchpoint_shared_kind ());
      return;

    case AMD_DBGAPI_PROCESS_INFO_PRECISE_MEMORY_SUPPORTED:
      utils::get_info (value_size, value,
                       m_supports_precise_memory
                         ? AMD_DBGAPI_MEMORY_PRECISION_PRECISE
                         : AMD_DBGAPI_MEMORY_PRECISION_NONE);
      return;

    case AMD_DBGAPI_PROCESS_INFO_OS_ID:
      if (!m_os_process_id)
        throw api_error_t (AMD_DBGAPI_STATUS_ERROR_NOT_AVAILABLE);
      utils::get_info (value_size, value, *m_os_process_id);
      return;
    }

  throw api_error_t (AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT);
}

void
process_t::enqueue_event (event_t &event)
{
  m_pending_events.emplace (&event);
  event.set_state (event_t::state_t::queued);

  /* Notify the client that a new event is available.  */
  client_notifier_pipe ().mark ();
}

std::pair<std::variant<process_t *, agent_t *, queue_t *>, os_exception_mask_t>
process_t::query_debug_event (os_exception_mask_t cleared_exceptions)
{
  if (!os_driver ().is_debug_enabled ())
    return { this, os_exception_mask_t::none };

  /* The caller should not request queue_new exceptions to be cleared, since
     they are only handled, and cleared, by this function if the runtime is
     enabled.  */
  dbgapi_assert ((cleared_exceptions & os_exception_mask_t::queue_new)
                   == os_exception_mask_t::none
                 && "invalid cleared_exceptions mask");

  if (m_runtime_info.runtime_state != os_runtime_state_t::disabled)
    cleared_exceptions |= os_exception_mask_t::queue_new;

  while (true)
    {
      os_queue_id_t os_queue_id;
      os_agent_id_t os_agent_id;
      os_exception_mask_t exceptions;

      amd_dbgapi_status_t status = os_driver ().query_debug_event (
        &exceptions, &os_queue_id, &os_agent_id, cleared_exceptions);

      if (status != AMD_DBGAPI_STATUS_SUCCESS
          && status != AMD_DBGAPI_STATUS_ERROR_PROCESS_EXITED)
        fatal_error ("os_driver_t::query_debug_event failed (%s)",
                     to_cstring (status));

      if (exceptions == os_exception_mask_t::none
          || status == AMD_DBGAPI_STATUS_ERROR_PROCESS_EXITED)
        {
          /* There are no more events.  */
          return { this, os_exception_mask_t::none };
        }

      if ((exceptions & os_process_exception_mask)
          != os_exception_mask_t::none)
        return { this, exceptions };

      dbgapi_assert (
        m_runtime_info.runtime_state != os_runtime_state_t::disabled
        && "runtime must be enabled for device and queue exceptions");

      if ((exceptions & os_agent_exception_mask) != os_exception_mask_t::none)
        {
          /* Find the agent by matching its os_agent_id with the one
             returned by the ioctl.  */
          agent_t *agent
            = find_if ([os_agent_id] (const agent_t &q)
                       { return q.os_agent_id () == os_agent_id; });

          if (!agent)
            fatal_error ("could not find os_agent_id %d", os_agent_id);

          if (!agent->supports_debugging ())
            {
              log_info (
                "dropping agent event (%s) on unsupported %s (os_gpu_id=%d)",
                to_cstring (exceptions), to_cstring (agent->id ()),
                os_agent_id);
              continue;
            }

          return { agent, exceptions };
        }

      /* We've handled process and device exceptions, the only exceptions
         left are queue exceptions.  */
      dbgapi_assert ((exceptions & os_queue_exception_mask)
                     != os_exception_mask_t::none);

      /* Find the queue by matching its os_queue_id with the one
         returned by the ioctl.  */

      queue_t *queue = find_if ([os_queue_id] (const queue_t &q)
                                { return q.os_queue_id () == os_queue_id; });

      /* If this is an event for a queue associated with an agent that does not
         support debugging, then discard it.  No queues are created for
         unsupported agents, so no need to check if we found a queue.  */
      if (!queue)
        {
          /* Find the agent by matching its os_agent_id with the one
             returned by the ioctl.  */
          agent_t *agent
            = find_if ([os_agent_id] (const agent_t &q)
                       { return q.os_agent_id () == os_agent_id; });

          if (!agent)
            fatal_error ("could not find os_agent_id %d", os_agent_id);

          if (!agent->supports_debugging ())
            {
              log_info ("dropping queue event (%s) for os_queue_id %d on "
                        "unsupported %s (os_gpu_id=%d)",
                        to_cstring (exceptions), os_queue_id,
                        to_cstring (agent->id ()), os_agent_id);
              continue;
            }
        }

      /* If this is a new queue, update the queues to make sure we don't
         return a stale queue with the same os_queue_id.  */
      if ((exceptions & os_exception_mask_t::queue_new)
          != os_exception_mask_t::none)
        {
          exceptions &= ~os_exception_mask_t::queue_new;

          /* queue_new exceptions are not subscribed to, so they can only be
             reported if other exceptions, which are subscribed to, are also
             reported.  */
          dbgapi_assert (exceptions != os_exception_mask_t::none);

          /* If there is a stale queue with the same os_queue_id, destroy it.
           */
          if (queue)
            {
              amd_dbgapi_queue_id_t stale_queue_id = queue->id ();

              destroy (queue);
              queue = nullptr;

              log_info ("destroyed stale %s (os_queue_id=%d)",
                        to_cstring (stale_queue_id), os_queue_id);
            }

          /* ABA handling: create a temporary, partially initialized, queue
             instance to hold a unique queue_id associated with this new
             os_queue_id, the call to update_queues below will fill in the
             missing information.

             Between now and the time update_queues is called, this os_queue
             may be destroyed, and a new os_queue with the same os_queue_id may
             be created.

             In that case the new queue instance created by update_queues will
             have a different unique queue_id, and the code below will not find
             the original queue_id in the process. The event will be
             dropped.  */

          os_queue_snapshot_entry_t queue_info{};
          queue_info.queue_id = os_queue_id;

          amd_dbgapi_queue_id_t queue_id
            = queue_t::create (std::nullopt, m_dummy_agent, queue_info).id ();

          update_queues ();

          /* Check that the queue still exists: it may have been deleted
             between the call to query_debug_event () and update_queues (); or
             update_queues () may have destroyed it if it isn't a supported
             queue type.  */
          queue = find (queue_id);
          if (!queue)
            {
              log_info ("dropping queue event (%s) for deleted os_queue_id %d",
                        to_cstring (exceptions), os_queue_id);
              continue;
            }

          return { queue, exceptions };
        }

      if (queue)
        return { queue, exceptions };

      fatal_error (
        "os_queue_id %d should have been reported as a new_queue before",
        os_queue_id);
    }
}

event_t *
process_t::next_pending_event ()
{
  if (!m_pending_events.empty ())
    {
      event_t *event = m_pending_events.front ();
      m_pending_events.pop ();
      return event;
    }

  /* Value used to mark agents that have reported a new device memory
     violation exception.  */
  epoch_t new_device_memory_violation_mark = agent_t::next_mark ();

  /* If we don't have any events left, we have to suspend the queues with
     pending events and process their context save area to add events for the
     waves that have reported events.  */

  std::vector<queue_t *> queues_needing_resume;
  queues_needing_resume.reserve (count<queue_t> ());

  /* It is possible for new events to be generated between the time we query
     the agents for pending queue events and the time we actually suspend the
     queues.  To make sure all events associated with the suspended queues are
     consumed, we loop until no new queue events are reported.

     This is guaranteed to terminate as once a queue is suspended it cannot
     create any new events. A queue will require at most 2 iterations; and if
     any new events occur on additional queues those queues will be suspended
     by additional iterations, and since there are a finite number of queues N,
     this can at most result in 2*N iterations.  */
  while (true)
    {
      std::unordered_set<queue_t *> queues_needing_suspend;

      while (true)
        {
          auto [source, exceptions] = query_debug_event (
            os_exception_mask_t::queue_wave_abort
            | os_exception_mask_t::queue_wave_trap
            | os_exception_mask_t::queue_wave_math_error
            | os_exception_mask_t::queue_wave_illegal_instruction
            | os_exception_mask_t::queue_wave_memory_violation
            | os_exception_mask_t::queue_wave_aperture_violation
            | os_exception_mask_t::device_memory_violation);
          dbgapi_assert (
            std::visit ([] (auto &&x) -> bool { return x; }, source)
            && "source cannot be null");

          if (exceptions == os_exception_mask_t::none)
            break;

          log_info (
            "%s has pending exceptions (%s)",
            std::visit ([] (auto &&x) { return to_string (x->id ()); }, source)
              .c_str (),
            to_cstring (exceptions));

          if ((exceptions & os_exception_mask_t::device_memory_violation)
              != os_exception_mask_t::none)
            {
              agent_t *agent = std::get<agent_t *> (source);

              agent->set_exceptions (
                os_exception_mask_t::device_memory_violation);

              /* Mark this agent so that we can later tell if the agent's
                 device memory violation exception is a new exception or a
                 deferred exception.  */
              agent->set_mark (new_device_memory_violation_mark);

              update_queues ();

              /* All queues on the device should be suspended to inspect the
                 state.  */
              for (auto &&queue : range<queue_t> ())
                if (!queue.is_suspended () && queue.agent () == *agent)
                  queues_needing_suspend.emplace (&queue);
            }

          if ((exceptions
               & (os_exception_mask_t::queue_wave_trap
                  | os_exception_mask_t::queue_wave_illegal_instruction
                  | os_exception_mask_t::queue_wave_memory_violation
                  | os_exception_mask_t::queue_wave_aperture_violation
                  | os_exception_mask_t::queue_wave_math_error
                  | os_exception_mask_t::queue_wave_abort))
              != os_exception_mask_t::none)
            {
              queue_t *queue = std::get<queue_t *> (source);

              /* The queue may already be suspended. This can happen if an
                 event occurs after requesting the queue to be suspended and
                 before that request has completed, or if the act of suspending
                 a wave generates a new event (such as for the single step
                 work-around in the CWSR handler).
               */
              if (!queue->is_suspended ())
                queues_needing_suspend.emplace (queue);
            }

          if ((exceptions & os_exception_mask_t::process_runtime)
              != os_exception_mask_t::none)
            {
              /* Make sure the runtime receives the process_runtime_enable
                 event even if an exception is thrown.  */
              auto send_runtime_enable_event = utils::make_scope_exit (
                [this] () {
                  send_exceptions (os_exception_mask_t::process_runtime, this);
                });

              /* Retrieve the runtime enable exception info.  */
              os_runtime_info_t runtime_info;
              amd_dbgapi_status_t status = os_driver ().query_exception_info (
                os_exception_code_t::process_runtime, {}, &runtime_info,
                sizeof (runtime_info), true);

              if (status == AMD_DBGAPI_STATUS_SUCCESS)
                {
                  /* Check that we have a valid runtime state in the exception
                     info.  There are only two valid state changes:
                     disabled -> !disabled, and !disabled -> disabled.  */
                  if ((runtime_info.runtime_state
                       != os_runtime_state_t::disabled)
                      == (m_runtime_info.runtime_state
                          != os_runtime_state_t::disabled))
                    fatal_error (
                      "spurious runtime exception (runtime_state %d->%d)",
                      m_runtime_info.runtime_state,
                      runtime_info.runtime_state);

                  runtime_enable (runtime_info);
                  enqueue_event (create<event_t> (
                    *this, AMD_DBGAPI_EVENT_KIND_RUNTIME, m_runtime_state));
                }
              else if (status != AMD_DBGAPI_STATUS_ERROR_PROCESS_EXITED)
                fatal_error ("query_exception_info failed (%s)",
                             to_cstring (status));
            }
        }

      /* Convert the std::set<queue_t *> into a std::vector<queue_t *>.  */
      std::vector<queue_t *> queues;
      queues.reserve (queues_needing_suspend.size ());
      std::copy (queues_needing_suspend.begin (),
                 queues_needing_suspend.end (), std::back_inserter (queues));

      /* Suspend the queues that have pending events which will cause all
         events to be created for any waves that have pending events.  */
      if (suspend_queues (queues, "next pending event") != queues.size ())
        {
          /* Some queues may have become invalid since we retrieved the event,
             failed to suspend, and marked by suspend_queues () as invalid.
             Remove such queues from our list, they will be destroyed next time
             we update the queues.  */
          for (auto it = queues.begin (); it != queues.end ();)
            it = (*it)->is_valid () ? std::next (it) : queues.erase (it);
        }

      /* Exit the loop if we did not add any new queues to suspend in this
         iteration.  */
      if (queues.empty ())
        break;

      /* If forward progress is needed, append queues into the list of queues
         needing resume.  */
      if (forward_progress_needed ())
        queues_needing_resume.insert (queues_needing_resume.end (),
                                      queues.begin (), queues.end ());
    }

  for (auto &agent : range<agent_t> ())
    if ((agent.exceptions () & os_exception_mask_t::device_memory_violation)
        != os_exception_mask_t::none)
      {
        bool waves_with_memory_violation = false;

        for (auto &&wave : range<wave_t> ())
          if (wave.agent () == agent
              && wave.state () == AMD_DBGAPI_WAVE_STATE_STOP
              && (wave.stop_reason ()
                  & AMD_DBGAPI_WAVE_STOP_REASON_MEMORY_VIOLATION))
            {
              /* Found a wave with a memory violation which has or will be
                 reported to the debugger.  Defer reporting the device memory
                 violation to the runtime until the wave is resumed.  */
              waves_with_memory_violation = true;
              break;
            }

        if (!waves_with_memory_violation
            && agent.mark () == new_device_memory_violation_mark)
          {
            /* This agent has a NEW device memory violation exception, and
               there are no waves with pending memory violations.  The hardware
               guarantees that all waves exceptions are reported before making
               their state visible.  Based on this guarantee, we can determine
               that this exception must have been raised either by CP or by a
               DMA engine. Let the runtime handle the exception.

               There is a possible race with the device memory violations being
               delayed and delivered after waves are resumed from an exception,
               which clears their memory violation exception.  To work around
               this, a grace period could be used, if memory violation
               exceptions are seen, to try to attribute all memory violation
               exceptions to their originating event.  */

            send_exceptions (os_exception_mask_t::device_memory_violation,
                             &agent);

            agent.clear_exceptions (
              os_exception_mask_t::device_memory_violation);
          }
        else if (waves_with_memory_violation)
          {
            /* There are waves with pending memory violation exceptions. Assume
               that the device exception was raised by a wavefront.

               The agent's device memory violation exception is recorded and
               its reporting deferred.  It will be sent to runtime along with
               the queue memory violation exception if/when the wave is resumed
               (see wave_t::set_state ()).

               NOTE: If a CP or DMA memory violation exception is raised while
               there are waves with pending memory violations, these exceptions
               will be lost as they will be attributed to a wave memory
               violation.

               FIXME: To avoid this, it is necessary for the hardware to report
               the origin of a device memory violation exception, and for KFD
               to separately record and report wave and CP/DMA exceptions.
               Currently KFD only remembers the first device exception received
               from any source.  */
          }
        else
          {
            /* The device memory violation exception reporting was deferred in
               an earlier call to this function by the immediately preceding
               ELSE IF.  At this point, all the wave exceptions have been
               processed, so clear the deferred device exception.  */

            agent.clear_exceptions (
              os_exception_mask_t::device_memory_violation);
          }
      }

  resume_queues (queues_needing_resume, "next pending event");

  if (m_pending_events.empty ())
    return nullptr;

  event_t *event = m_pending_events.front ();
  m_pending_events.pop ();
  return event;
}

void
process_t::send_exceptions (
  os_exception_mask_t exceptions,
  std::variant<process_t *, agent_t *, queue_t *> source) const
{
  os_agent_id_t agent_id = os_invalid_agentid;
  os_queue_id_t queue_id = os_invalid_queueid;

  if (std::holds_alternative<queue_t *> (source))
    {
      queue_id = std::get<queue_t *> (source)->os_queue_id ();
      agent_id = std::get<queue_t *> (source)->agent ().os_agent_id ();
    }
  else if (std::holds_alternative<agent_t *> (source))
    {
      dbgapi_assert ((exceptions & os_queue_exception_mask) == 0
                     && "should only have device or process exceptions");
      agent_id = std::get<agent_t *> (source)->os_agent_id ();
    }
  else
    {
      dbgapi_assert (std::holds_alternative<process_t *> (source));
      dbgapi_assert ((exceptions & ~os_process_exception_mask) == 0
                     && "should only have process exceptions");
      /* Process exceptions do not need a source_id.  */
    }

  log_info ("%s sending runtime exceptions [ %s ] (source=%s)",
            to_cstring (id ()), to_cstring (exceptions),
            std::visit ([] (auto &&x) { return to_string (x->id ()); }, source)
              .c_str ());

  amd_dbgapi_status_t status
    = os_driver ().send_exceptions (exceptions, agent_id, queue_id);

  if (status != AMD_DBGAPI_STATUS_SUCCESS
      && status != AMD_DBGAPI_STATUS_ERROR_PROCESS_EXITED)
    fatal_error ("send_exceptions failed (%s)", to_cstring (status));
}

amd_dbgapi_status_t
process_t::get_os_pid (amd_dbgapi_os_process_id_t *pid) const
{
  TRACE_CALLBACK_BEGIN (param_in (pid));
  return detail::process_callbacks.get_os_pid (m_client_process_id, pid);
  TRACE_CALLBACK_END (make_ref (param_out (pid)));
}

amd_dbgapi_status_t
process_t::insert_breakpoint (amd_dbgapi_global_address_t address,
                              amd_dbgapi_breakpoint_id_t breakpoint_id)
{
  TRACE_CALLBACK_BEGIN (make_hex (param_in (address)),
                        param_in (breakpoint_id));
  return detail::process_callbacks.insert_breakpoint (m_client_process_id,
                                                      address, breakpoint_id);
  TRACE_CALLBACK_END ();
}

amd_dbgapi_status_t
process_t::remove_breakpoint (amd_dbgapi_breakpoint_id_t breakpoint_id)
{
  TRACE_CALLBACK_BEGIN (param_in (breakpoint_id));
  return detail::process_callbacks.remove_breakpoint (m_client_process_id,
                                                      breakpoint_id);
  TRACE_CALLBACK_END ();
}

} /* namespace amd::dbgapi */

using namespace amd::dbgapi;

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_process_set_progress (amd_dbgapi_process_id_t process_id,
                                 amd_dbgapi_progress_t progress)
{
  TRACE_BEGIN (param_in (process_id), param_in (progress));
  TRY
  {
    if (!detail::is_initialized)
      THROW (AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED);

    bool forward_progress_needed;
    switch (progress)
      {
      case AMD_DBGAPI_PROGRESS_NORMAL:
        forward_progress_needed = true;
        break;
      case AMD_DBGAPI_PROGRESS_NO_FORWARD:
        forward_progress_needed = false;
        break;
      default:
        THROW (AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT);
      }

    std::vector<process_t *> processes;
    if (process_id == AMD_DBGAPI_PROCESS_NONE)
      {
        for (auto &&process : process_t::all ())
          processes.emplace_back (&process);
      }
    else
      {
        if (process_t *process = process_t::find (process_id); process)
          processes.emplace_back (process);
        else
          THROW (AMD_DBGAPI_STATUS_ERROR_INVALID_PROCESS_ID);
      }

    for (auto &&process : processes)
      {
        try
          {
            process->set_forward_progress_needed (forward_progress_needed);
          }
        catch (const process_exited_exception_t &)
          {
            /* The process has exited, forward progress is irrelevant.  */
          }
      }
  }
  CATCH (AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED,
         AMD_DBGAPI_STATUS_ERROR_INVALID_PROCESS_ID,
         AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT);
  TRACE_END ();
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_process_set_wave_creation (amd_dbgapi_process_id_t process_id,
                                      amd_dbgapi_wave_creation_t creation)
{
  TRACE_BEGIN (param_in (process_id), param_in (creation));
  TRY
  {
    if (!detail::is_initialized)
      THROW (AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED);

    process_t *process = process_t::find (process_id);

    if (!process)
      THROW (AMD_DBGAPI_STATUS_ERROR_INVALID_PROCESS_ID);

    try
      {
        switch (creation)
          {
          case AMD_DBGAPI_WAVE_CREATION_NORMAL:
            process->set_wave_launch_mode (os_wave_launch_mode_t::normal);
            break;
          case AMD_DBGAPI_WAVE_CREATION_STOP:
            process->set_wave_launch_mode (os_wave_launch_mode_t::halt);
            break;
          default:
            THROW (AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT);
          }
      }
    catch (const process_exited_exception_t &)
      {
        /* The process has exited, the wave launch mode is irrelevant.  */
      }
  }
  CATCH (AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED,
         AMD_DBGAPI_STATUS_ERROR_INVALID_PROCESS_ID,
         AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT);
  TRACE_END ();
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_process_attach (amd_dbgapi_client_process_id_t client_process_id,
                           amd_dbgapi_process_id_t *process_id)
{
  TRACE_BEGIN (param_in (client_process_id), param_in (process_id));
  TRY
  {
    if (!detail::is_initialized)
      THROW (AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED);

    if (!client_process_id || !process_id)
      THROW (AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT);

    /* Return an error if the client_process_id is already attached to another
       process instance.  */
    if (process_t::find (client_process_id))
      THROW (AMD_DBGAPI_STATUS_ERROR_ALREADY_ATTACHED);

    process_t *process;

    try
      {
        process = &process_t::create_process (client_process_id);
      }
    catch (const exception_t &)
      {
        /* process_t::create_process could throw a fatal error if it fails to
           create a new os_driver instance or create a pipe for the notifier,
           simply return an error.  */
        THROW (AMD_DBGAPI_STATUS_ERROR);
      }

    try
      {
        process->attach ();
      }
    catch (const process_exited_exception_t &)
      {
        /* Leave the process attached in case of a process exited exception. */
      }
    catch (const api_error_t &e)
      {
        /* For all other exceptions, destroy the newly created process.  */
        try
          {
            process->detach ();
            process_t::destroy_process (process);
          }
        catch (...)
          {
          }
        e.print_message ();
        THROW (e.code ());
      }

    *process_id = amd_dbgapi_process_id_t{ process->id () };
  }
  CATCH (AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED,
         AMD_DBGAPI_STATUS_ERROR_ALREADY_ATTACHED,
         AMD_DBGAPI_STATUS_ERROR_RESTRICTION,
         AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT, AMD_DBGAPI_STATUS_ERROR);
  TRACE_END (make_ref (param_out (process_id)));
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_process_detach (amd_dbgapi_process_id_t process_id)
{
  TRACE_BEGIN (param_in (process_id));
  TRY
  {
    if (!detail::is_initialized)
      THROW (AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED);

    process_t *process = process_t::find (process_id);

    if (!process)
      THROW (AMD_DBGAPI_STATUS_ERROR_INVALID_PROCESS_ID);

    try
      {
        process->detach ();
      }
    catch (const process_exited_exception_t &)
      {
      }
    process_t::destroy_process (process);
  }
  CATCH (AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED,
         AMD_DBGAPI_STATUS_ERROR_INVALID_PROCESS_ID);
  TRACE_END ();
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_process_get_info (amd_dbgapi_process_id_t process_id,
                             amd_dbgapi_process_info_t query,
                             size_t value_size, void *value)
{
  TRACE_BEGIN (param_in (process_id), param_in (query), param_in (value_size),
               param_in (value));
  TRY
  {
    if (!detail::is_initialized)
      THROW (AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED);

    process_t *process = process_t::find (process_id);

    if (!process)
      THROW (AMD_DBGAPI_STATUS_ERROR_INVALID_PROCESS_ID);

    try
      {
        process->get_info (query, value_size, value);
      }
    catch (const process_exited_exception_t &)
      {
        fatal_error (
          "process_t::get_info should not throw process_exited exceptions");
      }
  }
  CATCH (AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED,
         AMD_DBGAPI_STATUS_ERROR_INVALID_PROCESS_ID,
         AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT,
         AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT_COMPATIBILITY,
         AMD_DBGAPI_STATUS_ERROR_NOT_AVAILABLE,
         AMD_DBGAPI_STATUS_ERROR_CLIENT_CALLBACK);
  TRACE_END (make_query_ref (query, param_out (value)));
}
