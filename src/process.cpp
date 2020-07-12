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

#include "process.h"
#include "agent.h"
#include "architecture.h"
#include "callbacks.h"
#include "code_object.h"
#include "debug.h"
#include "event.h"
#include "initialization.h"
#include "logging.h"
#include "os_driver.h"
#include "queue.h"
#include "register.h"
#include "rocr_rdebug.h"
#include "wave.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstring>
#include <exception>
#include <future>
#include <iterator>
#include <limits>
#include <list>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <errno.h>
#include <limits.h>
#include <link.h>
#include <poll.h>
#include <signal.h>
#include <unistd.h>

namespace amd::dbgapi
{

class dispatch_t;
class watchpoint_t;

std::list<process_t *> process_list;

process_t::process_t (amd_dbgapi_client_process_id_t client_process_id,
                      amd_dbgapi_process_id_t process_id)
    : m_process_id (process_id), m_client_process_id (client_process_id),
      m_os_driver (os_driver_t::create ([this] () {
        std::optional<amd_dbgapi_os_pid_t> os_pid;

        amd_dbgapi_os_pid_t value;
        if (get_os_pid (&value) == AMD_DBGAPI_STATUS_SUCCESS)
          os_pid.emplace (value);

        return os_pid;
      }()))
{
  /* Create the notifier pipe.  */
  m_client_notifier_pipe.open ();

  /* See is_valid() for information about how failing to open the files or
     the notifier pipe is handled.  */
}

bool
process_t::is_valid () const
{
  /* This process is ready if /proc/pid/mem is open, the notifier pipe (used to
     communicate with the client) is ready, and the os_driver is ready. A
     process only exists in the not ready state while being created by the
     factory, and if not ready  will be destructed and never be put in the map
     of processes.
   */
  return m_os_driver->is_valid () && m_client_notifier_pipe.is_valid ();
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

      /* Resume all the waves halted at launch.  */
      set_wave_launch_mode (os_wave_launch_mode_t::NORMAL);

      /* Refresh the queues.  New queues may have been created, and waves may
         have hit breakpoints.  */
      update_queues ();

      /* Suspend the queues that weren't already suspended.  */
      for (auto &&queue : range<queue_t> ())
        if (!queue.is_suspended ())
          queues.emplace_back (&queue);

      suspend_queues (queues);

      /* Remove the watchpoints that may still be inserted.  */
      for (auto &&watchpoint : range<watchpoint_t> ())
        remove_watchpoint (watchpoint);

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
  amd_dbgapi_status_t status = stop_event_thread ();
  if (status != AMD_DBGAPI_STATUS_SUCCESS)
    error ("Could not stop the event thread (rc=%d)", status);

  /* Destruct the waves, dispatches, queues, and agents, in this order.  */
  std::get<handle_object_set_t<watchpoint_t>> (m_handle_object_sets).clear ();
  std::get<handle_object_set_t<wave_t>> (m_handle_object_sets).clear ();
  std::get<handle_object_set_t<dispatch_t>> (m_handle_object_sets).clear ();
  std::get<handle_object_set_t<queue_t>> (m_handle_object_sets).clear ();
  std::get<handle_object_set_t<agent_t>> (m_handle_object_sets).clear ();

  /* Destruct the breakpoints before the shared libraries and code objects  */
  std::get<handle_object_set_t<breakpoint_t>> (m_handle_object_sets).clear ();
  std::get<handle_object_set_t<code_object_t>> (m_handle_object_sets).clear ();
  std::get<handle_object_set_t<shared_library_t>> (m_handle_object_sets)
      .clear ();

  m_client_notifier_pipe.close ();

  if (exception)
    std::rethrow_exception (exception);
}

amd_dbgapi_status_t
process_t::read_global_memory_partial (amd_dbgapi_global_address_t address,
                                       void *buffer, size_t *size) const
{
  return os_driver ().xfer_global_memory_partial (address, buffer, nullptr,
                                                  size);
}

amd_dbgapi_status_t
process_t::read_global_memory (amd_dbgapi_global_address_t address,
                               void *buffer, size_t size) const
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
                        std::string *string, size_t size) const
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
                                        const void *buffer, size_t *size) const
{
  return os_driver ().xfer_global_memory_partial (address, nullptr, buffer,
                                                  size);
}

amd_dbgapi_status_t
process_t::write_global_memory (amd_dbgapi_global_address_t address,
                                const void *buffer, size_t size) const
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

      resume_queues (queues);
    }
}

amd_dbgapi_status_t
process_t::set_wave_launch_mode (os_wave_launch_mode_t wave_launch_mode)
{
  if (m_wave_launch_mode == wave_launch_mode)
    return AMD_DBGAPI_STATUS_SUCCESS;

  for (auto &&agent : range<agent_t> ())
    {
      amd_dbgapi_status_t status = os_driver ().set_wave_launch_mode (
          agent.os_agent_id (), wave_launch_mode);
      if (status == AMD_DBGAPI_STATUS_ERROR_PROCESS_EXITED)
        return status;
      else if (status != AMD_DBGAPI_STATUS_SUCCESS)
        error ("agent_t::set_wave_launch_mode (%s) failed (rc=%d)",
               to_string (wave_launch_mode).c_str (), status);
    }

  os_wave_launch_mode_t saved_wave_launch_mode = m_wave_launch_mode;
  m_wave_launch_mode = wave_launch_mode;

  /* When changing the wave launch mode from WAVE_LAUNCH_MODE_HALT, all waves
     halted at launch need to be resumed and reported to the client.  */
  if (saved_wave_launch_mode == os_wave_launch_mode_t::HALT)
    {
      update_queues ();

      std::vector<queue_t *> queues;
      queues.reserve (count<queue_t> ());

      for (auto &&queue : range<queue_t> ())
        if (!queue.is_suspended ())
          queues.emplace_back (&queue);

      suspend_queues (queues);

      /* For all waves in this process, resume the wave if it is halted at
         launch.  */
      for (auto &&wave : range<wave_t> ())
        if (wave.visibility ()
            == wave_t::visibility_t::HIDDEN_HALTED_AT_LAUNCH)
          {
            dbgapi_assert (wave.state () == AMD_DBGAPI_WAVE_STATE_STOP
                           && wave.stop_reason ()
                                  == AMD_DBGAPI_WAVE_STOP_REASON_NONE);

            wave.set_visibility (wave_t::visibility_t::VISIBLE);
            wave.set_state (AMD_DBGAPI_WAVE_STATE_RUN);
          }

      if (forward_progress_needed ())
        resume_queues (queues);
    }

  return AMD_DBGAPI_STATUS_SUCCESS;
}

amd_dbgapi_status_t
process_t::set_wave_launch_trap_override (os_wave_launch_trap_mask_t mask,
                                          os_wave_launch_trap_mask_t bits)
{
  /* Compute the mask that is supported by all agents.  */
  os_wave_launch_trap_mask_t supported_trap_mask_bits{
    ~os_wave_launch_trap_mask_t::NONE
  };
  for (auto &&agent : range<agent_t> ())
    supported_trap_mask_bits &= agent.supported_trap_mask ();

  /* Check that this operation is supported on all agents.  */
  if ((bits & supported_trap_mask_bits) != bits)
    return AMD_DBGAPI_STATUS_ERROR_NOT_SUPPORTED;

  for (auto &&agent : range<agent_t> ())
    {
      os_wave_launch_trap_mask_t ignored;

      amd_dbgapi_status_t status = os_driver ().set_wave_launch_trap_override (
          agent.os_agent_id (), os_wave_launch_trap_override_t::OR, mask, bits,
          &ignored, &ignored);

      if (status == AMD_DBGAPI_STATUS_ERROR_PROCESS_EXITED)
        return status;
      else if (status != AMD_DBGAPI_STATUS_SUCCESS)
        error ("os_driver::set_wave_launch_trap_override failed (rc=%d)",
               status);
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
process_t::update_agents ()
{
  std::vector<os_agent_snapshot_entry_t> agent_infos;
  size_t agent_count = count<agent_t> () + 16;

  do
    {
      agent_infos.resize (agent_count);

      amd_dbgapi_status_t status = os_driver ().agent_snapshot (
          agent_infos.data (), agent_count, &agent_count);
      if (status == AMD_DBGAPI_STATUS_ERROR_PROCESS_EXITED)
        agent_count = 0;
      else if (status != AMD_DBGAPI_STATUS_SUCCESS)
        return status;
    }
  while (agent_infos.size () < agent_count);
  agent_infos.resize (agent_count);

  amd_dbgapi_status_t retval = AMD_DBGAPI_STATUS_SUCCESS;

  /* Add new agents to the process.  */
  for (auto &&agent_info : agent_infos)
    {
      agent_t *agent = find_if ([&] (const agent_t &a) {
        return a.os_agent_id () == agent_info.os_agent_id;
      });

      const architecture_t *architecture
          = architecture_t::find (agent_info.e_machine);

      /* FIXME: May want to have a state for an agent so it can be listed as
         present, but marked as unsupported.  We would then remove the
         errors below.  */

      if (!architecture)
        {
          /* If debug mode is not enabled, simply skip this agent, it just
             won't be reported to the debugger client.  */
          if (!is_flag_set (flag_t::ENABLE_AGENT_DEBUG_MODE))
            continue;

          warning ("os_agent_id %d: `%s' architecture not supported.",
                   agent_info.os_agent_id, agent_info.name.c_str ());

          return AMD_DBGAPI_STATUS_ERROR_RESTRICTION;
        }

      if (!agent)
        agent = &create<agent_t> (*this,         /* process  */
                                  *architecture, /* architecture  */
                                  agent_info);   /* os_agent_info  */

      if (is_flag_set (flag_t::ENABLE_AGENT_DEBUG_MODE)
          && !agent->is_debug_mode_enabled ())
        {
          amd_dbgapi_status_t status = agent->enable_debug_mode ();
          if (status == AMD_DBGAPI_STATUS_ERROR_RESTRICTION)
            {
              /* This agent is not available for debugging in this process.  */
              warning ("os_agent_id %d cannot be enabled for debugging "
                       "in this process",
                       agent->os_agent_id ());
              retval = status;
            }
          else if (status != AMD_DBGAPI_STATUS_SUCCESS
                   && status != AMD_DBGAPI_STATUS_ERROR_PROCESS_EXITED)
            error ("Could not enable debugging on os_agent_id %d (rc=%d)",
                   agent->os_agent_id (), status);
        }
      else if (!is_flag_set (flag_t::ENABLE_AGENT_DEBUG_MODE)
               && agent->is_debug_mode_enabled ())
        {
          amd_dbgapi_status_t status = agent->disable_debug_mode ();
          if (status != AMD_DBGAPI_STATUS_SUCCESS
              && status != AMD_DBGAPI_STATUS_ERROR_PROCESS_EXITED)
            error ("Could not disable debugging on os_agent_id %d (rc=%d)",
                   agent->os_agent_id (), status);
        }

      if (agent->is_debug_mode_enabled ())
        {
          amd_dbgapi_status_t status = os_driver ().set_wave_launch_mode (
              agent->os_agent_id (), m_wave_launch_mode);
          if (status != AMD_DBGAPI_STATUS_SUCCESS
              && status != AMD_DBGAPI_STATUS_ERROR_PROCESS_EXITED)
            error ("Could not set the wave launch mode for os_agent_id %d "
                   "(rc=%d).",
                   agent->os_agent_id (), status);
        }
    }

  /* Delete agents that are no longer present in this process. */
  auto &&agent_range = range<agent_t> ();
  for (auto it = agent_range.begin (); it != agent_range.end ();)
    if (std::find_if (
            agent_infos.begin (), agent_infos.end (),
            [&] (const decltype (agent_infos)::value_type &agent_info) {
              return it->os_agent_id () == agent_info.os_agent_id;
            })
        == agent_infos.end ())
      it = destroy (it);
    else
      ++it;

  return retval;
}

size_t
process_t::watchpoint_count () const
{
  if (!count<agent_t> ())
    return 0;

  /* Return lowest watchpoint count amongst all the agents.  */

  size_t count = std::numeric_limits<size_t>::max ();

  for (auto &&agent : range<agent_t> ())
    count = std::min (count, agent.architecture ().watchpoint_count ());

  return count;
}

amd_dbgapi_watchpoint_share_kind_t
process_t::watchpoint_shared_kind () const
{
  if (!count<agent_t> ())
    return AMD_DBGAPI_WATCHPOINT_SHARE_KIND_UNSUPPORTED;

  /* Return the lowest capability is this order:
     AMD_DBGAPI_WATCHPOINT_SHARE_KIND_UNSUPPORTED
     AMD_DBGAPI_WATCHPOINT_SHARE_KIND_SHARED
     AMD_DBGAPI_WATCHPOINT_SHARE_KIND_UNSHARED  */

  amd_dbgapi_watchpoint_share_kind_t kind
      = AMD_DBGAPI_WATCHPOINT_SHARE_KIND_UNSHARED;

  for (auto &&agent : range<agent_t> ())
    {
      switch (agent.architecture ().watchpoint_share_kind ())
        {
        case AMD_DBGAPI_WATCHPOINT_SHARE_KIND_UNSUPPORTED:
          return AMD_DBGAPI_WATCHPOINT_SHARE_KIND_UNSUPPORTED;
        case AMD_DBGAPI_WATCHPOINT_SHARE_KIND_UNSHARED:
          continue;
        case AMD_DBGAPI_WATCHPOINT_SHARE_KIND_SHARED:
          kind = AMD_DBGAPI_WATCHPOINT_SHARE_KIND_SHARED;
          break;
        }
    }

  return kind;
}

amd_dbgapi_status_t
process_t::insert_watchpoint (const watchpoint_t &watchpoint,
                              amd_dbgapi_global_address_t *adjusted_address,
                              amd_dbgapi_global_address_t *adjusted_size)
{
  dbgapi_assert (adjusted_address && adjusted_size && "must not be null");

  /* If this is the first watchpoint we are setting on this device, we need
   to enable the address watch exception trap for all new waves as well as
   update the existing waves.  */
  const bool first_watchpoint = count<watchpoint_t> () == 1;
  if (first_watchpoint)
    {
      amd_dbgapi_status_t status;

      status = set_wave_launch_trap_override (
          os_wave_launch_trap_mask_t::ADDRESS_WATCH,
          os_wave_launch_trap_mask_t::ADDRESS_WATCH);
      if (status != AMD_DBGAPI_STATUS_SUCCESS)
        return status;

      status = update_queues ();
      if (status != AMD_DBGAPI_STATUS_SUCCESS)
        error ("process_t::update_queues failed (rc=%d)", status);

      std::vector<queue_t *> queues;
      queues.reserve (count<queue_t> ());

      for (auto &&queue : range<queue_t> ())
        if (!queue.is_suspended ())
          queues.emplace_back (&queue);

      suspend_queues (queues);

      for (auto &&wave : range<wave_t> ())
        {
          status = wave.architecture ().enable_wave_traps (
              wave, os_wave_launch_trap_mask_t::ADDRESS_WATCH);
          if (status != AMD_DBGAPI_STATUS_SUCCESS)
            error ("architecture_t::enable_wave_traps failed (rc=%d)", status);
        }

      if (forward_progress_needed ())
        resume_queues (queues);
    }

  dbgapi_assert (watchpoint.requested_size () && "requested size cannot be 0");

  /* The mask used to match an address range is in the form:

             47       39        Y       23       15        X     0
     Mask:   11111111 11111111 11xxxxxx xxxxxxxx xxxxxxxx xx000000

     Only the bits in mask[Y-1:X] (x`s) are user programmable. The x`s are what
     this routine is computing before passing the mask to
     agent_t::insert_watchpoint ().

     architecture_t::watchpoint_mask_bits () returns a mask (XBits) with 1`s
     where the x`s are located:

             47       39        Y       23       15        X     0
     XBits:  00000000 00000000 00111111 11111111 11111111 11000000
             [        A         ][             B           ][  C ]

     With the x`s determined for a given range, an address watch match is
     checked with:

     Match := (AccessAddress & Mask) == MatchedAddress

     The mask required to match a given address range is obtained by replacing
     the "Stable" bits between the first and last addresses of the range with
     ones, e.g.:

             47       39        Y       23       15        X     0
     First:  01111111 11111110 11100111 00000100 00000000 01000100
     Last:   01111111 11111110 11100111 00000100 00000000 01001000

     Stable := ~(next_power_of_2 (Start ^ End) - 1)

             47       39        Y       23       15        X     0
     Stable: 11111111 11111111 11111111 11111111 11111111 11110000

     If (Stable[47:Y] contains any 0 bits, a match cannot happen, and the
     watchpoint is rejected.

     The smallest adjusted_size is (1 << X).

     The adjusted mask (aMask) and adjusted address (aAddr) sent to
     agent_t::insert_watchpoint () are:

             47       39        Y       23       15        X     0
     aMask:  11111111 11111111 11111111 11111111 11111111 11000000
     aAddr:  01111111 11111110 11100111 00000100 00000000 01000000
   */

  amd_dbgapi_global_address_t first_address = watchpoint.requested_address ();
  amd_dbgapi_global_address_t last_address
      = first_address + watchpoint.requested_size () - 1;

  amd_dbgapi_global_address_t stable_bits
      = ~(utils::next_power_of_two (first_address ^ last_address) - 1);

  /* programmable_mask_bits is the intersection of all the process' agents
     capabilities.  architecture_t::watchpoint_mask_bits returns a mask
     with 1 bits in the positions that can be programmed (x`s).  */
  amd_dbgapi_global_address_t programmable_mask_bits{
    std::numeric_limits<decltype (programmable_mask_bits)>::max ()
  };
  for (auto &&agent : range<agent_t> ())
    programmable_mask_bits &= agent.architecture ().watchpoint_mask_bits ();

  amd_dbgapi_global_address_t field_B = programmable_mask_bits;
  amd_dbgapi_global_address_t field_A = ~(field_B | (field_B - 1));
  amd_dbgapi_global_address_t field_C = ~(field_A | field_B);

  /* Check that the required mask is within the agents capabilities.  */
  if (stable_bits < field_A)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;

  amd_dbgapi_global_address_t watch_mask = stable_bits & ~field_C;
  amd_dbgapi_global_address_t watch_address
      = watchpoint.requested_address () & watch_mask;

  dbgapi_assert (
      (watchpoint.requested_address () + watchpoint.requested_size ())
          <= (watch_address - watch_mask)
      && "the adjusted range does not contain the requested range");

  /* Insert the watchpoint on all agents.  */
  auto &&agent_range = range<agent_t> ();
  for (auto it = agent_range.begin (); it != agent_range.end (); ++it)
    {
      amd_dbgapi_status_t status
          = it->insert_watchpoint (watchpoint, watch_address, watch_mask);

      /* If we fail, remove all the watchpoints inserted so far.  */
      if (status != AMD_DBGAPI_STATUS_SUCCESS)
        {
          for (auto it2 = agent_range.begin (); it2 != it; ++it2)
            it2->remove_watchpoint (watchpoint);
          return status;
        }
    }

  *adjusted_address = watch_address;
  *adjusted_size = -watch_mask;

  return AMD_DBGAPI_STATUS_SUCCESS;
}

void
process_t::remove_watchpoint (const watchpoint_t &watchpoint)
{
  for (auto &&agent : range<agent_t> ())
    {
      amd_dbgapi_status_t status = agent.remove_watchpoint (watchpoint);
      if (status != AMD_DBGAPI_STATUS_SUCCESS
          && status != AMD_DBGAPI_STATUS_ERROR_PROCESS_EXITED)
        error ("failed to remove watchpoint (rc=%d)", status);
    }

  const bool last_watchpoint = count<watchpoint_t> () == 1;
  if (last_watchpoint)
    {
      amd_dbgapi_status_t status;

      status = set_wave_launch_trap_override (
          os_wave_launch_trap_mask_t::NONE,
          os_wave_launch_trap_mask_t::ADDRESS_WATCH);
      if (status != AMD_DBGAPI_STATUS_SUCCESS
          && status != AMD_DBGAPI_STATUS_ERROR_PROCESS_EXITED)
        error ("process_t::set_wave_launch_trap_override failed (rc=%d)",
               status);

      status = update_queues ();
      if (status != AMD_DBGAPI_STATUS_SUCCESS)
        error ("process_t::update_queues failed (rc=%d)", status);

      std::vector<queue_t *> queues;
      queues.reserve (count<queue_t> ());

      for (auto &&queue : range<queue_t> ())
        if (!queue.is_suspended ())
          queues.emplace_back (&queue);

      suspend_queues (queues);

      for (auto &&wave : range<wave_t> ())
        {
          status = wave.architecture ().disable_wave_traps (
              wave, os_wave_launch_trap_mask_t::ADDRESS_WATCH);
          if (status != AMD_DBGAPI_STATUS_SUCCESS)
            error ("architecture_t::enable_wave_traps failed (rc=%d)", status);
        }

      if (forward_progress_needed ())
        resume_queues (queues);
    }
}

size_t
process_t::suspend_queues (const std::vector<queue_t *> &queues) const
{
  if (queues.empty ())
    return 0;

  std::vector<os_queue_id_t> queue_ids;
  queue_ids.reserve (queues.size ());

  for (auto *queue : queues)
    {
      dbgapi_assert (queue && queue->state () != queue_t::state_t::SUSPENDED
                     && "queue is null or already suspended");

      /* Note that invalid queues will return the OS_INVALID_QUEUEID.  */
      queue_ids.emplace_back (queue->os_queue_id ());
    }

  int ret = os_driver ().suspend_queues (queue_ids.data (), queue_ids.size ());
  if (ret == AMD_DBGAPI_STATUS_ERROR_PROCESS_EXITED)
    {
      for (auto &&queue : queues)
        queue->set_state (queue_t::state_t::INVALID);
      return 0;
    }
  else if (ret < 0)
    error ("os_driver::suspend_queues failed (rc=%d)", ret);

  size_t num_suspended_queues = ret;
  [[maybe_unused]] bool invalid_queue_seen = false;
  for (size_t i = 0; i < queue_ids.size (); ++i)
    {
      /* Some queues may have failed to suspend because they are the
         OS_INVALID_QUEUEID, or no longer exist. Check the queue_ids returned
         by KFD and invalidate those marked as invalid.  It is allowed to
         invalidate a queue that is already invalid.  */

      if (queue_ids[i] & OS_QUEUE_ERROR_MASK)
        {
          error ("failed to suspend %s (%#x",
                 to_string (queues[i]->id ()).c_str (), queue_ids[i]);
        }
      else if (queue_ids[i] & OS_QUEUE_INVALID_MASK)
        {
          queues[i]->set_state (queue_t::state_t::INVALID);
          invalid_queue_seen = true;
        }
      else
        {
          queues[i]->set_state (queue_t::state_t::SUSPENDED);
        }
    }
  dbgapi_assert (
      (num_suspended_queues == queue_ids.size () || invalid_queue_seen)
      && "should have seen an invalid queue");

  return num_suspended_queues;
}

size_t
process_t::resume_queues (const std::vector<queue_t *> &queues) const
{
  if (queues.empty ())
    return 0;

  std::vector<os_queue_id_t> queue_ids;
  queue_ids.reserve (queues.size ());

  for (auto *queue : queues)
    {
      dbgapi_assert (queue && queue->state () != queue_t::state_t::RUNNING
                     && "queue is null or not suspended");

      /* Note that invalid queues will return the OS_INVALID_QUEUEID.  */
      queue_ids.emplace_back (queue->os_queue_id ());
    }

  int ret = os_driver ().resume_queues (queue_ids.data (), queue_ids.size ());
  if (ret == AMD_DBGAPI_STATUS_ERROR_PROCESS_EXITED)
    {
      for (auto &&queue : queues)
        queue->set_state (queue_t::state_t::INVALID);
      return 0;
    }
  else if (ret < 0)
    error ("os_driver::resume_queues failed (rc=%d)", ret);

  size_t num_resumed_queues = ret;
  [[maybe_unused]] bool invalid_queue_seen = false;
  for (size_t i = 0; i < queue_ids.size (); ++i)
    {
      /* Some queues may have failed to resume because they are the
         OS_INVALID_QUEUEID, or no longer exist. Check the queue_ids returned
         by KFD and invalidate those marked as invalid.  It is allowed to
         invalidate a queue that is already invalid.  */

      if (queue_ids[i] & OS_QUEUE_ERROR_MASK)
        {
          error ("failed to resume %s (%#x)",
                 to_string (queues[i]->id ()).c_str (), queue_ids[i]);
        }
      else if (queue_ids[i] & OS_QUEUE_INVALID_MASK)
        {
          queues[i]->set_state (queue_t::state_t::INVALID);
          invalid_queue_seen = true;
        }
      else
        {
          queues[i]->set_state (queue_t::state_t::RUNNING);
        }
    }
  dbgapi_assert (
      (num_resumed_queues == queue_ids.size () || invalid_queue_seen)
      && "should have seen an invalid queue");

  return num_resumed_queues;
}

amd_dbgapi_status_t
process_t::update_queues ()
{
  epoch_t queue_mark;
  std::vector<os_queue_snapshot_entry_t> snapshots;
  size_t snapshot_count;

  /* Prime the queue count with the current number of queues.  */
  size_t queue_count = count<queue_t> ();

  do
    {
      /* Until we see all the snapshots, increase the epoch so that we can
         sweep queues that may have been destroyed between iterations.  */
      queue_mark = m_next_queue_mark++;

      /* We should allocate enough memory for the snapshots. Let's start with
         the current number of queues + 16.  */
      snapshot_count = queue_count + 16;
      snapshots.resize (snapshot_count);

      amd_dbgapi_status_t status = os_driver ().queue_snapshot (
          snapshots.data (), snapshot_count, &queue_count);

      if (status == AMD_DBGAPI_STATUS_ERROR_PROCESS_EXITED)
        queue_count = 0;
      else if (status != AMD_DBGAPI_STATUS_SUCCESS)
        return status;

      /* We have to process the snapshots returned by the ioctl now, even
         if the list is incomplete, because we only get notified once that
         a queue is new.  The new_queue bit gets cleared after each query.  */

      snapshots.resize (std::min (queue_count, snapshot_count));
      for (auto &&queue_info : snapshots)
        {
          std::optional<amd_dbgapi_queue_id_t> queue_id;

          /* Find the queue by matching its os_queue_id with the one
             returned by the ioctl.  */
          queue_t *queue = find_if ([&] (const queue_t &x) {
            return x.os_queue_id () == queue_info.queue_id;
          });

          if (!!(os_queue_status (queue_info) & os_queue_status_t::NEW_QUEUE))
            {
              /* If there is a stale queue with the same os_queue_id,
                 destroy it.  */
              if (queue)
                {
                  amd_dbgapi_queue_id_t destroyed_queue_id = queue->id ();
                  os_queue_id_t os_queue_id = queue->os_queue_id ();

                  destroy (queue);

                  dbgapi_log (AMD_DBGAPI_LOG_LEVEL_INFO,
                              "destroyed stale %s (os_queue_id=%d)",
                              to_string (destroyed_queue_id).c_str (),
                              os_queue_id);
                }
            }
          else if (is_flag_set (flag_t::REQUIRE_NEW_QUEUE_BIT))
            {
              /* We should always have a valid queue for a given os_queue_id
                 after the process is initialized.  Not finding the queue means
                 that we either did not create a queue when a new queue_id was
                 reported (we consumed the event without action), or KFD did
                 not report the new queue.  */
              if (!queue)
                {
                  error ("os_queue_id %d should have been reported as a "
                         "NEW_QUEUE before",
                         queue_info.queue_id);
                }

              /* FIXME: If we could select which flags get cleared by the
                 query_debug_event ioctl, we would not need to create a
                 partially initialized queue in agent_t::next_os_event, and
                 fix it here with the information contained in the queue
                 snapshots.  */

              /* If the queue mark is null, the queue was created outside of
                 update_queues, and it does not have all the information yet
                 filled in.  */
              if (!queue->mark ())
                {
                  /* This is a partially initialized queue, re-create a fully
                     initialized instance with the same os_queue_id.  */
                  queue_id.emplace (queue->id ());
                  destroy (queue);
                }
              else
                {
                  dbgapi_assert (!!(os_queue_status (queue_info)
                                    & os_queue_status_t::SUSPENDED)
                                     == queue->is_suspended ()
                                 && "queue state does not match queue_info");

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

          /* Find the agent for this queue. */
          agent_t *agent = find_if ([&] (const agent_t &x) {
            return x.os_agent_id () == queue_info.gpu_id;
          });

          /* TODO: investigate when this could happen, e.g. the debugger
             cgroups not matching the application cgroups?  */
          if (!agent)
            error ("could not find an agent for gpu_id %d", queue_info.gpu_id);

          /* create<queue_t> requests a new id if queue_id is {0}.  */
          queue = &create<queue_t> (queue_id,    /* queue_id */
                                    *agent,      /* agent */
                                    queue_info); /* queue_info */
          queue->set_state (
              !(os_queue_status (queue_info) & os_queue_status_t::SUSPENDED)
                  ? queue_t::state_t::RUNNING
                  : queue_t::state_t::SUSPENDED);
          queue->set_mark (queue_mark);

          dbgapi_log (
              AMD_DBGAPI_LOG_LEVEL_INFO, "created new %s (os_queue_id=%d)",
              to_string (queue->id ()).c_str (), queue->os_queue_id ());
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

        dbgapi_log (AMD_DBGAPI_LOG_LEVEL_INFO,
                    "destroyed deleted %s (os_queue_id=%d)",
                    to_string (queue_id).c_str (), os_queue_id);
      }
    else
      ++it;

  return AMD_DBGAPI_STATUS_SUCCESS;
}

amd_dbgapi_status_t
process_t::update_code_objects ()
{
  epoch_t code_object_mark = m_next_code_object_mark++;
  amd_dbgapi_status_t status;

  decltype (r_debug::r_state) state;
  status = read_global_memory (m_r_debug_address + offsetof (r_debug, r_state),
                               &state, sizeof (state));
  if (status != AMD_DBGAPI_STATUS_SUCCESS)
    error ("read_global_memory failed (rc=%d)", status);

  /* If the state is not RT_CONSISTENT then that indicates there is a thread
     actively updating the code object list.  We cannot read the list as it is
     not consistent. But once the thread completes the update it will set state
     back to RT_CONSISTENT and hit the exiting breakpoint in the r_brk function
     which will trigger a read of the code object list.  */
  if (state != r_debug::RT_CONSISTENT)
    return AMD_DBGAPI_STATUS_SUCCESS;

  amd_dbgapi_global_address_t link_map_address;
  status = read_global_memory (m_r_debug_address + offsetof (r_debug, r_map),
                               &link_map_address, sizeof (link_map_address));
  if (status != AMD_DBGAPI_STATUS_SUCCESS)
    error ("read_global_memory failed (rc=%d)", status);

  while (link_map_address)
    {
      amd_dbgapi_global_address_t load_address;
      status
          = read_global_memory (link_map_address + offsetof (link_map, l_addr),
                                &load_address, sizeof (load_address));
      if (status != AMD_DBGAPI_STATUS_SUCCESS)
        error ("read_global_memory failed (rc=%d)", status);

      amd_dbgapi_global_address_t l_name_address;
      status
          = read_global_memory (link_map_address + offsetof (link_map, l_name),
                                &l_name_address, sizeof (l_name_address));
      if (status != AMD_DBGAPI_STATUS_SUCCESS)
        error ("read_global_memory failed (rc=%d)", status);

      std::string uri;
      status = read_string (l_name_address, &uri, -1);
      if (status != AMD_DBGAPI_STATUS_SUCCESS)
        error ("read_string failed (rc=%d)", status);

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

      status
          = read_global_memory (link_map_address + offsetof (link_map, l_next),
                                &link_map_address, sizeof (link_map_address));
      if (status != AMD_DBGAPI_STATUS_SUCCESS)
        error ("read_global_memory failed (rc=%d)", status);
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
  /* When we first attach to the process, all waves already running need to be
     assigned new wave_ids.  This flag will be cleared after the first call
     to suspend the queues (and update the waves).  */
  set_flag (flag_t::ASSIGN_NEW_IDS_TO_ALL_WAVES);

  auto on_runtime_load_callback = [this] (const shared_library_t &library) {
    amd_dbgapi_status_t status;

    /* The runtime is loaded, enable the debug trap on all devices.  */
    set_flag (flag_t::ENABLE_AGENT_DEBUG_MODE);

    if (os_driver ().check_version () != AMD_DBGAPI_STATUS_SUCCESS
        || (status = update_agents ()) == AMD_DBGAPI_STATUS_ERROR_RESTRICTION)
      {
        enqueue_event (create<event_t> (
            *this, AMD_DBGAPI_EVENT_KIND_RUNTIME,
            AMD_DBGAPI_RUNTIME_STATE_LOADED_ERROR_RESTRICTION));
        return;
      }
    else if (status != AMD_DBGAPI_STATUS_SUCCESS)
      error ("update_agents failed (rc=%d)", status);

    status = update_queues ();
    if (status != AMD_DBGAPI_STATUS_SUCCESS)
      error ("update_queues failed (rc=%d)", status);

    /* From now on, new queues reported by kfd should have NEW_QUEUE set.  */
    set_flag (flag_t::REQUIRE_NEW_QUEUE_BIT);

    std::vector<queue_t *> queues;
    for (auto &&queue : range<queue_t> ())
      queues.emplace_back (&queue);

    /* Suspend the newly created queues to update the waves, then resume them.
       We could have attached to the process while wavefronts were executing.
     */
    suspend_queues (queues);
    clear_flag (flag_t::ASSIGN_NEW_IDS_TO_ALL_WAVES);
    resume_queues (queues);

    status = start_event_thread ();
    if (status != AMD_DBGAPI_STATUS_SUCCESS)
      error ("Cannot start the event thread (rc=%d)", status);

    /* Retrieve the address of the rendez-vous structure (_amd_gpu_r_debug)
       used by the runtime loader to communicate details of code objects
       loading to the debugger.  */
    constexpr char amdgpu_r_debug_symbol_name[] = "_amdgpu_r_debug";
    if (get_symbol_address (library.id (), amdgpu_r_debug_symbol_name,
                            &m_r_debug_address)
        != AMD_DBGAPI_STATUS_SUCCESS)
      error ("Cannot find symbol `%s'", amdgpu_r_debug_symbol_name);

    /* Check the r_version.  */
    int r_version;
    status = read_global_memory (m_r_debug_address
                                     + offsetof (struct r_debug, r_version),
                                 &r_version, sizeof (r_version));
    if (status != AMD_DBGAPI_STATUS_SUCCESS)
      error ("read_global_memory failed (rc=%d)", status);

    if (r_version != ROCR_RDEBUG_VERSION)
      {
        warning ("%s: AMD GPU runtime _amdgpu_r_debug.r_version %d "
                 "does not match %d requirement",
                 library.name ().c_str (), r_version, ROCR_RDEBUG_VERSION);
        enqueue_event (create<event_t> (
            *this, AMD_DBGAPI_EVENT_KIND_RUNTIME,
            AMD_DBGAPI_RUNTIME_STATE_LOADED_ERROR_RESTRICTION));
        return;
      }

    /* Install a breakpoint at _amd_r_debug.r_brk.  The runtime calls this
       function before updating the code object list, and after completing
       updating the code object list.  */

    amd_dbgapi_global_address_t r_brk_address;
    status = read_global_memory (m_r_debug_address
                                     + offsetof (struct r_debug, r_brk),
                                 &r_brk_address, sizeof (r_brk_address));
    if (status != AMD_DBGAPI_STATUS_SUCCESS)
      error ("read_global_memory failed (rc=%d)", status);

    /* This function gets called when the client reports that the breakpoint
       has been hit.  */
    auto r_brk_callback
        = [this] (breakpoint_t &breakpoint,
                  amd_dbgapi_client_thread_id_t client_thread_id,
                  amd_dbgapi_breakpoint_action_t *action) {
            /* Save the current 'changed' status of code_object_t`s.  */
            bool saved_changed = set_changed<code_object_t> (false);

            update_code_objects ();

            /* If nothing has changed, we don't need to report an event.  */
            if (!set_changed<code_object_t> (changed<code_object_t> ()
                                             | saved_changed))
              {
                *action = AMD_DBGAPI_BREAKPOINT_ACTION_RESUME;
                return AMD_DBGAPI_STATUS_SUCCESS;
              }

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

    enqueue_event (create<event_t> (*this, AMD_DBGAPI_EVENT_KIND_RUNTIME,
                                    AMD_DBGAPI_RUNTIME_STATE_LOADED_SUCCESS));

    update_code_objects ();

    if (count<code_object_t> ())
      enqueue_event (create<event_t> (
          *this, AMD_DBGAPI_EVENT_KIND_CODE_OBJECT_LIST_UPDATED,
          AMD_DBGAPI_EVENT_NONE));
  };

  auto on_runtime_unload_callback = [this] (const shared_library_t &library) {
    process_t &process = library.process ();

    /* Remove the breakpoints we've inserted when the library was loaded.  */
    auto &&breakpoint_range = process.range<breakpoint_t> ();
    for (auto it = breakpoint_range.begin (); it != breakpoint_range.end ();)
      it = (it->shared_library ().id () == library.id ())
               ? process.destroy (it)
               : ++it;

    /* Destruct the code objects.  */
    std::get<handle_object_set_t<code_object_t>> (m_handle_object_sets)
        .clear ();

    /* Disable the debug trap on all devices.  */
    clear_flag (flag_t::ENABLE_AGENT_DEBUG_MODE);

    amd_dbgapi_status_t status = update_agents ();
    if (status != AMD_DBGAPI_STATUS_SUCCESS)
      error ("update_agents failed (rc=%d)", status);

    enqueue_event (
        create<event_t> (*this, AMD_DBGAPI_EVENT_KIND_CODE_OBJECT_LIST_UPDATED,
                         AMD_DBGAPI_EVENT_NONE));

    enqueue_event (create<event_t> (*this, AMD_DBGAPI_EVENT_KIND_RUNTIME,
                                    AMD_DBGAPI_RUNTIME_STATE_UNLOADED));
  };

  /* Set/remove internal breakpoints when the runtime is loaded/unloaded.  */
  shared_library_t &library = create<shared_library_t> (
      *this, "libhsa-runtime64.so.1", on_runtime_load_callback,
      on_runtime_unload_callback);

  /* If the runtime is not yet loaded, create agents without enabling the
     debug trap.  */
  if (library.state () != AMD_DBGAPI_SHARED_LIBRARY_STATE_LOADED)
    {
      amd_dbgapi_status_t status = update_agents ();
      if (status == AMD_DBGAPI_STATUS_ERROR_RESTRICTION)
        {
          destroy (&library);
          return status;
        }
      else if (status != AMD_DBGAPI_STATUS_SUCCESS)
        error ("update_agents failed (rc=%d)", status);
    }

  return AMD_DBGAPI_STATUS_SUCCESS;
}

namespace
{

void
process_event_thread (
    std::vector<std::pair<file_desc_t, std::function<void ()>>> input,
    pipe_t event_thread_exit_pipe, std::promise<void> thread_exception)
{
  bool event_thread_exit{ false };

  /* Add the read end of the event_thread_exit_pipe to the monitored fds. This
     pipe is marked when the event thread needs to terminate.  */
  input.emplace_back (event_thread_exit_pipe.read_fd (),
                      [&] () { event_thread_exit = true; });

  std::vector<struct pollfd> fds;
  fds.reserve (input.size ());

  std::transform (input.begin (), input.end (), std::back_inserter (fds),
                  [] (auto entry) {
                    return pollfd{ entry.first, POLLIN, 0 };
                  });

  try
    {
      do
        {
          int ret = poll (fds.data (), fds.size (), -1);

          if (ret == -1 && errno != EINTR)
            error ("poll: %s", strerror (errno));
          else if (ret <= 0)
            continue;

          /* Check the file descriptors for data ready to read.  */
          for (size_t i = 0; i < fds.size (); ++i)
            {
              struct pollfd &fd = fds[i];

              if (!(fd.revents & POLLIN))
                continue;

              /* flush the event pipe, ...  */
              do
                {
                  char buf;
                  ret = read (fd.fd, &buf, 1);
                }
              while (ret >= 0 || (ret == -1 && errno == EINTR));

              if (ret == -1 && errno != EAGAIN)
                error ("read: %s", strerror (errno));

              /* ... and invoke the event notifier callback.  */
              (input[i].second) ();
            }
        }
      while (!event_thread_exit);
    }
  catch (...)
    {
      thread_exception.set_exception (std::current_exception ());
    }
}

} /* namespace */

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

  /* The input to the event thread loop is a vector of file descriptors to
     monitor (read end of the pipe created by the os_driver), and associated
     callbacks to invoke when data is present in the pipe.  The os_driver will
     write to the pipe when an event for an agent is pending, causing the event
     thread to wake up and invoke the callback for that agent.  */
  std::vector<std::pair<file_desc_t, std::function<void ()>>> file_descriptors;
  file_descriptors.reserve (count<agent_t> ());

  for (auto &&agent : range<agent_t> ())
    file_descriptors.emplace_back (agent.event_poll_fd (), [&agent, this] () {
      agent.set_pending_events ();
      m_client_notifier_pipe.mark ();
    });

  std::promise<void> event_thread_exception;
  m_event_thread_exception = event_thread_exception.get_future ();

  /* Start a new event thread. We capture a snapshot of the file descriptors
     and associated callback. If agents were to be added to or removed from the
     agent_map, we would stop this thread and start a new event thread with a
     new set of file descriptors, and notifiers. */

  m_event_thread = new std::thread (
      process_event_thread, std::move (file_descriptors),
      m_event_thread_exit_pipe, std::move (event_thread_exception));

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

    case AMD_DBGAPI_PROCESS_INFO_WATCHPOINT_COUNT:
      return utils::get_info (value_size, value, watchpoint_count ());

    case AMD_DBGAPI_PROCESS_INFO_WATCHPOINT_SHARE:
      return utils::get_info (value_size, value, watchpoint_shared_kind ());

    case AMD_DBGAPI_PROCESS_INFO_PRECISE_MEMORY_SUPPORTED:
      return utils::get_info (value_size, value,
                              AMD_DBGAPI_MEMORY_PRECISION_NONE);

    default:
      return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;
    }

  return AMD_DBGAPI_STATUS_SUCCESS;
}

void
process_t::enqueue_event (event_t &event)
{
  dbgapi_log (AMD_DBGAPI_LOG_LEVEL_INFO, "reporting %s, %s",
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

} /* namespace amd::dbgapi */

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
      return process->set_wave_launch_mode (os_wave_launch_mode_t::NORMAL);
    case AMD_DBGAPI_WAVE_CREATION_STOP:
      return process->set_wave_launch_mode (os_wave_launch_mode_t::HALT);
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
