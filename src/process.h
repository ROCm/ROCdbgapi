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

#ifndef _AMD_DBGAPI_PROCESS_H
#define _AMD_DBGAPI_PROCESS_H 1

#include "defs.h"

#include "agent.h"
#include "callbacks.h"
#include "code_object.h"
#include "dispatch.h"
#include "displaced_stepping.h"
#include "event.h"
#include "handle_object.h"
#include "logging.h"
#include "queue.h"
#include "utils.h"
#include "wave.h"

#include <sys/types.h>

#include <functional>
#include <future>
#include <queue>
#include <string>
#include <thread>
#include <type_traits>
#include <vector>

#define TRACE_CALLBACK(prefix, ...)                                           \
  tracer __tracer__##__COUNTER__ { "[callback]", __FUNCTION__, ##__VA_ARGS__ }

namespace amd
{
namespace dbgapi
{

/* AMD Debugger API Process.  */

class process_t
{
  using notify_shared_library_callback_t = std::function<void (
      amd_dbgapi_shared_library_id_t, amd_dbgapi_shared_library_state_t)>;

  using handle_object_sets_t
      = handle_object_set_tuple_t<agent_t, breakpoint_t, code_object_t,
                                  dispatch_t, displaced_stepping_t, event_t,
                                  queue_t, shared_library_t, wave_t>;

  int dbg_trap_ioctl (uint32_t action, kfd_ioctl_dbg_trap_args *args);

public:
  enum class wave_launch_mode_t : uint32_t
  {
    NORMAL = 0,      /* Waves launch normally.  */
    HALT = 1,        /* Waves launch in halted mode.  */
    KILL = 2,        /* Waves terminate before executing any instructions.  */
    SINGLE_STEP = 3, /* Waves launch in single-step mode.  */
    DISABLE = 4,     /* Disable launching any new waves.  */
  };

  process_t (amd_dbgapi_client_process_id_t client_process_id,
             amd_dbgapi_process_id_t process_id);
  ~process_t () {}
  bool is_valid () const;

  amd_dbgapi_process_id_t id () const { return m_process_id; }
  amd_dbgapi_client_process_id_t client_id () const
  {
    return m_client_process_id;
  }

  pid_t os_pid () const { return m_os_pid; }

  amd_dbgapi_status_t
  read_global_memory_partial (amd_dbgapi_global_address_t address,
                              void *buffer, size_t *size);
  amd_dbgapi_status_t
  write_global_memory_partial (amd_dbgapi_global_address_t address,
                               const void *buffer, size_t *size);

  amd_dbgapi_status_t read_global_memory (amd_dbgapi_global_address_t address,
                                          void *buffer, size_t size);
  amd_dbgapi_status_t write_global_memory (amd_dbgapi_global_address_t address,
                                           const void *buffer, size_t size);

  amd_dbgapi_status_t read_string (amd_dbgapi_global_address_t address,
                                   std::string *string, size_t size);

  bool forward_progress_needed () const { return m_forward_progress_needed; }
  amd_dbgapi_status_t
  set_forward_progress_needed (bool forward_progress_needed);

  wave_launch_mode_t wave_launch_mode () const { return m_wave_launch_mode; }
  amd_dbgapi_status_t
  set_wave_launch_mode (wave_launch_mode_t wave_launch_mode);

  amd_dbgapi_status_t update_agents (bool enable_debug_trap);
  amd_dbgapi_status_t update_queues ();
  amd_dbgapi_status_t update_code_objects ();

  amd_dbgapi_status_t start_event_thread ();
  amd_dbgapi_status_t stop_event_thread ();
  void check_event_thread ();

  amd_dbgapi_status_t attach ();
  void detach ();

  void enqueue_event (event_t &event);
  event_t *dequeue_event ();

  amd_dbgapi_status_t enable_debug_trap (const agent_t &agent,
                                         file_desc_t *poll_fd);
  amd_dbgapi_status_t disable_debug_trap (const agent_t &agent);

  amd_dbgapi_status_t query_debug_event (const agent_t &agent,
                                         queue_t::kfd_queue_id_t *kfd_queue_id,
                                         uint32_t *status);

  amd_dbgapi_status_t suspend_queues (const std::vector<queue_t *> &queues,
                                      queue_t::update_waves_flag_t flags = {});
  amd_dbgapi_status_t resume_queues (const std::vector<queue_t *> &queues);

  amd_dbgapi_status_t set_wave_launch_mode (const agent_t &agent,
                                            wave_launch_mode_t mode);

  static process_t *find (amd_dbgapi_process_id_t process_id,
                          bool flush_cache = false);

  static process_t *find (amd_dbgapi_client_process_id_t client_process_id,
                          bool flush_cache = false);

  amd_dbgapi_status_t get_info (amd_dbgapi_process_info_t query,
                                size_t value_size, void *value) const;

  amd_dbgapi_status_t get_os_pid (pid_t *pid) const
  {
    TRACE_CALLBACK ();
    return (*process_callbacks.get_os_pid) (m_client_process_id, pid);
  }

  amd_dbgapi_status_t
  get_symbol_address (amd_dbgapi_shared_library_id_t library_id,
                      const char *symbol_name,
                      amd_dbgapi_global_address_t *address) const
  {
    TRACE_CALLBACK (library_id, symbol_name, address);
    return (*process_callbacks.get_symbol_address) (
        m_client_process_id, library_id, symbol_name, address);
  }

  amd_dbgapi_status_t enable_notify_shared_library (
      const char *library_name, amd_dbgapi_shared_library_id_t library_id,
      amd_dbgapi_shared_library_state_t *library_state)
  {
    TRACE_CALLBACK (library_name, library_id);
    return (*process_callbacks.enable_notify_shared_library) (
        m_client_process_id, library_name, library_id, library_state);
  }

  amd_dbgapi_status_t
  disable_notify_shared_library (amd_dbgapi_shared_library_id_t library_id)
  {
    TRACE_CALLBACK (library_id);
    return (*process_callbacks.disable_notify_shared_library) (
        m_client_process_id, library_id);
  }

  amd_dbgapi_status_t
  add_breakpoint (amd_dbgapi_shared_library_id_t shared_library_id,
                  amd_dbgapi_global_address_t address,
                  amd_dbgapi_breakpoint_id_t breakpoint_id)
  {
    TRACE_CALLBACK (address, breakpoint_id);
    return (*process_callbacks.add_breakpoint) (
        m_client_process_id, shared_library_id, address, breakpoint_id);
  }

  amd_dbgapi_status_t
  remove_breakpoint (amd_dbgapi_breakpoint_id_t breakpoint_id)
  {
    TRACE_CALLBACK (breakpoint_id);
    return (*process_callbacks.remove_breakpoint) (m_client_process_id,
                                                   breakpoint_id);
  }

  amd_dbgapi_status_t
  set_breakpoint_state (amd_dbgapi_breakpoint_id_t breakpoint_id,
                        amd_dbgapi_breakpoint_state_t breakpoint_state)
  {
    TRACE_CALLBACK (breakpoint_id, breakpoint_state);
    return (*process_callbacks.set_breakpoint_state) (
        m_client_process_id, breakpoint_id, breakpoint_state);
  }

  template <typename Object, typename... Args> Object &create (Args &&... args)
  {
    return m_handle_object_sets.get<Object> ().create_object (args...);
  }

  /* Destroy the given object.  */
  template <typename Object> void destroy (Object *object)
  {
    m_handle_object_sets.get<Object> ().destroy (object);
  }

  /* Destroy the object at object_it.  This should be used instead of
     process::destroy (Object *) if destroying objects while iterating
     with a process::range<Object>.  */
  template <typename ObjectIterator>
  ObjectIterator destroy (ObjectIterator object_it)
  {
    return m_handle_object_sets.get<typename ObjectIterator::value_type> ()
        .destroy (object_it);
  }

  /* Return an Object range. A range implements begin () and end (), and
     can be used to iterate the Objects.  */
  template <typename Object>
  typename handle_object_set_t<Object>::range_t range ()
  {
    return m_handle_object_sets.get<Object> ().range ();
  }

  /* Return the element count for the sub-Object.  */
  template <typename Object> size_t count () const
  {
    return m_handle_object_sets.get<Object> ().size ();
  }

  /* Return whether the Objects have changed since the last time changed
     was called. The flag is set whenever objects are created or destroyed,
     and cleared when it is queried.  */
  template <typename Object> bool reset_changed ()
  {
    return m_handle_object_sets.get<Object> ().reset_changed ();
  }

  /* Find an object with the given handle.  */
  template <typename Handle>
  typename handle_object_sets_t::find_object_type_from_handle<Handle>::type *
  find (Handle id)
  {
    using object_type =
        typename handle_object_sets_t::find_object_type_from_handle<
            Handle>::type;
    return m_handle_object_sets.get<object_type> ().find (id);
  }

  /* Find an object for which the unary predicate f returns true.  */
  template <typename Func>
  typename std::decay<typename utils::first_argument_type<Func>::type>::type *
  find_if (Func predicate)
  {
    return m_handle_object_sets
        .get<typename std::decay<
            typename utils::first_argument_type<Func>::type>::type> ()
        .find_if (predicate);
  }

  pipe_t &client_notifier_pipe () { return m_client_notifier_pipe; }

private:
  amd_dbgapi_process_id_t const m_process_id;
  amd_dbgapi_client_process_id_t const m_client_process_id;
  amd_dbgapi_global_address_t m_r_debug_address{ 0 };

  pid_t m_os_pid{ -1 };
  bool m_process_exited{ false };

  wave_launch_mode_t m_wave_launch_mode{ wave_launch_mode_t::NORMAL };
  bool m_forward_progress_needed{ true };
  bool m_initialized{ false };

  file_desc_t m_kfd_fd{ -1 };
  file_desc_t m_proc_mem_fd{ -1 };

  std::thread *m_event_thread{ nullptr };
  std::future<void> m_event_thread_exception;

  pipe_t m_client_notifier_pipe;
  pipe_t m_event_thread_exit_pipe;

  std::queue<event_t *> m_pending_events;

  /* Value used to mark queues that are reported by KFD. When sweeping, any
     queue found with a mark less than the current mark will be deleted, as
     these queues are no longer active.  */
  monotonic_counter_t<epoch_t> m_next_queue_mark{ 1 };

  /* Value used to mark code objects that are reported by the ROCR. When
     sweeping, any code object found with a mark less than the current mark
     will be deleted, as these code objects are not longer loaded.  */
  monotonic_counter_t<epoch_t> m_next_code_object_mark{ 1 };

  handle_object_sets_t m_handle_object_sets;
};

static inline void *
allocate_memory (size_t byte_size)
{
  TRACE_CALLBACK (byte_size);
  return (*process_callbacks.allocate_memory) (byte_size);
}

static inline void
deallocate_memory (void *data)
{
  TRACE_CALLBACK (data);
  (*process_callbacks.deallocate_memory) (data);
}

static inline void
log_message (amd_dbgapi_log_level_t level, const char *message)
{
  return (*process_callbacks.log_message) (level, message);
}

template <>
inline std::string
to_string (process_t::wave_launch_mode_t mode)
{
  switch (mode)
    {
    case process_t::wave_launch_mode_t::NORMAL:
      return "WAVE_LAUNCH_MODE_NORMAL";
    case process_t::wave_launch_mode_t::HALT:
      return "WAVE_LAUNCH_MODE_HALT";
    case process_t::wave_launch_mode_t::KILL:
      return "WAVE_LAUNCH_MODE_KILL";
    case process_t::wave_launch_mode_t::SINGLE_STEP:
      return "WAVE_LAUNCH_MODE_SINGLE_STEP";
    case process_t::wave_launch_mode_t::DISABLE:
      return "WAVE_LAUNCH_MODE_DISABLE";
    }
  return to_string (make_hex (
      static_cast<std::underlying_type<decltype (mode)>::type> (mode)));
}

#undef TRACE_CALLBACK

} /* namespace dbgapi */
} /* namespace amd */

#endif /* _AMD_DBGAPI_PROCESS_H */
