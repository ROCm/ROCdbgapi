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

#ifndef AMD_DBGAPI_PROCESS_H
#define AMD_DBGAPI_PROCESS_H 1

#include "amd-dbgapi.h"
#include "architecture.h"
#include "callbacks.h"
#include "code_object.h"
#include "debug.h"
#include "dispatch.h"
#include "displaced_stepping.h"
#include "event.h"
#include "handle_object.h"
#include "initialization.h"
#include "logging.h"
#include "os_driver.h"
#include "queue.h"
#include "utils.h"
#include "watchpoint.h"
#include "wave.h"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <future>
#include <list>
#include <memory>
#include <queue>
#include <string>
#include <thread>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

#define TRACE_CALLBACK(prefix, ...)                                           \
  tracer __tracer__##__COUNTER__ { "[callback]", __FUNCTION__, ##__VA_ARGS__ }

namespace amd::dbgapi
{

extern std::list<class process_t *> process_list;
extern amd_dbgapi_callbacks_s process_callbacks;

/* AMD Debugger API Process.  */

class process_t
{
  using notify_shared_library_callback_t = std::function<void (
      amd_dbgapi_shared_library_id_t, amd_dbgapi_shared_library_state_t)>;

public:
  enum class flag_t : uint32_t
  {
    /* Enable the device debug mode when updating the agents.  */
    ENABLE_AGENT_DEBUG_MODE = 1 << 0,
    /* Require the NEW_QUEUE bit to be set when a queue_id is reported for the
       first time by kfd to this process. When attaching to an already running
       process, a missing NEW_BIT may be ignored as it could have been cleared
       by another debugger session.  */
    REQUIRE_NEW_QUEUE_BIT = 1 << 1,
    /* Assign new ids to all waves regardless of the content of their wave_id
       register.  This is needed during attach as waves created before the
       debugger attached to the process may have corrupted wave_ids.  */
    ASSIGN_NEW_IDS_TO_ALL_WAVES = 1 << 2,
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

  const os_driver_t &os_driver () const { return *m_os_driver; }

  inline void set_flag (flag_t flags);
  inline void clear_flag (flag_t flags);
  inline bool is_flag_set (flag_t flags) const;

  amd_dbgapi_status_t
  read_global_memory_partial (amd_dbgapi_global_address_t address,
                              void *buffer, size_t *size) const;
  amd_dbgapi_status_t
  write_global_memory_partial (amd_dbgapi_global_address_t address,
                               const void *buffer, size_t *size) const;

  amd_dbgapi_status_t read_global_memory (amd_dbgapi_global_address_t address,
                                          void *buffer, size_t size) const;
  amd_dbgapi_status_t write_global_memory (amd_dbgapi_global_address_t address,
                                           const void *buffer,
                                           size_t size) const;

  amd_dbgapi_status_t read_string (amd_dbgapi_global_address_t address,
                                   std::string *string, size_t size) const;

  bool forward_progress_needed () const { return m_forward_progress_needed; }
  void set_forward_progress_needed (bool forward_progress_needed);

  os_wave_launch_mode_t wave_launch_mode () const
  {
    return m_wave_launch_mode;
  }
  amd_dbgapi_status_t
  set_wave_launch_mode (os_wave_launch_mode_t wave_launch_mode);

  amd_dbgapi_status_t
  set_wave_launch_trap_override (os_wave_launch_trap_mask_t mask,
                                 os_wave_launch_trap_mask_t bits);

  /* Suspend/resume a list of queues.  Queues may become invalid as a result of
     suspension/resumption, but not destroyed.  Queues made invalid will
     destroy associated dispatches and waves.  Since waves/dispatches can be
     destroyed, the caller is responsible for refetching wave/dispatche
     instances from their id.  */
  size_t suspend_queues (const std::vector<queue_t *> &queues) const;
  size_t resume_queues (const std::vector<queue_t *> &queues) const;

  /* update_* ensures that the only objects that exist are exactly those
     reported by the os_driver.  It creates new objects reported by os_driver,
     and destroy objects that no longer exist which includes objects that are
     no longer valid.  When an object is deleted, all objects associated with
     it are also deleted.  */
  amd_dbgapi_status_t update_agents ();
  amd_dbgapi_status_t update_queues ();
  amd_dbgapi_status_t update_code_objects ();

  amd_dbgapi_status_t start_event_thread ();
  amd_dbgapi_status_t stop_event_thread ();
  void check_event_thread ();

  amd_dbgapi_status_t attach ();
  void detach ();

  void enqueue_event (event_t &event);
  event_t *dequeue_event ();

  size_t watchpoint_count () const;
  amd_dbgapi_watchpoint_share_kind_t watchpoint_shared_kind () const;
  amd_dbgapi_status_t
  insert_watchpoint (const watchpoint_t &watchpoint,
                     amd_dbgapi_global_address_t *adjusted_address,
                     amd_dbgapi_global_address_t *adjusted_size);
  void remove_watchpoint (const watchpoint_t &watchpoint);

  static process_t *find (amd_dbgapi_process_id_t process_id,
                          bool flush_cache = false);

  static process_t *find (amd_dbgapi_client_process_id_t client_process_id,
                          bool flush_cache = false);

  amd_dbgapi_status_t get_info (amd_dbgapi_process_info_t query,
                                size_t value_size, void *value) const;

  amd_dbgapi_status_t get_os_pid (amd_dbgapi_os_process_id_t *pid) const
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
  insert_breakpoint (amd_dbgapi_shared_library_id_t shared_library_id,
                     amd_dbgapi_global_address_t address,
                     amd_dbgapi_breakpoint_id_t breakpoint_id)
  {
    TRACE_CALLBACK (address, breakpoint_id);
    return (*process_callbacks.insert_breakpoint) (
        m_client_process_id, shared_library_id, address, breakpoint_id);
  }

  amd_dbgapi_status_t
  remove_breakpoint (amd_dbgapi_breakpoint_id_t breakpoint_id)
  {
    TRACE_CALLBACK (breakpoint_id);
    return (*process_callbacks.remove_breakpoint) (m_client_process_id,
                                                   breakpoint_id);
  }

  template <typename Object, typename... Args> auto &create (Args &&... args)
  {
    return std::get<handle_object_set_t<Object>> (m_handle_object_sets)
        .create_object (std::forward<Args> (args)...);
  }

  /* Destroy the given object.  */
  template <typename Object> void destroy (Object *object)
  {
    std::get<handle_object_set_t<Object>> (m_handle_object_sets)
        .destroy (object);
  }

  /* Destroy the object at object_it.  This should be used instead of
     process::destroy (Object *) if destroying objects while iterating
     with a process::range<Object>.  */
  template <typename ObjectIterator>
  ObjectIterator destroy (ObjectIterator object_it)
  {
    return std::get<handle_object_set_t<typename ObjectIterator::value_type>> (
               m_handle_object_sets)
        .destroy (object_it);
  }

  /* Return an Object range. A range implements begin () and end (), and
     can be used to iterate the Objects.  */
  template <typename Object> auto range ()
  {
    return std::get<handle_object_set_t<Object>> (m_handle_object_sets)
        .range ();
  }
  template <typename Object> auto range () const
  {
    return std::get<handle_object_set_t<Object>> (m_handle_object_sets)
        .range ();
  }

  /* Return the element count for the sub-Object.  */
  template <typename Object> size_t count () const
  {
    return std::get<handle_object_set_t<Object>> (m_handle_object_sets)
        .size ();
  }

  template <typename Object> bool changed () const
  {
    return std::get<handle_object_set_t<Object>> (m_handle_object_sets)
        .changed ();
  }

  /* Set the flag that indicates whether the Objects have changed. Return its
     previous value. The flag is set whenever objects are created, or
     destroyed, or invalidated.  */
  template <typename Object> bool set_changed (bool changed)
  {
    return std::get<handle_object_set_t<Object>> (m_handle_object_sets)
        .set_changed (changed);
  }

  /* Find an object with the given handle.  */
  template <typename Handle> auto *find (Handle id)
  {
    using object_type
        = object_type_from_handle_t<Handle, decltype (m_handle_object_sets)>;

    return std::get<handle_object_set_t<object_type>> (m_handle_object_sets)
        .find (id);
  }

  /* Find an object for which the unary predicate f returns true.  */
  template <typename Functor> auto *find_if (Functor predicate)
  {
    using object_type = std::decay_t<utils::first_argument_of_t<Functor>>;

    return std::get<handle_object_set_t<object_type>> (m_handle_object_sets)
        .find_if (predicate);
  }

  pipe_t &client_notifier_pipe () { return m_client_notifier_pipe; }

private:
  amd_dbgapi_process_id_t const m_process_id;
  amd_dbgapi_client_process_id_t const m_client_process_id;
  amd_dbgapi_global_address_t m_r_debug_address{ 0 };

  std::unique_ptr<const os_driver_t> m_os_driver;
  flag_t m_flags{};

  os_wave_launch_mode_t m_wave_launch_mode{ os_wave_launch_mode_t::NORMAL };
  os_wave_launch_trap_mask_t m_wave_trap_mask{
    os_wave_launch_trap_mask_t::NONE
  };
  bool m_forward_progress_needed{ true };

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

  std::tuple<
      handle_object_set_t<agent_t>, handle_object_set_t<breakpoint_t>,
      handle_object_set_t<code_object_t>, handle_object_set_t<dispatch_t>,
      handle_object_set_t<displaced_stepping_t>, handle_object_set_t<event_t>,
      handle_object_set_t<queue_t>, handle_object_set_t<shared_library_t>,
      handle_object_set_t<watchpoint_t>, handle_object_set_t<wave_t>>
      m_handle_object_sets;
};

template <> struct is_flag<process_t::flag_t> : std::true_type
{
};

inline void
process_t::set_flag (flag_t flags)
{
  m_flags |= flags;
}

inline void
process_t::clear_flag (flag_t flags)
{
  m_flags &= ~flags;
}

inline bool
process_t::is_flag_set (flag_t flag) const
{
  dbgapi_assert (utils::is_power_of_two (
                     static_cast<std::underlying_type_t<flag_t>> (flag))
                 && "can only check one flag at a time");
  return (m_flags & flag) != 0;
}

inline void *
allocate_memory (size_t byte_size)
{
  TRACE_CALLBACK (byte_size);
  return (*process_callbacks.allocate_memory) (byte_size);
}

inline void
deallocate_memory (void *data)
{
  TRACE_CALLBACK (data);
  (*process_callbacks.deallocate_memory) (data);
}

inline void
log_message (amd_dbgapi_log_level_t level, const char *message)
{
  return (*process_callbacks.log_message) (level, message);
}

#undef TRACE_CALLBACK

} /* namespace amd::dbgapi */

#endif /* AMD_DBGAPI_PROCESS_H */
