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

#ifndef _AMD_DBGAPI_QUEUE_H
#define _AMD_DBGAPI_QUEUE_H 1

#include "agent.h"
#include "amd-dbgapi.h"
#include "debug.h"
#include "handle_object.h"
#include "os_driver.h"
#include "utils.h"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

namespace amd
{
namespace dbgapi
{

class architecture_t;
class process_t;

/* AMD Debugger API Queue.  */

class queue_t : public detail::handle_object<amd_dbgapi_queue_id_t>
{
private:
  class queue_impl_t; /* Base class for all queue implementations.  */

  /* TODO: Move the specific implementations to the detail namespace in
   * queue.cpp, need to expose access to queue_t's state from queue_impl_t.  */
  class aql_queue_impl_t; /* AQL queue implementation.  */
  class pm4_queue_impl_t; /* PM4 queue implemnatation.  */

public:
  enum class state_t
  {
    INVALID,   /* The queue is invalid. Calls to os_queue_id () will return the
                  OS_INVALID_QUEUEID.  Calls to process_t::find and
                  process_t::find_if will not return this queue.  Once a queue
                  becomes invalid, its state can no longer be changed.  */
    SUSPENDED, /* The queue is suspended, its state can be inspected.  */
    RUNNING    /* The queue is running.  */

  };

  queue_t (amd_dbgapi_queue_id_t queue_id, agent_t &agent,
           const os_queue_snapshot_entry_t &os_queue_info);

  /* Construct a temporary queue instance that must be updated by the next
     process_t::update_queues ().  */
  queue_t (amd_dbgapi_queue_id_t queue_id, agent_t &agent,
           os_queue_id_t os_queue_id);

  ~queue_t ();

  state_t state () const { return m_state; }
  void set_state (state_t state);

  bool is_valid () const { return m_state != state_t::INVALID; }
  bool is_suspended () const { return m_state == state_t::SUSPENDED; }
  bool is_running () const { return m_state == state_t::RUNNING; }

  os_queue_id_t os_queue_id () const
  {
    return is_valid () ? m_os_queue_info.queue_id : OS_INVALID_QUEUEID;
  }
  os_queue_type_t os_queue_type () const
  {
    return amd::dbgapi::os_queue_type (m_os_queue_info);
  }

  amd_dbgapi_queue_type_t type () const;

  epoch_t mark () const { return m_mark; }
  void set_mark (epoch_t mark) { m_mark = mark; }

  std::pair<amd_dbgapi_queue_packet_id_t, std::vector<uint8_t>>
  packets () const;

  amd_dbgapi_global_address_t displaced_stepping_buffer_address () const
  {
    return m_displaced_stepping_buffer_address;
  }
  amd_dbgapi_global_address_t parked_wave_buffer_address () const
  {
    return m_parked_wave_buffer_address;
  }
  amd_dbgapi_global_address_t endpgm_buffer_address () const
  {
    return m_endpgm_buffer_address;
  }

  amd_dbgapi_global_address_t scratch_backing_memory_address () const
  {
    dbgapi_assert (m_scratch_backing_memory_address);
    return m_scratch_backing_memory_address;
  }
  amd_dbgapi_size_t scratch_backing_memory_size () const
  {
    return m_scratch_backing_memory_size;
  }

  amd_dbgapi_global_address_t shared_address_space_aperture ()
  {
    dbgapi_assert (m_local_address_space_aperture);
    return m_local_address_space_aperture;
  }
  amd_dbgapi_global_address_t private_address_space_aperture ()
  {
    dbgapi_assert (m_private_address_space_aperture);
    return m_private_address_space_aperture;
  }

  amd_dbgapi_status_t get_info (amd_dbgapi_queue_info_t query,
                                size_t value_size, void *value) const;

  agent_t &agent () const { return m_agent; }
  process_t &process () const { return agent ().process (); }
  const architecture_t &architecture () const
  {
    return agent ().architecture ();
  }

private:
  os_queue_snapshot_entry_t const m_os_queue_info;
  state_t m_state{ state_t::RUNNING };

  amd_dbgapi_global_address_t m_displaced_stepping_buffer_address{ 0 };
  amd_dbgapi_global_address_t m_parked_wave_buffer_address{ 0 };
  amd_dbgapi_global_address_t m_endpgm_buffer_address{ 0 };

  amd_dbgapi_global_address_t m_scratch_backing_memory_address{ 0 };
  amd_dbgapi_global_address_t m_scratch_backing_memory_size{ 0 };

  amd_dbgapi_global_address_t m_local_address_space_aperture{ 0 };
  amd_dbgapi_global_address_t m_private_address_space_aperture{ 0 };

  epoch_t m_mark{ 0 };

  /* Value used to mark waves that are found in the context save area. When
     sweeping, any wave found with a mark less than the current mark will be
     deleted, as these waves are no longer active.  */
  monotonic_counter_t<epoch_t> m_next_wave_mark{ 1 };

  std::unique_ptr<queue_impl_t> m_impl;
  agent_t &m_agent;
};

/* Wraps a queue and provides a RAII mechanism to suspend it if it wasn't
   already suspended. The queue is suspended when the object is constructed,
   if the queue is not invalid or not already suspended.  When control leaves
   the scope in which the object was created, the queue is resumed if it was
   suspended by this instance of scoped_queue_suspend_t, the queue is still
   valid, and forward progress is required".
 */
class scoped_queue_suspend_t
{
public:
  scoped_queue_suspend_t (queue_t &queue);
  ~scoped_queue_suspend_t ();

  /* Disable copies.  */
  scoped_queue_suspend_t (const scoped_queue_suspend_t &) = delete;
  scoped_queue_suspend_t &operator= (const scoped_queue_suspend_t &) = delete;

public:
  queue_t *m_queue;
};

} /* namespace dbgapi */
} /* namespace amd */

#endif /* _AMD_DBGAPI_QUEUE_H */
