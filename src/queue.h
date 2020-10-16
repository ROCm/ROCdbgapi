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

#ifndef AMD_DBGAPI_QUEUE_H
#define AMD_DBGAPI_QUEUE_H 1

#include "agent.h"
#include "amd-dbgapi.h"
#include "handle_object.h"
#include "os_driver.h"
#include "utils.h"

#include <cstddef>
#include <memory>

namespace amd::dbgapi
{

class architecture_t;
class process_t;

/* AMD Debugger API Queue.  */

class queue_t : public detail::handle_object<amd_dbgapi_queue_id_t>
{
public:
  class queue_impl_t; /* Base class for all queue implementations.  */

  enum class state_t
  {
    invalid,   /* The queue is invalid. Calls to os_queue_id () will return the
                  os_invalid_queueid.  Calls to process_t::find and
                  process_t::find_if will not return this queue.  Once a queue
                  becomes invalid, its state can no longer be changed.  */
    suspended, /* The queue is suspended, its state can be inspected.  */
    running    /* The queue is running.  */

  };

private:
  state_t m_state{ state_t::running };
  epoch_t m_mark{ 0 };

  agent_t &m_agent;

  /* Must be initialized last.  */
  std::unique_ptr<queue_impl_t> m_impl;

public:
  queue_t (amd_dbgapi_queue_id_t queue_id, agent_t &agent,
           const os_queue_snapshot_entry_t &os_queue_info);

  /* Construct a temporary queue instance that must be updated by the next
     process_t::update_queues ().  */
  queue_t (amd_dbgapi_queue_id_t queue_id, agent_t &agent,
           os_queue_id_t os_queue_id);

  ~queue_t ();

  state_t state () const { return m_state; }
  void set_state (state_t state);

  bool is_valid () const { return m_state != state_t::invalid; }
  bool is_suspended () const { return m_state == state_t::suspended; }
  bool is_running () const { return m_state == state_t::running; }

  os_queue_id_t os_queue_id () const;

  epoch_t mark () const { return m_mark; }
  void set_mark (epoch_t mark) { m_mark = mark; }

  amd_dbgapi_status_t
  active_packets_info (amd_dbgapi_os_queue_packet_id_t *read_packet_id_p,
                       amd_dbgapi_os_queue_packet_id_t *write_packet_id_p,
                       size_t *packets_byte_size_p) const;

  amd_dbgapi_status_t
  active_packets_bytes (amd_dbgapi_os_queue_packet_id_t read_packet_id,
                        amd_dbgapi_os_queue_packet_id_t write_packet_id,
                        void *memory, size_t memory_size) const;

  amd_dbgapi_status_t get_info (amd_dbgapi_queue_info_t query,
                                size_t value_size, void *value) const;

  agent_t &agent () const { return m_agent; }
  process_t &process () const { return agent ().process (); }
  const architecture_t &architecture () const
  {
    return agent ().architecture ();
  }
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
  scoped_queue_suspend_t (queue_t &queue, const char *reason);
  ~scoped_queue_suspend_t ();

  /* Disable copies.  */
  scoped_queue_suspend_t (const scoped_queue_suspend_t &) = delete;
  scoped_queue_suspend_t &operator= (const scoped_queue_suspend_t &) = delete;

public:
  const char *const m_reason;
  queue_t *m_queue;
};

} /* namespace amd::dbgapi */

#endif /* AMD_DBGAPI_QUEUE_H */
