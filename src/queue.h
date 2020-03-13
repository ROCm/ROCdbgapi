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

#include "defs.h"

#include "agent.h"
#include "handle_object.h"
#include "utils.h"

#include "linux/kfd_ioctl.h"

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
  struct context_save_area_header_s
  {
    uint32_t ctrl_stack_offset;
    uint32_t ctrl_stack_size;
    uint32_t wave_state_offset;
    uint32_t wave_state_size;
  };

public:
  using kfd_queue_id_t = uint32_t;

  queue_t (amd_dbgapi_queue_id_t queue_id, agent_t &agent,
           const kfd_queue_snapshot_entry &kfd_queue_info);

  ~queue_t ();

  kfd_queue_id_t kfd_queue_id () const { return m_kfd_queue_info.queue_id; }
  uint32_t kfd_queue_type () const { return m_kfd_queue_info.queue_type; }

  bool suspended () const { return m_suspended; }
  void set_suspended (bool suspended) { m_suspended = suspended; }

  epoch_t mark () const { return m_mark; }
  void set_mark (epoch_t mark) { m_mark = mark; }

  amd_dbgapi_status_t update_waves (bool always_assign_wave_ids);

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

  amd_dbgapi_global_address_t scratch_backing_memory_address () const;
  amd_dbgapi_size_t scratch_backing_memory_size () const;

  amd_dbgapi_status_t get_info (amd_dbgapi_queue_info_t query,
                                size_t value_size, void *value) const;

  agent_t &agent () const { return m_agent; }
  process_t &process () const { return agent ().process (); }
  const architecture_t &architecture () const
  {
    return agent ().architecture ();
  }

private:
  kfd_queue_snapshot_entry const m_kfd_queue_info;
  amd_dbgapi_global_address_t m_displaced_stepping_buffer_address{ 0 };
  amd_dbgapi_global_address_t m_parked_wave_buffer_address{ 0 };
  amd_dbgapi_global_address_t m_endpgm_buffer_address{ 0 };
  amd_dbgapi_global_address_t m_context_save_start_address;

  epoch_t m_mark{ 0 };
  bool m_suspended{ false };

  /* Value used to mark waves that are found in the context save area. When
     sweeping, any wave found with a mark less than the current mark will be
     deleted, as these waves are no longer active.  */
  monotonic_counter_t<epoch_t> m_next_wave_mark{ 1 };

  agent_t &m_agent;
};

/* Wraps a queue and provides a RAII mechanism to suspend it if it wasn't
   already suspended. The queue is suspended when the object is constructed,
   and when control leaves the scope in which the object was created, the
   queue is resumed if and only if the queue was not already suspended when
   the object was created, and forward progress is required".
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
  queue_t *const m_queue;
};

} /* namespace dbgapi */
} /* namespace amd */

#endif /* _AMD_DBGAPI_QUEUE_H */
