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

#include "queue.h"
#include "architecture.h"
#include "debug.h"
#include "dispatch.h"
#include "handle_object.h"
#include "initialization.h"
#include "logging.h"
#include "process.h"
#include "register.h"
#include "utils.h"
#include "wave.h"

#include <hsa/amd_hsa_queue.h>
#include <hsa/hsa.h>

#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <tuple>
#include <type_traits>

namespace amd::dbgapi
{

constexpr uint32_t SQ_WAVE_STATUS_HALT_MASK = utils::bit_mask (13, 13);
constexpr uint32_t TTMP11_TRAP_HANDLER_EVENTS_MASK = utils::bit_mask (7, 8);

/* Base class for all queue implementations.  */

class queue_t::queue_impl_t
{
public:
  queue_impl_t (queue_t &queue) : m_queue (queue) {}
  virtual ~queue_impl_t () = default;

  /* Return a snapshot of the packets present in this queue.  */
  virtual std::pair<amd_dbgapi_os_queue_packet_id_t, size_t>
  packets (void **packets_bytes) const = 0;

  /* Return the queue's type.  */
  virtual amd_dbgapi_os_queue_type_t type () const = 0;

  /* Notify the impl that the queue was suspended/resumed.  */
  virtual void state_changed (state_t) {}

protected:
  queue_t &m_queue;
};

/* AQL Queue implementation.  */

class queue_t::aql_queue_impl_t : public queue_impl_t
{
private:
  static constexpr uint64_t AQL_PACKET_SIZE = 64;

  struct context_save_area_header_s
  {
    uint32_t ctrl_stack_offset;
    uint32_t ctrl_stack_size;
    uint32_t wave_state_offset;
    uint32_t wave_state_size;
  };

  amd_dbgapi_status_t update_waves ();

public:
  aql_queue_impl_t (queue_t &queue);
  virtual ~aql_queue_impl_t () override;

  std::pair<amd_dbgapi_os_queue_packet_id_t, size_t>
  packets (void **packets_bytes) const override;

  virtual amd_dbgapi_os_queue_type_t type () const override;

  virtual void state_changed (state_t state) override;

private:
  amd_dbgapi_global_address_t m_context_save_start_address;
  hsa_queue_t m_hsa_queue;
};

queue_t::aql_queue_impl_t::aql_queue_impl_t (queue_t &queue)
    : queue_impl_t (queue)
{
  const architecture_t &architecture = m_queue.architecture ();
  process_t &process = m_queue.process ();
  amd_dbgapi_status_t status;

  m_context_save_start_address
      = m_queue.m_os_queue_info.ctx_save_restore_address
        + sizeof (context_save_area_header_s);

  /* FIXME: This is only temporary, we are using the free space in the queue
     control stack memory. The control stack grows from high to low address, so
     we can steal bytes between the context save area header and the top of
     stack limit. update_waves () checks that the area is not overwritten.  */

  m_queue.m_displaced_stepping_buffer_address = m_context_save_start_address;
  m_context_save_start_address
      += architecture.displaced_stepping_buffer_size ();

  m_queue.m_parked_wave_buffer_address = m_context_save_start_address;
  m_context_save_start_address
      += architecture.breakpoint_instruction ().size ();

  status = process.write_global_memory (
      m_queue.m_parked_wave_buffer_address,
      architecture.breakpoint_instruction ().data (),
      architecture.breakpoint_instruction ().size ());
  if (status != AMD_DBGAPI_STATUS_SUCCESS)
    error ("Could not write to the parked wave instruction buffer (rc=%d)",
           status);

  m_queue.m_endpgm_buffer_address = m_context_save_start_address;
  m_context_save_start_address += architecture.endpgm_instruction ().size ();

  status = process.write_global_memory (
      m_queue.m_endpgm_buffer_address,
      architecture.endpgm_instruction ().data (),
      architecture.endpgm_instruction ().size ());
  if (status != AMD_DBGAPI_STATUS_SUCCESS)
    error ("Could not write to the endpgm instruction buffer (rc=%d)", status);

  /* Read the local and private apertures.  */

  /* The group_segment_aperture_base_hi is stored in the ABI-stable part of the
     amd_queue_t. Since we know the address of the read_dispatch_id (obtained
     from the KFD through the queue snapshot info), which is also stored in the
     ABI-stable part of the amd_queue_t, we can calculate the address of the
     pointer to the group_segment_aperture_base_hi and read it.   */

  uint32_t group_segment_aperture_base_hi;
  status = process.read_global_memory (
      m_queue.m_os_queue_info.read_pointer_address
          + offsetof (amd_queue_t, group_segment_aperture_base_hi)
          - offsetof (amd_queue_t, read_dispatch_id),
      &group_segment_aperture_base_hi,
      sizeof (group_segment_aperture_base_hi));
  if (status != AMD_DBGAPI_STATUS_SUCCESS)
    error ("Could not read the queue's group_segment_aperture_base_hi (rc=%d)",
           status);

  m_queue.m_local_address_space_aperture
      = amd_dbgapi_global_address_t{ group_segment_aperture_base_hi } << 32;

  uint32_t private_segment_aperture_base_hi;
  status = process.read_global_memory (
      m_queue.m_os_queue_info.read_pointer_address
          + offsetof (amd_queue_t, private_segment_aperture_base_hi)
          - offsetof (amd_queue_t, read_dispatch_id),
      &private_segment_aperture_base_hi,
      sizeof (private_segment_aperture_base_hi));
  if (status != AMD_DBGAPI_STATUS_SUCCESS)
    error (
        "Could not read the queue's private_segment_aperture_base_hi (rc=%d)",
        status);

  m_queue.m_private_address_space_aperture
      = amd_dbgapi_global_address_t{ private_segment_aperture_base_hi } << 32;

  /* Read the hsa_queue_t at the top of the amd_queue_t. Since the amd_queue_t
    structure could change, it can only be accessed by calculating its address
    from the address of the read_dispatch_id by subtracting
    read_dispatch_id_field_base_byte_offset .  */

  uint32_t read_dispatch_id_field_base_byte_offset;
  status = process.read_global_memory (
      m_queue.m_os_queue_info.read_pointer_address
          + offsetof (amd_queue_t, read_dispatch_id_field_base_byte_offset)
          - offsetof (amd_queue_t, read_dispatch_id),
      &read_dispatch_id_field_base_byte_offset,
      sizeof (read_dispatch_id_field_base_byte_offset));
  if (status != AMD_DBGAPI_STATUS_SUCCESS)
    error ("Could not read the queue's "
           "read_dispatch_id_field_base_byte_offset (rc=%d)",
           status);

  amd_dbgapi_global_address_t hsa_queue_address
      = m_queue.m_os_queue_info.read_pointer_address
        - read_dispatch_id_field_base_byte_offset;
  status = process.read_global_memory (hsa_queue_address, &m_hsa_queue,
                                       sizeof (m_hsa_queue));
  if (status != AMD_DBGAPI_STATUS_SUCCESS)
    error ("Could not read the hsa_queue_t struct (rc=%d)", status);

  if (reinterpret_cast<uintptr_t> (m_hsa_queue.base_address)
      != m_queue.m_os_queue_info.ring_base_address)
    error ("hsa_queue_t base address != kfd queue info base address");

  if ((m_hsa_queue.size * 64) != m_queue.m_os_queue_info.ring_size)
    error ("hsa_queue_t size != kfd queue info ring size");
}

queue_t::aql_queue_impl_t::~aql_queue_impl_t ()
{
  /* TODO: we need to iterate the waves belonging to this queue and enqueue
     events for aborted requests.  i.e. single-step or stop requests that were
     submitted, but the queue was invalidated/destroyed before reporting the
     event, we still need to notify the application, so that it does not wait
     forever.  */

  process_t &process = m_queue.process ();

  /* FIXME: need to submit events for aborted requests.  */
  auto &&wave_range = process.range<wave_t> ();
  for (auto it = wave_range.begin (); it != wave_range.end ();)
    it = (it->queue ().id () == m_queue.id ()) ? process.destroy (it) : ++it;

  auto &&dispatch_range = process.range<dispatch_t> ();
  for (auto it = dispatch_range.begin (); it != dispatch_range.end ();)
    it = (it->queue ().id () == m_queue.id ()) ? process.destroy (it) : ++it;
}

amd_dbgapi_status_t
queue_t::aql_queue_impl_t::update_waves ()
{
  const architecture_t &architecture = m_queue.architecture ();
  process_t &process = m_queue.process ();
  const epoch_t wave_mark = m_queue.m_next_wave_mark++;
  amd_dbgapi_status_t status;

  /* Read the queue's write_dispatch_id and read_dispatch_id.  */

  uint64_t write_dispatch_id;
  status = process.read_global_memory (
      m_queue.m_os_queue_info.write_pointer_address, &write_dispatch_id,
      sizeof (write_dispatch_id));
  if (status != AMD_DBGAPI_STATUS_SUCCESS)
    return status;

  uint64_t read_dispatch_id;
  status = process.read_global_memory (
      m_queue.m_os_queue_info.read_pointer_address, &read_dispatch_id,
      sizeof (read_dispatch_id));
  if (status != AMD_DBGAPI_STATUS_SUCCESS)
    return status;

  /* Retrieve the used control stack size and used wave area from the
     context save area header.  */

  struct context_save_area_header_s header;

  status = process.read_global_memory (
      m_queue.m_os_queue_info.ctx_save_restore_address, &header,
      sizeof (header));
  if (status != AMD_DBGAPI_STATUS_SUCCESS)
    return status;

  constexpr size_t max_ctrl_stack_size
      = 32 /* max_waves_per_cu */ * 2 /* registers */ * sizeof (uint32_t)
        + 2 /* PM4 packets */ * sizeof (uint32_t)
        + sizeof (context_save_area_header_s);

  /* Make sure the top of the control stack does not overwrite the displaced
     stepping buffer or parked wave buffer.  */
  if (m_queue.m_os_queue_info.ctx_save_restore_address
          + header.ctrl_stack_offset + header.ctrl_stack_size
          - max_ctrl_stack_size
      < m_context_save_start_address)
    error ("not enough free space in the control stack");

  /* Make sure the bottom of the ctrl stack == the start of the
     wave save area.  */
  dbgapi_assert ((header.ctrl_stack_offset + header.ctrl_stack_size)
                 == (header.wave_state_offset - header.wave_state_size));

  auto ctrl_stack = std::make_unique<uint32_t[]> (header.ctrl_stack_size
                                                  / sizeof (uint32_t));

  /* Read the entire ctrl stack from the inferior.  */
  status = process.read_global_memory (
      m_queue.m_os_queue_info.ctx_save_restore_address
          + header.ctrl_stack_offset,
      &ctrl_stack[0], header.ctrl_stack_size);
  if (status != AMD_DBGAPI_STATUS_SUCCESS)
    return status;

  wave_t *group_leader = nullptr;

  auto callback = [&] (std::unique_ptr<architecture_t::cwsr_descriptor_t>
                           descriptor) {
    amd_dbgapi_wave_id_t wave_id;
    wave_t::visibility_t visibility{ wave_t::visibility_t::VISIBLE };

    if (process.is_flag_set (process_t::flag_t::ASSIGN_NEW_IDS_TO_ALL_WAVES))
      {
        /* We will never have hidden waves when assigning new ids. All waves
           seen in the control stack get a new wave_t instance with a new wave
           id.  */
        wave_id = wave_t::undefined;
      }
    else
      {
        /* The wave id is preserved in ttmp registers.  */
        const amd_dbgapi_global_address_t wave_id_address
            = architecture
                  .register_address (*descriptor, amdgpu_regnum_t::WAVE_ID)
                  .value ();

        status = process.read_global_memory (wave_id_address, &wave_id,
                                             sizeof (wave_id));
        if (status != AMD_DBGAPI_STATUS_SUCCESS)
          error ("read_global_memory failed at %lx", wave_id_address);

        /* If this is a new wave, check its visibility.  Waves halted at launch
           should remain hidden until the wave creation mode is changed to
           NORMAL.  A wave is halted at launch if it is halted without having
           entered the trap handler.
         */
        if (wave_id == wave_t::undefined)
          {
            uint32_t status_reg;
            const amd_dbgapi_global_address_t status_reg_address
                = architecture
                      .register_address (*descriptor, amdgpu_regnum_t::STATUS)
                      .value ();

            status = process.read_global_memory (
                status_reg_address, &status_reg, sizeof (status_reg));
            if (status != AMD_DBGAPI_STATUS_SUCCESS)
              error ("read_global_memory failed at %lx", status_reg_address);

            const bool halted = !!(status_reg & SQ_WAVE_STATUS_HALT_MASK);

            const amd_dbgapi_global_address_t ttmp11_address
                = architecture
                      .register_address (*descriptor, amdgpu_regnum_t::TTMP11)
                      .value ();

            uint32_t ttmp11;
            status = process.read_global_memory (ttmp11_address, &ttmp11,
                                                 sizeof (ttmp11));
            if (status != AMD_DBGAPI_STATUS_SUCCESS)
              error ("read_global_memory failed at %lx", ttmp11_address);

            /* trap_handler_events is true if the trap handler was entered
               because of a trap instruction or an exception.  */
            const bool trap_handler_events
                = !!(ttmp11 & TTMP11_TRAP_HANDLER_EVENTS_MASK);

            /* Waves halted at launch do not have trap handler events).  */
            if (halted && !trap_handler_events)
              visibility = wave_t::visibility_t::HIDDEN_HALTED_AT_LAUNCH;
          }
      }

    wave_t *wave = nullptr;

    if (wave_id != wave_t::undefined)
      {
        /* The wave already exists, so we should find it and update its context
           save area address.  */
        wave = process.find (wave_id);
        if (!wave)
          warning ("%s not found in the process map",
                   to_string (wave_id).c_str ());
      }

    if (!wave)
      {
        const amd_dbgapi_global_address_t dispatch_ptr_address
            = architecture
                  .register_address (*descriptor,
                                     amdgpu_regnum_t::DISPATCH_PTR)
                  .value ();

        amd_dbgapi_global_address_t dispatch_ptr;
        status = process.read_global_memory (
            dispatch_ptr_address, &dispatch_ptr, sizeof (dispatch_ptr));
        if (status != AMD_DBGAPI_STATUS_SUCCESS)
          error ("read_global_memory failed at %lx", dispatch_ptr_address);

        if (!dispatch_ptr)
          /* TODO: See comment above for corrupted wavefronts. This could be
             attached to a CORRUPT_DISPATCH instance.  */
          error ("invalid null dispatch_ptr for %s",
                 to_string (wave_id).c_str ());

        /* SPI only sends us the lower 40 bits of the dispatch_ptr, so we need
           to reconstitute it using the ring_base_address for the missing upper
           8 bits.  */

        constexpr uint64_t spi_mask = utils::bit_mask (0, 39);
        if (dispatch_ptr)
          dispatch_ptr
              = (dispatch_ptr & spi_mask)
                | (m_queue.m_os_queue_info.ring_base_address & ~spi_mask);

        if ((dispatch_ptr % AQL_PACKET_SIZE) != 0)
          /* TODO: See comment above for corrupted wavefronts. This could be
             attached to a CORRUPT_DISPATCH instance.  */
          error ("dispatch_ptr is not aligned on the packet size");

        /* Calculate the monotonic dispatch id for this packet.  It is between
           read_dispatch_id and write_dispatch_id.  */

        amd_dbgapi_os_queue_packet_id_t os_queue_packet_id
            = (dispatch_ptr - m_queue.m_os_queue_info.ring_base_address)
              / AQL_PACKET_SIZE;

        /* Check that 0 <= os_queue_packet_id < queue_size.  */
        if (os_queue_packet_id
            >= m_queue.m_os_queue_info.ring_size / AQL_PACKET_SIZE)
          /* TODO: See comment above for corrupted wavefronts. This could be
             attached to a CORRUPT_DISPATCH instance.  */
          error ("invalid os_queue_packet_id (%#lx)", os_queue_packet_id);

        /* ring_size must be a power of 2.  */
        if (!utils::is_power_of_two (m_queue.m_os_queue_info.ring_size))
          error ("ring_size is not a power of 2");

        /* Need to mask by the number of packets in the ring (which is a power
           of 2 so -1 makes the correct mask).  */
        const uint64_t id_mask
            = m_queue.m_os_queue_info.ring_size / AQL_PACKET_SIZE - 1;

        os_queue_packet_id
            |= os_queue_packet_id >= (read_dispatch_id & id_mask)
                   ? (read_dispatch_id & ~id_mask)
                   : (write_dispatch_id & ~id_mask);

        /* Check that read_dispatch_id <= dispatch_id < write_dispatch_id */
        if (read_dispatch_id > os_queue_packet_id
            || os_queue_packet_id >= write_dispatch_id)
          /* TODO: See comment above for corrupted wavefronts. This could be
             attached to a CORRUPT_DISPATCH instance.  */
          error ("invalid dispatch id (%#lx), with read_dispatch_id=%#lx, "
                 "and write_dispatch_id=%#lx",
                 os_queue_packet_id, read_dispatch_id, write_dispatch_id);

        /* Check if the dispatch already exists.  */
        dispatch_t *dispatch = process.find_if ([&] (const dispatch_t &x) {
          return x.queue ().id () == m_queue.id ()
                 && x.os_queue_packet_id () == os_queue_packet_id;
        });

        /* If we did not find the current dispatch, create a new one.  */
        if (!dispatch)
          dispatch = &process.create<dispatch_t> (
              m_queue,            /* queue  */
              os_queue_packet_id, /* os_queue_packet_id  */
              dispatch_ptr);      /* packet_address  */

        wave = &process.create<wave_t> (*dispatch);

        wave->set_visibility (visibility);
      }

    bool first_wave = architecture.wave_get_info (
        *descriptor, architecture_t::wave_info_t::first_wave);
    bool last_wave = architecture.wave_get_info (
        *descriptor, architecture_t::wave_info_t::last_wave);

    /* The first wave in the group is the group leader.  The group leader owns
       the backing store for the group memory (LDS).  */
    if (first_wave)
      group_leader = wave;

    if (!group_leader)
      error ("No group_leader, the control stack may be corrupted");

    status = wave->update (*group_leader, std::move (descriptor));
    if (status != AMD_DBGAPI_STATUS_SUCCESS)
      error ("wave_t::update failed");

    /* This was the last wave in the group. Make sure we have a new group
       leader for the remaining waves.  */
    if (last_wave)
      group_leader = nullptr;

    /* Check that the wave is in the same group as its group leader.  */
    if (wave->group_ids () != wave->group_leader ().group_ids ())
      error ("wave is not in the same group as group_leader");

    wave->set_mark (wave_mark);
  };

  /* Decode the control stack.  For each wave entry in the control stack, the
     provided function is called with a wave descriptor and a pointer to its
     context save area.  */

  architecture.control_stack_iterate (
      &ctrl_stack[0], header.ctrl_stack_size / sizeof (uint32_t),
      m_queue.m_os_queue_info.ctx_save_restore_address
          + header.wave_state_offset,
      callback);

  /* Iterate all waves belonging to this queue, and prune those with a mark
     older than the current mark.  Note that the waves must be pruned before
     the dispatches to ensure there are no dangling pointers from waves to
     pruned dispatches.  */

  auto &&wave_range = process.range<wave_t> ();
  for (auto it = wave_range.begin (); it != wave_range.end ();)
    it = (it->queue ().id () == m_queue.id () && it->mark () < wave_mark)
             ? process.destroy (it)
             : ++it;

  /* Prune old dispatches. Dispatches with ids older (smaller) than the queue
     current read dispatch id are now retired, so remove them from the process.
   */

  auto &&dispatch_range = process.range<dispatch_t> ();
  for (auto it = dispatch_range.begin (); it != dispatch_range.end ();)
    it = (it->queue ().id () == m_queue.id ()
          && it->os_queue_packet_id () < read_dispatch_id)
             ? process.destroy (it)
             : ++it;

  return AMD_DBGAPI_STATUS_SUCCESS;
}

std::pair<amd_dbgapi_os_queue_packet_id_t, size_t>
queue_t::aql_queue_impl_t::packets (void **packets_bytes) const
{
  dbgapi_assert (m_queue.is_suspended ());
  process_t &process = m_queue.process ();
  amd_dbgapi_status_t status;

  uint64_t read_dispatch_id;
  status = process.read_global_memory (
      m_queue.m_os_queue_info.read_pointer_address, &read_dispatch_id,
      sizeof (read_dispatch_id));
  if (status != AMD_DBGAPI_STATUS_SUCCESS)
    error ("Could not read the queue's read_dispatch_id (rc=%d)", status);

  uint64_t write_dispatch_id;
  status = process.read_global_memory (
      m_queue.m_os_queue_info.write_pointer_address, &write_dispatch_id,
      sizeof (write_dispatch_id));
  if (status != AMD_DBGAPI_STATUS_SUCCESS)
    error ("Could not read the queue's write_dispatch_id (rc=%d)", status);

  amd_dbgapi_size_t packets_byte_size
      = (write_dispatch_id - read_dispatch_id) * AQL_PACKET_SIZE;

  if (!packets_bytes)
    return { read_dispatch_id, packets_byte_size };

  /* ring_size must be a power of 2.  */
  if (!utils::is_power_of_two (m_queue.m_os_queue_info.ring_size))
    error ("ring_size is not a power of 2");

  const uint64_t id_mask
      = m_queue.m_os_queue_info.ring_size / AQL_PACKET_SIZE - 1;

  amd_dbgapi_global_address_t read_dispatch_ptr
      = m_queue.m_os_queue_info.ring_base_address
        + (read_dispatch_id & id_mask) * AQL_PACKET_SIZE;
  amd_dbgapi_global_address_t write_dispatch_ptr
      = m_queue.m_os_queue_info.ring_base_address
        + (write_dispatch_id & id_mask) * AQL_PACKET_SIZE;

  dbgapi_assert (write_dispatch_id >= read_dispatch_id);

  *packets_bytes = amd::dbgapi::allocate_memory (packets_byte_size);
  if (packets_byte_size && !*packets_bytes)
    {
      /* The memory allocation failure will be detected by the caller.  */
    }
  else if (read_dispatch_ptr < write_dispatch_ptr)
    {
      status = process.read_global_memory (read_dispatch_ptr, *packets_bytes,
                                           packets_byte_size);
      /* FIXME: convert this to AMD_DBGAPI_STATUS_ERROR.  */
      if (status != AMD_DBGAPI_STATUS_SUCCESS)
        error ("Could not read the queue's packets (rc=%d)", status);
    }
  else if (read_dispatch_ptr > write_dispatch_ptr)
    {
      size_t size = m_queue.m_os_queue_info.ring_base_address
                    + m_queue.m_os_queue_info.ring_size - read_dispatch_ptr;

      status = process.read_global_memory (read_dispatch_ptr, *packets_bytes,
                                           size);
      /* FIXME: convert this to AMD_DBGAPI_STATUS_ERROR.  */
      if (status != AMD_DBGAPI_STATUS_SUCCESS)
        error ("Could not read the queue's packets (rc=%d)", status);

      size_t offset = size;
      size = write_dispatch_ptr - m_queue.m_os_queue_info.ring_base_address;

      status = process.read_global_memory (
          m_queue.m_os_queue_info.ring_base_address,
          static_cast<char *> (*packets_bytes) + offset, size);
      /* FIXME: convert this to AMD_DBGAPI_STATUS_ERROR.  */
      if (status != AMD_DBGAPI_STATUS_SUCCESS)
        error ("Could not read the queue's packets (rc=%d)", status);
    }

  return { read_dispatch_id, packets_byte_size };
}

amd_dbgapi_os_queue_type_t
queue_t::aql_queue_impl_t::type () const
{
  switch (m_hsa_queue.type)
    {
    case HSA_QUEUE_TYPE_SINGLE:
      return AMD_DBGAPI_OS_QUEUE_TYPE_HSA_KERNEL_DISPATCH_SINGLE_PRODUCER;
    case HSA_QUEUE_TYPE_MULTI:
      return AMD_DBGAPI_OS_QUEUE_TYPE_HSA_KERNEL_DISPATCH_MULTIPLE_PRODUCER;
    case HSA_QUEUE_TYPE_COOPERATIVE:
      return AMD_DBGAPI_OS_QUEUE_TYPE_HSA_KERNEL_DISPATCH_COOPERATIVE;
    }
  return AMD_DBGAPI_OS_QUEUE_TYPE_UNKNOWN;
}

void
queue_t::aql_queue_impl_t::state_changed (state_t state)
{
  if (state == state_t::SUSPENDED)
    {
      process_t &process = m_queue.process ();
      amd_dbgapi_status_t status;

      /* Refresh the scratch_backing_memory_location and
       scratch_backing_memory_size everytime we suspend the queue.  */

      /* The scratch backing memory address is stored in the ABI-stable part
         of the amd_queue_t. Since we know the address of the read_dispatch_id
         (obtained from the KFD through the queue snapshot info), which is also
         stored in the ABI-stable part of the amd_queue_t, we can calculate the
         address of the pointer to the scratch_backing_memory_location and read
         it.  We cannot cache this value as the runtime may change the
         allocation dynamically.  */

      status = process.read_global_memory (
          m_queue.m_os_queue_info.read_pointer_address
              + offsetof (amd_queue_t, scratch_backing_memory_location)
              - offsetof (amd_queue_t, read_dispatch_id),
          &m_queue.m_scratch_backing_memory_address,
          sizeof (m_scratch_backing_memory_address));
      if (status != AMD_DBGAPI_STATUS_SUCCESS)
        error ("Could not read the queue's scratch_backing_memory_location "
               "(rc=%d)",
               status);

      status = process.read_global_memory (
          m_queue.m_os_queue_info.read_pointer_address
              + offsetof (amd_queue_t, scratch_backing_memory_byte_size)
              - offsetof (amd_queue_t, read_dispatch_id),
          &m_queue.m_scratch_backing_memory_size,
          sizeof (m_scratch_backing_memory_size));
      if (status != AMD_DBGAPI_STATUS_SUCCESS)
        error (
            "Could not read the queue's scratch_backing_memory_size (rc=%d)",
            status);

      /* Update the waves from the content of the queue's context save area. */
      status = update_waves ();
      if (status != AMD_DBGAPI_STATUS_SUCCESS)
        warning ("%s update_waves failed (rc=%d)",
                 to_string (m_queue.id ()).c_str (), status);
    }
}

/* PM4 Queue implementation.  */

class queue_t::pm4_queue_impl_t : public queue_impl_t
{
public:
  pm4_queue_impl_t (queue_t &queue) : queue_impl_t (queue) {}

  std::pair<amd_dbgapi_os_queue_packet_id_t, size_t>
  packets (void **packets_bytes) const override
  {
    /* FIXME: Not yet implemented.  */
    return {};
  }

  virtual amd_dbgapi_os_queue_type_t type () const override
  {
    /* FIXME: Not yet implemented.  */
    return AMD_DBGAPI_OS_QUEUE_TYPE_AMD_PM4;
  }
};

queue_t::queue_t (amd_dbgapi_queue_id_t queue_id, agent_t &agent,
                  const os_queue_snapshot_entry_t &os_queue_info)
    : handle_object (queue_id), m_os_queue_info (os_queue_info),
      m_agent (agent)
{
  if (os_queue_type () == os_queue_type_t::COMPUTE)
    {
      m_impl.reset (new pm4_queue_impl_t (*this));
    }
  else if (os_queue_type () == os_queue_type_t::COMPUTE_AQL)
    {
      m_impl.reset (new aql_queue_impl_t (*this));
    }
}

queue_t::queue_t (amd_dbgapi_queue_id_t queue_id, agent_t &agent,
                  os_queue_id_t os_queue_id)
    : handle_object (queue_id), m_os_queue_info{ [os_queue_id] () {
        os_queue_snapshot_entry_t os_queue_info{};
        os_queue_info.queue_id = os_queue_id;
        return os_queue_info;
      }() },
      m_agent (agent)
{
}

queue_t::~queue_t () {}

std::pair<amd_dbgapi_os_queue_packet_id_t, size_t>
queue_t::packets (void **packets_bytes) const
{
  if (m_impl)
    return m_impl->packets (packets_bytes);

  return {};
}

amd_dbgapi_os_queue_type_t
queue_t::type () const
{
  if (m_impl)
    return m_impl->type ();

  return AMD_DBGAPI_OS_QUEUE_TYPE_UNKNOWN;
}

void
queue_t::set_state (state_t state)
{
  if (m_state == state)
    return; /* State is unchanged.  */

  dbgapi_assert (m_state != state_t::INVALID
                 && "an invalid queue cannot change state");

  switch (state)
    {
    case state_t::INVALID:
      /* Destructing the queue impl for compute queues also destructs all
         dispatches and waves associated with it, and enqueues events for
         aborted requests.  */
      m_impl.reset (nullptr);

      dbgapi_log (AMD_DBGAPI_LOG_LEVEL_INFO, "invalidated %s (os_queue_id=%d)",
                  to_string (id ()).c_str (), os_queue_id ());
      break;

    case state_t::SUSPENDED:
      dbgapi_log (AMD_DBGAPI_LOG_LEVEL_INFO, "suspended %s (os_queue_id=%d)",
                  to_string (id ()).c_str (), os_queue_id ());
      break;

    case state_t::RUNNING:
      dbgapi_log (AMD_DBGAPI_LOG_LEVEL_INFO, "resumed %s (os_queue_id=%d)",
                  to_string (id ()).c_str (), os_queue_id ());
      break;
    }

  m_state = state;

  /* Notify the queue_impl of the change of state. Some implementation may act
     on some state transitions, for example a compute queue may update its
     dispatches and waves.  */
  if (m_impl)
    m_impl->state_changed (state);
}

amd_dbgapi_status_t
queue_t::get_info (amd_dbgapi_queue_info_t query, size_t value_size,
                   void *value) const
{
  switch (query)
    {
    case AMD_DBGAPI_QUEUE_INFO_AGENT:
      return utils::get_info (value_size, value, agent ().id ());

    case AMD_DBGAPI_QUEUE_INFO_ARCHITECTURE:
      return utils::get_info (value_size, value, architecture ().id ());

    case AMD_DBGAPI_QUEUE_TYPE:
      return utils::get_info (value_size, value, type ());

    case AMD_DBGAPI_QUEUE_INFO_OS_ID:
      return utils::get_info (
          value_size, value,
          static_cast<amd_dbgapi_os_queue_id_t> (m_os_queue_info.queue_id));

    case AMD_DBGAPI_QUEUE_INFO_ADDRESS:
      return utils::get_info (value_size, value,
                              m_os_queue_info.ring_base_address);

    case AMD_DBGAPI_QUEUE_INFO_SIZE:
      return utils::get_info (value_size, value, m_os_queue_info.ring_size);

    case AMD_DBGAPI_QUEUE_INFO_STATE:
    case AMD_DBGAPI_QUEUE_INFO_ERROR_REASON:
      return AMD_DBGAPI_STATUS_ERROR_UNIMPLEMENTED;
    }
  return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;
}

scoped_queue_suspend_t::scoped_queue_suspend_t (queue_t &queue)
    : m_queue (!queue.is_suspended () ? &queue : nullptr)
{
  if (!m_queue)
    return;

  if (m_queue->process ().suspend_queues ({ m_queue }) != 1)
    {
      if (m_queue->is_valid ())
        error ("process::suspend_queues failed");

      /* The queue became invalid, we should not try to resume it.  */
      m_queue = nullptr;
    }
}

scoped_queue_suspend_t::~scoped_queue_suspend_t ()
{
  if (!m_queue /* scoped_queue_suspend instance did not suspend the queue. */
      || !m_queue->process ().forward_progress_needed ())
    return;

  if (m_queue->process ().resume_queues ({ m_queue }) != 1
      && m_queue->is_valid ())
    error ("process::resume_queues failed");
}

} /* namespace amd::dbgapi */

using namespace amd::dbgapi;

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_queue_get_info (amd_dbgapi_process_id_t process_id,
                           amd_dbgapi_queue_id_t queue_id,
                           amd_dbgapi_queue_info_t query, size_t value_size,
                           void *value)
{
  TRY;
  TRACE (process_id, queue_id, query, value_size, value);

  if (!amd::dbgapi::is_initialized)
    return AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED;

  process_t *process = process_t::find (process_id);

  if (!process)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_PROCESS_ID;

  queue_t *queue = process->find (queue_id);

  if (!queue)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_QUEUE_ID;

  return queue->get_info (query, value_size, value);
  CATCH;
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_queue_list (amd_dbgapi_process_id_t process_id, size_t *queue_count,
                       amd_dbgapi_queue_id_t **queues,
                       amd_dbgapi_changed_t *changed)
{
  TRY;
  TRACE (process_id);

  if (!amd::dbgapi::is_initialized)
    return AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED;

  return utils::get_handle_list<queue_t> (process_id, queue_count, queues,
                                          changed);
  CATCH;
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_queue_packet_list (amd_dbgapi_process_id_t process_id,
                              amd_dbgapi_queue_id_t queue_id,
                              amd_dbgapi_os_queue_packet_id_t *first_packet_id,
                              amd_dbgapi_size_t *packets_byte_size,
                              void **packets_bytes)
{
  TRY;
  TRACE (process_id, queue_id);

  if (!amd::dbgapi::is_initialized)
    return AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED;

  if (!first_packet_id || !packets_byte_size)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;

  process_t *process = process_t::find (process_id);

  if (!process)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_PROCESS_ID;

  queue_t *queue = process->find (queue_id);

  if (!queue)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_QUEUE_ID;

  scoped_queue_suspend_t suspend (*queue);

  std::tie (*first_packet_id, *packets_byte_size)
      = queue->packets (packets_bytes);

  if (packets_bytes && *packets_byte_size && !*packets_bytes)
    return AMD_DBGAPI_STATUS_ERROR_CLIENT_CALLBACK;

  return AMD_DBGAPI_STATUS_SUCCESS;
  CATCH;
}
