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

#include "queue.h"
#include "architecture.h"
#include "debug.h"
#include "dispatch.h"
#include "exception.h"
#include "handle_object.h"
#include "initialization.h"
#include "logging.h"
#include "memory.h"
#include "process.h"
#include "register.h"
#include "utils.h"
#include "wave.h"

#include <hsa/amd_hsa_queue.h>
#include <hsa/hsa.h>

#include <array>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace amd::dbgapi
{

process_t &
queue_t::process () const
{
  return agent ().process ();
}

const architecture_t &
queue_t::architecture () const
{
  dbgapi_assert (agent ().architecture ());
  return *agent ().architecture ();
}

namespace detail
{

class aql_queue_t : public compute_queue_t
{
private:
  static constexpr uint64_t aql_packet_size = 64;
  static constexpr amd_dbgapi_size_t debugger_memory_chunk_size = 32;

  struct context_save_area_header_s
  {
    uint32_t ctrl_stack_offset;
    uint32_t ctrl_stack_size;
    uint32_t wave_state_offset;
    uint32_t wave_state_size;
    uint32_t debugger_memory_offset;
    uint32_t debugger_memory_size;
  };

  class aql_dispatch_t : public dispatch_t
  {
  private:
    hsa_kernel_dispatch_packet_t m_packet{};
    std::unique_ptr<const architecture_t::kernel_descriptor_t>
      m_kernel_descriptor{};

  public:
    aql_dispatch_t (amd_dbgapi_dispatch_id_t dispatch_id,
                    compute_queue_t &queue,
                    amd_dbgapi_os_queue_packet_id_t os_queue_packet_id);

    const architecture_t::kernel_descriptor_t &
    kernel_descriptor () const override
    {
      dbgapi_assert (m_kernel_descriptor);
      return *m_kernel_descriptor;
    }
    void get_info (amd_dbgapi_dispatch_info_t query, size_t value_size,
                   void *value) const override;
  };

  amd_dbgapi_global_address_t m_scratch_backing_memory_address{ 0 };
  uint32_t m_compute_tmpring_size{ 0 };

  /* The content of the context save area header the last time the queue was
     suspended.  */
  context_save_area_header_s m_last_context_save_header{};

  /* The memory reserved by the thunk library for the debugger is used to store
     instruction buffers.  Instruction buffers are lazily allocated from the
     reserved memory, and when freed, their index is returned to a free list.
     Each wave is guaranteed its own unique instruction buffer.  */
  amd_dbgapi_global_address_t m_debugger_memory_base{};

  uint16_t m_debugger_memory_chunk_count{ 0 };
  uint16_t m_debugger_memory_next_chunk{ 0 };
  std::vector<uint16_t> m_debugger_memory_free_chunks{};

  displaced_instruction_ptr_t m_park_instruction_ptr{};
  displaced_instruction_ptr_t m_terminating_instruction_ptr{};

  /* Value used to mark waves that are found in the context save area. When
     sweeping, any wave found with a mark less than the current mark will be
     deleted, as these waves are no longer active.  */
  monotonic_counter_t<epoch_t, 1> m_next_wave_mark{};

  hsa_queue_t m_hsa_queue{};

  displaced_instruction_ptr_t
  allocate_displaced_instruction (const instruction_t &instruction) override;

  void queue_state_changed () override;

  void update_waves ();

public:
  aql_queue_t (amd_dbgapi_queue_id_t queue_id, const agent_t &agent,
               const os_queue_snapshot_entry_t &os_queue_info);
  ~aql_queue_t () override;

  /* Return the queue type.  */
  amd_dbgapi_os_queue_type_t type () const override;

  /* Return the address of a park instruction.  */
  amd_dbgapi_global_address_t park_instruction_address () override
  {
    return m_park_instruction_ptr.get ();
  }

  /* Return the address of a terminating instruction.  */
  amd_dbgapi_global_address_t terminating_instruction_address () override
  {
    return m_terminating_instruction_ptr.get ();
  }

  std::pair<amd_dbgapi_global_address_t /* address */,
            amd_dbgapi_size_t /* size */>
  scratch_memory_region (uint32_t engine_id, uint32_t slot_id) const override;

  void active_packets_info (amd_dbgapi_os_queue_packet_id_t *read_packet_id_p,
                            amd_dbgapi_os_queue_packet_id_t *write_packet_id_p,
                            size_t *packets_byte_size_p) const override;

  void active_packets_bytes (amd_dbgapi_os_queue_packet_id_t read_packet_id,
                             amd_dbgapi_os_queue_packet_id_t write_packet_id,
                             void *memory, size_t memory_size) const override;
};

aql_queue_t::aql_dispatch_t::aql_dispatch_t (
  amd_dbgapi_dispatch_id_t dispatch_id, compute_queue_t &queue,
  amd_dbgapi_os_queue_packet_id_t os_queue_packet_id)
  : dispatch_t (dispatch_id, queue, os_queue_packet_id)
{
  amd_dbgapi_global_address_t packet_address
    = queue.address ()
      + (os_queue_packet_id * aql_packet_size) % queue.size ();

  /* Read the dispatch packet and kernel descriptor.  */
  process ().read_global_memory (packet_address, &m_packet);

  m_kernel_descriptor = architecture ().make_kernel_descriptor (
    process (), m_packet.kernel_object);
}

void
aql_queue_t::aql_dispatch_t::get_info (amd_dbgapi_dispatch_info_t query,
                                       size_t value_size, void *value) const
{
  switch (query)
    {
    case AMD_DBGAPI_DISPATCH_INFO_QUEUE:
      utils::get_info (value_size, value, queue ().id ());
      return;

    case AMD_DBGAPI_DISPATCH_INFO_AGENT:
      utils::get_info (value_size, value, agent ().id ());
      return;

    case AMD_DBGAPI_DISPATCH_INFO_PROCESS:
      utils::get_info (value_size, value, process ().id ());
      return;

    case AMD_DBGAPI_DISPATCH_INFO_ARCHITECTURE:
      utils::get_info (value_size, value, architecture ().id ());
      return;

    case AMD_DBGAPI_DISPATCH_INFO_OS_QUEUE_PACKET_ID:
      utils::get_info (value_size, value, os_queue_packet_id ());
      return;

    case AMD_DBGAPI_DISPATCH_INFO_BARRIER:
      utils::get_info (value_size, value,
                       utils::bit_extract (m_packet.header,
                                           HSA_PACKET_HEADER_BARRIER,
                                           HSA_PACKET_HEADER_BARRIER)
                         ? AMD_DBGAPI_DISPATCH_BARRIER_PRESENT
                         : AMD_DBGAPI_DISPATCH_BARRIER_NONE);
      return;

    case AMD_DBGAPI_DISPATCH_INFO_ACQUIRE_FENCE:
      static_assert ((int)AMD_DBGAPI_DISPATCH_FENCE_SCOPE_NONE
                       == (int)HSA_FENCE_SCOPE_NONE,
                     "amd_dbgapi_dispatch_fence_scope_t != hsa_fence_scope_t");
      static_assert ((int)AMD_DBGAPI_DISPATCH_FENCE_SCOPE_AGENT
                       == (int)HSA_FENCE_SCOPE_AGENT,
                     "amd_dbgapi_dispatch_fence_scope_t != hsa_fence_scope_t");
      static_assert ((int)AMD_DBGAPI_DISPATCH_FENCE_SCOPE_SYSTEM
                       == (int)HSA_FENCE_SCOPE_SYSTEM,
                     "amd_dbgapi_dispatch_fence_scope_t != hsa_fence_scope_t");

      utils::get_info (
        value_size, value,
        static_cast<amd_dbgapi_dispatch_fence_scope_t> (utils::bit_extract (
          m_packet.header, HSA_PACKET_HEADER_SCACQUIRE_FENCE_SCOPE,
          HSA_PACKET_HEADER_SCACQUIRE_FENCE_SCOPE
            + HSA_PACKET_HEADER_WIDTH_SCACQUIRE_FENCE_SCOPE - 1)));
      return;

    case AMD_DBGAPI_DISPATCH_INFO_RELEASE_FENCE:
      utils::get_info (
        value_size, value,
        static_cast<amd_dbgapi_dispatch_fence_scope_t> (utils::bit_extract (
          m_packet.header, HSA_PACKET_HEADER_SCRELEASE_FENCE_SCOPE,
          HSA_PACKET_HEADER_SCRELEASE_FENCE_SCOPE
            + HSA_PACKET_HEADER_WIDTH_SCRELEASE_FENCE_SCOPE - 1)));
      return;

    case AMD_DBGAPI_DISPATCH_INFO_GRID_DIMENSIONS:
      utils::get_info (
        value_size, value,
        static_cast<uint32_t> (utils::bit_extract (
          m_packet.setup, HSA_KERNEL_DISPATCH_PACKET_SETUP_DIMENSIONS,
          HSA_KERNEL_DISPATCH_PACKET_SETUP_DIMENSIONS
            + HSA_KERNEL_DISPATCH_PACKET_SETUP_WIDTH_DIMENSIONS - 1)));
      return;

    case AMD_DBGAPI_DISPATCH_INFO_WORK_GROUP_SIZES:
      {
        uint16_t workgroup_sizes[3]
          = { m_packet.workgroup_size_x, m_packet.workgroup_size_y,
              m_packet.workgroup_size_z };
        utils::get_info (value_size, value, workgroup_sizes);
        return;
      }
    case AMD_DBGAPI_DISPATCH_INFO_GRID_SIZES:
      {
        uint32_t grid_sizes[3] = { m_packet.grid_size_x, m_packet.grid_size_y,
                                   m_packet.grid_size_z };
        utils::get_info (value_size, value, grid_sizes);
        return;
      }
    case AMD_DBGAPI_DISPATCH_INFO_PRIVATE_SEGMENT_SIZE:
      utils::get_info (
        value_size, value,
        static_cast<amd_dbgapi_size_t> (m_packet.private_segment_size));
      return;

    case AMD_DBGAPI_DISPATCH_INFO_GROUP_SEGMENT_SIZE:
      utils::get_info (
        value_size, value,
        static_cast<amd_dbgapi_size_t> (m_packet.group_segment_size));
      return;

    case AMD_DBGAPI_DISPATCH_INFO_KERNEL_ARGUMENT_SEGMENT_ADDRESS:
      utils::get_info (value_size, value, m_packet.kernarg_address);
      return;

    case AMD_DBGAPI_DISPATCH_INFO_KERNEL_DESCRIPTOR_ADDRESS:
      utils::get_info (value_size, value, kernel_descriptor ().address ());
      return;

    case AMD_DBGAPI_DISPATCH_INFO_KERNEL_CODE_ENTRY_ADDRESS:
      utils::get_info (value_size, value,
                       kernel_descriptor ().entry_address ());
      return;

    case AMD_DBGAPI_DISPATCH_INFO_KERNEL_COMPLETION_ADDRESS:
      utils::get_info (value_size, value, m_packet.completion_signal);
      return;
    }

  throw api_error_t (AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT);
}

aql_queue_t::aql_queue_t (amd_dbgapi_queue_id_t queue_id, const agent_t &agent,
                          const os_queue_snapshot_entry_t &os_queue_info)
  : compute_queue_t (queue_id, agent, os_queue_info)
{
  process_t &process = this->process ();

  amd_dbgapi_global_address_t ctx_save_base
    = m_os_queue_info.ctx_save_restore_address;

  struct context_save_area_header_s header;
  process.read_global_memory (ctx_save_base, &header);

  if (!header.debugger_memory_offset || !header.debugger_memory_size)
    fatal_error ("Per-queue memory reserved for the debugger is missing");

  /* Make sure the debugger memory is aligned so that it can be used to store
     displaced instructions, and that each chunk out of that memory register is
     also aligned.  */
  dbgapi_assert (
    utils::is_aligned (debugger_memory_chunk_size,
                       architecture ().minimum_instruction_alignment ()));

  m_debugger_memory_base = utils::align_up (
    ctx_save_base + header.debugger_memory_offset, debugger_memory_chunk_size);

  auto chunk_count = (ctx_save_base + header.debugger_memory_size
                      + header.debugger_memory_offset - m_debugger_memory_base)
                     / debugger_memory_chunk_size;

  /* Ensure that the number of chunks does not overflow the 16 bit index.  */
  if (chunk_count
      > std::numeric_limits<decltype (m_debugger_memory_chunk_count)>::max ())
    fatal_error ("Increase the width of m_debugger_memory_chunk_count");

  m_debugger_memory_chunk_count = chunk_count;

  m_debugger_memory_free_chunks.reserve (m_debugger_memory_chunk_count);

  /* Reserve 2 instruction buffers for parking, and terminating waves.  */
  m_park_instruction_ptr
    = allocate_displaced_instruction (architecture ().assert_instruction ());
  m_terminating_instruction_ptr = allocate_displaced_instruction (
    architecture ().terminating_instruction ());

  /* Read the hsa_queue_t at the top of the amd_queue_t. Since the amd_queue_t
    structure could change, it can only be accessed by calculating its address
    from the address of the read_dispatch_id by subtracting
    read_dispatch_id_field_base_byte_offset .  */

  uint32_t read_dispatch_id_field_base_byte_offset;
  process.read_global_memory (
    m_os_queue_info.read_pointer_address
      + offsetof (amd_queue_t, read_dispatch_id_field_base_byte_offset)
      - offsetof (amd_queue_t, read_dispatch_id),
    &read_dispatch_id_field_base_byte_offset);

  amd_dbgapi_global_address_t hsa_queue_address
    = m_os_queue_info.read_pointer_address
      - read_dispatch_id_field_base_byte_offset;
  process.read_global_memory (hsa_queue_address, &m_hsa_queue);

  if (reinterpret_cast<uintptr_t> (m_hsa_queue.base_address) != address ())
    fatal_error ("hsa_queue_t base address != kfd queue info base address");

  if ((m_hsa_queue.size * 64) != size ())
    fatal_error ("hsa_queue_t size != kfd queue info ring size");
}

aql_queue_t::~aql_queue_t ()
{
  process_t &process = this->process ();

  /* Destruct all waves and dispatches associated with this queue.  Waves that
     are executing a displaced stepped instruction will release the displaced
     stepping buffer when destructed, and raise a command terminated event if
     single-stepping (see wave_t::~wave_t).  */

  auto &&wave_range = process.range<wave_t> ();
  for (auto it = wave_range.begin (); it != wave_range.end ();)
    it = (it->queue () == *this) ? process.destroy (it) : ++it;

  auto &&dispatch_range = process.range<dispatch_t> ();
  for (auto it = dispatch_range.begin (); it != dispatch_range.end ();)
    it = (it->queue () == *this) ? process.destroy (it) : ++it;

  /* Discard any cached data associated with the queue since the queue may have
     been deleted.  In that case, the memory being used for the context save
     area could be re-allocated for other purposes.  If that happens, we do not
     want to see stale data, or for stale dirty data to corrupt the future use.
     If the process is being detached, then there's really no need to discard
     since the process will be destructed and the cache destructed.  */

  auto wave_save_address = m_os_queue_info.ctx_save_restore_address
                           + m_last_context_save_header.wave_state_offset
                           - m_last_context_save_header.wave_state_size;
  auto wave_save_size = m_last_context_save_header.wave_state_size;

  /* Need to write back only because discard requires no dirty data exists.  */
  process.memory_cache ().write_back (wave_save_address, wave_save_size);

  process.memory_cache ().discard (wave_save_address, wave_save_size);
}

compute_queue_t::displaced_instruction_ptr_t
aql_queue_t::allocate_displaced_instruction (const instruction_t &instruction)
{
  auto assert_instruction = architecture ().assert_instruction ();
  amd_dbgapi_global_address_t instruction_buffer_address;

  if (!m_debugger_memory_free_chunks.empty ())
    {
      auto index = m_debugger_memory_free_chunks.back ();
      m_debugger_memory_free_chunks.pop_back ();

      instruction_buffer_address
        = m_debugger_memory_base + index * debugger_memory_chunk_size;
    }
  else if (m_debugger_memory_next_chunk < m_debugger_memory_chunk_count)
    {
      instruction_buffer_address
        = m_debugger_memory_base
          + m_debugger_memory_next_chunk++ * debugger_memory_chunk_size;

      /* An instruction buffer is always terminated by a 'guard' instruction
         (assert_trap) so that the pc never points to an invalid instruction or
         unmapped memory if the instruction is single-stepped.  We use a trap
         instruction to prevent runaway waves from executing from unmapped
         memory.  */

      process ().write_global_memory (
        instruction_buffer_address + debugger_memory_chunk_size
          - assert_instruction.size (),
        assert_instruction.data (), assert_instruction.size ());
    }
  else
    fatal_error ("could not allocate debugger memory");

  auto deleter = [this] (amd_dbgapi_global_address_t ptr)
  {
    size_t index = (ptr - m_debugger_memory_base) / debugger_memory_chunk_size;

    dbgapi_assert (index < m_debugger_memory_chunk_count);
    m_debugger_memory_free_chunks.emplace_back (index);
  };

  dbgapi_assert (instruction.size () + assert_instruction.size ()
                 <= debugger_memory_chunk_size);

  amd_dbgapi_global_address_t displaced_instruction_address
    = instruction_buffer_address + debugger_memory_chunk_size
      - assert_instruction.size () - instruction.size ();

  /* Make sure the new displaced instruction is properly aligned for this
     architecture.  */
  dbgapi_assert (
    utils::is_aligned (displaced_instruction_address,
                       architecture ().minimum_instruction_alignment ()));

  process ().write_global_memory (displaced_instruction_address,
                                  instruction.data (), instruction.size ());

  return { displaced_instruction_address, deleter };
}

void
aql_queue_t::queue_state_changed ()
{
  switch (state ())
    {
    case state_t::running:
      m_waves_running.reset ();

      /* The queue just changed state and is about to be placed back onto the
         hardware.  Write back dirty cache lines in the wave saved state
         region, but leave the cache lines valid so that accessing stopped
         waves' cached registers does not require a queue suspend/resume.  The
         saved state cache lines will be discarded when this queue is next
         suspended again (see the 'case state_t::suspended:' below).  */
      process ().memory_cache ().write_back (
        m_os_queue_info.ctx_save_restore_address
          + m_last_context_save_header.wave_state_offset
          - m_last_context_save_header.wave_state_size,
        m_last_context_save_header.wave_state_size);
      break;

    case state_t::suspended:
      /* Discard the previously cached wave saved state lines.  The saved state
         areas may be mapped to a different address in this new context wave
         save.  */
      process ().memory_cache ().discard (
        m_os_queue_info.ctx_save_restore_address
          + m_last_context_save_header.wave_state_offset
          - m_last_context_save_header.wave_state_size,
        m_last_context_save_header.wave_state_size);

      /* Refresh the scratch_backing_memory_location and
         scratch_backing_memory_size everytime the queue is suspended.

         The scratch backing memory address is stored in the ABI-stable part of
         the amd_queue_t. Since we know the address of the read_dispatch_id
         (obtained from the KFD through the queue snapshot info), which is also
         stored in the ABI-stable part of the amd_queue_t, we can calculate the
         address of the pointer to the scratch_backing_memory_location and read
         it.  We cannot cache this value as the runtime may change the
         allocation dynamically.  */

      process ().read_global_memory (
        m_os_queue_info.read_pointer_address
          + offsetof (amd_queue_t, scratch_backing_memory_location)
          - offsetof (amd_queue_t, read_dispatch_id),
        &m_scratch_backing_memory_address);

      process ().read_global_memory (
        m_os_queue_info.read_pointer_address
          + offsetof (amd_queue_t, compute_tmpring_size)
          - offsetof (amd_queue_t, read_dispatch_id),
        &m_compute_tmpring_size);

      /* Iterate the control stack and update/create waves that were saved in
         the last context wave save.  Waves that are no longer present will be
         destroyed.  */
      update_waves ();
      break;

    case state_t::invalid:
      break;
    }
}

void
aql_queue_t::update_waves ()
{
  process_t &process = this->process ();

  /* Read the queue's write_dispatch_id and read_dispatch_id.  */

  uint64_t write_dispatch_id;
  process.read_global_memory (m_os_queue_info.write_pointer_address,
                              &write_dispatch_id);

  uint64_t read_dispatch_id;
  process.read_global_memory (m_os_queue_info.read_pointer_address,
                              &read_dispatch_id);

  /* Value used to mark waves that are found in the context save area. When
     sweeping, any wave found with a mark less than the current mark will be
     deleted, as these waves are no longer active.  */
  const epoch_t wave_mark = wave_t::next_mark ();
  wave_t *group_leader = nullptr;

  auto decode_one_wave = [=, &process, &group_leader] (auto cwsr_record)
  {
    dbgapi_assert (*this == cwsr_record->queue ());

    auto prefetch_begin
      = cwsr_record->register_address (amdgpu_regnum_t::first_hwreg).value ();
    auto prefetch_end
      = cwsr_record->register_address (amdgpu_regnum_t::last_ttmp).value ()
        + architecture ().register_size (amdgpu_regnum_t::last_ttmp);

    dbgapi_assert (prefetch_end > prefetch_begin);
    process.memory_cache ().prefetch (prefetch_begin,
                                      prefetch_end - prefetch_begin);

    std::optional<amd_dbgapi_wave_id_t> wave_id;
    if (process.is_flag_set (process_t::flag_t::runtime_enable_during_attach))
      {
        /* Assign new ids to all waves regardless of the content of their
           wave_id register.  This is needed during attach as waves created
           before the debugger attached to the process may have corrupted
           wave_ids.

           We will never have hidden waves when assigning new ids. All waves
           seen in the control stack get a new wave_t instance with a new wave
           id.
         */
      }
    else
      wave_id = cwsr_record->id ();

    wave_t *wave = nullptr;

    if (wave_id)
      {
        static constexpr bool including_invisible_waves = true;
        /* The wave already exists, so we should find it and update its context
           save area address.  Search all waves, visible and invisible.  */
        wave = process.find (*wave_id, including_invisible_waves);
        if (!wave)
          warning ("%s not found in the process map",
                   to_string (*wave_id).c_str ());
      }

    bool is_new_wave = !wave;
    if (is_new_wave)
      {
        const dispatch_t *dispatch = &m_dummy_dispatch;

        bool ttmps_initialized
          = process.is_flag_set (process_t::flag_t::ttmps_setup_enabled)
            || agent ().os_info ().ttmps_always_initialized;

        if (ttmps_initialized)
          {
            amd_dbgapi_global_address_t dispatch_ptr
              = architecture ().dispatch_packet_address (*cwsr_record);

            if ((dispatch_ptr % aql_packet_size) != 0)
              fatal_error ("dispatch_ptr is not aligned on the packet size");

            /* Calculate the monotonic dispatch id for this packet.  It is
               between read_dispatch_id and write_dispatch_id.  */

            amd_dbgapi_os_queue_packet_id_t os_queue_packet_id
              = (dispatch_ptr - address ()) / aql_packet_size;

            /* Check that 0 <= os_queue_packet_id < queue_size.  */
            if (os_queue_packet_id >= size () / aql_packet_size)
              /* TODO: See comment above for corrupted wavefronts. This could
                 be attached to a CORRUPT_DISPATCH instance.  */
              fatal_error ("invalid os_queue_packet_id (%#lx)",
                           os_queue_packet_id);

            /* size must be a power of 2.  */
            if (!utils::is_power_of_two (size ()))
              fatal_error ("size is not a power of 2");

            /* Need to mask by the number of packets in the ring (which is a
               power of 2 so -1 makes the correct mask).  */
            const uint64_t id_mask = size () / aql_packet_size - 1;

            os_queue_packet_id
              |= os_queue_packet_id >= (read_dispatch_id & id_mask)
                   ? (read_dispatch_id & ~id_mask)
                   : (write_dispatch_id & ~id_mask);

            /* Check that the dispatch_id is between the command processor's
               read_id and write_id.  */
            if (read_dispatch_id > os_queue_packet_id
                || os_queue_packet_id >= write_dispatch_id)
              /* TODO: See comment above for corrupted wavefronts. This could
                 be attached to a CORRUPT_DISPATCH instance.  */
              fatal_error (
                "invalid dispatch id (%#lx), with read_dispatch_id=%#lx, "
                "and write_dispatch_id=%#lx",
                os_queue_packet_id, read_dispatch_id, write_dispatch_id);

            /* Check if the dispatch already exists.  */
            dispatch = process.find_if (
              [&] (const dispatch_t &x)
              {
                return x.queue () == *this
                       && x.os_queue_packet_id () == os_queue_packet_id;
              });

            /* If we did not find the current dispatch, create a new one.  */
            if (!dispatch)
              dispatch = &process.create<aql_dispatch_t> (
                cwsr_record->queue (), /* queue  */
                os_queue_packet_id);   /* os_queue_packet_id  */
          }

        wave = &process.create<wave_t> (wave_id, *dispatch);
      }

    bool first_wave = cwsr_record->is_first_wave ();
    bool last_wave = cwsr_record->is_last_wave ();

    /* The first wave in the group is the group leader.  The group leader owns
       the backing store for the group memory (LDS).  */
    if (first_wave)
      group_leader = wave;

    if (!group_leader)
      fatal_error ("No group_leader, the control stack may be corrupted");

    wave->update (*group_leader, std::move (cwsr_record));

    if (wave->state () == AMD_DBGAPI_WAVE_STATE_RUN)
      ++*m_waves_running;

    /* Hide new waves halted at launch until the process' wave creation mode is
       changed to not halted.  A wave is halted at launch if it is halted
       without having entered the trap handler, and its pc points to the kernel
       entry point.  */
    if (is_new_wave && wave->state () == AMD_DBGAPI_WAVE_STATE_RUN
        && wave->pc ()
             == wave->dispatch ().kernel_descriptor ().entry_address ()
        && wave->is_halted ())
      {
        wave->set_visibility (wave_t::visibility_t::hidden_halted_at_launch);
        wave->set_halted (false);
        wave->set_state (AMD_DBGAPI_WAVE_STATE_STOP);
      }

    /* This was the last wave in the group. Make sure we have a new group
       leader for the remaining waves.  */
    if (last_wave)
      group_leader = nullptr;

    /* Check that the wave is in the same group as its group leader.  */
    if (wave->group_ids () != wave->group_leader ().group_ids ())
      fatal_error ("wave is not in the same group as group_leader");

    wave->set_mark (wave_mark);
  };

  amd_dbgapi_global_address_t ctx_save_base
    = m_os_queue_info.ctx_save_restore_address;

  /* Retrieve the used control stack size and used wave area from the context
     save area header.  */

  struct context_save_area_header_s header;
  process.read_global_memory (ctx_save_base, &header);

  /* Make sure the bottom of the ctrl stack == the start of the wave save
     area.  */
  if ((header.ctrl_stack_offset + header.ctrl_stack_size)
      != (header.wave_state_offset - header.wave_state_size))
    fatal_error ("Corrupted control stack or wave save area");

  /* Start with 0 running waves.  When iterating the control stack (below) each
     discovered wave in the running state will increment this count.  */
  m_waves_running.emplace (0);

  if (header.ctrl_stack_size)
    {
      dbgapi_log (
        AMD_DBGAPI_LOG_LEVEL_INFO,
        "decoding %s's context save area: "
        "ctrl_stk:[0x%lx..0x%lx[, wave_area:[0x%lx..0x%lx[",
        to_string (id ()).c_str (), ctx_save_base + header.ctrl_stack_offset,
        ctx_save_base + header.ctrl_stack_offset + header.ctrl_stack_size,
        ctx_save_base + header.wave_state_offset - header.wave_state_size,
        ctx_save_base + header.wave_state_offset);

      auto ctrl_stack = std::make_unique<uint32_t[]> (header.ctrl_stack_size
                                                      / sizeof (uint32_t));

      /* Read the entire ctrl stack from the inferior.  */
      process.read_global_memory (ctx_save_base + header.ctrl_stack_offset,
                                  &ctrl_stack[0], header.ctrl_stack_size);

      /* Decode the control stack.  For each wave entry in the control stack,
         the provided function is called with a wave descriptor and a pointer
         to its context save area.  */

      m_last_context_save_header = header;

      size_t wave_count = architecture ().control_stack_iterate (
        *this, &ctrl_stack[0], header.ctrl_stack_size / sizeof (uint32_t),
        ctx_save_base + header.wave_state_offset, header.wave_state_size,
        decode_one_wave);

      dbgapi_log (AMD_DBGAPI_LOG_LEVEL_INFO,
                  "%zu out of %zu wave%s running on %s", *m_waves_running,
                  wave_count, wave_count > 1 ? "s" : "",
                  to_string (id ()).c_str ());
    }

  /* Iterate all waves belonging to this queue, and prune those with a mark
     older than the current mark.  Note that the waves must be pruned before
     the dispatches to ensure there are no dangling pointers from waves to
     pruned dispatches.  */

  auto &&wave_range = process.range<wave_t> ();
  for (auto it = wave_range.begin (); it != wave_range.end ();)
    if (it->queue () == *this && it->mark () < wave_mark)
      {
        dbgapi_assert (it->state () != AMD_DBGAPI_WAVE_STATE_STOP
                       && "a stopped wave cannot terminate");
        it = process.destroy (it);
      }
    else
      ++it;

  /* Prune old dispatches. Dispatches with ids older (smaller) than the queue
     current read dispatch id are now retired, so remove them from the process.
   */

  auto &&dispatch_range = process.range<dispatch_t> ();
  for (auto it = dispatch_range.begin (); it != dispatch_range.end ();)
    if (it->queue () == *this && it->os_queue_packet_id () < read_dispatch_id)
      it = process.destroy (it);
    else
      ++it;
}

std::pair<amd_dbgapi_global_address_t /* address */,
          amd_dbgapi_size_t /* size */>
aql_queue_t::scratch_memory_region (uint32_t engine_id, uint32_t slot_id) const
{
  auto [offset, size] = architecture ().scratch_memory_region (
    m_compute_tmpring_size, agent ().os_info ().shader_engine_count, engine_id,
    slot_id);
  return { m_scratch_backing_memory_address + offset, size };
}

void
aql_queue_t::active_packets_info (
  amd_dbgapi_os_queue_packet_id_t *read_packet_id_p,
  amd_dbgapi_os_queue_packet_id_t *write_packet_id_p,
  size_t *packets_byte_size_p) const
{
  dbgapi_assert (is_suspended ());

  amd_dbgapi_os_queue_packet_id_t read_packet_id;
  process ().read_global_memory (m_os_queue_info.read_pointer_address,
                                 &read_packet_id);

  amd_dbgapi_os_queue_packet_id_t write_packet_id;
  process ().read_global_memory (m_os_queue_info.write_pointer_address,
                                 &write_packet_id);

  if (read_packet_id > write_packet_id)
    fatal_error ("corrupted read/write packet ids");

  *read_packet_id_p = read_packet_id;
  *write_packet_id_p = write_packet_id;
  *packets_byte_size_p = (write_packet_id - read_packet_id) * aql_packet_size;
}

void
aql_queue_t::active_packets_bytes (
  amd_dbgapi_os_queue_packet_id_t read_packet_id,
  amd_dbgapi_os_queue_packet_id_t write_packet_id, void *memory,
  size_t memory_size) const
{
  dbgapi_assert (is_suspended ());

  if (read_packet_id > write_packet_id)
    throw api_error_t (AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT);

  amd_dbgapi_size_t packets_byte_size
    = (write_packet_id - read_packet_id) * aql_packet_size;

  if (memory_size != packets_byte_size)
    throw api_error_t (AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT);

  /* size must be a power of 2.  */
  if (!utils::is_power_of_two (size ()))
    fatal_error ("size is not a power of 2");

  const uint64_t id_mask = size () / aql_packet_size - 1;

  amd_dbgapi_global_address_t read_packet_ptr
    = address () + (read_packet_id & id_mask) * aql_packet_size;
  amd_dbgapi_global_address_t write_packet_ptr
    = address () + (write_packet_id & id_mask) * aql_packet_size;

  if (read_packet_ptr < write_packet_ptr)
    process ().read_global_memory (read_packet_ptr, memory, packets_byte_size);

  else if (read_packet_ptr > write_packet_ptr)
    {
      size_t first_part_size = address () + size () - read_packet_ptr;

      process ().read_global_memory (read_packet_ptr, memory, first_part_size);

      size_t second_part_size = write_packet_ptr - address ();

      process ().read_global_memory (
        address (), static_cast<char *> (memory) + first_part_size,
        second_part_size);
    }
}

amd_dbgapi_os_queue_type_t
aql_queue_t::type () const
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

class unsupported_queue_t : public queue_t
{
public:
  unsupported_queue_t (amd_dbgapi_queue_id_t queue_id, const agent_t &agent,
                       const os_queue_snapshot_entry_t &os_queue_info)
    : queue_t (queue_id, agent, os_queue_info)
  {
  }

  amd_dbgapi_os_queue_type_t type () const override;

  void active_packets_info (amd_dbgapi_os_queue_packet_id_t *read_packet_id_p,
                            amd_dbgapi_os_queue_packet_id_t *write_packet_id_p,
                            size_t *packets_byte_size_p) const override;

  void active_packets_bytes (amd_dbgapi_os_queue_packet_id_t read_packet_id,
                             amd_dbgapi_os_queue_packet_id_t write_packet_id,
                             void *memory, size_t memory_size) const override;
};

amd_dbgapi_os_queue_type_t
unsupported_queue_t::type () const
{
  switch (os_queue_type (m_os_queue_info))
    {
    case os_queue_type_t::compute:
      return AMD_DBGAPI_OS_QUEUE_TYPE_AMD_PM4;

    case os_queue_type_t::sdma:
      return AMD_DBGAPI_OS_QUEUE_TYPE_AMD_SDMA;

    case os_queue_type_t::sdma_xgmi:
      return AMD_DBGAPI_OS_QUEUE_TYPE_AMD_SDMA_XGMI;

    case os_queue_type_t::compute_aql:
      fatal_error ("should not reach here");

    default:
      return AMD_DBGAPI_OS_QUEUE_TYPE_UNKNOWN;
    }
}

void
unsupported_queue_t::active_packets_info (
  amd_dbgapi_os_queue_packet_id_t * /* read_packet_id_p  */,
  amd_dbgapi_os_queue_packet_id_t * /* write_packet_id_p  */,
  size_t * /* packets_byte_size_p  */) const
{
  throw api_error_t (AMD_DBGAPI_STATUS_ERROR_NOT_SUPPORTED);
}

void
unsupported_queue_t::active_packets_bytes (
  amd_dbgapi_os_queue_packet_id_t /* read_packet_id  */,
  amd_dbgapi_os_queue_packet_id_t /* write_packet_id  */, void * /* memory  */,
  size_t /* memory_size  */) const
{
  throw api_error_t (AMD_DBGAPI_STATUS_ERROR_NOT_SUPPORTED);
}

} /* namespace detail */

bool
compute_queue_t::is_all_stopped () const
{
  return process ().wave_launch_mode () == os_wave_launch_mode_t::halt
         && m_waves_running && *m_waves_running == 0;
}

void
compute_queue_t::wave_state_changed (const wave_t &wave)
{
  dbgapi_assert (m_waves_running);

  if (wave.state () == AMD_DBGAPI_WAVE_STATE_STOP)
    {
      dbgapi_assert (*m_waves_running > 0);
      --*m_waves_running;
    }
  else
    ++*m_waves_running;
}

queue_t &
queue_t::create (std::optional<amd_dbgapi_queue_id_t> queue_id,
                 const agent_t &agent,
                 const os_queue_snapshot_entry_t &os_queue_info)
{
  switch (os_queue_type (os_queue_info))
    {
    case os_queue_type_t::compute_aql:
      return agent.process ().create<detail::aql_queue_t> (queue_id, agent,
                                                           os_queue_info);

    default:
      return agent.process ().create<detail::unsupported_queue_t> (
        queue_id, agent, os_queue_info);
    }
}

os_queue_id_t
queue_t::os_queue_id () const
{
  return is_valid () ? m_os_queue_info.queue_id : os_invalid_queueid;
}

void
queue_t::set_state (state_t state)
{
  if (m_state == state)
    return; /* State is unchanged.  */

  dbgapi_assert (m_state != state_t::invalid
                 && "an invalid queue cannot change state");

  std::swap (m_state, state);

  if (!is_all_stopped ())
    queue_state_changed ();

  if (m_state == state_t::invalid)
    dbgapi_log (AMD_DBGAPI_LOG_LEVEL_INFO, "invalidated %s",
                to_string (id ()).c_str ());
}

amd_dbgapi_global_address_t
queue_t::address () const
{
  return m_os_queue_info.ring_base_address;
}

amd_dbgapi_size_t
queue_t::size () const
{
  return m_os_queue_info.ring_size;
}

void
queue_t::get_info (amd_dbgapi_queue_info_t query, size_t value_size,
                   void *value) const
{
  switch (query)
    {
    case AMD_DBGAPI_QUEUE_INFO_AGENT:
      utils::get_info (value_size, value, agent ().id ());
      return;

    case AMD_DBGAPI_QUEUE_INFO_PROCESS:
      utils::get_info (value_size, value, process ().id ());
      return;

    case AMD_DBGAPI_QUEUE_INFO_ARCHITECTURE:
      utils::get_info (value_size, value, architecture ().id ());
      return;

    case AMD_DBGAPI_QUEUE_INFO_TYPE:
      utils::get_info (value_size, value, type ());
      return;

    case AMD_DBGAPI_QUEUE_INFO_OS_ID:
      utils::get_info (value_size, value,
                       static_cast<amd_dbgapi_os_queue_id_t> (os_queue_id ()));
      return;

    case AMD_DBGAPI_QUEUE_INFO_ADDRESS:
      utils::get_info (value_size, value, address ());
      return;

    case AMD_DBGAPI_QUEUE_INFO_SIZE:
      utils::get_info (value_size, value, size ());
      return;

    case AMD_DBGAPI_QUEUE_INFO_STATE:
    case AMD_DBGAPI_QUEUE_INFO_ERROR_REASON:
      throw api_error_t (AMD_DBGAPI_STATUS_ERROR_NOT_IMPLEMENTED);
    }

  throw api_error_t (AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT);
}

scoped_queue_suspend_t::scoped_queue_suspend_t (queue_t &queue,
                                                const char *reason)
  : m_reason (reason), m_queue (!queue.is_suspended () ? &queue : nullptr)
{
  if (!m_queue)
    return;

  if (m_queue->process ().suspend_queues ({ m_queue }, m_reason) != 1)
    {
      if (m_queue->is_valid ())
        fatal_error ("process::suspend_queues failed");

      /* The queue became invalid, we should not try to resume it.  */
      m_queue = nullptr;
    }
}

scoped_queue_suspend_t::~scoped_queue_suspend_t ()
{
  if (!m_queue /* scoped_queue_suspend instance did not suspend the queue. */
      || !m_queue->process ().forward_progress_needed ())
    return;

  if (m_queue->process ().resume_queues ({ m_queue }, m_reason) != 1
      && m_queue->is_valid ())
    fatal_error ("process::resume_queues failed");
}

} /* namespace amd::dbgapi */

using namespace amd::dbgapi;

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_queue_get_info (amd_dbgapi_queue_id_t queue_id,
                           amd_dbgapi_queue_info_t query, size_t value_size,
                           void *value)
{
  TRACE_BEGIN (param_in (queue_id), param_in (query), param_in (value_size),
               param_in (value));
  TRY
  {
    if (!detail::is_initialized)
      THROW (AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED);

    queue_t *queue = find (queue_id);

    if (!queue)
      THROW (AMD_DBGAPI_STATUS_ERROR_INVALID_QUEUE_ID);

    queue->get_info (query, value_size, value);

    return AMD_DBGAPI_STATUS_SUCCESS;
  }
  CATCH (AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED,
         AMD_DBGAPI_STATUS_ERROR_INVALID_QUEUE_ID,
         AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT,
         AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT_COMPATIBILITY,
         AMD_DBGAPI_STATUS_ERROR_CLIENT_CALLBACK);
  TRACE_END (make_query_ref (query, param_out (value)));
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_process_queue_list (amd_dbgapi_process_id_t process_id,
                               size_t *queue_count,
                               amd_dbgapi_queue_id_t **queues,
                               amd_dbgapi_changed_t *changed)
{
  TRACE_BEGIN (param_in (process_id), param_in (queue_count),
               param_in (queues), param_in (changed));
  TRY
  {
    if (!detail::is_initialized)
      THROW (AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED);

    std::vector<process_t *> processes = process_t::match (process_id);

    if (!queues || !queue_count)
      THROW (AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT);

    for (auto &&process : processes)
      process->update_queues ();

    std::tie (*queues, *queue_count)
      = utils::get_handle_list<queue_t> (processes, changed);

    return AMD_DBGAPI_STATUS_SUCCESS;
  }
  CATCH (AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED,
         AMD_DBGAPI_STATUS_ERROR_INVALID_PROCESS_ID,
         AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT,
         AMD_DBGAPI_STATUS_ERROR_CLIENT_CALLBACK);
  TRACE_END (make_ref (param_out (queue_count)),
             make_ref (make_ref (param_out (queues)), *queue_count),
             make_ref (param_out (changed)));
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_queue_packet_list (
  amd_dbgapi_queue_id_t queue_id,
  amd_dbgapi_os_queue_packet_id_t *read_packet_id_p,
  amd_dbgapi_os_queue_packet_id_t *write_packet_id_p,
  amd_dbgapi_size_t *packets_byte_size_p, void **packets_bytes_p)
{
  TRACE_BEGIN (param_in (queue_id), param_in (read_packet_id_p),
               param_in (write_packet_id_p), param_in (packets_byte_size_p),
               param_in (packets_bytes_p));
  TRY
  {
    if (!detail::is_initialized)
      THROW (AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED);

    if (!read_packet_id_p || !write_packet_id_p || !packets_byte_size_p)
      THROW (AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT);

    queue_t *queue = find (queue_id);

    if (!queue)
      THROW (AMD_DBGAPI_STATUS_ERROR_INVALID_QUEUE_ID);

    scoped_queue_suspend_t suspend (*queue, "refresh packet list");

    amd_dbgapi_os_queue_packet_id_t read_packet_id, write_packet_id;
    size_t memory_size;

    queue->active_packets_info (&read_packet_id, &write_packet_id,
                                &memory_size);

    if (packets_bytes_p)
      {
        auto memory = allocate_memory (memory_size);

        queue->active_packets_bytes (read_packet_id, write_packet_id,
                                     memory.get (), memory_size);

        *packets_bytes_p = memory.release ();
      }

    *read_packet_id_p = read_packet_id;
    *write_packet_id_p = write_packet_id;
    *packets_byte_size_p = memory_size;

    return AMD_DBGAPI_STATUS_SUCCESS;
  }
  CATCH (AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED,
         AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT,
         AMD_DBGAPI_STATUS_ERROR_NOT_SUPPORTED, AMD_DBGAPI_STATUS_ERROR,
         AMD_DBGAPI_STATUS_ERROR_CLIENT_CALLBACK);
  TRACE_END (make_hex (make_ref (param_out (read_packet_id_p))),
             make_hex (make_ref (param_out (write_packet_id_p))),
             make_ref (param_out (packets_byte_size_p)),
             make_hex (make_ref (make_ref (param_out (packets_bytes_p)),
                                 *packets_byte_size_p)));
}
