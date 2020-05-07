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

#include "defs.h"

#include "architecture.h"
#include "debug.h"
#include "dispatch.h"
#include "handle_object.h"
#include "logging.h"
#include "process.h"
#include "queue.h"
#include "register.h"
#include "utils.h"
#include "wave.h"

#include <hsa/amd_hsa_queue.h>

#include <memory>

namespace amd
{
namespace dbgapi
{

using utils::bit_extract;

#define SQ_WAVE_TRAPSTS_XNACK_ERROR(x) bit_extract ((x), 28, 28)
#define SQ_WAVE_STATUS_HALT_MASK utils::bit_mask (13, 13)
#define TTMP11_TRAP_HANDLER_EVENTS_MASK utils::bit_mask (7, 8)

/* COMPUTE_RELAUNCH register fields.  */
#define COMPUTE_RELAUNCH_PAYLOAD_VGPRS(x) bit_extract ((x), 0, 5)
#define COMPUTE_RELAUNCH_PAYLOAD_SGPRS(x) bit_extract ((x), 6, 8)
#define COMPUTE_RELAUNCH_GFX9_PAYLOAD_LDS_SIZE(x) bit_extract ((x), 9, 17)
#define COMPUTE_RELAUNCH_GFX10_PAYLOAD_LDS_SIZE(x) bit_extract ((x), 10, 17)
#define COMPUTE_RELAUNCH_PAYLOAD_LAST_WAVE(x) bit_extract ((x), 16, 16)
#define COMPUTE_RELAUNCH_PAYLOAD_FIRST_WAVE(x) bit_extract ((x), 17, 17)
#define COMPUTE_RELAUNCH_GFX10_PAYLOAD_W32_EN(x) bit_extract ((x), 24, 24)
#define COMPUTE_RELAUNCH_IS_EVENT(x) bit_extract ((x), 30, 30)
#define COMPUTE_RELAUNCH_IS_STATE(x) bit_extract ((x), 31, 31)

queue_t::queue_t (amd_dbgapi_queue_id_t queue_id, agent_t &agent,
                  const kfd_queue_snapshot_entry &kfd_queue_info)
    : handle_object (queue_id), m_kfd_queue_info (kfd_queue_info),
      m_agent (agent)
{
  if (kfd_queue_type () != KFD_IOC_QUEUE_TYPE_COMPUTE_AQL)
    {
      m_is_valid = true;
      return;
    }

  m_context_save_start_address = m_kfd_queue_info.ctx_save_restore_address
                                 + sizeof (context_save_area_header_s);

  /* FIXME: This is only temporary, we are using the free space in the queue
     control stack memory. The control stack grows from high to low address, so
     we can steal bytes between the context save area header and the top of
     stack limit. queue_t::update_waves () checks that the area is not
     overwritten.  */

  m_displaced_stepping_buffer_address = m_context_save_start_address;
  m_context_save_start_address
      += architecture ().displaced_stepping_buffer_size ();

  m_parked_wave_buffer_address = m_context_save_start_address;
  m_context_save_start_address
      += architecture ().breakpoint_instruction ().size ();

  if (process ().write_global_memory (
          m_parked_wave_buffer_address,
          architecture ().breakpoint_instruction ().data (),
          architecture ().breakpoint_instruction ().size ())
      != AMD_DBGAPI_STATUS_SUCCESS)
    error ("Could not write to the parked wave instruction buffer");

  m_endpgm_buffer_address = m_context_save_start_address;
  m_context_save_start_address
      += architecture ().endpgm_instruction ().size ();

  if (process ().write_global_memory (
          m_endpgm_buffer_address,
          architecture ().endpgm_instruction ().data (),
          architecture ().endpgm_instruction ().size ())
      != AMD_DBGAPI_STATUS_SUCCESS)
    error ("Could not write to the endpgm instruction buffer");

  /* Read the local and private apertures.  */

  /* The group_segment_aperture_base_hi is stored in the ABI-stable part of the
     amd_queue_t. Since we know the address of the read_dispatch_id (obtained
     from the KFD through the queue snapshot info), which is also stored in the
     ABI-stable part of the amd_queue_t, we can calculate the address of the
     pointer to the group_segment_aperture_base_hi and read it.   */

  uint32_t group_segment_aperture_base_hi;
  if (process ().read_global_memory (
          m_kfd_queue_info.read_pointer_address
              + offsetof (amd_queue_t, group_segment_aperture_base_hi)
              - offsetof (amd_queue_t, read_dispatch_id),
          &group_segment_aperture_base_hi,
          sizeof (group_segment_aperture_base_hi))
      != AMD_DBGAPI_STATUS_SUCCESS)
    error ("Could not read the queue's group_segment_aperture_base_hi");

  m_local_address_space_aperture
      = amd_dbgapi_global_address_t{ group_segment_aperture_base_hi } << 32;

  uint32_t private_segment_aperture_base_hi;
  if (process ().read_global_memory (
          m_kfd_queue_info.read_pointer_address
              + offsetof (amd_queue_t, private_segment_aperture_base_hi)
              - offsetof (amd_queue_t, read_dispatch_id),
          &private_segment_aperture_base_hi,
          sizeof (private_segment_aperture_base_hi))
      != AMD_DBGAPI_STATUS_SUCCESS)
    error ("Could not read the queue's private_segment_aperture_base_hi");

  m_private_address_space_aperture
      = amd_dbgapi_global_address_t{ private_segment_aperture_base_hi } << 32;

  /* Read the hsa_queue_t at the top of the amd_queue_t. Since the amd_queue_t
    structure could change, it can only be accessed by calculating its address
    from the address of the read_dispatch_id by subtracting
    read_dispatch_id_field_base_byte_offset .  */

  uint32_t read_dispatch_id_field_base_byte_offset;
  if (process ().read_global_memory (
          m_kfd_queue_info.read_pointer_address
              + offsetof (amd_queue_t, read_dispatch_id_field_base_byte_offset)
              - offsetof (amd_queue_t, read_dispatch_id),
          &read_dispatch_id_field_base_byte_offset,
          sizeof (read_dispatch_id_field_base_byte_offset))
      != AMD_DBGAPI_STATUS_SUCCESS)
    error (
        "Could not read the queue's read_dispatch_id_field_base_byte_offset");

  amd_dbgapi_global_address_t hsa_queue_address
      = m_kfd_queue_info.read_pointer_address
        - read_dispatch_id_field_base_byte_offset;
  if (process ().read_global_memory (hsa_queue_address, &m_hsa_queue,
                                     sizeof (m_hsa_queue))
      != AMD_DBGAPI_STATUS_SUCCESS)
    error ("Could not read the hsa_queue_t struct");

  if (reinterpret_cast<uintptr_t> (m_hsa_queue.base_address)
      != m_kfd_queue_info.ring_base_address)
    error ("hsa_queue_t base address != kfd queue info base address");

  if ((m_hsa_queue.size * 64) != m_kfd_queue_info.ring_size)
    error ("hsa_queue_t size != kfd queue info ring size");

  m_is_valid = true;
}

queue_t::~queue_t ()
{
  if (!mark ())
    /* This is a partially initialized queue, skip it.  */
    return;

  /* TODO: we need to iterate the waves belonging to this queue
     and enqueue events for aborted requests.  i.e. single-step
     or stop requests that were submitted, but the queue was
     destroyed before reporting the event, we still need to notify
     the application, so that it does not wait forever.  */
}

amd_dbgapi_queue_type_t
queue_t::type () const
{
  if (kfd_queue_type () == KFD_IOC_QUEUE_TYPE_COMPUTE)
    {
      return AMD_DBGAPI_QUEUE_TYPE_AMD_PM4;
    }
  else if (kfd_queue_type () == KFD_IOC_QUEUE_TYPE_COMPUTE_AQL)
    {
      switch (m_hsa_queue.type)
        {
        case HSA_QUEUE_TYPE_SINGLE:
          return AMD_DBGAPI_QUEUE_TYPE_HSA_KERNEL_DISPATCH_SINGLE_PRODUCER;
        case HSA_QUEUE_TYPE_MULTI:
          return AMD_DBGAPI_QUEUE_TYPE_HSA_KERNEL_DISPATCH_MULTIPLE_PRODUCER;
        case HSA_QUEUE_TYPE_COOPERATIVE:
          return AMD_DBGAPI_QUEUE_TYPE_HSA_KERNEL_DISPATCH_COOPERATIVE;
        }
    }

  return AMD_DBGAPI_QUEUE_TYPE_UNKNOWN;
}

amd_dbgapi_status_t
queue_t::update_waves (update_waves_flag_t flags)
{
  process_t &process = this->process ();
  const epoch_t wave_mark = m_next_wave_mark++;
  amd_dbgapi_status_t status;

  dbgapi_assert (kfd_queue_type () == KFD_IOC_QUEUE_TYPE_COMPUTE_AQL
                 && "queue_t::update_waves can only decode AQL queues");

  const bool force_assign_wave_ids
      = !!(flags & update_waves_flag_t::FORCE_ASSIGN_WAVE_IDS);
  const bool unhide_waves_halted_at_launch
      = !!(flags & update_waves_flag_t::UNHIDE_WAVES_HALTED_AT_LAUNCH);

  /* Read the queue's write_dispatch_id and read_dispatch_id.  */

  uint64_t write_dispatch_id;
  status = process.read_global_memory (m_kfd_queue_info.write_pointer_address,
                                       &write_dispatch_id,
                                       sizeof (write_dispatch_id));
  if (status != AMD_DBGAPI_STATUS_SUCCESS)
    return status;

  uint64_t read_dispatch_id;
  status = process.read_global_memory (m_kfd_queue_info.read_pointer_address,
                                       &read_dispatch_id,
                                       sizeof (read_dispatch_id));
  if (status != AMD_DBGAPI_STATUS_SUCCESS)
    return status;

  /* Retrieve the used control stack size and used wave area from the
     context save area header.  */

  struct context_save_area_header_s header;

  status = process.read_global_memory (
      m_kfd_queue_info.ctx_save_restore_address, &header, sizeof (header));
  if (status != AMD_DBGAPI_STATUS_SUCCESS)
    return status;

  constexpr size_t max_ctrl_stack_size
      = 32 /* max_waves_per_cu */ * 2 /* registers */ * sizeof (uint32_t)
        + 2 /* PM4 packets */ * sizeof (uint32_t)
        + sizeof (context_save_area_header_s);

  /* Make sure the top of the control stack does not overwrite the displaced
     stepping buffer or parked wave buffer.  */
  if (m_kfd_queue_info.ctx_save_restore_address + header.ctrl_stack_offset
          + header.ctrl_stack_size - max_ctrl_stack_size
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
      m_kfd_queue_info.ctx_save_restore_address + header.ctrl_stack_offset,
      &ctrl_stack[0], header.ctrl_stack_size);
  if (status != AMD_DBGAPI_STATUS_SUCCESS)
    return status;

  /* Decode the ctrl stack.  TODO: We should move the decoding to the
     architecture class as the layout may change between gfxips.  */
  amd_dbgapi_global_address_t wave_area_address
      = m_kfd_queue_info.ctx_save_restore_address + header.wave_state_offset;
  uint32_t wave_lanes = 0, n_vgprs = 0, n_accvgprs = 0, n_sgprs = 0,
           lds_size = 0, padding = 0;
  constexpr uint32_t n_ttmps = 16, n_hwregs = 16;
  wave_t *group_leader = nullptr;

  for (size_t i = 2; /* Skip the 2 PM4 packets at the top of the stack.  */
       i < header.ctrl_stack_size / sizeof (uint32_t); ++i)
    {
      uint32_t relaunch = ctrl_stack[i];

      if (COMPUTE_RELAUNCH_IS_EVENT (relaunch))
        {
          /* Skip events.  */
          continue;
        }
      else if (COMPUTE_RELAUNCH_IS_STATE (relaunch))
        {
          architecture_t::compute_relaunch_abi_t relaunch_abi
              = agent ().architecture ().compute_relaunch_abi ();

          switch (relaunch_abi)
            {
            case architecture_t::compute_relaunch_abi_t::GFX900:
            case architecture_t::compute_relaunch_abi_t::GFX908:
              /* gfx9 only supports wave64 mode.  */
              wave_lanes = 64;

              /* vgprs are allocated in blocks of 4 registers.  */
              n_vgprs = (1 + COMPUTE_RELAUNCH_PAYLOAD_VGPRS (relaunch)) * 4;

              /* sgprs are allocated in blocks of 16 registers. Subtract
                 the ttmps registers from this count, as they will be saved in
                 a different area than the sgprs.  */
              n_sgprs = (1 + COMPUTE_RELAUNCH_PAYLOAD_SGPRS (relaunch)) * 16
                        - n_ttmps;
              padding = n_ttmps * sizeof (uint32_t);

              /* lds_size: 128 bytes granularity.  */
              lds_size = COMPUTE_RELAUNCH_GFX9_PAYLOAD_LDS_SIZE (relaunch)
                         * 128 * 4;
              break;
            case architecture_t::compute_relaunch_abi_t::GFX1000:
              /* On gfx10, there are 2 COMPUTE_RELAUNCH registers for state.
                 Skip COMPUTE_RELAUNCH2 as it is currently unused.  */
              ++i;

              wave_lanes
                  = COMPUTE_RELAUNCH_GFX10_PAYLOAD_W32_EN (relaunch) ? 32 : 64;

              /* vgprs are allocated in blocks of 4/8 registers (W64/32).  */
              n_vgprs = (1 + COMPUTE_RELAUNCH_PAYLOAD_VGPRS (relaunch))
                        * (256 / wave_lanes);

              /* Each wave gets 128 scalar registers.  */
              n_sgprs = 128;
              padding = 0;

              /* lds_size: 128 bytes granularity.  */
              lds_size = COMPUTE_RELAUNCH_GFX10_PAYLOAD_LDS_SIZE (relaunch)
                         * 128 * 4;
              break;
            default:
              dbgapi_assert_not_reached (
                  "compute_relaunch register ABI not supported");
              break;
            }

          switch (relaunch_abi)
            {
            case architecture_t::compute_relaunch_abi_t::GFX900:
            case architecture_t::compute_relaunch_abi_t::GFX1000:
              n_accvgprs = 0;
              break;
            case architecture_t::compute_relaunch_abi_t::GFX908:
              n_accvgprs = n_vgprs;
              break;
            default:
              dbgapi_assert_not_reached (
                  "compute_relaunch register ABI not supported");
              break;
            }

          continue;
        }

      /* The first wave in the group saves the group lds in its save area.  */
      bool first_in_group = COMPUTE_RELAUNCH_PAYLOAD_FIRST_WAVE (relaunch);

      uint32_t wave_area_size = /* vgprs save area */
          (n_vgprs + n_accvgprs) * sizeof (uint32_t) * wave_lanes
          + n_sgprs * sizeof (uint32_t)     /* sgprs save area */
          + n_hwregs * sizeof (uint32_t)    /* hwregs save area */
          + n_ttmps * sizeof (uint32_t)     /* ttmp sgprs save area */
          + (first_in_group ? lds_size : 0) /* lds save area */
          + padding;

      wave_area_address -= wave_area_size;

      const amd_dbgapi_global_address_t hwregs_address
          = wave_area_address
            + (n_vgprs + n_accvgprs) * sizeof (uint32_t) * wave_lanes
            + n_sgprs * sizeof (uint32_t);

      const amd_dbgapi_global_address_t ttmps_address
          = hwregs_address + n_hwregs * sizeof (uint32_t);

      const amd_dbgapi_global_address_t lds_address
          = ttmps_address + n_ttmps * sizeof (uint32_t);

      amd_dbgapi_wave_id_t wave_id = wave_t::undefined;

      /* We will never have hidden waves when force assigning wave ids. All
         waves seen in the control stack get a new wave_t instance with a new
         wave id.  */
      if (!force_assign_wave_ids)
        {
          /* The wave id is preserved in registers ttmp[4:5].  */
          const amd_dbgapi_global_address_t ttmp4_5_address
              = ttmps_address
                + (amdgpu_regnum_t::TTMP4 - amdgpu_regnum_t::FIRST_TTMP)
                      * sizeof (uint32_t);

          status = process.read_global_memory (ttmp4_5_address, &wave_id,
                                               sizeof (wave_id));
          if (status != AMD_DBGAPI_STATUS_SUCCESS)
            return status;

          /* A wave should be ignored if its wave_id is wave_t::ignored_wave,
             or if its wave_id is wave_t::undefined and it is halted without
             having entered the trap handler.  */
          if (wave_id == wave_t::undefined && !unhide_waves_halted_at_launch)
            {
              uint32_t status_reg;
              const amd_dbgapi_global_address_t status_reg_address
                  = hwregs_address
                    + (amdgpu_regnum_t::STATUS - amdgpu_regnum_t::FIRST_HWREG)
                          * sizeof (uint32_t);

              status = process.read_global_memory (
                  status_reg_address, &status_reg, sizeof (status_reg));
              if (status != AMD_DBGAPI_STATUS_SUCCESS)
                return status;

              const bool halted = !!(status_reg & SQ_WAVE_STATUS_HALT_MASK);

              const amd_dbgapi_global_address_t ttmp11_address
                  = ttmps_address
                    + (amdgpu_regnum_t::TTMP11 - amdgpu_regnum_t::FIRST_TTMP)
                          * sizeof (uint32_t);

              uint32_t ttmp11;
              status = process.read_global_memory (ttmp11_address, &ttmp11,
                                                   sizeof (ttmp11));
              if (status != AMD_DBGAPI_STATUS_SUCCESS)
                return status;

              /* trap_handler_events is true if the trap handler was entered
                 because of a trap instruction or an exception.  */
              const bool trap_handler_events
                  = !!(ttmp11 & TTMP11_TRAP_HANDLER_EVENTS_MASK);

              /* Waves halted at launch do not have trap handler events).  */
              if (halted && !trap_handler_events)
                continue;
            }
          else if (wave_id == wave_t::ignored_wave)
            {
              continue;
            }
        }

      wave_t *wave = nullptr;

      if (wave_id != wave_t::undefined)
        {
          /* The wave already exists, so we should find it and update its
             context save area address.  */
          wave = process.find (wave_id);
          if (!wave)
            warning ("%s not found in the process map",
                     to_string (wave_id).c_str ());
        }

      if (!wave)
        {
          /* The dispatch_ptr is preserved in registers ttmp[6:7].  */
          const amd_dbgapi_global_address_t ttmp6_7_address
              = ttmps_address
                + (amdgpu_regnum_t::TTMP6 - amdgpu_regnum_t::FIRST_TTMP)
                      * sizeof (uint32_t);

          amd_dbgapi_global_address_t dispatch_ptr;
          status = process.read_global_memory (ttmp6_7_address, &dispatch_ptr,
                                               sizeof (dispatch_ptr));
          if (status != AMD_DBGAPI_STATUS_SUCCESS)
            return status;

          if (!dispatch_ptr)
            {
              warning ("invalid null dispatch_ptr at 0x%lx", ttmp6_7_address);
              /* TODO: See comment above for corrupted wavefronts. This could
                 be attached to a CORRUPT_DISPATCH instance.  */
              continue;
            }

          /* SPI only sends us the lower 40 bits of the dispatch_ptr, so we
             need to reconstitute it using the ring_base_address for the
             missing upper 8 bits.  */

          constexpr uint64_t spi_mask = utils::bit_mask (0, 39);
          if (dispatch_ptr)
            dispatch_ptr = (dispatch_ptr & spi_mask)
                           | (m_kfd_queue_info.ring_base_address & ~spi_mask);

          /* Calculate the monotonic dispatch id for this packet.  It is
             between read_dispatch_id and write_dispatch_id.  */

          amd_dbgapi_queue_packet_id_t queue_packet_id
              = (dispatch_ptr - m_kfd_queue_info.ring_base_address)
                / aql_packet_size;

          /* Check that 0 <= queue_packet_id < queue_size.  */
          if (queue_packet_id >= m_kfd_queue_info.ring_size / aql_packet_size)
            {
              warning ("invalid queue_packet_id (%#lx)", queue_packet_id);
              /* TODO: See comment above for corrupted wavefronts. This could
                 be attached to a CORRUPT_DISPATCH instance.  */
              continue;
            }

          /* ring_size must be a power of 2.  */
          if (!utils::is_power_of_two (m_kfd_queue_info.ring_size))
            error ("ring_size is not a power of 2");

          /* Need to mask by the number of packets in the ring (which is a
             power of 2 so -1 makes the correct mask).  */
          const uint64_t id_mask
              = m_kfd_queue_info.ring_size / aql_packet_size - 1;

          queue_packet_id |= queue_packet_id >= (read_dispatch_id & id_mask)
                                 ? (read_dispatch_id & ~id_mask)
                                 : (write_dispatch_id & ~id_mask);

          /* Check that read_dispatch_id <= dispatch_id < write_dispatch_id  */
          if (read_dispatch_id > queue_packet_id
              || queue_packet_id >= write_dispatch_id)
            {
              warning (
                  "invalid dispatch id (%#lx), with read_dispatch_id=%#lx, "
                  "and write_dispatch_id=%#lx",
                  queue_packet_id, read_dispatch_id, write_dispatch_id);
              /* TODO: See comment above for corrupted wavefronts. This could
                 be attached to a CORRUPT_DISPATCH instance.  */
              continue;
            }

          /* Check if the dispatch already exists.  */
          dispatch_t *dispatch = process.find_if ([&] (const dispatch_t &x) {
            return x.queue ().id () == id ()
                   && x.queue_packet_id () == queue_packet_id;
          });

          /* If we did not find the current dispatch, create a new one.  */
          if (!dispatch)
            dispatch = &process.create<dispatch_t> (
                *this,           /* queue  */
                queue_packet_id, /* queue_packet_id  */
                dispatch_ptr);   /* packet_address  */

          amd_dbgapi_size_t lds_offset = lds_address - wave_area_address;
          wave = &process.create<wave_t> (*dispatch,  /* dispatch  */
                                          n_vgprs,    /* vgpr_count  */
                                          n_accvgprs, /* accvgpr_count */
                                          n_sgprs,    /* sgpr_count  */
                                          lds_offset, /* local_memory_offset */
                                          lds_size,   /* local_memory_size  */
                                          wave_lanes); /* lane_count  */
        }

      /* The first wave in the group is the group leader.  The group leader
         owns the backing store for the group memory (LDS).  */
      if (first_in_group)
        group_leader = wave;

      if (!group_leader)
        error ("No group_leader, the control stack may be corrupted");

      status = wave->update (*group_leader, wave_area_address,
                             unhide_waves_halted_at_launch);
      if (status != AMD_DBGAPI_STATUS_SUCCESS)
        return status;

      /* This was the last wave in the group. Make sure we have a new group
         leader for the remaining waves.  */
      if (COMPUTE_RELAUNCH_PAYLOAD_LAST_WAVE (relaunch))
        group_leader = nullptr;

      /* Check that the wave is in the same group as its group leader.  */
      if (wave->group_ids () != wave->group_leader ().group_ids ())
        error ("wave is not in the same group as group_leader");

      wave->set_mark (wave_mark);
    }

  /* Check that we have correctly walked the control stack. At the end,
     wave_area_address should point to the bottom of the context save area.  */

  if (wave_area_address
      != (m_kfd_queue_info.ctx_save_restore_address + header.wave_state_offset
          - header.wave_state_size))
    warning ("ROCm-GDB: Computed save_area_size does match expected size. "
             "Expected %d bytes, calculated %lld bytes.",
             header.wave_state_size,
             m_kfd_queue_info.ctx_save_restore_address
                 + header.wave_state_offset - wave_area_address);

  /* Prune old dispatches. Dispatches with ids older (smaller) than the
     queue current read dispatch id are now retired, so remove them from
     the process.  */

  auto &&dispatch_range = process.range<dispatch_t> ();
  for (auto dispatch_it = dispatch_range.begin ();
       dispatch_it != dispatch_range.end ();)
    if (dispatch_it->queue ().id () == id ()
        && dispatch_it->queue_packet_id () < read_dispatch_id)
      dispatch_it = process.destroy (dispatch_it);
    else
      ++dispatch_it;

  /* Iterate all waves belonging to this queue, and prune those with a mark
     older than the current mark.  */

  auto &&wave_range = process.range<wave_t> ();
  for (auto wave_it = wave_range.begin (); wave_it != wave_range.end ();)
    if (wave_it->queue ().id () == id () && wave_it->mark () < wave_mark)
      wave_it = process.destroy (wave_it);
    else
      ++wave_it;

  return AMD_DBGAPI_STATUS_SUCCESS;
}

std::pair<amd_dbgapi_queue_packet_id_t, std::vector<uint8_t>>
queue_t::packets () const
{
  dbgapi_assert (suspended ());

  uint64_t read_dispatch_id;
  if (process ().read_global_memory (m_kfd_queue_info.read_pointer_address,
                                     &read_dispatch_id,
                                     sizeof (read_dispatch_id))
      != AMD_DBGAPI_STATUS_SUCCESS)
    error ("Could not read the queue's read_dispatch_id");

  uint64_t write_dispatch_id;
  if (process ().read_global_memory (m_kfd_queue_info.write_pointer_address,
                                     &write_dispatch_id,
                                     sizeof (write_dispatch_id))
      != AMD_DBGAPI_STATUS_SUCCESS)
    error ("Could not read the queue's write_dispatch_id");

  /* ring_size must be a power of 2.  */
  if (!utils::is_power_of_two (m_kfd_queue_info.ring_size))
    error ("ring_size is not a power of 2");

  const uint64_t id_mask = m_kfd_queue_info.ring_size / aql_packet_size - 1;

  amd_dbgapi_global_address_t read_dispatch_ptr
      = m_kfd_queue_info.ring_base_address
        + (read_dispatch_id & id_mask) * aql_packet_size;
  amd_dbgapi_global_address_t write_dispatch_ptr
      = m_kfd_queue_info.ring_base_address
        + (write_dispatch_id & id_mask) * aql_packet_size;

  std::vector<uint8_t> packets;
  dbgapi_assert (write_dispatch_id >= read_dispatch_id);
  packets.resize ((write_dispatch_id - read_dispatch_id) * aql_packet_size);

  if (read_dispatch_ptr < write_dispatch_ptr)
    {
      if (process ().read_global_memory (read_dispatch_ptr, &packets[0],
                                         packets.size ())
          != AMD_DBGAPI_STATUS_SUCCESS)
        error ("Could not read the queue's packets");
    }
  else if (read_dispatch_ptr > write_dispatch_ptr)
    {
      size_t size = m_kfd_queue_info.ring_base_address
                    + m_kfd_queue_info.ring_size - read_dispatch_ptr;

      if (process ().read_global_memory (read_dispatch_ptr, &packets[0], size)
          != AMD_DBGAPI_STATUS_SUCCESS)
        error ("Could not read the queue's packets");

      size_t offset = size;
      size = write_dispatch_ptr - m_kfd_queue_info.ring_base_address;

      if (process ().read_global_memory (m_kfd_queue_info.ring_base_address,
                                         &packets[offset], size)
          != AMD_DBGAPI_STATUS_SUCCESS)
        error ("Could not read the queue's packets");
    }

  return std::make_pair (read_dispatch_id, std::move (packets));
}

void
queue_t::set_suspended (bool suspended)
{
  m_suspended = suspended;

  /* Refresh the scratch_backing_memory_location and
     scratch_backing_memory_size everytime we suspend the queue.  */
  if (suspended)
    {
      /* The scratch backing memory address is stored in the ABI-stable part
         of the amd_queue_t. Since we know the address of the read_dispatch_id
         (obtained from the KFD through the queue snapshot info), which is also
         stored in the ABI-stable part of the amd_queue_t, we can calculate the
         address of the pointer to the scratch_backing_memory_location and read
         it.  We cannot cache this value as the runtime may change the
         allocation dynamically.  */

      if (process ().read_global_memory (
              m_kfd_queue_info.read_pointer_address
                  + offsetof (amd_queue_t, scratch_backing_memory_location)
                  - offsetof (amd_queue_t, read_dispatch_id),
              &m_scratch_backing_memory_address,
              sizeof (m_scratch_backing_memory_address))
          != AMD_DBGAPI_STATUS_SUCCESS)
        error ("Could not read the queue's scratch_backing_memory_location");

      if (process ().read_global_memory (
              m_kfd_queue_info.read_pointer_address
                  + offsetof (amd_queue_t, scratch_backing_memory_byte_size)
                  - offsetof (amd_queue_t, read_dispatch_id),
              &m_scratch_backing_memory_size,
              sizeof (m_scratch_backing_memory_size))
          != AMD_DBGAPI_STATUS_SUCCESS)
        error ("Could not read the queue's scratch_backing_memory_size");
    }
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

    case AMD_DBGAPI_QUEUE_INFO_STATE:
    case AMD_DBGAPI_QUEUE_INFO_ERROR_REASON:
      return AMD_DBGAPI_STATUS_ERROR_UNIMPLEMENTED;
    }
  return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;
}

scoped_queue_suspend_t::scoped_queue_suspend_t (queue_t &queue)
    : m_queue (!queue.suspended () ? &queue : nullptr)
{
  if (!m_queue)
    return;

  if (m_queue->process ().suspend_queues ({ m_queue })
      != AMD_DBGAPI_STATUS_SUCCESS)
    error ("process::suspend_queues failed");
}

scoped_queue_suspend_t::~scoped_queue_suspend_t ()
{
  if (!m_queue || !m_queue->process ().forward_progress_needed ())
    return;

  if (m_queue->process ().resume_queues ({ m_queue })
      != AMD_DBGAPI_STATUS_SUCCESS)
    error ("process::resume_queues failed");
}

} /* namespace dbgapi */
} /* namespace amd */

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
                              amd_dbgapi_queue_packet_id_t *first_packet_id,
                              amd_dbgapi_size_t *packets_byte_size,
                              void **packets_bytes)
{
  TRY;
  TRACE (process_id, queue_id);

  if (!amd::dbgapi::is_initialized)
    return AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED;

  if (!first_packet_id || !packets_byte_size || !packets_bytes)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;

  process_t *process = process_t::find (process_id);

  if (!process)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_PROCESS_ID;

  queue_t *queue = process->find (queue_id);

  if (!queue)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_QUEUE_ID;

  scoped_queue_suspend_t suspend (*queue);

  std::vector<uint8_t> packets;
  std::tie (*first_packet_id, packets) = queue->packets ();

  void *retval = amd::dbgapi::allocate_memory (packets.size ());
  if (packets.size () && !retval)
    return AMD_DBGAPI_STATUS_ERROR_CLIENT_CALLBACK;

  memcpy (retval, packets.data (), packets.size ());

  *packets_bytes = retval;
  *packets_byte_size = packets.size ();

  return AMD_DBGAPI_STATUS_SUCCESS;
  CATCH;
}
