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

#include "architecture.h"
#include "agent.h"
#include "debug.h"
#include "exception.h"
#include "initialization.h"
#include "logging.h"
#include "memory.h"
#include "process.h"
#include "register.h"
#include "utils.h"
#include "wave.h"

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <optional>
#include <set>
#include <string>
#include <sys/types.h>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include <amd_comgr.h>
#include <ctype.h>

namespace amd::dbgapi
{

namespace detail
{
const architecture_t *last_found_architecture = nullptr;
} /* namespace detail */

decltype (architecture_t::s_next_architecture_id)
  architecture_t::s_next_architecture_id;

const agent_t &
architecture_t::cwsr_record_t::agent () const
{
  return queue ().agent ();
}

process_t &
architecture_t::cwsr_record_t::process () const
{
  return agent ().process ();
}

const architecture_t &
architecture_t::cwsr_record_t::architecture () const
{
  return queue ().architecture ();
}

/* Base class for all AMDGCN architectures.  */

class amdgcn_architecture_t : public architecture_t
{
protected:
  mutable std::optional<amd_comgr_disassembly_info_t> m_disassembly_info;

  /* Instruction decoding helpers.  */
  enum class cbranch_cond_t
  {
    scc0,             /* Scalar condition code is 0.  */
    scc1,             /* Scalar condition code is 1.  */
    execz,            /* All EXEC mask bits are zero.  */
    execnz,           /* Not all EXEC mask bits are zero.  */
    vccz,             /* All Vector Condition Code bits are zero.  */
    vccnz,            /* Not all Vector Condition Code bits are zero.  */
    cdbgsys,          /* Conditional Debug for System is 1.  */
    cdbguser,         /* Conditional Debug for User is 1.  */
    cdbgsys_or_user,  /* Conditional Debug for System or User is 1.  */
    cdbgsys_and_user, /* Conditional Debug for System and User is 1.  */
  };

  static constexpr uint32_t sq_wave_status_scc_mask = 1 << 0;
  static constexpr uint32_t sq_wave_status_priv_mask = 1 << 5;
  static constexpr uint32_t sq_wave_status_trap_en_mask = 1 << 6;
  static constexpr uint32_t sq_wave_status_execz_mask = 1 << 9;
  static constexpr uint32_t sq_wave_status_vccz_mask = 1 << 10;
  static constexpr uint32_t sq_wave_status_halt_mask = 1 << 13;
  static constexpr uint32_t sq_wave_status_cond_dbg_user_mask = 1 << 20;
  static constexpr uint32_t sq_wave_status_cond_dbg_sys_mask = 1 << 21;

  static constexpr uint32_t ttmp11_wave_in_group_mask = utils::bit_mask (0, 5);
  static constexpr uint32_t ttmp6_wave_stopped_mask = 1 << 30;
  static constexpr uint32_t ttmp6_saved_status_halt_mask = 1 << 29;
  static constexpr uint32_t ttmp6_saved_trap_id_mask
    = utils::bit_mask (25, 28);
  static constexpr int ttmp6_saved_trap_id_shift = 25;
  static constexpr uint32_t ttmp6_queue_packet_id_mask
    = utils::bit_mask (0, 24);
  static constexpr int ttmp6_queue_packet_id_shift = 0;

  /* See https://llvm.org/docs/AMDGPUUsage.html#trap-handler-abi  */
  enum class trap_id_t : uint8_t
  {
    reserved = 0x0,
    breakpoint = 0x1,
    assert_trap = 0x2,
    debug_trap = 0x3,
  };

  /* The trap handler only saves 4 bits of the original trap_id.  trap_id 0
     cannot be detected and is invalid, trap_id >= 15 could be any ID between
     15 and 255, and therefore cannot be differentiated.  The debugger API
     library only handles trap IDs between 1 and 14.  */
  static constexpr std::optional<trap_id_t> ttmp6_saved_trap_id (uint32_t x)
  {
    if (uint8_t trap_id = utils::bit_extract (x, 25, 28); trap_id != 0)
      return trap_id_t{ trap_id };
    return std::nullopt;
  }

  static constexpr uint32_t sq_wave_mode_debug_en_mask = 1 << 11;
  static constexpr uint32_t sq_wave_mode_excp_en_invalid_mask = 1 << 12;
  static constexpr uint32_t sq_wave_mode_excp_en_input_denorm_mask = 1 << 13;
  static constexpr uint32_t sq_wave_mode_excp_en_div0_mask = 1 << 14;
  static constexpr uint32_t sq_wave_mode_excp_en_overflow_mask = 1 << 15;
  static constexpr uint32_t sq_wave_mode_excp_en_underflow_mask = 1 << 16;
  static constexpr uint32_t sq_wave_mode_excp_en_inexact_mask = 1 << 17;
  static constexpr uint32_t sq_wave_mode_excp_en_int_div0_mask = 1 << 18;
  static constexpr uint32_t sq_wave_mode_excp_en_addr_watch_mask = 1 << 19;

  static constexpr uint32_t sq_wave_trapsts_excp_invalid_mask = 1 << 0;
  static constexpr uint32_t sq_wave_trapsts_excp_input_denorm_mask = 1 << 1;
  static constexpr uint32_t sq_wave_trapsts_excp_div0_mask = 1 << 2;
  static constexpr uint32_t sq_wave_trapsts_excp_overflow_mask = 1 << 3;
  static constexpr uint32_t sq_wave_trapsts_excp_underflow_mask = 1 << 4;
  static constexpr uint32_t sq_wave_trapsts_excp_inexact_mask = 1 << 5;
  static constexpr uint32_t sq_wave_trapsts_excp_int_div0_mask = 1 << 6;
  static constexpr uint32_t sq_wave_trapsts_excp_addr_watch0_mask = 1 << 7;
  static constexpr uint32_t sq_wave_trapsts_excp_mem_viol_mask = 1 << 8;
  static constexpr uint32_t sq_wave_trapsts_savectx_mask = 1 << 10;
  static constexpr uint32_t sq_wave_trapsts_illegal_inst_mask = 1 << 11;
  static constexpr uint32_t sq_wave_trapsts_excp_hi_addr_watch1_mask = 1 << 12;
  static constexpr uint32_t sq_wave_trapsts_excp_hi_addr_watch2_mask = 1 << 13;
  static constexpr uint32_t sq_wave_trapsts_excp_hi_addr_watch3_mask = 1 << 14;
  static constexpr uint32_t sq_wave_trapsts_xnack_error_mask = 1 << 28;

  static constexpr uint32_t sq_wave_trapsts_excp_mask = utils::bit_mask (0, 8);
  static constexpr uint32_t sq_wave_trapsts_excp_hi_mask
    = utils::bit_mask (12, 14);

  static constexpr uint32_t compute_relaunch_is_event (uint32_t relaunch)
  {
    return utils::bit_extract (relaunch, 30, 30);
  }
  static constexpr uint32_t compute_relaunch_is_state (uint32_t relaunch)
  {
    return utils::bit_extract (relaunch, 31, 31);
  }

  class kernel_descriptor_t : public architecture_t::kernel_descriptor_t
  {
  private:
    struct
    {
      uint32_t group_segment_fixed_size;
      uint32_t private_segment_fixed_size;
      uint8_t reserved0[8];
      int64_t kernel_code_entry_byte_offset;
      uint8_t reserved1[20];
      uint32_t compute_pgm_rsrc3;
      uint32_t compute_pgm_rsrc1;
      uint32_t compute_pgm_rsrc2;
      uint16_t kernel_code_properties;
      uint8_t reserved2[6];
    } m_descriptor;

  public:
    kernel_descriptor_t (process_t &process,
                         amd_dbgapi_global_address_t address)
      : architecture_t::kernel_descriptor_t (process, address)
    {
      process.read_global_memory (address, &m_descriptor);
    }

    amd_dbgapi_global_address_t entry_address () const override
    {
      return address () + m_descriptor.kernel_code_entry_byte_offset;
    }
  };

  class cwsr_record_t : public architecture_t::cwsr_record_t
  {
  protected:
    cwsr_record_t (compute_queue_t &queue)
      : architecture_t::cwsr_record_t (queue)
    {
    }

    /* Number for vector registers.  */
    virtual size_t vgpr_count () const = 0;
    /* Number of scalar registers.  */
    virtual size_t sgpr_count () const = 0;
  };

  amdgcn_architecture_t (elf_amdgpu_machine_t e_machine,
                         std::string target_triple)
    : architecture_t (e_machine, std::move (target_triple))
  {
  }

  ~amdgcn_architecture_t ()
  {
    if (m_disassembly_info)
      amd_comgr_destroy_disassembly_info (*m_disassembly_info);
  }

  amd_comgr_disassembly_info_t disassembly_info () const;

public:
  std::unique_ptr<const architecture_t::kernel_descriptor_t>
  make_kernel_descriptor (
    process_t &process,
    amd_dbgapi_global_address_t kernel_descriptor_address) const override
  {
    return std::make_unique<amdgcn_architecture_t::kernel_descriptor_t> (
      process, kernel_descriptor_address);
  }

  std::pair<amd_dbgapi_segment_address_t /* to_address  */,
            amd_dbgapi_size_t /* to_contiguous_bytes  */>
  convert_address_space (
    const wave_t &wave, amd_dbgapi_lane_id_t lane_id,
    const address_space_t &from_address_space,
    const address_space_t &to_address_space,
    amd_dbgapi_segment_address_t from_address) const override;

  std::pair<const address_space_t & /* lowered_address_space  */,
            amd_dbgapi_segment_address_t /* lowered_address  */>
  lower_address_space (
    const wave_t &wave, const address_space_t &original_address_space,
    amd_dbgapi_segment_address_t original_address) const override;

  bool address_is_in_address_class (
    const wave_t &wave, amd_dbgapi_lane_id_t lane_id,
    const address_space_t &address_space,
    amd_dbgapi_segment_address_t segment_address,
    const address_class_t &address_class) const override;

  bool is_address_space_supported (
    const address_space_t &address_space) const override;

  bool is_address_class_supported (
    const address_class_t &address_class) const override;

  amd_dbgapi_global_address_t address_aperture_mask () const override
  {
    return utils::bit_mask (48, 63);
  }

  amd_dbgapi_size_t private_swizzled_interleave_size () const override
  {
    return sizeof (uint32_t);
  };

  std::vector<os_watch_id_t>
  triggered_watchpoints (const wave_t &wave) const override;

  std::string register_name (amdgpu_regnum_t regnum) const override;
  std::string register_type (amdgpu_regnum_t regnum) const override;
  amd_dbgapi_size_t register_size (amdgpu_regnum_t regnum) const override;
  const void *register_read_only_mask (amdgpu_regnum_t regnum) const override;
  amd_dbgapi_register_properties_t
  register_properties (amdgpu_regnum_t regnum) const override;

  bool is_pseudo_register_available (const wave_t &wave,
                                     amdgpu_regnum_t regnum) const override;

  void read_pseudo_register (const wave_t &wave, amdgpu_regnum_t regnum,
                             size_t offset, size_t value_size,
                             void *value) const override;

  void write_pseudo_register (wave_t &wave, amdgpu_regnum_t regnum,
                              size_t offset, size_t value_size,
                              const void *value) const override;

  std::pair<amd_dbgapi_wave_state_t, amd_dbgapi_wave_stop_reasons_t>
  wave_get_state (wave_t &wave) const override;
  void wave_set_state (wave_t &wave, amd_dbgapi_wave_state_t state,
                       amd_dbgapi_exceptions_t exceptions
                       = AMD_DBGAPI_EXCEPTION_NONE) const override;

  bool wave_get_halt (const wave_t &wave) const override;
  void wave_set_halt (wave_t &wave, bool halt) const override;

  virtual uint32_t os_wave_launch_trap_mask_to_wave_mode (
    os_wave_launch_trap_mask_t mask) const;

  void
  wave_enable_traps (wave_t &wave,
                     os_wave_launch_trap_mask_t mask) const override final;
  void
  wave_disable_traps (wave_t &wave,
                      os_wave_launch_trap_mask_t mask) const override final;

  size_t minimum_instruction_alignment () const override final;
  virtual instruction_t trap_instruction (std::optional<trap_id_t> trap_id
                                          = std::nullopt) const;
  instruction_t breakpoint_instruction () const override;
  instruction_t assert_instruction () const override;
  instruction_t debug_trap_instruction () const override;
  instruction_t terminating_instruction () const override;
  size_t breakpoint_instruction_pc_adjust () const override;

protected:
  static uint8_t ssrc0_operand (const instruction_t &instruction);
  static uint8_t ssrc1_operand (const instruction_t &instruction);
  static uint8_t sdst_operand (const instruction_t &instruction);
  static int16_t simm16_operand (const instruction_t &instruction);

  static uint8_t encoding_op7 (const instruction_t &instruction);
  template <int... op5>
  static bool is_sopk_encoding (const instruction_t &instruction);
  template <int... op8>
  static bool is_sop1_encoding (const instruction_t &instruction);
  template <int... op7>
  static bool is_sop2_encoding (const instruction_t &instruction);
  template <int... op7>
  static bool is_sopp_encoding (const instruction_t &instruction);

  /* Return the regnum for a scalar register operand.  If priv=false, the
     trap temporaries are unavailable and should be handled as the null
     register.  */
  virtual std::optional<amdgpu_regnum_t>
  scalar_operand_to_regnum (int operand, bool priv = false) const = 0;
  /* Return the maximum number of scalar registers  */
  virtual size_t scalar_register_count () const = 0;
  /* Return the number of aliased scalar registers (e.g. vcc, flat_scratch)  */
  virtual size_t scalar_alias_count () const = 0;

  /* Return the condition code for the given conditional branch.  */
  virtual cbranch_cond_t
  cbranch_condition_code (const instruction_t &instruction) const = 0;

  virtual bool is_sethalt (const instruction_t &instruction) const = 0;
  virtual bool is_barrier (const instruction_t &instruction) const = 0;
  virtual bool is_sleep (const instruction_t &instruction) const = 0;
  virtual bool is_call (const instruction_t &instruction) const = 0;
  virtual bool is_getpc (const instruction_t &instruction) const = 0;
  virtual bool is_setpc (const instruction_t &instruction) const = 0;
  virtual bool is_swappc (const instruction_t &instruction) const = 0;
  virtual bool is_branch (const instruction_t &instruction) const = 0;
  virtual bool is_cbranch (const instruction_t &instruction) const = 0;
  virtual bool is_cbranch_i_fork (const instruction_t &instruction) const = 0;
  virtual bool is_cbranch_g_fork (const instruction_t &instruction) const = 0;
  virtual bool is_cbranch_join (const instruction_t &instruction) const = 0;
  virtual bool is_trap (const instruction_t &instruction,
                        trap_id_t *trap_id = nullptr) const = 0;
  virtual bool is_endpgm (const instruction_t &instruction) const = 0;
  virtual bool is_sequential (const instruction_t &instruction) const = 0;

  bool
  is_terminating_instruction (const instruction_t &instruction) const override;

  virtual bool can_halt_at_endpgm () const = 0;
  bool park_stopped_waves () const override { return !can_halt_at_endpgm (); }

  virtual bool is_branch_taken (wave_t &wave,
                                const instruction_t &instruction) const;

  virtual amd_dbgapi_global_address_t
  branch_target (wave_t &wave, amd_dbgapi_global_address_t pc,
                 const instruction_t &instruction) const;

  amd_dbgapi_size_t
  instruction_size (const instruction_t &instruction) const override;

  std::tuple<amd_dbgapi_instruction_kind_t,       /* instruction_kind  */
             amd_dbgapi_instruction_properties_t, /* instruction_properties  */
             size_t,                              /* instruction_size  */
             std::vector<uint64_t> /* instruction_information  */>
  classify_instruction (amd_dbgapi_global_address_t address,
                        const instruction_t &instruction) const override;

  std::tuple<amd_dbgapi_size_t /* instruction_size  */,
             std::string /* instruction_text  */,
             std::vector<amd_dbgapi_global_address_t> /* address_operands  */>
  disassemble_instruction (amd_dbgapi_global_address_t address,
                           const instruction_t &instruction) const override;

  bool can_execute_displaced (wave_t &wave,
                              const instruction_t &instruction) const override;
  bool can_simulate (wave_t &wave,
                     const instruction_t &instruction) const override;

  virtual std::optional<amd_dbgapi_global_address_t>
  simulate_instruction (wave_t &wave, amd_dbgapi_global_address_t pc,
                        const instruction_t &instruction) const;

  virtual void
  simulate_trap_handler (wave_t &wave, amd_dbgapi_global_address_t pc,
                         std::optional<trap_id_t> trap_id = {}) const;

  virtual bool simulate (wave_t &wave, amd_dbgapi_global_address_t pc,
                         const instruction_t &instruction) const override;
};

namespace detail
{

struct disassembly_user_data_t
{
  const void *memory;
  size_t offset;
  size_t size;
  std::string *instruction;
  std::vector<amd_dbgapi_global_address_t> *operands;
};

} /* namespace detail */

amd_comgr_disassembly_info_t
amdgcn_architecture_t::disassembly_info () const
{
  if (!m_disassembly_info)
    {
      auto read_memory_callback = [] (uint64_t from, char *to, uint64_t size,
                                      void *user_data) -> uint64_t
      {
        detail::disassembly_user_data_t *data
          = static_cast<detail::disassembly_user_data_t *> (user_data);

        size_t offset = from - data->offset;
        if (offset >= data->size)
          return 0;

        size = std::min (size, data->size - offset);
        memcpy (to, static_cast<const char *> (data->memory) + offset, size);

        return size;
      };

      auto print_instruction_callback
        = [] (const char *instruction, void *user_data)
      {
        detail::disassembly_user_data_t *data
          = static_cast<detail::disassembly_user_data_t *> (user_data);

        while (isspace (*instruction))
          ++instruction;

        if (data->instruction)
          data->instruction->assign (instruction);
      };

      auto print_address_annotation_callback
        = [] (uint64_t address, void *user_data)
      {
        detail::disassembly_user_data_t *data
          = static_cast<detail::disassembly_user_data_t *> (user_data);
        if (data->operands)
          data->operands->emplace_back (
            static_cast<amd_dbgapi_global_address_t> (address));
      };

      if (amd_comgr_create_disassembly_info (
            target_triple ().c_str (), read_memory_callback,
            print_instruction_callback, print_address_annotation_callback,
            &m_disassembly_info.emplace ()))
        fatal_error ("amd_comgr_create_disassembly_info failed");
    }

  return *m_disassembly_info;
}

amd_dbgapi_size_t
amdgcn_architecture_t::instruction_size (
  const instruction_t &instruction) const
{
  struct detail::disassembly_user_data_t user_data
    = { /* .memory =  */ instruction.data (),
        /* .offset =  */ 0,
        /* .size =  */ instruction.capacity (),
        /* .instruction =  */ nullptr,
        /* .operands =  */ nullptr };
  size_t size;

  /* Disassemble one instruction.  */
  if (amd_comgr_disassemble_instruction (disassembly_info (), 0, &user_data,
                                         &size)
      != AMD_COMGR_STATUS_SUCCESS)
    return 0;

  return size;
}

std::tuple<amd_dbgapi_size_t /* instruction_size  */,
           std::string /* instruction_text  */,
           std::vector<amd_dbgapi_global_address_t> /* address_operands  */>
amdgcn_architecture_t::disassemble_instruction (
  amd_dbgapi_global_address_t address, const instruction_t &instruction) const
{
  dbgapi_assert (
    utils::is_aligned (address, minimum_instruction_alignment ()));

  std::string instruction_text;
  std::vector<uint64_t> address_operands;
  size_t size;

  struct detail::disassembly_user_data_t user_data
    = { /* .memory =  */ instruction.data (),
        /* .offset =  */ address,
        /* .size =  */ instruction.capacity (),
        /* .instruction =  */ &instruction_text,
        /* .operands =  */ &address_operands };

  /* Disassemble one instruction.  */
  if (amd_comgr_disassemble_instruction (disassembly_info (),
                                         static_cast<uint64_t> (address),
                                         &user_data, &size)
      != AMD_COMGR_STATUS_SUCCESS)
    return { 0, "<illegal instruction>", {} };

  return std::make_tuple (size, instruction_text, address_operands);
}

namespace
{

/* Helper routine to return the address space for a given generic address.  The
   returned address space can only be one of LOCAL, PRIVATE_SWIZZLED or GLOBAL
   address space. The queue_t is used to retrieve the apertures.
 */
const address_space_t &
address_space_for_generic_address (
  const wave_t &wave, amd_dbgapi_segment_address_t generic_address)
{
  amd_dbgapi_global_address_t aperture
    = generic_address & wave.architecture ().address_aperture_mask ();

  address_space_t::kind_t kind;
  if (aperture == wave.agent ().os_info ().private_address_space_aperture)
    kind = address_space_t::kind_t::private_swizzled;
  else if (aperture == wave.agent ().os_info ().local_address_space_aperture)
    kind = address_space_t::kind_t::local;
  else /* all other addresses (including the NULL address) are treated as
          global addresses.  */
    kind = address_space_t::kind_t::global;

  const address_space_t *segment_address_space = wave.architecture ().find_if (
    [=] (const address_space_t &as) { return as.kind () == kind; });

  dbgapi_assert (segment_address_space != nullptr
                 && "address space not found in architecture");
  return *segment_address_space;
}

/* Helper routine to return the generic address for a given segment address
   space, segment address pair.  Converting an address from an address space
   other than LOCAL, PRIVATE_SWIZZLED or GLOBAL is invalid, and an error is
   returned.  The agent_t::os_info is used to retrieve the apertures.
 */
std::optional<amd_dbgapi_segment_address_t>
generic_address_for_address_space (
  const wave_t &wave, const address_space_t &segment_address_space,
  amd_dbgapi_segment_address_t segment_address)
{
  amd_dbgapi_segment_address_t aperture;

  switch (segment_address_space.kind ())
    {
    case address_space_t::kind_t::local:
      aperture = wave.agent ().os_info ().local_address_space_aperture;
      break;
    case address_space_t::kind_t::private_swizzled:
      aperture = wave.agent ().os_info ().private_address_space_aperture;
      break;
    case address_space_t::kind_t::global:
      aperture = 0;
      break;
    default:
      /* not a valid address space conversion.  */
      return std::nullopt;
    }

  /* A segment NULL should be converted to a generic NULL.  */
  if (segment_address == segment_address_space.null_address ())
    {
      /* generic NULL is the same as global NULL.  */
      return address_space_t::s_global.null_address ();
    }

  amd_dbgapi_segment_address_t segment_mask
    = utils::bit_mask (0, segment_address_space.address_size () - 1);

  dbgapi_assert ((aperture & segment_mask) == 0
                 && "the segment address bits overlap with the aperture");
  return aperture | (segment_address & segment_mask);
}

} /* namespace */

std::pair<const address_space_t & /* lowered_address_space  */,
          amd_dbgapi_segment_address_t /* lowered_address  */>
amdgcn_architecture_t::lower_address_space (
  const wave_t &wave, const address_space_t &original_address_space,
  amd_dbgapi_segment_address_t original_address) const
{
  /* Remove the unused bits from the address.  */
  original_address
    &= utils::bit_mask (0, original_address_space.address_size () - 1);

  if (original_address_space.kind () != address_space_t::kind_t::generic)
    return { original_address_space, original_address };

  /* Convert a generic address into a local/private/global address.  */

  const address_space_t &segment_address_space
    = address_space_for_generic_address (wave, original_address);

  if (original_address == original_address_space.null_address ())
    return { segment_address_space, segment_address_space.null_address () };

  amd_dbgapi_size_t segment_mask
    = utils::bit_mask (0, segment_address_space.address_size () - 1);

  return { segment_address_space, original_address & segment_mask };
}

std::pair<amd_dbgapi_segment_address_t /* to_address  */,
          amd_dbgapi_size_t /* to_contiguous_bytes  */>
amdgcn_architecture_t::convert_address_space (
  const wave_t &wave, amd_dbgapi_lane_id_t lane_id,
  const address_space_t &from_address_space,
  const address_space_t &to_address_space,
  amd_dbgapi_segment_address_t from_address) const
{
  const bool is_null = (from_address == from_address_space.null_address ());

  /* A generic NULL converts to any other address spaces NULL.  */
  if (from_address_space.kind () == address_space_t::kind_t::generic
      && is_null)
    return { to_address_space.null_address (), 0 };

  auto [lowered_address_space, lowered_address]
    = wave.architecture ().lower_address_space (wave, from_address_space,
                                                from_address);

  amd_dbgapi_size_t contiguous_bytes
    = utils::bit_mask (0, lowered_address_space.address_size () - 1)
      - lowered_address + 1;

  if (lowered_address_space.kind () == to_address_space.kind ())
    return { lowered_address, is_null ? 0 : contiguous_bytes };

  /* Convert local, private_swizzled, global -> generic.  */
  if (to_address_space.kind () == address_space_t::kind_t::generic)
    {
      auto generic_address = generic_address_for_address_space (
        wave, lowered_address_space, lowered_address);

      if (!generic_address)
        throw api_error_t (
          AMD_DBGAPI_STATUS_ERROR_INVALID_ADDRESS_SPACE_CONVERSION);

      return { *generic_address, is_null ? 0 : contiguous_bytes };
    }

  /* Convert private_unswizzled -> global.  */
  if (lowered_address_space.kind ()
        == address_space_t::kind_t::private_unswizzled
      && to_address_space.kind () == address_space_t::kind_t::global)
    {
      auto [scratch_base, scratch_size] = wave.scratch_memory_region ();

      if (lowered_address >= scratch_size)
        throw api_error_t (
          AMD_DBGAPI_STATUS_ERROR_INVALID_ADDRESS_SPACE_CONVERSION);

      return { scratch_base + lowered_address,
               scratch_size - lowered_address };
    }

  /* Convert global -> private_unswizzled.  */
  if (lowered_address_space.kind () == address_space_t::kind_t::global
      && to_address_space.kind ()
           == address_space_t::kind_t::private_unswizzled)
    {
      auto [scratch_base, scratch_size] = wave.scratch_memory_region ();

      if (lowered_address < scratch_base
          || lowered_address >= (scratch_base + scratch_size))
        throw api_error_t (
          AMD_DBGAPI_STATUS_ERROR_INVALID_ADDRESS_SPACE_CONVERSION);

      return { lowered_address - scratch_base,
               scratch_base + scratch_size - lowered_address };
    }

  /* Convert private_swizzled -> global.  */
  if (lowered_address_space.kind ()
        == address_space_t::kind_t::private_swizzled
      && to_address_space.kind () == address_space_t::kind_t::global)
    {
      const amd_dbgapi_size_t interleave = private_swizzled_interleave_size ();
      auto [scratch_base, scratch_size] = wave.scratch_memory_region ();

      dbgapi_assert (lane_id < wave.lane_count ());
      amd_dbgapi_size_t offset
        = ((lowered_address / interleave) * wave.lane_count () * interleave)
          + (lane_id * interleave) + (lowered_address % interleave);

      if (offset > scratch_size)
        throw api_error_t (
          AMD_DBGAPI_STATUS_ERROR_INVALID_ADDRESS_SPACE_CONVERSION);

      return { scratch_base + offset, interleave - (offset % interleave) };
    }

  /* Convert global -> private_swizzled.  */
  if (lowered_address_space.kind () == address_space_t::kind_t::global
      && to_address_space.kind () == address_space_t::kind_t::private_swizzled)
    {
      const amd_dbgapi_size_t interleave = private_swizzled_interleave_size ();
      auto [scratch_base, scratch_size] = wave.scratch_memory_region ();

      if (lowered_address < scratch_base
          || lowered_address >= (scratch_base + scratch_size))
        throw api_error_t (
          AMD_DBGAPI_STATUS_ERROR_INVALID_ADDRESS_SPACE_CONVERSION);

      amd_dbgapi_size_t offset = lowered_address - scratch_base;

      dbgapi_assert (lane_id < wave.lane_count ());
      if (lane_id != (offset / interleave) % wave.lane_count ())
        throw api_error_t (
          AMD_DBGAPI_STATUS_ERROR_INVALID_ADDRESS_SPACE_CONVERSION);

      return { (offset / (wave.lane_count () * interleave)) * interleave
                 + offset % interleave,
               interleave - offset % interleave };
    }

  throw api_error_t (AMD_DBGAPI_STATUS_ERROR_INVALID_ADDRESS_SPACE_CONVERSION);
}

bool
amdgcn_architecture_t::address_is_in_address_class (
  const wave_t &wave, amd_dbgapi_lane_id_t /* lane_id  */,
  const address_space_t &address_space,
  amd_dbgapi_segment_address_t segment_address,
  const address_class_t &address_class) const
{
  /* The implementation follows this table:

     address_space     is in  address_class
     private_swizzled         generic, private
     local                    generic, local
     global                   generic, global
     generic                  generic, lowered generic address_space
     private_unswizzled       -
   */

  /* private_unswizzled addresses are not in any address classes.  */
  if (address_space.kind () == address_space_t::kind_t::private_unswizzled)
    return false;

  /* private_swizzled, local, global, and generic are in the generic address
     class.  */
  if (address_class.address_space ().kind ()
      == address_space_t::kind_t::generic)
    return true;

  if (address_space.kind () == address_space_t::kind_t::generic)
    {
      auto &lowered_address_space
        = lower_address_space (wave, address_space, segment_address).first;

      /* A generic private address is in the private address class, a generic
         local address is in the local address class, etc...  */
      return lowered_address_space.kind ()
             == address_class.address_space ().kind ();
    }

  /* The private_swizzled address space is in private address class, the local
     address space is in the local address class, etc...  */
  return address_space.kind () == address_class.address_space ().kind ();
}

bool
amdgcn_architecture_t::is_address_space_supported (
  const address_space_t &address_space) const
{
  return address_space == address_space_t::s_global
         || this->find (address_space.id ()) != nullptr;
}

bool
amdgcn_architecture_t::is_address_class_supported (
  const address_class_t &address_class) const
{
  return this->find (address_class.id ()) != nullptr;
}

std::vector<os_watch_id_t>
amdgcn_architecture_t::triggered_watchpoints (const wave_t &wave) const
{
  std::vector<os_watch_id_t> watchpoints;

  if (wave.state () != AMD_DBGAPI_WAVE_STATE_STOP
      || !(wave.stop_reason () & AMD_DBGAPI_WAVE_STOP_REASON_WATCHPOINT))
    return {};

  /* We don't have to suspend the queue because trapsts is a cached hwreg.  */
  dbgapi_assert (wave.process ().memory_cache ().contains_all (
    *wave.register_address (amdgpu_regnum_t::trapsts), sizeof (uint32_t)));

  uint32_t trapsts;
  wave.read_register (amdgpu_regnum_t::trapsts, &trapsts);

  if (trapsts & sq_wave_trapsts_excp_addr_watch0_mask)
    watchpoints.emplace_back (0);
  if (trapsts & sq_wave_trapsts_excp_hi_addr_watch1_mask)
    watchpoints.emplace_back (1);
  if (trapsts & sq_wave_trapsts_excp_hi_addr_watch2_mask)
    watchpoints.emplace_back (2);
  if (trapsts & sq_wave_trapsts_excp_hi_addr_watch3_mask)
    watchpoints.emplace_back (3);

  return watchpoints;
}

size_t
amdgcn_architecture_t::minimum_instruction_alignment () const
{
  /* Some of the PC register's least significant bits are zero (not wired),
     enforcing a minimum instruction alignment.  This alignment can be deduced
     from the PC register's writable bits mask.  */

  auto *mask = register_read_only_mask (amdgpu_regnum_t::pc);
  dbgapi_assert (mask);

  size_t align = *static_cast<const amd_dbgapi_global_address_t *> (mask) + 1;
  dbgapi_assert (utils::is_power_of_two (align));

  return align;
}

instruction_t
amdgcn_architecture_t::trap_instruction (
  std::optional<trap_id_t> trap_id) const
{
  uint8_t imm8 = static_cast<uint8_t> (trap_id.value_or (trap_id_t::reserved));
  return instruction_t (
    legal_instruction, *this,
    std::vector<std::byte> ({ /* s_trap #imm8  */ std::byte{ imm8 },
                              std::byte{ 0x00 }, std::byte{ 0x92 },
                              std::byte{ 0xBF } }));
}

instruction_t
amdgcn_architecture_t::breakpoint_instruction () const
{
  return trap_instruction (trap_id_t::breakpoint);
}

instruction_t
amdgcn_architecture_t::assert_instruction () const
{
  return trap_instruction (trap_id_t::assert_trap);
}

instruction_t
amdgcn_architecture_t::debug_trap_instruction () const
{
  return trap_instruction (trap_id_t::debug_trap);
}

instruction_t
amdgcn_architecture_t::terminating_instruction () const
{
  return instruction_t (
    legal_instruction, *this,
    std::vector<std::byte> ({ /* s_endpgm 0  */ std::byte{ 0x00 },
                              std::byte{ 0x00 }, std::byte{ 0x81 },
                              std::byte{ 0xBF } }));
}

size_t
amdgcn_architecture_t::breakpoint_instruction_pc_adjust () const
{
  return breakpoint_instruction ().size ();
}

bool
amdgcn_architecture_t::is_terminating_instruction (
  const instruction_t &instruction) const
{
  return is_endpgm (instruction);
};

bool
amdgcn_architecture_t::is_branch_taken (wave_t &wave,
                                        const instruction_t &instruction) const
{
  if (is_cbranch (instruction))
    {
      uint32_t status_reg;

      wave.read_register (amdgpu_regnum_t::status, &status_reg);

      /* Evaluate the condition.  */
      switch (cbranch_condition_code (instruction))
        {
        case cbranch_cond_t::scc0:
          return (status_reg & sq_wave_status_scc_mask) == 0;
        case cbranch_cond_t::scc1:
          return (status_reg & sq_wave_status_scc_mask) != 0;
        case cbranch_cond_t::execz:
          return (status_reg & sq_wave_status_execz_mask) != 0;
        case cbranch_cond_t::execnz:
          return (status_reg & sq_wave_status_execz_mask) == 0;
        case cbranch_cond_t::vccz:
          return (status_reg & sq_wave_status_vccz_mask) != 0;
        case cbranch_cond_t::vccnz:
          return (status_reg & sq_wave_status_vccz_mask) == 0;
        case cbranch_cond_t::cdbgsys:
          return (status_reg & sq_wave_status_cond_dbg_sys_mask) != 0;
        case cbranch_cond_t::cdbguser:
          return (status_reg & sq_wave_status_cond_dbg_user_mask) != 0;
        case cbranch_cond_t::cdbgsys_or_user:
          {
            uint32_t mask = sq_wave_status_cond_dbg_sys_mask
                            | sq_wave_status_cond_dbg_user_mask;
            return (status_reg & mask) != 0;
          }
        case cbranch_cond_t::cdbgsys_and_user:
          {
            uint32_t mask = sq_wave_status_cond_dbg_sys_mask
                            | sq_wave_status_cond_dbg_user_mask;
            return (status_reg & mask) == mask;
          }
        }
      dbgapi_assert_not_reached (
        "illegal instruction: invalid cbranch_cond_t");
    }

  if (is_cbranch_i_fork (instruction) || is_cbranch_g_fork (instruction))
    {
      dbgapi_assert (wave.lane_count () == 64);

      uint32_t mask_lo, mask_hi;

      auto regnum = scalar_operand_to_regnum (is_cbranch_i_fork (instruction)
                                                ? sdst_operand (instruction)
                                                : ssrc0_operand (instruction));

      /* The hardware requires a 64-bit address register pair to have the lower
         register number be even.  */
      dbgapi_assert (regnum && !(*regnum & 1));

      wave.read_register (*regnum + 0, &mask_lo);
      wave.read_register (*regnum + 1, &mask_hi);

      uint64_t mask, exec, mask_pass, mask_fail;
      mask = (static_cast<uint64_t> (mask_hi) << 32) | mask_lo;

      wave.read_register (amdgpu_regnum_t::exec_64, &exec);

      mask_pass = mask & exec;
      mask_fail = ~mask & exec;

      if (mask_pass == exec)
        return true;

      if (mask_fail == exec)
        return false;

      return utils::bit_count (mask_fail) >= utils::bit_count (mask_pass);
    }

  if (is_cbranch_join (instruction))
    {
      uint32_t csp, mask;

      auto regnum = scalar_operand_to_regnum (ssrc0_operand (instruction));
      dbgapi_assert (regnum);

      wave.read_register (amdgpu_regnum_t::csp, &csp);
      wave.read_register (*regnum, &mask);

      return csp != mask;
    }

  /* Always taken branch.  */
  if (is_branch (instruction) || is_call (instruction)
      || is_setpc (instruction) || is_swappc (instruction))
    return true;

  dbgapi_assert_not_reached ("not a branch instruction");
}

amd_dbgapi_global_address_t
amdgcn_architecture_t::branch_target (wave_t &wave,
                                      amd_dbgapi_global_address_t pc,
                                      const instruction_t &instruction) const
{
  dbgapi_assert (instruction.is_valid ());
  amd_dbgapi_global_address_t target;

  if (is_branch (instruction) || is_call (instruction)
      || is_cbranch (instruction) || is_cbranch_i_fork (instruction))
    {
      target
        = pc + instruction.size ()
          + (static_cast<std::ptrdiff_t> (simm16_operand (instruction)) << 2);
    }
  else if (is_cbranch_g_fork (instruction))
    {
      auto regnum = scalar_operand_to_regnum (ssrc1_operand (instruction));

      /* The hardware requires a 64-bit address register pair to have the lower
         register number be even.  */
      dbgapi_assert (regnum && !(*regnum & 1));

      uint32_t pc_lo, pc_hi;
      wave.read_register (*regnum + 0, &pc_lo);
      wave.read_register (*regnum + 1, &pc_hi);

      target = (static_cast<uint64_t> (pc_hi) << 32) | pc_lo;
    }
  else if (is_setpc (instruction) || is_swappc (instruction))
    {
      auto ssrc_regnum
        = scalar_operand_to_regnum (ssrc0_operand (instruction));

      /* The hardware requires a 64-bit address register pair to have the
         lower register number be even.  */
      dbgapi_assert (ssrc_regnum && !(*ssrc_regnum & 1));

      uint32_t ssrc_lo, ssrc_hi;
      wave.read_register (*ssrc_regnum + 0, &ssrc_lo);
      wave.read_register (*ssrc_regnum + 1, &ssrc_hi);

      target = amd_dbgapi_global_address_t{ ssrc_lo }
               | amd_dbgapi_global_address_t{ ssrc_hi } << 32;
    }
  else if (is_cbranch_join (instruction))
    {
      uint32_t csp;
      wave.read_register (amdgpu_regnum_t::csp, &csp);

      amdgpu_regnum_t regnum = amdgpu_regnum_t::s0 + --csp * 4;

      uint32_t pc_lo, pc_hi;
      wave.read_register (regnum + 2, &pc_lo);
      wave.read_register (regnum + 3, &pc_hi);

      target = (static_cast<uint64_t> (pc_hi) << 32) | pc_lo;
    }
  else
    dbgapi_assert_not_reached ("not a branch instruction");

  return utils::align_down (target, minimum_instruction_alignment ());
}

std::tuple<amd_dbgapi_instruction_kind_t, amd_dbgapi_instruction_properties_t,
           size_t, std::vector<uint64_t>>
amdgcn_architecture_t::classify_instruction (
  amd_dbgapi_global_address_t address, const instruction_t &instruction) const
{
  enum class information_kind_t
  {
    none = 0,
    pc_direct,
    pc_indirect,
    uint8,
  } information_kind;

  amd_dbgapi_instruction_kind_t instruction_kind;
  amd_dbgapi_instruction_properties_t instruction_properties
    = AMD_DBGAPI_INSTRUCTION_PROPERTY_NONE;

  std::optional<amdgpu_regnum_t> ssrc_regnum, sdst_regnum;

  if (is_branch (instruction))
    {
      instruction_kind = AMD_DBGAPI_INSTRUCTION_KIND_DIRECT_BRANCH;
      information_kind = information_kind_t::pc_direct;
    }
  else if (is_cbranch (instruction) || is_cbranch_i_fork (instruction))
    {
      instruction_kind = AMD_DBGAPI_INSTRUCTION_KIND_DIRECT_BRANCH_CONDITIONAL;
      information_kind = information_kind_t::pc_direct;
    }
  else if (is_cbranch_g_fork (instruction))
    {
      instruction_kind
        = AMD_DBGAPI_INSTRUCTION_KIND_INDIRECT_BRANCH_CONDITIONAL_REGISTER_PAIR;
      information_kind = information_kind_t::pc_indirect;
      ssrc_regnum = scalar_operand_to_regnum (ssrc1_operand (instruction));
    }
  else if (is_cbranch_join (instruction))
    {
      instruction_kind = AMD_DBGAPI_INSTRUCTION_KIND_SPECIAL;
      information_kind = information_kind_t::none;
    }
  else if (is_setpc (instruction))
    {
      instruction_kind
        = AMD_DBGAPI_INSTRUCTION_KIND_INDIRECT_BRANCH_REGISTER_PAIR;
      information_kind = information_kind_t::pc_indirect;
      ssrc_regnum = scalar_operand_to_regnum (ssrc0_operand (instruction));
    }
  else if (is_call (instruction))
    {
      instruction_kind = AMD_DBGAPI_INSTRUCTION_KIND_DIRECT_CALL_REGISTER_PAIR;
      information_kind = information_kind_t::pc_direct;
      sdst_regnum = scalar_operand_to_regnum (sdst_operand (instruction));
    }
  else if (is_swappc (instruction))
    {
      instruction_kind
        = AMD_DBGAPI_INSTRUCTION_KIND_INDIRECT_CALL_REGISTER_PAIRS;
      information_kind = information_kind_t::pc_indirect;
      ssrc_regnum = scalar_operand_to_regnum (ssrc0_operand (instruction));
      sdst_regnum = scalar_operand_to_regnum (sdst_operand (instruction));
    }
  else if (is_endpgm (instruction))
    {
      instruction_kind = AMD_DBGAPI_INSTRUCTION_KIND_TERMINATE;
      information_kind = information_kind_t::none;
    }
  else if (is_trap (instruction))
    {
      instruction_kind = AMD_DBGAPI_INSTRUCTION_KIND_TRAP;
      information_kind = information_kind_t::uint8;
    }
  else if (is_sethalt (instruction) && (simm16_operand (instruction) & 0x1))
    {
      instruction_kind = AMD_DBGAPI_INSTRUCTION_KIND_HALT;
      information_kind = information_kind_t::none;
    }
  else if (is_barrier (instruction))
    {
      instruction_kind = AMD_DBGAPI_INSTRUCTION_KIND_BARRIER;
      information_kind = information_kind_t::none;
    }
  else if (is_sleep (instruction))
    {
      instruction_kind = AMD_DBGAPI_INSTRUCTION_KIND_SLEEP;
      information_kind = information_kind_t::none;
    }
  else
    {
      instruction_kind = is_sequential (instruction)
                           ? AMD_DBGAPI_INSTRUCTION_KIND_SEQUENTIAL
                           : AMD_DBGAPI_INSTRUCTION_KIND_UNKNOWN;
      information_kind = information_kind_t::none;
    }

  std::vector<uint64_t> information;

  if (information_kind == information_kind_t::pc_direct)
    {
      information.emplace_back (
        address + instruction.size ()
        + (static_cast<std::ptrdiff_t> (simm16_operand (instruction)) << 2));

      if (sdst_regnum.has_value ())
        {
          information.emplace_back (static_cast<uint64_t> (
            regnum_to_register_id (*sdst_regnum + 0).handle));
          information.emplace_back (static_cast<uint64_t> (
            regnum_to_register_id (*sdst_regnum + 1).handle));
        }
    }
  else if (information_kind == information_kind_t::pc_indirect)
    {
      dbgapi_assert (ssrc_regnum.has_value ());
      information.emplace_back (static_cast<uint64_t> (
        regnum_to_register_id (*ssrc_regnum + 0).handle));
      information.emplace_back (static_cast<uint64_t> (
        regnum_to_register_id (*ssrc_regnum + 1).handle));

      if (sdst_regnum.has_value ())
        {
          information.emplace_back (static_cast<uint64_t> (
            regnum_to_register_id (*sdst_regnum + 0).handle));
          information.emplace_back (static_cast<uint64_t> (
            regnum_to_register_id (*sdst_regnum + 1).handle));
        }
    }
  else if (information_kind == information_kind_t::uint8)
    {
      information.emplace_back (static_cast<uint8_t> (
        utils::bit_extract (simm16_operand (instruction), 0, 7)));
    }

  return { instruction_kind, instruction_properties, instruction.size (),
           std::move (information) };
}

bool
amdgcn_architecture_t::can_execute_displaced (
  wave_t & /* wave  */, const instruction_t &instruction) const
{
  /* Illegal instructions cannot be displaced, their behavior is undefined.  */
  if (!instruction.is_valid ())
    return false;

  /* PC relative branch instructions cannot be displaced as a wave cannot be
     halted with a PC pointing at random or unmapped memory if the branch is
     taken.  */
  if (is_branch (instruction) || is_cbranch (instruction)
      || is_cbranch_i_fork (instruction) || is_call (instruction))
    return false;

  /* All PC reading/modifying instructions are simulated, so no attempt is made
     to fixup the state after instruction is displaced-stepped.  */
  return !(is_cbranch_g_fork (instruction) || is_cbranch_join (instruction)
           || is_getpc (instruction) || is_setpc (instruction)
           || is_swappc (instruction));
}

bool
amdgcn_architecture_t::can_simulate (wave_t & /* wave  */,
                                     const instruction_t &instruction) const
{
  /* The instruction simulation does not handle all possible source operands
     (for example: literals, apertures, vccz, scc, ...), so only simulate
     instructions that have a known register source and/or destination.  */

  if (is_getpc (instruction) || is_call (instruction)
      || is_cbranch_i_fork (instruction))
    return scalar_operand_to_regnum (sdst_operand (instruction)).has_value ();

  if (is_setpc (instruction) || is_cbranch_i_fork (instruction))
    return scalar_operand_to_regnum (ssrc0_operand (instruction)).has_value ();

  if (is_swappc (instruction))
    return scalar_operand_to_regnum (ssrc0_operand (instruction)).has_value ()
           && scalar_operand_to_regnum (sdst_operand (instruction))
                .has_value ();

  return is_branch (instruction) || is_cbranch (instruction)
         || is_cbranch_join (instruction) || is_endpgm (instruction);
}

bool
amdgcn_architecture_t::simulate (wave_t &wave, amd_dbgapi_global_address_t pc,
                                 const instruction_t &instruction) const
{
  dbgapi_assert (wave.state () == AMD_DBGAPI_WAVE_STATE_SINGLE_STEP
                 && "wave must be single-stepping to simulate instructions");
  dbgapi_assert (can_simulate (wave, instruction));

  uint32_t status_reg;
  wave.read_register (amdgpu_regnum_t::status, &status_reg);

  /* Don't simulate the instruction if the wave is halted.  */
  if (status_reg & sq_wave_status_halt_mask)
    return false;

  auto new_pc = simulate_instruction (wave, pc, instruction);
  if (new_pc)
    /* Since we are single-stepping the instruction, simulate entering the trap
       handler with no trap_id.  */
    simulate_trap_handler (wave, *new_pc);

  dbgapi_log (AMD_DBGAPI_LOG_LEVEL_INFO, "%s simulated \"%s\" (pc=%#lx)",
              to_string (wave.id ()).c_str (),
              std::get<std::string> (
                wave.architecture ().disassemble_instruction (pc, instruction))
                .c_str (),
              pc);

  return true;
}

std::optional<amd_dbgapi_global_address_t>
amdgcn_architecture_t::simulate_instruction (
  wave_t &wave, amd_dbgapi_global_address_t pc,
  const instruction_t &instruction) const
{
  dbgapi_assert (utils::is_aligned (pc, minimum_instruction_alignment ()));

  if (is_branch (instruction) || is_cbranch (instruction)
      || is_setpc (instruction))
    {
      /* Only the pc is modified.  */
    }
  else if (is_endpgm (instruction))
    {
      wave.terminate ();
      return std::nullopt;
    }
  else if (is_cbranch_i_fork (instruction) || is_cbranch_g_fork (instruction))
    {
      dbgapi_assert (wave.lane_count () == 64);

      auto mask_regnum = scalar_operand_to_regnum (
        is_cbranch_i_fork (instruction) ? sdst_operand (instruction)
                                        : ssrc0_operand (instruction));
      dbgapi_assert (mask_regnum);

      /* The hardware requires a 64-bit address register pair to have the lower
         register number be even.  */
      dbgapi_assert (mask_regnum && !(*mask_regnum & 1));

      uint32_t mask_lo, mask_hi;
      wave.read_register (*mask_regnum + 0, &mask_lo);
      wave.read_register (*mask_regnum + 1, &mask_hi);

      uint64_t mask, exec, mask_pass, mask_fail;
      mask = (static_cast<uint64_t> (mask_hi) << 32) | mask_lo;
      wave.read_register (amdgpu_regnum_t::exec_64, &exec);

      mask_pass = mask & exec;
      mask_fail = ~mask & exec;

      if (mask_pass != exec && mask_fail != exec)
        {
          bool taken = is_branch_taken (wave, instruction);

          uint64_t saved_pc = taken ? pc + instruction.size ()
                                    : branch_target (wave, pc, instruction);

          uint32_t saved_exec_lo = taken ? mask_fail : mask_pass;
          uint32_t saved_exec_hi = (taken ? mask_fail : mask_pass) >> 32;
          uint32_t saved_pc_lo = saved_pc;
          uint32_t saved_pc_hi = saved_pc >> 32;

          uint32_t csp;
          wave.read_register (amdgpu_regnum_t::csp, &csp);

          amdgpu_regnum_t regnum = amdgpu_regnum_t::s0 + csp++ * 4;
          wave.write_register (regnum + 0, saved_exec_lo);
          wave.write_register (regnum + 1, saved_exec_hi);
          wave.write_register (regnum + 2, saved_pc_lo);
          wave.write_register (regnum + 3, saved_pc_hi);

          wave.write_register (amdgpu_regnum_t::csp, csp);
          wave.write_register (amdgpu_regnum_t::exec_64,
                               taken ? mask_pass : mask_fail);
        }
    }
  else if (is_cbranch_join (instruction))
    {
      dbgapi_assert (wave.lane_count () == 64);

      if (is_branch_taken (wave, instruction))
        {
          uint32_t csp;
          wave.read_register (amdgpu_regnum_t::csp, &csp);

          amdgpu_regnum_t regnum = amdgpu_regnum_t::s0 + --csp * 4;

          uint32_t exec_lo, exec_hi;
          wave.read_register (regnum + 0, &exec_lo);
          wave.read_register (regnum + 1, &exec_hi);
          uint64_t exec = (static_cast<uint64_t> (exec_hi) << 32) | exec_lo;

          wave.write_register (amdgpu_regnum_t::csp, csp);
          wave.write_register (amdgpu_regnum_t::exec_64, exec);
        }
    }
  else if (is_call (instruction) || is_getpc (instruction)
           || is_swappc (instruction))
    {
      auto sdst_regnum = scalar_operand_to_regnum (sdst_operand (instruction));

      /* The hardware requires a 64-bit address register pair to have the
         lower register number be even.  */
      dbgapi_assert (sdst_regnum && !(*sdst_regnum & 1));

      uint64_t sdst_value = pc + instruction.size ();
      uint32_t sdst_lo = static_cast<uint32_t> (sdst_value);
      uint32_t sdst_hi = static_cast<uint32_t> (sdst_value >> 32);

      wave.write_register (*sdst_regnum + 0, sdst_lo);
      wave.write_register (*sdst_regnum + 1, sdst_hi);
    }
  else
    {
      /* We don't know how to simulate this instruction.  */
      dbgapi_assert_not_reached ("cannot simulate instruction");
    }

  amd_dbgapi_global_address_t new_pc
    = (is_sequential (instruction) || !is_branch_taken (wave, instruction))
        ? pc + instruction.size ()
        : branch_target (wave, pc, instruction);

  return new_pc;
}

void
amdgcn_architecture_t::simulate_trap_handler (
  wave_t &wave, amd_dbgapi_global_address_t pc,
  std::optional<trap_id_t> trap_id) const
{
  dbgapi_assert (utils::is_aligned (pc, minimum_instruction_alignment ()));

  uint32_t status_reg, ttmp6;
  wave.read_register (amdgpu_regnum_t::status, &status_reg);
  wave.read_register (amdgpu_regnum_t::ttmp6, &ttmp6);

  /* Set ttmp6.wave_stopped and save status.halt and trap_id[3:0].  */
  ttmp6 &= ~(ttmp6_saved_status_halt_mask | ttmp6_saved_trap_id_mask);

  ttmp6 |= ttmp6_wave_stopped_mask;

  if (trap_id)
    ttmp6 |= (static_cast<uint32_t> (*trap_id) << ttmp6_saved_trap_id_shift)
             & ttmp6_saved_trap_id_mask;

  if (status_reg & sq_wave_status_halt_mask)
    ttmp6 |= ttmp6_saved_status_halt_mask;

  wave.write_register (amdgpu_regnum_t::ttmp6, ttmp6);

  /* Park the wave.  */
  if (park_stopped_waves ())
    {
      uint32_t ttmp7, ttmp11;

      /* The trap handler saves PC[31:0] in ttmp7[31:0] ...  */
      ttmp7 = utils::bit_extract (pc, 0, 31);
      wave.write_register (amdgpu_regnum_t::ttmp7, ttmp7);

      /* ... and PC[47:32] in ttmp11[22:7].  */
      wave.read_register (amdgpu_regnum_t::ttmp11, &ttmp11);
      ttmp11 &= ~utils::bit_mask (7, 22);
      ttmp11 |= (utils::bit_extract (pc, 32, 47) << 7);
      wave.write_register (amdgpu_regnum_t::ttmp11, ttmp11);

      pc = wave.queue ().park_instruction_address ();
    }

  wave.write_register (amdgpu_regnum_t::pc, pc);

  /* Then halt the wave.  */
  status_reg |= sq_wave_status_halt_mask;
  wave.write_register (amdgpu_regnum_t::status, status_reg);
};

std::pair<amd_dbgapi_wave_state_t, amd_dbgapi_wave_stop_reasons_t>
amdgcn_architecture_t::wave_get_state (wave_t &wave) const
{
  uint32_t ttmp6;
  wave.read_register (amdgpu_regnum_t::ttmp6, &ttmp6);
  const bool is_stopped = (ttmp6 & ttmp6_wave_stopped_mask) != 0;

  if (!is_stopped)
    /* The wave is still running, nothing to do.  */
    return { wave.state (), AMD_DBGAPI_WAVE_STOP_REASON_NONE };

  if (wave.state () == AMD_DBGAPI_WAVE_STATE_STOP)
    /* The wave is still is stopped, the stop reason is unchanged.  */
    return { AMD_DBGAPI_WAVE_STATE_STOP, wave.stop_reason () };

  /* The wave was previously running, and it is now stopped after executing the
     trap handler.  Unpark the wave if it was parked by the trap handler and
     fill the stop_reason with the exceptions that have caused the wave to
     enter the trap handler.  */

  if (park_stopped_waves ())
    {
      /* The trap handler "parked" the wave and saved the PC in ttmp11[22:7]
         and ttmp7[31:0].  */

      uint32_t ttmp7, ttmp11;
      wave.read_register (amdgpu_regnum_t::ttmp7, &ttmp7);
      wave.read_register (amdgpu_regnum_t::ttmp11, &ttmp11);

      amd_dbgapi_global_address_t pc
        = static_cast<amd_dbgapi_global_address_t> (ttmp7)
          | static_cast<amd_dbgapi_global_address_t> (
              utils::bit_extract (ttmp11, 7, 22))
              << 32;
      wave.write_register (amdgpu_regnum_t::pc, pc);
    }

  amd_dbgapi_wave_stop_reasons_t stop_reason
    = AMD_DBGAPI_WAVE_STOP_REASON_NONE;

  uint32_t trapsts, mode_reg;
  wave.read_register (amdgpu_regnum_t::trapsts, &trapsts);
  wave.read_register (amdgpu_regnum_t::mode, &mode_reg);

  /* Check for exceptions.  Maskable exceptions may be mis-reported if
     trapsts.excp[x] is not cleared when mode.excp_en[x] is set.  */
  if (trapsts & sq_wave_trapsts_excp_invalid_mask
      && mode_reg & sq_wave_mode_excp_en_invalid_mask)
    stop_reason |= AMD_DBGAPI_WAVE_STOP_REASON_FP_INVALID_OPERATION;
  if (trapsts & sq_wave_trapsts_excp_input_denorm_mask
      && mode_reg & sq_wave_mode_excp_en_input_denorm_mask)
    stop_reason |= AMD_DBGAPI_WAVE_STOP_REASON_FP_INPUT_DENORMAL;
  if (trapsts & sq_wave_trapsts_excp_div0_mask
      && mode_reg & sq_wave_mode_excp_en_div0_mask)
    stop_reason |= AMD_DBGAPI_WAVE_STOP_REASON_FP_DIVIDE_BY_0;
  if (trapsts & sq_wave_trapsts_excp_overflow_mask
      && mode_reg & sq_wave_mode_excp_en_overflow_mask)
    stop_reason |= AMD_DBGAPI_WAVE_STOP_REASON_FP_OVERFLOW;
  if (trapsts & sq_wave_trapsts_excp_underflow_mask
      && mode_reg & sq_wave_mode_excp_en_underflow_mask)
    stop_reason |= AMD_DBGAPI_WAVE_STOP_REASON_FP_UNDERFLOW;
  if (trapsts & sq_wave_trapsts_excp_inexact_mask
      && mode_reg & sq_wave_mode_excp_en_inexact_mask)
    stop_reason |= AMD_DBGAPI_WAVE_STOP_REASON_FP_INEXACT;
  if (trapsts & sq_wave_trapsts_excp_int_div0_mask
      && mode_reg & sq_wave_mode_excp_en_int_div0_mask)
    stop_reason |= AMD_DBGAPI_WAVE_STOP_REASON_INT_DIVIDE_BY_0;
  if (trapsts & sq_wave_trapsts_xnack_error_mask)
    stop_reason |= AMD_DBGAPI_WAVE_STOP_REASON_MEMORY_VIOLATION;
  else if (trapsts & sq_wave_trapsts_excp_mem_viol_mask)
    stop_reason |= AMD_DBGAPI_WAVE_STOP_REASON_APERTURE_VIOLATION;
  if (trapsts & sq_wave_trapsts_illegal_inst_mask)
    stop_reason |= AMD_DBGAPI_WAVE_STOP_REASON_ILLEGAL_INSTRUCTION;
  if (trapsts
        & (sq_wave_trapsts_excp_addr_watch0_mask
           | sq_wave_trapsts_excp_hi_addr_watch1_mask
           | sq_wave_trapsts_excp_hi_addr_watch2_mask
           | sq_wave_trapsts_excp_hi_addr_watch3_mask)
      && mode_reg & sq_wave_mode_excp_en_addr_watch_mask)
    stop_reason |= AMD_DBGAPI_WAVE_STOP_REASON_WATCHPOINT;

  /* Check for traps caused by an s_trap instruction.  */
  if (auto trap_id = ttmp6_saved_trap_id (ttmp6); trap_id)
    {
      switch (*trap_id)
        {
        case trap_id_t::assert_trap:
          stop_reason |= AMD_DBGAPI_WAVE_STOP_REASON_ASSERT_TRAP;
          break;
        case trap_id_t::debug_trap:
          stop_reason |= AMD_DBGAPI_WAVE_STOP_REASON_DEBUG_TRAP;
          break;
        case trap_id_t::breakpoint:
          stop_reason |= AMD_DBGAPI_WAVE_STOP_REASON_BREAKPOINT;
          break;
        default:
          stop_reason |= AMD_DBGAPI_WAVE_STOP_REASON_TRAP;
          break;
        }

      /* For all trap IDs except trap_id_t::breakpoint, check that the PC
         points past an actual trap instruction.  We can't check
         trap_id_t::breakpoint because the debugger may have removed the
         breakpoint and restored the original instruction.  */
      if (*trap_id != trap_id_t::breakpoint && ![&] () {
            auto instruction
              = wave.instruction_at_pc (-breakpoint_instruction_pc_adjust ());
            return instruction && is_trap (*instruction);
          }())
        fatal_error ("trap exception not raised by a trap instruction");
    }

  return { AMD_DBGAPI_WAVE_STATE_STOP, stop_reason };
}

void
amdgcn_architecture_t::wave_set_state (
  wave_t &wave, amd_dbgapi_wave_state_t state,
  amd_dbgapi_exceptions_t exceptions) const
{
  dbgapi_assert ((exceptions == AMD_DBGAPI_EXCEPTION_NONE
                  || state != AMD_DBGAPI_WAVE_STATE_STOP)
                 && "raising an exception requires the wave to be resumed");

  uint32_t status_reg, mode_reg, ttmp6;

  wave.read_register (amdgpu_regnum_t::status, &status_reg);
  wave.read_register (amdgpu_regnum_t::mode, &mode_reg);
  wave.read_register (amdgpu_regnum_t::ttmp6, &ttmp6);

  if (state != AMD_DBGAPI_WAVE_STATE_STOP
      && exceptions != AMD_DBGAPI_EXCEPTION_NONE)
    /* Halt the wave if resuming with exceptions.  */
    wave_set_halt (wave, true);

  switch (state)
    {
    case AMD_DBGAPI_WAVE_STATE_STOP:
      /* Put the wave in the stop state (ttmp6.wave_stopped=1), save
         status.halt in ttmp6.saved_status_halt, and halt the wave
         (status.halt=1).  */
      ttmp6 &= ~(ttmp6_wave_stopped_mask | ttmp6_saved_status_halt_mask);

      if (status_reg & sq_wave_status_halt_mask)
        ttmp6 |= ttmp6_saved_status_halt_mask;
      ttmp6 |= ttmp6_wave_stopped_mask;

      status_reg |= sq_wave_status_halt_mask;
      break;

    case AMD_DBGAPI_WAVE_STATE_RUN:
      /* Restore status.halt from ttmp6.saved_status_halt, put the wave in the
         run state (ttmp6.wave_stopped=0), and set mode.debug_en=0.  */
      status_reg &= ~sq_wave_status_halt_mask;
      if (ttmp6 & ttmp6_saved_status_halt_mask)
        status_reg |= sq_wave_status_halt_mask;

      ttmp6 &= ~(ttmp6_wave_stopped_mask | ttmp6_saved_status_halt_mask);

      mode_reg &= ~sq_wave_mode_debug_en_mask;
      break;

    case AMD_DBGAPI_WAVE_STATE_SINGLE_STEP:
      /* Restore status.halt from ttmp6.saved_status_halt, put the wave in the
         run state (ttmp6.wave_stopped=0), and set mode.debug_en=1.  */
      status_reg &= ~sq_wave_status_halt_mask;
      if (ttmp6 & ttmp6_saved_status_halt_mask)
        status_reg |= sq_wave_status_halt_mask;

      ttmp6 &= ~(ttmp6_wave_stopped_mask | ttmp6_saved_status_halt_mask);

      mode_reg |= sq_wave_mode_debug_en_mask;
      break;

    default:
      dbgapi_assert_not_reached ("Invalid wave state");
    }

  wave.write_register (amdgpu_regnum_t::status, status_reg);
  wave.write_register (amdgpu_regnum_t::mode, mode_reg);
  wave.write_register (amdgpu_regnum_t::ttmp6, ttmp6);

  /* When resuming a wave, clear the exceptions in the trapsts register that
     have already been reported by a stop event (stop_reason != 0).  */
  if (state != AMD_DBGAPI_WAVE_STATE_STOP
      && wave.state () == AMD_DBGAPI_WAVE_STATE_STOP
      && wave.stop_reason () != AMD_DBGAPI_WAVE_STOP_REASON_NONE)
    {
      amd_dbgapi_wave_stop_reasons_t stop_reason = wave.stop_reason ();
      uint32_t clear_exceptions = 0;

      if (stop_reason & AMD_DBGAPI_WAVE_STOP_REASON_MEMORY_VIOLATION)
        clear_exceptions |= sq_wave_trapsts_excp_mem_viol_mask
                            | sq_wave_trapsts_xnack_error_mask;
      if (stop_reason & AMD_DBGAPI_WAVE_STOP_REASON_APERTURE_VIOLATION)
        clear_exceptions |= sq_wave_trapsts_excp_mem_viol_mask;
      if (stop_reason & AMD_DBGAPI_WAVE_STOP_REASON_ILLEGAL_INSTRUCTION)
        clear_exceptions |= sq_wave_trapsts_illegal_inst_mask;
      if (stop_reason & AMD_DBGAPI_WAVE_STOP_REASON_FP_INVALID_OPERATION)
        clear_exceptions |= sq_wave_trapsts_excp_invalid_mask;
      if (stop_reason & AMD_DBGAPI_WAVE_STOP_REASON_FP_INPUT_DENORMAL)
        clear_exceptions |= sq_wave_trapsts_excp_input_denorm_mask;
      if (stop_reason & AMD_DBGAPI_WAVE_STOP_REASON_FP_DIVIDE_BY_0)
        clear_exceptions |= sq_wave_trapsts_excp_div0_mask;
      if (stop_reason & AMD_DBGAPI_WAVE_STOP_REASON_FP_OVERFLOW)
        clear_exceptions |= sq_wave_trapsts_excp_overflow_mask;
      if (stop_reason & AMD_DBGAPI_WAVE_STOP_REASON_FP_UNDERFLOW)
        clear_exceptions |= sq_wave_trapsts_excp_underflow_mask;
      if (stop_reason & AMD_DBGAPI_WAVE_STOP_REASON_FP_INEXACT)
        clear_exceptions |= sq_wave_trapsts_excp_inexact_mask;
      if (stop_reason & AMD_DBGAPI_WAVE_STOP_REASON_INT_DIVIDE_BY_0)
        clear_exceptions |= sq_wave_trapsts_excp_int_div0_mask;
      if (stop_reason & AMD_DBGAPI_WAVE_STOP_REASON_WATCHPOINT)
        clear_exceptions |= sq_wave_trapsts_excp_addr_watch0_mask
                            | sq_wave_trapsts_excp_hi_addr_watch1_mask
                            | sq_wave_trapsts_excp_hi_addr_watch2_mask
                            | sq_wave_trapsts_excp_hi_addr_watch3_mask;

      if (clear_exceptions)
        {
          uint32_t trapsts;
          wave.read_register (amdgpu_regnum_t::trapsts, &trapsts);
          trapsts &= ~clear_exceptions;
          wave.write_register (amdgpu_regnum_t::trapsts, trapsts);
        }
    }
}

bool
amdgcn_architecture_t::wave_get_halt (const wave_t &wave) const
{
  uint32_t ttmp6;
  wave.read_register (amdgpu_regnum_t::ttmp6, &ttmp6);

  /* If the wave is stopped, status.halt is saved in ttmp6.  */
  if (ttmp6 & ttmp6_wave_stopped_mask)
    return ttmp6 & ttmp6_saved_status_halt_mask;

  uint32_t status_reg;
  wave.read_register (amdgpu_regnum_t::status, &status_reg);
  return status_reg & sq_wave_status_halt_mask;
}

void
amdgcn_architecture_t::wave_set_halt (wave_t &wave, bool halt) const
{
  uint32_t ttmp6;
  wave.read_register (amdgpu_regnum_t::ttmp6, &ttmp6);

  /* When the wave is stopped by the trap handler, status.halt is saved in
     ttmp6 so that it can be restored when the wave is resumed.  */
  if (ttmp6 & ttmp6_wave_stopped_mask)
    {
      ttmp6 = halt ? ttmp6 | ttmp6_saved_status_halt_mask
                   : ttmp6 & ~ttmp6_saved_status_halt_mask;

      wave.write_register (amdgpu_regnum_t::ttmp6, ttmp6);
      return;
    }

  uint32_t status_reg;
  wave.read_register (amdgpu_regnum_t::status, &status_reg);

  status_reg = halt ? status_reg | sq_wave_status_halt_mask
                    : status_reg & ~sq_wave_status_halt_mask;

  wave.write_register (amdgpu_regnum_t::status, status_reg);
}

/* Convert an os_wave_launch_trap_mask to a bit mask that can be or'ed in the
   SQ_WAVE_MODE register.  */
uint32_t
amdgcn_architecture_t::os_wave_launch_trap_mask_to_wave_mode (
  os_wave_launch_trap_mask_t mask) const
{
  uint32_t mode{ 0 };

  if (!!(mask & os_wave_launch_trap_mask_t::fp_invalid))
    mode |= sq_wave_mode_excp_en_invalid_mask;
  if (!!(mask & os_wave_launch_trap_mask_t::fp_input_denormal))
    mode |= sq_wave_mode_excp_en_input_denorm_mask;
  if (!!(mask & os_wave_launch_trap_mask_t::fp_divide_by_zero))
    mode |= sq_wave_mode_excp_en_div0_mask;
  if (!!(mask & os_wave_launch_trap_mask_t::fp_overflow))
    mode |= sq_wave_mode_excp_en_overflow_mask;
  if (!!(mask & os_wave_launch_trap_mask_t::fp_underflow))
    mode |= sq_wave_mode_excp_en_underflow_mask;
  if (!!(mask & os_wave_launch_trap_mask_t::fp_inexact))
    mode |= sq_wave_mode_excp_en_inexact_mask;
  if (!!(mask & os_wave_launch_trap_mask_t::int_divide_by_zero))
    mode |= sq_wave_mode_excp_en_int_div0_mask;
  if (!!(mask & os_wave_launch_trap_mask_t::address_watch))
    mode |= sq_wave_mode_excp_en_addr_watch_mask;

  return mode;
}

void
amdgcn_architecture_t::wave_enable_traps (
  wave_t &wave, os_wave_launch_trap_mask_t mask) const
{
  uint32_t mode;
  wave.read_register (amdgpu_regnum_t::mode, &mode);

  /* OR SQ_WAVE_MODE.EXCP_EN with mask.  */
  mode |= os_wave_launch_trap_mask_to_wave_mode (mask);

  wave.write_register (amdgpu_regnum_t::mode, mode);
}

void
amdgcn_architecture_t::wave_disable_traps (
  wave_t &wave, os_wave_launch_trap_mask_t mask) const
{
  uint32_t mode;

  wave.read_register (amdgpu_regnum_t::mode, &mode);

  /* AND SQ_WAVE_MODE.EXCP_EN with ~mask.  */
  mode &= ~os_wave_launch_trap_mask_to_wave_mode (mask);

  wave.write_register (amdgpu_regnum_t::mode, mode);
}

uint8_t
amdgcn_architecture_t::ssrc0_operand (const instruction_t &instruction)
{
  return utils::bit_extract (instruction.word<0> (), 0, 7);
}

uint8_t
amdgcn_architecture_t::ssrc1_operand (const instruction_t &instruction)
{
  return utils::bit_extract (instruction.word<0> (), 8, 15);
}

uint8_t
amdgcn_architecture_t::sdst_operand (const instruction_t &instruction)
{
  return utils::bit_extract (instruction.word<0> (), 16, 22);
}

int16_t
amdgcn_architecture_t::simm16_operand (const instruction_t &instruction)
{
  return utils::bit_extract (instruction.word<0> (), 0, 15);
}

uint8_t
amdgcn_architecture_t::encoding_op7 (const instruction_t &instruction)
{
  return utils::bit_extract (instruction.word<0> (), 16, 22);
}

template <int... op5>
bool
amdgcn_architecture_t::is_sopk_encoding (const instruction_t &instruction)
{
  static_assert (((utils::bit_extract (op5, 0, 4) == op5) && ...),
                 "opcode is wider than 5 bits");

  /* The instruction_t must have at least one word.  */
  if (instruction.capacity () < sizeof (instruction.word<0> ()))
    return false;

  /* SOPK [1011 OP5 SDST7 SIMM16]  */
  return (((instruction.word<0> () & 0xFF800000)
           == (0xB0000000 | (op5 & 0x1F) << 23))
          || ...);
}

template <int... op8>
bool
amdgcn_architecture_t::is_sop1_encoding (const instruction_t &instruction)
{
  static_assert (((utils::bit_extract (op8, 0, 7) == op8) && ...),
                 "opcode is wider than 8 bits");

  /* The instruction_t must have at least one word.  */
  if (instruction.capacity () < sizeof (instruction.word<0> ()))
    return false;

  /* SOP1 [10111110 1 SDST7 OP8 SSRC08]  */
  return (
    ((instruction.word<0> () & 0xFF80FF00) == (0xBE800000 | (op8 & 0xFF) << 8))
    || ...);
}

template <int... op7>
bool
amdgcn_architecture_t::is_sop2_encoding (const instruction_t &instruction)
{
  static_assert (((utils::bit_extract (op7, 0, 6) == op7) && ...),
                 "opcode is wider than 7 bits");

  /* The instruction_t must have at least one word.  */
  if (instruction.capacity () < sizeof (instruction.word<0> ()))
    return false;

  /* SOP2 [10 OP7 SDST7 SSRC18 SSRC08]  */
  return (((instruction.word<0> () & 0xFF800000)
           == (0x80000000 | (op7 & 0x7F) << 23))
          || ...);
}

template <int... op7>
bool
amdgcn_architecture_t::is_sopp_encoding (const instruction_t &instruction)
{
  static_assert (((utils::bit_extract (op7, 0, 6) == op7) && ...),
                 "opcode is wider than 7 bits");

  /* The instruction_t must have at least one word.  */
  if (instruction.capacity () < sizeof (instruction.word<0> ()))
    return false;

  /* SOPP [101111111 OP7 SIMM16]  */
  return (((instruction.word<0> () & 0xFFFF0000)
           == (0xBF800000 | (op7 & 0x7F) << 16))
          || ...);
}

std::string
amdgcn_architecture_t::register_name (amdgpu_regnum_t regnum) const
{
  if (regnum >= amdgpu_regnum_t::first_sgpr
      && regnum < amdgpu_regnum_t::last_sgpr)
    {
      return string_printf ("s%ld", regnum - amdgpu_regnum_t::first_sgpr);
    }
  if (regnum >= amdgpu_regnum_t::first_vgpr_64
      && regnum <= amdgpu_regnum_t::last_vgpr_64)
    {
      return string_printf ("v%ld", regnum - amdgpu_regnum_t::first_vgpr_64);
    }
  if (regnum >= amdgpu_regnum_t::first_ttmp
      && regnum <= amdgpu_regnum_t::last_ttmp)
    {
      switch (regnum)
        {
        case amdgpu_regnum_t::ttmp4:
        case amdgpu_regnum_t::ttmp5:
        case amdgpu_regnum_t::ttmp6:
        case amdgpu_regnum_t::ttmp7:
        case amdgpu_regnum_t::ttmp8:
        case amdgpu_regnum_t::ttmp9:
        case amdgpu_regnum_t::ttmp10:
        case amdgpu_regnum_t::ttmp11:
        case amdgpu_regnum_t::ttmp13:
          return string_printf ("ttmp%ld",
                                regnum - amdgpu_regnum_t::first_ttmp);
        default:
          break;
        }
    }
  if (regnum >= amdgpu_regnum_t::first_hwreg
      && regnum <= amdgpu_regnum_t::last_hwreg)
    {
      return string_printf ("hwreg%ld", regnum - amdgpu_regnum_t::first_hwreg);
    }

  if (regnum == amdgpu_regnum_t::exec_64
      || regnum == amdgpu_regnum_t::pseudo_exec_64)
    {
      return "exec";
    }
  if (regnum == amdgpu_regnum_t::vcc_64
      || regnum == amdgpu_regnum_t::pseudo_vcc_64)
    {
      return "vcc";
    }
  if (regnum == amdgpu_regnum_t::xnack_mask_64)
    {
      return "xnack_mask";
    }

  switch (regnum)
    {
    case amdgpu_regnum_t::pc:
      return "pc";
    case amdgpu_regnum_t::m0:
      return "m0";
    case amdgpu_regnum_t::pseudo_status:
    case amdgpu_regnum_t::status:
      return "status";
    case amdgpu_regnum_t::trapsts:
      return "trapsts";
    case amdgpu_regnum_t::mode:
      return "mode";
    case amdgpu_regnum_t::flat_scratch_lo:
      return "flat_scratch_lo";
    case amdgpu_regnum_t::flat_scratch_hi:
      return "flat_scratch_hi";
    case amdgpu_regnum_t::exec_lo:;
      return "exec_lo";
    case amdgpu_regnum_t::exec_hi:;
      return "exec_hi";
    case amdgpu_regnum_t::vcc_lo:;
      return "vcc_lo";
    case amdgpu_regnum_t::vcc_hi:;
      return "vcc_hi";
    case amdgpu_regnum_t::xnack_mask_lo:;
      return "xnack_mask_lo";
    case amdgpu_regnum_t::xnack_mask_hi:;
      return "xnack_mask_hi";
    case amdgpu_regnum_t::flat_scratch:
      return "flat_scratch";
    case amdgpu_regnum_t::wave_id:
      return "wave_id";
    case amdgpu_regnum_t::dispatch_grid:
      return "dispatch_grid";
    case amdgpu_regnum_t::wave_in_group:
      return "wave_in_group";
    case amdgpu_regnum_t::csp:
      return "csp";
    case amdgpu_regnum_t::null:
      return "null";
    default:
      break;
    }
  dbgapi_assert_not_reached ("invalid register number");
}

std::string
amdgcn_architecture_t::register_type (amdgpu_regnum_t regnum) const
{
  /* Vector registers.  */
  if (regnum >= amdgpu_regnum_t::first_vgpr_64
      && regnum <= amdgpu_regnum_t::last_vgpr_64)
    {
      return "int32_t[64]";
    }
  /* Scalar registers.  */
  if (regnum >= amdgpu_regnum_t::first_sgpr
      && regnum < amdgpu_regnum_t::last_sgpr)
    {
      return "int32_t";
    }
  /* hwregs, ttmps.  */
  if ((regnum >= amdgpu_regnum_t::first_hwreg
       && regnum <= amdgpu_regnum_t::last_hwreg)
      || (regnum >= amdgpu_regnum_t::first_ttmp
          && regnum <= amdgpu_regnum_t::last_ttmp))
    {
      return "uint32_t";
    }
  if (regnum == amdgpu_regnum_t::exec_64
      || regnum == amdgpu_regnum_t::pseudo_exec_64
      || regnum == amdgpu_regnum_t::vcc_64
      || regnum == amdgpu_regnum_t::pseudo_vcc_64
      || regnum == amdgpu_regnum_t::xnack_mask_64)
    {
      return "uint64_t";
    }
  switch (regnum)
    {
    case amdgpu_regnum_t::pc:
      return "void (*)()";

    case amdgpu_regnum_t::status:
    case amdgpu_regnum_t::mode:
    case amdgpu_regnum_t::trapsts:
    case amdgpu_regnum_t::m0:
    case amdgpu_regnum_t::flat_scratch_lo:
    case amdgpu_regnum_t::flat_scratch_hi:
    case amdgpu_regnum_t::exec_lo:
    case amdgpu_regnum_t::exec_hi:
    case amdgpu_regnum_t::vcc_lo:
    case amdgpu_regnum_t::vcc_hi:
    case amdgpu_regnum_t::xnack_mask_lo:
    case amdgpu_regnum_t::xnack_mask_hi:
    case amdgpu_regnum_t::pseudo_status:
    case amdgpu_regnum_t::wave_in_group:
    case amdgpu_regnum_t::csp:
    case amdgpu_regnum_t::null:
      return "uint32_t";

    case amdgpu_regnum_t::wave_id:
    case amdgpu_regnum_t::flat_scratch:
      return "uint64_t";

    case amdgpu_regnum_t::dispatch_grid:
      return "uint32_t[3]";

    default:
      dbgapi_assert_not_reached ("invalid register number");
    }
}

amd_dbgapi_size_t
amdgcn_architecture_t::register_size (amdgpu_regnum_t regnum) const
{
  /* Vector registers.  */
  if (regnum >= amdgpu_regnum_t::first_vgpr_64
      && regnum <= amdgpu_regnum_t::last_vgpr_64)
    {
      return sizeof (int32_t) * 64;
    }
  /* Scalar registers.  */
  if (regnum >= amdgpu_regnum_t::first_sgpr
      && regnum < amdgpu_regnum_t::last_sgpr)
    {
      return sizeof (int32_t);
    }
  /* hwregs, ttmps.  */
  if ((regnum >= amdgpu_regnum_t::first_hwreg
       && regnum <= amdgpu_regnum_t::last_hwreg)
      || (regnum >= amdgpu_regnum_t::first_ttmp
          && regnum <= amdgpu_regnum_t::last_ttmp))
    {
      return sizeof (uint32_t);
    }
  if (regnum == amdgpu_regnum_t::exec_64
      || regnum == amdgpu_regnum_t::pseudo_exec_64
      || regnum == amdgpu_regnum_t::vcc_64
      || regnum == amdgpu_regnum_t::pseudo_vcc_64
      || regnum == amdgpu_regnum_t::xnack_mask_64)
    {
      return sizeof (uint64_t);
    }
  switch (regnum)
    {
    case amdgpu_regnum_t::pc:
      return sizeof (void (*) ());

    case amdgpu_regnum_t::m0:
    case amdgpu_regnum_t::status:
    case amdgpu_regnum_t::trapsts:
    case amdgpu_regnum_t::mode:
    case amdgpu_regnum_t::flat_scratch_lo:
    case amdgpu_regnum_t::flat_scratch_hi:
    case amdgpu_regnum_t::exec_lo:
    case amdgpu_regnum_t::exec_hi:
    case amdgpu_regnum_t::vcc_lo:
    case amdgpu_regnum_t::vcc_hi:
    case amdgpu_regnum_t::xnack_mask_lo:
    case amdgpu_regnum_t::xnack_mask_hi:
    case amdgpu_regnum_t::pseudo_status:
    case amdgpu_regnum_t::wave_in_group:
    case amdgpu_regnum_t::csp:
    case amdgpu_regnum_t::null:
      return sizeof (uint32_t);

    case amdgpu_regnum_t::wave_id:
    case amdgpu_regnum_t::flat_scratch:
      return sizeof (uint64_t);

    case amdgpu_regnum_t::dispatch_grid:
      return sizeof (uint32_t[3]);

    default:
      dbgapi_assert_not_reached ("invalid register number");
    }
}

const void *
amdgcn_architecture_t::register_read_only_mask (amdgpu_regnum_t regnum) const
{
  switch (regnum)
    {
    case amdgpu_regnum_t::trapsts:
      static uint32_t trapsts_read_only_bits
        = utils::bit_mask (9, 9) /* 0  */ | utils::bit_mask (15, 15) /* 0  */
          | utils::bit_mask (22, 27) /* 0  */;
      return &trapsts_read_only_bits;

    case amdgpu_regnum_t::mode:
      static uint32_t mode_read_only_bits = utils::bit_mask (21, 22); /* 0 */
      return &mode_read_only_bits;

    case amdgpu_regnum_t::pseudo_status:
      static uint32_t status_read_only_bits
        = utils::bit_mask (5, 7)      /* priv, trap_en, ttrace_en  */
          | utils::bit_mask (9, 12)   /* execz, vccz, in_tg, in_barrier  */
          | utils::bit_mask (14, 16)  /* trap, ttrace_cu_en, valid  */
          | utils::bit_mask (19, 19)  /* perf_en  */
          | utils::bit_mask (22, 26)  /* allow_replay, fatal_halt, 0  */
          | utils::bit_mask (28, 31); /* 0  */
      return &status_read_only_bits;

    case amdgpu_regnum_t::pc:
      static uint64_t pc_read_only_bits = utils::bit_mask (0, 1); /* 0  */
      return &pc_read_only_bits;

    default:
      return nullptr;
    }
}

amd_dbgapi_register_properties_t
amdgcn_architecture_t::register_properties (amdgpu_regnum_t regnum) const
{
  amd_dbgapi_register_properties_t properties
    = register_read_only_mask (regnum) != nullptr
        ? AMD_DBGAPI_REGISTER_PROPERTY_READONLY_BITS
        : AMD_DBGAPI_REGISTER_PROPERTY_NONE;

  /* Writing to the vcc register may change the status.vccz bit.  */
  if (regnum == amdgpu_regnum_t::pseudo_status)
    properties |= AMD_DBGAPI_REGISTER_PROPERTY_VOLATILE;

  /* Writing to the exec or vcc register may change the status.execz
     status.vccz bits respectively.  */
  if (regnum == amdgpu_regnum_t::pseudo_exec_64
      || regnum == amdgpu_regnum_t::pseudo_vcc_64)
    properties |= AMD_DBGAPI_REGISTER_PROPERTY_INVALIDATE_VOLATILE;

  return properties;
}

bool
amdgcn_architecture_t::is_pseudo_register_available (
  const wave_t & /* wave  */, amdgpu_regnum_t regnum) const
{
  dbgapi_assert (is_pseudo_register (regnum));

  switch (regnum)
    {
    case amdgpu_regnum_t::pseudo_status:
    case amdgpu_regnum_t::pseudo_exec_64:
    case amdgpu_regnum_t::pseudo_vcc_64:
    case amdgpu_regnum_t::wave_id:
    case amdgpu_regnum_t::wave_in_group:
    case amdgpu_regnum_t::csp:
    case amdgpu_regnum_t::null:
      return true;
    default:
      return false;
    }
}

void
amdgcn_architecture_t::read_pseudo_register (const wave_t &wave,
                                             amdgpu_regnum_t regnum,
                                             size_t offset, size_t value_size,
                                             void *value) const
{
  dbgapi_assert (is_pseudo_register (regnum)
                 && is_pseudo_register_available (wave, regnum));

  dbgapi_assert (value_size && (offset + value_size) <= register_size (regnum)
                 && "read_pseudo_register is out of bounds");

  if (regnum == amdgpu_regnum_t::null)
    {
      memset (value, '\0', value_size);
      return;
    }

  if (regnum == amdgpu_regnum_t::pseudo_exec_64
      || regnum == amdgpu_regnum_t::pseudo_vcc_64)
    {
      dbgapi_assert (wave.lane_count () == 64);
      wave.read_register (regnum == amdgpu_regnum_t::pseudo_exec_64
                            ? amdgpu_regnum_t::exec_64
                            : amdgpu_regnum_t::vcc_64,
                          offset, value_size, value);
      return;
    }

  if (regnum == amdgpu_regnum_t::pseudo_status)
    {
      /* pseudo_status is a composite of: sq_wave_status[31:14], ttmp6[29]
         (halt), sq_wave_status [12:6], 0[0] (priv), sq_wave_status [4:0].  */

      uint32_t ttmp6, status_reg;

      wave.read_register (amdgpu_regnum_t::status, &status_reg);
      wave.read_register (amdgpu_regnum_t::ttmp6, &ttmp6);

      status_reg &= ~(sq_wave_status_priv_mask | sq_wave_status_halt_mask);
      if (ttmp6 & ttmp6_saved_status_halt_mask)
        status_reg |= sq_wave_status_halt_mask;

      memcpy (value, reinterpret_cast<const char *> (&status_reg) + offset,
              value_size);
      return;
    }

  if (regnum == amdgpu_regnum_t::wave_in_group)
    {
      uint32_t ttmp11;

      wave.read_register (amdgpu_regnum_t::ttmp11, &ttmp11);
      ttmp11 &= ttmp11_wave_in_group_mask;

      memcpy (value, reinterpret_cast<const char *> (&ttmp11) + offset,
              value_size);
      return;
    }

  if (regnum == amdgpu_regnum_t::wave_id)
    {
      std::array<uint32_t, 2> wave_id;

      wave.read_register (amdgpu_regnum_t::ttmp4, &wave_id[0]);
      wave.read_register (amdgpu_regnum_t::ttmp5, &wave_id[1]);

      memcpy (value, reinterpret_cast<const char *> (wave_id.data ()) + offset,
              value_size);
      return;
    }

  if (regnum == amdgpu_regnum_t::csp)
    {
      uint32_t mode;

      wave.read_register (amdgpu_regnum_t::mode, &mode);

      uint32_t csp = utils::bit_extract (mode, 29, 31);

      memcpy (value, reinterpret_cast<const char *> (&csp) + offset,
              value_size);
      return;
    }

  dbgapi_assert_not_reached ("Unhandled pseudo register");
}

void
amdgcn_architecture_t::write_pseudo_register (wave_t &wave,
                                              amdgpu_regnum_t regnum,
                                              size_t offset, size_t value_size,
                                              const void *value) const
{
  dbgapi_assert (is_pseudo_register (regnum)
                 && is_pseudo_register_available (wave, regnum));

  dbgapi_assert (value_size && (offset + value_size) <= register_size (regnum)
                 && "write_pseudo_register is out of bounds");

  if (regnum == amdgpu_regnum_t::null
      || regnum == amdgpu_regnum_t::wave_in_group)
    /* Writing to null or wave_in_group is a no-op.  */
    return;

  if (regnum == amdgpu_regnum_t::pseudo_exec_64
      || regnum == amdgpu_regnum_t::pseudo_vcc_64)
    {
      dbgapi_assert (wave.lane_count () == 64);
      uint64_t base_reg;
      uint32_t status_reg;

      amdgpu_regnum_t base_regnum = regnum == amdgpu_regnum_t::pseudo_exec_64
                                      ? amdgpu_regnum_t::exec_64
                                      : amdgpu_regnum_t::vcc_64;
      uint32_t status_mask = regnum == amdgpu_regnum_t::pseudo_exec_64
                               ? sq_wave_status_execz_mask
                               : sq_wave_status_vccz_mask;

      wave.read_register (amdgpu_regnum_t::status, &status_reg);
      wave.read_register (base_regnum, &base_reg);

      memcpy (reinterpret_cast<char *> (&base_reg) + offset, value,
              value_size);

      status_reg
        = (status_reg & ~status_mask) | (base_reg == 0 ? status_mask : 0);

      wave.write_register (amdgpu_regnum_t::status, status_reg);
      wave.write_register (base_regnum, base_reg);
      return;
    }

  if (regnum == amdgpu_regnum_t::pseudo_status)
    {
      /* pseudo_status is a composite of: status[31:14], ttmp6[29] (halt),
         status [12:6], 0[0] (priv), status [4:0].  */

      uint32_t status_reg, ttmp6;
      wave.read_register (amdgpu_regnum_t::status, &status_reg);
      wave.read_register (amdgpu_regnum_t::ttmp6, &ttmp6);

      memcpy (reinterpret_cast<char *> (&status_reg) + offset, value,
              value_size);

      ttmp6 &= ~ttmp6_saved_status_halt_mask;
      if (status_reg & sq_wave_status_halt_mask)
        ttmp6 |= ttmp6_saved_status_halt_mask;

      wave.write_register (amdgpu_regnum_t::status, status_reg);
      wave.write_register (amdgpu_regnum_t::ttmp6, ttmp6);
      return;
    }

  if (regnum == amdgpu_regnum_t::wave_id)
    {
      std::array<uint32_t, 2> wave_id;

      if (offset != 0 || value_size != sizeof (wave_id))
        {
          wave.read_register (amdgpu_regnum_t::ttmp4, &wave_id[0]);
          wave.read_register (amdgpu_regnum_t::ttmp5, &wave_id[1]);
        }

      memcpy (reinterpret_cast<char *> (wave_id.data ()) + offset, value,
              value_size);

      wave.write_register (amdgpu_regnum_t::ttmp4, wave_id[0]);
      wave.write_register (amdgpu_regnum_t::ttmp5, wave_id[1]);
      return;
    }

  if (regnum == amdgpu_regnum_t::csp)
    {
      uint32_t mode, csp;

      wave.read_register (amdgpu_regnum_t::mode, &mode);

      csp = utils::bit_extract (mode, 29, 31);
      memcpy (reinterpret_cast<char *> (&csp) + offset, value, value_size);

      mode = (mode & ~utils::bit_mask (29, 31)) | (csp << 29);

      wave.write_register (amdgpu_regnum_t::mode, mode);
      return;
    }

  dbgapi_assert_not_reached ("Unhandled pseudo register");
}

/* Base class for all GFX9 architectures.  */

class gfx9_architecture_t : public amdgcn_architecture_t
{
private:
  static const std::unordered_map<uint16_t, cbranch_cond_t>
    cbranch_opcodes_map;

protected:
  class cwsr_record_t : public amdgcn_architecture_t::cwsr_record_t
  {
  protected:
    uint32_t const m_compute_relaunch_wave;
    uint32_t const m_compute_relaunch_state;
    amd_dbgapi_global_address_t const m_context_save_address;

    static constexpr uint32_t
    compute_relaunch_state_payload_vgprs (uint32_t relaunch_state)
    {
      return utils::bit_extract (relaunch_state, 0, 5);
    }
    static constexpr uint32_t
    compute_relaunch_state_payload_sgprs (uint32_t relaunch_state)
    {
      return utils::bit_extract (relaunch_state, 6, 8);
    }
    static constexpr uint32_t
    compute_relaunch_state_payload_lds_size (uint32_t relaunch_state)
    {
      return utils::bit_extract (relaunch_state, 9, 17);
    }
    static constexpr uint32_t
    compute_relaunch_wave_payload_scratch_scoreboard_id (
      uint32_t relaunch_wave)
    {
      return utils::bit_extract (relaunch_wave, 0, 8);
    }
    static constexpr uint32_t
    compute_relaunch_wave_payload_se_id (uint32_t relaunch_wave)
    {
      return utils::bit_extract (relaunch_wave, 11, 12);
    }
    static constexpr uint32_t
    compute_relaunch_wave_payload_scratch_en (uint32_t relaunch_wave)
    {
      return utils::bit_extract (relaunch_wave, 15, 15);
    }
    static constexpr uint32_t
    compute_relaunch_wave_payload_last_wave (uint32_t relaunch_wave)
    {
      return utils::bit_extract (relaunch_wave, 16, 16);
    }
    static constexpr uint32_t
    compute_relaunch_wave_payload_first_wave (uint32_t relaunch_wave)
    {
      return utils::bit_extract (relaunch_wave, 17, 17);
    }

  public:
    cwsr_record_t (compute_queue_t &queue, uint32_t compute_relaunch_wave,
                   uint32_t compute_relaunch_state,
                   amd_dbgapi_global_address_t context_save_address)
      : amdgcn_architecture_t::cwsr_record_t (queue),
        m_compute_relaunch_wave (compute_relaunch_wave),
        m_compute_relaunch_state (compute_relaunch_state),
        m_context_save_address (context_save_address)
    {
    }

    std::optional<amd_dbgapi_wave_id_t> id () const override;

    /* Number for vector registers.  */
    size_t vgpr_count () const override;
    /* Number of scalar registers.  */
    size_t sgpr_count () const override;

    /* Return true is a scratch slot is allocated for this record.  */
    bool is_scratch_enabled () const override;

    /* The shader engine ID this wave was created on.  */
    uint32_t shader_engine_id () const override;
    /* Scratch region slot ID.  */
    uint32_t scratch_scoreboard_id () const override;

    /* Number of work-items in one wave.  */
    size_t lane_count () const override { return 64; };
    bool is_last_wave () const override;
    bool is_first_wave () const override;

    size_t lds_size () const override;

    amd_dbgapi_global_address_t begin () const override
    {
      return register_address (lane_count () == 32 ? amdgpu_regnum_t::v0_32
                                                   : amdgpu_regnum_t::v0_64)
        .value ();
    }
    amd_dbgapi_global_address_t end () const override
    {
      return m_context_save_address;
    }

    std::optional<amd_dbgapi_global_address_t>
    register_address (amdgpu_regnum_t regnum) const override;
  };

  virtual std::unique_ptr<architecture_t::cwsr_record_t>
  make_gfx9_cwsr_record (
    compute_queue_t &queue, uint32_t compute_relaunch_wave,
    uint32_t compute_relaunch_state,
    amd_dbgapi_global_address_t context_save_address) const
  {
    return std::make_unique<cwsr_record_t> (queue, compute_relaunch_wave,
                                            compute_relaunch_state,
                                            context_save_address);
  }

  std::optional<amdgpu_regnum_t>
  scalar_operand_to_regnum (int operand, bool priv = false) const override;
  size_t scalar_register_count () const override { return 102; }
  size_t scalar_alias_count () const override { return 6; }

  std::pair<amd_dbgapi_wave_state_t, amd_dbgapi_wave_stop_reasons_t>
  wave_get_state (wave_t &wave) const override;

  gfx9_architecture_t (elf_amdgpu_machine_t e_machine,
                       std::string target_triple);

public:
  std::string register_type (amdgpu_regnum_t regnum) const override;

  cbranch_cond_t
  cbranch_condition_code (const instruction_t &instruction) const override;

  bool is_sethalt (const instruction_t &instruction) const override;
  bool is_barrier (const instruction_t &instruction) const override;
  bool is_sleep (const instruction_t &instruction) const override;
  bool is_call (const instruction_t &instruction) const override;
  bool is_getpc (const instruction_t &instruction) const override;
  bool is_setpc (const instruction_t &instruction) const override;
  bool is_swappc (const instruction_t &instruction) const override;
  bool is_branch (const instruction_t &instruction) const override;
  bool is_cbranch (const instruction_t &instruction) const override;
  bool is_cbranch_i_fork (const instruction_t &instruction) const override;
  bool is_cbranch_g_fork (const instruction_t &instruction) const override;
  bool is_cbranch_join (const instruction_t &instruction) const override;
  bool is_trap (const instruction_t &instruction,
                trap_id_t *trap_id = nullptr) const override;
  bool is_endpgm (const instruction_t &instruction) const override;
  bool is_sequential (const instruction_t &instruction) const override;

  bool can_halt_at_endpgm () const override { return false; }
  size_t largest_instruction_size () const override { return 8; }

  size_t control_stack_iterate (
    compute_queue_t &queue, const uint32_t *control_stack,
    size_t control_stack_words, amd_dbgapi_global_address_t wave_area_address,
    amd_dbgapi_size_t wave_area_size,
    const std::function<void (std::unique_ptr<architecture_t::cwsr_record_t>)>
      &wave_callback) const override;

  std::optional<amd_dbgapi_global_address_t> dispatch_packet_address (
    const architecture_t::cwsr_record_t &cwsr_record) const override;

  std::pair<amd_dbgapi_size_t /* offset  */, amd_dbgapi_size_t /* size  */>
  scratch_memory_region (uint32_t compute_tmpring_size_register,
                         uint32_t bank_count, uint32_t bank_id,
                         uint32_t slot_id) const override;
};

gfx9_architecture_t::gfx9_architecture_t (elf_amdgpu_machine_t e_machine,
                                          std::string target_triple)
  : amdgcn_architecture_t (e_machine, std::move (target_triple))
{
  /* Create address spaces.  */

  auto &as_generic = create<address_space_t> (
    "generic", address_space_t::kind_t::generic, DW_ASPACE_AMDGPU_generic, 64,
    /* generic NULL is the same as global NULL  */
    address_space_t::s_global.null_address (),
    AMD_DBGAPI_ADDRESS_SPACE_ACCESS_ALL);

  auto &as_region = create<address_space_t> (
    "region", address_space_t::kind_t::region, DW_ASPACE_AMDGPU_region, 32,
    0xFFFFFFFF, AMD_DBGAPI_ADDRESS_SPACE_ACCESS_ALL);

  auto &as_local = create<address_space_t> (
    "local", address_space_t::kind_t::local, DW_ASPACE_AMDGPU_local, 32,
    0xFFFFFFFF, AMD_DBGAPI_ADDRESS_SPACE_ACCESS_ALL);

  auto &as_private_lane = create<address_space_t> (
    "private_lane", address_space_t::kind_t::private_swizzled,
    DW_ASPACE_AMDGPU_private_lane, 32, 0x00000000,
    AMD_DBGAPI_ADDRESS_SPACE_ACCESS_ALL);

  create<address_space_t> ("private_wave",
                           address_space_t::kind_t::private_unswizzled,
                           DW_ASPACE_AMDGPU_private_wave, 32, 0x00000000,
                           AMD_DBGAPI_ADDRESS_SPACE_ACCESS_ALL);

  /* Create address classes.  */

  create<address_class_t> ("none", DW_ADDR_none, as_generic);
  create<address_class_t> ("global", DW_ADDR_LLVM_global,
                           address_space_t::s_global);
  create<address_class_t> ("constant", DW_ADDR_LLVM_constant,
                           address_space_t::s_global);
  create<address_class_t> ("group", DW_ADDR_LLVM_group, as_local);
  create<address_class_t> ("private", DW_ADDR_LLVM_private, as_private_lane);
  create<address_class_t> ("region", DW_ADDR_AMDGPU_region, as_region);

  /* Create register classes.  */

  /* Scalar registers: [s0-s102].  */
  auto &scalar_registers = create<register_class_t> (*this, "scalar");
  scalar_registers.add_registers (
    amdgpu_regnum_t::first_sgpr,
    amdgpu_regnum_t::first_sgpr + gfx9_architecture_t::scalar_register_count ()
      - 1);

  /* Vector registers: [v0-v255]  */
  auto &vector_registers = create<register_class_t> (*this, "vector");
  vector_registers.add_registers (amdgpu_regnum_t::first_vgpr_64,
                                  amdgpu_regnum_t::last_vgpr_64);

  /* Trap temporary registers: [ttmp4-ttmp11, ttmp13]  */
  auto &trap_registers = create<register_class_t> (*this, "trap");
  trap_registers.add_registers (amdgpu_regnum_t::ttmp4,
                                amdgpu_regnum_t::ttmp11);
  trap_registers.add_registers (amdgpu_regnum_t::ttmp13,
                                amdgpu_regnum_t::ttmp13);

  /* System registers: [hwregs, flat_scratch, xnack_mask, vcc]  */
  auto &system_registers = create<register_class_t> (*this, "system");
  system_registers.add_registers (amdgpu_regnum_t::pseudo_status,
                                  amdgpu_regnum_t::pseudo_status);
  system_registers.add_registers (amdgpu_regnum_t::mode,
                                  amdgpu_regnum_t::mode);
  system_registers.add_registers (amdgpu_regnum_t::trapsts,
                                  amdgpu_regnum_t::trapsts);
  system_registers.add_registers (amdgpu_regnum_t::flat_scratch,
                                  amdgpu_regnum_t::flat_scratch);
  system_registers.add_registers (amdgpu_regnum_t::xnack_mask_64,
                                  amdgpu_regnum_t::xnack_mask_64);

  /* General registers: [{scalar}, {vector}, pc, exec, vcc]  */
  auto &general_registers = create<register_class_t> (*this, "general");
  general_registers.add_registers (
    amdgpu_regnum_t::first_sgpr,
    amdgpu_regnum_t::first_sgpr + gfx9_architecture_t::scalar_register_count ()
      - 1);
  general_registers.add_registers (amdgpu_regnum_t::first_vgpr_64,
                                   amdgpu_regnum_t::last_vgpr_64);
  general_registers.add_registers (amdgpu_regnum_t::m0, amdgpu_regnum_t::m0);
  general_registers.add_registers (amdgpu_regnum_t::pc, amdgpu_regnum_t::pc);
  general_registers.add_registers (amdgpu_regnum_t::pseudo_exec_64,
                                   amdgpu_regnum_t::pseudo_exec_64);
  general_registers.add_registers (amdgpu_regnum_t::pseudo_vcc_64,
                                   amdgpu_regnum_t::pseudo_vcc_64);
}

std::pair<amd_dbgapi_wave_state_t, amd_dbgapi_wave_stop_reasons_t>
gfx9_architecture_t::wave_get_state (wave_t &wave) const
{
  amd_dbgapi_wave_state_t prev_state = wave.state ();
  auto [new_state, stop_reason] = amdgcn_architecture_t::wave_get_state (wave);

  if (prev_state != AMD_DBGAPI_WAVE_STATE_SINGLE_STEP
      || new_state != AMD_DBGAPI_WAVE_STATE_STOP)
    return { new_state, stop_reason };

  /* Check for spurious single-step stop events (if the architecture does not
     support precise single-step exceptions reporting):

     Current architectures do not report single-step exceptions in the trapsts
     register, so there is no way to tell if a single-step exception has
     occurred in the presence of other exceptions (for example, context save).

     To work around this limitation, the 1st level trap handler calls the 2nd
     level trap handler when mode.debug_en == 1  && status.halt == 0, which may
     cause a spurious single-step stop event to be reported.

     To detect spurious events, a wave's last stopped pc is recorded before it
     is resumed so that it is possible to tell if the pc has changed as the
     result of executing the instruction.

     Non-sequential instructions may jump to self, making detecting execution
     difficult, so they are simulated when the wave is resumed instead of
     single-stepped on hardware.

     Instructions with invalid encodings, which may be non-sequential but
     cannot be simulated, may still report the spurious single-step exception
     to avoid infinite loops.
   */

  if (wave.pc () != wave.last_stopped_pc ())
    {
      stop_reason |= AMD_DBGAPI_WAVE_STOP_REASON_SINGLE_STEP;
    }
  else if (stop_reason == AMD_DBGAPI_WAVE_STOP_REASON_NONE)
    /* The only exception present is a single-step exception.  */
    {
      if (auto instruction = wave.instruction_at_pc ();
          /* The instruction is sequential.  */
          instruction && is_sequential (*instruction))
        {
          /* Resume the wave in single-step mode.  */
          wave_set_state (wave, AMD_DBGAPI_WAVE_STATE_SINGLE_STEP);

          dbgapi_log (AMD_DBGAPI_LOG_LEVEL_INFO,
                      "%s (pc=%#lx) ignore spurious single-step",
                      to_string (wave.id ()).c_str (), wave.pc ());

          return { AMD_DBGAPI_WAVE_STATE_SINGLE_STEP,
                   AMD_DBGAPI_WAVE_STOP_REASON_NONE };
        }

      /* The pc is unchanged, the instruction is inaccessible, invalid, or
         non-sequential, and no other exceptions were reported, yet the wave
         has stopped. The best we can do is report a possibly spurious
         single-step exception.  */
      stop_reason |= AMD_DBGAPI_WAVE_STOP_REASON_SINGLE_STEP;
    }

  return { AMD_DBGAPI_WAVE_STATE_STOP, stop_reason };
}

std::optional<amd_dbgapi_wave_id_t>
gfx9_architecture_t::cwsr_record_t::id () const
{
  dbgapi_assert (
    process ().is_flag_set (process_t::flag_t::ttmps_setup_enabled));

  const amd_dbgapi_global_address_t wave_id_address
    = register_address (amdgpu_regnum_t::ttmp4).value ();

  amd_dbgapi_wave_id_t wave_id;
  process ().read_global_memory (wave_id_address, &wave_id);

  if (wave_id == wave_t::undefined)
    return std::nullopt;

  return wave_id;
}

size_t
gfx9_architecture_t::cwsr_record_t::vgpr_count () const
{
  uint32_t vgpr_blocks
    = compute_relaunch_state_payload_vgprs (m_compute_relaunch_state) + 1;
  /* vgprs are allocated in blocks of 4 registers.  */
  return vgpr_blocks * 4;
}

size_t
gfx9_architecture_t::cwsr_record_t::sgpr_count () const
{
  uint32_t sgpr_blocks
    = compute_relaunch_state_payload_sgprs (m_compute_relaunch_state) + 1;
  /* sgprs are allocated in blocks of 16 registers. Subtract the ttmps
     registers from this count, as they will be saved in a different area
     than the sgprs.  */
  return sgpr_blocks * 16 - /* remove the ttmps  */ 16;
}

size_t
gfx9_architecture_t::cwsr_record_t::lds_size () const
{
  return compute_relaunch_state_payload_lds_size (m_compute_relaunch_state)
         * 128 * sizeof (uint32_t);
}

bool
gfx9_architecture_t::cwsr_record_t::is_scratch_enabled () const
{
  return compute_relaunch_wave_payload_scratch_en (m_compute_relaunch_wave);
}

uint32_t
gfx9_architecture_t::cwsr_record_t::shader_engine_id () const
{
  return compute_relaunch_wave_payload_se_id (m_compute_relaunch_wave);
}

uint32_t
gfx9_architecture_t::cwsr_record_t::scratch_scoreboard_id () const
{
  return compute_relaunch_wave_payload_scratch_scoreboard_id (
    m_compute_relaunch_wave);
}

bool
gfx9_architecture_t::cwsr_record_t::is_last_wave () const
{
  return compute_relaunch_wave_payload_last_wave (m_compute_relaunch_wave);
}

bool
gfx9_architecture_t::cwsr_record_t::is_first_wave () const
{
  return compute_relaunch_wave_payload_first_wave (m_compute_relaunch_wave);
}

std::optional<amdgpu_regnum_t>
gfx9_architecture_t::scalar_operand_to_regnum (int operand, bool priv) const
{
  if (operand >= 0 && operand <= 101)
    {
      /* SGPR[0] through SGPR[101]  */
      return amdgpu_regnum_t::s0 + operand;
    }

  if (operand >= 108 && operand <= 123)
    {
      /* TTMP[0] through TTMP[15]  */
      return priv ? amdgpu_regnum_t::first_ttmp + (operand - 108)
                  : amdgpu_regnum_t::null;
    }

  switch (operand)
    {
    case 102:
      return amdgpu_regnum_t::flat_scratch_lo;
    case 103:
      return amdgpu_regnum_t::flat_scratch_hi;
    case 104:
      return amdgpu_regnum_t::xnack_mask_lo;
    case 105:
      return amdgpu_regnum_t::xnack_mask_hi;
    case 106:
      return amdgpu_regnum_t::vcc_lo;
    case 107:
      return amdgpu_regnum_t::vcc_hi;
    case 124:
      return amdgpu_regnum_t::m0;
    case 126:
      return amdgpu_regnum_t::exec_lo;
    case 127:
      return amdgpu_regnum_t::exec_hi;
    default:
      return std::nullopt;
    }
}

std::string
gfx9_architecture_t::register_type (amdgpu_regnum_t regnum) const
{
  switch (regnum)
    {
    case amdgpu_regnum_t::pseudo_status:
      return "flags32_t status {"
             "  bool SCC @0;"
             "  uint32_t SPI_PRIO @1-2;"
             "  uint32_t USER_PRIO @3-4;"
             "  bool PRIV @5;"
             "  bool TRAP_EN @6;"
             "  bool TTRACE_EN @7;"
             "  bool EXPORT_RDY @8;"
             "  bool EXECZ @9;"
             "  bool VCCZ @10;"
             "  bool IN_TG @11;"
             "  bool IN_BARRIER @12;"
             "  bool HALT @13;"
             "  bool TRAP @14;"
             "  bool TTRACE_CU_EN @15;"
             "  bool VALID @16;"
             "  bool ECC_ERR @17;"
             "  bool SKIP_EXPORT @18;"
             "  bool PERF_EN @19;"
             "  bool COND_DBG_USER @20;"
             "  bool COND_DBG_SYS @21;"
             "  bool ALLOW_REPLAY @22;"
             "  bool FATAL_HALT @23;"
             "  bool MUST_EXPORT @27;"
             "}";

    case amdgpu_regnum_t::mode:
      return "flags32_t mode {"
             "  enum fp_round {"
             "    NEAREST_EVEN = 0,"
             "    PLUS_INF  = 1,"
             "    MINUS_INF = 2,"
             "    ZERO      = 3"
             "  } FP_ROUND.32 @0-1;"
             "  enum fp_round FP_ROUND.64_16 @2-3;"
             "  enum fp_denorm {"
             "    FLUSH_SRC_DST = 0,"
             "    FLUSH_DST     = 1,"
             "    FLUSH_SRC     = 2,"
             "    FLUSH_NONE    = 3"
             "  } FP_DENORM.32 @4-5;"
             "  enum fp_denorm FP_DENORM.64_16 @6-7;"
             "  bool DX10_CLAMP @8;"
             "  bool IEEE @9;"
             "  bool LOD_CLAMPED @10;"
             "  bool DEBUG_EN @11;"
             "  bool EXCP_EN.INVALID @12;"
             "  bool EXCP_EN.DENORM @13;"
             "  bool EXCP_EN.DIV0 @14;"
             "  bool EXCP_EN.OVERFLOW @15;"
             "  bool EXCP_EN.UNDERFLOW @16;"
             "  bool EXCP_EN.INEXACT @17;"
             "  bool EXCP_EN.INT_DIV0 @18;"
             "  bool EXCP_EN.ADDR_WATCH @19;"
             "  bool FP16_OVFL @23;"
             "  bool POPS_PACKER0 @24;"
             "  bool POPS_PACKER1 @25;"
             "  bool DISABLE_PERF @26;"
             "  bool GPR_IDX_EN @27;"
             "  bool VSKIP @28;"
             "  uint32_t CSP @29-31;"
             "}";

    case amdgpu_regnum_t::trapsts:
      return "flags32_t trapsts {"
             "  bool EXCP.INVALID @0;"
             "  bool EXCP.DENORM @1;"
             "  bool EXCP.DIV0 @2;"
             "  bool EXCP.OVERFLOW @3;"
             "  bool EXCP.UNDERFLOW @4;"
             "  bool EXCP.INEXACT @5;"
             "  bool EXCP.INT_DIV0 @6;"
             "  bool EXCP.ADDR_WATCH @7;"
             "  bool EXCP.MEM_VIOL @8;"
             "  bool SAVE_CTX @10;"
             "  bool ILLEGAL_INST @11;"
             "  bool EXCP_HI.ADDR_WATCH1 @12;"
             "  bool EXCP_HI.ADDR_WATCH2 @13;"
             "  bool EXCP_HI.ADDR_WATCH3 @14;"
             "  uint32_t EXCP_CYCLE @16-21;"
             "  bool XNACK_ERROR @28;"
             "  enum dp_rate {"
             "    NONE    = 0,"
             "    QUARTER = 1,"
             "    FULL    = 4"
             "  } DP_RATE @29-31;"
             "}";

    default:
      return amdgcn_architecture_t::register_type (regnum);
    }
}

decltype (gfx9_architecture_t::cbranch_opcodes_map)
  gfx9_architecture_t::cbranch_opcodes_map{
    { 4, cbranch_cond_t::scc0 },
    { 5, cbranch_cond_t::scc1 },
    { 6, cbranch_cond_t::vccz },
    { 7, cbranch_cond_t::vccnz },
    { 8, cbranch_cond_t::execz },
    { 9, cbranch_cond_t::execnz },
    { 23, cbranch_cond_t::cdbgsys },
    { 24, cbranch_cond_t::cdbguser },
    { 25, cbranch_cond_t::cdbgsys_or_user },
    { 26, cbranch_cond_t::cdbgsys_and_user },
  };

amdgcn_architecture_t::cbranch_cond_t
gfx9_architecture_t::cbranch_condition_code (
  const instruction_t &instruction) const
{
  dbgapi_assert (is_cbranch (instruction));
  return cbranch_opcodes_map.find (encoding_op7 (instruction))->second;
}

bool
gfx9_architecture_t::is_endpgm (const instruction_t &instruction) const
{
  /* As an optimization, do not call instruction_t::is_valid () for SOPP
     instructions.  The SOPP encoding does not use register operands that need
     to be validated, and any value for the rest of the bits is legal.  */

  /* s_endpgm: SOPP Opcode 1  */
  return is_sopp_encoding<1> (instruction);
}

bool
gfx9_architecture_t::is_trap (const instruction_t &instruction,
                              trap_id_t *trap_id) const
{
  /* s_trap: SOPP Opcode 18. See comment in ::is_endpgm.  */
  if (is_sopp_encoding<18> (instruction))
    {
      if (trap_id != nullptr)
        *trap_id = trap_id_t{ static_cast<std::underlying_type_t<trap_id_t>> (
          utils::bit_extract (simm16_operand (instruction), 0, 7)) };
      return true;
    }
  return false;
}

bool
gfx9_architecture_t::is_sethalt (const instruction_t &instruction) const
{
  /* s_sethalt: SOPP Opcode 13. See comment in ::is_endpgm.  */
  return is_sopp_encoding<13> (instruction);
}

bool
gfx9_architecture_t::is_barrier (const instruction_t &instruction) const
{
  /* s_barrier: SOPP Opcode 10. See comment in ::is_endpgm.  */
  return is_sopp_encoding<10> (instruction);
}

bool
gfx9_architecture_t::is_sleep (const instruction_t &instruction) const
{
  /* s_sleep: SOPP Opcode 14. See comment in ::is_endpgm.  */
  return is_sopp_encoding<14> (instruction);
}

bool
gfx9_architecture_t::is_call (const instruction_t &instruction) const
{
  /* s_call: SOPK Opcode 21  */
  return instruction.is_valid () && is_sopk_encoding<21> (instruction)
         && !(sdst_operand (instruction) & 1);
}

bool
gfx9_architecture_t::is_getpc (const instruction_t &instruction) const
{
  /* s_getpc: SOP1 Opcode 28  */
  return instruction.is_valid () && is_sop1_encoding<28> (instruction)
         && !(sdst_operand (instruction) & 1);
}

bool
gfx9_architecture_t::is_setpc (const instruction_t &instruction) const
{
  /* s_setpc: SOP1 Opcode 29  */
  return instruction.is_valid () && is_sop1_encoding<29> (instruction)
         && !(ssrc0_operand (instruction) & 1);
}

bool
gfx9_architecture_t::is_swappc (const instruction_t &instruction) const
{
  /* s_swappc: SOP1 Opcode 30  */
  return instruction.is_valid () && is_sop1_encoding<30> (instruction)
         && !(ssrc0_operand (instruction) & 1)
         && !(sdst_operand (instruction) & 1);
}

bool
gfx9_architecture_t::is_branch (const instruction_t &instruction) const
{
  /* s_branch: SOPP Opcode 2. See comment in ::is_endpgm.  */
  return is_sopp_encoding<2> (instruction);
}

bool
gfx9_architecture_t::is_cbranch (const instruction_t &instruction) const
{
  /* See comment in ::is_endpgm.  */

  if (instruction.capacity () < sizeof (instruction.word<0> ()))
    return false;

  /* s_cbranch_scc0:             SOPP Opcode 4  [10111111 10000100 SIMM16]
     s_cbranch_scc1:             SOPP Opcode 5  [10111111 10000101 SIMM16]
     s_cbranch_vccz:             SOPP Opcode 6  [10111111 10000110 SIMM16]
     s_cbranch_vccnz:            SOPP Opcode 7  [10111111 10000111 SIMM16]
     s_cbranch_execz:            SOPP Opcode 8  [10111111 10001000 SIMM16]
     s_cbranch_execnz:           SOPP Opcode 9  [10111111 10001001 SIMM16]
     s_cbranch_cdbgsys:          SOPP Opcode 23 [10111111 10010111 SIMM16]
     s_cbranch_cdbguser:         SOPP Opcode 24 [10111111 10011000 SIMM16]
     s_cbranch_cdbgsys_or_user:  SOPP Opcode 25 [10111111 10011001 SIMM16]
     s_cbranch_cdbgsys_and_user: SOPP Opcode 26 [10111111 10011010 SIMM16] */
  if ((instruction.word<0> () & 0xFF800000) != 0xBF800000)
    return false;

  return gfx9_architecture_t::cbranch_opcodes_map.find (
           encoding_op7 (instruction))
         != gfx9_architecture_t::cbranch_opcodes_map.end ();
}

bool
gfx9_architecture_t::is_cbranch_i_fork (const instruction_t &instruction) const
{
  /* s_cbranch_i_fork: SOPK Opcode 16  */
  return instruction.is_valid () && is_sopk_encoding<16> (instruction)
         && !(sdst_operand (instruction) & 1);
}

bool
gfx9_architecture_t::is_cbranch_g_fork (const instruction_t &instruction) const
{
  /* s_cbranch_g_fork: SOP2 Opcode 41  */
  return instruction.is_valid () && is_sop2_encoding<41> (instruction)
         && !(ssrc0_operand (instruction) & 1)
         && !(ssrc1_operand (instruction) & 1);
}

bool
gfx9_architecture_t::is_cbranch_join (const instruction_t &instruction) const
{
  /* s_cbranch_join: SOP1 Opcode 46  */
  return instruction.is_valid () && is_sop1_encoding<46> (instruction);
}

bool
gfx9_architecture_t::is_sequential (const instruction_t &instruction) const
{
  if (!instruction.is_valid ())
    return false;

  return /* s_endpgm/s_branch/s_cbranch  */
    !is_sopp_encoding<1, 2, 4, 5, 6, 7, 8, 9, 23, 24, 25, 26> (instruction)
    /* s_setpc_b64/s_swappc_b64/s_cbranch_join/  */
    && !is_sop1_encoding<29, 30, 46> (instruction)
    /* s_cbranch_g_fork  */
    && !is_sop2_encoding<41> (instruction)
    /* s_cbranch_i_fork/s_call_b64  */
    && !is_sopk_encoding<16, 21> (instruction);
}

std::optional<amd_dbgapi_global_address_t>
gfx9_architecture_t::cwsr_record_t::register_address (
  amdgpu_regnum_t regnum) const
{
  dbgapi_assert (
    dynamic_cast<const gfx9_architecture_t *> (&queue ().architecture ())
    != nullptr);

  const auto &architecture
    = static_cast<const gfx9_architecture_t &> (queue ().architecture ());

  amd_dbgapi_global_address_t save_area_addr = m_context_save_address;

  if (is_first_wave ())
    {
      save_area_addr -= lds_size ();

      if (regnum == amdgpu_regnum_t::lds_0)
        return save_area_addr;
    }

  size_t ttmp_size = sizeof (uint32_t);
  size_t ttmp_count = 16;
  size_t ttmps_addr = save_area_addr - ttmp_count * ttmp_size;

  switch (regnum)
    {
    case amdgpu_regnum_t::dispatch_grid:
      regnum = amdgpu_regnum_t::ttmp8;
      break;
    default:
      break;
    }

  if (regnum >= amdgpu_regnum_t::first_ttmp
      && regnum <= amdgpu_regnum_t::last_ttmp)
    {
      return ttmps_addr + (regnum - amdgpu_regnum_t::first_ttmp) * ttmp_size;
    }

  size_t hwreg_count = 16;
  size_t hwreg_size = sizeof (uint32_t);
  size_t hwregs_addr = ttmps_addr - hwreg_count * hwreg_size;

  /* Rename registers that map to the hwreg block.  */
  switch (regnum)
    {
    case amdgpu_regnum_t::m0:
      regnum = amdgpu_regnum_t::first_hwreg + 0;
      break;
    case amdgpu_regnum_t::pc:
      regnum = amdgpu_regnum_t::first_hwreg + 1;
      break;
    case amdgpu_regnum_t::exec_lo:
    case amdgpu_regnum_t::exec_64:
      regnum = amdgpu_regnum_t::first_hwreg + 3;
      break;
    case amdgpu_regnum_t::exec_hi:
      regnum = amdgpu_regnum_t::first_hwreg + 4;
      break;
    case amdgpu_regnum_t::status:
      regnum = amdgpu_regnum_t::first_hwreg + 5;
      break;
    case amdgpu_regnum_t::trapsts:
      regnum = amdgpu_regnum_t::first_hwreg + 6;
      break;
    case amdgpu_regnum_t::xnack_mask_lo:
    case amdgpu_regnum_t::xnack_mask_64:
      regnum = amdgpu_regnum_t::first_hwreg + 7;
      break;
    case amdgpu_regnum_t::xnack_mask_hi:
      regnum = amdgpu_regnum_t::first_hwreg + 8;
      break;
    case amdgpu_regnum_t::mode:
      regnum = amdgpu_regnum_t::first_hwreg + 9;
      break;
    default:
      break;
    }

  if (regnum >= amdgpu_regnum_t::first_hwreg
      && regnum <= amdgpu_regnum_t::last_hwreg)
    {
      return hwregs_addr
             + (regnum - amdgpu_regnum_t::first_hwreg) * hwreg_size;
    }

  size_t sgpr_count = this->sgpr_count ();
  size_t sgpr_size = sizeof (int32_t);
  size_t sgprs_addr = hwregs_addr - sgpr_count * sgpr_size;

  amdgpu_regnum_t aliased_sgpr_end
    = amdgpu_regnum_t::first_sgpr
      + std::min (architecture.scalar_register_count ()
                    + architecture.scalar_alias_count (),
                  sgpr_count);

  /* Exclude the aliased sgprs.  */
  if (regnum >= (aliased_sgpr_end - architecture.scalar_alias_count ())
      && regnum < aliased_sgpr_end)
    return std::nullopt;

  /* Rename registers that alias to sgprs.  */
  switch (regnum)
    {
    case amdgpu_regnum_t::vcc_64:
    case amdgpu_regnum_t::vcc_lo:
      regnum = aliased_sgpr_end - 2;
      break;
    case amdgpu_regnum_t::vcc_hi:
      regnum = aliased_sgpr_end - 1;
      break;
    case amdgpu_regnum_t::flat_scratch:
    case amdgpu_regnum_t::flat_scratch_lo:
      regnum = aliased_sgpr_end - 6;
      break;
    case amdgpu_regnum_t::flat_scratch_hi:
      regnum = aliased_sgpr_end - 5;
      break;
    default:
      break;
    }

  if (regnum >= amdgpu_regnum_t::first_sgpr && regnum < aliased_sgpr_end)
    {
      return sgprs_addr + (regnum - amdgpu_regnum_t::s0) * sgpr_size;
    }

  size_t vgpr_count = this->vgpr_count ();
  size_t vgpr_size = sizeof (int32_t) * 64;
  size_t vgprs_addr = sgprs_addr - vgpr_count * vgpr_size;

  if (regnum >= amdgpu_regnum_t::first_vgpr_64
      && regnum <= amdgpu_regnum_t::last_vgpr_64
      && ((regnum - amdgpu_regnum_t::v0_64) < vgpr_count))
    {
      return vgprs_addr + (regnum - amdgpu_regnum_t::v0_64) * vgpr_size;
    }

  return std::nullopt;
}

size_t
gfx9_architecture_t::control_stack_iterate (
  compute_queue_t &queue, const uint32_t *control_stack,
  size_t control_stack_words, amd_dbgapi_global_address_t wave_area_address,
  amd_dbgapi_size_t wave_area_size,
  const std::function<void (std::unique_ptr<architecture_t::cwsr_record_t>)>
    &wave_callback) const
{
  size_t wave_count = 0;
  uint32_t state = 0;

  amd_dbgapi_global_address_t last_wave_area = wave_area_address;

  for (size_t i = 2; /* Skip the 2 PM4 packets at the top of the stack.  */
       i < control_stack_words; ++i)
    {
      uint32_t relaunch = control_stack[i];

      if (compute_relaunch_is_event (relaunch))
        {
          /* Skip events.  */
        }
      else if (compute_relaunch_is_state (relaunch))
        {
          state = relaunch;
        }
      else
        {
          auto cwsr_record = make_gfx9_cwsr_record (queue, relaunch, state,
                                                    last_wave_area - 64);

          last_wave_area
            = cwsr_record->register_address (amdgpu_regnum_t::first_vgpr_64)
                .value ();

          wave_callback (std::move (cwsr_record));
          ++wave_count;
        }
    }

  /* After iterating the control stack, we should have consumed all the data in
     the wave save area, and last_wave_area should point to the bottom of the
     wave save area.  */
  if (last_wave_area != (wave_area_address - wave_area_size))
    fatal_error ("Corrupted control stack or wave save area");

  return wave_count;
}

std::optional<amd_dbgapi_global_address_t>
gfx9_architecture_t::dispatch_packet_address (
  const architecture_t::cwsr_record_t &cwsr_record) const
{
  compute_queue_t &queue = cwsr_record.queue ();

  const amd_dbgapi_global_address_t ttmp6_address
    = cwsr_record.register_address (amdgpu_regnum_t::ttmp6).value ();

  uint32_t ttmp6;
  cwsr_record.process ().read_global_memory (ttmp6_address, &ttmp6);

  uint64_t dispatch_packet_index
    = (ttmp6 & ttmp6_queue_packet_id_mask) >> ttmp6_queue_packet_id_shift;

  if ((dispatch_packet_index * queue.packet_size ()) >= queue.size ())
    {
      /* The dispatch_packet_index is out of bounds.  */
      warning ("dispatch_packet_index %#lx is out of bounds in %s",
               dispatch_packet_index, to_string (queue.id ()).c_str ());
      return std::nullopt;
    }

  return queue.address () + (dispatch_packet_index * queue.packet_size ());
}

std::pair<amd_dbgapi_size_t /* offset  */, amd_dbgapi_size_t /* size  */>
gfx9_architecture_t::scratch_memory_region (
  uint32_t compute_tmpring_size_register, uint32_t bank_count,
  uint32_t bank_id, uint32_t slot_id) const
{
  amd_dbgapi_size_t scratch_slot_count
    = utils::bit_extract (compute_tmpring_size_register, 0, 11);
  amd_dbgapi_size_t scratch_slot_size
    = utils::bit_extract (compute_tmpring_size_register, 12, 24) * 1024;

  dbgapi_assert (bank_count != 0);

  return { scratch_slot_size
             * ((scratch_slot_count / bank_count) * bank_id + slot_id),
           scratch_slot_size };
}

/* Vega10 Architecture.  */

class gfx900_t final : public gfx9_architecture_t
{
public:
  gfx900_t ()
    : gfx9_architecture_t (EF_AMDGPU_MACH_AMDGCN_GFX900,
                           "amdgcn-amd-amdhsa--gfx900")
  {
  }
};

/* Vega20 Architecture.  */

class gfx906_t final : public gfx9_architecture_t
{
public:
  gfx906_t ()
    : gfx9_architecture_t (EF_AMDGPU_MACH_AMDGCN_GFX906,
                           "amdgcn-amd-amdhsa--gfx906")
  {
  }
};

/* MI Architecture.  */

class mi_architecture_t : public gfx9_architecture_t
{
protected:
  class cwsr_record_t : public gfx9_architecture_t::cwsr_record_t
  {
  public:
    cwsr_record_t (compute_queue_t &queue, uint32_t compute_relaunch_wave,
                   uint32_t compute_relaunch_state,
                   amd_dbgapi_global_address_t context_save_address)
      : gfx9_architecture_t::cwsr_record_t (queue, compute_relaunch_wave,
                                            compute_relaunch_state,
                                            context_save_address)
    {
    }

    virtual size_t acc_vgpr_count () const = 0;

    std::optional<amd_dbgapi_global_address_t>
    register_address (amdgpu_regnum_t regnum) const override;
  };

  std::unique_ptr<architecture_t::cwsr_record_t> make_gfx9_cwsr_record (
    compute_queue_t &queue, uint32_t compute_relaunch_wave,
    uint32_t compute_relaunch_state,
    amd_dbgapi_global_address_t context_save_address) const override = 0;

  mi_architecture_t (elf_amdgpu_machine_t e_machine,
                     std::string target_triple);

public:
  std::string register_name (amdgpu_regnum_t regnum) const override;
  std::string register_type (amdgpu_regnum_t regnum) const override;
  amd_dbgapi_size_t register_size (amdgpu_regnum_t regnum) const override;
};

mi_architecture_t::mi_architecture_t (elf_amdgpu_machine_t e_machine,
                                      std::string target_triple)
  : gfx9_architecture_t (e_machine, std::move (target_triple))
{
  /* Vector registers: [a0-a255]  */
  register_class_t *vector_registers
    = find_if ([] (const register_class_t &register_class)
               { return register_class.name () == "vector"; });
  dbgapi_assert (vector_registers != nullptr);

  vector_registers->add_registers (amdgpu_regnum_t::first_accvgpr_64,
                                   amdgpu_regnum_t::last_accvgpr_64);

  /* General registers: [a0-a255]  */
  register_class_t *general_registers
    = find_if ([] (const register_class_t &register_class)
               { return register_class.name () == "general"; });
  dbgapi_assert (general_registers != nullptr);

  general_registers->add_registers (amdgpu_regnum_t::first_accvgpr_64,
                                    amdgpu_regnum_t::last_accvgpr_64);
}

std::string
mi_architecture_t::register_name (amdgpu_regnum_t regnum) const
{
  if (regnum >= amdgpu_regnum_t::first_accvgpr_64
      && regnum <= amdgpu_regnum_t::last_accvgpr_64)
    {
      return string_printf ("a%ld",
                            regnum - amdgpu_regnum_t::first_accvgpr_64);
    }

  return gfx9_architecture_t::register_name (regnum);
}

std::string
mi_architecture_t::register_type (amdgpu_regnum_t regnum) const
{
  if (regnum >= amdgpu_regnum_t::first_accvgpr_64
      && regnum <= amdgpu_regnum_t::last_accvgpr_64)
    {
      return "int32_t[64]";
    }

  return gfx9_architecture_t::register_type (regnum);
}

amd_dbgapi_size_t
mi_architecture_t::register_size (amdgpu_regnum_t regnum) const
{
  if (regnum >= amdgpu_regnum_t::first_accvgpr_64
      && regnum <= amdgpu_regnum_t::last_accvgpr_64)
    {
      return sizeof (int32_t) * 64;
    }

  return gfx9_architecture_t::register_size (regnum);
}

std::optional<amd_dbgapi_global_address_t>
mi_architecture_t::cwsr_record_t::register_address (
  amdgpu_regnum_t regnum) const
{
  /* Delegate to the gfx9 base for all registers except for the vgprs.  */
  if (regnum < amdgpu_regnum_t::first_vgpr
      || regnum > amdgpu_regnum_t::last_vgpr)
    return gfx9_architecture_t::cwsr_record_t::register_address (regnum);

  auto first_sgpr_addr = gfx9_architecture_t::cwsr_record_t::register_address (
    amdgpu_regnum_t::first_sgpr);
  dbgapi_assert (first_sgpr_addr);

  size_t sgprs_addr = *first_sgpr_addr;

  size_t accvgpr_count = this->acc_vgpr_count ();
  size_t accvgpr_size = sizeof (int32_t) * 64;
  size_t accvgprs_addr = sgprs_addr - accvgpr_count * accvgpr_size;

  if (regnum >= amdgpu_regnum_t::first_accvgpr_64
      && regnum <= amdgpu_regnum_t::last_accvgpr_64
      && ((regnum - amdgpu_regnum_t::a0_64) < accvgpr_count))
    {
      return accvgprs_addr + (regnum - amdgpu_regnum_t::a0_64) * accvgpr_size;
    }

  size_t vgpr_count = this->vgpr_count ();
  size_t vgpr_size = sizeof (int32_t) * 64;
  size_t vgprs_addr = accvgprs_addr - vgpr_count * vgpr_size;

  if (regnum >= amdgpu_regnum_t::first_vgpr_64
      && regnum <= amdgpu_regnum_t::last_vgpr_64
      && ((regnum - amdgpu_regnum_t::v0_64) < vgpr_count))
    {
      return vgprs_addr + (regnum - amdgpu_regnum_t::v0_64) * vgpr_size;
    }

  return std::nullopt;
}

/* Arcturus Architecture.  */

class gfx908_t final : public mi_architecture_t
{
  class cwsr_record_t final : public mi_architecture_t::cwsr_record_t
  {
  protected:
    static constexpr uint32_t
    compute_relaunch_wave_payload_se_id (uint32_t relaunch_wave)
    {
      return utils::bit_extract (relaunch_wave, 11, 13);
    }

  public:
    cwsr_record_t (compute_queue_t &queue, uint32_t compute_relaunch_wave,
                   uint32_t compute_relaunch_state,
                   amd_dbgapi_global_address_t context_save_address)
      : mi_architecture_t::cwsr_record_t (queue, compute_relaunch_wave,
                                          compute_relaunch_state,
                                          context_save_address)
    {
    }

    size_t acc_vgpr_count () const override { return vgpr_count (); }

    uint32_t shader_engine_id () const override
    {
      return compute_relaunch_wave_payload_se_id (m_compute_relaunch_wave);
    }
  };

  std::unique_ptr<architecture_t::cwsr_record_t> make_gfx9_cwsr_record (
    compute_queue_t &queue, uint32_t compute_relaunch_wave,
    uint32_t compute_relaunch_state,
    amd_dbgapi_global_address_t context_save_address) const override
  {
    return std::make_unique<cwsr_record_t> (queue, compute_relaunch_wave,
                                            compute_relaunch_state,
                                            context_save_address);
  }

public:
  gfx908_t ()
    : mi_architecture_t (EF_AMDGPU_MACH_AMDGCN_GFX908,
                         "amdgcn-amd-amdhsa--gfx908")
  {
  }
};

class gfx90a_t final : public mi_architecture_t
{
protected:
  class cwsr_record_t final : public mi_architecture_t::cwsr_record_t
  {
  private:
  protected:
    static constexpr uint32_t
    compute_relaunch_wave_payload_se_id (uint32_t relaunch_wave)
    {
      return utils::bit_extract (relaunch_wave, 9, 11);
    }
    static constexpr uint32_t
    compute_relaunch_state_payload_lds_size (uint32_t relaunch_state)
    {
      return utils::bit_extract (relaunch_state, 9, 16);
    }
    static constexpr uint32_t
    compute_relaunch_state_payload_accum_offset (uint32_t relaunch_state)
    {
      return utils::bit_extract (relaunch_state, 24, 29);
    }

  public:
    cwsr_record_t (compute_queue_t &queue, uint32_t compute_relaunch_wave,
                   uint32_t compute_relaunch_state,
                   amd_dbgapi_global_address_t context_save_address)
      : mi_architecture_t::cwsr_record_t (queue, compute_relaunch_wave,
                                          compute_relaunch_state,
                                          context_save_address)
    {
    }

    size_t vgpr_count () const override;
    size_t acc_vgpr_count () const override;
    size_t lds_size () const override;

    uint32_t shader_engine_id () const override
    {
      return compute_relaunch_wave_payload_se_id (m_compute_relaunch_wave);
    }
  };

  std::unique_ptr<architecture_t::cwsr_record_t> make_gfx9_cwsr_record (
    compute_queue_t &queue, uint32_t compute_relaunch_wave,
    uint32_t compute_relaunch_state,
    amd_dbgapi_global_address_t context_save_address) const override
  {
    return std::make_unique<cwsr_record_t> (queue, compute_relaunch_wave,
                                            compute_relaunch_state,
                                            context_save_address);
  }

public:
  gfx90a_t ()
    : mi_architecture_t (EF_AMDGPU_MACH_AMDGCN_GFX90A,
                         "amdgcn-amd-amdhsa--gfx90a")
  {
  }

  bool can_halt_at_endpgm () const override { return false; }
};

size_t
gfx90a_t::cwsr_record_t::vgpr_count () const
{
  uint32_t arch_vgpr_blocks
    = compute_relaunch_state_payload_accum_offset (m_compute_relaunch_state)
      + 1;
  return arch_vgpr_blocks * 4;
}

size_t
gfx90a_t::cwsr_record_t::acc_vgpr_count () const
{
  uint32_t vgpr_blocks
    = compute_relaunch_state_payload_vgprs (m_compute_relaunch_state) + 1;
  return vgpr_blocks * 8 - vgpr_count ();
}

size_t
gfx90a_t::cwsr_record_t::lds_size () const
{
  return compute_relaunch_state_payload_lds_size (m_compute_relaunch_state)
         * 128 * sizeof (uint32_t);
}

class gfx10_architecture_t : public gfx9_architecture_t
{
protected:
  class cwsr_record_t : public gfx9_architecture_t::cwsr_record_t
  {
  protected:
    /* On gfx10, there are 2 COMPUTE_RELAUNCH registers for state.  */
    uint32_t const m_compute_relaunch2_state;

    static constexpr uint32_t
    compute_relaunch_state_payload_lds_size (uint32_t relaunch_state)
    {
      return utils::bit_extract (relaunch_state, 10, 17);
    }
    static constexpr uint32_t
    compute_relaunch_state_payload_w32_en (uint32_t relaunch_state)
    {
      return utils::bit_extract (relaunch_state, 24, 24);
    }
    static constexpr uint32_t
    compute_relaunch_state_payload_shared_vgprs (uint32_t relaunch_state)
    {
      return utils::bit_extract (relaunch_state, 26, 29);
    }
    static constexpr uint32_t
    compute_relaunch_wave_payload_scratch_scoreboard_id (
      uint32_t relaunch_wave)
    {
      return utils::bit_extract (relaunch_wave, 0, 9);
    }
    static constexpr uint32_t
    compute_relaunch_wave_payload_scratch_en (uint32_t relaunch_wave)
    {
      return utils::bit_extract (relaunch_wave, 11, 11);
    }
    static constexpr uint32_t
    compute_relaunch_wave_payload_first_wave (uint32_t relaunch_wave)
    {
      return utils::bit_extract (relaunch_wave, 12, 12);
    }
    static constexpr uint32_t
    compute_relaunch_wave_payload_se_id (uint32_t relaunch_wave)
    {
      return utils::bit_extract (relaunch_wave, 24, 25);
    }
    static constexpr uint32_t
    compute_relaunch_wave_payload_last_wave (uint32_t relaunch_wave)
    {
      return utils::bit_extract (relaunch_wave, 29, 29);
    }

  public:
    cwsr_record_t (compute_queue_t &queue, uint32_t compute_relaunch_wave,
                   uint32_t compute_relaunch_state,
                   uint32_t compute_relaunch2_state,
                   amd_dbgapi_global_address_t context_save_address)
      : gfx9_architecture_t::cwsr_record_t (queue, compute_relaunch_wave,
                                            compute_relaunch_state,
                                            context_save_address),
        m_compute_relaunch2_state (compute_relaunch2_state)
    {
    }

    size_t sgpr_count () const override;
    size_t vgpr_count () const override;
    virtual size_t shared_vgpr_count () const;

    bool is_scratch_enabled () const override;

    uint32_t shader_engine_id () const override;
    uint32_t scratch_scoreboard_id () const override;

    size_t lane_count () const override;
    size_t lds_size () const override;

    bool is_last_wave () const override;
    bool is_first_wave () const override;

    std::optional<amd_dbgapi_global_address_t>
    register_address (amdgpu_regnum_t regnum) const override;
  };

  virtual std::unique_ptr<architecture_t::cwsr_record_t>
  make_gfx10_cwsr_record (
    compute_queue_t &queue, uint32_t compute_relaunch_wave,
    uint32_t compute_relaunch_state, uint32_t compute_relaunch2_state,
    amd_dbgapi_global_address_t context_save_address) const
  {
    return std::make_unique<cwsr_record_t> (
      queue, compute_relaunch_wave, compute_relaunch_state,
      compute_relaunch2_state, context_save_address);
  }

  std::optional<amdgpu_regnum_t>
  scalar_operand_to_regnum (int operand, bool priv = false) const override;
  size_t scalar_register_count () const override { return 106; }
  size_t scalar_alias_count () const override { return 2; }

  amd_dbgapi_global_address_t
  branch_target (wave_t &wave, amd_dbgapi_global_address_t pc,
                 const instruction_t &instruction) const override;

  gfx10_architecture_t (elf_amdgpu_machine_t e_machine,
                        std::string target_triple);

public:
  std::string register_name (amdgpu_regnum_t regnum) const override;
  std::string register_type (amdgpu_regnum_t regnum) const override;
  amd_dbgapi_size_t register_size (amdgpu_regnum_t regnum) const override;
  amd_dbgapi_register_properties_t
  register_properties (amdgpu_regnum_t regnum) const override;

  bool is_pseudo_register_available (const wave_t &wave,
                                     amdgpu_regnum_t regnum) const override;

  void read_pseudo_register (const wave_t &wave, amdgpu_regnum_t regnum,
                             size_t offset, size_t value_size,
                             void *value) const override;

  void write_pseudo_register (wave_t &wave, amdgpu_regnum_t regnum,
                              size_t offset, size_t value_size,
                              const void *value) const override;

  virtual bool is_code_end (const instruction_t &instruction) const;
  virtual bool
  is_subvector_loop_begin (const instruction_t &instruction) const;
  virtual bool is_subvector_loop_end (const instruction_t &instruction) const;

  bool is_call (const instruction_t &instruction) const override;
  bool is_getpc (const instruction_t &instruction) const override;
  bool is_setpc (const instruction_t &instruction) const override;
  bool is_swappc (const instruction_t &instruction) const override;
  bool is_cbranch_i_fork (const instruction_t &instruction) const override;
  bool is_cbranch_g_fork (const instruction_t &instruction) const override;
  bool is_cbranch_join (const instruction_t &instruction) const override;
  bool is_sequential (const instruction_t &instruction) const override;

  bool can_execute_displaced (wave_t &wave,
                              const instruction_t &instruction) const override;
  bool can_simulate (wave_t &wave,
                     const instruction_t &instruction) const override;

  std::optional<amd_dbgapi_global_address_t>
  simulate_instruction (wave_t &wave, amd_dbgapi_global_address_t pc,
                        const instruction_t &instruction) const override;

  std::tuple<amd_dbgapi_instruction_kind_t,       /* instruction_kind  */
             amd_dbgapi_instruction_properties_t, /* instruction_properties  */
             size_t,                              /* instruction_size  */
             std::vector<uint64_t> /* instruction_information  */>
  classify_instruction (amd_dbgapi_global_address_t address,
                        const instruction_t &instruction) const override;

  size_t control_stack_iterate (
    compute_queue_t &queue, const uint32_t *control_stack,
    size_t control_stack_words, amd_dbgapi_global_address_t wave_area_address,
    amd_dbgapi_size_t wave_area_size,
    const std::function<void (std::unique_ptr<architecture_t::cwsr_record_t>)>
      &wave_callback) const override;

  bool can_halt_at_endpgm () const override { return false; }
  size_t largest_instruction_size () const override { return 20; }
};

gfx10_architecture_t::gfx10_architecture_t (elf_amdgpu_machine_t e_machine,
                                            std::string target_triple)
  : gfx9_architecture_t (e_machine, std::move (target_triple))
{
  /* Scalar registers: [s103-s105]  */
  register_class_t *scalar_registers
    = find_if ([] (const register_class_t &register_class)
               { return register_class.name () == "scalar"; });
  dbgapi_assert (scalar_registers != nullptr);

  scalar_registers->add_registers (
    amdgpu_regnum_t::first_sgpr
      + gfx9_architecture_t::scalar_register_count (),
    amdgpu_regnum_t::first_sgpr
      + gfx10_architecture_t::scalar_register_count () - 1);

  /* Vector registers: [v0_32-v255_32]  */
  register_class_t *vector_registers
    = find_if ([] (const register_class_t &register_class)
               { return register_class.name () == "vector"; });
  dbgapi_assert (vector_registers != nullptr);

  vector_registers->add_registers (amdgpu_regnum_t::first_vgpr_32,
                                   amdgpu_regnum_t::last_vgpr_32);

  /* System registers: [xnack_mask_32]  */
  register_class_t *system_registers
    = find_if ([] (const register_class_t &register_class)
               { return register_class.name () == "system"; });
  dbgapi_assert (system_registers != nullptr);

  system_registers->add_registers (amdgpu_regnum_t::xnack_mask_32,
                                   amdgpu_regnum_t::xnack_mask_32);
  system_registers->remove_registers (amdgpu_regnum_t::xnack_mask_64,
                                      amdgpu_regnum_t::xnack_mask_64);

  /* General registers: [s103-s105, {vector}_32, exec_32, vcc_32]  */
  register_class_t *general_registers
    = find_if ([] (const register_class_t &register_class)
               { return register_class.name () == "general"; });
  dbgapi_assert (general_registers != nullptr);

  general_registers->add_registers (
    amdgpu_regnum_t::first_sgpr
      + gfx9_architecture_t::scalar_register_count (),
    amdgpu_regnum_t::first_sgpr
      + gfx10_architecture_t::scalar_register_count () - 1);
  general_registers->add_registers (amdgpu_regnum_t::first_vgpr_32,
                                    amdgpu_regnum_t::last_vgpr_32);
  general_registers->add_registers (amdgpu_regnum_t::pseudo_exec_32,
                                    amdgpu_regnum_t::pseudo_exec_32);
  general_registers->add_registers (amdgpu_regnum_t::pseudo_vcc_32,
                                    amdgpu_regnum_t::pseudo_vcc_32);
}

std::string
gfx10_architecture_t::register_name (amdgpu_regnum_t regnum) const
{
  if (regnum >= amdgpu_regnum_t::first_vgpr_32
      && regnum <= amdgpu_regnum_t::last_vgpr_32)
    {
      return string_printf ("v%ld", regnum - amdgpu_regnum_t::first_vgpr_32);
    }
  if (regnum == amdgpu_regnum_t::exec_32
      || regnum == amdgpu_regnum_t::pseudo_exec_32)
    {
      return "exec";
    }
  if (regnum == amdgpu_regnum_t::vcc_32
      || regnum == amdgpu_regnum_t::pseudo_vcc_32)
    {
      return "vcc";
    }
  if (regnum == amdgpu_regnum_t::xnack_mask_32)
    {
      return "xnack_mask";
    }

  switch (regnum)
    {
    case amdgpu_regnum_t::xnack_mask_hi:
    case amdgpu_regnum_t::xnack_mask_64:
    case amdgpu_regnum_t::csp:
      dbgapi_assert_not_reached ("invalid register number");

    default:
      return gfx9_architecture_t::register_name (regnum);
    }
}

std::string
gfx10_architecture_t::register_type (amdgpu_regnum_t regnum) const
{
  /* Vector registers (arch and acc).  */
  if ((regnum >= amdgpu_regnum_t::first_vgpr_32
       && regnum <= amdgpu_regnum_t::last_vgpr_32))
    {
      return "int32_t[32]";
    }
  if (regnum == amdgpu_regnum_t::exec_32
      || regnum == amdgpu_regnum_t::pseudo_exec_32
      || regnum == amdgpu_regnum_t::vcc_32
      || regnum == amdgpu_regnum_t::pseudo_vcc_32
      || regnum == amdgpu_regnum_t::xnack_mask_32)
    {
      return "uint32_t";
    }

  switch (regnum)
    {
    case amdgpu_regnum_t::pseudo_status:
      return "flags32_t status {"
             "  bool SCC @0;"
             "  uint32_t SPI_PRIO @1-2;"
             "  uint32_t USER_PRIO @3-4;"
             "  bool PRIV @5;"
             "  bool TRAP_EN @6;"
             "  bool TTRACE_EN @7;"
             "  bool EXPORT_RDY @8;"
             "  bool EXECZ @9;"
             "  bool VCCZ @10;"
             "  bool IN_TG @11;"
             "  bool IN_BARRIER @12;"
             "  bool HALT @13;"
             "  bool TRAP @14;"
             "  bool TTRACE_SIMD_EN @15;"
             "  bool VALID @16;"
             "  bool ECC_ERR @17;"
             "  bool SKIP_EXPORT @18;"
             "  bool PERF_EN @19;"
             "  bool COND_DBG_USER @20;"
             "  bool COND_DBG_SYS @21;"
             "  bool FATAL_HALT @23;"
             "  bool MUST_EXPORT @27;"
             "}";

    case amdgpu_regnum_t::mode:
      return "flags32_t mode {"
             "  enum fp_round {"
             "    NEAREST_EVEN = 0,"
             "    PLUS_INF  = 1,"
             "    MINUS_INF = 2,"
             "    ZERO      = 3"
             "  } FP_ROUND.32 @0-1;"
             "  enum fp_round FP_ROUND.64_16 @2-3;"
             "  enum fp_denorm {"
             "    FLUSH_SRC_DST = 0,"
             "    FLUSH_DST     = 1,"
             "    FLUSH_SRC     = 2,"
             "    FLUSH_NONE    = 3"
             "  } FP_DENORM.32 @4-5;"
             "  enum fp_denorm FP_DENORM.64_16 @6-7;"
             "  bool DX10_CLAMP @8;"
             "  bool IEEE @9;"
             "  bool LOD_CLAMPED @10;"
             "  bool DEBUG_EN @11;"
             "  bool EXCP_EN.INVALID @12;"
             "  bool EXCP_EN.DENORM @13;"
             "  bool EXCP_EN.DIV0 @14;"
             "  bool EXCP_EN.OVERFLOW @15;"
             "  bool EXCP_EN.UNDERFLOW @16;"
             "  bool EXCP_EN.INEXACT @17;"
             "  bool EXCP_EN.INT_DIV0 @18;"
             "  bool EXCP_EN.ADDR_WATCH @19;"
             "  bool FP16_OVFL @23;"
             "  bool DISABLE_PERF @27;"
             "}";

    case amdgpu_regnum_t::trapsts:
      return "flags32_t trapsts {"
             "  bool EXCP.INVALID @0;"
             "  bool EXCP.DENORM @1;"
             "  bool EXCP.DIV0 @2;"
             "  bool EXCP.OVERFLOW @3;"
             "  bool EXCP.UNDERFLOW @4;"
             "  bool EXCP.INEXACT @5;"
             "  bool EXCP.INT_DIV0 @6;"
             "  bool EXCP.ADDR_WATCH @7;"
             "  bool EXCP.MEM_VIOL @8;"
             "  bool SAVE_CTX @10;"
             "  bool ILLEGAL_INST @11;"
             "  bool EXCP_HI.ADDR_WATCH1 @12;"
             "  bool EXCP_HI.ADDR_WATCH2 @13;"
             "  bool EXCP_HI.ADDR_WATCH3 @14;"
             "  bool BUFFER_OOB @15;"
             "  uint32_t EXCP_CYCLE @16-19;"
             "  uint32_t EXCP_GROUP_MASK @20-23;"
             "  bool EXCP_WAVE64HI @24;"
             "  bool UTC_ERROR @28;"
             "  enum dp_rate {"
             "    NONE    = 0,"
             "    QUARTER = 1,"
             "    FULL    = 4"
             "  } DP_RATE @29-31;"
             "}";

    case amdgpu_regnum_t::xnack_mask_hi:
    case amdgpu_regnum_t::xnack_mask_64:
    case amdgpu_regnum_t::csp:
      dbgapi_assert_not_reached ("invalid register number");

    default:
      return gfx9_architecture_t::register_type (regnum);
    }
}

amd_dbgapi_size_t
gfx10_architecture_t::register_size (amdgpu_regnum_t regnum) const
{
  /* Vector registers (arch and acc).  */
  if ((regnum >= amdgpu_regnum_t::first_vgpr_32
       && regnum <= amdgpu_regnum_t::last_vgpr_32))
    {
      return sizeof (int32_t) * 32;
    }
  if (regnum == amdgpu_regnum_t::exec_32
      || regnum == amdgpu_regnum_t::pseudo_exec_32
      || regnum == amdgpu_regnum_t::vcc_32
      || regnum == amdgpu_regnum_t::pseudo_vcc_32
      || regnum == amdgpu_regnum_t::xnack_mask_32)
    {
      return sizeof (uint32_t);
    }
  switch (regnum)
    {
    case amdgpu_regnum_t::xnack_mask_hi:
    case amdgpu_regnum_t::xnack_mask_64:
    case amdgpu_regnum_t::csp:
      dbgapi_assert_not_reached ("invalid register number");

    default:
      return gfx9_architecture_t::register_size (regnum);
    }
}

amd_dbgapi_register_properties_t
gfx10_architecture_t::register_properties (amdgpu_regnum_t regnum) const
{
  amd_dbgapi_register_properties_t properties
    = gfx9_architecture_t::register_properties (regnum);

  /* Writing to the exec or vcc register may change the status.execz
     status.vccz bits respectively.  */
  if (regnum == amdgpu_regnum_t::pseudo_exec_32
      || regnum == amdgpu_regnum_t::pseudo_vcc_32)
    properties |= AMD_DBGAPI_REGISTER_PROPERTY_INVALIDATE_VOLATILE;

  return properties;
}

bool
gfx10_architecture_t::is_pseudo_register_available (
  const wave_t &wave, amdgpu_regnum_t regnum) const
{
  dbgapi_assert (is_pseudo_register (regnum));

  size_t lane_count = wave.lane_count ();

  if (regnum == amdgpu_regnum_t::pseudo_exec_32
      || regnum == amdgpu_regnum_t::pseudo_vcc_32)
    return lane_count == 32;

  if (regnum == amdgpu_regnum_t::pseudo_exec_64
      || regnum == amdgpu_regnum_t::pseudo_vcc_64)
    return lane_count == 64;

  return gfx9_architecture_t::is_pseudo_register_available (wave, regnum);
}

void
gfx10_architecture_t::read_pseudo_register (const wave_t &wave,
                                            amdgpu_regnum_t regnum,
                                            size_t offset, size_t value_size,
                                            void *value) const
{
  dbgapi_assert (is_pseudo_register (regnum)
                 && is_pseudo_register_available (wave, regnum));

  dbgapi_assert (value_size && (offset + value_size) <= register_size (regnum)
                 && "read_pseudo_register is out of bounds");

  if (regnum == amdgpu_regnum_t::pseudo_exec_32
      || regnum == amdgpu_regnum_t::pseudo_vcc_32)
    {
      dbgapi_assert (wave.lane_count () == 32);

      wave.read_register (regnum == amdgpu_regnum_t::pseudo_exec_32
                            ? amdgpu_regnum_t::exec_32
                            : amdgpu_regnum_t::vcc_32,
                          offset, value_size, value);
      return;
    }

  gfx9_architecture_t::read_pseudo_register (wave, regnum, offset, value_size,
                                             value);
}

void
gfx10_architecture_t::write_pseudo_register (wave_t &wave,
                                             amdgpu_regnum_t regnum,
                                             size_t offset, size_t value_size,
                                             const void *value) const
{
  dbgapi_assert (is_pseudo_register (regnum)
                 && is_pseudo_register_available (wave, regnum));

  dbgapi_assert (value_size && (offset + value_size) <= register_size (regnum)
                 && "write_pseudo_register is out of bounds");

  if (regnum == amdgpu_regnum_t::pseudo_exec_32
      || regnum == amdgpu_regnum_t::pseudo_vcc_32)
    {
      dbgapi_assert (wave.lane_count () == 32);
      uint32_t base_reg, status_reg;

      amdgpu_regnum_t base_regnum = regnum == amdgpu_regnum_t::pseudo_exec_32
                                      ? amdgpu_regnum_t::exec_32
                                      : amdgpu_regnum_t::vcc_32;
      uint32_t status_mask = regnum == amdgpu_regnum_t::pseudo_exec_32
                               ? sq_wave_status_execz_mask
                               : sq_wave_status_vccz_mask;

      wave.read_register (amdgpu_regnum_t::status, &status_reg);
      wave.read_register (base_regnum, &base_reg);

      memcpy (reinterpret_cast<char *> (&base_reg) + offset, value,
              value_size);

      status_reg
        = (status_reg & ~status_mask) | (base_reg == 0 ? status_mask : 0);

      wave.write_register (amdgpu_regnum_t::status, status_reg);
      wave.write_register (base_regnum, base_reg);
      return;
    }

  gfx9_architecture_t::write_pseudo_register (wave, regnum, offset, value_size,
                                              value);
}

std::optional<amdgpu_regnum_t>
gfx10_architecture_t::scalar_operand_to_regnum (int operand, bool priv) const
{
  if (operand >= 0 && operand <= 105)
    {
      /* SGPR[0] through SGPR[105]  */
      return amdgpu_regnum_t::s0 + operand;
    }

  if (operand >= 108 && operand <= 123)
    {
      /* TTMP[0] through TTMP[15]  */
      return priv ? amdgpu_regnum_t::first_ttmp + (operand - 108)
                  : amdgpu_regnum_t::null;
    }

  switch (operand)
    {
    case 106:
      return amdgpu_regnum_t::vcc_lo;
    case 107:
      return amdgpu_regnum_t::vcc_hi;
    case 124:
      return amdgpu_regnum_t::m0;
    case 125:
      return amdgpu_regnum_t::null;
    case 126:
      return amdgpu_regnum_t::exec_lo;
    case 127:
      return amdgpu_regnum_t::exec_hi;
    default:
      return std::nullopt;
    }
}

amd_dbgapi_global_address_t
gfx10_architecture_t::branch_target (wave_t &wave,
                                     amd_dbgapi_global_address_t pc,
                                     const instruction_t &instruction) const
{
  dbgapi_assert (instruction.is_valid ());

  if (is_subvector_loop_begin (instruction)
      || is_subvector_loop_end (instruction))
    {
      return pc + instruction.size ()
             + (static_cast<ssize_t> (simm16_operand (instruction)) << 2);
    }

  return gfx9_architecture_t::branch_target (wave, pc, instruction);
}

bool
gfx10_architecture_t::cwsr_record_t::is_scratch_enabled () const
{
  return compute_relaunch_wave_payload_scratch_en (m_compute_relaunch_wave);
}

uint32_t
gfx10_architecture_t::cwsr_record_t::shader_engine_id () const
{
  return compute_relaunch_wave_payload_se_id (m_compute_relaunch_wave);
}

uint32_t
gfx10_architecture_t::cwsr_record_t::scratch_scoreboard_id () const
{
  return compute_relaunch_wave_payload_scratch_scoreboard_id (
    m_compute_relaunch_wave);
}

std::optional<amd_dbgapi_global_address_t>
gfx10_architecture_t::cwsr_record_t::register_address (
  amdgpu_regnum_t regnum) const
{
  size_t lane_count = this->lane_count ();

  if (((regnum == amdgpu_regnum_t::exec_32
        || regnum == amdgpu_regnum_t::xnack_mask_32)
       && lane_count != 32)
      || (regnum == amdgpu_regnum_t::exec_64 && lane_count != 64))
    return std::nullopt;

  /* Rename registers that map to the hwreg block.  */
  switch (regnum)
    {
    case amdgpu_regnum_t::exec_32:
      regnum = amdgpu_regnum_t::first_hwreg + 3;
      break;

    case amdgpu_regnum_t::xnack_mask_32:
      regnum = amdgpu_regnum_t::first_hwreg + 7;
      break;

    case amdgpu_regnum_t::mode:
      regnum = amdgpu_regnum_t::first_hwreg + 8;
      break;

    case amdgpu_regnum_t::xnack_mask_hi:
    case amdgpu_regnum_t::xnack_mask_64:
      /* On gfx10, xnack_mask is now a 32bit register.  */
      return std::nullopt;

    case amdgpu_regnum_t::flat_scratch:
      /* On gfx10, flat_scratch is an architected register, so it is saved
         in the hwregs block.  */
      regnum = amdgpu_regnum_t::first_hwreg + 9;
      break;

    default:
      break;
    }

  if ((regnum == amdgpu_regnum_t::vcc_32 && lane_count != 32)
      || (regnum == amdgpu_regnum_t::vcc_64 && lane_count != 64))
    return std::nullopt;

  /* Rename aliased registers.  */
  switch (regnum)
    {
    case amdgpu_regnum_t::vcc_32:
      regnum = amdgpu_regnum_t::vcc_lo;
      break;

    default:
      break;
    }

  /* Now that renaming is done, delegate to the gfx9 base for all registers
     except vector registers.  The vector register slayout in the context save
     area is different for gfx10 because of the shared vgprs, so we'll have
     to handle it in this function.  */
  if (regnum < amdgpu_regnum_t::first_vgpr
      || regnum > amdgpu_regnum_t::last_vgpr)
    return gfx9_architecture_t::cwsr_record_t::register_address (regnum);

  auto first_sgpr_addr = gfx9_architecture_t::cwsr_record_t::register_address (
    amdgpu_regnum_t::first_sgpr);
  dbgapi_assert (first_sgpr_addr);

  size_t sgprs_addr = *first_sgpr_addr;

  /* The shared vgprs are 32-wide vector registers shared between the 2 halves
     of a wave64 on gfx10.  They are logically addressed right after the
     64-wide private vector registers.  Note: In wave32, although unsupported,
     they are still allocated.  */
  size_t shared_vgpr_count = this->shared_vgpr_count ();
  size_t shared_vgpr_size = sizeof (int32_t) * 32;
  size_t shared_vgprs_addr = sgprs_addr - shared_vgpr_count * shared_vgpr_size;

  size_t private_vgpr_count = this->vgpr_count ();
  size_t private_vgpr_size = sizeof (int32_t) * lane_count;
  size_t private_vgprs_addr
    = shared_vgprs_addr - private_vgpr_count * private_vgpr_size;

  if (regnum >= (amdgpu_regnum_t::v0_32 + private_vgpr_count)
      && regnum <= amdgpu_regnum_t::last_vgpr_32
      && ((regnum - amdgpu_regnum_t::v0_32)
          < (private_vgpr_count + shared_vgpr_count)))
    {
      return shared_vgprs_addr
             + (regnum - (amdgpu_regnum_t::v0_32 + private_vgpr_count))
                 * shared_vgpr_size;
    }

  if (lane_count == 32 && regnum >= amdgpu_regnum_t::first_vgpr_32
      && regnum <= amdgpu_regnum_t::last_vgpr_32
      && ((regnum - amdgpu_regnum_t::v0_32) < private_vgpr_count))
    {
      return private_vgprs_addr
             + (regnum - amdgpu_regnum_t::v0_32) * private_vgpr_size;
    }

  if (lane_count == 64 && regnum >= amdgpu_regnum_t::first_vgpr_64
      && regnum <= amdgpu_regnum_t::last_vgpr_64
      && ((regnum - amdgpu_regnum_t::v0_64) < private_vgpr_count))
    {
      return private_vgprs_addr
             + (regnum - amdgpu_regnum_t::v0_64) * private_vgpr_size;
    }

  return std::nullopt;
}

bool
gfx10_architecture_t::is_code_end (const instruction_t &instruction) const
{
  /* s_code_end: SOPP Opcode 31. See comment in ::is_endpgm.  */
  return is_sopp_encoding<31> (instruction);
}

bool
gfx10_architecture_t::is_call (const instruction_t &instruction) const
{
  /* s_call: SOPK Opcode 22  */
  return instruction.is_valid () && is_sopk_encoding<22> (instruction)
         && !(sdst_operand (instruction) & 1);
}

bool
gfx10_architecture_t::is_getpc (const instruction_t &instruction) const
{
  /* s_getpc: SOP1 Opcode 31  */
  return instruction.is_valid () && is_sop1_encoding<31> (instruction)
         && !(sdst_operand (instruction) & 1);
}

bool
gfx10_architecture_t::is_setpc (const instruction_t &instruction) const
{
  /* s_setpc: SOP1 Opcode 32  */
  return instruction.is_valid () && is_sop1_encoding<32> (instruction);
}

bool
gfx10_architecture_t::is_swappc (const instruction_t &instruction) const
{
  /* s_swappc: SOP1 Opcode 33  */
  return instruction.is_valid () && is_sop1_encoding<33> (instruction)
         && !(ssrc0_operand (instruction) & 1);
}

bool
gfx10_architecture_t::is_cbranch_i_fork (
  const instruction_t & /* instruction  */) const
{
  return false;
}

bool
gfx10_architecture_t::is_cbranch_g_fork (
  const instruction_t & /* instruction  */) const
{
  return false;
}

bool
gfx10_architecture_t::is_cbranch_join (
  const instruction_t & /* instruction  */) const
{
  return false;
}

bool
gfx10_architecture_t::is_subvector_loop_begin (
  const instruction_t &instruction) const
{
  /* s_subvector_loop_begin: SOPK Opcode 27  */
  return instruction.is_valid () && is_sopk_encoding<27> (instruction);
}

bool
gfx10_architecture_t::is_subvector_loop_end (
  const instruction_t &instruction) const
{
  /* s_subvector_loop_end: SOPK Opcode 28  */
  return instruction.is_valid () && is_sopk_encoding<28> (instruction);
}

bool
gfx10_architecture_t::is_sequential (const instruction_t &instruction) const
{
  if (!instruction.is_valid ())
    return false;

  return /* s_endpgm/s_branch/s_cbranch  */
    !is_sopp_encoding<1, 2, 4, 5, 6, 7, 8, 9, 23, 24, 25, 26> (instruction)
    /* s_setpc_b64/s_swappc_b64  */
    && !is_sop1_encoding<32, 33> (instruction)
    /* s_call_b64/s_subvector_loop_begin/s_subvector_loop_end  */
    && !is_sopk_encoding<22, 27, 28> (instruction);
}

size_t
gfx10_architecture_t::cwsr_record_t::sgpr_count () const
{
  return 128;
}
size_t
gfx10_architecture_t::cwsr_record_t::vgpr_count () const
{ /* vgprs are allocated in blocks of 8/4 registers (W32/W64).  */
  return (1 + utils::bit_extract (m_compute_relaunch_state, 0, 5))
         * (utils::bit_extract (m_compute_relaunch_state, 24, 24) ? 8 : 4);
}

size_t
gfx10_architecture_t::cwsr_record_t::shared_vgpr_count () const
{
  uint32_t shared_vgpr_blocks
    = compute_relaunch_state_payload_shared_vgprs (m_compute_relaunch_state);
  return shared_vgpr_blocks * 8;
}

size_t
gfx10_architecture_t::cwsr_record_t::lane_count () const
{
  return compute_relaunch_state_payload_w32_en (m_compute_relaunch_state) ? 32
                                                                          : 64;
}

size_t
gfx10_architecture_t::cwsr_record_t::lds_size () const
{
  /* lds_size: 128 dwords granularity.  */
  return compute_relaunch_state_payload_lds_size (m_compute_relaunch_state)
         * 128 * sizeof (uint32_t);
}

bool
gfx10_architecture_t::cwsr_record_t::is_last_wave () const
{
  return compute_relaunch_wave_payload_last_wave (m_compute_relaunch_wave);
}

bool
gfx10_architecture_t::cwsr_record_t::is_first_wave () const
{
  return compute_relaunch_wave_payload_first_wave (m_compute_relaunch_wave);
}

bool
gfx10_architecture_t::can_execute_displaced (
  wave_t &wave, const instruction_t &instruction) const
{
  /* Note: make sure that is_valid_encoding () is up to date so that invalid
     instruction encodings are not displaced-stepped.  */

  if (!instruction.is_valid ())
    return false;

  if (is_subvector_loop_begin (instruction)
      || is_subvector_loop_end (instruction))
    return false;

  return gfx9_architecture_t::can_execute_displaced (wave, instruction);
}

bool
gfx10_architecture_t::can_simulate (wave_t &wave,
                                    const instruction_t &instruction) const
{
  /* Note: make sure that is_valid_encoding () is up to date so that invalid
     instruction encodings are not simulated.  */

  if (!instruction.is_valid ())
    return false;

  if (is_subvector_loop_begin (instruction)
      || is_subvector_loop_end (instruction))
    return scalar_operand_to_regnum (sdst_operand (instruction)).has_value ()
           /* Can only simulate subvector_loop_* in wave64 mode.  */
           && wave.lane_count () == 64;

  return gfx9_architecture_t::can_simulate (wave, instruction);
}

std::optional<amd_dbgapi_global_address_t>
gfx10_architecture_t::simulate_instruction (
  wave_t &wave, amd_dbgapi_global_address_t pc,
  const instruction_t &instruction) const
{
  if (is_subvector_loop_begin (instruction))
    {
      dbgapi_assert (wave.lane_count () == 64);
      auto s0_regnum = scalar_operand_to_regnum (sdst_operand (instruction));
      dbgapi_assert (s0_regnum);

      uint32_t exec_lo, exec_hi;
      wave.read_register (amdgpu_regnum_t::exec_lo, &exec_lo);
      wave.read_register (amdgpu_regnum_t::exec_hi, &exec_hi);

      if (exec_lo == 0 && exec_hi == 0)
        return branch_target (wave, pc, instruction);

      if (exec_lo == 0)
        {
          /* Single pass, execute the high half now.  */
          wave.write_register (*s0_regnum, exec_lo);
        }
      else
        {
          /* Save the high half for the 2nd pass, and execute the low half.  */
          wave.write_register (*s0_regnum, exec_hi);
          wave.write_register (amdgpu_regnum_t::exec_hi, uint32_t{ 0 });
        }

      return pc + instruction.size ();
    }

  if (is_subvector_loop_end (instruction))
    {
      dbgapi_assert (wave.lane_count () == 64);
      auto s0_regnum = scalar_operand_to_regnum (sdst_operand (instruction));
      dbgapi_assert (s0_regnum);

      uint32_t exec_lo, exec_hi, s0;
      wave.read_register (amdgpu_regnum_t::exec_lo, &exec_lo);
      wave.read_register (amdgpu_regnum_t::exec_hi, &exec_hi);
      wave.read_register (*s0_regnum, &s0);

      if (exec_hi != 0)
        {
          /* Done executing the 2nd half.  */
          wave.write_register (amdgpu_regnum_t::exec_lo, s0);
        }
      else if (s0 != 0)
        {
          /* Jump to start and execute the 2nd half.  */
          wave.write_register (amdgpu_regnum_t::exec_hi, s0);
          wave.write_register (amdgpu_regnum_t::exec_lo, uint32_t{ 0 });
          wave.write_register (*s0_regnum, exec_lo);
          return branch_target (wave, pc, instruction);
        }

      return pc + instruction.size ();
    }

  return gfx9_architecture_t::simulate_instruction (wave, pc, instruction);
}

std::tuple<amd_dbgapi_instruction_kind_t, amd_dbgapi_instruction_properties_t,
           size_t, std::vector<uint64_t>>
gfx10_architecture_t::classify_instruction (
  amd_dbgapi_global_address_t address, const instruction_t &instruction) const
{
  dbgapi_assert (instruction.is_valid ());

  if (is_subvector_loop_begin (instruction)
      || is_subvector_loop_end (instruction))
    {
      return {
        AMD_DBGAPI_INSTRUCTION_KIND_DIRECT_BRANCH_CONDITIONAL,
        AMD_DBGAPI_INSTRUCTION_PROPERTY_NONE, instruction.size (),
        std::vector<uint64_t> (
          { address + instruction.size ()
            + (static_cast<ssize_t> (simm16_operand (instruction)) << 2) })
      };
    }

  if (is_code_end (instruction))
    {
      return { AMD_DBGAPI_INSTRUCTION_KIND_UNKNOWN,
               AMD_DBGAPI_INSTRUCTION_PROPERTY_NONE,
               instruction.size (),
               {} };
    }

  return gfx9_architecture_t::classify_instruction (address, instruction);
}

size_t
gfx10_architecture_t::control_stack_iterate (
  compute_queue_t &queue, const uint32_t *control_stack,
  size_t control_stack_words, amd_dbgapi_global_address_t wave_area_address,
  amd_dbgapi_size_t wave_area_size,
  const std::function<void (std::unique_ptr<architecture_t::cwsr_record_t>)>
    &wave_callback) const
{
  size_t wave_count = 0;
  uint32_t state0 = 0, state1 = 0;

  amd_dbgapi_global_address_t last_wave_area = wave_area_address;

  for (size_t i = 2; /* Skip the 2 PM4 packets at the top of the stack.  */
       i < control_stack_words; ++i)
    {
      uint32_t relaunch = control_stack[i];

      if (compute_relaunch_is_event (relaunch))
        {
          /* Skip events.  */
        }
      else if (compute_relaunch_is_state (relaunch))
        {
          state0 = relaunch;
          /* On gfx10, there are 2 COMPUTE_RELAUNCH registers for state.  */
          state1 = control_stack[++i];
        }
      else
        {
          auto cwsr_record = make_gfx10_cwsr_record (queue, relaunch, state0,
                                                     state1, last_wave_area);

          last_wave_area = cwsr_record->begin ();
          wave_callback (std::move (cwsr_record));
          ++wave_count;
        }
    }

  /* After iterating the control stack, we should have consumed all the data in
     the wave save area, and last_wave_area should point to the bottom of the
     wave save area.  */
  if (last_wave_area != (wave_area_address - wave_area_size))
    fatal_error ("Corrupted control stack or wave save area");

  return wave_count;
}

class gfx10_1_t : public gfx10_architecture_t
{
protected:
  gfx10_1_t (elf_amdgpu_machine_t e_machine, std::string target_triple)
    : gfx10_architecture_t (e_machine, std::move (target_triple))
  {
  }
};

class gfx1010_t final : public gfx10_1_t
{
public:
  gfx1010_t ()
    : gfx10_1_t (EF_AMDGPU_MACH_AMDGCN_GFX1010, "amdgcn-amd-amdhsa--gfx1010")
  {
  }
};

class gfx1011_t final : public gfx10_1_t
{
public:
  gfx1011_t ()
    : gfx10_1_t (EF_AMDGPU_MACH_AMDGCN_GFX1011, "amdgcn-amd-amdhsa--gfx1011")
  {
  }
};

class gfx1012_t final : public gfx10_1_t
{
public:
  gfx1012_t ()
    : gfx10_1_t (EF_AMDGPU_MACH_AMDGCN_GFX1012, "amdgcn-amd-amdhsa--gfx1012")
  {
  }
};

class gfx10_3_t : public gfx10_1_t
{
protected:
  gfx10_3_t (elf_amdgpu_machine_t e_machine, std::string target_triple)
    : gfx10_1_t (e_machine, std::move (target_triple))
  {
  }

public:
  bool can_halt_at_endpgm () const override { return true; }
};

class gfx1030_t final : public gfx10_3_t
{
public:
  gfx1030_t ()
    : gfx10_3_t (EF_AMDGPU_MACH_AMDGCN_GFX1030, "amdgcn-amd-amdhsa--gfx1030")
  {
  }
};

class gfx1031_t final : public gfx10_3_t
{
public:
  gfx1031_t ()
    : gfx10_3_t (EF_AMDGPU_MACH_AMDGCN_GFX1031, "amdgcn-amd-amdhsa--gfx1031")
  {
  }
};

architecture_t::architecture_t (elf_amdgpu_machine_t e_machine,
                                std::string target_triple)
  : m_architecture_id (
    amd_dbgapi_architecture_id_t{ s_next_architecture_id () }),
    m_e_machine (e_machine), m_target_triple (std::move (target_triple))
{
}

architecture_t::~architecture_t ()
{
  if (this == detail::last_found_architecture)
    detail::last_found_architecture = nullptr;
}

std::string
architecture_t::name () const
{
  size_t pos = m_target_triple.rfind ('-');
  dbgapi_assert (pos != std::string::npos);
  return m_target_triple.substr (pos + 1);
}

const architecture_t *
architecture_t::find (amd_dbgapi_architecture_id_t architecture_id, int)
{
  if (detail::last_found_architecture
      && detail::last_found_architecture->id () == architecture_id)
    return detail::last_found_architecture;

  auto it = s_architecture_map.find (architecture_id);
  if (it != s_architecture_map.end ())
    {
      auto architecture = it->second.get ();
      detail::last_found_architecture = architecture;
      return architecture;
    }

  return nullptr;
}

const architecture_t *
architecture_t::find (elf_amdgpu_machine_t elf_amdgpu_machine)
{
  if (detail::last_found_architecture
      && detail::last_found_architecture->elf_amdgpu_machine ()
           == elf_amdgpu_machine)
    return detail::last_found_architecture;

  auto it = std::find_if (
    s_architecture_map.begin (), s_architecture_map.end (),
    [&] (const auto &value)
    { return value.second->elf_amdgpu_machine () == elf_amdgpu_machine; });
  if (it != s_architecture_map.end ())
    {
      auto architecture = it->second.get ();
      detail::last_found_architecture = architecture;
      return architecture;
    }

  return nullptr;
}

const architecture_t *
architecture_t::find (const std::string &name)
{
  if (detail::last_found_architecture
      && detail::last_found_architecture->name () == name)
    return detail::last_found_architecture;

  auto it = std::find_if (
    s_architecture_map.begin (), s_architecture_map.end (),
    [&] (const auto &value) { return value.second->name () == name; });
  if (it != s_architecture_map.end ())
    {
      auto architecture = it->second.get ();
      detail::last_found_architecture = architecture;
      return architecture;
    }

  return nullptr;
}

std::set<amdgpu_regnum_t>
architecture_t::register_set () const
{
  std::set<amdgpu_regnum_t> all_registers;

  for (auto &&register_class : range<register_class_t> ())
    {
      auto class_registers = register_class.register_set ();
      all_registers.insert (class_registers.begin (), class_registers.end ());
    }

  return all_registers;
}

bool
architecture_t::is_register_available (amdgpu_regnum_t regnum) const
{
  for (auto &&register_class : range<register_class_t> ())
    if (register_class.contains (regnum))
      return true;
  return false;
}

void
architecture_t::get_info (amd_dbgapi_architecture_info_t query,
                          size_t value_size, void *value) const
{
  switch (query)
    {
    case AMD_DBGAPI_ARCHITECTURE_INFO_NAME:
      utils::get_info (value_size, value, name ());
      return;

    case AMD_DBGAPI_ARCHITECTURE_INFO_ELF_AMDGPU_MACHINE:
      utils::get_info (value_size, value, elf_amdgpu_machine ());
      return;

    case AMD_DBGAPI_ARCHITECTURE_INFO_LARGEST_INSTRUCTION_SIZE:
      utils::get_info (value_size, value, largest_instruction_size ());
      return;

    case AMD_DBGAPI_ARCHITECTURE_INFO_MINIMUM_INSTRUCTION_ALIGNMENT:
      utils::get_info (value_size, value, minimum_instruction_alignment ());
      return;

    case AMD_DBGAPI_ARCHITECTURE_INFO_BREAKPOINT_INSTRUCTION_SIZE:
      utils::get_info (value_size, value, breakpoint_instruction ().size ());
      return;

    case AMD_DBGAPI_ARCHITECTURE_INFO_BREAKPOINT_INSTRUCTION:
      utils::get_info (value_size, value, breakpoint_instruction ());
      return;

    case AMD_DBGAPI_ARCHITECTURE_INFO_BREAKPOINT_INSTRUCTION_PC_ADJUST:
      utils::get_info (value_size, value, breakpoint_instruction_pc_adjust ());
      return;

    case AMD_DBGAPI_ARCHITECTURE_INFO_PC_REGISTER:
      utils::get_info (value_size, value,
                       regnum_to_register_id (amdgpu_regnum_t::pc));
      return;
    }

  throw api_error_t (AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT);
}

template <typename Architecture, typename... Args>
auto
make_architecture (Args &&...args)
{
  auto arch = std::make_unique<Architecture> (std::forward<Args> (args)...);
  return std::make_pair (arch->id (), std::move (arch));
}

decltype (architecture_t::s_architecture_map)
  architecture_t::s_architecture_map{
    [] ()
    {
      decltype (s_architecture_map) map;
      map.emplace (make_architecture<gfx900_t> ());
      map.emplace (make_architecture<gfx906_t> ());
      map.emplace (make_architecture<gfx908_t> ());
      map.emplace (make_architecture<gfx90a_t> ());
      map.emplace (make_architecture<gfx1010_t> ());
      map.emplace (make_architecture<gfx1011_t> ());
      map.emplace (make_architecture<gfx1012_t> ());
      map.emplace (make_architecture<gfx1030_t> ());
      map.emplace (make_architecture<gfx1031_t> ());
      return map;
    }()
  };

size_t
instruction_t::size () const
{
  if (!m_size.has_value ())
    /* The disassembler may return zero if the instruction is not valid. */
    m_size.emplace (m_architecture.get ().instruction_size (*this));

  return *m_size;
}

} /* namespace amd::dbgapi */

using namespace amd::dbgapi;

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_get_architecture (uint32_t elf_amdgpu_machine,
                             amd_dbgapi_architecture_id_t *architecture_id)
{
  TRACE_BEGIN (param_in (elf_amdgpu_machine), param_in (architecture_id));
  TRY
  {
    if (!detail::is_initialized)
      THROW (AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED);

    if (!architecture_id)
      THROW (AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT);

    const architecture_t *architecture = architecture_t::find (
      static_cast<elf_amdgpu_machine_t> (elf_amdgpu_machine));

    if (!architecture)
      THROW (AMD_DBGAPI_STATUS_ERROR_INVALID_ELF_AMDGPU_MACHINE);

    *architecture_id = architecture->id ();

    return AMD_DBGAPI_STATUS_SUCCESS;
  }
  CATCH (AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED,
         AMD_DBGAPI_STATUS_ERROR_INVALID_ELF_AMDGPU_MACHINE,
         AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT);
  TRACE_END (make_ref (param_out (architecture_id)));
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_architecture_get_info (amd_dbgapi_architecture_id_t architecture_id,
                                  amd_dbgapi_architecture_info_t query,
                                  size_t value_size, void *value)
{
  TRACE_BEGIN (param_in (architecture_id), param_in (query),
               param_in (value_size), param_in (value));
  TRY
  {
    if (!detail::is_initialized)
      THROW (AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED);

    const architecture_t *architecture
      = architecture_t::find (architecture_id);

    if (!architecture)
      THROW (AMD_DBGAPI_STATUS_ERROR_INVALID_ARCHITECTURE_ID);

    architecture->get_info (query, value_size, value);

    return AMD_DBGAPI_STATUS_SUCCESS;
  }
  CATCH (AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED,
         AMD_DBGAPI_STATUS_ERROR_INVALID_ARCHITECTURE_ID,
         AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT,
         AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT_COMPATIBILITY,
         AMD_DBGAPI_STATUS_ERROR_CLIENT_CALLBACK);
  TRACE_END (make_query_ref (query, param_out (value)));
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_disassemble_instruction (
  amd_dbgapi_architecture_id_t architecture_id,
  amd_dbgapi_global_address_t address, amd_dbgapi_size_t *size,
  const void *memory, char **instruction_text,
  amd_dbgapi_symbolizer_id_t symbolizer_id,
  amd_dbgapi_status_t (*symbolizer) (amd_dbgapi_symbolizer_id_t symbolizer_id,
                                     amd_dbgapi_global_address_t address,
                                     char **symbol_text))
{
  TRACE_BEGIN (param_in (architecture_id), make_hex (param_in (address)),
               make_ref (param_in (size)),
               make_hex (make_ref (param_in (memory), size ? *size : 0)),
               param_in (instruction_text), param_in (symbolizer_id),
               param_in (symbolizer));
  TRY
  {
    if (!detail::is_initialized)
      THROW (AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED);

    if (!memory || !size || !*size)
      THROW (AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT);

    const architecture_t *architecture
      = architecture_t::find (architecture_id);

    if (!architecture)
      THROW (AMD_DBGAPI_STATUS_ERROR_INVALID_ARCHITECTURE_ID);

    if (utils::align_down (address,
                           architecture->minimum_instruction_alignment ())
        != address)
      THROW (AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT);

    instruction_t instruction (
      *architecture, std::vector<std::byte> (
                       static_cast<const std::byte *> (memory),
                       static_cast<const std::byte *> (memory) + *size));

    if (!instruction_text)
      {
        if (!instruction.is_valid ())
          THROW (AMD_DBGAPI_STATUS_ERROR_ILLEGAL_INSTRUCTION);

        *size = instruction.size ();
        return AMD_DBGAPI_STATUS_SUCCESS;
      }

    /* Don't call instruction_t::is_valid () as we would end-up calling the
       disassembler twice, first to validate the instruction's size, and a
       second time to disassemble the instruction.  Instead, check that the
       returned instruction size is not 0.  */

    auto [instruction_size, instruction_str, address_operands]
      = architecture->disassemble_instruction (address, instruction);

    if (!instruction_size)
      THROW (AMD_DBGAPI_STATUS_ERROR_ILLEGAL_INSTRUCTION);

    std::string address_operands_str;
    for (auto &&operand : address_operands)
      {
        address_operands_str += address_operands_str.empty () ? "  # " : ", ";

        if (symbolizer)
          {
            char *symbol_text{};

            amd_dbgapi_status_t status
              = symbolizer (symbolizer_id, operand, &symbol_text);

            if (status == AMD_DBGAPI_STATUS_SUCCESS)
              {
                if (!symbol_text)
                  THROW (AMD_DBGAPI_STATUS_ERROR);

                auto deallocate_symbol_text = utils::make_scope_exit (
                  [&] () { deallocate_memory (symbol_text); });

                if (!symbol_text[0])
                  THROW (AMD_DBGAPI_STATUS_ERROR);

                address_operands_str += symbol_text;
                continue;
              }
            else if (status != AMD_DBGAPI_STATUS_ERROR_SYMBOL_NOT_FOUND)
              THROW (AMD_DBGAPI_STATUS_ERROR_CLIENT_CALLBACK);
          }

        address_operands_str += string_printf ("%#lx", operand);
      }

    instruction_str += address_operands_str;

    /* Return the instruction text in client allocated memory.  */
    size_t mem_size = instruction_str.size () + 1;
    auto mem = allocate_memory<char[]> (mem_size);

    memcpy (mem.get (), instruction_str.c_str (), mem_size);
    *instruction_text = mem.release ();
    *size = instruction_size;

    return AMD_DBGAPI_STATUS_SUCCESS;
  }
  CATCH (AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED,
         AMD_DBGAPI_STATUS_ERROR_INVALID_ARCHITECTURE_ID,
         AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT, AMD_DBGAPI_STATUS_ERROR,
         AMD_DBGAPI_STATUS_ERROR_ILLEGAL_INSTRUCTION,
         AMD_DBGAPI_STATUS_ERROR_CLIENT_CALLBACK);
  TRACE_END (make_ref (param_out (size)),
             make_ref (param_out (instruction_text)));
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_classify_instruction (
  amd_dbgapi_architecture_id_t architecture_id,
  amd_dbgapi_global_address_t address, amd_dbgapi_size_t *size_p,
  const void *memory, amd_dbgapi_instruction_kind_t *instruction_kind_p,
  amd_dbgapi_instruction_properties_t *instruction_properties_p,
  void **instruction_information_p)
{
  TRACE_BEGIN (param_in (architecture_id), make_hex (param_in (address)),
               make_ref (param_in (size_p)),
               make_hex (make_ref (param_in (memory), size_p ? *size_p : 0)),
               param_in (instruction_kind_p),
               param_in (instruction_properties_p),
               param_in (instruction_information_p));
  TRY
  {
    if (!detail::is_initialized)
      THROW (AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED);

    if (!memory || !size_p || !*size_p || !instruction_kind_p)
      THROW (AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT);

    const architecture_t *architecture
      = architecture_t::find (architecture_id);

    if (!architecture)
      THROW (AMD_DBGAPI_STATUS_ERROR_INVALID_ARCHITECTURE_ID);

    if (utils::align_down (address,
                           architecture->minimum_instruction_alignment ())
        != address)
      THROW (AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT);

    instruction_t instruction (
      *architecture, std::vector<std::byte> (
                       static_cast<const std::byte *> (memory),
                       static_cast<const std::byte *> (memory) + *size_p));

    if (!instruction.is_valid ())
      THROW (AMD_DBGAPI_STATUS_ERROR_ILLEGAL_INSTRUCTION);

    auto [kind, properties, size, information]
      = architecture->classify_instruction (address, instruction);

    if (instruction_information_p)
      {
        using information_type = decltype (information)::value_type;
        size_t mem_size = information.size () * sizeof (information_type);

        if (!mem_size)
          {
            *instruction_information_p = nullptr;
          }
        else
          {
            auto mem = allocate_memory (mem_size);
            memcpy (mem.get (), information.data (), mem_size);
            *instruction_information_p = mem.release ();
          }
      }

    if (instruction_properties_p)
      *instruction_properties_p = properties;

    *size_p = size;
    *instruction_kind_p = kind;

    return AMD_DBGAPI_STATUS_SUCCESS;
  }
  CATCH (AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED,
         AMD_DBGAPI_STATUS_ERROR_INVALID_ARCHITECTURE_ID,
         AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT, AMD_DBGAPI_STATUS_ERROR,
         AMD_DBGAPI_STATUS_ERROR_ILLEGAL_INSTRUCTION,
         AMD_DBGAPI_STATUS_ERROR_CLIENT_CALLBACK);
  TRACE_END (
    make_ref (param_out (size_p)), make_ref (param_out (instruction_kind_p)),
    make_ref (param_out (instruction_properties_p)),
    make_query_ref (instruction_kind_p ? *instruction_kind_p
                                       : AMD_DBGAPI_INSTRUCTION_KIND_UNKNOWN,
                    param_out (instruction_information_p)));
}
