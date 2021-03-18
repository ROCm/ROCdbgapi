/* Copyright (c) 2019-2021 Advanced Micro Devices, Inc.

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

/* Base class for all AMDGCN architectures.  */

class amdgcn_architecture_t : public architecture_t
{
protected:
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

  static const std::unordered_map<uint16_t, cbranch_cond_t>
    cbranch_opcodes_map;

  static constexpr uint32_t sq_wave_status_scc_mask = 1 << 0;
  static constexpr uint32_t sq_wave_status_priv_mask = 1 << 5;
  static constexpr uint32_t sq_wave_status_trap_en_mask = 1 << 6;
  static constexpr uint32_t sq_wave_status_execz_mask = 1 << 9;
  static constexpr uint32_t sq_wave_status_vccz_mask = 1 << 10;
  static constexpr uint32_t sq_wave_status_halt_mask = 1 << 13;
  static constexpr uint32_t sq_wave_status_cond_dbg_user_mask = 1 << 20;
  static constexpr uint32_t sq_wave_status_cond_dbg_sys_mask = 1 << 21;

  static constexpr uint32_t ttmp11_wave_in_group_mask = 0x003f;
  static constexpr uint32_t ttmp7_dispatch_id_converted_mask = 1 << 31;
  static constexpr uint32_t ttmp7_wave_stopped_mask = 1 << 30;
  static constexpr uint32_t ttmp7_saved_status_halt_mask = 1 << 29;
  static constexpr uint32_t ttmp7_saved_trap_id_mask
    = utils::bit_mask (25, 28);
  static constexpr uint32_t ttmp7_saved_trap_id (uint32_t x)
  {
    return utils::bit_extract (x, 25, 28);
  }
  static constexpr uint32_t ttmp7_queue_packet_id (uint32_t x)
  {
    return utils::bit_extract (x, 0, 24);
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

  static constexpr uint32_t compute_relaunch_is_event (uint32_t x)
  {
    return utils::bit_extract (x, 30, 30);
  }
  static constexpr uint32_t compute_relaunch_is_state (uint32_t x)
  {
    return utils::bit_extract (x, 31, 31);
  }

  amdgcn_architecture_t (elf_amdgpu_machine_t e_machine,
                         std::string target_triple)
    : architecture_t (e_machine, std::move (target_triple))
  {
  }

public:
  void initialize () override;

  amd_dbgapi_status_t convert_address_space (
    const wave_t &wave, amd_dbgapi_lane_id_t lane_id,
    const address_space_t &from_address_space,
    const address_space_t &to_address_space,
    amd_dbgapi_segment_address_t from_address,
    amd_dbgapi_segment_address_t *to_address) const override;

  void lower_address_space (
    const wave_t &wave, amd_dbgapi_lane_id_t *lane_id,
    const address_space_t &original_address_space,
    const address_space_t **lowered_address_space,
    amd_dbgapi_segment_address_t original_address,
    amd_dbgapi_segment_address_t *lowered_address) const override;

  bool address_is_in_address_class (
    const wave_t &wave, amd_dbgapi_lane_id_t lane_id,
    const address_space_t &address_space,
    amd_dbgapi_segment_address_t segment_address,
    const address_class_t &address_class) const override;

  bool address_spaces_may_alias (
    const address_space_t &address_space1,
    const address_space_t &address_space2) const override;

  amd_dbgapi_watchpoint_share_kind_t watchpoint_share_kind () const override
  {
    return AMD_DBGAPI_WATCHPOINT_SHARE_KIND_SHARED;
  };

  size_t watchpoint_count () const override { return 4; };

  std::vector<os_watch_id_t>
  triggered_watchpoints (const wave_t &wave) const override;

  bool is_pseudo_register_available (const wave_t &wave,
                                     amdgpu_regnum_t regnum) const override;

  void read_pseudo_register (const wave_t &wave, amdgpu_regnum_t regnum,
                             size_t offset, size_t value_size,
                             void *value) const override;

  void write_pseudo_register (wave_t &wave, amdgpu_regnum_t regnum,
                              size_t offset, size_t value_size,
                              const void *value) const override;

  void
  get_wave_state (wave_t &wave, amd_dbgapi_wave_state_t *state,
                  amd_dbgapi_wave_stop_reason_t *stop_reason) const override;
  void set_wave_state (wave_t &wave,
                       amd_dbgapi_wave_state_t state) const override;

  virtual uint32_t os_wave_launch_trap_mask_to_wave_mode (
    os_wave_launch_trap_mask_t mask) const;

  void
  enable_wave_traps (wave_t &wave,
                     os_wave_launch_trap_mask_t mask) const override final;
  void
  disable_wave_traps (wave_t &wave,
                      os_wave_launch_trap_mask_t mask) const override final;

  size_t minimum_instruction_alignment () const override;
  const std::vector<uint8_t> &nop_instruction () const override;
  const std::vector<uint8_t> &breakpoint_instruction () const override;
  const std::vector<uint8_t> &assert_instruction () const override;
  const std::vector<uint8_t> &endpgm_instruction () const override;
  size_t breakpoint_instruction_pc_adjust () const override;

protected:
  /* See https://llvm.org/docs/AMDGPUUsage.html#trap-handler-abi  */
  static constexpr uint8_t assert_trap_id = 0x2;
  static constexpr uint8_t debug_trap_id = 0x3;
  static constexpr uint8_t breakpoint_trap_id = 0x7;

  static uint8_t encoding_ssrc0 (const std::vector<uint8_t> &bytes);
  static uint8_t encoding_ssrc1 (const std::vector<uint8_t> &bytes);
  static uint8_t encoding_sdst (const std::vector<uint8_t> &bytes);
  static uint8_t encoding_op7 (const std::vector<uint8_t> &bytes);
  static int encoding_simm16 (const std::vector<uint8_t> &bytes);

  static bool is_sopk_instruction (const std::vector<uint8_t> &bytes, int op5);
  static bool is_sop1_instruction (const std::vector<uint8_t> &bytes, int op8);
  static bool is_sop2_instruction (const std::vector<uint8_t> &bytes, int op7);
  static bool is_sopp_instruction (const std::vector<uint8_t> &bytes, int op7);

  /* Return the regnum for a scalar register operand.  */
  virtual amdgpu_regnum_t scalar_operand_to_regnum (int operand) const = 0;
  /* Return the number of aliased scalar registers (e.g. vcc, flat_scratch)  */
  virtual size_t scalar_alias_count () const = 0;

  virtual bool is_sethalt (const std::vector<uint8_t> &bytes) const;
  virtual bool is_barrier (const std::vector<uint8_t> &bytes) const;
  virtual bool is_sleep (const std::vector<uint8_t> &bytes) const;
  virtual bool is_code_end (const std::vector<uint8_t> &bytes) const;
  virtual bool is_call (const std::vector<uint8_t> &bytes) const;
  virtual bool is_getpc (const std::vector<uint8_t> &bytes) const;
  virtual bool is_setpc (const std::vector<uint8_t> &bytes) const;
  virtual bool is_swappc (const std::vector<uint8_t> &bytes) const;
  virtual bool is_branch (const std::vector<uint8_t> &bytes) const;
  virtual bool is_cbranch (const std::vector<uint8_t> &bytes) const;
  virtual bool is_cbranch_i_fork (const std::vector<uint8_t> &bytes) const;
  virtual bool is_cbranch_g_fork (const std::vector<uint8_t> &bytes) const;
  virtual bool is_cbranch_join (const std::vector<uint8_t> &bytes) const;
  virtual bool is_nop (const std::vector<uint8_t> &bytes) const;
  virtual bool is_trap (const std::vector<uint8_t> &bytes,
                        uint8_t *trap_id = nullptr) const;

  bool is_endpgm (const std::vector<uint8_t> &bytes) const override;
  bool is_breakpoint (const std::vector<uint8_t> &bytes) const override;

  virtual bool
  is_cbranch_taken (wave_t &wave,
                    const std::vector<uint8_t> &instruction) const;

  virtual amd_dbgapi_global_address_t
  branch_target (wave_t &wave, amd_dbgapi_global_address_t pc,
                 const std::vector<uint8_t> &instruction) const;

  std::tuple<amd_dbgapi_instruction_kind_t,       /* instruction_kind  */
             amd_dbgapi_instruction_properties_t, /* instruction_properties  */
             size_t,                              /* instruction_size  */
             std::vector<uint64_t> /* instruction_information  */>
  classify_instruction (const std::vector<uint8_t> &instruction,
                        amd_dbgapi_global_address_t address) const override;

  bool
  can_execute_displaced (const std::vector<uint8_t> &bytes) const override;
  bool can_simulate (const std::vector<uint8_t> &bytes) const override;

  bool simulate_instruction (
    wave_t &wave, amd_dbgapi_global_address_t pc,
    const std::vector<uint8_t> &instruction) const override;
};

decltype (amdgcn_architecture_t::cbranch_opcodes_map)
  amdgcn_architecture_t::cbranch_opcodes_map{
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

void
amdgcn_architecture_t::initialize ()
{
  /* Create address spaces.  */

  auto &as_global = create<address_space_t> (
    std::make_optional (AMD_DBGAPI_ADDRESS_SPACE_GLOBAL), *this, "global",
    address_space_t::global, DW_ASPACE_none, 64, 0x0000000000000000,
    AMD_DBGAPI_ADDRESS_SPACE_ACCESS_ALL);

  auto &as_generic = create<address_space_t> (
    *this, "generic", address_space_t::generic, DW_ASPACE_AMDGPU_generic, 64,
    0x0000000000000000, AMD_DBGAPI_ADDRESS_SPACE_ACCESS_ALL);

  auto &as_region = create<address_space_t> (
    *this, "region", address_space_t::region, DW_ASPACE_AMDGPU_region, 32,
    0xFFFFFFFF, AMD_DBGAPI_ADDRESS_SPACE_ACCESS_ALL);

  auto &as_local = create<address_space_t> (
    *this, "local", address_space_t::local, DW_ASPACE_AMDGPU_local, 32,
    0xFFFFFFFF, AMD_DBGAPI_ADDRESS_SPACE_ACCESS_ALL);

  auto &as_private_lane = create<address_space_t> (
    *this, "private_lane", address_space_t::private_swizzled,
    DW_ASPACE_AMDGPU_private_lane, 32, 0x00000000,
    AMD_DBGAPI_ADDRESS_SPACE_ACCESS_ALL);

  create<address_space_t> (*this, "private_wave",
                           address_space_t::private_unswizzled,
                           DW_ASPACE_AMDGPU_private_wave, 32, 0x00000000,
                           AMD_DBGAPI_ADDRESS_SPACE_ACCESS_ALL);

  for (int i = 0; i < 63; i++)
    create<address_space_t> (*this, string_printf ("private_lane%d", i),
                             address_space_t::private_swizzled_n,
                             DW_ASPACE_AMDGPU_private_lane0 + i, 32,
                             0x00000000, AMD_DBGAPI_ADDRESS_SPACE_ACCESS_ALL);

  /* Create address classes.  */

  create<address_class_t> (*this, "none", DW_ADDR_none, as_generic);
  create<address_class_t> (*this, "global", DW_ADDR_LLVM_global, as_global);
  create<address_class_t> (*this, "constant", DW_ADDR_LLVM_constant,
                           as_global);
  create<address_class_t> (*this, "group", DW_ADDR_LLVM_group, as_local);
  create<address_class_t> (*this, "private", DW_ADDR_LLVM_private,
                           as_private_lane);
  create<address_class_t> (*this, "region", DW_ADDR_AMDGPU_region, as_region);

  /* Create register classes.  */

  /* Scalar registers: [s0-sN] (sN depends on the gfxip).  */
  register_class_t::register_map_t scalar_registers;
  scalar_registers.emplace (amdgpu_regnum_t::first_sgpr,
                            amdgpu_regnum_t::first_sgpr
                              + scalar_register_count () - 1);
  create<register_class_t> (*this, "scalar", scalar_registers);

  /* Vector registers: [v0-v255, a0-a255]  */
  register_class_t::register_map_t vector_registers;
  if (has_wave32_vgprs ())
    {
      vector_registers.emplace (amdgpu_regnum_t::first_vgpr_32,
                                amdgpu_regnum_t::last_vgpr_32);
      if (has_acc_vgprs ())
        vector_registers.emplace (amdgpu_regnum_t::first_accvgpr_32,
                                  amdgpu_regnum_t::last_accvgpr_32);
    }

  if (has_wave64_vgprs ())
    {
      vector_registers.emplace (amdgpu_regnum_t::first_vgpr_64,
                                amdgpu_regnum_t::last_vgpr_64);
      if (has_acc_vgprs ())
        vector_registers.emplace (amdgpu_regnum_t::first_accvgpr_64,
                                  amdgpu_regnum_t::last_accvgpr_64);
    }
  create<register_class_t> (*this, "vector", vector_registers);

  /* Trap temporary registers: [ttmp4-ttmp11, ttmp13]  */
  register_class_t::register_map_t trap_registers;

  trap_registers.emplace (amdgpu_regnum_t::ttmp4, amdgpu_regnum_t::ttmp11);
  trap_registers.emplace (amdgpu_regnum_t::ttmp13, amdgpu_regnum_t::ttmp13);
  create<register_class_t> (*this, "trap", trap_registers);

  /* System registers: [hwregs, flat_scratch, xnack_mask, vcc]  */
  register_class_t::register_map_t system_registers;

  system_registers.emplace (amdgpu_regnum_t::pseudo_status,
                            amdgpu_regnum_t::pseudo_status);
  system_registers.emplace (amdgpu_regnum_t::mode, amdgpu_regnum_t::mode);
  system_registers.emplace (amdgpu_regnum_t::trapsts,
                            amdgpu_regnum_t::trapsts);
  system_registers.emplace (amdgpu_regnum_t::flat_scratch,
                            amdgpu_regnum_t::flat_scratch);
  if (has_wave32_vgprs ())
    system_registers.emplace (amdgpu_regnum_t::xnack_mask_32,
                              amdgpu_regnum_t::xnack_mask_32);
  if (has_wave64_vgprs ())
    system_registers.emplace (amdgpu_regnum_t::xnack_mask_64,
                              amdgpu_regnum_t::xnack_mask_64);
  create<register_class_t> (*this, "system", system_registers);

  /* General registers: [{scalar}, {vector}, pc, exec, vcc]  */
  register_class_t::register_map_t general_registers;
  general_registers.insert (scalar_registers.begin (),
                            scalar_registers.end ());
  general_registers.insert (vector_registers.begin (),
                            vector_registers.end ());
  general_registers.emplace (amdgpu_regnum_t::m0, amdgpu_regnum_t::m0);
  general_registers.emplace (amdgpu_regnum_t::pc, amdgpu_regnum_t::pc);
  if (has_wave32_vgprs ())
    {
      general_registers.emplace (amdgpu_regnum_t::pseudo_exec_32,
                                 amdgpu_regnum_t::pseudo_exec_32);
      general_registers.emplace (amdgpu_regnum_t::pseudo_vcc_32,
                                 amdgpu_regnum_t::pseudo_vcc_32);
    }
  if (has_wave64_vgprs ())
    {
      general_registers.emplace (amdgpu_regnum_t::pseudo_exec_64,
                                 amdgpu_regnum_t::pseudo_exec_64);
      general_registers.emplace (amdgpu_regnum_t::pseudo_vcc_64,
                                 amdgpu_regnum_t::pseudo_vcc_64);
    }
  create<register_class_t> (*this, "general", general_registers);
}

namespace
{

/* Helper routine to return the address space for a given generic address. The
   returned address space can only be one of LOCAL, PRIVATE_SWIZZLED or GLOBAL
   address space. The queue_t is used to retrieve the apertures.
 */
const address_space_t &
address_space_for_generic_address (
  const wave_t &wave, amd_dbgapi_segment_address_t generic_address)
{
  address_space_t::address_space_kind_t address_space_kind;
  amd_dbgapi_global_address_t aperture
    = generic_address & utils::bit_mask (32, 63);

  if (aperture == wave.agent ().private_address_space_aperture ())
    address_space_kind = address_space_t::private_swizzled;
  else if (aperture == wave.agent ().shared_address_space_aperture ())
    address_space_kind = address_space_t::local;
  else /* all other addresses are treated as global addresses  */
    address_space_kind = address_space_t::global;

  const address_space_t *segment_address_space
    = wave.architecture ().find_if ([=] (const address_space_t &as) {
        return as.kind () == address_space_kind;
      });
  if (!segment_address_space)
    error ("address space not found in architecture");

  return *segment_address_space;
}

/* Helper routine to return the generic address for a given segment address
   space, segment address pair.  Converting an address from an address space
   other than LOCAL, PRIVATE_SWIZZLED or GLOBAL is invalid, and an error is
   returned.  The queue_t is used to retrieve the apertures.
 */
std::optional<amd_dbgapi_segment_address_t>
generic_address_for_address_space (
  const wave_t &wave, const address_space_t &segment_address_space,
  amd_dbgapi_segment_address_t segment_address)
{
  amd_dbgapi_segment_address_t aperture{ 0 };

  if (segment_address_space.kind () == address_space_t::local)
    aperture = wave.agent ().shared_address_space_aperture ();
  if (segment_address_space.kind () == address_space_t::private_swizzled)
    aperture = wave.agent ().private_address_space_aperture ();
  else if (segment_address_space.kind () != address_space_t::global)
    /* not a valid address space conversion.  */
    return {};

  if (segment_address == segment_address_space.null_address ())
    return segment_address_space.null_address ();

  segment_address
    &= utils::bit_mask (0, segment_address_space.address_size () - 1);

  return aperture | segment_address;
}

} /* namespace */

amd_dbgapi_status_t
amdgcn_architecture_t::convert_address_space (
  const wave_t &wave, amd_dbgapi_lane_id_t /* lane_id  */,
  const address_space_t &from_address_space,
  const address_space_t &to_address_space,
  amd_dbgapi_segment_address_t from_address,
  amd_dbgapi_segment_address_t *to_address) const
{
  /* Remove the unused bits from the address.  */
  from_address &= utils::bit_mask (0, from_address_space.address_size () - 1);

  if (from_address_space.kind () == to_address_space.kind ())
    {
      *to_address = from_address;
      return AMD_DBGAPI_STATUS_SUCCESS;
    }

  if (from_address_space.kind () == address_space_t::generic)
    {
      if (from_address == from_address_space.null_address ())
        {
          *to_address = to_address_space.null_address ();
          return AMD_DBGAPI_STATUS_SUCCESS;
        }

      /* Check that the generic from_address is compatible with the
         to_address_space.  */
      if (address_space_for_generic_address (wave, from_address).kind ()
          != to_address_space.kind ())
        return AMD_DBGAPI_STATUS_ERROR_INVALID_ADDRESS_SPACE_CONVERSION;

      *to_address
        = from_address
          & utils::bit_mask (0, to_address_space.address_size () - 1);

      return AMD_DBGAPI_STATUS_SUCCESS;
    }

  /* Other conversions from local, private or global can only be to the generic
     address space.  */

  if (to_address_space.kind () != address_space_t::generic)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ADDRESS_SPACE_CONVERSION;

  auto generic_address = generic_address_for_address_space (
    wave, from_address_space, from_address);
  if (!generic_address)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ADDRESS_SPACE_CONVERSION;

  *to_address = *generic_address;
  return AMD_DBGAPI_STATUS_SUCCESS;
}

void
amdgcn_architecture_t::lower_address_space (
  const wave_t &wave, amd_dbgapi_lane_id_t *lane_id,
  const address_space_t &original_address_space,
  const address_space_t **lowered_address_space,
  amd_dbgapi_segment_address_t original_address,
  amd_dbgapi_segment_address_t *lowered_address) const
{
  if (original_address_space.kind () == address_space_t::generic)
    {
      const address_space_t &segment_address_space
        = address_space_for_generic_address (wave, original_address);

      wave.architecture ().convert_address_space (
        wave, AMD_DBGAPI_LANE_NONE, original_address_space,
        segment_address_space, original_address, lowered_address);

      *lowered_address_space = &segment_address_space;
      return;
    };

  if (original_address_space.kind () == address_space_t::private_swizzled_n)
    {
      uint64_t dwarf_value = original_address_space.dwarf_value ();
      if (dwarf_value >= DW_ASPACE_AMDGPU_private_lane0
          && dwarf_value <= DW_ASPACE_AMDGPU_private_lane63)
        {
          const address_space_t *as_private_lane
            = wave.architecture ().find_if ([=] (const address_space_t &as) {
                return as.kind () == address_space_t::private_swizzled;
              });
          if (!as_private_lane)
            error ("address space `private_lane' not found in architecture");

          *lowered_address_space = as_private_lane;
          *lane_id = dwarf_value - DW_ASPACE_AMDGPU_private_lane0;
          return;
        }
    }

  *lowered_address_space = &original_address_space;
  *lowered_address = original_address;
}

bool
amdgcn_architecture_t::address_is_in_address_class (
  const wave_t &wave, amd_dbgapi_lane_id_t lane_id,
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
     private_swizzled_n       -
     private_unswizzled       -
   */

  /* private_swizzled_n and private_unswizzled addresses are not in any address
     classes.  */
  if (address_space.kind () == address_space_t::private_swizzled_n
      || address_space.kind () == address_space_t::private_unswizzled)
    return false;

  /* private_swizzled, local, global, and generic are in the generic address
     class.  */
  if (address_class.address_space ().kind () == address_space_t::generic)
    return true;

  if (address_space.kind () == address_space_t::generic)
    {
      const address_space_t *lowered_address_space;
      lower_address_space (wave, &lane_id, address_space,
                           &lowered_address_space, segment_address,
                           &segment_address);

      /* A generic private address is in the private address class, a generic
         local address is in the local address class, etc...  */
      return lowered_address_space->kind ()
             == address_class.address_space ().kind ();
    }

  /* The private_swizzled address space is in private address class, the local
     address space is in the local address class, etc...  */
  return address_space.kind () == address_class.address_space ().kind ();
}

bool
amdgcn_architecture_t::address_spaces_may_alias (
  const address_space_t &address_space1,
  const address_space_t &address_space2) const
{
  /* generic aliases with private, local and global.  */
  if (address_space1.kind () == address_space_t::generic
      || address_space2.kind () == address_space_t::generic)
    return true;

  auto is_private = [] (const address_space_t &address_space) {
    return address_space.kind () == address_space_t::private_swizzled
           || address_space.kind () == address_space_t::private_swizzled_n
           || address_space.kind () == address_space_t::private_unswizzled;
  };

  /* private* aliases with private*.  */
  if (is_private (address_space1) && is_private (address_space2))
    return true;

  return false;
}

std::vector<os_watch_id_t>
amdgcn_architecture_t::triggered_watchpoints (const wave_t &wave) const
{
  std::vector<os_watch_id_t> watchpoints;

  if (wave.state () != AMD_DBGAPI_WAVE_STATE_STOP
      || !(wave.stop_reason () & AMD_DBGAPI_WAVE_STOP_REASON_WATCHPOINT))
    return {};

  /* We don't have to suspend the queue because trapsts is a cached hwreg.  */
  dbgapi_assert (wave.register_cache_policy (amdgpu_regnum_t::trapsts)
                 != memory_cache_t::policy_t::uncached);

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
  return sizeof (uint32_t);
}

const std::vector<uint8_t> &
amdgcn_architecture_t::nop_instruction () const
{
  static const std::vector<uint8_t> s_nop_instruction_bytes{
    0x00, 0x00, 0x80, 0xBF /* s_nop 0 */
  };

  return s_nop_instruction_bytes;
}

const std::vector<uint8_t> &
amdgcn_architecture_t::breakpoint_instruction () const
{
  static const std::vector<uint8_t> s_breakpoint_instruction_bytes{
    breakpoint_trap_id, 0x00, 0x92, 0xBF /* s_trap 7 */
  };

  return s_breakpoint_instruction_bytes;
}

const std::vector<uint8_t> &
amdgcn_architecture_t::assert_instruction () const
{
  static const std::vector<uint8_t> s_assert_instruction_bytes{
    assert_trap_id, 0x00, 0x92, 0xBF /* s_trap 2 */
  };

  return s_assert_instruction_bytes;
}

const std::vector<uint8_t> &
amdgcn_architecture_t::endpgm_instruction () const
{
  static const std::vector<uint8_t> s_endpgm_instruction_bytes{
    0x00, 0x00, 0x81, 0xBF /* s_endpgm 0 */
  };

  return s_endpgm_instruction_bytes;
}

size_t
amdgcn_architecture_t::breakpoint_instruction_pc_adjust () const
{
  return 0;
}

bool
amdgcn_architecture_t::is_cbranch_taken (
  wave_t &wave, const std::vector<uint8_t> &instruction) const
{
  if (is_cbranch (instruction))
    {
      uint32_t status_reg;

      wave.read_register (amdgpu_regnum_t::status, &status_reg);

      /* Evaluate the condition.  */
      switch (cbranch_opcodes_map.find (encoding_op7 (instruction))->second)
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
      error ("should not reach here: invalid cbranch_cond_t");
    }

  if (is_cbranch_i_fork (instruction) || is_cbranch_g_fork (instruction))
    {
      dbgapi_assert (wave.lane_count () == 64);

      uint32_t mask_lo, mask_hi;

      amdgpu_regnum_t regnum = scalar_operand_to_regnum (
        is_cbranch_i_fork (instruction) ? encoding_sdst (instruction)
                                        : encoding_ssrc0 (instruction));

      /* The hardware requires a 64-bit address register pair to have the lower
         register number be even.  */
      dbgapi_assert ((regnum & -2) == regnum);

      wave.read_register (regnum + 0, &mask_lo);
      wave.read_register (regnum + 1, &mask_hi);

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

      wave.read_register (amdgpu_regnum_t::csp, &csp);
      wave.read_register (
        scalar_operand_to_regnum (encoding_ssrc0 (instruction)), &mask);

      return csp != mask;
    }

  error ("Invalid instruction");
}

amd_dbgapi_global_address_t
amdgcn_architecture_t::branch_target (
  wave_t &wave, amd_dbgapi_global_address_t pc,
  const std::vector<uint8_t> &instruction) const
{
  if (is_branch (instruction) || is_call (instruction)
      || is_cbranch (instruction) || is_cbranch_i_fork (instruction))
    {
      return pc + instruction.size () + (encoding_simm16 (instruction) << 2);
    }

  if (is_cbranch_g_fork (instruction))
    {
      amdgpu_regnum_t regnum
        = scalar_operand_to_regnum (encoding_ssrc1 (instruction));

      /* The hardware requires a 64-bit address register pair to have the lower
         register number be even.  */
      dbgapi_assert ((regnum & -2) == regnum);

      uint32_t pc_lo, pc_hi;
      wave.read_register (regnum + 0, &pc_lo);
      wave.read_register (regnum + 1, &pc_hi);

      return (static_cast<uint64_t> (pc_hi) << 32) | pc_lo;
    }

  error ("Invalid instruction");
}

std::tuple<amd_dbgapi_instruction_kind_t, amd_dbgapi_instruction_properties_t,
           size_t, std::vector<uint64_t>>
amdgcn_architecture_t::classify_instruction (
  const std::vector<uint8_t> &instruction,
  amd_dbgapi_global_address_t address) const
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

  size_t size = instruction_size (instruction);
  if (!size)
    throw AMD_DBGAPI_STATUS_ERROR_ILLEGAL_INSTRUCTION;

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
      ssrc_regnum = scalar_operand_to_regnum (encoding_ssrc1 (instruction));
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
      ssrc_regnum = scalar_operand_to_regnum (encoding_ssrc0 (instruction));
    }
  else if (is_call (instruction))
    {
      instruction_kind = AMD_DBGAPI_INSTRUCTION_KIND_DIRECT_CALL_REGISTER_PAIR;
      information_kind = information_kind_t::pc_direct;
      sdst_regnum = scalar_operand_to_regnum (encoding_sdst (instruction));
    }
  else if (is_swappc (instruction))
    {
      instruction_kind
        = AMD_DBGAPI_INSTRUCTION_KIND_INDIRECT_CALL_REGISTER_PAIRS;
      information_kind = information_kind_t::pc_indirect;
      ssrc_regnum = scalar_operand_to_regnum (encoding_ssrc0 (instruction));
      sdst_regnum = scalar_operand_to_regnum (encoding_sdst (instruction));
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
  else if (is_sethalt (instruction) && (encoding_simm16 (instruction) & 0x1))
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
  else if (is_code_end (instruction))
    {
      instruction_kind = AMD_DBGAPI_INSTRUCTION_KIND_UNKNOWN;
      information_kind = information_kind_t::none;
    }
  else
    {
      instruction_kind = AMD_DBGAPI_INSTRUCTION_KIND_SEQUENTIAL;
      information_kind = information_kind_t::none;
    }

  std::vector<uint64_t> information;

  if (information_kind == information_kind_t::pc_direct)
    {
      ssize_t branch_offset = encoding_simm16 (instruction) << 2;
      information.emplace_back (address + size + branch_offset);

      if (sdst_regnum.has_value ())
        {
          information.emplace_back (static_cast<uint64_t> (*sdst_regnum) + 0);
          information.emplace_back (static_cast<uint64_t> (*sdst_regnum) + 1);
        }
    }
  else if (information_kind == information_kind_t::pc_indirect)
    {
      dbgapi_assert (ssrc_regnum.has_value ());
      information.emplace_back (static_cast<uint64_t> (*ssrc_regnum) + 0);
      information.emplace_back (static_cast<uint64_t> (*ssrc_regnum) + 1);

      if (sdst_regnum.has_value ())
        {
          information.emplace_back (static_cast<uint64_t> (*sdst_regnum) + 0);
          information.emplace_back (static_cast<uint64_t> (*sdst_regnum) + 1);
        }
    }
  else if (information_kind == information_kind_t::uint8)
    {
      information.emplace_back (
        utils::bit_extract (encoding_simm16 (instruction), 0, 7));
    }

  return { instruction_kind, instruction_properties, size,
           std::move (information) };
}

bool
amdgcn_architecture_t::can_execute_displaced (
  const std::vector<uint8_t> &bytes) const
{
  /* PC relative branch instructions cannot be displaced as a wave cannot be
     halted with a PC pointing at random or unmapped memory if the branch is
     taken.  */
  if (is_branch (bytes) || is_cbranch (bytes) || is_cbranch_i_fork (bytes)
      || is_call (bytes))
    return false;

  /* All PC reading/modifying instructions are simulated, so no attempt is made
     to fixup the state after instruction is displaced-stepped.  */
  return !(is_cbranch_g_fork (bytes) || is_cbranch_join (bytes)
           || is_getpc (bytes) || is_setpc (bytes) || is_swappc (bytes));
}

bool
amdgcn_architecture_t::can_simulate (const std::vector<uint8_t> &bytes) const
{
  /* s_call_b64 must have even aligned sdst.  */
  if (is_call (bytes))
    return !(encoding_sdst (bytes) & 1);

  /* s_getpc_b64 must have even aligned sdst.  */
  if (is_getpc (bytes))
    return !(encoding_sdst (bytes) & 1);

  /* s_setpc_b64 must have even aligned ssrc.  */
  if (is_setpc (bytes))
    return !(encoding_ssrc0 (bytes) & 1);

  /* s_swappc_b64 must have even aligned ssrc and sdst.  */
  if (is_swappc (bytes))
    return !(encoding_ssrc0 (bytes) & 1) && !(encoding_sdst (bytes) & 1);

  /* s_cbranch_i_fork must have even aligned arg0.  */
  if (is_cbranch_i_fork (bytes))
    return !(encoding_sdst (bytes) & 1);

  /* s_cbranch_i_fork must have even aligned arg0 & arg1.  */
  if (is_cbranch_g_fork (bytes))
    return !(encoding_ssrc0 (bytes) & 1) && !(encoding_ssrc1 (bytes) & 1);

  return is_nop (bytes) || is_branch (bytes) || is_cbranch (bytes)
         || is_cbranch_join (bytes) || is_endpgm (bytes);
}

bool
amdgcn_architecture_t::simulate_instruction (
  wave_t &wave, amd_dbgapi_global_address_t pc,
  const std::vector<uint8_t> &instruction) const
{
  uint32_t ttmp7;
  wave.read_register (amdgpu_regnum_t::ttmp7, &ttmp7);

  dbgapi_assert ((ttmp7 & ttmp7_wave_stopped_mask)
                 && "wave must be stopped to simulate instructions");

  /* Don't simulate the instruction if the wave is halted.  */
  if (ttmp7 & ttmp7_saved_status_halt_mask)
    return false;

  amd_dbgapi_global_address_t new_pc;

  if (is_nop (instruction))
    {
      new_pc = pc + nop_instruction ().size ();
    }
  else if (is_endpgm (instruction))
    {
      wave.terminate ();
      return true;
    }
  else if (is_branch (instruction))
    {
      new_pc = branch_target (wave, pc, instruction);
    }
  else if (is_cbranch (instruction))
    {
      new_pc = is_cbranch_taken (wave, instruction)
                 ? branch_target (wave, pc, instruction)
                 : pc + instruction.size ();
    }
  else if (is_cbranch_i_fork (instruction) || is_cbranch_g_fork (instruction))
    {
      dbgapi_assert (wave.lane_count () == 64);

      amdgpu_regnum_t mask_regnum = scalar_operand_to_regnum (
        is_cbranch_i_fork (instruction) ? encoding_sdst (instruction)
                                        : encoding_ssrc0 (instruction));

      /* The hardware requires a 64-bit address register pair to have the lower
         register number be even.  */
      dbgapi_assert ((mask_regnum & -2) == mask_regnum);

      uint32_t mask_lo, mask_hi;
      wave.read_register (mask_regnum + 0, &mask_lo);
      wave.read_register (mask_regnum + 1, &mask_hi);

      uint64_t mask, exec, mask_pass, mask_fail;
      mask = (static_cast<uint64_t> (mask_hi) << 32) | mask_lo;
      wave.read_register (amdgpu_regnum_t::exec_64, &exec);

      mask_pass = mask & exec;
      mask_fail = ~mask & exec;

      if (mask_pass == exec || mask_fail == exec)
        {
          new_pc = is_cbranch_taken (wave, instruction)
                     ? branch_target (wave, pc, instruction)
                     : pc + instruction.size ();
        }
      else
        {
          bool taken = is_cbranch_taken (wave, instruction);

          uint64_t saved_pc = taken ? pc + instruction.size ()
                                    : branch_target (wave, pc, instruction);

          uint32_t saved_exec_lo = taken ? mask_fail : mask_pass;
          uint32_t saved_exec_hi = (taken ? mask_fail : mask_pass) >> 32;
          uint32_t saved_pc_lo = saved_pc;
          uint32_t saved_pc_hi = saved_pc >> 32;

          uint32_t csp;
          wave.read_register (amdgpu_regnum_t::csp, &csp);

          amdgpu_regnum_t regnum = amdgpu_regnum_t::s0 + csp++ * 4;
          wave.write_register (regnum + 0, &saved_exec_lo);
          wave.write_register (regnum + 1, &saved_exec_hi);
          wave.write_register (regnum + 2, &saved_pc_lo);
          wave.write_register (regnum + 3, &saved_pc_hi);

          new_pc = taken ? branch_target (wave, pc, instruction)
                         : pc + instruction.size ();

          wave.write_register (amdgpu_regnum_t::csp, &csp);
          wave.write_register (amdgpu_regnum_t::exec_64,
                               taken ? &mask_pass : &mask_fail);
        }
    }
  else if (is_cbranch_join (instruction))
    {
      dbgapi_assert (wave.lane_count () == 64);

      if (is_cbranch_taken (wave, instruction))
        {
          uint32_t csp;
          wave.read_register (amdgpu_regnum_t::csp, &csp);

          amdgpu_regnum_t regnum = amdgpu_regnum_t::s0 + --csp * 4;

          uint32_t pc_lo, pc_hi, exec_lo, exec_hi;
          wave.read_register (regnum + 0, &exec_lo);
          wave.read_register (regnum + 1, &exec_hi);
          wave.read_register (regnum + 2, &pc_lo);
          wave.read_register (regnum + 3, &pc_hi);

          new_pc = (static_cast<uint64_t> (pc_hi) << 32) | pc_lo;
          uint64_t exec = (static_cast<uint64_t> (exec_hi) << 32) | exec_lo;

          wave.write_register (amdgpu_regnum_t::csp, &csp);
          wave.write_register (amdgpu_regnum_t::exec_64, &exec);
        }
      else
        {
          new_pc = pc + instruction.size ();
        }
    }
  else if (is_call (instruction) || is_getpc (instruction)
           || is_swappc (instruction) || is_setpc (instruction))
    {
      if (!is_setpc (instruction))
        {
          amdgpu_regnum_t sdst_regnum
            = scalar_operand_to_regnum (encoding_sdst (instruction));

          /* The hardware requires a 64-bit address register pair to have the
             lower register number be even.  */
          dbgapi_assert ((sdst_regnum & -2) == sdst_regnum);

          /* If the destination register pair is out of range of the allocated
             registers, then the hardware does no register write.  */
          bool commit_write = wave.is_register_available (sdst_regnum);

          /* A ttmp destination is only writable when in privileged mode.  */
          if (sdst_regnum >= amdgpu_regnum_t::first_ttmp
              && sdst_regnum <= amdgpu_regnum_t::last_ttmp)
            {
              uint32_t status;
              wave.read_register (amdgpu_regnum_t::status, &status);
              commit_write = (status & sq_wave_status_priv_mask) != 0;
            }

          uint64_t sdst_value = pc + instruction.size ();
          uint32_t sdst_lo = static_cast<uint32_t> (sdst_value);
          uint32_t sdst_hi = static_cast<uint32_t> (sdst_value >> 32);

          if (commit_write)
            {
              wave.write_register (sdst_regnum + 0, &sdst_lo);
              wave.write_register (sdst_regnum + 1, &sdst_hi);
            }

          new_pc = is_call (instruction)
                     ? branch_target (wave, pc, instruction)
                     : pc + instruction.size ();
        }

      if (is_setpc (instruction) || is_swappc (instruction))
        {
          amdgpu_regnum_t ssrc_regnum
            = scalar_operand_to_regnum (encoding_ssrc0 (instruction));

          /* The hardware requires a 64-bit address register pair to have the
             lower register number be even.  */
          dbgapi_assert ((ssrc_regnum & -2) == ssrc_regnum);

          /* If the source register pair is out of range of the allocated
             registers, then the hardware reads from s[0:1].  */
          if (!wave.is_register_available (ssrc_regnum))
            ssrc_regnum = amdgpu_regnum_t::s0;

          bool ssrc_is_null = ssrc_regnum == amdgpu_regnum_t::null;

          /* Reading a ttmp source when not in priviledged mode returns 0.  */
          if (ssrc_regnum >= amdgpu_regnum_t::first_ttmp
              && ssrc_regnum <= amdgpu_regnum_t::last_ttmp)
            {
              uint32_t status;
              wave.read_register (amdgpu_regnum_t::status, &status);
              ssrc_is_null = !(status & sq_wave_status_priv_mask);
            }

          uint32_t ssrc_lo = 0, ssrc_hi = 0;

          if (!ssrc_is_null)
            {
              wave.read_register (ssrc_regnum + 0, &ssrc_lo);
              wave.read_register (ssrc_regnum + 1, &ssrc_hi);
            }

          new_pc = amd_dbgapi_global_address_t{ ssrc_lo }
                   | amd_dbgapi_global_address_t{ ssrc_hi } << 32;
        }
    }
  else
    {
      /* We don't know how to simulate this instruction.  */
      dbgapi_assert (!"Cannot simulate instruction");
    }

  wave.write_register (amdgpu_regnum_t::pc, &new_pc);

  dbgapi_log (AMD_DBGAPI_LOG_LEVEL_INFO, "%s simulated \"%s\" (pc=%#lx)",
              to_string (wave.id ()).c_str (),
              std::get<1> (wave.architecture ().disassemble_instruction (
                             pc, instruction.data (), instruction.size ()))
                .c_str (),
              pc);

  return true;
}

void
amdgcn_architecture_t::get_wave_state (
  wave_t &wave, amd_dbgapi_wave_state_t *state,
  amd_dbgapi_wave_stop_reason_t *stop_reason) const
{
  dbgapi_assert (state && stop_reason && "Invalid parameter");

  uint32_t ttmp7, mode_reg;
  wave.read_register (amdgpu_regnum_t::ttmp7, &ttmp7);
  wave.read_register (amdgpu_regnum_t::mode, &mode_reg);

  amd_dbgapi_wave_state_t prev_state = wave.state ();

  amd_dbgapi_wave_state_t new_state
    = (ttmp7 & ttmp7_wave_stopped_mask)
        ? AMD_DBGAPI_WAVE_STATE_STOP
        : ((mode_reg & sq_wave_mode_debug_en_mask)
             ? AMD_DBGAPI_WAVE_STATE_SINGLE_STEP
             : AMD_DBGAPI_WAVE_STATE_RUN);

  if (new_state != AMD_DBGAPI_WAVE_STATE_STOP)
    {
      /* The wave is running, there is no stop reason.  */
      *state = new_state;
      *stop_reason = AMD_DBGAPI_WAVE_STOP_REASON_NONE;
      return;
    }

  if (prev_state == AMD_DBGAPI_WAVE_STATE_STOP)
    {
      /* The wave was previously stopped, and it still is stopped,
         the stop reason is unchanged.  */
      *state = new_state;
      *stop_reason = wave.stop_reason ();
      return;
    }

  /* The wave is stopped, but it was previously running.  */

  mode_reg &= ~sq_wave_mode_debug_en_mask;
  wave.write_register (amdgpu_regnum_t::mode, &mode_reg);

  amd_dbgapi_wave_stop_reason_t reason_mask
    = (prev_state == AMD_DBGAPI_WAVE_STATE_SINGLE_STEP)
        ? AMD_DBGAPI_WAVE_STOP_REASON_SINGLE_STEP
        : AMD_DBGAPI_WAVE_STOP_REASON_NONE;

  uint32_t trapsts;
  wave.read_register (amdgpu_regnum_t::trapsts, &trapsts);

  amd_dbgapi_global_address_t pc = wave.pc ();

  if (ttmp7 & ttmp7_dispatch_id_converted_mask)
    {
      /* The trap handler "parked" the wave and saved the PC in ttmp11[22:7]
         and ttmp6[31:0]  */

      uint32_t ttmp6, ttmp11;
      wave.read_register (amdgpu_regnum_t::ttmp6, &ttmp6);
      wave.read_register (amdgpu_regnum_t::ttmp11, &ttmp11);

      pc = static_cast<amd_dbgapi_global_address_t> (ttmp6)
           | static_cast<amd_dbgapi_global_address_t> (
               utils::bit_extract (ttmp11, 7, 22))
               << 32;
      wave.write_register (amdgpu_regnum_t::pc, &pc);
    }

  if ((trapsts & sq_wave_trapsts_excp_mem_viol_mask
       || trapsts & sq_wave_trapsts_illegal_inst_mask)
      /* FIXME: If the wave was single-stepping when the exception occurred,
         the first level trap handler did not decrement the PC as it took the
         SINGLE_STEP_WORKAROUND path.  */
      && prev_state != AMD_DBGAPI_WAVE_STATE_SINGLE_STEP)
    {
      /* The first-level trap handler subtracts 8 from the PC, so we add it
         back here.  */
      pc += 8;
      wave.write_register (amdgpu_regnum_t::pc, &pc);
    }

  /* Check for traps caused by an s_trap instruction.  */
  switch (ttmp7_saved_trap_id (ttmp7))
    {
    case 0:
      break;
    case 2:
      reason_mask |= AMD_DBGAPI_WAVE_STOP_REASON_ASSERT_TRAP;
      break;
    case 3:
      reason_mask |= AMD_DBGAPI_WAVE_STOP_REASON_DEBUG_TRAP;
      break;
    case 7:
      reason_mask |= AMD_DBGAPI_WAVE_STOP_REASON_BREAKPOINT;
      break;
    default:
      reason_mask |= AMD_DBGAPI_WAVE_STOP_REASON_TRAP;
      break;
    }

  /* Check for exceptions.  */
  if (trapsts & sq_wave_trapsts_excp_invalid_mask)
    reason_mask |= AMD_DBGAPI_WAVE_STOP_REASON_FP_INVALID_OPERATION;
  if (trapsts & sq_wave_trapsts_excp_input_denorm_mask)
    reason_mask |= AMD_DBGAPI_WAVE_STOP_REASON_FP_INPUT_DENORMAL;
  if (trapsts & sq_wave_trapsts_excp_div0_mask)
    reason_mask |= AMD_DBGAPI_WAVE_STOP_REASON_FP_DIVIDE_BY_0;
  if (trapsts & sq_wave_trapsts_excp_overflow_mask)
    reason_mask |= AMD_DBGAPI_WAVE_STOP_REASON_FP_OVERFLOW;
  if (trapsts & sq_wave_trapsts_excp_underflow_mask)
    reason_mask |= AMD_DBGAPI_WAVE_STOP_REASON_FP_UNDERFLOW;
  if (trapsts & sq_wave_trapsts_excp_inexact_mask)
    reason_mask |= AMD_DBGAPI_WAVE_STOP_REASON_FP_INEXACT;
  if (trapsts & sq_wave_trapsts_excp_int_div0_mask)
    reason_mask |= AMD_DBGAPI_WAVE_STOP_REASON_INT_DIVIDE_BY_0;
  if (trapsts & sq_wave_trapsts_excp_mem_viol_mask)
    reason_mask |= AMD_DBGAPI_WAVE_STOP_REASON_MEMORY_VIOLATION;
  if (trapsts & sq_wave_trapsts_illegal_inst_mask)
    reason_mask |= AMD_DBGAPI_WAVE_STOP_REASON_ILLEGAL_INSTRUCTION;
  if (trapsts
      & (sq_wave_trapsts_excp_addr_watch0_mask
         | sq_wave_trapsts_excp_hi_addr_watch1_mask
         | sq_wave_trapsts_excp_hi_addr_watch2_mask
         | sq_wave_trapsts_excp_hi_addr_watch3_mask))
    reason_mask |= AMD_DBGAPI_WAVE_STOP_REASON_WATCHPOINT;

  /* Check for spurious single-step events. A context save/restore before
     executing the single-stepped instruction could have caused the event to be
     reported with the wave halted at the instruction instead of after.  In
     such cases, un-halt the wave and let it resume in single-step mode, so
     that the instruction is executed.  There should be no other exception to
     report.  */
  if (reason_mask == AMD_DBGAPI_WAVE_STOP_REASON_SINGLE_STEP
      && pc == wave.last_stopped_pc ())
    {
      /* Branch instructions must be simulated and the event reported, as we
         cannot tell if a branch to self has executed.  */
      if (auto instruction = wave.instruction_at_pc ();
          instruction && can_simulate (*instruction))
        {
          /* Trim to size of instruction, simulate_instruction needs the
             exact instruction bytes.  */
          instruction->resize (instruction_size (*instruction));

          if (!simulate_instruction (wave, pc, *instruction))
            dbgapi_assert (!"halted waves cannot raise spurious events");

          /* The instruction was simulated, report the single-step event.
           */
          *state = AMD_DBGAPI_WAVE_STATE_STOP;
          *stop_reason = AMD_DBGAPI_WAVE_STOP_REASON_SINGLE_STEP;
          return;
        }

      dbgapi_log (AMD_DBGAPI_LOG_LEVEL_INFO,
                  "%s (pc=%#lx) ignore spurious single-step",
                  to_string (wave.id ()).c_str (), pc);

      /* Place the wave back into single-stepping state.  */
      set_wave_state (wave, AMD_DBGAPI_WAVE_STATE_SINGLE_STEP);

      *state = AMD_DBGAPI_WAVE_STATE_SINGLE_STEP;
      *stop_reason = AMD_DBGAPI_WAVE_STOP_REASON_NONE;
      return;
    }

  *state = new_state;
  *stop_reason = reason_mask;
}

void
amdgcn_architecture_t::set_wave_state (wave_t &wave,
                                       amd_dbgapi_wave_state_t state) const
{
  uint32_t status_reg, mode_reg, ttmp7;

  wave.read_register (amdgpu_regnum_t::status, &status_reg);
  wave.read_register (amdgpu_regnum_t::mode, &mode_reg);
  wave.read_register (amdgpu_regnum_t::ttmp7, &ttmp7);

  switch (state)
    {
    case AMD_DBGAPI_WAVE_STATE_STOP:
      /* Put the wave in the stop state (ttmp7.wave_stopped=1), save
         status.halt in ttmp7.saved_status_halt, and halt the wave
         (status.halt=1).  */
      ttmp7 &= ~(ttmp7_wave_stopped_mask | ttmp7_saved_status_halt_mask
                 | ttmp7_saved_trap_id_mask);

      if (status_reg & sq_wave_status_halt_mask)
        ttmp7 |= ttmp7_saved_status_halt_mask;
      ttmp7 |= ttmp7_wave_stopped_mask;

      mode_reg &= ~sq_wave_mode_debug_en_mask;

      status_reg |= sq_wave_status_halt_mask;
      break;

    case AMD_DBGAPI_WAVE_STATE_RUN:
      /* Restore status.halt from ttmp7.saved_status_halt, put the wave in the
         run state (ttmp7.wave_stopped=0), and set mode.debug_en=0.  */
      mode_reg &= ~sq_wave_mode_debug_en_mask;

      status_reg &= ~sq_wave_status_halt_mask;
      if (ttmp7 & ttmp7_saved_status_halt_mask)
        status_reg |= sq_wave_status_halt_mask;

      ttmp7 &= ~(ttmp7_wave_stopped_mask | ttmp7_saved_status_halt_mask);
      break;

    case AMD_DBGAPI_WAVE_STATE_SINGLE_STEP:
      /* Restore status.halt from ttmp7.saved_status_halt, put the wave in the
         run state (ttmp7.wave_stopped=0), and set mode.debug_en=1.  */
      mode_reg |= sq_wave_mode_debug_en_mask;

      status_reg &= ~sq_wave_status_halt_mask;
      if (ttmp7 & ttmp7_saved_status_halt_mask)
        status_reg |= sq_wave_status_halt_mask;

      ttmp7 &= ~(ttmp7_wave_stopped_mask | ttmp7_saved_status_halt_mask);
      break;

    default:
      dbgapi_assert (!"Invalid wave state");
    }

  wave.write_register (amdgpu_regnum_t::status, &status_reg);
  wave.write_register (amdgpu_regnum_t::mode, &mode_reg);
  wave.write_register (amdgpu_regnum_t::ttmp7, &ttmp7);

  /* If resuming the wave (run or single-step), clear the watchpoint exceptions
     in trapsts.  */
  if (state != AMD_DBGAPI_WAVE_STATE_STOP
      && wave.state () == AMD_DBGAPI_WAVE_STATE_STOP
      && wave.stop_reason () & AMD_DBGAPI_WAVE_STOP_REASON_WATCHPOINT)
    {
      uint32_t trapsts;
      wave.read_register (amdgpu_regnum_t::trapsts, &trapsts);

      trapsts &= ~(sq_wave_trapsts_excp_addr_watch0_mask
                   | sq_wave_trapsts_excp_hi_addr_watch1_mask
                   | sq_wave_trapsts_excp_hi_addr_watch2_mask
                   | sq_wave_trapsts_excp_hi_addr_watch3_mask);

      wave.write_register (amdgpu_regnum_t::trapsts, &trapsts);
    }
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
amdgcn_architecture_t::enable_wave_traps (
  wave_t &wave, os_wave_launch_trap_mask_t mask) const
{
  uint32_t mode;
  wave.read_register (amdgpu_regnum_t::mode, &mode);

  /* OR SQ_WAVE_MODE.EXCP_EN with mask.  */
  mode |= os_wave_launch_trap_mask_to_wave_mode (mask);

  wave.write_register (amdgpu_regnum_t::mode, &mode);
}

void
amdgcn_architecture_t::disable_wave_traps (
  wave_t &wave, os_wave_launch_trap_mask_t mask) const
{
  uint32_t mode;

  wave.read_register (amdgpu_regnum_t::mode, &mode);

  /* AND SQ_WAVE_MODE.EXCP_EN with ~mask.  */
  mode &= ~os_wave_launch_trap_mask_to_wave_mode (mask);

  wave.write_register (amdgpu_regnum_t::mode, &mode);
}

uint8_t
amdgcn_architecture_t::encoding_ssrc0 (const std::vector<uint8_t> &bytes)
{
  uint32_t encoding = *reinterpret_cast<const uint32_t *> (bytes.data ());
  return utils::bit_extract (encoding, 0, 7);
}

uint8_t
amdgcn_architecture_t::encoding_ssrc1 (const std::vector<uint8_t> &bytes)
{
  uint32_t encoding = *reinterpret_cast<const uint32_t *> (bytes.data ());
  return utils::bit_extract (encoding, 8, 15);
}

uint8_t
amdgcn_architecture_t::encoding_sdst (const std::vector<uint8_t> &bytes)
{
  uint32_t encoding = *reinterpret_cast<const uint32_t *> (bytes.data ());
  return utils::bit_extract (encoding, 16, 22);
}

uint8_t
amdgcn_architecture_t::encoding_op7 (const std::vector<uint8_t> &bytes)
{
  uint32_t encoding = *reinterpret_cast<const uint32_t *> (bytes.data ());
  return utils::bit_extract (encoding, 16, 22);
}

int
amdgcn_architecture_t::encoding_simm16 (const std::vector<uint8_t> &bytes)
{
  uint32_t encoding = *reinterpret_cast<const uint32_t *> (bytes.data ());
  return utils::sign_extend (utils::bit_extract (encoding, 0, 15), 16);
}

bool
amdgcn_architecture_t::is_sopk_instruction (const std::vector<uint8_t> &bytes,
                                            int op5)
{
  if (bytes.size () < sizeof (uint32_t))
    return false;

  uint32_t encoding = *reinterpret_cast<const uint32_t *> (bytes.data ());

  /* SOPK [1011 OP5 SDST7 SIMM16] */
  return (encoding & 0xFF800000) == (0xB0000000 | (op5 & 0x1F) << 23);
}

bool
amdgcn_architecture_t::is_sop1_instruction (const std::vector<uint8_t> &bytes,
                                            int op8)
{
  if (bytes.size () < sizeof (uint32_t))
    return false;

  uint32_t encoding = *reinterpret_cast<const uint32_t *> (bytes.data ());

  /* SOP1 [10111110 1 SDST7 OP8 SSRC08] */
  return (encoding & 0xFF80FF00) == (0xBE800000 | (op8 & 0xFF) << 8);
}

bool
amdgcn_architecture_t::is_sop2_instruction (const std::vector<uint8_t> &bytes,
                                            int op7)
{
  if (bytes.size () < sizeof (uint32_t))
    return false;

  uint32_t encoding = *reinterpret_cast<const uint32_t *> (bytes.data ());

  /* SOP2 [10 OP7 SDST7 SSRC18 SSRC08] */
  return (encoding & 0xFF800000) == (0x80000000 | (op7 & 0x7F) << 23);
}

bool
amdgcn_architecture_t::is_sopp_instruction (const std::vector<uint8_t> &bytes,
                                            int op7)
{
  if (bytes.size () < sizeof (uint32_t))
    return false;

  uint32_t encoding = *reinterpret_cast<const uint32_t *> (bytes.data ());

  /* SOPP [101111111 OP7 SIMM16]  */
  return (encoding & 0xFFFF0000) == (0xBF800000 | (op7 & 0x7F) << 16);
}

bool
amdgcn_architecture_t::is_nop (const std::vector<uint8_t> &bytes) const
{
  /* s_nop: SOPP Opcode 0  */
  return is_sopp_instruction (bytes, 0);
}

bool
amdgcn_architecture_t::is_endpgm (const std::vector<uint8_t> &bytes) const
{
  /* s_endpgm: SOPP Opcode 1  */
  return is_sopp_instruction (bytes, 1);
}

bool
amdgcn_architecture_t::is_breakpoint (const std::vector<uint8_t> &bytes) const
{
  uint8_t trap_id = 0;
  return is_trap (bytes, &trap_id) && trap_id == breakpoint_trap_id;
}

bool
amdgcn_architecture_t::is_trap (const std::vector<uint8_t> &bytes,
                                uint8_t *trap_id) const
{
  /* s_trap: SOPP Opcode 18  */
  if (is_sopp_instruction (bytes, 18))
    {
      if (trap_id)
        *trap_id = utils::bit_extract (encoding_simm16 (bytes), 0, 7);

      return true;
    }
  return false;
}

bool
amdgcn_architecture_t::is_sethalt (const std::vector<uint8_t> &bytes) const
{
  /* s_sethalt: SOPP Opcode 13  */
  return is_sopp_instruction (bytes, 13);
}

bool
amdgcn_architecture_t::is_barrier (const std::vector<uint8_t> &bytes) const
{
  /* s_barrier: SOPP Opcode 10  */
  return is_sopp_instruction (bytes, 10);
}

bool
amdgcn_architecture_t::is_sleep (const std::vector<uint8_t> &bytes) const
{
  /* s_sleep: SOPP Opcode 14  */
  return is_sopp_instruction (bytes, 14);
}

bool
amdgcn_architecture_t::is_code_end (
  const std::vector<uint8_t> & /* bytes  */) const
{
  return false;
}

bool
amdgcn_architecture_t::is_call (const std::vector<uint8_t> &bytes) const
{
  /* s_call: SOPK Opcode 21  */
  return is_sopk_instruction (bytes, 21);
}

bool
amdgcn_architecture_t::is_getpc (const std::vector<uint8_t> &bytes) const
{
  /* s_getpc: SOP1 Opcode 28  */
  return is_sop1_instruction (bytes, 28);
}

bool
amdgcn_architecture_t::is_setpc (const std::vector<uint8_t> &bytes) const
{
  /* s_setpc: SOP1 Opcode 29  */
  return is_sop1_instruction (bytes, 29);
}

bool
amdgcn_architecture_t::is_swappc (const std::vector<uint8_t> &bytes) const
{
  /* s_swappc: SOP1 Opcode 30  */
  return is_sop1_instruction (bytes, 30);
}

bool
amdgcn_architecture_t::is_branch (const std::vector<uint8_t> &bytes) const
{
  /* s_sleep: SOPP Opcode 2  */
  return is_sopp_instruction (bytes, 2);
}

bool
amdgcn_architecture_t::is_cbranch (const std::vector<uint8_t> &bytes) const
{
  if (bytes.size () < sizeof (uint32_t))
    return false;

  uint32_t encoding = *reinterpret_cast<const uint32_t *> (bytes.data ());

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
  if ((encoding & 0xFF800000) != 0xBF800000)
    return false;

  return cbranch_opcodes_map.find (encoding_op7 (bytes))
         != cbranch_opcodes_map.end ();
}

bool
amdgcn_architecture_t::is_cbranch_i_fork (
  const std::vector<uint8_t> &bytes) const
{
  /* s_cbranch_i_fork: SOPK Opcode 16  */
  return is_sopk_instruction (bytes, 16);
}

bool
amdgcn_architecture_t::is_cbranch_g_fork (
  const std::vector<uint8_t> &bytes) const
{
  /* s_cbranch_g_fork: SOP2 Opcode 41  */
  return is_sop2_instruction (bytes, 41);
}

bool
amdgcn_architecture_t::is_cbranch_join (
  const std::vector<uint8_t> &bytes) const
{
  /* s_cbranch_join: SOP1 Opcode 46  */
  return is_sop1_instruction (bytes, 46);
}

bool
amdgcn_architecture_t::is_pseudo_register_available (
  const wave_t &wave, amdgpu_regnum_t regnum) const
{
  dbgapi_assert (is_pseudo_register (regnum));

  size_t lane_count = wave.lane_count ();

  if ((regnum == amdgpu_regnum_t::pseudo_exec_32
       || regnum == amdgpu_regnum_t::pseudo_vcc_32)
      && lane_count != 32)
    return false;

  if ((regnum == amdgpu_regnum_t::pseudo_exec_64
       || regnum == amdgpu_regnum_t::pseudo_vcc_64)
      && lane_count != 64)
    return false;

  return true;
}

void
amdgcn_architecture_t::read_pseudo_register (const wave_t &wave,
                                             amdgpu_regnum_t regnum,
                                             size_t offset, size_t value_size,
                                             void *value) const
{
  dbgapi_assert (is_pseudo_register (regnum));

  auto reg_size = register_size (regnum);

  if (!reg_size)
    throw exception_t (AMD_DBGAPI_STATUS_ERROR_INVALID_REGISTER_ID);

  if (!value_size || (offset + value_size) > *reg_size)
    throw exception_t (AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT_COMPATIBILITY);

  size_t lane_count = wave.lane_count ();

  if (regnum == amdgpu_regnum_t::null)
    {
      memset (static_cast<char *> (value) + offset, '\0', value_size);
      return;
    }

  if (regnum == amdgpu_regnum_t::pseudo_exec_32 && lane_count == 32)
    {
      wave.read_register (amdgpu_regnum_t::exec_32, offset, value_size, value);
      return;
    }

  if (regnum == amdgpu_regnum_t::pseudo_exec_64 && lane_count == 64)
    {
      wave.read_register (amdgpu_regnum_t::exec_64, offset, value_size, value);
      return;
    }

  if (regnum == amdgpu_regnum_t::pseudo_vcc_32 && lane_count == 32)
    {
      wave.read_register (amdgpu_regnum_t::vcc_32, offset, value_size, value);
      return;
    }

  if (regnum == amdgpu_regnum_t::pseudo_vcc_64 && lane_count == 64)
    {
      wave.read_register (amdgpu_regnum_t::vcc_64, offset, value_size, value);
      return;
    }

  if (regnum == amdgpu_regnum_t::pseudo_status)
    {
      /* pseudo_status is a composite of: sq_wave_status[31:14], ttmp7[29]
         (halt), sq_wave_status [12:6], 0[0] (priv), sq_wave_status [4:0].  */

      uint32_t ttmp7, status_reg;

      wave.read_register (amdgpu_regnum_t::status, &status_reg);
      wave.read_register (amdgpu_regnum_t::ttmp7, &ttmp7);

      status_reg &= ~(sq_wave_status_priv_mask | sq_wave_status_halt_mask);
      if (ttmp7 & ttmp7_saved_status_halt_mask)
        status_reg |= sq_wave_status_halt_mask;

      memcpy (static_cast<char *> (value) + offset,
              reinterpret_cast<const char *> (&status_reg) + offset,
              value_size);
      return;
    }

  if (regnum == amdgpu_regnum_t::wave_in_group)
    {
      uint32_t ttmp11;

      wave.read_register (amdgpu_regnum_t::ttmp11, &ttmp11);
      ttmp11 &= ttmp11_wave_in_group_mask;

      memcpy (static_cast<char *> (value) + offset,
              reinterpret_cast<const char *> (&ttmp11) + offset, value_size);
      return;
    }

  if (regnum == amdgpu_regnum_t::csp)
    {
      uint32_t mode;

      wave.read_register (amdgpu_regnum_t::mode, &mode);

      uint32_t csp = utils::bit_extract (mode, 29, 31);

      memcpy (static_cast<char *> (value) + offset,
              reinterpret_cast<const char *> (&csp) + offset, value_size);
      return;
    }

  throw exception_t (AMD_DBGAPI_STATUS_ERROR_INVALID_REGISTER_ID);
}

void
amdgcn_architecture_t::write_pseudo_register (wave_t &wave,
                                              amdgpu_regnum_t regnum,
                                              size_t offset, size_t value_size,
                                              const void *value) const
{
  dbgapi_assert (is_pseudo_register (regnum));

  auto reg_size = register_size (regnum);

  if (!reg_size)
    throw exception_t (AMD_DBGAPI_STATUS_ERROR_INVALID_REGISTER_ID);

  if (!value_size || (offset + value_size) > *reg_size)
    throw exception_t (AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT_COMPATIBILITY);

  size_t lane_count = wave.lane_count ();

  if (regnum == amdgpu_regnum_t::null
      || regnum == amdgpu_regnum_t::wave_in_group)
    /* Writing to these registers is a no-op.  */
    return;

  if ((regnum == amdgpu_regnum_t::pseudo_exec_32
       || regnum == amdgpu_regnum_t::pseudo_vcc_32)
      && lane_count == 32)
    {
      uint32_t base_reg, status_reg;

      amdgpu_regnum_t base_regnum = regnum == amdgpu_regnum_t::pseudo_exec_32
                                      ? amdgpu_regnum_t::exec_32
                                      : amdgpu_regnum_t::vcc_32;
      uint32_t status_mask = regnum == amdgpu_regnum_t::pseudo_exec_32
                               ? sq_wave_status_execz_mask
                               : sq_wave_status_vccz_mask;

      wave.read_register (amdgpu_regnum_t::status, &status_reg);
      wave.read_register (base_regnum, &base_reg);

      memcpy (reinterpret_cast<char *> (&base_reg) + offset,
              static_cast<const char *> (value) + offset, value_size);

      status_reg
        = (status_reg & ~status_mask) | (base_reg == 0 ? status_mask : 0);

      wave.write_register (amdgpu_regnum_t::status, &status_reg);
      wave.write_register (base_regnum, &base_reg);
      return;
    }

  if ((regnum == amdgpu_regnum_t::pseudo_exec_64
       || regnum == amdgpu_regnum_t::pseudo_vcc_64)
      && lane_count == 64)
    {
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

      memcpy (reinterpret_cast<char *> (&base_reg) + offset,
              static_cast<const char *> (value) + offset, value_size);

      status_reg
        = (status_reg & ~status_mask) | (base_reg == 0 ? status_mask : 0);

      wave.write_register (amdgpu_regnum_t::status, &status_reg);
      wave.write_register (base_regnum, &base_reg);
      return;
    }

  if (regnum == amdgpu_regnum_t::pseudo_status)
    {
      /* pseudo_status is a composite of: status[31:14], ttmp7[29] (halt),
         status [12:6], 0[0] (priv), status [4:0].  */

      uint32_t prev_status_reg, status_reg, ttmp7;

      /* Only some fields of the status register are writable.  */
      constexpr uint32_t writable_fields_mask
        = utils::bit_mask (0, 4)      /* scc, spi_prio, user_prio  */
          | utils::bit_mask (8, 8)    /* export_rdy  */
          | utils::bit_mask (13, 13)  /* halt  */
          | utils::bit_mask (17, 18)  /* ecc_err, skip_export  */
          | utils::bit_mask (20, 21)  /* cond_dbg_user, cond_dbg_sys  */
          | utils::bit_mask (27, 27); /* must_export  */

      wave.read_register (amdgpu_regnum_t::status, &prev_status_reg);
      wave.read_register (amdgpu_regnum_t::ttmp7, &ttmp7);

      status_reg = prev_status_reg;
      memcpy (reinterpret_cast<char *> (&status_reg) + offset,
              static_cast<const char *> (value) + offset, value_size);

      /* We should only modify the writable bits.  */
      status_reg = (status_reg & writable_fields_mask)
                   | (prev_status_reg & ~writable_fields_mask);

      ttmp7 &= ~ttmp7_saved_status_halt_mask;
      if (status_reg & sq_wave_status_halt_mask)
        ttmp7 |= ttmp7_saved_status_halt_mask;

      wave.write_register (amdgpu_regnum_t::status, &status_reg);
      wave.write_register (amdgpu_regnum_t::ttmp7, &ttmp7);
      return;
    }

  if (regnum == amdgpu_regnum_t::csp)
    {
      uint32_t mode, csp;

      wave.read_register (amdgpu_regnum_t::mode, &mode);

      csp = utils::bit_extract (mode, 29, 31);
      memcpy (reinterpret_cast<char *> (&csp) + offset,
              static_cast<const char *> (value) + offset, value_size);

      mode = (mode & ~utils::bit_mask (29, 31)) | (csp << 29);

      wave.write_register (amdgpu_regnum_t::mode, &mode);
      return;
    }

  throw exception_t (AMD_DBGAPI_STATUS_ERROR_INVALID_REGISTER_ID);
}

/* Base class for all GFX9 architectures.  */

class gfx9_base_t : public amdgcn_architecture_t
{
protected:
  class cwsr_record_t : public architecture_t::cwsr_record_t
  {
  protected:
    uint32_t const m_compute_relaunch_wave;
    uint32_t const m_compute_relaunch_state;
    amd_dbgapi_global_address_t const m_context_save_address;

    static constexpr uint32_t compute_relaunch_payload_vgprs (uint32_t x)
    {
      return utils::bit_extract (x, 0, 5);
    }
    static constexpr uint32_t compute_relaunch_payload_sgprs (uint32_t x)
    {
      return utils::bit_extract (x, 6, 8);
    }
    static constexpr uint32_t compute_relaunch_payload_lds_size (uint32_t x)
    {
      return utils::bit_extract (x, 9, 17);
    }
    static constexpr uint32_t compute_relaunch_payload_last_wave (uint32_t x)
    {
      return utils::bit_extract (x, 16, 16);
    }
    static constexpr uint32_t compute_relaunch_payload_first_wave (uint32_t x)
    {
      return utils::bit_extract (x, 17, 17);
    }

  public:
    cwsr_record_t (queue_t &queue, uint32_t compute_relaunch_wave,
                   uint32_t compute_relaunch_state,
                   amd_dbgapi_global_address_t context_save_address)
      : architecture_t::cwsr_record_t (queue),
        m_compute_relaunch_wave (compute_relaunch_wave),
        m_compute_relaunch_state (compute_relaunch_state),
        m_context_save_address (context_save_address)
    {
    }

    uint64_t get_info (query_kind_t query) const override;

    std::optional<amd_dbgapi_global_address_t>
    register_address (amdgpu_regnum_t regnum) const override;
  };

  virtual std::unique_ptr<architecture_t::cwsr_record_t>
  make_gfx9_cwsr_record (
    queue_t &queue, uint32_t compute_relaunch_wave,
    uint32_t compute_relaunch_state,
    amd_dbgapi_global_address_t context_save_address) const
  {
    return std::make_unique<cwsr_record_t> (queue, compute_relaunch_wave,
                                            compute_relaunch_state,
                                            context_save_address);
  }

  amdgpu_regnum_t scalar_operand_to_regnum (int operand) const override;

  gfx9_base_t (elf_amdgpu_machine_t e_machine, std::string target_triple)
    : amdgcn_architecture_t (e_machine, std::move (target_triple))
  {
  }

  size_t scalar_register_count () const override { return 102; }
  size_t scalar_alias_count () const override { return 6; }

public:
  bool has_wave32_vgprs () const override { return false; }
  bool has_wave64_vgprs () const override { return true; }
  bool has_acc_vgprs () const override { return false; }

  bool can_halt_at_endpgm () const override { return false; }
  size_t largest_instruction_size () const override { return 8; }

  void control_stack_iterate (
    queue_t &queue, const uint32_t *control_stack, size_t control_stack_words,
    amd_dbgapi_global_address_t wave_area_address,
    amd_dbgapi_size_t wave_area_size,
    const std::function<void (std::unique_ptr<architecture_t::cwsr_record_t>)>
      &wave_callback) const override;

  amd_dbgapi_global_address_t dispatch_packet_address (
    const architecture_t::cwsr_record_t &cwsr_record) const override;

  size_t watchpoint_mask_bits () const override
  {
    return utils::bit_mask (6, 29);
  }
};

uint64_t
gfx9_base_t::cwsr_record_t::get_info (query_kind_t query) const
{
  uint32_t wave = m_compute_relaunch_wave;
  uint32_t state = m_compute_relaunch_state;

  switch (query)
    {
    case query_kind_t::vgprs:
      /* vgprs are allocated in blocks of 4 registers.  */
      return (1 + compute_relaunch_payload_vgprs (state)) * 4;

    case query_kind_t::acc_vgprs:
      return 0;

    case query_kind_t::sgprs:
      /* sgprs are allocated in blocks of 16 registers. Subtract the ttmps
         registers from this count, as they will be saved in a different area
         than the sgprs.  */
      return (1 + compute_relaunch_payload_sgprs (state)) * 16
             - /* ttmps */ 16;

    case query_kind_t::lds_size:
      return compute_relaunch_payload_lds_size (state) * 128
             * sizeof (uint32_t);

    case query_kind_t::lane_count:
      return 64;

    case query_kind_t::last_wave:
      return compute_relaunch_payload_last_wave (wave);

    case query_kind_t::first_wave:
      return compute_relaunch_payload_first_wave (wave);

    case query_kind_t::is_halted:
      {
        const amd_dbgapi_global_address_t status_reg_address
          = register_address (amdgpu_regnum_t::status).value ();

        uint32_t status_reg;
        if (process ().read_global_memory (status_reg_address, &status_reg,
                                           sizeof (status_reg))
            != AMD_DBGAPI_STATUS_SUCCESS)
          error ("Could not read the 'status' register");

        return (status_reg & sq_wave_status_halt_mask) != 0;
      }

    case query_kind_t::is_stopped:
      {
        const amd_dbgapi_global_address_t ttmp7_address
          = register_address (amdgpu_regnum_t::ttmp7).value ();

        uint32_t ttmp7;
        if (process ().read_global_memory (ttmp7_address, &ttmp7,
                                           sizeof (ttmp7))
            != AMD_DBGAPI_STATUS_SUCCESS)
          error ("Could not read the 'ttmp7' register");

        return (ttmp7 & ttmp7_wave_stopped_mask) != 0;
      }

    default:
      error ("Invalid wave_info query");
    }
}

amdgpu_regnum_t
gfx9_base_t::scalar_operand_to_regnum (int operand) const
{
  if (operand >= 0 && operand <= 101)
    {
      /* SGPR[0] through SGPR[101]  */
      return amdgpu_regnum_t::s0 + operand;
    }

  if (operand >= 108 && operand <= 123)
    {
      /* TTMP[0] through TTMP[15]  */
      return amdgpu_regnum_t::first_ttmp + (operand - 108);
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
      error ("Invalid scalar operand");
    }
}

std::optional<amd_dbgapi_global_address_t>
gfx9_base_t::cwsr_record_t::register_address (amdgpu_regnum_t regnum) const
{
  const auto &architecture
    = static_cast<const gfx9_base_t &> (queue ().architecture ());

  size_t lane_count = get_info (query_kind_t::lane_count);
  amd_dbgapi_global_address_t save_area_addr = m_context_save_address;

  if (get_info (query_kind_t::first_wave))
    {
      save_area_addr -= get_info (query_kind_t::lds_size);

      if (regnum == amdgpu_regnum_t::lds_0)
        return save_area_addr;
    }

  size_t ttmp_size = sizeof (uint32_t);
  size_t ttmp_count = 16;
  size_t ttmps_addr = save_area_addr - ttmp_count * ttmp_size;

  switch (regnum)
    {
    case amdgpu_regnum_t::wave_id:
      regnum = amdgpu_regnum_t::ttmp4;
      break;
    case amdgpu_regnum_t::dispatch_grid:
      regnum = amdgpu_regnum_t::ttmp8;
      break;
    case amdgpu_regnum_t::scratch_offset:
      regnum = amdgpu_regnum_t::ttmp13;
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

  if (((regnum == amdgpu_regnum_t::exec_32
        || regnum == amdgpu_regnum_t::xnack_mask_32)
       && lane_count != 32)
      || ((regnum == amdgpu_regnum_t::exec_64
           || regnum == amdgpu_regnum_t::xnack_mask_64)
          && lane_count != 64))
    return std::nullopt;

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
    case amdgpu_regnum_t::exec_32:
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
    case amdgpu_regnum_t::xnack_mask_32:
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

  size_t sgpr_count = get_info (query_kind_t::sgprs);
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

  if ((regnum == amdgpu_regnum_t::vcc_32 && lane_count != 32)
      || (regnum == amdgpu_regnum_t::vcc_64 && lane_count != 64))
    return std::nullopt;

  /* Rename registers that alias to sgprs.  */
  switch (regnum)
    {
    case amdgpu_regnum_t::vcc_32:
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

  size_t accvgpr_count = get_info (query_kind_t::acc_vgprs);
  size_t accvgpr_size = sizeof (int32_t) * lane_count;
  size_t accvgprs_addr = sgprs_addr - accvgpr_count * accvgpr_size;

  if (lane_count == 32 && regnum >= amdgpu_regnum_t::first_accvgpr_32
      && regnum <= amdgpu_regnum_t::last_accvgpr_32
      && ((regnum - amdgpu_regnum_t::a0_32) < accvgpr_count))
    {
      return accvgprs_addr + (regnum - amdgpu_regnum_t::a0_32) * accvgpr_size;
    }

  if (lane_count == 64 && regnum >= amdgpu_regnum_t::first_accvgpr_64
      && regnum <= amdgpu_regnum_t::last_accvgpr_64
      && ((regnum - amdgpu_regnum_t::a0_64) < accvgpr_count))
    {
      return accvgprs_addr + (regnum - amdgpu_regnum_t::a0_64) * accvgpr_size;
    }

  size_t vgpr_count = get_info (query_kind_t::vgprs);
  size_t vgpr_size = sizeof (int32_t) * lane_count;
  size_t vgprs_addr = accvgprs_addr - vgpr_count * vgpr_size;

  if (lane_count == 32 && regnum >= amdgpu_regnum_t::first_vgpr_32
      && regnum <= amdgpu_regnum_t::last_vgpr_32
      && ((regnum - amdgpu_regnum_t::v0_32) < vgpr_count))
    {
      return vgprs_addr + (regnum - amdgpu_regnum_t::v0_32) * vgpr_size;
    }

  if (lane_count == 64 && regnum >= amdgpu_regnum_t::first_vgpr_64
      && regnum <= amdgpu_regnum_t::last_vgpr_64
      && ((regnum - amdgpu_regnum_t::v0_64) < vgpr_count))
    {
      return vgprs_addr + (regnum - amdgpu_regnum_t::v0_64) * vgpr_size;
    }

  return std::nullopt;
}

void
gfx9_base_t::control_stack_iterate (
  queue_t &queue, const uint32_t *control_stack, size_t control_stack_words,
  amd_dbgapi_global_address_t wave_area_address,
  amd_dbgapi_size_t wave_area_size,
  const std::function<void (std::unique_ptr<architecture_t::cwsr_record_t>)>
    &wave_callback) const
{
  uint32_t state{ 0 };

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
        }
    }

  /* After iterating the control stack, we should have consumed all the data in
     the wave save area, and last_wave_area should point to the bottom of the
     wave save area.  */
  if (last_wave_area != (wave_area_address - wave_area_size))
    error ("Corrupted control stack or wave save area");
}

amd_dbgapi_global_address_t
gfx9_base_t::dispatch_packet_address (
  const architecture_t::cwsr_record_t &cwsr_record) const
{
  static constexpr uint64_t aql_packet_size = 64;

  amd_dbgapi_global_address_t packets_address;
  amd_dbgapi_status_t status = cwsr_record.queue ().get_info (
    AMD_DBGAPI_QUEUE_INFO_ADDRESS, sizeof (packets_address), &packets_address);
  if (status != AMD_DBGAPI_STATUS_SUCCESS)
    error ("Could not get the queue address (rc=%d)", status);

  const amd_dbgapi_global_address_t ttmp6_7_address
    = cwsr_record.register_address (amdgpu_regnum_t::ttmp6).value ();

  uint64_t ttmp6_7;
  status = cwsr_record.process ().read_global_memory (
    ttmp6_7_address, &ttmp6_7, sizeof (ttmp6_7));
  if (status != AMD_DBGAPI_STATUS_SUCCESS)
    error ("Could not read the 'ttmp6:7' registers (rc=%d)", status);

  if ((ttmp6_7 >> 32) & ttmp7_dispatch_id_converted_mask)
    {
      amd_dbgapi_os_queue_packet_id_t os_queue_packet_id
        = ttmp7_queue_packet_id (ttmp6_7 >> 32);
      return packets_address + (os_queue_packet_id * aql_packet_size);
    }
  else
    {
      if (!ttmp6_7)
        error ("invalid null dispatch_ptr");

      /* SPI only sends us the lower 40 bits of the dispatch_ptr, so we
         need to reconstitute it using the packets address for the
         missing upper 8 bits.  */

      constexpr uint64_t spi_mask = utils::bit_mask (0, 39);
      return (ttmp6_7 & spi_mask) | (packets_address & ~spi_mask);
    }
}

/* Vega10 Architecture.  */

class gfx900_t final : public gfx9_base_t
{
public:
  gfx900_t ()
    : gfx9_base_t (EF_AMDGPU_MACH_AMDGCN_GFX900, "amdgcn-amd-amdhsa--gfx900")
  {
  }
};

/* Vega20 Architecture.  */

class gfx906_t final : public gfx9_base_t
{
public:
  gfx906_t ()
    : gfx9_base_t (EF_AMDGPU_MACH_AMDGCN_GFX906, "amdgcn-amd-amdhsa--gfx906")
  {
  }
};

/* Arcturus Architecture.  */

class gfx908_t final : public gfx9_base_t
{
protected:
  class cwsr_record_t final : public gfx9_base_t::cwsr_record_t
  {
  public:
    cwsr_record_t (queue_t &queue, uint32_t compute_relaunch_wave,
                   uint32_t compute_relaunch_state,
                   amd_dbgapi_global_address_t context_save_address)
      : gfx9_base_t::cwsr_record_t (queue, compute_relaunch_wave,
                                    compute_relaunch_state,
                                    context_save_address)
    {
    }

    uint64_t get_info (query_kind_t query) const override
    {
      switch (query)
        {
        case query_kind_t::acc_vgprs:
          return gfx9_base_t::cwsr_record_t::get_info (query_kind_t::vgprs);

        default:
          return gfx9_base_t::cwsr_record_t::get_info (query);
        }
    }
  };

  std::unique_ptr<architecture_t::cwsr_record_t> make_gfx9_cwsr_record (
    queue_t &queue, uint32_t compute_relaunch_wave,
    uint32_t compute_relaunch_state,
    amd_dbgapi_global_address_t context_save_address) const override
  {
    return std::make_unique<cwsr_record_t> (queue, compute_relaunch_wave,
                                            compute_relaunch_state,
                                            context_save_address);
  }

public:
  gfx908_t ()
    : gfx9_base_t (EF_AMDGPU_MACH_AMDGCN_GFX908, "amdgcn-amd-amdhsa--gfx908")
  {
  }

  bool has_acc_vgprs () const override { return true; }
};

class gfx90a_t final : public gfx9_base_t
{
protected:
  class cwsr_record_t final : public gfx9_base_t::cwsr_record_t
  {
  private:
    static constexpr uint32_t compute_relaunch_payload_lds_size (uint32_t x)
    {
      return utils::bit_extract ((x), 9, 16);
    }
    static constexpr uint32_t
    compute_relaunch_payload_accum_offset (uint32_t x)
    {
      return utils::bit_extract ((x), 24, 29);
    }

  public:
    cwsr_record_t (queue_t &queue, uint32_t compute_relaunch_wave,
                   uint32_t compute_relaunch_state,
                   amd_dbgapi_global_address_t context_save_address)
      : gfx9_base_t::cwsr_record_t (queue, compute_relaunch_wave,
                                    compute_relaunch_state,
                                    context_save_address)
    {
    }

    uint64_t get_info (query_kind_t query) const override
    {
      uint32_t state = m_compute_relaunch_state;

      switch (query)
        {
        case query_kind_t::vgprs:
          return (1 + compute_relaunch_payload_accum_offset (state)) * 4;

        case query_kind_t::acc_vgprs:
          return (1 + compute_relaunch_payload_vgprs (state)) * 8
                 - (1 + compute_relaunch_payload_accum_offset (state)) * 4;

        case query_kind_t::lds_size:
          return compute_relaunch_payload_lds_size (state) * 128
                 * sizeof (uint32_t);

        default:
          return gfx9_base_t::cwsr_record_t::get_info (query);
        }
    }
  };

  std::unique_ptr<architecture_t::cwsr_record_t> make_gfx9_cwsr_record (
    queue_t &queue, uint32_t compute_relaunch_wave,
    uint32_t compute_relaunch_state,
    amd_dbgapi_global_address_t context_save_address) const override
  {
    return std::make_unique<cwsr_record_t> (queue, compute_relaunch_wave,
                                            compute_relaunch_state,
                                            context_save_address);
  }

public:
  gfx90a_t ()
    : gfx9_base_t (EF_AMDGPU_MACH_AMDGCN_GFX90A, "amdgcn-amd-amdhsa--gfx90a")
  {
  }

  bool has_acc_vgprs () const override { return true; }
  bool can_halt_at_endpgm () const override { return false; }
};

class gfx10_base_t : public gfx9_base_t
{
protected:
  class cwsr_record_t : public gfx9_base_t::cwsr_record_t
  {
  protected:
    /* On gfx10, there are 2 COMPUTE_RELAUNCH registers for state.  */
    uint32_t const m_compute_relaunch2_state;

    static constexpr uint32_t compute_relaunch_payload_lds_size (uint32_t x)
    {
      return utils::bit_extract (x, 10, 17);
    }
    static constexpr uint32_t compute_relaunch_payload_w32_en (uint32_t x)
    {
      return utils::bit_extract (x, 24, 24);
    }
    static constexpr uint32_t compute_relaunch_payload_last_wave (uint32_t x)
    {
      return utils::bit_extract (x, 29, 29);
    }
    static constexpr uint32_t compute_relaunch_payload_first_wave (uint32_t x)
    {
      return utils::bit_extract (x, 12, 12);
    }

  public:
    cwsr_record_t (queue_t &queue, uint32_t compute_relaunch_wave,
                   uint32_t compute_relaunch_state,
                   uint32_t compute_relaunch2_state,
                   amd_dbgapi_global_address_t context_save_address)
      : gfx9_base_t::cwsr_record_t (queue, compute_relaunch_wave,
                                    compute_relaunch_state,
                                    context_save_address),
        m_compute_relaunch2_state (compute_relaunch2_state)
    {
    }

    uint64_t get_info (query_kind_t query) const override;

    std::optional<amd_dbgapi_global_address_t>
    register_address (amdgpu_regnum_t regnum) const override;
  };

  virtual std::unique_ptr<architecture_t::cwsr_record_t>
  make_gfx10_cwsr_record (
    queue_t &queue, uint32_t compute_relaunch_wave,
    uint32_t compute_relaunch_state, uint32_t compute_relaunch2_state,
    amd_dbgapi_global_address_t context_save_address) const
  {
    return std::make_unique<cwsr_record_t> (
      queue, compute_relaunch_wave, compute_relaunch_state,
      compute_relaunch2_state, context_save_address);
  }

  amdgpu_regnum_t scalar_operand_to_regnum (int operand) const override;

  gfx10_base_t (elf_amdgpu_machine_t e_machine, std::string target_triple)
    : gfx9_base_t (e_machine, std::move (target_triple))
  {
  }

  size_t scalar_register_count () const override { return 106; }
  size_t scalar_alias_count () const override { return 2; }

public:
  bool has_wave32_vgprs () const override { return true; }
  bool has_wave64_vgprs () const override { return true; }
  bool has_acc_vgprs () const override { return false; }

  bool is_code_end (const std::vector<uint8_t> &bytes) const override;
  bool is_call (const std::vector<uint8_t> &bytes) const override;
  bool is_getpc (const std::vector<uint8_t> &bytes) const override;
  bool is_setpc (const std::vector<uint8_t> &bytes) const override;
  bool is_swappc (const std::vector<uint8_t> &bytes) const override;
  bool is_cbranch_i_fork (const std::vector<uint8_t> &bytes) const override;
  bool is_cbranch_g_fork (const std::vector<uint8_t> &bytes) const override;
  bool is_cbranch_join (const std::vector<uint8_t> &bytes) const override;

  void control_stack_iterate (
    queue_t &queue, const uint32_t *control_stack, size_t control_stack_words,
    amd_dbgapi_global_address_t wave_area_address,
    amd_dbgapi_size_t wave_area_size,
    const std::function<void (std::unique_ptr<architecture_t::cwsr_record_t>)>
      &wave_callback) const override;

  bool can_halt_at_endpgm () const override { return false; }
  size_t largest_instruction_size () const override { return 20; }

  size_t watchpoint_mask_bits () const override
  {
    return utils::bit_mask (7, 29);
  }
};

amdgpu_regnum_t
gfx10_base_t::scalar_operand_to_regnum (int operand) const
{
  if (operand >= 0 && operand <= 105)
    {
      /* SGPR[0] through SGPR[105]  */
      return amdgpu_regnum_t::s0 + operand;
    }

  if (operand >= 108 && operand <= 123)
    {
      /* TTMP[0] through TTMP[15]  */
      return amdgpu_regnum_t::first_ttmp + (operand - 108);
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
      error ("Invalid scalar operand");
    }
}

std::optional<amd_dbgapi_global_address_t>
gfx10_base_t::cwsr_record_t::register_address (amdgpu_regnum_t regnum) const
{
  switch (regnum)
    {
    case amdgpu_regnum_t::xnack_mask_32:
      regnum = amdgpu_regnum_t::first_hwreg + 7;
      break;

    case amdgpu_regnum_t::mode:
      regnum = amdgpu_regnum_t::first_hwreg + 8;
      break;

    case amdgpu_regnum_t::xnack_mask_lo:
    case amdgpu_regnum_t::xnack_mask_hi:
    case amdgpu_regnum_t::xnack_mask_64:
      /* On gfx10, xnack_mask is now a 32bit register.  */
      return {};

    case amdgpu_regnum_t::flat_scratch:
      /* On gfx10, flat_scratch is an architected register, so it is saved in
         the hwregs block.  */
      regnum = amdgpu_regnum_t::first_hwreg + 9;
      break;

    default:
      break;
    }

  return gfx9_base_t::cwsr_record_t::register_address (regnum);
}

bool
gfx10_base_t::is_code_end (const std::vector<uint8_t> &bytes) const
{
  /* s_code_end: SOPP Opcode 31  */
  return is_sopp_instruction (bytes, 31);
}

bool
gfx10_base_t::is_call (const std::vector<uint8_t> &bytes) const
{
  /* s_call: SOPK Opcode 22  */
  return is_sopk_instruction (bytes, 22);
}

bool
gfx10_base_t::is_getpc (const std::vector<uint8_t> &bytes) const
{
  /* s_getpc: SOP1 Opcode 31  */
  return is_sop1_instruction (bytes, 31);
}

bool
gfx10_base_t::is_setpc (const std::vector<uint8_t> &bytes) const
{
  /* s_setpc: SOP1 Opcode 32  */
  return is_sop1_instruction (bytes, 32);
}

bool
gfx10_base_t::is_swappc (const std::vector<uint8_t> &bytes) const
{
  /* s_swappc: SOP1 Opcode 33  */
  return is_sop1_instruction (bytes, 33);
}

bool
gfx10_base_t::is_cbranch_i_fork (
  const std::vector<uint8_t> & /* bytes  */) const
{
  return false;
}

bool
gfx10_base_t::is_cbranch_g_fork (
  const std::vector<uint8_t> & /* bytes  */) const
{
  return false;
}

bool
gfx10_base_t::is_cbranch_join (const std::vector<uint8_t> & /* bytes  */) const
{
  return false;
}

uint64_t
gfx10_base_t::cwsr_record_t::get_info (query_kind_t query) const
{
  uint32_t wave = m_compute_relaunch_wave;
  uint32_t state = m_compute_relaunch_state;

  switch (query)
    {
    case query_kind_t::sgprs:
      return 128;

    case query_kind_t::vgprs:
      /* vgprs are allocated in blocks of 8/4 registers (W32/W64).  */
      return (1 + compute_relaunch_payload_vgprs (state))
             * (compute_relaunch_payload_w32_en (state) ? 8 : 4);

    case query_kind_t::lane_count:
      return compute_relaunch_payload_w32_en (state) ? 32 : 64;

    case query_kind_t::lds_size:
      /* lds_size: 128 dwords granularity.  */
      return compute_relaunch_payload_lds_size (state) * 128
             * sizeof (uint32_t);

    case query_kind_t::last_wave:
      return compute_relaunch_payload_last_wave (wave);

    case query_kind_t::first_wave:
      return compute_relaunch_payload_first_wave (wave);

    default:
      return gfx9_base_t::cwsr_record_t::get_info (query);
    }
}

void
gfx10_base_t::control_stack_iterate (
  queue_t &queue, const uint32_t *control_stack, size_t control_stack_words,
  amd_dbgapi_global_address_t wave_area_address,
  amd_dbgapi_size_t wave_area_size,
  const std::function<void (std::unique_ptr<architecture_t::cwsr_record_t>)>
    &wave_callback) const
{
  uint32_t state0{ 0 }, state1{ 0 };

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

          last_wave_area
            = cwsr_record
                ->register_address (cwsr_record->get_info (
                                      cwsr_record_t::query_kind_t::lane_count)
                                        == 32
                                      ? amdgpu_regnum_t::first_vgpr_32
                                      : amdgpu_regnum_t::first_vgpr_64)
                .value ();

          wave_callback (std::move (cwsr_record));
        }
    }

  /* After iterating the control stack, we should have consumed all the data in
     the wave save area, and last_wave_area should point to the bottom of the
     wave save area.  */
  if (last_wave_area != (wave_area_address - wave_area_size))
    error ("Corrupted control stack or wave save area");
}

class gfx10_1_t : public gfx10_base_t
{
protected:
  gfx10_1_t (elf_amdgpu_machine_t e_machine, std::string target_triple)
    : gfx10_base_t (e_machine, std::move (target_triple))
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
    m_disassembly_info (new amd_comgr_disassembly_info_t{ 0 }),
    m_e_machine (e_machine), m_target_triple (std::move (target_triple))
{
}

architecture_t::~architecture_t ()
{
  if (this == detail::last_found_architecture)
    detail::last_found_architecture = nullptr;
  if (*m_disassembly_info != amd_comgr_disassembly_info_t{ 0 })
    amd_comgr_destroy_disassembly_info (*m_disassembly_info);
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

  auto it = std::find_if (s_architecture_map.begin (),
                          s_architecture_map.end (), [&] (const auto &value) {
                            return value.second->elf_amdgpu_machine ()
                                   == elf_amdgpu_machine;
                          });
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

std::optional<std::string>
architecture_t::register_name (amdgpu_regnum_t regnum) const
{
  if (regnum >= amdgpu_regnum_t::first_sgpr
      && regnum < (amdgpu_regnum_t::first_sgpr + scalar_register_count ()))
    {
      return string_printf ("s%ld", regnum - amdgpu_regnum_t::first_sgpr);
    }
  if (has_wave32_vgprs () && regnum >= amdgpu_regnum_t::first_vgpr_32
      && regnum <= amdgpu_regnum_t::last_vgpr_32)
    {
      return string_printf ("v%ld", regnum - amdgpu_regnum_t::first_vgpr_32);
    }
  if (has_wave64_vgprs () && regnum >= amdgpu_regnum_t::first_vgpr_64
      && regnum <= amdgpu_regnum_t::last_vgpr_64)
    {
      return string_printf ("v%ld", regnum - amdgpu_regnum_t::first_vgpr_64);
    }
  if (has_acc_vgprs () && has_wave32_vgprs ()
      && regnum >= amdgpu_regnum_t::first_accvgpr_32
      && regnum <= amdgpu_regnum_t::last_accvgpr_32)
    {
      return string_printf ("a%ld",
                            regnum - amdgpu_regnum_t::first_accvgpr_32);
    }
  if (has_acc_vgprs () && has_wave64_vgprs ()
      && regnum >= amdgpu_regnum_t::first_accvgpr_64
      && regnum <= amdgpu_regnum_t::last_accvgpr_64)
    {
      return string_printf ("a%ld",
                            regnum - amdgpu_regnum_t::first_accvgpr_64);
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
          return std::nullopt;
        }
    }
  if (regnum >= amdgpu_regnum_t::first_hwreg
      && regnum <= amdgpu_regnum_t::last_hwreg)
    {
      return string_printf ("hwreg%ld", regnum - amdgpu_regnum_t::first_hwreg);
    }

  if ((has_wave32_vgprs ()
       && (regnum == amdgpu_regnum_t::exec_32
           || regnum == amdgpu_regnum_t::pseudo_exec_32))
      || (has_wave64_vgprs ()
          && (regnum == amdgpu_regnum_t::exec_64
              || regnum == amdgpu_regnum_t::pseudo_exec_64)))
    {
      return "exec";
    }
  if ((has_wave32_vgprs ()
       && (regnum == amdgpu_regnum_t::vcc_32
           || regnum == amdgpu_regnum_t::pseudo_vcc_32))
      || (has_wave64_vgprs ()
          && (regnum == amdgpu_regnum_t::vcc_64
              || regnum == amdgpu_regnum_t::pseudo_vcc_64)))
    {
      return "vcc";
    }
  if ((has_wave32_vgprs () && regnum == amdgpu_regnum_t::xnack_mask_32)
      || (has_wave64_vgprs () && regnum == amdgpu_regnum_t::xnack_mask_64))
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
    case amdgpu_regnum_t::scratch_offset:
      return "scratch_offset";
    case amdgpu_regnum_t::csp:
      return "csp";
    case amdgpu_regnum_t::null:
      return "null";
    default:
      break;
    }
  return std::nullopt;
}

std::optional<std::string>
architecture_t::register_type (amdgpu_regnum_t regnum) const
{
  /* Vector registers (arch and acc).  */
  if (has_wave32_vgprs ()
      && ((regnum >= amdgpu_regnum_t::first_vgpr_32
           && regnum <= amdgpu_regnum_t::last_vgpr_32)
          || (has_acc_vgprs () && regnum >= amdgpu_regnum_t::first_accvgpr_32
              && regnum <= amdgpu_regnum_t::last_accvgpr_32)))
    {
      return "int32_t[32]";
    }
  if (has_wave64_vgprs ()
      && ((regnum >= amdgpu_regnum_t::first_vgpr_64
           && regnum <= amdgpu_regnum_t::last_vgpr_64)
          || (has_acc_vgprs () && regnum >= amdgpu_regnum_t::first_accvgpr_64
              && regnum <= amdgpu_regnum_t::last_accvgpr_64)))
    {
      return "int32_t[64]";
    }
  /* Scalar registers.  */
  if (regnum >= amdgpu_regnum_t::first_sgpr
      && regnum < (amdgpu_regnum_t::first_sgpr + scalar_register_count ()))
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
  if (has_wave32_vgprs ()
      && (regnum == amdgpu_regnum_t::exec_32
          || regnum == amdgpu_regnum_t::pseudo_exec_32
          || regnum == amdgpu_regnum_t::vcc_32
          || regnum == amdgpu_regnum_t::pseudo_vcc_32
          || regnum == amdgpu_regnum_t::xnack_mask_32))
    {
      return "uint32_t";
    }
  if (has_wave64_vgprs ()
      && (regnum == amdgpu_regnum_t::exec_64
          || regnum == amdgpu_regnum_t::pseudo_exec_64
          || regnum == amdgpu_regnum_t::vcc_64
          || regnum == amdgpu_regnum_t::pseudo_vcc_64
          || regnum == amdgpu_regnum_t::xnack_mask_64))
    {
      return "uint64_t";
    }
  switch (regnum)
    {
    case amdgpu_regnum_t::pc:
      return "void (*)()";

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
    case amdgpu_regnum_t::scratch_offset:
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
      return std::nullopt;
    }
}

std::optional<amd_dbgapi_size_t>
architecture_t::register_size (amdgpu_regnum_t regnum) const
{
  /* Vector registers (arch and acc).  */
  if (has_wave32_vgprs ()
      && ((regnum >= amdgpu_regnum_t::first_vgpr_32
           && regnum <= amdgpu_regnum_t::last_vgpr_32)
          || (has_acc_vgprs () && regnum >= amdgpu_regnum_t::first_accvgpr_32
              && regnum <= amdgpu_regnum_t::last_accvgpr_32)))
    {
      return sizeof (int32_t) * 32;
    }
  if (has_wave64_vgprs ()
      && ((regnum >= amdgpu_regnum_t::first_vgpr_64
           && regnum <= amdgpu_regnum_t::last_vgpr_64)
          || (has_acc_vgprs () && regnum >= amdgpu_regnum_t::first_accvgpr_64
              && regnum <= amdgpu_regnum_t::last_accvgpr_64)))
    {
      return sizeof (int32_t) * 64;
    }
  /* Scalar registers.  */
  if (regnum >= amdgpu_regnum_t::first_sgpr
      && regnum < (amdgpu_regnum_t::first_sgpr + scalar_register_count ()))
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
  if (has_wave32_vgprs ()
      && (regnum == amdgpu_regnum_t::exec_32
          || regnum == amdgpu_regnum_t::pseudo_exec_32
          || regnum == amdgpu_regnum_t::vcc_32
          || regnum == amdgpu_regnum_t::pseudo_vcc_32
          || regnum == amdgpu_regnum_t::xnack_mask_32))
    {
      return sizeof (uint32_t);
    }
  if (has_wave64_vgprs ()
      && (regnum == amdgpu_regnum_t::exec_64
          || regnum == amdgpu_regnum_t::pseudo_exec_64
          || regnum == amdgpu_regnum_t::vcc_64
          || regnum == amdgpu_regnum_t::pseudo_vcc_64
          || regnum == amdgpu_regnum_t::xnack_mask_64))
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
    case amdgpu_regnum_t::scratch_offset:
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
      return std::nullopt;
    }
}

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
architecture_t::disassembly_info () const
{
  if (*m_disassembly_info == amd_comgr_disassembly_info_t{ 0 })
    {
      auto read_memory_callback = [] (uint64_t from, char *to, uint64_t size,
                                      void *user_data) -> uint64_t {
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
        = [] (const char *instruction, void *user_data) {
            detail::disassembly_user_data_t *data
              = static_cast<detail::disassembly_user_data_t *> (user_data);

            while (isspace (*instruction))
              ++instruction;

            if (data->instruction)
              data->instruction->assign (instruction);
          };

      auto print_address_annotation_callback
        = [] (uint64_t address, void *user_data) {
            detail::disassembly_user_data_t *data
              = static_cast<detail::disassembly_user_data_t *> (user_data);
            if (data->operands)
              data->operands->emplace_back (
                static_cast<amd_dbgapi_global_address_t> (address));
          };

      if (amd_comgr_create_disassembly_info (
            target_triple ().c_str (), read_memory_callback,
            print_instruction_callback, print_address_annotation_callback,
            m_disassembly_info.get ()))
        error ("amd_comgr_create_disassembly_info failed");
    }

  return *m_disassembly_info;
}

amd_dbgapi_size_t
architecture_t::instruction_size (const void *instruction_bytes,
                                  size_t size) const
{
  struct detail::disassembly_user_data_t user_data
    = { /* .memory =  */ instruction_bytes,
        /* .offset =  */ 0,
        /* .size =  */ size,
        /* .instruction =  */ nullptr,
        /* .operands =  */ nullptr };

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
architecture_t::disassemble_instruction (amd_dbgapi_global_address_t address,
                                         const void *instruction_bytes,
                                         size_t size) const
{
  std::string instruction_text;
  std::vector<uint64_t> address_operands;

  struct detail::disassembly_user_data_t user_data
    = { /* .memory =  */ instruction_bytes,
        /* .offset =  */ address,
        /* .size =  */ size,
        /* .instruction =  */ &instruction_text,
        /* .operands =  */ &address_operands };

  /* Disassemble one instruction.  */
  if (amd_comgr_disassemble_instruction (disassembly_info (),
                                         static_cast<uint64_t> (address),
                                         &user_data, &size)
      != AMD_COMGR_STATUS_SUCCESS)
    throw exception_t (AMD_DBGAPI_STATUS_ERROR_ILLEGAL_INSTRUCTION);

  return std::make_tuple (size, instruction_text, address_operands);
}

amd_dbgapi_status_t
architecture_t::get_info (amd_dbgapi_architecture_info_t query,
                          size_t value_size, void *value) const
{
  switch (query)
    {
    case AMD_DBGAPI_ARCHITECTURE_INFO_NAME:
      return utils::get_info (value_size, value, m_target_triple);

    case AMD_DBGAPI_ARCHITECTURE_INFO_ELF_AMDGPU_MACHINE:
      return utils::get_info (value_size, value, elf_amdgpu_machine ());

    case AMD_DBGAPI_ARCHITECTURE_INFO_LARGEST_INSTRUCTION_SIZE:
      return utils::get_info (value_size, value, largest_instruction_size ());

    case AMD_DBGAPI_ARCHITECTURE_INFO_MINIMUM_INSTRUCTION_ALIGNMENT:
      return utils::get_info (value_size, value,
                              minimum_instruction_alignment ());

    case AMD_DBGAPI_ARCHITECTURE_INFO_BREAKPOINT_INSTRUCTION_SIZE:
      return utils::get_info (value_size, value,
                              breakpoint_instruction ().size ());

    case AMD_DBGAPI_ARCHITECTURE_INFO_BREAKPOINT_INSTRUCTION:
      return utils::get_info (value_size, value, breakpoint_instruction ());

    case AMD_DBGAPI_ARCHITECTURE_INFO_BREAKPOINT_INSTRUCTION_PC_ADJUST:
      return utils::get_info (value_size, value,
                              breakpoint_instruction_pc_adjust ());

    case AMD_DBGAPI_ARCHITECTURE_INFO_PC_REGISTER:
      return utils::get_info (value_size, value,
                              regnum_to_register_id (amdgpu_regnum_t::pc));
    }
  return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;
}

template <typename ArchitectureType, typename... Args>
auto
architecture_t::create_architecture (Args &&... args)
{
  auto *arch = new ArchitectureType (std::forward<Args> (args)...);
  if (!arch)
    error ("could not create architecture");

  arch->initialize ();
  return std::make_pair (arch->id (), std::unique_ptr<architecture_t> (arch));
}

decltype (
  architecture_t::s_architecture_map) architecture_t::s_architecture_map{
  [] () {
    decltype (s_architecture_map) map;
    map.emplace (create_architecture<gfx900_t> ());
    map.emplace (create_architecture<gfx906_t> ());
    map.emplace (create_architecture<gfx908_t> ());
    map.emplace (create_architecture<gfx90a_t> ());
    map.emplace (create_architecture<gfx1010_t> ());
    map.emplace (create_architecture<gfx1011_t> ());
    map.emplace (create_architecture<gfx1012_t> ());
    map.emplace (create_architecture<gfx1030_t> ());
    map.emplace (create_architecture<gfx1031_t> ());
    return map;
  }()
};

} /* namespace amd::dbgapi */

using namespace amd::dbgapi;

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_get_architecture (uint32_t elf_amdgpu_machine,
                             amd_dbgapi_architecture_id_t *architecture_id)
{
  TRACE_BEGIN (elf_amdgpu_machine, architecture_id);
  TRY;

  if (!detail::is_initialized)
    return AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED;

  if (!architecture_id)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;

  const architecture_t *architecture = architecture_t::find (
    static_cast<elf_amdgpu_machine_t> (elf_amdgpu_machine));

  if (!architecture)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ELF_AMDGPU_MACHINE;

  *architecture_id = architecture->id ();

  return AMD_DBGAPI_STATUS_SUCCESS;
  CATCH;
  TRACE_END (make_ref (architecture_id));
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_architecture_get_info (amd_dbgapi_architecture_id_t architecture_id,
                                  amd_dbgapi_architecture_info_t query,
                                  size_t value_size, void *value)
{
  TRACE_BEGIN (architecture_id, query, value_size, value);
  TRY;

  if (!detail::is_initialized)
    return AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED;

  const architecture_t *architecture = architecture_t::find (architecture_id);

  if (!architecture)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARCHITECTURE_ID;

  return architecture->get_info (query, value_size, value);
  CATCH;
  TRACE_END (make_query_ref (query, value));
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
  TRACE_BEGIN (architecture_id, make_hex (address), make_ref (size),
               make_hex (make_ref (static_cast<const uint8_t *> (memory),
                                   size ? *size : 0)),
               instruction_text, symbolizer_id,
               reinterpret_cast<const void *> (symbolizer));
  TRY;

  if (!detail::is_initialized)
    return AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED;

  if (!memory || !size || !*size)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;

  const architecture_t *architecture = architecture_t::find (architecture_id);

  if (!architecture)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARCHITECTURE_ID;

  if (!instruction_text)
    {
      *size = architecture->instruction_size (memory, *size);
      return AMD_DBGAPI_STATUS_SUCCESS;
    }

  auto [instruction_size, instruction_str, address_operands]
    = architecture->disassemble_instruction (address, memory, *size);

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
                return AMD_DBGAPI_STATUS_ERROR;

              const bool empty = !symbol_text[0];
              address_operands_str += symbol_text;
              deallocate_memory (symbol_text);

              if (empty)
                return AMD_DBGAPI_STATUS_ERROR;

              continue;
            }
          else if (status != AMD_DBGAPI_STATUS_ERROR_SYMBOL_NOT_FOUND)
            return AMD_DBGAPI_STATUS_ERROR_CLIENT_CALLBACK;
        }

      address_operands_str += string_printf ("%#lx", operand);
    }

  instruction_str += address_operands_str;

  /* Return the instruction text in client allocated memory.  */
  size_t mem_size = instruction_str.size () + 1;
  void *mem = allocate_memory (mem_size);
  if (!mem)
    return AMD_DBGAPI_STATUS_ERROR_CLIENT_CALLBACK;

  memcpy (mem, instruction_str.c_str (), mem_size);
  *instruction_text = static_cast<char *> (mem);
  *size = instruction_size;

  return AMD_DBGAPI_STATUS_SUCCESS;
  CATCH;
  TRACE_END (make_ref (size), make_ref (instruction_text));
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_classify_instruction (
  amd_dbgapi_architecture_id_t architecture_id,
  amd_dbgapi_global_address_t address, amd_dbgapi_size_t *size_p,
  const void *memory, amd_dbgapi_instruction_kind_t *instruction_kind_p,
  amd_dbgapi_instruction_properties_t *instruction_properties_p,
  void **instruction_information_p)
{
  TRACE_BEGIN (architecture_id, make_hex (address), make_ref (size_p),
               make_hex (make_ref (static_cast<const uint8_t *> (memory),
                                   size_p ? *size_p : 0)),
               instruction_kind_p, instruction_properties_p,
               instruction_information_p);
  TRY;

  if (!detail::is_initialized)
    return AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED;

  if (!memory || !size_p || !*size_p || !instruction_kind_p)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;

  const architecture_t *architecture = architecture_t::find (architecture_id);

  if (!architecture)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARCHITECTURE_ID;

  std::vector<uint8_t> instruction (static_cast<const uint8_t *> (memory),
                                    static_cast<const uint8_t *> (memory)
                                      + *size_p);

  auto [kind, properties, size, information]
    = architecture->classify_instruction (instruction, address);

  if (instruction_information_p)
    {
      size_t mem_size
        = information.size () * sizeof (decltype (information)::value_type);

      if (!mem_size)
        {
          *instruction_information_p = nullptr;
        }
      else
        {
          void *mem = allocate_memory (mem_size);
          if (!mem)
            return AMD_DBGAPI_STATUS_ERROR_CLIENT_CALLBACK;

          memcpy (mem, information.data (), mem_size);
          *instruction_information_p = mem;
        }
    }

  if (instruction_properties_p)
    *instruction_properties_p = properties;

  *size_p = size;
  *instruction_kind_p = kind;

  return AMD_DBGAPI_STATUS_SUCCESS;
  CATCH;
  TRACE_END (make_ref (size_p), make_ref (instruction_kind_p),
             make_ref (instruction_properties_p),
             make_query_ref (instruction_kind_p
                               ? *instruction_kind_p
                               : AMD_DBGAPI_INSTRUCTION_KIND_UNKNOWN,
                             instruction_information_p));
}
