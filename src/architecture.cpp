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

#include "architecture.h"
#include "agent.h"
#include "debug.h"
#include "displaced_stepping.h"
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
#include <iomanip>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <sys/types.h>
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
  static constexpr uint32_t ttmp11_trap_handler_trap_raised_mask = 1 << 7;
  static constexpr uint32_t ttmp11_trap_handler_excp_raised_mask = 1 << 8;
  static constexpr uint32_t ttmp11_trap_handler_events_mask
      = (ttmp11_trap_handler_trap_raised_mask
         | ttmp11_trap_handler_excp_raised_mask);

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
  virtual void initialize () override;

  virtual amd_dbgapi_status_t convert_address_space (
      const wave_t &wave, amd_dbgapi_lane_id_t lane_id,
      const address_space_t &from_address_space,
      const address_space_t &to_address_space,
      amd_dbgapi_segment_address_t from_address,
      amd_dbgapi_segment_address_t *to_address) const override;

  virtual void lower_address_space (
      const wave_t &wave, amd_dbgapi_lane_id_t *lane_id,
      const address_space_t &original_address_space,
      const address_space_t **lowered_address_space,
      amd_dbgapi_segment_address_t original_address,
      amd_dbgapi_segment_address_t *lowered_address) const override;

  virtual bool address_is_in_address_class (
      const wave_t &wave, amd_dbgapi_lane_id_t lane_id,
      const address_space_t &address_space,
      amd_dbgapi_segment_address_t segment_address,
      const address_class_t &address_class) const override;

  virtual bool address_spaces_may_alias (
      const address_space_t &address_space1,
      const address_space_t &address_space2) const override;

  virtual amd_dbgapi_watchpoint_share_kind_t
  watchpoint_share_kind () const override
  {
    return AMD_DBGAPI_WATCHPOINT_SHARE_KIND_SHARED;
  };

  virtual size_t watchpoint_count () const override { return 4; };

  std::optional<os_watch_mode_t>
  watchpoint_mode (amd_dbgapi_watchpoint_kind_t kind) const override;

  virtual std::vector<os_watch_id_t>
  triggered_watchpoints (const wave_t &wave) const override;

  virtual void displaced_stepping_fixup (
      wave_t &wave, displaced_stepping_t &displaced_stepping) const override;

  virtual void get_wave_coords (wave_t &wave,
                                std::array<uint32_t, 3> &group_ids,
                                uint32_t *wave_in_group) const override;

  virtual void
  get_wave_state (wave_t &wave, amd_dbgapi_wave_state_t *state,
                  amd_dbgapi_wave_stop_reason_t *stop_reason) const override;
  virtual void set_wave_state (wave_t &wave,
                               amd_dbgapi_wave_state_t state) const override;

  virtual uint32_t os_wave_launch_trap_mask_to_wave_mode (
      os_wave_launch_trap_mask_t mask) const;

  virtual void
  enable_wave_traps (wave_t &wave,
                     os_wave_launch_trap_mask_t mask) const override final;
  virtual void
  disable_wave_traps (wave_t &wave,
                      os_wave_launch_trap_mask_t mask) const override final;

  virtual size_t minimum_instruction_alignment () const override;
  virtual const std::vector<uint8_t> &nop_instruction () const override;
  virtual const std::vector<uint8_t> &breakpoint_instruction () const override;
  virtual const std::vector<uint8_t> &assert_instruction () const override;
  virtual const std::vector<uint8_t> &endpgm_instruction () const override;
  virtual size_t breakpoint_instruction_pc_adjust () const override;

protected:
  /* See https://llvm.org/docs/AMDGPUUsage.html#trap-handler-abi  */
  static constexpr uint8_t assert_trap_id = 0x2;
  static constexpr uint8_t debug_trap_id = 0x3;
  static constexpr uint8_t breakpoint_trap_id = 0x7;

  static uint8_t encoding_ssrc0 (const std::vector<uint8_t> &bytes);
  static uint8_t encoding_sdst (const std::vector<uint8_t> &bytes);
  static uint8_t encoding_op7 (const std::vector<uint8_t> &bytes);
  static int encoding_simm16 (const std::vector<uint8_t> &bytes);

  static bool is_sopk_instruction (const std::vector<uint8_t> &bytes, int op5);
  static bool is_sop1_instruction (const std::vector<uint8_t> &bytes, int op7);
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
  virtual bool is_nop (const std::vector<uint8_t> &bytes) const;
  virtual bool is_trap (const std::vector<uint8_t> &bytes,
                        uint8_t *trap_id = nullptr) const;
  virtual bool is_endpgm (const std::vector<uint8_t> &bytes) const override;
  virtual bool
  is_breakpoint (const std::vector<uint8_t> &bytes) const override;

  virtual bool
  can_execute_displaced (const std::vector<uint8_t> &bytes) const override
  {
    /* Displace stepping over a cbranch_i_fork is not supported.  */
    return !is_cbranch_i_fork (bytes);
  }

  virtual bool can_simulate (const std::vector<uint8_t> &bytes) const override
  {
    return is_branch (bytes) || is_cbranch (bytes) || is_call (bytes)
           || is_getpc (bytes) || is_setpc (bytes) || is_swappc (bytes)
           || is_endpgm (bytes) || is_nop (bytes);
  }

  virtual amd_dbgapi_global_address_t
  branch_target (wave_t &wave, amd_dbgapi_global_address_t pc,
                 const std::vector<uint8_t> &instruction) const;

  virtual std::tuple<amd_dbgapi_instruction_kind_t, /* instruction_kind  */
                     size_t,                        /* instruction_size  */
                     std::vector<uint64_t> /* instruction_properties  */>
  classify_instruction (const std::vector<uint8_t> &instruction,
                        amd_dbgapi_global_address_t address) const override;

  virtual void
  simulate_instruction (wave_t &wave, amd_dbgapi_global_address_t pc,
                        const std::vector<uint8_t> &instruction) const;
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

  /* Scalar registers: [s0-s111]  */
  register_class_t::register_map_t scalar_registers;
  scalar_registers.emplace (amdgpu_regnum_t::first_sgpr,
                            amdgpu_regnum_t::last_sgpr);
  create<register_class_t> (*this, "scalar", scalar_registers);

  /* Vector registers: [v0-v255, acc0-acc255]  */
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

  /* System registers: [ttmps, hwregs, flat_scratch, xnack_mask, vcc]  */
  register_class_t::register_map_t system_registers;

  system_registers.emplace (amdgpu_regnum_t::ttmp4, amdgpu_regnum_t::ttmp11);
  system_registers.emplace (amdgpu_regnum_t::ttmp13, amdgpu_regnum_t::ttmp13);

  system_registers.emplace (amdgpu_regnum_t::m0, amdgpu_regnum_t::m0);
  system_registers.emplace (amdgpu_regnum_t::status, amdgpu_regnum_t::status);
  system_registers.emplace (amdgpu_regnum_t::mode, amdgpu_regnum_t::mode);
  system_registers.emplace (amdgpu_regnum_t::trapsts,
                            amdgpu_regnum_t::trapsts);
  system_registers.emplace (amdgpu_regnum_t::flat_scratch,
                            amdgpu_regnum_t::flat_scratch);
  if (has_wave32_vgprs ())
    {
      system_registers.emplace (amdgpu_regnum_t::exec_32,
                                amdgpu_regnum_t::exec_32);
      system_registers.emplace (amdgpu_regnum_t::xnack_mask_32,
                                amdgpu_regnum_t::xnack_mask_32);
      system_registers.emplace (amdgpu_regnum_t::vcc_32,
                                amdgpu_regnum_t::vcc_32);
    }
  if (has_wave64_vgprs ())
    {
      system_registers.emplace (amdgpu_regnum_t::exec_64,
                                amdgpu_regnum_t::exec_64);
      system_registers.emplace (amdgpu_regnum_t::xnack_mask_64,
                                amdgpu_regnum_t::xnack_mask_64);
      system_registers.emplace (amdgpu_regnum_t::vcc_64,
                                amdgpu_regnum_t::vcc_64);
    }
  create<register_class_t> (*this, "system", system_registers);

  /* General registers: [{scalar}, {vector}, pc, exec, vcc]  */
  register_class_t::register_map_t general_registers;
  general_registers.insert (scalar_registers.begin (),
                            scalar_registers.end ());
  general_registers.insert (vector_registers.begin (),
                            vector_registers.end ());
  general_registers.emplace (amdgpu_regnum_t::pc, amdgpu_regnum_t::pc);
  if (has_wave32_vgprs ())
    {
      general_registers.emplace (amdgpu_regnum_t::exec_32,
                                 amdgpu_regnum_t::exec_32);
      general_registers.emplace (amdgpu_regnum_t::vcc_32,
                                 amdgpu_regnum_t::vcc_32);
    }
  if (has_wave64_vgprs ())
    {
      general_registers.emplace (amdgpu_regnum_t::exec_64,
                                 amdgpu_regnum_t::exec_64);
      general_registers.emplace (amdgpu_regnum_t::vcc_64,
                                 amdgpu_regnum_t::vcc_64);
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
    const wave_t &wave, amd_dbgapi_lane_id_t lane_id,
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
  dbgapi_assert (wave.is_register_cached (amdgpu_regnum_t::trapsts));

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

amd_dbgapi_global_address_t
amdgcn_architecture_t::branch_target (
    wave_t &wave, amd_dbgapi_global_address_t pc,
    const std::vector<uint8_t> &instruction) const
{
  amd_dbgapi_global_address_t new_pc = pc + instruction.size ();
  ssize_t branch_offset = encoding_simm16 (instruction) << 2;

  if (is_branch (instruction) || is_call (instruction))
    {
      /* Unconditional branch is always taken.  */
      new_pc += branch_offset;
    }
  else if (is_cbranch (instruction))
    {
      uint32_t status_reg;

      wave.read_register (amdgpu_regnum_t::status, &status_reg);

      /* Evaluate the condition.  */
      bool branch_taken{};
      switch (cbranch_opcodes_map.find (encoding_op7 (instruction))->second)
        {
        case cbranch_cond_t::scc0:
          branch_taken = (status_reg & sq_wave_status_scc_mask) == 0;
          break;
        case cbranch_cond_t::scc1:
          branch_taken = (status_reg & sq_wave_status_scc_mask) != 0;
          break;
        case cbranch_cond_t::execz:
          branch_taken = (status_reg & sq_wave_status_execz_mask) != 0;
          break;
        case cbranch_cond_t::execnz:
          branch_taken = (status_reg & sq_wave_status_execz_mask) == 0;
          break;
        case cbranch_cond_t::vccz:
          branch_taken = (status_reg & sq_wave_status_vccz_mask) != 0;
          break;
        case cbranch_cond_t::vccnz:
          branch_taken = (status_reg & sq_wave_status_vccz_mask) == 0;
          break;
        case cbranch_cond_t::cdbgsys:
          branch_taken = (status_reg & sq_wave_status_cond_dbg_sys_mask) != 0;
          break;
        case cbranch_cond_t::cdbguser:
          branch_taken = (status_reg & sq_wave_status_cond_dbg_user_mask) != 0;
          break;
        case cbranch_cond_t::cdbgsys_or_user:
          {
            uint32_t mask = sq_wave_status_cond_dbg_sys_mask
                            | sq_wave_status_cond_dbg_user_mask;
            branch_taken = (status_reg & mask) != 0;
            break;
          }
        case cbranch_cond_t::cdbgsys_and_user:
          {
            uint32_t mask = sq_wave_status_cond_dbg_sys_mask
                            | sq_wave_status_cond_dbg_user_mask;
            branch_taken = (status_reg & mask) == mask;
            break;
          }
        }

      if (branch_taken)
        new_pc += branch_offset;
    }
  else
    error ("Invalid instruction");

  return new_pc;
}

std::tuple<amd_dbgapi_instruction_kind_t, size_t, std::vector<uint64_t>>
amdgcn_architecture_t::classify_instruction (
    const std::vector<uint8_t> &instruction,
    amd_dbgapi_global_address_t address) const
{
  enum class properties_kind_t
  {
    none = 0,
    pc_direct,
    pc_indirect,
    uint8,
  } properties_kind;

  amd_dbgapi_instruction_kind_t instruction_kind;

  size_t size = instruction_size (instruction);
  if (!size)
    return { AMD_DBGAPI_INSTRUCTION_KIND_UNKNOWN, 0, {} };

  if (is_branch (instruction))
    {
      instruction_kind = AMD_DBGAPI_INSTRUCTION_KIND_DIRECT_BRANCH;
      properties_kind = properties_kind_t::pc_direct;
    }
  else if (is_cbranch (instruction))
    {
      instruction_kind = AMD_DBGAPI_INSTRUCTION_KIND_DIRECT_BRANCH_CONDITIONAL;
      properties_kind = properties_kind_t::pc_direct;
    }
  else if (is_setpc (instruction))
    {
      instruction_kind
          = AMD_DBGAPI_INSTRUCTION_KIND_INDIRECT_BRANCH_REGISTER_PAIR;
      properties_kind = properties_kind_t::pc_indirect;
    }
  else if (is_call (instruction))
    {
      instruction_kind = AMD_DBGAPI_INSTRUCTION_KIND_DIRECT_CALL_REGISTER_PAIR;
      properties_kind = properties_kind_t::pc_direct;
    }
  else if (is_swappc (instruction))
    {
      instruction_kind
          = AMD_DBGAPI_INSTRUCTION_KIND_INDIRECT_CALL_REGISTER_PAIRS;
      properties_kind = properties_kind_t::pc_indirect;
    }
  else if (is_endpgm (instruction))
    {
      instruction_kind = AMD_DBGAPI_INSTRUCTION_KIND_TERMINATE;
      properties_kind = properties_kind_t::none;
    }
  else if (is_trap (instruction))
    {
      instruction_kind = AMD_DBGAPI_INSTRUCTION_KIND_TRAP;
      properties_kind = properties_kind_t::uint8;
    }
  else if (is_sethalt (instruction) && (encoding_simm16 (instruction) & 0x1))
    {
      instruction_kind = AMD_DBGAPI_INSTRUCTION_KIND_HALT;
      properties_kind = properties_kind_t::none;
    }
  else if (is_barrier (instruction))
    {
      instruction_kind = AMD_DBGAPI_INSTRUCTION_KIND_BARRIER;
      properties_kind = properties_kind_t::none;
    }
  else if (is_sleep (instruction))
    {
      instruction_kind = AMD_DBGAPI_INSTRUCTION_KIND_SLEEP;
      properties_kind = properties_kind_t::none;
    }
  else if (is_code_end (instruction))
    {
      instruction_kind = AMD_DBGAPI_INSTRUCTION_KIND_UNKNOWN;
      properties_kind = properties_kind_t::none;
    }
  else
    {
      instruction_kind = AMD_DBGAPI_INSTRUCTION_KIND_SEQUENTIAL;
      properties_kind = properties_kind_t::none;
    }

  std::vector<uint64_t> properties;

  if (properties_kind == properties_kind_t::pc_direct)
    {
      ssize_t branch_offset = encoding_simm16 (instruction) << 2;
      properties.emplace_back (address + size + branch_offset);
    }
  else if (properties_kind == properties_kind_t::pc_indirect)
    {
      amdgpu_regnum_t sdst_regnum
          = scalar_operand_to_regnum (encoding_sdst (instruction));
      properties.emplace_back (static_cast<uint64_t> (sdst_regnum));
      properties.emplace_back (static_cast<uint64_t> (sdst_regnum) + 1);
    }
  else if (properties_kind == properties_kind_t::uint8)
    {
      properties.emplace_back (
          utils::bit_extract (encoding_simm16 (instruction), 0, 7));
    }

  return { instruction_kind, size, std::move (properties) };
}

void
amdgcn_architecture_t::simulate_instruction (
    wave_t &wave, amd_dbgapi_global_address_t pc,
    const std::vector<uint8_t> &instruction) const
{
  dbgapi_assert (
      wave.state () == AMD_DBGAPI_WAVE_STATE_STOP
      && "We can only simulate instructions when the wave is not running.");

  amd_dbgapi_global_address_t new_pc;

  if (is_nop (instruction))
    {
      new_pc = pc + nop_instruction ().size ();
    }
  else if (is_endpgm (instruction))
    {
      wave.terminate ();
      return;
    }
  else if (is_branch (instruction) || is_cbranch (instruction))
    {
      new_pc = branch_target (wave, pc, instruction);
    }
  else if (is_call (instruction) || is_getpc (instruction)
           || is_swappc (instruction) || is_setpc (instruction))
    {
      if (!is_setpc (instruction))
        {
          amdgpu_regnum_t sdst_regnum
              = scalar_operand_to_regnum (encoding_sdst (instruction));

          /* The hardware requires a 64-bit address register pair to have the
             lower register number be even.  If it is not, then it is treated
             as if the lower register number is the preceding even register
             number.  So mask out lower bit of the register number.  */
          sdst_regnum = sdst_regnum & -2;

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
              wave.write_register (sdst_regnum, &sdst_lo);
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
             lower register number be even.  If it is not, then it is treated
             as if the lower register number is the preceding even register
             number.  So mask out lower bit of the register number.  */
          ssrc_regnum = ssrc_regnum & -2;

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
              wave.read_register (ssrc_regnum, &ssrc_lo);
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
}

void
amdgcn_architecture_t::displaced_stepping_fixup (
    wave_t &wave, displaced_stepping_t &displaced_stepping) const
{
  amd_dbgapi_global_address_t displaced_pc = wave.pc ();

  if (displaced_stepping.is_simulated ()
      /* The pc could still pointing at the displaced instruction is if the
         single stepping operation did not execute (we can't have a branch to
         self here since branches are simulated).  In that case, fixup will
         simply restore the pc to the original location.  */
      && displaced_pc != displaced_stepping.to ())
    {
      simulate_instruction (wave, displaced_stepping.from (),
                            displaced_stepping.original_instruction ());
    }
  else
    {
      amd_dbgapi_global_address_t restored_pc = displaced_pc
                                                + displaced_stepping.from ()
                                                - displaced_stepping.to ();

      wave.write_register (amdgpu_regnum_t::pc, &restored_pc);
    }
}

void
amdgcn_architecture_t::get_wave_coords (wave_t &wave,
                                        std::array<uint32_t, 3> &group_ids,
                                        uint32_t *wave_in_group) const
{
  dbgapi_assert (wave_in_group && "Invalid parameter");

  /* Read group_ids[0:3].  */
  if (wave.process ().read_global_memory (
          wave.register_address (amdgpu_regnum_t::dispatch_grid_x).value (),
          &group_ids[0], sizeof (group_ids))
      != AMD_DBGAPI_STATUS_SUCCESS)
    error ("Could not read the 'dispatch_grid_*' registers");

  uint32_t ttmp11;
  wave.read_register (amdgpu_regnum_t::ttmp11, &ttmp11);

  *wave_in_group = ttmp11 & ttmp11_wave_in_group_mask;
}

void
amdgcn_architecture_t::get_wave_state (
    wave_t &wave, amd_dbgapi_wave_state_t *state,
    amd_dbgapi_wave_stop_reason_t *stop_reason) const
{
  dbgapi_assert (state && stop_reason && "Invalid parameter");

  uint32_t status_reg, mode_reg;
  wave.read_register (amdgpu_regnum_t::status, &status_reg);
  wave.read_register (amdgpu_regnum_t::mode, &mode_reg);

  amd_dbgapi_wave_state_t saved_state = wave.state ();

  *state = (status_reg & sq_wave_status_halt_mask)
               ? AMD_DBGAPI_WAVE_STATE_STOP
               : ((mode_reg & sq_wave_mode_debug_en_mask)
                      ? AMD_DBGAPI_WAVE_STATE_SINGLE_STEP
                      : AMD_DBGAPI_WAVE_STATE_RUN);

  if (*state != AMD_DBGAPI_WAVE_STATE_STOP)
    {
      /* The wave is running, there is no stop reason.  */
      *stop_reason = AMD_DBGAPI_WAVE_STOP_REASON_NONE;

      dbgapi_assert ([&wave] () {
        uint32_t ttmp11;
        wave.read_register (amdgpu_regnum_t::ttmp11, &ttmp11);
        return !(ttmp11 & ttmp11_trap_handler_events_mask);
      }()
                     && "Waves should not have trap handler events while "
                        "running. These are reset when unhalting the wave.");
    }
  else if (saved_state == AMD_DBGAPI_WAVE_STATE_STOP)
    {
      /* The wave was previously stopped, and it still is stopped, the stop
         reason is unchanged.  */
      *stop_reason = wave.stop_reason ();
    }
  else
    {
      /* The wave is stopped, but it was previously running.  */

      amd_dbgapi_wave_stop_reason_t reason_mask
          = (saved_state == AMD_DBGAPI_WAVE_STATE_SINGLE_STEP)
                ? AMD_DBGAPI_WAVE_STOP_REASON_SINGLE_STEP
                : AMD_DBGAPI_WAVE_STOP_REASON_NONE;

      uint32_t trapsts, ttmp11;
      wave.read_register (amdgpu_regnum_t::trapsts, &trapsts);
      wave.read_register (amdgpu_regnum_t::ttmp11, &ttmp11);

      bool trap_raised = !!(ttmp11 & ttmp11_trap_handler_trap_raised_mask);
      /* bool excp_raised
             = !!(ttmp11 & TTMP11_TRAP_HANDLER_EXCP_RAISED_MASK);
       */

      amd_dbgapi_global_address_t pc = wave.pc ();

      if ((trapsts & sq_wave_trapsts_excp_mem_viol_mask
           || trapsts & sq_wave_trapsts_illegal_inst_mask)
          /* FIXME: If the wave was single-stepping when the exception
             occurred, the first level trap handler did not decrement the PC as
             it took the SINGLE_STEP_WORKAROUND path.  */
          && saved_state != AMD_DBGAPI_WAVE_STATE_SINGLE_STEP)
        {
          /* FIXME: Enable this when the trap handler is modified to send
             debugger notifications for exceptions.
             if (!excp_raised)
               error ("The trap handler should have set the excp_raised bit.");
           */

          /* The first-level trap handler subtracts 8 from the PC, so
             we add it back here.  */
          pc += 8;
          wave.write_register (amdgpu_regnum_t::pc, &pc);
        }

      auto instruction = wave.instruction_at_pc ();

      /* Check for spurious single-step events. A context save/restore before
         executing the single-stepped instruction could have caused the event
         to be reported with the wave halted at the instruction instead of
         after.  In such cases, un-halt the wave and let it continue, so that
         the instruction is executed.  */
      bool ignore_single_step_event
          = saved_state == AMD_DBGAPI_WAVE_STATE_SINGLE_STEP
            && wave.saved_pc () == pc && !trap_raised;

      if (instruction && ignore_single_step_event)
        {
          /* Trim to size of instruction, simulate_instruction needs the exact
           * instruction bytes.  */
          size_t size = instruction_size (*instruction);
          dbgapi_assert (size != 0 && "Invalid instruction");
          instruction->resize (size);

          /* Branch instructions should be simulated, and the event reported,
             as we cannot tell if a branch to self instruction has executed. */
          if (can_simulate (*instruction))
            {
              simulate_instruction (wave, pc, *instruction);

              /* We successfully simulated the instruction, report the
                 single-step event.  */
              ignore_single_step_event = false;

              /* Reload the instruction since the pc may have changed.  */
              instruction = wave.instruction_at_pc ();
            }
        }

      if (ignore_single_step_event)
        {
          /* Place the wave back into single-stepping state.  */
          wave.set_state (AMD_DBGAPI_WAVE_STATE_SINGLE_STEP);

          dbgapi_log (AMD_DBGAPI_LOG_LEVEL_INFO,
                      "%s (pc=%#lx) ignore spurious single-step",
                      to_string (wave.id ()).c_str (), pc);

          *state = wave.state ();
          *stop_reason = AMD_DBGAPI_WAVE_STOP_REASON_NONE;

          return;
        }
      else if (trap_raised)
        {
          uint8_t trap_id = 0;

          if (instruction && !is_trap (*instruction, &trap_id))
            {
              /* FIXME: We should be getting the stop reason from the trap
                 handler.  As a workaround, assume the the trap_id is of a
                 breakpoint instruction, that could have been removed by the
                 debugger since the trap was raised.  */
              trap_id = 7;
            }

          switch (trap_id)
            {
            case 1:
              reason_mask |= AMD_DBGAPI_WAVE_STOP_REASON_DEBUG_TRAP;
              break;
            case 2:
              reason_mask |= AMD_DBGAPI_WAVE_STOP_REASON_ASSERT_TRAP;
              break;
            case 7:
              reason_mask |= AMD_DBGAPI_WAVE_STOP_REASON_BREAKPOINT;
              break;
            default:
              reason_mask |= AMD_DBGAPI_WAVE_STOP_REASON_TRAP;
              break;
            }
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

      *stop_reason = reason_mask;

      if (!can_halt_at (instruction))
        wave.park ();
    }
}

void
amdgcn_architecture_t::set_wave_state (wave_t &wave,
                                       amd_dbgapi_wave_state_t state) const
{
  uint32_t status_reg, mode_reg;

  wave.read_register (amdgpu_regnum_t::status, &status_reg);
  wave.read_register (amdgpu_regnum_t::mode, &mode_reg);

  switch (state)
    {
    case AMD_DBGAPI_WAVE_STATE_STOP:
      mode_reg &= ~sq_wave_mode_debug_en_mask;
      status_reg |= sq_wave_status_halt_mask;
      break;

    case AMD_DBGAPI_WAVE_STATE_RUN:
      mode_reg &= ~sq_wave_mode_debug_en_mask;
      status_reg &= ~sq_wave_status_halt_mask;
      break;

    case AMD_DBGAPI_WAVE_STATE_SINGLE_STEP:
      mode_reg |= sq_wave_mode_debug_en_mask;
      status_reg &= ~sq_wave_status_halt_mask;
      break;

    default:
      dbgapi_assert (!"Invalid wave state");
    }

  wave.write_register (amdgpu_regnum_t::status, &status_reg);
  wave.write_register (amdgpu_regnum_t::mode, &mode_reg);

  /* If resuming the wave (run or single-step), clear the trap handler events
     in ttmp11 and the watchpoint exceptions in trapsts.  */
  if (state != AMD_DBGAPI_WAVE_STATE_STOP)
    {
      uint32_t ttmp11;
      wave.read_register (amdgpu_regnum_t::ttmp11, &ttmp11);
      ttmp11 &= ~ttmp11_trap_handler_events_mask;
      wave.write_register (amdgpu_regnum_t::ttmp11, &ttmp11);

      if (wave.stop_reason () & AMD_DBGAPI_WAVE_STOP_REASON_WATCHPOINT)
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

  /* SOP1 [10111110 1 SDST7 OP7 SSRC08] */
  return (encoding & 0xFF80FF00) == (0xBE800000 | (op8 & 0xFF) << 8);
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
amdgcn_architecture_t::is_code_end (const std::vector<uint8_t> &bytes) const
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

std::optional<os_watch_mode_t>
amdgcn_architecture_t::watchpoint_mode (
    amd_dbgapi_watchpoint_kind_t kind) const
{
  switch (kind)
    {
    case AMD_DBGAPI_WATCHPOINT_KIND_LOAD:
      return os_watch_mode_t::read;
    case AMD_DBGAPI_WATCHPOINT_KIND_STORE_AND_RMW:
      return os_watch_mode_t::nonread;
    case AMD_DBGAPI_WATCHPOINT_KIND_RMW:
      return os_watch_mode_t::atomic;
    case AMD_DBGAPI_WATCHPOINT_KIND_ALL:
      return os_watch_mode_t::all;
    }
  return {};
}

/* Base class for all GFX9 architectures.  */

class gfx9_base_t : public amdgcn_architecture_t
{
protected:
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

  virtual amdgpu_regnum_t
  scalar_operand_to_regnum (int operand) const override;

  gfx9_base_t (elf_amdgpu_machine_t e_machine, std::string target_triple)
      : amdgcn_architecture_t (e_machine, std::move (target_triple))
  {
  }

  virtual size_t scalar_alias_count () const override { return 6; }

  struct gfx9_cwsr_descriptor_t : cwsr_descriptor_t
  {
    gfx9_cwsr_descriptor_t (uint32_t compute_relaunch_wave,
                            uint32_t compute_relaunch_state,
                            amd_dbgapi_global_address_t context_save_address)
        : m_compute_relaunch_wave (compute_relaunch_wave),
          m_compute_relaunch_state (compute_relaunch_state),
          m_context_save_address (context_save_address)
    {
    }
    uint32_t const m_compute_relaunch_wave;
    uint32_t const m_compute_relaunch_state;
    amd_dbgapi_global_address_t const m_context_save_address;
  };

public:
  bool has_wave32_vgprs () const override { return false; }
  bool has_wave64_vgprs () const override { return true; }
  bool has_acc_vgprs () const override { return false; }

  bool can_halt_at_endpgm () const override { return false; }
  size_t largest_instruction_size () const override { return 8; }

  virtual uint64_t wave_get_info (const cwsr_descriptor_t &descriptor,
                                  wave_info_t query) const override;

  virtual std::optional<amd_dbgapi_global_address_t>
  register_address (const cwsr_descriptor_t &descriptor,
                    amdgpu_regnum_t regnum) const override;

  virtual void control_stack_iterate (
      const uint32_t *control_stack, size_t control_stack_words,
      amd_dbgapi_global_address_t wave_area_address,
      amd_dbgapi_size_t wave_area_size,
      const std::function<void (std::unique_ptr<cwsr_descriptor_t>)>
          &wave_callback) const override;

  virtual size_t watchpoint_mask_bits () const override
  {
    return utils::bit_mask (6, 29);
  }
};

uint64_t
gfx9_base_t::wave_get_info (const cwsr_descriptor_t &descriptor,
                            wave_info_t query) const
{
  const gfx9_cwsr_descriptor_t &gfx9_descriptor
      = static_cast<const gfx9_cwsr_descriptor_t &> (descriptor);

  uint32_t wave = gfx9_descriptor.m_compute_relaunch_wave;
  uint32_t state = gfx9_descriptor.m_compute_relaunch_state;

  switch (query)
    {
    case wave_info_t::vgprs:
      /* vgprs are allocated in blocks of 4 registers.  */
      return (1 + compute_relaunch_payload_vgprs (state)) * 4;

    case wave_info_t::acc_vgprs:
      return 0;

    case wave_info_t::sgprs:
      /* sgprs are allocated in blocks of 16 registers. Subtract the ttmps
         registers from this count, as they will be saved in a different area
         than the sgprs.  */
      return (1 + compute_relaunch_payload_sgprs (state)) * 16
             - /* ttmps */ 16;

    case wave_info_t::lds_size:
      return compute_relaunch_payload_lds_size (state) * 128
             * sizeof (uint32_t);

    case wave_info_t::lane_count:
      return 64;

    case wave_info_t::last_wave:
      return compute_relaunch_payload_last_wave (wave);

    case wave_info_t::first_wave:
      return compute_relaunch_payload_first_wave (wave);

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
gfx9_base_t::register_address (const cwsr_descriptor_t &descriptor,
                               amdgpu_regnum_t regnum) const
{
  if (regnum == amdgpu_regnum_t::null)
    return 0;

  size_t lane_count = wave_get_info (descriptor, wave_info_t::lane_count);
  amd_dbgapi_global_address_t save_area_addr
      = static_cast<const gfx9_cwsr_descriptor_t &> (descriptor)
            .m_context_save_address;

  if (wave_get_info (descriptor, wave_info_t::first_wave))
    {
      save_area_addr -= wave_get_info (descriptor, wave_info_t::lds_size);

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
    case amdgpu_regnum_t::dispatch_ptr:
      regnum = amdgpu_regnum_t::ttmp6;
      break;
    case amdgpu_regnum_t::dispatch_grid_x:
      regnum = amdgpu_regnum_t::ttmp8;
      break;
    case amdgpu_regnum_t::dispatch_grid_y:
      regnum = amdgpu_regnum_t::ttmp9;
      break;
    case amdgpu_regnum_t::dispatch_grid_z:
      regnum = amdgpu_regnum_t::ttmp10;
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

  size_t sgpr_count = wave_get_info (descriptor, wave_info_t::sgprs);
  size_t sgpr_size = sizeof (int32_t);
  size_t sgprs_addr = hwregs_addr - sgpr_count * sgpr_size;

  /* Exclude the aliased sgprs.  */
  if (regnum >= amdgpu_regnum_t::first_sgpr
      && regnum <= amdgpu_regnum_t::last_sgpr
      && (regnum - amdgpu_regnum_t::s0)
             >= (std::min (108ul, sgpr_count) - scalar_alias_count ()))
    return std::nullopt;

  /* Rename registers that alias to sgprs.  */
  if ((lane_count == 32 && regnum == amdgpu_regnum_t::vcc_32)
      || (lane_count == 64 && regnum == amdgpu_regnum_t::vcc_64)
      || regnum == amdgpu_regnum_t::vcc_lo)
    {
      regnum = amdgpu_regnum_t::s0 + std::min (108ul, sgpr_count) - 2;
    }
  if (regnum == amdgpu_regnum_t::vcc_hi)
    {
      regnum = amdgpu_regnum_t::s0 + std::min (108ul, sgpr_count) - 1;
    }

  /* Note: While EXEC_32 and EXEC_64 alias to sgpr_count - 2, the CWSR
     handler saves them in the hwreg block.  */

  if (regnum == amdgpu_regnum_t::flat_scratch
      || regnum == amdgpu_regnum_t::flat_scratch_lo)
    {
      regnum = amdgpu_regnum_t::s0 + std::min (108ul, sgpr_count) - 6;
    }
  if (regnum == amdgpu_regnum_t::flat_scratch_hi)
    {
      regnum = amdgpu_regnum_t::s0 + std::min (108ul, sgpr_count) - 5;
    }

  if (regnum >= amdgpu_regnum_t::first_sgpr
      && regnum <= amdgpu_regnum_t::last_sgpr
      && (regnum - amdgpu_regnum_t::s0) < std::min (108ul, sgpr_count))
    {
      return sgprs_addr + (regnum - amdgpu_regnum_t::s0) * sgpr_size;
    }

  size_t accvgpr_count = wave_get_info (descriptor, wave_info_t::acc_vgprs);
  size_t accvgpr_size = sizeof (int32_t) * lane_count;
  size_t accvgprs_addr = sgprs_addr - accvgpr_count * accvgpr_size;

  if (lane_count == 32 && regnum >= amdgpu_regnum_t::first_accvgpr_32
      && regnum <= amdgpu_regnum_t::last_accvgpr_32
      && ((regnum - amdgpu_regnum_t::acc0_32) < accvgpr_count))
    {
      return accvgprs_addr
             + (regnum - amdgpu_regnum_t::acc0_32) * accvgpr_size;
    }

  if (lane_count == 64 && regnum >= amdgpu_regnum_t::first_accvgpr_64
      && regnum <= amdgpu_regnum_t::last_accvgpr_64
      && ((regnum - amdgpu_regnum_t::acc0_64) < accvgpr_count))
    {
      return accvgprs_addr
             + (regnum - amdgpu_regnum_t::acc0_64) * accvgpr_size;
    }

  size_t vgpr_count = wave_get_info (descriptor, wave_info_t::vgprs);
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
    const uint32_t *control_stack, size_t control_stack_words,
    amd_dbgapi_global_address_t wave_area_address,
    amd_dbgapi_size_t wave_area_size,
    const std::function<void (std::unique_ptr<cwsr_descriptor_t>)>
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
          std::unique_ptr<cwsr_descriptor_t> descriptor (
              new gfx9_cwsr_descriptor_t{ relaunch, state,
                                          last_wave_area - 64 });

          last_wave_area
              = register_address (*descriptor, amdgpu_regnum_t::first_vgpr_64)
                    .value ();

          wave_callback (std::move (descriptor));
        }
    }

  /* After iterating the control stack, we should have consumed all the data in
     the wave save area, and last_wave_area should point to the bottom of the
     wave save area.  */
  if (last_wave_area != (wave_area_address - wave_area_size))
    error ("Corrupted control stack or wave save area");
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
public:
  gfx908_t ()
      : gfx9_base_t (EF_AMDGPU_MACH_AMDGCN_GFX908, "amdgcn-amd-amdhsa--gfx908")
  {
  }

  bool has_acc_vgprs () const override { return true; }

  virtual uint64_t wave_get_info (const cwsr_descriptor_t &descriptor,
                                  wave_info_t query) const override
  {
    switch (query)
      {
      case wave_info_t::acc_vgprs:
        return gfx9_base_t::wave_get_info (descriptor, wave_info_t::vgprs);

      default:
        return gfx9_base_t::wave_get_info (descriptor, query);
      }
  }
};

class gfx10_base_t : public gfx9_base_t
{
protected:
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

  struct gfx10_cwsr_descriptor_t : gfx9_cwsr_descriptor_t
  {
    gfx10_cwsr_descriptor_t (uint32_t compute_relaunch_wave,
                             uint32_t compute_relaunch_state,
                             uint32_t compute_relaunch2_state,
                             amd_dbgapi_global_address_t context_save_address)
        : gfx9_cwsr_descriptor_t (compute_relaunch_wave,
                                  compute_relaunch_state,
                                  context_save_address),
          m_compute_relaunch2_state (compute_relaunch2_state)
    {
    }
    /* On gfx10, there are 2 COMPUTE_RELAUNCH registers for state.  */
    uint32_t const m_compute_relaunch2_state;
  };

  virtual amdgpu_regnum_t
  scalar_operand_to_regnum (int operand) const override;

  gfx10_base_t (elf_amdgpu_machine_t e_machine, std::string target_triple)
      : gfx9_base_t (e_machine, std::move (target_triple))
  {
  }

  virtual size_t scalar_alias_count () const override { return 2; }

public:
  bool has_wave32_vgprs () const override { return true; }
  bool has_wave64_vgprs () const override { return true; }
  bool has_acc_vgprs () const override { return false; }

  bool is_code_end (const std::vector<uint8_t> &bytes) const override;
  bool is_call (const std::vector<uint8_t> &bytes) const override;
  bool is_getpc (const std::vector<uint8_t> &bytes) const override;
  bool is_setpc (const std::vector<uint8_t> &bytes) const override;
  bool is_swappc (const std::vector<uint8_t> &bytes) const override;

  std::optional<amd_dbgapi_global_address_t>
  register_address (const cwsr_descriptor_t &descriptor,
                    amdgpu_regnum_t regnum) const override;

  virtual uint64_t wave_get_info (const cwsr_descriptor_t &descriptor,
                                  wave_info_t query) const override;

  virtual void control_stack_iterate (
      const uint32_t *control_stack, size_t control_stack_words,
      amd_dbgapi_global_address_t wave_area_address,
      amd_dbgapi_size_t wave_area_size,
      const std::function<void (std::unique_ptr<cwsr_descriptor_t>)>
          &wave_callback) const override;

  bool can_halt_at_endpgm () const override { return false; }
  size_t largest_instruction_size () const override { return 20; }

  virtual size_t watchpoint_mask_bits () const override
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
gfx10_base_t::register_address (const cwsr_descriptor_t &descriptor,
                                amdgpu_regnum_t regnum) const
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

  return gfx9_base_t::register_address (descriptor, regnum);
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

uint64_t
gfx10_base_t::wave_get_info (const cwsr_descriptor_t &descriptor,
                             wave_info_t query) const
{
  const gfx10_cwsr_descriptor_t &gfx10_descriptor
      = static_cast<const gfx10_cwsr_descriptor_t &> (descriptor);

  uint32_t wave = gfx10_descriptor.m_compute_relaunch_wave;
  uint32_t state = gfx10_descriptor.m_compute_relaunch_state;

  switch (query)
    {
    case wave_info_t::sgprs:
      return 128;

    case wave_info_t::vgprs:
      /* vgprs are allocated in blocks of 8/4 registers (W32/W64).  */
      return (1 + compute_relaunch_payload_vgprs (state))
             * (compute_relaunch_payload_w32_en (state) ? 8 : 4);

    case wave_info_t::lane_count:
      return compute_relaunch_payload_w32_en (state) ? 32 : 64;

    case wave_info_t::lds_size:
      /* lds_size: 128 dwords granularity.  */
      return compute_relaunch_payload_lds_size (state) * 128
             * sizeof (uint32_t);

    case wave_info_t::last_wave:
      return compute_relaunch_payload_last_wave (wave);

    case wave_info_t::first_wave:
      return compute_relaunch_payload_first_wave (wave);

    default:
      return gfx9_base_t::wave_get_info (descriptor, query);
    }
}

void
gfx10_base_t::control_stack_iterate (
    const uint32_t *control_stack, size_t control_stack_words,
    amd_dbgapi_global_address_t wave_area_address,
    amd_dbgapi_size_t wave_area_size,
    const std::function<void (std::unique_ptr<cwsr_descriptor_t>)>
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
          std::unique_ptr<cwsr_descriptor_t> descriptor (
              new gfx10_cwsr_descriptor_t{ relaunch, state0, state1,
                                           last_wave_area });

          last_wave_area
              = register_address (
                    *descriptor,
                    wave_get_info (*descriptor, wave_info_t::lane_count) == 32
                        ? amdgpu_regnum_t::first_vgpr_32
                        : amdgpu_regnum_t::first_vgpr_64)
                    .value ();

          wave_callback (std::move (descriptor));
        }
    }

  /* After iterating the control stack, we should have consumed all the data in
     the wave save area, and last_wave_area should point to the bottom of the
     wave save area.  */
  if (last_wave_area != (wave_area_address - wave_area_size))
    error ("Corrupted control stack or wave save area");
}

class gfx1010_t final : public gfx10_base_t
{
public:
  gfx1010_t ()
      : gfx10_base_t (EF_AMDGPU_MACH_AMDGCN_GFX1010,
                      "amdgcn-amd-amdhsa--gfx1010")
  {
  }
};

class gfx1011_t final : public gfx10_base_t
{
public:
  gfx1011_t ()
      : gfx10_base_t (EF_AMDGPU_MACH_AMDGCN_GFX1011,
                      "amdgcn-amd-amdhsa--gfx1011")
  {
  }
};

class gfx1012_t final : public gfx10_base_t
{
public:
  gfx1012_t ()
      : gfx10_base_t (EF_AMDGPU_MACH_AMDGCN_GFX1012,
                      "amdgcn-amd-amdhsa--gfx1012")
  {
  }
};

class gfx1030_t final : public gfx10_base_t
{
public:
  gfx1030_t ()
      : gfx10_base_t (EF_AMDGPU_MACH_AMDGCN_GFX1030,
                      "amdgcn-amd-amdhsa--gfx1030")
  {
  }
};

class gfx1031_t final : public gfx10_base_t
{
public:
  gfx1031_t ()
      : gfx10_base_t (EF_AMDGPU_MACH_AMDGCN_GFX1031,
                      "amdgcn-amd-amdhsa--gfx1031")
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

const architecture_t *
architecture_t::find (amd_dbgapi_architecture_id_t architecture_id, int ignore)
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

bool
architecture_t::can_halt_at (
    const std::optional<std::vector<uint8_t>> &instruction) const
{
  /* A wave cannot halt at an s_endpgm instruction. An s_trap may be used as a
     breakpoint instruction, and since it could be removed and the original
     instruction restored, which could reveal an s_endpgm, we also cannot halt
     at an s_strap. */
  return can_halt_at_endpgm ()
         || (instruction && !is_endpgm (*instruction)
             && !is_breakpoint (*instruction));
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
      && regnum <= amdgpu_regnum_t::last_sgpr)
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
      return string_printf ("acc%ld",
                            regnum - amdgpu_regnum_t::first_accvgpr_32);
    }
  if (has_acc_vgprs () && has_wave64_vgprs ()
      && regnum >= amdgpu_regnum_t::first_accvgpr_64
      && regnum <= amdgpu_regnum_t::last_accvgpr_64)
    {
      return string_printf ("acc%ld",
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
          return {};
        }
    }
  if (regnum >= amdgpu_regnum_t::first_hwreg
      && regnum <= amdgpu_regnum_t::last_hwreg)
    {
      return string_printf ("hwreg%ld", regnum - amdgpu_regnum_t::first_hwreg);
    }

  if ((has_wave32_vgprs () && regnum == amdgpu_regnum_t::exec_32)
      || (has_wave64_vgprs () && regnum == amdgpu_regnum_t::exec_64))
    {
      return "exec";
    }
  if ((has_wave32_vgprs () && regnum == amdgpu_regnum_t::vcc_32)
      || (has_wave64_vgprs () && regnum == amdgpu_regnum_t::vcc_64))
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
    case amdgpu_regnum_t::dispatch_ptr:
      return "dispatch_ptr";
    case amdgpu_regnum_t::dispatch_grid_x:
      return "grid_x";
    case amdgpu_regnum_t::dispatch_grid_y:
      return "grid_y";
    case amdgpu_regnum_t::dispatch_grid_z:
      return "grid_z";
    case amdgpu_regnum_t::scratch_offset:
      return "scratch_offset";
    case amdgpu_regnum_t::null:
      return "null";
    default:
      break;
    }
  return {};
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
      && regnum <= amdgpu_regnum_t::last_sgpr)
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
          || regnum == amdgpu_regnum_t::vcc_32
          || regnum == amdgpu_regnum_t::xnack_mask_32))
    {
      return "uint32_t";
    }
  if (has_wave64_vgprs ()
      && (regnum == amdgpu_regnum_t::exec_64
          || regnum == amdgpu_regnum_t::vcc_64
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
    case amdgpu_regnum_t::dispatch_grid_x:
    case amdgpu_regnum_t::dispatch_grid_y:
    case amdgpu_regnum_t::dispatch_grid_z:
    case amdgpu_regnum_t::scratch_offset:
    case amdgpu_regnum_t::null:
      return "uint32_t";

    case amdgpu_regnum_t::wave_id:
    case amdgpu_regnum_t::flat_scratch:
    case amdgpu_regnum_t::dispatch_ptr:
      return "uint64_t";

    default:
      return {};
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
      && regnum <= amdgpu_regnum_t::last_sgpr)
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
          || regnum == amdgpu_regnum_t::vcc_32
          || regnum == amdgpu_regnum_t::xnack_mask_32))
    {
      return sizeof (uint32_t);
    }
  if (has_wave64_vgprs ()
      && (regnum == amdgpu_regnum_t::exec_64
          || regnum == amdgpu_regnum_t::vcc_64
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
    case amdgpu_regnum_t::dispatch_grid_x:
    case amdgpu_regnum_t::dispatch_grid_y:
    case amdgpu_regnum_t::dispatch_grid_z:
    case amdgpu_regnum_t::scratch_offset:
    case amdgpu_regnum_t::null:
      return sizeof (uint32_t);

    case amdgpu_regnum_t::wave_id:
    case amdgpu_regnum_t::flat_scratch:
    case amdgpu_regnum_t::dispatch_ptr:
      return sizeof (uint64_t);

    default:
      return {};
    }
}

namespace detail
{

struct disassembly_user_data_t
{
  const void *memory;
  amd_dbgapi_size_t offset;
  amd_dbgapi_size_t size;
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

size_t
architecture_t::instruction_size (const std::vector<uint8_t> &bytes) const
{
  size_t size = bytes.size ();

  if (instruction_size (bytes.data (), &size) != AMD_DBGAPI_STATUS_SUCCESS)
    return 0;

  return size;
}

amd_dbgapi_status_t
architecture_t::instruction_size (const void *memory, size_t *size) const
{
  struct detail::disassembly_user_data_t user_data = { .memory = memory,
                                                       .offset = 0,
                                                       .size = *size,
                                                       .instruction = nullptr,
                                                       .operands = nullptr };

  /* Disassemble one instruction.  */
  return (amd_comgr_disassemble_instruction (disassembly_info (), 0,
                                             &user_data, size))
             ? AMD_DBGAPI_STATUS_ERROR
             : AMD_DBGAPI_STATUS_SUCCESS;
}

amd_dbgapi_status_t
architecture_t::disassemble_instruction (
    amd_dbgapi_global_address_t address, amd_dbgapi_size_t *size,
    const void *instruction_bytes, std::string &instruction_text,
    std::vector<amd_dbgapi_global_address_t> &address_operands) const
{
  instruction_text.clear ();
  address_operands.clear ();

  struct detail::disassembly_user_data_t user_data
      = { .memory = instruction_bytes,
          .offset = address,
          .size = *size,
          .instruction = &instruction_text,
          .operands = &address_operands };

  /* Disassemble one instruction.  */
  return amd_comgr_disassemble_instruction (disassembly_info (),
                                            static_cast<uint64_t> (address),
                                            &user_data, size)
             ? AMD_DBGAPI_STATUS_ERROR
             : AMD_DBGAPI_STATUS_SUCCESS;
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
  TRY;
  TRACE (elf_amdgpu_machine);

  if (!detail::is_initialized)
    return AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED;

  if (!architecture_id)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;

  const architecture_t *architecture = architecture_t::find (
      static_cast<elf_amdgpu_machine_t> (elf_amdgpu_machine));

  if (!architecture)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ELF_AMDGPU_MACHINE;

  *architecture_id = { architecture->id () };

  return AMD_DBGAPI_STATUS_SUCCESS;
  CATCH;
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_architecture_get_info (amd_dbgapi_architecture_id_t architecture_id,
                                  amd_dbgapi_architecture_info_t query,
                                  size_t value_size, void *value)
{
  TRY;
  TRACE (architecture_id, query, value_size, value);

  if (!detail::is_initialized)
    return AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED;

  const architecture_t *architecture = architecture_t::find (architecture_id);

  if (!architecture)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARCHITECTURE_ID;

  return architecture->get_info (query, value_size, value);
  CATCH;
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_disassemble_instruction (
    amd_dbgapi_architecture_id_t architecture_id,
    amd_dbgapi_global_address_t address, amd_dbgapi_size_t *size,
    const void *memory, char **instruction_text,
    amd_dbgapi_symbolizer_id_t symbolizer_id,
    amd_dbgapi_status_t (*symbolizer) (
        amd_dbgapi_symbolizer_id_t symbolizer_id,
        amd_dbgapi_global_address_t address, char **symbol_text))
{
  TRY;
  TRACE (architecture_id, address, size);

  if (!detail::is_initialized)
    return AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED;

  if (!memory || !size)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;

  const architecture_t *architecture = architecture_t::find (architecture_id);

  if (!architecture)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARCHITECTURE_ID;

  if (!instruction_text)
    return architecture->instruction_size (memory, size);

  std::string instruction_str;
  std::vector<amd_dbgapi_global_address_t> operands_vec;

  amd_dbgapi_status_t status = architecture->disassemble_instruction (
      address, size, memory, instruction_str, operands_vec);
  if (status != AMD_DBGAPI_STATUS_SUCCESS)
    return status;

  std::string operand_str;
  for (auto addr : operands_vec)
    {
      operand_str += operand_str.empty () ? "  # " : ", ";

      if (symbolizer)
        {
          char *symbol_text = nullptr;
          status = (*symbolizer) (symbolizer_id, addr, &symbol_text);
          if (status == AMD_DBGAPI_STATUS_SUCCESS)
            {
              if (!symbol_text)
                return AMD_DBGAPI_STATUS_ERROR;
              std::string symbol_string = symbol_text;
              deallocate_memory (symbol_text);
              if (symbol_string.empty ())
                return AMD_DBGAPI_STATUS_ERROR;
              operand_str += symbol_string;
              continue;
            }
          else if (status != AMD_DBGAPI_STATUS_ERROR_SYMBOL_NOT_FOUND)
            return AMD_DBGAPI_STATUS_ERROR_CLIENT_CALLBACK;
        }

      std::stringstream sstream;
      sstream << "0x" << std::hex << std::setfill ('0')
              << std::setw (sizeof (addr) * 2) << addr;
      operand_str += sstream.str ();
    }
  instruction_str += operand_str;

  /* Return the instruction text in client allocated memory.  */
  size_t mem_size = instruction_str.length () + 1;
  void *mem = allocate_memory (mem_size);
  if (!mem)
    return AMD_DBGAPI_STATUS_ERROR_CLIENT_CALLBACK;

  memcpy (mem, instruction_str.c_str (), mem_size);
  *instruction_text = static_cast<char *> (mem);

  return AMD_DBGAPI_STATUS_SUCCESS;
  CATCH;
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_classify_instruction (
    amd_dbgapi_architecture_id_t architecture_id,
    amd_dbgapi_global_address_t address, amd_dbgapi_size_t *size_p,
    const void *memory, amd_dbgapi_instruction_kind_t *instruction_kind_p,
    void **instruction_properties_p)
{
  TRY;
  TRACE (architecture_id, address);

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

  auto [kind, size, properties]
      = architecture->classify_instruction (instruction, address);

  if (instruction_properties_p)
    {
      size_t mem_size
          = properties.size () * sizeof (decltype (properties)::value_type);

      if (!mem_size)
        {
          *instruction_properties_p = nullptr;
        }
      else
        {
          void *mem = allocate_memory (mem_size);
          if (!mem)
            return AMD_DBGAPI_STATUS_ERROR_CLIENT_CALLBACK;

          memcpy (mem, properties.data (), mem_size);
          *instruction_properties_p = mem;
        }
    }

  *size_p = size;
  *instruction_kind_p = kind;

  return AMD_DBGAPI_STATUS_SUCCESS;
  CATCH;
}
