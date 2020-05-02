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
#include "displaced_stepping.h"
#include "logging.h"
#include "process.h"
#include "register.h"
#include "utils.h"
#include "wave.h"

#include <algorithm>
#include <cstring>
#include <iomanip>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <amd_comgr.h>
#include <ctype.h>

namespace amd
{
namespace dbgapi
{

#if defined(__GNUC__)
#define __maybe_unused__ __attribute__ ((unused))
#else /* !defined(__GNUC__) */
#define __maybe_unused__
#endif /* !defined(__GNUC__) */

constexpr uint32_t SQ_WAVE_STATUS_SCC_MASK = (1 << 0);
constexpr uint32_t SQ_WAVE_STATUS_EXECZ_MASK = (1 << 9);
constexpr uint32_t SQ_WAVE_STATUS_VCCZ_MASK = (1 << 10);
constexpr uint32_t SQ_WAVE_STATUS_COND_DBG_USER_MASK = (1 << 20);
constexpr uint32_t SQ_WAVE_STATUS_COND_DBG_SYS_MASK = (1 << 21);

constexpr uint32_t TTMP11_WAVE_IN_GROUP_MASK = 0x003F;
constexpr uint32_t TTMP11_TRAP_HANDLER_TRAP_RAISED_MASK = 0x0080;
constexpr uint32_t TTMP11_TRAP_HANDLER_EXCP_RAISED_MASK = 0x0100;
constexpr uint32_t TTMP11_TRAP_HANDLER_EVENTS_MASK
    = (TTMP11_TRAP_HANDLER_TRAP_RAISED_MASK
       | TTMP11_TRAP_HANDLER_EXCP_RAISED_MASK);

constexpr uint32_t SQ_WAVE_STATUS_HALT_MASK = 0x2000;
constexpr uint32_t SQ_WAVE_MODE_DEBUG_EN_MASK = 0x800;

constexpr int SQ_EX_MODE_EXCP_INVALID = 0;
constexpr int SQ_EX_MODE_EXCP_INPUT_DENORM = 1;
constexpr int SQ_EX_MODE_EXCP_DIV0 = 2;
constexpr int SQ_EX_MODE_EXCP_OVERFLOW = 3;
constexpr int SQ_EX_MODE_EXCP_UNDERFLOW = 4;
constexpr int SQ_EX_MODE_EXCP_INEXACT = 5;
constexpr int SQ_EX_MODE_EXCP_INT_DIV0 = 6;
constexpr int __maybe_unused__ SQ_EX_MODE_EXCP_ADDR_WATCH0 = 7;
constexpr int SQ_EX_MODE_EXCP_MEM_VIOL = 8;

constexpr int __maybe_unused__ SQ_EX_MODE_EXCP_HI_ADDR_WATCH1 = 0;
constexpr int __maybe_unused__ SQ_EX_MODE_EXCP_HI_ADDR_WATCH2 = 1;
constexpr int __maybe_unused__ SQ_EX_MODE_EXCP_HI_ADDR_WATCH3 = 2;

constexpr uint32_t SQ_WAVE_TRAPSTS_EXCP_MASK = 0x1FF;
constexpr uint32_t __maybe_unused__ SQ_WAVE_TRAPSTS_SAVECTX_MASK = 0x400;
constexpr uint32_t SQ_WAVE_TRAPSTS_ILLEGAL_INST_MASK = 0x800;
constexpr uint32_t __maybe_unused__ SQ_WAVE_TRAPSTS_EXCP_HI_MASK = 0x7000;

monotonic_counter_t<decltype (amd_dbgapi_architecture_id_t::handle)>
    architecture_t::s_next_architecture_id{ 1 };

/* Base class for all AMDGCN architectures.  */

class amdgcn_architecture_t : public architecture_t
{
protected:
  amdgcn_architecture_t (uint8_t gfxip_major, uint8_t gfxip_minor,
                         uint8_t gfxip_stepping)
      : architecture_t (gfxip_major, gfxip_minor, gfxip_stepping)
  {
  }

public:
  virtual void initialize () override;

  virtual const address_space_t &default_global_address_space () const override
  {
    dbgapi_assert (m_default_global_address_space
                   && "address spaces are not initialized");
    return *m_default_global_address_space;
  }

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

  virtual size_t displaced_stepping_buffer_size () const override;

  virtual bool
  displaced_stepping_copy (displaced_stepping_t &displaced_stepping,
                           bool *simulate) const override;
  virtual bool displaced_stepping_fixup (
      wave_t &wave, displaced_stepping_t &displaced_stepping) const override;
  virtual bool displaced_stepping_simulate (
      wave_t &wave, displaced_stepping_t &displaced_stepping) const override;

  virtual amd_dbgapi_status_t
  get_wave_coords (wave_t &wave, uint32_t (&group_ids)[3],
                   uint32_t *wave_in_group) const override;

  virtual amd_dbgapi_status_t
  get_wave_state (wave_t &wave, amd_dbgapi_wave_state_t *state,
                  amd_dbgapi_wave_stop_reason_t *stop_reason) const override;
  virtual amd_dbgapi_status_t
  set_wave_state (wave_t &wave, amd_dbgapi_wave_state_t state) const override;

  virtual size_t minimum_instruction_alignment () const override;
  virtual const std::vector<uint8_t> &nop_instruction () const override;
  virtual const std::vector<uint8_t> &breakpoint_instruction () const override;
  virtual const std::vector<uint8_t> &endpgm_instruction () const override;
  virtual size_t breakpoint_instruction_pc_adjust () const override;

protected:
  /* Instruction decoding helpers.  */

  enum class cbranch_cond_t
  {
    SCC0,             /* Scalar condition code is 0.  */
    SCC1,             /* Scalar condition code is 1.  */
    EXECZ,            /* All EXEC mask bits are zero.  */
    EXECNZ,           /* Not all EXEC mask bits are zero.  */
    VCCZ,             /* All Vector Condition Code bits are zero.  */
    VCCNZ,            /* Not all Vector Condition Code bits are zero.  */
    CDBGSYS,          /* Conditional Debug for System is 1.  */
    CDBGUSER,         /* Conditional Debug for User is 1.  */
    CDBGSYS_OR_USER,  /* Conditional Debug for System or User is 1.  */
    CDBGSYS_AND_USER, /* Conditional Debug for System and User is 1.  */
  };

  static const std::unordered_map<uint16_t, cbranch_cond_t>
      cbranch_opcodes_map;

  static uint8_t encoding_ssrc0 (const std::vector<uint8_t> &bytes);
  static uint8_t encoding_sdst (const std::vector<uint8_t> &bytes);
  static uint8_t encoding_op7 (const std::vector<uint8_t> &bytes);
  static int encoding_simm16 (const std::vector<uint8_t> &bytes);

  /* Return the regnum for a scalar register operand.  */
  static amdgpu_regnum_t scalar_operand_to_regnum (wave_t &wave, int code);

  virtual bool is_call (const std::vector<uint8_t> &bytes) const;
  virtual bool is_getpc (const std::vector<uint8_t> &bytes) const;
  virtual bool is_setpc (const std::vector<uint8_t> &bytes) const;
  virtual bool is_swappc (const std::vector<uint8_t> &bytes) const;
  virtual bool is_branch (const std::vector<uint8_t> &bytes) const;
  virtual bool is_cbranch (const std::vector<uint8_t> &bytes) const;
  virtual bool is_cbranch_i_fork (const std::vector<uint8_t> &bytes) const;
  virtual bool is_endpgm (const std::vector<uint8_t> &bytes) const override;
  virtual bool is_trap (const std::vector<uint8_t> &bytes,
                        uint16_t *trap_id = nullptr) const override;

  virtual amd_dbgapi_global_address_t
  branch_target (wave_t &wave, amd_dbgapi_global_address_t pc,
                 const std::vector<uint8_t> &instruction) const;

  virtual amd_dbgapi_status_t
  simulate_instruction (wave_t &wave, amd_dbgapi_global_address_t pc,
                        const std::vector<uint8_t> &instruction) const;

private:
  const address_space_t *m_default_global_address_space{ nullptr };
};

decltype (amdgcn_architecture_t::cbranch_opcodes_map)
    amdgcn_architecture_t::cbranch_opcodes_map{
      { 4, cbranch_cond_t::SCC0 },
      { 5, cbranch_cond_t::SCC1 },
      { 6, cbranch_cond_t::VCCZ },
      { 7, cbranch_cond_t::VCCNZ },
      { 8, cbranch_cond_t::EXECZ },
      { 9, cbranch_cond_t::EXECNZ },
      { 23, cbranch_cond_t::CDBGSYS },
      { 24, cbranch_cond_t::CDBGUSER },
      { 25, cbranch_cond_t::CDBGSYS_OR_USER },
      { 26, cbranch_cond_t::CDBGSYS_AND_USER },
    };

void
amdgcn_architecture_t::initialize ()
{
  /* Create address spaces.  */

  auto &as_global = create<address_space_t> (
      "none", address_space_t::GLOBAL, DW_ASPACE_none, 64, 0x0000000000000000,
      AMD_DBGAPI_ADDRESS_SPACE_ACCESS_ALL);

  auto &as_generic = create<address_space_t> (
      "generic", address_space_t::GENERIC, DW_ASPACE_AMDGPU_generic, 64,
      0x0000000000000000, AMD_DBGAPI_ADDRESS_SPACE_ACCESS_ALL);

  auto &as_region = create<address_space_t> (
      "region", address_space_t::REGION, DW_ASPACE_AMDGPU_region, 32,
      0xFFFFFFFF, AMD_DBGAPI_ADDRESS_SPACE_ACCESS_ALL);

  auto &as_local = create<address_space_t> (
      "local", address_space_t::LOCAL, DW_ASPACE_AMDGPU_local, 32, 0xFFFFFFFF,
      AMD_DBGAPI_ADDRESS_SPACE_ACCESS_ALL);

  auto &as_private_lane = create<address_space_t> (
      "private_lane", address_space_t::PRIVATE_SWIZZLED,
      DW_ASPACE_AMDGPU_private_lane, 32, 0x00000000,
      AMD_DBGAPI_ADDRESS_SPACE_ACCESS_ALL);

  create<address_space_t> ("private_wave", address_space_t::PRIVATE_UNSWIZZLED,
                           DW_ASPACE_AMDGPU_private_wave, 32, 0x00000000,
                           AMD_DBGAPI_ADDRESS_SPACE_ACCESS_ALL);

  for (int i = 0; i < 63; i++)
    create<address_space_t> (string_printf ("private_lane%d", i),
                             address_space_t::PRIVATE_SWIZZLED_N,
                             DW_ASPACE_AMDGPU_private_lane0 + i, 32,
                             0x00000000, AMD_DBGAPI_ADDRESS_SPACE_ACCESS_ALL);

  m_default_global_address_space = &as_generic; // FIXME: &as_global;

  /* Create address classes.  */

  create<address_class_t> ("none", DW_ADDR_none, as_generic);
  create<address_class_t> ("global", DW_ADDR_LLVM_global, as_global);
  create<address_class_t> ("constant", DW_ADDR_LLVM_constant, as_global);
  create<address_class_t> ("group", DW_ADDR_LLVM_group, as_local);
  create<address_class_t> ("private", DW_ADDR_LLVM_private, as_private_lane);
  create<address_class_t> ("region", DW_ADDR_AMDGPU_region, as_region);

  /* Create register classes.  */

  /* Scalar registers: [s0-s111]  */
  register_class_t::register_map_t scalar_registers;
  scalar_registers.emplace (amdgpu_regnum_t::FIRST_SGPR,
                            amdgpu_regnum_t::LAST_SGPR);
  create<register_class_t> ("scalar", scalar_registers);

  /* Vector registers: [v0-v255, acc0-acc255]  */
  register_class_t::register_map_t vector_registers;
  if (has_wave32_vgprs ())
    vector_registers.emplace (amdgpu_regnum_t::FIRST_VGPR_32,
                              amdgpu_regnum_t::LAST_VGPR_32);
  if (has_wave64_vgprs ())
    vector_registers.emplace (amdgpu_regnum_t::FIRST_VGPR_64,
                              amdgpu_regnum_t::LAST_VGPR_64);
  if (has_acc_vgprs ())
    vector_registers.emplace (amdgpu_regnum_t::FIRST_ACCVGPR_64,
                              amdgpu_regnum_t::LAST_ACCVGPR_64);
  create<register_class_t> ("vector", vector_registers);

  /* System registers: [hwreg0-hwreg15, flat_scratch, xnack_mask]  */
  register_class_t::register_map_t system_registers;
  system_registers.emplace (amdgpu_regnum_t::FIRST_HWREG,
                            amdgpu_regnum_t::LAST_HWREG);
  system_registers.emplace (amdgpu_regnum_t::FIRST_TTMP,
                            amdgpu_regnum_t::LAST_TTMP);
  system_registers.emplace (amdgpu_regnum_t::FLAT_SCRATCH,
                            amdgpu_regnum_t::FLAT_SCRATCH);
  system_registers.emplace (amdgpu_regnum_t::XNACK_MASK_64,
                            amdgpu_regnum_t::XNACK_MASK_64);
  create<register_class_t> ("system", system_registers);

  /* General registers: [{scalar}, {vector}, pc, exec, vcc]  */
  register_class_t::register_map_t general_registers;
  general_registers.insert (scalar_registers.begin (),
                            scalar_registers.end ());
  general_registers.insert (vector_registers.begin (),
                            vector_registers.end ());
  general_registers.emplace (amdgpu_regnum_t::PC, amdgpu_regnum_t::PC);
  if (has_wave32_vgprs ())
    {
      general_registers.emplace (amdgpu_regnum_t::EXEC_32,
                                 amdgpu_regnum_t::EXEC_32);
      general_registers.emplace (amdgpu_regnum_t::VCC_32,
                                 amdgpu_regnum_t::VCC_32);
    }
  if (has_wave64_vgprs ())
    {
      general_registers.emplace (amdgpu_regnum_t::EXEC_64,
                                 amdgpu_regnum_t::EXEC_64);
      general_registers.emplace (amdgpu_regnum_t::VCC_64,
                                 amdgpu_regnum_t::VCC_64);
    }
  create<register_class_t> ("general", general_registers);
}

/* Helper routine to return the address space for a given generic address. The
   returned address space can only be one of LOCAL, PRIVATE_SWIZZLED or GLOBAL
   address space. The queue_t is used to retrieve the apertures.
 */
static inline const address_space_t &
address_space_for_generic_address (
    const wave_t &wave, amd_dbgapi_segment_address_t generic_address)
{
  address_space_t::address_space_kind_t address_space_kind;
  amd_dbgapi_global_address_t aperture
      = generic_address & utils::bit_mask (32, 63);

  if (aperture == wave.queue ().private_address_space_aperture ())
    address_space_kind = address_space_t::PRIVATE_SWIZZLED;
  else if (aperture == wave.queue ().shared_address_space_aperture ())
    address_space_kind = address_space_t::LOCAL;
  else /* all other addresses are treated as global addresses  */
    address_space_kind = address_space_t::GLOBAL;

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
static inline std::pair<amd_dbgapi_segment_address_t, bool>
generic_address_for_address_space (
    const wave_t &wave, const address_space_t &segment_address_space,
    amd_dbgapi_segment_address_t segment_address)
{
  amd_dbgapi_segment_address_t aperture{ 0 };

  if (segment_address_space.kind () == address_space_t::LOCAL)
    aperture = wave.queue ().shared_address_space_aperture ();
  if (segment_address_space.kind () == address_space_t::PRIVATE_SWIZZLED)
    aperture = wave.queue ().private_address_space_aperture ();
  else if (segment_address_space.kind () != address_space_t::GLOBAL)
    /* not a valid address space conversion.  */
    return std::make_pair (segment_address_space.null_address (), false);

  if (segment_address == segment_address_space.null_address ())
    return std::make_pair (segment_address_space.null_address (), true);

  segment_address
      &= utils::bit_mask (0, segment_address_space.address_size () - 1);

  return std::make_pair (aperture | segment_address, true);
}

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

  if (from_address_space.kind () == address_space_t::GENERIC)
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

  /* Other conversions from local, private or global can only be to the
     the generic address space.  */

  if (to_address_space.kind () != address_space_t::GENERIC)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ADDRESS_SPACE_CONVERSION;

  bool valid_conversion;
  std::tie (*to_address, valid_conversion)
      = generic_address_for_address_space (wave, from_address_space,
                                           from_address);
  if (!valid_conversion)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ADDRESS_SPACE_CONVERSION;

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
  if (original_address_space.kind () == address_space_t::GENERIC)
    {
      const address_space_t &segment_address_space
          = address_space_for_generic_address (wave, original_address);

      wave.architecture ().convert_address_space (
          wave, AMD_DBGAPI_LANE_NONE, original_address_space,
          segment_address_space, original_address, lowered_address);

      *lowered_address_space = &segment_address_space;
      return;
    };

  if (original_address_space.kind () == address_space_t::PRIVATE_SWIZZLED_N)
    {
      uint64_t dwarf_value = original_address_space.dwarf_value ();
      if (dwarf_value >= DW_ASPACE_AMDGPU_private_lane0
          && dwarf_value <= DW_ASPACE_AMDGPU_private_lane63)
        {
          const address_space_t *as_private_lane
              = wave.architecture ().find_if ([=] (const address_space_t &as) {
                  return as.kind () == address_space_t::PRIVATE_SWIZZLED;
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
  if (address_space.kind () == address_space_t::PRIVATE_SWIZZLED_N
      || address_space.kind () == address_space_t::PRIVATE_UNSWIZZLED)
    return false;

  /* private_swizzled, local, global, and generic are in the generic address
     class.  */
  if (address_class.address_space ().kind () == address_space_t::GENERIC)
    return true;

  if (address_space.kind () == address_space_t::GENERIC)
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
  if (address_space1.kind () == address_space_t::GENERIC
      || address_space2.kind () == address_space_t::GENERIC)
    return true;

  auto is_private = [] (const address_space_t &address_space) {
    return address_space.kind () == address_space_t::PRIVATE_SWIZZLED
           || address_space.kind () == address_space_t::PRIVATE_SWIZZLED_N
           || address_space.kind () == address_space_t::PRIVATE_UNSWIZZLED;
  };

  /* private* aliases with private*.  */
  if (is_private (address_space1) && is_private (address_space2))
    return true;

  return false;
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
    0x07, 0x00, 0x92, 0xBF /* s_trap 7 */
  };

  return s_breakpoint_instruction_bytes;
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

/* Return the regnum for a scalar register operand.  */
amdgpu_regnum_t
amdgcn_architecture_t::scalar_operand_to_regnum (wave_t &wave, int operand)
{
  if (operand >= 0 && operand <= 101)
    {
      /* SGPR[0] through SGPR[101]  */
      return amdgpu_regnum_t::S0 + operand;
    }
  else if (operand >= 102 && operand <= 107)
    {
      /* FLAT_SCRATCH[102:103], XNACK_MASK[104:105], VCC[106:107]  */
      return amdgpu_regnum_t::S0 + (wave.sgpr_count () + operand - 108);
    }
  else if (operand >= 108 && operand <= 123)
    {
      /* TTMP[0] through TTMP[15]  */
      return amdgpu_regnum_t::FIRST_TTMP + (operand - 108);
    }
  else if (operand >= 126 && operand <= 127)
    {
      return amdgpu_regnum_t::EXEC_32 + (operand - 126);
    }
  else
    {
      error ("Invalid scalar operand");
    }
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
      if (wave.read_register (amdgpu_regnum_t::STATUS, &status_reg)
          != AMD_DBGAPI_STATUS_SUCCESS)
        error ("wave::read_register failed");

      /* Evaluate the condition.  */
      bool branch_taken{};
      switch (cbranch_opcodes_map.find (encoding_op7 (instruction))->second)
        {
        case cbranch_cond_t::SCC0:
          branch_taken = (status_reg & SQ_WAVE_STATUS_SCC_MASK) == 0;
          break;
        case cbranch_cond_t::SCC1:
          branch_taken = (status_reg & SQ_WAVE_STATUS_SCC_MASK) != 0;
          break;
        case cbranch_cond_t::EXECZ:
          branch_taken = (status_reg & SQ_WAVE_STATUS_EXECZ_MASK) != 0;
          break;
        case cbranch_cond_t::EXECNZ:
          branch_taken = (status_reg & SQ_WAVE_STATUS_EXECZ_MASK) == 0;
          break;
        case cbranch_cond_t::VCCZ:
          branch_taken = (status_reg & SQ_WAVE_STATUS_VCCZ_MASK) != 0;
          break;
        case cbranch_cond_t::VCCNZ:
          branch_taken = (status_reg & SQ_WAVE_STATUS_VCCZ_MASK) == 0;
          break;
        case cbranch_cond_t::CDBGSYS:
          branch_taken = (status_reg & SQ_WAVE_STATUS_COND_DBG_SYS_MASK) != 0;
          break;
        case cbranch_cond_t::CDBGUSER:
          branch_taken = (status_reg & SQ_WAVE_STATUS_COND_DBG_USER_MASK) != 0;
          break;
        case cbranch_cond_t::CDBGSYS_OR_USER:
          {
            uint32_t mask = SQ_WAVE_STATUS_COND_DBG_SYS_MASK
                            | SQ_WAVE_STATUS_COND_DBG_USER_MASK;
            branch_taken = (status_reg & mask) != 0;
            break;
          }
        case cbranch_cond_t::CDBGSYS_AND_USER:
          {
            uint32_t mask = SQ_WAVE_STATUS_COND_DBG_SYS_MASK
                            | SQ_WAVE_STATUS_COND_DBG_USER_MASK;
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

amd_dbgapi_status_t
amdgcn_architecture_t::simulate_instruction (
    wave_t &wave, amd_dbgapi_global_address_t pc,
    const std::vector<uint8_t> &instruction) const
{
  amd_dbgapi_global_address_t new_pc;
  amd_dbgapi_status_t status;

  dbgapi_assert (
      wave.state () == AMD_DBGAPI_WAVE_STATE_STOP
      && "We can only simulate instructions when the wave is not running.");

  if (is_endpgm (instruction))
    {
      /* Mark the wave as invalid and un-halt it at an s_endpgm instruction.
         This allows the hardware to terminate the wave, while ensuring that
         the wave is never reported to the client as existing.  */

      /* Make the PC point to an immutable s_endpgm instruction.  */
      amd_dbgapi_global_address_t pc = wave.queue ().endpgm_buffer_address ();
      amd_dbgapi_status_t status
          = wave.write_register (amdgpu_regnum_t::PC, &pc);
      if (status != AMD_DBGAPI_STATUS_SUCCESS)
        return status;

      /* Set the wave_id to the ignored_wave sentinel.  */
      amd_dbgapi_wave_id_t wave_id = wave_t::ignored_wave;
      status = wave.write_register (amdgpu_regnum_t::WAVE_ID, &wave_id);
      if (status != AMD_DBGAPI_STATUS_SUCCESS)
        return status;

      return wave.set_state (AMD_DBGAPI_WAVE_STATE_RUN);
    }
  else if (is_branch (instruction) || is_cbranch (instruction))
    {
      new_pc = branch_target (wave, pc, instruction);
    }
  else if (is_call (instruction))
    {
      amd_dbgapi_global_address_t return_pc = pc + instruction.size ();

      /* Save the return address.  */
      amdgpu_regnum_t sdst_regnum
          = scalar_operand_to_regnum (wave, encoding_sdst (instruction));

      uint32_t sdst_lo = static_cast<uint32_t> (return_pc);
      uint32_t sdst_hi = static_cast<uint32_t> (return_pc >> 32);

      status = wave.write_register (sdst_regnum, &sdst_lo);
      if (status != AMD_DBGAPI_STATUS_SUCCESS)
        return status;

      status = wave.write_register (sdst_regnum + 1, &sdst_hi);
      if (status != AMD_DBGAPI_STATUS_SUCCESS)
        return status;

      new_pc = branch_target (wave, pc, instruction);
    }
  else if (is_getpc (instruction) || is_swappc (instruction)
           || is_setpc (instruction))
    {
      if (is_getpc (instruction) || is_swappc (instruction))
        {
          amdgpu_regnum_t sdst_regnum
              = scalar_operand_to_regnum (wave, encoding_sdst (instruction));

          new_pc = pc + instruction.size ();

          uint32_t sdst_lo = static_cast<uint32_t> (new_pc);
          uint32_t sdst_hi = static_cast<uint32_t> (new_pc >> 32);

          status = wave.write_register (sdst_regnum, &sdst_lo);
          if (status != AMD_DBGAPI_STATUS_SUCCESS)
            return status;

          status = wave.write_register (sdst_regnum + 1, &sdst_hi);
          if (status != AMD_DBGAPI_STATUS_SUCCESS)
            return status;
        }

      if (is_setpc (instruction) || is_swappc (instruction))
        {
          amdgpu_regnum_t ssrc_regnum
              = scalar_operand_to_regnum (wave, encoding_ssrc0 (instruction));

          uint32_t ssrc_lo, ssrc_hi;

          status = wave.read_register (ssrc_regnum, &ssrc_lo);
          if (status != AMD_DBGAPI_STATUS_SUCCESS)
            return status;

          status = wave.read_register (ssrc_regnum + 1, &ssrc_hi);
          if (status != AMD_DBGAPI_STATUS_SUCCESS)
            return status;

          new_pc = amd_dbgapi_global_address_t{ ssrc_lo }
                   | amd_dbgapi_global_address_t{ ssrc_hi } << 32;
        }
    }
  else
    {
      /* We don't know how to simulate this instruction.  */
      return AMD_DBGAPI_STATUS_ERROR_NOT_SUPPORTED;
    }

  status = wave.write_register (amdgpu_regnum_t::PC, &new_pc);
  if (status != AMD_DBGAPI_STATUS_SUCCESS)
    return status;

  if (!can_halt_at (wave.instruction_at_pc ()))
    wave.park ();

  return AMD_DBGAPI_STATUS_SUCCESS;
}

size_t
amdgcn_architecture_t::displaced_stepping_buffer_size () const
{
  /* 1 displaced instruction + 1 terminating breakpoint instruction.  The
     terminating instruction is used to catch runaway waves.  */
  return largest_instruction_size () + breakpoint_instruction ().size ();
}

bool
amdgcn_architecture_t::displaced_stepping_copy (
    displaced_stepping_t &displaced_stepping, bool *simulate) const
{
  process_t &process = displaced_stepping.process ();
  const std::vector<uint8_t> &original_instruction
      = displaced_stepping.original_instruction ();

  if (is_cbranch_i_fork (original_instruction))
    {
      /* Displace stepping over a cbranch_i_fork is not supported.  */
      return false;
    }

  /* Copy a single instruction into the displaced stepping buffer.  */

  amd_dbgapi_global_address_t buffer = displaced_stepping.to ();

  if (is_branch (original_instruction) || is_cbranch (original_instruction)
      || is_call (original_instruction) || is_getpc (original_instruction)
      || is_setpc (original_instruction) || is_swappc (original_instruction)
      || is_endpgm (original_instruction))
    {
      /* We simulate PC relative branch instructions to avoid reading
         uninitialized memory at the branch target.  */

      /* We simulate ENDPGM do make sure we are reporting to the client that
         the displaced instruction has completed the single step operation.  */

      /* FIXME: If we were simulating the original instruction at 'start' time,
         we would not need to copy a nop into the displaced instruction buffer,
         we would simply enqueue a single-step event and skip the resume.   */

      if (process.write_global_memory (buffer, nop_instruction ().data (),
                                       nop_instruction ().size ())
          != AMD_DBGAPI_STATUS_SUCCESS)
        return false;

      buffer += nop_instruction ().size ();
      *simulate = true;
    }
  else
    {
      if (process.write_global_memory (buffer, original_instruction.data (),
                                       original_instruction.size ())
          != AMD_DBGAPI_STATUS_SUCCESS)
        return false;

      buffer += original_instruction.size ();
      *simulate = false;
    }

  /* Insert a terminating instruction (breakpoint) in case the wave is
     resumed before calling displaced_stepping_fixup.  */
  if (process.write_global_memory (buffer, breakpoint_instruction ().data (),
                                   breakpoint_instruction ().size ())
      != AMD_DBGAPI_STATUS_SUCCESS)
    return false;

  return true;
}

bool
amdgcn_architecture_t::displaced_stepping_simulate (
    wave_t &wave, displaced_stepping_t &displaced_stepping) const
{
  return simulate_instruction (wave, displaced_stepping.from (),
                               displaced_stepping.original_instruction ())
         == AMD_DBGAPI_STATUS_SUCCESS;
}

bool
amdgcn_architecture_t::displaced_stepping_fixup (
    wave_t &wave, displaced_stepping_t &displaced_stepping) const
{
  __maybe_unused__ const std::vector<uint8_t> &original_instruction
      = displaced_stepping.original_instruction ();

  dbgapi_assert (
      !is_call (original_instruction) && !is_getpc (original_instruction)
      && !is_setpc (original_instruction) && !is_swappc (original_instruction)
      && !is_endpgm (original_instruction)
      && "Should be simulated: these instructions require special handling");

  amd_dbgapi_global_address_t restored_pc
      = wave.pc () + displaced_stepping.from () - displaced_stepping.to ();

  if (wave.write_register (amdgpu_regnum_t::PC, &restored_pc)
      != AMD_DBGAPI_STATUS_SUCCESS)
    return false;

  if (!can_halt_at (wave.instruction_at_pc ()))
    wave.park ();

  return true;
}

amd_dbgapi_status_t
amdgcn_architecture_t::get_wave_coords (wave_t &wave, uint32_t (&group_ids)[3],
                                        uint32_t *wave_in_group) const
{
  amd_dbgapi_status_t status;
  dbgapi_assert (wave_in_group && "Invalid parameter");

  /* Read group_ids[0:3] from ttmp[8:10].  */
  status = wave.process ().read_global_memory (
      wave.context_save_address ()
          + wave.register_offset_and_size (amdgpu_regnum_t::TTMP8).first,
      &group_ids[0], sizeof (group_ids));
  if (status != AMD_DBGAPI_STATUS_SUCCESS)
    {
      warning ("Could not read ttmp[8:10]");
      return status;
    }

  uint32_t ttmp11;
  status = wave.process ().read_global_memory (
      wave.context_save_address ()
          + wave.register_offset_and_size (amdgpu_regnum_t::TTMP11).first,
      &ttmp11, sizeof (ttmp11));
  if (status != AMD_DBGAPI_STATUS_SUCCESS)
    {
      warning ("Could not read ttmp11");
      return status;
    }
  *wave_in_group = ttmp11 & TTMP11_WAVE_IN_GROUP_MASK;

  return AMD_DBGAPI_STATUS_SUCCESS;
}

amd_dbgapi_status_t
amdgcn_architecture_t::get_wave_state (
    wave_t &wave, amd_dbgapi_wave_state_t *state,
    amd_dbgapi_wave_stop_reason_t *stop_reason) const
{
  amd_dbgapi_status_t status;
  dbgapi_assert (state && stop_reason && "Invalid parameter");

  uint32_t status_reg;
  status = wave.read_register (amdgpu_regnum_t::STATUS, &status_reg);
  if (status != AMD_DBGAPI_STATUS_SUCCESS)
    return status;

  uint32_t mode_reg;
  status = wave.read_register (amdgpu_regnum_t::MODE, &mode_reg);
  if (status != AMD_DBGAPI_STATUS_SUCCESS)
    return status;

  amd_dbgapi_wave_state_t saved_state = wave.state ();

  *state = (status_reg & SQ_WAVE_STATUS_HALT_MASK)
               ? AMD_DBGAPI_WAVE_STATE_STOP
               : ((mode_reg & SQ_WAVE_MODE_DEBUG_EN_MASK)
                      ? AMD_DBGAPI_WAVE_STATE_SINGLE_STEP
                      : AMD_DBGAPI_WAVE_STATE_RUN);

  if (*state != AMD_DBGAPI_WAVE_STATE_STOP)
    {
      /* The wave is running, there is no stop reason.  */
      *stop_reason = AMD_DBGAPI_WAVE_STOP_REASON_NONE;

#if !defined(NDEBUG)
      uint32_t ttmp11;
      status = wave.read_register (amdgpu_regnum_t::TTMP11, &ttmp11);
      if (status != AMD_DBGAPI_STATUS_SUCCESS)
        return status;

      dbgapi_assert (!(ttmp11 & TTMP11_TRAP_HANDLER_EVENTS_MASK)
                     && "Waves should not have trap handler events while "
                        "running. These are reset when unhalting the wave.");
#endif /* !defined (NDEBUG) */
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

      uint32_t reason_mask = (saved_state == AMD_DBGAPI_WAVE_STATE_SINGLE_STEP)
                                 ? AMD_DBGAPI_WAVE_STOP_REASON_SINGLE_STEP
                                 : AMD_DBGAPI_WAVE_STOP_REASON_NONE;

      uint32_t trapsts;
      status = wave.read_register (amdgpu_regnum_t::TRAPSTS, &trapsts);
      if (status != AMD_DBGAPI_STATUS_SUCCESS)
        return status;

      uint32_t ttmp11;
      status = wave.read_register (amdgpu_regnum_t::TTMP11, &ttmp11);
      if (status != AMD_DBGAPI_STATUS_SUCCESS)
        return status;

      bool trap_raised = !!(ttmp11 & TTMP11_TRAP_HANDLER_TRAP_RAISED_MASK);
      /* bool excp_raised
             = !!(ttmp11 & TTMP11_TRAP_HANDLER_EXCP_RAISED_MASK);
       */

      amd_dbgapi_global_address_t pc = wave.pc ();

      if (trapsts
          & (SQ_WAVE_TRAPSTS_EXCP_MASK | SQ_WAVE_TRAPSTS_ILLEGAL_INST_MASK))
        {
          /* FIXME: Enable this when the trap handler is modified to send
             debugger notifications for exceptions.
             if (!excp_raised)
               error ("The trap handler should have set the excp_raised bit.");
           */

          /* The first-level trap handler subtracts 8 from the PC, so
             we add it back here.  */

          pc += 8;
          status = wave.write_register (amdgpu_regnum_t::PC, &pc);
          if (status != AMD_DBGAPI_STATUS_SUCCESS)
            return status;

          uint32_t excp = trapsts & SQ_WAVE_TRAPSTS_EXCP_MASK;
          if (excp & (1u << SQ_EX_MODE_EXCP_INVALID))
            reason_mask |= AMD_DBGAPI_WAVE_STOP_REASON_FP_INVALID_OPERATION;
          if (excp & (1u << SQ_EX_MODE_EXCP_INPUT_DENORM))
            reason_mask |= AMD_DBGAPI_WAVE_STOP_REASON_FP_INPUT_DENORMAL;
          if (excp & (1u << SQ_EX_MODE_EXCP_DIV0))
            reason_mask |= AMD_DBGAPI_WAVE_STOP_REASON_FP_DIVIDE_BY_0;
          if (excp & (1u << SQ_EX_MODE_EXCP_OVERFLOW))
            reason_mask |= AMD_DBGAPI_WAVE_STOP_REASON_FP_OVERFLOW;
          if (excp & (1u << SQ_EX_MODE_EXCP_UNDERFLOW))
            reason_mask |= AMD_DBGAPI_WAVE_STOP_REASON_FP_UNDERFLOW;
          if (excp & (1u << SQ_EX_MODE_EXCP_INEXACT))
            reason_mask |= AMD_DBGAPI_WAVE_STOP_REASON_FP_INEXACT;
          if (excp & (1u << SQ_EX_MODE_EXCP_INT_DIV0))
            reason_mask |= AMD_DBGAPI_WAVE_STOP_REASON_INT_DIVIDE_BY_0;
          if (excp & (1u << SQ_EX_MODE_EXCP_MEM_VIOL))
            reason_mask |= AMD_DBGAPI_WAVE_STOP_REASON_MEMORY_VIOLATION;

          if (trapsts & SQ_WAVE_TRAPSTS_ILLEGAL_INST_MASK)
            reason_mask |= AMD_DBGAPI_WAVE_STOP_REASON_ILLEGAL_INSTRUCTION;
        }
      else
        {
          /* FIXME: If we had a way to tell which trap instruction caused the
             trap_raised, we would not have to read the instruction on the
             common path.  */
          auto instruction = wave.instruction_at_pc ();

          /* Check for spurious single-step events. A context save/restore
             before executing the single-stepped instruction could have caused
             the event to be reported with the wave halted at the instruction
             instead of after.  In such cases, un-halt the wave and let it
             continue, so that the instruction is executed.
           */
          bool ignore_single_step_event
              = saved_state == AMD_DBGAPI_WAVE_STATE_SINGLE_STEP
                && wave.saved_pc () == pc && !trap_raised;

          if (ignore_single_step_event)
            {
              /* Trim to size of instruction, simulate_instruction needs the
                 exact instruction bytes.  */
              size_t size = instruction_size (instruction);
              dbgapi_assert (size != 0 && "Invalid instruction");
              instruction.resize (size);

              /* Branch instructions should be simulated, and the event
                 reported, as we cannot tell if a branch to self
                 instruction has executed.  */
              status = simulate_instruction (wave, pc, instruction);
              if (status == AMD_DBGAPI_STATUS_SUCCESS)
                {
                  /* We successfully simulated the instruction, report the
                     single-step event.  */
                  ignore_single_step_event = false;
                }
              else if (status != AMD_DBGAPI_STATUS_ERROR_NOT_SUPPORTED)
                {
                  /* The instruction should have been simulated but an error
                     has occurred.  */
                  error ("simulate_instruction failed (rc=%d)", status);
                }
            }

          if (ignore_single_step_event)
            {
              /* Place the wave back into single-stepping state.  */
              *state = AMD_DBGAPI_WAVE_STATE_SINGLE_STEP;
              status_reg &= ~SQ_WAVE_STATUS_HALT_MASK;

              status
                  = wave.write_register (amdgpu_regnum_t::STATUS, &status_reg);
              if (status != AMD_DBGAPI_STATUS_SUCCESS)
                return status;

              dbgapi_log (AMD_DBGAPI_LOG_LEVEL_INFO,
                          "%s (pc=%#lx) ignore spurious single-step",
                          to_string (wave.id ()).c_str (), pc);

              /* Don't report the event.  */
              reason_mask = AMD_DBGAPI_WAVE_STOP_REASON_NONE;
            }
          else if (trap_raised)
            {
              uint16_t trap_id;

              if (!is_trap (instruction, &trap_id))
                error ("trap_raised should only be set for trap instructions");

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
        }

      *stop_reason = static_cast<amd_dbgapi_wave_stop_reason_t> (reason_mask);
    }

  return AMD_DBGAPI_STATUS_SUCCESS;
}

amd_dbgapi_status_t
amdgcn_architecture_t::set_wave_state (wave_t &wave,
                                       amd_dbgapi_wave_state_t state) const
{
  uint32_t status_reg, mode_reg;
  amd_dbgapi_status_t status;

  status = wave.read_register (amdgpu_regnum_t::STATUS, &status_reg);
  if (status != AMD_DBGAPI_STATUS_SUCCESS)
    return status;

  status = wave.read_register (amdgpu_regnum_t::MODE, &mode_reg);
  if (status != AMD_DBGAPI_STATUS_SUCCESS)
    return status;

  switch (state)
    {
    case AMD_DBGAPI_WAVE_STATE_STOP:
      mode_reg &= ~SQ_WAVE_MODE_DEBUG_EN_MASK;
      status_reg |= SQ_WAVE_STATUS_HALT_MASK;
      break;

    case AMD_DBGAPI_WAVE_STATE_RUN:
      mode_reg &= ~SQ_WAVE_MODE_DEBUG_EN_MASK;
      status_reg &= ~SQ_WAVE_STATUS_HALT_MASK;
      break;

    case AMD_DBGAPI_WAVE_STATE_SINGLE_STEP:
      mode_reg |= SQ_WAVE_MODE_DEBUG_EN_MASK;
      status_reg &= ~SQ_WAVE_STATUS_HALT_MASK;
      break;

    default:
      return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;
    }

  status = wave.write_register (amdgpu_regnum_t::STATUS, &status_reg);
  if (status != AMD_DBGAPI_STATUS_SUCCESS)
    return status;

  status = wave.write_register (amdgpu_regnum_t::MODE, &mode_reg);
  if (status != AMD_DBGAPI_STATUS_SUCCESS)
    return status;

  /* Clear the trap handler events if resuming the wave.  */
  if (state != AMD_DBGAPI_WAVE_STATE_STOP)
    {
      uint32_t ttmp11_reg;

      status = wave.read_register (amdgpu_regnum_t::TTMP11, &ttmp11_reg);
      if (status != AMD_DBGAPI_STATUS_SUCCESS)
        return status;

      ttmp11_reg &= ~TTMP11_TRAP_HANDLER_EVENTS_MASK;

      status = wave.write_register (amdgpu_regnum_t::TTMP11, &ttmp11_reg);
      if (status != AMD_DBGAPI_STATUS_SUCCESS)
        return status;
    }

  return AMD_DBGAPI_STATUS_SUCCESS;
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
amdgcn_architecture_t::is_endpgm (const std::vector<uint8_t> &bytes) const
{
  if (bytes.size () < sizeof (uint32_t))
    return false;

  uint32_t encoding = *reinterpret_cast<const uint32_t *> (bytes.data ());

  /* s_endpgm: SOPP Opcode 1 [10111111 10000001 SIMM16] */
  return (encoding & 0xFFFF0000) == 0xBF810000;
}

bool
amdgcn_architecture_t::is_trap (const std::vector<uint8_t> &bytes,
                                uint16_t *trap_id) const
{
  if (bytes.size () < sizeof (uint32_t))
    return false;

  uint32_t encoding = *reinterpret_cast<const uint32_t *> (bytes.data ());

  /* s_trap: SOPP Opcode 18 [10111111 10010010 SIMM16] */
  if ((encoding & 0xFFFF0000) == 0xBF920000)
    {
      if (trap_id)
        *trap_id = encoding & 0xFFFF;
      return true;
    }
  return false;
}

bool
amdgcn_architecture_t::is_call (const std::vector<uint8_t> &bytes) const
{
  if (bytes.size () < sizeof (uint32_t))
    return false;

  uint32_t encoding = *reinterpret_cast<const uint32_t *> (bytes.data ());

  /* s_call: SOPK Opcode 21 [10111010 1 SDST7 SIMM16] */
  return (encoding & 0xFF800000) == 0xBA800000;
}

bool
amdgcn_architecture_t::is_getpc (const std::vector<uint8_t> &bytes) const
{
  if (bytes.size () < sizeof (uint32_t))
    return false;

  uint32_t encoding = *reinterpret_cast<const uint32_t *> (bytes.data ());

  /* s_getpc: SOP1 Opcode 28 [10111110 1 SDST7 00011100 SSRC08] */
  return (encoding & 0xFF80FF00) == 0xBE801C00;
}

bool
amdgcn_architecture_t::is_setpc (const std::vector<uint8_t> &bytes) const
{
  if (bytes.size () < sizeof (uint32_t))
    return false;

  uint32_t encoding = *reinterpret_cast<const uint32_t *> (bytes.data ());

  /* s_setpc: SOP1 Opcode 29 [10111110 1 SDST7 00011101 SSRC08] */
  return (encoding & 0xFF80FF00) == 0xBE801D00;
}

bool
amdgcn_architecture_t::is_swappc (const std::vector<uint8_t> &bytes) const
{
  if (bytes.size () < sizeof (uint32_t))
    return false;

  uint32_t encoding = *reinterpret_cast<const uint32_t *> (bytes.data ());

  /* s_swappc: SOP1 Opcode 30 [10111110 1 SDST7 00011110 SSRC08] */
  return (encoding & 0xFF80FF00) == 0xBE801E00;
}

bool
amdgcn_architecture_t::is_branch (const std::vector<uint8_t> &bytes) const
{
  if (bytes.size () < sizeof (uint32_t))
    return false;

  uint32_t encoding = *reinterpret_cast<const uint32_t *> (bytes.data ());

  /* s_branch: SOPP Opcode 2 [10111111 10000010 SIMM16] */
  return (encoding & 0xFFFF0000) == 0xBF820000;
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
     s_cbranch_cdbgsys_and_user: SOPP Opcode 26 [10111111 10011010 SIMM16]  */
  if ((encoding & 0xFF800000) != 0xBF800000)
    return false;

  return cbranch_opcodes_map.find (encoding_op7 (bytes))
         != cbranch_opcodes_map.end ();
}

bool
amdgcn_architecture_t::is_cbranch_i_fork (
    const std::vector<uint8_t> &bytes) const
{
  if (bytes.size () < sizeof (uint32_t))
    return false;

  uint32_t encoding = *reinterpret_cast<const uint32_t *> (bytes.data ());

  /* s_cbranch_i_fork: SOPK Opcode 16 [10111000 0 SDST7 SIMM16] */
  return (encoding & 0xFF800000) == 0xB8000000;
}

/* Base class for all GFX9 architectures.  */

class gfx9_base_t : public amdgcn_architecture_t
{
protected:
  gfx9_base_t (uint8_t gfxip_minor, uint8_t gfxip_stepping)
      : amdgcn_architecture_t (9, gfxip_minor, gfxip_stepping)
  {
  }

public:
  bool has_wave32_vgprs () const final { return false; }
  bool has_wave64_vgprs () const final { return true; }
  bool has_acc_vgprs () const override { return false; }
  bool can_halt_at_endpgm () const override { return false; }
  size_t largest_instruction_size () const override { return 8; }

  compute_relaunch_abi_t compute_relaunch_abi () const override
  {
    return compute_relaunch_abi_t::GFX900;
  }
};

/* Vega10 Architecture.  */

class gfx900_t final : public gfx9_base_t
{
public:
  gfx900_t () : gfx9_base_t (0, 0) {}

  elf_amdgpu_machine_t elf_amdgpu_machine () const override
  {
    return EF_AMDGPU_MACH_AMDGCN_GFX900;
  }
};

/* Raven Architecture.  */

class gfx902_t final : public gfx9_base_t
{
public:
  gfx902_t () : gfx9_base_t (0, 2) {}

  elf_amdgpu_machine_t elf_amdgpu_machine () const override
  {
    return EF_AMDGPU_MACH_AMDGCN_GFX902;
  }
};

/* Vega12 Architecture.  */

class gfx904_t final : public gfx9_base_t
{
public:
  gfx904_t () : gfx9_base_t (0, 4) {}

  elf_amdgpu_machine_t elf_amdgpu_machine () const override
  {
    return EF_AMDGPU_MACH_AMDGCN_GFX904;
  }
};

/* Vega20 Architecture.  */

class gfx906_t final : public gfx9_base_t
{
public:
  gfx906_t () : gfx9_base_t (0, 6) {}

  elf_amdgpu_machine_t elf_amdgpu_machine () const override
  {
    return EF_AMDGPU_MACH_AMDGCN_GFX906;
  }
};

/* Arcturus Architecture.  */

class gfx908_t final : public gfx9_base_t
{
public:
  gfx908_t () : gfx9_base_t (0, 8) {}

  bool has_acc_vgprs () const override { return true; }

  compute_relaunch_abi_t compute_relaunch_abi () const override
  {
    return compute_relaunch_abi_t::GFX908;
  }

  elf_amdgpu_machine_t elf_amdgpu_machine () const override
  {
    return EF_AMDGPU_MACH_AMDGCN_GFX908;
  }
};

class gfx10_base_t : public amdgcn_architecture_t
{
protected:
  gfx10_base_t (uint8_t gfxip_minor, uint8_t gfxip_stepping)
      : amdgcn_architecture_t (10, gfxip_minor, gfxip_stepping)
  {
  }

public:
  bool has_wave32_vgprs () const final { return true; }
  bool has_wave64_vgprs () const final { return true; }
  bool has_acc_vgprs () const override { return false; }
  bool can_halt_at_endpgm () const override { return false; }
  size_t largest_instruction_size () const override { return 20; }

  compute_relaunch_abi_t compute_relaunch_abi () const override
  {
    return compute_relaunch_abi_t::GFX1000;
  }
};

class gfx1010_t final : public gfx10_base_t
{
public:
  gfx1010_t () : gfx10_base_t (1, 0) {}

  elf_amdgpu_machine_t elf_amdgpu_machine () const override
  {
    return EF_AMDGPU_MACH_AMDGCN_GFX1010;
  }
};

class gfx1011_t final : public gfx10_base_t
{
public:
  gfx1011_t () : gfx10_base_t (1, 1) {}

  elf_amdgpu_machine_t elf_amdgpu_machine () const override
  {
    return EF_AMDGPU_MACH_AMDGCN_GFX1011;
  }
};

class gfx1012_t final : public gfx10_base_t
{
public:
  gfx1012_t () : gfx10_base_t (1, 2) {}

  elf_amdgpu_machine_t elf_amdgpu_machine () const override
  {
    return EF_AMDGPU_MACH_AMDGCN_GFX1012;
  }
};

architecture_t::architecture_t (int gfxip_major, int gfxip_minor,
                                int gfxip_stepping)
    : m_architecture_id (
        amd_dbgapi_architecture_id_t{ s_next_architecture_id++ }),
      m_disassembly_info (new amd_comgr_disassembly_info_t{ 0 }),
      m_gfxip_major (gfxip_major), m_gfxip_minor (gfxip_minor),
      m_gfxip_stepping (gfxip_stepping),
      m_name (string_printf (
          "amdgcn-amd-amdhsa--gfx%d%d%c", gfxip_major, gfxip_minor,
          gfxip_stepping < 10 ? '0' + gfxip_stepping : 'a' + gfxip_stepping))
{
}

architecture_t::~architecture_t ()
{
  if (*m_disassembly_info != amd_comgr_disassembly_info_t{ 0 })
    amd_comgr_destroy_disassembly_info (*m_disassembly_info);
}

const architecture_t *
architecture_t::find (amd_dbgapi_architecture_id_t architecture_id, int ignore)
{
  auto it = s_architecture_map.find (architecture_id);
  return it != s_architecture_map.end () ? it->second.get () : nullptr;
}

const architecture_t *
architecture_t::find (int gfxip_major, int gfxip_minor, int gfxip_stepping)
{
  auto it = std::find_if (
      s_architecture_map.begin (), s_architecture_map.end (),
      [&] (const decltype (s_architecture_map)::value_type &value) {
        return value.second->gfxip_major () == gfxip_major
               && value.second->gfxip_minor () == gfxip_minor
               && value.second->gfxip_stepping () == gfxip_stepping;
      });

  return it != s_architecture_map.end () ? it->second.get () : nullptr;
}

const architecture_t *
architecture_t::find (elf_amdgpu_machine_t elf_amdgpu_machine)
{
  auto it = std::find_if (
      s_architecture_map.begin (), s_architecture_map.end (),
      [&] (const decltype (s_architecture_map)::value_type &value) {
        return value.second->elf_amdgpu_machine () == elf_amdgpu_machine;
      });

  return it != s_architecture_map.end () ? it->second.get () : nullptr;
}

bool
architecture_t::can_halt_at (const std::vector<uint8_t> &instruction) const
{
  /* A wave cannot halt at an s_endpgm instruction. An s_trap may be used as a
     breakpoint instruction, and since it could be removed and the original
     instruction restored, which could reveal an s_endpgm, we also
     cannot halt at an s_strap. */
  return can_halt_at_endpgm ()
         || (!is_endpgm (instruction) && !is_trap (instruction));
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

std::string
architecture_t::register_name (amdgpu_regnum_t regnum) const
{
  if (regnum >= amdgpu_regnum_t::FIRST_SGPR
      && regnum <= amdgpu_regnum_t::LAST_SGPR)
    {
      return string_printf ("s%ld", regnum - amdgpu_regnum_t::FIRST_SGPR);
    }
  if (regnum >= amdgpu_regnum_t::FIRST_VGPR_32
      && regnum <= amdgpu_regnum_t::LAST_VGPR_32)
    {
      return has_wave32_vgprs () ? string_printf (
                 "v%ld", regnum - amdgpu_regnum_t::FIRST_VGPR_32)
                                 : "";
    }
  if (regnum >= amdgpu_regnum_t::FIRST_VGPR_64
      && regnum <= amdgpu_regnum_t::LAST_VGPR_64)
    {
      return has_wave64_vgprs () ? string_printf (
                 "v%ld", regnum - amdgpu_regnum_t::FIRST_VGPR_64)
                                 : "";
    }
  if (regnum >= amdgpu_regnum_t::FIRST_ACCVGPR_64
      && regnum <= amdgpu_regnum_t::LAST_ACCVGPR_64)
    {
      return has_wave64_vgprs () && has_acc_vgprs () ? string_printf (
                 "acc%ld", regnum - amdgpu_regnum_t::FIRST_ACCVGPR_64)
                                                     : "";
    }
  if (regnum >= amdgpu_regnum_t::FIRST_TTMP
      && regnum <= amdgpu_regnum_t::LAST_TTMP)
    {
      switch (regnum)
        {
        case amdgpu_regnum_t::TTMP4:
        case amdgpu_regnum_t::TTMP5:
        case amdgpu_regnum_t::TTMP6:
        case amdgpu_regnum_t::TTMP7:
        case amdgpu_regnum_t::TTMP8:
        case amdgpu_regnum_t::TTMP9:
        case amdgpu_regnum_t::TTMP10:
        case amdgpu_regnum_t::TTMP11:
        case amdgpu_regnum_t::TTMP13:
          return string_printf ("ttmp%ld",
                                regnum - amdgpu_regnum_t::FIRST_TTMP);
        default:
          return "";
        }
    }
  if (regnum >= amdgpu_regnum_t::FIRST_HWREG
      && regnum <= amdgpu_regnum_t::LAST_HWREG)
    {
      switch (regnum)
        {
        case amdgpu_regnum_t::M0:
          return "m0";
        case amdgpu_regnum_t::STATUS:
          return "status";
        case amdgpu_regnum_t::TRAPSTS:
          return "trapsts";
        case amdgpu_regnum_t::MODE:
          return "mode";
        default:
          return "";
        }
    }
  if (regnum >= amdgpu_regnum_t::FIRST_PSEUDO
      && regnum <= amdgpu_regnum_t::LAST_PSEUDO)
    {
      switch (regnum)
        {
        case amdgpu_regnum_t::PC:
          return "pc";
        case amdgpu_regnum_t::EXEC_32:
          return has_wave32_vgprs () ? "exec" : "";
        case amdgpu_regnum_t::EXEC_64:
          return has_wave64_vgprs () ? "exec" : "";
        case amdgpu_regnum_t::VCC_32:
          return has_wave32_vgprs () ? "vcc" : "";
        case amdgpu_regnum_t::VCC_64:
          return has_wave64_vgprs () ? "vcc" : "";
        default:
          return "";
        }
    }

  return "";
}

std::string
architecture_t::register_type (amdgpu_regnum_t regnum) const
{
  /* Vector registers (arch and acc).  */
  if (regnum >= amdgpu_regnum_t::FIRST_VGPR_32
      && regnum <= amdgpu_regnum_t::LAST_VGPR_32)
    {
      return has_wave32_vgprs () ? "int32_t[32]" : "";
    }
  if (regnum >= amdgpu_regnum_t::FIRST_VGPR_64
      && regnum <= amdgpu_regnum_t::LAST_VGPR_64)
    {
      return has_wave64_vgprs () ? "int32_t[64]" : "";
    }
  if (regnum >= amdgpu_regnum_t::FIRST_ACCVGPR_64
      && regnum <= amdgpu_regnum_t::LAST_ACCVGPR_64)
    {
      return has_wave64_vgprs () && has_acc_vgprs () ? "int32_t[64]" : "";
    }
  /* Scalar registers.  */
  if (regnum >= amdgpu_regnum_t::FIRST_SGPR
      && regnum <= amdgpu_regnum_t::LAST_SGPR)
    {
      return "int32_t";
    }
  /* Everything else (hwregs, ttmps).  */
  if ((regnum >= amdgpu_regnum_t::FIRST_HWREG
       && regnum <= amdgpu_regnum_t::LAST_HWREG)
      || (regnum >= amdgpu_regnum_t::FIRST_TTMP
          && regnum <= amdgpu_regnum_t::LAST_TTMP))
    {
      return "uint32_t";
    }
  /* Pseudo registers.  */
  if (regnum == amdgpu_regnum_t::PC)
    {
      return "void (*)()";
    }
  if (regnum == amdgpu_regnum_t::EXEC_32 || regnum == amdgpu_regnum_t::VCC_32)
    {
      return has_wave32_vgprs () ? "uint32_t" : "";
    }
  if (regnum == amdgpu_regnum_t::EXEC_64 || regnum == amdgpu_regnum_t::VCC_64)
    {
      return has_wave64_vgprs () ? "uint64_t" : "";
    }

  return "";
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
              name ().c_str (), read_memory_callback,
              print_instruction_callback, print_address_annotation_callback,
              m_disassembly_info.get ()))
        error ("amd_comgr_create_disassembly_info failed");
    }

  return *m_disassembly_info;
}

size_t
architecture_t::instruction_size (const std::vector<uint8_t> &bytes) const
{
  size_t size;

  struct detail::disassembly_user_data_t user_data = { .memory = bytes.data (),
                                                       .offset = 0,
                                                       .size = bytes.size (),
                                                       .instruction = nullptr,
                                                       .operands = nullptr };

  /* Disassemble one instruction.  */
  if (amd_comgr_disassemble_instruction (disassembly_info (), 0, &user_data,
                                         &size))
    return 0;

  return size;
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
      return utils::get_info (value_size, value, m_name);

    case AMD_DBGAPI_ARCHITECTURE_INFO_ELF_AMDGPU_MACHINE:
      return utils::get_info (value_size, value, elf_amdgpu_machine ());

    case AMD_DBGAPI_ARCHITECTURE_INFO_LARGEST_INSTRUCTION_SIZE:
      return utils::get_info (value_size, value, largest_instruction_size ());

    case AMD_DBGAPI_ARCHITECTURE_INFO_MINIMUM_INSTRUCTION_ALIGNMENT:
      return utils::get_info (value_size, value,
                              minimum_instruction_alignment ());

    case AMD_DBGAPI_ARCHITECTURE_INFO_PC_REGISTER:
      return utils::get_info (value_size, value, amdgpu_regnum_t::PC);

    case AMD_DBGAPI_ARCHITECTURE_INFO_BREAKPOINT_INSTRUCTION_SIZE:
      return utils::get_info (value_size, value,
                              breakpoint_instruction ().size ());

    case AMD_DBGAPI_ARCHITECTURE_INFO_BREAKPOINT_INSTRUCTION:
      return utils::get_info (value_size, value, breakpoint_instruction ());

    case AMD_DBGAPI_ARCHITECTURE_INFO_BREAKPOINT_INSTRUCTION_PC_ADJUST:
      return utils::get_info (value_size, value,
                              breakpoint_instruction_pc_adjust ());

    case AMD_DBGAPI_ARCHITECTURE_INFO_DEFAULT_GLOBAL_ADDRESS_SPACE:
      return utils::get_info (value_size, value,
                              default_global_address_space ().id ());

    default:
      return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;
    }

  return AMD_DBGAPI_STATUS_SUCCESS;
}

namespace detail
{

template <class ElementType, std::size_t N> struct initializer_list_t
{
  /* Move the elements out of the temporary container.  */
  template <class T> operator T () &&
  {
    return { std::make_move_iterator (elements.begin ()),
             std::make_move_iterator (elements.end ()) };
  }
  /* Temporary container.  */
  std::array<ElementType, N> elements;
};

/* Create a container to hold the map values.  */
template <class... Keys, class... Values, std::size_t... I>
initializer_list_t<
    std::pair<amd_dbgapi_architecture_id_t, std::unique_ptr<architecture_t>>,
    sizeof...(I)>
make_architecture_initializer_list (std::tuple<Keys...> keys,
                                    std::tuple<Values &...> values,
                                    std::index_sequence<I...>)
{
  return { { { std::make_pair (std::get<I> (keys),
                               std::move (std::get<I> (values)))... } } };
}

} /* namespace detail */

/* Helper function to create an initializer list with movable elements.  */
template <class... Args>
auto
make_architecture_initializer_list (Args... args)
{
  return detail::make_architecture_initializer_list (
      std::make_tuple (args->id ()...), std::tie (args...),
      std::make_index_sequence<sizeof...(Args)>{});
}

std::unordered_map<amd_dbgapi_architecture_id_t,
                   std::unique_ptr<const architecture_t>,
                   hash<amd_dbgapi_architecture_id_t>>
    architecture_t::s_architecture_map = make_architecture_initializer_list (
        create_architecture<gfx900_t> (), create_architecture<gfx902_t> (),
        create_architecture<gfx904_t> (), create_architecture<gfx906_t> (),
        create_architecture<gfx908_t> (), create_architecture<gfx1010_t> (),
        create_architecture<gfx1011_t> (), create_architecture<gfx1012_t> ());

} /* namespace dbgapi */
} /* namespace amd */

using namespace amd::dbgapi;

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_get_architecture (uint32_t elf_amdgpu_machine,
                             amd_dbgapi_architecture_id_t *architecture_id)
{
  TRY;
  TRACE (elf_amdgpu_machine);

  if (!architecture_id)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;

  const architecture_t *architecture = architecture_t::find (
      static_cast<architecture_t::elf_amdgpu_machine_t> (elf_amdgpu_machine));

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

  if (!memory || !size || !instruction_text)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;

  const architecture_t *architecture = architecture_t::find (architecture_id);

  if (!architecture)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARCHITECTURE_ID;

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
      sstream << std::showbase << std::hex << std::setfill ('0')
              << std::setw (sizeof (addr) * 2) << addr;
      operand_str += sstream.str ();
    }
  instruction_str += operand_str;

  /* Return the instruction text in client allocated memory.  */
  void *mem;
  size_t mem_size = instruction_str.length () + 1;
  mem = allocate_memory (mem_size);
  if (!mem)
    return AMD_DBGAPI_STATUS_ERROR_CLIENT_CALLBACK;

  memcpy (mem, instruction_str.c_str (), mem_size);
  *instruction_text = static_cast<char *> (mem);

  return AMD_DBGAPI_STATUS_SUCCESS;
  CATCH;
}
