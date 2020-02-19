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

std::unordered_map<amd_dbgapi_architecture_id_t, const architecture_t *,
                   hash<amd_dbgapi_architecture_id_t>>
    architecture_t::architecture_map;

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

  virtual amd_dbgapi_global_address_t
  branch_target (wave_t &wave, amd_dbgapi_global_address_t pc,
                 const std::vector<uint8_t> &instruction) const;

  virtual amd_dbgapi_status_t
  simulate_instruction (wave_t &wave, amd_dbgapi_global_address_t pc,
                        const std::vector<uint8_t> &instruction) const;

  /* Check the instruction at the current pc. If it is an s_endpgm, we can't
     halt the wave, or we'll hang the gpu, so resume the wave and make it look
     like the wave has terminated.  */
  bool terminate_wave_if_at_endpgm (wave_t &wave) const;
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

  if (is_branch (instruction) || is_cbranch (instruction))
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

  if (!can_halt_at_endpgm ())
    terminate_wave_if_at_endpgm (wave);

  return AMD_DBGAPI_STATUS_SUCCESS;
}

bool
amdgcn_architecture_t::terminate_wave_if_at_endpgm (wave_t &wave) const
{
  size_t size = largest_instruction_size ();
  std::vector<uint8_t> instruction_bytes (size);

  if (wave.process ().read_global_memory_partial (
          wave.pc (), instruction_bytes.data (), &size)
      != AMD_DBGAPI_STATUS_SUCCESS)
    return false;

  /* Trim unread bytes.  */
  instruction_bytes.resize (size);

  if (!is_endpgm (instruction_bytes))
    return false;

  /* Write ignored_wave_id into ttmp[4:5].  */
  amd_dbgapi_wave_id_t wave_id = wave_t::ignored_wave_id;

  wave.process ().write_global_memory (
      wave.context_save_address ()
          + wave.register_offset_and_size (amdgpu_regnum_t::TTMP4).first,
      &wave_id, sizeof (wave_id));

  /* FIXME: We should report a command_terminated event if there was an
     outstanding stop or single-step request.  */

  wave.set_state (AMD_DBGAPI_WAVE_STATE_RUN);
  return true;
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
      || is_setpc (original_instruction) || is_swappc (original_instruction))
    {
      /* We simulate PC relative branch instructions to avoid reading
         uninitialized memory at the branch target.  */

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

  /* If the device cannot halt at an ENDPGM instruction (hardware bug), make
     sure there isn't one following the displaced instruction by inserting
     a NOP.  */
  if (!can_halt_at_endpgm ())
    {
      if (process.write_global_memory (buffer, nop_instruction ().data (),
                                       nop_instruction ().size ())
          != AMD_DBGAPI_STATUS_SUCCESS)
        return false;
    }

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
      && "Should be simulated: these instructions require special handling");

  amd_dbgapi_global_address_t new_pc
      = wave.pc () + displaced_stepping.from () - displaced_stepping.to ();

  if (wave.write_register (amdgpu_regnum_t::PC, &new_pc)
      != AMD_DBGAPI_STATUS_SUCCESS)
    return false;

  if (!can_halt_at_endpgm ())
    terminate_wave_if_at_endpgm (wave);

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

      uint32_t trapsts;
      status = wave.read_register (amdgpu_regnum_t::TRAPSTS, &trapsts);
      if (status != AMD_DBGAPI_STATUS_SUCCESS)
        return status;

      amd_dbgapi_global_address_t pc = wave.pc ();

      if (trapsts
          & (SQ_WAVE_TRAPSTS_EXCP_MASK | SQ_WAVE_TRAPSTS_ILLEGAL_INST_MASK))
        {
          /* The first-level trap handler subtracts 8 from the PC, so
             we add it back here.  */

          pc += 8;
          status = wave.write_register (amdgpu_regnum_t::PC, &pc);
          if (status != AMD_DBGAPI_STATUS_SUCCESS)
            return status;

          uint32_t reason_mask = 0;
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

          *stop_reason
              = static_cast<amd_dbgapi_wave_stop_reason_t> (reason_mask);
        }
      else
        {
          /* Check for spurious single-step events.

             A context save/restore before executing the single-stepped
             instruction could have caused the event to be reported with
             the wave halted at the instruction instead of after.
             In such cases, un-halt the wave and let it continue, so that
             the instruction is executed.
           */
          bool ignore_single_step_event
              = saved_state == AMD_DBGAPI_WAVE_STATE_SINGLE_STEP
                && wave.prev_pc () == pc;

          if (ignore_single_step_event)
            {
              size_t size = largest_instruction_size ();
              std::vector<uint8_t> instruction (size);

              /* Read up to largest_instruction_size bytes.  */
              status = wave.process ().read_global_memory_partial (
                  pc, instruction.data (), &size);
              if (status != AMD_DBGAPI_STATUS_SUCCESS)
                return status;

              /* Trim unread bytes.  */
              instruction.resize (size);

              /* Trim to size of instruction.  */
              size = instruction_size (instruction);
              dbgapi_assert (size != 0 && "Invalid instruction");
              instruction.resize (size);

              /* Branch instructions should be simulated, and the event
                 reported, as we cannot tell if a branch to self instruction
                has executed.  */
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
              *stop_reason = AMD_DBGAPI_WAVE_STOP_REASON_NONE;
            }
          else
            {
              *stop_reason = (saved_state == AMD_DBGAPI_WAVE_STATE_SINGLE_STEP)
                                 ? AMD_DBGAPI_WAVE_STOP_REASON_SINGLE_STEP
                                 : AMD_DBGAPI_WAVE_STOP_REASON_BREAKPOINT;
            }
        }
    }

  return AMD_DBGAPI_STATUS_SUCCESS;
}

amd_dbgapi_status_t
amdgcn_architecture_t::set_wave_state (wave_t &wave,
                                       amd_dbgapi_wave_state_t state) const
{
  amd_dbgapi_status_t status;

  uint32_t status_reg;
  status = wave.read_register (amdgpu_regnum_t::STATUS, &status_reg);
  if (status != AMD_DBGAPI_STATUS_SUCCESS)
    return status;

  uint32_t mode_reg;
  status = wave.read_register (amdgpu_regnum_t::MODE, &mode_reg);
  if (status != AMD_DBGAPI_STATUS_SUCCESS)
    return status;

  if (state == AMD_DBGAPI_WAVE_STATE_STOP)
    {
      if (wave.state () != AMD_DBGAPI_WAVE_STATE_STOP
          && !can_halt_at_endpgm ())
        {
          /* Check the instruction at the current pc. If it is a s_endpgm, we
             can't halt the wave, or we'll hang the gpu, so make it look like
             the wave has terminated by returning an invalid wave id error.  */

          if (terminate_wave_if_at_endpgm (wave))
            return AMD_DBGAPI_STATUS_ERROR_INVALID_WAVE_ID;
        }

      mode_reg &= ~SQ_WAVE_MODE_DEBUG_EN_MASK;
      status_reg |= SQ_WAVE_STATUS_HALT_MASK;
    }
  else if (state == AMD_DBGAPI_WAVE_STATE_RUN)
    {
      mode_reg &= ~SQ_WAVE_MODE_DEBUG_EN_MASK;
      status_reg &= ~SQ_WAVE_STATUS_HALT_MASK;
    }
  else if (state == AMD_DBGAPI_WAVE_STATE_SINGLE_STEP)
    {
      mode_reg |= SQ_WAVE_MODE_DEBUG_EN_MASK;
      status_reg &= ~SQ_WAVE_STATUS_HALT_MASK;
    }
  else
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;

  status = wave.write_register (amdgpu_regnum_t::STATUS, &status_reg);
  if (status != AMD_DBGAPI_STATUS_SUCCESS)
    return status;

  status = wave.write_register (amdgpu_regnum_t::MODE, &mode_reg);
  if (status != AMD_DBGAPI_STATUS_SUCCESS)
    return status;

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

  /* s_call: SOPP Opcode 1 [10111111 10000001 SIMM16] */
  return (encoding & 0xFFFF0000) == 0xBF810000;
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

static class gfx900_t final : public gfx9_base_t
{
public:
  gfx900_t () : gfx9_base_t (0, 0) {}

  elf_amdgpu_machine_t elf_amdgpu_machine () const override
  {
    return EF_AMDGPU_MACH_AMDGCN_GFX900;
  }
} gfx900_instance;

/* Raven Architecture.  */

static class gfx902_t final : public gfx9_base_t
{
public:
  gfx902_t () : gfx9_base_t (0, 2) {}

  elf_amdgpu_machine_t elf_amdgpu_machine () const override
  {
    return EF_AMDGPU_MACH_AMDGCN_GFX902;
  }
} gfx902_instance;

/* Vega12 Architecture.  */

static class gfx904_t final : public gfx9_base_t
{
public:
  gfx904_t () : gfx9_base_t (0, 4) {}

  elf_amdgpu_machine_t elf_amdgpu_machine () const override
  {
    return EF_AMDGPU_MACH_AMDGCN_GFX904;
  }
} gfx904_instance;

/* Vega20 Architecture.  */

static class gfx906_t final : public gfx9_base_t
{
public:
  gfx906_t () : gfx9_base_t (0, 6) {}

  elf_amdgpu_machine_t elf_amdgpu_machine () const override
  {
    return EF_AMDGPU_MACH_AMDGCN_GFX906;
  }
} gfx906_instance;

/* Arcturus Architecture.  */

static class gfx908_t final : public gfx9_base_t
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
} gfx908_instance;

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

static class gfx1010_t final : public gfx10_base_t
{
public:
  gfx1010_t () : gfx10_base_t (1, 0) {}

  elf_amdgpu_machine_t elf_amdgpu_machine () const override
  {
    return EF_AMDGPU_MACH_AMDGCN_GFX1010;
  }
} gfx1010_instance;

static class gfx1011_t final : public gfx10_base_t
{
public:
  gfx1011_t () : gfx10_base_t (1, 1) {}

  elf_amdgpu_machine_t elf_amdgpu_machine () const override
  {
    return EF_AMDGPU_MACH_AMDGCN_GFX1011;
  }
} gfx1011_instance;

static class gfx1012_t final : public gfx10_base_t
{
public:
  gfx1012_t () : gfx10_base_t (1, 2) {}

  elf_amdgpu_machine_t elf_amdgpu_machine () const override
  {
    return EF_AMDGPU_MACH_AMDGCN_GFX1012;
  }
} gfx1012_instance;

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
  /* Register this architecture.  */
  architecture_map[m_architecture_id] = this;
}

architecture_t::~architecture_t ()
{
  if (*m_disassembly_info != amd_comgr_disassembly_info_t{ 0 })
    amd_comgr_destroy_disassembly_info (*m_disassembly_info);
}

const architecture_t *
architecture_t::find (amd_dbgapi_architecture_id_t architecture_id)
{
  auto it = architecture_map.find (architecture_id);
  return it != architecture_map.end () ? it->second : nullptr;
}

const architecture_t *
architecture_t::find (int gfxip_major, int gfxip_minor, int gfxip_stepping)
{
  auto it = std::find_if (
      architecture_map.begin (), architecture_map.end (),
      [&] (const decltype (architecture_map)::value_type &value) {
        return value.second->gfxip_major () == gfxip_major
               && value.second->gfxip_minor () == gfxip_minor
               && value.second->gfxip_stepping () == gfxip_stepping;
      });

  return it != architecture_map.end () ? it->second : nullptr;
}

const architecture_t *
architecture_t::find (elf_amdgpu_machine_t elf_amdgpu_machine)
{
  auto it = std::find_if (
      architecture_map.begin (), architecture_map.end (),
      [&] (const decltype (architecture_map)::value_type &value) {
        return value.second->elf_amdgpu_machine () == elf_amdgpu_machine;
      });

  return it != architecture_map.end () ? it->second : nullptr;
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
                              amd_dbgapi_address_space_id_t{ 0 });

    default:
      return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;
    }

  return AMD_DBGAPI_STATUS_SUCCESS;
}

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
    const void *memory, char **instruction_text, size_t *address_operand_count,
    amd_dbgapi_global_address_t **address_operands)
{
  TRY;
  TRACE (architecture_id, address, size);

  if (!memory || !size || !instruction_text
      || !address_operands != !address_operand_count)
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

  /* Return the instruction text in client allocated memory.  */
  void *mem;
  size_t mem_size = instruction_str.length () + 1;
  mem = allocate_memory (mem_size);
  if (!mem)
    return AMD_DBGAPI_STATUS_ERROR_CLIENT_CALLBACK;

  memcpy (mem, instruction_str.c_str (), mem_size);
  *instruction_text = static_cast<char *> (mem);

  /* Return the operands in client allocated memory.  */
  if (address_operands)
    {
      mem_size = operands_vec.size () * sizeof (amd_dbgapi_global_address_t);
      mem = allocate_memory (mem_size);
      if (mem_size && !mem)
        {
          amd::dbgapi::deallocate_memory (*instruction_text);
          *instruction_text = nullptr;
          return AMD_DBGAPI_STATUS_ERROR_CLIENT_CALLBACK;
        }

      memcpy (mem, operands_vec.data (), mem_size);
      *address_operands = static_cast<amd_dbgapi_global_address_t *> (mem);
    }

  if (address_operand_count)
    *address_operand_count = operands_vec.size ();

  return AMD_DBGAPI_STATUS_SUCCESS;
  CATCH;
}
