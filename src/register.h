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

#ifndef _AMD_DBGAPI_REGISTER_H
#define _AMD_DBGAPI_REGISTER_H 1

#include "defs.h"

namespace amd
{
namespace dbgapi
{

/* ROCm Registers  */

enum class amdgpu_regnum_t : decltype (amd_dbgapi_register_id_t::handle)
{
  FIRST_RAW = 0,
  FIRST_VGPR_32 = FIRST_RAW,

  /* 32-bit Vector registers (vgprs) for wave32 wavefronts.  */
  V0_32 = FIRST_VGPR_32,
  V255_32 = V0_32 + 255,

  LAST_VGPR_32 = V255_32,
  FIRST_VGPR_64 = LAST_VGPR_32 + 1,

  /* 32-bit Vector registers (vgprs) for wave64 wavefronts.  */
  V0_64 = FIRST_VGPR_64,
  V255_64 = V0_64 + 255,

  LAST_VGPR_64 = V255_64,
  FIRST_ACCVGPR_32 = LAST_VGPR_64 + 1,

  /* 32-bit Accumulation Vector registers (accvgprs) for wave64 wavefronts  */
  ACC0_32 = FIRST_ACCVGPR_32,
  ACC255_32 = ACC0_32 + 255,

  LAST_ACCVGPR_32 = ACC255_32,
  FIRST_ACCVGPR_64 = LAST_ACCVGPR_32 + 1,

  /* 32-bit Accumulation Vector registers (accvgprs) for wave64 wavefronts  */
  ACC0_64 = FIRST_ACCVGPR_64,
  ACC255_64 = ACC0_64 + 255,

  LAST_ACCVGPR_64 = ACC255_64,
  FIRST_SGPR = LAST_ACCVGPR_64 + 1,

  /* 32-bit Scalar registers (sgprs).  */
  S0 = FIRST_SGPR,
  S111 = S0 + 111,

  LAST_SGPR = S111,
  FIRST_HWREG = LAST_SGPR + 1,

  /* Hardware registers (hwregs).  */
  M0 = FIRST_HWREG,   /* Memory Descriptor.  */
  PC,                 /* Program counter.  */
  EXEC = PC + 2,      /* Execution mask.  */
  STATUS = EXEC + 2,  /* Status register.  */
  TRAPSTS,            /* Exception status registers.  */
  MODE = TRAPSTS + 3, /* Mode register.  */

  LAST_HWREG = MODE,
  FIRST_TTMP = LAST_HWREG + 1,

  /* Trap temporary registers (ttmps).  */
  TTMP4 = FIRST_TTMP + 4,
  TTMP5,
  TTMP6,
  TTMP7,
  TTMP8,
  TTMP9,
  TTMP10,
  TTMP11,
  TTMP13 = TTMP11 + 2,

  LAST_TTMP = TTMP13 + 2,

  LAST_RAW = LAST_TTMP,
  FIRST_PSEUDO = LAST_RAW + 1,

  /* Pseudo registers.  */
  VCC = FIRST_PSEUDO, /* Vector Condition Code.  */
  XNACK_MASK,         /* XNACK mask.  */
  FLAT_SCRATCH,       /* Flat scratch.  */

  LAST_PSEUDO = FLAT_SCRATCH,
};

constexpr ssize_t
operator- (amdgpu_regnum_t lhs, amdgpu_regnum_t rhs)
{
  return static_cast<decltype (amd_dbgapi_register_id_t::handle)> (lhs)
         - static_cast<decltype (amd_dbgapi_register_id_t::handle)> (rhs);
}

constexpr amdgpu_regnum_t
operator+ (amdgpu_regnum_t lhs, int rhs)
{
  return static_cast<amdgpu_regnum_t> (
      static_cast<decltype (amd_dbgapi_register_id_t::handle)> (lhs) + rhs);
}

static inline amdgpu_regnum_t
operator++ (amdgpu_regnum_t &regnum)
{
  regnum = regnum + 1;
  return regnum;
}

static inline amdgpu_regnum_t
operator++ (amdgpu_regnum_t &regnum, int)
{
  amdgpu_regnum_t prev = regnum;
  regnum = regnum + 1;
  return prev;
}

constexpr size_t AMDGPU_VGPRS_32_COUNT
    = amdgpu_regnum_t::LAST_VGPR_32 - amdgpu_regnum_t::FIRST_VGPR_32 + 1;
constexpr size_t AMDGPU_VGPRS_64_COUNT
    = amdgpu_regnum_t::LAST_VGPR_64 - amdgpu_regnum_t::FIRST_VGPR_64 + 1;
constexpr size_t AMDGPU_ACCVGPRS_64_COUNT
    = amdgpu_regnum_t::LAST_ACCVGPR_64 - amdgpu_regnum_t::FIRST_ACCVGPR_64 + 1;
constexpr size_t AMDGPU_SGPRS_COUNT
    = amdgpu_regnum_t::LAST_SGPR - amdgpu_regnum_t::FIRST_SGPR + 1;
constexpr size_t AMDGPU_HWREGS_COUNT
    = amdgpu_regnum_t::LAST_HWREG - amdgpu_regnum_t::FIRST_HWREG + 1;
constexpr size_t AMDGPU_TTMPS_COUNT
    = amdgpu_regnum_t::LAST_TTMP - amdgpu_regnum_t::FIRST_TTMP + 1;
constexpr size_t AMDGPU_RAW_REGS_COUNT
    = amdgpu_regnum_t::LAST_RAW - amdgpu_regnum_t::FIRST_RAW + 1;
constexpr size_t AMDGPU_PSEUDO_REGS_COUNT
    = amdgpu_regnum_t::LAST_PSEUDO - amdgpu_regnum_t::FIRST_PSEUDO + 1;

} /* namespace dbgapi */
} /* namespace amd */

#endif /* _AMD_DBGAPI_REGISTER_H */
