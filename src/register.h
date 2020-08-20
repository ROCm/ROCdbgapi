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

#ifndef AMD_DBGAPI_REGISTER_H
#define AMD_DBGAPI_REGISTER_H 1

#include "amd-dbgapi.h"
#include "handle_object.h"

#include <cstddef>
#include <map>
#include <set>
#include <string>
#include <utility>

namespace amd::dbgapi
{

/* Registers  */

enum class amdgpu_regnum_t : decltype (amd_dbgapi_register_id_t::handle)
{
  FIRST_VGPR_32 = 0,

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

  /* Hardware registers (hwregs).  */
  FIRST_HWREG = LAST_SGPR + 1,
  LAST_HWREG = FIRST_HWREG + 15,

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

  /* Register aliases.  */
  M0 = LAST_TTMP + 1, /* Memory Descriptor.  */
  PC,                 /* Program counter.  */
  STATUS,             /* Status register.  */
  MODE,               /* Mode register.  */
  TRAPSTS,            /* Exception status registers.  */
  EXEC_32,            /* Execution mask for wave32 wavefronts.  */
  EXEC_64,            /* Execution mask for wave64 wavefronts.  */
  VCC_32,             /* Vector Condition Code for wave32 wavefronts.  */
  VCC_64,             /* Vector Condition Code for wave64 wavefronts.  */
  XNACK_MASK_32,      /* XNACK mask for wave32 wavefronts.  */
  XNACK_MASK_64,      /* XNACK mask for wave64 wavefronts.  */
  FLAT_SCRATCH,       /* Flat scratch.  */

  WAVE_ID,         /* Debug[0:1].  */
  DISPATCH_PTR,    /* Pointer to the dispatch packet.  */
  DISPATCH_GRID_X, /* Dispatch grid X.  */
  DISPATCH_GRID_Y, /* Dispatch grid Y.  */
  DISPATCH_GRID_Z, /* Dispatch grid Z.  */
  SCRATCH_OFFSET,  /* Scracth memory offset from the scratch base.  */

  LDS_0, /* First dword of the LDS backing store.  */
};

constexpr size_t
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

constexpr amdgpu_regnum_t
operator- (amdgpu_regnum_t lhs, int rhs)
{
  return static_cast<amdgpu_regnum_t> (
      static_cast<decltype (amd_dbgapi_register_id_t::handle)> (lhs) - rhs);
}

constexpr amdgpu_regnum_t
operator++ (amdgpu_regnum_t &regnum)
{
  regnum = regnum + 1;
  return regnum;
}

constexpr amdgpu_regnum_t
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

/* Register class.  */

class register_class_t
    : public detail::handle_object<amd_dbgapi_register_class_id_t>
{
public:
  using register_map_t = std::map<amdgpu_regnum_t, amdgpu_regnum_t>;

  register_class_t (amd_dbgapi_register_class_id_t register_class_id,
                    std::string name, register_map_t register_map)
      : handle_object (register_class_id), m_name (std::move (name)),
        m_register_map (std::move (register_map))
  {
  }

  const std::string &name () const { return m_name; }

  bool contains (amdgpu_regnum_t regnum) const;
  std::set<amdgpu_regnum_t> register_set () const;

  amd_dbgapi_status_t get_info (amd_dbgapi_register_class_info_t query,
                                size_t value_size, void *value) const;

private:
  std::string const m_name;
  register_map_t const m_register_map;
};

} /* namespace amd::dbgapi */

#endif /* AMD_DBGAPI_REGISTER_H */
