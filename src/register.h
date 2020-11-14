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
#include <cstdint>
#include <map>
#include <set>
#include <string>
#include <type_traits>
#include <utility>

namespace amd::dbgapi
{

class architecture_t;

/* Registers  */

enum class amdgpu_regnum_t : uint32_t
{
  first_vgpr_32 = 0,

  /* 32-bit Vector registers (vgprs) for wave32 wavefronts.  */
  v0_32 = first_vgpr_32,
  v255_32 = v0_32 + 255,

  last_vgpr_32 = v255_32,
  first_vgpr_64 = last_vgpr_32 + 1,

  /* 32-bit Vector registers (vgprs) for wave64 wavefronts.  */
  v0_64 = first_vgpr_64,
  v255_64 = v0_64 + 255,

  last_vgpr_64 = v255_64,
  first_accvgpr_32 = last_vgpr_64 + 1,

  /* 32-bit Accumulation Vector registers (accvgprs) for wave64 wavefronts  */
  acc0_32 = first_accvgpr_32,
  acc255_32 = acc0_32 + 255,

  last_accvgpr_32 = acc255_32,
  first_accvgpr_64 = last_accvgpr_32 + 1,

  /* 32-bit Accumulation Vector registers (accvgprs) for wave64 wavefronts  */
  acc0_64 = first_accvgpr_64,
  acc255_64 = acc0_64 + 255,

  last_accvgpr_64 = acc255_64,
  first_sgpr = last_accvgpr_64 + 1,

  /* 32-bit Scalar registers (sgprs).  */
  s0 = first_sgpr,
  s105 = s0 + 105,

  last_sgpr = s105,

  /* Hardware registers (hwregs).  */
  first_hwreg = last_sgpr + 1,
  last_hwreg = first_hwreg + 15,

  first_ttmp = last_hwreg + 1,

  /* Trap temporary registers (ttmps).  */
  ttmp4 = first_ttmp + 4,
  ttmp5,
  ttmp6,
  ttmp7,
  ttmp8,
  ttmp9,
  ttmp10,
  ttmp11,
  ttmp13 = ttmp11 + 2,

  last_ttmp = ttmp13 + 2,

  /* Register aliases.  */
  m0 = last_ttmp + 1, /* Memory Descriptor.  */
  pc,                 /* Program counter.  */
  status,             /* Status register.  */
  mode,               /* Mode register.  */
  trapsts,            /* Exception status registers.  */
  exec_32,            /* Execution mask for wave32 wavefronts.  */
  exec_64,            /* Execution mask for wave64 wavefronts.  */
  vcc_32,             /* Vector Condition Code for wave32 wavefronts.  */
  vcc_64,             /* Vector Condition Code for wave64 wavefronts.  */
  xnack_mask_32,      /* XNACK mask for wave32 wavefronts.  */
  xnack_mask_64,      /* XNACK mask for wave64 wavefronts.  */
  flat_scratch,       /* Flat scratch.  */

  flat_scratch_lo, /* Flat scratch lower 32 bits.  */
  flat_scratch_hi, /* Flat scratch lower 32 bits.  */
  exec_lo,         /* Execution mask lower 32bits.  */
  exec_hi,         /* Execution mask lower 32bits.  */
  vcc_lo,          /* Vector Condition Code lower 32 bits.  */
  vcc_hi,          /* Vector Condition Code higher 32 bits.  */
  xnack_mask_lo,   /* XNACK mask lower 32 bits.  */
  xnack_mask_hi,   /* XNACK mask higher 32 bits.  */

  wave_id,         /* Debug[0:1].  */
  dispatch_ptr,    /* Pointer to the dispatch packet.  */
  dispatch_grid_x, /* Dispatch grid X.  */
  dispatch_grid_y, /* Dispatch grid Y.  */
  dispatch_grid_z, /* Dispatch grid Z.  */
  scratch_offset,  /* Scracth memory offset from the scratch base.  */

  lds_0, /* First dword of the LDS backing store.  */

  null, /* Special register: read returns 0, write is ignored.  */

  last_regnum = null
};

constexpr size_t
operator- (amdgpu_regnum_t lhs, amdgpu_regnum_t rhs)
{
  return static_cast<std::underlying_type_t<decltype (lhs)>> (lhs)
         - static_cast<std::underlying_type_t<decltype (rhs)>> (rhs);
}

constexpr amdgpu_regnum_t
operator+ (amdgpu_regnum_t lhs, int rhs)
{
  return static_cast<amdgpu_regnum_t> (
      static_cast<std::underlying_type_t<decltype (lhs)>> (lhs) + rhs);
}

constexpr amdgpu_regnum_t
operator- (amdgpu_regnum_t lhs, int rhs)
{
  return static_cast<amdgpu_regnum_t> (
      static_cast<std::underlying_type_t<decltype (lhs)>> (lhs) - rhs);
}

constexpr amdgpu_regnum_t operator& (amdgpu_regnum_t lhs, int rhs)
{
  return static_cast<amdgpu_regnum_t> (
      static_cast<std::underlying_type_t<decltype (lhs)>> (lhs) & rhs);
}

constexpr amdgpu_regnum_t
operator| (amdgpu_regnum_t lhs, int rhs)
{
  return static_cast<amdgpu_regnum_t> (
      static_cast<std::underlying_type_t<decltype (lhs)>> (lhs) | rhs);
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

constexpr size_t amdgpu_vgprs_32_count
    = amdgpu_regnum_t::last_vgpr_32 - amdgpu_regnum_t::first_vgpr_32 + 1;
constexpr size_t amdgpu_vgprs_64_count
    = amdgpu_regnum_t::last_vgpr_64 - amdgpu_regnum_t::first_vgpr_64 + 1;
constexpr size_t amdgpu_accvgprs_64_count
    = amdgpu_regnum_t::last_accvgpr_64 - amdgpu_regnum_t::first_accvgpr_64 + 1;
constexpr size_t amdgpu_sgprs_count
    = amdgpu_regnum_t::last_sgpr - amdgpu_regnum_t::first_sgpr + 1;
constexpr size_t amdgpu_hwregs_count
    = amdgpu_regnum_t::last_hwreg - amdgpu_regnum_t::first_hwreg + 1;
constexpr size_t amdgpu_ttmps_count
    = amdgpu_regnum_t::last_ttmp - amdgpu_regnum_t::first_ttmp + 1;

/* Register class.  */

class register_class_t
    : public detail::handle_object<amd_dbgapi_register_class_id_t>
{
public:
  using register_map_t = std::map<amdgpu_regnum_t, amdgpu_regnum_t>;

  register_class_t (amd_dbgapi_register_class_id_t register_class_id,
                    const architecture_t &architecture, std::string name,
                    register_map_t register_map)
      : handle_object (register_class_id), m_name (std::move (name)),
        m_register_map (std::move (register_map)),
        m_architecture (architecture)
  {
  }

  const std::string &name () const { return m_name; }

  bool contains (amdgpu_regnum_t regnum) const;
  std::set<amdgpu_regnum_t> register_set () const;

  amd_dbgapi_status_t get_info (amd_dbgapi_register_class_info_t query,
                                size_t value_size, void *value) const;

  const architecture_t &architecture () const { return m_architecture; }

private:
  std::string const m_name;
  register_map_t const m_register_map;
  const architecture_t &m_architecture;
};

} /* namespace amd::dbgapi */

#endif /* AMD_DBGAPI_REGISTER_H */
