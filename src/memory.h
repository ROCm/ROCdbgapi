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

#ifndef AMD_DBGAPI_MEMORY_H
#define AMD_DBGAPI_MEMORY_H 1

#include "amd-dbgapi.h"
#include "handle_object.h"
#include "utils.h"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace amd::dbgapi
{

class architecture_t;
class process_t;

/* AMDGPU DWARF Address Class Mapping
   See https://llvm.org/docs/AMDGPUUsage.html#address-class-identifier
 */
constexpr uint64_t DW_ADDR_none = 0x0000;
constexpr uint64_t DW_ADDR_LLVM_global = 0x0001;
constexpr uint64_t DW_ADDR_LLVM_constant = 0x0002;
constexpr uint64_t DW_ADDR_LLVM_group = 0x0003;
constexpr uint64_t DW_ADDR_LLVM_private = 0x0004;
constexpr uint64_t DW_ADDR_AMDGPU_region = 0x8000;

/* AMDGPU DWARF Address Space Mapping
   See https://llvm.org/docs/AMDGPUUsage.html#address-space-identifier
 */
constexpr uint64_t DW_ASPACE_none = 0x00;
constexpr uint64_t DW_ASPACE_AMDGPU_generic = 0x01;
constexpr uint64_t DW_ASPACE_AMDGPU_region = 0x02;
constexpr uint64_t DW_ASPACE_AMDGPU_local = 0x03;
constexpr uint64_t DW_ASPACE_AMDGPU_private_lane = 0x05;
constexpr uint64_t DW_ASPACE_AMDGPU_private_wave = 0x06;

class address_space_t
  : public detail::handle_object<amd_dbgapi_address_space_id_t>
{
public:
  enum address_space_kind_t
  {
    generic = 1,
    local,
    global,
    private_swizzled,
    private_unswizzled,
    region
  };

private:
  std::string const m_name;
  address_space_kind_t const m_kind;
  uint64_t const m_dwarf_value;
  amd_dbgapi_size_t const m_address_size;
  amd_dbgapi_segment_address_t const m_null_address;
  amd_dbgapi_address_space_access_t const m_access;
  const architecture_t &m_architecture;

public:
  address_space_t (amd_dbgapi_address_space_id_t address_space_id,
                   const architecture_t &architecture, std::string name,
                   address_space_kind_t kind, uint64_t dwarf_value,
                   amd_dbgapi_size_t address_size,
                   amd_dbgapi_segment_address_t null_address,
                   amd_dbgapi_address_space_access_t access)
    : handle_object (address_space_id), m_name (std::move (name)),
      m_kind (kind), m_dwarf_value (dwarf_value),
      m_address_size (address_size), m_null_address (null_address),
      m_access (access), m_architecture (architecture)
  {
  }

  uint64_t dwarf_value () const { return m_dwarf_value; }
  const std::string &name () const { return m_name; }
  address_space_kind_t kind () const { return m_kind; }
  amd_dbgapi_size_t address_size () const { return m_address_size; }
  amd_dbgapi_segment_address_t null_address () const { return m_null_address; }

  amd_dbgapi_status_t get_info (amd_dbgapi_address_space_info_t query,
                                size_t value_size, void *value) const;

  const architecture_t &architecture () const { return m_architecture; }
};

/* The amd_dbgapi_address_space_id_t{1} is reserved for the distinguished
   global address space id, to we need to start numbering at 2.  */
template <>
struct monotonic_counter_start_t<amd_dbgapi_address_space_id_t>
  : public std::integral_constant<
      decltype (amd_dbgapi_address_space_id_t::handle), 2>
{
};

class address_class_t
  : public detail::handle_object<amd_dbgapi_address_class_id_t>
{
public:
  address_class_t (amd_dbgapi_address_class_id_t address_class_id,
                   const architecture_t &architecture, std::string name,
                   uint64_t dwarf_value, const address_space_t &address_space)
    : handle_object (address_class_id), m_name (std::move (name)),
      m_dwarf_value (dwarf_value), m_address_space (address_space),
      m_architecture (architecture)
  {
  }

public:
  uint64_t dwarf_value () const { return m_dwarf_value; }
  const std::string &name () const { return m_name; }
  const address_space_t &address_space () const { return m_address_space; }

  amd_dbgapi_status_t get_info (amd_dbgapi_address_class_info_t query,
                                size_t value_size, void *value) const;

  const architecture_t &architecture () const { return m_architecture; }

private:
  std::string const m_name;
  uint64_t const m_dwarf_value;
  const address_space_t &m_address_space;
  const architecture_t &m_architecture;
};

class memory_cache_t
  : public utils::doubly_linked_list_t<memory_cache_t>::entry_type
{
public:
  enum class policy_t
  {
    /* If uncached is used, data is immediately written to global memory, and
       is not written to the cache.  */
    uncached = 0,
    /* If write-through is used, data is written both to global memory and to
       the cache.  */
    write_through,
    /* If write-back is used, data is immediately updated in the cache, and
       later updated in memory when the cache is flushed.  */
    write_back
  };

private:
  static monotonic_counter_t<uint64_t, 1> m_next_id;

  uint64_t const m_id{ m_next_id () };
  std::vector<uint8_t> m_cached_bytes{};
  std::optional<amd_dbgapi_global_address_t> m_address{};

  bool m_dirty{ false };

  policy_t m_policy;
  process_t &m_process;

public:
  memory_cache_t (process_t &process, policy_t policy = policy_t::write_back)
    : m_policy (policy), m_process (process)
  {
  }

  uint64_t id () const { return m_id; }
  size_t size () const { return m_cached_bytes.size (); }
  policy_t policy () const { return m_policy; }

  bool is_dirty () const { return m_dirty; }

  /* Returns true if the given memory region is cached in this instance.  The
     region must either be completely contained by the cache, or completely not
     contained by the cache.  */
  bool contains (amd_dbgapi_global_address_t address,
                 amd_dbgapi_size_t value_size) const;

  /* Changes the address of this cached region without affecting the data
     present in the cache.  The cached region may be dirty.  */
  void relocate (std::optional<amd_dbgapi_global_address_t> address);

  /* Invalidate the cache.  The cached region must not be dirty.  */
  void reset (amd_dbgapi_global_address_t address,
              amd_dbgapi_size_t cache_size);

  /* Make changes to the cached data visible in global memory.  */
  void flush ();

  /* The region must be completely contained by the cache.  */
  amd_dbgapi_status_t read (amd_dbgapi_global_address_t from, void *value,
                            size_t value_size) const;
  amd_dbgapi_status_t write (amd_dbgapi_global_address_t to, const void *value,
                             size_t value_size);
};

} /* namespace amd::dbgapi */

#endif /* AMD_DBGAPI_MEMORY_H */
