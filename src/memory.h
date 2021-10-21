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
#include <map>
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

  void get_info (amd_dbgapi_address_space_info_t query, size_t value_size,
                 void *value) const;

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

  void get_info (amd_dbgapi_address_class_info_t query, size_t value_size,
                 void *value) const;

  const architecture_t &architecture () const { return m_architecture; }

private:
  std::string const m_name;
  uint64_t const m_dwarf_value;
  const address_space_t &m_address_space;
  const architecture_t &m_architecture;
};

class memory_cache_t
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

  static constexpr size_t cache_line_size = 64;
  static constexpr policy_t policy = policy_t::write_back;

private:
  using delegate_fn_type = std::function<size_t (
    amd_dbgapi_global_address_t /* address */, void * /* read */,
    const void * /* write */, size_t /* size */)>;

  struct cache_line_t
  {
    std::array<std::byte, cache_line_size> m_data{};
    bool m_dirty{ false };
  };

  std::map<amd_dbgapi_global_address_t, cache_line_t> m_cache_line_map;
  delegate_fn_type const m_xfer_global_memory;

  void fetch_cache_line (cache_line_t &cache_line,
                         amd_dbgapi_global_address_t address) const;
  void commit_cache_line (cache_line_t &cache_line,
                          amd_dbgapi_global_address_t address) const;
  void allocate_0_cache_line (cache_line_t &cache_line) const;

  size_t xfer_global_memory (amd_dbgapi_global_address_t address, void *read,
                             const void *write, size_t size);

public:
  memory_cache_t (delegate_fn_type xfer_global_memory)
    : m_xfer_global_memory (std::move (xfer_global_memory))
  {
  }
  ~memory_cache_t () { dbgapi_assert (m_cache_line_map.empty ()); }

  bool contains_all (amd_dbgapi_global_address_t address,
                     amd_dbgapi_size_t size) const;

  /* Create cache lines if not already valid, and immediately fill them in.  */
  void prefetch (amd_dbgapi_global_address_t address, amd_dbgapi_size_t size);

  /* Discard all cache lines in the specified range.  The discarded cache lines
     must not be dirty.  */
  void discard (amd_dbgapi_global_address_t address = 0,
                amd_dbgapi_size_t size = -1);

  /* Write dirty lines back to memory.  */
  void write_back (amd_dbgapi_global_address_t address = 0,
                   amd_dbgapi_size_t size = -1);

  [[nodiscard]] size_t read_global_memory (amd_dbgapi_global_address_t address,
                                           void *buffer, size_t size)
  {
    return xfer_global_memory (address, buffer, nullptr, size);
  }

  [[nodiscard]] size_t
  write_global_memory (amd_dbgapi_global_address_t address, const void *buffer,
                       size_t size)
  {
    return xfer_global_memory (address, nullptr, buffer, size);
  }
};

/* An instruction_buffer holds the address and capacity of a global memory
   region used to store instructions. It behaves like a std::unique_ptr but is
   optimized to contain the instruction buffer instance data to avoid the cost
   associated with allocate/free.  An instruction buffer can hold one or more
   instructions, and is always terminated by a 'guard' instruction (s_trap). */
class instruction_buffer_t
{
private:
  using deleter_type = std::function<void (amd_dbgapi_global_address_t)>;

  struct
  {
    std::optional<amd_dbgapi_global_address_t> m_buffer_address{};
    uint32_t m_size{}; /* size of the instruction stored in this buffer.  */
    uint32_t m_capacity{}; /* the buffer's capacity in bytes.  */

    size_t size () const { return m_size; }
    void resize (size_t size)
    {
      if (size > m_capacity)
        fatal_error ("size exceeds capacity");
      m_size = size;
    }

    amd_dbgapi_global_address_t begin () const { return end () - size (); }
    amd_dbgapi_global_address_t end () const
    {
      dbgapi_assert (m_buffer_address.has_value ());
      return *m_buffer_address + m_capacity;
    }

    bool empty () const { return !size (); }
    void clear () { resize (0); }
  } m_data;

  deleter_type m_deleter; /* functor to deallocate the buffer when this
                               buffer is reset.  */

public:
  instruction_buffer_t () : m_data (), m_deleter () {}

  instruction_buffer_t (amd_dbgapi_global_address_t buffer_address,
                        uint32_t capacity, deleter_type deleter);

  instruction_buffer_t (instruction_buffer_t &&other);
  instruction_buffer_t &operator= (instruction_buffer_t &&other);

  /* Disable copies.  */
  instruction_buffer_t (const instruction_buffer_t &other) = delete;
  instruction_buffer_t &operator= (const instruction_buffer_t &other) = delete;

  ~instruction_buffer_t () { reset (); }

  decltype (m_data) *operator-> () { return &m_data; }
  decltype (m_data) const *operator-> () const { return &m_data; }

  operator bool () const { return m_data.m_buffer_address.has_value (); }

  void reset ();
  std::optional<amd_dbgapi_global_address_t> release ();
};

} /* namespace amd::dbgapi */

#endif /* AMD_DBGAPI_MEMORY_H */
