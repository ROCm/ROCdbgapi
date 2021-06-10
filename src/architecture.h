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

#ifndef AMD_DBGAPI_ARCHITECTURE_H
#define AMD_DBGAPI_ARCHITECTURE_H 1

#include "agent.h"
#include "amd-dbgapi.h"
#include "handle_object.h"
#include "memory.h"
#include "os_driver.h"
#include "queue.h"
#include "register.h"
#include "utils.h"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <tuple>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

#include <amd_comgr.h>

namespace amd::dbgapi
{

class architecture_t;
class process_t;
class wave_t;

namespace detail
{

/* The architecture that contained the last successful find result.  The next
   global find will start searching here.

   FIXME: If multi-process support is added this mechanism will need
   re-implementing to be thread safe.  */
extern const architecture_t *last_found_architecture;

} /* namespace detail */

/* Architecture.  */

class architecture_t
{
  static_assert (is_handle_type_v<amd_dbgapi_architecture_id_t>,
                 "amd_dbgapi_architecture_id_t is not a handle type");

private:
  static monotonic_counter_t<
    uint32_t, monotonic_counter_start_v<amd_dbgapi_architecture_id_t>>
    s_next_architecture_id;

  static std::unordered_map<amd_dbgapi_architecture_id_t,
                            std::unique_ptr<const architecture_t>,
                            hash<amd_dbgapi_architecture_id_t>>
    s_architecture_map;

  amd_dbgapi_architecture_id_t const m_architecture_id;
  std::unique_ptr<amd_comgr_disassembly_info_t> m_disassembly_info;

  elf_amdgpu_machine_t const m_e_machine;
  std::string const m_target_triple;

  std::tuple<handle_object_set_t<address_space_t>,
             handle_object_set_t<address_class_t>,
             handle_object_set_t<register_class_t>>
    m_handle_object_sets{};

  amd_comgr_disassembly_info_t disassembly_info () const;

protected:
  architecture_t (elf_amdgpu_machine_t e_machine, std::string target_triple);
  virtual void initialize () = 0;

public:
  /* Return the map of all instantiated architectures  */
  static const auto &all () { return s_architecture_map; }

  class cwsr_record_t
  {
  private:
    queue_t &m_queue;

  public:
    enum class query_kind_t
    {
      vgprs,      /* Number of VGPRs.  */
      acc_vgprs,  /* Number of AccVGPR. */
      sgprs,      /* Number of SGPRs.  */
      lds_size,   /* LDS space (bytes).  */
      lane_count, /* Number of lanes.  */

      last_wave,  /* Last wave of threadgroup.  */
      first_wave, /* First wave of threadgroup.  */

      is_halted,  /* The wave is halted (status.halt=1).  */
      is_stopped, /* The wave is stopped at the request of the trap handler. */
    };

    cwsr_record_t (queue_t &queue) : m_queue (queue) {}

    /* cwsr_record_t is a polymorphic base class.  */
    virtual ~cwsr_record_t () = default;

    virtual uint64_t get_info (query_kind_t query) const = 0;

    virtual std::optional<amd_dbgapi_global_address_t>
    register_address (amdgpu_regnum_t regnum) const = 0;

    /* The address of the first byte in the wave's context save.  */
    virtual amd_dbgapi_global_address_t begin () const = 0;
    /* The address of the byte following the last byte in the context save. */
    virtual amd_dbgapi_global_address_t end () const = 0;

    queue_t &queue () const { return m_queue; }
    agent_t &agent () const { return queue ().agent (); }
    process_t &process () const { return agent ().process (); }
  };

  virtual ~architecture_t ();

  /* Disallow copying & moving architecture instances.  */
  architecture_t (const architecture_t &) = delete;
  architecture_t (architecture_t &&) = delete;
  architecture_t &operator= (const architecture_t &) = delete;
  architecture_t &operator= (architecture_t &&) = delete;

  std::string name () const;

  /* Since architecture objects disallow copying & moving, two architecture
     objects are identical if they have the same address.  */
  bool operator== (const architecture_t &other) const
  {
    return this == &other;
  }
  bool operator!= (const architecture_t &other) const
  {
    return this != &other;
  }

  /* FIXME: add SQ prefetch instruction bytes size.  */

  /* Return the maximum number of scalar registers  */
  virtual size_t scalar_register_count () const = 0;
  virtual bool has_wave32_vgprs () const = 0;
  virtual bool has_wave64_vgprs () const = 0;
  virtual bool has_acc_vgprs () const = 0;

  virtual void control_stack_iterate (
    queue_t &queue, const uint32_t *control_stack, size_t control_stack_words,
    amd_dbgapi_global_address_t wave_area_address,
    amd_dbgapi_size_t wave_area_size,
    const std::function<void (std::unique_ptr<cwsr_record_t>)> &wave_callback)
    const = 0;

  virtual amd_dbgapi_global_address_t
  dispatch_packet_address (const cwsr_record_t &cwsr_record) const = 0;

  virtual bool can_halt_at_endpgm () const = 0;
  virtual bool is_endpgm (const std::vector<uint8_t> &bytes) const = 0;
  virtual bool is_breakpoint (const std::vector<uint8_t> &bytes) const = 0;

  virtual bool
  can_execute_displaced (const std::vector<uint8_t> &bytes) const = 0;
  virtual bool can_simulate (const std::vector<uint8_t> &bytes) const = 0;

  virtual amd_dbgapi_status_t
  convert_address_space (const wave_t &wave, amd_dbgapi_lane_id_t lane_id,
                         const address_space_t &from_address_space,
                         const address_space_t &to_address_space,
                         amd_dbgapi_segment_address_t from_address,
                         amd_dbgapi_segment_address_t *to_address) const = 0;

  virtual void lower_address_space (
    const wave_t &wave, amd_dbgapi_lane_id_t *lane_id,
    const address_space_t &original_address_space,
    const address_space_t **lowered_address_space,
    amd_dbgapi_segment_address_t original_address,
    amd_dbgapi_segment_address_t *lowered_address) const = 0;

  virtual bool
  address_is_in_address_class (const wave_t &wave,
                               amd_dbgapi_lane_id_t lane_id,
                               const address_space_t &address_space,
                               amd_dbgapi_segment_address_t segment_address,
                               const address_class_t &address_class) const = 0;

  virtual bool
  address_spaces_may_alias (const address_space_t &address_space1,
                            const address_space_t &address_space2) const = 0;

  /* Return true if this architecture supports setting precise memory.  */
  virtual bool supports_precise_memory () const = 0;

  /* Return the bits that can be programmed in the address watch mask.  */
  virtual size_t watchpoint_mask_bits () const = 0;

  virtual amd_dbgapi_watchpoint_share_kind_t
  watchpoint_share_kind () const = 0;

  /* Return the number of of address watch registers this architecture
     supports.  */
  virtual size_t watchpoint_count () const = 0;

  /* Return the watchpoints for which an exception was generated in the given
     stopped wave.  */
  virtual std::vector<os_watch_id_t>
  triggered_watchpoints (const wave_t &wave) const = 0;

  virtual size_t largest_instruction_size () const = 0;
  virtual size_t minimum_instruction_alignment () const = 0;
  virtual size_t breakpoint_instruction_pc_adjust () const = 0;
  virtual std::vector<uint8_t> nop_instruction () const = 0;
  virtual std::vector<uint8_t> breakpoint_instruction () const = 0;
  virtual std::vector<uint8_t> assert_instruction () const = 0;
  virtual std::vector<uint8_t> debug_trap_instruction () const = 0;
  virtual std::vector<uint8_t> endpgm_instruction () const = 0;

  virtual std::tuple<amd_dbgapi_instruction_kind_t /* instruction_kind  */,
                     amd_dbgapi_instruction_properties_t /* properties  */,
                     size_t /* instruction_size  */,
                     std::vector<uint64_t> /* information  */>
  classify_instruction (const std::vector<uint8_t> &instruction,
                        amd_dbgapi_global_address_t address) const = 0;

  virtual bool
  simulate_instruction (wave_t &wave, amd_dbgapi_global_address_t pc,
                        const std::vector<uint8_t> &instruction) const = 0;

  virtual void
  get_wave_state (wave_t &wave, amd_dbgapi_wave_state_t *state,
                  amd_dbgapi_wave_stop_reasons_t *stop_reason) const = 0;
  virtual void set_wave_state (wave_t &wave, amd_dbgapi_wave_state_t state,
                               amd_dbgapi_exceptions_t exceptions
                               = AMD_DBGAPI_EXCEPTION_NONE) const = 0;

  virtual void enable_wave_traps (wave_t &wave,
                                  os_wave_launch_trap_mask_t mask) const = 0;
  virtual void disable_wave_traps (wave_t &wave,
                                   os_wave_launch_trap_mask_t mask) const = 0;

  amd_dbgapi_architecture_id_t id () const { return m_architecture_id; }
  elf_amdgpu_machine_t elf_amdgpu_machine () const { return m_e_machine; }
  const std::string &target_triple () const { return m_target_triple; }

  template <typename ArchitectureType, typename... Args>
  static auto create_architecture (Args &&...args);

  static const architecture_t *
  find (amd_dbgapi_architecture_id_t architecture_id, int ignore = 0);
  static const architecture_t *find (elf_amdgpu_machine_t elf_amdgpu_machine);

  amd_dbgapi_register_id_t regnum_to_register_id (amdgpu_regnum_t regnum) const
  {
    using integral_type = decltype (amd_dbgapi_register_id_t::handle);

    static_assert (
      sizeof (typename decltype (s_next_architecture_id)::value_type) <= 4
      && sizeof (amdgpu_regnum_t) <= 4 && sizeof (integral_type) >= 8);

    auto register_id = (static_cast<integral_type> (id ().handle) << 32)
                       | static_cast<integral_type> (regnum);

    return amd_dbgapi_register_id_t{ register_id };
  }
  static std::optional<amdgpu_regnum_t>
  register_id_to_regnum (amd_dbgapi_register_id_t register_id)
  {
    amdgpu_regnum_t regnum = static_cast<amdgpu_regnum_t> (
      utils::bit_extract (register_id.handle, 0, 31));

    return (regnum <= amdgpu_regnum_t::last_regnum)
             ? std::make_optional (regnum)
             : std::nullopt;
  }
  static const architecture_t *
  register_id_to_architecture (amd_dbgapi_register_id_t register_id)
  {
    return find (amd_dbgapi_architecture_id_t{
      utils::bit_extract (register_id.handle, 32, 63) });
  }

  std::set<amdgpu_regnum_t> register_set () const;
  std::optional<std::string> register_name (amdgpu_regnum_t regnum) const;
  std::optional<std::string> register_type (amdgpu_regnum_t regnum) const;
  std::optional<amd_dbgapi_size_t>
  register_size (amdgpu_regnum_t regnum) const;

  virtual bool is_pseudo_register_available (const wave_t &wave,
                                             amdgpu_regnum_t regnum) const = 0;

  virtual void read_pseudo_register (const wave_t &wave,
                                     amdgpu_regnum_t regnum, size_t offset,
                                     size_t value_size, void *value) const = 0;

  virtual void write_pseudo_register (wave_t &wave, amdgpu_regnum_t regnum,
                                      size_t offset, size_t value_size,
                                      const void *value) const = 0;

  amd_dbgapi_size_t instruction_size (const void *instruction_bytes,
                                      size_t size) const;

  amd_dbgapi_size_t instruction_size (const std::vector<uint8_t> &bytes) const
  {
    return instruction_size (bytes.data (), bytes.size ());
  }

  std::tuple<amd_dbgapi_size_t /* instruction_size  */,
             std::string /* instruction_text  */,
             std::vector<amd_dbgapi_global_address_t> /* address_operands  */>
  disassemble_instruction (amd_dbgapi_global_address_t address,
                           const void *instruction_bytes, size_t size) const;

  amd_dbgapi_status_t get_info (amd_dbgapi_architecture_info_t query,
                                size_t value_size, void *value) const;

  template <typename Object, typename... Args> auto &create (Args &&...args)
  {
    return std::get<handle_object_set_t<Object>> (m_handle_object_sets)
      .create_object (std::forward<Args> (args)...);
  }

  /* Return an Object range. A range implements begin () and end (), and
     can be used to iterate the Objects.  */
  template <typename Object> auto range () const
  {
    return std::get<handle_object_set_t<Object>> (m_handle_object_sets)
      .range ();
  }

  /* Return the element count for the sub-Object.  */
  template <typename Object> size_t count () const
  {
    return std::get<handle_object_set_t<Object>> (m_handle_object_sets)
      .size ();
  }

  /* Find an object with the given handle.  */
  template <typename Handle,
            std::enable_if_t<!std::is_void_v<object_type_from_handle_t<
                               Handle, decltype (m_handle_object_sets)>>,
                             int> = 0>
  const auto *find (Handle id) const
  {
    using object_type
      = object_type_from_handle_t<Handle, decltype (m_handle_object_sets)>;

    return std::get<handle_object_set_t<object_type>> (m_handle_object_sets)
      .find (id);
  }

  /* Find an object for which the unary predicate f returns true.  */
  template <typename Functor> const auto *find_if (Functor predicate) const
  {
    using object_type = std::decay_t<utils::first_argument_of_t<Functor>>;

    return std::get<handle_object_set_t<object_type>> (m_handle_object_sets)
      .find_if (predicate);
  }
};

namespace detail
{
template <typename Handle>
using architecture_find_t
  = decltype (std::declval<architecture_t> ().find (std::declval<Handle> ()));
} /* namespace detail */

/* Find an object with the given handle.  */
template <
  typename Handle,
  std::enable_if_t<utils::is_detected_v<detail::architecture_find_t, Handle>,
                   int> = 0>
auto *
find (Handle id)
{
  if (detail::last_found_architecture)
    if (auto value = detail::last_found_architecture->find (id); value)
      return value;

  for (auto &&architecture : architecture_t::all ())
    {
      if (architecture.second.get () == detail::last_found_architecture)
        continue;
      if (auto value = architecture.second->find (id); value)
        {
          detail::last_found_architecture = architecture.second.get ();
          return value;
        }
    }

  return detail::architecture_find_t<Handle>{};
}

} /* namespace amd::dbgapi */

#endif /* AMD_DBGAPI_ARCHITECTURE_H */
