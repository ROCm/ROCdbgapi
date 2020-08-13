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

#ifndef AMD_DBGAPI_ARCHITECTURE_H
#define AMD_DBGAPI_ARCHITECTURE_H 1

#include "amd-dbgapi.h"
#include "handle_object.h"
#include "memory.h"
#include "os_driver.h"
#include "register.h"
#include "utils.h"

#include <cstddef>
#include <cstdint>
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

class displaced_stepping_t;
class wave_t;

/* Architecture.  */

class architecture_t
{
private:
  static_assert (is_handle_type_v<amd_dbgapi_architecture_id_t>,
                 "amd_dbgapi_architecture_id_t is not a handle type");

  amd_comgr_disassembly_info_t disassembly_info () const;

protected:
  architecture_t (elf_amdgpu_machine_t e_machine, std::string target_triple);
  virtual void initialize () = 0;

public:
  virtual ~architecture_t ();

  /* Disallow copying & moving architecture instances.  */
  architecture_t (const architecture_t &) = delete;
  architecture_t (architecture_t &&) = delete;
  architecture_t &operator= (const architecture_t &) = delete;
  architecture_t &operator= (architecture_t &&) = delete;

  /* FIXME: add SQ prefetch instruction bytes size.  */

  virtual bool has_wave32_vgprs () const = 0;
  virtual bool has_wave64_vgprs () const = 0;
  virtual bool has_acc_vgprs () const = 0;

  enum class wave_info_t
  {
    vgprs,      /* Number of VGPRs.  */
    acc_vgprs,  /* Number of AccVGPR. */
    sgprs,      /* Number of SGPRs.  */
    lds_size,   /* LDS space (bytes).  */
    lane_count, /* Number of lanes.  */

    last_wave,  /* Last wave of threadgroup.  */
    first_wave, /* First wave of threadgroup.  */
  };

  using cwsr_descriptor_t = std::array<uint32_t, 2>;
  virtual uint32_t wave_get_info (cwsr_descriptor_t descriptor,
                                  wave_info_t query) const = 0;

  virtual void control_stack_iterate (
      const uint32_t *control_stack, size_t control_stack_words,
      amd_dbgapi_global_address_t wave_area_address,
      std::function<void (cwsr_descriptor_t, amd_dbgapi_global_address_t)>
          wave_callback) const = 0;

  virtual bool can_halt_at_endpgm () const = 0;
  virtual bool is_endpgm (const std::vector<uint8_t> &bytes) const = 0;
  virtual bool is_trap (const std::vector<uint8_t> &bytes,
                        uint16_t *trap_id = nullptr) const = 0;

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

  /* Return the bits that can be programmed in the address watch mask.  */
  virtual size_t watchpoint_mask_bits () const = 0;

  /* Convert a watchpoint kind into this architecture address watch mode.  */
  virtual std::optional<os_watch_mode_t>
  watchpoint_mode (amd_dbgapi_watchpoint_kind_t kind) const = 0;

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
  virtual const std::vector<uint8_t> &nop_instruction () const = 0;
  virtual const std::vector<uint8_t> &breakpoint_instruction () const = 0;
  virtual const std::vector<uint8_t> &endpgm_instruction () const = 0;

  bool can_halt_at (const std::vector<uint8_t> &instruction) const;

  virtual size_t displaced_stepping_buffer_size () const = 0;
  virtual bool
  displaced_stepping_copy (displaced_stepping_t &displaced_stepping,
                           bool *simulate) const = 0;
  virtual bool displaced_stepping_fixup (
      wave_t &wave, displaced_stepping_t &displaced_stepping) const = 0;
  virtual bool displaced_stepping_simulate (
      wave_t &wave, displaced_stepping_t &displaced_stepping) const = 0;

  virtual amd_dbgapi_status_t
  get_wave_coords (wave_t &wave, std::array<uint32_t, 3> &group_ids,
                   uint32_t *wave_in_group) const = 0;

  virtual amd_dbgapi_status_t
  get_wave_state (wave_t &wave, amd_dbgapi_wave_state_t *state,
                  amd_dbgapi_wave_stop_reason_t *stop_reason) const = 0;
  virtual amd_dbgapi_status_t
  set_wave_state (wave_t &wave, amd_dbgapi_wave_state_t state) const = 0;

  virtual amd_dbgapi_status_t
  enable_wave_traps (wave_t &wave, os_wave_launch_trap_mask_t mask) const = 0;
  virtual amd_dbgapi_status_t
  disable_wave_traps (wave_t &wave, os_wave_launch_trap_mask_t mask) const = 0;

  amd_dbgapi_architecture_id_t id () const { return m_architecture_id; }
  elf_amdgpu_machine_t elf_amdgpu_machine () const { return m_e_machine; }
  const std::string &target_triple () const { return m_target_triple; }

  template <typename ArchitectureType, typename... Args>
  static auto create_architecture (Args &&... args);

  static const architecture_t *
  find (amd_dbgapi_architecture_id_t architecture_id, int ignore = 0);
  static const architecture_t *find (elf_amdgpu_machine_t elf_amdgpu_machine);

  std::set<amdgpu_regnum_t> register_set () const;
  std::optional<std::string> register_name (amdgpu_regnum_t regnum) const;
  std::optional<std::string> register_type (amdgpu_regnum_t regnum) const;

  virtual amd_dbgapi_status_t read_pseudo_register (const wave_t &wave,
                                                    amdgpu_regnum_t regnum,
                                                    size_t offset,
                                                    size_t value_size,
                                                    void *value) const = 0;
  virtual amd_dbgapi_status_t
  write_pseudo_register (wave_t &wave, amdgpu_regnum_t regnum, size_t offset,
                         size_t value_size, const void *value) const = 0;

  amd_dbgapi_status_t instruction_size (const void *memory,
                                        size_t *size) const;
  size_t instruction_size (const std::vector<uint8_t> &bytes) const;
  amd_dbgapi_status_t disassemble_instruction (
      amd_dbgapi_global_address_t address, amd_dbgapi_size_t *size,
      const void *instruction_bytes, std::string &instruction_text,
      std::vector<amd_dbgapi_global_address_t> &address_operands) const;

  amd_dbgapi_status_t get_info (amd_dbgapi_architecture_info_t query,
                                size_t value_size, void *value) const;

  template <typename Object, typename... Args> auto &create (Args &&... args)
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
  template <typename Handle> const auto *find (Handle id) const
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

private:
  amd_dbgapi_architecture_id_t const m_architecture_id;
  std::unique_ptr<amd_comgr_disassembly_info_t> m_disassembly_info;

  elf_amdgpu_machine_t const m_e_machine;
  std::string const m_target_triple;

  std::tuple<handle_object_set_t<address_space_t>,
             handle_object_set_t<address_class_t>,
             handle_object_set_t<register_class_t>>
      m_handle_object_sets;

private:
  static monotonic_counter_t<decltype (amd_dbgapi_architecture_id_t::handle)>
      s_next_architecture_id;

  static std::unordered_map<amd_dbgapi_architecture_id_t,
                            std::unique_ptr<const architecture_t>,
                            hash<amd_dbgapi_architecture_id_t>>
      s_architecture_map;
};

} /* namespace amd::dbgapi */

#endif /* AMD_DBGAPI_ARCHITECTURE_H */
