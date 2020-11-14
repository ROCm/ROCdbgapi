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

#include "register.h"
#include "architecture.h"
#include "debug.h"
#include "displaced_stepping.h"
#include "initialization.h"
#include "logging.h"
#include "process.h"
#include "queue.h"
#include "utils.h"
#include "wave.h"

#include <cstdint>
#include <iterator>
#include <map>
#include <optional>
#include <utility>

namespace amd::dbgapi
{

/* Register class.  */

bool
register_class_t::contains (amdgpu_regnum_t regnum) const
{
  auto it = m_register_map.upper_bound (regnum);
  if (it == m_register_map.begin ())
    return false;

  std::advance (it, -1);
  return regnum >= it->first && regnum <= it->second;
}

std::set<amdgpu_regnum_t>
register_class_t::register_set () const
{
  std::set<amdgpu_regnum_t> all_registers;

  for (auto interval : m_register_map)
    for (amdgpu_regnum_t regnum = interval.first; regnum <= interval.second;
         ++regnum)
      all_registers.insert (regnum);

  return all_registers;
}

amd_dbgapi_status_t
register_class_t::get_info (amd_dbgapi_register_class_info_t query,
                            size_t value_size, void *value) const
{
  switch (query)
    {
    case AMD_DBGAPI_REGISTER_CLASS_INFO_ARCHITECTURE:
      return utils::get_info (value_size, value, architecture ().id ());

    case AMD_DBGAPI_REGISTER_CLASS_INFO_NAME:
      return utils::get_info (value_size, value, name ());
    }
  return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;
}

} /* namespace amd::dbgapi */

using namespace amd::dbgapi;

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_architecture_register_class_get_info (
    amd_dbgapi_register_class_id_t register_class_id,
    amd_dbgapi_register_class_info_t query, size_t value_size, void *value)
{
  TRY;
  TRACE (register_class_id, query, value_size);

  if (!detail::is_initialized)
    return AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED;

  const register_class_t *register_class = find (register_class_id);

  if (!register_class)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_REGISTER_CLASS_ID;

  return register_class->get_info (query, value_size, value);
  CATCH;
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_architecture_register_class_list (
    amd_dbgapi_architecture_id_t architecture_id, size_t *register_class_count,
    amd_dbgapi_register_class_id_t **register_classes)
{
  TRY;
  TRACE (architecture_id);

  if (!detail::is_initialized)
    return AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED;

  const architecture_t *architecture = architecture_t::find (architecture_id);

  if (!architecture)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARCHITECTURE_ID;

  if (!register_class_count || !register_classes)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;

  size_t count = architecture->count<register_class_t> ();

  amd_dbgapi_register_class_id_t *class_ids
      = static_cast<amd_dbgapi_register_class_id_t *> (
          allocate_memory (count * sizeof (amd_dbgapi_register_class_id_t)));

  if (count && !class_ids)
    return AMD_DBGAPI_STATUS_ERROR_CLIENT_CALLBACK;

  *register_class_count = count;
  *register_classes = class_ids;

  for (auto &&register_class : architecture->range<register_class_t> ())
    *class_ids++ = register_class.id ();

  return AMD_DBGAPI_STATUS_SUCCESS;
  CATCH;
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_register_get_info (amd_dbgapi_register_id_t register_id,
                              amd_dbgapi_register_info_t query,
                              size_t value_size, void *value)
{
  TRY;
  TRACE (register_id, query, value_size);

  if (!detail::is_initialized)
    return AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED;

  auto regnum = architecture_t::register_id_to_regnum (register_id);

  const architecture_t *architecture
      = architecture_t::register_id_to_architecture (register_id);

  if (!architecture || !regnum)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_REGISTER_ID;

  if (!value)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;

  switch (query)
    {
    case AMD_DBGAPI_REGISTER_INFO_ARCHITECTURE:
      return utils::get_info (value_size, value, architecture->id ());

    case AMD_DBGAPI_REGISTER_INFO_NAME:
      {
        auto name = architecture->register_name (*regnum);
        if (!name)
          return AMD_DBGAPI_STATUS_ERROR_INVALID_REGISTER_ID;

        return utils::get_info (value_size, value, *name);
      }

    case AMD_DBGAPI_REGISTER_INFO_TYPE:
      {
        auto type = architecture->register_type (*regnum);
        if (!type)
          return AMD_DBGAPI_STATUS_ERROR_INVALID_REGISTER_ID;

        return utils::get_info (value_size, value, *type);
      }

    case AMD_DBGAPI_REGISTER_INFO_SIZE:
      {
        auto size = architecture->register_size (*regnum);
        if (!size)
          return AMD_DBGAPI_STATUS_ERROR_INVALID_REGISTER_ID;

        return utils::get_info (value_size, value, *size);
      }

    default:
      return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;
    }
  CATCH;
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_architecture_register_list (
    amd_dbgapi_architecture_id_t architecture_id, size_t *register_count,
    amd_dbgapi_register_id_t **registers)
{
  TRY;
  TRACE (architecture_id);

  if (!detail::is_initialized)
    return AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED;

  const architecture_t *architecture = architecture_t::find (architecture_id);

  if (!architecture)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARCHITECTURE_ID;

  if (!register_count || !registers)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;

  auto arch_registers = architecture->register_set ();

  auto *retval = static_cast<amd_dbgapi_register_id_t *> (allocate_memory (
      arch_registers.size () * sizeof (amd_dbgapi_register_id_t)));

  if (!retval)
    return AMD_DBGAPI_STATUS_ERROR_CLIENT_CALLBACK;

  size_t count = 0;
  for (auto it = arch_registers.begin (); it != arch_registers.end (); ++it)
    retval[count++] = architecture->regnum_to_register_id (*it);

  *register_count = count;
  *registers = retval;

  return AMD_DBGAPI_STATUS_SUCCESS;
  CATCH;
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_register_is_in_register_class (
    amd_dbgapi_register_class_id_t register_class_id,
    amd_dbgapi_register_id_t register_id,
    amd_dbgapi_register_class_state_t *register_class_state)
{
  TRY;
  TRACE (register_class_id, register_id);

  if (!detail::is_initialized)
    return AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED;

  const register_class_t *register_class = find (register_class_id);

  if (!register_class)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_REGISTER_CLASS_ID;

  auto regnum = architecture_t::register_id_to_regnum (register_id);

  const architecture_t *architecture
      = architecture_t::register_id_to_architecture (register_id);

  if (!regnum || !architecture)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_REGISTER_ID;

  if (!register_class_state)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;

  if (*architecture != register_class->architecture ())
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT_COMPATIBILITY;

  *register_class_state = register_class->contains (*regnum)
                              ? AMD_DBGAPI_REGISTER_CLASS_STATE_MEMBER
                              : AMD_DBGAPI_REGISTER_CLASS_STATE_NOT_MEMBER;

  return AMD_DBGAPI_STATUS_SUCCESS;
  CATCH;
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_dwarf_register_to_register (
    amd_dbgapi_architecture_id_t architecture_id, uint64_t dwarf_register,
    amd_dbgapi_register_id_t *register_id)
{
  TRY;
  TRACE (architecture_id, dwarf_register);

  if (!detail::is_initialized)
    return AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED;

  const architecture_t *architecture = architecture_t::find (architecture_id);

  if (!architecture)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARCHITECTURE_ID;

  if (!register_id)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;

  amdgpu_regnum_t regnum;

  /* See https://llvm.org/docs/AMDGPUUsage.html#register-mapping.  */
  if (dwarf_register == 1)
    {
      regnum = amdgpu_regnum_t::exec_32;
    }
  else if (dwarf_register == 17)
    {
      regnum = amdgpu_regnum_t::exec_64;
    }
  else if (dwarf_register == 16)
    {
      regnum = amdgpu_regnum_t::pc;
    }
  else if (dwarf_register >= 32 && dwarf_register <= 95)
    {
      /* Scalar registers 0-63.  */
      regnum = amdgpu_regnum_t::first_sgpr + (dwarf_register - 32);
    }
  else if (dwarf_register == 128)
    {
      regnum = amdgpu_regnum_t::status;
    }
  else if (dwarf_register == 512)
    {
      regnum = amdgpu_regnum_t::vcc_32;
    }
  else if (dwarf_register == 768)
    {
      regnum = amdgpu_regnum_t::vcc_64;
    }
  else if (dwarf_register >= 1088 && dwarf_register <= 1129)
    {
      /* Scalar registers 64-105.  */
      regnum = amdgpu_regnum_t::first_sgpr + (dwarf_register - 1024);
    }
  else if (dwarf_register >= 1536 && dwarf_register <= 1791)
    {
      /* Vector registers 0-255 (wave32).  */
      regnum = amdgpu_regnum_t::first_vgpr_32 + (dwarf_register - 1536);
    }
  else if (dwarf_register >= 2048 && dwarf_register <= 2303)
    {
      /* Accumulation Vector registers 0-255 (wave32).  */
      regnum = amdgpu_regnum_t::first_accvgpr_32 + (dwarf_register - 2048);
    }
  else if (dwarf_register >= 2560 && dwarf_register <= 2815)
    {
      /* Vector registers 0-255 (wave64).  */
      regnum = amdgpu_regnum_t::first_vgpr_64 + (dwarf_register - 2560);
    }
  else if (dwarf_register >= 3072 && dwarf_register <= 3327)
    {
      /* Accumulation Vector registers 0-255 (wave64).  */
      regnum = amdgpu_regnum_t::first_accvgpr_64 + (dwarf_register - 3072);
    }
  else
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT_COMPATIBILITY;

  if (!architecture->register_size (regnum))
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT_COMPATIBILITY;

  *register_id = architecture->regnum_to_register_id (regnum);

  return AMD_DBGAPI_STATUS_SUCCESS;
  CATCH;
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_read_register (amd_dbgapi_wave_id_t wave_id,
                          amd_dbgapi_register_id_t register_id,
                          amd_dbgapi_size_t offset,
                          amd_dbgapi_size_t value_size, void *value)
{
  TRY;
  TRACE (wave_id, register_id, offset, value_size);

  if (!detail::is_initialized)
    return AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED;

  wave_t *wave = find (wave_id);

  if (!wave)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_WAVE_ID;

  auto regnum = architecture_t::register_id_to_regnum (register_id);

  const architecture_t *architecture
      = architecture_t::register_id_to_architecture (register_id);

  if (!regnum || !architecture)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_REGISTER_ID;

  if (wave->state () != AMD_DBGAPI_WAVE_STATE_STOP)
    return AMD_DBGAPI_STATUS_ERROR_WAVE_NOT_STOPPED;

  if (!value)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;

  if (*architecture != wave->architecture ())
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT_COMPATIBILITY;

  std::optional<scoped_queue_suspend_t> suspend;

  if (!wave->is_register_cached (*regnum))
    {
      suspend.emplace (wave->queue (), "read register");

      /* Look for the wave_id again, the wave may have exited.  */
      if (!(wave = find (wave_id)))
        return AMD_DBGAPI_STATUS_ERROR_INVALID_WAVE_ID;
    }

  wave->read_register (*regnum, offset, value_size, value);

  return AMD_DBGAPI_STATUS_SUCCESS;
  CATCH;
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_write_register (amd_dbgapi_wave_id_t wave_id,
                           amd_dbgapi_register_id_t register_id,
                           amd_dbgapi_size_t offset,
                           amd_dbgapi_size_t value_size, const void *value)
{
  TRY;
  TRACE (wave_id, register_id, offset, value_size);

  if (!detail::is_initialized)
    return AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED;

  wave_t *wave = find (wave_id);

  if (!wave)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_WAVE_ID;

  auto regnum = architecture_t::register_id_to_regnum (register_id);

  const architecture_t *architecture
      = architecture_t::register_id_to_architecture (register_id);

  if (!regnum || !architecture)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_REGISTER_ID;

  if (wave->state () != AMD_DBGAPI_WAVE_STATE_STOP)
    return AMD_DBGAPI_STATUS_ERROR_WAVE_NOT_STOPPED;

  /* FIXME: Enable this check when the FIXME below is removed.
   *
   * / * Is displaced stepping active?  * /
   * if (wave->displaced_stepping ())
   *   return AMD_DBGAPI_STATUS_ERROR_DISPLACED_STEPPING_ACTIVE;
   */

  if (!value)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;

  if (*architecture != wave->architecture ())
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT_COMPATIBILITY;

  /* FIXME: This is a hack to work around a misbehaving gdb.  To cancel a
     displaced stepping operation, gdb resets the pc to the original location
     instead of calling amd_dbgapi_displaced_stepping_complete.  We detect this
     condition here, and complete the aborted displaced stepping.  */
  if (*regnum == amdgpu_regnum_t::pc && wave->displaced_stepping ()
      && offset == 0 && value_size == sizeof (uint64_t)
      && *(uint64_t *)(value) != wave->displaced_stepping ()->to ())
    {
      scoped_queue_suspend_t suspend (wave->queue (),
                                      "displaced stepping complete");
      wave->displaced_stepping_complete ();
    }

  std::optional<scoped_queue_suspend_t> suspend;

  if (!wave->is_register_cached (*regnum)
      /* Write-through needs to update the memory as well as the cache, so we
         always need to suspend the queue.  */
      || wave_t::register_cache_policy
             == wave_t::register_cache_policy_t::write_through)
    {
      suspend.emplace (wave->queue (), "write register");

      /* Look for the wave_id again, the wave may have exited.  */
      if (!(wave = find (wave_id)))
        return AMD_DBGAPI_STATUS_ERROR_INVALID_WAVE_ID;
    }

  wave->write_register (*regnum, offset, value_size, value);

  return AMD_DBGAPI_STATUS_SUCCESS;
  CATCH;
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_wave_register_exists (amd_dbgapi_wave_id_t wave_id,
                                 amd_dbgapi_register_id_t register_id,
                                 amd_dbgapi_register_exists_t *exists)
{
  TRY;
  TRACE (wave_id, register_id);

  if (!detail::is_initialized)
    return AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED;

  wave_t *wave = find (wave_id);

  if (!wave)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_WAVE_ID;

  auto regnum = architecture_t::register_id_to_regnum (register_id);

  const architecture_t *architecture
      = architecture_t::register_id_to_architecture (register_id);

  if (!regnum || !architecture)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_REGISTER_ID;

  if (!exists)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;

  if (*architecture != wave->architecture ())
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT_COMPATIBILITY;

  *exists = wave->is_register_available (*regnum) ? AMD_DBGAPI_REGISTER_PRESENT
                                                  : AMD_DBGAPI_REGISTER_ABSENT;

  return AMD_DBGAPI_STATUS_SUCCESS;
  CATCH;
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_wave_register_list (amd_dbgapi_wave_id_t wave_id,
                               size_t *register_count,
                               amd_dbgapi_register_id_t **registers)
{
  TRY;
  TRACE (wave_id);

  if (!detail::is_initialized)
    return AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED;

  wave_t *wave = find (wave_id);

  if (!wave)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_WAVE_ID;

  if (!registers || !register_count)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;

  auto architecture_registers = wave->architecture ().register_set ();
  auto *retval = static_cast<amd_dbgapi_register_id_t *> (allocate_memory (
      architecture_registers.size () * sizeof (amd_dbgapi_register_id_t)));

  if (!retval)
    return AMD_DBGAPI_STATUS_ERROR_CLIENT_CALLBACK;

  size_t count = 0;
  for (auto &&regnum : architecture_registers)
    if (wave->is_register_available (regnum))
      retval[count++] = wave->architecture ().regnum_to_register_id (regnum);

  *register_count = count;
  *registers = retval;

  return AMD_DBGAPI_STATUS_SUCCESS;
  CATCH;
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_prefetch_register (amd_dbgapi_wave_id_t wave_id,
                              amd_dbgapi_register_id_t register_id,
                              amd_dbgapi_size_t register_count)
{
  TRY;
  TRACE (wave_id, register_id, register_count);

  if (!detail::is_initialized)
    return AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED;

  wave_t *wave = find (wave_id);

  if (!wave)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_WAVE_ID;

  auto regnum = architecture_t::register_id_to_regnum (register_id);

  const architecture_t *architecture
      = architecture_t::register_id_to_architecture (register_id);

  if (!regnum || !architecture)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_REGISTER_ID;

  if (wave->state () != AMD_DBGAPI_WAVE_STATE_STOP)
    return AMD_DBGAPI_STATUS_ERROR_WAVE_NOT_STOPPED;

  if (*architecture != wave->architecture ())
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT_COMPATIBILITY;

  return AMD_DBGAPI_STATUS_SUCCESS;
  CATCH;
}
