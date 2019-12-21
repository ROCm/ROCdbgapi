/* Copyright (c) 2019 Advanced Micro Devices, Inc.

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
#include "logging.h"
#include "process.h"
#include "queue.h"
#include "register.h"
#include "utils.h"
#include "wave.h"

#include <cstring>
#include <string>
#include <utility> // for pair

namespace amd
{
namespace dbgapi
{

/* ROCm Register class.  */

enum register_class_t
{
  REGISTER_CLASS_NONE = 0,
  REGISTER_CLASS_GENERAL,
  REGISTER_CLASS_VECTOR,
  REGISTER_CLASS_SCALAR,
  REGISTER_CLASS_SYSTEM,
};

static std::string register_class_name[] = {
  /* [REGISTER_CLASS_NONE] = */ "",
  /* [REGISTER_CLASS_GENERAL] = */ "general",
  /* [REGISTER_CLASS_VECTOR] = */ "vector",
  /* [REGISTER_CLASS_SCALAR] = */ "scalar",
  /* [REGISTER_CLASS_SYSTEM] = */ "system",
};

} /* namespace dbgapi */
} /* namespace amd */

using namespace amd::dbgapi;

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_architecture_register_class_get_info (
    amd_dbgapi_architecture_id_t architecture_id,
    amd_dbgapi_register_class_id_t register_class_id,
    amd_dbgapi_register_class_info_t query, size_t value_size, void *value)
{
  TRY;
  TRACE (architecture_id, register_class_id, query, value_size);

  if (!amd::dbgapi::is_initialized)
    return AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED;

  const architecture_t *architecture = architecture_t::find (architecture_id);

  if (!architecture)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARCHITECTURE_ID;

  decltype (amd_dbgapi_register_class_id_t::handle) regclass
      = register_class_id.handle;

  if (!regclass
      || regclass >= (sizeof (register_class_name)
                      / sizeof (register_class_name[0])))
    return AMD_DBGAPI_STATUS_ERROR_INVALID_REGISTER_CLASS_ID;

  if (!value)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;

  switch (query)
    {
    case AMD_DBGAPI_REGISTER_CLASS_INFO_NAME:
      return utils::get_info (value_size, value,
                              register_class_name[regclass]);

    default:
      return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;
    }

  return AMD_DBGAPI_STATUS_SUCCESS;
  CATCH;
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_architecture_register_class_list (
    amd_dbgapi_architecture_id_t architecture_id, size_t *register_class_count,
    amd_dbgapi_register_class_id_t **register_classes)
{
  TRY;
  TRACE (architecture_id);

  if (!amd::dbgapi::is_initialized)
    return AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED;

  const architecture_t *architecture = architecture_t::find (architecture_id);

  if (!architecture)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARCHITECTURE_ID;

  if (!register_class_count || !register_classes)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;

  /* Don't include the first element (class_none).  */
  size_t count
      = sizeof (register_class_name) / sizeof (register_class_name[0]) - 1;

  amd_dbgapi_register_class_id_t *classes
      = static_cast<amd_dbgapi_register_class_id_t *> (
          allocate_memory (count * sizeof (amd_dbgapi_register_class_id_t)));

  if (!classes)
    return AMD_DBGAPI_STATUS_ERROR_CLIENT_CALLBACK;

  /* Skip the first element (class_none).  */
  for (size_t i = 1; i < (count + 1); ++i)
    classes[i - 1] = amd_dbgapi_register_class_id_t{ i };

  *register_class_count = count;
  *register_classes = classes;

  return AMD_DBGAPI_STATUS_SUCCESS;
  CATCH;
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_architecture_register_get_info (
    amd_dbgapi_architecture_id_t architecture_id,
    amd_dbgapi_register_id_t register_id, amd_dbgapi_register_info_t query,
    size_t value_size, void *value)
{
  TRY;
  TRACE (architecture_id, register_id, query, value_size);

  if (!amd::dbgapi::is_initialized)
    return AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED;

  const architecture_t *architecture = architecture_t::find (architecture_id);

  if (!architecture)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARCHITECTURE_ID;

  if (!value)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;

  switch (query)
    {
    case AMD_DBGAPI_REGISTER_INFO_NAME:
      {
        std::string name = architecture->register_name (
            static_cast<amdgpu_regnum_t> (register_id.handle));

        if (name.empty ())
          return AMD_DBGAPI_STATUS_ERROR_INVALID_REGISTER_ID;

        return utils::get_info (value_size, value, name);
      }

    default:
      return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;
    }

  return AMD_DBGAPI_STATUS_SUCCESS;
  CATCH;
}

amd_dbgapi_status_t
amd_dbgapi_architecture_register_list_1 (const architecture_t &architecture,
                                         size_t *register_count,
                                         amd_dbgapi_register_id_t **registers)
{
  if (!register_count || !registers)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;

  amdgpu_regnum_t i;
  size_t cur_pos = 0,
         count = AMDGPU_VGPRS_COUNT
                 + (architecture.has_acc_vgprs () ? AMDGPU_ACCVGPRS_COUNT : 0)
                 + AMDGPU_SGPRS_COUNT + AMDGPU_HWREGS_COUNT
                 + AMDGPU_TTMPS_COUNT;

  amd_dbgapi_register_id_t *retval;
  retval = static_cast<amd_dbgapi_register_id_t *> (
      allocate_memory (count * sizeof (amd_dbgapi_register_id_t)));

  if (!retval)
    return AMD_DBGAPI_STATUS_ERROR_CLIENT_CALLBACK;

  for (i = amdgpu_regnum_t::FIRST_VGPR; i <= amdgpu_regnum_t::LAST_VGPR; ++i)
    retval[cur_pos++].handle
        = static_cast<decltype (amd_dbgapi_register_id_t::handle)> (i);

  if (architecture.has_acc_vgprs ())
    for (i = amdgpu_regnum_t::FIRST_ACCVGPR;
         i <= amdgpu_regnum_t::LAST_ACCVGPR; ++i)
      retval[cur_pos++].handle
          = static_cast<decltype (amd_dbgapi_register_id_t::handle)> (i);

  for (i = amdgpu_regnum_t::FIRST_SGPR; i <= amdgpu_regnum_t::LAST_SGPR; ++i)
    retval[cur_pos++].handle
        = static_cast<decltype (amd_dbgapi_register_id_t::handle)> (i);

  for (i = amdgpu_regnum_t::FIRST_HWREG; i <= amdgpu_regnum_t::LAST_HWREG; ++i)
    retval[cur_pos++].handle
        = static_cast<decltype (amd_dbgapi_register_id_t::handle)> (i);

  for (i = amdgpu_regnum_t::FIRST_TTMP; i <= amdgpu_regnum_t::LAST_TTMP; ++i)
    retval[cur_pos++].handle
        = static_cast<decltype (amd_dbgapi_register_id_t::handle)> (i);

  dbgapi_assert (count == cur_pos && "register_list size mismatch");

  *register_count = count;
  *registers = retval;

  return AMD_DBGAPI_STATUS_SUCCESS;
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_architecture_register_list (
    amd_dbgapi_architecture_id_t architecture_id, size_t *register_count,
    amd_dbgapi_register_id_t **registers)
{
  TRY;
  TRACE (architecture_id);

  if (!amd::dbgapi::is_initialized)
    return AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED;

  const architecture_t *architecture = architecture_t::find (architecture_id);

  if (!architecture)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARCHITECTURE_ID;

  return amd_dbgapi_architecture_register_list_1 (*architecture,
                                                  register_count, registers);

  CATCH;
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_register_is_in_register_class (
    amd_dbgapi_architecture_id_t architecture_id,
    amd_dbgapi_register_id_t register_id,
    amd_dbgapi_register_class_id_t register_class_id,
    amd_dbgapi_register_class_state_t *register_class_state)
{
  TRY;
  TRACE (architecture_id, register_id, register_class_id);

  if (!amd::dbgapi::is_initialized)
    return AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED;

  const architecture_t *architecture = architecture_t::find (architecture_id);

  if (!architecture)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARCHITECTURE_ID;

  if (!register_class_state)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;

  decltype (amd_dbgapi_register_class_id_t::handle) regclass
      = register_class_id.handle;

  amdgpu_regnum_t regnum = static_cast<amdgpu_regnum_t> (register_id.handle);

  if (!regclass
      || regclass >= (sizeof (register_class_name)
                      / sizeof (register_class_name[0])))
    return AMD_DBGAPI_STATUS_ERROR_INVALID_REGISTER_CLASS_ID;

  bool is_in_register_class;

  if ((regnum >= amdgpu_regnum_t::FIRST_VGPR
       && regnum <= amdgpu_regnum_t::LAST_VGPR)
      || (architecture->has_acc_vgprs ()
          && regnum >= amdgpu_regnum_t::FIRST_ACCVGPR
          && regnum <= amdgpu_regnum_t::LAST_ACCVGPR))
    {
      is_in_register_class = (regclass == REGISTER_CLASS_GENERAL
                              || regclass == REGISTER_CLASS_VECTOR);
    }
  else if ((regnum >= amdgpu_regnum_t::FIRST_SGPR
            && regnum <= amdgpu_regnum_t::LAST_SGPR))
    {
      is_in_register_class = (regclass == REGISTER_CLASS_GENERAL
                              || regclass == REGISTER_CLASS_SCALAR);
    }
  else if (regnum == amdgpu_regnum_t::VCC || regnum == amdgpu_regnum_t::PC
           || regnum == amdgpu_regnum_t::EXEC)
    {
      is_in_register_class = (regclass == REGISTER_CLASS_GENERAL);
    }
  else if ((regnum >= amdgpu_regnum_t::FIRST_HWREG
            && regnum <= amdgpu_regnum_t::LAST_HWREG)
           || (regnum >= amdgpu_regnum_t::FIRST_TTMP
               && regnum <= amdgpu_regnum_t::LAST_TTMP)
           || regnum == amdgpu_regnum_t::FLAT_SCRATCH
           || regnum == amdgpu_regnum_t::XNACK_MASK)
    {
      is_in_register_class = (regclass == REGISTER_CLASS_SYSTEM);
    }
  else
    return AMD_DBGAPI_STATUS_ERROR_INVALID_REGISTER_ID;

  *register_class_state = is_in_register_class
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

  if (!amd::dbgapi::is_initialized)
    return AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED;

  const architecture_t *architecture = architecture_t::find (architecture_id);

  if (!architecture)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARCHITECTURE_ID;

  if (!register_id)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;

  amdgpu_regnum_t regnum;

  if (dwarf_register >= 0 && dwarf_register <= 105)
    {
      /* Scalar registers.  */
      regnum = amdgpu_regnum_t::FIRST_SGPR + dwarf_register;
    }
  else if (dwarf_register == 106)
    {
      regnum = amdgpu_regnum_t::VCC;
    }
  else if (dwarf_register >= 108 && dwarf_register <= 123)
    {
      /* Trap temporary registers.  */
      regnum = amdgpu_regnum_t::FIRST_TTMP + (dwarf_register - 108);
    }
  else if (dwarf_register >= 256 && dwarf_register <= 511)
    {
      /* Vector registers.  */
      regnum = amdgpu_regnum_t::FIRST_VGPR + (dwarf_register - 256);
    }
  else
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;

  register_id->handle
      = static_cast<decltype (amd_dbgapi_register_id_t::handle)> (regnum);

  return AMD_DBGAPI_STATUS_SUCCESS;
  CATCH;
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_read_register (amd_dbgapi_process_id_t process_id,
                          amd_dbgapi_wave_id_t wave_id,
                          amd_dbgapi_register_id_t register_id,
                          amd_dbgapi_size_t offset,
                          amd_dbgapi_size_t value_size, void *value)
{
  TRY;
  TRACE (process_id, wave_id, register_id, offset, value_size);

  if (!amd::dbgapi::is_initialized)
    return AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED;

  process_t *process = process_t::find (process_id);

  if (!process)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_PROCESS_ID;

  wave_t *wave = process->find (wave_id);

  if (!wave)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_WAVE_ID;

  if (!value)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;

  if (wave->state () != AMD_DBGAPI_WAVE_STATE_STOP)
    return AMD_DBGAPI_STATUS_ERROR_WAVE_NOT_STOPPED;

  amdgpu_regnum_t regnum = static_cast<amdgpu_regnum_t> (register_id.handle);

  /* TODO: Implement a proper register cache. For now, we only cache the
     program counter.  */
  if (regnum == amdgpu_regnum_t::PC)
    {
      amd_dbgapi_global_address_t pc = wave->pc ();

      if (!value_size || (offset + value_size) > sizeof (pc))
        return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT_SIZE;

      memcpy (static_cast<char *> (value) + offset,
              reinterpret_cast<const char *> (&pc) + offset, value_size);

      return AMD_DBGAPI_STATUS_SUCCESS;
    }

  {
    scoped_queue_suspend_t suspend (wave->queue ());

    /* Look for the wave_id again, the wave may have exited.  */
    if (!(wave = process->find (wave_id)))
      return AMD_DBGAPI_STATUS_ERROR_INVALID_WAVE_ID;

    return wave->read_register (regnum, offset, value_size, value);
  }
  CATCH;
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_write_register (amd_dbgapi_process_id_t process_id,
                           amd_dbgapi_wave_id_t wave_id,
                           amd_dbgapi_register_id_t register_id,
                           amd_dbgapi_size_t offset,
                           amd_dbgapi_size_t value_size, const void *value)
{
  TRY;
  TRACE (process_id, wave_id, register_id, offset, value_size);

  if (!amd::dbgapi::is_initialized)
    return AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED;

  process_t *process = process_t::find (process_id);

  if (!process)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_PROCESS_ID;

  wave_t *wave = process->find (wave_id);

  if (!wave)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_WAVE_ID;

  if (!value)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;

  if (wave->state () != AMD_DBGAPI_WAVE_STATE_STOP)
    return AMD_DBGAPI_STATUS_ERROR_WAVE_NOT_STOPPED;

  amdgpu_regnum_t regnum = static_cast<amdgpu_regnum_t> (register_id.handle);

  {
    scoped_queue_suspend_t suspend (wave->queue ());

    /* Look for the wave_id again, the wave may have exited.  */
    if (!(wave = process->find (wave_id)))
      return AMD_DBGAPI_STATUS_ERROR_INVALID_WAVE_ID;

    return wave->write_register (regnum, offset, value_size, value);
  }
  CATCH;
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_wave_register_get_info (amd_dbgapi_process_id_t process_id,
                                   amd_dbgapi_wave_id_t wave_id,
                                   amd_dbgapi_register_id_t register_id,
                                   amd_dbgapi_register_info_t query,
                                   size_t value_size, void *value)
{
  TRY;
  TRACE (process_id, wave_id, register_id, query, value_size);

  if (!amd::dbgapi::is_initialized)
    return AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED;

  process_t *process = process_t::find (process_id);

  if (!process)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_PROCESS_ID;

  wave_t *wave = process->find (wave_id);

  if (!wave)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_WAVE_ID;

  if (!value)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;

  amdgpu_regnum_t regnum = static_cast<amdgpu_regnum_t> (register_id.handle);

  switch (query)
    {
    case AMD_DBGAPI_REGISTER_INFO_NAME:
      {
        char *retval;

        if (value_size != sizeof (retval))
          return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT_SIZE;

        std::string name = wave->register_name (regnum);
        if (name.empty ())
          return AMD_DBGAPI_STATUS_ERROR_INVALID_REGISTER_ID;

        if (!(retval
              = static_cast<char *> (allocate_memory (name.length () + 1))))
          return AMD_DBGAPI_STATUS_ERROR_CLIENT_CALLBACK;

        strcpy (retval, name.c_str ());
        *static_cast<decltype (retval) *> (value) = retval;
      }
      break;

    case AMD_DBGAPI_REGISTER_INFO_TYPE:
      {
        char *retval;

        if (value_size != sizeof (retval))
          return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT_SIZE;

        std::string type = wave->register_type (regnum);
        if (type.empty ())
          return AMD_DBGAPI_STATUS_ERROR_INVALID_REGISTER_ID;

        if (!(retval
              = static_cast<char *> (allocate_memory (type.length () + 1))))
          return AMD_DBGAPI_STATUS_ERROR_CLIENT_CALLBACK;

        strcpy (retval, type.c_str ());
        *static_cast<decltype (retval) *> (value) = retval;
      }
      break;

    case AMD_DBGAPI_REGISTER_INFO_SIZE:
      {
        amd_dbgapi_size_t size;

        if (value_size != sizeof (size))
          return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT_SIZE;

        size = wave->register_offset_and_size (regnum).second;
        if (!size)
          return AMD_DBGAPI_STATUS_ERROR_INVALID_REGISTER_ID;

        *static_cast<decltype (size) *> (value) = size;
      }
      break;

    default:
      return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;
    }

  return AMD_DBGAPI_STATUS_SUCCESS;
  CATCH;
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_wave_register_list (amd_dbgapi_process_id_t process_id,
                               amd_dbgapi_wave_id_t wave_id,
                               size_t *register_count,
                               amd_dbgapi_register_id_t **registers)
{
  TRY;
  TRACE (process_id, wave_id);

  if (!amd::dbgapi::is_initialized)
    return AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED;

  process_t *process = process_t::find (process_id);

  if (!process)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_PROCESS_ID;

  wave_t *wave = process->find (wave_id);

  if (!wave)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_WAVE_ID;

  size_t count;
  amd_dbgapi_register_id_t *retval;

  /* Get the list of registers supported by this architecture.  */
  amd_dbgapi_status_t status = amd_dbgapi_architecture_register_list_1 (
      wave->architecture (), &count, &retval);
  if (status != AMD_DBGAPI_STATUS_SUCCESS)
    return status;

  /* Prune the registers that aren't available in the wave.  We re-use the
     list returned by the architecture, and may waste some memory since the
     wave list may be shorter, but this more efficient than allocating
     another buffer */
  size_t cur_pos = 0;
  for (size_t i = 0; i < count; ++i)
    if (wave->register_available (
            static_cast<amdgpu_regnum_t> (retval[i].handle)))
      retval[cur_pos++] = retval[i];

  *register_count = cur_pos;
  *registers = retval;

  return AMD_DBGAPI_STATUS_SUCCESS;
  CATCH;
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_prefetch_register (amd_dbgapi_process_id_t process_id,
                              amd_dbgapi_wave_id_t wave_id,
                              amd_dbgapi_register_id_t register_id,
                              amd_dbgapi_size_t register_count)
{
  TRY;
  TRACE (process_id, wave_id, register_id, register_count);

  return AMD_DBGAPI_STATUS_SUCCESS;
  CATCH;
}
