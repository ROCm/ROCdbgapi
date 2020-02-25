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

  if (count && !classes)
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
  amdgpu_regnum_t i;

  if (!register_count || !registers)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;

  constexpr size_t max_reg_count
      = AMDGPU_RAW_REGS_COUNT + AMDGPU_PSEUDO_REGS_COUNT;

  auto *retval = static_cast<amd_dbgapi_register_id_t *> (
      allocate_memory (max_reg_count * sizeof (amd_dbgapi_register_id_t)));

  if (!retval)
    return AMD_DBGAPI_STATUS_ERROR_CLIENT_CALLBACK;

  size_t cur_pos = 0;

  if (architecture.has_wave32_vgprs ())
    for (i = amdgpu_regnum_t::FIRST_VGPR_32;
         i <= amdgpu_regnum_t::LAST_VGPR_32; ++i)
      retval[cur_pos++].handle
          = static_cast<decltype (amd_dbgapi_register_id_t::handle)> (i);

  if (architecture.has_wave64_vgprs ())
    for (i = amdgpu_regnum_t::FIRST_VGPR_64;
         i <= amdgpu_regnum_t::LAST_VGPR_64; ++i)
      retval[cur_pos++].handle
          = static_cast<decltype (amd_dbgapi_register_id_t::handle)> (i);

  if (architecture.has_acc_vgprs ())
    for (i = amdgpu_regnum_t::FIRST_ACCVGPR_64;
         i <= amdgpu_regnum_t::LAST_ACCVGPR_64; ++i)
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

  retval[cur_pos++].handle
      = static_cast<decltype (amd_dbgapi_register_id_t::handle)> (
          amdgpu_regnum_t::PC);

  if (architecture.has_wave32_vgprs ())
    {
      retval[cur_pos++].handle
          = static_cast<decltype (amd_dbgapi_register_id_t::handle)> (
              amdgpu_regnum_t::EXEC_32);
      retval[cur_pos++].handle
          = static_cast<decltype (amd_dbgapi_register_id_t::handle)> (
              amdgpu_regnum_t::XNACK_MASK_32);
      retval[cur_pos++].handle
          = static_cast<decltype (amd_dbgapi_register_id_t::handle)> (
              amdgpu_regnum_t::VCC_32);
    }

  if (architecture.has_wave64_vgprs ())
    {
      retval[cur_pos++].handle
          = static_cast<decltype (amd_dbgapi_register_id_t::handle)> (
              amdgpu_regnum_t::EXEC_64);
      retval[cur_pos++].handle
          = static_cast<decltype (amd_dbgapi_register_id_t::handle)> (
              amdgpu_regnum_t::XNACK_MASK_64);
      retval[cur_pos++].handle
          = static_cast<decltype (amd_dbgapi_register_id_t::handle)> (
              amdgpu_regnum_t::VCC_64);
    }

  dbgapi_assert (cur_pos <= max_reg_count && "register_list overrun");

  *register_count = cur_pos;
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

  if ((architecture->has_wave32_vgprs ()
       && regnum >= amdgpu_regnum_t::FIRST_VGPR_32
       && regnum <= amdgpu_regnum_t::LAST_VGPR_32)
      || (architecture->has_wave64_vgprs ()
          && regnum >= amdgpu_regnum_t::FIRST_VGPR_64
          && regnum <= amdgpu_regnum_t::LAST_VGPR_64)
      || (architecture->has_acc_vgprs ()
          && regnum >= amdgpu_regnum_t::FIRST_ACCVGPR_64
          && regnum <= amdgpu_regnum_t::LAST_ACCVGPR_64))
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
  else if (regnum == amdgpu_regnum_t::PC
           || (architecture->has_wave32_vgprs ()
               && (regnum == amdgpu_regnum_t::EXEC_32
                   || regnum == amdgpu_regnum_t::VCC_32))
           || (architecture->has_wave64_vgprs ()
               && (regnum == amdgpu_regnum_t::EXEC_64
                   || regnum == amdgpu_regnum_t::VCC_64)))
    {
      is_in_register_class = (regclass == REGISTER_CLASS_GENERAL);
    }
  else if ((regnum >= amdgpu_regnum_t::FIRST_HWREG
            && regnum <= amdgpu_regnum_t::LAST_HWREG)
           || (regnum >= amdgpu_regnum_t::FIRST_TTMP
               && regnum <= amdgpu_regnum_t::LAST_TTMP)
           || regnum == amdgpu_regnum_t::FLAT_SCRATCH
           || (architecture->has_wave32_vgprs ()
               && regnum == amdgpu_regnum_t::XNACK_MASK_32)
           || (architecture->has_wave64_vgprs ()
               && regnum == amdgpu_regnum_t::XNACK_MASK_64))
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

  /* See https://llvm.org/docs/AMDGPUUsage.html#register-mapping.  */
  if (dwarf_register == 1)
    {
      regnum = amdgpu_regnum_t::EXEC_32;
    }
  else if (dwarf_register == 17)
    {
      regnum = amdgpu_regnum_t::EXEC_64;
    }
  else if (dwarf_register == 16)
    {
      regnum = amdgpu_regnum_t::PC;
    }
  else if (dwarf_register >= 32 && dwarf_register <= 95)
    {
      /* Scalar registers 0-63.  */
      regnum = amdgpu_regnum_t::FIRST_SGPR + (dwarf_register - 32);
    }
  else if (dwarf_register >= 1088 && dwarf_register <= 1129)
    {
      /* Scalar registers 64-105.  */
      regnum = amdgpu_regnum_t::FIRST_SGPR + (dwarf_register - 1024);
    }
  else if (dwarf_register >= 1536 && dwarf_register <= 1791)
    {
      /* Vector registers 0-255 (wave32).  */
      regnum = amdgpu_regnum_t::FIRST_VGPR_32 + (dwarf_register - 1536);
    }
  else if (dwarf_register >= 2048 && dwarf_register <= 2303)
    {
      /* Accumulation Vector registers 0-255 (wave32).  */
      regnum = amdgpu_regnum_t::FIRST_ACCVGPR_32 + (dwarf_register - 2048);
    }
  else if (dwarf_register >= 2560 && dwarf_register <= 2815)
    {
      /* Vector registers 0-255 (wave64).  */
      regnum = amdgpu_regnum_t::FIRST_VGPR_64 + (dwarf_register - 2560);
    }
  else if (dwarf_register >= 3072 && dwarf_register <= 3327)
    {
      /* Accumulation Vector registers 0-255 (wave64).  */
      regnum = amdgpu_regnum_t::FIRST_ACCVGPR_64 + (dwarf_register - 3072);
    }
  else
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;

  *register_id = amd_dbgapi_register_id_t{
    static_cast<std::underlying_type<decltype (regnum)>::type> (regnum)
  };

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

  {
    utils::optional<scoped_queue_suspend_t> suspend;

    if (!wave->is_register_cached (regnum))
      {
        suspend.emplace (wave->queue ());

        /* Look for the wave_id again, the wave may have exited.  */
        if (!(wave = process->find (wave_id)))
          return AMD_DBGAPI_STATUS_ERROR_INVALID_WAVE_ID;
      }

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

        retval = static_cast<char *> (allocate_memory (name.length () + 1));
        if (!retval)
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

        retval = static_cast<char *> (allocate_memory (type.length () + 1));
        if (!retval)
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

        size = wave->register_offset_and_size (regnum, false).second;
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
