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

#include "logging.h"
#include "memory.h"
#include "process.h"
#include "utils.h"
#include "wave.h"

namespace amd
{
namespace dbgapi
{

} /* namespace dbgapi */
} /* namespace amd */

using namespace amd::dbgapi;

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_architecture_address_class_get_info (
    amd_dbgapi_architecture_id_t architecture_id,
    amd_dbgapi_address_class_id_t address_class_id,
    amd_dbgapi_address_class_info_t query, size_t value_size, void *value)
{
  TRY;
  TRACE (architecture_id, address_class_id, query);

  return AMD_DBGAPI_STATUS_ERROR;
  CATCH;
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_architecture_address_class_list (
    amd_dbgapi_architecture_id_t architecture_id, size_t *address_class_count,
    amd_dbgapi_address_class_id_t **address_classes)
{
  TRY;
  TRACE (architecture_id);

  return AMD_DBGAPI_STATUS_ERROR;
  CATCH;
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_dwarf_address_class_to_address_class (
    amd_dbgapi_architecture_id_t architecture_id, uint64_t dwarf_address_class,
    amd_dbgapi_address_class_id_t *address_class_id)
{
  TRY;
  TRACE (architecture_id, dwarf_address_class);

  return AMD_DBGAPI_STATUS_ERROR;
  CATCH;
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_address_space_get_info (
    amd_dbgapi_architecture_id_t architecture_id,
    amd_dbgapi_address_space_id_t address_space_id,
    amd_dbgapi_address_space_info_t query, size_t value_size, void *value)
{
  TRY;
  TRACE (architecture_id, address_space_id, query);

  return AMD_DBGAPI_STATUS_ERROR;
  CATCH;
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_architecture_address_space_list (
    amd_dbgapi_architecture_id_t architecture_id, size_t *address_space_count,
    amd_dbgapi_address_space_id_t **address_spaces)
{
  TRY;
  TRACE (architecture_id);

  return AMD_DBGAPI_STATUS_ERROR;
  CATCH;
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_dwarf_address_space_to_address_space (
    amd_dbgapi_architecture_id_t architecture_id, uint64_t dwarf_address_space,
    amd_dbgapi_address_space_id_t *address_space_id)
{
  TRY;
  TRACE (architecture_id, dwarf_address_space);

  return AMD_DBGAPI_STATUS_ERROR;
  CATCH;
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_address_spaces_may_alias (
    amd_dbgapi_architecture_id_t architecture_id,
    amd_dbgapi_address_space_id_t address_space_id1,
    amd_dbgapi_address_space_id_t address_space_id2,
    amd_dbgapi_address_space_alias_t *address_space_alias)
{
  TRY;
  TRACE (architecture_id, address_space_id1, address_space_id2);

  return AMD_DBGAPI_STATUS_ERROR;
  CATCH;
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_convert_address_space (
    amd_dbgapi_process_id_t process_id, amd_dbgapi_wave_id_t wave_id,
    amd_dbgapi_lane_id_t lane_id,
    amd_dbgapi_address_space_id_t source_address_space_id,
    amd_dbgapi_segment_address_t source_segment_address,
    amd_dbgapi_address_space_id_t destination_address_space_id,
    amd_dbgapi_segment_address_t *destination_segment_address)
{
  TRY;
  TRACE (process_id, wave_id, lane_id, source_address_space_id,
         source_segment_address, destination_address_space_id);

  return AMD_DBGAPI_STATUS_ERROR;
  CATCH;
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_address_is_in_address_class (
    amd_dbgapi_process_id_t process_id, amd_dbgapi_wave_id_t wave_id,
    amd_dbgapi_lane_id_t lane_id,
    amd_dbgapi_address_space_id_t address_space_id,
    amd_dbgapi_segment_address_t segment_address,
    amd_dbgapi_address_class_id_t address_class_id,
    amd_dbgapi_address_class_state_t *address_class_state)
{
  TRY;
  TRACE (process_id, wave_id, lane_id, address_space_id, segment_address,
         address_class_id);

  return AMD_DBGAPI_STATUS_ERROR;
  CATCH;
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_read_memory (amd_dbgapi_process_id_t process_id,
                        amd_dbgapi_wave_id_t wave_id,
                        amd_dbgapi_lane_id_t lane_id,
                        amd_dbgapi_address_space_id_t address_space_id,
                        amd_dbgapi_segment_address_t segment_address,
                        amd_dbgapi_size_t *value_size, void *value)
{
  TRY;
  TRACE (process_id, wave_id, lane_id, address_space_id, segment_address,
         value_size, value);

  if (!amd::dbgapi::is_initialized)
    return AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED;

  if (!value || !value_size)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;

  process_t *process = process_t::find (process_id);

  if (!process)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_PROCESS_ID;

  wave_t *wave = process->find (wave_id);

  if (!wave)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_WAVE_ID;

  if (wave->state () != AMD_DBGAPI_WAVE_STATE_STOP)
    return AMD_DBGAPI_STATUS_ERROR_WAVE_NOT_STOPPED;

  /* FIXME: We need to call the proper function based on the address space.  */
  return process->read_global_memory_partial (segment_address, value,
                                              value_size);
  CATCH;
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_write_memory (amd_dbgapi_process_id_t process_id,
                         amd_dbgapi_wave_id_t wave_id,
                         amd_dbgapi_lane_id_t lane_id,
                         amd_dbgapi_address_space_id_t address_space_id,
                         amd_dbgapi_segment_address_t segment_address,
                         amd_dbgapi_size_t *value_size, const void *value)
{
  TRY;
  TRACE (process_id, wave_id, lane_id, address_space_id, segment_address,
         value_size, value);

  if (!amd::dbgapi::is_initialized)
    return AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED;

  if (!value || !value_size)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;

  process_t *process = process_t::find (process_id);

  if (!process)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_PROCESS_ID;

  wave_t *wave = process->find (wave_id);

  if (!wave)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_WAVE_ID;

  if (wave->state () != AMD_DBGAPI_WAVE_STATE_STOP)
    return AMD_DBGAPI_STATUS_ERROR_WAVE_NOT_STOPPED;

  /* FIXME: We need to call the proper function based on the address space.  */
  return process->write_global_memory_partial (segment_address, value,
                                               value_size);
  CATCH;
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_set_memory_precision (
    amd_dbgapi_process_id_t process_id, amd_dbgapi_agent_id_t agent_id,
    amd_dbgapi_memory_precision_t memory_precision)
{
  TRY;
  TRACE (process_id, agent_id, memory_precision);

  return AMD_DBGAPI_STATUS_ERROR;
  CATCH;
}
