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

#include "memory.h"
#include "architecture.h"
#include "debug.h"
#include "initialization.h"
#include "logging.h"
#include "process.h"
#include "queue.h"
#include "utils.h"
#include "wave.h"

#include <cstring>
#include <optional>

namespace amd::dbgapi
{

amd_dbgapi_status_t
address_class_t::get_info (amd_dbgapi_address_class_info_t query,
                           size_t value_size, void *value) const
{
  switch (query)
    {
    case AMD_DBGAPI_ADDRESS_CLASS_INFO_ARCHITECTURE:
      return utils::get_info (value_size, value, architecture ().id ());

    case AMD_DBGAPI_ADDRESS_CLASS_INFO_NAME:
      return utils::get_info (value_size, value, name ());

    case AMD_DBGAPI_ADDRESS_CLASS_INFO_ADDRESS_SPACE:
      return utils::get_info (value_size, value, m_address_space.id ());

    case AMD_DBGAPI_ADDRESS_CLASS_INFO_DWARF:
      return utils::get_info (value_size, value, dwarf_value ());
    }
  return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;
}

amd_dbgapi_status_t
address_space_t::get_info (amd_dbgapi_address_space_info_t query,
                           size_t value_size, void *value) const
{
  switch (query)
    {
    case AMD_DBGAPI_ADDRESS_SPACE_INFO_ARCHITECTURE:
      return utils::get_info (value_size, value, architecture ().id ());

    case AMD_DBGAPI_ADDRESS_SPACE_INFO_NAME:
      return utils::get_info (value_size, value, name ());

    case AMD_DBGAPI_ADDRESS_SPACE_INFO_ADDRESS_SIZE:
      return utils::get_info (value_size, value, m_address_size);

    case AMD_DBGAPI_ADDRESS_SPACE_INFO_NULL_ADDRESS:
      return utils::get_info (value_size, value, m_null_address);

    case AMD_DBGAPI_ADDRESS_SPACE_INFO_ACCESS:
      return utils::get_info (value_size, value, m_access);

    case AMD_DBGAPI_ADDRESS_SPACE_INFO_DWARF:
      return utils::get_info (value_size, value, dwarf_value ());
    }
  return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;
}

decltype (memory_cache_t::m_next_id) memory_cache_t::m_next_id;

void
memory_cache_t::relocate (std::optional<amd_dbgapi_global_address_t> address)
{
  if (address == m_address)
    return;

  dbgapi_log (
    AMD_DBGAPI_LOG_LEVEL_VERBOSE, "relocated cache_%ld %s -> %s", id (),
    m_address
      ? string_printf ("[%#lx..%#lx[", *m_address, *m_address + size ())
          .c_str ()
      : "inaccessible",
    address
      ? string_printf ("[%#lx..%#lx[", *address, *address + size ()).c_str ()
      : "inaccessible");

  m_address = address;
}

void
memory_cache_t::reset (amd_dbgapi_global_address_t address,
                       amd_dbgapi_size_t cache_size)
{
  dbgapi_assert (!is_dirty () && "cannot reset a dirty cache");

  size_t prev_size = size ();
  m_address = address;
  m_cached_bytes.resize (cache_size);

  /* Reload the cache from memory.  */
  if (m_process.read_global_memory (*m_address, &m_cached_bytes[0],
                                    m_cached_bytes.size ())
      != AMD_DBGAPI_STATUS_SUCCESS)
    error ("Could not reload the hwregs cache");

  if (cache_size != 0)
    dbgapi_log (AMD_DBGAPI_LOG_LEVEL_VERBOSE, "%s cache_%ld [%#lx..%#lx[",
                prev_size == 0 ? "loaded" : "reloaded", id (), *m_address,
                *m_address + size ());
  else if (prev_size != 0)
    dbgapi_log (AMD_DBGAPI_LOG_LEVEL_VERBOSE, "cleared cache_%ld", id ());
}

void
memory_cache_t::flush ()
{
  dbgapi_assert (m_address && "cache is not accessible");

  if (is_dirty () && policy () == policy_t::write_back)
    {
      /* Write back the cache in memory.  */
      if (m_process.write_global_memory (*m_address, &m_cached_bytes[0],
                                         size ())
          != AMD_DBGAPI_STATUS_SUCCESS)
        error ("Could not write the hwregs cache back to memory");

      dbgapi_log (AMD_DBGAPI_LOG_LEVEL_VERBOSE,
                  "flushed cache_%ld [%#lx..%#lx[", id (), *m_address,
                  *m_address + size ());
    }

  m_dirty = false;
}

bool
memory_cache_t::contains (amd_dbgapi_global_address_t address,
                          amd_dbgapi_size_t value_size) const
{
  dbgapi_assert (m_address && "cache is not accessible");

  bool start_in_range
    = address >= m_address && address < (*m_address + size ());
  bool end_in_range = (address + value_size) > *m_address
                      && (address + value_size) <= (*m_address + size ());

  dbgapi_assert (start_in_range == end_in_range
                 && ((address >= *m_address)
                     || (address + value_size) <= (*m_address + size ()))
                 && "cannot be partially contained");

  return start_in_range && end_in_range;
}

amd_dbgapi_status_t
memory_cache_t::read (amd_dbgapi_global_address_t from, void *value,
                      size_t value_size) const
{
  dbgapi_assert (contains (from, value_size) && "invalid access");

  if (policy () == policy_t::uncached)
    return m_process.read_global_memory (from, value, value_size);

  memcpy (value, &m_cached_bytes[0] + from - *m_address, value_size);

  return AMD_DBGAPI_STATUS_SUCCESS;
}

amd_dbgapi_status_t
memory_cache_t::write (amd_dbgapi_global_address_t to, const void *value,
                       size_t value_size)
{
  dbgapi_assert (contains (to, value_size) && "invalid access");

  if (policy () != policy_t::write_back)
    {
      if (amd_dbgapi_status_t status
          = m_process.write_global_memory (to, value, value_size);
          status != AMD_DBGAPI_STATUS_SUCCESS)
        return status;
    }

  if (policy () != policy_t::uncached)
    {
      memcpy (&m_cached_bytes[0] + to - *m_address, value, value_size);
      m_dirty = true;
    }

  return AMD_DBGAPI_STATUS_SUCCESS;
}

} /* namespace amd::dbgapi */

using namespace amd::dbgapi;

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_address_class_get_info (
  amd_dbgapi_address_class_id_t address_class_id,
  amd_dbgapi_address_class_info_t query, size_t value_size, void *value)
{
  TRACE_BEGIN (param_in (address_class_id), param_in (query),
               param_in (value_size), param_in (value));
  TRY;

  if (!detail::is_initialized)
    return AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED;

  const address_class_t *address_class = find (address_class_id);

  if (!address_class)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ADDRESS_CLASS_ID;

  if (!value)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;

  return address_class->get_info (query, value_size, value);

  CATCH;
  TRACE_END (make_query_ref (query, param_out (value)));
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_architecture_address_class_list (
  amd_dbgapi_architecture_id_t architecture_id, size_t *address_class_count,
  amd_dbgapi_address_class_id_t **address_classes)
{
  TRACE_BEGIN (param_in (architecture_id), param_in (address_class_count),
               param_in (address_classes));
  TRY;

  if (!detail::is_initialized)
    return AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED;

  if (!address_class_count || !address_classes)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;

  const architecture_t *architecture = architecture_t::find (architecture_id);

  if (!architecture)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARCHITECTURE_ID;

  size_t count = architecture->count<address_class_t> ();

  amd_dbgapi_address_class_id_t *class_ids
    = static_cast<amd_dbgapi_address_class_id_t *> (
      allocate_memory (count * sizeof (amd_dbgapi_address_class_id_t)));

  if (count && !class_ids)
    return AMD_DBGAPI_STATUS_ERROR_CLIENT_CALLBACK;

  *address_class_count = count;
  *address_classes = class_ids;

  for (auto &&address_class : architecture->range<address_class_t> ())
    *class_ids++ = address_class.id ();

  return AMD_DBGAPI_STATUS_SUCCESS;

  CATCH;
  TRACE_END (
    make_ref (param_out (address_class_count)),
    make_ref (make_ref (param_out (address_classes)), *address_class_count));
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_dwarf_address_class_to_address_class (
  amd_dbgapi_architecture_id_t architecture_id, uint64_t dwarf_address_class,
  amd_dbgapi_address_class_id_t *address_class_id)
{
  TRACE_BEGIN (param_in (architecture_id), param_in (dwarf_address_class),
               param_in (address_class_id));
  TRY;

  if (!detail::is_initialized)
    return AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED;

  const architecture_t *architecture = architecture_t::find (architecture_id);

  if (!architecture)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARCHITECTURE_ID;

  if (!address_class_id)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;

  const address_class_t *address_class = architecture->find_if (
    [=] (const address_class_t &ac)
    { return ac.dwarf_value () == dwarf_address_class; });

  if (!address_class)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT_COMPATIBILITY;

  *address_class_id = address_class->id ();

  return AMD_DBGAPI_STATUS_SUCCESS;

  CATCH;
  TRACE_END (make_ref (param_out (address_class_id)));
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_address_space_get_info (
  amd_dbgapi_address_space_id_t address_space_id,
  amd_dbgapi_address_space_info_t query, size_t value_size, void *value)
{
  TRACE_BEGIN (param_in (address_space_id), param_in (query),
               param_in (value_size), param_in (value));
  TRY;

  if (!detail::is_initialized)
    return AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED;

  const address_space_t *address_space = find (address_space_id);

  if (!address_space)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ADDRESS_SPACE_ID;

  if (!value)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;

  return address_space->get_info (query, value_size, value);

  CATCH;
  TRACE_END (make_query_ref (query, param_out (value)));
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_architecture_address_space_list (
  amd_dbgapi_architecture_id_t architecture_id, size_t *address_space_count,
  amd_dbgapi_address_space_id_t **address_spaces)
{
  TRACE_BEGIN (param_in (architecture_id), param_in (address_space_count),
               param_in (address_spaces));
  TRY;

  if (!detail::is_initialized)
    return AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED;

  if (!address_space_count || !address_spaces)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;

  const architecture_t *architecture = architecture_t::find (architecture_id);

  if (!architecture)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARCHITECTURE_ID;

  size_t count = architecture->count<address_space_t> ();

  amd_dbgapi_address_space_id_t *space_ids
    = static_cast<amd_dbgapi_address_space_id_t *> (
      allocate_memory (count * sizeof (amd_dbgapi_address_space_id_t)));

  if (count && !space_ids)
    return AMD_DBGAPI_STATUS_ERROR_CLIENT_CALLBACK;

  *address_space_count = count;
  *address_spaces = space_ids;

  for (auto &&address_space : architecture->range<address_space_t> ())
    *space_ids++ = address_space.id ();

  return AMD_DBGAPI_STATUS_SUCCESS;

  CATCH;
  TRACE_END (
    make_ref (param_out (address_space_count)),
    make_ref (make_ref (param_out (address_spaces)), *address_space_count));
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_dwarf_address_space_to_address_space (
  amd_dbgapi_architecture_id_t architecture_id, uint64_t dwarf_address_space,
  amd_dbgapi_address_space_id_t *address_space_id)
{
  TRACE_BEGIN (param_in (architecture_id), param_in (dwarf_address_space),
               param_in (address_space_id));
  TRY;

  if (!detail::is_initialized)
    return AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED;

  const architecture_t *architecture = architecture_t::find (architecture_id);

  if (!architecture)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARCHITECTURE_ID;

  if (!address_space_id)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;

  const address_space_t *address_space = architecture->find_if (
    [=] (const address_space_t &ac)
    { return ac.dwarf_value () == dwarf_address_space; });

  if (!address_space)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT_COMPATIBILITY;

  *address_space_id = address_space->id ();

  return AMD_DBGAPI_STATUS_SUCCESS;

  CATCH;
  TRACE_END (make_ref (param_out (address_space_id)));
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_address_spaces_may_alias (
  amd_dbgapi_address_space_id_t address_space_id1,
  amd_dbgapi_address_space_id_t address_space_id2,
  amd_dbgapi_address_space_alias_t *address_space_alias)
{
  TRACE_BEGIN (param_in (address_space_id1), param_in (address_space_id2),
               param_in (address_space_alias));
  TRY;

  if (!detail::is_initialized)
    return AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED;

  const address_space_t *address_space1 = find (address_space_id1);
  const address_space_t *address_space2 = find (address_space_id2);

  if (!address_space1 || !address_space2)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ADDRESS_SPACE_ID;

  if (address_space1->architecture () != address_space2->architecture ())
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT_COMPATIBILITY;

  if (!address_space_alias)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;

  *address_space_alias
    = address_space1->architecture ().address_spaces_may_alias (
        *address_space1, *address_space2)
        ? AMD_DBGAPI_ADDRESS_SPACE_ALIAS_MAY
        : AMD_DBGAPI_ADDRESS_SPACE_ALIAS_NONE;

  return AMD_DBGAPI_STATUS_SUCCESS;

  CATCH;
  TRACE_END (make_ref (param_out (address_space_alias)));
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_convert_address_space (
  amd_dbgapi_wave_id_t wave_id, amd_dbgapi_lane_id_t lane_id,
  amd_dbgapi_address_space_id_t source_address_space_id,
  amd_dbgapi_segment_address_t source_segment_address,
  amd_dbgapi_address_space_id_t destination_address_space_id,
  amd_dbgapi_segment_address_t *destination_segment_address)
{
  TRACE_BEGIN (
    param_in (wave_id), param_in (lane_id), param_in (source_address_space_id),
    param_in (source_segment_address), param_in (destination_address_space_id),
    param_in (destination_segment_address));
  TRY;

  if (!detail::is_initialized)
    return AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED;

  wave_t *wave = find (wave_id);

  if (!wave)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_WAVE_ID;

  const address_space_t *source_address_space = find (source_address_space_id);
  const address_space_t *destination_address_space
    = find (destination_address_space_id);

  if (!source_address_space || !destination_address_space)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ADDRESS_SPACE_ID;

  const architecture_t &architecture = wave->architecture ();
  if (source_address_space->architecture () != architecture
      || destination_address_space->architecture () != architecture)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT_COMPATIBILITY;

  if (!destination_segment_address)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;

  return architecture.convert_address_space (
    *wave, lane_id, *source_address_space, *destination_address_space,
    source_segment_address, destination_segment_address);

  CATCH;
  TRACE_END (make_ref (param_out (destination_segment_address)));
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_address_is_in_address_class (
  amd_dbgapi_wave_id_t wave_id, amd_dbgapi_lane_id_t lane_id,
  amd_dbgapi_address_space_id_t address_space_id,
  amd_dbgapi_segment_address_t segment_address,
  amd_dbgapi_address_class_id_t address_class_id,
  amd_dbgapi_address_class_state_t *address_class_state)
{
  TRACE_BEGIN (param_in (wave_id), param_in (lane_id),
               param_in (address_space_id), param_in (segment_address),
               param_in (address_class_id), param_in (address_class_state));
  TRY;

  if (!detail::is_initialized)
    return AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED;

  wave_t *wave = find (wave_id);

  if (!wave)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_WAVE_ID;

  const address_space_t *address_space = find (address_space_id);

  if (!address_space)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ADDRESS_SPACE_ID;

  const address_class_t *address_class = find (address_class_id);

  if (!address_class)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ADDRESS_CLASS_ID;

  const architecture_t &architecture = wave->architecture ();
  if (address_space->architecture () != architecture
      || address_class->architecture () != architecture)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT_COMPATIBILITY;

  if (!address_class_state)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;

  *address_class_state
    = architecture.address_is_in_address_class (
        *wave, lane_id, *address_space, segment_address, *address_class)
        ? AMD_DBGAPI_ADDRESS_CLASS_STATE_MEMBER
        : AMD_DBGAPI_ADDRESS_CLASS_STATE_NOT_MEMBER;

  return AMD_DBGAPI_STATUS_SUCCESS;

  CATCH;
  TRACE_END (make_ref (param_out (address_class_state)));
}
amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_read_memory (amd_dbgapi_process_id_t process_id,
                        amd_dbgapi_wave_id_t wave_id,
                        amd_dbgapi_lane_id_t lane_id,
                        amd_dbgapi_address_space_id_t address_space_id,
                        amd_dbgapi_segment_address_t segment_address,
                        amd_dbgapi_size_t *value_size, void *value)
{
  TRACE_BEGIN (param_in (process_id), param_in (wave_id), param_in (lane_id),
               param_in (address_space_id),
               make_hex (param_in (segment_address)),
               make_ref (param_in (value_size)), param_in (value));
  TRY;

  if (!detail::is_initialized)
    return AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED;

  process_t *process = process_t::find (process_id);

  if (!process)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_PROCESS_ID;

  if (!value || !value_size)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;

  if (wave_id == AMD_DBGAPI_WAVE_NONE
      && address_space_id == AMD_DBGAPI_ADDRESS_SPACE_GLOBAL)
    return process->read_global_memory_partial (segment_address, value,
                                                value_size);

  wave_t *wave = find (wave_id);

  if (!wave)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_WAVE_ID;

  if (wave->state () != AMD_DBGAPI_WAVE_STATE_STOP)
    return AMD_DBGAPI_STATUS_ERROR_WAVE_NOT_STOPPED;

  if (lane_id >= wave->lane_count () && lane_id != AMD_DBGAPI_LANE_NONE)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_LANE_ID;

  const address_space_t *address_space = find (address_space_id);

  if (!address_space)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ADDRESS_SPACE_ID;

  if (wave->process () != *process
      || address_space->architecture () != wave->architecture ())
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT_COMPATIBILITY;

  wave->architecture ().lower_address_space (*wave, &lane_id, *address_space,
                                             &address_space, segment_address,
                                             &segment_address);

  std::optional<scoped_queue_suspend_t> suspend;
  if (address_space->kind () == address_space_t::local)
    {
      suspend.emplace (wave->queue (), "read local memory");

      /* Look for the wave_id again, the wave may have exited.  */
      if (!(wave = process->find (wave_id)))
        return AMD_DBGAPI_STATUS_ERROR_INVALID_WAVE_ID;
    }

  return wave->xfer_segment_memory (*address_space, lane_id, segment_address,
                                    value, nullptr, value_size);

  CATCH;
  TRACE_END (make_ref (param_out (value_size)),
             make_hex (make_ref (param_out (value), *value_size)));
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_write_memory (amd_dbgapi_process_id_t process_id,
                         amd_dbgapi_wave_id_t wave_id,
                         amd_dbgapi_lane_id_t lane_id,
                         amd_dbgapi_address_space_id_t address_space_id,
                         amd_dbgapi_segment_address_t segment_address,
                         amd_dbgapi_size_t *value_size, const void *value)
{
  TRACE_BEGIN (
    param_in (process_id), param_in (wave_id), param_in (lane_id),
    param_in (address_space_id), make_hex (param_in (segment_address)),
    make_ref (param_in (value_size)),
    make_hex (make_ref (param_in (value), value_size ? *value_size : 0)));
  TRY;

  if (!detail::is_initialized)
    return AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED;

  process_t *process = process_t::find (process_id);

  if (!process)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_PROCESS_ID;

  if (!value || !value_size)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;

  if (wave_id == AMD_DBGAPI_WAVE_NONE
      && address_space_id == AMD_DBGAPI_ADDRESS_SPACE_GLOBAL)
    return process->write_global_memory_partial (segment_address, value,
                                                 value_size);

  wave_t *wave = find (wave_id);

  if (!wave)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_WAVE_ID;

  if (wave->state () != AMD_DBGAPI_WAVE_STATE_STOP)
    return AMD_DBGAPI_STATUS_ERROR_WAVE_NOT_STOPPED;

  if (lane_id >= wave->lane_count () && lane_id != AMD_DBGAPI_LANE_NONE)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_LANE_ID;

  const address_space_t *address_space = find (address_space_id);

  if (!address_space)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ADDRESS_SPACE_ID;

  if (wave->process () != *process
      || address_space->architecture () != wave->architecture ())
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT_COMPATIBILITY;

  wave->architecture ().lower_address_space (*wave, &lane_id, *address_space,
                                             &address_space, segment_address,
                                             &segment_address);

  std::optional<scoped_queue_suspend_t> suspend;
  if (address_space->kind () == address_space_t::local)
    {
      /* FIXME: How can we optimize this?  */
      suspend.emplace (wave->queue (), "write local memory");

      /* Look for the wave_id again, the wave may have exited.  */
      if (!(wave = process->find (wave_id)))
        return AMD_DBGAPI_STATUS_ERROR_INVALID_WAVE_ID;
    }

  return wave->xfer_segment_memory (*address_space, lane_id, segment_address,
                                    nullptr, value, value_size);
  CATCH;
  TRACE_END (make_ref (param_out (value_size)));
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_set_memory_precision (
  amd_dbgapi_process_id_t process_id,
  amd_dbgapi_memory_precision_t memory_precision)
{
  TRACE_BEGIN (param_in (process_id), param_in (memory_precision));
  TRY;

  if (!detail::is_initialized)
    return AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED;

  process_t *process = process_t::find (process_id);

  if (!process)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_PROCESS_ID;

  if (memory_precision != AMD_DBGAPI_MEMORY_PRECISION_NONE
      && memory_precision != AMD_DBGAPI_MEMORY_PRECISION_PRECISE)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;

  return process->set_precise_memory (memory_precision
                                      == AMD_DBGAPI_MEMORY_PRECISION_PRECISE);
  CATCH;
  TRACE_END ();
}
