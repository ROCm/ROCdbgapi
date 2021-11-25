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
#include "exception.h"
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

void
address_class_t::get_info (amd_dbgapi_address_class_info_t query,
                           size_t value_size, void *value) const
{
  switch (query)
    {
    case AMD_DBGAPI_ADDRESS_CLASS_INFO_NAME:
      utils::get_info (value_size, value, name ());
      return;

    case AMD_DBGAPI_ADDRESS_CLASS_INFO_ADDRESS_SPACE:
      utils::get_info (value_size, value, m_address_space.id ());
      return;

    case AMD_DBGAPI_ADDRESS_CLASS_INFO_DWARF:
      utils::get_info (value_size, value, dwarf_value ());
      return;
    }

  throw api_error_t (AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT);
}

address_space_t address_space_t::s_global{
  AMD_DBGAPI_ADDRESS_SPACE_GLOBAL,
  "global",
  address_space_t::kind_t::global,
  DW_ASPACE_none,
  64,
  0x0000000000000000,
  AMD_DBGAPI_ADDRESS_SPACE_ACCESS_ALL
};

void
address_space_t::get_info (amd_dbgapi_address_space_info_t query,
                           size_t value_size, void *value) const
{
  switch (query)
    {
    case AMD_DBGAPI_ADDRESS_SPACE_INFO_NAME:
      utils::get_info (value_size, value, name ());
      return;

    case AMD_DBGAPI_ADDRESS_SPACE_INFO_ADDRESS_SIZE:
      utils::get_info (value_size, value, m_address_size);
      return;

    case AMD_DBGAPI_ADDRESS_SPACE_INFO_NULL_ADDRESS:
      utils::get_info (value_size, value, m_null_address);
      return;

    case AMD_DBGAPI_ADDRESS_SPACE_INFO_ACCESS:
      utils::get_info (value_size, value, m_access);
      return;

    case AMD_DBGAPI_ADDRESS_SPACE_INFO_DWARF:
      utils::get_info (value_size, value, dwarf_value ());
      return;
    }

  throw api_error_t (AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT);
}

void
memory_cache_t::fetch_cache_line (cache_line_t &cache_line,
                                  amd_dbgapi_global_address_t address) const
{
  dbgapi_assert (!cache_line.m_dirty);

  size_t xfer_size = m_xfer_global_memory (address, &cache_line.m_data[0],
                                           nullptr, cache_line.m_data.size ());

  if (xfer_size != cache_line.m_data.size ())
    throw memory_access_error_t (address + cache_line_size);

  cache_line.m_dirty = false;
}

void
memory_cache_t::commit_cache_line (cache_line_t &cache_line,
                                   amd_dbgapi_global_address_t address) const
{
  if (!cache_line.m_dirty)
    return;

  size_t xfer_size = m_xfer_global_memory (
    address, nullptr, &cache_line.m_data[0], cache_line.m_data.size ());

  if (xfer_size != cache_line.m_data.size ())
    throw memory_access_error_t (address + xfer_size);

  cache_line.m_dirty = false;
}
void
memory_cache_t::allocate_0_cache_line (cache_line_t &cache_line) const
{
  dbgapi_assert (!cache_line.m_dirty);

  memset (&cache_line.m_data[0], '\0', cache_line.m_data.size ());

  cache_line.m_dirty = false;
}

bool
memory_cache_t::contains_all (amd_dbgapi_global_address_t address,
                              amd_dbgapi_size_t size) const
{
  dbgapi_assert (address < (address + size) && "invalid size");
  auto cache_line_begin = utils::align_down (address, cache_line_size);
  auto cache_line_end = utils::align_up (address + size, cache_line_size);

  for (auto cache_line_address = cache_line_begin;
       cache_line_address < cache_line_end;
       cache_line_address += cache_line_size)
    if (m_cache_line_map.find (cache_line_address) == m_cache_line_map.end ())
      return false;

  return true;
}

void
memory_cache_t::prefetch (amd_dbgapi_global_address_t address,
                          amd_dbgapi_size_t size)
{
  if (policy == policy_t::uncached || size == 0)
    return;

  dbgapi_assert (address < (address + size) && "invalid size");
  auto cache_line_begin = utils::align_down (address, cache_line_size);
  auto cache_line_end = utils::align_up (address + size, cache_line_size);

  auto staging_buffer
    = std::make_unique<std::byte[]> (cache_line_end - cache_line_begin);

  try
    {
      m_xfer_global_memory (cache_line_begin, &staging_buffer[0], nullptr,
                            cache_line_end - cache_line_begin);
    }
  catch (const memory_access_error_t &)
    {
      /* If a memory access error exception is raised while prefetching, simply
         drop the prefetch.  */
      return;
    }

  for (auto cache_line_address = cache_line_begin;
       cache_line_address < cache_line_end;
       cache_line_address += cache_line_size)
    {
      if (contains_all (cache_line_address, cache_line_size))
        /* There's already a cache line for that address that will have already
           been read, and possibly updated.  So leave cache line with its
           current contents.  */
        continue;

      auto [it, success] = m_cache_line_map.try_emplace (cache_line_address);
      dbgapi_assert (success && "failed to create memory cache");
      [] (auto &&...) {}(success); /* Silence an unused variable warning when
                                      building without assertions enabled.  */

      memcpy (&it->second.m_data[0],
              &staging_buffer[cache_line_address - cache_line_begin],
              cache_line_size);
    }
}

void
memory_cache_t::write_back (amd_dbgapi_global_address_t address,
                            amd_dbgapi_size_t size)
{
  if (policy != policy_t::write_back || size == 0)
    return;

  dbgapi_assert (address < (address + size) && "invalid size");
  auto first_line = utils::align_down (address, cache_line_size);
  auto last_line = utils::align_down (address + size - 1, cache_line_size);

  auto it = m_cache_line_map.lower_bound (first_line);
  auto limit = m_cache_line_map.upper_bound (last_line);

  std::unique_ptr<std::byte[]> staging_buffer;
  size_t staging_buffer_size = 0;

  while (it != limit)
    {
      auto &[cache_line_address, cache_line] = *it;
      size_t request_size = cache_line_size;

      /* Skip this line if it isn't dirty as we do not need to commit it to
         memory.  */
      if (!cache_line.m_dirty)
        {
          std::advance (it, 1);
          continue;
        }

      /* It is more efficient to do a single large memory access, so try to
         group as many contiguous cache lines as possible.  */
      auto next = std::next (it);
      while (next->second.m_dirty
             && next->first == (cache_line_address + request_size))
        {
          request_size += cache_line_size;
          std::advance (next, 1);
        }

      if (request_size > staging_buffer_size)
        {
          staging_buffer = std::make_unique<std::byte[]> (request_size);
          staging_buffer_size = request_size;
        }

      while (it != next)
        {
          memcpy (&staging_buffer[0] + it->first - cache_line_address,
                  &it->second.m_data[0], cache_line_size);
          it->second.m_dirty = false;
          std::advance (it, 1);
        }

      try
        {
          size_t xfer_size = m_xfer_global_memory (
            cache_line_address, nullptr, &staging_buffer[0], request_size);

          if (xfer_size != request_size)
            throw memory_access_error_t (cache_line_address + xfer_size);
        }
      catch (const process_exited_exception_t &)
        {
          /* The process has exited, simply discard the dirty cached bytes.  */
        }
    }
}

void
memory_cache_t::discard (amd_dbgapi_global_address_t address,
                         amd_dbgapi_size_t size)
{
  if (size == 0)
    return;

  dbgapi_assert (address < (address + size) && "invalid size");
  auto first_line = utils::align_down (address, cache_line_size);
  auto last_line = utils::align_down (address + size - 1, cache_line_size);

  auto it = m_cache_line_map.lower_bound (first_line);
  auto limit = m_cache_line_map.upper_bound (last_line);

  while (it != limit)
    {
      dbgapi_assert (!it->second.m_dirty && "discarding a dirty cache line");
      it = m_cache_line_map.erase (it);
    }
}

size_t
memory_cache_t::xfer_global_memory (amd_dbgapi_global_address_t address,
                                    void *read, const void *write, size_t size)
{
  if (size == 0)
    return 0;

  /* Clamp to the end of the global address space.  */
  if (address > (address + size))
    size = -address;

  auto first_line = utils::align_down (address, cache_line_size);
  auto last_line = utils::align_down (address + size - 1, cache_line_size);

  /* Iterators to the first cache line, and one past the last cache line.  */
  auto begin = m_cache_line_map.lower_bound (first_line);
  auto end = m_cache_line_map.upper_bound (last_line);

  /* If uncached or there are no cache lines affected by this access.  */
  if (policy == policy_t::uncached || begin == end)
    return m_xfer_global_memory (address, read, write, size);

  /* For cached accesses, handle one cache line at a time.  */
  if (first_line != last_line)
    {
      auto ptr = address;
      while (size > 0)
        {
          auto limit = utils::align_up (ptr + 1, cache_line_size);
          auto request_size = std::min (limit, ptr + size) - ptr;

          auto xfer_size = xfer_global_memory (ptr, read, write, request_size);

          ptr += xfer_size;
          size -= xfer_size;

          if (xfer_size != request_size)
            break;
        }
      return ptr - address;
    }

  dbgapi_assert (begin != m_cache_line_map.end ());
  auto &cache_line = begin->second;
  auto offset = address - first_line;

  if (read)
    {
      memcpy (read, &cache_line.m_data[0] + offset, size);
    }
  else
    {
      memcpy (&cache_line.m_data[0] + offset, write, size);

      if (policy != policy_t::write_back)
        return m_xfer_global_memory (address, nullptr, write, size);

      cache_line.m_dirty = true;
    }

  return size;
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
  TRY
  {
    if (!detail::is_initialized)
      THROW (AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED);

    const address_class_t *address_class = find (address_class_id);

    if (!address_class)
      THROW (AMD_DBGAPI_STATUS_ERROR_INVALID_ADDRESS_CLASS_ID);

    if (!value)
      THROW (AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT);

    address_class->get_info (query, value_size, value);

    return AMD_DBGAPI_STATUS_SUCCESS;
  }
  CATCH (AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED,
         AMD_DBGAPI_STATUS_ERROR_INVALID_ADDRESS_CLASS_ID,
         AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT,
         AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT_COMPATIBILITY,
         AMD_DBGAPI_STATUS_ERROR_CLIENT_CALLBACK);
  TRACE_END (make_query_ref (query, param_out (value)));
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_architecture_address_class_list (
  amd_dbgapi_architecture_id_t architecture_id, size_t *address_class_count,
  amd_dbgapi_address_class_id_t **address_classes)
{
  TRACE_BEGIN (param_in (architecture_id), param_in (address_class_count),
               param_in (address_classes));
  TRY
  {
    if (!detail::is_initialized)
      THROW (AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED);

    if (!address_class_count || !address_classes)
      THROW (AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT);

    const architecture_t *architecture
      = architecture_t::find (architecture_id);

    if (!architecture)
      THROW (AMD_DBGAPI_STATUS_ERROR_INVALID_ARCHITECTURE_ID);

    size_t count = architecture->count<address_class_t> ();
    auto class_ids = allocate_memory<amd_dbgapi_address_class_id_t[]> (
      count * sizeof (amd_dbgapi_address_class_id_t));

    size_t pos = 0;
    for (auto &&address_class : architecture->range<address_class_t> ())
      class_ids[pos++] = address_class.id ();

    *address_class_count = count;
    *address_classes = class_ids.release ();

    return AMD_DBGAPI_STATUS_SUCCESS;
  }
  CATCH (AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED,
         AMD_DBGAPI_STATUS_ERROR_INVALID_ARCHITECTURE_ID,
         AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT,
         AMD_DBGAPI_STATUS_ERROR_CLIENT_CALLBACK);
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
  TRY
  {
    if (!detail::is_initialized)
      THROW (AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED);

    const architecture_t *architecture
      = architecture_t::find (architecture_id);

    if (!architecture)
      THROW (AMD_DBGAPI_STATUS_ERROR_INVALID_ARCHITECTURE_ID);

    if (!address_class_id)
      THROW (AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT);

    const address_class_t *address_class = architecture->find_if (
      [=] (const address_class_t &ac)
      { return ac.dwarf_value () == dwarf_address_class; });

    if (!address_class)
      THROW (AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT_COMPATIBILITY);

    *address_class_id = address_class->id ();

    return AMD_DBGAPI_STATUS_SUCCESS;
  }
  CATCH (AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED,
         AMD_DBGAPI_STATUS_ERROR_INVALID_ARCHITECTURE_ID,
         AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT,
         AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT_COMPATIBILITY);
  TRACE_END (make_ref (param_out (address_class_id)));
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_address_space_get_info (
  amd_dbgapi_address_space_id_t address_space_id,
  amd_dbgapi_address_space_info_t query, size_t value_size, void *value)
{
  TRACE_BEGIN (param_in (address_space_id), param_in (query),
               param_in (value_size), param_in (value));
  TRY
  {
    if (!detail::is_initialized)
      THROW (AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED);

    const address_space_t *address_space = find (address_space_id);

    if (!address_space)
      THROW (AMD_DBGAPI_STATUS_ERROR_INVALID_ADDRESS_SPACE_ID);

    if (!value)
      THROW (AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT);

    address_space->get_info (query, value_size, value);

    return AMD_DBGAPI_STATUS_SUCCESS;
  }
  CATCH (AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED,
         AMD_DBGAPI_STATUS_ERROR_INVALID_ADDRESS_SPACE_ID,
         AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT,
         AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT_COMPATIBILITY,
         AMD_DBGAPI_STATUS_ERROR_CLIENT_CALLBACK);
  TRACE_END (make_query_ref (query, param_out (value)));
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_architecture_address_space_list (
  amd_dbgapi_architecture_id_t architecture_id, size_t *address_space_count,
  amd_dbgapi_address_space_id_t **address_spaces)
{
  TRACE_BEGIN (param_in (architecture_id), param_in (address_space_count),
               param_in (address_spaces));
  TRY
  {
    if (!detail::is_initialized)
      THROW (AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED);

    if (!address_space_count || !address_spaces)
      THROW (AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT);

    const architecture_t *architecture
      = architecture_t::find (architecture_id);

    if (!architecture)
      THROW (AMD_DBGAPI_STATUS_ERROR_INVALID_ARCHITECTURE_ID);

    size_t count = 1 + architecture->count<address_space_t> ();
    auto space_ids = allocate_memory<amd_dbgapi_address_space_id_t[]> (
      count * sizeof (amd_dbgapi_address_space_id_t));

    size_t pos = 0;
    space_ids[pos++] = AMD_DBGAPI_ADDRESS_SPACE_GLOBAL;
    for (auto &&address_space : architecture->range<address_space_t> ())
      space_ids[pos++] = address_space.id ();

    dbgapi_assert (pos == count);
    *address_space_count = count;
    *address_spaces = space_ids.release ();

    return AMD_DBGAPI_STATUS_SUCCESS;
  }
  CATCH (AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED,
         AMD_DBGAPI_STATUS_ERROR_INVALID_ARCHITECTURE_ID,
         AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT,
         AMD_DBGAPI_STATUS_ERROR_CLIENT_CALLBACK);
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
  TRY
  {
    if (!detail::is_initialized)
      THROW (AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED);

    const architecture_t *architecture
      = architecture_t::find (architecture_id);

    if (!architecture)
      THROW (AMD_DBGAPI_STATUS_ERROR_INVALID_ARCHITECTURE_ID);

    if (!address_space_id)
      THROW (AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT);

    const address_space_t *address_space = architecture->find_if (
      [=] (const address_space_t &ac)
      { return ac.dwarf_value () == dwarf_address_space; });

    if (!address_space)
      THROW (AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT_COMPATIBILITY);

    *address_space_id = address_space->id ();

    return AMD_DBGAPI_STATUS_SUCCESS;
  }
  CATCH (AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED,
         AMD_DBGAPI_STATUS_ERROR_INVALID_ARCHITECTURE_ID,
         AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT,
         AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT_COMPATIBILITY);
  TRACE_END (make_ref (param_out (address_space_id)));
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_convert_address_space (
  amd_dbgapi_wave_id_t wave_id, amd_dbgapi_lane_id_t lane_id,
  amd_dbgapi_address_space_id_t source_address_space_id,
  amd_dbgapi_segment_address_t source_segment_address,
  amd_dbgapi_address_space_id_t destination_address_space_id,
  amd_dbgapi_segment_address_t *destination_segment_address,
  amd_dbgapi_size_t *destination_contiguous_bytes)
{
  TRACE_BEGIN (
    param_in (wave_id), param_in (lane_id), param_in (source_address_space_id),
    param_in (source_segment_address), param_in (destination_address_space_id),
    param_in (destination_segment_address),
    param_in (destination_contiguous_bytes));
  TRY
  {
    if (!detail::is_initialized)
      THROW (AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED);

    if (!destination_segment_address || !destination_contiguous_bytes)
      THROW (AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT);

    /* Handle global->global conversions early since it does not require to
       pass in a wave_id.  */
    if (wave_id == AMD_DBGAPI_WAVE_NONE
        && source_address_space_id == AMD_DBGAPI_ADDRESS_SPACE_GLOBAL
        && destination_address_space_id == AMD_DBGAPI_ADDRESS_SPACE_GLOBAL)
      {
        const address_space_t *global_address_space
          = find (AMD_DBGAPI_ADDRESS_SPACE_GLOBAL);
        dbgapi_assert (global_address_space != nullptr);

        amd_dbgapi_segment_address_t global_address_mask
          = utils::bit_mask (0, global_address_space->address_size () - 1);

        *destination_segment_address = source_segment_address;
        *destination_contiguous_bytes
          = global_address_mask - source_segment_address + 1;

        return AMD_DBGAPI_STATUS_SUCCESS;
      }

    wave_t *wave = find (wave_id);

    if (!wave)
      THROW (AMD_DBGAPI_STATUS_ERROR_INVALID_WAVE_ID);

    const address_space_t *source_address_space
      = find (source_address_space_id);
    const address_space_t *destination_address_space
      = find (destination_address_space_id);

    if (!source_address_space || !destination_address_space)
      THROW (AMD_DBGAPI_STATUS_ERROR_INVALID_ADDRESS_SPACE_ID);

    const architecture_t &architecture = wave->architecture ();
    if (!architecture.is_address_space_supported (*destination_address_space)
        || !architecture.is_address_space_supported (*source_address_space))
      THROW (AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT_COMPATIBILITY);

    std::tie (*destination_segment_address, *destination_contiguous_bytes)
      = architecture.convert_address_space (
        *wave, lane_id, *source_address_space, *destination_address_space,
        source_segment_address);

    return AMD_DBGAPI_STATUS_SUCCESS;
  }
  CATCH (AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED,
         AMD_DBGAPI_STATUS_ERROR_INVALID_WAVE_ID,
         AMD_DBGAPI_STATUS_ERROR_INVALID_LANE_ID,
         AMD_DBGAPI_STATUS_ERROR_INVALID_ADDRESS_SPACE_ID,
         AMD_DBGAPI_STATUS_ERROR_INVALID_ADDRESS_SPACE_CONVERSION,
         AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT,
         AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT_COMPATIBILITY);
  TRACE_END (make_hex (make_ref (param_out (destination_segment_address))),
             make_ref (param_out (destination_contiguous_bytes)));
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
  TRY
  {
    if (!detail::is_initialized)
      THROW (AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED);

    wave_t *wave = find (wave_id);

    if (!wave)
      THROW (AMD_DBGAPI_STATUS_ERROR_INVALID_WAVE_ID);

    const address_space_t *address_space = find (address_space_id);

    if (!address_space)
      THROW (AMD_DBGAPI_STATUS_ERROR_INVALID_ADDRESS_SPACE_ID);

    const address_class_t *address_class = find (address_class_id);

    if (!address_class)
      THROW (AMD_DBGAPI_STATUS_ERROR_INVALID_ADDRESS_CLASS_ID);

    const architecture_t &architecture = wave->architecture ();
    if (!architecture.is_address_space_supported (*address_space)
        || !architecture.is_address_class_supported (*address_class))
      THROW (AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT_COMPATIBILITY);

    if (!address_class_state)
      THROW (AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT);

    *address_class_state
      = architecture.address_is_in_address_class (
          *wave, lane_id, *address_space, segment_address, *address_class)
          ? AMD_DBGAPI_ADDRESS_CLASS_STATE_MEMBER
          : AMD_DBGAPI_ADDRESS_CLASS_STATE_NOT_MEMBER;

    return AMD_DBGAPI_STATUS_SUCCESS;
  }
  CATCH (AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED,
         AMD_DBGAPI_STATUS_ERROR_INVALID_WAVE_ID,
         AMD_DBGAPI_STATUS_ERROR_INVALID_LANE_ID,
         AMD_DBGAPI_STATUS_ERROR_INVALID_ADDRESS_SPACE_ID,
         AMD_DBGAPI_STATUS_ERROR_INVALID_ADDRESS_CLASS_ID,
         AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT,
         AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT_COMPATIBILITY);
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
  TRY
  {
    if (!detail::is_initialized)
      THROW (AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED);

    process_t *process = process_t::find (process_id);

    if (!process)
      THROW (AMD_DBGAPI_STATUS_ERROR_INVALID_PROCESS_ID);

    if (!value || !value_size)
      THROW (AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT);

    if (wave_id == AMD_DBGAPI_WAVE_NONE
        && address_space_id == AMD_DBGAPI_ADDRESS_SPACE_GLOBAL)
      {
        *value_size = process->read_global_memory_partial (segment_address,
                                                           value, *value_size);
      }
    else
      {
        wave_t *wave = find (wave_id);

        if (!wave)
          THROW (AMD_DBGAPI_STATUS_ERROR_INVALID_WAVE_ID);

        if (wave->state () != AMD_DBGAPI_WAVE_STATE_STOP)
          THROW (AMD_DBGAPI_STATUS_ERROR_WAVE_NOT_STOPPED);

        if (lane_id >= wave->lane_count () && lane_id != AMD_DBGAPI_LANE_NONE)
          THROW (AMD_DBGAPI_STATUS_ERROR_INVALID_LANE_ID);

        const address_space_t *address_space = find (address_space_id);

        if (!address_space)
          THROW (AMD_DBGAPI_STATUS_ERROR_INVALID_ADDRESS_SPACE_ID);

        if (!wave->architecture ().is_address_space_supported (*address_space)
            || wave->process () != *process)
          THROW (AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT_COMPATIBILITY);

        auto [lowered_address_space, lowered_address]
          = wave->architecture ().lower_address_space (*wave, *address_space,
                                                       segment_address);

        *value_size = wave->xfer_segment_memory (lowered_address_space,
                                                 lane_id, lowered_address,
                                                 value, nullptr, *value_size);
      }

    return AMD_DBGAPI_STATUS_SUCCESS;
  }
  CATCH (AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED,
         AMD_DBGAPI_STATUS_ERROR_INVALID_PROCESS_ID,
         AMD_DBGAPI_STATUS_ERROR_INVALID_WAVE_ID,
         AMD_DBGAPI_STATUS_ERROR_INVALID_LANE_ID,
         AMD_DBGAPI_STATUS_ERROR_INVALID_ADDRESS_SPACE_ID,
         AMD_DBGAPI_STATUS_ERROR_WAVE_NOT_STOPPED,
         AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT,
         AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT_COMPATIBILITY,
         AMD_DBGAPI_STATUS_ERROR_MEMORY_ACCESS);
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
  TRY
  {
    if (!detail::is_initialized)
      THROW (AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED);

    process_t *process = process_t::find (process_id);

    if (!process)
      THROW (AMD_DBGAPI_STATUS_ERROR_INVALID_PROCESS_ID);

    if (!value || !value_size)
      THROW (AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT);

    if (wave_id == AMD_DBGAPI_WAVE_NONE
        && address_space_id == AMD_DBGAPI_ADDRESS_SPACE_GLOBAL)
      {
        *value_size = process->write_global_memory_partial (
          segment_address, value, *value_size);
      }
    else
      {
        wave_t *wave = find (wave_id);

        if (!wave)
          THROW (AMD_DBGAPI_STATUS_ERROR_INVALID_WAVE_ID);

        if (wave->state () != AMD_DBGAPI_WAVE_STATE_STOP)
          THROW (AMD_DBGAPI_STATUS_ERROR_WAVE_NOT_STOPPED);

        if (lane_id >= wave->lane_count () && lane_id != AMD_DBGAPI_LANE_NONE)
          THROW (AMD_DBGAPI_STATUS_ERROR_INVALID_LANE_ID);

        const address_space_t *address_space = find (address_space_id);

        if (!address_space)
          THROW (AMD_DBGAPI_STATUS_ERROR_INVALID_ADDRESS_SPACE_ID);

        if (!wave->architecture ().is_address_space_supported (*address_space)
            || wave->process () != *process)
          THROW (AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT_COMPATIBILITY);

        auto [lowered_address_space, lowered_address]
          = wave->architecture ().lower_address_space (*wave, *address_space,
                                                       segment_address);

        *value_size = wave->xfer_segment_memory (lowered_address_space,
                                                 lane_id, lowered_address,
                                                 nullptr, value, *value_size);
      }

    return AMD_DBGAPI_STATUS_SUCCESS;
  }
  CATCH (AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED,
         AMD_DBGAPI_STATUS_ERROR_INVALID_PROCESS_ID,
         AMD_DBGAPI_STATUS_ERROR_INVALID_WAVE_ID,
         AMD_DBGAPI_STATUS_ERROR_INVALID_LANE_ID,
         AMD_DBGAPI_STATUS_ERROR_INVALID_ADDRESS_SPACE_ID,
         AMD_DBGAPI_STATUS_ERROR_WAVE_NOT_STOPPED,
         AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT,
         AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT_COMPATIBILITY,
         AMD_DBGAPI_STATUS_ERROR_MEMORY_ACCESS);
  TRACE_END (make_ref (param_out (value_size)));
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_set_memory_precision (
  amd_dbgapi_process_id_t process_id,
  amd_dbgapi_memory_precision_t memory_precision)
{
  TRACE_BEGIN (param_in (process_id), param_in (memory_precision));
  TRY
  {
    if (!detail::is_initialized)
      THROW (AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED);

    process_t *process = process_t::find (process_id);

    if (!process)
      THROW (AMD_DBGAPI_STATUS_ERROR_INVALID_PROCESS_ID);

    if (memory_precision != AMD_DBGAPI_MEMORY_PRECISION_NONE
        && memory_precision != AMD_DBGAPI_MEMORY_PRECISION_PRECISE)
      THROW (AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT);

    process->set_precise_memory (memory_precision
                                 == AMD_DBGAPI_MEMORY_PRECISION_PRECISE);

    return AMD_DBGAPI_STATUS_SUCCESS;
  }
  CATCH (AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED,
         AMD_DBGAPI_STATUS_ERROR_INVALID_PROCESS_ID,
         AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT,
         AMD_DBGAPI_STATUS_ERROR_NOT_SUPPORTED);
  TRACE_END ();
}
