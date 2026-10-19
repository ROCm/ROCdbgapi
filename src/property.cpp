/* Copyright (c) 2026 Advanced Micro Devices, Inc.

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

#include "property.h"
#include "amd-dbgapi.h"
#include "process.h"

#include <algorithm>

namespace amd::dbgapi
{

std::pair<amd_dbgapi_property_id_t, std::unique_ptr<property_t>>
make_property (property_id_t property_id, std::string name, std::string descr)
{
  amd_dbgapi_property_id_t id = { static_cast<uint64_t> (property_id) };
  std::unique_ptr<property_t> property
    = std::make_unique<property_t> (id, name, descr);
  return std::make_pair (id, std::move (property));
}

property_t::properties_map_t property_t::s_property_map{
  [] ()
  {
    properties_map_t map;

    map.emplace (make_property (property_id_t::precise_memory_exception,
                                "PRECISE_MEMORY_EXCEPTION",
                                "Precise memory exception reporting"));

    map.emplace (make_property (property_id_t::precise_alu_exception,
                                "PRECISE_ALU_EXCEPTION",
                                "Precise ALU exception reporting"));

    map.emplace (
      make_property (property_id_t::lds_exception, "LDS_EXCEPTION",
                     "Group segment out-of-range address exception"));

    return map;
  }()
};

void
property_t::set (uint32_t value)
{
  switch (m_type)
    {
    case property_t::property_type_t::boolean:
      m_value = (value != 0);
      return;
    };

  dbgapi_assert_not_reached ("invalid property type");
}

/* property_t::property_type_t --> amd_dbgapi_property_type_t  */

static amd_dbgapi_property_type_t
type_to_intf (property_t::property_type_t type)
{
  switch (type)
    {
    case property_t::property_type_t::boolean:
      return AMD_DBGAPI_PROPERTY_TYPE_BOOL;
    }
  dbgapi_assert_not_reached ("property type is unknown");
}

/* property_t::property_scope_t --> amd_dbgapi_property_scope_t  */

static amd_dbgapi_property_scope_t
scope_to_intf (property_t::property_scope_t scope)
{
  switch (scope)
    {
    case property_t::property_scope_t::process:
      return AMD_DBGAPI_PROPERTY_SCOPE_PROCESS;
    }
  dbgapi_assert_not_reached ("property scope is unknown");
}

void
property_t::get_info (amd_dbgapi_property_info_t query,
                      size_t value_size, void *value) const
{
  switch (query)
    {
    case AMD_DBGAPI_PROPERTY_INFO_NAME:
      utils::get_info (value_size, value, name ());
      return;

    case AMD_DBGAPI_PROPERTY_INFO_DESCRIPTION:
      utils::get_info (value_size, value, desc ());
      return;

    case AMD_DBGAPI_PROPERTY_INFO_TYPE:
      utils::get_info (value_size, value, type_to_intf (type ()));
      return;

    case AMD_DBGAPI_PROPERTY_INFO_SCOPE:
      utils::get_info (value_size, value, scope_to_intf (scope ()));
      return;
    }

  throw api_error_t (AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT);
}

} /* namespace amd::dbgapi */

using namespace amd::dbgapi;

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_property_list (size_t *property_count,
                          amd_dbgapi_property_id_t **properties)
{
  TRACE_BEGIN (param_in (property_count), param_in (properties));
  TRY
  {
    if (!detail::is_initialized)
      THROW (AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED);

    if (property_count == nullptr || properties == nullptr)
      THROW (AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT);

    const property_t::properties_map_t &all_properties_map
      = property_t::all ();
    size_t byte_size
      = all_properties_map.size () * sizeof (amd_dbgapi_property_id_t);
    auto list
      = amd::dbgapi::allocate_memory<amd_dbgapi_property_id_t[]> (byte_size);
    size_t cnt = 0;
    for (const property_t::properties_map_element_t &property :
         all_properties_map)
      list[cnt++] = property.first;

    /* The underlying map is unordered, sort by handle to report the
       properties in a deterministic way.  */
    std::sort (list.get (), list.get () + cnt,
               [] (const amd_dbgapi_property_id_t &a,
                   const amd_dbgapi_property_id_t &b)
               { return a.handle < b.handle; });

    *property_count = cnt;
    *properties = list.release ();
  }
  CATCH (AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED,
         AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT,
         AMD_DBGAPI_STATUS_ERROR_CLIENT_CALLBACK);
  TRACE_END (make_ref (param_out (property_count)),
             make_ref (make_ref (param_out (properties)), *property_count));
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_property_get_info (amd_dbgapi_property_id_t property_id,
                              amd_dbgapi_property_info_t query,
                              size_t value_size, void *value)
{
  TRACE_BEGIN (param_in (property_id), param_in (query), param_in (value_size),
               param_in (value));
  TRY
  {
    if (!detail::is_initialized)
      THROW (AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED);

    const property_t::properties_map_t &all_properties_map
      = property_t::all ();

    if (all_properties_map.find (property_id) == all_properties_map.end ())
      THROW (AMD_DBGAPI_STATUS_ERROR_INVALID_PROPERTY_ID);

    property_t::property_map_value_t &property
      = all_properties_map.at (property_id);

    property->get_info (query, value_size, value);
  }
  CATCH (AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED,
         AMD_DBGAPI_STATUS_ERROR_INVALID_PROPERTY_ID,
         AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT,
         AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT_COMPATIBILITY,
         AMD_DBGAPI_STATUS_ERROR_CLIENT_CALLBACK);
  TRACE_END (make_query_ref (query, param_out (value)));
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_property_set (amd_dbgapi_property_id_t property_id,
                         const amd_dbgapi_property_context_t *ctx,
                         size_t value_size, void *value)
{
  TRACE_BEGIN (param_in (property_id), param_in (ctx), param_in (value_size),
               make_hex (make_ref (param_in (value), value_size)));
  TRY
  {
    if (!detail::is_initialized)
      THROW (AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED);

    const property_t::properties_map_t &all_properties_map
      = property_t::all ();

    if (all_properties_map.find (property_id) == all_properties_map.end ())
      THROW (AMD_DBGAPI_STATUS_ERROR_INVALID_PROPERTY_ID);

    property_t::property_map_value_t &property
      = all_properties_map.at (property_id);

    process_t *process = nullptr;

    /* If a process_id is needed, retrieve it.  */
    if (!!(property->scope () & property_t::property_scope_t::process))
      {
        if (ctx == nullptr)
          THROW (AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT);

        if (ctx->mode != AMD_DBGAPI_PROPERTY_CONTEXT_MODE_PROCESS)
          THROW (AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT_COMPATIBILITY);

        process = process_t::find (ctx->process_id);

        if (process == nullptr)
          THROW (AMD_DBGAPI_STATUS_ERROR_INVALID_PROCESS_ID);
      }

    if (value == nullptr)
      THROW (AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT);

    bool value_bool;
    bool value_bool_initialized = false;

    /* Verify the type and cast "value" based on it.  */
    if (property->type () == property_t::property_type_t::boolean)
      {
        if (value_size != sizeof (amd_dbgapi_property_bool_t))
          THROW (AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT_COMPATIBILITY);
        auto ptr = static_cast<amd_dbgapi_property_bool_t *> (value);
        value_bool = !!(*ptr);
        value_bool_initialized = true;
      }

    /* Right now, there's only bool support.  So assert here once,
       instead of before each ...->set().  */
    dbgapi_assert (value_bool_initialized);

    switch (static_cast<property_id_t> (property->id ().handle))
      {
      case property_id_t::none:
        THROW (AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT_COMPATIBILITY);
      case property_id_t::precise_memory_exception:
        process->set_precise_memory (value_bool);
        break;
      case property_id_t::precise_alu_exception:
        process->set_precise_alu_exceptions (value_bool);
        break;
      case property_id_t::lds_exception:
        process->set_reporting_lds_exceptions (value_bool);
        break;
      }
    property->set (value_bool);
  }
  CATCH (AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED,
         AMD_DBGAPI_STATUS_ERROR_INVALID_PROPERTY_ID,
         AMD_DBGAPI_STATUS_ERROR_INVALID_PROCESS_ID,
         AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT,
         AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT_COMPATIBILITY,
         AMD_DBGAPI_STATUS_ERROR_NOT_SUPPORTED);
  TRACE_END ();
}
