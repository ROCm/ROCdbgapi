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

#include "agent.h"
#include "architecture.h"
#include "debug.h"
#include "handle_object.h"
#include "initialization.h"
#include "logging.h"
#include "os_driver.h"
#include "process.h"
#include "utils.h"

#include <cstdint>
#include <vector>

namespace amd::dbgapi
{

agent_t::agent_t (amd_dbgapi_agent_id_t agent_id, process_t &process,
                  const architecture_t &architecture,
                  const os_agent_snapshot_entry_t &os_agent_info)
  : handle_object (agent_id), m_os_agent_info (os_agent_info),
    m_architecture (architecture), m_process (process)
{
}

agent_t::~agent_t () {}

void
agent_t::set_exception (os_exception_code_t exception_code)
{
  m_exceptions |= os_exception_mask (exception_code);
}

void
agent_t::clear_exception (os_exception_code_t exception_code)
{
  m_exceptions &= ~os_exception_mask (exception_code);
}

bool
agent_t::has_exception (os_exception_code_t exception_code) const
{
  return (m_exceptions & os_exception_mask (exception_code)) != 0;
}

amd_dbgapi_status_t
agent_t::get_info (amd_dbgapi_agent_info_t query, size_t value_size,
                   void *value) const
{
  switch (query)
    {
    case AMD_DBGAPI_AGENT_INFO_PROCESS:
      return utils::get_info (value_size, value, process ().id ());

    case AMD_DBGAPI_AGENT_INFO_NAME:
      return utils::get_info (value_size, value, m_os_agent_info.name);

    case AMD_DBGAPI_AGENT_INFO_ARCHITECTURE:
      return utils::get_info (value_size, value, architecture ().id ());

    case AMD_DBGAPI_AGENT_INFO_PCI_SLOT:
      return utils::get_info (value_size, value, m_os_agent_info.location_id);

    case AMD_DBGAPI_AGENT_INFO_PCI_VENDOR_ID:
      return utils::get_info (value_size, value, m_os_agent_info.vendor_id);

    case AMD_DBGAPI_AGENT_INFO_PCI_DEVICE_ID:
      return utils::get_info (value_size, value, m_os_agent_info.device_id);

    case AMD_DBGAPI_AGENT_INFO_EXECUTION_UNIT_COUNT:
      return utils::get_info (value_size, value, m_os_agent_info.simd_count);

    case AMD_DBGAPI_AGENT_INFO_MAX_WAVES_PER_EXECUTION_UNIT:
      return utils::get_info (value_size, value,
                              m_os_agent_info.max_waves_per_simd);

    case AMD_DBGAPI_AGENT_INFO_OS_ID:
      return utils::get_info (
        value_size, value,
        static_cast<amd_dbgapi_os_agent_id_t> (m_os_agent_info.os_agent_id));
    }
  return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;
}

} /* namespace amd::dbgapi */

using namespace amd::dbgapi;

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_agent_get_info (amd_dbgapi_agent_id_t agent_id,
                           amd_dbgapi_agent_info_t query, size_t value_size,
                           void *value)
{
  TRACE_BEGIN (param_in (agent_id), param_in (query), param_in (value_size),
               param_in (value));
  TRY;

  if (!detail::is_initialized)
    return AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED;

  agent_t *agent = find (agent_id);

  if (!agent)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_AGENT_ID;

  return agent->get_info (query, value_size, value);

  CATCH;
  TRACE_END (make_query_ref (query, param_out (value)));
}

amd_dbgapi_status_t AMD_DBGAPI
amd_dbgapi_process_agent_list (amd_dbgapi_process_id_t process_id,
                               size_t *agent_count,
                               amd_dbgapi_agent_id_t **agents,
                               amd_dbgapi_changed_t *changed)
{
  TRACE_BEGIN (param_in (process_id), param_in (agent_count),
               param_in (agents), param_in (changed));
  TRY;

  if (!detail::is_initialized)
    return AMD_DBGAPI_STATUS_ERROR_NOT_INITIALIZED;

  std::vector<process_t *> processes;
  if (process_id != AMD_DBGAPI_PROCESS_NONE)
    {
      process_t *process = process_t::find (process_id);

      if (!process)
        return AMD_DBGAPI_STATUS_ERROR_INVALID_PROCESS_ID;

      if (amd_dbgapi_status_t status = process->update_agents ();
          status != AMD_DBGAPI_STATUS_SUCCESS)
        error ("process_t::update_agents failed (rc=%d)", status);

      processes.emplace_back (process);
    }
  else
    {
      for (auto &&process : process_t::all ())
        {
          if (amd_dbgapi_status_t status = process.update_agents ();
              status != AMD_DBGAPI_STATUS_SUCCESS)
            error ("process_t::update_agents failed (rc=%d)", status);

          processes.emplace_back (&process);
        }
    }

  return utils::get_handle_list<agent_t> (processes, agent_count, agents,
                                          changed);
  CATCH;
  TRACE_END (make_ref (param_out (agent_count)),
             make_ref (make_ref (param_out (agents)), *agent_count),
             make_ref (param_out (changed)));
}
