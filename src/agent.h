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

#ifndef AMD_DBGAPI_AGENT_H
#define AMD_DBGAPI_AGENT_H 1

#include "amd-dbgapi.h"
#include "debug.h"
#include "handle_object.h"
#include "os_driver.h"

#include <cstddef>

namespace amd::dbgapi
{

class architecture_t;
class process_t;

/* Agent.  */

class agent_t : public detail::handle_object<amd_dbgapi_agent_id_t>
{
private:
  os_agent_info_t const m_os_agent_info;
  os_exception_mask_t m_exceptions{ os_exception_mask_t::none };
  epoch_t m_mark{ 0 };

  const architecture_t *const m_architecture;
  process_t &m_process;

public:
  agent_t (amd_dbgapi_agent_id_t agent_id, process_t &process,
           const architecture_t *architecture,
           const os_agent_info_t &os_agent_info);
  ~agent_t ();

  os_agent_id_t os_agent_id () const { return m_os_agent_info.os_agent_id; }
  const os_agent_info_t &os_info () const { return m_os_agent_info; }

  bool supports_debugging () const
  {
    return architecture () != nullptr && m_os_agent_info.debugging_supported
           && m_os_agent_info.firmware_supported;
  }

  amd_dbgapi_watchpoint_share_kind_t watchpoint_share_kind () const;

  epoch_t mark () const { return m_mark; }
  void set_mark (epoch_t mark) { m_mark = mark; }

  void set_exceptions (os_exception_mask_t exceptions);
  void clear_exceptions (os_exception_mask_t exceptions);
  os_exception_mask_t exceptions () const { return m_exceptions; }

  void get_info (amd_dbgapi_agent_info_t query, size_t value_size,
                 void *value) const;

  const architecture_t *architecture () const { return m_architecture; }

  process_t &process () const { return m_process; }
};

} /* namespace amd::dbgapi */

#endif /* AMD_DBGAPI_AGENT_H */
