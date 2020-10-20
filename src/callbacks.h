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

#ifndef AMD_DBGAPI_CALLBACKS_H
#define AMD_DBGAPI_CALLBACKS_H 1

#include "amd-dbgapi.h"
#include "handle_object.h"

#include <cstddef>
#include <functional>
#include <string>

namespace amd::dbgapi
{

class process_t;

class shared_library_t
    : public detail::handle_object<amd_dbgapi_shared_library_id_t>
{
private:
  using notify_callback_t = std::function<void (const shared_library_t &)>;

  bool m_is_valid{ false };

  std::string const m_name;
  notify_callback_t const m_on_load;
  notify_callback_t const m_on_unload;
  amd_dbgapi_shared_library_state_t m_state;

  process_t &m_process;

public:
  shared_library_t (amd_dbgapi_shared_library_id_t library_id,
                    process_t &process, std::string name,
                    notify_callback_t on_load, notify_callback_t on_unload);
  ~shared_library_t ();

  const std::string &name () const { return m_name; }
  bool is_valid () const { return m_is_valid; }

  void set_state (amd_dbgapi_shared_library_state_t state);
  amd_dbgapi_shared_library_state_t state () const { return m_state; }

  amd_dbgapi_status_t get_info (amd_dbgapi_shared_library_info_t query,
                                size_t value_size, void *value) const;

  process_t &process () const { return m_process; }
};

class breakpoint_t : public detail::handle_object<amd_dbgapi_breakpoint_id_t>
{
private:
  using action_callback_t = std::function<amd_dbgapi_status_t (
      breakpoint_t &, amd_dbgapi_client_thread_id_t,
      amd_dbgapi_breakpoint_action_t *)>;

  bool m_inserted{ false };

  amd_dbgapi_global_address_t const m_address;
  action_callback_t const m_action;

  const shared_library_t &m_shared_library;

public:
  breakpoint_t (amd_dbgapi_breakpoint_id_t breakpoint_id,
                const shared_library_t &shared_library,
                amd_dbgapi_global_address_t address, action_callback_t action);

  ~breakpoint_t ();

  bool is_valid () const { return m_inserted; }

  amd_dbgapi_global_address_t address () const { return m_address; }
  action_callback_t action () const { return m_action; }

  amd_dbgapi_status_t get_info (amd_dbgapi_breakpoint_info_t query,
                                size_t value_size, void *value) const;

  const shared_library_t &shared_library () const { return m_shared_library; }
  process_t &process () const { return shared_library ().process (); }
};

} /* namespace amd::dbgapi */

#endif /* AMD_DBGAPI_CALLBACKS_H */
