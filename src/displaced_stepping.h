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

#ifndef _AMD_DBGAPI_DISPLACED_STEPPING_H
#define _AMD_DBGAPI_DISPLACED_STEPPING_H 1

#include "defs.h"

#include "agent.h"
#include "handle_object.h"
#include "queue.h"

#include <vector>

#include <hsa/hsa.h>

namespace amd
{
namespace dbgapi
{

class architecture_t;
class wave_t;
class process_t;

/* AMD Debugger API Displaced Stepping.  */

class displaced_stepping_t
    : public detail::handle_object<amd_dbgapi_displaced_stepping_id_t>
{
public:
  displaced_stepping_t (
      amd_dbgapi_displaced_stepping_id_t displaced_stepping_id, queue_t *queue,
      amd_dbgapi_global_address_t from, const void *saved_instruction_bytes);

  ~displaced_stepping_t () {}

  bool is_valid () const { return m_is_valid; }
  bool is_simulated () const { return m_is_simulated; }

  amd_dbgapi_global_address_t from () const { return m_from; }
  /* The address of the displaced stepping buffer is this object's id.  */
  amd_dbgapi_global_address_t to () const { return id ().handle; }
  const std::vector<uint8_t> &original_instruction () const
  {
    return m_original_instruction;
  }

  amd_dbgapi_status_t start (wave_t &wave);
  amd_dbgapi_status_t complete (wave_t &wave);

  queue_t *queue () const { return m_queue; }
  agent_t *agent () const { return queue ()->agent (); }
  process_t *process () const { return agent ()->process (); }
  const architecture_t *architecture () const
  {
    return agent ()->architecture ();
  }

private:
  bool m_is_valid{ false };
  bool m_is_simulated{ false };

  amd_dbgapi_global_address_t const m_from;
  std::vector<uint8_t> m_original_instruction;

  queue_t *const m_queue;
};

} /* namespace dbgapi */
} /* namespace amd */

#endif /* _AMD_DBGAPI_DISPLACED_STEPPING_H */
