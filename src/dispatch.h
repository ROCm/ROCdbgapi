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

#ifndef AMD_DBGAPI_DISPATCH_H
#define AMD_DBGAPI_DISPATCH_H 1

#include "agent.h"
#include "amd-dbgapi.h"
#include "architecture.h"
#include "handle_object.h"
#include "queue.h"

#include <cstddef>
#include <cstdint>

#include <hsa/hsa.h>

namespace amd::dbgapi
{

class architecture_t;
class process_t;

/* AMD Debugger API Dispatch.  */

class dispatch_t : public detail::handle_object<amd_dbgapi_dispatch_id_t>
{
private:
  amd_dbgapi_os_queue_packet_id_t const m_os_queue_packet_id;

  hsa_kernel_dispatch_packet_t m_packet{};
  std::unique_ptr<const architecture_t::kernel_descriptor_t>
    m_kernel_descriptor{};

  compute_queue_t &m_queue;

public:
  dispatch_t (amd_dbgapi_dispatch_id_t dispatch_id, compute_queue_t &queue,
              amd_dbgapi_os_queue_packet_id_t os_queue_packet_id,
              amd_dbgapi_global_address_t packet_address);

  ~dispatch_t () {}

  uint64_t os_queue_packet_id () const { return m_os_queue_packet_id; }
  const architecture_t::kernel_descriptor_t &kernel_descriptor () const;

  void get_info (amd_dbgapi_dispatch_info_t query, size_t value_size,
                 void *value) const;

  compute_queue_t &queue () const { return m_queue; }
  const agent_t &agent () const { return queue ().agent (); }
  process_t &process () const { return agent ().process (); }
  const architecture_t &architecture () const
  {
    return queue ().architecture ();
  }
};

} /* namespace amd::dbgapi */

#endif /* AMD_DBGAPI_DISPATCH_H */
