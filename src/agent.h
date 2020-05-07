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

#ifndef _AMD_DBGAPI_AGENT_H
#define _AMD_DBGAPI_AGENT_H 1

#include "defs.h"

#include "handle_object.h"

#include <atomic>
#include <string>

namespace amd
{
namespace dbgapi
{

class architecture_t;
class process_t;

/* ROCm Debug Agent.  */

class agent_t : public detail::handle_object<amd_dbgapi_agent_id_t>
{
public:
  using kfd_gpu_id_t = uint32_t;

  struct properties_t
  {
    /* Public name of the "device".  */
    std::string name;
    /* BDF - Identifies the device location in the overall system.  */
    uint32_t location_id{ 0 };
    /* Number of FCompute cores.  */
    uint32_t simd_count{ 0 };
    /* Number of Shader Banks or Shader Engines.  */
    uint32_t shader_engine_count{ 0 };
    /* Number of SIMD arrays per engine.  */
    uint32_t simd_arrays_per_engine{ 0 };
    /* Number of Compute Units (CU) per SIMD array.  */
    uint32_t cu_per_simd_array{ 0 };
    /* Number of SIMD representing a Compute Unit (CU).  */
    uint32_t simd_per_cu{ 0 };
    /* Maximum number of launched waves per SIMD.  */
    uint32_t max_waves_per_simd{ 0 };
    /* PCI vendor id.  */
    uint32_t vendor_id{ 0 };
    /* PCI device id.  */
    uint32_t device_id{ 0 };
    /* ucode version.  */
    uint32_t fw_version{ 0 };
  };

  agent_t (amd_dbgapi_agent_id_t agent_id, process_t &process,
           kfd_gpu_id_t gpu_id, const architecture_t &architecture,
           const properties_t &properties);
  ~agent_t ();

  kfd_gpu_id_t gpu_id () const { return m_gpu_id; }
  const properties_t &properties () const { return m_properties; }

  file_desc_t poll_fd () const { return m_poll_fd; }

  std::atomic<bool> &kfd_event_notifier () { return m_kfd_event_notifier; }
  amd_dbgapi_status_t next_kfd_event (amd_dbgapi_queue_id_t *queue_id,
                                      uint32_t *queue_status);

  amd_dbgapi_status_t get_info (amd_dbgapi_agent_info_t query,
                                size_t value_size, void *value) const;

  const architecture_t &architecture () const { return m_architecture; }
  process_t &process () const { return m_process; }

  amd_dbgapi_status_t enable_debug_trap (void);
  amd_dbgapi_status_t disable_debug_trap (void);
  bool debug_trap_enabled () const { return m_poll_fd != -1; }

private:
  kfd_gpu_id_t const m_gpu_id;
  properties_t const m_properties;

  file_desc_t m_poll_fd{ -1 };
  std::atomic<bool> m_kfd_event_notifier{ false };

  const architecture_t &m_architecture;
  process_t &m_process;
};

} /* namespace dbgapi */
} /* namespace amd */

#endif /* _AMD_DBGAPI_AGENT_H */
