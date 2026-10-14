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

#ifndef AMD_DBGAPI_PROPERTY_H
#define AMD_DBGAPI_PROPERTY_H 1

#include "amd-dbgapi.h"
#include "handle_object.h"

#include <set>
#include <unordered_map>
#include <vector>

namespace amd::dbgapi
{

/* Properties  */

enum class amdgpu_property_t : uint32_t
{
  none = 0,

  /* Report memory exceptions with the precise incurring PC address.  */
  memory_precise_exception,

  /* Report ALU exceptions with the precise incurring PC address.  */
  alu_precise_exception,

  /* Enable LDS out-of-range address exceptions.  */
  lds_exception,
};

class property_t
{
public:
  using property_values_t
    = std::unordered_map<amdgpu_property_t, std::vector<uint32_t>>;

  static std::unordered_map<amd_dbgapi_property_id_t,
                            std::unique_ptr<const property_t>,
                            hash<amd_dbgapi_property_id_t>>
                                s_properties;

  enum property_type
  {
      boolean,          /* Of an Enable/Disable nature.  */
  };

  property_type get_type () const;
  std::string   get_name () const;
  std::string   get_descr () const;
  bool          set ()
  {
      switch () {
          case
          case
      }
  }

private:
  std::string name;
  property_name_t m_names;     /* Key: amdgpu_property_t  */
  property_values_t m_values;  /* Key: amdgpu_property_t  */
};

} /* namespace amd::dbgapi */

#endif /* AMD_DBGAPI_PROPERTY_H */
