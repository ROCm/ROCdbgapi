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

#include "utils.h"
#include "handle_object.h"
#include "process.h"

#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <string>

#if !defined(_GNU_SOURCE) /* For poll.h. See feature_test_macros(7) */
#error "_GNU_SOURCE must be defined"
#endif /* !defined (_GNU_SOURCE) */

#include <errno.h>
#include <fcntl.h>
#include <unistd.h>

namespace amd::dbgapi
{

namespace utils
{
namespace detail
{
/* Make sure the storage size of enums passed by reference matches the
   specification.
 */

using amd_dbgapi_enum_underlying_type32_t = uint32_t;
using amd_dbgapi_enum_underlying_type64_t = uint64_t;

static_assert (
    true
        && sizeof (amd_dbgapi_address_class_state_t)
               == sizeof (amd_dbgapi_enum_underlying_type32_t)
        && sizeof (amd_dbgapi_address_space_access_t)
               == sizeof (amd_dbgapi_enum_underlying_type32_t)
        && sizeof (amd_dbgapi_address_space_alias_t)
               == sizeof (amd_dbgapi_enum_underlying_type32_t)
        && sizeof (amd_dbgapi_breakpoint_action_t)
               == sizeof (amd_dbgapi_enum_underlying_type32_t)
        && sizeof (amd_dbgapi_changed_t)
               == sizeof (amd_dbgapi_enum_underlying_type32_t)
        && sizeof (amd_dbgapi_dispatch_barrier_t)
               == sizeof (amd_dbgapi_enum_underlying_type32_t)
        && sizeof (amd_dbgapi_dispatch_fence_scope_t)
               == sizeof (amd_dbgapi_enum_underlying_type32_t)
        && sizeof (amd_dbgapi_event_kind_t)
               == sizeof (amd_dbgapi_enum_underlying_type32_t)
        && sizeof (amd_dbgapi_memory_precision_t)
               == sizeof (amd_dbgapi_enum_underlying_type32_t)
        && sizeof (amd_dbgapi_queue_error_reason_t)
               == sizeof (amd_dbgapi_enum_underlying_type64_t)
        && sizeof (amd_dbgapi_queue_state_t)
               == sizeof (amd_dbgapi_enum_underlying_type32_t)
        && sizeof (amd_dbgapi_os_queue_type_t)
               == sizeof (amd_dbgapi_enum_underlying_type32_t)
        && sizeof (amd_dbgapi_register_class_state_t)
               == sizeof (amd_dbgapi_enum_underlying_type32_t)
        && sizeof (amd_dbgapi_runtime_state_t)
               == sizeof (amd_dbgapi_enum_underlying_type32_t)
        && sizeof (amd_dbgapi_shared_library_state_t)
               == sizeof (amd_dbgapi_enum_underlying_type32_t)
        && sizeof (amd_dbgapi_watchpoint_share_kind_t)
               == sizeof (amd_dbgapi_enum_underlying_type32_t)
        && sizeof (amd_dbgapi_wave_state_t)
               == sizeof (amd_dbgapi_enum_underlying_type32_t)
        && sizeof (amd_dbgapi_wave_stop_reason_t)
               == sizeof (amd_dbgapi_enum_underlying_type64_t),
    "an enum type is not compatible with the dbgapi enum underlying type");

} /* namespace detail */

template <>
amd_dbgapi_status_t
get_info (size_t value_size, void *ret, const std::string &value)
{
  char *retval;

  if (value_size != sizeof (retval))
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT_COMPATIBILITY;

  const size_t size = value.size ();
  retval = static_cast<char *> (amd::dbgapi::allocate_memory (size + 1));
  if (!retval)
    return AMD_DBGAPI_STATUS_ERROR_CLIENT_CALLBACK;

  value.copy (retval, size);
  retval[size] = '\0';

  *static_cast<decltype (retval) *> (ret) = retval;
  return AMD_DBGAPI_STATUS_SUCCESS;
}

template <typename T>
amd_dbgapi_status_t
get_info (size_t value_size, void *ret, const std::vector<T> &value)
{
  T *retval;

  if (value_size != sizeof (retval))
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT_COMPATIBILITY;

  const size_t size = sizeof (T) * value.size ();
  retval = static_cast<T *> (amd::dbgapi::allocate_memory (size));
  if (size && !retval)
    return AMD_DBGAPI_STATUS_ERROR_CLIENT_CALLBACK;

  memcpy (retval, value.data (), size);

  *static_cast<decltype (retval) *> (ret) = retval;
  return AMD_DBGAPI_STATUS_SUCCESS;
}

template amd_dbgapi_status_t
get_info (size_t value_size, void *ret,
          const std::vector<amd_dbgapi_watchpoint_id_t> &value);

template amd_dbgapi_status_t get_info (size_t value_size, void *ret,
                                       const std::vector<uint8_t> &value);

template <typename Object>
amd_dbgapi_status_t
get_handle_list (const std::vector<process_t *> &processes,
                 size_t *object_count, typename Object::handle_type **objects,
                 amd_dbgapi_changed_t *changed)
{
  using Handle = typename Object::handle_type;

  if (!objects || !object_count)
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT;

  /* Check whether the Objects have changed since the last time get_handle_list
     was called. The flag is set whenever objects are created, or destroyed, or
     invalidated, and cleared when get_handle_list is called.  */
  if (changed)
    {
      bool one_changed{ false };

      for (auto &&process : processes)
        one_changed |= process->set_changed<Object> (false);

      if (!one_changed)
        {
          *objects = nullptr;
          *object_count = 0;
          *changed = AMD_DBGAPI_CHANGED_NO;
          return AMD_DBGAPI_STATUS_SUCCESS;
        }
    }

  size_t count{ 0 };
  for (auto &&process : processes)
    count += process->count<Object> ();

  Handle *retval;

  /* The size allocated includes space for all objects, so it is larger than
     necessary if there are invalid objects, but that is conservatively safe.
   */
  retval = static_cast<Handle *> (allocate_memory (count * sizeof (Handle)));
  if (count && !retval)
    return AMD_DBGAPI_STATUS_ERROR_CLIENT_CALLBACK;

  size_t pos{ 0 };
  for (auto &&process : processes)
    for (auto &&object : process->range<Object> ())
      {
        if (!is_valid (object))
          continue;

        dbgapi_assert (pos < count);
        retval[pos++] = object.id ();
      }

  *objects = retval;
  *object_count = count;

  if (changed)
    *changed = AMD_DBGAPI_CHANGED_YES;

  return AMD_DBGAPI_STATUS_SUCCESS;
}

template amd_dbgapi_status_t get_handle_list<code_object_t> (
    const std::vector<process_t *> &processes, size_t *object_count,
    amd_dbgapi_code_object_id_t **objects, amd_dbgapi_changed_t *changed);

template amd_dbgapi_status_t get_handle_list<agent_t> (
    const std::vector<process_t *> &processes, size_t *object_count,
    amd_dbgapi_agent_id_t **objects, amd_dbgapi_changed_t *changed);

template amd_dbgapi_status_t get_handle_list<queue_t> (
    const std::vector<process_t *> &processes, size_t *object_count,
    amd_dbgapi_queue_id_t **objects, amd_dbgapi_changed_t *changed);

template amd_dbgapi_status_t get_handle_list<dispatch_t> (
    const std::vector<process_t *> &processes, size_t *object_count,
    amd_dbgapi_dispatch_id_t **objects, amd_dbgapi_changed_t *changed);

template amd_dbgapi_status_t
get_handle_list<wave_t> (const std::vector<process_t *> &processes,
                         size_t *object_count, amd_dbgapi_wave_id_t **objects,
                         amd_dbgapi_changed_t *changed);

} /* namespace utils */

std::string
string_vprintf (const char *format, va_list va)
{
  va_list copy;

  va_copy (copy, va);
  size_t size = vsnprintf (NULL, 0, format, copy);
  va_end (copy);

  std::string str (size, '\0');
  vsprintf (&str[0], format, va);

  return str;
}

std::string
string_printf (const char *format, ...)
{
  va_list va;
  va_start (va, format);
  std::string str (string_vprintf (format, va));
  va_end (va);

  return str;
}

bool
pipe_t::open ()
{
  std::array<file_desc_t, 2> pipe;
  if (::pipe2 (pipe.data (), O_CLOEXEC))
    {
      warning ("pipe_t::open: pipe2 failed: %s", strerror (errno));
      return false;
    }

  m_pipe_fd.emplace (pipe);

  if (::fcntl (read_fd (), F_SETFL, O_NONBLOCK)
      || ::fcntl (write_fd (), F_SETFL, O_NONBLOCK))
    {
      warning ("pipe_t::open: fcntl failed: %s", strerror (errno));
      close ();
      return false;
    }

  return true;
}

void
pipe_t::close ()
{
  if (is_valid ())
    {
      ::close (read_fd ());
      ::close (write_fd ());
    }

  m_pipe_fd.reset ();
}

int
pipe_t::flush ()
{
  int ret;

  do
    {
      char buf;
      ret = ::read (read_fd (), &buf, 1);
    }
  while (ret >= 0 || (ret == -1 && errno == EINTR));

  if (ret == -1 && errno != EAGAIN)
    error ("read: %s", strerror (errno));

  return ret == -1 ? -errno : 0;
}

int
pipe_t::mark ()
{
  int ret;

  /* First, flush the pipe.  */
  flush ();

  do
    {
      ret = ::write (write_fd (), "+", 1);
    }
  while (ret == -1 && errno == EINTR);

  if (ret == -1 && errno != EAGAIN)
    error ("write: %s", strerror (errno));

  return ret == -1 ? -errno : 0;
}

} /* namespace amd::dbgapi */
