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

#ifndef AMD_DBGAPI_LOGGING_H
#define AMD_DBGAPI_LOGGING_H 1

#include "amd-dbgapi.h"
#include "utils.h"

#include <cstdarg>
#include <cstddef>
#include <sstream>
#include <string>
#include <tuple>
#include <type_traits>
#include <utility>

#define dbgapi_log(level, format, ...)                                        \
  do                                                                          \
    {                                                                         \
      if ((level) <= amd::dbgapi::log_level)                                  \
        amd::dbgapi::detail::log (level, format, ##__VA_ARGS__);              \
    }                                                                         \
  while (0)

namespace amd::dbgapi
{
namespace detail
{

extern size_t log_indent_depth;

extern void log (amd_dbgapi_log_level_t level, const char *format, ...)
#if defined(__GNUC__)
  __attribute__ ((format (printf, 2, 3)))
#endif /* defined (__GNUC__) */
  ;

} /* namespace detail */

extern void vlog (amd_dbgapi_log_level_t level, const char *format,
                  va_list va);

extern amd_dbgapi_log_level_t log_level;

template <typename T>
inline std::string
to_string (T v)
{
  std::ostringstream ss;
  ss << v;
  return ss.str ();
}

template <>
inline std::string
to_string (char *v)
{
  std::ostringstream ss;
  ss << '"' << v << '"';
  return ss.str ();
}

template <>
inline std::string
to_string (const char *v)
{
  std::ostringstream ss;
  ss << '"' << v << '"';
  return ss.str ();
}

namespace detail
{

template <typename T> struct hex
{
  T m_value;
};

template <typename T> struct query_ref
{
  T query;
  const void *memory;
};

/* Forward declare the ref struct so that we can define is_ref.  */
template <typename T> struct ref;

/* Check whether T is a ref struct.  */
template <typename T> struct is_ref : std::false_type
{
};

template <typename T> struct is_ref<ref<T>> : std::true_type
{
};

template <class T> inline constexpr bool is_ref_v = is_ref<T>::value;

template <typename T> struct ref
{
  T m_reference;
  size_t m_element_count;

  auto *value () const
  {
    if constexpr (is_ref_v<T>)
      return m_reference.value ();
    else
      return m_reference;
  }

  size_t count () const
  {
    if constexpr (is_ref_v<T>)
      return m_reference.count ();
    else
      return m_element_count;
  }

  auto operator* () const
  {
    if constexpr (is_ref_v<T>)
      return ref<decltype (*m_reference)>{ *m_reference, m_element_count };
    else
      return *m_reference;
  }

  ref &operator++ ()
  {
    ++m_reference;
    return *this;
  }

  ref operator++ (int)
  {
    ref old_ref = *this;
    ++m_reference;
    return old_ref;
  }
};

} /* namespace detail  */

/* Apply the hex modifier to the value of type T.  */
template <typename T>
auto
make_hex (T value)
{
  return detail::hex<T>{ value };
}

template <typename T>
detail::query_ref<T>
make_query_ref (T query, const void *memory)
{
  return detail::query_ref<T>{ query, memory };
}

template <typename T>
auto
make_ref (T *pointer, size_t count = 1)
{
  return detail::ref<T *>{ pointer, count };
}

template <typename T>
auto
make_ref (detail::ref<T> reference, size_t count = 1)
{
  return detail::ref<detail::ref<T>>{ reference, count };
}

template <typename T> std::string to_string (detail::hex<detail::ref<T>> hex);

template <typename T>
std::string
to_string (detail::hex<T> v)
{
  std::ostringstream ss;
  ss << "0x" << std::hex << +v.m_value;
  return ss.str ();
}

template <typename T, typename U = decltype (*std::declval<detail::ref<T>> ())>
std::string
to_string (detail::ref<T> ref)
{
  if (!ref.value ())
    return "null";

  std::string str
    = string_printf ("*%p=", static_cast<const void *> (ref.value ()));
  const size_t count = ref.count ();

  if (count != 1)
    str += '[';

  for (size_t i = 0; i < count; ++i, ++ref)
    {
      if (i != 0)
        str += ',';

      if (i >= 16)
        return str + string_printf ("... %ld more elements]", count - i);

      str += to_string (U{ *ref });
    }

  if (count != 1)
    str += ']';

  return str;
}

template <typename T>
std::string
to_string (detail::hex<detail::ref<T>> hex)
{
  using Modifier = detail::hex<decltype (*std::declval<detail::ref<T>> ())>;
  return to_string<T, Modifier> (hex.m_value);
}

#define AMD_DBGAPI_TYPES_DO(F)                                                \
  F (amd_dbgapi_address_class_id_t)                                           \
  F (amd_dbgapi_address_class_info_t)                                         \
  F (amd_dbgapi_address_class_state_t)                                        \
  F (amd_dbgapi_address_space_access_t)                                       \
  F (amd_dbgapi_address_space_alias_t)                                        \
  F (amd_dbgapi_address_space_id_t)                                           \
  F (amd_dbgapi_address_space_info_t)                                         \
  F (amd_dbgapi_agent_id_t)                                                   \
  F (amd_dbgapi_agent_info_t)                                                 \
  F (amd_dbgapi_architecture_id_t)                                            \
  F (amd_dbgapi_architecture_info_t)                                          \
  F (amd_dbgapi_breakpoint_action_t)                                          \
  F (amd_dbgapi_breakpoint_id_t)                                              \
  F (amd_dbgapi_breakpoint_info_t)                                            \
  F (amd_dbgapi_changed_t)                                                    \
  F (amd_dbgapi_code_object_id_t)                                             \
  F (amd_dbgapi_code_object_info_t)                                           \
  F (amd_dbgapi_direct_call_register_pair_information_t)                      \
  F (amd_dbgapi_dispatch_barrier_t)                                           \
  F (amd_dbgapi_dispatch_fence_scope_t)                                       \
  F (amd_dbgapi_dispatch_id_t)                                                \
  F (amd_dbgapi_dispatch_info_t)                                              \
  F (amd_dbgapi_displaced_stepping_id_t)                                      \
  F (amd_dbgapi_displaced_stepping_info_t)                                    \
  F (amd_dbgapi_event_id_t)                                                   \
  F (amd_dbgapi_event_info_t)                                                 \
  F (amd_dbgapi_event_kind_t)                                                 \
  F (amd_dbgapi_instruction_kind_t)                                           \
  F (amd_dbgapi_instruction_properties_t)                                     \
  F (amd_dbgapi_log_level_t)                                                  \
  F (amd_dbgapi_memory_precision_t)                                           \
  F (amd_dbgapi_os_queue_type_t)                                              \
  F (amd_dbgapi_process_id_t)                                                 \
  F (amd_dbgapi_process_info_t)                                               \
  F (amd_dbgapi_progress_t)                                                   \
  F (amd_dbgapi_queue_error_reason_t)                                         \
  F (amd_dbgapi_queue_id_t)                                                   \
  F (amd_dbgapi_queue_info_t)                                                 \
  F (amd_dbgapi_queue_state_t)                                                \
  F (amd_dbgapi_register_class_id_t)                                          \
  F (amd_dbgapi_register_class_info_t)                                        \
  F (amd_dbgapi_register_class_state_t)                                       \
  F (amd_dbgapi_register_exists_t)                                            \
  F (amd_dbgapi_register_id_t)                                                \
  F (amd_dbgapi_register_info_t)                                              \
  F (amd_dbgapi_resume_mode_t)                                                \
  F (amd_dbgapi_runtime_state_t)                                              \
  F (amd_dbgapi_shared_library_id_t)                                          \
  F (amd_dbgapi_shared_library_info_t)                                        \
  F (amd_dbgapi_shared_library_state_t)                                       \
  F (amd_dbgapi_status_t)                                                     \
  F (amd_dbgapi_wave_creation_t)                                              \
  F (amd_dbgapi_wave_id_t)                                                    \
  F (amd_dbgapi_wave_info_t)                                                  \
  F (amd_dbgapi_wave_state_t)                                                 \
  F (amd_dbgapi_wave_stop_reason_t)                                           \
  F (amd_dbgapi_watchpoint_id_t)                                              \
  F (amd_dbgapi_watchpoint_info_t)                                            \
  F (amd_dbgapi_watchpoint_kind_t)                                            \
  F (amd_dbgapi_watchpoint_share_kind_t)                                      \
  F (detail::query_ref<amd_dbgapi_agent_info_t>)                              \
  F (detail::query_ref<amd_dbgapi_address_class_info_t>)                      \
  F (detail::query_ref<amd_dbgapi_address_space_info_t>)                      \
  F (detail::query_ref<amd_dbgapi_architecture_info_t>)                       \
  F (detail::query_ref<amd_dbgapi_breakpoint_info_t>)                         \
  F (detail::query_ref<amd_dbgapi_code_object_info_t>)                        \
  F (detail::query_ref<amd_dbgapi_dispatch_info_t>)                           \
  F (detail::query_ref<amd_dbgapi_displaced_stepping_info_t>)                 \
  F (detail::query_ref<amd_dbgapi_event_info_t>)                              \
  F (detail::query_ref<amd_dbgapi_instruction_kind_t>)                        \
  F (detail::query_ref<amd_dbgapi_process_info_t>)                            \
  F (detail::query_ref<amd_dbgapi_queue_info_t>)                              \
  F (detail::query_ref<amd_dbgapi_register_info_t>)                           \
  F (detail::query_ref<amd_dbgapi_register_class_info_t>)                     \
  F (detail::query_ref<amd_dbgapi_shared_library_info_t>)                     \
  F (detail::query_ref<amd_dbgapi_watchpoint_info_t>)                         \
  F (detail::query_ref<amd_dbgapi_wave_info_t>)

#define EXPLICIT_SPECIALIZATION(...)                                          \
  template <> std::string to_string (__VA_ARGS__);
AMD_DBGAPI_TYPES_DO (EXPLICIT_SPECIALIZATION)

#undef EXPLICIT_SPECIALIZATION
#undef AMD_DBGAPI_TYPES_DO

inline std::string
to_string ()
{
  return "";
}

template <typename T, typename... Args>
inline std::string
to_string (T &&first, Args &&...args)
{
  std::string str = to_string (std::forward<T> (first));
  std::string last = to_string (std::forward<Args> (args)...);
  if (!last.empty ())
    str += ", " + last;
  return str;
}

template <typename... Args>
inline std::string
to_string (std::tuple<Args...> &&t)
{
  return std::apply (
    [] (auto &&...ts) { return to_string (std::forward<Args> (ts)...); }, t);
}

namespace detail
{

template <typename Functor,
          typename Result = decltype (std::declval<Functor> () ())>
struct tracer_closure
{
  Result m_result;

  tracer_closure (Functor f) : m_result (f ()){};
  Result operator() () const { return m_result; }

  operator std::string () const { return to_string (m_result); }
};

template <typename Functor> struct tracer_closure<Functor, void>
{
  tracer_closure (Functor f) { f (); }
  void operator() () const {}

  operator std::string () const { return "void"; }
};

} /* namespace detail */

class tracer
{
private:
  const char *m_prefix;
  const char *m_function;

public:
  template <typename... Args>
  tracer (const char *prefix, const char *function)
    : m_prefix (prefix), m_function (function)
  {
  }

  template <typename... Args, typename Functor>
  detail::tracer_closure<Functor> enter (std::tuple<Args...> in_args,
                                         Functor func);

  template <typename... Args, typename Functor>
  decltype (std::declval<Functor> () ())
  leave (std::tuple<Args...> out_args,
         const detail::tracer_closure<Functor> &closure);
};

template <typename... Args, typename Functor>
detail::tracer_closure<Functor>
tracer::enter (std::tuple<Args...> in_args, Functor func)
{
  if (log_level != AMD_DBGAPI_LOG_LEVEL_VERBOSE)
    return detail::tracer_closure (func);

  detail::log (AMD_DBGAPI_LOG_LEVEL_VERBOSE, "%s%s (%s) {", m_prefix,
               m_function, to_string (std::move (in_args)).c_str ());
  ++detail::log_indent_depth;

  try
    {
      return detail::tracer_closure (func);
    }
  catch (...)
    {
      --detail::log_indent_depth;
      dbgapi_log (AMD_DBGAPI_LOG_LEVEL_VERBOSE, "%s} throw", m_prefix);
      throw;
    }
}

template <typename... Args, typename Functor>
decltype (std::declval<Functor> () ())
tracer::leave (std::tuple<Args...> out_args,
               const detail::tracer_closure<Functor> &closure)
{
  if (log_level != AMD_DBGAPI_LOG_LEVEL_VERBOSE)
    return closure ();

  /* Print the outargs unless the return type is amd_dbgapi_status_t and the
     result is not AMD_DBGAPI_STATUS_SUCCESS.  */
  bool print_out_args = true;
  if constexpr (std::is_same_v<decltype (closure ()), amd_dbgapi_status_t>)
    print_out_args = closure () == AMD_DBGAPI_STATUS_SUCCESS;

  std::string results = closure;
  if (print_out_args && std::tuple_size_v<decltype (out_args)> != 0)
    results += ", " + to_string (std::move (out_args));

  --detail::log_indent_depth;
  detail::log (AMD_DBGAPI_LOG_LEVEL_VERBOSE, "%s} = %s", m_prefix,
               results.c_str ());

  return closure ();
}

#if defined(WITH_API_TRACING)

#define _TRACE_BEGIN(prefix, ...)                                             \
  amd::dbgapi::tracer _tracer (prefix, __FUNCTION__);                         \
  auto _closure = _tracer.enter (std::make_tuple (__VA_ARGS__), [&] () {

#define _TRACE_END(...)                                                       \
  });                                                                         \
  return _tracer.leave (std::make_tuple (__VA_ARGS__), _closure);

#define TRACE_BEGIN(...) _TRACE_BEGIN ("", __VA_ARGS__)
#define TRACE_END(...) _TRACE_END (__VA_ARGS__)

#define TRACE_CALLBACK_BEGIN(...) _TRACE_BEGIN ("[callback] ", __VA_ARGS__)
#define TRACE_CALLBACK_END(...) _TRACE_END (__VA_ARGS__)

#else /* !defined (WITH_API_TRACING) */

#define TRACE_BEGIN(...)
#define TRACE_END(...)

#define TRACE_CALLBACK_BEGIN(...)
#define TRACE_CALLBACK_END(...)

#endif /* !defined (WITH_API_TRACING) */

} /* namespace amd::dbgapi */

#endif /* AMD_DBGAPI_LOGGING_H */
