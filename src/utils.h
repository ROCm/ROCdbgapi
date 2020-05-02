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

#ifndef _AMD_DBGAPI_UTILS_H
#define _AMD_DBGAPI_UTILS_H 1

#include "defs.h"

#include "debug.h"

#include <cstdarg>
#include <cstring>
#include <functional>
#include <string>
#include <tuple>
#include <type_traits>
#include <vector>

#define TRY                                                                   \
  try                                                                         \
    {

#define CATCH                                                                 \
  }                                                                           \
  catch (const amd::dbgapi::exception_t &ex)                                  \
  {                                                                           \
    ex.print_message ();                                                      \
    return ex.error_code ();                                                  \
  }

#define CONCAT_NX(x, y) x##y
#define CONCAT(x, y) CONCAT_NX (x, y)

#define DELIM_COMMA() ,
#define DELIM_SEMICOLON() ;

#define NTH_ARG(_1, _2, _3, _4, _5, _6, _7, _8, _9, _10, N, ...) N
#define ARGS_COUNT(...) NTH_ARG (__VA_ARGS__, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1)

#define FOR_EACH_1(FUNCTION, DELIM, ARG, ...) FUNCTION (ARG)
#define FOR_EACH_2(FUNCTION, DELIM, ARG, ...)                                 \
  FUNCTION (ARG) DELIM () FOR_EACH_1 (FUNCTION, DELIM, __VA_ARGS__)
#define FOR_EACH_3(FUNCTION, DELIM, ARG, ...)                                 \
  FUNCTION (ARG) DELIM () FOR_EACH_2 (FUNCTION, DELIM, __VA_ARGS__)
#define FOR_EACH_4(FUNCTION, DELIM, ARG, ...)                                 \
  FUNCTION (ARG) DELIM () FOR_EACH_3 (FUNCTION, DELIM, __VA_ARGS__)
#define FOR_EACH_5(FUNCTION, DELIM, ARG, ...)                                 \
  FUNCTION (ARG) DELIM () FOR_EACH_4 (FUNCTION, DELIM, __VA_ARGS__)
#define FOR_EACH_6(FUNCTION, DELIM, ARG, ...)                                 \
  FUNCTION (ARG) DELIM () FOR_EACH_5 (FUNCTION, DELIM, __VA_ARGS__)
#define FOR_EACH_7(FUNCTION, DELIM, ARG, ...)                                 \
  FUNCTION (ARG) DELIM () FOR_EACH_6 (FUNCTION, DELIM, __VA_ARGS__)
#define FOR_EACH_8(FUNCTION, DELIM, ARG, ...)                                 \
  FUNCTION (ARG) DELIM () FOR_EACH_7 (FUNCTION, DELIM, __VA_ARGS__)
#define FOR_EACH_9(FUNCTION, DELIM, ARG, ...)                                 \
  FUNCTION (ARG) DELIM () FOR_EACH_8 (FUNCTION, DELIM, __VA_ARGS__)
#define FOR_EACH_10(FUNCTION, DELIM, ARG, ...)                                \
  FUNCTION (ARG) DELIM () FOR_EACH_9 (FUNCTION, DELIM, __VA_ARGS__)

#define FOR_EACH_N(FUNCTION, DELIM, N, ...)                                   \
  CONCAT (FOR_EACH_, N) (FUNCTION, DELIM, __VA_ARGS__)
#define FOR_EACH(FUNCTION, DELIM, ...)                                        \
  FOR_EACH_N (FUNCTION, DELIM, ARGS_COUNT (__VA_ARGS__), __VA_ARGS__)

namespace amd
{
namespace dbgapi
{

extern std::string string_vprintf (const char *format, va_list va);
extern std::string string_printf (const char *format, ...)
#if defined(__GNUC__)
    __attribute__ ((format (printf, 1, 2)))
#endif /* defined (__GNUC__) */
    ;

namespace detail
{

template <typename Functor, typename Return, typename First, typename... Rest>
First first_argument_type_helper_t (Return (Functor::*) (First, Rest...));

template <typename Functor, typename Return, typename First, typename... Rest>
First first_argument_type_helper_t (Return (Functor::*) (First, Rest...)
                                        const);

template <typename What, size_t Index, typename... Types>
struct find_type_index_helper_t;

/* Holds the result (Index) of the type search, and checks that no other types
   match in the remaining type list.  */
template <typename What, size_t Index, typename... Types>
struct find_type_index_helper_result_t
{
  static constexpr size_t value = Index;
  static_assert (find_type_index_helper_t<What, Index, Types...>::value == -1,
                 "more than one match in type list");
};

/* Partial specialization for not match found.  */
template <typename What, size_t Index>
struct find_type_index_helper_t<What, Index>
{
  static constexpr size_t value = -1;
};

/* Partial specialization for recursive search.  */
template <typename What, size_t Index, typename FirstType, typename... Rest>
struct find_type_index_helper_t<What, Index, FirstType, Rest...>
{
  static constexpr size_t value = std::conditional<
      std::is_same<What, FirstType>::value,
      find_type_index_helper_result_t<What, Index, Rest...>,
      find_type_index_helper_t<What, Index + 1, Rest...>>::type::value;
};

} /* namespace detail */

namespace utils
{

template <typename Integral = uint64_t>
constexpr Integral
bit_mask (int first, int last)
{
  dbgapi_assert (last >= first && "invalid argument");
  size_t num_bits = last - first + 1;
  return ((num_bits >= sizeof (Integral) * 8)
              ? ~Integral{ 0 } /* num_bits exceed the size of Integral */
              : ((Integral{ 1 } << num_bits) - 1))
         << first;
}

template <typename Integral>
constexpr int
bit_count (Integral x)
{
  /* Count the number of one bits using sideways additions. Works for integral
     types up to 128-bit wide.  */
  static_assert (sizeof (Integral) <= 16, "Integral type is too wide");
  using T = typename std::conditional<sizeof (Integral) >= sizeof (uint32_t),
                                      Integral, uint32_t>::type;

  x = x - ((x >> 1) & ~T{ 0 } / 3);
  x = (x & ~T{ 0 } / 15 * 3) + ((x >> 2) & ~T{ 0 } / 15 * 3);
  x = (x + (x >> 4)) & ~T{ 0 } / 255 * 15;
  return (x * (~T{ 0 } / 255)) >> (sizeof (T) - 1) * 8;
}

template <typename Integral>
constexpr typename std::make_unsigned<Integral>::type
zero_extend (Integral x, int width)
{
  return x & bit_mask (0, width - 1);
}

template <typename Integral>
constexpr typename std::make_signed<Integral>::type
sign_extend (Integral x, int width)
{
  Integral sign_mask = bit_mask (width - 1, sizeof (Integral) * 8 - 1);
  return (zero_extend (x, width) ^ sign_mask) - sign_mask;
}

/* Extract bits [last:first] from t.  */
template <typename Integral>
constexpr Integral
bit_extract (Integral x, int first, int last)
{
  return (x >> first) & bit_mask<Integral> (0, last - first);
}

template <typename Integral>
constexpr bool
is_power_of_two (Integral x)
{
  return x && (x & (x - 1)) == 0;
}

template <typename Integral>
constexpr Integral
align_down (Integral x, int alignment)
{
  dbgapi_assert (is_power_of_two (alignment));
  return x & -alignment;
}

template <typename Integral>
constexpr Integral
align_up (Integral x, int alignment)
{
  dbgapi_assert (is_power_of_two (alignment));
  return (x + alignment - 1) & -alignment;
}

/* Holds the type of the Functor's first argument.  */
template <typename Functor> struct first_argument_type
{
  using type
      = decltype (detail::first_argument_type_helper_t (&Functor::operator()));
};

/* Holds the index of the one and only type that matches in the type list.
   This fails to compile if there is no match, or if there are more than one
   match.  */
template <typename What, typename... Types> struct find_type_index_t
{
  static constexpr size_t value
      = detail::find_type_index_helper_t<What, 0, Types...>::value;
  static_assert (value != -1, "type not found");
};

/* Return the tuple element whose type matches Type. This fails to compile if
   there is no match, or if there are more than one match.  */
template <typename What, typename... Types>
constexpr What &
get (std::tuple<Types...> &tuple)
{
  return std::get<find_type_index_t<What, Types...>::value> (tuple);
}

/* Check the size, and copy `value' into the memory pointed to by `ret'.  */
template <typename T>
amd_dbgapi_status_t
get_info (size_t value_size, void *ret, const T &value)
{
  if (value_size != sizeof (T))
    return AMD_DBGAPI_STATUS_ERROR_INVALID_ARGUMENT_SIZE;

  memcpy (ret, &value, sizeof (T));
  return AMD_DBGAPI_STATUS_SUCCESS;
}

template <>
amd_dbgapi_status_t get_info (size_t value_size, void *ret,
                              const std::string &value);

template <typename T>
amd_dbgapi_status_t get_info (size_t value_size, void *ret,
                              const std::vector<T> &value);

template <typename Object>
amd_dbgapi_status_t get_handle_list (amd_dbgapi_process_id_t process_id,
                                     size_t *count,
                                     typename Object::handle_type **objects,
                                     amd_dbgapi_changed_t *changed);

/* Minimal implementation of std::optional. std::optional requires C++17.  */
template <typename T> class optional
{
  optional (const optional &) = delete;
  optional (optional &&) = delete;
  optional &operator= (const optional &) = delete;
  optional &operator= (optional &&) = delete;

public:
  optional () : m_dummy () {}

  ~optional ()
  {
    if (m_instantiated)
      m_object.~T ();
  }

  template <typename... Args> T &emplace (Args &&... args)
  {
    /* Only support setting to a value once, and do not support setting back to
       undefined.  */
    dbgapi_assert (!m_instantiated && "not supported");

    new (&m_object) T (std::forward<Args> (args)...);
    m_instantiated = true;
    return m_object;
  }

private:
  bool m_instantiated{ false };
  union
  {
    struct
    {
    } m_dummy;
    T m_object;
  };
};

} /* namespace utils */

template <typename T, bool = std::is_enum<T>::value> struct is_flag;
template <typename T> struct is_flag<T, true> : std::false_type
{
};

/* To enable bitwise operators on enum classes use:
   template <> struct is_flag<my_enum_class> : std::true_type;
*/

template <typename T>
typename std::enable_if<is_flag<T>::value, bool>::type operator! (T flag)
{
  using t = typename std::underlying_type<T>::type;
  return static_cast<t> (flag) == t{};
}

template <typename T>
typename std::enable_if<is_flag<T>::value, T>::type
operator| (T lhs, T rhs)
{
  using t = typename std::underlying_type<T>::type;
  return static_cast<T> (static_cast<t> (lhs) | static_cast<t> (rhs));
}

template <typename T>
typename std::enable_if<is_flag<T>::value, T>::type &
operator|= (T &lhs, T rhs)
{
  return lhs = lhs | rhs;
}

template <typename T>
typename std::enable_if<is_flag<T>::value, T>::type operator& (T lhs, T rhs)
{
  using t = typename std::underlying_type<T>::type;
  return static_cast<T> (static_cast<t> (lhs) & static_cast<t> (rhs));
}

template <typename T>
typename std::enable_if<is_flag<T>::value, T>::type &
operator&= (T &lhs, T rhs)
{
  return lhs = lhs & rhs;
}

/* A monotonic counter with wrap-around detection. Type must be an integral
   unsigned type. The WrapAroundCheck functor returns true if the counter has
   wrapped around. The default functor is (new_value > old_value).  */
template <typename Type, typename WrapAroundCheck = std::greater<Type>>
class monotonic_counter_t
{
  static_assert (std::is_integral<Type>::value
                     && std::is_unsigned<Type>::value,
                 "Type is not an unsigned integral type");

public:
  /* Construct a new monotonic counter.  */
  monotonic_counter_t (Type value) : m_value (value) {}

  /* Prefix and postfix increment operators.  */
  monotonic_counter_t &operator++ ()
  {
    Type old_value = m_value++;
    if (WrapAroundCheck () (old_value, m_value))
      error ("monotonic counter wrapped around");

    return *this;
  }
  monotonic_counter_t operator++ (int)
  {
    monotonic_counter_t counter (m_value);
    ++(*this);
    return counter;
  }

  /* Default cast operator.  */
  operator Type () const { return m_value; }

private:
  Type m_value{ 0 };
};

class pipe_t
{
public:
  bool open ();
  void close ();

  /* Return true if the pipe is valid and ready for use.  */
  bool is_valid () const { return m_pipe_fd[0] != -1 && m_pipe_fd[1] != -1; }

  /* Return the read-end file descriptor of the pipe.  */
  file_desc_t read_fd () const
  {
    dbgapi_assert (is_valid () && "this pipe is not valid");
    return m_pipe_fd[0];
  }

  /* Return the write-end file descriptor of the pipe.  */
  file_desc_t write_fd () const
  {
    dbgapi_assert (is_valid () && "this pipe is not valid");
    return m_pipe_fd[1];
  }

  /* Write a single char, '+', to the pipe.  Return 0 if successful, errno
     otherwise.  */
  int mark ();

  /* Consume all the data in the pipe.  Return 0 if successful, errno
     otherwise.  */
  int flush ();

private:
  file_desc_t m_pipe_fd[2]{ -1, -1 };
};

} /* namespace dbgapi */
} /* namespace amd */

#endif /* _AMD_DBGAPI_UTILS_H */
