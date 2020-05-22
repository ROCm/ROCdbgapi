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

#ifndef _AMD_DBGAPI_HANDLE_OBJECT_H
#define _AMD_DBGAPI_HANDLE_OBJECT_H 1

#include "utils.h"

#include <algorithm>
#include <unordered_map>

namespace amd
{
namespace dbgapi
{

template <typename Handle>
constexpr Handle
null_id ()
{
  return Handle{ decltype (Handle::handle){ 0 } };
}

constexpr int HANDLE_SENTINEL_COUNT = 5;

template <typename Handle, int N = 0>
constexpr Handle
sentinel_id ()
{
  static_assert (N < HANDLE_SENTINEL_COUNT, "invalid parameter");
  return Handle{ ~decltype (Handle::handle){ N } };
}

namespace detail
{

/* Base class for all handle objects, i.e. agents_t, queue_t, wave_t, ...
   Handle objects should only by allocated within their handle object sets,
   should not be copied nor moved.  */
template <typename Handle> class handle_object
{
public:
  using handle_type = Handle;

protected:
  explicit handle_object (const handle_type &id) : m_id (id) {}
  ~handle_object () = default;

  /* Disable copies and move assignments.  */
  handle_object (const handle_object &) = delete;
  handle_object (handle_object &&) = delete;
  handle_object &operator= (const handle_object &) = delete;
  handle_object &operator= (handle_object &&) = delete;

  /* handle_objects are not heap objects.  */
  static void *operator new (std::size_t) = delete;
  static void *operator new[] (std::size_t) = delete;
  static void operator delete (void *) = delete;
  static void operator delete[] (void *) = delete;

public:
  handle_type id () const { return m_id; }

private:
  handle_type const m_id;
};

/* Wrap-around check functor for handles. An error is thrown if the
   handle value reaches ~0 as it is reserved for a mark object.  */
template <typename Type> struct handle_has_wrapped_around
{
  bool operator() (const Type &old_value, const Type &new_value) const
  {
    return new_value == ~Type{ HANDLE_SENTINEL_COUNT - 1 };
  }
};

/* Type of Object::is_valid ()  */
template <typename Object>
using is_valid_t = decltype (std::declval<Object> ().is_valid ());

/* Default is_valid_helper_t::is_valid always returns true.  */
template <typename AlwaysVoid, typename Object> struct is_valid_helper_t
{
  static_assert (std::is_same<AlwaysVoid, void>::value, "must be void");
  static constexpr bool is_valid (Object &&object) { return true; }
};

/* Partial specialization calls is_valid () if Object implements it.   */
template <typename Object>
struct is_valid_helper_t<
    std::enable_if_t<utils::is_detected_v<is_valid_t, Object>, void>, Object>
{
  static constexpr bool is_valid (Object &&object)
  {
    return object.is_valid ();
  }
};

} /* namespace detail */

/* TODO: This could be simplified with a if constexpr (is_detected...).  */
template <typename Object>
constexpr bool
is_valid (Object &&object)
{
  return detail::is_valid_helper_t<void, Object>::is_valid (
      std::forward<Object> (object));
}

/* A handle type is a type that has an unsigned integral handle data member. */

namespace detail
{

/* Type of Handle::handle  */
template <typename Type>
using id_handle_t = decltype (std::declval<Type> ().handle);

} /* namespace detail */

template <typename Type>
constexpr bool is_handle_type_v
    = std::is_integral<utils::detected_t<detail::id_handle_t, Type>>::value &&
        std::is_unsigned<utils::detected_t<detail::id_handle_t, Type>>::value;

/* Hash function for handle types.  */
template <typename Handle, std::enable_if_t<is_handle_type_v<Handle>, int> = 0>
struct hash
{
  std::size_t operator() (const Handle &rhs) const noexcept
  {
    return std::hash<decltype (rhs.handle)>{}(rhs.handle);
  }
};

/* A handle object type is a class that implements the id() member function,
   and the return type of the id() function is a handle type.  */

namespace detail
{

/* Type of Object::id ()  */
template <typename Type>
using object_id_t = decltype (std::declval<Type> ().id ());

} /* namespace detail */

template <typename Type>
constexpr bool is_handle_object_type_v
    = is_handle_type_v<utils::detected_t<detail::object_id_t, Type>>;

/* A set container type that holds objects that are referenced using handles.
   If the Object type supports the is_valid concept, then when creating objects
   an error is reported if the constructor creates an object that is not valid.
 */
template <typename Object> class handle_object_set_t
{
  static_assert (is_handle_object_type_v<Object>,
                 "Object is not a handle object");

public:
  using object_type = Object;
  using handle_type = utils::detected_t<detail::object_id_t, Object>;
  using map_type = std::unordered_map<handle_type, Object, hash<handle_type>>;

  class iterator
  {
  public:
    using self_type = iterator;
    using value_type = Object;
    using reference = Object &;
    using pointer = Object *;
    using iterator_category = std::forward_iterator_tag;

    iterator (typename map_type::iterator it) : m_it (it) {}
    typename map_type::iterator get () const { return m_it; }
    self_type operator++ ()
    {
      ++m_it;
      return *this;
    }
    self_type operator++ (int)
    {
      self_type i = *this;
      ++m_it;
      return i;
    }
    reference operator* () { return m_it->second; }
    pointer operator-> () { return &m_it->second; }
    bool operator== (const self_type &rhs) { return m_it == rhs.m_it; }
    bool operator!= (const self_type &rhs) { return m_it != rhs.m_it; }

  private:
    typename map_type::iterator m_it;
  };

  class const_iterator
  {
  public:
    using self_type = const_iterator;
    using value_type = Object;
    using const_reference = const Object &;
    using const_pointer = const Object *;
    using iterator_category = std::forward_iterator_tag;

    const_iterator (typename map_type::const_iterator it) : m_it (it) {}
    typename map_type::const_iterator get () const { return m_it; }
    self_type operator++ ()
    {
      ++m_it;
      return *this;
    }
    self_type operator++ (int)
    {
      self_type i = *this;
      ++m_it;
      return i;
    }
    const_reference operator* () const { return m_it->second; }
    const_pointer operator-> () const { return &m_it->second; }
    bool operator== (const self_type &rhs) const { return m_it == rhs.m_it; }
    bool operator!= (const self_type &rhs) const { return m_it != rhs.m_it; }

  private:
    typename map_type::const_iterator m_it;
  };

  class range_t
  {
  public:
    explicit range_t (handle_object_set_t *objects) : m_objects (objects) {}
    iterator begin () { return iterator (m_objects->begin ()); }
    iterator end () { return iterator (m_objects->end ()); }

  private:
    handle_object_set_t *m_objects;
  };

  class const_range_t
  {
  public:
    explicit const_range_t (const handle_object_set_t *objects)
        : m_objects (objects)
    {
    }
    const_iterator begin () { return const_iterator (m_objects->begin ()); }
    const_iterator end () { return const_iterator (m_objects->end ()); }

  private:
    const handle_object_set_t *m_objects;
  };

  /* Delete copy and move.  */
  handle_object_set_t (const handle_object_set_t &) = delete;
  handle_object_set_t &operator= (const handle_object_set_t &) = delete;
  handle_object_set_t (handle_object_set_t &&) = delete;
  handle_object_set_t &operator= (handle_object_set_t &&) = delete;

  /* Default constructor.  */
  handle_object_set_t () = default;

  template <typename... Args>
  inline Object &create_object (handle_type id, Args &&... args);

  template <typename... Args> Object &create_object (Args &&... args)
  {
    return create_object (handle_type{ 0 }, std::forward<Args> (args)...);
  }

  void destroy (Object *object)
  {
    auto object_it = m_map.find (object->id ());
    dbgapi_assert (object_it != m_map.end ());

    m_changed = true;
    m_map.erase (object_it);
  }

  iterator destroy (iterator object_it)
  {
    m_changed = true;
    typename map_type::iterator it = object_it.get ();
    return iterator (m_map.erase (it));
  }

  Object *find (handle_type id)
  {
    auto it = m_map.find (id);
    if (it == m_map.end () || !is_valid (it->second))
      return nullptr;
    return &it->second;
  }

  const Object *find (handle_type id) const
  {
    auto it = m_map.find (id);
    if (it == m_map.end () || !is_valid (it->second))
      return nullptr;
    return &it->second;
  }

  template <typename Functor> Object *find_if (Functor predicate)
  {
    auto it = std::find_if (m_map.begin (), m_map.end (),
                            [=] (const typename map_type::value_type &value) {
                              return bool{ predicate (value.second) };
                            });
    if (it == m_map.end () || !is_valid (it->second))
      return nullptr;
    return &it->second;
  }

  template <typename Functor> const Object *find_if (Functor predicate) const
  {
    auto it = std::find_if (m_map.begin (), m_map.end (),
                            [=] (const typename map_type::value_type &value) {
                              return bool{ predicate (value.second) };
                            });
    if (it == m_map.end () || !is_valid (it->second))
      return nullptr;
    return &it->second;
  }

  iterator begin () { return iterator (m_map.begin ()); }
  const_iterator begin () const { return const_iterator (m_map.cbegin ()); }

  iterator end () { return iterator (m_map.end ()); }
  const_iterator end () const { return const_iterator (m_map.cend ()); }

  range_t range () { return range_t{ this }; }
  const_range_t range () const { return const_range_t{ this }; }

  size_t size () const { return m_map.size (); }

  bool set_changed (bool changed)
  {
    bool ret = m_changed;
    m_changed = changed;
    return ret;
  }

  void clear ()
  {
    if (!m_map.empty ())
      {
        m_changed = true;
        m_map.clear ();
      }
  }

private:
  /* Map holding the objects, keyed by object::id ()'s return type. */
  map_type m_map;

  /* Counter to assign ids to new objects. We start all the counters at 1,
     as 0 is the null object handle.  */
  monotonic_counter_t<
      decltype (handle_type::handle),
      detail::handle_has_wrapped_around<decltype (handle_type::handle)>>
      m_next_id{ 1 };

  /* Flag to tell if the content of the map has changed.  */
  bool m_changed = false;
};

template <typename Object>
template <typename... Args>
inline Object &
handle_object_set_t<Object>::create_object (handle_type id, Args &&... args)
{
  /* Re-use id if non-null, request a new one otherwise.  This allows us to
     "re-create" objects that were place-holders (e.g. partially initialized
     queue object.  */
  if (!id.handle)
    id.handle = m_next_id++;

  dbgapi_assert (id.handle && "must not be null");

  auto result = m_map.emplace (
      std::piecewise_construct, std::forward_as_tuple (id),
      std::forward_as_tuple (id, std::forward<Args> (args)...));

  if (!result.second)
    error ("could not create new object");

  if (!is_valid (result.first->second))
    {
      m_map.erase (result.first);
      error ("object is not valid");
    }

  m_changed = true;
  return result.first->second;
}

namespace detail
{
/* Helper struct to return the object type associated with a handle type, e.g.
   process_t and amd_dbgapi_process_id_t.  It uses the handle_object_set tuple
   to match a given handle type with the return type of the id () function
   implemented by the tuple elements.  */

template <typename Handle, typename Tuple, size_t I>
struct object_type_from_handle;

template <typename Handle, typename Tuple>
struct object_type_from_handle<Handle, Tuple, 0>
{
  using element_0_type = typename std::tuple_element<0, Tuple>::type;
  using type = typename element_0_type::object_type;
};

template <typename Handle, typename Tuple, size_t I>
struct object_type_from_handle
{
  using element_I_type = typename std::tuple_element<I, Tuple>::type;
  using type = std::conditional_t<
      std::is_same<Handle, typename element_I_type::handle_type>::value,
      typename element_I_type::object_type,
      typename object_type_from_handle<Handle, Tuple, I - 1>::type>;
};

} /* namespace detail */

template <typename Handle, typename Tuple,
          size_t I = std::tuple_size<Tuple>::value - 1>
using object_type_from_handle_t =
    typename detail::object_type_from_handle<Handle, Tuple, I>::type;

} /* namespace dbgapi */
} /* namespace amd */

template <typename Handle,
          std::enable_if_t<amd::dbgapi::is_handle_type_v<Handle>, int> = 0>
bool
operator== (const Handle &lhs, const Handle &rhs)
{
  return lhs.handle == rhs.handle;
}

template <typename Handle,
          std::enable_if_t<amd::dbgapi::is_handle_type_v<Handle>, int> = 0>
bool
operator!= (const Handle &lhs, const Handle &rhs)
{
  return !(lhs == rhs);
}

template <typename Handle,
          std::enable_if_t<amd::dbgapi::is_handle_type_v<Handle>, int> = 0>
bool operator! (const Handle &handle)
{
  return handle == Handle{};
}

#endif /* _AMD_DBGAPI_HANDLE_OBJECT_H */
