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

#include "defs.h"

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

template <typename Type, typename = int> struct has_is_valid : std::false_type
{
};
template <typename Type>
struct has_is_valid<Type, decltype ((void)&Type::is_valid, 0)> : std::true_type
{
};

/* Call is_valid () if Object implements it, return true otherwise.  */
template <typename Object>
constexpr typename std::enable_if<has_is_valid<Object>::value, bool>::type
is_valid (Object &object)
{
  return object.is_valid ();
}
template <typename Object>
constexpr typename std::enable_if<has_is_valid<Object>::value, bool>::type
is_valid (const Object &object)
{
  return object.is_valid ();
}
template <typename Object>
constexpr typename std::enable_if<!has_is_valid<Object>::value, bool>::type
is_valid (const Object &object)
{
  return true;
}

} /* namespace detail */

/* A handle type is a type that has an unsigned integral handle data member. */
template <typename Type, typename = int>
struct is_handle_type : std::false_type
{
};

template <typename Type>
struct is_handle_type<Type, decltype ((void)Type::handle, 0)>
    : std::integral_constant<
          bool, std::is_integral<decltype (Type::handle)>::value
                    && std::is_unsigned<decltype (Type::handle)>::value>
{
};

/* A handle object type is a class that implements the id() member function,
   and the return type of the id() function is a handle type.  */
template <typename Type, typename = int>
struct is_handle_object_type : std::false_type
{
};

template <typename Type>
struct is_handle_object_type<Type, decltype ((void)&Type::id, 0)>
    : std::integral_constant<bool,
                             is_handle_type<typename std::result_of<decltype (
                                 &Type::id) (Type)>::type>::value>
{
};

/* Hash function for handle types.  */
template <typename Handle> struct hash
{
  typename std::enable_if<is_handle_type<Handle>::value, std::size_t>::type
  operator() (const Handle &rhs) const noexcept
  {
    return std::hash<decltype (rhs.handle)>{}(rhs.handle);
  }
};

/* A set container type that holds objects that are referenced using handles.
   If the Object type supports the is_valid concept, then when creating objects
   an error is reported if the constructor creates an object that is not valid.
 */
template <typename Object> class handle_object_set_t
{
  static_assert (is_handle_object_type<Object>::value,
                 "Object is not a handle object");

  using handle_type =
      typename std::result_of<decltype (&Object::id) (Object)>::type;

  using map_type = std::unordered_map<handle_type, Object, hash<handle_type>>;

public:
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
      self_type i = *this;
      ++m_it;
      return i;
    }
    self_type operator++ (int)
    {
      ++m_it;
      return *this;
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
      self_type i = *this;
      ++m_it;
      return i;
    }
    self_type operator++ (int)
    {
      ++m_it;
      return *this;
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
    auto object_it = m_map.find (id);
    return (object_it != m_map.end ()) ? &object_it->second : nullptr;
  }

  const Object *find (handle_type id) const
  {
    auto object_it = m_map.find (id);
    return (object_it != m_map.end ()) ? &object_it->second : nullptr;
  }

  template <typename Functor> Object *find_if (Functor predicate)
  {
    auto it = std::find_if (m_map.begin (), m_map.end (),
                            [=] (const typename map_type::value_type &value) {
                              return bool{ predicate (value.second) };
                            });
    return it != m_map.end () ? &it->second : nullptr;
  }

  template <typename Functor> const Object *find_if (Functor predicate) const
  {
    auto it = std::find_if (m_map.begin (), m_map.end (),
                            [=] (const typename map_type::value_type &value) {
                              return bool{ predicate (value.second) };
                            });
    return it != m_map.end () ? &it->second : nullptr;
  }

  iterator begin () { return iterator (m_map.begin ()); }
  const_iterator begin () const { return const_iterator (m_map.cbegin ()); }

  iterator end () { return iterator (m_map.end ()); }
  const_iterator end () const { return const_iterator (m_map.cend ()); }

  range_t range () { return range_t{ this }; }
  const_range_t range () const { return const_range_t{ this }; }

  size_t size () const { return m_map.size (); }

  bool reset_changed ()
  {
    bool ret = m_changed;
    m_changed = false;
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

  if (!detail::is_valid (result.first->second))
    {
      m_map.erase (result.first);
      error ("object is not valid");
    }

  m_changed = true;
  return result.first->second;
}

namespace detail
{

template <typename Handle, typename...> struct object_type_helper_t
{
  using type = void;
};

template <typename Handle, typename FirstObject, typename... RemainingObjects>
struct object_type_helper_t<Handle, FirstObject, RemainingObjects...>
{
  using FirstObjectHandle =
      typename std::result_of<decltype (&FirstObject::id) (FirstObject)>::type;

  using type = typename std::conditional<
      std::is_same<Handle, FirstObjectHandle>::value, FirstObject,
      typename object_type_helper_t<Handle, RemainingObjects...>::type>::type;
};

} /* namespace detail */

template <typename...> class handle_object_set_tuple_t
{
public:
  void clear () {}
};

/* A tuple of distinct typed elements that can be accessed by the element type.
   TODO: maybe move this as a detail of process_t, since it is the only one
   needing this.  */
template <typename Head, typename... Tail>
class handle_object_set_tuple_t<Head, Tail...>
{
public:
  /* Helper struct to find the Object type whose handle type is equivalent
     to Handle.  */
  template <typename Handle> struct find_object_type_from_handle
  {
    using type =
        typename detail::object_type_helper_t<Handle, Head, Tail...>::type;
  };

  /* Getters if Object and Head types are equivalent.  */
  template <typename Object>
  typename std::enable_if<std::is_same<Object, Head>::value,
                          handle_object_set_t<Object>>::type &
  get ()
  {
    return m_head;
  }
  template <typename Object>
  const typename std::enable_if<std::is_same<Object, Head>::value,
                                handle_object_set_t<Object>>::type &
  get () const
  {
    return m_head;
  }

  /* Getters if Object and Head types are not equivalent.  */
  template <typename Object>
  typename std::enable_if<!std::is_same<Object, Head>::value,
                          handle_object_set_t<Object>>::type &
  get ()
  {
    return m_tail.template get<Object> ();
  }
  template <typename Object>
  const typename std::enable_if<!std::is_same<Object, Head>::value,
                                handle_object_set_t<Object>>::type &
  get () const
  {
    return m_tail.template get<Object> ();
  }

  void clear ()
  {
    m_head.clear ();
    m_tail.clear ();
  }

private:
  handle_object_set_t<Head> m_head;
  handle_object_set_tuple_t<Tail...> m_tail;
};

} /* namespace dbgapi */
} /* namespace amd */

template <typename Handle>
typename std::enable_if<amd::dbgapi::is_handle_type<Handle>::value, bool>::type
operator== (const Handle &lhs, const Handle &rhs)
{
  return lhs.handle == rhs.handle;
}

template <typename Handle>
typename std::enable_if<amd::dbgapi::is_handle_type<Handle>::value, bool>::type
operator!= (const Handle &lhs, const Handle &rhs)
{
  return !(lhs == rhs);
}

template <typename Handle>
typename std::enable_if<amd::dbgapi::is_handle_type<Handle>::value, bool>::type
operator! (const Handle &handle)
{
  return handle == Handle{};
}

#endif /* _AMD_DBGAPI_HANDLE_OBJECT_H */
