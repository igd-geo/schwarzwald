#pragma once

#include "types/Typelist.h"

#include <cista/reflection/arity.h>
#include <cista/reflection/to_tuple.h>
#include <type_traits>

namespace refl {

namespace detail {

template<typename T>
struct TupleToTypeList;

template<typename... Ts>
struct TupleToTypeList<std::tuple<Ts...>> {
  using TypeList = meta::Typelist<std::remove_reference_t<Ts>...>;
};

template <typename T>
inline auto to_tuple_by_val(T t) {
  return cista::to_tuple(t);
}

template<typename T>
inline constexpr decltype(auto)
members_of_aggregate()
{
  static_assert(std::is_aggregate_v<T>, "T must be aggregate type");
  static_assert(std::is_default_constructible_v<T>, "T must be default constructible");
  using TupleType = decltype(to_tuple_by_val(T{}));
  using TypeListType = typename TupleToTypeList<TupleType>::TypeList;
  return TypeListType{};

}
}

/**
 * Returns a Typelist containing the types of the members of type T in the order that they are
 * declared
 */
template<typename T>
inline constexpr decltype(auto)
members_of()
{
  if constexpr (std::is_scalar_v<T>) {
    return meta::Typelist<>{};
  } else if constexpr (std::is_aggregate_v<T>) {
    return detail::members_of_aggregate<T>();
  } else {
    static_assert(AlwaysFalse<T>::value,
                  "StaticReflection only supports scalar and aggregate types at the moment");
  }
}

/**
 * Is the given type reflectable using this simple static reflection library?
 */
template<typename T>
inline constexpr bool
is_reflectable()
{
  return std::is_scalar_v<T> || std::is_aggregate_v<T>;
}

}