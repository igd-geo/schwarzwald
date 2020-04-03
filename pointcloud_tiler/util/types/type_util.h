#pragma once

#include <type_traits>

template <class... Ts> struct overloaded : Ts... { using Ts::operator()...; };
template <class... Ts> overloaded(Ts...)->overloaded<Ts...>;

template <typename T> struct AlwaysFalse : std::bool_constant<false> {};