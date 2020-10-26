#pragma once

#include <stdint.h>
#include <type_traits>

#include "types/type_util.h"

namespace detail {
/**
 * Error type serving as a termination criterion for UnsignedBits<>
 */
template<unsigned int MaxBits>
struct UndefinedBitsError
{
  static_assert(AlwaysFalseForIntegralTypes<MaxBits>::value,
                "There is no default C++ datatype that can hold the desired number of bits!");
};

}

/**
 * Expands to an unsigned integer datatype that can store at least 'MaxBits' bits
 */
template<unsigned int MaxBits>
using UnsignedBits = std::conditional_t<
  MaxBits <= 8,
  uint8_t,
  std::conditional_t<
    MaxBits <= 16,
    uint16_t,
    std::conditional_t<
      MaxBits <= 32,
      uint32_t,
      std::conditional_t<
        MaxBits <= 64,
        uint64_t,
        std::conditional_t<MaxBits <= 128, __uint128_t, detail::UndefinedBitsError<MaxBits>>>>>>;

namespace detail {

template<unsigned int Bits>
constexpr auto
all_bits_set_helper()
{
  using IntType = UnsignedBits<Bits>;
  constexpr auto shift_bits = sizeof(IntType) * 8 - Bits;
  return static_cast<IntType>((~IntType{ 0 }) >> IntType{ shift_bits });
}
}

/**
 * A bitmask where the first 'Bits' bits are set to one:
 *
 * AllBitsSet<5> = 0b0001'1111
 */
template<unsigned int Bits>
constexpr auto AllBitsSet = detail::all_bits_set_helper<Bits>();
