#pragma once

#include <algorithm>
#include <iterator>
#include <numeric>
#include <type_traits>

namespace util {

template<typename Iter>
struct Range
{

  using iterator = Iter;
  using const_iterator = Iter;
  using value_type = typename std::iterator_traits<Iter>::value_type;
  using iterator_category = typename std::iterator_traits<Iter>::iterator_category;

  static_assert(std::is_assignable_v<std::bidirectional_iterator_tag, iterator_category>,
                "util::Range only works for iterators that are at least bidirectional!");

  constexpr Range()
    : _begin(Iter{})
    , _end(Iter{})
  {}
  constexpr Range(Iter begin, Iter end)
    : _begin(begin)
    , _end(end)
  {}

  constexpr Range(const Range&) = default;
  constexpr Range(Range&&) = default;

  Range& operator=(const Range&) = default;
  Range& operator=(Range&&) = default;

  constexpr Iter begin() const { return _begin; }
  constexpr Iter end() const { return _end; }

  constexpr size_t size() const { return static_cast<size_t>(std::distance(_begin, _end)); }

  decltype(auto) first() const { return *_begin; }

  decltype(auto) last() const { return *(_end - 1); }

  template<typename Func>
  auto accumulate(Func binary_op) const
  {
    using ValType = typename std::iterator_traits<Iter>::value_type;
    return std::accumulate(_begin, _end, ValType{}, binary_op);
  }

  template<typename OutIter, typename UnaryOperation>
  auto transform(OutIter out_begin, UnaryOperation op) const
  {
    return std::transform(_begin, _end, out_begin, op);
  }

  /**
   * Sorts this range using the given comparator (defaults to std::less<>)
   */
  template<typename Comp = std::less<value_type>>
  void sort() const
  {
    std::sort(_begin, _end, Comp{});
  }

  /**
   * Skip the first 'count' elements of this range. 'count' must be >= size()
   */
  constexpr Range skip(const size_t count) const
  {
    assert(size() >= count);
    return { _begin + count, _end };
  }

private:
  Iter _begin, _end;
};

template<typename Container>
auto
range(Container& container)
{
  // TODO This is almost certainly wrong because it doesn't cover all cases. But writing something
  // as trivial as my Range<> thing here that really just adapts a begin and end iterator CORRECTLY
  // requires a Ph.D. in programming language theory, 25 years of professional C++ experience and a
  // couple of months of work, so screw that
  using IterType = typename Container::iterator;
  return Range<IterType>{ std::begin(container), std::end(container) };
}

template<typename Container>
auto
range(const Container& container)
{
  using IterType = typename Container::const_iterator;
  return Range<IterType>{ std::begin(container), std::end(container) };
}

}