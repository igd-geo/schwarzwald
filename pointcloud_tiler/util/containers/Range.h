#pragma once

#include <algorithm>
#include <iterator>
#include <type_traits>

namespace util {

template<typename Iter>
struct Range
{

  Range()
    : _begin(Iter{})
    , _end(Iter{})
  {}
  Range(Iter begin, Iter end)
    : _begin(begin)
    , _end(end)
  {}

  Range(const Range&) = default;
  Range(Range&&) = default;

  Range& operator=(const Range&) = default;
  Range& operator=(Range&&) = default;

  Iter begin() const { return _begin; }
  Iter end() const { return _end; }

  size_t size() const { return static_cast<size_t>(std::distance(_begin, _end)); }

  template<typename Func>
  auto accumulate(Func binary_op) const
  {
    using ValType = typename std::iterator_traits<Iter>::value_type;
    return std::accumulate(_begin, _end, ValType{}, binary_op);
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
  using IterType = std::remove_cv_t<decltype(std::begin(container))>;
  return Range<IterType>{ std::begin(container), std::end(container) };
}

template<typename Container>
auto
range(const Container& container)
{
  using IterType = std::remove_cv_t<decltype(std::begin(container))>;
  return Range<IterType>{ std::begin(container), std::end(container) };
}

}