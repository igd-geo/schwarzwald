#pragma once

#include <boost/iterator_adaptors.hpp>
#include <functional>
#include <iterator>
#include <type_traits>

template<typename SourceIter, typename TargetType, typename SourceToTargetConvertFunction>
struct DestructuringIterator
  : boost::iterator_facade<
      DestructuringIterator<SourceIter, TargetType, SourceToTargetConvertFunction>,
      TargetType,
      typename std::iterator_traits<SourceIter>::iterator_category>
{
  DestructuringIterator()
    : _convert_func(nullptr)
  {}
  DestructuringIterator(SourceIter source, SourceToTargetConvertFunction convert_func)
    : _source(source)
    , _convert_func(convert_func)
  {}

private:
  friend class boost::iterator_core_access;

  void increment() { ++_source; }

  void decrement() { --_source; }

  void advance(std::ptrdiff_t n) { std::advance(_source, n); }

  bool equal(DestructuringIterator const& other) const { return _source == other._source; }

  TargetType& dereference() const { return *_source.*_convert_func; }

  std::ptrdiff_t distance_to(DestructuringIterator const& other) const
  {
    return std::distance(_source, other._source);
  }

  SourceIter _source;
  SourceToTargetConvertFunction _convert_func;
};

template<typename SourceIter, typename SourceToTargetConvertFunction>
decltype(auto)
destructure_iterator(SourceIter source, SourceToTargetConvertFunction func)
{
  using TargetType = std::invoke_result_t<SourceToTargetConvertFunction, decltype(*source)>;
  return DestructuringIterator<std::decay_t<SourceIter>, TargetType, SourceToTargetConvertFunction>{
    source, func
  };
}

/**
 * Creates a DestructuringIterator from the SourceIter type using a point-to-member-data
 * TODO This thing is fragile as f**k for sure...
 */
template<typename SourceIter, typename MemberDataType, typename SourceType>
decltype(auto)
member_iterator(SourceIter source, MemberDataType SourceType::*ptr_to_member_data)
{
  using TargetType = std::invoke_result_t<MemberDataType SourceType::*, decltype(*source)>;
  return DestructuringIterator<std::decay_t<SourceIter>, TargetType, MemberDataType SourceType::*>{
    source, ptr_to_member_data
  };
}