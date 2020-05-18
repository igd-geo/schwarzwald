#pragma once

#include <algorithm>
#include <iterator>
#include <vector>

#include "containers/Range.h"

/*
 * Like std::stable_partition, but the predicate can operate on a whole
 * subrange instead of a single entry at a time. This enables partitioning
 * with predicates that have to 'reconsider' a previous decision based on
 * a later entry.
 *
 * The predicate is called with the current iterator and end iterator and has
 * to return an iterator pair where the first entry points to the selected element
 * and the second entry is the new begin iterator from which the partitioning should
 * continue. If no match is found by the predicate, both iterators point to the next
 * entry that shall be processed.
 **/
template<typename Iter, typename Pred>
Iter
stable_partition_with_jumps(const Iter begin, const Iter end, Pred pred)
{
  const auto count = std::distance(begin, end);
  if (!count)
    return end;

  using Value_t = typename std::iterator_traits<Iter>::value_type;
  const auto tmp_buffer = reinterpret_cast<Value_t*>(malloc(sizeof(Value_t) * count));
  const auto tmp_buffer_begin = tmp_buffer;
  const auto tmp_buffer_rbegin = std::make_reverse_iterator(tmp_buffer_begin + count);

  // Selected elements are copied in forward sorted order to the beginning of the buffer,
  // unselected elements are copied in reverse sorted order to the end of the buffer. This way,
  // the buffer grows from front and back until eventually both iterators meet
  auto selected_insert_position = tmp_buffer_begin;
  auto unselected_insert_position = tmp_buffer_rbegin;

  auto current = begin;
  while (current != end) {
    const auto selected_and_next = pred(current, end);
    const auto selected = selected_and_next.first;
    const auto next = selected_and_next.second;
    assert(next != current);

    if (selected == next) {
      // Everything is unselected, copy everything to the end of the tmp_buffer
      unselected_insert_position = std::move(current, next, unselected_insert_position);
    } else {
      // Copy everything prior to the selected element to the tmp_buffer, then the selected
      // element to the
      unselected_insert_position = std::move(current, selected, unselected_insert_position);
      selected_insert_position = std::move(selected, selected + 1, selected_insert_position);
      unselected_insert_position = std::move(selected + 1, next, unselected_insert_position);
    }

    current = next;
  }

  // Copy everything back into the original range
  const auto pivot_point = std::move(tmp_buffer_begin, selected_insert_position, begin);
  std::move(tmp_buffer_rbegin, unselected_insert_position, pivot_point);

  free(tmp_buffer);

  return pivot_point;
}

/**
 * Split a range into equally-sized chunks. Each chunk is expressed with a begin
 * and end iterator. If 'num_chunks' does not equally divide the size of the range,
 * the remainder will be in the last chunk. 'num_chunks' has to be >= the size of
 * the range
 */
template<typename Iter>
std::vector<std::pair<Iter, Iter>>
split_range_into_chunks(size_t num_chunks, Iter begin, Iter end)
{
  const auto num_elements = std::distance(begin, end);
  assert(num_elements >= static_cast<ptrdiff_t>(num_chunks));
  const auto chunk_size = num_elements / num_chunks;
  std::vector<std::pair<Iter, Iter>> chunks;
  chunks.reserve(num_chunks);
  for (size_t idx = 0; idx < num_chunks - 1; ++idx) {
    chunks.push_back(std::make_pair(begin + (idx * chunk_size), begin + (idx + 1) * chunk_size));
  }
  chunks.push_back(std::make_pair(begin + (num_chunks - 1) * chunk_size, end));
  return chunks;
}

#pragma region range_algorithms

/**
 * Merge the given range of ranges into a single range. This is the N-ary equivalent
 * to std::merge. It requires that the input ranges are at least forward iterators.
 *
 * ForwardIter should be dereferencable to util::Range<T> for some T
 */
template<typename ForwardIter, typename OutputIter, typename Comp>
void
merge_ranges(util::Range<ForwardIter> in_ranges, util::Range<OutputIter> out_range, Comp comparator)
{
  using InputRangeType = std::decay_t<decltype(*std::declval<ForwardIter>())>;
  std::vector<InputRangeType> remaining_ranges = { std::begin(in_ranges), std::end(in_ranges) };

  /**
   * Looks for the range with the lowest next element in 'remaining_ranges'. If such a range
   * exists, its front element is stored in 'element' and the range is moved to the next element
   */
  const auto get_and_increment_lowest_head = [&comparator](std::vector<InputRangeType>& heads,
                                                           auto& element) {
    auto iter_to_lowest_head = std::begin(heads);
    for (auto iter = std::begin(heads) + 1; iter != std::end(heads); ++iter) {
      if (!iter->size())
        continue;

      if (comparator(*std::begin(*iter), *std::begin(*iter_to_lowest_head))) {
        iter_to_lowest_head = iter;
      }
    }

    if (iter_to_lowest_head == std::end(heads))
      return;

    auto& lowest_range = *iter_to_lowest_head;
    element = *std::begin(lowest_range);
    lowest_range = lowest_range.skip(1);
  };

  for (auto& out : out_range) {
    get_and_increment_lowest_head(remaining_ranges, out);
  }
}

#pragma endregion