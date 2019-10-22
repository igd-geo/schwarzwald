#pragma once

#include <algorithm>
#include <iterator>
#include <vector>

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