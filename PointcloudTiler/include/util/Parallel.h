#pragma once

#include "Algorithm.h"
#include "TaskSystem.h"

namespace parallel {

template<typename InputIterator, typename Func>
void
for_each(InputIterator first, InputIterator last, Func func, TaskSystem& task_system)
{
  const auto distance = std::distance(first, last);
  if (distance <= task_system.concurrency()) {
    std::for_each(first, last, func);
    return;
  }

  const auto chunks = split_range_into_chunks(task_system.concurrency(), first, last);

  std::vector<async::Awaitable<void>> awaitables;
  awaitables.reserve(task_system.concurrency());
  std::transform(std::begin(chunks),
                 std::end(chunks),
                 std::back_inserter(awaitables),
                 [&task_system, &func](const auto& chunk) {
                   return task_system.push(
                     [=]() { std::for_each(chunk.first, chunk.second, func); });
                 });

  async::all(std::move(awaitables)).await();
}

template<typename InputIterator, typename RandomAccessIterator, typename Func>
void
transform(InputIterator first,
          InputIterator last,
          RandomAccessIterator result,
          Func func,
          TaskSystem& task_system)
{
  const auto distance = static_cast<size_t>(std::abs(std::distance(first, last)));
  if (distance <= task_system.concurrency()) {
    std::transform(first, last, result, func);
    return;
  }

  const auto input_chunks = split_range_into_chunks(task_system.concurrency(), first, last);
  const auto result_chunks =
    split_range_into_chunks(task_system.concurrency(), result, result + distance);

  std::vector<async::Awaitable<void>> awaitables;
  awaitables.reserve(task_system.concurrency());
  for (size_t idx = 0; idx < task_system.concurrency(); ++idx) {
    const auto [in_chunk_begin, in_chunk_end] = input_chunks[idx];
    const auto result_chunk_begin = result_chunks[idx].first;
    awaitables.push_back(task_system.push(
      [=]() { std::transform(in_chunk_begin, in_chunk_end, result_chunk_begin, func); }));
  }

  async::all(std::move(awaitables)).await();
}
}