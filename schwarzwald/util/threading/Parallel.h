#pragma once

#include "algorithms/Algorithm.h"
#include "algorithms/Strings.h"
#include "threading/TaskSystem.h"

#include <taskflow/taskflow.hpp>

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

template<typename InputIterator, typename Func, typename Taskflow>
std::pair<tf::Task, tf::Task>
for_each(InputIterator first,
         InputIterator last,
         Func func,
         Taskflow& taskflow,
         size_t concurrency,
         std::string name_prefix = "")
{
  const auto distance = static_cast<size_t>(std::distance(first, last));
  if (distance <= concurrency) {
    std::for_each(first, last, func);
    return {};
  }

  const auto chunks = split_range_into_chunks(concurrency, first, last);

  const auto name_tasks = !name_prefix.empty();

  auto begin_task = taskflow.placeholder();
  auto end_task = taskflow.placeholder();

  if (name_tasks) {
    begin_task.name(util::concat(name_prefix, "_begin"));
    end_task.name(util::concat(name_prefix, "_end"));
  }

  for (size_t idx = 0; idx < concurrency; ++idx) {
    const auto [chunk_begin, chunk_end] = chunks[idx];

    auto worker_task = taskflow.emplace([=]() { std::for_each(chunk_begin, chunk_end, func); });
    begin_task.precede(worker_task);
    worker_task.precede(end_task);

    if (name_tasks) {
      worker_task.name(util::concat(name_prefix, "_", idx));
    }
  }

  return std::make_pair(begin_task, end_task);
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

template<typename InputIterator, typename RandomAccessIterator, typename Func, typename Taskflow>
std::pair<tf::Task, tf::Task>
transform(InputIterator first,
          InputIterator last,
          RandomAccessIterator result,
          Func func,
          Taskflow& taskflow,
          size_t concurrency,
          std::string name_prefix = "")
{
  const auto distance = static_cast<size_t>(std::abs(std::distance(first, last)));
  if (distance <= concurrency) {
    std::transform(first, last, result, func);
    return {};
  }

  const auto input_chunks = split_range_into_chunks(concurrency, first, last);
  const auto result_chunks = split_range_into_chunks(concurrency, result, result + distance);

  const auto name_tasks = !name_prefix.empty();

  auto begin_task = taskflow.placeholder();
  auto end_task = taskflow.placeholder();

  if (name_tasks) {
    begin_task.name(util::concat(name_prefix, "_begin"));
    end_task.name(util::concat(name_prefix, "_end"));
  }

  for (size_t idx = 0; idx < concurrency; ++idx) {
    const auto [in_chunk_begin, in_chunk_end] = input_chunks[idx];
    const auto result_chunk_begin = result_chunks[idx].first;

    auto worker_task = taskflow.emplace(
      [=]() { std::transform(in_chunk_begin, in_chunk_end, result_chunk_begin, func); });
    begin_task.precede(worker_task);
    worker_task.precede(end_task);

    if (name_tasks) {
      worker_task.name(util::concat(name_prefix, "_", idx));
    }
  }

  return std::make_pair(begin_task, end_task);
}

/**
 * Result of calling 'scatter'. Contains all the scatter tasks, as well as
 * a convenience task that runs just before all the scatter tasks.
 */
struct ScatterResult
{
  tf::Task begin_task;
  std::vector<tf::Task> scattered_tasks;
};

/**
 * Parallel scatter operation using taskflow. Takes a number 'scatter_factor', a range of at
 * least 'scatter_factor' elements as well as a function taking a begin and end iterator and an
 * index. Splits the range into 'scatter_factor' subranges of equal length and generates a task for
 * each of the subranges that calls the given function with the subrange begin and end iterators.
 *
 * Returns the task that runs before the scatter tasks, as well as all the scatter tasks themselves
 */
template<typename InputIterator, typename Func, typename Taskflow>
ScatterResult
scatter(InputIterator begin,
        InputIterator end,
        Func func,
        Taskflow& taskflow,
        size_t scatter_factor,
        std::string name_prefix = "")
{
  const auto distance = static_cast<size_t>(std::abs(std::distance(begin, end)));
  if (distance < scatter_factor) {
    throw std::runtime_error{
      "Can't scatter a range that has less than 'scatter_factor' elements!"
    };
  }

  const auto scatter_chunks = split_range_into_chunks(scatter_factor, begin, end);

  ScatterResult result;
  result.begin_task = taskflow.placeholder();
  result.scattered_tasks.reserve(scatter_factor);

  const auto name_tasks = !name_prefix.empty();
  if (name_tasks) {
    result.begin_task.name(util::concat(name_prefix, "_begin"));
  }

  for (size_t idx = 0; idx < scatter_factor; ++idx) {
    const auto [chunk_begin, chunk_end] = scatter_chunks[idx];
    auto worker_task = taskflow.emplace(
      [chunk_begin, chunk_end, func, idx]() { func(chunk_begin, chunk_end, idx); });

    if (name_tasks) {
      worker_task.name(util::concat(name_prefix, "_", idx));
    }

    result.begin_task.precede(worker_task);
    result.scattered_tasks.push_back(worker_task);
  }

  return result;
}

} // namespace parallel