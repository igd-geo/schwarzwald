#include "util/Scheduler.h"
#include "util/Config.h"

#include <logging/Journal.h>

#include <gsl/gsl>

TilingScheduler::~TilingScheduler() {}

FixedThreadsScheduler::FixedThreadsScheduler(FixedThreadsSchedulerArgs args)
  : _read_executor(args.read_threads)
  , _indexing_executor(args.indexing_threads)
{
  if (!global_config().is_journaling_enabled)
    return;

  _read_executor_observer =
    _read_executor.make_observer<tf::ChromeObserver>();
  _indexing_executor_observer =
    _indexing_executor.make_observer<tf::ChromeObserver>();
}

FixedThreadsScheduler::~FixedThreadsScheduler()
{
  if (!_read_executor_observer)
    return;

  auto read_trace = _read_executor_observer->dump();
  auto indexing_trace = _indexing_executor_observer->dump();

  auto read_journal = logging::JournalStore::global()
                        .new_journal("executor_read_trace")
                        .with_flat_type<std::string>()
                        .as_text(global_config().journal_directory)
                        .into_unique_files()
                        .build();
  read_journal->add_record(std::move(read_trace));

  auto indexing_journal = logging::JournalStore::global()
                            .new_journal("executor_indexing_trace")
                            .with_flat_type<std::string>()
                            .as_text(global_config().journal_directory)
                            .into_unique_files()
                            .build();
  indexing_journal->add_record(std::move(indexing_trace));
}

std::future<void>
FixedThreadsScheduler::execute_tiling_iteration(tf::Taskflow& read_graph,
                                                tf::Taskflow& index_graph)
{
  auto read_future = _read_executor.run(read_graph).share();
  auto index_future = _indexing_executor.run(index_graph).share();

  return std::async(std::launch::deferred,
                    [wait_for_read = std::move(read_future),
                     wait_for_index = std::move(index_future)]() mutable {
                      wait_for_read.wait();
                      wait_for_index.wait();
                    });
}

std::pair<uint32_t, uint32_t>
FixedThreadsScheduler::get_read_and_index_concurrency(uint32_t remaining_files)
{
  return { _read_executor.num_workers(), _indexing_executor.num_workers() };
}

AdaptiveScheduler::AdaptiveScheduler(
  uint32_t num_threads,
  ThroughputSampler& read_throughput_sampler,
  ThroughputSampler& indexing_throughput_sampler)
  : _num_read_threads(1)
  , _num_index_threads(std::max(1u, num_threads - 1))
  , _executor(
      _num_read_threads +
      _num_index_threads) // Important to use the sum of read+index threads,
                          // instead of num_threads, because num_threads can be
                          // <= 1 but we need at least two threads!
  , _read_throughput_sampler(read_throughput_sampler)
  , _indexing_throughput_sampler(indexing_throughput_sampler)
{
  if (global_config().is_journaling_enabled) {
    _executor_observer = _executor.make_observer<tf::ChromeObserver>();
  }
}

AdaptiveScheduler::~AdaptiveScheduler()
{
  if (!_executor_observer)
    return;

  auto trace = _executor_observer->dump();

  auto journal = logging::JournalStore::global()
                   .new_journal("executor_adaptive_trace")
                   .with_flat_type<std::string>()
                   .as_text(global_config().journal_directory)
                   .into_unique_files()
                   .build();
  journal->add_record(std::move(trace));
}

std::future<void>
AdaptiveScheduler::execute_tiling_iteration(tf::Taskflow& read_graph,
                                            tf::Taskflow& index_graph)
{
  auto read_future = _executor.run(read_graph).share();
  auto index_future = _executor.run(index_graph).share();

  return std::async(std::launch::deferred,
                    [this,
                     wait_for_read = std::move(read_future),
                     wait_for_index = std::move(index_future)]() mutable {
                      wait_for_read.wait();
                      wait_for_index.wait();
                    });
}

std::pair<uint32_t, uint32_t>
AdaptiveScheduler::get_read_and_index_concurrency(uint32_t remaining_files)
{
  // The main question that this function answers is: Should we use more or less
  // read threads, or keep the count the same? For this, we solve the equation
  // following set of equations: 1) R*tr = I*ti 2) R + I = max_threads
  //
  // R = number of read threads
  // tr = throughput of a single read thread in points/s
  // I = number of index threads
  // ti = throughput of a single index thread in points/s

  const auto read_throughput_per_thread =
    _read_throughput_sampler.get_throughput_per_second() / _num_read_threads;
  const auto index_throughput_per_thread =
    _indexing_throughput_sampler.get_throughput_per_second() /
    _num_index_threads;

  const auto total_thread_count =
    gsl::narrow_cast<uint32_t>(_executor.num_workers());

  _num_read_threads = std::min(_num_read_threads, remaining_files);
  _num_index_threads = total_thread_count - _num_read_threads;

  // Any of the throughputs can be zero (e.g. on the first and last iteration).
  // In this case we can't calculate a meaningful new thread distribution
  if (read_throughput_per_thread == 0 || index_throughput_per_thread == 0) {
    return { _num_read_threads, _num_index_threads };
  }

  const auto exact_index_threads =
    total_thread_count /
    (1 + (index_throughput_per_thread / read_throughput_per_thread));
  const auto exact_read_threads = total_thread_count - exact_index_threads;

  const auto rounded_read_threads = std::ceil(exact_read_threads);

  const auto max_read_threads =
    std::min(total_thread_count - 1, remaining_files);

  _num_read_threads = gsl::narrow_cast<uint32_t>(
    std::min<double>(max_read_threads, rounded_read_threads));
  _num_index_threads = total_thread_count - _num_read_threads;

  return { _num_read_threads, _num_index_threads };
}