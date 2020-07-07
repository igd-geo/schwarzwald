#include "Tiler.h"

#include "containers/Range.h"
#include "datastructures/PointBuffer.h"
#include "io/PNTSReader.h"
#include "io/PNTSWriter.h"
#include "io/TileSetWriter.h"
#include "pointcloud/Tileset.h"
#include "tiling/Node.h"
#include "tiling/OctreeAlgorithms.h"
#include "tiling/OctreeIndexWriter.h"
#include "tiling/TilingAlgorithms.h"
#include "util/ExecutorObserver.h"
#include "util/stuff.h"
#include <containers/DestructuringIterator.h>
#include <debug/Journal.h>
#include <debug/ProgressReporter.h>
#include <debug/ThroughputCounter.h>
#include <debug/Timing.h>
#include <terminal/stdout_helper.h>
#include <threading/Parallel.h>

#include <algorithm>
#include <boost/algorithm/string/join.hpp>
#include <boost/scope_exit.hpp>
#include <cmath>
#include <experimental/filesystem>
#include <functional>
#include <future>
#include <iomanip>
#include <map>
#include <numeric>
#include <queue>
#include <stdint.h>
#include <unordered_map>

#include <taskflow/taskflow.hpp>

constexpr uint32_t MAX_OCTREE_LEVELS = 21;

Tiler::Tiler(const AABB& aabb,
             TilerMetaParameters meta_parameters,
             SamplingStrategy sampling_strategy,
             ProgressReporter* progress_reporter,
             MultiReaderPointSource point_source,
             PointsPersistence& persistence,
             const PointAttributes& input_attributes,
             fs::path output_directory)
  : _aabb(aabb)
  , _meta_parameters(meta_parameters)
  , _sampling_strategy(std::move(sampling_strategy))
  , _progress_reporter(progress_reporter)
  , _point_source(std::move(point_source))
  , _persistence(persistence)
  , _input_attributes(input_attributes)
  , _output_directory(std::move(output_directory))
  , _producers(0)
  , _consumers(1)
  , _indexing_thread_state(IndexingThreadState::Run)
{
  const auto root_spacing_to_bounds_ratio =
    std::log2f(aabb.extent().x / meta_parameters.spacing_at_root);
  if (root_spacing_to_bounds_ratio >= MAX_OCTREE_LEVELS) {
    throw std::runtime_error{ "spacing at root node is too small compared to bounds of data!" };
  }

  _indexing_thread = std::thread([this]() { run_worker(); });

  switch (meta_parameters.tiling_strategy) {
    case TilingStrategy::Accurate:
      _tiling_algorithm =
        std::make_unique<TilingAlgorithmV1>(_sampling_strategy,
                                            _progress_reporter,
                                            _persistence,
                                            _meta_parameters,
                                            std::max(1u, std::thread::hardware_concurrency() - 1));
      break;
    case TilingStrategy::Fast:
      _tiling_algorithm =
        std::make_unique<TilingAlgorithmV3>(_sampling_strategy,
                                            _progress_reporter,
                                            _persistence,
                                            _meta_parameters,
                                            _output_directory,
                                            std::max(1u, std::thread::hardware_concurrency() - 1));
      break;
  }
}

Tiler::~Tiler()
{
  if (!_indexing_thread.joinable())
    return;

  join_worker(IndexingThreadState::TerminateImmediately);
}

size_t
Tiler::run()
{
  size_t points_read = 0;

  ThroughputSampler points_throughput_sampler{ 64 };

  // Allocate points cache for producer and consumer. This is done so that we can read from
  // the PointSource directly into existing memory, which is efficient. Furthermore, it will
  // allow true concurrent reading, as each reader can be assigned to a distinct, pre-allocated
  // region in memory
  _points_cache_for_consumers = { _meta_parameters.internal_cache_size, _input_attributes };
  _points_cache_for_producers = { _meta_parameters.internal_cache_size, _input_attributes };

  // We keep track of the current write position in the producer cache here
  auto start_index_in_producer_cache = std::begin(_points_cache_for_producers);
  auto producer_cache_end = std::end(_points_cache_for_producers);

  while (true) {
    auto next_file = _point_source.lock_source();
    if (!next_file)
      break;

    auto [new_producer_cache_begin, read_time] = util::time([&]() {
      return next_file->read_next_into({ start_index_in_producer_cache, producer_cache_end },
                                       _input_attributes);
    });

    const auto num_points_read =
      std::distance(start_index_in_producer_cache, new_producer_cache_begin);
    points_read += num_points_read;
    points_throughput_sampler.push_entry(num_points_read, read_time);
    if (_progress_reporter) {
      _progress_reporter->increment_progress<size_t>(progress::LOADING, num_points_read);
    }

    util::write_log(
      (boost::format("Current I/O throughput: %1%pts/s\n") %
       unit::format_with_metric_prefix(points_throughput_sampler.get_throughput_per_second()))
        .str());

    start_index_in_producer_cache = new_producer_cache_begin;

    if (start_index_in_producer_cache == producer_cache_end) {
      swap_point_buffers();

      start_index_in_producer_cache = std::begin(_points_cache_for_producers);
      producer_cache_end = std::end(_points_cache_for_producers);
    }

    _point_source.release_source(*next_file);
  }

  // The last batch can (and most often will) contain less than 'internal_cache_size' points,
  // so we have to shrink the producer buffer to the actual number of points in the last batch
  _points_cache_for_producers.resize(
    std::distance(std::begin(_points_cache_for_producers), start_index_in_producer_cache));

  swap_point_buffers();

  join_worker(IndexingThreadState::ExitGracefully);

  _tiling_algorithm->finalize(_aabb);

  return points_read;
}

void
Tiler::do_indexing_for_current_batch(tf::Executor& executor)
{
  static uint32_t s_batch_idx = 0;

  const auto t_start = std::chrono::high_resolution_clock::now();

  tf::Taskflow taskflow;

  _tiling_algorithm->build_execution_graph(_points_cache_for_consumers, _aabb, taskflow);

  executor.run(taskflow).wait();

  const auto t_end = std::chrono::high_resolution_clock::now();
  const auto delta_t = t_end - t_start;

  if (debug::Journal::instance().is_enabled()) {
    const auto delta_t_seconds = static_cast<double>(delta_t.count()) / 1e9;
    const auto points_per_second = _points_cache_for_consumers.count() / delta_t_seconds;
    debug::Journal::instance().add_entry(
      (boost::format("Indexing (batch %1%): %2% s | %3% pts/s") % s_batch_idx++ % delta_t_seconds %
       unit::format_with_metric_prefix(points_per_second))
        .str());

    std::ofstream ofs{ concat(
      _output_directory.string(), "/taskflow_structure_", s_batch_idx, ".gv") };
    taskflow.dump(ofs);
  }
}

void
Tiler::swap_point_buffers()
{
  _consumers.wait();
  std::swap(_points_cache_for_producers, _points_cache_for_consumers);
  //_points_cache_for_producers.clear();
  _producers.notify();
}

void
Tiler::run_worker()
{
  // Use max_threads - 1 because one thread is reserved for reading points
  const auto concurrency = std::max(1u, std::thread::hardware_concurrency() - 1);
  // const auto concurrency = 1u;

  tf::Executor executor{ concurrency };
  ExecutorObserver* executor_observer = nullptr;
  if (debug::Journal::instance().is_enabled()) {
    executor_observer = executor.make_observer<ExecutorObserver>();
  }

  while (true) {
    _producers.wait();
    if (_indexing_thread_state != IndexingThreadState::Run)
      break;

    do_indexing_for_current_batch(executor);

    _consumers.notify();
  }

  if (_indexing_thread_state == IndexingThreadState::TerminateImmediately)
    return;

  if (debug::Journal::instance().is_enabled()) {
    std::ofstream ofs{ concat(_output_directory.string(), "/taskflow.json") };
    executor_observer->dump(ofs);
  }
}

void
Tiler::join_worker(IndexingThreadState state)
{
  _consumers.wait();
  _indexing_thread_state = state;
  _producers.notify();

  _indexing_thread.join();
}