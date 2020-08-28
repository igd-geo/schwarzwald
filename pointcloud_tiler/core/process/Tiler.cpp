#include "Tiler.h"

#include "containers/Range.h"
#include "datastructures/PointBuffer.h"
#include "io/PNTSReader.h"
#include "io/PNTSWriter.h"
#include "io/TileSetWriter.h"
#include "logging/Journal.h"
#include "pointcloud/Tileset.h"
#include "tiling/Node.h"
#include "tiling/OctreeAlgorithms.h"
#include "tiling/OctreeIndexWriter.h"
#include "tiling/TilingAlgorithms.h"
#include "util/Config.h"
#include "util/Scheduler.h"
#include "util/Stats.h"
#include "util/stuff.h"
#include <containers/DestructuringIterator.h>
#include <debug/Journal.h>
#include <debug/ProgressReporter.h>
#include <debug/ThroughputCounter.h>
#include <debug/Timing.h>
#include <terminal/stdout_helper.h>
#include <threading/Parallel.h>
#include <types/type_util.h>

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

struct ThroughputStats
{
  size_t iteration;
  double read_throughput;
  double index_throughput;
  uint32_t read_concurrency;
  uint32_t index_concurrency;

  REFLECT()
};

REFLECT_STRUCT_BEGIN(ThroughputStats)
REFLECT_STRUCT_MEMBER(iteration)
REFLECT_STRUCT_MEMBER(read_throughput)
REFLECT_STRUCT_MEMBER(index_throughput)
REFLECT_STRUCT_MEMBER(read_concurrency)
REFLECT_STRUCT_MEMBER(index_concurrency)
REFLECT_STRUCT_END()

struct JournalReadCommand
{
  std::string file_name;
  size_t to_read_count;

  REFLECT()
};

REFLECT_STRUCT_BEGIN(JournalReadCommand)
REFLECT_STRUCT_MEMBER(file_name)
REFLECT_STRUCT_MEMBER(to_read_count)
REFLECT_STRUCT_END()

static void
journal_taskflow(const tf::Taskflow& taskflow, const std::string& taskflow_type)
{
  std::stringstream ss;
  taskflow.dump(ss);

  auto taskflow_string = ss.str();

  auto journal = logging::JournalStore::global().get_journal(taskflow_type);

  if (!journal) {
    logging::JournalStore::global()
      .new_journal(taskflow_type)
      .with_flat_type<std::string>()
      .as_text(global_config().journal_directory)
      .into_unique_files()
      .build();
    journal = logging::JournalStore::global().get_journal(taskflow_type);
  }

  journal->add_record_untyped(std::move(taskflow_string));
}

static void
journal_throughput_stats(size_t iteration,
                         double read_throughput,
                         double index_throughput,
                         uint32_t read_concurrency,
                         uint32_t index_concurrency)
{
  static const std::string journal_name = "throughput_stats";
  auto journal = logging::JournalStore::global().get_journal(journal_name);
  if (!journal) {
    logging::JournalStore::global()
      .new_journal(journal_name)
      .with_flat_type<ThroughputStats>()
      .as_csv(global_config().journal_directory)
      .into_single_file()
      .build();
    journal = logging::JournalStore::global().get_journal(journal_name);
  }

  ThroughputStats record{
    iteration, read_throughput, index_throughput, read_concurrency, index_concurrency
  };
  journal->add_record_untyped(record);
}

static void
journal_read_commands(const std::vector<std::vector<ReadCommand>>& read_commands_per_thread)
{
  static const std::string journal_name = "read_commands";
  auto journal = logging::JournalStore::global().get_journal(journal_name);
  if (!journal) {
    logging::JournalStore::global()
      .new_journal(journal_name)
      .with_flat_type<std::string>()
      .as_text(global_config().journal_directory)
      .into_unique_files()
      .build();
    journal = logging::JournalStore::global().get_journal(journal_name);
  }

  // TODO Use JSON functionality of Journal!
  std::stringstream ss;
  ss << "{\n";
  ss << "\t\"threads\": [\n";
  for (auto& cur_thread : read_commands_per_thread) {
    ss << "\t\t[\n";
    for (auto& cmd : cur_thread) {
      ss << "\t\t\t{\n";
      ss << "\t\t\t\t\"file\": \"" << cmd.file_path->string() << "\",\n";
      ss << "\t\t\t\t\"to_read_count\": " << cmd.to_read_count << "\n";
      ss << "\t\t\t}\n";
    }
    ss << "\t\t],\n";
  }
  ss << "\t]\n";
  ss << "}";

  journal->add_record_untyped(ss.str());
}

Tiler::Tiler(DatasetMetadata dataset_metadata,
             TilerMetaParameters meta_parameters,
             SamplingStrategy sampling_strategy,
             ProgressReporter* progress_reporter,
             MultiReaderPointSource point_source,
             PointsPersistence& persistence,
             const PointAttributes& input_attributes,
             fs::path output_directory)
  : _dataset_metadata(std::move(dataset_metadata))
  , _meta_parameters(meta_parameters)
  , _sampling_strategy(std::move(sampling_strategy))
  , _progress_reporter(progress_reporter)
  , _point_source(std::move(point_source))
  , _persistence(persistence)
  , _input_attributes(input_attributes)
  , _output_directory(std::move(output_directory))
  , _producers(0)
  , _consumers(1)
{
  const auto root_spacing_to_bounds_ratio =
    std::log2f(_dataset_metadata.total_bounds_cubic().extent().x / meta_parameters.spacing_at_root);
  if (root_spacing_to_bounds_ratio >= MAX_OCTREE_LEVELS) {
    throw std::runtime_error{ "spacing at root node is too small compared to bounds of data!" };
  }

  _bounds =
    (meta_parameters.shift_points_to_origin ? _dataset_metadata.total_bounds_cubic_at_origin()
                                            : _dataset_metadata.total_bounds_cubic());

  switch (meta_parameters.tiling_strategy) {
    case TilingStrategy::Accurate:
      _tiling_algorithm = std::make_unique<TilingAlgorithmV1>(
        _sampling_strategy, _progress_reporter, _persistence, _meta_parameters);
      break;
    case TilingStrategy::Fast:
      _tiling_algorithm = std::make_unique<TilingAlgorithmV3>(
        _sampling_strategy, _progress_reporter, _persistence, _meta_parameters, _output_directory);
      break;
  }
}

Tiler::~Tiler() {}

size_t
Tiler::run()
{
  size_t points_read = 0;

  // TODO I'm not 100% sure how I want to sample. If I use a sample buffer size
  // of 1, I will get the exact timings of every iteration, but there might be a
  // lot of variance there. If I go higher, I will have smoothed out values but
  // I'm missing the actual information for each iteration... It seems better to
  // stick with a buffer size of 1, statistical analyses can be done afterwards
  ThroughputSampler read_throughput_sampler{ 1 };
  ThroughputSampler index_throughput_sampler{ 1 };

  std::unique_ptr<TilingScheduler> scheduler;
  std::visit(overloaded{ [&](const FixedThreadCount& thread_count) {
                          FixedThreadsSchedulerArgs args;
                          args.indexing_threads = thread_count.num_threads_for_indexing;
                          args.read_threads = thread_count.num_threads_for_reading;
                          scheduler = std::make_unique<FixedThreadsScheduler>(args);
                        },
                         [&](const AdaptiveThreadCount& thread_count) {
                           scheduler =
                             std::make_unique<AdaptiveScheduler>(thread_count.num_threads,
                                                                 read_throughput_sampler,
                                                                 index_throughput_sampler);
                         } },
             _meta_parameters.thread_count);

  // Allocate points cache for producer and consumer. This is done so that we
  // can read from the PointSource directly into existing memory, which is
  // efficient. Furthermore, it will allow true concurrent reading, as each
  // reader can be assigned to a distinct, pre-allocated region in memory
  _points_cache_for_consumers = { _meta_parameters.internal_cache_size, _input_attributes };
  _points_cache_for_producers = { _meta_parameters.internal_cache_size, _input_attributes };

  create_read_commands();

  auto first_run = true;
  auto last_run = false;

  size_t iteration = 0;

  while (true) {
    tf::Taskflow read_taskflow, index_taskflow;

    const auto [read_concurrency, index_concurrency] =
      scheduler->get_read_and_index_concurrency(max_read_parallelism());

    if (!last_run) {
      if (!build_execution_graph_for_reading(
            read_taskflow, read_concurrency, read_throughput_sampler)) {
        last_run = true;
      }
    }

    if (!first_run) {
      build_execution_graph_for_indexing(
        index_taskflow, index_concurrency, index_throughput_sampler);
    } else {
      first_run = false;
    }

    auto batch_finished = scheduler->execute_tiling_iteration(read_taskflow, index_taskflow);
    batch_finished.wait();

    if (global_config().is_journaling_enabled) {
      journal_taskflow(read_taskflow, "read_taskflow");
      journal_taskflow(index_taskflow, "index_taskflow");

      const auto read_throughput = read_throughput_sampler.get_throughput_per_second();
      const auto index_throughput = index_throughput_sampler.get_throughput_per_second();
      journal_throughput_stats(
        iteration, read_throughput, index_throughput, read_concurrency, index_concurrency);
    }
    ++iteration;

    if (last_run) {
      break;
    }
  }

  _tiling_algorithm->finalize(_bounds);

  return points_read;
}

bool
Tiler::build_execution_graph_for_reading(tf::Taskflow& tf,
                                         uint32_t num_read_threads,
                                         ThroughputSampler& throughput_sampler)
{
  // Start task (nothing) --> N*read tasks --> swap buffer task

  auto start_task =
    tf.emplace([this]() { _begin_read_cycle_time = std::chrono::high_resolution_clock::now(); });

  adjust_read_thread_count(num_read_threads);

  size_t num_read_points_in_current_batch = 0;
  const auto total_max_points_in_batch = _meta_parameters.internal_cache_size;
  const auto max_points_per_thread = total_max_points_in_batch / num_read_threads;

  std::vector<std::vector<ReadCommand>> read_commands_per_read_thread;
  read_commands_per_read_thread.resize(num_read_threads);

  std::vector<size_t> remaining_points_to_read_per_thread;
  remaining_points_to_read_per_thread.resize(num_read_threads);
  std::generate_n(std::begin(remaining_points_to_read_per_thread),
                  num_read_threads,
                  [max_points_per_thread]() { return max_points_per_thread; });

  // Take as many ReadCommands as we can for a thread, then move onto the next
  // thread

  for (uint32_t thread_idx = 0; thread_idx < num_read_threads; ++thread_idx) {
    auto& next_read_command_cur_thread = _next_read_commands_per_thread[thread_idx];
    auto& scheduled_read_commands_cur_thread = read_commands_per_read_thread[thread_idx];
    auto& remaining_points_to_read_cur_thread = remaining_points_to_read_per_thread[thread_idx];

    // Schedule commands for this thread until we either scheduled enough points
    // to read, or we run out of ReadCommands
    while (remaining_points_to_read_cur_thread) {
      if (next_read_command_cur_thread.to_read_count == 0) {
        if (_remaining_read_commands.empty()) {
          // Distribute the remaining points that could not be read by this
          // thread evenly to the remaining threads
          auto remaining_points = remaining_points_to_read_per_thread[thread_idx];
          const auto remaining_threads = num_read_threads - thread_idx;
          if (remaining_points && remaining_threads) {
            for (uint32_t next_thread_idx = thread_idx + 1; next_thread_idx < num_read_threads;
                 ++next_thread_idx) {
              const auto remainder_for_this_thread = (next_thread_idx == num_read_threads - 1)
                                                       ? remaining_points
                                                       : (remaining_points / remaining_threads);
              remaining_points_to_read_per_thread[next_thread_idx] += remainder_for_this_thread;
              remaining_points -= remainder_for_this_thread;
            }
          }
          break;
        }
        next_read_command_cur_thread = _remaining_read_commands.front();
        _remaining_read_commands.pop_front();

        // Ignore empty files
        if (next_read_command_cur_thread.to_read_count == 0) {
          continue;
        }
      }

      // Take as many points as we can
      const auto points_to_read_from_cur_file =
        std::min(remaining_points_to_read_cur_thread, next_read_command_cur_thread.to_read_count);
      scheduled_read_commands_cur_thread.push_back(
        { next_read_command_cur_thread.file_path, points_to_read_from_cur_file });

      remaining_points_to_read_cur_thread -= points_to_read_from_cur_file;
      next_read_command_cur_thread.to_read_count -= points_to_read_from_cur_file;

      num_read_points_in_current_batch += points_to_read_from_cur_file;
    }
  }

  if (global_config().is_journaling_enabled) {
    journal_read_commands(read_commands_per_read_thread);
  }

  if (!num_read_points_in_current_batch) {
    // Nothing more to read, all points are read!
    return false;
  }

  // Now that we have all the ReadCommands for each thread, we have to determine
  // the memory areas in the producer buffer that the threads will write into
  std::vector<tf::Task> read_task_handles;
  read_task_handles.reserve(num_read_threads);

  size_t offset_in_producer_buffer = 0;
  for (auto& read_commands_for_current_thread : read_commands_per_read_thread) {
    const auto total_points_to_read_cur_thread =
      std::accumulate(std::begin(read_commands_for_current_thread),
                      std::end(read_commands_for_current_thread),
                      size_t{ 0 },
                      [](auto accum, const auto& cmd) { return accum + cmd.to_read_count; });
    const auto next_offset_in_producer_buffer =
      offset_in_producer_buffer + total_points_to_read_cur_thread;

    const auto read_task =
      tf.emplace([this,
                  offset_in_buffer = offset_in_producer_buffer,
                  to_read_count = total_points_to_read_cur_thread,
                  read_commands = std::move(read_commands_for_current_thread)]() {
          execute_read_commands(
            read_commands,
            { std::begin(_points_cache_for_producers) + offset_in_buffer,
              std::begin(_points_cache_for_producers) + offset_in_buffer + to_read_count });
        })
        .name(concat("read_", total_points_to_read_cur_thread));

    start_task.precede(read_task);
    read_task_handles.push_back(read_task);

    offset_in_producer_buffer = next_offset_in_producer_buffer;
  }

  const auto total_points_to_read_in_cur_batch = offset_in_producer_buffer;

  const auto swap_buffer_task =
    tf.emplace([this, total_points_to_read_in_cur_batch, &throughput_sampler]() {
        estimate_read_throughput(throughput_sampler, total_points_to_read_in_cur_batch);
        swap_point_buffers(total_points_to_read_in_cur_batch);
      })
      .name("swap_buffers");

  for (auto& read_task : read_task_handles) {
    read_task.precede(swap_buffer_task);
  }

  return true;
}

void
Tiler::adjust_read_thread_count(size_t num_read_threads)
{
  if (num_read_threads == _next_read_commands_per_thread.size())
    return;

  if (num_read_threads > _next_read_commands_per_thread.size()) {
    _next_read_commands_per_thread.resize(num_read_threads);
    return;
  }

  // Shrink and put back read commands into _remaining_read_commands
  _remaining_read_commands.insert(std::begin(_remaining_read_commands),
                                  std::begin(_next_read_commands_per_thread) + num_read_threads,
                                  std::end(_next_read_commands_per_thread));

  _next_read_commands_per_thread.resize(num_read_threads);
}

void
Tiler::create_read_commands()
{
  // Create one read command per file and assign it the files total number of
  // points. In build_execution_graph_for_reading, we will slice off points from
  // these ReadCommands. Once a ReadCommand has zero points left, it is finished

  std::transform(std::begin(_dataset_metadata.get_all_files_metadata()),
                 std::end(_dataset_metadata.get_all_files_metadata()),
                 std::back_inserter(_remaining_read_commands),
                 [](const auto& kv) -> ReadCommand {
                   const auto& file_path = kv.first;
                   const auto& metadata = kv.second;
                   return { &file_path, metadata.points_count };
                 });
}

void
Tiler::execute_read_commands(const std::vector<ReadCommand>& read_commands,
                             util::Range<PointBuffer::PointIterator> read_destination)
{
  auto read_destination_start = std::begin(read_destination);
  for (const auto& read_command : read_commands) {
    auto next_file = _point_source.lock_specific_source(*read_command.file_path);
    if (!next_file) {
      // TODO Error or ignore? For now error
      throw std::runtime_error{ (boost::format("Could not lock file source %1% but there "
                                               "is a ReadCommand for it...") %
                                 *read_command.file_path)
                                  .str() };
    }

    const auto new_read_destination_start = next_file->read_next_into(
      { read_destination_start, read_destination_start + read_command.to_read_count },
      _input_attributes);
    const auto num_points_read =
      static_cast<size_t>(std::distance(read_destination_start, new_read_destination_start));
    assert(num_points_read == read_command.to_read_count);
    read_destination_start = new_read_destination_start;

    if (_progress_reporter) {
      _progress_reporter->increment_progress<size_t>(progress::LOADING, num_points_read);
    }

    _point_source.release_source(*next_file);
  }
}

uint32_t
Tiler::max_read_parallelism() const
{
  // Calculate the total number of remaining files, which equals the maximum
  // parallelism for reading that we can sustain
  return gsl::narrow<uint32_t>(_next_read_commands_per_thread.size() +
                               _remaining_read_commands.size());
}

void
Tiler::build_execution_graph_for_indexing(tf::Taskflow& tf,
                                          uint32_t num_indexing_threads,
                                          ThroughputSampler& throughput_sampler)
{
  util::Range<PointBuffer::PointIterator> produced_points_range{
    std::begin(_points_cache_for_consumers),
    std::begin(_points_cache_for_consumers) + _produced_points_count
  };

  auto [indexing_first_task, indexing_last_task] = _tiling_algorithm->build_execution_graph(
    produced_points_range, _bounds, num_indexing_threads, tf);

  // The actual indexing is bounded by first waiting for the _producers
  // Semaphore, and at the end incrementing the _consumers Semaphore
  auto wait_for_producers_task = tf.emplace([this]() {
    _producers.wait();
    _begin_index_cycle_time = std::chrono::high_resolution_clock::now();
  });

  const auto produced_points_count = _produced_points_count;
  auto notify_consumers_task = tf.emplace([this, &throughput_sampler, produced_points_count]() {
    estimate_index_throughput(throughput_sampler, produced_points_count);
    _consumers.notify();
  });

  wait_for_producers_task.precede(indexing_first_task);
  indexing_last_task.precede(notify_consumers_task);
}

void
Tiler::estimate_read_throughput(ThroughputSampler& sampler, size_t num_points_in_last_cycle) const
{
  const auto delta_t = std::chrono::high_resolution_clock::now() - _begin_read_cycle_time;
  sampler.push_entry(num_points_in_last_cycle, delta_t);
}

void
Tiler::estimate_index_throughput(ThroughputSampler& sampler, size_t num_points_in_last_cycle) const
{
  const auto delta_t = std::chrono::high_resolution_clock::now() - _begin_index_cycle_time;
  sampler.push_entry(num_points_in_last_cycle, delta_t);
}

void
Tiler::swap_point_buffers(size_t produced_points_count)
{
  _consumers.wait();
  std::swap(_points_cache_for_producers, _points_cache_for_consumers);
  _produced_points_count = produced_points_count;
  //_points_cache_for_producers.clear();
  _producers.notify();
}