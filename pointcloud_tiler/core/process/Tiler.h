#pragma once

#include "datastructures/PointBuffer.h"
#include "io/PointsPersistence.h"
#include "math/AABB.h"
#include "point_source/PointSource.h"
#include "pointcloud/FileStats.h"
#include "pointcloud/PointAttributes.h"
#include "tiling/Sampling.h"
#include "util/Definitions.h"
#include "util/Transformation.h"
#include <reflection/StaticReflection.h>
#include <threading/Semaphore.h>
#include <threading/TaskSystem.h>

#include <atomic>
#include <deque>
#include <gsl/gsl>
#include <string>
#include <thread>

#include <taskflow/taskflow.hpp>

struct ProgressReporter;
struct TilingAlgorithmBase;
struct ThroughputSampler;

/**
 * Which tiling strategy to use?
 */
enum class TilingStrategy
{
  /**
   * Use the accurate strategy that samples from the root node. This will be
   * slower than the 'Fast' strategy, that skips the first couple of levels and
   * reconstructs them afterwards
   */
  Accurate,
  /**
   * Use the fast strategy that skips the first couple of levels and starts
   * tiling deeper in the octree to enable increased parallelism. The skipped
   * levels are reconstructed and thus contain duplicated data
   */
  Fast
};

struct FixedThreadCount
{
  uint32_t num_threads_for_reading;
  uint32_t num_threads_for_indexing;
};

struct AdaptiveThreadCount
{
  uint32_t num_threads;
};

/**
 * Thread number configuration. Either a fixed number of read and index threads, or
 * a total fixed number of threads that are scheduled adaptively
 */
using ThreadConfig = std::variant<FixedThreadCount, AdaptiveThreadCount>;

struct TilerMetaParameters
{
  float spacing_at_root;
  uint32_t max_depth;
  size_t max_points_per_node;
  size_t batch_read_size;
  size_t internal_cache_size;
  bool shift_points_to_origin;
  bool create_journal;
  TilingStrategy tiling_strategy;
  std::variant<FixedThreadCount, AdaptiveThreadCount> thread_count;
};

/**
 * Abstract command for reading points from a file.
 */
struct ReadCommand
{
  const fs::path* file_path;
  size_t to_read_count;
};

struct Tiler
{
  Tiler(DatasetMetadata dataset_metadata,
        TilerMetaParameters meta_parameters,
        SamplingStrategy sampling_strategy,
        ProgressReporter* progress_reporter,
        MultiReaderPointSource point_source,
        PointsPersistence& persistence,
        const PointAttributes& input_attributes,
        fs::path output_directory);
  ~Tiler();

  /**
   * Run the tiler. Returns the total number of points that were processed
   */
  size_t run();

private:
  void swap_point_buffers(size_t produced_points_count);

  bool build_execution_graph_for_reading(tf::Taskflow& tf,
                                         uint32_t num_read_threads,
                                         ThroughputSampler& throughput_sampler);
  void execute_read_commands(const std::vector<ReadCommand>& read_commands,
                             util::Range<PointBuffer::PointIterator> read_destination);

  void build_execution_graph_for_indexing(tf::Taskflow& tf,
                                          uint32_t num_indexing_threads,
                                          ThroughputSampler& throughput_sampler);

  void create_read_commands();
  void adjust_read_thread_count(size_t num_read_threads);
  uint32_t max_read_parallelism() const;

  void estimate_read_throughput(ThroughputSampler& sampler, size_t num_points_in_last_cycle) const;
  void estimate_index_throughput(ThroughputSampler& sampler, size_t num_points_in_last_cycle) const;

  DatasetMetadata _dataset_metadata;
  TilerMetaParameters _meta_parameters;
  SamplingStrategy _sampling_strategy;
  ProgressReporter* _progress_reporter;
  MultiReaderPointSource _point_source;
  PointsPersistence& _persistence;

  AABB _bounds;

  const PointAttributes& _input_attributes;
  fs::path _output_directory;

  PointBuffer _points_cache_for_producers, _points_cache_for_consumers;
  size_t _produced_points_count;

  std::deque<ReadCommand> _remaining_read_commands;
  std::vector<ReadCommand> _next_read_commands_per_thread;

  std::unique_ptr<TilingAlgorithmBase> _tiling_algorithm;

  Semaphore _producers, _consumers;

  std::chrono::high_resolution_clock::time_point _begin_read_cycle_time;
  std::chrono::high_resolution_clock::time_point _begin_index_cycle_time;
};