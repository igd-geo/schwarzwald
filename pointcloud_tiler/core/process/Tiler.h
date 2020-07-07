#pragma once

#include "datastructures/PointBuffer.h"
#include "io/PointsPersistence.h"
#include "math/AABB.h"
#include "point_source/PointSource.h"
#include "pointcloud/PointAttributes.h"
#include "tiling/Sampling.h"
#include "util/Definitions.h"
#include "util/Transformation.h"
#include <threading/Semaphore.h>
#include <threading/TaskSystem.h>

#include <atomic>
#include <gsl/gsl>
#include <string>
#include <thread>

#include <taskflow/taskflow.hpp>

struct ProgressReporter;
struct TilingAlgorithmBase;

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

struct TilerMetaParameters
{
  float spacing_at_root;
  uint32_t max_depth;
  size_t max_points_per_node;
  size_t batch_read_size;
  size_t internal_cache_size;
  bool create_journal;
  TilingStrategy tiling_strategy;
};

struct Tiler
{
  Tiler(const AABB& aabb,
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
  enum class IndexingThreadState
  {
    Run,
    ExitGracefully,
    TerminateImmediately
  };

  void run_worker();
  void join_worker(IndexingThreadState state);

  void do_indexing_for_current_batch(tf::Executor& executor);
  void swap_point_buffers();

  AABB _aabb;
  TilerMetaParameters _meta_parameters;
  SamplingStrategy _sampling_strategy;
  ProgressReporter* _progress_reporter;
  MultiReaderPointSource _point_source;
  PointsPersistence& _persistence;

  const PointAttributes& _input_attributes;
  fs::path _output_directory;

  PointBuffer _points_cache_for_producers, _points_cache_for_consumers;

  std::unique_ptr<TilingAlgorithmBase> _tiling_algorithm;

  std::thread _indexing_thread;
  Semaphore _producers, _consumers;

  std::atomic<IndexingThreadState> _indexing_thread_state;
};