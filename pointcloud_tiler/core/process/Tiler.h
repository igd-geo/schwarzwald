#pragma once

#include "datastructures/PointBuffer.h"
#include "io/PointsPersistence.h"
#include "math/AABB.h"
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

struct ProgressReporter;
struct TilingAlgorithmBase;

/**
 * Which tiling strategy to use?
 */
enum class TilingStrategy
{
  /**
   * Use the accurate strategy that samples from the root node. This will be slower than
   * the 'Fast' strategy, that skips the first couple of levels and reconstructs them afterwards
   */
  Accurate,
  /**
   * Use the fast strategy that skips the first couple of levels and starts tiling deeper in the
   * octree to enable increased parallelism. The skipped levels are reconstructed and thus contain
   * duplicated data
   */
  Fast
};

struct TilerMetaParameters
{
  float spacing_at_root;
  uint32_t max_depth;
  size_t max_points_per_node;
  size_t internal_cache_size;
  bool create_journal;
  TilingStrategy tiling_strategy;
};

struct Tiler
{
  Tiler(const AABB& aabb,
        const TilerMetaParameters& meta_parameters,
        SamplingStrategy sampling_strategy,
        ProgressReporter* progress_reporter,
        PointsPersistence& persistence,
        fs::path output_directory);
  ~Tiler();

  void index();

  void wait_until_indexed();

  bool needs_indexing() const;

  void cache(const PointBuffer& points);

  void close();

private:
  void run_worker();

  AABB _aabb;
  TilerMetaParameters _meta_parameters;
  SamplingStrategy _sampling_strategy;
  ProgressReporter* _progress_reporter;
  PointsPersistence& _persistence;
  fs::path _output_directory;

  PointBuffer _store;

  std::unique_ptr<TilingAlgorithmBase> _tiling_algorithm;

  std::thread _indexing_thread;
  Semaphore _producers, _consumers;
  PointBuffer _indexing_point_cache;
  std::atomic_bool _run_indexing_thread;
};