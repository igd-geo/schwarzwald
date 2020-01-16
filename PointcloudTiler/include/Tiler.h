#pragma once

#include "AABB.h"
#include "Definitions.h"
#include "IWriter.h"
#include "PointAttributes.hpp"
#include "PointBuffer.h"
#include "Transformation.h"
#include "octree/Sampling.h"
#include "util/Semaphore.h"
#include "util/TaskSystem.h"

#include <atomic>
#include <gsl/gsl>
#include <string>
#include <thread>

struct ProgressReporter;
struct IPointsPersistence;

struct TilerMetaParameters
{
  float spacing_at_root;
  uint32_t max_depth;
  size_t max_points_per_node;
  size_t internal_cache_size;
  bool create_journal;
};

struct Tiler : IWriter
{
  Tiler(const AABB& aabb,
        const TilerMetaParameters& meta_parameters,
        SamplingStrategy sampling_strategy,
        ProgressReporter* progress_reporter,
        IPointsPersistence& persistence);

  void index() override;

  void wait_until_indexed() override;

  bool needs_indexing() const override;

  void cache(const PointBuffer& points) override;

  void close() override;

private:
  void run_worker();

  AABB _aabb;
  TilerMetaParameters _meta_parameters;
  SamplingStrategy _sampling_strategy;
  ProgressReporter* _progress_reporter;
  IPointsPersistence& _persistence;

  PointBuffer _store;

  std::thread _indexing_thread;
  Semaphore _producers, _consumers;
  PointBuffer _indexing_point_cache;
  std::atomic_bool _run_indexing_thread;
};