#pragma once

#include "AABB.h"
#include "IWriter.h"
#include "PointAttributes.hpp"
#include "PointBuffer.h"
#include "Transformation.h"
#include "definitions.hpp"
#include "util/Semaphore.h"

#include <atomic>
#include <gsl/gsl>
#include <string>
#include <thread>

struct ProgressReporter;
struct IPointsPersistence;

namespace Potree {

struct BatchedPotreeWriter : IWriter
{
  BatchedPotreeWriter(const std::string& work_dir,
                      const AABB& aabb,
                      float spacing,
                      uint32_t maxDepth,
                      PointAttributes pointAttributes,
                      ConversionQuality quality,
                      const SRSTransformHelper& transform,
                      uint32_t max_memory_usage_MiB,
                      ProgressReporter* progress_reporter,
                      IPointsPersistence& persistence);

  void index() override;

  void wait_until_indexed() override;

  bool needs_indexing() const override;

  void cache(const PointBuffer& points) override;

  void flush() override;

  bool needs_flush() const override;

  void close() override;

private:
  void run_worker();

  std::string _work_dir;
  AABB _aabb;
  float _spacing;
  uint32_t _max_depth;
  PointAttributes _point_attributes;
  const SRSTransformHelper& _transform_helper;
  ProgressReporter* _progress_reporter;
  IPointsPersistence& _persistence;

  PointBuffer _store;

  std::thread _indexing_thread;
  Semaphore _producers, _consumers;
  PointBuffer _indexing_point_cache;
  std::atomic_bool _run_indexing_thread;
};

} // namespace Potree