#pragma once

#include <experimental/filesystem>

#include "AABB.h"
#include "PointBuffer.h"

constexpr size_t DEFAULT_MAX_POINT_BATCH_SIZE = 4096;

class PointReader
{
public:
  virtual ~PointReader(){};

  /// <summary>
  /// Reads a batch of points and returns their attributes in a PointBuffer
  /// structure. The maxBatchCount argument specifies the maximum number of
  /// points that should be read with a single call to readPoints. Note that
  /// less points can be returned if the reader has less than maxBatchCount
  /// points remaining. If no points can be read, an empty PointBuffer is
  /// returned
  /// </summary>
  virtual PointBuffer readPointBatch(size_t maxBatchCount = DEFAULT_MAX_POINT_BATCH_SIZE) = 0;

  virtual AABB getAABB() = 0;

  virtual long long numPoints() = 0;

  virtual void close() = 0;
};