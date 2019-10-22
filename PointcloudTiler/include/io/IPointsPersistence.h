#pragma once

#include "AABB.h"
#include "PointBuffer.h"
#include "octree/MortonIndex.h"

#include <gsl/gsl>

/**
 * Sink for persisting the processed points and indices
 */
struct IPointsPersistence
{
  virtual ~IPointsPersistence() {}

  /**
   * Persist the given range of points for the given node
   */
  virtual void persist_points(gsl::span<PointBuffer::PointReference> points,
                              const AABB& bounds,
                              const std::string& node_name) = 0;
  /**
   * Persist the given range of point indices for the given node
   */
  virtual void persist_indices(gsl::span<MortonIndex64> indices, const std::string& node_name) = 0;

  /**
   * Retrieve points for the given node from storage
   */
  virtual void retrieve_points(const std::string& node_name, PointBuffer& points) = 0;
  /**
   * Retrieve indices for the given node from storage
   */
  virtual void retrieve_indices(const std::string& node_name,
                                std::vector<MortonIndex64>& indices) = 0;
};