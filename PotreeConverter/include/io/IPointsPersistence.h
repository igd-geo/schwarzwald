#pragma once

#include "AABB.h"
#include "PointBuffer.h"
#include "octree/OctreeNodeKey.h"

#include <gsl/gsl>

/**
 * Sink for persisting the processed points, indices and the octree hierarchy
 */
struct IPointsPersistence
{
  virtual ~IPointsPersistence() {}

  /**
   * Persist the given range of points for the given node
   */
  virtual void persist_points(gsl::span<Potree::PointBuffer::PointReference> points,
                              const std::string& node_name) = 0;
  /**
   * Persist the given range of point indices for the given node
   */
  virtual void persist_indices(gsl::span<OctreeNodeKey64> indices,
                               const std::string& node_name) = 0;

  /**
   * Persist the given node hierarchy for the given node
   */
  virtual void persist_hierarchy(const std::string& node_name, const Potree::AABB& bounds) = 0;

  /**
   * Retrieve points for the given node from storage
   */
  virtual void retrieve_points(const std::string& node_name, Potree::PointBuffer& points) = 0;
  /**
   * Retrieve indices for the given node from storage
   */
  virtual void retrieve_indices(const std::string& node_name,
                                std::vector<OctreeNodeKey64>& indices) = 0;
  /**
   * Retrieve the node hierarchy for the given node from storage
   */
  virtual void retrieve_hierarchy(const std::string& node_name, Potree::AABB& bounds) = 0;
};