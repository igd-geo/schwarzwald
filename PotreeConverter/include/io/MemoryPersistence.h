#pragma once

#include "IPointsPersistence.h"

#include <unordered_map>

/**
 * Persists points in memory
 */
struct MemoryPersistence : IPointsPersistence
{
  void persist_points(gsl::span<Potree::PointBuffer::PointReference> points,
                      const std::string& node_name) override;
  void persist_indices(gsl::span<OctreeNodeKey64> indices, const std::string& node_name) override;
  void persist_hierarchy(const std::string& node_name, const Potree::AABB& bounds) override;

  void retrieve_points(const std::string& node_name, Potree::PointBuffer& points) override;
  void retrieve_indices(const std::string& node_name,
                        std::vector<OctreeNodeKey64>& indices) override;
  void retrieve_hierarchy(const std::string& node_name, Potree::AABB& bounds) override;

  const auto& get_points() const { return _points_cache; }
  const auto& get_indices() const { return _indices_cache; }

private:
  std::unordered_map<std::string, Potree::PointBuffer> _points_cache;
  std::unordered_map<std::string, std::vector<OctreeNodeKey64>> _indices_cache;
};