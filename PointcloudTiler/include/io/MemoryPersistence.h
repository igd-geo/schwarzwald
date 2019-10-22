#pragma once

#include "IPointsPersistence.h"

#include <mutex>
#include <unordered_map>

/**
 * Persists points in memory
 */
struct MemoryPersistence : IPointsPersistence
{
  void persist_points(gsl::span<PointBuffer::PointReference> points,
                      const AABB& bounds,
                      const std::string& node_name) override;
  void persist_indices(gsl::span<MortonIndex64> indices, const std::string& node_name) override;

  void retrieve_points(const std::string& node_name, PointBuffer& points) override;
  void retrieve_indices(const std::string& node_name, std::vector<MortonIndex64>& indices) override;

  const auto& get_points() const { return _points_cache; }
  const auto& get_indices() const { return _indices_cache; }

private:
  std::mutex _lock;
  std::unordered_map<std::string, PointBuffer> _points_cache;
  std::unordered_map<std::string, std::vector<MortonIndex64>> _indices_cache;
};