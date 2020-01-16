#pragma once

#include "IPointsPersistence.h"
#include "util/LRUCache.h"
#include "util/Units.h"

#include <deque>
#include <memory>
#include <mutex>

struct CachedPersistence : IPointsPersistence
{

  CachedPersistence(unit::byte memory_capacity, std::unique_ptr<IPointsPersistence> persistence);
  ~CachedPersistence() override;

  void persist_points(gsl::span<PointBuffer::PointReference> points,
                      const AABB& bounds,
                      const std::string& node_name) override;
  void persist_points(PointBuffer const& points,
                      const AABB& bounds,
                      const std::string& node_name) override;
  void persist_indices(gsl::span<MortonIndex64> indices, const std::string& node_name) override;

  void retrieve_points(const std::string& node_name, PointBuffer& points) override;
  void retrieve_indices(const std::string& node_name, std::vector<MortonIndex64>& indices) override;

private:
  unit::byte const _memory_capacity;

  std::unique_ptr<IPointsPersistence> _persistence;
  LRUCache<std::string, std::pair<PointBuffer, AABB>> _points_cache;
  // LRUCache<std::string, std::vector<MortonIndex64>> _indices_cache;
  std::mutex _cache_lock;
};