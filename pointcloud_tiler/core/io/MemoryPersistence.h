#pragma once

#include "datastructures/MortonIndex.h"
#include "datastructures/PointBuffer.h"
#include "math/AABB.h"
#include "pointcloud/PointAttributes.h"

#include <mutex>
#include <unordered_map>

/**
 * Persists points in memory
 */
struct MemoryPersistence
{
  explicit MemoryPersistence(const PointAttributes& input_attributes);
  MemoryPersistence(const MemoryPersistence&) = delete;
  MemoryPersistence(MemoryPersistence&&) = default;
  MemoryPersistence& operator=(const MemoryPersistence&) = delete;
  MemoryPersistence& operator=(MemoryPersistence&&) = default;

  template<typename Iter>
  void persist_points(Iter points_begin,
                      Iter points_end,
                      const AABB& bounds,
                      const std::string& node_name)
  {
    std::lock_guard<std::mutex> lock{ *_lock };
    auto& buffer = _points_cache[node_name];
    std::for_each(
      points_begin, points_end, [&buffer](const auto& point_ref) { buffer.push_point(point_ref); });
  }

  void persist_points(PointBuffer const& points, const AABB& bounds, const std::string& node_name);

  void retrieve_points(const std::string& node_name, PointBuffer& points);

  bool node_exists(const std::string& node_name) const;

  inline bool is_lossless() const { return true; }

  const auto& get_points() const { return _points_cache; }

private:
  PointAttributes _input_attributes;
  // No output attributes, the MemoryPersistence is mostly for unit testing, so it doesn't matter if
  // we store more attributes than strictly necessary. Input attributes however are important, so
  // that retrieve_points returns a PointBuffer with the correct schema

  std::unique_ptr<std::mutex> _lock;
  std::unordered_map<std::string, PointBuffer> _points_cache;
};