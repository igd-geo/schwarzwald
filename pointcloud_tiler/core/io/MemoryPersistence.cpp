#include "io/MemoryPersistence.h"

MemoryPersistence::MemoryPersistence()
  : _lock(std::make_unique<std::mutex>())
{}

void
MemoryPersistence::persist_points(PointBuffer const& points,
                                  const AABB& bounds,
                                  const std::string& node_name)
{
  std::lock_guard<std::mutex> lock{ *_lock };
  auto& buffer = _points_cache[node_name];
  buffer = points;
}

void
MemoryPersistence::retrieve_points(const std::string& node_name, PointBuffer& points)
{
  std::lock_guard<std::mutex> lock{ *_lock };
  points = _points_cache[node_name];
}

bool
MemoryPersistence::node_exists(const std::string& node_name) const
{
  std::lock_guard<std::mutex> lock{ *_lock };
  return _points_cache.find(node_name) != std::end(_points_cache);
}