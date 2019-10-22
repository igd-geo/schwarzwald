#include "io/MemoryPersistence.h"

void
MemoryPersistence::persist_points(gsl::span<PointBuffer::PointReference> points,
                                  const AABB& bounds,
                                  const std::string& node_name)
{
  std::lock_guard<std::mutex> lock{ _lock };
  auto& buffer = _points_cache[node_name];
  for (auto& point : points)
    buffer.push_point(point);
}

void
MemoryPersistence::persist_indices(gsl::span<MortonIndex64> indices, const std::string& node_name)
{
  std::lock_guard<std::mutex> lock{ _lock };
  auto& buffer = _indices_cache[node_name];
  buffer = { indices.begin(), indices.end() };
}

void
MemoryPersistence::retrieve_points(const std::string& node_name, PointBuffer& points)
{
  std::lock_guard<std::mutex> lock{ _lock };
  points = _points_cache[node_name];
}

void
MemoryPersistence::retrieve_indices(const std::string& node_name,
                                    std::vector<MortonIndex64>& indices)
{
  std::lock_guard<std::mutex> lock{ _lock };
  indices = _indices_cache[node_name];
}