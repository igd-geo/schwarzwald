#include "io/MemoryPersistence.h"

void
MemoryPersistence::persist_points(gsl::span<Potree::PointBuffer::PointReference> points,
                                  const std::string& node_name)
{
  auto& buffer = _points_cache[node_name];
  for (auto& point : points)
    buffer.push_point(point);
}

void
MemoryPersistence::persist_indices(gsl::span<OctreeNodeKey64> indices, const std::string& node_name)
{
  auto& buffer = _indices_cache[node_name];
  buffer = { indices.begin(), indices.end() };
}

void
MemoryPersistence::persist_hierarchy(const std::string& node_name, const Potree::AABB& bounds)
{}

void
MemoryPersistence::retrieve_points(const std::string& node_name, Potree::PointBuffer& points)
{
  points = _points_cache[node_name];
}

void
MemoryPersistence::retrieve_indices(const std::string& node_name,
                                    std::vector<OctreeNodeKey64>& indices)
{
  indices = _indices_cache[node_name];
}

void
MemoryPersistence::retrieve_hierarchy(const std::string& node_name, Potree::AABB& bounds)
{}