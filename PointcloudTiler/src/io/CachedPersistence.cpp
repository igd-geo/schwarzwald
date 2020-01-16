#include "io/CachedPersistence.h"

CachedPersistence::CachedPersistence(unit::byte memory_capacity,
                                     std::unique_ptr<IPointsPersistence> persistence)
  : _memory_capacity(memory_capacity)
  , _persistence(std::move(persistence))
  , _points_cache(memory_capacity)
{
  // The evict-handler will guarantee that points that don't fit in the cache get written to the
  // underlying IPointsPersistence
  _points_cache.add_evict_handler(
    [this](std::string const& key, std::pair<PointBuffer, AABB> const& value) {
      _persistence->persist_points(value.first, value.second, key);
    });
}
CachedPersistence::~CachedPersistence()
{
  // Clear cache, this will dump contents to the underlying IPointsPersistence
  _points_cache.clear();
}

void
CachedPersistence::persist_points(gsl::span<PointBuffer::PointReference> points,
                                  const AABB& bounds,
                                  const std::string& node_name)
{
  std::lock_guard guard{ _cache_lock };
  _points_cache.put(node_name, { PointBuffer{ points }, bounds });
}

void
CachedPersistence::persist_points(PointBuffer const& points,
                                  const AABB& bounds,
                                  const std::string& node_name)
{
  std::lock_guard guard{ _cache_lock };
  _points_cache.put(node_name, { points, bounds });
}

void
CachedPersistence::persist_indices(gsl::span<MortonIndex64> indices, const std::string& node_name)
{
  throw std::runtime_error{ "Not implemented" };
}

void
CachedPersistence::retrieve_points(const std::string& node_name, PointBuffer& points)
{
  std::lock_guard guard{ _cache_lock };
  // Try to load the points from the cache
  std::pair<PointBuffer, AABB> entry;
  if (_points_cache.try_get(node_name, entry)) {
    points = std::move(entry.first);
    return;
  }

  // Load from the underlying IPointsPersistence if that didn't work
  _persistence->retrieve_points(node_name, points);
}

void
CachedPersistence::retrieve_indices(const std::string& node_name,
                                    std::vector<MortonIndex64>& indices)
{
  throw std::runtime_error{ "Not implemented" };
}