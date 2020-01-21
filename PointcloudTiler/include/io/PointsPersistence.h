#pragma once

#include <variant>

#include "BinaryPersistence.h"
#include "Cesium3DTilesPersistence.h"
#include "LASPersistence.h"
#include "MemoryPersistence.h"

struct PointsPersistence
{
  template<typename T>
  explicit PointsPersistence(T impl)
    : _impl(std::move(impl))
  {}

  PointsPersistence(PointsPersistence const&) = delete;
  PointsPersistence(PointsPersistence&&) = default;
  PointsPersistence& operator=(PointsPersistence const&) = delete;
  PointsPersistence& operator=(PointsPersistence&&) = default;

  template<typename Iter>
  void persist_points(Iter points_begin,
                      Iter points_end,
                      const AABB& bounds,
                      const std::string& node_name)
  {
    std::visit(
      [&](auto& impl) { impl.persist_points(points_begin, points_end, bounds, node_name); }, _impl);
  }

  inline void persist_points(PointBuffer const& points,
                             const AABB& bounds,
                             const std::string& node_name)
  {
    std::visit([&](auto& impl) { impl.persist_points(points, bounds, node_name); }, _impl);
  }

  inline void retrieve_points(const std::string& node_name, PointBuffer& points)
  {
    std::visit([&](auto& impl) { impl.retrieve_points(node_name, points); }, _impl);
  }

  template<typename T>
  const T& get() const
  {
    return std::get<T>(_impl);
  }

private:
  std::variant<BinaryPersistence, Cesium3DTilesPersistence, LASPersistence, MemoryPersistence>
    _impl;
};