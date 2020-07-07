#pragma once

#include <variant>

#include "BinaryPersistence.h"
#include "Cesium3DTilesPersistence.h"
#include "EntwinePersistence.h"
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

  inline bool node_exists(const std::string& node_name) const
  {
    return std::visit([&](auto& impl) { return impl.node_exists(node_name); }, _impl);
  }

  inline bool is_lossless() const
  {
    return std::visit([&](auto& impl) { return impl.is_lossless(); }, _impl);
  }

  template<typename T>
  const T& get() const
  {
    return std::get<T>(_impl);
  }

private:
  std::variant<BinaryPersistence,
               Cesium3DTilesPersistence,
               LASPersistence,
               MemoryPersistence,
               EntwinePersistence>
    _impl;
};

/**
 * Factory function for creating a PointsPersistence for the given format and
 * parameters
 */
PointsPersistence
make_persistence(OutputFormat format,
                 const fs::path& output_directory,
                 const PointAttributes& input_attributes,
                 const PointAttributes& output_attributes,
                 RGBMapping rgb_mapping,
                 float spacing,
                 const AABB& bounds);

/**
 * Returns the set of point attributes supported by the given output format
 */
PointAttributes
supported_output_attributes_for_format(OutputFormat format);