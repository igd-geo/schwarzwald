#pragma once

#include "IPointsPersistence.h"

namespace Potree {
struct SRSTransformHelper;
class PointAttributes;
}

/**
 * Sink for writing 3D Tiles files
 */
struct Cesium3DTilesPersistence : IPointsPersistence
{
  Cesium3DTilesPersistence(const std::string& work_dir,
                           const Potree::PointAttributes& point_attributes,
                           const Potree::SRSTransformHelper& transform_helper);
  ~Cesium3DTilesPersistence() override;

  void persist_points(gsl::span<Potree::PointBuffer::PointReference> points,
                      const std::string& node_name) override;
  void persist_indices(gsl::span<OctreeNodeKey64> indices, const std::string& node_name) override;
  void persist_hierarchy(const std::string& node_name, const Potree::AABB& bounds) override;

  void retrieve_points(const std::string& node_name, Potree::PointBuffer& points) override;
  void retrieve_indices(const std::string& node_name,
                        std::vector<OctreeNodeKey64>& indices) override;
  void retrieve_hierarchy(const std::string& node_name, Potree::AABB& bounds) override;

private:
  std::string _work_dir;
  const Potree::PointAttributes& _point_attributes;
  const Potree::SRSTransformHelper& _transform_helper;
};