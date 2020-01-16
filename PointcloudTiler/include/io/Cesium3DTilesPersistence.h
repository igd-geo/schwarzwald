#pragma once

#include "IPointsPersistence.h"
#include "PointAttributes.hpp"

struct SRSTransformHelper;

/**
 * Sink for writing 3D Tiles files
 */
struct Cesium3DTilesPersistence : IPointsPersistence
{
  Cesium3DTilesPersistence(const std::string& work_dir,
                           const PointAttributes& point_attributes,
                           const SRSTransformHelper& transform_helper);
  ~Cesium3DTilesPersistence() override;

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
  std::string _work_dir;
  const PointAttributes& _point_attributes;
  const SRSTransformHelper& _transform_helper;
};