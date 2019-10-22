#pragma once

#include "IPointsPersistence.h"

struct SRSTransformHelper;
class PointAttributes;

/**
 * Sink for writing LAS files
 */
struct LASPersistence : IPointsPersistence
{
  enum class Compressed
  {
    No,
    Yes
  };

  LASPersistence(const std::string& work_dir,
                 const PointAttributes& point_attributes,
                 Compressed compressed = Compressed::No);
  ~LASPersistence() override;

  void persist_points(gsl::span<PointBuffer::PointReference> points,
                      const AABB& bounds,
                      const std::string& node_name) override;
  void persist_indices(gsl::span<MortonIndex64> indices, const std::string& node_name) override;

  void retrieve_points(const std::string& node_name, PointBuffer& points) override;
  void retrieve_indices(const std::string& node_name, std::vector<MortonIndex64>& indices) override;

private:
  std::string _work_dir;
  const PointAttributes& _point_attributes;
  Compressed _compressed;
  std::string _file_extension;
};