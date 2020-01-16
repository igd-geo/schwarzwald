#include "io/Cesium3DTilesPersistence.h"

#include "PNTSReader.h"
#include "PNTSWriter.h"
#include "PointAttributes.hpp"
#include "Transformation.h"
#include "stuff.h"

#include <fstream>

Cesium3DTilesPersistence::Cesium3DTilesPersistence(const std::string& work_dir,
                                                   const PointAttributes& point_attributes,
                                                   const SRSTransformHelper& transform_helper)
  : _work_dir(work_dir)
  , _point_attributes(point_attributes)
  , _transform_helper(transform_helper)
{}

Cesium3DTilesPersistence::~Cesium3DTilesPersistence() {}

void
Cesium3DTilesPersistence::persist_points(gsl::span<PointBuffer::PointReference> points,
                                         const AABB& bounds,
                                         const std::string& node_name)
{
  PNTSWriter writer{ concat(_work_dir, "/", node_name, ".pnts"), _point_attributes };
  auto local_offset_to_world = setOriginToSmallestPoint(points);

  writer.write_points(points);
  writer.flush(local_offset_to_world);
}

void
Cesium3DTilesPersistence::persist_points(PointBuffer const& points,
                                         const AABB& bounds,
                                         const std::string& node_name)
{
  PNTSWriter writer{ concat(_work_dir, "/", node_name, ".pnts"), _point_attributes };
  auto points_copy = points;
  auto local_offset_to_world = setOriginToSmallestPoint(points_copy.positions());

  writer.write_points(points_copy);
  writer.flush(local_offset_to_world);
}

void
Cesium3DTilesPersistence::persist_indices(gsl::span<MortonIndex64> indices,
                                          const std::string& node_name)
{
  const auto file_path = concat(_work_dir, "/", node_name, ".idx");
  std::ofstream fs{ file_path, std::ios::out | std::ios::binary };
  if (!fs.is_open()) {
    std::cerr << "Could not write index file " << file_path << std::endl;
    return;
  }

  const uint64_t num_indices = indices.size();
  fs.write(reinterpret_cast<char const*>(&num_indices), sizeof(uint64_t));
  for (auto idx : indices) {
    const uint64_t idx_val = idx.get();
    fs.write(reinterpret_cast<char const*>(&idx_val), sizeof(uint64_t));
  }

  fs.flush();
  fs.close();
}

void
Cesium3DTilesPersistence::retrieve_points(const std::string& node_name, PointBuffer& points)
{
  const auto file_path = concat(_work_dir, "/", node_name, ".pnts");
  if (!std::experimental::filesystem::exists(file_path))
    return;

  auto pnts_content = readPNTSFile(file_path);
  points = std::move(pnts_content->points);

  // Transform back into world space
  for (auto& position : points.positions()) {
    position += pnts_content->rtc_center;
  }
}

void
Cesium3DTilesPersistence::retrieve_indices(const std::string& node_name,
                                           std::vector<MortonIndex64>& indices)
{
  static_assert(sizeof(MortonIndex64) == sizeof(uint64_t), "");

  const auto file_path = concat(_work_dir, "/", node_name, ".idx");
  std::ifstream fs{ file_path, std::ios::in | std::ios::binary };
  if (!fs.is_open()) {
    std::cerr << "Could not read index file " << file_path << std::endl;
    return;
  }

  fs.unsetf(std::ios::skipws);

  fs.seekg(0, std::ios::end);
  auto fileSize = fs.tellg();
  fs.seekg(0, std::ios::beg);

  std::vector<char> rawData;
  rawData.reserve(fileSize);

  rawData.insert(rawData.begin(), std::istream_iterator<char>(fs), std::istream_iterator<char>());

  const auto indices_begin =
    reinterpret_cast<MortonIndex64 const*>(rawData.data() + sizeof(size_t));
  const auto indices_end = reinterpret_cast<MortonIndex64 const*>(rawData.data() + rawData.size());

  indices = { indices_begin, indices_end };
}