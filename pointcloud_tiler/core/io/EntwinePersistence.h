#pragma once

#include "datastructures/PointBuffer.h"
#include "io/LASFile.h"
#include "io/LASPersistence.h"
#include "math/AABB.h"
#include "pointcloud/PointAttributes.h"
#include "util/Definitions.h"
#include "util/stuff.h"

#include <mutex>
#include <unordered_map>

enum class EntwineFormat
{
  LAS,
  LAZ
  // Binary
};

struct EptSchemaEntry
{
  std::string name;
  std::optional<double> offset;
  std::optional<double> scale;
  uint32_t size;
  std::string type;
};

/**
 * Converts PointAttributes to an Entwine schema
 */
std::vector<EptSchemaEntry>
point_attributes_to_ept_schema(const PointAttributes& point_attributes);

struct EptSRS
{
  std::string authority;
  std::string horizontal;
  std::string wkt;
};

struct EptJson
{
  AABB bounds, conforming_bounds;
  EntwineFormat data_type;
  std::string hierarchy_type;
  size_t points;
  std::vector<EptSchemaEntry> schema;
  double span;
  EptSRS srs;
  std::string version;
};

/**
 * Writes the ept.json file to the given path
 */
void
write_ept_json(const fs::path& file_path, const EptJson& ept_json);

struct EntwinePersistence
{
  EntwinePersistence(const std::string& work_dir,
                     const PointAttributes& input_attributes,
                     const PointAttributes& output_attributes,
                     EntwineFormat format);
  EntwinePersistence(EntwinePersistence&&) = default;
  ~EntwinePersistence();

  template<typename Iter>
  void persist_points(Iter points_begin,
                      Iter points_end,
                      const AABB& bounds,
                      const std::string& node_name)
  {
    const auto num_points = std::distance(points_begin, points_end);
    if (!num_points)
      return;

    const auto entwine_name = potree_name_to_entwine_name(node_name);

    _las_persistence.persist_points(points_begin, points_end, bounds, entwine_name);

    std::lock_guard guard{ *_hierarchy_lock };
    _hierarchy[entwine_name] = num_points;
  }

  void persist_points(PointBuffer const& points, const AABB& bounds, const std::string& node_name);

  void retrieve_points(const std::string& node_name, PointBuffer& points);

  bool node_exists(const std::string& node_name) const;

  inline bool is_lossless() const { return false; }

  auto& las_persistence() { return _las_persistence; }

private:
  fs::path _work_dir;
  EntwineFormat _format;
  std::string _file_extension;

  LASPersistence _las_persistence;

  std::unique_ptr<std::mutex> _hierarchy_lock;
  std::unordered_map<std::string, size_t> _hierarchy;

  static std::string potree_name_to_entwine_name(const std::string& potree_name);
};