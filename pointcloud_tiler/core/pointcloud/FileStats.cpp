#include "pointcloud/FileStats.h"

#include <boost/format.hpp>

DatasetMetadata::DatasetMetadata()
  : _total_points_count(0)
{}

void
DatasetMetadata::add_file_metadata(const fs::path& file_path,
                                   size_t points_count,
                                   const AABB& bounds)
{
  const auto iter_to_metadata = _metadata_per_file.find(file_path);
  if (iter_to_metadata != std::end(_metadata_per_file)) {
    throw std::invalid_argument{
      (boost::format("Metadata for file %1% has already been added!") % file_path).str()
    };
  }

  auto& common_metadata = _metadata_per_file[file_path];
  common_metadata.points_count = points_count;
  common_metadata.bounds = bounds;

  _total_points_count += points_count;
  _total_bounds_tight.update(bounds);
  _total_bounds_cubic = _total_bounds_tight.cubic();
}

AABB
DatasetMetadata::total_bounds_cubic_at_origin() const
{
  return {
    _total_bounds_cubic.min - _total_bounds_cubic.getCenter(),
    _total_bounds_cubic.max - _total_bounds_cubic.getCenter(),
  };
}