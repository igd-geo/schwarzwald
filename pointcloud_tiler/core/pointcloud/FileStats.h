#pragma once

#include "math/AABB.h"
#include "util/Definitions.h"
#include "algorithms/Hash.h"

#include <unordered_map>


/**
 * Common metadata information for a single point cloud file of arbitrary type. This is
 * the smallest common denominator of the typed Metadata in 'PointcloudFile.h'
 */
struct CommonMetadata
{
  size_t points_count;
  AABB bounds;
};

/**
 * Stores common metadata for all point cloud files in the active dataset
 */
struct DatasetMetadata
{
  DatasetMetadata();
  DatasetMetadata(const DatasetMetadata&) = default;
  DatasetMetadata(DatasetMetadata&&) = default;
  DatasetMetadata& operator=(const DatasetMetadata&) = default;
  DatasetMetadata& operator=(DatasetMetadata&&) = default;

  /**
   * Total number of points in the dataset
   */
  size_t total_points_count() const { return _total_points_count; }
  /**
   * Tight-fitting bounds of the dataset
   */
  const AABB& total_bounds_tight() const { return _total_bounds_tight; }
  /**
   * Cubic bounds of the dataset
   */
  const AABB& total_bounds_cubic() const { return _total_bounds_cubic; }
  /**
   * Cubic bounds, shifted to the coordinate system origin
   */
  AABB total_bounds_cubic_at_origin() const;

  /**
   * Returns the CommonMetadata of all files
   */
  const auto& get_all_files_metadata() const { return _metadata_per_file; }
  /**
   * Adds metadata for the given file
   */
  void add_file_metadata(const fs::path& file_path, size_t points_count, const AABB& bounds);

private:
  size_t _total_points_count;
  AABB _total_bounds_tight;
  AABB _total_bounds_cubic;
  std::unordered_map<fs::path, CommonMetadata, util::PathHash> _metadata_per_file;
};