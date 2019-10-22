#include "io/LASPersistence.h"

#include "LASPointReader.h"
#include "Transformation.h"
#include "laszip_api.h"

#include <experimental/filesystem>

#include <boost/scope_exit.hpp>

/**
 * Computes the scale factor for the LAS file based on the size of the given
 * bounding box. This code is adopted straight from Potree
 */
static double
compute_las_scale_from_bounds(const AABB& bounds)
{
  const auto bounds_diagonal = bounds.extent().length();
  if (bounds_diagonal > 1'000'000) {
    return 0.01;
  } else if (bounds_diagonal > 100'000) {
    return 0.001;
  } else if (bounds_diagonal > 1) {
    return 0.001;
  }
  return 0.0001;
}

LASPersistence::LASPersistence(const std::string& work_dir,
                               const PointAttributes& point_attributes,
                               Compressed compressed)
  : _work_dir(work_dir)
  , _point_attributes(point_attributes)
  , _compressed(compressed)
  , _file_extension(compressed == Compressed::Yes ? ".laz" : ".las")
{}

LASPersistence::~LASPersistence() {}

void
LASPersistence::persist_points(gsl::span<PointBuffer::PointReference> points,
                               const AABB& bounds,
                               const std::string& node_name)
{
  laszip_POINTER laswriter = nullptr;
  laszip_create(&laswriter);

  if (!laswriter) {
    std::cerr << "Could not create LAS writer for node " << node_name << std::endl;
    return;
  }

  BOOST_SCOPE_EXIT(&laswriter) { laszip_destroy(laswriter); }
  BOOST_SCOPE_EXIT_END

  laszip_header* las_header;
  if (laszip_get_header_pointer(laswriter, &las_header)) {
    std::cerr << "Could not write LAS header for node " << node_name << std::endl;
    return;
  }

  // las_header->file_source_ID = 0;
  // las_header->global_encoding = 0;
  // las_header->project_ID_GUID_data_1 = 0;
  // las_header->project_ID_GUID_data_2 = 0;
  // las_header->project_ID_GUID_data_3 = 0;
  // std::memset(las_header->project_ID_GUID_data_4, 0, 8);
  // las_header->header_size = sizeof(laszip_header);
  las_header->number_of_point_records = points.size();
  las_header->number_of_points_by_return[0] = points.size();
  las_header->number_of_points_by_return[1] = las_header->number_of_points_by_return[2] =
    las_header->number_of_points_by_return[3] = las_header->number_of_points_by_return[4] = 0;
  las_header->version_major = 1;
  las_header->version_minor = 2;
  std::memcpy(las_header->generating_software, "pointcloud_tiler", sizeof("pointcloud_tiler"));
  las_header->offset_to_point_data = las_header->header_size;
  las_header->number_of_variable_length_records = 0;
  las_header->point_data_format = 2;
  las_header->point_data_record_length = 26;

  // auto local_offset_to_world = setOriginToSmallestPoint(points);
  // las_header->x_offset = local_offset_to_world.x;
  // las_header->y_offset = local_offset_to_world.y;
  // las_header->z_offset = local_offset_to_world.z;
  for (auto& point : points) {
    point.position() -= bounds.min;
  }
  las_header->x_offset = bounds.min.x;
  las_header->y_offset = bounds.min.y;
  las_header->z_offset = bounds.min.z;
  las_header->min_x = bounds.min.x; // - local_offset_to_world.x;
  las_header->min_y = bounds.min.y; // - local_offset_to_world.y;
  las_header->min_z = bounds.min.z; // - local_offset_to_world.z;
  las_header->max_x = bounds.max.x; // - local_offset_to_world.x;
  las_header->max_y = bounds.max.y; // - local_offset_to_world.y;
  las_header->max_z = bounds.max.z; // - local_offset_to_world.z;

  las_header->x_scale_factor = las_header->y_scale_factor = las_header->z_scale_factor =
    compute_las_scale_from_bounds(bounds);

  const auto file_path = concat(_work_dir, "/", node_name, _file_extension);
  if (laszip_open_writer(
        laswriter, file_path.c_str(), (_compressed == LASPersistence::Compressed::Yes))) {
    std::cerr << "Could not write LAS file for node " << node_name << std::endl;
    return;
  }

  BOOST_SCOPE_EXIT(&laswriter) { laszip_close_writer(laswriter); }
  BOOST_SCOPE_EXIT_END

  laszip_point* laspoint;
  if (laszip_get_point_pointer(laswriter, &laspoint)) {
    std::cerr << "Could not write LAS points for node " << node_name << std::endl;
    return;
  }

  for (const auto& point_ref : points) {
    const auto pos = point_ref.position();
    laszip_F64 coordinates[3] = { pos.x, pos.y, pos.z };
    if (laszip_set_coordinates(laswriter, coordinates)) {
      std::cerr << "Could not set coordinates for LAS point at node " << node_name << std::endl;
      return;
    }

    const auto rgb = point_ref.rgbColor();
    if (rgb) {
      laspoint->rgb[0] = rgb->x;
      laspoint->rgb[1] = rgb->y;
      laspoint->rgb[2] = rgb->z;
      laspoint->rgb[3] = static_cast<uint8_t>(255);
    }

    const auto intensity = point_ref.intensity();
    if (intensity) {
      laspoint->intensity = *intensity;
    }

    const auto classification = point_ref.classification();
    if (classification) {
      laspoint->classification = *classification;
    }

    if (laszip_write_point(laswriter)) {
      std::cerr << "Could not write LAS point for node " << node_name << std::endl;
      return;
    }
  }
}

void
LASPersistence::persist_indices(gsl::span<MortonIndex64> indices, const std::string& node_name)
{
  // TODO REFACTOR This is identical to Cesium3DTilesPersistence
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
LASPersistence::retrieve_points(const std::string& node_name, PointBuffer& points)
{
  const auto file_path = concat(_work_dir, "/", node_name, _file_extension);
  if (!std::experimental::filesystem::exists(file_path))
    return;
  LIBLASReader las_reader{ file_path, _point_attributes };

  const auto offset = las_reader.getOffset();
  const auto num_points = las_reader.numPoints();
  points = las_reader.readNextBatch(num_points);

  for (auto& pos : points.positions()) {
    pos += offset;
  }
}

void
LASPersistence::retrieve_indices(const std::string& node_name, std::vector<MortonIndex64>& indices)
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