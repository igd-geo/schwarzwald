#include "io/LASFile.h"
#include "pointcloud/PointAttributes.h"
#include "util/stuff.h"

#include <boost/format.hpp>
#include <expected.hpp>
#include <limits>

#pragma region Helpers

constexpr static laszip_I32 LASZIP_NO_ERROR = 0;

namespace las {
struct TryHelper
{
  template<typename Call>
  void or_else(Call call)
  {
    if (error_code == LASZIP_NO_ERROR)
      return;
    call(error_code);
  }

  laszip_I32 error_code;
};

TryHelper
ctry(laszip_I32 las_error_code)
{
  return { las_error_code };
}

static bool
is_compressed_las_file(fs::path const& path)
{
  return path.extension() == ".laz";
}
} // namespace las

/**
 * Get world coordinates from the given LAS point
 */
Vector3<double>
position_from_las_point(laszip_point const& point, laszip_header const& las_header)
{
  return { las_header.x_offset + point.X * las_header.x_scale_factor,
           las_header.y_offset + point.Y * las_header.y_scale_factor,
           las_header.z_offset + point.Z * las_header.z_scale_factor };
}

#pragma endregion

#pragma region LASInputIterator

LASInputIterator::LASInputIterator()
  : _las_reader(nullptr)
  , _index(std::numeric_limits<size_t>::max())
  , _size(std::numeric_limits<size_t>::max())
{}
LASInputIterator::LASInputIterator(LASFile const& las_file, size_t index)
  : _las_reader(las_file._laszip_handle)
  , _index(index)
  , _size(las_file.size())
{
// Index must be a valid index inside the file. For end iterator, use default
// constructor!
#ifdef DEBUG
  assert(index < las_file.size())
#endif
    laszip_seek_point(_las_reader, static_cast<laszip_I64>(index));
  laszip_read_point(_las_reader);
}

bool
LASInputIterator::operator==(const LASInputIterator& other) const
{
  return !operator!=(other);
}
bool
LASInputIterator::operator!=(const LASInputIterator& other) const
{
  return _las_reader != other._las_reader || _index != other._index;
}

LASInputIterator&
LASInputIterator::operator++()
{
  ++_index;
  if (_index == _size) {
    *this = {};
    return *this;
  }
  laszip_read_point(_las_reader);
  return *this;
}

laszip_point const& LASInputIterator::operator*() const
{
  laszip_point* point;
  laszip_get_point_pointer(_las_reader, &point);
  return *point;
}

size_t
LASInputIterator::distance_to_end() const
{
  return _size - _index;
}

#pragma endregion

#pragma region LASOutputIterator
LASOutputIterator::LASOutputIterator()
  : _las_writer(nullptr)
{}
LASOutputIterator::LASOutputIterator(LASFile& las_file)
  : _las_writer(las_file._laszip_handle)
{}

bool
LASOutputIterator::operator==(const LASOutputIterator& other) const
{
  return !operator!=(other);
}
bool
LASOutputIterator::operator!=(const LASOutputIterator& other) const
{
  return _las_writer != other._las_writer;
}

LASOutputIterator&
LASOutputIterator::operator++()
{
  return *this;
}

LASOutputIterator& LASOutputIterator::operator*()
{
  return *this;
}

void
LASOutputIterator::operator=(laszip_point const& point)
{
  laszip_set_point(_las_writer, &point);
  laszip_write_point(_las_writer);
}
#pragma endregion

#pragma region LASFile

LASFile::LASFile()
  : _laszip_handle(nullptr)
  , _state(State::Closed)
{}

LASFile::LASFile(const fs::path& path, OpenMode file_open_mode)
  : LASFile()
{
  open(path, file_open_mode);
}

LASFile::LASFile(LASFile&& other)
  : _laszip_handle(other._laszip_handle)
  , _file_path(other._file_path)
  , _state(other._state)
{
  other._laszip_handle = nullptr;
  other._state = State::Closed;
}

LASFile::~LASFile()
{
  if (_state == State::Closed)
    return;

  close();
}

LASFile&
LASFile::operator=(LASFile&& other)
{
  close();

  _laszip_handle = other._laszip_handle;
  _file_path = other._file_path;
  _state = other._state;

  other._laszip_handle = nullptr;
  other._state = State::Closed;

  return *this;
}

laszip_header const&
LASFile::get_metadata() const
{
  laszip_header* las_header;
  laszip_get_header_pointer(_laszip_handle, &las_header);
  return *las_header;
}

void
LASFile::set_metadata(laszip_header const& metadata)
{
  assert(_state == State::OpenWrite);

  // HACK LASzip only writes the header when the writer is OPENED...
  laszip_close_writer(_laszip_handle);
  laszip_set_header(_laszip_handle, &metadata);
  laszip_open_writer(_laszip_handle, _file_path.c_str(), las::is_compressed_las_file(_file_path));
}

size_t
LASFile::size() const
{
  const auto& las_header = get_metadata();
  if (las_header.version_major >= 1 && las_header.version_minor >= 4) {
    return las_header.extended_number_of_point_records;
  }
  return las_header.number_of_point_records;
}

LASOutputIterator
LASFile::begin()
{
  assert(_state == State::OpenWrite);
  return LASOutputIterator{ *this };
}

LASOutputIterator
LASFile::end()
{
  assert(_state == State::OpenWrite);
  return {};
}

LASInputIterator
LASFile::cbegin() const
{
  assert(_state == State::OpenRead);
  return LASInputIterator{ *this, 0 };
}

LASInputIterator
LASFile::cend() const
{
  assert(_state == State::OpenRead);
  return {};
}

void
LASFile::open(fs::path const& path, OpenMode file_open_mode)
{
  assert(_state == State::Closed);

  las::ctry(laszip_create(&_laszip_handle)).or_else([](auto ec) {
    throw new std::runtime_error{
      (boost::format("Could not create LASZip handle (error code %1%)") % ec).str()
    };
  });

  switch (file_open_mode) {
    case OpenMode::Read: {
      laszip_BOOL is_compressed;
      las::ctry(laszip_open_reader(_laszip_handle, path.c_str(), &is_compressed))
        .or_else([this, &path](auto ec) {
          laszip_destroy(_laszip_handle);

          throw new std::runtime_error{
            (boost::format("Could not open LAS reader for file %1% (error code %2%)") %
             path.string() % ec)
              .str()
          };
        });
      _state = State::OpenRead;
    } break;
    case OpenMode::Write: {
      las::ctry(laszip_open_writer(_laszip_handle, path.c_str(), las::is_compressed_las_file(path)))
        .or_else([this, &path](auto ec) {
          laszip_destroy(_laszip_handle);

          throw new std::runtime_error{
            (boost::format("Could not open LAS writer for file %1% (error code %2%)") %
             path.string() % ec)
              .str()
          };
        });
      _state = State::OpenWrite;
    } break;
  }

  _file_path = path;
}

void
LASFile::close()
{
  switch (_state) {
    case State::OpenRead:
      las::ctry(laszip_close_reader(_laszip_handle)).or_else([](auto ec) {
        throw std::runtime_error{
          (boost::format("Could not close LAS reader (error code %1%)") % ec).str()
        };
      });
      break;
    case State::OpenWrite:
      las::ctry(laszip_close_writer(_laszip_handle)).or_else([](auto ec) {
        throw std::runtime_error{
          (boost::format("Could not close LAS writer (error code %1%)") % ec).str()
        };
      });
      break;
    case State::Closed:
      return;
  }

  las::ctry(laszip_destroy(_laszip_handle)).or_else([](auto ec) {
    throw std::runtime_error{
      (boost::format("Could not release LASzip handle (error code %1%)") % ec).str()
    };
  });

  _laszip_handle = nullptr;
  _state = State::Closed;
}

void
LASFile::flush()
{
  assert(_state == State::OpenWrite);

  // I don't see another way to flush using LASzip but to close and re-open the
  // writer...
  close();
  open(_file_path, OpenMode::Write);
}

#pragma endregion

#pragma region PointcloudFileImpl

AABB
get_bounds_from_las_header(laszip_header const& header)
{
  return { { header.min_x, header.min_y, header.min_z },
           { header.max_x, header.max_y, header.max_z } };
}

Vector3<double>
get_offset_from_las_header(laszip_header const& header)
{
  return { header.x_offset, header.y_offset, header.z_offset };
}

bool
las_file_has_attribute(laszip_header const& header, PointAttribute const& attribute)
{
  switch (attribute.ordinal) {
    case attributes::POSITION_CARTESIAN:
      return true; // LAS always has positions
    case attributes::COLOR_PACKED:
      return header.point_data_format == 2 || header.point_data_format == 3;
    case attributes::INTENSITY:
    case attributes::COLOR_FROM_INTENSITY:
      return true;
    case attributes::NORMAL:
    case attributes::NORMAL_OCT16:
    case attributes::NORMAL_SPHEREMAPPED:
      return false; // TODO We could look in the additional data fields, but only
                    // from the header we can't tell, so this requires an API
                    // change
    case attributes::CLASSIFICATION:
      return true; // LAS always has classifications
    default:
      throw std::runtime_error{ "Unrecognized attribute type!" };
  }
}

LASInputIterator
las_read_points(LASInputIterator begin,
                size_t count,
                laszip_header const& header,
                PointAttributes const& attributes,
                PointBuffer& points)
{
  const auto to_read_count = std::min(count, begin.distance_to_end());

  std::vector<Vector3<double>> positions;
  std::vector<Vector3<uint8_t>> colors;
  std::vector<Vector3<float>> normals;
  std::vector<uint16_t> intensities;
  std::vector<uint8_t> classifications;

  const auto has_color_from_intensity = has_attribute(attributes, attributes::COLOR_FROM_INTENSITY);
  const auto has_colors = has_attribute(attributes, attributes::COLOR_PACKED);
  // const auto has_normals = has_attribute(attributes, attributes::NORMAL) ||
  //                         has_attribute(attributes, attributes::NORMAL_OCT16)
  //                         || has_attribute(attributes,
  //                         attributes::NORMAL_SPHEREMAPPED);
  const auto has_intensities = has_attribute(attributes, attributes::INTENSITY);
  const auto has_classifications = has_attribute(attributes, attributes::CLASSIFICATION);

  positions.reserve(to_read_count);
  if (has_colors || has_color_from_intensity) {
    colors.reserve(to_read_count);
  }
  //   if (has_normals) {
  //     normals.reserve(to_read_count);
  //   }
  if (has_intensities) {
    intensities.reserve(to_read_count);
  }
  if (has_classifications) {
    classifications.reserve(to_read_count);
  }

  for (size_t idx = 0; idx < to_read_count; ++idx, ++begin) {
    auto& point = *begin;
    positions.push_back(position_from_las_point(point, header));
    if (has_colors) {
      // TODO Implement correct color scaling
      colors.emplace_back(static_cast<uint8_t>(point.rgb[0] >> 8),
                          static_cast<uint8_t>(point.rgb[1] >> 8),
                          static_cast<uint8_t>(point.rgb[2] >> 8));
    }
    if (has_color_from_intensity) {
      colors.push_back(intensityToRGB_Log(point.intensity));
    }
    // TODO Support normals
    // if(has_normals) {
    //}
    if (has_intensities) {
      intensities.push_back(point.intensity);
    }
    if (has_classifications) {
      classifications.push_back(point.classification);
    }
  }

  points = { to_read_count,      std::move(positions),   std::move(colors),
             std::move(normals), std::move(intensities), std::move(classifications) };
  return begin;
}

#pragma endregion