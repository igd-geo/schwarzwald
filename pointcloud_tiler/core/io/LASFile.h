#pragma once

#include "io/PointcloudFile.h"
#include "util/Definitions.h"

#include <experimental/filesystem>
#include <iterator>
#include <laszip_api.h>

class PointAttribute;

struct LASFile;

/**
 * Input iterator into a LAS/LAZ file
 */
struct LASInputIterator
{
  using iterator_category = std::input_iterator_tag;
  using value_type = laszip_point;
  using difference_type = std::ptrdiff_t;
  using pointer = laszip_point const*;
  using reference = laszip_point const&;

  LASInputIterator();
  LASInputIterator(LASFile const& las_file, size_t index);

  bool operator==(const LASInputIterator& other) const;
  bool operator!=(const LASInputIterator& other) const;

  LASInputIterator& operator++();

  laszip_point const& operator*() const;

  size_t distance_to_end() const;

private:
  laszip_POINTER _las_reader;
  size_t _index;
  size_t _size;
};

/**
 * Output iterator for a LAS/LAZ file
 */
struct LASOutputIterator
{
  friend struct PointProxy;

  // Typedefs like https://en.cppreference.com/w/cpp/iterator/ostream_iterator
  using iterator_category = std::input_iterator_tag;
  using value_type = void;
  using difference_type = void;
  using pointer = void;
  using reference = void;

  LASOutputIterator();
  explicit LASOutputIterator(LASFile& las_file);

  bool operator==(const LASOutputIterator& other) const;
  bool operator!=(const LASOutputIterator& other) const;

  LASOutputIterator& operator++(); // NO-OP
  LASOutputIterator& operator*();  // NO-OP
  void operator=(laszip_point const& point);

private:
  laszip_POINTER _las_writer;
};

/**
 * LAS/LAZ file abstraction in the form of an iterable container
 */
struct LASFile
{
  enum class OpenMode
  {
    Read,
    Write
  };

  friend struct LASInputIterator;
  friend struct LASOutputIterator;

  using iterator = LASOutputIterator;
  using const_iterator = LASInputIterator;
  using metadata = laszip_header;

  LASFile();
  LASFile(fs::path const& path, OpenMode file_open_mode);
  LASFile(LASFile const&) = delete;
  LASFile(LASFile&&);
  ~LASFile();

  LASFile& operator=(LASFile const&) = delete;
  LASFile& operator=(LASFile&&);

  metadata const& get_metadata() const;
  void set_metadata(metadata const& metadata);
  size_t size() const;

  LASOutputIterator begin();
  LASOutputIterator end();

  LASInputIterator begin() const { return cbegin(); }
  LASInputIterator end() const { return cend(); }

  LASInputIterator cbegin() const;
  LASInputIterator cend() const;

  void open(fs::path const& path, OpenMode file_open_mode);
  void flush();
  void close();

private:
  enum class State
  {
    Closed,
    OpenRead,
    OpenWrite
  };

  laszip_POINTER _laszip_handle;
  fs::path _file_path;
  State _state;
};

AABB
get_bounds_from_las_header(laszip_header const& header);
Vector3<double>
get_offset_from_las_header(laszip_header const& header);
bool
las_file_has_attribute(laszip_header const& header, PointAttribute const& attribute);
bool
las_file_has_all_attributes(laszip_header const& header,
                            PointAttributes const& attributes,
                            PointAttributes* missing_attributes = nullptr);

/**
 * Get world coordinates from the given LAS point
 */
Vector3<double>
position_from_las_point(laszip_point const& point, laszip_header const& las_header);

LASInputIterator
las_read_points(LASInputIterator begin,
                size_t count,
                laszip_header const& metadata,
                PointAttributes const& attributes,
                PointBuffer& points);

namespace pc {
template<>
inline AABB
get_bounds(LASFile const& f)
{
  return get_bounds_from_las_header(f.get_metadata());
}

template<>
inline Vector3<double>
get_offset(LASFile const& f)
{
  return get_offset_from_las_header(f.get_metadata());
}

template<>
inline size_t
get_point_count(LASFile const& f)
{
  return f.size();
}

template<>
inline bool
has_attribute(LASFile const& f, PointAttribute const& attribute)
{
  return las_file_has_attribute(f.get_metadata(), attribute);
}

template<>
inline LASInputIterator
read_points(LASInputIterator begin,
            size_t count,
            laszip_header const& header,
            PointAttributes const& attributes,
            PointBuffer& points)
{
  return las_read_points(begin, count, header, attributes, points);
}
} // namespace pc