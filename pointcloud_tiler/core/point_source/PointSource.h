#pragma once

#include <experimental/filesystem>
#include <functional>
#include <optional>
#include <vector>

#include "datastructures/PointBuffer.h"
#include "io/PointcloudFactory.h"
#include "pointcloud/PointAttributes.h"
#include "util/Error.h"

namespace fs = std::experimental::filesystem;

struct PointSource {
  using Transform = std::function<void(PointBuffer &)>;

  PointSource(std::vector<fs::path> files, util::IgnoreErrors errors_to_ignore);

  std::optional<PointBuffer> read_next(size_t count,
                                       const PointAttributes &point_attributes);

  void add_transformation(Transform transform);

private:
  // TODO We can create this type through some meta-programming from the
  // PointFile type
  using CurrentFileCursor = std::variant<typename LASFile::const_iterator>;

  bool try_open_file(std::vector<fs::path>::const_iterator file_cursor);
  bool move_to_next_file();

  std::vector<fs::path> _files;
  util::IgnoreErrors _errors_to_ignore;

  std::vector<fs::path>::const_iterator _file_cursor;

  std::optional<PointFile> _current_file;
  std::optional<CurrentFileCursor> _current_file_cursor;

  std::vector<Transform> _transformations;
};