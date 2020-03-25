#include "PointSource.h"

#include "io/PointcloudFile.h"
#include "terminal/stdout_helper.h"

#include <boost/format.hpp>

PointSource::PointSource(std::vector<fs::path> files)
  : _files(std::move(files))
{
  // Try to open the first file (the first file that can be opened)
  _file_cursor = std::begin(_files);
  while (_file_cursor != std::end(_files) && !try_open_file(_file_cursor)) {
    ++_file_cursor;
  }
}

std::optional<PointBuffer>
PointSource::read_next(size_t count, const PointAttributes& attributes)
{
  if (!_current_file)
    return std::nullopt;

  return std::visit(
    [this, count, &attributes](auto& typed_file) -> std::optional<PointBuffer> {
      const auto& metadata = pc::metadata(typed_file);
      auto point_buffer = std::visit(
        [this, count, &attributes, &typed_file, &metadata](auto& typed_file_cursor) -> PointBuffer {
          PointBuffer point_buffer;
          typed_file_cursor =
            pc::read_points(typed_file_cursor, count, metadata, attributes, point_buffer);

          // If at end of current file, move to next file
          if (typed_file_cursor == std::cend(typed_file)) {
            move_to_next_file();
          }

          return point_buffer;
        },
        *_current_file_cursor);

      if (point_buffer.empty())
        return std::nullopt;

      // Apply all transformations
      for (auto& transformation : _transformations) {
        transformation(point_buffer);
      }

      return { std::move(point_buffer) };
    },
    *_current_file);
}

void
PointSource::add_transformation(Transform transform)
{
  _transformations.push_back(std::move(transform));
}

bool
PointSource::try_open_file(std::vector<fs::path>::const_iterator file_cursor)
{
  if (file_cursor == std::end(_files))
    return false;

  return open_point_file(*file_cursor)
    .map([this](PointFile point_file) mutable {
      return std::visit(
        [this](auto& typed_file) mutable {
          _current_file_cursor = { std::cbegin(typed_file) };
          _current_file = { std::move(typed_file) };
          return true;
        },
        point_file);
    })
    .or_else([file_cursor](std::error_code ec) {
      util::write_log(
        (boost::format("Could not open file %1% (%2%)") % file_cursor->string() % ec.message())
          .str());
    })
    .value_or(false);
}

bool
PointSource::move_to_next_file()
{
  ++_file_cursor;
  while (_file_cursor != std::end(_files) && !try_open_file(_file_cursor)) {
    ++_file_cursor;
  }
  if (_file_cursor == std::end(_files)) {
    _current_file = std::nullopt;
    _current_file_cursor = std::nullopt;
    return false;
  }
  return true;
}