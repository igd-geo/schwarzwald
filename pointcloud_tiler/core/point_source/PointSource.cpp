#include "PointSource.h"

#include "io/PointcloudFile.h"
#include "terminal/stdout_helper.h"

#include <boost/format.hpp>

PointSource::PointSource(std::vector<fs::path> files, util::IgnoreErrors errors_to_ignore)
  : _files(std::move(files))
  , _errors_to_ignore(errors_to_ignore)
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
          try {
            typed_file_cursor =
              pc::read_points(typed_file_cursor, count, metadata, attributes, point_buffer);
          } catch (const std::exception& ex) {
            throw util::chain_error(
              ex,
              (boost::format("Could not read points from file %1%") % _file_cursor->string())
                .str());
          }

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
    .or_else([this](const util::ErrorChain& error_chain) {
      if (_errors_to_ignore & util::IgnoreErrors::InaccessibleFiles) {
        util::write_log(
          (boost::format("Opening next point file failed: %1%") % error_chain.what()).str());
        return;
      }

      throw util::chain_error(error_chain, "Opening next point file failed");
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