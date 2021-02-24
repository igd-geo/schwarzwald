#include "PointSource.h"

#include "io/PointcloudFile.h"
#include "terminal/stdout_helper.h"

#include <boost/format.hpp>

PointSource::PointSource(std::vector<fs::path> files,
                         util::IgnoreErrors errors_to_ignore)
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
        [this, count, &attributes, &typed_file, &metadata](
          auto& typed_file_cursor) -> PointBuffer {
          PointBuffer point_buffer;
          try {
            typed_file_cursor = pc::read_points(
              typed_file_cursor, count, metadata, attributes, point_buffer);
          } catch (const std::exception& ex) {
            if (_errors_to_ignore & util::IgnoreErrors::CorruptedFiles) {
              // Drop this file, move on to next file
              util::write_log((boost::format("Could not read points from "
                                             "file %1%\n\tcaused by: %2%\n") %
                               _file_cursor->string() % ex.what())
                                .str());
              typed_file_cursor = std::cend(typed_file);
            } else {
              throw util::chain_error(
                ex,
                (boost::format("Could not read points from file %1%") %
                 _file_cursor->string())
                  .str());
            }
          }

          // If at end of current file, move to next file
          if (typed_file_cursor == std::cend(typed_file)) {
            move_to_next_file();
          }

          return point_buffer;
        },
        *_current_file_cursor);

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
        util::write_log((boost::format("Opening next point file failed: %1%") %
                         error_chain.what())
                          .str());
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

#pragma region MultiReaderPointSource
MultiReaderPointSource::MultiReaderPointSource(
  std::vector<fs::path> files,
  util::IgnoreErrors errors_to_ignore)
  : _files(std::move(files))
  , _next_files(std::begin(_files), std::end(_files))
  , _errors_to_ignore(errors_to_ignore)
  , _open_files_lock(std::make_unique<std::mutex>())
{
  std::cout << "Input files:\n";
  for (auto& file : _files) {
    std::cout << file << "\n";
  }
}

std::optional<MultiReaderPointSource::PointSourceHandle>
MultiReaderPointSource::lock_source()
{
  std::lock_guard guard{ *_open_files_lock };

  const auto next_available_file = find_next_available_open_file();
  if (next_available_file) {
    next_available_file->available = false;
    return std::make_optional(PointSourceHandle{ next_available_file, this });
  }

  auto next_file = try_open_next_file();
  if (!next_file) {
    return std::nullopt;
  }

  // Lock new file
  next_file->available = false;
  PointSourceHandle handle{ next_file.get(), this };

  _open_files.push_back(std::move(next_file));

  return handle;
}

std::optional<MultiReaderPointSource::PointSourceHandle>
MultiReaderPointSource::lock_specific_source(const fs::path& file_name)
{
  std::lock_guard guard{ *_open_files_lock };

  // Check first if file_name is in _open_files
  const auto open_file_record_or_failure = get_specific_open_file(file_name);

  if (open_file_record_or_failure.has_value()) {
    auto file_record = open_file_record_or_failure.value();
    file_record->available = false;
    return std::make_optional(PointSourceHandle{ file_record, this });
  }

  const auto reason_for_failure = open_file_record_or_failure.error();
  switch (reason_for_failure) {
    case GetSpecificOpenFileFailure::NotAvailable:
      throw std::runtime_error{
        "Requested file source is already locked by another thread!"
      };
    case GetSpecificOpenFileFailure::NotYetOpened: {
      auto opened_file = try_open_specific_file(file_name);
      if (!opened_file) {
        return std::nullopt;
      }

      opened_file->available = false;
      PointSourceHandle handle{ opened_file.get(), this };

      _open_files.push_back(std::move(opened_file));

      return handle;
    }
    default:
      throw std::runtime_error{
        "Unrecognized GetSpecificOpenFileFailure constant"
      };
  }
}

void
MultiReaderPointSource::release_source(const PointSourceHandle& source_handle)
{
  std::lock_guard guard{ *_open_files_lock };

  // Find matching PointFileEntry
  const auto matching_entry_iter =
    std::find_if(std::begin(_open_files),
                 std::end(_open_files),
                 [&source_handle](const auto& entry) {
                   return entry.get() == source_handle._point_file_entry;
                 });
  if (matching_entry_iter == std::end(_open_files)) {
    throw std::invalid_argument{
      "source_handle does not refer to an open file!"
    };
  }

  auto& matching_entry = **matching_entry_iter;
  // If the file is at the end, remove it from _open_files
  if (point_file_entry_is_at_end(matching_entry)) {
    _open_files.erase(matching_entry_iter);
    return;
  }

  // Otherwise we mark the file as available
  matching_entry.available = true;
}

size_t
MultiReaderPointSource::max_concurrent_reads()
{
  std::lock_guard guard{ *_open_files_lock };
  const auto open_files =
    std::count_if(std::begin(_open_files),
                  std::end(_open_files),
                  [](const auto& open_file) { return open_file->available; });
  const auto remaining_files = _next_files.size();
  return open_files + remaining_files;
}

void
MultiReaderPointSource::add_transformation(Transform transform)
{
  _transformations.push_back(std::move(transform));
}

std::unique_ptr<MultiReaderPointSource::PointFileEntry>
MultiReaderPointSource::try_open_next_file()
{
  const auto next_file_iter = std::begin(_next_files);
  if (next_file_iter == std::end(_next_files))
    return nullptr;

  return do_open_file(next_file_iter);
}

std::unique_ptr<MultiReaderPointSource::PointFileEntry>
MultiReaderPointSource::try_open_specific_file(const fs::path& file)
{
  const auto file_iter = _next_files.find(file);
  if (file_iter == std::end(_next_files)) {
    return nullptr;
  }

  return do_open_file(file_iter);
}

std::unique_ptr<MultiReaderPointSource::PointFileEntry>
MultiReaderPointSource::do_open_file(
  std::unordered_set<fs::path, util::PathHash>::iterator file_iter)
{
  const auto file_path = *file_iter;
  _next_files.erase(file_iter);

  const auto file_index = static_cast<size_t>(
    std::distance(std::begin(_files),
                  std::find(std::begin(_files), std::end(_files), file_path)));

  return open_point_file(file_path)
    .map([this, &file_path, file_index](PointFile point_file) mutable {
      return std::make_unique<PointFileEntry>(
        std::move(point_file), file_path, file_index);
    })
    .or_else([this](const util::ErrorChain& error_chain) {
      if (_errors_to_ignore & util::IgnoreErrors::InaccessibleFiles) {
        util::write_log((boost::format("Opening next point file failed: %1%") %
                         error_chain.what())
                          .str());
        return;
      }

      throw util::chain_error(error_chain, "Opening next point file failed");
    })
    .value_or(nullptr);
}

MultiReaderPointSource::PointFileEntry*
MultiReaderPointSource::find_next_available_open_file()
{
  const auto next_available_file_iter = std::find_if(
    std::begin(_open_files),
    std::end(_open_files),
    [](const auto& point_file_entry) { return point_file_entry->available; });
  if (next_available_file_iter == std::end(_open_files)) {
    return nullptr;
  }

  return next_available_file_iter->get();
}

tl::expected<MultiReaderPointSource::PointFileEntry*,
             MultiReaderPointSource::GetSpecificOpenFileFailure>
MultiReaderPointSource::get_specific_open_file(const fs::path& file) const
{
  const auto is_open_iter =
    std::find_if(std::begin(_open_files),
                 std::end(_open_files),
                 [&file](const auto& point_file_entry) {
                   return point_file_entry->file_path == file;
                 });

  if (is_open_iter == std::end(_open_files)) {
    return tl::make_unexpected(GetSpecificOpenFileFailure::NotYetOpened);
  }

  if (!(*is_open_iter)->available) {
    return tl::make_unexpected(GetSpecificOpenFileFailure::NotAvailable);
  }

  return { is_open_iter->get() };
}

bool
MultiReaderPointSource::point_file_entry_is_at_end(
  const PointFileEntry& point_file_entry) const
{
  return std::visit(
    [&point_file_entry](const auto& typed_file) {
      return std::visit(
        [&typed_file](const auto& cursor) {
          return cursor == std::cend(typed_file);
        },
        point_file_entry.cursor);
    },
    point_file_entry.point_file);
}

std::optional<PointBuffer>
MultiReaderPointSource::PointSourceHandle::read_next(
  size_t count,
  const PointAttributes& point_attributes)
{
  // The std::visit syntax makes me want to cry
  return std::visit(
    [count, &point_attributes, this](auto& typed_cursor) {
      return std::visit(
        [count, &point_attributes, &typed_cursor, this](
          auto& typed_file) -> std::optional<PointBuffer> {
          PointBuffer point_buffer;
          try {
            typed_cursor = pc::read_points(typed_cursor,
                                           count,
                                           pc::metadata(typed_file),
                                           point_attributes,
                                           point_buffer);
          } catch (const std::exception& ex) {
            if (_multi_reader_source->_errors_to_ignore &
                util::IgnoreErrors::CorruptedFiles) {
              // Log error and move this file to end, we assume that the file is
              // dead now
              util::write_log((boost::format("Could not read points from "
                                             "file %1%\n\tcaused by: %2%\n") %
                               pc::source(typed_file) % ex.what())
                                .str());
              typed_cursor = std::cend(typed_file);
              return std::nullopt;
            } else {
              throw util::chain_error(
                ex,
                (boost::format("Could not read points from file %1%") %
                 pc::source(typed_file))
                  .str());
            }
          }

          for (auto& transformation : _multi_reader_source->_transformations) {
            transformation(
              { std::begin(point_buffer), std::end(point_buffer) });
          }

          return { std::move(point_buffer) };
        },
        _point_file_entry->point_file);
    },
    _point_file_entry->cursor);
}

PointBuffer::PointIterator
MultiReaderPointSource::PointSourceHandle::read_next_into(
  util::Range<PointBuffer::PointIterator> point_range,
  const PointAttributes& point_attributes)
{
  return std::visit(
    [point_range, &point_attributes, this](auto& typed_cursor) {
      return std::visit(
        [point_range, &point_attributes, &typed_cursor, this](
          auto& typed_file) -> PointBuffer::PointIterator {
          PointBuffer::PointIterator new_end_of_point_range =
            std::begin(point_range);
          try {
            auto [_new_file_iter, _new_out_iter] =
              pc::read_points_into(typed_cursor,
                                   std::cend(typed_file),
                                   pc::metadata(typed_file),
                                   point_attributes,
                                   point_range);

            for (auto point_ref : point_range) {
              *point_ref.point_source_id() =
                static_cast<uint16_t>(_point_file_entry->file_index);
            }

            typed_cursor = _new_file_iter;
            new_end_of_point_range = _new_out_iter;
          } catch (const std::exception& ex) {
            if (_multi_reader_source->_errors_to_ignore &
                util::IgnoreErrors::CorruptedFiles) {
              // Log error and move this file to end, we assume that the file is
              // dead now
              util::write_log((boost::format("Could not read points from "
                                             "file %1%\n\tcaused by: %2%\n") %
                               pc::source(typed_file) % ex.what())
                                .str());
              typed_cursor = std::cend(typed_file);
              return std::begin(point_range);
            } else {
              throw util::chain_error(
                ex,
                (boost::format("Could not read points from file %1%") %
                 pc::source(typed_file))
                  .str());
            }
          }

          for (auto& transformation : _multi_reader_source->_transformations) {
            transformation({ std::begin(point_range), new_end_of_point_range });
          }

          return new_end_of_point_range;
        },
        _point_file_entry->point_file);
    },
    _point_file_entry->cursor);
}

MultiReaderPointSource::PointSourceHandle::PointSourceHandle(
  PointFileEntry* point_file_entry,
  MultiReaderPointSource* multi_reader_source)
  : _point_file_entry(point_file_entry)
  , _multi_reader_source(multi_reader_source)
{}

MultiReaderPointSource::PointFileEntry::PointFileEntry(
  PointFile file,
  const fs::path& file_path,
  size_t file_index)
  : point_file(std::move(file))
  , available(true)
  , file_path(file_path)
  , file_index(file_index)
{
  // Get an iterator to the start of the file and store it
  //( std::variant syntax is so nasty :( )
  std::visit(
    [this](auto& typed_file_cursor) {
      std::visit([&typed_file_cursor](
                   const auto& file) { typed_file_cursor = std::begin(file); },
                 point_file);
    },
    cursor);
}

#pragma endregion