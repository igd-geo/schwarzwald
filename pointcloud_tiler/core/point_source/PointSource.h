#pragma once

#include <experimental/filesystem>
#include <functional>
#include <mutex>
#include <optional>
#include <vector>

#include "datastructures/PointBuffer.h"
#include "io/PointcloudFactory.h"
#include "pointcloud/PointAttributes.h"
#include "util/Error.h"

namespace fs = std::experimental::filesystem;

struct PointSource
{
  using Transform = std::function<void(PointBuffer&)>;

  PointSource(std::vector<fs::path> files, util::IgnoreErrors errors_to_ignore);

  std::optional<PointBuffer> read_next(size_t count, const PointAttributes& point_attributes);

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

/**
 * A point cloud file source that allows multiple (concurrent) readers at once.
 * Concurrent reading is implemented on a file-basis, so the maximum number of
 * concurrent readers will be equal to the number of files that the
 * MultiReaderPointSource manages
 */
struct MultiReaderPointSource
{
  /**
   * API description:
   *
   * Readers have to request a unique source for reading:
   *   auto source = async_source.lock_source();
   *   if(!source) return;
   *   auto points = source.read_next(count, attributes);
   *
   * Once a reader is done, it has to release the source:
   *   async_source.release(source);
   */

  using Transform = std::function<void(util::Range<PointBuffer::PointIterator>)>;

  // TODO HACK Storing the file types as variants and then storing a SEPARATE
  // variant for the iterators won't compile with more than one type in the
  // variants, as we have to do a double-visit on both variants I don't how to
  // change this code at the moment, maybe we can get rid of the variants by
  // using some regular polymorphism?
  using PointFileCursor = std::variant<typename LASFile::const_iterator>;

  struct PointFileEntry
  {
    explicit PointFileEntry(PointFile point_file);

    PointFile point_file;
    PointFileCursor cursor;
    bool available;
  };

  /**
   * Handle for an open PointSource that is currently in use by some reader
   */
  struct PointSourceHandle
  {
    friend struct MultiReaderPointSource;

    std::optional<PointBuffer> read_next(size_t count, const PointAttributes& point_attributes);

    PointBuffer::PointIterator read_next_into(util::Range<PointBuffer::PointIterator> point_range,
                                              const PointAttributes& point_attributes);

  private:
    PointSourceHandle(PointFileEntry* point_file_entry,
                      MultiReaderPointSource* multi_reader_source);

    PointFileEntry* _point_file_entry;
    MultiReaderPointSource* _multi_reader_source;
  };

  friend struct PointSourceHandle;

  MultiReaderPointSource(std::vector<fs::path> files, util::IgnoreErrors errors_to_ignore);
  MultiReaderPointSource(MultiReaderPointSource&&) = default;

  std::optional<PointSourceHandle> lock_source();
  void release_source(const PointSourceHandle& source_handle);

  size_t max_concurrent_reads();

  void add_transformation(Transform transform);

private:
  std::unique_ptr<PointFileEntry> try_open_next_file();
  PointFileEntry* find_next_available_open_file();
  bool point_file_entry_is_at_end(const PointFileEntry& point_file_entry) const;

  std::vector<fs::path> _files;
  util::IgnoreErrors _errors_to_ignore;

  std::vector<fs::path>::const_iterator _next_file;

  std::vector<std::unique_ptr<PointFileEntry>> _open_files;
  std::unique_ptr<std::mutex> _open_files_lock;

  std::vector<Transform> _transformations;
};