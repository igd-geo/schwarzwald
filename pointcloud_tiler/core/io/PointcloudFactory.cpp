#include "PointcloudFactory.h"

#include <boost/format.hpp>

tl::expected<PointFile, std::error_code>
open_point_file(const std::experimental::filesystem::path& path)
{
  if (path.extension() == ".las" || path.extension() == ".laz") {
    try {
      return { PointFile{ LASFile{ path, LASFile::OpenMode::Read } } };
    } catch (const std::exception& ex) {
      return tl::make_unexpected(std::make_error_code(std::errc::io_error));
    }
  }
  return tl::make_unexpected(std::make_error_code(std::errc::not_supported));
}