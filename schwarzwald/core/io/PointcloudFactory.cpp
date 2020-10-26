#include "PointcloudFactory.h"

#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>

tl::expected<PointFile, util::ErrorChain>
open_point_file(const std::experimental::filesystem::path &path) {
  const auto extension_as_lower =
      boost::algorithm::to_lower_copy(path.extension().string());

  if (extension_as_lower == ".las" || extension_as_lower == ".laz") {
    try {
      return {PointFile{LASFile{path, LASFile::OpenMode::Read}}};
    } catch (const std::exception &ex) {
      const auto reason =
          (boost::format("Could not open file %1%") % path.string()).str();
      return tl::make_unexpected(util::chain_error(ex, reason));
    }
  }
  auto reason =
      (boost::format("Could not open file %1%, file format %2% not supported") %
       path.string() % path.extension())
          .str();
  return tl::make_unexpected(util::chain_error(std::move(reason)));
}

bool file_format_is_supported(const std::string &format) {
  constexpr static std::array<const char *, 2> SUPPORTED_FORMATS = {".las",
                                                                    ".laz"};

  const auto format_as_lower = boost::algorithm::to_lower_copy(format);

  return std::find_if(std::begin(SUPPORTED_FORMATS),
                      std::end(SUPPORTED_FORMATS),
                      [&format_as_lower](auto supported_format) {
                        return format_as_lower == supported_format;
                      }) != std::end(SUPPORTED_FORMATS);
}