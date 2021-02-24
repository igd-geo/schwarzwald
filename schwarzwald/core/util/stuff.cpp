#include "util/stuff.h"

#include <fstream>
#include <iostream>
#include <map>
#include <math.h>
#include <string>
#include <vector>

//#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>

#include "math/AABB.h"
#include "math/Vector3.h"
#include "pointcloud/Point.h"
#include "util/Error.h"

#include <rapidjson/error/en.h>
#include <rapidjson/filewritestream.h>
#include <rapidjson/writer.h>

/**
 *   y
 *   |-z
 *   |/
 *   O----x
 *
 *   3----7
 *  /|   /|
 * 2----6 |
 * | 1--|-5
 * |/   |/
 * 0----4
 *
 */
AABB
childAABB(const AABB& aabb, const int& index)
{
  Vector3<double> min = aabb.min;
  Vector3<double> max = aabb.max;

  const auto bounds_extent = aabb.extent();

  if ((index & 0b0001) > 0) {
    min.z += bounds_extent.z / 2;
  } else {
    max.z -= bounds_extent.z / 2;
  }

  if ((index & 0b0010) > 0) {
    min.y += bounds_extent.y / 2;
  } else {
    max.y -= bounds_extent.y / 2;
  }

  if ((index & 0b0100) > 0) {
    min.x += bounds_extent.x / 2;
  } else {
    max.x -= bounds_extent.x / 2;
  }

  return AABB(min, max);
}

/**
 *   y
 *   |-z
 *   |/
 *   O----x
 *
 *   3----7
 *  /|   /|
 * 2----6 |
 * | 1--|-5
 * |/   |/
 * 0----4
 *
 */
int
nodeIndex(const AABB& aabb, const Vector3<double>& pointPosition)
{
  const auto bounds_extent = aabb.extent();
  int mx = (int)(2.0 * (pointPosition.x - aabb.min.x) / bounds_extent.x);
  int my = (int)(2.0 * (pointPosition.y - aabb.min.y) / bounds_extent.y);
  int mz = (int)(2.0 * (pointPosition.z - aabb.min.z) / bounds_extent.z);

  mx = std::min(mx, 1);
  my = std::min(my, 1);
  mz = std::min(mz, 1);

  return (mx << 2) | (my << 1) | mz;
}

/**
 * from
 * http://stackoverflow.com/questions/5840148/how-can-i-get-a-files-size-in-c
 */
long
filesize(std::string filename)
{
  struct stat stat_buf;
  int rc = stat(filename.c_str(), &stat_buf);
  return rc == 0 ? stat_buf.st_size : -1;
}

///**
// * from
// http://stackoverflow.com/questions/874134/find-if-string-endswith-another-string-in-c
// */
// bool endsWith (std::string const &fullString, std::string const &ending)
//{
//    if (fullString.length() >= ending.length()) {
//        return (0 == fullString.compare (fullString.length() -
//        ending.length(), ending.length(), ending));
//    } else {
//        return false;
//    }
//}

/**
 * see
 * http://stackoverflow.com/questions/735204/convert-a-string-in-c-to-upper-case
 */
std::string
toUpper(std::string str)
{
  auto tmp = str;
  std::transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);

  return tmp;
}

// http://stackoverflow.com/questions/8593608/how-can-i-copy-a-directory-using-boost-filesystem
bool
copyDir(fs::path source, fs::path destination)
{
  try {
    // Check whether the function call is valid
    if (!fs::exists(source) || !fs::is_directory(source)) {
      std::cerr << "Source directory " << source.string()
                << " does not exist or is not a directory." << '\n';
      return false;
    }
    // if(fs::exists(destination)){
    //    std::cerr << "Destination directory " << destination.string()
    //        << " already exists." << '\n';
    //    return false;
    //}
    // Create the destination directory
    if (!fs::exists(destination)) {
      if (!fs::create_directory(destination)) {
        std::cerr << "Unable to create destination directory"
                  << destination.string() << '\n';
        return false;
      }
    }
  } catch (fs::filesystem_error const& e) {
    std::cerr << e.what() << '\n';
    return false;
  }
  // Iterate through the source directory
  for (fs::directory_iterator file(source); file != fs::directory_iterator();
       ++file) {
    try {
      fs::path current(file->path());
      if (fs::is_directory(current)) {
        // Found directory: Recursion
        if (!copyDir(current, destination / current.filename())) {
          return false;
        }
      } else {
        // Found file: Copy
        fs::copy_file(current,
                      destination / current.filename(),
                      fs::copy_options::overwrite_existing);
      }
    } catch (fs::filesystem_error const& e) {
      std::cerr << e.what() << '\n';
    }
  }
  return true;
}

float
psign(float value)
{
  if (value == 0.0) {
    return 0.0;
  } else if (value < 0.0) {
    return -1.0;
  } else {
    return 1.0;
  }
}

// see
// https://stackoverflow.com/questions/23943728/case-insensitive-standard-string-comparison-in-c
bool
icompare_pred(unsigned char a, unsigned char b)
{
  return std::tolower(a) == std::tolower(b);
}

// see
// https://stackoverflow.com/questions/23943728/case-insensitive-standard-string-comparison-in-c
bool
icompare(std::string const& a, std::string const& b)
{
  if (a.length() == b.length()) {
    return std::equal(b.begin(), b.end(), a.begin(), icompare_pred);
  } else {
    return false;
  }
}

// bool endsWith(const std::string &str, const std::string &suffix) {
//	return str.size() >= suffix.size() && str.compare(str.size() -
// suffix.size(), suffix.size(), suffix) == 0;
//}

bool
endsWith(const std::string& str, const std::string& suffix)
{
  if (str.size() < suffix.size()) {
    return false;
  }

  auto tstr = str.substr(str.size() - suffix.size());

  return tstr.compare(suffix) == 0;
}

bool
iEndsWith(const std::string& str, const std::string& suffix)
{
  if (str.size() < suffix.size()) {
    return false;
  }

  auto tstr = str.substr(str.size() - suffix.size());

  return icompare(tstr, suffix);
}

// see
// https://stackoverflow.com/questions/216823/whats-the-best-way-to-trim-stdstring
std::string
ltrim(std::string s)
{
  s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](unsigned char ch) {
            return !std::isspace(ch);
          }));

  return s;
}

// see
// https://stackoverflow.com/questions/216823/whats-the-best-way-to-trim-stdstring
std::string
rtrim(std::string s)
{
  s.erase(std::find_if(s.rbegin(),
                       s.rend(),
                       [](unsigned char ch) { return !std::isspace(ch); })
            .base(),
          s.end());

  return s;
}

// see
// https://stackoverflow.com/questions/216823/whats-the-best-way-to-trim-stdstring
std::string
trim(std::string s)
{
  s = ltrim(s);
  s = rtrim(s);

  return s;
}

Vector3<uint8_t>
intensityToRGB_Log(uint16_t intensity)
{
  const auto correctedIntensity = std::log(intensity + 1) / std::log(0xffff);
  const auto grey = static_cast<uint8_t>(255 * correctedIntensity);
  return { grey, grey, grey };
}

std::vector<fs::path>
get_all_files_in_directory(const std::string& dir_path, Recursive recursive)
{
  std::vector<fs::path> files;
  if (recursive == Recursive::Yes) {
    for (auto& f : fs::recursive_directory_iterator{ dir_path }) {
      if (!fs::is_regular_file(f))
        continue;
      files.push_back(f);
    }
  } else {
    for (auto& f : fs::directory_iterator{ dir_path }) {
      if (!fs::is_regular_file(f))
        continue;
      files.push_back(f);
    }
  }
  return files;
}

void
write_json_to_file(const rapidjson::Document& doc, const fs::path& file_path)
{
  struct Stream
  {
    std::ofstream of;

    explicit Stream(const std::string& filepath)
      : of{ filepath, std::ios::binary }
    {}

    typedef char Ch;
    void Put(Ch ch) { of.put(ch); }
    void Flush() {}
  };

  Stream fs{ file_path };
  if (!fs.of.is_open()) {
    throw util::chain_error({ concat("Can't open file ", file_path.string()) });
  }

  rapidjson::Writer<Stream> writer(fs);
  if (!doc.Accept(writer)) {
    throw util::chain_error(
      { concat("Unknown error while writing JSON document to file ",
               file_path.string()) });
  }
}

uint32_t
get_prev_power_of_two(uint32_t x)
{
  x = x | (x >> 1);
  x = x | (x >> 2);
  x = x | (x >> 4);
  x = x | (x >> 8);
  x = x | (x >> 16);
  return x - (x >> 1);
}

uint64_t
get_prev_power_of_two(uint64_t x)
{
  x = x | (x >> 1);
  x = x | (x >> 2);
  x = x | (x >> 4);
  x = x | (x >> 8);
  x = x | (x >> 16);
  x = x | (x >> 32);
  return x - (x >> 1);
}