#include "datastructures/DynamicMortonIndex.h"
#include "util/stuff.h"

#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>

#include <sstream>

namespace {
tl::expected<DynamicMortonIndex, std::string>
parse_string_octant_based(const std::string& str, bool has_root)
{
  auto start_iter = str.begin();
  if (has_root)
    ++start_iter;

  const auto depth = std::distance(start_iter, str.end());
  if (depth < 0) {
    const auto error_msg = boost::format("Can't parse Morton index in Potree format from string "
                                         "\"%1%\" because first character is not 'r'!") %
                           str;
    return tl::make_unexpected(error_msg.str());
  }

  std::vector<DynamicMortonIndex::Octant_t> octants;
  octants.reserve(static_cast<size_t>(depth));

  for (; start_iter != str.end(); ++start_iter) {
    const auto octant_char = *start_iter;
    const auto octant_index = (octant_char - '0');
    if (octant_index < 0 || octant_index > 7) {
      const auto error_msg = boost::format("Can't parse Morton index from string \"%1%\" because "
                                           "of unexpected character '%2%'!") %
                             str % octant_char;
      return tl::make_unexpected(error_msg.str());
    }
    octants.push_back(static_cast<DynamicMortonIndex::Octant_t>(octant_index));
  }
  return DynamicMortonIndex{ std::move(octants) };
}

tl::expected<DynamicMortonIndex, std::string>
parse_string_grid_based(const std::string& str)
{
  // String is of the form DEPTH-X-Y-Z
  std::vector<std::string> tokens;
  boost::split(tokens, str, boost::is_any_of("-"));
  if (tokens.size() != 4) {
    const auto error_msg = boost::format("Can't parse Morton index in Entwine format from string "
                                         "\"%1%\" because it is malformed!") %
                           str;
    return tl::make_unexpected(error_msg.str());
  }

  uint32_t depth;
  uint64_t x, y, z;
  try {
    depth = std::stoul(tokens[0]);
    x = std::stoull(tokens[1]);
    y = std::stoull(tokens[2]);
    z = std::stoull(tokens[3]);
  } catch (const std::exception& parse_error) {
    const auto error_msg = boost::format("Can't parse Morton index in Entwine "
                                         "format from string \"%1%\": %2%") %
                           str % parse_error.what();
    return tl::make_unexpected(error_msg.str());
  }

  std::vector<DynamicMortonIndex::Octant_t> octants;
  octants.reserve(depth);

  for (uint32_t level = 0; level < depth; ++level) {
    const auto shift = (depth - level - 1);
    const auto x_bit = (x >> shift) & 1;
    const auto y_bit = (y >> shift) & 1;
    const auto z_bit = (z >> shift) & 1;

    const auto octant = static_cast<uint8_t>(z_bit | (y_bit << 1) | (x_bit << 2));
    octants.push_back(octant);
  }
  return DynamicMortonIndex{ std::move(octants) };
}

std::string
print_morton_index_octant_based(const DynamicMortonIndex& morton_index, bool has_root)
{
  std::stringstream ss;
  if (has_root) {
    ss << 'r';
  }
  for (auto octant : morton_index) {
    ss << static_cast<char>(octant + '0');
  }
  return ss.str();
}

std::string
print_morton_index_grid_based(const DynamicMortonIndex& morton_index)
{
  const auto depth = morton_index.depth();
  uint64_t x = 0, y = 0, z = 0;
  for (auto octant : morton_index) {
    x <<= 1;
    y <<= 1;
    z <<= 1;

    x |= ((octant >> 2) & 1);
    y |= ((octant >> 1) & 1);
    z |= (octant & 1);
  }

  std::stringstream ss;
  ss << depth << '-' << x << '-' << y << '-' << z;
  return ss.str();
}
} // namespace

DynamicMortonIndex::DynamicMortonIndex() {}

DynamicMortonIndex::DynamicMortonIndex(std::vector<Octant_t> octants)
  : _octants(std::move(octants))
{}

size_t
DynamicMortonIndex::depth() const
{
  return _octants.size();
}

DynamicMortonIndex::Octant_t DynamicMortonIndex::operator[](size_t level) const
{
  return _octants[level];
}
DynamicMortonIndex::Octant_t& DynamicMortonIndex::operator[](size_t level)
{
  return _octants[level];
}

bool
DynamicMortonIndex::operator==(const DynamicMortonIndex& other) const
{
  return _octants == other._octants;
}

std::vector<DynamicMortonIndex::Octant_t>::const_iterator
DynamicMortonIndex::begin() const
{
  return _octants.begin();
}

std::vector<DynamicMortonIndex::Octant_t>::const_iterator
DynamicMortonIndex::end() const
{
  return _octants.end();
}

DynamicMortonIndex
DynamicMortonIndex::truncate_to_depth(size_t new_depth) const
{
  if (new_depth > depth())
    throw std::invalid_argument{ "new_depth must not exceed current depth!" };

  return DynamicMortonIndex{ std::vector<Octant_t>{ std::begin(_octants),
                                                    std::begin(_octants) + new_depth } };
}

DynamicMortonIndex
DynamicMortonIndex::child(Octant_t octant) const
{
  assert(octant < 8);
  auto octants = _octants;
  octants.push_back(octant);
  return DynamicMortonIndex{ std::move(octants) };
}

DynamicMortonIndex
DynamicMortonIndex::parent() const
{
  assert(depth() > 0);
  auto parent_octants = _octants;
  parent_octants.pop_back();
  return DynamicMortonIndex{ std::move(parent_octants) };
}

tl::expected<DynamicMortonIndex, std::string>
DynamicMortonIndex::parse_string(const std::string& str,
                                 MortonIndexNamingConvention naming_convention)
{
  switch (naming_convention) {
    case MortonIndexNamingConvention::Simple:
      return parse_string_octant_based(str, false);
    case MortonIndexNamingConvention::Potree:
      return parse_string_octant_based(str, true);
    case MortonIndexNamingConvention::Entwine:
      return parse_string_grid_based(str);
    default:
      return tl::make_unexpected(std::string{ "Unexpected naming_convention " } +
                                 std::to_string(static_cast<uint32_t>(naming_convention)));
  }
}

std::string
to_string(const DynamicMortonIndex& morton_index, MortonIndexNamingConvention naming_convention)
{
  switch (naming_convention) {
    case MortonIndexNamingConvention::Simple:
      return print_morton_index_octant_based(morton_index, false);
    case MortonIndexNamingConvention::Potree:
      return print_morton_index_octant_based(morton_index, true);
    case MortonIndexNamingConvention::Entwine:
      return print_morton_index_grid_based(morton_index);
    default:
      throw std::logic_error{ std::string{ "Unexpected naming_convention " } +
                              std::to_string(static_cast<uint32_t>(naming_convention)) };
  }
}