#pragma once

#include <algorithm>
#include <string>
#include <vector>

class PointAttribute
{
public:
  const int ordinal;
  const char* name;
  const int numElements;
  const int byteSize;

  constexpr PointAttribute(int ordinal, const char* name, int numElements, int byteSize)
    : ordinal(ordinal)
    , name(name)
    , numElements(numElements)
    , byteSize(byteSize)
  {}

  constexpr PointAttribute(const PointAttribute& other)
    : ordinal(other.ordinal)
    , name(other.name)
    , numElements(other.numElements)
    , byteSize(other.byteSize)
  {}

  static PointAttribute fromString(const std::string& name);
  static PointAttribute fromStringLiteral(const char* name);

  constexpr operator int() const { return ordinal; }
};

bool
operator==(const PointAttribute& lhs, const PointAttribute& rhs);

namespace attributes {
constexpr PointAttribute POSITION_CARTESIAN = { 0, "POSITION_CARTESIAN", 3, 12 };
constexpr PointAttribute COLOR_PACKED = { 1, "COLOR_PACKED", 4, 4 };
constexpr PointAttribute INTENSITY = { 2, "INTENSITY", 1, 2 };
constexpr PointAttribute CLASSIFICATION = { 3, "CLASSIFICATION", 1, 1 };
constexpr PointAttribute NORMAL_SPHEREMAPPED = { 4, "NORMAL_SPHEREMAPPED", 2, 2 };
constexpr PointAttribute NORMAL_OCT16 = { 5, "NORMAL_OCT16", 2, 2 };
constexpr PointAttribute NORMAL = { 6, "NORMAL", 3, 12 };
constexpr PointAttribute COLOR_FROM_INTENSITY = { 7, "COLOR_FROM_INTENSITY", 3, 3 };
} // namespace attributes

using PointAttributes = std::vector<PointAttribute>;

bool
has_attribute(PointAttributes const& attributes, PointAttribute const& attribute_to_find);
std::string
print_attributes(PointAttributes const& attributes);

PointAttributes
point_attributes_from_strings(const std::vector<std::string>& attribute_names);