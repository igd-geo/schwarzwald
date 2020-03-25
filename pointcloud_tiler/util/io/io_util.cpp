#include "io/io_util.h"

#include <sstream>

std::string
concat_path(const std::string& l, const std::string& r)
{
  std::stringstream ss;
  ss << l << "/" << r;
  return ss.str();
}