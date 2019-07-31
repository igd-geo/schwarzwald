#pragma once

#include <string>

/**
 * Concatenates two paths (e.g. '/some/path' and 'file.jpg' to '/some/path/file.jpg')
 */
std::string
concat_path(const std::string& l, const std::string& r);