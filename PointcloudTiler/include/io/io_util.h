#pragma once

#include <fstream>
#include <string>

/**
 * Concatenates two paths (e.g. '/some/path' and 'file.jpg' to
 * '/some/path/file.jpg')
 */
std::string concat_path(const std::string &l, const std::string &r);

/**
 * Write binary data to the given file stream
 */
template <typename T, typename Stream>
void write_binary(const T &val, Stream &stream) {
  stream.write(reinterpret_cast<const char *>(&val), sizeof(T));
}

template <typename T, typename Stream>
void read_binary(T &val, Stream &stream) {
  stream.read(reinterpret_cast<char *>(&val), sizeof(T));
}