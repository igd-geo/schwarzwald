#pragma once

#include "OctreeNodeKey.h"

#include <fstream>
#include <gsl/gsl>
#include <iostream>
#include <string>

struct OctreeIndexFileHeader
{
  const char magic[4] = { 'i', 'n', 'd', 'x' };
  uint32_t levels_per_index;
  size_t num_indices;
};

template<unsigned int MaxLevels>
void
write_octree_indices_to_file(const std::string& file_path,
                             gsl::span<OctreeNodeKey<MaxLevels>> indices)
{
  OctreeIndexFileHeader header;
  header.levels_per_index = MaxLevels;
  header.num_indices = indices.size();

  std::ofstream fs{ file_path, std::ios::out | std::ios::binary };
  if (!fs.is_open()) {
    std::cerr << "Could not write index file " << file_path << std::endl;
    return;
  }

  fs.write(reinterpret_cast<char const*>(&header), sizeof(OctreeIndexFileHeader));
  for (auto idx : indices) {
    const auto idx_val = idx.get();
    fs.write(reinterpret_cast<char const*>(&idx_val), sizeof(idx_val));
  }

  fs.flush();
  fs.close();
}

template<unsigned int MaxLevels>
std::vector<OctreeNodeKey<MaxLevels>>
read_octree_indices_from_file(const std::string& file_path)
{
  std::ifstream fs{ file_path, std::ios::in | std::ios::binary };
  if (!fs.is_open()) {
    std::cerr << "Could not read index file " << file_path << std::endl;
    return {};
  }

  fs.unsetf(std::ios::skipws);

  fs.seekg(0, std::ios::end);
  auto fileSize = fs.tellg();
  fs.seekg(0, std::ios::beg);

  std::vector<char> rawData;
  rawData.reserve(fileSize);

  rawData.insert(rawData.begin(), std::istream_iterator<char>(fs), std::istream_iterator<char>());

  const auto& header = *reinterpret_cast<OctreeIndexFileHeader const*>(rawData.data());

  if (header.levels_per_index != MaxLevels) {
    std::cerr << "Reading octree index file with indices that contain " << header.levels_per_index
              << " levels but requested to read into OctreeNodeKeys with " << MaxLevels
              << " levels instead!" << std::endl;
    return {};
  }

  auto indices_begin = rawData.data() + sizeof(OctreeIndexFileHeader);
  const auto indices_end = rawData.data() + rawData.size();

  using IndexType_t = typename OctreeNodeKey<MaxLevels>::Store_t;
  const auto index_binary_size = sizeof(IndexType_t);
  std::vector<OctreeNodeKey<MaxLevels>> indices;
  indices.reserve(header.num_indices);
  for (; indices_begin < indices_end; indices_begin += index_binary_size) {
    indices.emplace_back(*reinterpret_cast<IndexType_t const*>(indices_begin));
  }

  return indices;
}