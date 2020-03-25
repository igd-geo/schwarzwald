#pragma once

#include "pointcloud/Tileset.h"

/// <summary>
/// Writes the given Tileset as JSON to the given filepath
/// </summary>
bool
writeTilesetJSON(const std::string& filepath, const Tileset& tileset, uint32_t max_depth = std::numeric_limits<uint32_t>::max());
