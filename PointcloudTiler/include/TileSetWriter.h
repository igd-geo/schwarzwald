#pragma once

#include "Tileset.h"

/// <summary>
/// Writes the given Tileset as JSON to the given filepath
/// </summary>
bool
writeTilesetJSON(const std::string& filepath, const Tileset& tileset);
