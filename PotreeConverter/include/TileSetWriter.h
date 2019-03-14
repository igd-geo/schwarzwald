#ifndef TILESETWRITER_H
#define TILESETWRITER_H

#include "Tileset.h"

namespace Potree {

/// <summary>
/// Writes the given Tileset as JSON to the given filepath
/// </summary>
bool writeTilesetJSON(const string& filepath, const Tileset& tileset);

}  // namespace Potree

#endif
