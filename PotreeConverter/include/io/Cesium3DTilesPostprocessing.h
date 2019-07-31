#pragma once

#include <string>

namespace Potree {
struct SRSTransformHelper;
class PointAttributes;
class AABB;
}

struct ProgressReporter;

/**
 * Runs postprocessing steps necessary for writing valid 3D-Tiles files:
 *   - Removing .idx files
 *   - Transforming point coordinates into target space using the given transform
 *   - Writing tileset JSON files
 */
void
do_cesium_3dtiles_postprocessing(const std::string& work_dir,
                                 const Potree::AABB& bounds,
                                 float spacing_at_root,
                                 const Potree::SRSTransformHelper& transform,
                                 const Potree::PointAttributes& point_attributes,
                                 ProgressReporter* progress);