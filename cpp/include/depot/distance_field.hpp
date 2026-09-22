// Exact Euclidean distance transform, and the obstacle distance field built on
// top of it (depot_planner/world/grid.py and hybrid/collision.py both use
// scipy's ``distance_transform_edt`` for this).

#ifndef DEPOT_DISTANCE_FIELD_HPP_
#define DEPOT_DISTANCE_FIELD_HPP_

#include "depot/types.hpp"

namespace depot {

/// Distance, in samples, from every true cell of `mask` to the nearest false
/// cell. False cells get 0, exactly as ``distance_transform_edt`` defines it.
///
/// The transform is exact: the intermediate squared distances are integers and
/// only the final square root is inexact, so the result is bit-identical to
/// scipy's. A mask with no false cell at all has no nearest obstacle, and every
/// cell is reported as infinitely far.
Grid2D<double> EuclideanDistanceTransform(const MaskView& mask);

/// `EuclideanDistanceTransform` in metres: the distance from every drivable
/// cell to the nearest non-drivable one.
Grid2D<double> ObstacleDistance(const MaskView& drivable, double resolution);

}  // namespace depot

#endif  // DEPOT_DISTANCE_FIELD_HPP_
