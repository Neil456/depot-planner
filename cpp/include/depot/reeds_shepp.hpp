// Reeds-Shepp curves: shortest paths for a car that can drive forwards and back
// (depot_planner/hybrid/reeds_shepp.py).
//
// Used as the analytic expansion of hybrid A*: once the search gets near the
// goal, a Reeds-Shepp curve connects the last state to the goal pose exactly,
// which is what makes the 0.3 m / 5 degree goal tolerance reachable at all.
//
// The word set covers CSC (LSL, LSR), CCC (LRL) and SCS (SLS), each expanded by
// the standard timeflip and reflect symmetries. That is not the full 48-word
// Reeds-Shepp set, so the curve returned is a valid shortest *candidate*, not
// provably the global optimum; every candidate is collision checked and its
// endpoint verified before use. The C++ port keeps the same word set and the
// same generation order, so it picks the same candidate as the reference.

#ifndef DEPOT_REEDS_SHEPP_HPP_
#define DEPOT_REEDS_SHEPP_HPP_

#include <vector>

#include "depot/car.hpp"
#include "depot/types.hpp"

namespace depot {

constexpr int kLeft = 1;
constexpr int kStraight = 0;
constexpr int kRight = -1;

/// One Reeds-Shepp segment: a steering direction and a signed length in units
/// of the turning radius. The sign of `length` is the gear.
struct RSSegment {
  int steering = 0;
  double length = 0.0;
};

/// A sequence of Reeds-Shepp segments.
struct RSPath {
  std::vector<RSSegment> segments;

  /// Total path length in units of the turning radius.
  double Length() const {
    double total = 0.0;
    for (const RSSegment& segment : segments) total += std::fabs(segment.length);
    return total;
  }

  /// How many times the curve switches between forward and reverse.
  int DirectionChanges() const;
};

/// Every candidate curve from `start` to `goal`, in the reference's order.
std::vector<RSPath> ReedsSheppPaths(const Pose& start, const Pose& goal, double max_curvature);

/// The shortest candidate curve; `found` is false when no word applies.
RSPath ShortestReedsShepp(const Pose& start, const Pose& goal, double max_curvature, bool* found);

/// Sample a curve into poses and per-step gears (+1 forward, -1 reverse).
///
/// The first pose returned is `start`; `directions[i]` is the gear used to get
/// from `poses[i]` to `poses[i + 1]`.
void InterpolateReedsShepp(const Pose& start, const RSPath& path, const CarModel& car, double step,
                           std::vector<Pose>* poses, std::vector<int>* directions);

/// Curve length in metres for a given car's turning radius.
double ReedsSheppLengthMetres(const RSPath& path, const CarModel& car);

}  // namespace depot

#endif  // DEPOT_REEDS_SHEPP_HPP_
