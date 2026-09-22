// Hybrid A*: car-shaped planning over continuous (x, y, heading) states
// (depot_planner/hybrid/search.py and hybrid/collision.py).
//
// The search expands constant-steering arcs of a fixed length, forward and
// reverse, collision checking each arc in sub-steps. Duplicate detection
// discretises the continuous state to a position cell and a heading bin. The
// heuristic is the larger of the straight-line distance and an obstacle-aware
// grid distance from the goal. A Reeds-Shepp analytic expansion is attempted
// periodically so the exact goal pose is reachable.

#ifndef DEPOT_HYBRID_HPP_
#define DEPOT_HYBRID_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "depot/car.hpp"
#include "depot/types.hpp"

namespace depot {

/// A conservative lower bound on the distance to the nearest obstacle,
/// sampled on a regular grid. Points outside it are treated as obstacles
/// (distance 0), which keeps the car inside the modelled world.
class DistanceFieldView {
 public:
  DistanceFieldView() = default;
  DistanceFieldView(const CostMapView& distance, double resolution)
      : distance_(distance), resolution_(resolution) {}

  /// Nearest-sample lookup; 0 outside the field.
  double DistanceAt(double x, double y) const {
    const int j = static_cast<int>(std::floor(x / resolution_));
    const int i = static_cast<int>(std::floor(y / resolution_));
    if (i < 0 || i >= distance_.rows() || j < 0 || j >= distance_.cols()) return 0.0;
    return distance_.at(j, i);
  }

 private:
  CostMapView distance_;
  double resolution_ = 0.05;
};

/// Checks car poses by covering the footprint with discs.
class CarCollisionChecker {
 public:
  CarCollisionChecker(DistanceFieldView field, const CarModel& car, double safety_margin)
      : field_(field),
        offsets_(car.disc_offsets()),
        clearance_(car.disc_radius() + safety_margin) {}

  /// True if the car at `pose` clears every obstacle.
  bool IsFree(const Pose& pose) const {
    ++checks_;
    return Clear(pose);
  }

  /// True if every pose in [begin, end) clears every obstacle. The check count
  /// grows by the number of poses examined, exactly as the reference's
  /// vectorised call does, whatever the early exit finds.
  bool AllFree(const Pose* poses, std::size_t count) const {
    checks_ += static_cast<std::int64_t>(count);
    for (std::size_t i = 0; i < count; ++i) {
      if (!Clear(poses[i])) return false;
    }
    return true;
  }

  std::int64_t checks() const { return checks_; }

 private:
  bool Clear(const Pose& pose) const {
    const double cos_t = std::cos(pose.theta);
    const double sin_t = std::sin(pose.theta);
    for (double offset : offsets_) {
      const double cx = pose.x + offset * cos_t;
      const double cy = pose.y + offset * sin_t;
      if (!(field_.DistanceAt(cx, cy) >= clearance_)) return false;
    }
    return true;
  }

  DistanceFieldView field_;
  std::vector<double> offsets_;
  double clearance_ = 0.0;
  mutable std::int64_t checks_ = 0;
};

/// Everything configs/hybrid.yaml controls.
struct HybridOptions {
  double xy_resolution = 0.5;
  int heading_bins = 72;

  double arc_length = 1.0;
  std::vector<double> steer_fractions{-1.0, -0.5, 0.0, 0.5, 1.0};
  int substeps = 5;
  std::vector<int> directions{1, -1};

  double reverse_multiplier = 2.0;
  double direction_change = 5.0;
  double steer_penalty = 0.5;
  double steer_change_penalty = 0.3;

  double position_tolerance = 0.3;
  double heading_tolerance = 0.08726646259971647;  // 5 degrees, in radians

  std::int64_t max_expansions = 30000;
  double time_limit_s = 20.0;

  bool analytic_enabled = true;
  int analytic_every = 8;
  double analytic_max_distance = 30.0;
  double analytic_step = 0.2;

  bool octile_correction = true;
  double safety_margin = 0.0;
};

/// Outcome of one hybrid A* search.
struct HybridResult {
  bool success = false;
  std::vector<Pose> poses;
  std::vector<int> directions;
  double cost = std::numeric_limits<double>::infinity();
  double path_length_m = 0.0;
  int direction_switches = 0;
  std::int64_t nodes_expanded = 0;
  double runtime_ms = 0.0;
  std::int64_t collision_checks = 0;
  bool used_analytic = false;
  std::string reason;
};

/// Index of the heading bin containing `theta`.
int HeadingBin(double theta, int bins);

/// True if `pose` is within both tolerances of `goal`, wrapping the heading.
bool AtGoal(const Pose& pose, const Pose& goal, double position_tolerance,
            double heading_tolerance);

/// Plan a car-shaped path from `start` to `goal`.
///
/// `heuristic_field` is the obstacle-aware distance in metres from every coarse
/// cell to the goal; pass an empty view for the straight-line heuristic alone.
HybridResult HybridPlan(const Pose& start, const Pose& goal, const CarModel& car,
                        const DistanceFieldView& field, const CostMapView& heuristic_field,
                        double heuristic_resolution, const HybridOptions& options);

}  // namespace depot

#endif  // DEPOT_HYBRID_HPP_
