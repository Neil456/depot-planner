// Kinematic bicycle car model and its footprint (depot_planner/hybrid/car.py).
//
// Coordinates follow the rest of the project: x grows right, y grows *down*,
// and the heading is measured from +x towards +y. The reference point is the
// centre of the rear axle.

#ifndef DEPOT_CAR_HPP_
#define DEPOT_CAR_HPP_

#include <cmath>
#include <vector>

#include "depot/numeric.hpp"
#include "depot/types.hpp"

namespace depot {

/// Geometry and kinematics of the ego car.
struct CarModel {
  double wheelbase = 2.7;
  double length = 4.5;
  double width = 1.9;
  double max_steer = 0.6;
  double rear_overhang = 0.9;
  int n_discs = 4;

  /// Distance from the rear axle to the front bumper.
  double front_overhang() const { return length - rear_overhang; }

  /// Turning radius at full steering lock.
  double min_turning_radius() const { return wheelbase / std::tan(max_steer); }

  double max_curvature() const { return 1.0 / min_turning_radius(); }

  /// Radius of each covering disc: the half-diagonal of one footprint slice.
  double disc_radius() const { return PythonHypot(length / (2.0 * n_discs), width / 2.0); }

  /// Longitudinal offsets of the disc centres from the rear axle.
  std::vector<double> disc_offsets() const {
    const double slice_length = length / n_discs;
    std::vector<double> offsets;
    offsets.reserve(static_cast<std::size_t>(n_discs));
    for (int index = 0; index < n_discs; ++index) {
      offsets.push_back(-rear_overhang + (index + 0.5) * slice_length);
    }
    return offsets;
  }

  /// Exact integration of one constant-steering arc of signed `distance`.
  Pose Step(const Pose& pose, double steer, double distance) const {
    const double curvature = std::tan(steer) / wheelbase;
    if (std::fabs(curvature) < 1e-9) {
      return Pose{pose.x + distance * std::cos(pose.theta),
                  pose.y + distance * std::sin(pose.theta), pose.theta};
    }
    const double new_theta = pose.theta + curvature * distance;
    return Pose{pose.x + (std::sin(new_theta) - std::sin(pose.theta)) / curvature,
                pose.y - (std::cos(new_theta) - std::cos(pose.theta)) / curvature,
                WrapAngle(new_theta)};
  }

  /// Poses along one arc, excluding the start and including the end.
  std::vector<Pose> SampleArc(const Pose& pose, double steer, double distance, int substeps) const {
    const double step_distance = distance / substeps;
    std::vector<Pose> poses;
    poses.reserve(static_cast<std::size_t>(substeps));
    Pose current = pose;
    for (int i = 0; i < substeps; ++i) {
      current = Step(current, steer, step_distance);
      poses.push_back(current);
    }
    return poses;
  }
};

}  // namespace depot

#endif  // DEPOT_CAR_HPP_
