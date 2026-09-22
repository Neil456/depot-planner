#include "depot/hybrid.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <queue>
#include <unordered_map>

#include "depot/numeric.hpp"
#include "depot/reeds_shepp.hpp"

namespace depot {
namespace {

/// The motion primitives, precomputed once in the car's own frame.
///
/// The bicycle model is invariant to the starting pose, so each arc is
/// integrated once from the origin and then rotated and translated onto every
/// node, which is what the reference's numpy version does.
class PrimitiveSet {
 public:
  PrimitiveSet(const CarModel& car, double arc_length, const std::vector<double>& steers,
               const std::vector<int>& directions, int substeps)
      : substeps_(substeps) {
    for (int direction : directions) {
      for (double steer : steers) {
        const std::vector<Pose> arc =
            car.SampleArc(Pose{0.0, 0.0, 0.0}, steer, direction * arc_length, substeps);
        local_.insert(local_.end(), arc.begin(), arc.end());
        gears_.push_back(direction);
        steers_.push_back(steer);
      }
    }
  }

  std::size_t size() const { return gears_.size(); }
  int substeps() const { return substeps_; }
  int gear(std::size_t index) const { return gears_[index]; }
  double steer(std::size_t index) const { return steers_[index]; }

  /// Write the poses of primitive `index` reached from `pose` into `out`.
  ///
  /// The heading is left unwrapped here, exactly as the reference leaves it:
  /// the collision check runs on these values and only the stored successor is
  /// wrapped afterwards.
  void Expand(const Pose& pose, std::size_t index, Pose* out) const {
    const double cos_t = std::cos(pose.theta);
    const double sin_t = std::sin(pose.theta);
    const Pose* arc = &local_[index * static_cast<std::size_t>(substeps_)];
    for (int i = 0; i < substeps_; ++i) {
      out[i].x = pose.x + arc[i].x * cos_t - arc[i].y * sin_t;
      out[i].y = pose.y + arc[i].x * sin_t + arc[i].y * cos_t;
      out[i].theta = pose.theta + arc[i].theta;
    }
  }

 private:
  std::vector<Pose> local_;
  std::vector<int> gears_;
  std::vector<double> steers_;
  int substeps_ = 1;
};

/// max(Euclidean, obstacle-aware grid distance) to the goal.
class Heuristic {
 public:
  Heuristic(Pose goal, CostMapView field, double resolution, bool correct_octile)
      : goal_(goal), field_(field), resolution_(resolution) {
    // An 8-connected grid overestimates a straight line by up to 1 / cos(pi/8);
    // scaling the field back keeps it a lower bound in free space.
    scale_ = correct_octile ? std::cos(kPi / 8.0) : 1.0;
  }

  double operator()(const Pose& pose) const {
    const double euclid = PythonHypot(pose.x - goal_.x, pose.y - goal_.y);
    if (field_.empty()) return euclid;
    const int j = static_cast<int>(PythonFloorDiv(pose.x, resolution_));
    const int i = static_cast<int>(PythonFloorDiv(pose.y, resolution_));
    if (i < 0 || i >= field_.rows() || j < 0 || j >= field_.cols()) return euclid;
    const double grid = field_.at(j, i) * scale_;
    if (!std::isfinite(grid)) return std::numeric_limits<double>::infinity();
    return std::max(euclid, grid);
  }

 private:
  Pose goal_;
  CostMapView field_;
  double resolution_ = 1.0;
  double scale_ = 1.0;
};

/// One search node. Replaces the reference's `_Node` plus its `g_score` entry
/// and its membership of `closed`.
struct Node {
  Pose pose;
  double g = 0.0;
  int direction = 1;
  double steer = 0.0;
  std::int64_t parent = -1;  // parent key, or -1 at the root
  bool has_parent = false;
  bool closed = false;
  std::vector<Pose> segment;  // dense poses from the parent to this node
};

struct Entry {
  double f;
  std::int64_t order;
  std::int64_t key;
};

struct EntryGreater {
  bool operator()(const Entry& a, const Entry& b) const {
    if (a.f != b.f) return a.f > b.f;
    return a.order > b.order;
  }
};

constexpr std::int64_t kKeyBias = 1 << 20;

/// Pack the discretised state into one integer. The lattice cell indices are
/// non-negative in practice (a pose outside the field is in collision), but the
/// bias keeps the packing valid if one ever is not.
std::int64_t PackKey(std::int64_t gx, std::int64_t gy, int bin) {
  return ((gx + kKeyBias) * (2 * kKeyBias) + (gy + kKeyBias)) * 4096 + bin;
}

std::int64_t StateKey(const Pose& pose, double xy_resolution, int bins) {
  const auto gx = static_cast<std::int64_t>(std::floor(pose.x / xy_resolution));
  const auto gy = static_cast<std::int64_t>(std::floor(pose.y / xy_resolution));
  return PackKey(gx, gy, HeadingBin(pose.theta, bins));
}

HybridResult Failure(const std::string& reason, std::int64_t expansions, double runtime_ms,
                     std::int64_t checks) {
  HybridResult result;
  result.nodes_expanded = expansions;
  result.runtime_ms = runtime_ms;
  result.collision_checks = checks;
  result.reason = reason;
  return result;
}

/// Walk the parent chain back to the start and assemble the dense path.
HybridResult Finish(const std::unordered_map<std::int64_t, Node>& nodes, std::int64_t key,
                    const Pose& start, const std::vector<Pose>& tail_poses,
                    const std::vector<int>& tail_directions, double cost, std::int64_t expansions,
                    double runtime_ms, std::int64_t checks, bool used_analytic,
                    const std::string& reason) {
  std::vector<std::pair<const std::vector<Pose>*, int>> segments;
  std::int64_t current = key;
  bool walking = true;
  while (walking) {
    const Node& node = nodes.at(current);
    if (node.has_parent) segments.emplace_back(&node.segment, node.direction);
    walking = node.has_parent;
    current = node.parent;
  }
  std::reverse(segments.begin(), segments.end());

  HybridResult result;
  result.success = true;
  result.poses.push_back(start);
  for (const auto& entry : segments) {
    for (const Pose& pose : *entry.first) {
      result.poses.push_back(pose);
      result.directions.push_back(entry.second);
    }
  }
  for (const Pose& pose : tail_poses) result.poses.push_back(pose);
  for (int direction : tail_directions) result.directions.push_back(direction);

  double length = 0.0;
  for (std::size_t i = 0; i + 1 < result.poses.size(); ++i) {
    length += PythonHypot(result.poses[i + 1].x - result.poses[i].x,
                          result.poses[i + 1].y - result.poses[i].y);
  }
  result.path_length_m = length;
  for (std::size_t i = 0; i + 1 < result.directions.size(); ++i) {
    if (result.directions[i] != result.directions[i + 1]) ++result.direction_switches;
  }
  result.cost = cost;
  result.nodes_expanded = expansions;
  result.runtime_ms = runtime_ms;
  result.collision_checks = checks;
  result.used_analytic = used_analytic;
  result.reason = reason;
  return result;
}

/// Try to connect `node` to `goal` with a collision-free Reeds-Shepp curve.
bool AnalyticShot(const Node& node, const Pose& goal, const CarModel& car,
                  const CarCollisionChecker& checker, double step, double reverse_multiplier,
                  double direction_change, std::vector<Pose>* tail_poses,
                  std::vector<int>* tail_directions, double* tail_cost) {
  bool found = false;
  const RSPath path = ShortestReedsShepp(node.pose, goal, car.max_curvature(), &found);
  if (!found) return false;

  std::vector<Pose> poses;
  std::vector<int> directions;
  InterpolateReedsShepp(node.pose, path, car, step, &poses, &directions);
  // The first pose is the node itself and was already checked.
  if (poses.size() > 1 && !checker.AllFree(poses.data() + 1, poses.size() - 1)) return false;

  double cost = 0.0;
  int previous = node.direction;
  const double radius = car.min_turning_radius();
  for (const RSSegment& segment : path.segments) {
    const double distance = std::fabs(segment.length) * radius;
    const int gear = segment.length >= 0.0 ? 1 : -1;
    cost += distance * (gear < 0 ? reverse_multiplier : 1.0);
    if (gear != previous) cost += direction_change;
    previous = gear;
  }

  tail_poses->assign(poses.begin() + 1, poses.end());
  *tail_directions = std::move(directions);
  *tail_cost = cost;
  return true;
}

}  // namespace

int HeadingBin(double theta, int bins) {
  const double width = kTwoPi / bins;
  const int index = static_cast<int>(PythonMod(theta, kTwoPi) / width);
  return ((index % bins) + bins) % bins;
}

bool AtGoal(const Pose& pose, const Pose& goal, double position_tolerance,
            double heading_tolerance) {
  if (PythonHypot(pose.x - goal.x, pose.y - goal.y) > position_tolerance) return false;
  return std::fabs(AngleDifference(pose.theta, goal.theta)) <= heading_tolerance;
}

HybridResult HybridPlan(const Pose& start, const Pose& goal, const CarModel& car,
                        const DistanceFieldView& field, const CostMapView& heuristic_field,
                        double heuristic_resolution, const HybridOptions& options) {
  const auto began = std::chrono::steady_clock::now();
  const auto elapsed_ms = [&began]() {
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - began)
        .count();
  };

  const CarCollisionChecker checker(field, car, options.safety_margin);
  if (!checker.IsFree(start)) {
    return Failure("start pose is in collision", 0, elapsed_ms(), checker.checks());
  }
  if (!checker.IsFree(goal)) {
    return Failure("goal pose is in collision", 0, elapsed_ms(), checker.checks());
  }

  std::vector<double> steers;
  steers.reserve(options.steer_fractions.size());
  for (double fraction : options.steer_fractions) steers.push_back(fraction * car.max_steer);
  const PrimitiveSet fan(car, options.arc_length, steers, options.directions, options.substeps);

  const Heuristic heuristic(goal, heuristic_field, heuristic_resolution, options.octile_correction);
  const double start_h = heuristic(start);
  if (!std::isfinite(start_h)) {
    return Failure("goal is unreachable from the start", 0, elapsed_ms(), checker.checks());
  }

  std::unordered_map<std::int64_t, Node> nodes;
  nodes.reserve(4096);
  const std::int64_t start_key = StateKey(start, options.xy_resolution, options.heading_bins);
  Node root;
  root.pose = start;
  root.g = 0.0;
  root.direction = 1;
  root.steer = 0.0;
  nodes.emplace(start_key, std::move(root));

  std::priority_queue<Entry, std::vector<Entry>, EntryGreater> open;
  open.push(Entry{start_h, 0, start_key});
  std::int64_t order = 0;
  std::int64_t expansions = 0;

  const int substeps = options.substeps;
  std::vector<Pose> arc(static_cast<std::size_t>(substeps));
  std::vector<Pose> tail_poses;
  std::vector<int> tail_directions;
  const int analytic_every = std::max(1, options.analytic_every);

  while (!open.empty()) {
    const Entry entry = open.top();
    open.pop();
    Node& node = nodes.at(entry.key);
    if (node.closed) continue;
    node.closed = true;
    ++expansions;

    const Pose node_pose = node.pose;
    const double node_g = node.g;
    const int node_direction = node.direction;
    const double node_steer = node.steer;

    if (AtGoal(node_pose, goal, options.position_tolerance, options.heading_tolerance)) {
      return Finish(nodes, entry.key, start, {}, {}, node_g, expansions, elapsed_ms(),
                    checker.checks(), false, "goal reached");
    }

    if (expansions >= options.max_expansions) {
      return Failure("expansion limit reached", expansions, elapsed_ms(), checker.checks());
    }
    if (elapsed_ms() > options.time_limit_s * 1000.0) {
      return Failure("time limit reached", expansions, elapsed_ms(), checker.checks());
    }

    // Reeds-Shepp shot at the goal.
    if (options.analytic_enabled && expansions % analytic_every == 0 &&
        PythonHypot(node_pose.x - goal.x, node_pose.y - goal.y) <= options.analytic_max_distance) {
      double tail_cost = 0.0;
      if (AnalyticShot(nodes.at(entry.key), goal, car, checker, options.analytic_step,
                       options.reverse_multiplier, options.direction_change, &tail_poses,
                       &tail_directions, &tail_cost)) {
        return Finish(nodes, entry.key, start, tail_poses, tail_directions, node_g + tail_cost,
                      expansions, elapsed_ms(), checker.checks(), true,
                      "goal reached via Reeds-Shepp expansion");
      }
    }

    for (std::size_t index = 0; index < fan.size(); ++index) {
      fan.Expand(node_pose, index, arc.data());
      if (!checker.AllFree(arc.data(), arc.size())) continue;

      const int direction = fan.gear(index);
      const double steer = fan.steer(index);
      Pose successor = arc[static_cast<std::size_t>(substeps) - 1];
      successor.theta = WrapAngle(successor.theta);
      const std::int64_t successor_key =
          StateKey(successor, options.xy_resolution, options.heading_bins);

      auto found = nodes.find(successor_key);
      if (found != nodes.end() && found->second.closed) continue;

      double step_cost = options.arc_length * (direction < 0 ? options.reverse_multiplier : 1.0);
      if (direction != node_direction) step_cost += options.direction_change;
      step_cost += options.steer_penalty * std::fabs(steer);
      step_cost += options.steer_change_penalty * std::fabs(steer - node_steer);
      const double tentative = node_g + step_cost;
      const double known =
          found == nodes.end() ? std::numeric_limits<double>::infinity() : found->second.g;
      if (tentative >= known - 1e-12) continue;

      const double h = heuristic(successor);
      if (!std::isfinite(h)) continue;

      Node& slot = found == nodes.end() ? nodes[successor_key] : found->second;
      slot.pose = successor;
      slot.g = tentative;
      slot.direction = direction;
      slot.steer = steer;
      slot.parent = entry.key;
      slot.has_parent = true;
      slot.closed = false;
      slot.segment.assign(arc.begin(), arc.end());
      for (Pose& pose : slot.segment) pose.theta = WrapAngle(pose.theta);
      ++order;
      open.push(Entry{tentative + h, order, successor_key});
    }
  }

  return Failure("no path found", expansions, elapsed_ms(), checker.checks());
}

}  // namespace depot
