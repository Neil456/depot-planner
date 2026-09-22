#include "depot/reeds_shepp.hpp"

#include <cmath>

#include "depot/numeric.hpp"

namespace depot {
namespace {

/// One word's solution: three signed lengths, or "no solution".
struct Lengths {
  bool ok = false;
  double t = 0.0;
  double u = 0.0;
  double v = 0.0;
};

void Polar(double x, double y, double* radius, double* angle) {
  *radius = PythonHypot(x, y);
  *angle = std::atan2(y, x);
}

Lengths Lsl(double x, double y, double phi) {
  double u, t;
  Polar(x - std::sin(phi), y - 1.0 + std::cos(phi), &u, &t);
  if (t >= 0.0 && t <= kPi) {
    const double v = Mod2Pi(phi - t);
    if (v >= 0.0 && v <= kPi) return Lengths{true, t, u, v};
  }
  return Lengths{};
}

Lengths Lsr(double x, double y, double phi) {
  double u1, t1;
  Polar(x + std::sin(phi), y - 1.0 - std::cos(phi), &u1, &t1);
  const double squared = u1 * u1;
  if (squared >= 4.0) {
    const double u = std::sqrt(squared - 4.0);
    const double theta = std::atan2(2.0, u);
    const double t = Mod2Pi(t1 + theta);
    const double v = Mod2Pi(t - phi);
    if (t >= 0.0 && v >= 0.0) return Lengths{true, t, u, v};
  }
  return Lengths{};
}

Lengths Lrl(double x, double y, double phi) {
  double u1, t1;
  Polar(x - std::sin(phi), y - 1.0 + std::cos(phi), &u1, &t1);
  if (u1 <= 4.0) {
    const double u = -2.0 * std::asin(0.25 * u1);
    const double t = Mod2Pi(t1 + 0.5 * u + kPi);
    const double v = Mod2Pi(phi - t + u);
    if (t >= 0.0 && u <= 0.0) return Lengths{true, t, u, v};
  }
  return Lengths{};
}

Lengths Sls(double x, double y, double phi) {
  phi = Mod2Pi(phi);
  if (std::fabs(y) < 1e-9 || !(std::fabs(phi) > 1e-9 && std::fabs(phi) < kPi * 0.99)) {
    return Lengths{};
  }
  if (phi < 0.0) return Lengths{};
  const double xd = -y / std::tan(phi) + x;
  const double t = xd - std::tan(phi / 2.0);
  const double u = phi;
  double v = PythonHypot(x - xd, y) - std::tan(phi / 2.0);
  if (y < 0.0) v = -PythonHypot(x - xd, y) - std::tan(phi / 2.0);
  return Lengths{true, t, u, v};
}

using WordFunction = Lengths (*)(double, double, double);

struct Word {
  WordFunction base;
  int steering[3];
};

/// (base function, steering word) pairs; each is expanded by four symmetries,
/// in exactly the order _WORDS and _candidates list them.
const Word kWords[4] = {
    {&Lsl, {kLeft, kStraight, kLeft}},
    {&Lsr, {kLeft, kStraight, kRight}},
    {&Lrl, {kLeft, kRight, kLeft}},
    {&Sls, {kStraight, kLeft, kStraight}},
};

void Add(std::vector<RSPath>* paths, const Lengths& lengths, const int steering[3]) {
  const double values[3] = {lengths.t, lengths.u, lengths.v};
  for (double value : values) {
    if (!std::isfinite(value)) return;
  }
  RSPath path;
  for (int i = 0; i < 3; ++i) {
    if (std::fabs(values[i]) > 1e-9) path.segments.push_back(RSSegment{steering[i], values[i]});
  }
  if (!path.segments.empty()) paths->push_back(std::move(path));
}

Lengths Negated(const Lengths& lengths) {
  return Lengths{lengths.ok, -lengths.t, -lengths.u, -lengths.v};
}

}  // namespace

int RSPath::DirectionChanges() const {
  std::vector<int> gears;
  for (const RSSegment& segment : segments) {
    if (std::fabs(segment.length) > 1e-9) gears.push_back(segment.length >= 0.0 ? 1 : -1);
  }
  int changes = 0;
  for (std::size_t i = 0; i + 1 < gears.size(); ++i) {
    if (gears[i] != gears[i + 1]) ++changes;
  }
  return changes;
}

std::vector<RSPath> ReedsSheppPaths(const Pose& start, const Pose& goal, double max_curvature) {
  const double dx = goal.x - start.x;
  const double dy = goal.y - start.y;
  const double cos_t = std::cos(start.theta);
  const double sin_t = std::sin(start.theta);
  const double x = (cos_t * dx + sin_t * dy) * max_curvature;
  const double y = (-sin_t * dx + cos_t * dy) * max_curvature;
  const double phi = WrapAngle(goal.theta - start.theta);

  std::vector<RSPath> paths;
  for (const Word& word : kWords) {
    const int mirrored[3] = {-word.steering[0], -word.steering[1], -word.steering[2]};

    Lengths result = word.base(x, y, phi);
    if (result.ok) Add(&paths, result, word.steering);

    result = word.base(-x, y, -phi);  // timeflip
    if (result.ok) Add(&paths, Negated(result), word.steering);

    result = word.base(x, -y, -phi);  // reflect
    if (result.ok) Add(&paths, result, mirrored);

    result = word.base(-x, -y, phi);  // timeflip + reflect
    if (result.ok) Add(&paths, Negated(result), mirrored);
  }
  return paths;
}

RSPath ShortestReedsShepp(const Pose& start, const Pose& goal, double max_curvature, bool* found) {
  const std::vector<RSPath> paths = ReedsSheppPaths(start, goal, max_curvature);
  if (paths.empty()) {
    *found = false;
    return RSPath{};
  }
  // Python's min() keeps the first of several equal minima, so the comparison
  // has to be strictly less-than and the scan has to run in order.
  std::size_t best = 0;
  double best_length = paths[0].Length();
  for (std::size_t i = 1; i < paths.size(); ++i) {
    const double length = paths[i].Length();
    if (length < best_length) {
      best = i;
      best_length = length;
    }
  }
  *found = true;
  return paths[best];
}

void InterpolateReedsShepp(const Pose& start, const RSPath& path, const CarModel& car, double step,
                           std::vector<Pose>* poses, std::vector<int>* directions) {
  const double radius = car.min_turning_radius();
  poses->clear();
  directions->clear();
  poses->push_back(start);
  Pose pose = start;
  for (const RSSegment& segment : path.segments) {
    const double distance_m = std::fabs(segment.length) * radius;
    const int gear = segment.length >= 0.0 ? 1 : -1;
    const double steer = segment.steering * car.max_steer;
    const int substeps = std::max(1, static_cast<int>(std::ceil(distance_m / step)));
    const double piece = gear * distance_m / substeps;
    for (int i = 0; i < substeps; ++i) {
      pose = car.Step(pose, steer, piece);
      poses->push_back(pose);
      directions->push_back(gear);
    }
  }
}

double ReedsSheppLengthMetres(const RSPath& path, const CarModel& car) {
  return path.Length() * car.min_turning_radius();
}

}  // namespace depot
