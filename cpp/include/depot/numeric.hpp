// Scalar helpers that have to agree with CPython bit for bit.
//
// Most of libm is shared: `math.cos` and friends call the same functions this
// code does, and numpy's array versions agree with them on this platform, so an
// ordinary `std::cos` is already identical. Two operations are not:
//
//   * `math.hypot` is not the platform `hypot`. CPython implements its own
//     correctly-rounded norm, and on 200k random pairs `std::hypot` disagreed
//     with it on 0.6% of them by one ulp. One ulp is enough to reorder an open
//     list, so `PythonHypot` reimplements CPython's algorithm exactly.
//   * Python's `%` on floats takes the sign of the divisor; C's `fmod` takes
//     the sign of the dividend. `PythonMod` is the former, `std::fmod` the
//     latter, and the reference uses both (`theta % (2 * pi)` for the heading
//     bin, `math.fmod` for the angle wrap).

#ifndef DEPOT_NUMERIC_HPP_
#define DEPOT_NUMERIC_HPP_

#include <cmath>

namespace depot {

constexpr double kPi = 3.141592653589793;
constexpr double kTwoPi = 6.283185307179586;

/// CPython's two-argument `math.hypot` (Modules/mathmodule.c, vector_norm).
inline double PythonHypot(double ax, double ay) {
  const double kSplit = 134217729.0;  // ldexp(1.0, 27) + 1.0
  double vec[2] = {std::fabs(ax), std::fabs(ay)};
  const bool found_nan = std::isnan(vec[0]) || std::isnan(vec[1]);
  double max = vec[0] > vec[1] ? vec[0] : vec[1];

  if (std::isinf(max)) return max;
  if (found_nan) return std::nan("");
  if (max == 0.0) return max;

  int max_e = 0;
  std::frexp(max, &max_e);
  double x, t, hi, lo, h, oldcsum;
  double csum = 1.0, frac1 = 0.0, frac2 = 0.0, frac3 = 0.0;

  if (max_e >= -1023) {
    const double scale = std::ldexp(1.0, -max_e);
    for (int i = 0; i < 2; ++i) {
      x = vec[i] * scale;
      t = x * kSplit;
      hi = t - (t - x);
      lo = x - hi;

      x = hi * hi;
      oldcsum = csum;
      csum += x;
      frac1 += (oldcsum - csum) + x;

      x = 2.0 * hi * lo;
      oldcsum = csum;
      csum += x;
      frac2 += (oldcsum - csum) + x;

      frac3 += lo * lo;
    }
    h = std::sqrt(csum - 1.0 + (frac1 + frac2 + frac3));

    x = h;
    t = x * kSplit;
    hi = t - (t - x);
    lo = x - hi;

    x = -hi * hi;
    oldcsum = csum;
    csum += x;
    frac1 += (oldcsum - csum) + x;

    x = -2.0 * hi * lo;
    oldcsum = csum;
    csum += x;
    frac2 += (oldcsum - csum) + x;

    x = -lo * lo;
    oldcsum = csum;
    csum += x;
    frac3 += (oldcsum - csum) + x;

    x = csum - 1.0 + (frac1 + frac2 + frac3);
    return (h + x / (2.0 * h)) / scale;
  }

  // Scaling would overflow, so divide by max instead.
  for (int i = 0; i < 2; ++i) {
    x = vec[i] / max;
    x = x * x;
    oldcsum = csum;
    csum += x;
    frac1 += (oldcsum - csum) + x;
  }
  return max * std::sqrt(csum - 1.0 + frac1);
}

/// Python's `value % modulus` for floats: the result takes the sign of the
/// modulus, where C's `fmod` takes the sign of the dividend.
inline double PythonMod(double value, double modulus) {
  double mod = std::fmod(value, modulus);
  if (mod != 0.0) {
    if ((modulus < 0.0) != (mod < 0.0)) mod += modulus;
  } else {
    mod = std::copysign(0.0, modulus);
  }
  return mod;
}

/// Python's `a // b` for floats. It is not `std::floor(a / b)`: CPython derives
/// the quotient from the remainder and then corrects it, which differs from a
/// plain floor when `a / b` rounds across an integer.
inline double PythonFloorDiv(double a, double b) {
  double mod = std::fmod(a, b);
  double div = (a - mod) / b;
  if (mod != 0.0) {
    if ((b < 0.0) != (mod < 0.0)) div -= 1.0;
  }
  double floordiv;
  if (div != 0.0) {
    floordiv = std::floor(div);
    if (div - floordiv > 0.5) floordiv += 1.0;
  } else {
    floordiv = std::copysign(0.0, a / b);
  }
  return floordiv;
}

/// Wrap an angle to (-pi, pi], as hybrid/car.py does.
inline double WrapAngle(double theta) {
  double wrapped = std::fmod(theta, kTwoPi);
  if (wrapped > kPi) {
    wrapped -= kTwoPi;
  } else if (wrapped <= -kPi) {
    wrapped += kTwoPi;
  }
  return wrapped;
}

/// Shortest signed difference a - b, wrapped to (-pi, pi].
inline double AngleDifference(double a, double b) {
  return WrapAngle(a - b);
}

/// Wrap to (-pi, pi] with the convention the Reeds-Shepp word formulas assume.
inline double Mod2Pi(double theta) {
  double value = std::fmod(theta, kTwoPi);
  if (value < -kPi) {
    value += kTwoPi;
  } else if (value > kPi) {
    value -= kTwoPi;
  }
  return value;
}

}  // namespace depot

#endif  // DEPOT_NUMERIC_HPP_
