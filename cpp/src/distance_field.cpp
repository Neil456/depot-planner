#include "depot/distance_field.hpp"

#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

namespace depot {
namespace {

// The transform runs in two passes. The first reduces every column to the
// number of rows to the nearest background cell; the second takes the lower
// envelope of the parabolas those column distances define, which is the
// Felzenszwalb-Huttenlocher algorithm.
//
// Both passes stay in exact integer arithmetic. The parabola intersections are
// rationals, so they are kept as (numerator, denominator) pairs and compared by
// cross-multiplication rather than being evaluated in floating point: that way
// the squared distances this produces are exactly the integers scipy produces,
// and the only inexact step in the whole transform is the final square root.
//
// A column with no background cell anywhere is given the sentinel distance
// `rows + cols + 1`, which is larger than any real distance in the grid, so a
// column that does have a background cell always wins. If the whole mask is
// background-free the result stays at the sentinel and is reported as infinite.

struct Envelope {
  std::vector<int> vertex;          // the parabola owning each interval
  std::vector<std::int64_t> numer;  // interval boundaries, as numer / denom
  std::vector<std::int64_t> denom;
};

/// Lower envelope of the parabolas (q - i)^2 + f[i], written into `out`.
void LowerEnvelope(const std::vector<std::int64_t>& f, int n, Envelope& scratch,
                   std::vector<std::int64_t>& out) {
  scratch.vertex.assign(n, 0);
  scratch.numer.assign(n + 1, 0);
  scratch.denom.assign(n + 1, 0);

  int top = 0;
  scratch.vertex[0] = 0;
  // The first interval starts at minus infinity, which no candidate can undercut.
  scratch.numer[0] = -1;
  scratch.denom[0] = 0;

  for (int q = 1; q < n; ++q) {
    std::int64_t numer = 0;
    std::int64_t denom = 0;
    for (;;) {
      const std::int64_t v = scratch.vertex[top];
      numer = (f[q] + static_cast<std::int64_t>(q) * q) - (f[v] + v * v);
      denom = 2 * (static_cast<std::int64_t>(q) - v);
      if (top == 0) break;
      // numer / denom <= numer[top] / denom[top], with both denominators positive.
      if (numer * scratch.denom[top] <= scratch.numer[top] * denom) {
        --top;
        continue;
      }
      break;
    }
    ++top;
    scratch.vertex[top] = q;
    scratch.numer[top] = numer;
    scratch.denom[top] = denom;
  }

  int k = 0;
  for (int q = 0; q < n; ++q) {
    while (k < top && scratch.numer[k + 1] < static_cast<std::int64_t>(q) * scratch.denom[k + 1]) {
      ++k;
    }
    const std::int64_t v = scratch.vertex[k];
    const std::int64_t offset = static_cast<std::int64_t>(q) - v;
    out[q] = offset * offset + f[v];
  }
}

}  // namespace

Grid2D<double> EuclideanDistanceTransform(const MaskView& mask) {
  const int rows = mask.rows();
  const int cols = mask.cols();
  const std::int64_t sentinel = static_cast<std::int64_t>(rows) + cols + 1;
  const std::int64_t unreachable = sentinel * sentinel;

  // Pass 1: rows to the nearest background cell within each column.
  Grid2D<std::int64_t> vertical(rows, cols, 0);
  for (int x = 0; x < cols; ++x) {
    std::int64_t running = mask.at(x, 0) ? sentinel : 0;
    vertical.at(x, 0) = running;
    for (int y = 1; y < rows; ++y) {
      running = mask.at(x, y) ? (running >= sentinel ? sentinel : running + 1) : 0;
      vertical.at(x, y) = running;
    }
    for (int y = rows - 2; y >= 0; --y) {
      const std::int64_t from_below =
          vertical.at(x, y + 1) >= sentinel ? sentinel : vertical.at(x, y + 1) + 1;
      if (from_below < vertical.at(x, y)) vertical.at(x, y) = from_below;
    }
  }

  // Pass 2: the lower envelope along each row.
  Grid2D<double> distance(rows, cols, 0.0);
  Envelope scratch;
  std::vector<std::int64_t> squared(cols, 0);
  std::vector<std::int64_t> row(cols, 0);
  for (int y = 0; y < rows; ++y) {
    for (int x = 0; x < cols; ++x) {
      const std::int64_t d = vertical.at(x, y);
      squared[x] = d * d;
    }
    LowerEnvelope(squared, cols, scratch, row);
    for (int x = 0; x < cols; ++x) {
      distance.at(x, y) = row[x] >= unreachable ? std::numeric_limits<double>::infinity()
                                                : std::sqrt(static_cast<double>(row[x]));
    }
  }
  return distance;
}

Grid2D<double> ObstacleDistance(const MaskView& drivable, double resolution) {
  Grid2D<double> distance = EuclideanDistanceTransform(drivable);
  const std::size_t cells = distance.size();
  double* values = distance.data();
  for (std::size_t i = 0; i < cells; ++i) values[i] *= resolution;
  return distance;
}

}  // namespace depot
