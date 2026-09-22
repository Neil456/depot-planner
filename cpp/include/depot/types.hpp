// Shared value types for the planning core.
//
// Everything here is either a plain aggregate or an owning container; the
// library never hands out or takes ownership of a raw pointer. Views are
// explicitly non-owning and borrow for the duration of a call.

#ifndef DEPOT_TYPES_HPP_
#define DEPOT_TYPES_HPP_

#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace depot {

/// A grid cell addressed as (x, y); the backing arrays are indexed [y, x].
struct Cell {
  int x = 0;
  int y = 0;

  friend bool operator==(const Cell& a, const Cell& b) { return a.x == b.x && a.y == b.y; }
  friend bool operator!=(const Cell& a, const Cell& b) { return !(a == b); }
};

/// A continuous planar pose. ``x`` grows right, ``y`` grows *down*, and
/// ``theta`` is measured from +x towards +y, as everywhere else in the project.
struct Pose {
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
};

/// A sequence of grid cells.
using Path = std::vector<Cell>;

/// A dense row-major 2D array that owns its storage.
template <typename T>
class Grid2D {
 public:
  Grid2D() = default;

  Grid2D(int rows, int cols, T fill = T())
      : rows_(rows), cols_(cols), data_(static_cast<std::size_t>(rows) * cols, fill) {
    if (rows < 0 || cols < 0) throw std::invalid_argument("grid dimensions must not be negative");
  }

  int rows() const { return rows_; }
  int cols() const { return cols_; }
  std::size_t size() const { return data_.size(); }

  bool InBounds(int x, int y) const { return x >= 0 && x < cols_ && y >= 0 && y < rows_; }
  std::size_t Index(int x, int y) const { return static_cast<std::size_t>(y) * cols_ + x; }

  T& at(int x, int y) { return data_[Index(x, y)]; }
  const T& at(int x, int y) const { return data_[Index(x, y)]; }

  T* data() { return data_.data(); }
  const T* data() const { return data_.data(); }

 private:
  int rows_ = 0;
  int cols_ = 0;
  std::vector<T> data_;
};

/// A read-only row-major view over memory somebody else owns (a numpy buffer,
/// or a Grid2D). It is valid only for as long as that owner is.
template <typename T>
class GridView {
 public:
  GridView() = default;

  GridView(const T* data, int rows, int cols) : data_(data), rows_(rows), cols_(cols) {
    if (data == nullptr || rows <= 0 || cols <= 0) {
      throw std::invalid_argument("a grid view needs a non-empty 2D buffer");
    }
  }

  explicit GridView(const Grid2D<T>& grid)
      : data_(grid.data()), rows_(grid.rows()), cols_(grid.cols()) {}

  int rows() const { return rows_; }
  int cols() const { return cols_; }
  std::size_t size() const { return static_cast<std::size_t>(rows_) * cols_; }
  /// A default-constructed view holds nothing; callers use this for an
  /// optional argument such as a precomputed heuristic field.
  bool empty() const { return data_ == nullptr; }

  bool InBounds(int x, int y) const { return x >= 0 && x < cols_ && y >= 0 && y < rows_; }
  std::size_t Index(int x, int y) const { return static_cast<std::size_t>(y) * cols_ + x; }

  const T& at(int x, int y) const { return data_[Index(x, y)]; }
  const T& operator[](std::size_t index) const { return data_[index]; }
  const T* data() const { return data_; }

 private:
  const T* data_ = nullptr;
  int rows_ = 0;
  int cols_ = 0;
};

/// Per-cell driving cost; non-finite means "not drivable".
using CostMap = Grid2D<double>;
using CostMapView = GridView<double>;
using MaskView = GridView<bool>;

/// Budgets shared by every search in the core.
struct SearchLimits {
  /// Hard cap on expansions; <= 0 means unlimited.
  std::int64_t max_nodes = 0;
  /// Wall-clock budget in milliseconds. Negative means unlimited; zero expires
  /// at once, which is what the Python side does with ``time_limit_ms=0.0``.
  double time_limit_ms = -1.0;
};

/// Outcome of one grid search.
struct SearchResult {
  bool success = false;
  Path path;
  double cost = std::numeric_limits<double>::infinity();
  std::int64_t nodes_expanded = 0;
  double runtime_ms = 0.0;
  std::string reason;
};

}  // namespace depot

#endif  // DEPOT_TYPES_HPP_
