#include "depot/grid_search.hpp"

#include <cmath>
#include <cstddef>
#include <queue>
#include <vector>

namespace depot {
namespace {

/// One entry of the Dijkstra-field open list. Mirrors the Python heap, which
/// pushes the tuple (distance, x, y): ties on the distance go to the smaller x,
/// then to the smaller y.
struct FieldEntry {
  double distance;
  int x;
  int y;
};

struct FieldEntryGreater {
  bool operator()(const FieldEntry& a, const FieldEntry& b) const {
    if (a.distance != b.distance) return a.distance > b.distance;
    if (a.x != b.x) return a.x > b.x;
    return a.y > b.y;
  }
};

}  // namespace

const Move kMoves[8] = {
    {1, 0, 1.0},    {-1, 0, 1.0},    {0, 1, 1.0},     {0, -1, 1.0},
    {1, 1, kSqrt2}, {1, -1, kSqrt2}, {-1, 1, kSqrt2}, {-1, -1, kSqrt2},
};

double Octile(Cell a, Cell b) {
  const double dx = std::abs(a.x - b.x);
  const double dy = std::abs(a.y - b.y);
  return (dx + dy) + (kSqrt2 - 2.0) * std::min(dx, dy);
}

double MinFiniteCost(const CostMapView& cost) {
  double smallest = std::numeric_limits<double>::infinity();
  const std::size_t cells = cost.size();
  for (std::size_t i = 0; i < cells; ++i) {
    if (std::isfinite(cost[i]) && cost[i] < smallest) smallest = cost[i];
  }
  return std::isfinite(smallest) ? smallest : 0.0;
}

SearchResult GridSearch(const CostMapView& cost, Cell start, Cell goal,
                        const GridSearchOptions& options) {
  // The Python side scales the octile heuristic by the cheapest drivable cell,
  // and a weight of zero turns it into Dijkstra. Both branches below are
  // numerically identical to that, but the zero branch skips the distance.
  if (options.weight == 0.0) {
    return BestFirstSearch(cost, start, goal, ZeroHeuristic{}, options.limits);
  }
  const double scale =
      std::isnan(options.min_cell_cost) ? MinFiniteCost(cost) : options.min_cell_cost;
  OctileHeuristic heuristic{goal, scale * options.weight};
  return BestFirstSearch(cost, start, goal, heuristic, options.limits);
}

CostMap DijkstraField(const CostMapView& cost, const std::vector<Cell>& sources) {
  const int rows = cost.rows();
  const int cols = cost.cols();
  CostMap distance(rows, cols, std::numeric_limits<double>::infinity());
  const auto drivable = [&cost](int x, int y) { return std::isfinite(cost.at(x, y)); };

  std::priority_queue<FieldEntry, std::vector<FieldEntry>, FieldEntryGreater> open;
  for (const Cell& source : sources) {
    if (!cost.InBounds(source.x, source.y) || !drivable(source.x, source.y)) continue;
    if (distance.at(source.x, source.y) > 0.0) {
      distance.at(source.x, source.y) = 0.0;
      open.push(FieldEntry{0.0, source.x, source.y});
    }
  }

  while (!open.empty()) {
    const FieldEntry entry = open.top();
    open.pop();
    if (entry.distance > distance.at(entry.x, entry.y)) continue;
    const double current_cost = cost.at(entry.x, entry.y);
    for (const Move& move : kMoves) {
      const int nx = entry.x + move.dx;
      const int ny = entry.y + move.dy;
      if (nx < 0 || nx >= cols || ny < 0 || ny >= rows || !drivable(nx, ny)) continue;
      if (move.dx != 0 && move.dy != 0 && (!drivable(nx, entry.y) || !drivable(entry.x, ny))) {
        continue;
      }
      const double next = entry.distance + move.length * 0.5 * (current_cost + cost.at(nx, ny));
      if (next < distance.at(nx, ny) - kRelaxEpsilon) {
        distance.at(nx, ny) = next;
        open.push(FieldEntry{next, nx, ny});
      }
    }
  }
  return distance;
}

}  // namespace depot
