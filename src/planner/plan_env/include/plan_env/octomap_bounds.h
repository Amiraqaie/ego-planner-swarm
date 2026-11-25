#ifndef PLAN_ENV_OCTOMAP_BOUNDS_H
#define PLAN_ENV_OCTOMAP_BOUNDS_H

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>

#include <Eigen/Core>
#include <octomap/OcTree.h>

namespace plan_env
{
struct OctomapBounds
{
  Eigen::Vector3d origin;
  Eigen::Vector3i cells;
};

// Include the complete leaf extents and an unknown border for frontier detection.
inline std::optional<OctomapBounds> octomapBounds(
    octomap::OcTree &tree, double resolution, double inflation)
{
  if (tree.size() == 0 || !std::isfinite(resolution) || resolution <= 0.0)
    return std::nullopt;

  Eigen::Vector3d lo, hi;
  tree.getMetricMin(lo.x(), lo.y(), lo.z());
  tree.getMetricMax(hi.x(), hi.y(), hi.z());

  const double margin = std::max(resolution, tree.getResolution()) + std::max(0.0, inflation);
  lo = ((lo.array() - margin) / resolution).floor();
  hi = ((hi.array() + margin) / resolution).ceil();
  const Eigen::Vector3d cells = hi - lo;
  if (!cells.allFinite() || (cells.array() < 1.0).any() ||
      cells.prod() > std::numeric_limits<int>::max())
    return std::nullopt;  // Grid addresses are signed ints.

  return OctomapBounds{lo * resolution, cells.cast<int>()};
}
}  // namespace plan_env

#endif
