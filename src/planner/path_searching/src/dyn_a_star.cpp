#include "path_searching/dyn_a_star.h"

AStar::AStar(GridMap::Ptr map) : grid_map_(std::move(map)) {}

bool AStar::search(Eigen::Vector3d start, Eigen::Vector3d goal, std::vector<Eigen::Vector3d> &path)
{
  path.clear();
  if (!start.allFinite() || !goal.allFinite() || (start - goal).squaredNorm() < 1e-12)
    return false;

  // Preserve the local planner's ability to find free guide endpoints just
  // beyond a colliding control-point segment.
  const Eigen::Vector3d step = (goal - start).normalized() * grid_map_->getResolution();
  auto moveOut = [&](Eigen::Vector3d &point, const Eigen::Vector3d &outward) {
    while (grid_map_->getInflateOccupancy(point) > 0)
      point += outward;
    return grid_map_->getInflateOccupancy(point) == 0;
  };

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(200);
  return moveOut(start, -step) && moveOut(goal, step) &&
      grid_map_->searchLocalPath(start, goal, path, deadline);
}
