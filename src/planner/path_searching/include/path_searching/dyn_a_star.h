#ifndef _DYN_A_STAR_H_
#define _DYN_A_STAR_H_

#include <plan_env/grid_map.h>

class AStar
{
public:
  using Ptr = std::shared_ptr<AStar>;

  explicit AStar(GridMap::Ptr map);
  bool search(Eigen::Vector3d start, Eigen::Vector3d goal, std::vector<Eigen::Vector3d> &path);

private:
  GridMap::Ptr grid_map_;
};

#endif
