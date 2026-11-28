#include "bspline_opt/bspline_optimizer.h"
#include "bspline_opt/gradient_descent_optimizer.h"
// using namespace std;

namespace ego_planner
{

  void BsplineOptimizer::setParam(rclcpp::Node::SharedPtr node)
  {

    node->declare_parameter("optimization/lambda_smooth", -1.0);
    node->declare_parameter("optimization/lambda_collision", -1.0);
    node->declare_parameter("optimization/lambda_feasibility", -1.0);
    node->declare_parameter("optimization/lambda_fitness", -1.0);

    node->declare_parameter("optimization/dist0", -1.0);
    node->declare_parameter("optimization/swarm_clearance", -1.0);
    node->declare_parameter("optimization/max_vel", -1.0);
    node->declare_parameter("optimization/max_acc", -1.0);

    node->declare_parameter("optimization/order", 3);
    collision_check_fraction_ = node->declare_parameter("planning/collision_check_fraction", 1.0);
    if (!std::isfinite(collision_check_fraction_) || collision_check_fraction_ <= 0.0 ||
        collision_check_fraction_ > 1.0)
      throw std::invalid_argument("planning/collision_check_fraction must be in (0, 1]");

    node->get_parameter("optimization/lambda_smooth", lambda1_);
    node->get_parameter("optimization/lambda_collision", lambda2_);
    node->get_parameter("optimization/lambda_feasibility", lambda3_);
    node->get_parameter("optimization/lambda_fitness", lambda4_);

    node->get_parameter("optimization/dist0", dist0_);
    node->get_parameter("optimization/swarm_clearance", swarm_clearance_);
    node->get_parameter("optimization/max_vel", max_vel_);
    node->get_parameter("optimization/max_acc", max_acc_);

    node->get_parameter("optimization/order", order_);
  }

  void BsplineOptimizer::setEnvironment(const GridMap::Ptr &map)
  {
    this->grid_map_ = map;
  }

  void BsplineOptimizer::setEnvironment(const GridMap::Ptr &map, const fast_planner::ObjPredictor::Ptr mov_obj)
  {
    this->grid_map_ = map;
    this->moving_objs_ = mov_obj;
  }

  void BsplineOptimizer::setControlPoints(const Eigen::MatrixXd &points)
  {
    cps_.points = points;
  }

  void BsplineOptimizer::setBsplineInterval(const double &ts) { bspline_interval_ = ts; }

  void BsplineOptimizer::setSwarmTrajs(SwarmTrajData *swarm_trajs_ptr) { swarm_trajs_ = swarm_trajs_ptr; }

  void BsplineOptimizer::setDroneId(const int drone_id) { drone_id_ = drone_id; }

  // 返回多个安全的控制点集
  std::vector<ControlPoints> BsplineOptimizer::distinctiveTrajs(vector<std::pair<int, int>> segments)
  {
    if (segments.size() == 0) // will be invoked again later.
    {
      std::vector<ControlPoints> oneSeg;
      oneSeg.push_back(cps_);
      return oneSeg;
    }

    constexpr int MAX_TRAJS = 8;                                                                            // 最多的轨迹数量
    constexpr int VARIS = 2;                                                                                // 允许的变化种类数
    int seg_upbound = std::min((int)segments.size(), static_cast<int>(floor(log(MAX_TRAJS) / log(VARIS)))); // 允许变换的片段数量上限
    std::vector<ControlPoints> control_pts_buf;
    control_pts_buf.reserve(MAX_TRAJS);
    const double RESOLUTION = grid_map_->getResolution();
    const double CTRL_PT_DIST = (cps_.points.col(0) - cps_.points.col(cps_.size - 1)).norm() / (cps_.size - 1); // 计算控制点间的平均距离

    // Step 1. Find the opposite vectors and base points for every segment.
    std::vector<std::pair<ControlPoints, ControlPoints>> RichInfoSegs;
    // 初始化两套控制点信息
    for (int i = 0; i < seg_upbound; i++)
    {
      std::pair<ControlPoints, ControlPoints> RichInfoOneSeg;
      ControlPoints RichInfoOneSeg_temp;
      // 获取指定片段的控制点信息
      cps_.segment(RichInfoOneSeg_temp, segments[i].first, segments[i].second);
      RichInfoOneSeg.first = RichInfoOneSeg_temp;
      RichInfoOneSeg.second = RichInfoOneSeg_temp;
      RichInfoSegs.push_back(RichInfoOneSeg);

      // cout << "RichInfoOneSeg_temp, out" << endl;
      // cout << "RichInfoSegs[" << i << "].first" << endl;
      // for ( int k=0; k<RichInfoOneSeg_temp.size; k++ )
      //   if ( RichInfoOneSeg_temp.base_point[k].size() > 0 )
      //   {
      //     cout << "###" << RichInfoOneSeg_temp.points.col(k).transpose() << endl;
      //     for (int k2 = 0; k2 < RichInfoOneSeg_temp.base_point[k].size(); k2++)
      //     {
      //       cout << "      " << RichInfoOneSeg_temp.base_point[k][k2].transpose() << " @ " << RichInfoOneSeg_temp.direction[k][k2].transpose() << endl;
      //     }
      //   }
    }

    for (int i = 0; i < seg_upbound; i++)
    {

      // 1.1 Find the start occupied point id and the last occupied point id
      if (RichInfoSegs[i].first.size > 1)
      {
        int occ_start_id = -1, occ_end_id = -1;
        Eigen::Vector3d occ_start_pt, occ_end_pt;
        for (int j = 0; j < RichInfoSegs[i].first.size - 1; j++)
        {
          // cout << "A *" << j << "*" << endl;
          //  遍历每个控制点及其后一个点，在两点间通过线性插值生成采样点
          double step_size = RESOLUTION / (RichInfoSegs[i].first.points.col(j) - RichInfoSegs[i].first.points.col(j + 1)).norm() / 2;
          for (double a = 1; a > 0; a -= step_size)
          {
            Eigen::Vector3d pt(a * RichInfoSegs[i].first.points.col(j) + (1 - a) * RichInfoSegs[i].first.points.col(j + 1));
            // cout << " " << grid_map_->getInflateOccupancy(pt) << " pt=" << pt.transpose() << endl;
            //  如果检测到在障碍物内，则存储对应数据
            if (grid_map_->getInflateOccupancy(pt))
            {
              occ_start_id = j;
              occ_start_pt = pt;
              goto exit_multi_loop1;
            }
          }
        }
      exit_multi_loop1:;
        // 查找片段与最后一个障碍物的交点
        for (int j = RichInfoSegs[i].first.size - 1; j >= 1; j--)
        {
          // cout << "j=" << j << endl;
          // cout << "B *" << j << "*" << endl;
          ;
          // 和上面同样的采样，然后检测是否在障碍物中
          double step_size = RESOLUTION / (RichInfoSegs[i].first.points.col(j) - RichInfoSegs[i].first.points.col(j - 1)).norm();
          for (double a = 1; a > 0; a -= step_size)
          {
            Eigen::Vector3d pt(a * RichInfoSegs[i].first.points.col(j) + (1 - a) * RichInfoSegs[i].first.points.col(j - 1));
            // cout << " " << grid_map_->getInflateOccupancy(pt) << " pt=" << pt.transpose() << endl;
            ;
            if (grid_map_->getInflateOccupancy(pt))
            {
              occ_end_id = j;
              occ_end_pt = pt;
              goto exit_multi_loop2;
            }
          }
        }
      exit_multi_loop2:;

        // double check
        // 如果片段的起点或者终点在障碍物中，将会被移除
        if (occ_start_id == -1 || occ_end_id == -1)
        {
          // It means that the first or the last control points of one segment are in obstacles, which is not allowed.
          // ROS_WARN("What? occ_start_id=%d, occ_end_id=%d", occ_start_id, occ_end_id);

          segments.erase(segments.begin() + i);
          RichInfoSegs.erase(RichInfoSegs.begin() + i);
          seg_upbound--;
          i--;

          continue;

          // cout << "RichInfoSegs[" << i << "].first" << endl;
          // for (int k = 0; k < RichInfoSegs[i].first.size; k++)
          // {
          //   if (RichInfoSegs[i].first.base_point.size() > 0)
          //   {
          //     cout << "###" << RichInfoSegs[i].first.points.col(k).transpose() << endl;
          //     for (int k2 = 0; k2 < RichInfoSegs[i].first.base_point[k].size(); k2++)
          //     {
          //       cout << "      " << RichInfoSegs[i].first.base_point[k][k2].transpose() << " @ " << RichInfoSegs[i].first.direction[k][k2].transpose() << endl;
          //     }
          //   }
          // }
        }

        // 1.2 Reverse the vector and find new base points from occ_start_id to occ_end_id.
        for (int j = occ_start_id; j <= occ_end_id; j++)
        {
          Eigen::Vector3d base_pt_reverse, base_vec_reverse;
          // 检查控制点的base point是否为1
          if (RichInfoSegs[i].first.base_point[j].size() != 1)
          {
            cout << "RichInfoSegs[" << i << "].first.base_point[" << j << "].size()=" << RichInfoSegs[i].first.base_point[j].size() << endl;
            RCLCPP_ERROR(rclcpp::get_logger("distinctiveTrajs"), "Wrong number of base_points!!! Should not be happen!.");

            cout << setprecision(5);
            cout << "cps_" << endl;
            cout << " clearance=" << cps_.clearance << " cps.size=" << cps_.size << endl;
            // 输出错误信息
            for (int temp_i = 0; temp_i < cps_.size; temp_i++)
            {
              if (cps_.base_point[temp_i].size() > 1 && cps_.base_point[temp_i].size() < 1000)
              {
                RCLCPP_ERROR(rclcpp::get_logger("distinctiveTrajs"), "Should not happen!!!");
                cout << "######" << cps_.points.col(temp_i).transpose() << endl;
                for (size_t temp_j = 0; temp_j < cps_.base_point[temp_i].size(); temp_j++)
                  cout << "      " << cps_.base_point[temp_i][temp_j].transpose() << " @ " << cps_.direction[temp_i][temp_j].transpose() << endl;
              }
            }

            std::vector<ControlPoints> blank;
            return blank;
          }

          // 通过取反获得相反方向的向量
          base_vec_reverse = -RichInfoSegs[i].first.direction[j][0];

          // The start and the end case must get taken special care of.
          // 若当前控制点为片段的起始点 occ_start_id，则将障碍物交点 occ_start_pt 直接设为 base_pt_reverse
          if (j == occ_start_id)
          {
            base_pt_reverse = occ_start_pt;
          }
          // 若当前控制点为片段的终止点 occ_end_id，则将终点交点 occ_end_pt 设为 base_pt_reverse
          else if (j == occ_end_id)
          {
            base_pt_reverse = occ_end_pt;
          }
          // 对于片段中的中间控制点，将基准点 base_pt_reverse 设置为当前控制点 points.col(j) 沿反向向量 base_vec_reverse 方向延伸的某一距离位置
          else
          {
            base_pt_reverse = RichInfoSegs[i].first.points.col(j) + base_vec_reverse * (RichInfoSegs[i].first.base_point[j][0] - RichInfoSegs[i].first.points.col(j)).norm();
          }

          // 检查base_pt_reverse是否在障碍物中
          if (grid_map_->getInflateOccupancy(base_pt_reverse)) // Search outward.
          {
            // 最大搜索范围
            double l_upbound = 5 * CTRL_PT_DIST; // "5" is the threshold.
            double l = RESOLUTION;
            for (; l <= l_upbound; l += RESOLUTION)
            {
              // 不断将控制点向外移动，寻找不在障碍物中的控制点
              Eigen::Vector3d base_pt_temp = base_pt_reverse + l * base_vec_reverse;
              // cout << base_pt_temp.transpose() << endl;
              if (!grid_map_->getInflateOccupancy(base_pt_temp))
              {
                RichInfoSegs[i].second.base_point[j][0] = base_pt_temp;
                RichInfoSegs[i].second.direction[j][0] = base_vec_reverse;
                break;
              }
            }
            // 如果找不到则删除这一段
            if (l > l_upbound)
            {
              RCLCPP_WARN(rclcpp::get_logger("distinctiveTrajs"), "Can't find the new base points at the opposite within the threshold. i=%d, j=%d", i, j);

              segments.erase(segments.begin() + i);
              RichInfoSegs.erase(RichInfoSegs.begin() + i);
              seg_upbound--;
              i--;

              goto exit_multi_loop3; // break "for (int j = 0; j < RichInfoSegs[i].first.size; j++)"
            }
          }
          // 如果距离控制点足够远且不再障碍物中则无需继续搜索
          else if ((base_pt_reverse - RichInfoSegs[i].first.points.col(j)).norm() >= RESOLUTION) // Unnecessary to search.
          {
            RichInfoSegs[i].second.base_point[j][0] = base_pt_reverse;
            RichInfoSegs[i].second.direction[j][0] = base_vec_reverse;
          }
          // 基点和控制点太近则删除这一段
          else
          {
            RCLCPP_WARN(rclcpp::get_logger("distinctiveTrajs"), "base_point and control point are too close!");
            cout << "base_point=" << RichInfoSegs[i].first.base_point[j][0].transpose() << " control point=" << RichInfoSegs[i].first.points.col(j).transpose() << endl;

            segments.erase(segments.begin() + i);
            RichInfoSegs.erase(RichInfoSegs.begin() + i);
            seg_upbound--;
            i--;

            goto exit_multi_loop3; // break "for (int j = 0; j < RichInfoSegs[i].first.size; j++)"
          }
        }

        // 1.3 Assign the base points to control points within [0, occ_start_id) and (occ_end_id, RichInfoSegs[i].first.size()-1].
        if (RichInfoSegs[i].second.size)
        {
          // 为片段起点之前和终点之后的控制点设置统一的基准点和方向，使得这些控制点在障碍物影响范围外时能够保持一致的路径属性
          for (int j = occ_start_id - 1; j >= 0; j--)
          {
            RichInfoSegs[i].second.base_point[j][0] = RichInfoSegs[i].second.base_point[occ_start_id][0];
            RichInfoSegs[i].second.direction[j][0] = RichInfoSegs[i].second.direction[occ_start_id][0];
          }
          for (int j = occ_end_id + 1; j < RichInfoSegs[i].second.size; j++)
          {
            RichInfoSegs[i].second.base_point[j][0] = RichInfoSegs[i].second.base_point[occ_end_id][0];
            RichInfoSegs[i].second.direction[j][0] = RichInfoSegs[i].second.direction[occ_end_id][0];
          }
        }

      exit_multi_loop3:;
      }
      // 片段只有一个控制点的情况
      else if (RichInfoSegs[i].first.size == 1)
      {
        cout << "i=" << i << " RichInfoSegs.size()=" << RichInfoSegs.size() << endl;
        cout << "RichInfoSegs[i].first.size=" << RichInfoSegs[i].first.size << endl;
        cout << "RichInfoSegs[i].first.direction.size()=" << RichInfoSegs[i].first.direction.size() << endl;
        cout << "RichInfoSegs[i].first.direction[0].size()=" << RichInfoSegs[i].first.direction[0].size() << endl;
        cout << "RichInfoSegs[i].first.points.cols()=" << RichInfoSegs[i].first.points.cols() << endl;
        cout << "RichInfoSegs[i].first.base_point.size()=" << RichInfoSegs[i].first.base_point.size() << endl;
        cout << "RichInfoSegs[i].first.base_point[0].size()=" << RichInfoSegs[i].first.base_point[0].size() << endl;
        Eigen::Vector3d base_vec_reverse = -RichInfoSegs[i].first.direction[0][0];
        Eigen::Vector3d base_pt_reverse = RichInfoSegs[i].first.points.col(0) + base_vec_reverse * (RichInfoSegs[i].first.base_point[0][0] - RichInfoSegs[i].first.points.col(0)).norm();

        if (grid_map_->getInflateOccupancy(base_pt_reverse)) // Search outward.
        {
          double l_upbound = 5 * CTRL_PT_DIST; // "5" is the threshold.
          double l = RESOLUTION;
          for (; l <= l_upbound; l += RESOLUTION)
          {
            Eigen::Vector3d base_pt_temp = base_pt_reverse + l * base_vec_reverse;
            // cout << base_pt_temp.transpose() << endl;
            if (!grid_map_->getInflateOccupancy(base_pt_temp))
            {
              RichInfoSegs[i].second.base_point[0][0] = base_pt_temp;
              RichInfoSegs[i].second.direction[0][0] = base_vec_reverse;
              break;
            }
          }
          if (l > l_upbound)
          {
            RCLCPP_WARN(rclcpp::get_logger("distinctiveTrajs"), 
                        "Can't find the new base points at the opposite within the threshold, 2. i=%d", i);

            segments.erase(segments.begin() + i);
            RichInfoSegs.erase(RichInfoSegs.begin() + i);
            seg_upbound--;
            i--;
          }
        }
        else if ((base_pt_reverse - RichInfoSegs[i].first.points.col(0)).norm() >= RESOLUTION) // Unnecessary to search.
        {
          RichInfoSegs[i].second.base_point[0][0] = base_pt_reverse;
          RichInfoSegs[i].second.direction[0][0] = base_vec_reverse;
        }
        else
        {
          RCLCPP_WARN(rclcpp::get_logger("distinctiveTrajs"), 
                        "base_point and control point are too close!, 2");
          cout << "base_point=" << RichInfoSegs[i].first.base_point[0][0].transpose() << " control point=" << RichInfoSegs[i].first.points.col(0).transpose() << endl;

          segments.erase(segments.begin() + i);
          RichInfoSegs.erase(RichInfoSegs.begin() + i);
          seg_upbound--;
          i--;
        }
      }
      else
      {
        segments.erase(segments.begin() + i);
        RichInfoSegs.erase(RichInfoSegs.begin() + i);
        seg_upbound--;
        i--;
      }
    }
    // cout << "A3" << endl;

    // Step 2. Assemble each segment to make up the new control point sequence.
    // 将每个分段组合起来，组成新的控制点序列
    if (seg_upbound == 0) // After the erase operation above, segment legth will decrease to 0 again.
    {
      std::vector<ControlPoints> oneSeg;
      oneSeg.push_back(cps_);
      return oneSeg;
    }

    // 初始化选择向量
    std::vector<int> selection(seg_upbound);
    std::fill(selection.begin(), selection.end(), 0);
    selection[0] = -1; // init
    // 计算最大组合数
    int max_traj_nums = static_cast<int>(pow(VARIS, seg_upbound));
    for (int i = 0; i < max_traj_nums; i++)
    {
      // 2.1 Calculate the selection table.
      int digit_id = 0;
      selection[digit_id]++;
      // 生成一个选择表
      while (digit_id < seg_upbound && selection[digit_id] >= VARIS)
      {
        selection[digit_id] = 0;
        digit_id++;
        if (digit_id >= seg_upbound)
        {
          RCLCPP_ERROR(rclcpp::get_logger("distinctiveTrajs"), 
                        "Should not happen!!! digit_id=%d, seg_upbound=%d", digit_id, seg_upbound);
          
        }
        selection[digit_id]++;
      }

      // 2.2 Assign params according to the selection table.
      ControlPoints cpsOneSample;
      cpsOneSample.resize(cps_.size);
      cpsOneSample.clearance = cps_.clearance;
      int cp_id = 0, seg_id = 0, cp_of_seg_id = 0;
      // 遍历所有控制点
      while (/*seg_id < RichInfoSegs.size() ||*/ cp_id < cps_.size)
      {
        // cout << "A ";
        //  if ( seg_id >= RichInfoSegs.size() )
        //  {
        //    cout << "seg_id=" << seg_id << " RichInfoSegs.size()=" << RichInfoSegs.size() << endl;
        //  }
        //  if ( cp_id >= cps_.base_point.size() )
        //  {
        //    cout << "cp_id=" << cp_id << " cps_.base_point.size()=" << cps_.base_point.size() << endl;
        //  }
        //  if ( cp_of_seg_id >= RichInfoSegs[seg_id].first.base_point.size() )
        //  {
        //    cout << "cp_of_seg_id=" << cp_of_seg_id << " RichInfoSegs[seg_id].first.base_point.size()=" << RichInfoSegs[seg_id].first.base_point.size() << endl;
        //  }
        //  判断控制点是否在当前控制范围内
        //  如果不在则直接从原始控制点集中复制数据
        if (seg_id >= seg_upbound || cp_id < segments[seg_id].first || cp_id > segments[seg_id].second)
        {
          cpsOneSample.points.col(cp_id) = cps_.points.col(cp_id);
          cpsOneSample.base_point[cp_id] = cps_.base_point[cp_id];
          cpsOneSample.direction[cp_id] = cps_.direction[cp_id];
        }
        // 如果 cp_id 位于当前片段范围内，根据 selection[seg_id] 的值选择片段的第一套或第二套基准点和方向
        else if (cp_id >= segments[seg_id].first && cp_id <= segments[seg_id].second)
        {
          if (!selection[seg_id]) // zx-todo
          {
            cpsOneSample.points.col(cp_id) = RichInfoSegs[seg_id].first.points.col(cp_of_seg_id);
            cpsOneSample.base_point[cp_id] = RichInfoSegs[seg_id].first.base_point[cp_of_seg_id];
            cpsOneSample.direction[cp_id] = RichInfoSegs[seg_id].first.direction[cp_of_seg_id];
            cp_of_seg_id++;
          }
          else
          {
            if (RichInfoSegs[seg_id].second.size)
            {
              cpsOneSample.points.col(cp_id) = RichInfoSegs[seg_id].second.points.col(cp_of_seg_id);
              cpsOneSample.base_point[cp_id] = RichInfoSegs[seg_id].second.base_point[cp_of_seg_id];
              cpsOneSample.direction[cp_id] = RichInfoSegs[seg_id].second.direction[cp_of_seg_id];
              cp_of_seg_id++;
            }
            else
            {
              // Abandon this trajectory.
              goto abandon_this_trajectory;
            }
          }

          // 当遍历到片段的最后一个控制点时，将 cp_of_seg_id 重置为 0，并将 seg_id 指向下一个片段
          if (cp_id == segments[seg_id].second)
          {
            cp_of_seg_id = 0;
            seg_id++;
          }
        }
        else
        {
          RCLCPP_ERROR(rclcpp::get_logger("distinctiveTrajs"), 
                    "Shold not happen!!!!, cp_id=%d, seg_id=%d, segments.front().first=%d, segments.back().second=%d, segments[seg_id].first=%d, segments[seg_id].second=%d",
                    cp_id, seg_id, segments.front().first, segments.back().second, segments[seg_id].first, segments[seg_id].second);
        }

        cp_id++;
      }

      control_pts_buf.push_back(cpsOneSample);

    abandon_this_trajectory:;
    }

    return control_pts_buf;
  } // namespace ego_planner

  /* This function is very similar to check_collision_and_rebound().
   * It was written separately, just because I did it once and it has been running stably since March 2020.
   * But I will merge then someday.*/
  // 初始化控制点
  void BsplineOptimizer::beginReplan(const GlobalTrajData &reference, double interval)
  {
    reference_ = reference.hasGridRoute() ? &reference : nullptr;
    diagnostics_ = ReplanDiagnostics{};
    bspline_interval_ = interval;
  }

  vector<Eigen::Vector3d> BsplineOptimizer::referenceGuide(const Eigen::MatrixXd &points, int first, int last)
  {
    const auto &raw = reference_->raw_route_;
    const auto &times = reference_->raw_times_;
    const double start = reference_->local_reference_start_;
    const double duration = reference_->local_reference_end_ - start;
    // A cubic control point is centered roughly one knot before its index.
    const double intervals = points.cols() - 3;
    const auto &knot_times = reference_->local_knot_reference_times_;
    const auto referenceTime = [&](int control) {
      const double progress = std::clamp((control - 1.0) / intervals, 0.0, 1.0);
      if (knot_times.empty())
        return start + duration * progress;
      const double index = progress * (knot_times.size() - 1);
      const size_t i = static_cast<size_t>(index);
      return knot_times[i] + (index - i) * (knot_times[std::min(i + 1, knot_times.size() - 1)] - knot_times[i]);
    };
    const double a = referenceTime(first);
    const double b = referenceTime(last);
    const double middle = (a + b) * 0.5;
    const double padding = std::min(1.0, duration * 0.25);
    auto edgeClear = [&](const Eigen::Vector3d &from, const Eigen::Vector3d &to) {
      return std::isfinite(grid_map_->segmentCost(from, to, true));
    };

    auto connect = [&](const Eigen::Vector3d &point, double low, double high) {
      size_t best = raw.size();
      double distance = 1.5; // A distant route is not a local collision guide.
      size_t i = std::lower_bound(times.begin(), times.end(), low) - times.begin();
      for (; i < raw.size() && times[i] <= high; ++i)
      {
        const double candidate = (point - raw[i]).norm();
        if (candidate < distance && edgeClear(point, raw[i]))
        {
          distance = candidate;
          best = i;
        }
      }
      return best;
    };

    const size_t from = connect(points.col(first), a - padding, middle);
    const size_t to = connect(points.col(last), middle, b + padding);
    if (from >= to || to >= raw.size())
      return {};
    for (size_t i = from; i < to; ++i)
      if (!edgeClear(raw[i], raw[i + 1]))
        return {};

    vector<Eigen::Vector3d> guide{points.col(first)};
    guide.insert(guide.end(), raw.begin() + from, raw.begin() + to + 1);
    guide.push_back(points.col(last));
    return guide;
  }

  bool BsplineOptimizer::addGuideAnchors(const Eigen::MatrixXd &points, int first, int last,
                                         int lower, int upper, const vector<Eigen::Vector3d> &guide)
  {
    int anchor = -1;
    for (int j = lower; j <= upper; ++j)
      cps_.flag_temp[j] = false;
    const bool single_edge = last - first == 1;
    const double resolution = grid_map_->getResolution();
    for (int j = std::max(lower, single_edge ? first : first + 1); j < last && j <= upper; ++j)
    {
      const Eigen::Vector3d center = single_edge ? (points.col(first) + points.col(last)) / 2
                                                : Eigen::Vector3d(points.col(j));
      const Eigen::Vector3d tangent = points.col(single_edge ? last : j + 1) -
                                      points.col(single_edge ? first : j - 1);
      // Intersect the guide with the control-point plane used by rebound.
      // Scan edges explicitly so guides with turns cannot index past an end.
      Eigen::Vector3d intersection;
      double length = std::numeric_limits<double>::infinity();
      for (size_t k = 1; k < guide.size(); ++k)
      {
        const double a = (guide[k - 1] - center).dot(tangent);
        const double b = (guide[k] - center).dot(tangent);
        if ((a > 0 && b > 0) || (a < 0 && b < 0) || std::abs(a - b) < 1e-10)
          continue;
        const Eigen::Vector3d point = guide[k - 1] + a / (a - b) * (guide[k] - guide[k - 1]);
        const double distance = (point - center).norm();
        if (distance > (single_edge ? 0.01 : 1e-5) && distance < length &&
            grid_map_->getInflateOccupancy(point) == 0)
        {
          intersection = point;
          length = distance;
        }
      }
      if (!std::isfinite(length))
        continue;

      const Eigen::Vector3d direction = (intersection - center) / length;
      double offset = 0.0;
      for (double d = length; !single_edge && d >= 0.0; d -= resolution)
      {
        const bool occupied = grid_map_->getInflateOccupancy(center + d * direction) != 0;
        if (occupied || d < resolution)
        {
          offset = occupied ? std::min(length, d + resolution) : d;
          break;
        }
      }
      cps_.base_point[j].push_back(points.col(j) + offset * direction);
      cps_.direction[j].push_back(direction);
      cps_.flag_temp[j] = true;
      anchor = j;
    }
    if (anchor < 0)
      return false;

    for (int direction : {-1, 1})
      for (int j = anchor + direction; j >= lower && j <= upper; j += direction)
        if (!cps_.flag_temp[j])
        {
          cps_.base_point[j].push_back(cps_.base_point[j - direction].back());
          cps_.direction[j].push_back(cps_.direction[j - direction].back());
        }
    return true;
  }

  bool BsplineOptimizer::assignCollisionGuide(const Eigen::MatrixXd &points, int first, int last,
                                              int lower, int upper)
  {
    if (last <= first || lower > upper)
      return false;
    if (reference_)
    {
      ++diagnostics_.reuse_attempts;
      auto guide = referenceGuide(points, first, last);
      if (!guide.empty() && addGuideAnchors(points, first, last, lower, upper, guide))
      {
        ++diagnostics_.reuse_successes;
        diagnostics_.reused_guides.push_back(std::move(guide));
        return true;
      }
    }

    ++diagnostics_.local_attempts;
    const auto start = std::chrono::steady_clock::now();
    vector<Eigen::Vector3d> guide;
    const bool found = a_star_->search(points.col(first), points.col(last), guide);
    diagnostics_.local_search_ms += std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start).count();
    if (!found)
    {
      ++diagnostics_.local_failures;
      return false;
    }

    ++diagnostics_.local_successes;
    const bool assigned = addGuideAnchors(points, first, last, lower, upper, guide);
    diagnostics_.local_guides.push_back(std::move(guide));
    return assigned;
  }

  void BsplineOptimizer::appendSplineCollisions(const Eigen::MatrixXd &points,
                                                vector<std::pair<int, int>> &segments, bool initial)
  {
    UniformBspline spline(points, order_, bspline_interval_);
    const double step = collisionSampleStep(spline);
    const double end = spline.getTimeSum();
    int first_hit = -1, last_hit = -1;
    bool collided = false;
    auto append = [&] {
      if (first_hit < 0)
        return;
      int first = std::max(0, first_hit - 1);
      int last = std::min<int>(points.cols() - 1, last_hit + 2);
      while (first > 0 && grid_map_->getInflateOccupancy(points.col(first)) != 0)
        --first;
      while (last + 1 < points.cols() && grid_map_->getInflateOccupancy(points.col(last)) != 0)
        ++last;
      segments.emplace_back(first, last);
      first_hit = -1;
    };
    if (step > 0.0 && std::isfinite(end))
      for (double t = 0.0;; t = std::min(t + step, end))
      {
        const Eigen::Vector3d point = spline.evaluateDeBoorT(t);
        if (!point.allFinite() || grid_map_->getInflateOccupancy(point) != 0)
        {
          const int control = static_cast<int>(t / bspline_interval_) + 1;
          if (first_hit < 0)
            first_hit = control;
          last_hit = control;
          collided = true;
        }
        else
          append();
        if (t >= end)
          break;
      }
    append();
    if (initial && collided)
      ++diagnostics_.initial_collisions;
  }

  std::vector<std::pair<int, int>> BsplineOptimizer::initControlPoints(Eigen::MatrixXd &init_points, bool flag_first_init /*= true*/)
  {

    if (flag_first_init)
    {
      cps_.clearance = dist0_;
      cps_.resize(init_points.cols());
      cps_.points = init_points;
    }

    /*** Segment the initial trajectory according to obstacles ***/
    // 进入或离开障碍物稳定的时间间隔
    constexpr int ENOUGH_INTERVAL = 2;
    int in_id = -1, out_id = -1;
    vector<std::pair<int, int>> segment_ids;
    int same_occ_state_times = ENOUGH_INTERVAL + 1;
    bool occ, last_occ = false;
    // 标识片段的起点和终点是否找到
    bool flag_got_start = false, flag_got_end = false, flag_got_end_maybe = false;
    const int i_end = collisionCheckEndIndex(init_points.cols() - order_);
    // The reference-seeded path uses collisions on the continuous spline.
    // Keep the legacy control polygon check for goals without a grid route.
    for (int i = order_; !reference_ && i <= i_end; ++i)
    {
      // cout << " *" << i-1 << "*" ;
      //  相邻两个点之间进行线性插值并检测障碍物
      const double length = (init_points.col(i) - init_points.col(i - 1)).norm();
      const int samples = std::max(1, static_cast<int>(std::ceil(2 * length / grid_map_->getResolution())));
      for (int j = 0; j < samples + (i == i_end); ++j)
      {
        const double a = 1.0 - static_cast<double>(j) / samples;
        // TODO:没搞懂这是干嘛的
        occ = grid_map_->getInflateOccupancy(a * init_points.col(i - 1) + (1 - a) * init_points.col(i));
        // cout << " " << occ;
        //  cout << setprecision(5);
        //  cout << (a * init_points.col(i-1) + (1-a) * init_points.col(i)).transpose() << " occ1=" << occ << endl;

        // 进入障碍物
        if (occ && !last_occ)
        {
          if (same_occ_state_times > ENOUGH_INTERVAL || i == order_)
          {
            in_id = i - 1;
            flag_got_start = true;
          }
          same_occ_state_times = 0;
          flag_got_end_maybe = false; // terminate in advance
        }
        // 离开障碍物
        else if (!occ && last_occ)
        {
          out_id = i;
          flag_got_end_maybe = true;
          same_occ_state_times = 0;
        }
        // 如果状态没发生变化
        else
        {
          ++same_occ_state_times;
        }

        // 如果已经离开障碍物，则结束
        if (flag_got_end_maybe && (same_occ_state_times > ENOUGH_INTERVAL || (i == i_end)))
        {
          flag_got_end_maybe = false;
          flag_got_end = true;
        }

        last_occ = occ;

        // 重置标志位并存储信息
        if (flag_got_start && flag_got_end)
        {
          flag_got_start = false;
          flag_got_end = false;
          segment_ids.push_back(std::pair<int, int>(in_id, out_id));
        }
      }
    }
    // cout << endl;

    // for (size_t i = 0; i < segment_ids.size(); i++)
    // {
    //   cout << "segment_ids=" << segment_ids[i].first << " ~ " << segment_ids[i].second << endl;
    // }

    if (flag_first_init || reference_)
    {
      vector<std::pair<int, int>> spline_segments;
      appendSplineCollisions(init_points, spline_segments, flag_first_init);
      segment_ids.insert(segment_ids.end(), spline_segments.begin(), spline_segments.end());
    }
    std::sort(segment_ids.begin(), segment_ids.end());
    size_t count = 0;
    for (const auto segment : segment_ids)
    {
      if (count && segment.first <= segment_ids[count - 1].second)
        segment_ids[count - 1].second = std::max(segment_ids[count - 1].second, segment.second);
      else
        segment_ids[count++] = segment;
    }
    segment_ids.resize(count);
    if (segment_ids.empty())
      return {};

    /*** calculate bounds ***/
    int id_low_bound, id_up_bound;
    vector<std::pair<int, int>> bounds(segment_ids.size());
    // 遍历每一段，设置边界
    for (size_t i = 0; i < segment_ids.size(); i++)
    {

      if (i == 0) // first segment
      {
        id_low_bound = order_;
        if (segment_ids.size() > 1)
        {
          // 取当前片段结束点 segment_ids[0].second 和下一个片段起始点 segment_ids[1].first 的中间位置，向下取整后作为高边界
          id_up_bound = (int)(((segment_ids[0].second + segment_ids[1].first) - 1.0f) / 2); // id_up_bound : -1.0f fix()
        }
        else
        {
          // 距末尾 order_ + 1 个点的位置
          id_up_bound = init_points.cols() - order_ - 1;
        }
      }
      // 末尾的边界
      else if (i == segment_ids.size() - 1) // last segment, i != 0 here
      {
        // 尾段的低边界为当前片段起点和上一个片段终点的中间位置，向上取整
        id_low_bound = (int)(((segment_ids[i].first + segment_ids[i - 1].second) + 1.0f) / 2); // id_low_bound : +1.0f ceil()
        // 距末尾点 order_ + 1 个点
        id_up_bound = init_points.cols() - order_ - 1;
      }
      else
      {
        // 低边界：为当前片段起点和前一片段终点的中间位置，向上取整
        id_low_bound = (int)(((segment_ids[i].first + segment_ids[i - 1].second) + 1.0f) / 2); // id_low_bound : +1.0f ceil()
        // 高边界：为当前片段终点和下一片段起点的中间位置，向下取整
        id_up_bound = (int)(((segment_ids[i].second + segment_ids[i + 1].first) - 1.0f) / 2); // id_up_bound : -1.0f fix()
      }

      bounds[i] = std::pair<int, int>(id_low_bound, id_up_bound);
    }

    // cout << "+++++++++" << endl;
    // for ( int j=0; j<bounds.size(); ++j )
    // {
    //   cout << bounds[j].first << "  " << bounds[j].second << endl;
    // }

    /*** Adjust segment length ***/
    vector<std::pair<int, int>> adjusted_segment_ids(segment_ids.size());
    // 控制点的最小比例
    constexpr double MINIMUM_PERCENT = 0.0; // Each segment is guaranteed to have sufficient points to generate sufficient force
    // 控制点的最小数量
    int minimum_points = round(init_points.cols() * MINIMUM_PERCENT), num_points;
    for (size_t i = 0; i < segment_ids.size(); i++)
    {
      /*** Adjust segment length ***/
      // 获取当前片段的点数
      num_points = segment_ids[i].second - segment_ids[i].first + 1;
      // cout << "i = " << i << " first = " << segment_ids[i].first << " second = " << segment_ids[i].second << endl;
      if (num_points < minimum_points)
      {
        // 如果点数不够则在两侧扩展点，确保不能超过边界
        double add_points_each_side = (int)(((minimum_points - num_points) + 1.0f) / 2);

        adjusted_segment_ids[i].first = segment_ids[i].first - add_points_each_side >= bounds[i].first ? segment_ids[i].first - add_points_each_side : bounds[i].first;

        adjusted_segment_ids[i].second = segment_ids[i].second + add_points_each_side <= bounds[i].second ? segment_ids[i].second + add_points_each_side : bounds[i].second;
      }
      else
      {
        adjusted_segment_ids[i].first = segment_ids[i].first;
        adjusted_segment_ids[i].second = segment_ids[i].second;
      }

      // cout << "final:" << "i = " << i << " first = " << adjusted_segment_ids[i].first << " second = " << adjusted_segment_ids[i].second << endl;
    }
    // 避免重叠
    for (size_t i = 1; i < adjusted_segment_ids.size(); i++) // Avoid overlap
    {
      if (adjusted_segment_ids[i - 1].second >= adjusted_segment_ids[i].first)
      {
        double middle = (double)(adjusted_segment_ids[i - 1].second + adjusted_segment_ids[i].first) / 2.0;
        adjusted_segment_ids[i - 1].second = static_cast<int>(middle - 0.1);
        adjusted_segment_ids[i].first = static_cast<int>(middle + 1.1);
      }
    }

    // Used for return
    vector<std::pair<int, int>> final_segment_ids;

    for (size_t i = 0; i < segment_ids.size(); ++i)
    {
      const auto [first, last] = segment_ids[i];
      const auto [lower, upper] = adjusted_segment_ids[i];
      if (assignCollisionGuide(init_points, first, last, lower, upper))
        final_segment_ids.push_back(adjusted_segment_ids[i]);
    }

    return final_segment_ids;
  }

  // 急停情况下提前退出
  int BsplineOptimizer::earlyExit(void *func_data, const double *x, const double *g, const double fx, const double xnorm, const double gnorm, const double step, int n, int k, int ls)
  {
    BsplineOptimizer *opt = reinterpret_cast<BsplineOptimizer *>(func_data);
    // cout << "k=" << k << endl;
    // cout << "opt->flag_continue_to_optimize_=" << opt->flag_continue_to_optimize_ << endl;
    return (opt->force_stop_type_ == STOP_FOR_ERROR || opt->force_stop_type_ == STOP_FOR_REBOUND);
  }

  // 利用combineCostRebound计算损失
  double BsplineOptimizer::costFunctionRebound(void *func_data, const double *x, double *grad, const int n)
  {
    BsplineOptimizer *opt = reinterpret_cast<BsplineOptimizer *>(func_data);

    double cost;
    opt->combineCostRebound(x, grad, cost, n);

    opt->iter_num_ += 1;
    return cost;
  }

  // 利用combineCostRefine计算优化后的损失
  double BsplineOptimizer::costFunctionRefine(void *func_data, const double *x, double *grad, const int n)
  {
    BsplineOptimizer *opt = reinterpret_cast<BsplineOptimizer *>(func_data);

    double cost;
    opt->combineCostRefine(x, grad, cost, n);

    opt->iter_num_ += 1;
    return cost;
  }

  // 几个计算损失的函数
  void BsplineOptimizer::calcSwarmCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient)
  {
    cost = 0.0;
    const int end_idx = collisionCheckEndIndex(q.cols() - order_);
    const double CLEARANCE = swarm_clearance_ * 2;
    double t_now = rclcpp::Clock().now().seconds();
    constexpr double a = 2.0, b = 1.0, inv_a2 = 1 / a / a, inv_b2 = 1 / b / b;

    for (int i = order_; i < end_idx; i++)
    {
      double glb_time = t_now + ((double)(order_ - 1) / 2 + (i - order_ + 1)) * bspline_interval_;

      for (size_t id = 0; id < swarm_trajs_->size(); id++)
      {
        if ((swarm_trajs_->at(id).drone_id != (int)id) || swarm_trajs_->at(id).drone_id == drone_id_)
        {
          continue;
        }

        double traj_i_satrt_time = swarm_trajs_->at(id).start_time_.seconds();
        if (glb_time < traj_i_satrt_time + swarm_trajs_->at(id).duration_ - 0.1)
        {
          /* def cost=(c-sqrt([Q-O]'D[Q-O]))^2, D=[1/b^2,0,0;0,1/b^2,0;0,0,1/a^2] */
          Eigen::Vector3d swarm_prid = swarm_trajs_->at(id).position_traj_.evaluateDeBoorT(glb_time - traj_i_satrt_time);
          Eigen::Vector3d dist_vec = cps_.points.col(i) - swarm_prid;
          double ellip_dist = sqrt(dist_vec(2) * dist_vec(2) * inv_a2 + (dist_vec(0) * dist_vec(0) + dist_vec(1) * dist_vec(1)) * inv_b2);
          double dist_err = CLEARANCE - ellip_dist;

          Eigen::Vector3d dist_grad = cps_.points.col(i) - swarm_prid;
          Eigen::Vector3d Coeff;
          Coeff(0) = -2 * (CLEARANCE / ellip_dist - 1) * inv_b2;
          Coeff(1) = Coeff(0);
          Coeff(2) = -2 * (CLEARANCE / ellip_dist - 1) * inv_a2;

          if (dist_err < 0)
          {
            /* do nothing */
          }
          else
          {
            cost += pow(dist_err, 2);
            gradient.col(i) += (Coeff.array() * dist_grad.array()).matrix();
          }

          if (min_ellip_dist_ > dist_err)
          {
            min_ellip_dist_ = dist_err;
          }
        }
      }
    }
  }

  void BsplineOptimizer::calcMovingObjCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient)
  {
    cost = 0.0;
    int end_idx = q.cols() - order_;
    constexpr double CLEARANCE = 1.5;
    double t_now = rclcpp::Clock().now().seconds();

    for (int i = order_; i < end_idx; i++)
    {
      double time = ((double)(order_ - 1) / 2 + (i - order_ + 1)) * bspline_interval_;

      for (int id = 0; id < moving_objs_->getObjNums(); id++)
      {
        Eigen::Vector3d obj_prid = moving_objs_->evaluateConstVel(id, t_now + time);
        double dist = (cps_.points.col(i) - obj_prid).norm();
        // cout /*<< "cps_.points.col(i)=" << cps_.points.col(i).transpose()*/ << " moving_objs_=" << obj_prid.transpose() << " dist=" << dist << endl;
        double dist_err = CLEARANCE - dist;
        Eigen::Vector3d dist_grad = (cps_.points.col(i) - obj_prid).normalized();

        if (dist_err < 0)
        {
          /* do nothing */
        }
        else
        {
          cost += pow(dist_err, 2);
          gradient.col(i) += -2.0 * dist_err * dist_grad;
        }
      }
      // cout << "time=" << time << " i=" << i << " order_=" << order_ << " end_idx=" << end_idx << endl;
      // cout << "--" << endl;
    }
    // cout << "---------------" << endl;
  }

  void BsplineOptimizer::calcDistanceCostRebound(const Eigen::MatrixXd &q, double &cost,
                                                 Eigen::MatrixXd &gradient, int iter_num, double smoothness_cost)
  {
    cost = 0.0;
    int end_idx = q.cols() - order_;
    double demarcation = cps_.clearance;
    double a = 3 * demarcation, b = -3 * pow(demarcation, 2), c = pow(demarcation, 3);

    force_stop_type_ = DONT_STOP;
    if (iter_num > 3 && smoothness_cost / (cps_.size - 2 * order_) < 0.1) // 0.1 is an experimental value that indicates the trajectory is smooth enough.
    {
      check_collision_and_rebound();
    }

    /*** calculate distance cost and gradient ***/
    for (auto i = order_; i < end_idx; ++i)
    {
      for (size_t j = 0; j < cps_.direction[i].size(); ++j)
      {
        double dist = (cps_.points.col(i) - cps_.base_point[i][j]).dot(cps_.direction[i][j]);
        double dist_err = cps_.clearance - dist;
        Eigen::Vector3d dist_grad = cps_.direction[i][j];

        if (dist_err < 0)
        {
          /* do nothing */
        }
        else if (dist_err < demarcation)
        {
          cost += pow(dist_err, 3);
          gradient.col(i) += -3.0 * dist_err * dist_err * dist_grad;
        }
        else
        {
          cost += a * dist_err * dist_err + b * dist_err + c;
          gradient.col(i) += -(2.0 * a * dist_err + b) * dist_grad;
        }
      }
    }
  }

  void BsplineOptimizer::calcSoftClearanceCost(const Eigen::MatrixXd &q, double &cost,
                                                 Eigen::MatrixXd &gradient)
  {
    if (order_ != 3 || !grid_map_->hasSoftClearance() || q.cols() < 4)
      return;

    // Sample the cubic curve between knots. Applying the same basis weights
    // to the gradient propagates its clearance cost back to the control points.
    const double sample_weight = 1.0 / (3.0 * (q.cols() - 3));
    for (const double t : {1.0 / 6, 0.5, 5.0 / 6})
    {
      const double t2 = t * t, t3 = t2 * t, a = 1.0 - t;
      const Eigen::Vector4d basis = Eigen::Vector4d(a * a * a, 3 * t3 - 6 * t2 + 4,
                                                   -3 * t3 + 3 * t2 + 3 * t + 1, t3) / 6.0;
      for (Eigen::Index i = 0; i + 3 < q.cols(); ++i)
      {
        Eigen::Vector3d slope;
        cost += sample_weight * grid_map_->softClearanceCost(q.middleCols<4>(i) * basis, slope);
        gradient.middleCols<4>(i) += sample_weight * slope * basis.transpose();
      }
    }
  }

  void BsplineOptimizer::calcFitnessCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient)
  {

    cost = 0.0;

    int end_idx = q.cols() - order_;

    // def: f = |x*v|^2/a^2 + |x×v|^2/b^2
    double a2 = 25, b2 = 1;
    for (auto i = order_ - 1; i < end_idx + 1; ++i)
    {
      Eigen::Vector3d x = (q.col(i - 1) + 4 * q.col(i) + q.col(i + 1)) / 6.0 - ref_pts_[i - 1];
      Eigen::Vector3d v = (ref_pts_[i] - ref_pts_[i - 2]).normalized();

      double xdotv = x.dot(v);
      Eigen::Vector3d xcrossv = x.cross(v);

      double f = pow((xdotv), 2) / a2 + pow(xcrossv.norm(), 2) / b2;
      cost += f;

      Eigen::Matrix3d m;
      m << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
      Eigen::Vector3d df_dx = 2 * xdotv / a2 * v + 2 / b2 * m * xcrossv;

      gradient.col(i - 1) += df_dx / 6;
      gradient.col(i) += 4 * df_dx / 6;
      gradient.col(i + 1) += df_dx / 6;
    }
  }

  void BsplineOptimizer::calcSmoothnessCost(const Eigen::MatrixXd &q, double &cost,
                                            Eigen::MatrixXd &gradient, bool falg_use_jerk /* = true*/)
  {

    cost = 0.0;

    if (falg_use_jerk)
    {
      Eigen::Vector3d jerk, temp_j;

      for (int i = 0; i < q.cols() - 3; i++)
      {
        /* evaluate jerk */
        jerk = q.col(i + 3) - 3 * q.col(i + 2) + 3 * q.col(i + 1) - q.col(i);
        cost += jerk.squaredNorm();
        temp_j = 2.0 * jerk;
        /* jerk gradient */
        gradient.col(i + 0) += -temp_j;
        gradient.col(i + 1) += 3.0 * temp_j;
        gradient.col(i + 2) += -3.0 * temp_j;
        gradient.col(i + 3) += temp_j;
      }
    }
    else
    {
      Eigen::Vector3d acc, temp_acc;

      for (int i = 0; i < q.cols() - 2; i++)
      {
        /* evaluate acc */
        acc = q.col(i + 2) - 2 * q.col(i + 1) + q.col(i);
        cost += acc.squaredNorm();
        temp_acc = 2.0 * acc;
        /* acc gradient */
        gradient.col(i + 0) += temp_acc;
        gradient.col(i + 1) += -2.0 * temp_acc;
        gradient.col(i + 2) += temp_acc;
      }
    }
  }

  void BsplineOptimizer::calcTerminalCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient)
  {
    cost = 0.0;

    // zero cost and gradient in hard constraints
    Eigen::Vector3d q_3, q_2, q_1, dq;
    q_3 = q.col(q.cols() - 3);
    q_2 = q.col(q.cols() - 2);
    q_1 = q.col(q.cols() - 1);

    dq = 1 / 6.0 * (q_3 + 4 * q_2 + q_1) - local_target_pt_;
    cost += dq.squaredNorm();

    gradient.col(q.cols() - 3) += 2 * dq * (1 / 6.0);
    gradient.col(q.cols() - 2) += 2 * dq * (4 / 6.0);
    gradient.col(q.cols() - 1) += 2 * dq * (1 / 6.0);
  }

  void BsplineOptimizer::calcFeasibilityCost(const Eigen::MatrixXd &q, double &cost,
                                             Eigen::MatrixXd &gradient)
  {

    // #define SECOND_DERIVATIVE_CONTINOUS

#ifdef SECOND_DERIVATIVE_CONTINOUS

    cost = 0.0;
    double demarcation = 1.0; // 1m/s, 1m/s/s
    double ar = 3 * demarcation, br = -3 * pow(demarcation, 2), cr = pow(demarcation, 3);
    double al = ar, bl = -br, cl = cr;

    /* abbreviation */
    double ts, ts_inv2, ts_inv3;
    ts = bspline_interval_;
    ts_inv2 = 1 / ts / ts;
    ts_inv3 = 1 / ts / ts / ts;

    /* velocity feasibility */
    for (int i = 0; i < q.cols() - 1; i++)
    {
      Eigen::Vector3d vi = (q.col(i + 1) - q.col(i)) / ts;

      for (int j = 0; j < 3; j++)
      {
        if (vi(j) > max_vel_ + demarcation)
        {
          double diff = vi(j) - max_vel_;
          cost += (ar * diff * diff + br * diff + cr) * ts_inv3; // multiply ts_inv3 to make vel and acc has similar magnitude

          double grad = (2.0 * ar * diff + br) / ts * ts_inv3;
          gradient(j, i + 0) += -grad;
          gradient(j, i + 1) += grad;
        }
        else if (vi(j) > max_vel_)
        {
          double diff = vi(j) - max_vel_;
          cost += pow(diff, 3) * ts_inv3;
          ;

          double grad = 3 * diff * diff / ts * ts_inv3;
          ;
          gradient(j, i + 0) += -grad;
          gradient(j, i + 1) += grad;
        }
        else if (vi(j) < -(max_vel_ + demarcation))
        {
          double diff = vi(j) + max_vel_;
          cost += (al * diff * diff + bl * diff + cl) * ts_inv3;

          double grad = (2.0 * al * diff + bl) / ts * ts_inv3;
          gradient(j, i + 0) += -grad;
          gradient(j, i + 1) += grad;
        }
        else if (vi(j) < -max_vel_)
        {
          double diff = vi(j) + max_vel_;
          cost += -pow(diff, 3) * ts_inv3;

          double grad = -3 * diff * diff / ts * ts_inv3;
          gradient(j, i + 0) += -grad;
          gradient(j, i + 1) += grad;
        }
        else
        {
          /* nothing happened */
        }
      }
    }

    /* acceleration feasibility */
    for (int i = 0; i < q.cols() - 2; i++)
    {
      Eigen::Vector3d ai = (q.col(i + 2) - 2 * q.col(i + 1) + q.col(i)) * ts_inv2;

      for (int j = 0; j < 3; j++)
      {
        if (ai(j) > max_acc_ + demarcation)
        {
          double diff = ai(j) - max_acc_;
          cost += ar * diff * diff + br * diff + cr;

          double grad = (2.0 * ar * diff + br) * ts_inv2;
          gradient(j, i + 0) += grad;
          gradient(j, i + 1) += -2 * grad;
          gradient(j, i + 2) += grad;
        }
        else if (ai(j) > max_acc_)
        {
          double diff = ai(j) - max_acc_;
          cost += pow(diff, 3);

          double grad = 3 * diff * diff * ts_inv2;
          gradient(j, i + 0) += grad;
          gradient(j, i + 1) += -2 * grad;
          gradient(j, i + 2) += grad;
        }
        else if (ai(j) < -(max_acc_ + demarcation))
        {
          double diff = ai(j) + max_acc_;
          cost += al * diff * diff + bl * diff + cl;

          double grad = (2.0 * al * diff + bl) * ts_inv2;
          gradient(j, i + 0) += grad;
          gradient(j, i + 1) += -2 * grad;
          gradient(j, i + 2) += grad;
        }
        else if (ai(j) < -max_acc_)
        {
          double diff = ai(j) + max_acc_;
          cost += -pow(diff, 3);

          double grad = -3 * diff * diff * ts_inv2;
          gradient(j, i + 0) += grad;
          gradient(j, i + 1) += -2 * grad;
          gradient(j, i + 2) += grad;
        }
        else
        {
          /* nothing happened */
        }
      }
    }

#else

    cost = 0.0;
    /* abbreviation */
    double ts, /*vm2, am2, */ ts_inv2;
    // vm2 = max_vel_ * max_vel_;
    // am2 = max_acc_ * max_acc_;

    ts = bspline_interval_;
    ts_inv2 = 1 / ts / ts;

    /* velocity feasibility */
    for (int i = 0; i < q.cols() - 1; i++)
    {
      Eigen::Vector3d vi = (q.col(i + 1) - q.col(i)) / ts;

      // cout << "temp_v * vi=" ;
      for (int j = 0; j < 3; j++)
      {
        if (vi(j) > max_vel_)
        {
          // cout << "zx-todo VEL" << endl;
          // cout << vi(j) << endl;
          cost += pow(vi(j) - max_vel_, 2) * ts_inv2; // multiply ts_inv3 to make vel and acc has similar magnitude

          gradient(j, i + 0) += -2 * (vi(j) - max_vel_) / ts * ts_inv2;
          gradient(j, i + 1) += 2 * (vi(j) - max_vel_) / ts * ts_inv2;
        }
        else if (vi(j) < -max_vel_)
        {
          cost += pow(vi(j) + max_vel_, 2) * ts_inv2;

          gradient(j, i + 0) += -2 * (vi(j) + max_vel_) / ts * ts_inv2;
          gradient(j, i + 1) += 2 * (vi(j) + max_vel_) / ts * ts_inv2;
        }
        else
        {
          /* code */
        }
      }
    }

    /* acceleration feasibility */
    for (int i = 0; i < q.cols() - 2; i++)
    {
      Eigen::Vector3d ai = (q.col(i + 2) - 2 * q.col(i + 1) + q.col(i)) * ts_inv2;

      // cout << "temp_a * ai=" ;
      for (int j = 0; j < 3; j++)
      {
        if (ai(j) > max_acc_)
        {
          // cout << "zx-todo ACC" << endl;
          // cout << ai(j) << endl;
          cost += pow(ai(j) - max_acc_, 2);

          gradient(j, i + 0) += 2 * (ai(j) - max_acc_) * ts_inv2;
          gradient(j, i + 1) += -4 * (ai(j) - max_acc_) * ts_inv2;
          gradient(j, i + 2) += 2 * (ai(j) - max_acc_) * ts_inv2;
        }
        else if (ai(j) < -max_acc_)
        {
          cost += pow(ai(j) + max_acc_, 2);

          gradient(j, i + 0) += 2 * (ai(j) + max_acc_) * ts_inv2;
          gradient(j, i + 1) += -4 * (ai(j) + max_acc_) * ts_inv2;
          gradient(j, i + 2) += 2 * (ai(j) + max_acc_) * ts_inv2;
        }
        else
        {
          /* code */
        }
      }
      // cout << endl;
    }

#endif
  }
  /*                    上面七个都是计算损失的函数           */

  // 检查是否有障碍物？
  bool BsplineOptimizer::check_collision_and_rebound(void)
  {

    int end_idx = cps_.size - order_;

    /*** Check and segment the initial trajectory according to obstacles ***/
    int in_id, out_id;
    vector<std::pair<int, int>> segment_ids;
    bool flag_new_obs_valid = false;
    const int i_end = collisionCheckEndIndex(end_idx);
    for (int i = order_ - 1; i <= i_end; ++i)
    {

      bool occ = grid_map_->getInflateOccupancy(cps_.points.col(i));

      /*** check if the new collision will be valid ***/
      if (occ)
      {
        for (size_t k = 0; k < cps_.direction[i].size(); ++k)
        {
          cout.precision(2);
          if ((cps_.points.col(i) - cps_.base_point[i][k]).dot(cps_.direction[i][k]) < 1 * grid_map_->getResolution()) // current point is outside all the collision_points.
          {
            occ = false; // Not really takes effect, just for better hunman understanding.
            break;
          }
        }
      }

      if (occ)
      {
        flag_new_obs_valid = true;

        int j;
        for (j = i - 1; j >= 0; --j)
        {
          occ = grid_map_->getInflateOccupancy(cps_.points.col(j));
          if (!occ)
          {
            in_id = j;
            break;
          }
        }
        if (j < 0) // fail to get the obs free point
        {
          RCLCPP_ERROR(rclcpp::get_logger("check_collision_and_rebound"), "ERROR! the drone is in obstacle. This should not happen.");
          in_id = 0;
        }

        for (j = i + 1; j < cps_.size; ++j)
        {
          occ = grid_map_->getInflateOccupancy(cps_.points.col(j));

          if (!occ)
          {
            out_id = j;
            break;
          }
        }
        if (j >= cps_.size) // fail to get the obs free point
        {
          RCLCPP_WARN(rclcpp::get_logger("check_collision_and_rebound"), 
                      "WARN! terminal point of the current trajectory is in obstacle, skip this planning.");

          force_stop_type_ = STOP_FOR_ERROR;
          return false;
        }

        i = j + 1;

        segment_ids.push_back(std::pair<int, int>(in_id, out_id));
      }
    }

    if (flag_new_obs_valid)
    {
      for (size_t i = 1; i < segment_ids.size(); i++) // Avoid overlap
      {
        if (segment_ids[i - 1].second >= segment_ids[i].first)
        {
          double middle = (double)(segment_ids[i - 1].second + segment_ids[i].first) / 2.0;
          segment_ids[i - 1].second = static_cast<int>(middle - 0.1);
          segment_ids[i].first = static_cast<int>(middle + 1.1);
        }
      }

      for (const auto &[first, last] : segment_ids)
        if (!assignCollisionGuide(cps_.points, first, last, first, last))
          RCLCPP_DEBUG(rclcpp::get_logger("check_collision_and_rebound"), "No compatible collision guide");

      force_stop_type_ = STOP_FOR_REBOUND;
      return true;
    }

    return false;
  }

  // 设置时间间隔ts，调用rebound_optimize(final_cost)将轨迹推出障碍物，得到最优的无碰撞轨迹，并将其控制点赋值给optimal_points
  bool BsplineOptimizer::BsplineOptimizeTrajRebound(Eigen::MatrixXd &optimal_points, double ts)
  {
    setBsplineInterval(ts);

    double final_cost;
    bool flag_success = rebound_optimize(final_cost);

    optimal_points = cps_.points;

    return flag_success;
  }

  // 设置初始控制点control_points、时间间隔ts，调用rebound_optimize(final_cost)将轨迹推出障碍物，
  // 得到最优的无碰撞轨迹，并将其控制点赋值给optimal_points
  bool BsplineOptimizer::BsplineOptimizeTrajRebound(Eigen::MatrixXd &optimal_points, double &final_cost, const ControlPoints &control_points, double ts)
  {
    // 将时间间隔存储到成员变量
    setBsplineInterval(ts);

    cps_ = control_points;

    bool flag_success = rebound_optimize(final_cost);

    optimal_points = cps_.points;

    return flag_success;
  }

  // 设置初始控制点init_points、时间间隔ts，调用refine_optimize()重新分配时间，得到最优的动力学可行轨迹，并将其控制点赋值给optimal_points
  bool BsplineOptimizer::BsplineOptimizeTrajRefine(const Eigen::MatrixXd &init_points, const double ts, Eigen::MatrixXd &optimal_points)
  {

    // 将控制点数据存储到成员变量
    setControlPoints(init_points);
    setBsplineInterval(ts);

    bool flag_success = refine_optimize();

    optimal_points = cps_.points;

    return flag_success;
  }

  // 使用L-BFGS方法对目标函数进行优化，得到光滑、无碰撞、动力学可行、与其他无人机碰撞、结束项的轨迹。
  int BsplineOptimizer::collisionCheckEndIndex(int end) const
  {
    return end - static_cast<int>((1.0 - collision_check_fraction_) * std::max(0, end - order_));
  }

  double BsplineOptimizer::collisionSampleStep(UniformBspline &trajectory) const
  {
    const double speed = trajectory.getDerivative().getMaxControlPointNorm();
    return std::isfinite(speed) ? 0.5 * grid_map_->getResolution() / std::max(speed, 1e-6) : 0.0;
  }

  bool BsplineOptimizer::trajectoryInCollision(UniformBspline &trajectory)
  {
    double time;
    return firstCollision(trajectory, time, 1.0);
  }

  bool BsplineOptimizer::firstCollision(UniformBspline &trajectory, double &time, double fraction)
  {
    const double end = trajectory.getTimeSum() * fraction;
    const double step = collisionSampleStep(trajectory);
    if (!std::isfinite(end) || end < 0.0 || step <= 0.0)
    {
      time = 0.0;
      return true;
    }
    for (time = 0.0;; time = std::min(time + step, end))
    {
      const Eigen::Vector3d position = trajectory.evaluateDeBoorT(time);
      if (!position.allFinite() || grid_map_->getInflateOccupancy(position))
        return true;
      if (time >= end)
        return false;
    }
  }

  bool BsplineOptimizer::rebound_optimize(double &final_cost)
  {
    iter_num_ = 0;
    int start_id = order_;
    // int end_id = this->cps_.size - order_; //Fixed end
    int end_id = this->cps_.size; // Free end
    // 变量个数
    variable_num_ = 3 * (end_id - start_id);

    rclcpp::Time t0 = rclcpp::Clock().now(), t1, t2;
    int restart_nums = 0, rebound_times = 0;
    ;
    bool flag_force_return, flag_occ, success;
    new_lambda2_ = lambda2_;
    constexpr int MAX_RESART_NUMS_SET = 3;
    do
    {
      /* ---------- prepare ---------- */
      min_cost_ = std::numeric_limits<double>::max();
      min_ellip_dist_ = INIT_min_ellip_dist_;
      iter_num_ = 0;
      flag_force_return = false;
      flag_occ = false;
      success = false;

      // 控制点数组初始化
      double q[variable_num_];
      memcpy(q, cps_.points.data() + 3 * start_id, variable_num_ * sizeof(q[0]));

      // 初始化L-BFGS算法的参数
      lbfgs::lbfgs_parameter_t lbfgs_params;
      lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
      lbfgs_params.mem_size = 16; // 算法保存的历史优化步数
      lbfgs_params.max_iterations = 200;
      lbfgs_params.g_epsilon = 0.01;

      /* ---------- optimize ---------- */
      t1 = rclcpp::Clock().now();
      // 执行优化
      int result = lbfgs::lbfgs_optimize(variable_num_, q, &final_cost, BsplineOptimizer::costFunctionRebound, NULL, BsplineOptimizer::earlyExit, this, &lbfgs_params);
      t2 = rclcpp::Clock().now();
      double time_ms = (t2 - t1).seconds() * 1000;
      double total_time_ms = (t2 - t0).seconds() * 1000;

      /* ---------- success temporary, check collision again ---------- */
      // 收敛、达到最大迭代次数、已达到最小值或被停止，则进入碰撞检测阶段
      if (result == lbfgs::LBFGS_CONVERGENCE ||
          result == lbfgs::LBFGSERR_MAXIMUMITERATION ||
          result == lbfgs::LBFGS_ALREADY_MINIMIZED ||
          result == lbfgs::LBFGS_STOP)
      {
        // ROS_WARN("Solver error in planning!, return = %s", lbfgs::lbfgs_strerror(result));
        flag_force_return = false;

        /*** collision check, phase 1 ***/
        if ((min_ellip_dist_ != INIT_min_ellip_dist_) && (min_ellip_dist_ > swarm_clearance_))
        {
          success = false;
          restart_nums++;
          initControlPoints(cps_.points, false);
          new_lambda2_ *= 2; // 提高规避权重

          printf("\033[32miter(+1)=%d,time(ms)=%5.3f, swarm too close, keep optimizing\n\033[0m", iter_num_, time_ms);

          continue;
        }

        /*** collision check, phase 2 ***/
        UniformBspline traj(cps_.points, 3, bspline_interval_);
        double collision_time;
        flag_occ = firstCollision(traj, collision_time, collision_check_fraction_);
        if (flag_occ && collision_time <= bspline_interval_)
        {
          RCLCPP_WARN(rclcpp::get_logger("rebound_optimize"),
                      "Trajectory starts in an obstacle, t=%f", collision_time);
          return false;
        }

        /*** collision check, phase 3 ***/
// #define USE_SECOND_CLEARENCE_CHECK
#ifdef USE_SECOND_CLEARENCE_CHECK
        bool flag_cls_xyp, flag_cls_xyn, flag_cls_zp, flag_cls_zn;
        Eigen::Vector3d start_end_vec = traj.evaluateDeBoorT(traj.getTimeSum()) - traj.evaluateDeBoorT(0.0);
        Eigen::Vector3d offset_xy(-start_end_vec(0), start_end_vec(1), 0);
        offset_xy.normalize();
        Eigen::Vector3d offset_z = start_end_vec.cross(offset_xy);
        offset_z.normalize();
        offset_xy *= cps_.clearance / 2;
        offset_z *= cps_.clearance / 2;

        Eigen::MatrixXd check_pts(cps_.points.rows(), cps_.points.cols());
        for (Eigen::Index i = 0; i < cps_.points.cols(); i++)
        {
          check_pts.col(i) = cps_.points.col(i);
          check_pts(0, i) += offset_xy(0);
          check_pts(1, i) += offset_xy(1);
          check_pts(2, i) += offset_xy(2);
        }
        flag_cls_xyp = initControlPoints(check_pts, false).size() > 0;
        for (Eigen::Index i = 0; i < cps_.points.cols(); i++)
        {
          check_pts(0, i) -= 2 * offset_xy(0);
          check_pts(1, i) -= 2 * offset_xy(1);
          check_pts(2, i) -= 2 * offset_xy(2);
        }
        flag_cls_xyn = initControlPoints(check_pts, false).size() > 0;
        for (Eigen::Index i = 0; i < cps_.points.cols(); i++)
        {
          check_pts(0, i) += offset_xy(0) + offset_z(0);
          check_pts(1, i) += offset_xy(1) + offset_z(1);
          check_pts(2, i) += offset_xy(2) + offset_z(2);
        }
        flag_cls_zp = initControlPoints(check_pts, false).size() > 0;
        for (Eigen::Index i = 0; i < cps_.points.cols(); i++)
        {
          check_pts(0, i) -= 2 * offset_z(0);
          check_pts(1, i) -= 2 * offset_z(1);
          check_pts(2, i) -= 2 * offset_z(2);
        }
        flag_cls_zn = initControlPoints(check_pts, false).size() > 0;
        if ((flag_cls_xyp ^ flag_cls_xyn) || (flag_cls_zp ^ flag_cls_zn))
          flag_occ = true;
#endif

        // 如果没有检测到碰撞视为优化成功
        if (!flag_occ)
        {
          printf("\033[32miter(+1)=%d,time(ms)=%5.3f,total_t(ms)=%5.3f,cost=%5.3f\n\033[0m", iter_num_, time_ms, total_time_ms, final_cost);
          success = true;
        }
        // 如果有碰撞则重新初始化
        else // restart
        {
          restart_nums++;
          initControlPoints(cps_.points, false);
          new_lambda2_ *= 2;

          printf("\033[32miter(+1)=%d,time(ms)=%5.3f, collided, keep optimizing\n\033[0m", iter_num_, time_ms);
        }
      }
      // 如果优化被强制取消
      else if (result == lbfgs::LBFGSERR_CANCELED)
      {
        flag_force_return = true;
        rebound_times++;
        cout << "iter=" << iter_num_ << ",time(ms)=" << time_ms << ",rebound." << endl;
      }
      else
      {
        RCLCPP_WARN(rclcpp::get_logger("rebound_optimize"), 
                                        "Solver error. Return = %d, %s. Skip this planning.", result, lbfgs::lbfgs_strerror(result));
        // while (rclcpp::ok());
      }

    } while (
        ((flag_occ || ((min_ellip_dist_ != INIT_min_ellip_dist_) && (min_ellip_dist_ > swarm_clearance_))) && restart_nums < MAX_RESART_NUMS_SET) ||
        (flag_force_return && force_stop_type_ == STOP_FOR_REBOUND && rebound_times <= 20));

    return success;
  }

  // 使用L-BFGS方法对目标函数进行优化，得到重新分配时间后，光滑、拟合较好、动力学可行的轨迹。
  bool BsplineOptimizer::refine_optimize()
  {
    iter_num_ = 0;
    int start_id = order_;
    int end_id = this->cps_.points.cols() - order_;
    variable_num_ = 3 * (end_id - start_id);

    double q[variable_num_];
    double final_cost;

    memcpy(q, cps_.points.data() + 3 * start_id, variable_num_ * sizeof(q[0]));

    double origin_lambda4 = lambda4_;
    bool flag_safe = true;
    int iter_count = 0;
    do
    {
      lbfgs::lbfgs_parameter_t lbfgs_params;
      lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
      lbfgs_params.mem_size = 16;
      lbfgs_params.max_iterations = 200;
      lbfgs_params.g_epsilon = 0.001;

      int result = lbfgs::lbfgs_optimize(variable_num_, q, &final_cost, BsplineOptimizer::costFunctionRefine, NULL, NULL, this, &lbfgs_params);
      if (result == lbfgs::LBFGS_CONVERGENCE ||
          result == lbfgs::LBFGSERR_MAXIMUMITERATION ||
          result == lbfgs::LBFGS_ALREADY_MINIMIZED ||
          result == lbfgs::LBFGS_STOP)
      {
        // pass
      }
      else
      {
        RCLCPP_ERROR(rclcpp::get_logger("refine_optimize"), 
                                        "Solver error in refining!, return = %d, %s", result, lbfgs::lbfgs_strerror(result));
      }

      UniformBspline traj(cps_.points, 3, bspline_interval_);
      double collision_time;
      flag_safe = !firstCollision(traj, collision_time, collision_check_fraction_);

      // 如果存在碰撞则调整参数并重新迭代
      if (!flag_safe)
        lambda4_ *= 2;

      iter_count++;
    } while (!flag_safe && iter_count <= 0);

    lambda4_ = origin_lambda4;

    // cout << "iter_num_=" << iter_num_ << endl;

    return flag_safe;
  }

  // 计算损失
  void BsplineOptimizer::combineCostRebound(const double *x, double *grad, double &f_combine, const int n)
  {
    // cout << "drone_id_=" << drone_id_ << endl;
    // cout << "cps_.points.size()=" << cps_.points.size() << endl;
    // cout << "n=" << n << endl;
    // cout << "sizeof(x[0])=" << sizeof(x[0]) << endl;

    memcpy(cps_.points.data() + 3 * order_, x, n * sizeof(x[0]));

    /* ---------- evaluate cost and gradient ---------- */
    double f_smoothness, f_distance, f_feasibility /*, f_mov_objs*/, f_swarm, f_terminal;

    Eigen::MatrixXd g_smoothness = Eigen::MatrixXd::Zero(3, cps_.size);
    Eigen::MatrixXd g_distance = Eigen::MatrixXd::Zero(3, cps_.size);
    Eigen::MatrixXd g_feasibility = Eigen::MatrixXd::Zero(3, cps_.size);
    // Eigen::MatrixXd g_mov_objs = Eigen::MatrixXd::Zero(3, cps_.size);
    Eigen::MatrixXd g_swarm = Eigen::MatrixXd::Zero(3, cps_.size);
    Eigen::MatrixXd g_terminal = Eigen::MatrixXd::Zero(3, cps_.size);

    calcSmoothnessCost(cps_.points, f_smoothness, g_smoothness);
    calcDistanceCostRebound(cps_.points, f_distance, g_distance, iter_num_, f_smoothness);
    calcFeasibilityCost(cps_.points, f_feasibility, g_feasibility);
    // calcMovingObjCost(cps_.points, f_mov_objs, g_mov_objs);
    calcSwarmCost(cps_.points, f_swarm, g_swarm);
    calcTerminalCost(cps_.points, f_terminal, g_terminal);

    double f_clearance = 0.0;
    Eigen::MatrixXd g_clearance = Eigen::MatrixXd::Zero(3, cps_.points.cols());
    calcSoftClearanceCost(cps_.points, f_clearance, g_clearance);
    f_combine = lambda1_ * f_smoothness + new_lambda2_ * f_distance + lambda3_ * f_feasibility + new_lambda2_ * f_swarm + lambda2_ * f_terminal + f_clearance;
    // f_combine = lambda1_ * f_smoothness + new_lambda2_ * f_distance + lambda3_ * f_feasibility + new_lambda2_ * f_mov_objs;
    // printf("origin %f %f %f %f\n", f_smoothness, f_distance, f_feasibility, f_combine);

    Eigen::MatrixXd grad_3D = lambda1_ * g_smoothness + new_lambda2_ * g_distance + lambda3_ * g_feasibility + new_lambda2_ * g_swarm + lambda2_ * g_terminal + g_clearance;
    // Eigen::MatrixXd grad_3D = lambda1_ * g_smoothness + new_lambda2_ * g_distance + lambda3_ * g_feasibility + new_lambda2_ * g_mov_objs;
    memcpy(grad, grad_3D.data() + 3 * order_, n * sizeof(grad[0]));
  }

  // 计算优化后的损失
  void BsplineOptimizer::combineCostRefine(const double *x, double *grad, double &f_combine, const int n)
  {

    memcpy(cps_.points.data() + 3 * order_, x, n * sizeof(x[0]));

    /* ---------- evaluate cost and gradient ---------- */
    double f_smoothness, f_fitness, f_feasibility;

    Eigen::MatrixXd g_smoothness = Eigen::MatrixXd::Zero(3, cps_.points.cols());
    Eigen::MatrixXd g_fitness = Eigen::MatrixXd::Zero(3, cps_.points.cols());
    Eigen::MatrixXd g_feasibility = Eigen::MatrixXd::Zero(3, cps_.points.cols());

    // time_satrt = rclcpp::Clock().now();

    calcSmoothnessCost(cps_.points, f_smoothness, g_smoothness);
    calcFitnessCost(cps_.points, f_fitness, g_fitness);
    calcFeasibilityCost(cps_.points, f_feasibility, g_feasibility);

    /* ---------- convert to solver format...---------- */
    double f_clearance = 0.0;
    Eigen::MatrixXd g_clearance = Eigen::MatrixXd::Zero(3, cps_.points.cols());
    calcSoftClearanceCost(cps_.points, f_clearance, g_clearance);
    f_combine = lambda1_ * f_smoothness + lambda4_ * f_fitness + lambda3_ * f_feasibility + f_clearance;
    // printf("origin %f %f %f %f\n", f_smoothness, f_fitness, f_feasibility, f_combine);

    Eigen::MatrixXd grad_3D = lambda1_ * g_smoothness + lambda4_ * g_fitness + lambda3_ * g_feasibility + g_clearance;
    memcpy(grad, grad_3D.data() + 3 * order_, n * sizeof(grad[0]));
  }

} // namespace ego_planner