
#include <ego_planner/ego_replan_fsm.h>

namespace ego_planner
{

  void EGOReplanFSM::init(rclcpp::Node::SharedPtr &node)
  {
    node_ = node;
    
    current_wp_ = 0;
    exec_state_ = FSM_EXEC_STATE::INIT;
    have_target_ = false;
    have_odom_ = false;
    have_recv_pre_agent_ = false;

    node_->declare_parameter("fsm/flight_type", -1);
    node_->declare_parameter("fsm/thresh_replan_time", -1.0);
    node_->declare_parameter("fsm/thresh_no_replan_meter", -1.0);
    node_->declare_parameter("fsm/planning_horizon", -1.0);
    node_->declare_parameter("fsm/planning_horizen_time", -1.0);
    node_->declare_parameter("fsm/emergency_time", 1.0);
    node_->declare_parameter("fsm/realworld_experiment", false);
    node_->declare_parameter("fsm/fail_safe", true);
    node_->declare_parameter("fsm/default_goal_height", 1.0);
    node_->get_parameter("fsm/default_goal_height", default_goal_height_);
    node_->declare_parameter("fsm/goal_topic", std::string("/move_base_simple/goal"));
    const auto goal_3d_topic = node_->declare_parameter<std::string>("fsm/goal_3d_topic", "/ego/goal_pose_3d");
    node_->declare_parameter("fsm/goal_z_valid_min", 0.1);
    node_->declare_parameter("fsm/use_astar_global", false);
    std::string goal_topic;

    node_->get_parameter("fsm/use_astar_global", use_astar_global_);
    node_->get_parameter("fsm/goal_topic", goal_topic);
    node_->get_parameter("fsm/goal_z_valid_min", goal_z_valid_min_);
    node_->get_parameter("fsm/flight_type", target_type_);
    node_->get_parameter("fsm/thresh_replan_time", replan_thresh_);
    node_->get_parameter("fsm/thresh_no_replan_meter", no_replan_thresh_);
    node_->get_parameter("fsm/planning_horizon", planning_horizen_);
    dynamic_planning_horizon_ = node_->declare_parameter("fsm/dynamic_planning_horizon", false);
    if (dynamic_planning_horizon_ && (!std::isfinite(planning_horizen_) || planning_horizen_ <= 0.0))
      throw std::invalid_argument("Dynamic planning requires a positive finite fallback horizon");
    node_->get_parameter("fsm/planning_horizen_time", planning_horizen_time_);
    node_->get_parameter("fsm/emergency_time", emergency_time_);
    node_->get_parameter("fsm/realworld_experiment", flag_realworld_experiment_);
    node_->get_parameter("fsm/fail_safe", enable_fail_safe_);

    have_trigger_ = !flag_realworld_experiment_;

    node_->declare_parameter("fsm/waypoint_num", -1);
    node_->get_parameter("fsm/waypoint_num", waypoint_num_);

    for (int i = 0; i < waypoint_num_; i++)
    {
      node_->declare_parameter("fsm/waypoint" + to_string(i) + "_x", -1.0);
      node_->declare_parameter("fsm/waypoint" + to_string(i) + "_y", -1.0);
      node_->declare_parameter("fsm/waypoint" + to_string(i) + "_z", -1.0);

      node_->get_parameter("fsm/waypoint" + to_string(i) + "_x", waypoints_[i][0]);
      node_->get_parameter("fsm/waypoint" + to_string(i) + "_y", waypoints_[i][1]);
      node_->get_parameter("fsm/waypoint" + to_string(i) + "_z", waypoints_[i][2]);
    }

    /* initialize main modules */
    visualization_.reset(new PlanningVisualization(node_));

    planner_manager_.reset(new EGOPlannerManager);

    planner_manager_->initPlanModules(node_, visualization_);

    planner_manager_->deliverTrajToOptimizer(); // store trajectories
    planner_manager_->setDroneIdtoOpt();

    px4_offboard_.init(*node_, [this]() { cancelOffboardGoal(); });

    /* callback*/
    exec_timer_ = node_->create_wall_timer(std::chrono::milliseconds(10),
                                           std::bind(&EGOReplanFSM::execFSMCallback, this));

    safety_timer_ = node_->create_wall_timer(std::chrono::milliseconds(50),
                                             std::bind(&EGOReplanFSM::checkCollisionCallback, this));

    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "odom_world",
        1,
        [this](const std::shared_ptr<const nav_msgs::msg::Odometry> &msg)
        {
          this->odometryCallback(msg);
        });
    // std::bind(&EGOReplanFSM::odometryCallback, this, std::placeholders::_1));

    if (planner_manager_->pp_.drone_id >= 1)
    {
      string sub_topic_name = string("/drone_") + std::to_string(planner_manager_->pp_.drone_id - 1) + string("_planning/swarm_trajs");
      swarm_trajs_sub_ = node_->create_subscription<traj_utils::msg::MultiBsplines>(
          sub_topic_name,
          10,
          [this](const std::shared_ptr<const traj_utils::msg::MultiBsplines> &msg)
          {
            this->swarmTrajsCallback(msg);
          });
    }

    // ros2 中topic名字中不能出现负号，单机id是-1需要处理
    // string pub_topic_name = string("/drone_") + std::to_string(planner_manager_->pp_.drone_id) + string("_planning/swarm_trajs");
    string pub_topic_name;
    if (planner_manager_->pp_.drone_id <= -1)
    {
      RCLCPP_INFO(node_->get_logger(), "single drone:%d", planner_manager_->pp_.drone_id);
      pub_topic_name = string("/drone_") + "single" + string("_planning/swarm_trajs");
    }else
    {
      pub_topic_name = string("/drone_") + std::to_string(planner_manager_->pp_.drone_id) + string("_planning/swarm_trajs");
    }
    
    swarm_trajs_pub_ = node_->create_publisher<traj_utils::msg::MultiBsplines>(pub_topic_name, 10);
    new_goal_pub_ = node_->create_publisher<std_msgs::msg::Int32>("/ego/new_goal", 10);
    broadcast_bspline_pub_ = node_->create_publisher<traj_utils::msg::Bspline>("planning/broadcast_bspline_from_planner", 10);
    broadcast_bspline_sub_ = node_->create_subscription<traj_utils::msg::Bspline>(
        "planning/broadcast_bspline_to_planner",
        100,
        [this](const std::shared_ptr<const traj_utils::msg::Bspline> &msg)
        {
          this->BroadcastBsplineCallback(msg);
        });

    bspline_pub_ = node_->create_publisher<traj_utils::msg::Bspline>("planning/bspline", 10);
    data_disp_pub_ = node_->create_publisher<traj_utils::msg::DataDisp>("planning/data_display", 100);

    if (target_type_ == TARGET_TYPE::MANUAL_TARGET)
    {
      frontier_centers_sub_ = node_->create_subscription<geometry_msgs::msg::PoseArray>(
          "/frontier/centers", 1,
          [this](geometry_msgs::msg::PoseArray::ConstSharedPtr msg)
          {
            frontier_centers_ = std::move(msg);
            frontier_received_ = std::chrono::steady_clock::now();
          });
      goal_preview_service_ = node_->create_service<nav_msgs::srv::GetPlan>(
          "/ego/preview_goal",
          [this](const std::shared_ptr<nav_msgs::srv::GetPlan::Request> request,
                 std::shared_ptr<nav_msgs::srv::GetPlan::Response> response)
          {
            std::string reason, frontier_status;
            response->plan = previewGoal(request->goal, request->tolerance, reason, frontier_status);
          });
#ifdef NGD_GOAL_PREVIEW
      goal_check_service_ = node_->create_service<ngd_ros_wrapper::srv::PreviewGoal>(
          "/ego/check_goal",
          [this](const std::shared_ptr<ngd_ros_wrapper::srv::PreviewGoal::Request> request,
                 std::shared_ptr<ngd_ros_wrapper::srv::PreviewGoal::Response> response)
          {
            response->plan = previewGoal(request->goal, request->tolerance,
                                         response->rejection_reason, response->frontier_status);
          });
#endif
      waypoint_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
          goal_topic,        // was "/move_base_simple/goal"
          1,
          [this](const std::shared_ptr<const geometry_msgs::msg::PoseStamped> &msg)
          {
            this->waypointCallback(msg, false);
          });
      waypoint_3d_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
          goal_3d_topic, 1,
          [this](const std::shared_ptr<const geometry_msgs::msg::PoseStamped> &msg)
          {
            if (msg->header.frame_id != "map")
            {
              RCLCPP_WARN(node_->get_logger(), "3D goal ignored: expected the map frame.");
              return;
            }
            this->waypointCallback(msg, true);
          });
    }
    else if (target_type_ == TARGET_TYPE::PRESET_TARGET)
    {
      trigger_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
          "/traj_start_trigger",
          1,
          [this](const std::shared_ptr<const geometry_msgs::msg::PoseStamped> &msg)
          {
            this->triggerCallback(msg);
          });

      RCLCPP_INFO(node_->get_logger(), "Wait for 1 second.");
      int count = 0;
      while (rclcpp::ok() && count++ < 1000)
      {
        rclcpp::spin_some(node_);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }

      RCLCPP_WARN(node_->get_logger(), "Waiting for trigger from [n3ctrl] from RC");

      while (rclcpp::ok() && (!have_odom_ || !have_trigger_))
      {
        rclcpp::spin_some(node_);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }

      readGivenWps();
    }
    else
      cout << "Wrong target_type_ value! target_type_=" << target_type_ << endl;
  }

  void EGOReplanFSM::readGivenWps()

  {
    if (waypoint_num_ <= 0)
    {
      RCLCPP_ERROR(node_->get_logger(), "Wrong waypoint_num_ = %d", waypoint_num_);
      return;
    }

    wps_.resize(waypoint_num_);
    for (int i = 0; i < waypoint_num_; i++)
    {
      wps_[i](0) = waypoints_[i][0];
      wps_[i](1) = waypoints_[i][1];
      wps_[i](2) = waypoints_[i][2];
    }

    // 用 visualization_->displayGoalPoint() 方法对waypoint进行可视化
    for (size_t i = 0; i < (size_t)waypoint_num_; i++)
    {
      visualization_->displayGoalPoint(wps_[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // plan first global waypoint
    wp_id_ = 0;
    planNextWaypoint(wps_[wp_id_]);
  }

  void EGOReplanFSM::planNextWaypoint(const Eigen::Vector3d next_wp, bool restart, bool exact)
  {
    bool success = planGlobalRoute(next_wp, exact);
    if (success)
    {
      exact_global_goal_ = exact;
      end_pt_ = next_wp;

      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
      vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
      }

      end_vel_.setZero();
      have_target_ = true;
      have_new_target_ = true;   // forces polynomial init in reboundReplan

      // Internal preset waypoint advances share the original goal identity.
      const bool new_goal = restart || exec_state_ == WAIT_TARGET || exec_state_ == INIT;
      if (new_goal)
      {
        auto &last_traj_id = planner_manager_->local_data_.traj_id_;
        // Reserve a unique reference even when this goal never produces a spline.
        if (px4_offboard_.enabled())
          ++last_traj_id;
        goal_reference_traj_id_ = last_traj_id;

        if (restart || px4_offboard_.enabled())
        {
          std_msgs::msg::Int32 goal_notice;
          goal_notice.data = goal_reference_traj_id_;
          new_goal_pub_->publish(goal_notice);
        }
      }

      if (restart)
      {
        pending_swarm_publish_.reset();
        // A zero start time disables collision checks against the old trajectory.
        planner_manager_->local_data_.start_time_ = rclcpp::Time(0, 0, RCL_SYSTEM_TIME);
        flag_escape_emergency_ = true;

        // GEN_NEW_TRAJ plans from the latest odometry.
        changeFSMExecState(GEN_NEW_TRAJ, "NEW_GOAL");
      }
      else if (exec_state_ == WAIT_TARGET)
      {
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      }
      else
      {
        while (exec_state_ != EXEC_TRAJ)
        {
          rclcpp::spin_some(node_);
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        changeFSMExecState(REPLAN_TRAJ, "TRIG");
      }

      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory!");
    }
  }
  
  void EGOReplanFSM::triggerCallback(const std::shared_ptr<const geometry_msgs::msg::PoseStamped> &msg)
  {
    px4_offboard_.poll();
    have_trigger_ = true;
    if (px4_offboard_.enabled() && exec_state_ == WAIT_TARGET && !have_target_ && !wps_.empty())
    {
      wp_id_ = 0;
      planNextWaypoint(wps_[wp_id_]);
    }
    cout << "Triggered!" << endl;
    init_pt_ = odom_pos_;
  }

  void EGOReplanFSM::waypointCallback(const std::shared_ptr<const geometry_msgs::msg::PoseStamped> &msg, bool explicit_z)
  {
    if (!have_odom_)
    {
      RCLCPP_WARN(node_->get_logger(), "Goal ignored: no odometry yet.");
      return;
    }

    const double gx = msg->pose.position.x;
    const double gy = msg->pose.position.y;
    const double gz = msg->pose.position.z;

    if (!std::isfinite(gx) || !std::isfinite(gy) || !std::isfinite(gz))
    {
      RCLCPP_WARN(node_->get_logger(), "Goal ignored: non-finite position.");
      return;
    }

    if (!msg->header.frame_id.empty() && msg->header.frame_id != "map")
      RCLCPP_WARN(node_->get_logger(),
                  "Goal frame is '%s', planner works in 'map'. Position is used as-is.",
                  msg->header.frame_id.c_str());

    // z rule: RViz 2D goal gives z = 0 -> keep the current UAV height.
    const bool z_given = explicit_z || gz > goal_z_valid_min_;
    const double goal_z = z_given ? gz : odom_pos_(2);

    Eigen::Vector3d end_wp(gx, gy, goal_z);

    // reboundReplan() rejects start/target closer than 0.2 m and would retry forever.
    if ((end_wp - odom_pos_).norm() < 0.25)
    {
      RCLCPP_WARN(node_->get_logger(), "Goal ignored: closer than 0.25 m to the UAV.");
      return;
    }

    RCLCPP_INFO(node_->get_logger(),
                "New goal (%.2f, %.2f, %.2f) [z %s]", gx, gy, goal_z,
                z_given ? "from goal" : "= current UAV height");

    const auto map = planner_manager_->grid_map_;
    const bool exact = explicit_z && use_astar_global_ && map->useFreeSpaceOnly();
    if (explicit_z && !exact)
    {
      std::string reason, frontier_status;
      if (previewGoal(*msg, 0.0, reason, frontier_status).poses.empty())
      {
        RCLCPP_WARN(node_->get_logger(), "3D goal rejected: %s", reason.c_str());
        return;
      }
    }
    px4_offboard_.poll();
    init_pt_ = odom_pos_;
    planNextWaypoint(end_wp, true, exact);
  }

  void EGOReplanFSM::odometryCallback(const std::shared_ptr<const nav_msgs::msg::Odometry> &msg)
  {
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;

    // odom_acc_ = estimateAcc( msg );

    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;

    have_odom_ = true;
  }

  void EGOReplanFSM::BroadcastBsplineCallback(const std::shared_ptr<const traj_utils::msg::Bspline> &msg)
  {
    size_t id = msg->drone_id;
    if ((int)id == planner_manager_->pp_.drone_id)
      return;

    // if (abs((ros::Time::now() - msg->start_time).toSec()) > 0.25)
    rclcpp::Clock clock(RCL_SYSTEM_TIME);  // 确保使用当前节点的时间源
    auto msg_time = rclcpp::Time(msg->start_time, clock.get_clock_type());
    // RCLCPP_INFO(node_->get_logger(), "Clock type: %d", rclcpp::Clock().now().get_clock_type());
    // RCLCPP_INFO(node_->get_logger(), "Start time clock type: %d", rclcpp::Time(msg->start_time).get_clock_type());
    // RCLCPP_INFO(node_->get_logger(), "msg_time: %d", msg_time.get_clock_type());
    if (abs((rclcpp::Clock().now() - msg_time).seconds()) > 0.25)
    {
      // ROS_ERROR("Time difference is too large! Local - Remote Agent %d = %fs", msg->drone_id, (ros::Time::now() - msg->start_time).toSec());
      RCLCPP_ERROR(node_->get_logger(), "Time difference is too large! Local - Remote Agent %d = %fs",
                   msg->drone_id, (rclcpp::Clock().now() - msg_time).seconds());
      return;
    }

    // 路径缓冲区初始化
    if (planner_manager_->swarm_trajs_buf_.size() <= id)
    {
      for (size_t i = planner_manager_->swarm_trajs_buf_.size(); i <= id; i++)
      {
        OneTrajDataOfSwarm blank;
        blank.drone_id = -1;
        planner_manager_->swarm_trajs_buf_.push_back(blank);
      }
    }

    /* Test distance to the agent */
    Eigen::Vector3d cp0(msg->pos_pts[0].x, msg->pos_pts[0].y, msg->pos_pts[0].z);
    Eigen::Vector3d cp1(msg->pos_pts[1].x, msg->pos_pts[1].y, msg->pos_pts[1].z);
    Eigen::Vector3d cp2(msg->pos_pts[2].x, msg->pos_pts[2].y, msg->pos_pts[2].z);
    Eigen::Vector3d swarm_start_pt = (cp0 + 4 * cp1 + cp2) / 6;
    if ((swarm_start_pt - odom_pos_).norm() > planning_horizen_ * 4.0f / 3.0f)
    {
      planner_manager_->swarm_trajs_buf_[id].drone_id = -1;
      return; // if the current drone is too far to the received agent.
    }

    /* Store data */
    Eigen::MatrixXd pos_pts(3, msg->pos_pts.size());
    Eigen::VectorXd knots(msg->knots.size());
    for (size_t j = 0; j < msg->knots.size(); ++j)
    {
      knots(j) = msg->knots[j];
    }
    for (size_t j = 0; j < msg->pos_pts.size(); ++j)
    {
      pos_pts(0, j) = msg->pos_pts[j].x;
      pos_pts(1, j) = msg->pos_pts[j].y;
      pos_pts(2, j) = msg->pos_pts[j].z;
    }

    planner_manager_->swarm_trajs_buf_[id].drone_id = id;

    // 计算路径持续时间
    if (msg->order % 2)
    {
      double cutback = (double)msg->order / 2 + 1.5;
      planner_manager_->swarm_trajs_buf_[id].duration_ = msg->knots[msg->knots.size() - ceil(cutback)];
    }
    else
    {
      double cutback = (double)msg->order / 2 + 1.5;
      planner_manager_->swarm_trajs_buf_[id].duration_ = (msg->knots[msg->knots.size() - floor(cutback)] + msg->knots[msg->knots.size() - ceil(cutback)]) / 2;
    }

    // 生成bspline并存储
    UniformBspline pos_traj(pos_pts, msg->order, msg->knots[1] - msg->knots[0]);
    pos_traj.setKnot(knots);
    planner_manager_->swarm_trajs_buf_[id].position_traj_ = pos_traj;

    planner_manager_->swarm_trajs_buf_[id].start_pos_ = planner_manager_->swarm_trajs_buf_[id].position_traj_.evaluateDeBoorT(0);

    planner_manager_->swarm_trajs_buf_[id].start_time_ = msg->start_time;

    /* Check Collision */
    if (px4_offboard_.ready() && !pending_swarm_publish_ && planner_manager_->checkCollision(id))
    {
      changeFSMExecState(REPLAN_TRAJ, "TRAJ_CHECK");
    }
  }

  void EGOReplanFSM::swarmTrajsCallback(const std::shared_ptr<const traj_utils::msg::MultiBsplines> &msg)
  {

    multi_bspline_msgs_buf_.traj.clear();
    multi_bspline_msgs_buf_ = *msg;

    if (!have_odom_)
    {
      RCLCPP_ERROR(node_->get_logger(), "swarmTrajsCallback(): no odom!, return.");
      return;
    }

    if ((int)msg->traj.size() != msg->drone_id_from + 1) // drone_id must start from 0
    {
      RCLCPP_ERROR(node_->get_logger(), "Wrong trajectory size!msg->traj.size()=%d, msg->drone_id_from+1=%d", (int)msg->traj.size(), msg->drone_id_from + 1);
      return;
    }

    if (msg->traj[0].order != 3) // only support B-spline order equals 3.
    {
      RCLCPP_ERROR(node_->get_logger(), "Only support B-spline order equals 3.");
      return;
    }

    // Step 1. receive the trajectories
    planner_manager_->swarm_trajs_buf_.clear();
    planner_manager_->swarm_trajs_buf_.resize(msg->traj.size());

    // 处理每条路径
    for (size_t i = 0; i < msg->traj.size(); i++)
    {

      Eigen::Vector3d cp0(msg->traj[i].pos_pts[0].x, msg->traj[i].pos_pts[0].y, msg->traj[i].pos_pts[0].z);
      Eigen::Vector3d cp1(msg->traj[i].pos_pts[1].x, msg->traj[i].pos_pts[1].y, msg->traj[i].pos_pts[1].z);
      Eigen::Vector3d cp2(msg->traj[i].pos_pts[2].x, msg->traj[i].pos_pts[2].y, msg->traj[i].pos_pts[2].z);
      Eigen::Vector3d swarm_start_pt = (cp0 + 4 * cp1 + cp2) / 6;
      if ((swarm_start_pt - odom_pos_).norm() > planning_horizen_ * 4.0f / 3.0f)
      {
        planner_manager_->swarm_trajs_buf_[i].drone_id = -1;
        continue;
      }

      // 存储路径控制点和节点
      Eigen::MatrixXd pos_pts(3, msg->traj[i].pos_pts.size());
      Eigen::VectorXd knots(msg->traj[i].knots.size());
      for (size_t j = 0; j < msg->traj[i].knots.size(); ++j)
      {
        knots(j) = msg->traj[i].knots[j];
      }
      for (size_t j = 0; j < msg->traj[i].pos_pts.size(); ++j)
      {
        pos_pts(0, j) = msg->traj[i].pos_pts[j].x;
        pos_pts(1, j) = msg->traj[i].pos_pts[j].y;
        pos_pts(2, j) = msg->traj[i].pos_pts[j].z;
      }

      planner_manager_->swarm_trajs_buf_[i].drone_id = i;

      // 计算路径持续时间
      if (msg->traj[i].order % 2)
      {
        double cutback = (double)msg->traj[i].order / 2 + 1.5;
        planner_manager_->swarm_trajs_buf_[i].duration_ = msg->traj[i].knots[msg->traj[i].knots.size() - ceil(cutback)];
      }
      else
      {
        double cutback = (double)msg->traj[i].order / 2 + 1.5;
        planner_manager_->swarm_trajs_buf_[i].duration_ = (msg->traj[i].knots[msg->traj[i].knots.size() - floor(cutback)] + msg->traj[i].knots[msg->traj[i].knots.size() - ceil(cutback)]) / 2;
      }

      // planner_manager_->swarm_trajs_buf_[i].position_traj_ =
      UniformBspline pos_traj(pos_pts, msg->traj[i].order, msg->traj[i].knots[1] - msg->traj[i].knots[0]);
      pos_traj.setKnot(knots);
      planner_manager_->swarm_trajs_buf_[i].position_traj_ = pos_traj;

      planner_manager_->swarm_trajs_buf_[i].start_pos_ = planner_manager_->swarm_trajs_buf_[i].position_traj_.evaluateDeBoorT(0);

      planner_manager_->swarm_trajs_buf_[i].start_time_ = msg->traj[i].start_time;
    }

    have_recv_pre_agent_ = true;
  }

  void EGOReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
  {

    if (new_state == exec_state_)
      continously_called_times_++;
    else
      continously_called_times_ = 1;

    static string state_str[8] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP", "SEQUENTIAL_START"};
    int pre_s = int(exec_state_);
    exec_state_ = new_state;
    cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
  }

  std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> EGOReplanFSM::timesOfConsecutiveStateCalls()
  {
    return std::pair<int, FSM_EXEC_STATE>(continously_called_times_, exec_state_);
  }

  void EGOReplanFSM::printFSMExecState()
  {
    static string state_str[8] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP", "SEQUENTIAL_START"};

    cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
  }

  void EGOReplanFSM::cancelOffboardGoal()
  {
    have_target_ = false;
    have_new_target_ = false;
    have_trigger_ = false;
    pending_swarm_publish_.reset();
    planner_manager_->local_data_.start_time_ = rclcpp::Time(0, 0, RCL_SYSTEM_TIME);
    if (exec_state_ != INIT)
      changeFSMExecState(WAIT_TARGET, "OFFBOARD_LOST");
  }

  void EGOReplanFSM::execFSMCallback()
  {
    exec_timer_->cancel(); // To avoid blockage
    px4_offboard_.poll();

    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 100)
    {
      printFSMExecState();
      if (!have_odom_)
        cout << "no odom." << endl;
      if (!have_target_)
        cout << "wait for goal or trigger." << endl;
      fsm_num = 0;
    }

    switch (exec_state_)
    {
    case INIT:
    {
      if (!have_odom_)
      {
        goto force_return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET:
    {
      if (!have_target_ || !have_trigger_)
        goto force_return;
      else
      {
        changeFSMExecState(SEQUENTIAL_START, "FSM");
      }
      break;
    }

    case SEQUENTIAL_START: // for swarm
    {
      if (planner_manager_->pp_.drone_id <= 0 || (planner_manager_->pp_.drone_id >= 1 && have_recv_pre_agent_))
      {
        if (have_odom_ && have_target_ && have_trigger_)
        {
          bool success = planFromGlobalTraj(10); // zx-todo
          if (success)
          {
            changeFSMExecState(EXEC_TRAJ, "FSM");

            publishSwarmTrajs(true);
          }
          else
          {
            RCLCPP_ERROR(node_->get_logger(), "Failed to generate the first trajectory!!!");
            changeFSMExecState(SEQUENTIAL_START, "FSM");
          }
        }
        else
        {
          RCLCPP_ERROR(node_->get_logger(), "No odom or no target! have_odom_=%d, have_target_=%d", have_odom_, have_target_);
        }
      }

      break;
    }

    case GEN_NEW_TRAJ:
    {

      bool success = planFromGlobalTraj(10); // zx-todo
      if (success)
      {
        changeFSMExecState(EXEC_TRAJ, "FSM");
        flag_escape_emergency_ = true;
        publishSwarmTrajs(false);
      }
      else
      {
        if (continously_called_times_ > 200)
        {
          RCLCPP_ERROR(node_->get_logger(), "Planning keeps failing, dropping the goal. Send a new goal.");
          have_target_ = false;
          changeFSMExecState(WAIT_TARGET, "GIVEUP");
        }
        else
        {
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        }
      }
      break;
    }

    case REPLAN_TRAJ:
    {

      if (planFromCurrentTraj(1))
      {
        changeFSMExecState(EXEC_TRAJ, "FSM");
        publishSwarmTrajs(false);
      }
      else
      {
        static rclcpp::Time last_route_time(0, 0, RCL_SYSTEM_TIME);
        const rclcpp::Time now_t = rclcpp::Clock().now();
        if (use_astar_global_ &&
            planner_manager_->grid_map_->useFreeSpaceOnly() &&
            continously_called_times_ % 5 == 0 &&
            (now_t - last_route_time).seconds() > 1.0)
        {
          last_route_time = now_t;
          planGlobalRoute(end_pt_, exact_global_goal_);
        }

        if (continously_called_times_ > 200)   // give up instead of retrying forever
        {
          RCLCPP_ERROR(node_->get_logger(), "Replan keeps failing, dropping the goal. Send a new goal.");
          have_target_ = false;
          changeFSMExecState(WAIT_TARGET, "GIVEUP");
        }
        else
        {
          changeFSMExecState(REPLAN_TRAJ, "FSM");
        }
      }
      break;
    }

    case EXEC_TRAJ:
    {
      if (!px4_offboard_.ready())
        break;

      auto &start = planner_manager_->local_data_.start_time_;
      start = px4_offboard_.executionStart(start);
      if (pending_swarm_publish_)
      {
        publishSwarmTrajs(*pending_swarm_publish_);
        pending_swarm_publish_.reset();
      }

      static int blk_cnt = 0;
      if (use_astar_global_ && planner_manager_->grid_map_->useFreeSpaceOnly() && ++blk_cnt >= 50)
      {
        blk_cnt = 0;
        if (globalPathBlocked())
        {
          RCLCPP_WARN(node_->get_logger(), "Global path blocked, re-routing");
          if (planGlobalRoute(end_pt_, exact_global_goal_))
          {
            have_new_target_ = true;
            changeFSMExecState(REPLAN_TRAJ, "GLOBAL");
            break;
          }
        }
      }
      /* determine if need to replan */
      LocalTrajData *info = &planner_manager_->local_data_;
      rclcpp::Time time_now = rclcpp::Clock().now();
      double t_cur = (time_now - info->start_time_).seconds();
      t_cur = std::min(info->duration_, t_cur);

      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur);

      /* && (end_pt_ - pos).norm() < 0.5 */
      if ((target_type_ == TARGET_TYPE::PRESET_TARGET) &&
          (wp_id_ < waypoint_num_ - 1) &&
          (end_pt_ - pos).norm() < no_replan_thresh_)
      {
        wp_id_++;
        planNextWaypoint(wps_[wp_id_]);
      }
      else if ((local_target_pt_ - end_pt_).norm() < 1e-3) // close to the global target
      {
        if (t_cur > info->duration_ - 1e-2)
        {
          have_target_ = false;
          have_trigger_ = false;

          if (target_type_ == TARGET_TYPE::PRESET_TARGET)
          {
            wp_id_ = 0;
            planNextWaypoint(wps_[wp_id_]);
          }

          changeFSMExecState(WAIT_TARGET, "FSM");
          goto force_return;
        }
        else if (!dynamic_planning_horizon_ &&
                 (end_pt_ - pos).norm() > no_replan_thresh_ && t_cur > replan_thresh_)
        {
          changeFSMExecState(REPLAN_TRAJ, "FSM");
        }
      }
      else if (t_cur > replan_thresh_)
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }

      break;
    }

    case EMERGENCY_STOP:
    {

      if (flag_escape_emergency_) // Avoiding repeated calls
      {
        callEmergencyStop(odom_pos_);
      }
      else
      {
        if (enable_fail_safe_ && odom_vel_.norm() < 0.1)
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }

      flag_escape_emergency_ = false;
      break;
    }
    }

    data_disp_.header.stamp = rclcpp::Clock().now();
    data_disp_pub_->publish(data_disp_);

  force_return:;
    // exec_timer_.start();
    if (exec_timer_ && exec_timer_->is_canceled())
    {
      // 取消状态下无需重新创建，可以复用现有计时器
      exec_timer_->reset();
    }
  }

  bool EGOReplanFSM::planFromGlobalTraj(const int trial_times /*=1*/) // zx-todo
  {
    start_pt_ = odom_pos_;
    start_vel_ = odom_vel_;
    start_acc_.setZero();

    bool flag_random_poly_init;
    if (timesOfConsecutiveStateCalls().first == 1)
      flag_random_poly_init = false;
    else
      flag_random_poly_init = true;

    if (dynamic_planning_horizon_)
      return callReboundReplan(true, false);

    for (int i = 0; i < trial_times; i++)
    {
      if (callReboundReplan(true, flag_random_poly_init))
      {
        return true;
      }
    }
    return false;
  }

  bool EGOReplanFSM::planFromCurrentTraj(const int trial_times /*=1*/)
  {

    LocalTrajData *info = &planner_manager_->local_data_;
    // ros::Time time_now = ros::Time::now();
    auto time_now = rclcpp::Clock().now();
    // double t_cur = (time_now - info->start_time_).toSec();
    double t_cur = (time_now - info->start_time_).seconds();

    start_pt_ = info->position_traj_.evaluateDeBoorT(t_cur);
    start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
    start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

    if (dynamic_planning_horizon_)
      return callReboundReplan(false, false);

    bool success = callReboundReplan(false, false);

    if (!success)
    {
      success = callReboundReplan(true, false);
      if (!success)
      {
        for (int i = 0; i < trial_times; i++)
        {
          success = callReboundReplan(true, true);
          if (success)
            break;
        }
        if (!success)
        {
          return false;
        }
      }
    }

    return true;
  }

  void EGOReplanFSM::checkCollisionCallback()
  {

    LocalTrajData *info = &planner_manager_->local_data_;
    auto map = planner_manager_->grid_map_;
    
    px4_offboard_.poll();
    if (!px4_offboard_.ready() || pending_swarm_publish_ ||
        exec_state_ == WAIT_TARGET || info->start_time_.seconds() < 1e-5)
      return;

    /* ---------- check lost of depth ---------- */
    if (map->getOdomDepthTimeout())
    {
      RCLCPP_ERROR(node_->get_logger(), "Depth Lost! EMERGENCY_STOP");

      enable_fail_safe_ = false;
      changeFSMExecState(EMERGENCY_STOP, "SAFETY");
    }

    /* ---------- check trajectory ---------- */
    const double t_cur = (rclcpp::Clock().now() - info->start_time_).seconds();
    if (t_cur > info->duration_)
      return;
    const double fraction_end = info->duration_ * planner_manager_->getCollisionCheckFraction();
    const double end = t_cur < fraction_end ? fraction_end : info->duration_;
    const double speed_bound = info->velocity_traj_.getMaxControlPointNorm();
    if (!std::isfinite(end) || !std::isfinite(t_cur) || !std::isfinite(speed_bound))
    {
      changeFSMExecState(EMERGENCY_STOP, "INVALID_TRAJECTORY");
      return;
    }
    const double time_step = std::min(0.01, 0.5 * map->getResolution() / std::max(speed_bound, 1e-6));
    const double CLEARANCE = planner_manager_->getSwarmClearance();

    for (double t = std::max(0.0, t_cur);; t = std::min(t + time_step, end))
    {
      const Eigen::Vector3d position = info->position_traj_.evaluateDeBoorT(t);
      bool occ = !position.allFinite() || map->getInflateOccupancy(position);

      for (size_t id = 0; id < planner_manager_->swarm_trajs_buf_.size(); id++)
      {
        if ((planner_manager_->swarm_trajs_buf_.at(id).drone_id != (int)id) || (planner_manager_->swarm_trajs_buf_.at(id).drone_id == planner_manager_->pp_.drone_id))
        {
          continue;
        }

        auto &other = planner_manager_->swarm_trajs_buf_.at(id);
        const double other_time = (info->start_time_ - other.start_time_).seconds() + t;
        if (other_time < 0.0 || other_time > other.duration_)
          continue;
        const double dist = (position - other.position_traj_.evaluateDeBoorT(other_time)).norm();

        if (dist < CLEARANCE)
        {
          occ = true;
          break;
        }
      }

      if (occ)
      {

        if (planFromCurrentTraj()) // Make a chance
        {
          changeFSMExecState(EXEC_TRAJ, "SAFETY");
          publishSwarmTrajs(false);
          return;
        }
        else
        {
          if (t - t_cur < emergency_time_) // 0.8s of emergency time
          {
            RCLCPP_WARN(node_->get_logger(), "Suddenly discovered obstacles. emergency stop! time=%f", t - t_cur);

            changeFSMExecState(EMERGENCY_STOP, "SAFETY");
          }
          else
          {
            RCLCPP_WARN(node_->get_logger(), "current traj in collision, replan.");
            changeFSMExecState(REPLAN_TRAJ, "SAFETY");
          }
          return;
        }
        break;
      }
      if (t >= end)
        break;
    }
  }

  bool EGOReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj)
  {

    bool plan_and_refine_success;
    if (dynamic_planning_horizon_)
    {
      plan_and_refine_success = planDynamicTrajectory();
    }
    else
    {
      getLocalTarget();
      plan_and_refine_success = planner_manager_->reboundReplan(
          start_pt_, start_vel_, start_acc_, local_target_pt_, local_target_vel_,
          have_new_target_ || flag_use_poly_init, flag_randomPolyTraj);
    }
    have_new_target_ = false;

    cout << "refine_success=" << plan_and_refine_success << endl;

    if (plan_and_refine_success)
    {

      auto info = &planner_manager_->local_data_;

      traj_utils::msg::Bspline bspline;
      bspline.order = 3;
      bspline.start_time = info->start_time_;
      bspline.traj_id = info->traj_id_;
      bspline.goal_reference_traj_id = goal_reference_traj_id_;

      Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
      bspline.pos_pts.reserve(pos_pts.cols());
      for (int i = 0; i < pos_pts.cols(); ++i)
      {
        geometry_msgs::msg::Point pt;
        pt.x = pos_pts(0, i);
        pt.y = pos_pts(1, i);
        pt.z = pos_pts(2, i);
        bspline.pos_pts.push_back(pt);
      }

      Eigen::VectorXd knots = info->position_traj_.getKnot();

      bspline.knots.reserve(knots.rows());
      for (int i = 0; i < knots.rows(); ++i)
      {
        bspline.knots.push_back(knots(i));
      }

      /* 1. publish traj to traj_server */
      bspline_pub_->publish(bspline);

      /* 2. publish traj to the next drone of swarm */

      /* 3. publish traj for visualization */
      visualization_->displayOptimalList(info->position_traj_.get_control_points(), 0);
    }

    return plan_and_refine_success;
  }

  void EGOReplanFSM::publishSwarmTrajs(bool startup_pub)
  {
    if (!px4_offboard_.ready())
    {
      pending_swarm_publish_ = startup_pub;
      return;
    }

    auto info = &planner_manager_->local_data_;

    traj_utils::msg::Bspline bspline;
    bspline.order = 3;
    bspline.start_time = info->start_time_;
    bspline.drone_id = planner_manager_->pp_.drone_id;
    bspline.traj_id = info->traj_id_;
    bspline.goal_reference_traj_id = goal_reference_traj_id_;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    bspline.pos_pts.reserve(pos_pts.cols());
    for (int i = 0; i < pos_pts.cols(); ++i)
    {
      geometry_msgs::msg::Point pt;
      pt.x = pos_pts(0, i);
      pt.y = pos_pts(1, i);
      pt.z = pos_pts(2, i);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();

    bspline.knots.reserve(knots.rows());
    for (int i = 0; i < knots.rows(); ++i)
    {
      bspline.knots.push_back(knots(i));
    }

    if (startup_pub)
    {
      multi_bspline_msgs_buf_.drone_id_from = planner_manager_->pp_.drone_id; // zx-todo
      if ((int)multi_bspline_msgs_buf_.traj.size() == planner_manager_->pp_.drone_id + 1)
      {
        multi_bspline_msgs_buf_.traj.back() = bspline;
      }
      else if ((int)multi_bspline_msgs_buf_.traj.size() == planner_manager_->pp_.drone_id)
      {
        multi_bspline_msgs_buf_.traj.push_back(bspline);
      }
      else
      {
        RCLCPP_ERROR(node_->get_logger(), "Wrong traj nums and drone_id pair!!! traj.size()=%d, drone_id=%d", (int)multi_bspline_msgs_buf_.traj.size(), planner_manager_->pp_.drone_id);
        // return plan_and_refine_success;
      }
      // swarm_trajs_pub_.publish(multi_bspline_msgs_buf_);
      swarm_trajs_pub_->publish(multi_bspline_msgs_buf_);
    }

    broadcast_bspline_pub_->publish(bspline);
  }

  nav_msgs::msg::Path EGOReplanFSM::previewGoal(const geometry_msgs::msg::PoseStamped &goal,
                                              double tolerance, std::string &rejection_reason,
                                              std::string &frontier_status)
  {
    using namespace std::chrono_literals;
    auto map = planner_manager_->grid_map_;
    const bool known_free_only = map->useFreeSpaceOnly();
    const bool global_astar = use_astar_global_ && known_free_only;
    if (goal.header.frame_id != "map")
      rejection_reason = "The goal must be in the map frame. Set RViz Fixed Frame to map.";
    else if (!have_odom_ || !odom_pos_.allFinite())
      rejection_reason = "Waiting for valid drone odometry.";
    else if (map->waitingForOctomap())
      rejection_reason = "Waiting for a usable full OctoMap.";
    else if (!map->isInMap(odom_pos_))
      rejection_reason = "The drone's current position is outside the planner map bounds.";
    else if (map->getInflateOccupancy(odom_pos_) > 0 &&
             (!known_free_only || !map->isUnseen(odom_pos_)))
      rejection_reason = "The drone's current position is in occupied space or inside obstacle clearance.";
    if (!rejection_reason.empty())
      return {};

    const auto &p = goal.pose.position;
    const Eigen::Vector3d requested(p.x, p.y, p.z);
    if (!requested.allFinite())
    {
      rejection_reason = "The goal coordinates must be finite.";
      return {};
    }

    const auto now = std::chrono::steady_clock::now();
    const auto deadline = now + 250ms;
    std::vector<Eigen::Vector3d> path;
    bool search_limit_hit = false;
    if (!map->isInMap(requested))
      rejection_reason = "The point is outside the planner map bounds.";
    else if (map->getOccupancy(requested) > 0)
      rejection_reason = "The goal voxel is occupied in the map. Move the marker into free space.";
    else if (known_free_only ? map->isUnseen(requested) : map->isUnknown(requested))
      rejection_reason = "The goal voxel is unscanned. Move the marker into mapped free space.";
    else if (map->getInflateOccupancy(requested) > 0)
      rejection_reason = "The goal voxel is inside inflated obstacle clearance. Move the marker farther from the obstacle.";
    else if ((requested - odom_pos_).norm() < 0.25)
      rejection_reason = "The goal is less than 0.25 m from the drone.";
    else if (global_astar && !map->searchFreePath(odom_pos_, requested, path, true, deadline, &search_limit_hit))
      rejection_reason = search_limit_hit ?
          "The route search reached its time or node limit before verifying a path." :
          "The goal voxel is free, but no connected known-free route was found from the drone.";

    // A single pose confirms the destination alone in local planning mode.
    if (!global_astar && rejection_reason.empty())
      path.push_back(requested);

    if (!rejection_reason.empty())
    {
      if (!(tolerance > 0.0))
        return {};
      if (std::chrono::steady_clock::now() >= deadline)
      {
        frontier_status = "The preview time limit expired before a frontier could be checked.";
        return {};
      }
      if (!frontier_centers_ || frontier_centers_->header.frame_id != "map" ||
          now - frontier_received_ > 3s)
      {
        frontier_status = "No recent OctoMap frontier data is available.";
        return {};
      }

      std::vector<Eigen::Vector3d> candidates;
      for (const auto &pose : frontier_centers_->poses)
      {
        const auto &c = pose.position;
        const Eigen::Vector3d center(c.x, c.y, c.z);
        if (!center.allFinite() || !map->isInMap(center))
          continue;
        const bool free = known_free_only ? map->isKnownFreePosition(center) :
            !map->isUnknown(center) && map->getOccupancy(center) == 0 && map->getInflateOccupancy(center) == 0;
        if (free && (center - requested).norm() <= tolerance &&
            (center - odom_pos_).norm() >= 0.25)
          candidates.push_back(center);
      }
      if (candidates.empty())
      {
        frontier_status = "No frontier center passed the known-free and minimum-distance checks.";
        return {};
      }
      std::sort(candidates.begin(), candidates.end(), [&](const auto &a, const auto &b) {
        return (a - requested).squaredNorm() < (b - requested).squaredNorm();
      });
      bool frontier_limit_hit = false;
      for (const auto &candidate : candidates)
      {
        if (!global_astar)
        {
          path.push_back(candidate);
          // Existing goal tools show the rejection reason beside the suggestion.
          rejection_reason += " Route unverified for the proposed frontier in local planning mode.";
          break;
        }
        if (std::chrono::steady_clock::now() >= deadline)
          break;
        if (map->searchFreePath(odom_pos_, candidate, path, true, deadline, &search_limit_hit))
          break;
        frontier_limit_hit |= search_limit_hit;
      }
      if (path.empty())
      {
        frontier_status = frontier_limit_hit || std::chrono::steady_clock::now() >= deadline ?
            "The frontier search reached its time or node limit before finding an alternative." :
            "No frontier center was connected to the drone through known free space.";
        return {};
      }
    }

    nav_msgs::msg::Path plan;
    plan.header.frame_id = "map";
    plan.header.stamp = node_->now();
    for (const auto &point : path)
    {
      geometry_msgs::msg::PoseStamped waypoint;
      waypoint.header = plan.header;
      waypoint.pose.position.x = point.x();
      waypoint.pose.position.y = point.y();
      waypoint.pose.position.z = point.z();
      waypoint.pose.orientation.w = 1.0;
      plan.poses.push_back(waypoint);
    }
    return plan;
  }

  bool EGOReplanFSM::planGlobalRoute(const Eigen::Vector3d &goal, bool exact)
  {
    auto map = planner_manager_->grid_map_;
    if (use_astar_global_ && map->useFreeSpaceOnly())
    {
      std::vector<Eigen::Vector3d> path;
      const bool reached = map->searchFreePath(odom_pos_, goal, path, exact);
      if (exact && !reached)
      {
        RCLCPP_WARN(node_->get_logger(), "3D goal is no longer reachable through known free space.");
        return false;
      }
      if (!reached && path.size() >= 2 && (path.back() - odom_pos_).norm() < 0.5)
      {
        RCLCPP_ERROR(node_->get_logger(),
                     "Drone is boxed in (start blocked or no free cells around it). Move it to open space.");
        return false;
      }
      if (path.size() >= 2)
      {
        if (!reached)
          RCLCPP_WARN(node_->get_logger(),
                      "Goal not reachable through known free space, routing to the closest reachable point");
        std::vector<Eigen::Vector3d> wps(path.begin() + 1, path.end());
        if (planner_manager_->planGlobalTrajWaypoints(
                odom_pos_, odom_vel_, Eigen::Vector3d::Zero(),
                wps, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()))
        {
          setReferenceRoute(path);
          RCLCPP_INFO(node_->get_logger(), "Free-space A* route: %zu waypoints", wps.size());
          return true;
        }
      }
      if (exact)
        return false;
      RCLCPP_WARN(node_->get_logger(), "Free-space A* failed, using straight line");
    }
    const bool planned = planner_manager_->planGlobalTraj(
        odom_pos_, odom_vel_, Eigen::Vector3d::Zero(),
        goal, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    if (planned)
      setReferenceRoute({odom_pos_, goal});
    return planned;
  }

  void EGOReplanFSM::setReferenceRoute(const std::vector<Eigen::Vector3d> &route)
  {
    if (!dynamic_planning_horizon_)
      return;
    reference_route_ = route;
    reference_segment_ = 0;
    failed_full_start_.reset();
  }

  bool EGOReplanFSM::planDynamicTrajectory()
  {
    if (reference_route_.size() < 2)
      return false;

    // Start at the closest point on the remaining route. Progress only advances
    // after a successful plan so a rejected candidate cannot skip waypoints.
    size_t segment = reference_segment_;
    Eigen::Vector3d nearest = reference_route_[segment];
    double best_distance = std::numeric_limits<double>::infinity();
    for (size_t i = segment; i + 1 < reference_route_.size(); ++i)
    {
      const Eigen::Vector3d &a = reference_route_[i];
      const Eigen::Vector3d edge = reference_route_[i + 1] - a;
      if (edge.squaredNorm() < 1e-12)
        continue;
      const double u = std::clamp((start_pt_ - a).dot(edge) / edge.squaredNorm(), 0.0, 1.0);
      const Eigen::Vector3d point = a + u * edge;
      const double distance = (point - start_pt_).squaredNorm();
      if (distance < best_distance)
      {
        best_distance = distance;
        nearest = point;
        segment = i;
      }
    }

    std::vector<Eigen::Vector3d> remaining{start_pt_, nearest};
    remaining.insert(remaining.end(), reference_route_.begin() + segment + 1, reference_route_.end());
    auto map = planner_manager_->grid_map_;
    const auto now = std::chrono::steady_clock::now();
    const bool try_full = !failed_full_start_ || (start_pt_ - *failed_full_start_).norm() >= 0.5 ||
        now - failed_full_time_ >= std::chrono::seconds(2);
    const double full = std::numeric_limits<double>::infinity();
    const double first_limit = try_full ? full : planning_horizen_;
    const double min_target_distance = 0.2;
    double previous_length = -1.0;
    for (const double limit : {first_limit, planning_horizen_, planning_horizen_ * 0.5, planning_horizen_ * 0.25})
    {
      std::vector<Eigen::Vector3d> seed{start_pt_};
      double length = 0.0;
      for (size_t i = 1; i < remaining.size() && length < limit; ++i)
      {
        const Eigen::Vector3d edge = remaining[i] - remaining[i - 1];
        const double edge_length = edge.norm();
        if (edge_length <= 1e-6)
          continue;
        const double take = std::min(edge_length, limit - length);
        seed.push_back(remaining[i - 1] + (take / edge_length) * edge);
        length += take;
      }

      const bool complete = (seed.back() - end_pt_).norm() < 1e-6;
      if (std::abs(length - previous_length) < 1e-6 || (!try_full && complete))
        continue;
      previous_length = length;
      if ((seed.back() - start_pt_).norm() >= min_target_distance &&
          map->getInflateOccupancy(seed.back()) == 0 &&
          planner_manager_->reboundReplan(
              start_pt_, start_vel_, start_acc_, seed.back(), Eigen::Vector3d::Zero(), true, false, seed))
      {
        local_target_pt_ = seed.back();
        local_target_vel_.setZero();
        reference_segment_ = segment;
        if (complete)
          failed_full_start_.reset();
        RCLCPP_INFO(node_->get_logger(), "Dynamic horizon: optimized %s (%.2f m reference)",
                    complete ? "complete route to goal" : "shorter route segment", length);
        return true;
      }
      if (complete)
      {
        failed_full_start_ = start_pt_;
        failed_full_time_ = now;
      }
    }
    return false;
  }

  bool EGOReplanFSM::globalPathBlocked()
  {
    auto &gd = planner_manager_->global_data_;
    auto map = planner_manager_->grid_map_;
    double travelled = 0.0;
    Eigen::Vector3d last = odom_pos_;
    for (double t = gd.last_progress_time_; t < gd.global_duration_ && travelled < planning_horizen_; t += 0.05)
    {
      const Eigen::Vector3d p = gd.getPosition(t);
      travelled += (p - last).norm();
      last = p;
      if ((p - odom_pos_).norm() < 0.3)
        continue;   // ignore the voxels around the body
      if (map->getInflateOccupancy(p) != 0)
        return true;
    }
    return false;
  }

  bool EGOReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos)
  {

    planner_manager_->EmergencyStop(stop_pos);

    auto info = &planner_manager_->local_data_;

    /* publish traj */
    traj_utils::msg::Bspline bspline;
    bspline.order = 3;
    bspline.start_time = info->start_time_;
    bspline.traj_id = info->traj_id_;
    bspline.goal_reference_traj_id = goal_reference_traj_id_;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    bspline.pos_pts.reserve(pos_pts.cols());
    for (int i = 0; i < pos_pts.cols(); ++i)
    {
      geometry_msgs::msg::Point pt;
      pt.x = pos_pts(0, i);
      pt.y = pos_pts(1, i);
      pt.z = pos_pts(2, i);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();
    bspline.knots.reserve(knots.rows());
    for (int i = 0; i < knots.rows(); ++i)
    {
      bspline.knots.push_back(knots(i));
    }

    bspline_pub_->publish(bspline);

    return true;
  }

  void EGOReplanFSM::getLocalTarget()
  {
    double t;

    double t_step = planning_horizen_ / 20 / planner_manager_->pp_.max_vel_;
    double dist_min = 9999, dist_min_t = 0.0;
    for (t = planner_manager_->global_data_.last_progress_time_; t < planner_manager_->global_data_.global_duration_; t += t_step)
    {
      Eigen::Vector3d pos_t = planner_manager_->global_data_.getPosition(t);
      double dist = (pos_t - start_pt_).norm();

      if (t < planner_manager_->global_data_.last_progress_time_ + 1e-5 && dist > planning_horizen_)
      {
        // Important cornor case!
        for (; t < planner_manager_->global_data_.global_duration_; t += t_step)
        {
          Eigen::Vector3d pos_t_temp = planner_manager_->global_data_.getPosition(t);
          double dist_temp = (pos_t_temp - start_pt_).norm();
          if (dist_temp < planning_horizen_)
          {
            pos_t = pos_t_temp;
            dist = (pos_t - start_pt_).norm();
            cout << "Escape cornor case \"getLocalTarget\"" << endl;
            break;
          }
        }
      }

      if (dist < dist_min)
      {
        dist_min = dist;
        dist_min_t = t;
      }

      if (dist >= planning_horizen_)
      {
        local_target_pt_ = pos_t;
        planner_manager_->global_data_.last_progress_time_ = dist_min_t;
        break;
      }
    }
    if (t > planner_manager_->global_data_.global_duration_) // Last global point
    {
      local_target_pt_ = end_pt_;
      planner_manager_->global_data_.last_progress_time_ = planner_manager_->global_data_.global_duration_;
    }

    if ((end_pt_ - local_target_pt_).norm() < (planner_manager_->pp_.max_vel_ * planner_manager_->pp_.max_vel_) / (2 * planner_manager_->pp_.max_acc_))
    {
      local_target_vel_ = Eigen::Vector3d::Zero();
    }
    else
    {
      local_target_vel_ = planner_manager_->global_data_.getVelocity(t);
    }

    // free-space-only: pull the local target back only if it is in UNSEEN space
    auto map = planner_manager_->grid_map_;
    if (map->useFreeSpaceOnly() && map->isUnseen(local_target_pt_))
    {
      Eigen::Vector3d dir = start_pt_ - local_target_pt_;
      const double len = dir.norm();
      if (len > 1e-3)
      {
        dir /= len;
        const double res = map->getResolution();
        for (double d = res; d < len; d += res)
        {
          Eigen::Vector3d p = local_target_pt_ + dir * d;
          if (map->getInflateOccupancy(p) == 0)
          {
            local_target_pt_ = p;
            local_target_vel_.setZero();
            break;
          }
        }
      }
    }
  }

} // namespace ego_planner
