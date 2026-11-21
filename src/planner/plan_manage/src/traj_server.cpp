#include "bspline_opt/uniform_bspline.h"
#include "nav_msgs/msg/odometry.hpp"
#include "traj_utils/msg/bspline.hpp"
#include "quadrotor_msgs/msg/position_command.hpp"
#include "std_msgs/msg/empty.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/int32.hpp"
#include "ego_planner/px4_offboard.h"

rclcpp::Time time_last_(0, 0, RCL_SYSTEM_TIME);
double odom_yaw_ = 0.0;
bool reset_on_next_traj_ = true;               // true so the first trajectory also starts clean
int64_t min_valid_traj_id_ = 0;
int64_t goal_reference_traj_id_ = -1;
int64_t canceled_goal_reference_traj_id_ = -1;
rclcpp::Publisher<quadrotor_msgs::msg::PositionCommand>::SharedPtr pos_cmd_pub;

quadrotor_msgs::msg::PositionCommand cmd;
double pos_gain[3] = {0, 0, 0};
double vel_gain[3] = {0, 0, 0};

using ego_planner::UniformBspline;

bool receive_traj_ = false;
traj_utils::msg::Bspline::ConstSharedPtr pending_traj_;
vector<UniformBspline> traj_;
double traj_duration_;
rclcpp::Time start_time_;
int64_t traj_id_ = -1;

// yaw control
double last_yaw_, last_yaw_dot_;
double time_forward_;

void odomCallback(nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  const auto &q = msg->pose.pose.orientation;
  odom_yaw_ = atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

void clearTrajectory()
{
  receive_traj_ = false;
  traj_.clear();
  pending_traj_.reset();
  reset_on_next_traj_ = true;
}

void cancelOffboardGoal()
{
  canceled_goal_reference_traj_id_ = goal_reference_traj_id_;
  clearTrajectory();
}

void newGoalCallback(std_msgs::msg::Int32::ConstSharedPtr msg, ego_planner::Px4Offboard &offboard)
{
  offboard.poll();
  if (msg->data < min_valid_traj_id_ || msg->data < traj_id_)
    return;

  if (!offboard.ready() || reset_on_next_traj_)
    clearTrajectory();
  min_valid_traj_id_ = static_cast<int64_t>(msg->data) + 1;
  goal_reference_traj_id_ = msg->data;
}

void activateSpline(const traj_utils::msg::Bspline::ConstSharedPtr &msg);

void bsplineCallback(traj_utils::msg::Bspline::ConstSharedPtr msg, ego_planner::Px4Offboard &offboard)
{
  offboard.poll();
  if (msg->traj_id < min_valid_traj_id_ || msg->traj_id <= traj_id_ ||
      (pending_traj_ && msg->traj_id <= pending_traj_->traj_id))
    return;
  if (offboard.enabled())
  {
    if (msg->goal_reference_traj_id <= canceled_goal_reference_traj_id_ ||
        msg->goal_reference_traj_id < goal_reference_traj_id_)
      return;

    if (msg->goal_reference_traj_id > goal_reference_traj_id_ &&
        (!offboard.ready() || reset_on_next_traj_))
      clearTrajectory();
    goal_reference_traj_id_ = msg->goal_reference_traj_id;
  }

  if (rclcpp::Time(msg->start_time, RCL_SYSTEM_TIME) > rclcpp::Clock().now())
  {
    pending_traj_ = std::move(msg);
    return;
  }

  pending_traj_.reset();
  activateSpline(msg);
}

void activateSpline(const traj_utils::msg::Bspline::ConstSharedPtr &msg)
{
  // parse pos traj

  Eigen::MatrixXd pos_pts(3, msg->pos_pts.size());

  Eigen::VectorXd knots(msg->knots.size());
  for (size_t i = 0; i < msg->knots.size(); ++i)
  {
    knots(i) = msg->knots[i];
  }

  for (size_t i = 0; i < msg->pos_pts.size(); ++i)
  {
    pos_pts(0, i) = msg->pos_pts[i].x;
    pos_pts(1, i) = msg->pos_pts[i].y;
    pos_pts(2, i) = msg->pos_pts[i].z;
  }

  UniformBspline pos_traj(pos_pts, msg->order, 0.1);
  pos_traj.setKnot(knots);

  // parse yaw traj

  // Eigen::MatrixXd yaw_pts(msg->yaw_pts.size(), 1);
  // for (int i = 0; i < msg->yaw_pts.size(); ++i) {
  //   yaw_pts(i, 0) = msg->yaw_pts[i];
  // }

  // UniformBspline yaw_traj(yaw_pts, msg->order, msg->yaw_dt);

  start_time_ = rclcpp::Time(msg->start_time, RCL_SYSTEM_TIME);
  traj_id_ = msg->traj_id;

  traj_.clear();
  traj_.push_back(pos_traj);
  traj_.push_back(traj_[0].getDerivative());
  traj_.push_back(traj_[1].getDerivative());

  traj_duration_ = traj_[0].getTimeSum();

  receive_traj_ = true;
}

std::pair<double, double> calculate_yaw(double t_cur, Eigen::Vector3d &pos, rclcpp::Time &time_now, rclcpp::Time &time_last)
{
  constexpr double PI = 3.1415926;
  constexpr double YAW_DOT_MAX_PER_SEC = PI;
  // constexpr double YAW_DOT_DOT_MAX_PER_SEC = PI;
  std::pair<double, double> yaw_yawdot(0, 0);
  double yaw = 0;
  double yawdot = 0;

  Eigen::Vector3d dir = t_cur + time_forward_ <= traj_duration_ ? traj_[0].evaluateDeBoorT(t_cur + time_forward_) - pos : traj_[0].evaluateDeBoorT(traj_duration_) - pos;
  double yaw_temp = dir.norm() > 0.1 ? atan2(dir(1), dir(0)) : last_yaw_;
  double max_yaw_change = YAW_DOT_MAX_PER_SEC * (time_now - time_last).seconds();
  if (yaw_temp - last_yaw_ > PI)
  {
    if (yaw_temp - last_yaw_ - 2 * PI < -max_yaw_change)
    {
      yaw = last_yaw_ - max_yaw_change;
      if (yaw < -PI)
        yaw += 2 * PI;

      yawdot = -YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yaw = yaw_temp;
      if (yaw - last_yaw_ > PI)
        yawdot = -YAW_DOT_MAX_PER_SEC;
      else
        yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).seconds();
    }
  }
  else if (yaw_temp - last_yaw_ < -PI)
  {
    if (yaw_temp - last_yaw_ + 2 * PI > max_yaw_change)
    {
      yaw = last_yaw_ + max_yaw_change;
      if (yaw > PI)
        yaw -= 2 * PI;

      yawdot = YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yaw = yaw_temp;
      if (yaw - last_yaw_ < -PI)
        yawdot = YAW_DOT_MAX_PER_SEC;
      else
        yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).seconds();
    }
  }
  else
  {
    if (yaw_temp - last_yaw_ < -max_yaw_change)
    {
      yaw = last_yaw_ - max_yaw_change;
      if (yaw < -PI)
        yaw += 2 * PI;

      yawdot = -YAW_DOT_MAX_PER_SEC;
    }
    else if (yaw_temp - last_yaw_ > max_yaw_change)
    {
      yaw = last_yaw_ + max_yaw_change;
      if (yaw > PI)
        yaw -= 2 * PI;

      yawdot = YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yaw = yaw_temp;
      if (yaw - last_yaw_ > PI)
        yawdot = -YAW_DOT_MAX_PER_SEC;
      else if (yaw - last_yaw_ < -PI)
        yawdot = YAW_DOT_MAX_PER_SEC;
      else
        yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).seconds();
    }
  }

  if (fabs(yaw - last_yaw_) <= max_yaw_change)
    yaw = 0.5 * last_yaw_ + 0.5 * yaw; // nieve LPF
  yawdot = 0.5 * last_yaw_dot_ + 0.5 * yawdot;
  last_yaw_ = yaw;
  last_yaw_dot_ = yawdot;

  yaw_yawdot.first = yaw;
  yaw_yawdot.second = yawdot;

  return yaw_yawdot;
}

void cmdCallback(ego_planner::Px4Offboard &offboard)
{
  offboard.poll();
  if (!offboard.ready())
    return;

  rclcpp::Time time_now = rclcpp::Clock().now();
  if (pending_traj_ && time_now >= rclcpp::Time(pending_traj_->start_time, RCL_SYSTEM_TIME))
  {
    activateSpline(pending_traj_);
    pending_traj_.reset();
  }
  if (!receive_traj_)
    return;
  start_time_ = offboard.executionStart(start_time_);
  double t_cur = (time_now - start_time_).seconds();
  if (t_cur < 0.0)
    return;

  const bool first_command = reset_on_next_traj_;
  if (first_command)
  {
    last_yaw_ = odom_yaw_;
    last_yaw_dot_ = 0.0;
    reset_on_next_traj_ = false;
  }

  Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero()), acc(Eigen::Vector3d::Zero()), pos_f;
  std::pair<double, double> yaw_yawdot(0, 0);

  // static rclcpp::Time time_last = clock.now();
  if (t_cur < traj_duration_)
  {
    pos = traj_[0].evaluateDeBoorT(t_cur);
    vel = traj_[1].evaluateDeBoorT(t_cur);
    acc = traj_[2].evaluateDeBoorT(t_cur);

    /*** calculate yaw ***/
    yaw_yawdot = first_command ? std::make_pair(last_yaw_, 0.0)
                               : calculate_yaw(t_cur, pos, time_now, time_last_);
    /*** calculate yaw ***/

    double tf = min(traj_duration_, t_cur + 2.0);
    pos_f = traj_[0].evaluateDeBoorT(tf);
  }
  else
  {
    /* hover when finish traj_ */
    pos = traj_[0].evaluateDeBoorT(traj_duration_);
    vel.setZero();
    acc.setZero();

    yaw_yawdot.first = last_yaw_;
    yaw_yawdot.second = 0;

    pos_f = pos;
  }
  time_last_ = time_now;

  cmd.header.stamp = time_now;
  cmd.header.frame_id = "map";
  cmd.trajectory_flag = quadrotor_msgs::msg::PositionCommand::TRAJECTORY_STATUS_READY;
  cmd.trajectory_id = traj_id_;

  cmd.position.x = pos(0);
  cmd.position.y = pos(1);
  cmd.position.z = pos(2);

  cmd.velocity.x = vel(0);
  cmd.velocity.y = vel(1);
  cmd.velocity.z = vel(2);

  cmd.acceleration.x = acc(0);
  cmd.acceleration.y = acc(1);
  cmd.acceleration.z = acc(2);

  cmd.yaw = yaw_yawdot.first;
  cmd.yaw_dot = yaw_yawdot.second;

  last_yaw_ = cmd.yaw;

  pos_cmd_pub->publish(cmd);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("traj_server");
  ego_planner::Px4Offboard offboard;
  offboard.init(*node, cancelOffboardGoal);

  auto bspline_sub = node->create_subscription<traj_utils::msg::Bspline>(
      "planning/bspline",
      10,
      [&offboard](traj_utils::msg::Bspline::ConstSharedPtr msg) { bsplineCallback(std::move(msg), offboard); });
  auto odom_sub = node->create_subscription<nav_msgs::msg::Odometry>(
      "odom_world", 10, odomCallback);
  auto goal_sub = node->create_subscription<std_msgs::msg::Int32>(
      "/ego/new_goal", 10,
      [&offboard](std_msgs::msg::Int32::ConstSharedPtr msg) { newGoalCallback(std::move(msg), offboard); });
  pos_cmd_pub = node->create_publisher<quadrotor_msgs::msg::PositionCommand>(
      "/position_cmd",
      50);

  auto cmd_timer = node->create_wall_timer(
      std::chrono::milliseconds(10),
      [&offboard]() { cmdCallback(offboard); });

  /* control parameter */
  cmd.kx[0] = pos_gain[0];
  cmd.kx[1] = pos_gain[1];
  cmd.kx[2] = pos_gain[2];

  cmd.kv[0] = vel_gain[0];
  cmd.kv[1] = vel_gain[1];
  cmd.kv[2] = vel_gain[2];

  node->declare_parameter("traj_server/time_forward", -1.0);
  node->get_parameter("traj_server/time_forward", time_forward_);

  last_yaw_ = 0.0;
  last_yaw_dot_ = 0.0;

  rclcpp::sleep_for(std::chrono::seconds(1));

  RCLCPP_WARN(node->get_logger(), "[Traj server]: ready.");

  rclcpp::spin(node);
  pos_cmd_pub.reset();
  rclcpp::shutdown();

  return 0;
}