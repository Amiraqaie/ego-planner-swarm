#ifndef EGO_PLANNER_PX4_OFFBOARD_H
#define EGO_PLANNER_PX4_OFFBOARD_H

#include <chrono>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <string>
#include <utility>

#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>

namespace ego_planner
{
class Px4Offboard
{
public:
  void init(rclcpp::Node &node, std::function<void()> on_exit)
  {
    enabled_ = node.declare_parameter("px4/require_offboard", false);
    const auto topic = node.declare_parameter<std::string>("px4/vehicle_status_topic", "/fmu/out/vehicle_status");
    const auto timeout = node.declare_parameter("px4/vehicle_status_timeout", 1.0);
    if (!enabled_)
      return;

    if (!(timeout > 0.0) || !std::isfinite(timeout))
      throw std::invalid_argument("px4/vehicle_status_timeout must be finite and positive");

    timeout_ = std::chrono::duration<double>(timeout);
    on_exit_ = std::move(on_exit);
    status_sub_ = node.create_subscription<px4_msgs::msg::VehicleStatus>(
        topic, rclcpp::SensorDataQoS(),
        [this](px4_msgs::msg::VehicleStatus::ConstSharedPtr status)
        {
          poll();
          last_status_ = std::chrono::steady_clock::now();
          const bool offboard = status->nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
          if (offboard && !offboard_)
            entered_at_ = rclcpp::Clock().now();
          if (!offboard && offboard_)
            exitOffboard();
          offboard_ = offboard;
        });
  }

  void poll()
  {
    if (offboard_ && std::chrono::steady_clock::now() - last_status_ >= timeout_)
      exitOffboard();
  }

  bool ready() const { return !enabled_ || offboard_; }
  bool enabled() const { return enabled_; }

  rclcpp::Time executionStart(const rclcpp::Time &planned_start) const
  {
    return enabled_ && planned_start < entered_at_ ? entered_at_ : planned_start;
  }

private:
  void exitOffboard()
  {
    offboard_ = false;
    on_exit_();
  }

  bool enabled_ = false;
  bool offboard_ = false;
  std::chrono::steady_clock::time_point last_status_;
  std::chrono::duration<double> timeout_{1.0};
  rclcpp::Time entered_at_{0, 0, RCL_SYSTEM_TIME};
  std::function<void()> on_exit_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
};
}  // namespace ego_planner

#endif
