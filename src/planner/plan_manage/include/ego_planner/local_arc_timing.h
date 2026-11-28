#ifndef EGO_PLANNER_LOCAL_ARC_TIMING_H
#define EGO_PLANNER_LOCAL_ARC_TIMING_H

#include <algorithm>
#include <cmath>

namespace ego_planner
{
// Accelerate, cruise if distance permits, then brake to the terminal speed.
struct LocalArcTiming
{
  double length, acceleration, start_speed, end_speed;
  double peak, rise, cruise, fall, cruise_start, brake_start;

  LocalArcTiming(double distance, double speed_limit, double acceleration_limit,
                 double initial_speed, double terminal_speed)
      : length(distance), acceleration(acceleration_limit)
  {
    const double reach = 2.0 * acceleration * length;
    start_speed = std::clamp(initial_speed, 0.0, speed_limit);
    end_speed = std::min(terminal_speed, std::sqrt(start_speed * start_speed + reach));
    start_speed = std::min(start_speed, std::sqrt(end_speed * end_speed + reach));
    peak = std::min(speed_limit, std::sqrt(acceleration * length +
                              0.5 * (start_speed * start_speed + end_speed * end_speed)));
    rise = std::max(0.0, (peak - start_speed) / acceleration);
    fall = std::max(0.0, (peak - end_speed) / acceleration);
    cruise_start = 0.5 * (start_speed + peak) * rise;
    const double brake_distance = 0.5 * (end_speed + peak) * fall;
    cruise = std::max(0.0, (length - cruise_start - brake_distance) / peak);
    brake_start = cruise_start + cruise * peak;
  }

  double duration() const { return rise + cruise + fall; }

  double distanceAt(double t) const
  {
    if (t <= 0.0)
      return 0.0;
    if (t >= duration())
      return length;
    if (t < rise)
      return (start_speed + 0.5 * acceleration * t) * t;
    t -= rise;
    if (t < cruise)
      return cruise_start + peak * t;
    t -= cruise;
    return brake_start + (peak - 0.5 * acceleration * t) * t;
  }
};
}  // namespace ego_planner
#endif
