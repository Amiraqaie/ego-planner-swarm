// Standalone regression: c++ -std=c++17 -I../include local_arc_timing_test.cpp -o /tmp/local_arc_timing_test
#include "ego_planner/local_arc_timing.h"
#include <cassert>

int main()
{
  using ego_planner::LocalArcTiming;
  // Same curved geometry with two radically different reference clocks.
  auto check = [](double global_duration) {
    double length = 0.0, x = 0.0, y = 0.0;
    for (int i = 1; i <= 512; ++i)
    {
      const double global_time = global_duration * i / 512;
      const double u = global_time / global_duration;
      const double next_x = 7.0 * u;
      const double next_y = std::sin(u);
      length += std::hypot(next_x - x, next_y - y);
      x = next_x;
      y = next_y;
    }
    return LocalArcTiming(length, 0.285, 0.342, 0.0, 0.2);
  };
  const auto normal = check(15.0);
  const auto slow = check(450.0);
  assert(std::abs(normal.duration() - slow.duration()) < 1e-9);
  assert(std::abs(normal.distanceAt(1.0) - slow.distanceAt(1.0)) < 1e-9);
  assert(slow.duration() < 27.0 && slow.distanceAt(1.0) > 0.14);
  const LocalArcTiming short_stop(0.25, 0.285, 0.342, 0.12, 0.0);
  assert(short_stop.distanceAt(short_stop.duration()) == 0.25);
  assert(short_stop.end_speed == 0.0);
  for (double t = 0.01; t < short_stop.duration(); t += 0.01)
    assert(short_stop.distanceAt(t) > short_stop.distanceAt(t - 0.01));
}
