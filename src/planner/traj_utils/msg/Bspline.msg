int32 drone_id

int32 order
int64 traj_id
# Trajectory ID boundary assigned when the planner accepted this goal.
int64 goal_reference_traj_id
builtin_interfaces/Time start_time

float64[] knots
geometry_msgs/Point[] pos_pts

float64[] yaw_pts
float64 yaw_dt

