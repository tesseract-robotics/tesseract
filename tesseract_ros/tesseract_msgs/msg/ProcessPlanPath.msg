# This is a single process path, e.g. one raster strip or one freespace motion

trajectory_msgs/JointTrajectory trajectory       # The trajectory for the robot
float32[] velocity                               # Cartesian velocity for MoveL. I don't think we will have target velocities for the MoveJ (freespace) terms
float32[] work
