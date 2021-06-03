****************************
Tesseract Monitoring Package
****************************

This package contains different types of environment monitors. It currently contains a
contact monitor and environment monitor. The contact monitor will monitor the active
environment state and publish contact information. This is useful if the robot is being
controlled outside of ROS, but you want to make sure it does not collide with objects
in the environment. The second is the environment monitor, which is the main environment
which facilitates requests to add, remove, disable and enable collision objects, while
publishing its current state to keep other ROS nodes updated with the latest environment.
