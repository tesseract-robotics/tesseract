# tesseract
The new planning framework (Tesseract) was designed to be lightweight, limiting the number of dependencies, mainly to only used standard library, eigen, boost, orocos and very few external ROS packages. It currently contains seven packaged described below:

* **tesseract_core** – This package contains only the interface header files and data type structures to be used. This is the only package that does not depend on ROS for the sole purpose to facilitate integration outside of ROS, with little changes, which can sometimes be required.
* **tesseract_ros** – This package is the ROS implementation of the interface identified in the tesseract_core package. It currently leverages orocos libraries for data storage of both the environment and kinematics.
* **tesseract_collision** – This package contains the ROS implementation of a Bullet collision library. It includes both continuous and discrete collision checking for convex-convex and convex-concave shapes.
* **tesseract_msgs** – This package contains the ROS message types used by Tesseract.
* **tesseract_rviz** – This package contains the ROS visualization plugins for Rviz to visualize both the environment state and trajectories.
* **tesseract_monitoring** – This package contains different types of environment monitors. It currently contains a contact monitor and environment monitor. The contact monitor will monitor the active environment state and publish contact information. This is useful if the robot is being controlled outside of ROS, but you want to make sure it does not collide with objects in the environment. The second is the environment monitor, which is the main environment which facilitates requests to add, remove, disable and enable collision objects, while publishing its current state to keep other ROS nodes updated with the latest environment.
* **tesseract_planning** – This package contains the planners available to be used with the tesseract environment. It currently contains OMPL and TrajOpt.

*WARNING: These packages are under heavy development and are subject to change.*

## Clone Repository

This repository contains submodules so use the *--recursive* flag as shown below.

`git clone --recursive`

## Build Branch Sphinx Documentation

```
cd gh_pages
sphinx-build . output
```
Now open gh_pages/output/index.rst and remove *output* directory before commiting changes.
