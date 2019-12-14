# Tesseract

[![Build Status](https://travis-ci.com/ros-industrial-consortium/tesseract.svg?branch=master)](https://travis-ci.com/ros-industrial-consortium/tesseract)
[![Github Issues](https://img.shields.io/github/issues/ros-industrial-consortium/tesseract.svg)](http://github.com/ros-industrial-consortium/tesseract/issues)

[![license - apache 2.0](https://img.shields.io/:license-Apache%202.0-yellowgreen.svg)](https://opensource.org/licenses/Apache-2.0)
[![license - bsd 2 clause](https://img.shields.io/:license-BSD%202--Clause-blue.svg)](https://opensource.org/licenses/BSD-2-Clause)

[![support level: consortium](https://img.shields.io/badge/support%20level-consortium-brightgreen.png)](http://rosindustrial.org/news/2016/10/7/better-supporting-a-growing-ros-industrial-software-platform)

The planning framework (Tesseract) was designed to be light weight, limiting the number of dependencies, mainly only using standard libraries like, eigen, boost, orocos and to the packages below. The core packages are ROS agnostic and have full python support.

## Tesseract Core Packages

* **tesseract** – This is the main class that manages the major component Environment, Forward Kinematics, Inverse Kinematics and loading from various data.
* **tesseract_collision** – This package contains privides a common interface for collision checking prividing several implementation of a Bullet collision library and FCL collision library. It includes both continuous and discrete collision checking for convex-convex, convex-concave and concave-concave shapes.
* **tesseract_common** – This package contains common functionality needed by the majority of the packages.
* **tesseract_environment** – This package contains the Tesseract Environment which provides functionality to add,remove,move and modify links and joint. It also manages adding object to the contact managers and provides the ability.
* **tesseract_geometry** – This package contains geometry types used by Tesseract including primitive shapes, mesh, convex hull mesh, octomap and signed distance field.
* **tesseract_kinematics** –  This package contains a common interface for Forward and Inverse kinematics for Chain, Tree's and Graphs including implementation using KDL and OPW Kinematics.
* **tesseract_motion_planners** – This package contains a common interface for Motion Planners and includes implementation for OMPL, TrajOpt and Descartes.
* **tesseract_scene_graph** – This package contains the scene graph which is the data structure used to manage the connectivity of objects in the environment. It inherits from boost graph and provides addition functionality for adding,removing and modifying Links and Joints along with search implementation.
* **tesseract_support** – This package contains support data used for unit tests and examples throughout Tesseract.
* **tesseract_visualization** – This package contains visualization utilities and libraries.

## Tesseract ROS Packages

* **tesseract_examples** – This package contains examples using tesseract and tesseract_ros for motion planning and collision checking.
* **tesseract_plugins** – This contains plugins for collision and kinematics which are automatically loaded by the monitors.
* **tesseract_rosutils** – This package contains the utilities like converting from ROS message types to native Tesseract types and the reverse.
* **tesseract_msgs** – This package contains the ROS message types used by Tesseract ROS.
* **tesseract_rviz** – This package contains the ROS visualization plugins for Rviz to visualize Tesseract. All of the features have been composed in libraries to enable to the ability to create custom displays quickly.
* **tesseract_monitoring** – This package contains different types of environment monitors. It currently contains a contact monitor and environment monitor. The contact monitor will monitor the active environment state and publish contact information. This is useful if the robot is being controlled outside of ROS, but you want to make sure it does not collide with objects in the environment. The second is the environment monitor, which is the main environment which facilitates requests to add, remove, disable and enable collision objects, while publishing its current state to keep other ROS nodes updated with the latest environment.

## TODO's

.. Warning:: These packages are under heavy development and are subject to change.


See [issue #66](https://github.com/ros-industrial-consortium/tesseract/issues/66)

## Clone Repository

This repository contains submodule tesseract_ext so use the *--recursive* flag as shown below.

`git clone --recursive`

.. NOTE: To speed up clean build you may want to add tesseract_ext to an extended workspace. If so do not clone with submodules and clone https://github.com/ros-industrial-consortium/tesseract_ext.git into your extended workspace.

## Building with Clang-Tidy Enabled

Must pass the -DTESSERACT_ENABLE_CLANG_TIDY=ON to cmake when building. This is automatically enabled if cmake argument -DTESSERACT_ENABLE_TESTS=ON is passed.

## Building Tesseract Tests

Must pass the -DTESSERACT_ENABLE_TESTS=ON to cmake when wanting to build tests. This automatically enables clang tidy.

.. NOTE: If you are building using catkin tools, use `catkin build --force-cmake -DTESSERACT_ENABLE_TESTS=ON`.

## Running Tesseract Tests

Tesseract packages use ctest because it is ROS agnostic, so to run the test call `catkin test --no-deps tesseract_collision tesseract_environment tesseract_geometry tesseract_kinematics tesseract_motion_planners tesseract_process_planners tesseract_scene_graph tesseract_urdf`

## Running Tesseract ROS Tests

Tesseract ROS Packages use the typical method so pass catkin argument `run_tests` when wanting to run these tests.

## Build Branch Sphinx Documentation

```
cd gh_pages
sphinx-build . output
```
Now open gh_pages/output/index.rst and remove *output* directory before commiting changes.
