*****************************
Tesseract Motion Planning API
*****************************

Overview
========
Tesseract contains an easy to use motion planning API that enables developers to quickly create
custom-tailored motion plans. Tesseract's interface includes plugins for several popular planning
algorithms such as OMPL, TrajOpt, TrajOpt IFOPT and Descartes.

In this tutorial, we will step through an example using the Tesseract motion planning API to plan
a simple catesian trajectory using the TrajOpt planner.

.. note: This example is currently specific to ROS, but will soon be converted to ROS-agnostic and
         will moved from the tesseract_ros repository to the tesseract_planning repository.


Running the Example
===================

* Run the following command: ::

    roslaunch tesseract_ros_examples basic_cartesian_example.launch


The Full Example
================

.. rli:: https://raw.githubusercontent.com/ros-industrial-consortium/tesseract_ros/master/tesseract_ros_examples/src/basic_cartesian_example.cpp
   :language: c++


Initial Setup
=============

Load in the URDF and SRDF files.

.. rli:: https://raw.githubusercontent.com/ros-industrial-consortium/tesseract_ros/master/tesseract_ros_examples/src/basic_cartesian_example.cpp
   :language: c++
   :lines: 133-139

Initialize the environment monitor.

.. rli:: https://raw.githubusercontent.com/ros-industrial-consortium/tesseract_ros/master/tesseract_ros_examples/src/basic_cartesian_example.cpp
   :language: c++
   :lines: 142-144

Create an octomap and add it to the local environment.

.. rli:: https://raw.githubusercontent.com/ros-industrial-consortium/tesseract_ros/master/tesseract_ros_examples/src/basic_cartesian_example.cpp
   :language: c++
   :lines: 147-149

Create plotting tool.

.. rli:: https://raw.githubusercontent.com/ros-industrial-consortium/tesseract_ros/master/tesseract_ros_examples/src/basic_cartesian_example.cpp
   :language: c++
   :lines: 152-154


Defining Instructions
=====================

Set the initial state of the robot.

.. rli:: https://raw.githubusercontent.com/ros-industrial-consortium/tesseract_ros/master/tesseract_ros_examples/src/basic_cartesian_example.cpp
   :language: c++
   :lines: 157-175

Create a CompositeInstruction object. This will be our root instruction and will contain an ordered list of instructions corresponding to each movement.

.. rli:: https://raw.githubusercontent.com/ros-industrial-consortium/tesseract_ros/master/tesseract_ros_examples/src/basic_cartesian_example.cpp
   :language: c++
   :lines: 178

Define the first waypoint as the starting position.

.. rli:: https://raw.githubusercontent.com/ros-industrial-consortium/tesseract_ros/master/tesseract_ros_examples/src/basic_cartesian_example.cpp
   :language: c++
   :lines: 181

Create and set the starting instruction.

.. rli:: https://raw.githubusercontent.com/ros-industrial-consortium/tesseract_ros/master/tesseract_ros_examples/src/basic_cartesian_example.cpp
   :language: c++
   :lines: 182-183

Define two waypoints in cartesian space.

.. rli:: https://raw.githubusercontent.com/ros-industrial-consortium/tesseract_ros/master/tesseract_ros_examples/src/basic_cartesian_example.cpp
   :language: c++
   :lines: 186-190

Create an instruction for a freespace movement from waypoint 0 to waypoint 1.

.. rli:: https://raw.githubusercontent.com/ros-industrial-consortium/tesseract_ros/master/tesseract_ros_examples/src/basic_cartesian_example.cpp
   :language: c++
   :lines: 193-194


Create an instruction for a linear movement from waypoint 1 to waypoint 2.

.. rli:: https://raw.githubusercontent.com/ros-industrial-consortium/tesseract_ros/master/tesseract_ros_examples/src/basic_cartesian_example.cpp
   :language: c++
   :lines: 197


Create an instruction for a linear movement from waypoint 2 to waypoint 0.

.. rli:: https://raw.githubusercontent.com/ros-industrial-consortium/tesseract_ros/master/tesseract_ros_examples/src/basic_cartesian_example.cpp
   :language: c++
   :lines: 200-201

Push back each movement instruction into the root CompositeInstruction (in the order that they will be executed).

.. rli:: https://raw.githubusercontent.com/ros-industrial-consortium/tesseract_ros/master/tesseract_ros_examples/src/basic_cartesian_example.cpp
   :language: c++
   :lines: 204-206


Generating a Motion Plan
========================

Initialize a process planning server.

.. rli:: https://raw.githubusercontent.com/ros-industrial-consortium/tesseract_ros/master/tesseract_ros_examples/src/basic_cartesian_example.cpp
   :language: c++
   :lines: 211

Load default process planners.

.. rli:: https://raw.githubusercontent.com/ros-industrial-consortium/tesseract_ros/master/tesseract_ros_examples/src/basic_cartesian_example.cpp
   :language: c++
   :lines: 212

Create a process planning request.

.. rli:: https://raw.githubusercontent.com/ros-industrial-consortium/tesseract_ros/master/tesseract_ros_examples/src/basic_cartesian_example.cpp
   :language: c++
   :lines: 215

Set the planner type to TrajOpt.

.. rli:: https://raw.githubusercontent.com/ros-industrial-consortium/tesseract_ros/master/tesseract_ros_examples/src/basic_cartesian_example.cpp
   :language: c++
   :lines: 216

Set instructions to the root CompositeInstruction.

.. rli:: https://raw.githubusercontent.com/ros-industrial-consortium/tesseract_ros/master/tesseract_ros_examples/src/basic_cartesian_example.cpp
   :language: c++
   :lines: 217

Run the planning request and wait for a response.

.. rli:: https://raw.githubusercontent.com/ros-industrial-consortium/tesseract_ros/master/tesseract_ros_examples/src/basic_cartesian_example.cpp
   :language: c++
   :lines: 223-224


Visualizing the Motion Plan
===========================

Plot the motion plan using Rviz.

.. rli:: https://raw.githubusercontent.com/ros-industrial-consortium/tesseract_ros/master/tesseract_ros_examples/src/basic_cartesian_example.cpp
   :language: c++
   :lines: 227-235
