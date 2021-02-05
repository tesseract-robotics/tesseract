Getting Started
===============

Install ROS and Catkin
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`Install ROS Noetic <http://wiki.ros.org/noetic/Installation/Ubuntu>`_.

It is easy to miss steps when going through the ROS installation tutorial. If you run into errors in the next few steps, a good place to start is to go back and make sure you have installed ROS correctly.

Once you have ROS installed, make sure you have the most up to date packages: ::

  rosdep update
  sudo apt-get update
  sudo apt-get dist-upgrade

Install `catkin <http://wiki.ros.org/catkin>`_ the ROS build system: ::

  sudo apt-get install ros-noetic-catkin python-catkin-tools

Install `wstool <http://wiki.ros.org/wstool>`_ the ROS workspace management tool: ::

  sudo apt-get install python-wstool

Build Instructions
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Create a workspace (in this case we'll name it `tesseract_ws`): ::

  mkdir -p ~/tesseract_ws/src

Move into source directory and clone repository into your workspace: ::

  cd ~/tesseract_ws/src
  git clone https://github.com/ros-industrial-consortium/tesseract

Clone the repositories in the dependencies.rosinstall file using wstool or some other method (e.g. manually git cloning them): ::

  wstool init ~/tesseract_ws/src/ ~/tesseract_ws/src/tesseract/dependencies.rosinstall

Install dependencies: ::

  rosdep install -y --from-paths ~/tesseract_ws/src --ignore-src --rosdistro noetic

Build the workspace using catkin tools: ::

  cd ~/tesseract_ws/
  source /opt/ros/noetic/setup.bash
  catkin build

Source the catkin workspace: ::

  source ~/tesseract_ws/devel/setup.bash

NOTE: For noetic tesseract_ext is not required. Install the following dependencies: libbullet-dev, libbullet-extras-dev and ros-noetic-fcl. Taskflow can be install using the PPA below.

NOTE: Install TaskFlow from [ROS-Industrial PPA](https://launchpad.net/~ros-industrial/+archive/ubuntu/ppa).

Building with Clang-Tidy Enabled
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

To build with Clang-Tidy enabled you must pass the -DTESSERACT_ENABLE_CLANG_TIDY=ON to cmake when building. This is automatically enabled if cmake argument -DTESSERACT_ENABLE_TESTING_ALL=ON is passed.
