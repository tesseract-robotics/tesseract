Getting Started
===============

.. note:: Instructions are intended for installation on the Ubuntu 20.04 Focal operating system.

Before installing Tesseract and it's dependencies, make sure you have the most up to date packages: ::

  sudo apt-get update
  sudo apt-get dist-upgrade

Installing ROS Noetic (Optionl)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The Robot Operating System (ROS) is an open source set of software libraries and tools that help you
build robot applications. The `tesseract`, `tesseract_planning`, and `tesseract_python` repositories are
all ROS-agnostic. The `tesseract_ros` and `tesseract_ros2` repositories contain tools that are specific
to ROS and make integrating Tesseract and ROS extremely easy.

To install the latest version of ROS (Noetic) follow the instructions from the ROS
`website <https://wiki.ros.org/noetic/Installation/Ubuntu>`_.

.. note:: It's easy to miss steps when going through the ROS installation tutorial. If you run into errors in
          the next few steps, a good place to start is to go back and make sure you have installed ROS correctly.

Installing Tools
^^^^^^^^^^^^^^^^

Catkin
------

Catkin is the official build system for ROS. Catkin can also be used to build ROS-agnostic packages containing CMakeLists.txt
and package.xml files.

You can install Catkin with the following command: ::

  sudo apt-get install ros-noetic-catkin python3-catkin-tools

ROSDep (Optional)
-----------------

ROSDep is the official ROS command line tool for installing system dependencies.

You can install ROSDep with the following commands: ::

  sudo apt-get install python3-rosdep
  sudo rosdep init
  rosdep update

WSTool
------

WSTool is the official ROS workspace management tool.

You can install WSTool with the following command: ::

  sudo apt-get install python3-wstool

Creating a Workspace
^^^^^^^^^^^^^^^^^^^^

Create a workspace (in this case we'll name it `tesseract_ws`): ::

  mkdir -p ~/tesseract_ws/src

Cloning Tesseract Planning
^^^^^^^^^^^^^^^^^^^^^^^^^^

Move to the source directory: ::

  cd ~/tesseract_ws/src

Clone Tesseract Planning repository into your workspace: ::

  git clone https://github.com/ros-industrial-consortium/tesseract_planning

If you are using the Robot Operating System (ROS), you can also clone the Tesseract ROS repository: ::

  git clone https://github.com/ros-industrial-consortium/tesseract_ros

Installing Dependencies
^^^^^^^^^^^^^^^^^^^^^^^

Installing Taskflow
-------------------

Taskflow is a general purpose parallel and heterogenous task programming system.

You can install taskflow by with the following commands: ::

  sudo add-apt-repository ppa:ros-industrial/ppa
  sudo apt-get update
  sudo apt-get install taskflow

Cloning Source Dependencies with WSTool
---------------------------------------

Clone the repositories in the dependencies.rosinstall file using wstool: ::

  wstool init ~/tesseract_ws/src/ ~/tesseract_ws/src/tesseract_planning/dependencies.rosinstall

Installing Debian Dependencies with ROSDep (Optionl)
----------------------------------------------------

Run the following command to automatically install all debian dependencies listed in each package.xml file: ::

  rosdep install -y --from-paths ~/tesseract_ws/src --ignore-src --rosdistro noetic

.. note:: If you don't use the ROSDep tool you will need to manually install (via `apt-get`) each debian dependency.

Building Your Workspace
^^^^^^^^^^^^^^^^^^^^^^^

Build your workspace using catkin tools: ::

  cd ~/tesseract_ws/
  source /opt/ros/noetic/setup.bash
  catkin build

Source the catkin workspace: ::

  source ~/tesseract_ws/devel/setup.bash

.. note:: To build with Clang-Tidy enabled you must pass the `-DTESSERACT_ENABLE_CLANG_TIDY=ON` to cmake when building.
          This is automatically enabled if cmake argument `-DTESSERACT_ENABLE_TESTING_ALL=ON` is passed.
