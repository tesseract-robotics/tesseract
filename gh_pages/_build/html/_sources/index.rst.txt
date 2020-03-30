.. SphinxTest documentation master file, created by
   sphinx-quickstart on Tue Oct  3 11:09:13 2017.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

=============================
Welcome to the Tesseract wiki
=============================

The new planning framework (Tesseract) was designed to be lightweight, limiting the number of dependencies, mainly to only used standard library, eigen, boost, orocos and to the core packages below are ROS agnostic and have full python support.

Tesseract Core Packages
-----------------------

* **tesseract** – This is the main class that manages the major component Environment, Forward Kinematics, Inverse Kinematics and loading from various data.
* **tesseract_collision** – This package contains privides a common interface for collision checking prividing several implementation of a Bullet collision library and FCL collision library. It includes both continuous and discrete collision checking for convex-convex, convex-concave and concave-concave shapes.
* **tesseract_common** – This package contains common functionality needed by the majority of the packages.
* **tesseract_environment** – This package contains the Tesseract Environment which provides functionality to add,remove,move and modify links and joint. It also manages adding object to the contact managers and provides the ability.
* **tesseract_geometry** – This package contains geometry types used by Tesseract including primitive shapes, mesh, convex hull mesh, octomap and signed distance field.
* **tesseract_kinematics** –  This package contains a common interface for Forward and Inverse kinematics for Chain, Tree's and Graphs including implementation using KDL and OPW Kinematics.
* **tesseract_planners** – This package contains a common interface for Planners and includes implementation for OMPL, TrajOpt and Descartes.
* **tesseract_scene_graph** – This package contains the scene graph which is the data structure used to manage the connectivity of objects in the environment. It inherits from boost graph and provides addition functionality for adding,removing and modifying Links and Joints along with search implementation.
* **tesseract_support** – This package contains support data used for unit tests and examples throughout Tesseract.
* **tesseract_visualization** – This package contains visualization utilities and libraries.
* **tesseract_urdf** - This package contains a custom urdf parser supporting addition shapes and features currently not supported by urdfdom.

Tesseract ROS Packages
----------------------

* **tesseract_examples** – This package contains examples using tesseract and tesseract_ros for motion planning and collision checking.
* **tesseract_plugins** – This contains plugins for collision and kinematics which are automatically loaded by the monitors.
* **tesseract_rosutils** – This package contains the utilities like converting from ROS message types to native Tesseract types and the reverse.
* **tesseract_msgs** – This package contains the ROS message types used by Tesseract ROS.
* **tesseract_rviz** – This package contains the ROS visualization plugins for Rviz to visualize Tesseract. All of the features have been composed in libraries to enable to the ability to create custom displays quickly.
* **tesseract_monitoring** – This package contains different types of environment monitors. It currently contains a contact monitor and environment monitor. The contact monitor will monitor the active environment state and publish contact information. This is useful if the robot is being controlled outside of ROS, but you want to make sure it does not collide with objects in the environment. The second is the environment monitor, which is the main environment which facilitates requests to add, remove, disable and enable collision objects, while publishing its current state to keep other ROS nodes updated with the latest environment.

.. Warning:: These packages are under heavy development and are subject to change.


Packages
------------

.. toctree::
   :maxdepth: 1

   tesseract_scene_graph <_source/tesseract_scene_graph_doc.rst>
   tesseract_collision <_source/tesseract_collision_doc.rst>
   tesseract_geometry <_source/tesseract_geometry_doc.rst>
   tesseract_ros <_source/tesseract_ros_doc.rst>
   tesseract_msgs <_source/tesseract_msgs_doc.rst>
   tesseract_rviz <_source/tesseract_rviz_doc.rst>
   tesseract_monitoring <_source/tesseract_monitoring_doc.rst>
   tesseract_planning <_source/tesseract_planning_doc.rst>
   tesseract_urdf <_source/tesseract_urdf_doc.rst>

FAQ
---
.. toctree::
   :maxdepth: 2

   Questions?<_source/FAQ.rst>
