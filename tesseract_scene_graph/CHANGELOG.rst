^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tesseract_scene_graph
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.32.0 (2025-09-10)
-------------------

0.31.0 (2025-07-05)
-------------------
* Make sure serialized objects have friend struct tesseract_common::Serialization
* Contributors: Levi Armstrong

0.30.0 (2025-04-23)
-------------------
* Improve tesseract_common code coverage (`#1138 <https://github.com/tesseract-robotics/tesseract/issues/1138>`_)
* Improve codecov CI
* Contributors: Levi Armstrong

0.29.1 (2025-03-26)
-------------------

0.29.0 (2025-03-20)
-------------------
* Fix clang-tidy-17 errors (`#1122 <https://github.com/tesseract-robotics/tesseract/issues/1122>`_)
* Contributors: Levi Armstrong

0.28.8 (2025-02-01)
-------------------

0.28.7 (2025-01-29)
-------------------

0.28.6 (2025-01-26)
-------------------

0.28.5 (2025-01-21)
-------------------

0.28.4 (2025-01-18)
-------------------

0.28.3 (2025-01-18)
-------------------

0.28.2 (2025-01-17)
-------------------

0.28.1 (2025-01-17)
-------------------

0.28.0 (2025-01-16)
-------------------
* Add floating joint support
* Contributors: Levi Armstrong

0.27.1 (2024-12-03)
-------------------

0.27.0 (2024-12-01)
-------------------

0.26.0 (2024-10-27)
-------------------
* Remove TesseractSupportResourceLocator
* Fix serialization
* Contributors: Levi Armstrong

0.25.0 (2024-09-28)
-------------------

0.24.1 (2024-08-19)
-------------------

0.24.0 (2024-08-14)
-------------------

0.23.1 (2024-07-28)
-------------------
* Cleanup boost serialization
* Contributors: Levi Armstrong

0.23.0 (2024-07-24)
-------------------

0.22.2 (2024-06-10)
-------------------

0.22.1 (2024-06-03)
-------------------

0.22.0 (2024-06-02)
-------------------
* Add support for jerk limits
* Leverage forward declarations to improve compile times (`#990 <https://github.com/tesseract-robotics/tesseract/issues/990>`_)
* Contributors: Levi Armstrong

0.21.5 (2023-12-14)
-------------------

0.21.4 (2023-11-20)
-------------------

0.21.3 (2023-11-16)
-------------------
* Fix thread safety in environment (`#964 <https://github.com/tesseract-robotics/tesseract/issues/964>`_)
  * Fix thread safety in environment.cpp
  * Clang format
* Contributors: Levi Armstrong

0.21.2 (2023-11-10)
-------------------
* Support message conversions
* Contributors: Levi Armstrong

0.21.1 (2023-11-09)
-------------------

0.21.0 (2023-11-07)
-------------------

0.20.2 (2023-10-26)
-------------------

0.20.1 (2023-10-13)
-------------------
* Unused includes cleanup (`#946 <https://github.com/tesseract-robotics/tesseract/issues/946>`_)
* Contributors: Roelof

0.20.0 (2023-09-27)
-------------------

0.19.2 (2023-09-06)
-------------------

0.19.1 (2023-09-05)
-------------------

0.19.0 (2023-09-05)
-------------------
* Update kinematics and collision packages to leverage cmake components (`#927 <https://github.com/tesseract-robotics/tesseract/issues/927>`_)
* Update emails
* Contributors: Levi Armstrong

0.18.1 (2023-06-30)
-------------------

0.18.0 (2023-06-29)
-------------------
* Add package cmake flags for testing, examples and benchmarks
* Contributors: Levi Armstrong

0.17.0 (2023-06-06)
-------------------

0.16.3 (2023-05-04)
-------------------

0.16.2 (2023-04-28)
-------------------

0.16.1 (2023-04-11)
-------------------

0.16.0 (2023-04-09)
-------------------
* Add AddTrajectoryLinkCommand
* Contributors: Levi Armstrong

0.15.3 (2023-03-22)
-------------------

0.15.2 (2023-03-15)
-------------------

0.15.1 (2023-03-14)
-------------------

0.15.0 (2023-03-03)
-------------------
* Performance improvements found using callgrind (`#852 <https://github.com/tesseract-robotics/tesseract/issues/852>`_)
* Closes `#848 <https://github.com/tesseract-robotics/tesseract/issues/848>`_
* Improve tesseract_scene_graph code coverage (`#838 <https://github.com/tesseract-robotics/tesseract/issues/838>`_)
* Contributors: Levi Armstrong, Roelof Oomen

0.14.0 (2022-10-23)
-------------------
* Including <boost/serialization/library_version_type.hpp> for Boost 1.74. Fixes `tesseract-robotics/tesseract#764 <https://github.com/tesseract-robotics/tesseract/issues/764>`_
* Contributors: Roelof Oomen

0.13.1 (2022-08-25)
-------------------
* Move most SWIG commands to tesseract_python package (`#809 <https://github.com/tesseract-robotics/tesseract/issues/809>`_)
* Contributors: John Wason

0.13.0 (2022-07-11)
-------------------
* Update code based on clang-tidy-14
* Contributors: Levi Armstrong

0.10.0 (2022-07-06)
-------------------
* Update ros_industrial_cmake_boilerplate to 0.3.0 (`#795 <https://github.com/tesseract-robotics/tesseract/issues/795>`_)

0.9.11 (2022-06-30)
-------------------
* Updated CPack (`#786 <https://github.com/tesseract-robotics/tesseract/issues/786>`_)
* Update to use find_gtest macro
* Contributors: Levi Armstrong, Michael Ripperger

0.9.10 (2022-06-14)
-------------------

0.9.9 (2022-05-30)
------------------

0.9.8 (2022-05-30)
------------------

0.9.7 (2022-05-30)
------------------

0.9.6 (2022-05-02)
------------------

0.9.5 (2022-04-24)
------------------
* Add utility for rebuilding link and joint maps in scene graph
* Contributors: Levi Armstrong

0.9.4 (2022-04-22)
------------------
* Windows fixes with passing unit tests (`#751 <https://github.com/tesseract-robotics/tesseract/issues/751>`_)
  * Fix bug in OFKTStateSolver::moveLinkHelper
  * Use binary ifstream ond ofstream in serialization.h
  * Add c++17 flag to windows_noetic_build.yml
  * Fix SceneGraph move constructor, restore modified unit tests
* Contributors: John Wason

0.9.3 (2022-04-18)
------------------

0.9.2 (2022-04-03)
------------------

0.9.1 (2022-04-01)
------------------

0.9.0 (2022-03-31)
------------------
* fix spelling
* Contributors: Levi Armstrong

0.8.7 (2022-03-24)
------------------

0.8.6 (2022-03-24)
------------------

0.8.5 (2022-03-24)
------------------
* Add boost serialization for Environment commands and all underlying types (`#726 <https://github.com/tesseract-robotics/tesseract/issues/726>`_)
  * Add serialization macros to tesseract_common
  * Add serialization for tesseract_geometry primatives
  * Add serialization for meshes and octree
  * Add serialization for Link and Joint
  * Add serialization for tesseract_common types
  * Add serialization for SceneGraph and SceneState
  * Add serialization for tesseract_srdf and tesseract_common types
  * Add serialization for environment commands
  * Fix bug in getCollisionObjectPairs
* Fix incorrect linking to console_bridge for tesseract_scene_graph example
* Contributors: Josh Langsfeld, Matthew Powelson

0.8.4 (2022-03-03)
------------------
* cmake format
* Add TESSERACT_ENABLE_EXAMPLES compile option
* Contributors: John Wason

0.8.3 (2022-02-22)
------------------
* Python patches for Feb 2022 update (`#716 <https://github.com/tesseract-robotics/tesseract/issues/716>`_)
* Contributors: John Wason

0.8.2 (2022-01-27)
------------------

0.8.1 (2022-01-24)
------------------

0.8.0 (2022-01-19)
------------------

0.7.5 (2022-01-10)
------------------

0.7.4 (2021-12-15)
------------------

0.7.3 (2021-12-15)
------------------

0.7.2 (2021-12-15)
------------------

0.7.1 (2021-12-15)
------------------
* Move checkKinematics to getKinematicGroup and add support for clang-tidy-12 (`#682 <https://github.com/tesseract-robotics/tesseract/issues/682>`_)
  * Move checkKinematics to getKinematicGroup and add support for clang-tidy-12
  * Reduce the number of checks perform in checkKinematics
  * Leverage checkKinematics in unit tests
* Contributors: Levi Armstrong

0.7.0 (2021-12-04)
------------------
* Move AllowedCollisionMatrix into tesseract_common
* Contributors: Matthew Powelson

0.6.9 (2021-11-29)
------------------

0.6.8 (2021-11-29)
------------------
* Fix spelling errors
* Contributors: Levi Armstrong

0.6.7 (2021-11-16)
------------------

0.6.6 (2021-11-10)
------------------

0.5.0 (2021-07-02)
------------------

0.4.1 (2021-04-24)
------------------

0.4.0 (2021-04-23)
------------------
* Move srdf code to its own package tesseract_srdf
* Contributors: Levi Armstrong

0.3.1 (2021-04-14)
------------------
* Move tesseract_variables() before any use of custom macros
* Contributors: Levi Armstrong

0.3.0 (2021-04-09)
------------------
* Only enable code coverage if compiler definition is set
* Add cmake format
* Add support for defining collision margin data in SRDF (`#573 <https://github.com/ros-industrial-consortium/tesseract/issues/573>`_)
* Use boost targets, add cpack and license file (`#572 <https://github.com/ros-industrial-consortium/tesseract/issues/572>`_)
* Fix the way in which Eigen is included (`#570 <https://github.com/ros-industrial-consortium/tesseract/issues/570>`_)
* Add ability to construct ROP and REP kinematic solver with different solver names
* Contributors: Hervé Audren, Levi Armstrong

0.2.0 (2021-02-17)
------------------
* Add getSourceLink, getTargetLink, getInboundJoints and getOutboundJoints scene graph unit test
* Add manipulator manager unit tests
* Add support for replacing links and joints
* Switch addJoint, addLink, moveLink and addSceneGraph to use const&
* Fix scene graph default visibility and collision enabled
* Refactor tesseract_environment to use applyCommands
* Improve tesseract_scene_graph test coverage and fix found issues
* Update cmake_common_scripts to ros_industrial_cmake_boilerplate
* Add tesseract_scene_graph resource locator unit test
* Fix bug in SimpleLocateResource::getResourceContentStream
* Improve srdf test coverage
* Add more unit tests for scene graph
* Fix bug in scene graph change joint limits
* Add missing depend in tesseract_scene_graph
* Move all directories in tesseract directory up one level
* Contributors: Levi Armstrong

0.1.0 (2020-12-31)
------------------
* Add getAdjacentLinkNames to tesseract_scene_graph
* Add moveJoint method to scene graph
* Make minor fixes to tesseract_scene_graph
* Add parser for SceneGraph to KDL Tree
* Expose graph getVertex and getEdge method
* Add urdf parser and tests to tesseract_scene_graph
* Update mesh parser to include convex hulls and add unit tests
* Add mesh parser test
* Add mesh parser to tesseract_scene_graph
* Add macros.h
* Add tesseract_scene_graph
* Contributors: Levi Armstrong
