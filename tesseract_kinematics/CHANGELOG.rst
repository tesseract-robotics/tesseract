^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tesseract_kinematics
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.8.7 (2022-03-24)
------------------

0.8.6 (2022-03-24)
------------------

0.8.5 (2022-03-24)
------------------

0.8.4 (2022-03-03)
------------------
* Add overload method for calcInvKin to take single KinGroupIKInput
* Contributors: Levi Armstrong

0.8.3 (2022-02-22)
------------------
* Python patches for Feb 2022 update (`#716 <https://github.com/tesseract-robotics/tesseract/issues/716>`_)
* Update UR Kinematics Parameters for e-series
  UR10eParameters, UR5eParameters and UR3eParameters values were slightly
  off compared to official documentation. We updated them to match.
* Contributors: John Wason, Leo Ghafari

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
* Add redundancy capable joints to the harmonizeTowardZero function
* Contributors: Levi Armstrong

0.7.0 (2021-12-04)
------------------

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
* Update ikfast plugin
* Add determinant check and make kdl solvers thread safe (`#664 <https://github.com/ros-industrial-consortium/tesseract/issues/664>`_)
* Fix Kinematic Group working frames
* Contributors: Levi Armstrong, Levi-Armstrong

0.5.0 (2021-07-02)
------------------
* Add convex decomposition support (`#609 <https://github.com/ros-industrial-consortium/tesseract/issues/609>`_)
* IK Solver Redundant Solutions Update (`#601 <https://github.com/ros-industrial-consortium/tesseract/issues/601>`_)
* Contributors: Levi Armstrong, Michael Ripperger

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
* Fix satisfiesPositionLimits to use relative equal and calculation of redundant solutions to include all permutations
* Fix inv kinematics to only return solution within limits and add redundant solutions for kdl ik solvers
* Add cmake format
* Add kinematics utility function for calculating manipulability (`#571 <https://github.com/ros-industrial-consortium/tesseract/issues/571>`_)
* Use boost targets, add cpack and license file (`#572 <https://github.com/ros-industrial-consortium/tesseract/issues/572>`_)
* Add kinematics utility function isNearSingularity (`#569 <https://github.com/ros-industrial-consortium/tesseract/issues/569>`_)
* Fix the way in which Eigen is included (`#570 <https://github.com/ros-industrial-consortium/tesseract/issues/570>`_)
* Fix checkJoints in UR Kinematics
* Add universal robot inverse kinematics
* Fix kinematics checkJoints not returning false when outside limits (`#564 <https://github.com/ros-industrial-consortium/tesseract/issues/564>`_)
* Add ability to construct ROP and REP kinematic solver with different solver names
* Fix IKFast inverse kinematics wrapper
* Update forward kinematics interface to return solutions versus out parameters
* Update inverse kinematics interface to return solutions versus out parameters
* Contributors: Hervé Audren, Levi Armstrong

0.2.0 (2021-02-17)
------------------
* Improve tesseract_kinematics test coverage (`#543 <https://github.com/ros-industrial-consortium/tesseract/issues/543>`_)
* Add another test case for kinematics core harmonize function
* Fix harmonizeTowardZero
* Update cmake_common_scripts to ros_industrial_cmake_boilerplate
* Improve code coverage for tesseract_kinematics core
* Move all directories in tesseract directory up one level
* Contributors: Levi Armstrong

0.1.0 (2020-12-31)
------------------
* Update tesseract_kinematics to leverage tesseract_scene_graph
* Fix depends in tesseract_kinematics
* Semi-Isolate Tesseract Kinematics
* Contributors: Levi Armstrong
