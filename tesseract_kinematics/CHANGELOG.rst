^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tesseract_kinematics
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
