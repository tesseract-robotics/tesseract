^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tesseract_kinematics
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

0.28.0 (2025-01-16)
-------------------
* Add floating joint support
* Leverage tesseract_common loadYamlFile and loadYamlString
* Add yaml include directive support
* Expose Inverse Kinematics in Kinematics Group
* Contributors: Levi Armstrong

0.27.1 (2024-12-03)
-------------------

0.27.0 (2024-12-01)
-------------------

0.26.0 (2024-10-27)
-------------------
* Remove TesseractSupportResourceLocator
* Contributors: Levi Armstrong

0.25.0 (2024-09-28)
-------------------

0.24.1 (2024-08-19)
-------------------

0.24.0 (2024-08-14)
-------------------

0.23.1 (2024-07-28)
-------------------

0.23.0 (2024-07-24)
-------------------
* Do not export plugin libraries (`#1028 <https://github.com/tesseract-robotics/tesseract/issues/1028>`_)
* Fixes for building on Ubuntu Noble (`#1016 <https://github.com/tesseract-robotics/tesseract/issues/1016>`_)
* Contributors: Levi Armstrong, Roelof Oomen

0.22.2 (2024-06-10)
-------------------
* Add backtrace to type erasure casting to quickly identify where the issue location
* Contributors: Levi Armstrong

0.22.1 (2024-06-03)
-------------------
* - Also add KDL parameters to KDLInvKinChainNR_JL (see `#843 <https://github.com/tesseract-robotics/tesseract/issues/843>`_)
  - Some clang_tidy and typo fixes
* Contributors: Roelof Oomen

0.22.0 (2024-06-02)
-------------------
* Add the ability to change KDL parameters from kinematics configuration (`#843 <https://github.com/tesseract-robotics/tesseract/issues/843>`_)
  Co-authored-by: Levi Armstrong <levi.armstrong@gmail.com>
* Add support for jerk limits
* Leverage forward declarations to improve compile times (`#990 <https://github.com/tesseract-robotics/tesseract/issues/990>`_)
* Remove ineffective frame.Identity() call
  See `#984 <https://github.com/tesseract-robotics/tesseract/issues/984>`_
* Contributors: Levi Armstrong, Roelof, Sean Cardello

0.21.5 (2023-12-14)
-------------------

0.21.4 (2023-11-20)
-------------------

0.21.3 (2023-11-16)
-------------------

0.21.2 (2023-11-10)
-------------------

0.21.1 (2023-11-09)
-------------------

0.21.0 (2023-11-07)
-------------------

0.20.2 (2023-10-26)
-------------------

0.20.1 (2023-10-13)
-------------------
* Replaced the implementation of tesseract_kinematics::parseSceneGraph (`#947 <https://github.com/tesseract-robotics/tesseract/issues/947>`_)
* Unused includes cleanup (`#946 <https://github.com/tesseract-robotics/tesseract/issues/946>`_)
* Add Eigen::Vector3d yaml support (`#945 <https://github.com/tesseract-robotics/tesseract/issues/945>`_)
* Merge pull request `#943 <https://github.com/tesseract-robotics/tesseract/issues/943>`_ from marrts/fix/ik_fast/free_joint_state_parsing
  Make `free_joint_state` parsing more generalized
* Make `free_joint_state` parsing more generalized
* Contributors: Levi Armstrong, Roelof, Tyler Marr

0.20.0 (2023-09-27)
-------------------
* Add support for KDL::ChainIkSolverPos_NR_JL, which takes joint limits (`#928 <https://github.com/tesseract-robotics/tesseract/issues/928>`_)
* Contributors: Roelof

0.19.2 (2023-09-06)
-------------------

0.19.1 (2023-09-05)
-------------------

0.19.0 (2023-09-05)
-------------------
* Update kinematics and collision packages to leverage cmake components (`#927 <https://github.com/tesseract-robotics/tesseract/issues/927>`_)
* Update emails
* Improved reporting of errors in IK plugin loader (`#924 <https://github.com/tesseract-robotics/tesseract/issues/924>`_)
* Added IKFast factory boilerplate (`#916 <https://github.com/tesseract-robotics/tesseract/issues/916>`_)
  * Added IKFast factory boilerplate
  * Updated IKFast unit tests to use new IKFast boilerplate factory
  * Reverted change to kinematic_group.h
  * Add include guards
  * Fix filename in header comment
  Co-authored-by: Tyler Marr <41449746+marrts@users.noreply.github.com>
  * IKFast plugins as cpp instead of h
  * Fix ikfast clang-tidy issues
  ---------
  Co-authored-by: Tyler Marr <41449746+marrts@users.noreply.github.com>
  Co-authored-by: Levi Armstrong <levi.armstrong@gmail.com>
* Fix of ManipulatorInfo and typos (`#914 <https://github.com/tesseract-robotics/tesseract/issues/914>`_)
  - ManipulatorInfo constructor now accepts tcp_offset as variant to match data member.
  - Fixed typos in rep and rop factories.
* Contributors: Levi Armstrong, Michael Ripperger, Roelof

0.18.1 (2023-06-30)
-------------------

0.18.0 (2023-06-29)
-------------------
* Update kinematics group inverse kinematics to harmonize within joint limits (`#899 <https://github.com/tesseract-robotics/tesseract/issues/899>`_)
* Add package cmake flags for testing, examples and benchmarks
* Contributors: John Wason, Levi Armstrong

0.17.0 (2023-06-06)
-------------------

0.16.3 (2023-05-04)
-------------------

0.16.2 (2023-04-28)
-------------------

0.16.1 (2023-04-11)
-------------------
* Improve tesseract_kinematics code coverage
* Contributors: Levi Armstrong

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
* Contributors: Levi Armstrong, Roelof Oomen

0.14.0 (2022-10-23)
-------------------
* Remove deprecated items
* Fix codecov build using ros_industrial_cmake_boilerplate 0.3.1
* Contributors: Levi Armstrong

0.13.1 (2022-08-25)
-------------------
* Move most SWIG commands to tesseract_python package (`#809 <https://github.com/tesseract-robotics/tesseract/issues/809>`_)
* Contributors: John Wason

0.13.0 (2022-07-11)
-------------------
* Update code based on clang-tidy-14
* Make limits utility functions templates
* Contributors: Levi Armstrong

0.10.0 (2022-07-06)
-------------------
* Update ros_industrial_cmake_boilerplate to 0.3.0 (`#795 <https://github.com/tesseract-robotics/tesseract/issues/795>`_)
* Static plugin loading using symbol module resolution (`#782 <https://github.com/tesseract-robotics/tesseract/issues/782>`_)

0.9.11 (2022-06-30)
-------------------
* Improve manipulability calculcation (`#787 <https://github.com/tesseract-robotics/tesseract/issues/787>`_)
  * Use SelfAdjointEigenSolver
  * Simplify calculation of measure = sqrt(condition)
  * Simplify calculation of volume
  Co-authored-by: christian.petersmeier <christian.petersmeier@uni-bielefeld.de>
* Updated CPack (`#786 <https://github.com/tesseract-robotics/tesseract/issues/786>`_)
* Add lapack test_depends tesseract_kinematics
* Update to use find_gtest macro
* Contributors: Levi Armstrong, Michael Ripperger, Robert Haschke

0.9.10 (2022-06-14)
-------------------

0.9.9 (2022-05-30)
------------------

0.9.8 (2022-05-30)
------------------

0.9.7 (2022-05-30)
------------------
* Fix numerical issue in manipulability calculation
* Contributors: Levi Armstrong

0.9.6 (2022-05-02)
------------------

0.9.5 (2022-04-24)
------------------

0.9.4 (2022-04-22)
------------------

0.9.3 (2022-04-18)
------------------
* Updated plugin capability to support sections (`#741 <https://github.com/tesseract-robotics/tesseract/issues/741>`_)
* Contributors: Levi Armstrong

0.9.2 (2022-04-03)
------------------

0.9.1 (2022-04-01)
------------------

0.9.0 (2022-03-31)
------------------
* Make ResourceLocator serializable
* Contributors: Levi Armstrong

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
