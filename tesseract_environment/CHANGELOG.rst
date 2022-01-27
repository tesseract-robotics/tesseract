^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tesseract_environment
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.8.2 (2022-01-27)
------------------
* Add ability to provide calibration information in the SRDF (`#703 <https://github.com/tesseract-robotics/tesseract/issues/703>`_)
  * Add missing package tesseract_srdf in CI after script
  * Add support for calibration info in SRDF
* Contributors: Levi Armstrong

0.8.1 (2022-01-24)
------------------

0.8.0 (2022-01-19)
------------------
* Fix check trajectory which should return a vector same length as trajectory (`#698 <https://github.com/tesseract-robotics/tesseract/issues/698>`_)
* Update Kinematics Cache To Include IK Solver (`#695 <https://github.com/tesseract-robotics/tesseract/issues/695>`_)
* Contributors: Levi Armstrong, marrts

0.7.5 (2022-01-10)
------------------
* Updated environment benchmark (`#694 <https://github.com/tesseract-robotics/tesseract/issues/694>`_)
* Update library names in benchmarks (`#681 <https://github.com/tesseract-robotics/tesseract/issues/681>`_)
* Contributors: Matthew Powelson, Michael Ripperger

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
* Add modify_object_enabled to ContactManagerConfig
* Contributors: Levi Armstrong, Matthew Powelson

0.7.0 (2021-12-04)
------------------
* Rename member variables of ContactManagerConfig
* Fix KinematicGroup and JointGroup cache to clear on current state changed
* Add ContactManagerConfig inside CollisionCheckConfig
  This separates the up front setup things for the contact manager from things specific to the contactTest or the way the contact manager should be called.
* Add unit test for checkTrajectoryState and checkTrajectorySegment
* Add applyCollisionCheckConfig to contact managers
* Add AllowedCollisionMatrix to CollisionCheckConfig
* Move AllowedCollisionMatrix into tesseract_common
* Correctly set the collision margin data in the environment utilities
* Contributors: Levi Armstrong, Matthew Powelson

0.6.9 (2021-11-29)
------------------

0.6.8 (2021-11-29)
------------------

0.6.7 (2021-11-16)
------------------

0.6.6 (2021-11-10)
------------------

0.5.0 (2021-07-02)
------------------
* Add convex decomposition support (`#609 <https://github.com/ros-industrial-consortium/tesseract/issues/609>`_)
* Fix environment clone benchmarks
* Remove deprecated code in tesseract_environment
* Store timestamp when environment state is set
* Contributors: Levi Armstrong

0.4.1 (2021-04-24)
------------------

0.4.0 (2021-04-23)
------------------
* Update tesseract_srdf to leverage nested exceptions
* Move srdf code to its own package tesseract_srdf
* Move printNestedException and leverage forward declarations for tesseract_urdf
* Do not catch exception in parseURDFString and parseURDFFile
* Contributors: Levi Armstrong

0.3.1 (2021-04-14)
------------------
* Move tesseract_variables() before any use of custom macros
* Contributors: Levi Armstrong

0.3.0 (2021-04-09)
------------------
* Only enable code coverage if compiler definition is set
* Fix issue in trajectory player setCurrentDuration not handling finished bool
* Fix bullet broadphase when new links are added
* Debug unit test
* Add cmake format
* Add support for defining collision margin data in SRDF (`#573 <https://github.com/ros-industrial-consortium/tesseract/issues/573>`_)
* Use boost targets, add cpack and license file (`#572 <https://github.com/ros-industrial-consortium/tesseract/issues/572>`_)
* Fix the way in which Eigen is included (`#570 <https://github.com/ros-industrial-consortium/tesseract/issues/570>`_)
* Add libomp-dev as test_depend to tesseract_environment and tesseract_collision
* Add multithreaded environment unit test
* Fix mutex locking bug in environment applyCommands
* Add ability to construct ROP and REP kinematic solver with different solver names
* Contributors: Hervé Audren, Levi Armstrong, Matthew Powelson

0.2.0 (2021-02-17)
------------------
* Add ability to replace link and joint pair where the link is the child link of joint
* Improve clone cache unit tests and fix issues with getting clone
* Add manipulator manager unit tests
* Add support for replacing links and joints
* Rename AddCommand to AddLinkCommand
* Update environment to leverage shared mutex
* Improve unit test coverage and registar FCL as an available contact manager
* Update StateSolver init to take a revision number
* Fix mutex dead lock in tesseract environment
* Switch addJoint, addLink, moveLink and addSceneGraph to use const&
* Improve tesseract_environment unit test coverage
* Refactor tesseract_environment to use applyCommands
* tesseract_environement: Improve documentation
* Update cmake_common_scripts to ros_industrial_cmake_boilerplate
* Move all directories in tesseract directory up one level
* Contributors: Levi Armstrong, Thomas Kostas

0.1.0 (2020-12-31)
------------------
* Add tesseract_environment package
* Create tesseract_environment and semi-isolate
* Contributors: Levi Armstrong
