^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tesseract_environment
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

0.27.0 (2024-12-01)
-------------------
* Fix environment clear such that cache is cleared before plugin factories are destroyed
* Contributors: Levi Armstrong

0.26.0 (2024-10-27)
-------------------
* Remove TesseractSupportResourceLocator
* Fix serialization
* Contributors: Levi Armstrong

0.25.0 (2024-09-28)
-------------------
* Only run kinematic check when testing is enabled and debug is enabled
* Contributors: Levi Armstrong

0.24.1 (2024-08-19)
-------------------
* Expose environment init revision
* Contributors: Levi Armstrong

0.24.0 (2024-08-14)
-------------------
* Fix clang detection on Mac OS (`#1038 <https://github.com/tesseract-robotics/tesseract/issues/1038>`_)
* Contributors: John Wason

0.23.1 (2024-07-28)
-------------------
* Cleanup boost serialization
* Add export implementation environment command
* Contributors: Levi Armstrong

0.23.0 (2024-07-24)
-------------------
* Add any poly support for environment
* Fixes for building on Ubuntu Noble (`#1016 <https://github.com/tesseract-robotics/tesseract/issues/1016>`_)
* Contributors: Levi Armstrong, Roelof Oomen

0.22.2 (2024-06-10)
-------------------
* Fix command serialization
* Contributors: Levi Armstrong

0.22.1 (2024-06-03)
-------------------

0.22.0 (2024-06-02)
-------------------
* Add more clone benchmarks
* Leverage forward declarations to improve compile times (`#990 <https://github.com/tesseract-robotics/tesseract/issues/990>`_)
* Add kinematics benchmarks
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

0.21.1 (2023-11-09)
-------------------

0.21.0 (2023-11-07)
-------------------

0.20.2 (2023-10-26)
-------------------
* Update environment clear method to clear all internal cache data
* Contributors: Levi Armstrong

0.20.1 (2023-10-13)
-------------------
* Unused includes cleanup (`#946 <https://github.com/tesseract-robotics/tesseract/issues/946>`_)
* Contributors: Roelof

0.20.0 (2023-09-27)
-------------------
* Fix missing locks in tesseract_environment (`#938 <https://github.com/tesseract-robotics/tesseract/issues/938>`_)
* Contributors: Roelof

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
* Update kinematics group inverse kinematics to harmonize within joint limits (`#899 <https://github.com/tesseract-robotics/tesseract/issues/899>`_)
* Trajectory logging fixup (`#908 <https://github.com/tesseract-robotics/tesseract/issues/908>`_)
* Improve Trajectory Collision Logging (`#765 <https://github.com/tesseract-robotics/tesseract/issues/765>`_)
* Add package cmake flags for testing, examples and benchmarks
* Contributors: John Wason, Levi Armstrong, Tyler Marr

0.17.0 (2023-06-06)
-------------------
* Windows updates (`#893 <https://github.com/tesseract-robotics/tesseract/issues/893>`_)
* Fix check trajectory print statements (`#892 <https://github.com/tesseract-robotics/tesseract/issues/892>`_)
* implemented benchmarking for checking trajectories (`#887 <https://github.com/tesseract-robotics/tesseract/issues/887>`_)
* Contributors: John Wason, Levi Armstrong, Tyler Marr

0.16.3 (2023-05-04)
-------------------

0.16.2 (2023-04-28)
-------------------

0.16.1 (2023-04-11)
-------------------
* Add unit tests to cover new addTrajectoryLinkCommand
* Contributors: Levi Armstrong

0.16.0 (2023-04-09)
-------------------
* Improve collision code coverage
* Add ContactResultMap shrinkToFit and CollisionCheckProgramType
* Add AddTrajectoryLinkCommand
* General cleanup of commands moving things to cpp
* Add contact results class
* Contributors: Levi Armstrong

0.15.3 (2023-03-22)
-------------------

0.15.2 (2023-03-15)
-------------------
* Switch include in tesseract_collision
* Contributors: Levi Armstrong

0.15.1 (2023-03-14)
-------------------

0.15.0 (2023-03-03)
-------------------
* Improve tesseract_environment code coverage
* Contributors: Levi Armstrong

0.14.0 (2022-10-23)
-------------------
* Add modify allowed collisions command
* Add forgotten source file extension to CMakeLists.
* Fix issue where cache is emptied by other threads after refresh causing segfault when popping environment on empty queue
* Add EnvironmentCache and DefaultEnvironmentCache
* Including <boost/serialization/library_version_type.hpp> for Boost 1.74. Fixes `tesseract-robotics/tesseract#764 <https://github.com/tesseract-robotics/tesseract/issues/764>`_
* Contributors: Levi Armstrong, Roelof Oomen

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
* Make missing contact manager plugins a debug vs warn message
* Contributors: Levi Armstrong

0.9.9 (2022-05-30)
------------------

0.9.8 (2022-05-30)
------------------

0.9.7 (2022-05-30)
------------------
* Add environment discrete_manager_mutex_ and continuous_manager_mutex_
* Allow not providing contact manager plugins
* Add the ability to set the environment discrete and continuous manager to nullptr to save space when needed
* Contributors: Levi Armstrong

0.9.6 (2022-05-02)
------------------

0.9.5 (2022-04-24)
------------------

0.9.4 (2022-04-22)
------------------

0.9.3 (2022-04-18)
------------------
* Enable ability to remove event callback
* Add environment serialization
* Updated plugin capability to support sections (`#741 <https://github.com/tesseract-robotics/tesseract/issues/741>`_)
* Update triggering of event callbacks to take a shared lock
* Contributors: Levi Armstrong

0.9.2 (2022-04-03)
------------------
* Add timestamp to environment
* Contributors: Levi Armstrong

0.9.1 (2022-04-01)
------------------

0.9.0 (2022-03-31)
------------------
* Make ResourceLocator serializable
* Add environment monitor interfaces
* Add event callbacks to environment
* Add tcp to iiwa srdf
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
* Add methods for getting link transform information from state solver
* Contributors: Levi Armstrong, Matthew Powelson

0.8.4 (2022-03-03)
------------------
* Add method to environment to get relative link transform
* Contributors: Levi Armstrong

0.8.3 (2022-02-22)
------------------
* Python patches for Feb 2022 update (`#716 <https://github.com/tesseract-robotics/tesseract/issues/716>`_)
* A few fixes that were needed for Windows (`#708 <https://github.com/tesseract-robotics/tesseract/issues/708>`_)
  * Make HACDConvexDecomposition library optional
  Bullet extras are not easily obtained on Windows. If found, build library, otherwise ignore. Also the plain ConvexDecomposition library is looked for but never used and so removed entirely.
  * Check if Bullet CMake variables are using absolute paths
  For some reasons, the vcpkg ported version changes the config file to
  use absolute paths instead of relative to BULLET_ROOT_DIR
  * Add include for std::string
  Co-authored-by: Levi Armstrong <levi.armstrong@gmail.com>
* Contributors: John Wason, Josh Langsfeld

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
