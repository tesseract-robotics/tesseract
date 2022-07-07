^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tesseract_common
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.10.0 (2022-07-06)
-------------------
* Update ros_industrial_cmake_boilerplate to 0.3.0 (`#795 <https://github.com/tesseract-robotics/tesseract/issues/795>`_)
* Static plugin loading using symbol module resolution (`#782 <https://github.com/tesseract-robotics/tesseract/issues/782>`_)

0.9.11 (2022-06-30)
-------------------
* Renames in type erasure to avoid WIN32 defines
* Updated CPack (`#786 <https://github.com/tesseract-robotics/tesseract/issues/786>`_)
* Update to use find_gtest macro
* Fix message in type_erasure.h
* Contributors: John Wason, Levi Armstrong, Michael Ripperger

0.9.10 (2022-06-14)
-------------------
* Add type erasure interface (`#776 <https://github.com/tesseract-robotics/tesseract/issues/776>`_)
  * Add type erasure interface
  * revert change to type erasure constructor
* Update FindTinyXML2.cmake
* Contributors: Levi Armstrong

0.9.9 (2022-05-30)
------------------
* Fix find tcmalloc on melodic
* Contributors: Levi Armstrong

0.9.8 (2022-05-30)
------------------
* Fix Findtcmalloc_minimal.cmake
* Contributors: Levi Armstrong

0.9.7 (2022-05-30)
------------------
* Update Findtcmalloc.cmake to include threads and split out tcmalloc_minimal to Findtcmalloc_minimal.cmake
* Contributors: Levi Armstrong

0.9.6 (2022-05-02)
------------------
* Normalize quaternion when decoding yaml Eigen::Isometry3d
* Contributors: Levi Armstrong

0.9.5 (2022-04-24)
------------------
* yaml_utils.h nullptr comparison fixup (`#755 <https://github.com/tesseract-robotics/tesseract/issues/755>`_)
* Fix JointTrajectory SWIG container (`#756 <https://github.com/tesseract-robotics/tesseract/issues/756>`_)
* Contributors: John Wason

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
* Make JointTrajectory a struct
* Add environment serialization
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
* Add atomic serialization
* Contributors: Levi Armstrong

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
* Adjust for breaking change in Boost DLL 1.76
* Contributors: Josh Langsfeld, Matthew Powelson

0.8.4 (2022-03-03)
------------------
* Set TESSERACT_ENABLE_EXAMPLES default to ON
* Add TESSERACT_ENABLE_EXAMPLES compile option
* Contributors: John Wason, Levi Armstrong

0.8.3 (2022-02-22)
------------------
* Python patches for Feb 2022 update (`#716 <https://github.com/tesseract-robotics/tesseract/issues/716>`_)
* Contributors: John Wason

0.8.2 (2022-01-27)
------------------
* Add ability to provide calibration information in the SRDF (`#703 <https://github.com/tesseract-robotics/tesseract/issues/703>`_)
  * Add missing package tesseract_srdf in CI after script
  * Add support for calibration info in SRDF
* Contributors: Levi Armstrong

0.8.1 (2022-01-24)
------------------
* Add any.cpp
* Contributors: Levi Armstrong

0.8.0 (2022-01-19)
------------------
* CPack Update (`#693 <https://github.com/tesseract-robotics/tesseract/issues/693>`_)
* Add BOOST_SERIALIZATION_ASSUME_ABSTRACT to Any type erasure
* Contributors: Levi Armstrong, Michael Ripperger

0.7.5 (2022-01-10)
------------------
* Add -Wdeprecated-declarations to push pop macros
* Contributors: Levi Armstrong

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
* Add contact margin data override type MODIFY (`#669 <https://github.com/tesseract-robotics/tesseract/issues/669>`_)
  * Add contact margin data override type MODIFY
  * Add unit test for type MODIFY
* Fix spelling errors
* Contributors: Levi Armstrong

0.6.7 (2021-11-16)
------------------

0.6.6 (2021-11-10)
------------------

0.5.0 (2021-07-02)
------------------
* Add convex decomposition support (`#609 <https://github.com/ros-industrial-consortium/tesseract/issues/609>`_)
* Contributors: Levi Armstrong

0.4.1 (2021-04-24)
------------------
* Remove windows compiler definition NOMINMAX
* Do not add compiler option -mno-avx if processor is uknown
* Contributors: Levi Armstrong

0.4.0 (2021-04-23)
------------------
* Add windows compile definition NOMINMAX
* Improve tesseract_common unit test coverage
* Add equal operator support to Any type erasure
* Fix package build depends
* Improve tesseract_common unit coverage
* Disable compile option -mno-avx for arm builds
* Move printNestedException and leverage forward declarations for tesseract_urdf
* Contributors: Levi Armstrong

0.3.1 (2021-04-14)
------------------
* Move tesseract_variables() before any use of custom macros
* Contributors: Levi Armstrong

0.3.0 (2021-04-09)
------------------
* Only enable code coverage if compiler definition is set
* Move serialize implementation to cpp based on boost documentation for shared libraries
* Rename Any method cast() and cast_const() to as()
* Remove NullAny structure
* Cleanup equal operator
* Fix satisfiesPositionLimits to use relative equal and calculation of redundant solutions to include all permutations
* Split loading plugins into two classes ClassLoader and PluginLoader
* Remove dependency on class_loader and leverage Boost DLL
* Add PluginLoader class to tesseract_common
* Fixup enforceJointLimits
  Up to now, it would incorrectly apply the upper limit to any position
  that's outside the range. For example, a position that's slightly under
  the lower limit would get assigned the upper limit. Fix this by using
  Eigen's min and max functions, resulting in a proper clamp.
* Add satisfy and enforce position limits utility functions (`#576 <https://github.com/ros-industrial-consortium/tesseract/issues/576>`_)
* Add QueryIntAttributeRequired utility function
* Add cmake format
* Add support for defining collision margin data in SRDF (`#573 <https://github.com/ros-industrial-consortium/tesseract/issues/573>`_)
* Use boost targets, add cpack and license file (`#572 <https://github.com/ros-industrial-consortium/tesseract/issues/572>`_)
* Fix the way in which Eigen is included (`#570 <https://github.com/ros-industrial-consortium/tesseract/issues/570>`_)
* Add serializable any type erasure (`#555 <https://github.com/ros-industrial-consortium/tesseract/issues/555>`_)
* Add ToolCenterPoint unit tests
* Start to adding boost serialization support
* Contributors: Hervé Audren, Levi Armstrong

0.2.0 (2021-02-17)
------------------
* Improve clone cache unit tests and fix issues with getting clone
* Allow almostEqualRelativeAndAbs handle empty vectors
* Refactor tesseract_environment to use applyCommands
* Add tesseract_common::BytesResource unit test (`#545 <https://github.com/ros-industrial-consortium/tesseract/issues/545>`_)
* Add simple timer class
* Add vectorized version of almostEqualRelativeAndAbs to compare if two vectors are equal
* Update cmake_common_scripts to ros_industrial_cmake_boilerplate
* Add marker support and remove dependency on command language
* Update Findtcmalloc.cmake to support windows
* Add Findtcmalloc.cmake file
* Move all directories in tesseract directory up one level
* Contributors: John Wason, Levi Armstrong, Matthew Powelson

0.1.0 (2020-12-31)
------------------
