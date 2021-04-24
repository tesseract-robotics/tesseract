^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tesseract_common
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
