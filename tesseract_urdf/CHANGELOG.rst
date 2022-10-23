^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tesseract_urdf
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.14.0 (2022-10-23)
-------------------

0.13.1 (2022-08-25)
-------------------

0.13.0 (2022-07-11)
-------------------
* Update code based on clang-tidy-14
* Fixed convex mesh export (`#799 <https://github.com/tesseract-robotics/tesseract/issues/799>`_)
* Contributors: Levi Armstrong, Michael Ripperger

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

0.9.4 (2022-04-22)
------------------
* Use components for PCL (`#752 <https://github.com/tesseract-robotics/tesseract/issues/752>`_)
  * Find components common and io for PCL
  * Include PCL_INCLUDE_DIRS if target not available
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
* cmake format
* Add TESSERACT_ENABLE_EXAMPLES compile option
* Contributors: John Wason

0.8.3 (2022-02-22)
------------------
* Fix Boost_VERSION in tesseract_urdf to check for Boost_VERSION_MACRO (`#717 <https://github.com/tesseract-robotics/tesseract/issues/717>`_)
  * Fix Boost_VERSION in tesseract_urdf to check for Boost_VERSION_MACRO
  * cmake-format
* Modifying Boost_VERSION check to use semver
* Contributors: John Wason, Kyle Staub

0.8.2 (2022-01-27)
------------------

0.8.1 (2022-01-24)
------------------

0.8.0 (2022-01-19)
------------------

0.7.5 (2022-01-10)
------------------
* Add creation method to convex mesh
* URDF Writer: Small Bug Fixes
* Contributors: David Merz, Jr, Levi Armstrong

0.7.4 (2021-12-15)
------------------

0.7.3 (2021-12-15)
------------------

0.7.2 (2021-12-15)
------------------

0.7.1 (2021-12-15)
------------------
* Improve creating octree from point cloud using lazy_eval (`#680 <https://github.com/tesseract-robotics/tesseract/issues/680>`_)
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
* Improve tesseract_common unit test coverage
* Improve exception text in urdf_parser
* Fix package build depends
* Move printNestedException and leverage forward declarations for tesseract_urdf
* Do not catch exception in parseURDFString and parseURDFFile
* Move tesseract_urdf implementation to cpp and fix clang tidy errors
* Improve tesseract_urdf unit test coverage
* Switch tesseract_urdf to use nested exception instead of custom status code class
* Contributors: Levi Armstrong

0.3.1 (2021-04-14)
------------------
* Add missing pcl depends to tesseract_urdf package.xml
* Move tesseract_variables() before any use of custom macros
* Contributors: Levi Armstrong

0.3.0 (2021-04-09)
------------------
* Only enable code coverage if compiler definition is set
* Add cmake format
* Use boost targets, add cpack and license file (`#572 <https://github.com/ros-industrial-consortium/tesseract/issues/572>`_)
* Fix the way in which Eigen is included (`#570 <https://github.com/ros-industrial-consortium/tesseract/issues/570>`_)
* Contributors: Hervé Audren, Levi Armstrong

0.2.0 (2021-02-17)
------------------
* Switch addJoint, addLink, moveLink and addSceneGraph to use const&
* Fix scene graph default visibility and collision enabled
* Update cmake_common_scripts to ros_industrial_cmake_boilerplate
* Move all directories in tesseract directory up one level
* Contributors: Levi Armstrong

0.1.0 (2020-12-31)
------------------
