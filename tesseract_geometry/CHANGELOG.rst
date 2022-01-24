^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tesseract_geometry
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

0.8.0 (2022-01-19)
------------------

0.7.5 (2022-01-10)
------------------
* Add creation method to convex mesh
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
* Improve creating octree from point cloud using lazy_eval (`#680 <https://github.com/tesseract-robotics/tesseract/issues/680>`_)
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

0.5.0 (2021-07-02)
------------------

0.4.1 (2021-04-24)
------------------

0.4.0 (2021-04-23)
------------------
* No changes

0.3.1 (2021-04-14)
------------------
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
* Update cmake_common_scripts to ros_industrial_cmake_boilerplate
* Move all directories in tesseract directory up one level
* Contributors: Levi Armstrong

0.1.0 (2020-12-31)
------------------
* Add missing include to tesseract_geometry/types.h
* Add asserts to mesh.h sdf_mesh.h to error if not triangle meshes
* Fix incorrect includes in cylinder.h and octree.h
* remove unused dependencies
* Add tesseract_geometry package and update tesseract_collision to leverage new package
* Contributors: Levi Armstrong
