^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tesseract_geometry
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
