^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tesseract_urdf
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
