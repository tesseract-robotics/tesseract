^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tesseract_scene_graph
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.0 (2021-07-02)
------------------

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
* Add cmake format
* Add support for defining collision margin data in SRDF (`#573 <https://github.com/ros-industrial-consortium/tesseract/issues/573>`_)
* Use boost targets, add cpack and license file (`#572 <https://github.com/ros-industrial-consortium/tesseract/issues/572>`_)
* Fix the way in which Eigen is included (`#570 <https://github.com/ros-industrial-consortium/tesseract/issues/570>`_)
* Add ability to construct ROP and REP kinematic solver with different solver names
* Contributors: Hervé Audren, Levi Armstrong

0.2.0 (2021-02-17)
------------------
* Add getSourceLink, getTargetLink, getInboundJoints and getOutboundJoints scene graph unit test
* Add manipulator manager unit tests
* Add support for replacing links and joints
* Switch addJoint, addLink, moveLink and addSceneGraph to use const&
* Fix scene graph default visibility and collision enabled
* Refactor tesseract_environment to use applyCommands
* Improve tesseract_scene_graph test coverage and fix found issues
* Update cmake_common_scripts to ros_industrial_cmake_boilerplate
* Add tesseract_scene_graph resource locator unit test
* Fix bug in SimpleLocateResource::getResourceContentStream
* Improve srdf test coverage
* Add more unit tests for scene graph
* Fix bug in scene graph change joint limits
* Add missing depend in tesseract_scene_graph
* Move all directories in tesseract directory up one level
* Contributors: Levi Armstrong

0.1.0 (2020-12-31)
------------------
* Add getAdjacentLinkNames to tesseract_scene_graph
* Add moveJoint method to scene graph
* Make minor fixes to tesseract_scene_graph
* Add parser for SceneGraph to KDL Tree
* Expose graph getVertex and getEdge method
* Add urdf parser and tests to tesseract_scene_graph
* Update mesh parser to include convex hulls and add unit tests
* Add mesh parser test
* Add mesh parser to tesseract_scene_graph
* Add macros.h
* Add tesseract_scene_graph
* Contributors: Levi Armstrong
