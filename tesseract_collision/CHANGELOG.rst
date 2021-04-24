^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tesseract_collision
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.1 (2021-04-24)
------------------

0.4.0 (2021-04-23)
------------------
* Fix package build depends
* Contributors: Levi Armstrong

0.3.1 (2021-04-14)
------------------
* Add bullet-extras depends to tesseract_collision package.xml
* Move tesseract_variables() before any use of custom macros
* Contributors: Levi Armstrong

0.3.0 (2021-04-09)
------------------
* Only enable code coverage if compiler definition is set
* Fix issue in trajectory player setCurrentDuration not handling finished bool
* Fix bullet broadphase when new links are added
* Debug unit test
* Fix conversion warnings
  - Use size_t everywhere we expect to index a vector
  - Cast the result of rand unsigned
* Update benchmarks to use collision margin data
* Make compatible with fcl version 0.6
* Add cmake format
* Add support for defining collision margin data in SRDF (`#573 <https://github.com/ros-industrial-consortium/tesseract/issues/573>`_)
* Use boost targets, add cpack and license file (`#572 <https://github.com/ros-industrial-consortium/tesseract/issues/572>`_)
* Fix the way in which Eigen is included (`#570 <https://github.com/ros-industrial-consortium/tesseract/issues/570>`_)
* Add logic to how a provided collision margin data can be applied
* Fix method for updating max margin in CollisionMarginData
* Add libomp-dev as test_depend to tesseract_environment and tesseract_collision
* Fix method for changing bullet extern gDbvtMargin
* Contributors: Hervé Audren, Levi Armstrong, Matthew Powelson, david.hooks

0.2.0 (2021-02-17)
------------------
* Add utility function to scale vertices about a point
* Improve tesseract_environment unit test coverage
* Update cmake_common_scripts to ros_industrial_cmake_boilerplate
* Move all directories in tesseract directory up one level
* Contributors: Levi Armstrong

0.1.0 (2020-12-31)
------------------
* Remove export library from tesseract_collision that does not exist
* Add tesseract_geometry package and update tesseract_collision to leverage new package
* Make minor fixes in tesseract_collision
* Update create_convex_hull to not use ros
* Switch to using console bridge
* Isolate tesseract_collision namespace
* Switch to using built in Collision Shapes
* Fix clang formating
* Fixes to run_clang_format_check
* Fix formatting using clang
* Fix warnings in unit tests
* Update due to changes in FCL Convex Shape Constructor
* Add additional compiler warning options
* Ignore unused param warnings in bullet
* Add EIGEN_MAKE_ALIGNED_OPERATOR_NEW macros
* Disable tesseract_collision FCL ConvexHull tests
* Fix/Clean depends in CMakeLists.txt and package.xml for travis-ci
* Merge pull request `#41 <https://github.com/ros-industrial-consortium/tesseract/issues/41>`_ from Levi-Armstrong/issue/FixMultiLayerCompoundShape
  Fix use of multi layer compound shape
  Fix/add cmake install commands
* Fix cmake install commands
* Fix use of multi layer compound shape
* Merge pull request `#40 <https://github.com/ros-industrial-consortium/tesseract/issues/40>`_ from Levi-Armstrong/feature/RemoveContactRequestStruct
  Refractor out ContactRequest type
* Refractor out ContactRequest type
* Merge pull request `#39 <https://github.com/ros-industrial-consortium/tesseract/issues/39>`_ from Levi-Armstrong/issue/FixBulletCast
  This fixes the continuous collision checking
* Fix the use of ContactRequestType::FIRST with broadphase
* Fix cast bvh manager
* Fix bullet continous collision checking
* Merge pull request `#34 <https://github.com/ros-industrial-consortium/tesseract/issues/34>`_ from Levi-Armstrong/issue/FixBulletCast
  * This fixes the bullet cast simple manager
  * Fix the plotting of frames
  * Add unit test when using change base in kdl kin
  * Remove bullet build flags.
  * When adding use double precision this causes trajopt_ros test to fail. I believe this is due to inaccuracies in the EPA algorithm.
* Remove bullet build flags
  This for some reason causes TrajOpt to fail most likely due to bad results from the EPA algorithm
* Fix compound and children aabb when updating cast transform
* Fix bullet cast simple manager
* Restructure bullet managers to be in separate files
* Merge pull request `#32 <https://github.com/ros-industrial-consortium/tesseract/issues/32>`_ from Levi-Armstrong/issue/testCollisionClone
  Add unit test for clone method and fix mesh to mesh unit test names
* Add unit test for clone method and fix mesh to mesh unit test names
* Merge pull request `#33 <https://github.com/ros-industrial-consortium/tesseract/issues/33>`_ from Levi-Armstrong/issue/fixPluginDescription
  Fix namespace in plugin description
* Fix namespace in plugin description
* Merge pull request `#29 <https://github.com/ros-industrial-consortium/tesseract/issues/29>`_ from Levi-Armstrong/issue/addCollisionNamespaces
  Add namespaces specific to collision implementation
* Fix lambda functions
* Add Bullet detailed mesh to detailed mesh collision checking along with unit test
* Adjust test to run for both primitive and convex shape.
* Add namespaces specific to collision implementation
* Merge pull request `#26 <https://github.com/ros-industrial-consortium/tesseract/issues/26>`_ from Levi-Armstrong/issue/FixContactMonitor
  Update contact monitor to use the latest version
* Merge pull request `#28 <https://github.com/ros-industrial-consortium/tesseract/issues/28>`_ from Jmeyer1292/fix/bullet_include
  Bullet Convex Hull Computer Include
* Corrected include file path to work with the bullet3_ros package include paths
* Fix asserts in CollisionObjectWrapper for bullet and fcl
* Merge pull request `#23 <https://github.com/ros-industrial-consortium/tesseract/issues/23>`_ from Levi-Armstrong/feature/addFCLNew
  Add fcl discrete collision manager
* Make requested changes and fixes
* Add ros node for creating convex hull meshes
* Add fcl convex hull support and update tests
* Fix bullet cast assert in setCollisionObjectsTransform
* Add FCL discrete manager
* Merge pull request `#20 <https://github.com/ros-industrial-consortium/tesseract/issues/20>`_ from Levi-Armstrong/feature/Isometry3d
  switch from using affine3d to isometry3d
* Add large octomap collision unit test enable aabb tree for compound shapes
* switch from using affine3d to isometry3d
* Merge pull request `#15 <https://github.com/ros-industrial-consortium/tesseract/issues/15>`_ from Levi-Armstrong/feature/largeDataSetTest
  Restructure Collision Checking for Performance Improvements
* Run clang-format
* Restructure Collision Checking for Performance Improvements
* Merge pull request `#1 <https://github.com/ros-industrial-consortium/tesseract/issues/1>`_ from Levi-Armstrong/fixSubmodule
  Fix submodule for bullet3
* Fix submodule for bullet3
* Move tesseract into its own repository
* Contributors: Alessio Rocchi, Jonathan Meyer, Levi, Levi Armstrong, Matthew Powelson
