^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tesseract_collision
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.27.1 (2024-12-03)
-------------------

0.27.0 (2024-12-01)
-------------------
* Update package.xml
* Update fcl rosdep key in package.xml
* Contributors: Levi Armstrong

0.26.0 (2024-10-27)
-------------------
* Remove TesseractSupportResourceLocator
* Contributors: Levi Armstrong

0.25.0 (2024-09-28)
-------------------
* Add geometry type CompoundMesh
* Contributors: Levi Armstrong

0.24.1 (2024-08-19)
-------------------

0.24.0 (2024-08-14)
-------------------
* Add any poly support for collision types
* Contributors: Levi Armstrong

0.23.1 (2024-07-28)
-------------------

0.23.0 (2024-07-24)
-------------------
* Do not export plugin libraries (`#1028 <https://github.com/tesseract-robotics/tesseract/issues/1028>`_)
* Contributors: Levi Armstrong

0.22.2 (2024-06-10)
-------------------
* Add ability to briefly summarize trajectory collisions (`#1011 <https://github.com/tesseract-robotics/tesseract/issues/1011>`_)
* Contributors: Tyler Marr

0.22.1 (2024-06-03)
-------------------

0.22.0 (2024-06-02)
-------------------
* Fix 2 for older bullet versions (e.g. on Ubuntu Focal)
* Fix for older bullet versions (e.g. on Ubuntu Focal)
* - Apply [bugfix](https://github.com/bulletphysics/bullet3/commit/fce12964139c8073676a50db0201c1460ad3fcad) and [bugfix](https://github.com/bulletphysics/bullet3/commit/a808d7489500935ac6665dcfe930c43a2008566b) from bullet
  - Clang-tidy fix
* Fix map type in comment of ContactResultMap
  See `#991 <https://github.com/tesseract-robotics/tesseract/issues/991>`_
* Leverage forward declarations to improve compile times (`#990 <https://github.com/tesseract-robotics/tesseract/issues/990>`_)
* Added application for performing convex decomposition (`#968 <https://github.com/tesseract-robotics/tesseract/issues/968>`_)
* Fix default test type in comment (`#980 <https://github.com/tesseract-robotics/tesseract/issues/980>`_)
* clang-format
* - Fixes rounding errors in progress logging (went up to 101%)
  - Fixes some clang-tidy suggestions
* Contributors: Levi Armstrong, Michael Ripperger, Roelof, Roelof Oomen

0.21.5 (2023-12-14)
-------------------
* Add Mac OSX support (`#969 <https://github.com/tesseract-robotics/tesseract/issues/969>`_)
* Contributors: John Wason

0.21.4 (2023-11-20)
-------------------

0.21.3 (2023-11-16)
-------------------

0.21.2 (2023-11-10)
-------------------

0.21.1 (2023-11-09)
-------------------

0.21.0 (2023-11-07)
-------------------
* Fix FCL contact test when calculate distance is false but contact threshold is greater than zero
* Add collision unit test with calculate distance disabled
* Contributors: Levi Armstrong

0.20.2 (2023-10-26)
-------------------

0.20.1 (2023-10-13)
-------------------
* Unused includes cleanup (`#946 <https://github.com/tesseract-robotics/tesseract/issues/946>`_)
* Contributors: Roelof

0.20.0 (2023-09-27)
-------------------

0.19.2 (2023-09-06)
-------------------
* Fix Ubunut Jammy release build
* Contributors: Levi Armstrong

0.19.1 (2023-09-05)
-------------------
* Fix benchmark build
* Contributors: Levi Armstrong

0.19.0 (2023-09-05)
-------------------
* Update kinematics and collision packages to leverage cmake components (`#927 <https://github.com/tesseract-robotics/tesseract/issues/927>`_)
* Update emails
* Contributors: Levi Armstrong

0.18.1 (2023-06-30)
-------------------
* Fix convert convex hull to set correct creation method
* Contributors: Levi Armstrong

0.18.0 (2023-06-29)
-------------------
* Update kinematics group inverse kinematics to harmonize within joint limits (`#899 <https://github.com/tesseract-robotics/tesseract/issues/899>`_)
* Trajectory logging fixup (`#908 <https://github.com/tesseract-robotics/tesseract/issues/908>`_)
* Improve Trajectory Collision Logging (`#765 <https://github.com/tesseract-robotics/tesseract/issues/765>`_)
* Add package cmake flags for testing, examples and benchmarks
* Add assert to ContactResultsMap to make sure key is an ordered pair
* Fix makeConvexMesh to pass through scale used on resource
* Update VHACD to latest (v4.1, tag 454913f) (`#896 <https://github.com/tesseract-robotics/tesseract/issues/896>`_)
* Contributors: John Wason, Levi Armstrong, Roelof, Tyler Marr

0.17.0 (2023-06-06)
-------------------
* Remove invalid assert from FCL collision and distance callback functions
* Contributors: Levi Armstrong

0.16.3 (2023-05-04)
-------------------

0.16.2 (2023-04-28)
-------------------

0.16.1 (2023-04-11)
-------------------

0.16.0 (2023-04-09)
-------------------
* Improve collision code coverage
* Add ContactResultMap shrinkToFit and CollisionCheckProgramType
* Fix ContactResultMap serialization
* Add AddTrajectoryLinkCommand
* Add documentation to ContactResultMap
* Remove reserve(100) in ContactResultMap does not improve performance
* Add contact results class
* Contributors: Levi Armstrong

0.15.3 (2023-03-22)
-------------------
* Update tesseract_collision benchmarks (`#868 <https://github.com/tesseract-robotics/tesseract/issues/868>`_)
* Add ContactResult boost serialization
* Contributors: Levi Armstrong

0.15.2 (2023-03-15)
-------------------
* Expose Bullet collision pool allocator configuration
* Switch include in tesseract_collision
* Contributors: Levi Armstrong

0.15.1 (2023-03-14)
-------------------
* Add flattenWrapperResults methods
* Contributors: Levi Armstrong

0.15.0 (2023-03-03)
-------------------
* Update collision benchmarks
* Remove unused fcl selfCollisionContactTest method
* Improve tesseract_collision code coverage
* Fix fcl unregistar bug
* Contributors: Levi Armstrong

0.14.0 (2022-10-23)
-------------------
* Remove deprecated items
* Fix codecov build using ros_industrial_cmake_boilerplate 0.3.1
* Contributors: Levi Armstrong

0.13.1 (2022-08-25)
-------------------
* Move most SWIG commands to tesseract_python package (`#809 <https://github.com/tesseract-robotics/tesseract/issues/809>`_)
* Add find_bullet macro which creates a target to link against (`#803 <https://github.com/tesseract-robotics/tesseract/issues/803>`_)
* Contributors: John Wason, Levi Armstrong

0.13.0 (2022-07-11)
-------------------
* Update code based on clang-tidy-14
* Contributors: Levi Armstrong

0.10.0 (2022-07-06)
-------------------
* Update ros_industrial_cmake_boilerplate to 0.3.0 (`#795 <https://github.com/tesseract-robotics/tesseract/issues/795>`_)
* Static plugin loading using symbol module resolution (`#782 <https://github.com/tesseract-robotics/tesseract/issues/782>`_)

0.9.11 (2022-06-30)
-------------------
* Updated CPack (`#786 <https://github.com/tesseract-robotics/tesseract/issues/786>`_)
* Fix benchmark CI
* Minor fixes
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
* Reduce bullet octomap storage
* Allow not providing contact manager plugins
* Contributors: Levi Armstrong

0.9.6 (2022-05-02)
------------------

0.9.5 (2022-04-24)
------------------

0.9.4 (2022-04-22)
------------------

0.9.3 (2022-04-18)
------------------
* Fix invalid iterator in bullet_cast_simple_manager (`#746 <https://github.com/tesseract-robotics/tesseract/issues/746>`_)
  * Fix invalid iterator in bullet_cast_simple_manager
  * clang format
* Updated plugin capability to support sections (`#741 <https://github.com/tesseract-robotics/tesseract/issues/741>`_)
* Contributors: John Wason, Levi Armstrong

0.9.2 (2022-04-03)
------------------

0.9.1 (2022-04-01)
------------------

0.9.0 (2022-03-31)
------------------

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
* Contributors: Matthew Powelson

0.8.4 (2022-03-03)
------------------
* Add TESSERACT_ENABLE_EXAMPLES compile option
* Contributors: John Wason

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
* Remove unneeded boost bind include
  Not needed since C++11 and this header puts placeholder objects in the
  global namespace on system-installed Boost versions
* Contributors: Josh Langsfeld

0.8.1 (2022-01-24)
------------------

0.8.0 (2022-01-19)
------------------

0.7.5 (2022-01-10)
------------------
* Add creation method to convex mesh
* Produce cmake error if libraries provided by libbullet-extras are not… (`#688 <https://github.com/tesseract-robotics/tesseract/issues/688>`_)
* Add ability to check if collision object is enabled (`#687 <https://github.com/tesseract-robotics/tesseract/issues/687>`_)
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
* Add modify_object_enabled to ContactManagerConfig
* Contributors: Levi Armstrong, Matthew Powelson

0.7.0 (2021-12-04)
------------------
* Rename member variables of ContactManagerConfig
* Add ContactManagerConfig inside CollisionCheckConfig
  This separates the up front setup things for the contact manager from things specific to the contactTest or the way the contact manager should be called.
* Add applyCollisionCheckConfig to contact managers
* Add AllowedCollisionMatrix to CollisionCheckConfig
* Move AllowedCollisionMatrix into tesseract_common
* Contributors: Levi Armstrong, Matthew Powelson

0.6.9 (2021-11-29)
------------------
* Fix CollisionCheckConfig to set collision_margin_override_type for constructor
* Contributors: Levi Armstrong

0.6.8 (2021-11-29)
------------------
* Add contact margin data override type MODIFY (`#669 <https://github.com/tesseract-robotics/tesseract/issues/669>`_)
  * Add contact margin data override type MODIFY
  * Add unit test for type MODIFY
* Fix spelling errors
* Contributors: Levi Armstrong

0.6.7 (2021-11-16)
------------------
* Fix linking issue when building repo alongside debian releae
* Contributors: Levi Armstrong

0.6.6 (2021-11-10)
------------------
* Update ikfast plugin
* Update tesseract_collision benchmarks
* Contributors: Levi-Armstrong

0.5.0 (2021-07-02)
------------------
* Add convex decomposition support (`#609 <https://github.com/ros-industrial-consortium/tesseract/issues/609>`_)
* Contributors: Levi Armstrong

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
