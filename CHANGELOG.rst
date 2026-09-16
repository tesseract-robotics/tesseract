^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tesseract
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.35.0 (2026-05-28)
-------------------
* Add yaml node and string parsing to srdf configs (`#1302 <https://github.com/tesseract-robotics/tesseract/issues/1302>`_)
* Add optional tolerances on calcJacobianTransformErrorDiff (`#1299 <https://github.com/tesseract-robotics/tesseract/issues/1299>`_)
* Bump conda-incubator/setup-miniconda from 3 to 4 (`#1287 <https://github.com/tesseract-robotics/tesseract/issues/1287>`_)
* Fix silently failing Windows Conda tests (`#1301 <https://github.com/tesseract-robotics/tesseract/issues/1301>`_)
* Upate conda build
* Bump johnwason/vcpkg-action from 7 to 8
  Bumps [johnwason/vcpkg-action](https://github.com/johnwason/vcpkg-action) from 7 to 8.
  - [Release notes](https://github.com/johnwason/vcpkg-action/releases)
  - [Commits](https://github.com/johnwason/vcpkg-action/compare/v7...v8)
  ---
  updated-dependencies:
  - dependency-name: johnwason/vcpkg-action
  dependency-version: '8'
  dependency-type: direct:production
  update-type: version-update:semver-major
  ...
* Remove octomap from dependencies as deleteNodeChild issue when pruning octrees is fixed.
* Forward pruned/binary_octree flags through Octree clone and URDF parse
  clone() only forwarded octree\_ and sub_type\_ to the copy constructor,
  silently dropping pruned\_ and binary_octree\_ — clones miscompared
  against their originals (operator== checks pruned\_) and serialized the
  wrong values (cereal emits both fields, and binary_octree\_ controls
  whether the OcTree payload is written via writeBinary or write).
  parseOctree applied prune="true" to the OcTree but constructed the
  Octree geometry without the flag (defaulted to false). This caused
  writeOctomap to emit prune="false", cereal to serialize the wrong
  value, and operator== to miscompare round-tripped trees.
  Tests: extend parse_octree and add OctreeClonePreservesPrunedFlag /
  OctreeCloneBinaryFlag. binary_octree\_ has no public getter and is not
  checked by operator==, so the clone test for that field compares the
  cereal-XML output of the original vs clone, which differs in both the
  binary_octree NVP and the OcTree blob bytes (writeBinary vs write).
* Fix Octree::pruneNode assertion failure on octomap >= 1.9
  The custom pruneNode deleted children individually via deleteNodeChild()
  but never freed the children array itself. The array pointer (protected
  member) stayed non-null, triggering assert(children == NULL) in
  ~OcTreeDataNode on destruction. The assertion only fires under debug
  asserts (NDEBUG unset) with octomap >= 1.9, so default RelWithDebInfo
  builds silently leaked the children array instead of crashing.
  Equalize child values so octomap's own pruneNode() passes its stricter
  value-equality check, then delegate to it. Octomap's implementation has
  friend access to properly free the array. Tesseract's more aggressive
  collapsibility check (above occupancy threshold, not requiring equal
  values) still controls the decision.
* Include cassert
* Remove unused template-heavy serialization header from any_poly.h
* test(tesseract_collision): fix data race on shared location map in parallel test loop
  collision_multi_threaded_unit.hpp's OpenMP parallel region declared the
  caller-mutable `location` map as shared and iterated it via the non-const
  `begin()`/`end()` overloads on each thread. Per the C++ memory model,
  concurrent invocations of non-const member functions on the same object
  are a data race even when the operations don't actually mutate state.
  Bind a `const auto&` alias to `location` before the parallel region and
  iterate that alias instead, so all threads only invoke const member
  functions on the shared map.
  Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
* fix(scene_graph): kdl_sub_tree_builder crashed on non-empty floating joints
  The sub-tree builder constructor did
  data\_.floating_joint_values.at(key) = value
  on a freshly-default-constructed KDLTreeData whose map is always empty,
  so any non-empty floating_joint_values argument threw std::out_of_range.
  Use operator[] (insert-or-assign) like the sibling full-tree builder.
  Production impact: JointGroup construction threw on any scene with a
  floating joint. No existing test covered the path, hence the bug
  escaped detection.
  Add KDLParserSubTreeFloatingJointUnit which feeds parseSceneGraph a
  non-empty TransformMap and asserts both that the constructor no longer
  throws and that the transform is propagated into KDLTreeData.
* Fix typo in serialization key from 'iyz' to 'izz'
* ci: include scene_graph and support in pull_request path filters
  PRs touching only scene_graph/ or support/ matched no path filter and
  triggered no workflow (e.g. `#1293 <https://github.com/tesseract-robotics/tesseract/issues/1293>`_).
  - Add 'scene_graph/**' and 'support/**' to all seven PR-triggering workflows.
  - Sort the directory list alphabetically.
  - cmake_format.yml: also list 'CMakeLists.txt' and 'package.xml'; the
  existing '**cmake-format' glob doesn't match either.
  - windows.yml: move '**.repos' to end-of-list for consistency.
* Use stdint types for property tree (`#1282 <https://github.com/tesseract-robotics/tesseract/issues/1282>`_)
* Fix dangling references to yaml-cpp iterator proxy temporaries
  yaml-cpp's iterator::operator->() returns a proxy by value, so binding
  `const YAML::Node&` to `it->second` dangles after the full expression.
  ASan caught this as stack-use-after-scope. Copy by value instead —
  Node is a cheap reference-counted handle.
  Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
* Update Mac OS build action (`#1283 <https://github.com/tesseract-robotics/tesseract/issues/1283>`_)
* Install hpp header files (`#1286 <https://github.com/tesseract-robotics/tesseract/issues/1286>`_)
* Windows build fixes (`#1273 <https://github.com/tesseract-robotics/tesseract/issues/1273>`_)
* Fix sign-conversion warnings in geometry/conversions.cpp (`#1279 <https://github.com/tesseract-robotics/tesseract/issues/1279>`_)
* Update dependencies to use opw_kinematics version 0.5.3
* Bump actions/upload-pages-artifact from 4 to 5 (`#1278 <https://github.com/tesseract-robotics/tesseract/issues/1278>`_)
* Fix sign conversion compiler error in common/utils.cpp (`#1276 <https://github.com/tesseract-robotics/tesseract/issues/1276>`_)
* Use insert_or_assign in OFKT getLinkTransforms
  insert() silently skips existing keys, so callers that reuse a
  pre-populated TransformMap (e.g. static thread_local caches) get
  stale transforms for links whose values changed. insert_or_assign
  overwrites existing entries, making map reuse safe without clearing.
  The KDL state solver already overwrites via operator[] in
  calculateTransformsHelper; this brings OFKT in line with that
  behavior.
  Also add self-assignment guard to copy-assignment operator.
* Fix setCollisionObjectsTransform assuming an ordered map
* Bump actions/configure-pages from 5 to 6 (`#1269 <https://github.com/tesseract-robotics/tesseract/issues/1269>`_)
* Bump actions/deploy-pages from 4 to 5
  Bumps [actions/deploy-pages](https://github.com/actions/deploy-pages) from 4 to 5.
  - [Release notes](https://github.com/actions/deploy-pages/releases)
  - [Commits](https://github.com/actions/deploy-pages/compare/v4...v5)
  ---
  updated-dependencies:
  - dependency-name: actions/deploy-pages
  dependency-version: '5'
  dependency-type: direct:production
  update-type: version-update:semver-major
  ...
* Fix use of markdown in docs
* Fix property tree derived type implementation
* Add eigen types to the property tree builder
* Fix getting started documentation
* Add property tree and schema derived type support
* Move Findassimp.cmake to common
* Bump docker/login-action from 3 to 4
  Bumps [docker/login-action](https://github.com/docker/login-action) from 3 to 4.
  - [Release notes](https://github.com/docker/login-action/releases)
  - [Commits](https://github.com/docker/login-action/compare/v3...v4)
  ---
  updated-dependencies:
  - dependency-name: docker/login-action
  dependency-version: '4'
  dependency-type: direct:production
  update-type: version-update:semver-major
  ...
* Bump docker/metadata-action from 5 to 6
  Bumps [docker/metadata-action](https://github.com/docker/metadata-action) from 5 to 6.
  - [Release notes](https://github.com/docker/metadata-action/releases)
  - [Commits](https://github.com/docker/metadata-action/compare/v5...v6)
  ---
  updated-dependencies:
  - dependency-name: docker/metadata-action
  dependency-version: '6'
  dependency-type: direct:production
  update-type: version-update:semver-major
  ...
* Bump docker/build-push-action from 6 to 7
  Bumps [docker/build-push-action](https://github.com/docker/build-push-action) from 6 to 7.
  - [Release notes](https://github.com/docker/build-push-action/releases)
  - [Commits](https://github.com/docker/build-push-action/compare/v6...v7)
  ---
  updated-dependencies:
  - dependency-name: docker/build-push-action
  dependency-version: '7'
  dependency-type: direct:production
  update-type: version-update:semver-major
  ...
* Add property tree class to support yaml schema and validation
* Clean up documentation sidebar
* Bump actions/upload-artifact from 6 to 7
  Bumps [actions/upload-artifact](https://github.com/actions/upload-artifact) from 6 to 7.
  - [Release notes](https://github.com/actions/upload-artifact/releases)
  - [Commits](https://github.com/actions/upload-artifact/compare/v6...v7)
  ---
  updated-dependencies:
  - dependency-name: actions/upload-artifact
  dependency-version: '7'
  dependency-type: direct:production
  update-type: version-update:semver-major
  ...
* Port legacy documentation
* Fix CI based on new directory structure
* Update documentation
* Fix incorrect cmake module path in tesseract geometry cmake file
* Consolidate into single cmake project leveraging sub components
* Update to clang-format-18
* Leverage nested namespaces
* Fix pair margin and normal for distanceCallback (`#1249 <https://github.com/tesseract-robotics/tesseract/issues/1249>`_)
* Remove unused field active from ContactTestData (`#1250 <https://github.com/tesseract-robotics/tesseract/issues/1250>`_)
* Move duplicate code to needsCollisionCheck function (`#1247 <https://github.com/tesseract-robotics/tesseract/issues/1247>`_)
* Reenable working FCL tests
* Add aarch64 lib to search paths tcmalloc_minimal library
* Contributors: John Wason, Levi Armstrong, Roelof Oomen, dependabot[bot]

0.34.1 (2026-01-30)
-------------------
* Update changelogs
* Use thread local for jacobian in kdl state solver
* Move data when adding interpolated data instead of copy
* Add macro to allow disabling use of thread_local
* Contributors: Levi Armstrong

0.34.0 (2026-01-28)
-------------------
* Update changelogs
* Use thread_local for link pair key
* Reduce allocation in ContactResultMap
* Reduce scope of PCL dependency
* Remove ifopt dep
* Add hash to link class (`#1242 <https://github.com/tesseract-robotics/tesseract/issues/1242>`_)
* Catch error when substep == num_substeps and add test
* Bump actions/upload-artifact from 5 to 6
  Bumps [actions/upload-artifact](https://github.com/actions/upload-artifact) from 5 to 6.
  - [Release notes](https://github.com/actions/upload-artifact/releases)
  - [Commits](https://github.com/actions/upload-artifact/compare/v5...v6)
  ---
  updated-dependencies:
  - dependency-name: actions/upload-artifact
  dependency-version: '6'
  dependency-type: direct:production
  update-type: version-update:semver-major
  ...
* Update colcon action to v14
* Bump actions/checkout from 1 to 6
  Bumps [actions/checkout](https://github.com/actions/checkout) from 1 to 6.
  - [Release notes](https://github.com/actions/checkout/releases)
  - [Changelog](https://github.com/actions/checkout/blob/main/CHANGELOG.md)
  - [Commits](https://github.com/actions/checkout/compare/v1...v6)
  ---
  updated-dependencies:
  - dependency-name: actions/checkout
  dependency-version: '6'
  dependency-type: direct:production
  update-type: version-update:semver-major
  ...
* Bump docker/build-push-action from 5 to 6
  Bumps [docker/build-push-action](https://github.com/docker/build-push-action) from 5 to 6.
  - [Release notes](https://github.com/docker/build-push-action/releases)
  - [Commits](https://github.com/docker/build-push-action/compare/v5...v6)
  ---
  updated-dependencies:
  - dependency-name: docker/build-push-action
  dependency-version: '6'
  dependency-type: direct:production
  update-type: version-update:semver-major
  ...
* Bump peaceiris/actions-gh-pages from 3 to 4
  Bumps [peaceiris/actions-gh-pages](https://github.com/peaceiris/actions-gh-pages) from 3 to 4.
  - [Release notes](https://github.com/peaceiris/actions-gh-pages/releases)
  - [Changelog](https://github.com/peaceiris/actions-gh-pages/blob/main/CHANGELOG.md)
  - [Commits](https://github.com/peaceiris/actions-gh-pages/compare/v3...v4)
  ---
  updated-dependencies:
  - dependency-name: peaceiris/actions-gh-pages
  dependency-version: '4'
  dependency-type: direct:production
  update-type: version-update:semver-major
  ...
* Bump cpina/github-action-push-to-another-repository from 1.6 to 1.7
  Bumps [cpina/github-action-push-to-another-repository](https://github.com/cpina/github-action-push-to-another-repository) from 1.6 to 1.7.
  - [Release notes](https://github.com/cpina/github-action-push-to-another-repository/releases)
  - [Commits](https://github.com/cpina/github-action-push-to-another-repository/compare/v1.6...v1.7)
  ---
  updated-dependencies:
  - dependency-name: cpina/github-action-push-to-another-repository
  dependency-version: '1.7'
  dependency-type: direct:production
  update-type: version-update:semver-minor
  ...
* fix benchmark ci (`#1222 <https://github.com/tesseract-robotics/tesseract/issues/1222>`_)
* Bump tesseract-robotics/colcon-action from 11 to 13
  Bumps [tesseract-robotics/colcon-action](https://github.com/tesseract-robotics/colcon-action) from 11 to 13.
  - [Release notes](https://github.com/tesseract-robotics/colcon-action/releases)
  - [Commits](https://github.com/tesseract-robotics/colcon-action/compare/v11...v13)
  ---
  updated-dependencies:
  - dependency-name: tesseract-robotics/colcon-action
  dependency-version: '13'
  dependency-type: direct:production
  update-type: version-update:semver-major
  ...
* Bump actions/setup-python from 4 to 6 (`#1225 <https://github.com/tesseract-robotics/tesseract/issues/1225>`_)
* Bump actions/upload-pages-artifact from 3 to 4
  Bumps [actions/upload-pages-artifact](https://github.com/actions/upload-pages-artifact) from 3 to 4.
  - [Release notes](https://github.com/actions/upload-pages-artifact/releases)
  - [Commits](https://github.com/actions/upload-pages-artifact/compare/v3...v4)
  ---
  updated-dependencies:
  - dependency-name: actions/upload-pages-artifact
  dependency-version: '4'
  dependency-type: direct:production
  update-type: version-update:semver-major
  ...
* Bump actions/cache from 1.1.0 to 4.3.0
  Bumps [actions/cache](https://github.com/actions/cache) from 1.1.0 to 4.3.0.
  - [Release notes](https://github.com/actions/cache/releases)
  - [Changelog](https://github.com/actions/cache/blob/main/RELEASES.md)
  - [Commits](https://github.com/actions/cache/compare/v1.1.0...v4.3.0)
  ---
  updated-dependencies:
  - dependency-name: actions/cache
  dependency-version: 4.3.0
  dependency-type: direct:production
  update-type: version-update:semver-major
  ...
* Bump actions/upload-artifact from 4 to 5
  Bumps [actions/upload-artifact](https://github.com/actions/upload-artifact) from 4 to 5.
  - [Release notes](https://github.com/actions/upload-artifact/releases)
  - [Commits](https://github.com/actions/upload-artifact/compare/v4...v5)
  ---
  updated-dependencies:
  - dependency-name: actions/upload-artifact
  dependency-version: '5'
  dependency-type: direct:production
  update-type: version-update:semver-major
  ...
* Cleanup doxygen file headers
* Add dependabot
* Remove old console bridge cmake target logic
* Fix run-cpack script
* Add createKey static function to Profile class
* Improve serialization coverage
* Add doxygen to tesseract collision example
* Update documenation to include Why Tesseract and Tesseract vs MoveIt (`#1215 <https://github.com/tesseract-robotics/tesseract/issues/1215>`_)
* Switch to using Cereal for serialization (`#1216 <https://github.com/tesseract-robotics/tesseract/issues/1216>`_)
* Contributors: Levi Armstrong, Roelof Oomen, Tyler Marr, dependabot[bot]

0.33.1 (2025-11-03)
-------------------
* Update changelogs
* Update boost plugin loader to 0.4.3
* Contributors: Levi Armstrong

0.33.0 (2025-10-28)
-------------------
* Update changelogs
* Update due to changes with boost plugin loader package
* Add std vector any poly types
* Contributors: Levi Armstrong

0.32.0 (2025-09-10)
-------------------
* Update changelogs
* Remove focal CI builds
* Add AMENT environment variable to GeneralResourceLocator
* Use per object max margin for the broadphase and the actual link pair margin for the narrow phase, instead of the overall max margin everywhere (`#1198 <https://github.com/tesseract-robotics/tesseract/issues/1198>`_)
* Implement a condensed summary for ContactTrajectoryResults (`#1205 <https://github.com/tesseract-robotics/tesseract/issues/1205>`_)
* Check trajectory return contact location (`#1200 <https://github.com/tesseract-robotics/tesseract/issues/1200>`_)
* Increase OKFT joint change tolerance to 1e-10
* Fix Joint Group jacobian when base link is an active link with unit test
* Fix timer
* Fix benchmarks (`#1201 <https://github.com/tesseract-robotics/tesseract/issues/1201>`_)
* Replace PairHash by template specialization of std::hash (`#1197 <https://github.com/tesseract-robotics/tesseract/issues/1197>`_)
* Leverage TransformMap in tesseract_srdf package (`#1196 <https://github.com/tesseract-robotics/tesseract/issues/1196>`_)
* Add yaml encode/decode for commonly used Eigen::Matrix<double, 6, 1> (`#1193 <https://github.com/tesseract-robotics/tesseract/issues/1193>`_)
* Fix clang-tidy errors
* Reduce number of thread local variables by making class member variable
* Improve tesseract_kinematics unit test coverage
* Fix focal build with changes to TransformMap
* Improve memory allocation during motion planning
* Fix environment clone missing resource locator
* Update boost_plugin_loader to version 0.3.2
* Improve memory allocations in createConvexHull
* Improve memory allocations in satisfiesLimits
* Fix double lookups of collision objects
* Add ProfilePluginFactory to fwd.h
* Add trajectory link collision representation options
* Add profile plugin factory
* Add Profiles Plugin Info
* Update boost_plugin_loader to version 0.3.1 (`#1177 <https://github.com/tesseract-robotics/tesseract/issues/1177>`_)
* Improve GitHub actions for Windows and MacOS (`#1179 <https://github.com/tesseract-robotics/tesseract/issues/1179>`_)
* tesseract collision yaml extensions (`#1176 <https://github.com/tesseract-robotics/tesseract/issues/1176>`_)
  Co-authored-by: Samantha Smith <Troyandme04@gmail.com>
* Contributors: John Wason, Levi Armstrong, Roelof Oomen, Tyler Marr

0.31.0 (2025-07-05)
-------------------
* Update changelogs
* Fix yaml extensions file name spelling
* Improve processYamlIncludeDirective peformance
* Fix docker copy install
* Update README.md
* Docker update (`#1167 <https://github.com/tesseract-robotics/tesseract/issues/1167>`_)
* Allow Docker workflow to be run manually (`#1162 <https://github.com/tesseract-robotics/tesseract/issues/1162>`_)
* Make printouts during convex decomposition optional at compute time
* Move the templated getProfile utility function to the profile dictionary
* Update yaml serialization of std::unordered_map to be generic
* Add AllowedCollisionMatrix and CollisionMarginData to yaml extensions (`#1152 <https://github.com/tesseract-robotics/tesseract/issues/1152>`_)
* Fix boost::archive type for binary serialization
* Remove PluginLoader and ClassLoader from tesseract_common fwd.h
* Make sure serialized objects have friend struct tesseract_common::Serialization
* Contributors: Levi Armstrong, Michael Ripperger, Roelof Oomen, Tyler Marr

0.30.0 (2025-04-23)
-------------------
* Update changelogs
* Pin boost_plugin_loader to 0.3.0 in rosinstall files
* Move profile dictionary to tesseract_common
* Add unit tests for trajectory collision logging (`#1146 <https://github.com/tesseract-robotics/tesseract/issues/1146>`_)
* Add increment and scale to ContactManagerConfig
* Removed debug cache prints (`#1147 <https://github.com/tesseract-robotics/tesseract/issues/1147>`_)
* Update cmake format CI to leverage 22.04 image
* Update to leverage boost_plugin_loader
* Improve tesseract_geometry code coverage (`#1144 <https://github.com/tesseract-robotics/tesseract/issues/1144>`_)
* Improve tesseract_environment codecov
* Fix contact manager config serialization
* Improve tesseract_collision code coverage
* Update tesseract_common fwd.h
* Add constructor to CollisionMarginPairData
* Improve tesseract_kinematics code coverage (`#1140 <https://github.com/tesseract-robotics/tesseract/issues/1140>`_)
* Improve tesseract_urdf code coverage (`#1139 <https://github.com/tesseract-robotics/tesseract/issues/1139>`_)
* Improve tesseract_common code coverage (`#1138 <https://github.com/tesseract-robotics/tesseract/issues/1138>`_)
* Improve codecov CI
* Fix use of CollisionCheckConfig and remove ContactMangerConfig from this struct
* Add incrementCollisionMarginData to contact manager interface
* Add validate method to ContactManagerConfig
* KinematicsGroup: return complete list of possible tip links and working frames (`#1136 <https://github.com/tesseract-robotics/tesseract/issues/1136>`_)
* Documentation Update (`#1130 <https://github.com/tesseract-robotics/tesseract/issues/1130>`_)
* Add misc-include-cleaner.IgnoreHeaders
* Contributors: Levi Armstrong, Michael Ripperger, Tyler Marr

0.29.1 (2025-03-26)
-------------------
* Update changelogs
* Fix polygon mesh not serializing its resource locator
* Add missing declared namespace for tesseract in xacro and urdf files
* Contributors: Levi Armstrong

0.29.0 (2025-03-20)
-------------------
* Update changelogs
* Fully enable collision shape caching
* Update URDF parser to use Tesseract XML namespace (`#1081 <https://github.com/tesseract-robotics/tesseract/issues/1081>`_)
* Move tesseract_collision any poly types to types.h
* Leverage inheritance for AnyPoly (`#1128 <https://github.com/tesseract-robotics/tesseract/issues/1128>`_)
* Do not log an error when correctly parsing a yaml include.
* Update to leverage std::filesystem
* Remove unused header from Timer
* Do not pass double by const ref in CollisionMarginData
* Improve python support for octree_utils.h
* Fix SimpleResource::locateResource
* Expose ability to set UUID in Geometry class
* Update readme
* Fix clang-tidy-17 errors (`#1122 <https://github.com/tesseract-robotics/tesseract/issues/1122>`_)
* Fix missing includes causing clang warnings
* Contributors: Levi Armstrong, Michael Ripperger, Roelof Oomen

0.28.8 (2025-02-01)
-------------------
* Update changelogs
* Add UUID to tesseract_geometry::Geometry and update collision shape cache
* Fix collision shape caching
* Contributors: Levi Armstrong

0.28.7 (2025-01-29)
-------------------
* Update changelogs
* Fix thread safety issue with set active contact manager
* Disable collision shape caching in bullet and fcl (`#1116 <https://github.com/tesseract-robotics/tesseract/issues/1116>`_)
* Add tesseract installer script to pull release debians and install (`#1113 <https://github.com/tesseract-robotics/tesseract/issues/1113>`_)
* Contributors: Levi Armstrong

0.28.6 (2025-01-26)
-------------------
* Update changelogs
* Add collision shape caching based on tesseract_geometry::Geometry object
* Contributors: Levi Armstrong

0.28.5 (2025-01-21)
-------------------
* Update changelogs
* Update RICB to version 0.7.2
* Contributors: Levi Armstrong

0.28.4 (2025-01-18 17:42)
-------------------------
* Update changelogs
* Add missing floating joint methods to environment monitor interface
* Add missing floating joint methods to environment
* Contributors: Levi Armstrong

0.28.3 (2025-01-18 08:41)
-------------------------
* Update changelogs
* Fix dockerfile to use correct rosinstall file based on distro (`#1109 <https://github.com/tesseract-robotics/tesseract/issues/1109>`_)
* Contributors: Levi Armstrong

0.28.2 (2025-01-17 21:02)
-------------------------
* Update changelogs
* Fix package debian pipeline
* Contributors: Levi Armstrong

0.28.1 (2025-01-17 20:55)
-------------------------
* Update changelogs
* Fix rosinstall so focal has its own so newer versions leverage system depends
* Contributors: Levi Armstrong

0.28.0 (2025-01-16)
-------------------
* Update changelogs
* Add package debian CI pipeline (`#1105 <https://github.com/tesseract-robotics/tesseract/issues/1105>`_)
* Enable cpack in ubuntu CI pipeline
* Fix cpack build
* Update RICB and OPW Kinematics tags
* Upgrade RICB package to 0.7.0
* Update tesseract_environment benchmarks
* Add floating joint support
* Fix Bullet linking on MacOS (`#1101 <https://github.com/tesseract-robotics/tesseract/issues/1101>`_)
  * Use Bullet cmake targets when available
  * Guard creation of Bullet3::Bullet target
  * Windows does not have Bullet3Geometry.lib
  * Restore boost-stacktrace changes
* Add GeneralResourceLocator API functions
* Leverage tesseract_common loadYamlFile and loadYamlString
* Add ContactAllowedValidator and ContactResultValidator (`#1095 <https://github.com/tesseract-robotics/tesseract/issues/1095>`_)
* Add installed headers to core component
* Change environment getJointGroup and getKinematicGroup to return shared pointers
* Leverage fs::path::preferred_separator in locateResource
* Add yaml include directive support
* Expose Inverse Kinematics in Kinematics Group
* Move getSummary implementation to cpp
* Get a brief string summary of a contact map (`#1079 <https://github.com/tesseract-robotics/tesseract/issues/1079>`_)
* Simplify shape_id counter
* Update Mac CI to use mac-13
* Fix use of compound mesh in FCL
* Fix issue templates (`#1087 <https://github.com/tesseract-robotics/tesseract/issues/1087>`_)
  * Fixed bug report
  * Converted feature request template to YAML
  * Minor updates to issue templates
* Changed file extension of bug report to yml (`#1086 <https://github.com/tesseract-robotics/tesseract/issues/1086>`_)
* Update bug_report.md (`#1085 <https://github.com/tesseract-robotics/tesseract/issues/1085>`_)
* Update bug report issue template (`#1084 <https://github.com/tesseract-robotics/tesseract/issues/1084>`_)
  * Update bug report issue template
  * Updated Windows version specifications
* Add issue templates (`#1082 <https://github.com/tesseract-robotics/tesseract/issues/1082>`_)
* Fix typo in tesseract_common CMakeLists.txt
* Add more serialization for eigen types and collision types
* Fix typo in unit tests
* Contributors: John Wason, Levi Armstrong, Max DeSantis, Michael Ripperger, Roelof Oomen, Tyler Marr

0.27.1 (2024-12-03)
-------------------
* Update changelogs
* Update ros_industrial_cmake_boilerplate version (`#1073 <https://github.com/tesseract-robotics/tesseract/issues/1073>`_)
* Include stdexcept in type_erasure.h
* Contributors: John Wason, Levi Armstrong

0.27.0 (2024-12-01)
-------------------
* Update changelogs
* Update package.xml
* Simplify type erasure
* Fix mesh parser passing eigen types by value
* Add install of gdb to DockerfileDev
* Enable ptrace in docker-compose-dev.yaml
* Update fcl rosdep key in package.xml
* Fix environment clear such that cache is cleared before plugin factories are destroyed
* Add missing depends to DockerFileDev
* Update to leverage clang-format-12
* Add uuid to tesseract_common::JointTrajectory
* Add dev docker and compose file that launches Qt Creator
* Contributors: Levi Armstrong

0.26.0 (2024-10-27)
-------------------
* Update changelog
* Add utility toArchiveFile method
* Remove TesseractSupportResourceLocator
* Fix serialization
* Fix abstract class serialization
* Fix serialization extension marco so they must include . char
* Fix to archive file with no extension
* Contributors: Levi Armstrong

0.25.0 (2024-09-28)
-------------------
* Update changelogs
* Only run kinematic check when testing is enabled and debug is enabled
* Add missing package libraries cmake variable
* Update conda build to source before running tests
* Add geometry type CompoundMesh
* Add timer class with callback
* Rename Timer to Stopwatch
* Contributors: Levi Armstrong

0.24.1 (2024-08-19)
-------------------
* Update changelogs
* Expose environment init revision
* Contributors: Levi Armstrong

0.24.0 (2024-08-14)
-------------------
* Update changelogs
* Add any poly support for collision types
* Use build-only for conda build in conda action to speed up test
* Fix clang detection on Mac OS (`#1038 <https://github.com/tesseract-robotics/tesseract/issues/1038>`_)
* Contributors: John Wason, Levi Armstrong

0.23.1 (2024-07-28)
-------------------
* Update changelogs
* Cleanup boost serialization
* Fix macos GitHub Action  (`#1036 <https://github.com/tesseract-robotics/tesseract/issues/1036>`_)
* Use latest vcpkg release for GitHub Action (`#1035 <https://github.com/tesseract-robotics/tesseract/issues/1035>`_)
  * Use latest vcpkg release in windows action
  * ci
  * Fix deprecated boost filesystem functions
* Add export implementation environment command
* Fix use of boost stacktrace
* Contributors: John Wason, Levi Armstrong

0.23.0 (2024-07-24)
-------------------
* Update changelogs
* Add any poly support for environment
* Add any poly support to manipulator info
* Add std::size_t to any poly types
* Add integral any poly types
* fix clang-format file to work on 20.04
* Add checkForUnknownKeys to yaml utils
* Do not export plugin libraries (`#1028 <https://github.com/tesseract-robotics/tesseract/issues/1028>`_)
* Update .clang-format
* Update RICB to version 0.6.2
* Handle edge case in calcJacobianTransformErrorDiff
* Improve any poly serialization macros
* Fixes for building on Ubuntu Noble (`#1016 <https://github.com/tesseract-robotics/tesseract/issues/1016>`_)
* Add missing serialization header to AnyPoly
* Add support for shared pointers to tesseract_common::AnyPoly
* Updated Dockerfile to support 24.04 (`#1015 <https://github.com/tesseract-robotics/tesseract/issues/1015>`_)
* Add CI for Ubuntu Noble (`#1006 <https://github.com/tesseract-robotics/tesseract/issues/1006>`_)
  Co-authored-by: Roelof Oomen <rj.oomen@isohorti.com>
* Contributors: Levi Armstrong, Michael Ripperger, Roelof Oomen

0.22.2 (2024-06-10)
-------------------
* Update changelogs
* Add ability to briefly summarize trajectory collisions (`#1011 <https://github.com/tesseract-robotics/tesseract/issues/1011>`_)
* Upgrade ros_industrial_cmake_boilerplate version to 0.6.0
* Fix command serialization
* Fix windows issue in kinematics_limits.h with using eigen array max and min
* Add backtrace to type erasure casting to quickly identify where the issue location
* Contributors: Levi Armstrong, Tyler Marr

0.22.1 (2024-06-03)
-------------------
* Update changelogs
* - Also add KDL parameters to KDLInvKinChainNR_JL (see `#843 <https://github.com/tesseract-robotics/tesseract/issues/843>`_)
  - Some clang_tidy and typo fixes
* Contributors: Levi Armstrong, Roelof Oomen

0.22.0 (2024-06-02)
-------------------
* Update changelogs
* Add the ability to change KDL parameters from kinematics configuration (`#843 <https://github.com/tesseract-robotics/tesseract/issues/843>`_)
  Co-authored-by: Levi Armstrong <levi.armstrong@gmail.com>
* Fix 2 for older bullet versions (e.g. on Ubuntu Focal)
* Fix for older bullet versions (e.g. on Ubuntu Focal)
* - Apply [bugfix](https://github.com/bulletphysics/bullet3/commit/fce12964139c8073676a50db0201c1460ad3fcad) and [bugfix](https://github.com/bulletphysics/bullet3/commit/a808d7489500935ac6665dcfe930c43a2008566b) from bullet
  - Clang-tidy fix
* Fix deprecated exec_program command (`#1004 <https://github.com/tesseract-robotics/tesseract/issues/1004>`_)
* Add binary data serialization support functions
* Add support for jerk limits
* Fix map type in comment of ContactResultMap
  See `#991 <https://github.com/tesseract-robotics/tesseract/issues/991>`_
* Pin cmake version to 3.28.x for windows and mac
* Add more clone benchmarks
* Leverage forward declarations to improve compile times (`#990 <https://github.com/tesseract-robotics/tesseract/issues/990>`_)
* Add kinematics benchmarks
* Faster pair hash for ACM
* Trajectory player fixes (`#989 <https://github.com/tesseract-robotics/tesseract/issues/989>`_)
* Fix bug in calcJacobianTransformErrorDiff
* Correctly handle angle axis singularity when calculating numerical jacobian
* Remove ineffective frame.Identity() call
  See `#984 <https://github.com/tesseract-robotics/tesseract/issues/984>`_
* Update to vcpkg-action@v6 and use vcpkg-binarycache
* Added application for performing convex decomposition (`#968 <https://github.com/tesseract-robotics/tesseract/issues/968>`_)
* Update URDF Robot version tag to 'tesseract_version' for ROS2 compatibilty (`#979 <https://github.com/tesseract-robotics/tesseract/issues/979>`_)
* Fix default test type in comment (`#980 <https://github.com/tesseract-robotics/tesseract/issues/980>`_)
* Fix conda tests (`#981 <https://github.com/tesseract-robotics/tesseract/issues/981>`_)
* Remove deprecated AnalyzeTemporaryDtors
* clang-format
* - Fixes rounding errors in progress logging (went up to 101%)
  - Fixes some clang-tidy suggestions
* Contributors: John Wason, Levi Armstrong, Michael Ripperger, Roelof, Roelof Oomen, Sean Cardello, Tyler Marr

0.21.5 (2023-12-14)
-------------------
* Update changelogs
* Add Mac OSX support (`#969 <https://github.com/tesseract-robotics/tesseract/issues/969>`_)
* Add jacobian transform error calculation function (`#971 <https://github.com/tesseract-robotics/tesseract/issues/971>`_)
* Update docker CI name
* Cleanup CI and docker file (`#966 <https://github.com/tesseract-robotics/tesseract/issues/966>`_)
* Contributors: John Wason, Levi Armstrong

0.21.4 (2023-11-20)
-------------------
* Update changelog
* Remove old industrial CI docker build
* Add docker build CI pipeline
* Added docker files (`#960 <https://github.com/tesseract-robotics/tesseract/issues/960>`_)
* Contributors: Levi Armstrong, Michael Ripperger

0.21.3 (2023-11-16)
-------------------
* Update changelogs
* Fix thread safety in environment (`#964 <https://github.com/tesseract-robotics/tesseract/issues/964>`_)
  * Fix thread safety in environment.cpp
  * Clang format
* Use colcon-action (`#963 <https://github.com/tesseract-robotics/tesseract/issues/963>`_)
* Remove use of Industrial CI
* Contributors: Levi Armstrong

0.21.2 (2023-11-10)
-------------------
* Update changelogs
* Support message conversions
* Contributors: Levi Armstrong

0.21.1 (2023-11-09)
-------------------
* Updated changelogs
* Revert "Changed CI base directory to /opt" (`#959 <https://github.com/tesseract-robotics/tesseract/issues/959>`_)
  This reverts commit a74c9efe968ad4b906d5b6e898c9fb476d141388.
* Contributors: Michael Ripperger

0.21.0 (2023-11-07)
-------------------
* Update changelogs
* Changed CI base directory to /opt
* Updated to version of ICI with VCS shallow cloning (`#955 <https://github.com/tesseract-robotics/tesseract/issues/955>`_)
  Co-authored-by: Levi Armstrong <levi.armstrong@gmail.com>
* Pin windows vcpkg version
* Fix FCL contact test when calculate distance is false but contact threshold is greater than zero
* Add collision unit test with calculate distance disabled
* Contributors: Levi Armstrong, Michael Ripperger

0.20.2 (2023-10-26)
-------------------
* Update changelogs
* Update environment clear method to clear all internal cache data
* Fix resource locator windows build
* Contributors: Levi Armstrong

0.20.1 (2023-10-13)
-------------------
* Update changelogs
* Replaced the implementation of tesseract_kinematics::parseSceneGraph (`#947 <https://github.com/tesseract-robotics/tesseract/issues/947>`_)
* Unused includes cleanup (`#946 <https://github.com/tesseract-robotics/tesseract/issues/946>`_)
* Add Eigen::Vector3d yaml support (`#945 <https://github.com/tesseract-robotics/tesseract/issues/945>`_)
* Merge pull request `#943 <https://github.com/tesseract-robotics/tesseract/issues/943>`_ from marrts/fix/ik_fast/free_joint_state_parsing
  Make `free_joint_state` parsing more generalized
* Make `free_joint_state` parsing more generalized
* Contributors: Levi Armstrong, Roelof, Tyler Marr

0.20.0 (2023-09-27)
-------------------
* Update changelog
* Add support for KDL::ChainIkSolverPos_NR_JL, which takes joint limits (`#928 <https://github.com/tesseract-robotics/tesseract/issues/928>`_)
* Add std::variant boost serialization support
* Fix missing locks in tesseract_environment (`#938 <https://github.com/tesseract-robotics/tesseract/issues/938>`_)
* Fix conda tests (`#936 <https://github.com/tesseract-robotics/tesseract/issues/936>`_)
* Contributors: John Wason, Levi Armstrong, Roelof

0.19.2 (2023-09-06)
-------------------
* Update changelogs
* Fix Ubunut Jammy release build
* Contributors: Levi Armstrong

0.19.1 (2023-09-05 15:47)
-------------------------
* Update changelogs
* Fix benchmark build
* Contributors: Levi Armstrong

0.19.0 (2023-09-05 13:03)
-------------------------
* Update changelogs
* Update kinematics and collision packages to leverage cmake components (`#927 <https://github.com/tesseract-robotics/tesseract/issues/927>`_)
* Contributors: Levi Armstrong

0.18.2 (2023-09-04)
-------------------
* CI build on branches starting with 'dev**' (`#931 <https://github.com/tesseract-robotics/tesseract/issues/931>`_)
* Add conda-forge CI test (`#930 <https://github.com/tesseract-robotics/tesseract/issues/930>`_)
* Update emails
* Improved reporting of errors in IK plugin loader (`#924 <https://github.com/tesseract-robotics/tesseract/issues/924>`_)
* Added IKFast factory boilerplate (`#916 <https://github.com/tesseract-robotics/tesseract/issues/916>`_)
  * Added IKFast factory boilerplate
  * Updated IKFast unit tests to use new IKFast boilerplate factory
  * Reverted change to kinematic_group.h
  * Add include guards
  * Fix filename in header comment
  Co-authored-by: Tyler Marr <41449746+marrts@users.noreply.github.com>
  * IKFast plugins as cpp instead of h
  * Fix ikfast clang-tidy issues
  ---------
  Co-authored-by: Tyler Marr <41449746+marrts@users.noreply.github.com>
  Co-authored-by: Levi Armstrong <levi.armstrong@gmail.com>
* Do not call find_package() if package has already been found.
* Fix of ManipulatorInfo and typos (`#914 <https://github.com/tesseract-robotics/tesseract/issues/914>`_)
  - ManipulatorInfo constructor now accepts tcp_offset as variant to match data member.
  - Fixed typos in rep and rop factories.
* Remove deletion of -dev libraries (`#913 <https://github.com/tesseract-robotics/tesseract/issues/913>`_)
* Add Jammy CI Build (`#912 <https://github.com/tesseract-robotics/tesseract/issues/912>`_)
  * Add Jammy build
  * Update install script for ROS package management and build tools
* Reduce CI-built docker image size (`#911 <https://github.com/tesseract-robotics/tesseract/issues/911>`_)
  * Remove upstream and target workspace build directories after CI build before docker commit
  * Removed -dev libraries from CI before unit tests
* Contributors: John Wason, Levi Armstrong, Michael Ripperger, Roelof, Roelof Oomen

0.18.1 (2023-06-30)
-------------------
* Update changelogs
* Fix convert convex hull to set correct creation method
* Contributors: Levi Armstrong

0.18.0 (2023-06-29)
-------------------
* Update changelogs
* Update RICB version to 0.4.7
* Update kinematics group inverse kinematics to harmonize within joint limits (`#899 <https://github.com/tesseract-robotics/tesseract/issues/899>`_)
* Trajectory logging fixup (`#908 <https://github.com/tesseract-robotics/tesseract/issues/908>`_)
* Improve Trajectory Collision Logging (`#765 <https://github.com/tesseract-robotics/tesseract/issues/765>`_)
* Add package cmake flags for testing, examples and benchmarks
* Add assert to ContactResultsMap to make sure key is an ordered pair
* Removed gcc-specific options from clang config
* Fix JointState equal operator
* Fix makeConvexMesh to pass through scale used on resource
* Update VHACD to latest (v4.1, tag 454913f) (`#896 <https://github.com/tesseract-robotics/tesseract/issues/896>`_)
* Contributors: John Wason, Levi Armstrong, Roelof, Tyler Marr

0.17.0 (2023-06-06)
-------------------
* Update changelogs
* Remove invalid assert from FCL collision and distance callback functions
* Windows updates (`#893 <https://github.com/tesseract-robotics/tesseract/issues/893>`_)
* Fix check trajectory print statements (`#892 <https://github.com/tesseract-robotics/tesseract/issues/892>`_)
* Update resource_locator.cpp (`#889 <https://github.com/tesseract-robotics/tesseract/issues/889>`_)
* implemented benchmarking for checking trajectories (`#887 <https://github.com/tesseract-robotics/tesseract/issues/887>`_)
* Contributors: John Wason, Levi Armstrong, Tyler Marr

0.16.3 (2023-05-04)
-------------------
* Update changelog
* Upgrade gazebo versions
* Contributors: Levi Armstrong

0.16.2 (2023-04-28)
-------------------
* Update changelogs
* Add yaml support for tool path
* Contributors: Levi Armstrong

0.16.1 (2023-04-11)
-------------------
* Update changelogs
* Improve tesseract_kinematics code coverage
* Fix polygon_mesh default geometry type in constructor
* Improve tesseract_common unit test coverage
* Improve general resource locator
* Add unit tests to cover new addTrajectoryLinkCommand
* Contributors: Levi Armstrong

0.16.0 (2023-04-09)
-------------------
* Updage changelogs
* Update readme
* Improve geometry code coverage
* Improve collision code coverage
* Add ContactResultMap shrinkToFit and CollisionCheckProgramType
* Documentation/Code Coverage CI Update (`#876 <https://github.com/tesseract-robotics/tesseract/issues/876>`_)
* Fix ContactResultMap serialization
* Add AddTrajectoryLinkCommand
* General cleanup of commands moving things to cpp
* Updated code coverage job (`#875 <https://github.com/tesseract-robotics/tesseract/issues/875>`_)
* Use tesseract organization fork of ICI (`#874 <https://github.com/tesseract-robotics/tesseract/issues/874>`_)
* Add documentation to ContactResultMap
* Avoid multiple memory allocations in PairHash::operator()
* Remove reserve(100) in ContactResultMap does not improve performance
* Add contact results class
* Contributors: Levi Armstrong, Michael Ripperger

0.15.3 (2023-03-22)
-------------------
* Update changelogs
* Update tesseract_collision benchmarks (`#868 <https://github.com/tesseract-robotics/tesseract/issues/868>`_)
* Add ContactResult boost serialization
* Contributors: Levi Armstrong

0.15.2 (2023-03-15)
-------------------
* Update changelog
* Expose Bullet collision pool allocator configuration
* Switch include in tesseract_collision
* Contributors: Levi Armstrong

0.15.1 (2023-03-14)
-------------------
* Update changelogs
* Add flattenWrapperResults methods
* Use ROS-independent version of ICI (`#861 <https://github.com/tesseract-robotics/tesseract/issues/861>`_)
  * Use ROS-independent version of ICI
  * Added liboctomap-dev as dependency and ignore rosdep install
  * Use variables for ccache directory
  * Standardize rosdep skip list
  * Added clang-tidy dependency
  * Remove taskflow from rosdep exclusion list
  * Added FCL 6.1 to dependencies file
  * Changed from .rosinstall to .repos
  * Added name to build stage of clang-tidy job
  * Added catkin to rosdep skip list
  * Added octomap to source dependencies
  * Consolidated regular Ubuntu jobs into single matrix job
  * Set fail-fast false
  * Simplified nightly workflow
  * Updated code coverage command
  * Combined code coverage and clang tidy jobs
  * Simplify clang format job
  * Remove bionic build
  * Support CI build on multiple versions of Windows
  * Updated job names
  * Added RICB to windows vcpkg list
  * Add gtest to windows build
  * Remove octomap and fcl from vcpkg install
  * Add custom window dependencies
  ---------
  Co-authored-by: Levi Armstrong <levi.armstrong@gmail.com>
* Update readme benchmark link (`#860 <https://github.com/tesseract-robotics/tesseract/issues/860>`_)
* Benchmark CI Update (`#859 <https://github.com/tesseract-robotics/tesseract/issues/859>`_)
  * Update benchmark CI to use SSH key to push to tesseract_docs
  * Tag specific version of github push action
* Contributors: Levi Armstrong, Michael Ripperger

0.15.0 (2023-03-03)
-------------------
* Update changelog
* Update rosinstall depends versions
* Updated benchmarking job to push to tesseract_docs (`#856 <https://github.com/tesseract-robotics/tesseract/issues/856>`_)
* Update collision benchmarks
* Performance improvements found using callgrind (`#852 <https://github.com/tesseract-robotics/tesseract/issues/852>`_)
* Update readme doxygen link to tesseract_docs.
* CI Updates (`#851 <https://github.com/tesseract-robotics/tesseract/issues/851>`_)
  * Use SSH deploy key for docs CI
  * Moved logo and doxygen config to docs folder
  * Added path filters to CI jobs to prevent unnecessary runs
  * Changed to inclusive path filters
  * Only use path filters on pull requests
* Deploy documentation to separate repository (`#847 <https://github.com/tesseract-robotics/tesseract/issues/847>`_)
  * Deploy documentation to separate repository
  * Updated benchmarking workflow to push to tesseract_docs
  * Added tesseract folder as destination subdirectory
* Closes `#848 <https://github.com/tesseract-robotics/tesseract/issues/848>`_
* Update TaskComposerPluginInfo
* Update all geometry types to use default tracking for serialization
* Update mine_package_info.py
* Update windows CI
* Improve tesseract_state_solver code coverage
* Improve tesseract_scene_graph code coverage (`#838 <https://github.com/tesseract-robotics/tesseract/issues/838>`_)
* Improve tesseract_geometry code coverage
* Improve tesseract_environment code coverage
* Remove unused fcl selfCollisionContactTest method
* Improve tesseract_collision code coverage
* Fix fcl unregistar bug
* Improve tesseract_srdf code coverage (`#836 <https://github.com/tesseract-robotics/tesseract/issues/836>`_)
* Fix tesseract_common plugin info implementations equal and insert methods
* Fix KinematicsInformation equal and insert methods
* Contributors: Levi Armstrong, Michael Ripperger, Roelof Oomen

0.14.0 (2022-10-23)
-------------------
* Update changelog
* Order srdf save so plugins are above the acm
* Add general resource locator
* Add modify allowed collisions command
* Remove deprecated items
* Add fileToString utility function
* Add Python instructions to readme (`#821 <https://github.com/tesseract-robotics/tesseract/issues/821>`_)
* Use vcpkg-action@v3 in windows_2019.yml workflow (`#822 <https://github.com/tesseract-robotics/tesseract/issues/822>`_)
* Fix codecov build using ros_industrial_cmake_boilerplate 0.3.1
* Add forgotten source file extension to CMakeLists.
* Fix issue where cache is emptied by other threads after refresh causing segfault when popping environment on empty queue
* Add EnvironmentCache and DefaultEnvironmentCache
* Rename Any to AnyPoly
* Remove StatusCode and fix eigen being passed by value in JointState
* Including <boost/serialization/library_version_type.hpp> for Boost 1.74. Fixes `tesseract-robotics/tesseract#764 <https://github.com/tesseract-robotics/tesseract/issues/764>`_
* Contributors: John Wason, Levi Armstrong, Roelof Oomen

0.13.1 (2022-08-25)
-------------------
* Update changelog
* Move boost serialization export outside tesseract_common namespace in srdf_model.cpp
* Add tesseract extension macro
* Move most SWIG commands to tesseract_python package (`#809 <https://github.com/tesseract-robotics/tesseract/issues/809>`_)
* Add isNull method to TypeErasureBase
* Fix TypeErasure to fully support being null
* Fix the Windows 2019 github action (`#805 <https://github.com/tesseract-robotics/tesseract/issues/805>`_)
* Add find_bullet macro which creates a target to link against (`#803 <https://github.com/tesseract-robotics/tesseract/issues/803>`_)
* Update almostEqualRelativeAndAbs to support vector of max_diff and max_rel_diff (`#802 <https://github.com/tesseract-robotics/tesseract/issues/802>`_)
* Contributors: John Wason, Levi Armstrong

0.13.0 (2022-07-11)
-------------------
* Update changelog
* Update code based on clang-tidy-14
* Make limits utility functions templates
* Fixed convex mesh export (`#799 <https://github.com/tesseract-robotics/tesseract/issues/799>`_)
* Contributors: Levi Armstrong, Michael Ripperger

0.10.0 (2022-07-06)
-------------------
* Update changelog
* Update ros_industrial_cmake_boilerplate to 0.3.0 (`#795 <https://github.com/tesseract-robotics/tesseract/issues/795>`_)
* Fix tesseract_ignition_visualization anchor
* Fix Windows CI by checking for error after colcon commands
* Static plugin loading using symbol module resolution (`#782 <https://github.com/tesseract-robotics/tesseract/issues/782>`_)
* 0.9.11
* Update changelog
* Renames in type erasure to avoid WIN32 defines
* Update run_combine_benchmark_results script to python
* Improve manipulability calculcation (`#787 <https://github.com/tesseract-robotics/tesseract/issues/787>`_)
  * Use SelfAdjointEigenSolver
  * Simplify calculation of measure = sqrt(condition)
  * Simplify calculation of volume
  Co-authored-by: christian.petersmeier <christian.petersmeier@uni-bielefeld.de>
* Updated CPack (`#786 <https://github.com/tesseract-robotics/tesseract/issues/786>`_)
* Fix benchmark CI
* Minor fixes
* Remove old Windows CI
* Add missing gtest test_depends in tesseract_visualization
* Add lapack test_depends tesseract_kinematics
* Update to use find_gtest macro
* Add missing gtest test_depend to tesseract_support
* Update opw_kinematics tag
* Add colcon windows CI build
* Fix message in type_erasure.h
* Contributors: John Wason, Levi Armstrong, Michael Ripperger, Robert Haschke

0.9.10 (2022-06-14)
-------------------
* Update changelog
* Add type erasure interface (`#776 <https://github.com/tesseract-robotics/tesseract/issues/776>`_)
  * Add type erasure interface
  * revert change to type erasure constructor
* Make missing contact manager plugins a debug vs warn message
* Update FindTinyXML2.cmake
* Contributors: Levi Armstrong

0.9.9 (2022-05-30 22:22)
------------------------
* Update changelog
* Fix find tcmalloc on melodic
* Contributors: Levi Armstrong

0.9.8 (2022-05-30 17:33)
------------------------
* Update changelog
* Fix Findtcmalloc_minimal.cmake
* Contributors: Levi Armstrong

0.9.7 (2022-05-30 15:50)
------------------------
* Update changelog
* Reduce bullet octomap storage
* Add environment discrete_manager_mutex\_ and continuous_manager_mutex\_
* Allow not providing contact manager plugins
* Add the ability to set the environment discrete and continuous manager to nullptr to save space when needed
* Update Findtcmalloc.cmake to include threads and split out tcmalloc_minimal to Findtcmalloc_minimal.cmake
* fixup
* Update manipulability unit test
* Fix numerical issue in manipulability calculation
* Contributors: Levi Armstrong

0.9.6 (2022-05-02)
------------------
* Update changelog
* Normalize quaternion when decoding yaml Eigen::Isometry3d
* Add fwd_kin_plugins to abb_irb2400_plugins.yaml
* Contributors: John Wason, Levi Armstrong

0.9.5 (2022-04-24)
------------------
* Update changelog
* yaml_utils.h nullptr comparison fixup (`#755 <https://github.com/tesseract-robotics/tesseract/issues/755>`_)
* Fix JointTrajectory SWIG container (`#756 <https://github.com/tesseract-robotics/tesseract/issues/756>`_)
* Add utility for rebuilding link and joint maps in scene graph
* Contributors: John Wason, Levi Armstrong

0.9.4 (2022-04-22)
------------------
* Update changelog
* Use components for PCL (`#752 <https://github.com/tesseract-robotics/tesseract/issues/752>`_)
  * Find components common and io for PCL
  * Include PCL_INCLUDE_DIRS if target not available
* Windows fixes with passing unit tests (`#751 <https://github.com/tesseract-robotics/tesseract/issues/751>`_)
  * Fix bug in OFKTStateSolver::moveLinkHelper
  * Use binary ifstream ond ofstream in serialization.h
  * Add c++17 flag to windows_noetic_build.yml
  * Fix SceneGraph move constructor, restore modified unit tests
* Contributors: John Wason, Levi Armstrong

0.9.3 (2022-04-18)
------------------
* Update changelog
* Make JointTrajectory a struct
* Fix invalid iterator in bullet_cast_simple_manager (`#746 <https://github.com/tesseract-robotics/tesseract/issues/746>`_)
  * Fix invalid iterator in bullet_cast_simple_manager
  * clang format
* Enable ability to remove event callback
* Add environment serialization
* Updated plugin capability to support sections (`#741 <https://github.com/tesseract-robotics/tesseract/issues/741>`_)
* Update triggering of event callbacks to take a shared lock
* Contributors: John Wason, Levi Armstrong

0.9.2 (2022-04-03)
------------------
* Update changelog
* Add timestamp to environment
* Contributors: Levi Armstrong

0.9.1 (2022-04-01)
------------------
* Update changelog
* Generate urdf files
* Consolidate all resources into tesseract_support
* Contributors: Levi Armstrong

0.9.0 (2022-03-31)
------------------
* Update changelog
* Make ResourceLocator serializable
* Add environment monitor interfaces
* Add event callbacks to environment
* Add tcp to iiwa srdf
* fix spelling
* Add ctest calls to windows_noetic_build.yml (`#729 <https://github.com/tesseract-robotics/tesseract/issues/729>`_)
* Update ros_industrial_cmake_boilerplate and opw_kinematics tag (`#730 <https://github.com/tesseract-robotics/tesseract/issues/730>`_)
* Contributors: John Wason, Levi Armstrong

0.8.7 (2022-03-24 21:18)
------------------------
* Update changelog
* Contributors: Levi Armstrong

0.8.6 (2022-03-24 15:17:43 -0500)
---------------------------------
* Update changelog
* Add atomic serialization
* Contributors: Levi Armstrong

0.8.5 (2022-03-24 09:34)
------------------------
* Update changelog
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
* Fix incorrect linking to console_bridge for tesseract_scene_graph example
* Add methods for getting link transform information from state solver
* Contributors: Josh Langsfeld, Levi Armstrong, Matthew Powelson

0.8.4 (2022-03-03)
------------------
* Update changelog
* Add method to environment to get relative link transform
* Set TESSERACT_ENABLE_EXAMPLES default to ON
* cmake format
* Add TESSERACT_ENABLE_EXAMPLES compile option
* Add overload method for calcInvKin to take single KinGroupIKInput
* Contributors: John Wason, Levi Armstrong

0.8.3 (2022-02-22)
------------------
* Update changelog
* Fix Boost_VERSION in tesseract_urdf to check for Boost_VERSION_MACRO (`#717 <https://github.com/tesseract-robotics/tesseract/issues/717>`_)
  * Fix Boost_VERSION in tesseract_urdf to check for Boost_VERSION_MACRO
  * cmake-format
* Python patches for Feb 2022 update (`#716 <https://github.com/tesseract-robotics/tesseract/issues/716>`_)
* Modifying Boost_VERSION check to use semver
* Use windows-2019 for github action
* Update UR Kinematics Parameters for e-series
  UR10eParameters, UR5eParameters and UR3eParameters values were slightly
  off compared to official documentation. We updated them to match.
* Add missing UPtr and ConstUPtr typedef for KDL and OFKT state solver (`#711 <https://github.com/tesseract-robotics/tesseract/issues/711>`_)
* Sort ACM alphabetically when writing
* A few fixes that were needed for Windows (`#708 <https://github.com/tesseract-robotics/tesseract/issues/708>`_)
  * Make HACDConvexDecomposition library optional
  Bullet extras are not easily obtained on Windows. If found, build library, otherwise ignore. Also the plain ConvexDecomposition library is looked for but never used and so removed entirely.
  * Check if Bullet CMake variables are using absolute paths
  For some reasons, the vcpkg ported version changes the config file to
  use absolute paths instead of relative to BULLET_ROOT_DIR
  * Add include for std::string
  Co-authored-by: Levi Armstrong <levi.armstrong@gmail.com>
* Update cmake format CI script to print error message and diff
* Contributors: John Wason, Josh Langsfeld, Kyle Staub, Leo Ghafari, Levi Armstrong, Matthew Powelson

0.8.2 (2022-01-27)
------------------
* Update changelog
* Add ability to provide calibration information in the SRDF (`#703 <https://github.com/tesseract-robotics/tesseract/issues/703>`_)
  * Add missing package tesseract_srdf in CI after script
  * Add support for calibration info in SRDF
* Remove unneeded boost bind include
  Not needed since C++11 and this header puts placeholder objects in the
  global namespace on system-installed Boost versions
* Contributors: Josh Langsfeld, Levi Armstrong

0.8.1 (2022-01-24)
------------------
* Update changelog
* Add any.cpp
* Contributors: Levi Armstrong

0.8.0 (2022-01-19)
------------------
* Update changelog
* Fix check trajectory which should return a vector same length as trajectory (`#698 <https://github.com/tesseract-robotics/tesseract/issues/698>`_)
* CPack Update (`#693 <https://github.com/tesseract-robotics/tesseract/issues/693>`_)
* Update Kinematics Cache To Include IK Solver (`#695 <https://github.com/tesseract-robotics/tesseract/issues/695>`_)
* Add BOOST_SERIALIZATION_ASSUME_ABSTRACT to Any type erasure
* Contributors: Levi Armstrong, Michael Ripperger, marrts

0.7.5 (2022-01-10)
------------------
* Update changelog
* Add -Wdeprecated-declarations to push pop macros
* Updated environment benchmark (`#694 <https://github.com/tesseract-robotics/tesseract/issues/694>`_)
* Update to checkout@v2 action (`#692 <https://github.com/tesseract-robotics/tesseract/issues/692>`_)
* Add creation method to convex mesh
* URDF Writer: Small Bug Fixes
* Produce cmake error if libraries provided by libbullet-extras are not… (`#688 <https://github.com/tesseract-robotics/tesseract/issues/688>`_)
* Add ability to check if collision object is enabled (`#687 <https://github.com/tesseract-robotics/tesseract/issues/687>`_)
* Update library names in benchmarks (`#681 <https://github.com/tesseract-robotics/tesseract/issues/681>`_)
* Contributors: David Merz, Jr, Levi Armstrong, Matthew Powelson, Michael Ripperger

0.7.4 (2021-12-15 17:12)
------------------------
* Update changelog
* Fix docker push (`#685 <https://github.com/tesseract-robotics/tesseract/issues/685>`_)
* Contributors: Levi Armstrong

0.7.3 (2021-12-15 10:35)
------------------------
* Update changelog
* Switch docker build to use pep440
* Contributors: Levi Armstrong

0.7.2 (2021-12-15 10:10)
------------------------
* Update changelog
* Add docker creation for major.minor
* Contributors: Levi Armstrong

0.7.1 (2021-12-15 08:36)
------------------------
* Update changelog
* Move checkKinematics to getKinematicGroup and add support for clang-tidy-12 (`#682 <https://github.com/tesseract-robotics/tesseract/issues/682>`_)
  * Move checkKinematics to getKinematicGroup and add support for clang-tidy-12
  * Reduce the number of checks perform in checkKinematics
  * Leverage checkKinematics in unit tests
* Switch CodeCov build to use focal and remove CCache for that build
* Add modify_object_enabled to ContactManagerConfig
* Improve creating octree from point cloud using lazy_eval (`#680 <https://github.com/tesseract-robotics/tesseract/issues/680>`_)
* Add redundancy capable joints to the harmonizeTowardZero function
* Contributors: Levi Armstrong, Matthew Powelson

0.7.0 (2021-12-04)
------------------
* Update changelog
* Rename member variables of ContactManagerConfig
* Update dependency versions
* Fix KinematicGroup and JointGroup cache to clear on current state changed
* Add ContactManagerConfig inside CollisionCheckConfig
  This separates the up front setup things for the contact manager from things specific to the contactTest or the way the contact manager should be called.
* Add unit test for checkTrajectoryState and checkTrajectorySegment
* Add applyCollisionCheckConfig to contact managers
* Add AllowedCollisionMatrix to CollisionCheckConfig
* Move AllowedCollisionMatrix into tesseract_common
* Correctly set the collision margin data in the environment utilities
* Contributors: Levi Armstrong, Matthew Powelson

0.6.9 (2021-11-29 18:38)
------------------------
* Update changelog
* Add dependencies section to readme
* Fix CollisionCheckConfig to set collision_margin_override_type for constructor
* Contributors: Levi Armstrong

0.6.8 (2021-11-29 09:29)
------------------------
* Update changelog
* Add contact margin data override type MODIFY (`#669 <https://github.com/tesseract-robotics/tesseract/issues/669>`_)
  * Add contact margin data override type MODIFY
  * Add unit test for type MODIFY
* Fix spelling errors
* Contributors: Levi Armstrong

0.6.7 (2021-11-16)
------------------
* Update changelog
* Fix thread safety issue in kdl state solver
* Fix linking issue when building repo alongside debian releae
* Contributors: Levi Armstrong

0.6.6 (2021-11-10)
------------------
* Update changelog
* Update ikfast plugin
* Update tesseract_collision benchmarks
* Add determinant check and make kdl solvers thread safe (`#664 <https://github.com/tesseract-robotics/tesseract/issues/664>`_)
* Fix Kinematic Group working frames
* Contributors: Levi Armstrong, Levi-Armstrong

0.6.5 (2021-11-04)
------------------
* Fix scene graph shortest path debug print out
* Update checkKinematics to check all tip links and working frames
* Add joint and kinematic group cache
* Update readme with new repo locations
* Remove CMake messages
* Fix kinematics validate function (`#658 <https://github.com/tesseract-robotics/tesseract/issues/658>`_)
* Added ikfast compatibility with 7 DOF robots (`#657 <https://github.com/tesseract-robotics/tesseract/issues/657>`_)
  Co-authored-by: Tyler Marr <tylermarr17@gmail.com>
* Contributors: Levi Armstrong, Levi-Armstrong

0.6.4 (2021-10-29)
------------------
* Fix tesseract_environment assignment of allowed collision function
* Contributors: Levi-Armstrong

0.6.3 (2021-10-28)
------------------
* Improve tesseract_environment code coverage (`#655 <https://github.com/tesseract-robotics/tesseract/issues/655>`_)
  * Improve tesseract_environment code coverage
  * Improve tesseract_common code coverage
  * Improve tesseract_geometry code coverage
  * Fix bug in tesseract_environment with populating contact managers
  * Fix issue in JointGroup and StateSolver getStaticLinkNames
  * Fix clang-tidy error
  * Fix issue with findTCPOffset and improve code coverage
  * Improve tesseract_environment code coverage
  * Add checkTrajectory unit tests
  * Improve checkTrajectory code coverage
* Add ability to load contact managers as plugins (`#654 <https://github.com/tesseract-robotics/tesseract/issues/654>`_)
  * Add ability to load contact managers as plugins
  * Update tesseract_collision unit test coverage
  * Update tesseract_common unit test coverage
  * Refactor plugins yaml file format
  * Update factory interface to return copy for methods
  * Fix clang-tidy error
  * Improve tesseract_geometry code coverage
  * Fix clang-tidy errors
  * Fix unit tests
* Fix error in kinematics plugin factory
* Use std::list versus std::vector in kinematic group
* Remove dynamic_cast in tesseract_compound_collision_algorithm.cpp
* Add sub-directories for external dependencies of tesseract_collision (`#649 <https://github.com/tesseract-robotics/tesseract/issues/649>`_)
  * Moved core code into sub-directory
  * Moved Bullet code to separate subdirectory
  * Moved FCL code into separate sub-directory
  * Moved VHACD code to separate sub-directory
  * Revised top-level CMakeLists
  * Moved implementation of createConvexHull function into bullet library given the dependence on Bullet
  * Moved convex hull creator to bullet subdirectory given dependence on Bullet
  * Moved collision profile to benchmarks given dependence on FCL and Bullet
  * Link tesseract_urdf against Bullet for create convex hull definition
  * Updated CMake config template with build options
  * CMake formatting
  * Update targets install
  * Moved convex hull and convex mesh creation functions to bullet subdirectory
  * Updated references to convex hull and convex mesh creation functions
  * Link VHACD against bullet
  * Make build with bullet compulsory
  * Remove target include of core library in examples subdirectory
  * Updated VHACD CMakeLists
  * Remove recursive include
  * Move implementation of convex_hull_utils to cpp
  * Fix vhacd build errors
  Co-authored-by: Michael Ripperger <michael.ripperger@swri.org>
* Contributors: Levi Armstrong, Levi-Armstrong

0.6.2 (2021-10-22)
------------------
* Fix kinematic group working frames and updated so the joint names always align with the same joint group
* Update SceneGraph get shortest path to use an undirected graph
* cassert also needed on osx (`#642 <https://github.com/tesseract-robotics/tesseract/issues/642>`_)
  Co-authored-by: Levi Armstrong <levi.armstrong@gmail.com>
* Kinematics Reorganization (`#647 <https://github.com/tesseract-robotics/tesseract/issues/647>`_)
  * Moved core to separate subdirectory
  * Moved IKFast implementation to separate subdirectory
  * Moved KDL implementation to separate subdirectory
  * Moved OPW implementation to separate subdirectory
  * Moved UR implementation to separate subdirectory
  * Updated top-level CMake files
* Contributors: Levi-Armstrong, Michael Ripperger, Tobias Fischer

0.6.1 (2021-10-19)
------------------
* Bump version of CMake boilerplate tools (`#645 <https://github.com/tesseract-robotics/tesseract/issues/645>`_)
  * Bump version of CMake boilerplate tools
  * Remove addition of non-existent benchmark directory
* Fix missing macro include in tesseract_visualization cmake config
* Contributors: Levi-Armstrong, Michael Ripperger

0.6.0 (2021-10-15)
------------------
* Docker Images from CI (`#636 <https://github.com/tesseract-robotics/tesseract/issues/636>`_)
  * Updated CI configuration to push post-build Docker images to container registry
  * Updated if statement syntax
  * Updated to use version of industrial_ci that builds docker image in specifiable location
  * Update source command with new location of built workspace
* Update CodeCov badge
* Remove IKInput alias and cleanup KinematicGroup
* Fix bug in kdl sub tree parser
* Additional changes to support tesseract_ros update
* Rebase and fix clang tidy errors
* Update environment getTCPOffset
* Integrated tool center point struct into manipulator info struct
* Rebase on upstream/master
* Remove getName from forward and inverse kinematics interface
* Update visualization to use SceneState
* Add isActiveLinkName and hasLinkName to state solver interface
* Fix kdl sub tree parser
* Set limits for continuous joints when adding joint to scene_graph
* Add isActiveLinkName and hasLinkName and additional getJacobian methods to kinematics
* Rename getCurrentState to getState and return copy
* Move calcRotationError from trajopt
* Update documentation
  Co-authored-by: Michael Ripperger <michael.ripperger@swri.org>
* Remove KDLFwdKinTree
* Remove kdl_multi_chain_tree_builder
* Remove AdjacencyMap class
* Fix CI builds and code coverage
* CMake Format
* Switch to using .clang-tidy file and fix clang-tidy errors
* Update tesseract_visualization based on recent changes
* Update tesseract_environment to leverage kin plugin factory, mutable state solver and remove manipulator manager
* Update OFKT state solver to allow replace and move
* Add helper methods to KinematicsInformation
* Fix kin plugin factory interface, UR e-series parameters, and changes to resource locator
* Update state solver interface to get active and static links and active joints
* Update to leverage resource locator as const reference instead of shared pointer
* Update tesseract_srdf to leverage KinematicsPluginInfo structure
* Fix PluginLoader member variables names
* Add KinematicsPluginInfo to tesseract_common
* Move tesseract_common yaml conversions to yaml_utils.h
* Update kinematics to allow setting the solver name at construction
* Update kinematic plugins yaml use a solver name map
* Update kinematic plugins yaml use a group name map
* Add generic plugin info struct
* Add UR e models
* Add ability to load kinematics_plugin_config file to srdf
* Move resource locator from tesseract_scene_graph to tesseract_common
* Add individual plugin loading unit tests
* Add ability to set default fwd and inv kin solvers
* Add JointGroup to tesseract kinematics
* Add tesseract kinematics plugin factory
* Update plugin loader to search all libraries
* Replace init function in kinematics with constructors
* Add calcInvKin input structure and update so IKSolver have a single working frame
* Add jacobian calculation to OFKT State solver
* Improve kinematic group and fix remaining unit tests
* Update state solver interface to not return references
* Update kinematic group to leverage state solver for fwd kin and jacobian
* Update state solver to include jacobian and active links
* Update paresSceneGraph subgraph to be relative to world
* Move state solver into its own package with unit tests
* Restructure inv kin factory
* Restructure fwd kin factory
* Add more kdl parsers, add scene state, and support methods to scene graph
* Add kinematics group class
* Fix multiple definitions of IKFAST_HAS_LIBRARY
* Update manipulator manager to take environment object instead of scene_graph
* Inv Kin base and tip link name should not be synchronized with fwd kin
* Update REP and ROP unit test to use synchronization
* Fix generation of active link names in ROP and REP kinematics
* Fix generation of active link names in kdl kinematics
* Add ikfast unit test and fix clang-tidy errors
* Add tesseract_kinematics ikfast build test
* Add ability to synchronize inverse kinematics object to forward kinematics object
* Remove References to Deprecated tesseract_geometry Functions (`#635 <https://github.com/tesseract-robotics/tesseract/issues/635>`_)
* Add URDF Writer function and common interface for mesh types (`#633 <https://github.com/tesseract-robotics/tesseract/issues/633>`_)
* Fix documentation for getInboundJoints and getOutboundJoints
* fix typo
* Update convex_decomposition_vhacd.cpp
* remove duplication in rep inv kinematics (`#624 <https://github.com/tesseract-robotics/tesseract/issues/624>`_)
* Add support function for getting active and static links along with collision pairs
* Documentation move to tesseract_docs and updates example code (`#593 <https://github.com/tesseract-robotics/tesseract/issues/593>`_)
* Fix Spelling of Convex Decomposition (`#620 <https://github.com/tesseract-robotics/tesseract/issues/620>`_)
* fix typo in rep inverse kinematics (`#617 <https://github.com/tesseract-robotics/tesseract/issues/617>`_)
* Add redundancy methods to Forward and Inverse Kinematics interface (`#616 <https://github.com/tesseract-robotics/tesseract/issues/616>`_)
* Contributors: DavidMerzJr, Levi Armstrong, Levi-Armstrong, Michael Ripperger, Mohamed Ahmed, Steven Dale, mohamedsayed18

0.5.0 (2021-07-02)
------------------
* Update changelog
* Add convex decomposition support (`#609 <https://github.com/tesseract-robotics/tesseract/issues/609>`_)
* Fix benchmarking github actions file
* Fix environment clone benchmarks
* Fix SRDF REP and ROP positioner joint parsing
* IK Solver Redundant Solutions Update (`#601 <https://github.com/tesseract-robotics/tesseract/issues/601>`_)
* fix error so that initial state has dt=0 (`#604 <https://github.com/tesseract-robotics/tesseract/issues/604>`_)
* Remove deprecated code in tesseract_environment
* Change tesseract_visualization cmake message type to STATUS
* Store timestamp when environment state is set
* Contributors: Levi Armstrong, Michael Ripperger, cbw36

0.4.1 (2021-04-24)
------------------
* Update changelogs
* Remove windows compiler definition NOMINMAX
* Do not add compiler option -mno-avx if processor is uknown
* Contributors: Levi Armstrong

0.4.0 (2021-04-23)
------------------
* Update changelogs
* Disable building ompl for windows build
* Add windows compile definition NOMINMAX
* Fix test names in tesseract_srdf
* Improve tesseract_common unit test coverage
* Improve exception text in urdf_parser
* Fix spelling tesseract_srdf package.xml
* Add equal operator support to Any type erasure
* Fix package build depends
* Improve tesseract_common unit coverage
* Disable compile option -mno-avx for arm builds
* Update tesseract_srdf to leverage nested exceptions
* Move srdf code to its own package tesseract_srdf
* Move printNestedException and leverage forward declarations for tesseract_urdf
* Do not catch exception in parseURDFString and parseURDFFile
* Move tesseract_urdf implementation to cpp and fix clang tidy errors
* Improve tesseract_urdf unit test coverage
* Fix issue in trajectory_player calling size if not trajectory exists
* Switch tesseract_urdf to use nested exception instead of custom status code class
* Contributors: Levi Armstrong

0.3.1 (2021-04-14)
------------------
* Update changelog
* Add missing pcl depends to tesseract_urdf package.xml
* Add bullet-extras depends to tesseract_collision package.xml
* Move tesseract_variables() before any use of custom macros
* Contributors: Levi Armstrong

0.3.0 (2021-04-09)
------------------
* Add changelog files
* BSD Code is actually BSD-2-Clause so updating license file
* Only enable code coverage if compiler definition is set
* Fix issue in trajectory player setCurrentDuration not handling finished bool
* Fix bullet broadphase when new links are added
* Debug unit test
* Move serialize implementation to cpp based on boost documentation for shared libraries
* Rename Any method cast() and cast_const() to as()
* Remove NullAny structure
* Cleanup equal operator
* Fix satisfiesPositionLimits to use relative equal and calculation of redundant solutions to include all permutations
* Fix inv kinematics to only return solution within limits and add redundant solutions for kdl ik solvers
* Fix conversion warnings
  - Use size_t everywhere we expect to index a vector
  - Cast the result of rand unsigned
* Update benchmarks to use collision margin data
* Make compatible with fcl version 0.6
* Split loading plugins into two classes ClassLoader and PluginLoader
* Remove dependency on class_loader and leverage Boost DLL
* Add PluginLoader class to tesseract_common
* Fixup enforceJointLimits
  Up to now, it would incorrectly apply the upper limit to any position
  that's outside the range. For example, a position that's slightly under
  the lower limit would get assigned the upper limit. Fix this by using
  Eigen's min and max functions, resulting in a proper clamp.
* Add satisfy and enforce position limits utility functions (`#576 <https://github.com/tesseract-robotics/tesseract/issues/576>`_)
* Add QueryIntAttributeRequired utility function
* Add cmake format
* Add kinematics utility function for calculating manipulability (`#571 <https://github.com/tesseract-robotics/tesseract/issues/571>`_)
* Add support for defining collision margin data in SRDF (`#573 <https://github.com/tesseract-robotics/tesseract/issues/573>`_)
* Use boost targets, add cpack and license file (`#572 <https://github.com/tesseract-robotics/tesseract/issues/572>`_)
* Add kinematics utility function isNearSingularity (`#569 <https://github.com/tesseract-robotics/tesseract/issues/569>`_)
* Fix the way in which Eigen is included (`#570 <https://github.com/tesseract-robotics/tesseract/issues/570>`_)
* Add logic to how a provided collision margin data can be applied
* Fix method for updating max margin in CollisionMarginData
* Fix checkJoints in UR Kinematics
* Add universal robot inverse kinematics
* Fix kinematics checkJoints not returning false when outside limits (`#564 <https://github.com/tesseract-robotics/tesseract/issues/564>`_)
* Add scale to tool path marker
* Add libomp-dev as test_depend to tesseract_environment and tesseract_collision
* Add multithreaded environment unit test
* Fix mutex locking bug in environment applyCommands
* Add ability to construct ROP and REP kinematic solver with different solver names
* Modify trajectory interpolator to visualize trajectories w/o time
* Fix method for changing bullet extern gDbvtMargin
* Add serializable any type erasure (`#555 <https://github.com/tesseract-robotics/tesseract/issues/555>`_)
* Add ToolCenterPoint unit tests
* Fix IKFast inverse kinematics wrapper
* Start to adding boost serialization support
* Update forward kinematics interface to return solutions versus out parameters
* Update rosinstall to leverage tags
* Update inverse kinematics interface to return solutions versus out parameters
* Contributors: Hervé Audren, Levi Armstrong, Matthew Powelson, david.hooks

0.2.0 (2021-02-17)
------------------
* Add utility function to scale vertices about a point
* Add ability to replace link and joint pair where the link is the child link of joint
* Improve clone cache unit tests and fix issues with getting clone
* Add getSourceLink, getTargetLink, getInboundJoints and getOutboundJoints scene graph unit test
* Add manipulator manager unit tests
* Add support for replacing links and joints
* Allow almostEqualRelativeAndAbs handle empty vectors
* Rename AddCommand to AddLinkCommand
* Update environment to leverage shared mutex
* Improve unit test coverage and registar FCL as an available contact manager
* Update StateSolver init to take a revision number
* Fix mutex dead lock in tesseract environment
* Switch addJoint, addLink, moveLink and addSceneGraph to use const&
* Improve tesseract_environment unit test coverage
* Fix scene graph default visibility and collision enabled
* Refactor tesseract_environment to use applyCommands
* Add tesseract_common::BytesResource unit test (`#545 <https://github.com/tesseract-robotics/tesseract/issues/545>`_)
* Improve tesseract_kinematics test coverage (`#543 <https://github.com/tesseract-robotics/tesseract/issues/543>`_)
* Add simple timer class
* tesseract_environement: Improve documentation
* Improve tesseract_scene_graph test coverage and fix found issues
* Add vectorized version of almostEqualRelativeAndAbs to compare if two vectors are equal
* Add another test case for kinematics core harmonize function
* Fix harmonizeTowardZero
* Update cmake_common_scripts to ros_industrial_cmake_boilerplate
* Improve code coverage for tesseract_kinematics core
* Add tesseract_scene_graph resource locator unit test
* Fix bug in SimpleLocateResource::getResourceContentStream
* Improve srdf test coverage
* Add more unit tests for scene graph
* Fix bug in scene graph change joint limits
* Add missing depend in tesseract_scene_graph
* Add SWIG shared_ptr commands to visualization markers
* Move tesseract_command_langauge package to tesseract_planning repo
* Add marker support and remove dependency on command language
* Update Findtcmalloc.cmake to support windows
* Fix focal ci by adding ici_with_unset_variables EMBED script
* Add Findtcmalloc.cmake file
* Updated rosinstall to use specific versions for dependencies
* Fix bug in isToleranced
* Update build instruction for noetic
* Update CI
* Move all directories in tesseract directory up one level
* Remove tesseract_planning sub directory which is its own repo now
* Remove tesseract_python which is now its own repo
* Add utility to get allowed collisions for a given link (`#382 <https://github.com/tesseract-robotics/tesseract/issues/382>`_)
* Update packages package.xml to include buildtool_depend on cmake and exec_depend on catkin
* Add tesseract_ext to windows ci build
* remove tesseract_ext from rosinstall
* Update to build on noetic without tesseract_ext
* Move tesseract_command_language out of tesseract_planning directory
* Add maybe-unitialized to push pop macro
* Switch DebugObserver to use console bridge
* Add error task and done task to GraphTaskflow
* Add TrajOpt Ifopt planner (`#443 <https://github.com/tesseract-robotics/tesseract/issues/443>`_)
* Move ProcessInfo into ProcessInterface for outside access (`#514 <https://github.com/tesseract-robotics/tesseract/issues/514>`_)
  * Move ProcessInfo into ProcessInterface for outside access
  * Rename Process to Task for generators and associated types
  ProcessGenerator -> TaskGenerator
  ProcessInterface -> TaskflowInterface
  ProcessInfo -> TaskInfo
  ProcessInfoContainer -> TaskInfoContainer
  ProcessInput -> TaskInput
  * Fix remaining changes
  Co-authored-by: Levi Armstrong <levi.armstrong@swri.org>
* Update processResults in tesseract_collision distance check
* Clang format code
* Add unit tests for improved mesh parser and fix normals loading bug (`#509 <https://github.com/tesseract-robotics/tesseract/issues/509>`_)
* Add note to mesh parser about possible fix for Y_UP issue with assimp
* Remove unused header from tesseract_time_parameterization_python.i
* Remove yaml-cpp from the tesseract_python_module.cmake
* Improved mesh parsing to include normals, vertex color, material and texture (`#490 <https://github.com/tesseract-robotics/tesseract/issues/490>`_)
* Split clang-tidy args across multiple lines to make it easier to read
* Update planners to use CollisionCheckConfig
* Fix readme instructions for testing by changing tesseract_process_planners to tesseract_process_managers and add tesseract_python
* Update motion planners to account for Joint and State Waypoints unordered joints relative to kinematics
* Add support for external tcp attached to kinematic link
* Switch get_tesseract() to get_environment() in python tests
* Fix warnings in tesseract scene graph unit test
* Utilize  parameter in TrajOpt planner
* Update to use initialize_code_coverage() macro and compiler definition
* Extract package name and version from package.xml
* Get Robot Redundancy (`#486 <https://github.com/tesseract-robotics/tesseract/issues/486>`_)
  Co-authored-by: Colin Lewis <ctlewis@swri.org>
  Co-authored-by: David Merz, Jr <david.merz@swri.org>
* Remove tesseract_process_planners package
* Call reset when new program is set in trajectory player
* Fix message in default ompl plan profile
* Add unit tests for tesseract_urdf primitives with values less than one
* Several fixes in tesseract_collision
* Remove use of new operator
* Update to clang-tidy version 10
* Make non-virtual-dtor errors
* Remove process_managers, replaced by planning server
* Remove deprecated collision class methods and utility functions
* Update StateSolver to leverage KinematicLimits (`#494 <https://github.com/tesseract-robotics/tesseract/issues/494>`_)
* Add clang-tidy and python badge (`#492 <https://github.com/tesseract-robotics/tesseract/issues/492>`_)
* Python package updates for command language
* Add missing colcon.pkg files
* Simplify the process generator interface to avoid std::function
* remove tesseract package from windows ci build
* Fix bug in tesseract_collision processResult function
* Make changes to better support python wrapping
* Remove taskflow from target_workspace in CI, now included in docker
* Remove tesseract package
* Make getShortestPath const
  Indeed, it does not (and should not) modify the underlying graph.
* Add external tool center point support
* Add generateNaiveSeedGenerator function
* Add TrajOpt Solver Profile
* Fix handling of collision margin data in environment
* Clean up warnings related to setContactDistanceThreshold
* Fix bug in createCollisionTermInfo
* Add core directory to tesseract_process_managers
* Update ProfileDictionary and add additional unit tests
* Update state sampler allocator function signature
* Add Tesseract Evolution Gif
* Switch to using lambda over std::bind and remove NOLINT
* Add bool has_seed to ProcessInput and add back GraphTaskflow
* More documentation, remove commented code, some requested changes
* Add doxygen and a few bug fixes
* Make profiles and ProfileDictionary const
* Fix issue in lvs cart cart interpolation
* Restructure taskflow generators to support composition
* Make trajopt, ompl, descartes, freespace and cartesian taskflow generators
* Fix enabling of simple planner
* Move default process planners to method that user calls
* Add profile dictionary
* Create process planning server
* Add console_bridge dependency to tesseract_common
* Add CloneCache
* Add feedback of contacts to FixStateCollisionProcessGenerator
* Add virtual destructor to ProcessInfo as well as bug fixes
* Remove limits from revolute nodes in OFKT state solver
* Fix bug in TesseractIgnitionVisualization::plotEnvironment
* Include missing libraries in cmake message in tesseract_visualization
* Add environment commands to modify contact margin data
* Add tesseract_time_parameterization to windows build
* Enable building tesseract process managers on windows
* Add some useful methods to collision structs
* Add SFINAE function signature check to command language
* Remove xenial CI build
* Fix tesseract_collision example
* Add SFINAE function signature checks
* Add SFINAE utils
* Improve error handling in joint and state waypoint
* Fix issue parsing opw from srdf with unit test
* Add ProcessInfo to process generators (`#450 <https://github.com/tesseract-robotics/tesseract/issues/450>`_)
* Add wait and timer instruction to command language
* Add CollisionCheckConfig
* Update to leverage kinematics setLimits for environment changes
* Fix bug in simple planner not resetting start waypoint
* fix lvs process flow and step calculation
* Add clone method to moiton planner base class
* Add vertex evaluator to descartes
* Clear KDL data in init
* Code Simplification in StateInCollision
* Add executable that profiles currently implemented contact checkers
* Add ability to change collision margin on a pair-to-pair basis
* Fix constraint from error function in trajopt plan profile
* Fix eigen issues with kinematics information
* Fix tesseract initialize with environment
* Add KinematicsInformation class and modify srdf model class
* Update ChangeJointLimits is only for joint limits
* REP and ROP update sub kinematics first
* Add setLimits to forward and inverse kinematics
* Move ManipulatorManager into Environment
* Add ManipulatorManager update method
* Add changeJointLimits Command
* Limit the use of new operator in tesseract_collision
* Add MoveWaypointFromCollisionRandomSampler to FixStateCollisionProcess (`#426 <https://github.com/tesseract-robotics/tesseract/issues/426>`_)
  * Add MoveWaypointFromCollisionRandomSampler to FixStateCollisionProcess
  * Add more generalized way of specifying correction methods
  * Bug fix
  * Add assert to catch mismatched sizes
  * Rebase fixes and a bug fix
* Add seed min length process generator and unit tests
* Update unit tests and fix lvs_interpolation
* Updated uses of fixed size interpolation to lvs interpolation
* Updated lvs tests to be more thorough
* Fix bug in trajopt default problem generator not getting composite profile correctly
* Add verbose options to process input so planner verbosity can be turned on
* Fix issue in freespace taskflow for the trajopt first condition
* Add raster only process managers
* Fix SimplePlanner step generators to correctly set profile
* Add ProfileSwitchProcessGenerator
  This generator simply returns a value specified in the composite profile. This can be used to switch execution based on the profile
* Add utility for getting profiles (`#412 <https://github.com/tesseract-robotics/tesseract/issues/412>`_)
* Add unit test for generateSkeletonSeed
* Enable tesseract_motion_planners build on windows
* Address console bridge issue `#91 <https://github.com/tesseract-robotics/tesseract/issues/91>`_
* Fix to handle console_bridge target renaming in noetic
* Separate public and private compiler option and add back -mno-avx
* Fix tesseract collision benchmarks
* Add individual CI badges and Windows CI build
* Add missing build depend cmake_common_scripts to tesseract_visualization
* Update Tesseract clone to increase performance
* Add Tesseract clone benchmark
* Add framework for Tesseract init/clone unit tests
* Add asserts to setState that the number of joint names/values match
  Otherwise, an empty joint names vector will silently not set the state.
* Check validity of longest valid segment
* Set active links based on ManipulatorInfo in contact check processes
* Fix bug in checkTrajectory where discrete collisions ignored last step
* Add visibility control to all packages
* Expose transpose method for Joint Waypoint
* Add print to waypoint
* Update CI and rosinstall to point to feature/CommandLanguage branches
* Update due to changes in descartes compound edge evaluator
* Fix done and error callback in simple process manager
* Change how Bullet is found to use PkgConfig which the recommended way
* Fix Code Coverage due to executable in tesseract_collision
* Fix tesseract_common utils c++ version check for inline variables
* Remove inheritance of Eigen::VectorXd from Joint Waypoint
* Remove inheritance of Eigen::Isometry3d from Cartesian Waypoint
* Remove inheritance of std::vector from Composite Instruction
* Rename buffer_margin to safety_margin for consistency
* Change Tesseract findTCP to throw exception when not found and update planners to handle this exception
* Add support for user defined find tcp callbacks
* Remove const from return type in ikfast.h
* Improve global raster taskflow
* Switch from Cast Continuous to Discrete Continuous
* Update default longest valid segment length
* Update trajectory player to support rviz visualization
* Add taskflow debug and profile observer
* Improve trajectory player and add utility getJointNames from waypoint
* Fix ompl default plan profile not setting planning time
* Update CompositeInstruction toXML so Null StartInstructions are not output
* Change freespace taskflow to still try trajopt if ompl fails
* Fix graph taskflow handeling of TASK type
* Add isIdentical for two vectors of strings
* Fix trajectory interpolator where time restarts
* Fix rep and rop active links and filtering duplicates
* Improve logging for checkKinematics functions
* Fix descartes handeling of freespace plan types
* Add simple planner longest valid segment interpolation (`#385 <https://github.com/tesseract-robotics/tesseract/issues/385>`_)
  Co-authored-by: Stevie Dale <steven.dale@swri.org>
* Change ProcessInput to better support changing data structure throughout the taskflow
* Update REP and ROP Kinematics along with ManipulatorInfo TCP support
* Add global raster variant
* Correct handling of limits for OFKTStateSolver
* Improve ignition material conversion
* Leverage trajectory player in ignition visualizer
* Add manipulator manager to support tracking changes in the future
* Refactor fix state bounds utils to eliminate repetitive inform msgs
* Pass verbose to motion planners only when debug messages enabled
* Add clang static analyzers
* Leverage cmake_common_scripts
* Add tesseract ignition visualization
* Improve tesseract init and add reset function
* Remove debugging printout from OFKT
* Add fixStateBoundsProcessGenerator
* Clean up tesseract_process_managers and tesseract_motion_planners package
* Add fixStateBoundsProcessGenerator
* Add clampToJointLimits utility
* Change OFKT joint limits warning to a debug message
* Change default state solver to OFKT
  This solver is considerably faster than KDL when modifying the environment
* Fix OFKT clone method
* Switch ISP to use MoveInstructions instead of PlanInstructions
* Add motion planner serialization (`#356 <https://github.com/tesseract-robotics/tesseract/issues/356>`_)
* Add Profiles to ISP Time parameterization process generator
* Allow ISP scaling factors to be changed on a point by point basis
* Process managers: Only print "Generating Taskflow for..." when log debug
* Optimized Forward Kinematic Tree State Solver
* Move isTree check in kdl parser into assert
* Split command_language_utils into multiple files
* Simplify raster example program
* Add simple process manager and planner profile mapping
* Add BiTRRT Configurator
* Add streaming operators to acm and JointLimits
* Fix is_contact_allowed_fn\_ in Environment::clone
* Break up serialization and deserialization and make deserialization more flexible
* Add XML serialization to tesseract_command_language
* Add XML QueryStringText to tesseract_common
* Expose velocity and acceleration scaling factors in process generators
* Add debugging information when planning fails due to collisions
* Change OMPL default safety margin to 0.0
  This essentially removes the 0.025 inflation that was added previously.
* Fix typo in ISP ProcessGenerator
* Update documentation on environment init method
* Fix clang tidy warnings
* Fix tesseract_environment unit tests
* Fix scene graph clone to include visibility and collision enabled
* Improve environment clone along with add AddSceneGraphCommand
* Improve scene graph insert methods and add unit tests
* Add clone method to Tesseract
* Add clone method to Environment
* Add applyCommands to Environment
* Add clone method to scene graph
* Fix Clang Tidy errors
* Fix/Add clearing of graph and sequential taskflow
* Update main github action yaml trajopt branch to feature/CommandLanguage
* Add graph taskflow
* Add trajectory interpolator and trajectory player to tesseract_visualization
* Fix const and indexing issue in tesseract planning
* Update rosinstall to point to respective feature/CommandLanguage branches
* Remove unused examples and dependencies from tesseract_command_language
* Add documentation for new acceleration for limits in urdf
* Add iterative spline parameterization process generator
* Add support for velocity and acceleration limits
* Rename iterative spline parameterization methods
* Add tesseract_time_parameterization package include iterative spline algorithm
* Add function for checking if two doubles are equal
* Set local LD_LIBRARY_PATH for visualization loader of snap plugin
* Remove random generators and validators
* Add discrete and continuous process generators
* Add snapcraft badge for tesseract_ignition tools to readme
* remove dependency descartes_opw
* Add new JointWaypoint constructor and fix clang tidy errors
* Update tesseract_visualization_loader to support command language
* Set OPWKin add default IK solver if in srdf
* Switch to using unique pointer for Process Generator
* Rename sequential_failure_tree_taskflow to sequential_taskflow
* Make command language utility function generic and move planner specific ones to motion planners package
* Get tesseract process managers working
* Swith process input to leverage pointer instead of references
* Improve support for state waypoint in simple motion planner
* Add srdf wiki documentation
* Update Readme
* Update tesseract_process_managers
* Add plotTrajectory for command language to visualization interface
* Add tesseract srdf documentation
* Add opw kinematics to abb_irb2400.srdf
* Update tesseract_command_language and tesseract_motion_planners
* Update/Fix tesseract process manager
* Make requested changes
* Remove unused header from motion planning example
* Add unit tests for fixed size assign position
* Update/Add examples to leverage ignition visualization
* Update motion planners to leverage new flatten utils and non-const getWaypoint
* Fix flatten utils and add non-const getWaypoint for Move and Plan Instruction
* Disable VisualizationLoader for class_loader versions less than 4.0.0
* Add VisualizationLoader to tesseract_visualization
* Address requested changes
* Add missing include <atomic>
* Add missing SHARED to libraries
* Add skeleton unit test for fixed size assign position
* Update motion planner example
* Address todo's in tesseract_motion_planners
* Fix simple planner fixed size interpolate unit tests
* Handle multple solutions in fixed_size_interpolate.cpp
* Update ci to use new dockers from rosindustrial
* Fix motion planners unit tests
* Bring back generateSeed, add readme, and add task validators
* tesseract_motion_planners: Alphabetize CMake targets
* Add SimpleMotionPlanner
  The simple planner is meant to be a tool for assigning values to the seed. The planner simply loops over all of the PlanInstructions and then calls the appropriate function from the profile. These functions do not depend on the seed, so this may be used to initialize the seed appropriately using e.g. linear interpolation.
* Replace position, velocity, etc in MoveInstruction with StateWaypoint
  This will allow us to change what the results of planners are without changing the MoveInstruction interface
* Add ManipulatorInfo to PlanInstruction
* Misc improvements and rebase fixes
  Modify examples so the complete successfully and clean some things
* Fix CCACHE_DIR for Focal build
* Update Defaults and add ability to abort process
* Add OMPL and Descartes support
* Update start and end Instructions in process managers
* tesseract_process_managers: Add raster_process_manager
  Adds the groundwork for a raster process manager along with an example using random processes.
* tesseract_process_managers initial commit
* Add missing include
* Add simple motion planning example using command language
* Fix ompl planner unit test
* Add missing license and warnings macro to files
* Switch setStartWaypoint to setStartInstruciton and update planners
* Fix descartes processing of results to handle freespace correctly
* Fix use of flatten functions and fix trajopt problem generator
* Tesseract_planning: Add data to request/response
* Clang Tidy fixes
* Move Flatten Utilities into tesseract_command_language
* Add option to include composites in results when flattening
* Tesseract planners: Make solve method const
* Simplify instruction class signature and utility functions
* Clang format
* Descartes planner: Copy solution into response
* Fix motion planner unit tests
* Add command language utils
* Fix trajopt and descartes missed merge issues
* Bug Fixes
* Refactor OMPL to use request/response
* Refactor Descartes to use request/response
* Refactor TrajOpt to use request/response
* Add command_language.h
* Change how start waypoint is defined, now provided by CompositeInstruction
* Clang format
* Update OMPL planner to support cartesian waypoints and supporting unit tests
* Add tesseract 512 logo
* Expose tolerance for checkKinematics functions
* Update package.xml email for tesseract_support and tesseract_scene_graph
* Remove hybrid planners
* Add Flatten utility
* Add basic print functions to instructions
* Improve descartes collision edge evaluator unit run time
* Add tesseract_kinematics robot with external positioner unit tests
* Add robot with external positioner to tesseract_support
* Clang-Format
* Add robot on positioner inverse kinematics unit test
* Add robot on positioner urdf and srdf to tesseract_support
* Update ompl to use new kinematics objects and fix clang-tidy
* Update descartes to only use new tesseract_kinematics objects
* Add Robot on Positioner and Robot with External Positioner to tesseract_kinematics
* Update OMPL to leverage command language
* Remove hybrid planners
* Working descartes unit tests with command language
* Working trajopt unit tests with command language
* Update generateSeed utility function for linear
* First pass at updating tesseract_motion_planners unit test with command language
* Fix error in isJointWaypoint
* Move new planner profiles to tesseract_motion_planners
* Switch to using profiles for plan instructions and composite instructions
* Add tesseract_command_language package
* Contributors: DavidMerzJr, Hervé Audren, John Wason, Levi Armstrong, Matthew Powelson, Michael Ripperger, Thomas Kostas, Tyler Marr, marrts

0.1.0 (2020-12-31)
------------------
* Added fixed timesteps to TrajOpt config
* Updated tesseract_python for change in trajopt config member
* Fix error message IKFast -> OPW
  The error message when attempting to use a non-implemented method in OPWInvKin mentions IKFastInvKin, so update that as it's probably a copy-paste error.
* Add forgotten include in opw_inv_kin.h
  Indeed, the header uses CONSOLE_BRIDGE_logError but does not include the
  relevant header, which means that including only that header causes
  compilation errors.
* Make getShortestPath const
  Indeed, it does not (and should not) modify the underlying graph.
* Fixed return tuple construction
* Fix use-after-move in Tesseract::init.
* Setting Active collision objects for the contact managers in trajopt motion planner
* Update api_docs.yml to keep benchmark files.
* Fix tesseract_viewer_python install Python executable in CMakeLists.txt
* Correctly return continuous joints as active joints
* Fix ikfast visibility and compiler options
* Add more logging to mesh parser and resource locator
* Add cmake_common_scripts to rosinstall file
  This is a new dependency of opw_kinematics
* Use grid material for ground in tesseract_viewer_python
* Add cube, box, and sphere shapes support to tesseract_viewer_python
* Add conditional to package.xml due to key renaming
* Bugfix when OMPL simplifies down to two states and trajopt was assuming > 2, so segfaulting
* Add plotting JointTrajectory to Visualization
  A Joint Trajectory currently contains joint names and a TrajArray, but that will likely change soon.
* Set CXX_STANDARD 14 instead of -std=c++14 for xenial builds
* Manually set C++14 for Xenial Tesseract_ros builds
* Add COLCON environment hooks to update ROS_PACKAGE_PATH
* Update CMake to work better with clang
* Remove two typos from README
* Add Noetic CI Build (`#305 <https://github.com/tesseract-robotics/tesseract/issues/305>`_)
  * Add Noetic CI build
  * remove redundant move
  * Add missing static_casts
  * remove more redundant moves
  * Another redundant move
  * Remove old header.
  * Add Python 3.8
  * Add Colcon environment hooks for Python packages
  * Bump tesseract_viewer_python required cmake version to 3.5.0
  * Add python version to tesseract_viewer_python
  * Source workspace before testing
  * Remove after script
  The tests are being run by colcon anyway
  Co-authored-by: Matthew Powelson <powelson.matthew@gmail.com>
* Add colcon.pkg files to all packages (`#303 <https://github.com/tesseract-robotics/tesseract/issues/303>`_)
  * Add colcon.pkg files to all packages
  Addresses issue `#302 <https://github.com/tesseract-robotics/tesseract/issues/302>`_ as discussed on rosdep issue 724.
  * tesseract_collision: Remove pluginlib workaround
  This is now handled in the tesseract_configure_package macro
  * Add benchmark to the xenial nightly build skip keys
* Switch octomap to use CMake targets
* Update index.html
  https://ros-industrial-consortium.github.io/tesseract/doxygen/html/index.html
* Add documentation links to readme
* Deploy API documentation gh-pages branch
* Rewrite of the srdf model class within tesseract (`#292 <https://github.com/tesseract-robotics/tesseract/issues/292>`_)
  * Clean up SRDFModel and restructure
  * Add opw kinematic parsing to srdfmodel and update tesseract python
  * Fix SWIG Python data types in srdf_model.i
  * Add new construction method to joint waypoint type
  * Move SRDFModel OPWKinematicsParameters structure outside the class
  * Fix SWIG build error in sdf_model.i
  * Clang format and fix random number definition
  * Remove unsupported methods in TinyXML2 in Kinetic
  * Expose resource locator in tesseract object
  * Modify collision large dataset unit to print information
  * Break up srdf_model.cpp into smaller files and fix requested changes
  Co-authored-by: John Wason <wason@wasontech.com>
* Add IFOPT to the rosinstall
  This is a new dependency for TrajOpt until IFOPT is released again
* Clean up ikfast_inv_kin
* Add missing console_bridge to tesseract_kinematics_core.
* add process definition generator interface (`#289 <https://github.com/tesseract-robotics/tesseract/issues/289>`_)
  * add process definition generator interface
  * Created Doxygen, removed constructor, and standardized compiler def
  * clang'd
  Co-authored-by: ctlewis <colin.lewis@swri.org>
* Add ability to provided IsContactResultValid function in contact request.
  * Added special collision pairs for trajopt planner
  * Added capablities to allow negative special collision pairs to pass post-check
  * Removed commented code
  * Clang formatting
  * Removed unnecessary lines
  * Removed unused variables
  * Changed collision pairs to use safetyMarginData type
  * changed nullptr assignment
  * Moved negative collision checking into trajectoryValid function
  * Clang formatting
  * Fixed build test failing
  * Clean up
  * Fix clang-tidy errors
  Co-authored-by: Tyler Marr <tyler.marr@swri.org>
  Co-authored-by: Levi Armstrong <levi.armstrong@gmail.com>
* Add support for collision managers that only provide a single contact point
* Add missing fcl contact result information for transforms and local transfroms
* Fix TesseractOverlapFilterCallback
* Add documentation to code
* Improve tesseract_collision test coverage
* Fix issues in FCL Manager found during unit testing
* Fix clang tidy errors
* Improve bullet collision checking performance to fail faster for contact test type FIRST and enable binary contact check only
* Add fcl collision object wrapper to improve distance calc performance
* Update test_suite to use new contact request object
* Update fcl to use new contact request and use bvh for static and one for dynamic
* Create collision request data structure
* Disable benchmarks in cron jobs
  Cron jobs do not currently work due to an issue in rhysd/github-action-benchmark.
* Add BENCHMARK_ARGS to disable long running ones in CI
* Benchmark: BM_LARGE_DATASET_SINGLELINK
  This compares a single link with lots of collision object with a link with one object. This is opposed to the multi link example that adds each object as its own link. This should be faster as filtering should remove collisions between objects in the same link.
* Fixes after rebase
* Link against correct test_suite library
* Fix benchmarking in Pull requests
* Benchmark: Add FCLDiscreteBVH
  This has some of the tests removed by default since they are too slow to run in any practical setting
* Add ConstPtr to FCLDiscreteBVHManager
* Benchmark: Add BulletDiscreteBVH
* Benchmarking: Add Large Dataset benchmark
  This checks collisions between a large grid of spheres with the edge size of the grid increasing.
* Move tesseract_collision test resources to tesseract_support
* Add broadphase overlap filter callback
* Disable benchmarking fail on alert
  Until we get this more stable, failing seems like a bad idea. It will still comment if there is a performance regression.
* Correctly overwrite collision objects with the same name
* Add Github Action Benchmarking
* Benchmark setCollisionObjectTransform setting from vector and map
* Add benchmark to ROSDEP_SKIP_KEYS for kinetic build
* Benchmark setCollisionObjectTransform
* Benchmark Collision Manager Clone Method
* Add Bullet Discrete Simple ContactTest Benchmarks
* Fix Tesseract Nightly Workflow
  Previously was getting `pull_request` requires a map value error
* Split downstream builds into another ci workflow that runs nightly
* Add freespace step to github actions
* Improve bullet compound collision algorithm to exit early
* Update tesseract_collision unit tests to different contact test types
* Improve tesseract_collision unit test coverage
* Remove unused code from bullet contact checkers
* Update readme to include how to exclude code from code coverage
* Changes to tesseract_collision ContactTestData structure
* Remove tesseract_ext submodule
* Update types.i
* Store joint transforms in EnvState structure (`#265 <https://github.com/tesseract-robotics/tesseract/issues/265>`_)
* Daily cron job at midnight CST
* Add ccache to github actions
* Explictily instantiate Descartes Hybrid planner
  Indeed, this template is defined in a .cpp so it needs to have explicit
  instantiation, done for double and float
* Fix error message for samplers in Descartes
  It used to say that the number of waypoints was wrong
* Create a test suite for tesseract_collision to be used by external packages
* Check dot file is open before writing
* Remove Unreachable code
* Add doxygen to convex_mesh, mesh and sdf_mesh geometry
* Improve tesseract_geometry unit tests
* Add lcov to package.xml
* Add codecov.yml
* Disable github actions fail-fast
* Improve tesseract_urdf unit tests
* Add codecov badge to readme
* Add code coverage to packages tesseract, tesseract_process_planners and tesseract_visualization
* Add TesseractInitInfo
  This struct contains information that can be used to recreate a Tesseract at the time of it's initialization
* Add code coverage macros and add code coverage to packages
* Configurable post-plan collision check (`#247 <https://github.com/tesseract-robotics/tesseract/issues/247>`_)
  * Added trajectory validator class
  * Updated planner base class to use trajectory validator class
  * Updated planners to use trajectory validator class
  * Updated python interface
  * Updated OMPL TrajOpt unit test
  * Clang format
* Update CI badge to point to github actions
* Add support for creating kinematic chain from multiple chains
* Disable ompl trajopt hybrid unit test
* Improve mesh parser debug message
* Adjust ompl trajopt hybrid unit test
* Add github actions and remove travis
* Speed up clang-tidy ci test
* update readme and rosinstall to include tesseract_ros
* Modify hybrid ompl trajopt planner to set range on ompl planner
* update .travis.yml file
* Move ompl constrained example to new tesseract_ros repo
* Fix ompl kinetic unit tests
* Remove additional planners from the ompl unit tests
* Adjust ompl unit tests and add asserts
* Only add state collision validator when continuous_collision is false
* Allow multiple threads for CI builds
* Break up the unit testing to speed up CI
* Add ompl glass up right example
* Move ompl constrained to its own config
* Use ompl state extractor to eigen and add state validator
* Update to use generic method for extracting data out of ompl state
* Add ability to add constraints to ompl planner
* Fix Python 2/3 Interaction
* Cleanup ROS references
* Enable Python 3 wrappers
* Remove tesseract_ros python wrappers
* Remove tesseract_ros
  This will be moved to an independent repository
* Remove OMPL EST planner from the unit tests
* Add Extension Approach Generator
* Modify link_widget to always load collision mesh from data
* Update toWaypoint to accept parent_link name
* Add end state to the process definition structure
* Fix parsing of multiple visual nodes in tesseract_urdf and add unit test
* Adding contact monitor library to catkin package
* Refactor contact monitor into library (`#234 <https://github.com/tesseract-robotics/tesseract/issues/234>`_)
  * First building state
  * Refactoring contact monitor as a library
  * Editing license comments
  * Adjustments to Appease Clang for Contact Monitor Refactor
  * Adding Explicit Constructors for Contact Monitor
  - explicitly delete copy constructor, copy assignment operator, move constructor, and move assignment operator, as default values fail. (Some class members cannot be properly copied or moved.)
  - This is to appease Clang.
* Add tesseract_scene_graph_example
  This example initializes 2 robots from a URDF. It then reattaches one of the robots to the end effector of the other robot
* Adjust Departure Generator (`#228 <https://github.com/tesseract-robotics/tesseract/issues/228>`_)
  * Adjust Departure Generator
  * Moving extension departure generator to separate file
  * Removing Whitespace to Appease Clang
  * Adding License to Extension Departure Generator
  * Adding License Text to tesseract_planning Files
  * Adding @briefs to the comment blocks at head of tesseract_planning files
* Update ompl trajopt hybrid test to only add collision as a constraint
* Use the ompl seed trajectory to set trajopt num_steps in hybrid planner
* Fix ompl unit tests
* Trajopt Planner: Set init data when using JOINT_INTERPOLATED
  Currently JOINT_INTERPOLATED is unusable since the data is not set.
* Add JOINT_WAYPOINT to fixed_steps list only if it isCritical
  Currently it treats any joint position waypoint as fixed which may not be the case depending on the coefficient
* Add libclang-dev as a test_depend to tesseract_common
  This addresses `#198 <https://github.com/tesseract-robotics/tesseract/issues/198>`_
* Modify OMPL planner and config to accept multiple planner types
* Add ability to merge a SceneGraph into another one (`#219 <https://github.com/tesseract-robotics/tesseract/issues/219>`_)
  * Allow to merge a SceneGraph into another one
  Needed to create prefixed copy operators for links and joints
  * Delete Link & Joint copy constructor / assignment
  This means a large refactoring of the codebase to remove all instances
  - Add some functions that take a Ptr as argument, to avoid having to
  move instances being pointed at
  - Add calls to std::move where appropriate
  - Modify the code to no longer use moved instances
  * Use std::move in tesseract_rosutils
  * Use std::move in tesseract_scene_graph unit tests
  * Use std::move in tesseract_motion_planners
  * Use std::move in tesseract_rviz
  * Use std::move in tesseract_examples
  * Update tesseract_python to support move semantics
  This requires the introduction of 3 changes:
  - In scene_graph, only bind Ptr versions
  - In environment, introduce custom wrappers that copy the incoming Ptr
  - In msg conversions, use a new macro type that moves the return value
  into a Ptr
  * Fix the clang-tidy warnings
  * Make adding of joints / links pointers protected
  This ensures that nobody can modify the scene graph once built
  This required a tiny hack in the URDF parser, we should upgrade the
  interface to unique pointers in the future.
  * Update documentation for addSceneGraph
  * Make name\_ a non-const member of Joint and Link
  * Fix tesseract_python to clone the links
  They can only be passed by pointer
  * Wrap <queue> include with ignore warnings macros
  * Use variables for joint & link names in tests
  This only concerns tesseract_environment_unit for now
  * Fix test: was using link after moving it
  Created a variable to hold the name, and use that instead of getName()
* Add eigen to package.xml
  and alphabetize the entries.
* Expose trajopt collision term use_weighted_sum
* Updated axial approach and departure generators to interpolate orientation and to not duplicate the starting point
* Set collision cost safety margin buffer to zero by default
* Add safety_margin_buffer to Swig wrapper
* Add safety_margin_buffer fields to Tesseract Trajopt planner objects
* Add initial support for Groups and GroupStates (`#208 <https://github.com/tesseract-robotics/tesseract/issues/208>`_)
  * Add initial support for Groups and GroupStates
  This commit adds a new binding for srdf_parser.h
  Current support is:
  - Retrieving Groups and GroupStates
  - Accessing their members
  Note that due to SWIG's lack of support of nested classes, they end up
  flattened, at the root of the module.
  * Add extra templates to SWIG bindings for SRDF
  * Add SRDF parser tests
  * Move template definitions to tesseract_python.i
  * Run the SRDF parser test in main()
* Use BMPD solver for puzzle piece examples
* Fix missed disabling ompl planner hybirdization when config param optimize set to true
* Restructure ompl to leverage config structures like the trajopt planner
* Add optimization capability for OMPL freespace planner
* Allow adding TrajOpt collision terms as both constraints and costs (`#210 <https://github.com/tesseract-robotics/tesseract/issues/210>`_)
  * Add separate collisions terms for constraint and cost and expose in planner config
  * Add config structs for collision costs and constraints
  * Use 'enabled' instead of 'check'
  * Add missing license block
  * Clang format
  * Fix typo in license
  * Add swig wrapper for trajopt_collision_config.h
  * Add collision config members to Swig wrapper for default planner config
  * Fix collision enable/disable in tests
  * Update collision constraint def to new format
* Update submodule
* Update Travis CI to solve CI hanging
* Fix bug in descartes robot positioner sampler storing positioner limits as wrong type
* Change Eigen arguments that are passed by value to reference
* Update tesseract_ext submodule (`#211 <https://github.com/tesseract-robotics/tesseract/issues/211>`_)
* Add double precision flag to tesseract_collision target compile definitions (`#213 <https://github.com/tesseract-robotics/tesseract/issues/213>`_)
  * Add double precision flag to tesseract_collision target compile defs
  * Reduce number of jobs for both melodic and kinetic CI builds
  * Make all CI jobs limit ros parallel jobs to 2
  * Add tesseract_common to CI After Script
  * Change language to minimal per travis documentation
  https://docs.travis-ci.com/user/common-build-problems/#im-running-out-of-disk-space-in-my-build
  * Update CTEST Arguments
  * Remove CTEST_PROGRESS_OUTPUT=TRUE
  * Move CTEST args to CATKIN_CONFIG variable
  * Reduce CI to one process
  Co-authored-by: Levi Armstrong <levi.armstrong@gmail.com>
* Fix bug in trajopt default config accessing nullptr
* Add useful operators to Joint and Cartesian Waypoints
* Add use of trajopt params to python motion planning unit test
* Update tesseract_python
* Make requested changes
* Add custom FindSWIG.cmake to look for swig4.0 to enable building without passing compiler definition
* update travis ci script
* Update rviz link_widget to support octree render with sphere inside and outside
* Update to support bullet built with double precision
* Update ompl freespace planner to use Parallel Plan with hybridization disabled
* Remove descrete collision check from ompl continuous motion validator
* Update to support trajopt new discrete continuous
* Update trajopt planner handling of fixed start and end states for collision
* Add tesseract_python and tesseract_viewer_python
* Update testing instructions in readme
* Explictly look for libccd
  As FCL doesn't do it in their config file, we need to do it ourselves
* Turned avoid singularity off by default
* Changed planner debug logging from debug to info
* Clang tidy updates
* Changed default waypoint constraint names
* Added avoid singularity to TrajOpt motion planner utils and default configuration
* Set user index for CastHullShape
* Make requested changes
* Update due to changes in TrajOpt CollisionTerm supporting longest valid segment length
* Update motion planners post check to only use continuous contact checking
* Enable continuous collision checking for moving to moving objects
* Fix bug in tesseract kdl kinematics
* Switch to using state solver in descartes edge evaluator and ompl motion validator
* Add descartes collision edge evaluator to descartes unit tests
* Fix debug message in checkTrajectory
* Update dates and add asserts
* Clang Formatting
* Add descartes collision edge evaluator
* Update checkTrajectory and supporting funtion to state solver and contact test type
* Fix manipulation widget when urdf has continuous joints
* added the default constructor to the ProcessPlanner class
* Change WriteSimplePlyFile to use floats in vertices and ints in faces
  PCL previoulsy didn't like the meshes that resulted.
  property 'list uint8 uint32 vertex_indices' of element 'face' is not handled
  Failed to find match for field 'x'.
  Failed to find match for field 'y'.
  Failed to find match for field 'z'.
* Remove writeSimplePlyFile default color
  It will now return a mesh with vertex length 3 if no color is given rather than setting it to (100,100,100). This is primarily because PCL IO expects input meshes to have vertex length of 3.
* Add doxygen comment to contact_dist_threshold\_ member
* Add parameter to set DescartesCollision contact distance threshold
* Add processing of header files to clang-tidy
* Update rosinstall trajopt to master branch
* Update travis.yml to use jobs
* Change how unit test are ran
* Add docker image containing tesseract_ext libraries
* Set trajopt log level to Error to limit CI error log to long
* Add ability to edit joint values in manipulation widget
* Update travis ci ros and catkin jobs count
* Fix ompl to obey collision safety margin
* Update rosinstall to show CI passes ***Remove Before Merging***
* Improve checkTrajectory, OMPL and TrajOpt planners by adding longest_valid_segment_fraction and longest_valid_segment_length
* Add clang, clang-tidy and clang-tools to CI
* Clang format
* Fix ompl planner response and verify final trajectory is collision free
* Address remaining compiler and clang tidy warnings
* Improve ompl handling of the number of output states
* Expose ability to set collision coeff in trajopt configs
* Add ability to add user defined trajopt constraint type and coeff
* Make clang-tidy only run if ENABLE_CLANG_TIDY or ENABLE_TESTS is enabled
* Update based on Clang-Tidy
* Add tesseract_ext as a submodule
* Update README with badges
* Update based on Clang-Tidy and Clazy
* Check double joint / link removing in test
* Replace raw lock / unlock by lock_guard
  This is to avoid potential deadlock when returning early
  e.g. when deleting a non-existing link.
* Fix quaternion error and add test
* Fix issue with descartes pose sampler
* Improve debug message for utility function checkTrajectory
* Update ompl trajopt hybrid planner to use new resoure locator api
* Use ResourceLocator class instead of ResourceLocatorFn (`#172 <https://github.com/tesseract-robotics/tesseract/issues/172>`_)
  * Use Resource and ResourceLocator instead of locateResource function
  * More updates to use ResourceLocator
  * More updates to use ResourceLocator
  * Fix clang-format
  * Update Resource and ResourceLocator to use ROS Cpp style guidelines
  * Fix comments in resource_locator.cpp
  * Improve doxygen comments in resource.h and resource_locator.h
  * Clang format
* Added license to OMPL hybrid planner
* Added unit test for OMPL TrajOpt planner
* Added OMPL hybrid planner
* Fix allow failure in travis.yml for melodic
* Clang format
* Add capsule to fcl collision and create a unit test for capsule collision shape
* Update create geometry example and documentaiton
* Add cone and capsule to bullet collision and tesseract_rviz
* Updated octomap parsing error printout
* Add check in trajopt config for start joint waypoint not matching seed trajectory start
* Adjust for joint waypoint joint name order
* Add name to tesseract trajopt planner constraint from error function
* Update tesseract_urdf documentation
* Only add new commands to the environment
* Update trajopt planner to use trajopt UserDefinedTermInfo for error functions
* Only add ikfast solution if within joint limits
* OMPL Planner Simplification (`#160 <https://github.com/tesseract-robotics/tesseract/issues/160>`_)
  * Updated OMPL config structure
  * Updated OMPL unit to use typed test to test all OMPL planners
  * Clang format
  * Reorganized collision checking logic
  * Added optional interpolation parameter to OMPL config
  * Turned off continuous collision checking, added interpolation, and increased planning time in OMPL test
* Process Planner: Generating add from_start to generateProcessDefinition
* Trajopt Planner: Switch setConfiguration to pass shared_ptr by value
  When passed by reference, calling clear on the planner also clears the config that was passed in. If it is by reference, you will just be setting the planners config to nullptr not the original.
* Exposes joint weighting in trajopt default configuration
* limit the scope of the lock in the contact monitor
* cleanup contact_monitor.launch
* Use thread for contact monitor computation and add option to publish contact markers
* Add iterators to process segment definition class
* Fix removing joint and link in RViz (`#163 <https://github.com/tesseract-robotics/tesseract/issues/163>`_)
  * Delete joint->child instead of empty remove_link
  * Delete joint, then link
  * Remove joint before link in REMOVE_LINK
* Trajopt Planner: Expose QP Solver selection
* Fix current state monitor to store timestamps when new joint state message is received
* Allow is_valid nullptr for descartes samplers
* Fix casting of float array to Eigen VectorXd in descartes_collision.hpp
* Add plotWaypoint
* Fix getMarkerCylinderMsg header frame id
* Add constraint from error function option to the trajopt default config
* Add opw_kinematics as a depend in tesseract_kinematics
* Add cmake macros to simplify cmake files
* Updated to correctly parse the -D's and -m's of the PCL CMake arguments
* Use GTest named targets instead of lib and include
  ${GTEST_BOTH_LIBRARIES} becomes GTest::GTest and GTest::Main
  GTEST_INCLUDE_DIRS is no longer needed
* Add missing header to tesseract_kinematics validate.h
* Rename class and document new code
* Add descartes sampler for a single manipulator
* Add opw kinematics solver
* Switch joint limit check from warning to debug log
* Switch to using descartes samplers for railed and positioner systems
* Add tesseract inverse kinematics ikfast solver
* Add descartes collision, railed kinematics and positioner kinematics
* Fixed Boost version checking in tesseract_urdf
* Updated planner inheritance; added licenses; changed header include symbols
* Merged TrajOpt planner config base with planner config
* Clang formatting
* Added check for joint waypoint in first or last position for default TrajOpt planner config
* Updated trajopt motion planner test
* Updated pick and place example
* Updated Descartes hybrid planner with new configuration classes
* Removed TrajOpt array and freespace planners
* Created TrajOpt configuration classes and utilities
* Created TrajOpt planner configuration abstract base class with a method to create a TrajOptProb. Updated the TrajOpt motion planner to utilize the base configuration class
* Fix check for duplicate joints in urdf_parser.cpp
* Add missing inline to methods in tesseract_rosutils conversions
* delete unused #include <ros/console.h>
* Resolved PCL compile options error
* Allow duplicate material definitions
* Add urdf version number to tesseract_urdf for backwards compatibility
* Remove automatic conversion of geometry type mesh to convex shape
* Fix parsing of collision geometry types convex_mesh and add unit test
* Add ctest output log
* Fix ctest verbose output
* Updated TrajOpt planner unit test
* Clang formatting
* Updates to generators and examples to utilize cartesian pose getParentTransform method
* Updated Cartesian waypoint to hold a link relative to which its transformation is relative
* Updated joint toleranced waypoint to inherit from joint waypoint
* Add an explanatory comment to CMakeLists.txt
* Write file on installation that allows tesseract_collision plugins to be found by ROS2 packages
* Clange format
* Disable urdf parsing of point clouds for xenial due to pcl not supporting c++11
* Clean up urdfdom references
* Allow anonymous material names (Empty String)
* Fix material names in abb_irb2400 urdf
* Add octree geometry constructor that takes a point cloud
* Add AVX warning when compiling with non-GNU compiler
* Add missing test depends to tesseract_common and clang format
* Add -mno-avx as compile option to fix Eigen Alignment Issues
* Add tesseract_rviz as exec_depends to tesseract_ros_examples
* Change parsing of available materials where materials defined in links get added to the list
* Changes for compatibility with TinyXML2 16.04 system version
* add toNumeric utils function to convert string to floats to allow use of tinyxml2 on kinetic
* Clang Format
* switch quaternion attribute from q to wxyz
* Add available materials unit test
* Fix parsing of safety controller tags
* Fix tesseract_scene_graph srdf unit due to changes in urdf parser
* Use std::make_shared in tesseract_geometry
* Add tesseract_common unit tests for isNumeric function
* Improve tesseract_urdf unit tests based on PR comments
* Clang format changes
* Add tesseract_urdf documentation
* Move urdf parser to its own package
* Add geometry type capsule
* Add urdf parser independent of urdfdom or urdf_headers
* Clang Format
* Remove use of shared_ptr for KDLState solver member variable kde_tree\_
* Make KDLStateSolver thread safe
* Add the ability to clone the state solver and expose a method to get a cloned solver from environment
* Make tesseract environment thread safe
* Add missing code for flags JOINT_TRANSFORMS and ALLOWED_COLLISION_MATRIX in getEnvironmentInformation
* Modifying Flags in GetEnvironmentInformation.srv
  The GetEnvironmentInformation service uses the bitwise AND operator to separate user requests and send only certain information.  However, there were two logical errors.
  - First, the 'flag' for ALLOWED_COLLISION_MATRIX was 254.  254 converts to 1111 1110.  Requesting this matrix was identical to requesting everything from LINK_LIST through JOINT_TRANSFORM.  Therefore, all of those would return if the ACM was requested, and the ACM would return if any of those were requested.
  - Second, the 'flag' for COMMAND_HISTORY was 0.  0 converts to 0000 0000.  Since bitwise AND between any number and 0 is 0 (which C++ translates as false), COMMAND_HISTORY was never returned.
  In order to fix this:
  - Changed from 8-bit ints to 16-bit ints
  - Incremented all 'flag' numbers by one power of two (0->1, 1->2, 2->4, etc.)
  - Added a comment to the srv file explaining how these values are used.
* Descartes_tesseract_kinematics: Add license and harmonizeTowardsZero
* Add DescartesTesseractKinematics wrapper
  This adds a wrapper for a TesseractKinematics object such that it can be used with Descartes. It has currently only been tested with the default KDL kinematics
* Update travis to limit number of packages for melodic builds
* Add addition doxygen, unit tests, and clang format  addressing PR comments
* Add discrete checking to ompl continuous motion validator to catch self collisions
* clang format
* Add descartes motion planner unit test
* Add num_threads to descartes config and remove use of ROS_ERROR for descartes planner
* Add abb irb2400 to tesseract support
* Fix description in utils jacobian changeBase method
* ompl freespace check start and end position for collision
* Add isValid check in continuous and discrete motion validators
* Add discrete motion validator and cache contact managers in validators
* Update ompl freespace planner to use OMPL OptimizePlan
* Fix compiler warnings in waypoint.h
* Add license to new ompl files and add doxygen
* Add ompl planner specific setting and fix naming
* Switch to use Valid State Sampler to avoid cloning contact manager for every isValid check
* Update OMPL planner to planner interface
* Sphinx docs now use :start-after: and :end-before: to  identify code segments
* Clang format
* Reduced number of jobs for Melodic CI build
* Added assert header
* Changed logic to fail if optimizer does not converge
* Updated planners to implement changed in base class solve method
* Updated motion planner base class solve method to take optional verbosity argument
* Added functionality for printing collision information to console
* Added call to update collision transforms after setting active contact manager
* Add succeeded waypoints and failed waypoints to PlannerResponse
* Add bash script to run clang format
* Remove catkin buildtool dependency from tesseract_ext packages
* Avoid flattening collision meshes
* Fix bug in toEigen when there are joints not in the joint_name list
* clang format
* Add missing test_depends to tesseract_collision
* Update rosinstall to include opw_kinematics
* Add unit test validating shape id and subshape id from contact results
* Add ability to get collision object geometries and transfroms from contact managers
* Add shape id and subshape id to contact results
* Update to allow null collision interface for descartes planner
* Update contact_monitor.launch to point to urdf and srdf in tesseract_ros_examples
* Fix descartes config struct
* Fix Bug in toMsg for process definition
* Expose tesseract object in environment monitor
* Add JointTrajectory structure
* Update waypoint types with constructors and setters and getters
* Clean up descartes planner
* Add conversion for process plan message types
* Add process planning messages
* Add conversion from geometry_msgs/pose to eigen
* Rename process planner to_start to to_end
* Fix namespace name in tesseract_rosutils conversions.h
* Update planners to use status code and add descartes planner and descartes-trajopt hybrid planner
* Add status code class to tesseract_common
* Enable FCL tests since PR `#338 <https://github.com/tesseract-robotics/tesseract/issues/338>`_ was merged
* Don't set_and_check nonexistant tesseract_support include directory
* Add documentation, license to files and update dates
* Add exec_depends and increase test run times
* Fix clang formatting
* Fix current state changing to all zeros when environment changes
* Reconfigure test into libraryies and nodes with unit tests
* Add unit tests for examples in tesseract_ros_examples
* Correct planners to fill out response when not configured
* Get tesseract_ros_examples working
* Add missing active_link_lists\_ in copy constructor and equal operator
* Fix assert in tesseract.cpp
* Fix basic cartesian plan example
* Add TrajOpt Planner unit tests
  These tests test the TrajOptArrayPlanner and the TrajOptFreespacePlanner. They primarily check that the correct types
  of costs and constraints are added when the flags like smooth_velocity are specified. However they are not foolproof.
  They only check that at least one term of the correct type is in the cost or constraint vector. If there should be
  more than one, then it might not be caught. This could be improved in the future, but it is better than nothing.
  Additional features that could be tested in the future
  * Configuration costs added correctly
  * Intermediate waypoints added correctly to freespace
  * coeffs set correctly
  * init info is set correctly
  * Seed trajectory is set correctly
  * callbacks are added correctly
  * Number of steps are obeyed for freespace
  * continuous collision checking flag set correctly
* Add dependencies for tests on package libraries
* Fix incorrect test names in tesseract_collision
* Add tesseract build tests cmake arg
* Switch to using ExternalProject_Add for gtest building
* Reduce number of ROS jobs allowed on melodic ci builds
* Fix clang warnings
* Remove unused header for geometric_shapes
* Add missing depend tf2_eigen to tesseract_monitoring
* Remove unused header kdl_parser/kdl_parser.hpp
* Update rosdep keys in package.xml
* Add flags to ignore formating the macros.h file
* Fix macro in tesseract_common macros.h and clang format of kdl_fwd_kin_tree.h
* Set CI clang version to 8
* Update dependency to trajopt master branch
* Clange format version 8
* Unify shared pointer definition and switch typedef to using
* remove unused file bullet_collision_object.h
* Create process planning package (`#16 <https://github.com/tesseract-robotics/tesseract/issues/16>`_)
  * added the tesseract_process_planning package
  * added the conversions header to the tesseract_rosutils package
  * added the improvements made by @mpowelson to the process definition methods
  * renamed tesseract_process_planning to ...planners for consistency
  * reinstated previous dependencies in cmake file
  * corrected namespaces and header guards
  * renamed some directories in tesseract_process_planners and documented a base class
  * renamed tesseract_planners to tesseract_motion_planners
  * renamed the base class BasicPlanner to MotionPlanner
  * renamed from_home field to from_start
  * improvements to the tesseract MotionPlanner interface and trajopt derived classes
  * removed the const attribute from all the solve(...) methods
  * Clean up cmake add missed renaming from tesseract_planners to tesseract_motion_planners
* Post checks trajectories in trajopt_planner for collisions (`#15 <https://github.com/tesseract-robotics/tesseract/issues/15>`_)
  * Add checkTrajectory for discrete collision check
  * Add discrete collision check to trajopt_planner
  And return false if error code is negative
  * Fix checkTrajectory
* Add issue number for tracking todo's
* Add test depends to tesseract_geometry
* Add octomap to cmakelist
* Move kinematic managers from tesseract_kinematics to tesseract package
* Update readme to reflect new packages
* Add examples with sphinx documentation for tesseract_scene_graph, tesseract_collision and tesseract_geometry
* Update unit tests
* Move contents of tesseract_planning into tesseract_planners
* Namespace targets
* Add tesseract_common package
* Fix newTesseractState callback parsing of commands
* Add new service to environment monitor to environment information
* Add ability set tcp link for manipulation widget
* Fix release build of scene graph
* Fix manipulation widget and improve widget initialization
* remove duplicate target_compile_feature
* Use typedef type for urdf shared pointers
* Fix c11 compiler option in tesseract_visualization
* Improve free space planner to not add costs and aux constraints on fix joint waypoints
* Fix KDL build for versions less than 1.4
* Fix find_dependency for components in kinetic
* Fix kinetic c++11 cmake flag
* Add joint manipulation functionality to manipulation widget
* Add isWithinLimits to tesseract_kinematics utils.h
* Fix interface targets method of adding c++11 compiler option
* Convert shape_marker to individual shapes and improve manipulation widget
* Add cpp methods for resizing interactive marker
* Add ability to remove items from kinematics managers
* Fix issues with saving DOT files when vertices have spaces in names
* Bug fix to allow multiple Environment commands in processMsg
* Add service to save the scene graph in the environment monitor
* Fix spelling in EnvironmentCommand.msg
* Fix filling out response message for modify tesseract environment from contact and environment monitor
* Improve the ros resource located to allow paths starting with file://
* Add missing depend roslib to tesseract_rosutils
* Isolate rviz widgets under a parent property and fix reference frame in manipulation widget
* Working version of manipulator widget
* Fix missing return in kinematics clone methods
* Fix Runtime bug in contact monitor
* Allow multiple Tesseract State plugins in rviz
* Add changeJointOrigin to rviz visualization widget
* Add ChangeJointOrigin Unit test
* Add changeJointOrigin to environment
* Add descriptions to EnvironmentCommand.msg
* Add tf publisher to current_state_monitor class
* Fix tesseract_rviz visualization and trajectory visualization
* Update to account for scaling
* Add cmake support for xenial builds
* Add tesseract_collision to tesseract_rosutils package.xml
* Fix Clang warning
  cannot pass object of non-trivial type 'const std::string' (aka 'const basic_string<char>') through variadic function; call will abort at runtime
* Header and namespace bug fixes
* Stage 3 of minipulation widget
* Fix tesseract_collision plugins and environment loading of plugins
* Stage 2 of minipulation widget
* Start to manipulation widget
* Add forward and inverse kinematics factories
* Add inverse kinematics and get all link poses from forward kinematics
* Update to changes in trajopt problem description use of tesseract
* Fix ompl planner test to use tesseract class
* Add tesseract class
* Rename tesseract ros examples
* Rename tesseract_core folder to tesseract
* Fix tesseract_rviz trajectory visualizaiton when start state is not visible
* Update rviz widgets due to changes in tesseract
* Update rosutils parsing changed messages
* Add command history to tesseract state message
* Add scale variable to Mesh message
* Get tesseract trajopt examples working
* Add scale meta data when parsing meshes
* Add untility function to calculate jacobian numerically and update test to use
* Add scale meta data to meshes loaded from file
* Make getCommandHistory const and intialize member variables
* Fix basic cartesian plan example
* Minor fixes in tesseract_rviz render tools
* Fix tesseract_environment to get/set name from scene graph
* Switch to using gtest_discover_tests versus add_tests
* Update unit tests and first pass at updating examples
* Seperate the environment state solver into its own class to simplify creation of new environment classes
* Simplify tesseract_rviz and update to work with new version of tesseract
* Fix naming of environment commands
* Update the environment monitor to leverate services for updating environment and getting updates from the environment
* Fix invalid warning generated from tesseract_rviz EnvVisualization
* Improve tesseract_rviz environment visualization by cloning entities for trajectory visualization
* Add command history to tesseract environment to be leverage by distributed systems
* Configure bullet to be used in a multi threaded environment
* Add contact manager factory and move allowed collision matrix to tesseract_scene_graph
* Move trajopt examples to tesseract_ros and update to new version
* Fix tesseract_rosutils plotting.h build failure
* Fix environment creation, missing allowed collision matrix
* Fix kdl parser
* Update checkTrajectory and unit test for tesseract_environment
* Make ompl planner test an gtest
* Convert tesseract_planning to pure cmake
* Update unit tests with changes in tesseract scene graph
* Get tesseract_rvis updated with changes to tesseract core packages
* WIP: Move ROS package into sub folder
* Remove use of Eigen3::Eigen target due to bug in catkin
* Improve package config files
* move tesseract_planning new file to correct location after rebase
* Add tesseract_support package for containing files for tests and examples
* Rename fcl_ros and libccd_ros packages
* Make bullet3 a pure cmake package
* Make tesseract_core pure cmake packages
* Add gtest source to tesseract_ext for pure cmake packages
* Remove tesseract_ext metapackage
* Fix getLinkChildrenNames function and add unit tests
* Add getActiveLinkNames to forward kinematics
* Add ability to enable and disable collision for link through environment api
* Add getLinkChildrenNames and getJointChildrenNames to scene graph
* First pass clean up for tesseract_ros
* Add additional check to tesseract_scene_graph unit test
* Enable ability to add and remove joints in tesseract_environment
* Add ability to clone contact manager as empty
* Fix removing of link and joint in tesseract_scene_graph
* Add SRDFModel and parser
* Add getInvAdjacencyLinks method
* Fix urdf parser of calibration tag
* Add createKinematicsMap from SRDFModel
* Add getAdjacencyMap, Remove AllowedCollisionMatrix, Force Convex Hull for Collision Objects
* add missing export dependency to tesseract_collision
* Fix tesseract_visualization include directory structure
* Move tesseract_core test to tesseract_environment
* Switch mesh face storage from std::vector<int> to Eigen::VectorXi>
* Remove tesseract_core package
* Add package tesseract_visualization
* Move tesseract_scene_graph under tesseract_core
* Move tesseract_planning under tesseract_core
* Move tesseract_kinematics under tesseract_core
* Move tesseract_geometry under tesseract_core
* Move tesseract_environment under tesseract_core
* Move tesseract_collision under tesseract_core
* Add getAdjacentLinkNames to tesseract_scene_graph
* Add tesseract_environment package
* Add moveJoint method to scene graph
* Remove export library from tesseract_collision that does not exist
* Make minor fixes to tesseract_scene_graph
* Update tesseract_kinematics to leverage tesseract_scene_graph
* Add parser for SceneGraph to KDL Tree
* Expose graph getVertex and getEdge method
* Add urdf parser and tests to tesseract_scene_graph
* Add missing include to tesseract_geometry/types.h
* Update mesh parser to include convex hulls and add unit tests
* Add asserts to mesh.h sdf_mesh.h to error if not triangle meshes
* Fix incorrect includes in cylinder.h and octree.h
* Add mesh parser test
* Add mesh parser to tesseract_scene_graph
* Add macros.h
* remove unused dependencies
* Add tesseract_scene_graph
* Add tesseract_geometry package and update tesseract_collision to leverage new package
* Create tesseract_environment and semi-isolate
* Fix depends in tesseract_kinematics
* Add meshes to tesseract_ros to remove dependency on trajopt_examples
* Semi-Isolate Tesseract Kinematics
* Make minor fixes in tesseract_collision
* Update create_convex_hull to not use ros
* Switch to using console bridge
* Isolate tesseract_collision namespace
* Switch to using built in Collision Shapes
* Add passthrough for coeffs when interpolating waypoints
  It uses the coeffs and is_critical from the starting waypoint. This was a fairly arbitrary choice.
* Fix trajopt planner to return correct status
* Fix Clang Warnings and Clang Format
* Tesseract Planner: Add Desired Configuration parameter
* Tesseract Planner: Add JointTolerancedWaypoint
  And update the Trajopt Freespace and Trajopt Array planners to use them. Note: This API is probably not set in stone at this point.
* Add utils to interpolate between cartesian poses
* Tesseract Planner: Add flag to allow switching b/n costs/constraints
* Add coeffs to Tesseract Planner Waypoints
  Used to weight different terms in the waypoint. For example: joint 1 vs joint 2 of the same waypoint or waypoint 1 vs waypoint 2
* Add ability to add tcp to array and freespace planners
* Fix clang format for tesseract_planning
* Add missing break statements in freespace planner
* Fix waypoint orientation return order and change joint waypoint to eigen
* Flip  quaternion orientation order (xyzw -> wxyz)
* Switch WaypointType to enum class, doxygen, and add ConstPtr typedefs
* Bug Fixes and Cleanup
* Add TrajOpt Array Planner
  This takes an array of points and plans through them. This could correspond to a PoseArray msg in ROS. The idea is that this planner could be used to plan one stroke in a raster path
* Add TrajOptFreespacePlanner
  This planner is intended to provide an easy to use interface to TrajOpt for freespace planning. It is made to take a start and end point and automate the generation of the TrajOpt problem. It does not expose all of the configuration available in TrajOpt, but is made in hopes of lowering the barrier to entry for using TrajOpt to solve simple problems.
* Clang formatting changes
* Added service server to tesseract environment monitor for updating the environment
* Implemented move functionality for attached bodies
* Merge pull request `#69 <https://github.com/tesseract-robotics/tesseract/issues/69>`_ from mpowelson/acm_test
  Add test to benchmark ACM isCollisionAllowed
* Add test to benchmark ACM isCollisionAllowed
* Merge pull request `#59 <https://github.com/tesseract-robotics/tesseract/issues/59>`_ from arocchi/acm_fixes
  ACM  improvements: serialization and tests
* Fixed tesseract_ros::getActiveLinkNamesRecursive; added test
* Added missing main() in ros_tesseract_utils_unit.cpp
* Revert "fixed kdl_env_unit.cpp tests by using setActiveCollisionObjects() for continuous contact manager"
  This reverts commit c84bdc55d209a5a660c212c5d1a803f32d5390f6.
  This change should have been rendered obsolete by `#64 <https://github.com/tesseract-robotics/tesseract/issues/64>`_
* Merge branch 'kinetic-devel' into acm_fixes
* Update contact managers active links when environment changes
* Add active link unit test
* fixed kdl_env_unit.cpp tests by using setActiveCollisionObjects() for continuous contact manager
* Fixed tests using discrete collision manager in kdl_env_unit.cpp
* Added installation of tesseract_monitoring launch files to CMakeLists
* Update to work with ReadTheDocs
* Add tesseract_collision documentation
* Fix clang formating
* Add clang-format AlignAfterOpenBracket: Align
* Export HACD library from bullet
* Fixed typo in documentation for AllowedCollisionEntry.msg
* Added AllowedCollisionEntry.msg in tesseract_msgs
* Fixes to run_clang_format_check
* Added unit tests for ros_tesseract_utils and KDLEnv
* Added test for AllowedCollisionMatrix in tesseract_core
* TesseractState includes information on allowed collisions, and ros_tesseract_utils are able to use them to correctly serialize and deserialize tesseract_ros::ROSBasicEnv from TesseractState messages
* Changed the AllowedCollisionMatrix lookup table to be indexed by std::pair<std::string, std::string>
* Fix formatting using clang
* Update travis ci to include clang-format test
* Fix warnings in unit tests
* Update due to changes in FCL Convex Shape Constructor
* Add additional compiler warning options
* Updated bullet_ros to not build unit tests; added line for installation of plugin XML files in Rviz package
* Implement topic subscriber for updating collision monitor environment
* Implement synchronous "compute_contact_reports" service in contact_monitor.cpp
* Eigen Alignment fixes
* Update due to change in trajopt
* Clean up TrajOptPlannerConfig doxygen
* Remove allocator for std::vector<Eigen::Vector3d>
* Fix axis plotting and add util for JointStateMsg
* Add monitoring of joint state topic to tesseract state display
* Simplify trajopt planner
* Ignore unused param warnings in bullet
* Add EIGEN_MAKE_ALIGNED_OPERATOR_NEW macros
* Fixed typo 'constacts' in ContactResultVector.msg
* Update bullet3 submodule to master
* Disable tesseract_collision FCL ConvexHull tests
* Fix/Clean depends in CMakeLists.txt and package.xml for travis-ci
* Create .travis.yml
* Fix trajopt planner collision check
* Merge pull request `#13 <https://github.com/tesseract-robotics/tesseract/issues/13>`_ from mpowelson/trajopt_planning_fixes
  Got TrajoptPlanner in working order
* Trajopt Planner: Pass prob by ref
* Trajopt planning changes: Add docs and rearrange parameters
* Add callbacks/parameters to trajopt planner
* Minor Fix
* Clang Format
* Got TrajoptPlanner in working order
* Merge pull request `#41 <https://github.com/tesseract-robotics/tesseract/issues/41>`_ from Levi-Armstrong/issue/FixMultiLayerCompoundShape
  Fix use of multi layer compound shape
  Fix/add cmake install commands
* Fix cmake install commands
* Fix use of multi layer compound shape
* Merge pull request `#40 <https://github.com/tesseract-robotics/tesseract/issues/40>`_ from Levi-Armstrong/feature/RemoveContactRequestStruct
  Refractor out ContactRequest type
* Refractor out ContactRequest type
* Merge pull request `#38 <https://github.com/tesseract-robotics/tesseract/issues/38>`_ from Jmeyer1292/melodic-fixes
  On Melodic, Fix Issue Where OMPL Package Is Not Found
* Merge pull request `#39 <https://github.com/tesseract-robotics/tesseract/issues/39>`_ from Levi-Armstrong/issue/FixBulletCast
  This fixes the continuous collision checking
* Fix the use of ContactRequestType::FIRST with broadphase
* Fix cast bvh manager
* Fix bullet continous collision checking
* Find the lower-case ompl package. Fixes an issue where OMPL was not found my CMAKE. Note that Moveit made this transition in kinetic.
* Merge pull request `#34 <https://github.com/tesseract-robotics/tesseract/issues/34>`_ from Levi-Armstrong/issue/FixBulletCast
  * This fixes the bullet cast simple manager
  * Fix the plotting of frames
  * Add unit test when using change base in kdl kin
  * Remove bullet build flags.
  * When adding use double precision this causes trajopt_ros test to fail. I believe this is due to inaccuracies in the EPA algorithm.
* Remove bullet build flags
  This for some reason causes TrajOpt to fail most likely due to bad results from the EPA algorithm
* Add unit test for using change base in kdl kin
* Fix the plotting of frames
* Merge pull request `#37 <https://github.com/tesseract-robotics/tesseract/issues/37>`_ from arocchi/fix_36
  Changed destruction order of plugin loader and instantiation of loaded class
* Changed destruction order of plugin loader and instantiation of loaded class
* Fix compound and children aabb when updating cast transform
* Fix bullet cast simple manager
* Restructure bullet managers to be in separate files
* Merge pull request `#32 <https://github.com/tesseract-robotics/tesseract/issues/32>`_ from Levi-Armstrong/issue/testCollisionClone
  Add unit test for clone method and fix mesh to mesh unit test names
* Fix kdl jacobian calculation and add unit tests
* Add unit test for clone method and fix mesh to mesh unit test names
* Merge pull request `#33 <https://github.com/tesseract-robotics/tesseract/issues/33>`_ from Levi-Armstrong/issue/fixPluginDescription
  Fix namespace in plugin description
* Fix namespace in plugin description
* Merge pull request `#29 <https://github.com/tesseract-robotics/tesseract/issues/29>`_ from Levi-Armstrong/issue/addCollisionNamespaces
  Add namespaces specific to collision implementation
* Fix lambda functions
* Add Bullet detailed mesh to detailed mesh collision checking along with unit test
* Adjust test to run for both primitive and convex shape.
* Add namespaces specific to collision implementation
* Merge pull request `#31 <https://github.com/tesseract-robotics/tesseract/issues/31>`_ from Jmeyer1292/fixup/planning
  OMPL Planner Fixups
* Fixups required to use the OMPL planner effectively with other ROS systems. This includes a) a change to compute the transforms and set the transforms of the whole scene, and b) a fix to the CMakeLists to export the right libraries.
* Merge pull request `#26 <https://github.com/tesseract-robotics/tesseract/issues/26>`_ from Levi-Armstrong/issue/FixContactMonitor
  Update contact monitor to use the latest version
* Merge pull request `#28 <https://github.com/tesseract-robotics/tesseract/issues/28>`_ from Jmeyer1292/fix/bullet_include
  Bullet Convex Hull Computer Include
* Corrected include file path to work with the bullet3_ros package include paths
* Fix the contact monitor to use the new contact managers
* Fix asserts in CollisionObjectWrapper for bullet and fcl
* Merge pull request `#23 <https://github.com/tesseract-robotics/tesseract/issues/23>`_ from Levi-Armstrong/feature/addFCLNew
  Add fcl discrete collision manager
* Make requested changes and fixes
* Add ros node for creating convex hull meshes
* Add fcl convex hull support and update tests
* Fix bullet cast assert in setCollisionObjectsTransform
* Add FCL discrete manager
* Fix comments from affine to isometry
* Merge pull request `#20 <https://github.com/tesseract-robotics/tesseract/issues/20>`_ from Levi-Armstrong/feature/Isometry3d
  switch from using affine3d to isometry3d
* Add FCL and libccd submodules
* Add virtual destructors to base classes
* Add large octomap collision unit test enable aabb tree for compound shapes
* Update kdl chain kin unit to Isometry
* Convert to use templated using for vectors and maps of eigen types
* switch from using affine3d to isometry3d
* Merge pull request `#18 <https://github.com/tesseract-robotics/tesseract/issues/18>`_ from Jmeyer1292/fix/add_manip
  addManipulator() checks for failure
* addManipulator() now ensures that the group could be created prior to returning success.
* Merge pull request `#17 <https://github.com/tesseract-robotics/tesseract/issues/17>`_ from Levi-Armstrong/fixIssue7
  Fix issue `#7 <https://github.com/tesseract-robotics/tesseract/issues/7>`_ with test
* Fix issue `#7 <https://github.com/tesseract-robotics/tesseract/issues/7>`_ with test
* Merge pull request `#15 <https://github.com/tesseract-robotics/tesseract/issues/15>`_ from Levi-Armstrong/feature/largeDataSetTest
  Restructure Collision Checking for Performance Improvements
* Update .clang-format comment
* Run clang-format
* Restructure Collision Checking for Performance Improvements
* Merge pull request `#16 <https://github.com/tesseract-robotics/tesseract/issues/16>`_ from Levi-Armstrong/issue14
  Remove stray node handle
* Remove stray node handle
* Merge pull request `#5 <https://github.com/tesseract-robotics/tesseract/issues/5>`_ from ros-industrial-consortium/Levi-Armstrong-patch-1
  Update README.md
* Update README.md
* Merge pull request `#3 <https://github.com/tesseract-robotics/tesseract/issues/3>`_ from Levi-Armstrong/FixGHPages
  fix gh_pages config file
* fix gh_pages config file
* Merge pull request `#2 <https://github.com/tesseract-robotics/tesseract/issues/2>`_ from Levi-Armstrong/addGHPages
  Add gh_pages for sphinx documentation
* Add gh_pages for sphinx documentation
* Merge pull request `#1 <https://github.com/tesseract-robotics/tesseract/issues/1>`_ from Levi-Armstrong/fixSubmodule
  Fix submodule for bullet3
* Fix submodule for bullet3
* Move tesseract into its own repository
* Merge pull request `#12 <https://github.com/tesseract-robotics/tesseract/issues/12>`_ from larmstrong/clangFormat
  clang format code, use Eigen::Ref and add kdl_joint_kin
* Add kdl_joint_kin to handle auxillary axes
* Fix kdl_chain_kin to handle links not in chain
* Add override to headers and clang format
* Make use of Eigen::Ref
* clang format code
* Merge pull request `#11 <https://github.com/tesseract-robotics/tesseract/issues/11>`_ from larmstrong/unusedParamWarn
  Fix remaining warning
* Uncomment unused names in headers
* Add ability to check if two attachable objects and bodies are the same
* rename package tesseract_ros_planning to tesseract_planning
* Fix tesseract_monitoring package
* Add trajectory norm calculation to glass_up_right_plan.cpp
* Uncomment node tag in trajopt_examples launch files
* Fix planning_unit.cpp test
* Revert change in kdl_chain_kin.cpp
* Fix remaining warning
* Merge pull request `#10 <https://github.com/tesseract-robotics/tesseract/issues/10>`_ from larmstrong/mergeJMeyer
  Merge jmeyer pull requests
* Merge pull request `#9 <https://github.com/tesseract-robotics/tesseract/issues/9>`_ from larmstrong/removeOpenRave
  Merge removeOpenRave branch
* Fix failed rebase
* Remove append operation from AttachableObject
* Added continuous motion validator. We'll probably want a better API for this.
* Added some docs
* Implemented a basic interface to the OMPL SimpleSetup class for use with Tesseract
* Remove append operation from AttachableObject
* Removed warnings again. Just too many in included libraries to deal with.
* Gobs more small fixups. I don't believe I changed anything that would affect actual logic.
* Removed use of deprecated JSON_CPP function calls
* Cleaning up warnings
* Set the transform for the attached body in the glass upright example
* Fix contact monitoring
* Fix contact plugin loader crashing
* Merge branch 'removeOpenRave' of https://raesgit.datasys.swri.edu/larmstrong/trajopt_ros into removeOpenRave
* Intialize AttachedBodyInfo transform to identity
* Rename isCollisionAllowed to isContactAllowed
* Merge pull request `#4 <https://github.com/tesseract-robotics/tesseract/issues/4>`_ from Jmeyer/fix/no_include
  Trajopt Tools Include Directory
* Removed include directory references to an include directory that doesn't exist
* Create custom rviz environment plugin
* Add Car Seat Example
* Add a few bullet3 build options
* Fix glass_up_right_plan, need to add collision obj type
* Clean up trajopt_ext CMakeLists.txt file
* Add branch to submodule
* Add submodule to bullet3
* Remove copy of bullet
* Add ability to define collision object type
* Remove SHAPE_EXPANSION from bullet
* Disable moveit related packages
* Refractor collision checking into its own package
* Add tesseract_ros_monitor with environment monitor
* Switch boost::function to std::function
* Switch boost::shared_ptr to std::shared_ptr
* Fix getActiveLinkNamesRecursive method
* Add method getActiveLinkNames to env base class
* Add parameters to contact_monitoring node
* Add package tesseract_ros_planning package
* Add basic environment singleton and contact monitoring node
* Add missing license information
* Add getJointValues() and getLinkTransforms() to basic_env.h
* Rename DistanceRequest DistanceResults to ContactRequest ContactResults
* Separate Plotting from environment and fix object color typedef
* In examples add node to launch file
* Add tesseract packages
* replace std::map with std::unordered_map
* Make AllowedCollisionMatrix a class
* replace trajopt_scene with tesseract package
* Add ability to set safety margin for link pairs
* Move data directory content to trajopt_test_support/config directory
* Remove const from std::map key
* Add ability to visualize trajopt_scene using robot state
* Move moveit items to its own package and create trajopt_scene package
* Remove moveit depend from ros_kin_chain
* Add system depend to CMakeLists.txt
* Fix bug in collision_common.h
* Add ability to get global minimum for pair instead of just all
* Tune optimization parameters for puzzle piece example
* Move the plotWaitForInput to the plot callback function
* Rename ROSKin to ROSKinChain and add JointAccCost JointJerkCost
* Rename getManipulatorKin to getManipulator
* Add alternative continuousCollisionCheckTrajectory function
* enable collision for puzzle piece example
* Integrate changes to moveit collision
* Add trajopt_tools package with hacd and vhacd
* Add vhacd to trajopt_ext
* Add puzzle piece example
* Add tcp capability to kinematics_terms
* Remove debug code from glass_up_right_plan.cpp
* Update the iiwa dae to be shadeless
* Switch examples to use kuka iiwa copied into this pacakge
* Fix commented out plotting calls
* Add ability to publish axes
* Remove additional refferences to openrave
* Make distance and collision calls const and fix ROS_INFO warnings
* Add glass up right example
* Expose optimization parameters to user via cpp and json
* Remove the use of global ProblemConstructionInfo variable when parsing json data
* Add trajopt_examples package with one cartesian example
* Remove old json unit tests
* Remove old test collision-checker-unit
* Restructure trajopt_ext directory
* Remove local version of jsoncpp
* Remove pr2 moveit_cofig package
* Add octomap unit test and fix convert bullet convertBulletCollisions
* Add test for objects attached to links without geometry
* Fix bullet collision to handle attached object connected to links without geometry
* Fix use of attached collision objects and add a unit test for it
* Make use of BULLET_DEFAULT_CONTACT_DISTANCE
* Implement remaining collision_robot bullet methods
* Add attached object functionality
* Add collision world test and make use of xacros
* Integrate collision world
* Update isCollision allowed to handle Attached objects
* Change link2cow typedef
* Remove temp file
* Add/Update cast cost unit test
* Remove openrave utils
* Remove unused file pr2.gv and pr2.pdf
* Remove osgviewer package
* Switch planning unit test to use ROS_DEBUG
* Fix continuous collision checking and add original cast method
* Add Continuous Collision Checking and Filter Masking
* Add plotting parameter to trajopt_planning_unit
* MoveIt Bullet Collision Checker (Single State)
* Second pass at planning-unit test
* First pass at planning-unit test
* Working numerical ik test
* Fixup
* Add test support package and moveit config package
* Divide package into multiple packages
* Clean up CMakeLists.txt file
* Get tests working
* Minor fixes for finding openrave and openscenegraph
* TrajOpt Successful Build
* initial commit with sco utils and json building
* Initial commit
* Contributors: Alessio Rocchi, Andrew Price, Armstrong, Levi H, Ben Berman, Chris Lewis, Colin Lewis, DavidMerzJr, Hervé Audren, John Wason, Jonathan Meyer, Joseph Schornak, Levi, Levi Armstrong, Marco Bassa, Matthew Powelson, Michael Ripperger, Patrick Beeson, Reid Christopher, Roger Pi, Tyler Marr, Unknown, dmerz, jrgnicho, marrts, mpowelson, mripperger, rchristopher
