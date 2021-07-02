^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tesseract_visualization
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.0 (2021-07-02)
------------------
* fix error so that initial state has dt=0 (`#604 <https://github.com/ros-industrial-consortium/tesseract/issues/604>`_)
* Change tesseract_visualization cmake message type to STATUS
* Contributors: Levi Armstrong, cbw36

0.4.1 (2021-04-24)
------------------

0.4.0 (2021-04-23)
------------------
* Fix issue in trajectory_player calling size if trajectory does not exist
* Contributors: Levi Armstrong

0.3.1 (2021-04-14)
------------------
* Move tesseract_variables() before any use of custom macros
* Contributors: Levi Armstrong

0.3.0 (2021-04-09)
------------------
* Only enable code coverage if compiler definition is set
* Fix issue in trajectory player setCurrentDuration not handling finished bool
* Move serialize implementation to cpp based on boost documentation for shared libraries
* Rename Any method cast() and cast_const() to as()
* Split loading plugins into two classes ClassLoader and PluginLoader
* Remove dependency on class_loader and leverage Boost DLL
* Add cmake format
* Add support for defining collision margin data in SRDF (`#573 <https://github.com/ros-industrial-consortium/tesseract/issues/573>`_)
* Use boost targets, add cpack and license file (`#572 <https://github.com/ros-industrial-consortium/tesseract/issues/572>`_)
* Fix the way in which Eigen is included (`#570 <https://github.com/ros-industrial-consortium/tesseract/issues/570>`_)
* Add scale to tool path marker
* Modify trajectory interpolator to visualize trajectories w/o time
* Start to adding boost serialization support
* Contributors: Hervé Audren, Levi Armstrong, Matthew Powelson

0.2.0 (2021-02-17)
------------------
* Update cmake_common_scripts to ros_industrial_cmake_boilerplate
* Improve code coverage for tesseract_kinematics core
* Add SWIG shared_ptr commands to visualization markers
* Add marker support and remove dependency on command language
* Move all directories in tesseract directory up one level
* Contributors: John Wason, Levi Armstrong

0.1.0 (2020-12-31)
------------------
