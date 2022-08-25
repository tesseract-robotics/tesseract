^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tesseract_visualization
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.13.1 (2022-08-25)
-------------------
* Move most SWIG commands to tesseract_python package (`#809 <https://github.com/tesseract-robotics/tesseract/issues/809>`_)
* Contributors: John Wason

0.13.0 (2022-07-11)
-------------------
* Update code based on clang-tidy-14
* Contributors: Levi Armstrong

0.10.0 (2022-07-06)
-------------------
* Update ros_industrial_cmake_boilerplate to 0.3.0 (`#795 <https://github.com/tesseract-robotics/tesseract/issues/795>`_)
* Fix tesseract_ignition_visualization anchor
* Static plugin loading using symbol module resolution (`#782 <https://github.com/tesseract-robotics/tesseract/issues/782>`_)

0.9.11 (2022-06-30)
-------------------
* Updated CPack (`#786 <https://github.com/tesseract-robotics/tesseract/issues/786>`_)
* Add missing gtest test_depends in tesseract_visualization
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

0.9.3 (2022-04-18)
------------------
* Updated plugin capability to support sections (`#741 <https://github.com/tesseract-robotics/tesseract/issues/741>`_)
* Contributors: Levi Armstrong

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

0.8.4 (2022-03-03)
------------------

0.8.3 (2022-02-22)
------------------

0.8.2 (2022-01-27)
------------------

0.8.1 (2022-01-24)
------------------

0.8.0 (2022-01-19)
------------------

0.7.5 (2022-01-10)
------------------

0.7.4 (2021-12-15)
------------------

0.7.3 (2021-12-15)
------------------

0.7.2 (2021-12-15)
------------------

0.7.1 (2021-12-15)
------------------

0.7.0 (2021-12-04)
------------------

0.6.9 (2021-11-29)
------------------

0.6.8 (2021-11-29)
------------------

0.6.7 (2021-11-16)
------------------

0.6.6 (2021-11-10)
------------------

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
