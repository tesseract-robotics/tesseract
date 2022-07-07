^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tesseract_state_solver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* Add methods for getting link transform information from state solver
* Contributors: Levi Armstrong

0.8.4 (2022-03-03)
------------------

0.8.3 (2022-02-22)
------------------
* Python patches for Feb 2022 update (`#716 <https://github.com/tesseract-robotics/tesseract/issues/716>`_)
* Add missing UPtr and ConstUPtr typedef for KDL and OFKT state solver (`#711 <https://github.com/tesseract-robotics/tesseract/issues/711>`_)
* Contributors: John Wason, Levi Armstrong

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
* Fix spelling errors
* Contributors: Levi Armstrong

0.6.7 (2021-11-16)
------------------
* Fix thread safety issue in kdl state solver
* Contributors: Levi Armstrong

0.6.6 (2021-11-10)
------------------

0.6.5 (2021-11-04)
------------------

0.6.4 (2021-10-29)
------------------

0.6.3 (2021-10-28)
------------------

0.6.2 (2021-10-22)
------------------

0.6.1 (2021-10-19)
------------------
* Bump version of CMake boilerplate tools (`#645 <https://github.com/ros-industrial-consortium/tesseract/issues/645>`_)
  * Bump version of CMake boilerplate tools
  * Remove addition of non-existent benchmark directory
* Contributors: Michael Ripperger

0.6.0 (2021-10-15)
------------------
* Add isActiveLinkName and hasLinkName to state solver interface
* Fix CI builds and code coverage
* CMake Format
* Switch to using .clang-tidy file and fix clang-tidy errors
* Update OFKT state solver to allow replace and move
* Update state solver interface to get active and static links and active joints
* Move resource locator from tesseract_scene_graph to tesseract_common
* Add jacobian calculation to OFKT State solver
* Update state solver interface to not return references
* Update state solver to include jacobian and active links
* Move state solver into its own package with unit tests
* Contributors: Levi-Armstrong

0.5.0 (2021-07-02)
------------------

0.4.1 (2021-04-24)
------------------

0.4.0 (2021-04-23)
------------------

0.3.1 (2021-04-14)
------------------

0.3.0 (2021-04-09)
------------------

0.2.0 (2021-02-17)
------------------

0.1.0 (2020-12-31)
------------------
