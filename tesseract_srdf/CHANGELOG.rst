^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tesseract_srdf
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Remove TesseractSupportResourceLocator
* Fix serialization
* Contributors: Levi Armstrong

0.25.0 (2024-09-28)
-------------------

0.24.1 (2024-08-19)
-------------------

0.24.0 (2024-08-14)
-------------------

0.23.1 (2024-07-28)
-------------------
* Cleanup boost serialization
* Contributors: Levi Armstrong

0.23.0 (2024-07-24)
-------------------

0.22.2 (2024-06-10)
-------------------

0.22.1 (2024-06-03)
-------------------

0.22.0 (2024-06-02)
-------------------
* Add support for jerk limits
* Leverage forward declarations to improve compile times (`#990 <https://github.com/tesseract-robotics/tesseract/issues/990>`_)
* Contributors: Levi Armstrong

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

0.19.1 (2023-09-05)
-------------------

0.19.0 (2023-09-05)
-------------------
* Update kinematics and collision packages to leverage cmake components (`#927 <https://github.com/tesseract-robotics/tesseract/issues/927>`_)
* Update emails
* Contributors: Levi Armstrong

0.18.1 (2023-06-30)
-------------------

0.18.0 (2023-06-29)
-------------------
* Add package cmake flags for testing, examples and benchmarks
* Contributors: Levi Armstrong

0.17.0 (2023-06-06)
-------------------

0.16.3 (2023-05-04)
-------------------

0.16.2 (2023-04-28)
-------------------

0.16.1 (2023-04-11)
-------------------

0.16.0 (2023-04-09)
-------------------

0.15.3 (2023-03-22)
-------------------

0.15.2 (2023-03-15)
-------------------

0.15.1 (2023-03-14)
-------------------

0.15.0 (2023-03-03)
-------------------
* Performance improvements found using callgrind (`#852 <https://github.com/tesseract-robotics/tesseract/issues/852>`_)
* Improve tesseract_srdf code coverage (`#836 <https://github.com/tesseract-robotics/tesseract/issues/836>`_)
* Fix KinematicsInformation equal and insert methods
* Contributors: Levi Armstrong

0.14.0 (2022-10-23)
-------------------
* Order srdf save so plugins are above the acm
* Contributors: Levi Armstrong

0.13.1 (2022-08-25)
-------------------
* Move boost serialization export outside tesseract_common namespace in srdf_model.cpp
* Move most SWIG commands to tesseract_python package (`#809 <https://github.com/tesseract-robotics/tesseract/issues/809>`_)
* Contributors: John Wason, Levi Armstrong

0.13.0 (2022-07-11)
-------------------
* Update code based on clang-tidy-14
* Contributors: Levi Armstrong

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
* cmake format
* Add TESSERACT_ENABLE_EXAMPLES compile option
* Contributors: John Wason

0.8.3 (2022-02-22)
------------------
* Python patches for Feb 2022 update (`#716 <https://github.com/tesseract-robotics/tesseract/issues/716>`_)
* Sort ACM alphabetically when writing
* Contributors: John Wason, Matthew Powelson

0.8.2 (2022-01-27)
------------------
* Add ability to provide calibration information in the SRDF (`#703 <https://github.com/tesseract-robotics/tesseract/issues/703>`_)
  * Add missing package tesseract_srdf in CI after script
  * Add support for calibration info in SRDF
* Contributors: Levi Armstrong

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
* Fix SRDF REP and ROP positioner joint parsing
* Contributors: Levi Armstrong

0.4.1 (2021-04-24)
------------------

0.4.0 (2021-04-23)
------------------
* Fix test names in tesseract_srdf
* Fix spelling in tesseract_srdf package.xml
* Update tesseract_srdf to leverage nested exceptions
* Move srdf code to its own package tesseract_srdf
* Contributors: Levi Armstrong
