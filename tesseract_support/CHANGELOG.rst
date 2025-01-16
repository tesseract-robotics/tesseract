^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tesseract_support
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.28.0 (2025-01-16)
-------------------

0.27.1 (2024-12-03)
-------------------

0.27.0 (2024-12-01)
-------------------

0.26.0 (2024-10-27)
-------------------
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
* Use latest vcpkg release for GitHub Action (`#1035 <https://github.com/tesseract-robotics/tesseract/issues/1035>`_)
  * Use latest vcpkg release in windows action
  * ci
  * Fix deprecated boost filesystem functions
* Contributors: John Wason, Levi Armstrong

0.23.0 (2024-07-24)
-------------------

0.22.2 (2024-06-10)
-------------------

0.22.1 (2024-06-03)
-------------------

0.22.0 (2024-06-02)
-------------------
* Leverage forward declarations to improve compile times (`#990 <https://github.com/tesseract-robotics/tesseract/issues/990>`_)
* Contributors: Levi Armstrong

0.21.5 (2023-12-14)
-------------------

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
* Fix tesseract_common plugin info implementations equal and insert methods
* Fix KinematicsInformation equal and insert methods
* Contributors: Levi Armstrong

0.14.0 (2022-10-23)
-------------------

0.13.1 (2022-08-25)
-------------------
* Fix the Windows 2019 github action (`#805 <https://github.com/tesseract-robotics/tesseract/issues/805>`_)
* Contributors: John Wason

0.13.0 (2022-07-11)
-------------------

0.10.0 (2022-07-06)
-------------------
* Update ros_industrial_cmake_boilerplate to 0.3.0 (`#795 <https://github.com/tesseract-robotics/tesseract/issues/795>`_)

0.9.11 (2022-06-30)
-------------------
* Update to use find_gtest macro
* Add missing gtest test_depend to tesseract_support
* Contributors: Levi Armstrong

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
* Add fwd_kin_plugins to abb_irb2400_plugins.yaml
* Contributors: John Wason

0.9.5 (2022-04-24)
------------------

0.9.4 (2022-04-22)
------------------

0.9.3 (2022-04-18)
------------------

0.9.2 (2022-04-03)
------------------

0.9.1 (2022-04-01)
------------------
* Generate urdf files
* Consolidate all resources into tesseract_support
* Contributors: Levi Armstrong

0.9.0 (2022-03-31)
------------------
* Make ResourceLocator serializable
* Add tcp to iiwa srdf
* Contributors: Levi Armstrong

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

0.4.1 (2021-04-24)
------------------

0.4.0 (2021-04-23)
------------------
* Fix package build depends
* Contributors: Levi Armstrong

0.3.1 (2021-04-14)
------------------

0.3.0 (2021-04-09)
------------------
* Add cmake format
* Use boost targets, add cpack and license file (`#572 <https://github.com/ros-industrial-consortium/tesseract/issues/572>`_)
* Contributors: Levi Armstrong

0.2.0 (2021-02-17)
------------------
* Add manipulator manager unit tests
* Update cmake_common_scripts to ros_industrial_cmake_boilerplate
* Move all directories in tesseract directory up one level
* Contributors: Levi Armstrong

0.1.0 (2020-12-31)
------------------
