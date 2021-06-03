##########
Unit Tests
##########

Building and Running Unit Tests
===============================

Build and Run All
-----------------

Run the following command: ::

    catkin build --force-cmake -DENABLE_TESTS=ON

Build and Run One Package
-------------------------

Run the following command: ::

    catkin build pkg_name --force-cmake -DENABLE_TESTS=ON

Make Output Verbose
-------------------

Run the following command: ::

    catkin build pkg_name --v --force-cmake -DENABLE_TESTS=ON

Building with Clang-Tidy
========================

.. note:: Clang-tidy is automatically enabled if cmake argument -DTESSERACT_ENABLE_TESTING_ALL=ON is passed.

To build with clang-tidy, you must pass the -DTESSERACT_ENABLE_CLANG_TIDY=ON to cmake when building: ::

    catkin build --force-cmake -DTESSERACT_ENABLE_CLANG_TIDY=ON

