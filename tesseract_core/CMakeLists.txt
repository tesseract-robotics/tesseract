cmake_minimum_required(VERSION 2.8.3)
project(tesseract_core)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(
  -std=c++11
  -Wall
  -Wextra
  -Wsuggest-override
  -Wconversion
  -Wsign-conversion)
#-Wunused -Wunreachable-code -Wno-write-strings -pedantic
#-Wfloat-equal

find_package(catkin REQUIRED geometric_shapes)

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
    ${EIGEN3_INCLUDE_DIRS}
  #LIBRARIES
  CATKIN_DEPENDS
    geometric_shapes
  DEPENDS
    EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  SYSTEM ${EIGEN3_INCLUDE_DIRS}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
