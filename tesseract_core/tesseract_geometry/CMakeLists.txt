cmake_minimum_required(VERSION 2.8.3)
project(tesseract_geometry)

add_compile_options(
  -std=c++11
  -Wall
  -Wextra
  -Wsuggest-override
  -Wconversion
  -Wsign-conversion)

find_package(catkin REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(console_bridge REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${console_bridge_INCLUDE_DIRS}
  LIBRARIES
    ${catkin_LIBRARIES}
    ${console_bridge_LIBRARIES}

  #CATKIN_DEPENDS

  DEPENDS
    EIGEN3
    console_bridge
)

include_directories(
  include
  SYSTEM ${EIGEN3_INCLUDE_DIRS}
  SYSTEM ${console_bridge_INCLUDE_DIRS}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
