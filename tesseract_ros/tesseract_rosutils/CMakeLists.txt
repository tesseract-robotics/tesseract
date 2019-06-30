cmake_minimum_required(VERSION 3.5.0)
project(tesseract_rosutils)

find_package(catkin REQUIRED COMPONENTS
  tesseract_msgs
  roscpp
  eigen_conversions
)

find_package(Eigen3 REQUIRED)
find_package(tesseract_scene_graph REQUIRED)
find_package(tesseract_geometry REQUIRED) # This should not be required, must be doing something wrong when creating targets
find_package(tesseract_visualization REQUIRED)
find_package(tesseract_collision REQUIRED)
find_package(tesseract_common REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
    ${EIGEN3_INCLUDE_DIRS}
#  LIBRARIES
  CATKIN_DEPENDS
    tesseract_msgs
    roscpp
    eigen_conversions
    roslib
  DEPENDS
    EIGEN3
    tesseract_scene_graph
    tesseract_geometry
    tesseract_visualization
    tesseract_collision
    tesseract_common
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
 )
