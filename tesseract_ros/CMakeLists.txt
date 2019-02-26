cmake_minimum_required(VERSION 2.8.3)
project(tesseract_ros)

add_compile_options(-std=c++11 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  tesseract_core
  tesseract_msgs
  tesseract_collision
  roscpp
  urdf
  srdfdom
  geometric_shapes
  kdl_parser
  eigen_conversions
  pluginlib
)

find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS system thread REQUIRED)
find_package(octomap REQUIRED)
find_package(orocos_kdl REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
    ${EIGEN3_INCLUDE_DIRS}
  LIBRARIES
    ${PROJECT_NAME}_kdl
    ${OCTOMAP_LIBRARIES}
    ${orocos_kdl_LIBRARIES}
  CATKIN_DEPENDS
    tesseract_core
    tesseract_msgs
    tesseract_collision
    roscpp
    urdf
    srdfdom
    geometric_shapes
    kdl_parser
    eigen_conversions
    pluginlib
  DEPENDS
    EIGEN3
    Boost
    OCTOMAP
    orocos_kdl
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  SYSTEM ${EIGEN3_INCLUDE_DIRS}
  SYSTEM ${Boost_INCLUDE_DIRS}
  SYSTEM ${OCTOMAP_INCLUDE_DIRS}
  SYSTEM ${orocos_kdl_INCLUDE_DIRS}
)

add_library(tesseract_collision_bullet_plugin src/plugins/bullet_plugin.cpp)
target_link_libraries(tesseract_collision_bullet_plugin ${catkin_LIBRARIES})
target_compile_options(tesseract_collision_bullet_plugin PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)

add_library(tesseract_collision_fcl_plugin src/plugins/fcl_plugin.cpp)
target_link_libraries(tesseract_collision_fcl_plugin ${catkin_LIBRARIES})
target_compile_options(tesseract_collision_fcl_plugin PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)

# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}_kdl tesseract_collision_bullet_plugin tesseract_collision_fcl_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
 )

install(FILES tesseract_collision_bullet_plugin_description.xml tesseract_collision_fcl_plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
