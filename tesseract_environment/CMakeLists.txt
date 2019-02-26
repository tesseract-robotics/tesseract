cmake_minimum_required(VERSION 2.8.3)
project(tesseract_environment)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(
  -std=c++11
  -Wall
  -Wextra
  -Wsuggest-override
  -Wconversion
  -Wsign-conversion)

find_package(catkin REQUIRED tesseract_collision tesseract_kinematics geometric_shapes kdl_parser)

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)
find_package(urdfdom REQUIRED)
find_package(urdfdom_headers REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(console_bridge REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
    ${EIGEN3_INCLUDE_DIRS}
    ${urdfdom_INCLUDE_DIRS}
    ${urdfdom_headers_INCLUDE_DIRS}
    ${orocos_kdl_INCLUDE_DIRS}
    ${console_bridge_INCLUDE_DIRS}
  LIBRARIES
    ${PROJECT_NAME}_kdl
    ${catkin_LIBRARIES}
    ${orocos_kdl_LIBRARIES}
    ${urdfdom_LIBRARIES}
    ${urdf_LIBRARIES}
    ${urdfdom_headers_LIBRARIES}
    ${console_bridge_LIBRARIES}
  CATKIN_DEPENDS
    tesseract_collision
    tesseract_kinematics
    geometric_shapes
    kdl_parser
  DEPENDS
    EIGEN3
    urdfdom
    urdfdom_headers
    orocos_kdl
    console_bridge
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  SYSTEM ${EIGEN3_INCLUDE_DIRS}
  SYSTEM ${urdfdom_INCLUDE_DIRS}
  SYSTEM ${urdfdom_headers_INCLUDE_DIRS}
  SYSTEM ${orocos_kdl_INCLUDE_DIRS}
  SYSTEM ${console_bridge_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_kdl
  src/kdl/kdl_env.cpp
)
target_link_libraries(${PROJECT_NAME}_kdl ${catkin_LIBRARIES} ${urdfdom_LIBRARIES} ${urdfdom_headers_LIBRARIES} ${orocos_kdl_LIBRARIES} ${console_bridge_LIBRARIES})
target_compile_options(${PROJECT_NAME}_kdl PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)

# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}_kdl
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

if (CATKIN_ENABLE_TESTING)

  find_package(rostest REQUIRED)

  include_directories(${catkin_INCLUDE_DIRS})

  add_definitions(-DDATA_DIR="${CMAKE_SOURCE_DIR}/test")

  catkin_add_gtest(${PROJECT_NAME}_kdl_unit test/kdl_env_unit.cpp)
  target_link_libraries(${PROJECT_NAME}_kdl_unit ${PROJECT_NAME}_kdl ${catkin_LIBRARIES} ${urdfdom_LIBRARIES} ${urdfdom_headers_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES} ${orocos_kdl_LIBRARIES})
  target_compile_options(${PROJECT_NAME}_kdl_unit PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)

endif()
