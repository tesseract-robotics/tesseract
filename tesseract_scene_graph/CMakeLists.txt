cmake_minimum_required(VERSION 2.8.3)
project(tesseract_scene_graph)

add_compile_options(
  -std=c++11
  -Wall
  -Wextra
  -Wsuggest-override
  -Wconversion
  -Wsign-conversion)

find_package(catkin REQUIRED COMPONENTS tesseract_geometry)

set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(urdfdom REQUIRED)
find_package(urdfdom_headers REQUIRED)
find_package(console_bridge REQUIRED)
find_package(orocos_kdl REQUIRED)

find_package(ASSIMP QUIET)
if (NOT ASSIMP_FOUND)
  find_package(PkgConfig REQUIRED)
  # assimp is required, so REQUIRE the second attempt
  pkg_check_modules(ASSIMP REQUIRED assimp)
endif()

catkin_package(
  INCLUDE_DIRS
    include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${urdfdom_INCLUDE_DIRS}
    ${urdfdom_headers_INCLUDE_DIRS}
    ${console_bridge_INCLUDE_DIRS}
    ${orocos_kdl_INCLUDE_DIRS}
  LIBRARIES
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${urdfdom_LIBRARIES}
    ${urdfdom_headers_LIBRARIES}
    ${console_bridge_LIBRARIES}
    ${ASSIMP_LIBRARIES}
    ${orocos_kdl_LIBRARIES}
  CATKIN_DEPENDS
    tesseract_geometry
  DEPENDS
    EIGEN3
    urdfdom
    urdfdom_headers
    console_bridge
    ASSIMP
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  SYSTEM ${EIGEN3_INCLUDE_DIRS}
  SYSTEM ${Boost_INCLUDE_DIRS}
  SYSTEM ${urdfdom_INCLUDE_DIRS}
  SYSTEM ${urdfdom_headers_INCLUDE_DIRS}
  SYSTEM ${console_bridge_INCLUDE_DIRS}
  SYSTEM ${ASSIMP_INCLUDE_DIRS}
  SYSTEM ${orocos_kdl_INCLUDE_DIRS}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

if (CATKIN_ENABLE_TESTING)

  find_package(catkin REQUIRED COMPONENTS rostest roslib)

  add_definitions(-DDATA_DIR="${CMAKE_SOURCE_DIR}/test")

  include_directories(${catkin_INCLUDE_DIRS})

  catkin_add_gtest(${PROJECT_NAME}_unit test/tesseract_scene_graph_unit.cpp)
  target_link_libraries(${PROJECT_NAME}_unit ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${ASSIMP_LIBRARIES} ${urdfdom_LIBRARIES} ${urdfdom_headers_LIBRARIES} ${orocos_kdl_LIBRARIES})
  target_compile_options(${PROJECT_NAME}_unit PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)

endif()
