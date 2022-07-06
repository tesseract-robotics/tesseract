cmake_minimum_required(VERSION 3.5.0)

# Extract package name and version
find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)
project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES CXX)

if(WIN32)
  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(console_bridge REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(tesseract_geometry REQUIRED)
find_package(tesseract_common REQUIRED)
find_package(tesseract_scene_graph REQUIRED)
find_package(tesseract_collision REQUIRED)

if(NOT TARGET console_bridge::console_bridge)
  add_library(console_bridge::console_bridge INTERFACE IMPORTED)
  set_target_properties(console_bridge::console_bridge PROPERTIES INTERFACE_INCLUDE_DIRECTORIES
                                                                  ${console_bridge_INCLUDE_DIRS})
  set_target_properties(console_bridge::console_bridge PROPERTIES INTERFACE_LINK_LIBRARIES ${console_bridge_LIBRARIES})
else()
  get_target_property(CHECK_INCLUDE_DIRECTORIES console_bridge::console_bridge INTERFACE_INCLUDE_DIRECTORIES)
  if(NOT ${CHECK_INCLUDE_DIRECTORIES})
    set_target_properties(console_bridge::console_bridge PROPERTIES INTERFACE_INCLUDE_DIRECTORIES
                                                                    ${console_bridge_INCLUDE_DIRS})
  endif()
endif()

# Load variable for clang tidy args, compiler options and cxx version
tesseract_variables()

initialize_code_coverage(ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
set(COVERAGE_EXCLUDE
    /usr/*
    /opt/*
    ${CMAKE_CURRENT_LIST_DIR}/test/*
    /*/gtest/*
    /*/bullet/LinearMath/*)
add_code_coverage_all_targets(EXCLUDE ${COVERAGE_EXCLUDE} ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})

add_library(
  ${PROJECT_NAME}
  src/box.cpp
  src/calibration.cpp
  src/capsule.cpp
  src/collision.cpp
  src/cone.cpp
  src/convex_mesh.cpp
  src/cylinder.cpp
  src/dynamics.cpp
  src/geometry.cpp
  src/inertial.cpp
  src/joint.cpp
  src/limits.cpp
  src/link.cpp
  src/material.cpp
  src/mesh.cpp
  src/mimic.cpp
  src/octomap.cpp
  src/octree.cpp
  src/origin.cpp
  src/point_cloud.cpp
  src/safety_controller.cpp
  src/sdf_mesh.cpp
  src/sphere.cpp
  src/urdf_parser.cpp
  src/visual.cpp
  src/utils.cpp)
target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC Boost::boost
         Eigen3::Eigen
         tesseract::tesseract_common
         tesseract::tesseract_geometry
         tesseract::tesseract_scene_graph
         tesseract::tesseract_collision_bullet
         console_bridge::console_bridge)
target_link_libraries(${PROJECT_NAME} PRIVATE Boost::boost console_bridge::console_bridge)
target_compile_options(${PROJECT_NAME} PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(${PROJECT_NAME} PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME} PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME} PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})

# PCL does not support c++11 on Xenial so cannot include point cloud parsing from urdf Boost version number is in XYYYZZ
# format such that: (BOOST_VERSION % 100) is the sub-minor version ((BOOST_VERSION / 100) % 1000) is the minor version
# (BOOST_VERSION_MACRO / 100000) is the major version.
if(Boost_VERSION_MACRO)
  set(TESSERACT_Boost_VERSION_MACRO ${Boost_VERSION_MACRO})
else(TESSERACT_Boost_VERSION_MACRO ${Boost_VERSION_MACRO})
  set(TESSERACT_Boost_VERSION_MACRO ${Boost_VERSION})
endif()
if(TESSERACT_Boost_VERSION_MACRO VERSION_GREATER "106000")
  target_link_libraries(${PROJECT_NAME} PUBLIC ${PCL_IO_LIBRARIES} ${PCL_COMMON_LIBRARIES})
  target_compile_definitions(${PROJECT_NAME} PUBLIC TESSERACT_PARSE_POINT_CLOUDS="ON")
  foreach(DEF ${PCL_DEFINITIONS})
    string(STRIP ${DEF} DEF)
    if(NOT
       "${DEF}"
       STREQUAL
       "")
      string(
        SUBSTRING "${DEF}"
                  0
                  2
                  DEF_PREFIX)
      if("${DEF_PREFIX}" STREQUAL "-m")
        string(
          REPLACE " "
                  ";"
                  DEF
                  ${DEF})
        foreach(OPTION_DEF ${DEF})
          target_compile_options(${PROJECT_NAME} PUBLIC ${OPTION_DEF})
        endforeach()
      else()
        target_compile_definitions(${PROJECT_NAME} PUBLIC ${DEF})
      endif()
    endif()
  endforeach()
  if(NOT TARGET pcl_common)
    target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${PCL_INCLUDE_DIRS})
  endif()
endif()

target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                  "$<INSTALL_INTERFACE:include>")

configure_package(NAMESPACE tesseract TARGETS ${PROJECT_NAME})

# Mark cpp header files for installation
install(FILES include/${PROJECT_NAME}/urdf_parser.h DESTINATION include/${PROJECT_NAME})
install(FILES include/${PROJECT_NAME}/utils.h DESTINATION include/${PROJECT_NAME})

# ADD Examples
if(TESSERACT_ENABLE_EXAMPLES)
  add_subdirectory(examples)
endif()

if(TESSERACT_ENABLE_TESTING)
  enable_testing()
  add_run_tests_target(ENABLE ${TESSERACT_ENABLE_RUN_TESTING})
  add_subdirectory(test)
endif()

if(TESSERACT_PACKAGE)
  tesseract_cpack(
    VERSION ${pkg_extracted_version}
    MAINTAINER <https://github.com/ros-industrial-consortium/tesseract>
    DESCRIPTION ${pkg_extracted_description}
    LICENSE_FILE ${CMAKE_CURRENT_LIST_DIR}/../LICENSE
    README_FILE ${CMAKE_CURRENT_LIST_DIR}/../README.md
    LINUX_DEPENDS
      "libconsole-bridge-dev"
      "libeigen3-dev"
      "libpcl-dev"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-collision"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-common"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-geometry"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-scene-graph"
    WINDOWS_DEPENDS
      "console-bridge"
      "Eigen3"
      "pcl"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-collision"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-common"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-geometry"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-scene-graph")
endif()
