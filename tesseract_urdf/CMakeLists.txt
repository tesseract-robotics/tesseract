cmake_minimum_required(VERSION 3.15.0)

# Extract package name and version
find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)
project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES C CXX)

if(WIN32)
  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(console_bridge REQUIRED)
find_package(PCL REQUIRED COMPONENTS io)
find_package(tesseract_geometry REQUIRED)
find_package(tesseract_common REQUIRED)
find_package(tesseract_scene_graph REQUIRED)
find_package(tesseract_collision REQUIRED COMPONENTS bullet)

# Load variable for clang tidy args, compiler options and cxx version
tesseract_variables()

initialize_code_coverage(ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
set(COVERAGE_EXCLUDE
    /usr/*
    /opt/*
    ${CMAKE_CURRENT_LIST_DIR}/test/*
    /*/install/*
    /*/devel/*
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

target_link_libraries(${PROJECT_NAME} PRIVATE ${PCL_IO_LIBRARIES})
target_include_directories(${PROJECT_NAME} PRIVATE ${PCL_INCLUDE_DIRS})
target_compile_definitions(${PROJECT_NAME} PUBLIC TESSERACT_PARSE_POINT_CLOUDS="ON")

target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                  "$<INSTALL_INTERFACE:include>")

target_clang_tidy(${PROJECT_NAME} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME} PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})

configure_package(
  NAMESPACE tesseract
  TARGETS ${PROJECT_NAME}
  DEPENDENCIES
    Boost
    Eigen3
    console_bridge
    "tesseract_collision COMPONENTS bullet"
    tesseract_geometry
    tesseract_common
    tesseract_scene_graph)

# Mark cpp header files for installation
install(FILES include/${PROJECT_NAME}/urdf_parser.h DESTINATION include/${PROJECT_NAME})
install(FILES include/${PROJECT_NAME}/utils.h DESTINATION include/${PROJECT_NAME})

# ADD Examples
if(TESSERACT_ENABLE_EXAMPLES OR TESSERACT_URDF_ENABLE_EXAMPLES)
  add_subdirectory(examples)
endif()

if(TESSERACT_ENABLE_TESTING OR TESSERACT_URDF_ENABLE_TESTING)
  enable_testing()
  add_run_tests_target(ENABLE ${TESSERACT_ENABLE_RUN_TESTING})
  add_subdirectory(test)
endif()

if(TESSERACT_PACKAGE)
  cpack(
    VERSION ${pkg_extracted_version}
    MAINTAINER_NAME ${pkg_extracted_maintainer_name}
    MAINTAINER_EMAIL ${pkg_extracted_maintainer_email}
    DESCRIPTION ${pkg_extracted_description}
    LICENSE_FILE ${CMAKE_CURRENT_LIST_DIR}/../LICENSE
    README_FILE ${CMAKE_CURRENT_LIST_DIR}/../README.md
    LINUX_DEPENDS
      "libconsole-bridge-dev"
      "libeigen3-dev"
      "libpcl-dev"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-collision-bullet"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-common"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-geometry"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-scene-graph"
    WINDOWS_DEPENDS
      "console-bridge"
      "Eigen3"
      "pcl"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-collision-bullet"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-common"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-geometry"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-scene-graph")

  if(UNIX AND TESSERACT_PACKAGE_SOURCE)
    cpack_debian_source_package(
      CHANGLELOG ${CMAKE_CURRENT_LIST_DIR}/CHANGELOG.rst
      UPLOAD ${TESSERACT_PACKAGE_SOURCE_UPLOAD}
      DPUT_HOST ${TESSERACT_PACKAGE_SOURCE_DPUT_HOST}
      DEBIAN_INCREMENT ${TESSERACT_PACKAGE_SOURCE_DEBIAN_INCREMENT}
      DISTRIBUTIONS ${TESSERACT_PACKAGE_SOURCE_DISTRIBUTIONS})
  endif()
endif()
