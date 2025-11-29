cmake_minimum_required(VERSION 3.15.0)

# Extract package name and version
find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)
project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES C CXX)

if(WIN32)
  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

find_package(Eigen3 REQUIRED)
find_package(console_bridge REQUIRED)
find_package(tesseract_collision REQUIRED COMPONENTS core)
find_package(tesseract_common REQUIRED)
find_package(tesseract_environment REQUIRED)
find_package(tesseract_scene_graph REQUIRED)
find_package(tesseract_state_solver REQUIRED)
find_package(boost_plugin_loader REQUIRED)

# Optional to build gazebo visualization
find_package(gz-common5 COMPONENTS profiler events av QUIET)
find_package(gz-transport11 QUIET)
find_package(gz-msgs8 QUIET)
find_package(gz-math7-eigen3 QUIET)

set(GAZEBO_FOUND FALSE)
if(gz-common5_FOUND
   AND gz-transport11_FOUND
   AND gz-msgs8_FOUND
   AND gz-math7-eigen3_FOUND)
  set(GAZEBO_FOUND TRUE)
  message(STATUS "Gazebo Visualization Library will be built!")
endif()

# Load variable for clang tidy args, compiler options and cxx version
tesseract_variables()

initialize_code_coverage(ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
set(COVERAGE_EXCLUDE
    /usr/*
    /opt/*
    ${CMAKE_CURRENT_LIST_DIR}/test/*
    /*/install/*
    /*/devel/*
    /*/gtest/*)
add_code_coverage_all_targets(EXCLUDE ${COVERAGE_EXCLUDE} ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})

# ######################################################################################################################
# Define compile-time default variables
if(MSVC)
  set(TESSERACT_VISUALIZATION_PLUGIN_PATH ${CMAKE_INSTALL_PREFIX}/bin)
else()
  set(TESSERACT_VISUALIZATION_PLUGIN_PATH ${CMAKE_INSTALL_PREFIX}/lib)
endif()

add_library(
  ${PROJECT_NAME}
  src/visualization_loader.cpp
  src/trajectory_interpolator.cpp
  src/trajectory_player.cpp
  src/markers/marker.cpp)
target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC Eigen3::Eigen
         tesseract::tesseract_environment
         tesseract::tesseract_scene_graph
         tesseract::tesseract_state_solver_core
         tesseract::tesseract_collision_core
         tesseract::tesseract_common
         boost_plugin_loader::boost_plugin_loader
         console_bridge::console_bridge)
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
target_compile_definitions(${PROJECT_NAME}
                           PRIVATE TESSERACT_VISUALIZATION_PLUGIN_PATH="${TESSERACT_VISUALIZATION_PLUGIN_PATH}")
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                  "$<INSTALL_INTERFACE:include>")

list(APPEND PACKAGE_LIBRARIES ${PROJECT_NAME})
option(TESSERACT_BUILD_GZ_VISUALIZATION "Build Gazebo visualization plugin" ON)
if(GAZEBO_FOUND AND TESSERACT_BUILD_GZ_VISUALIZATION)
  add_library(${PROJECT_NAME}_ignition src/ignition/entity_manager.cpp src/ignition/conversions.cpp)
  target_link_libraries(
    ${PROJECT_NAME}_ignition
    PUBLIC tesseract::tesseract_scene_graph
           tesseract::tesseract_common
           gz-common5::gz-common5-profiler
           gz-common5::gz-common5-events
           gz-common5::gz-common5-av
           gz-msg8::gz-msg8
           console_bridge::console_bridge)
  target_compile_options(${PROJECT_NAME}_ignition PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
  target_compile_options(${PROJECT_NAME}_ignition PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
  target_compile_definitions(${PROJECT_NAME}_ignition PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
  target_clang_tidy(${PROJECT_NAME}_ignition ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
  target_cxx_version(${PROJECT_NAME}_ignition PUBLIC VERSION ${TESSERACT_CXX_VERSION})
  target_code_coverage(
    ${PROJECT_NAME}_ignition
    PRIVATE
    ALL
    EXCLUDE ${COVERAGE_EXCLUDE}
    ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
  target_include_directories(${PROJECT_NAME}_ignition PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                             "$<INSTALL_INTERFACE:include>")

  add_library(${PROJECT_NAME}_ignition_visualization src/ignition/tesseract_ignition_visualization.cpp)
  target_link_libraries(
    ${PROJECT_NAME}_ignition_visualization
    PUBLIC ${PROJECT_NAME}
           ${PROJECT_NAME}_ignition
           tesseract::tesseract_scene_graph
           tesseract::tesseract_common
           gz-common5::gz-common5-profiler
           gz-common5::gz-common5-events
           gz-common5::gz-common5-av
           gz-transport11::gz-transport11
           gz-msg8::gz-msg8
           console_bridge::console_bridge)
  target_compile_options(${PROJECT_NAME}_ignition_visualization PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
  target_compile_options(${PROJECT_NAME}_ignition_visualization PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
  target_compile_definitions(${PROJECT_NAME}_ignition_visualization PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
  target_clang_tidy(${PROJECT_NAME}_ignition_visualization ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
  target_cxx_version(${PROJECT_NAME}_ignition_visualization PUBLIC VERSION ${TESSERACT_CXX_VERSION})
  target_code_coverage(
    ${PROJECT_NAME}_ignition_visualization
    PRIVATE
    ALL
    EXCLUDE ${COVERAGE_EXCLUDE}
    ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
  target_include_directories(
    ${PROJECT_NAME}_ignition_visualization PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                  "$<INSTALL_INTERFACE:include>")

  list(
    APPEND
    PACKAGE_LIBRARIES
    ${PROJECT_NAME}_ignition
    ${PROJECT_NAME}_ignition_visualization)
else()
  list(APPEND GAZEBO_MISSING "")
  if(NOT gz-common5_FOUND)
    list(APPEND GAZEBO_MISSING "gz-common5")
  endif()
  if(NOT ignition-transport11_FOUND)
    list(APPEND GAZEBO_MISSING "gz-transport11")
  endif()
  if(NOT gz-msgs8_FOUND)
    list(APPEND GAZEBO_MISSING "gz-msgs8")
  endif()
  if(NOT gz-math7-eigen3_FOUND)
    list(APPEND GAZEBO_MISSING "gz-math7-eigen3")
  endif()
  message(STATUS "Gazebo Visualization Library will not be built! Missing: ${GAZEBO_MISSING}")
endif()

configure_package(
  NAMESPACE tesseract
  TARGETS ${PACKAGE_LIBRARIES}
  DEPENDENCIES
    Eigen3
    console_bridge
    "tesseract_collision COMPONENTS core"
    tesseract_common
    tesseract_environment
    tesseract_scene_graph
    tesseract_state_solver)

# Mark cpp header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)

if(TESSERACT_ENABLE_TESTING OR TESSERACT_VISUALIZATION_ENABLE_TESTING)
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
      "${TESSERACT_PACKAGE_PREFIX}tesseract-collision-core"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-common"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-environment"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-scene-graph"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-state-solver"
    WINDOWS_DEPENDS
      "console-bridge"
      "Eigen3"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-collision-core"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-common"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-environment"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-scene-graph"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-state-solver")

  if(UNIX AND TESSERACT_PACKAGE_SOURCE)
    cpack_debian_source_package(
      CHANGLELOG ${CMAKE_CURRENT_LIST_DIR}/CHANGELOG.rst
      UPLOAD ${TESSERACT_PACKAGE_SOURCE_UPLOAD}
      DPUT_HOST ${TESSERACT_PACKAGE_SOURCE_DPUT_HOST}
      DEBIAN_INCREMENT ${TESSERACT_PACKAGE_SOURCE_DEBIAN_INCREMENT}
      DISTRIBUTIONS ${TESSERACT_PACKAGE_SOURCE_DISTRIBUTIONS})
  endif()
endif()
