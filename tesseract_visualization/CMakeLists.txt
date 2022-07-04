cmake_minimum_required(VERSION 3.5.0)

# Extract package name and version
find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)
project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES CXX)

if(WIN32)
  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

find_package(Eigen3 REQUIRED)
find_package(console_bridge REQUIRED)
find_package(tesseract_collision REQUIRED)
find_package(tesseract_common REQUIRED)
find_package(tesseract_environment REQUIRED)
find_package(tesseract_scene_graph REQUIRED)
find_package(tesseract_state_solver REQUIRED)

# Optional to build ignition visualization
find_package(ignition-common3 COMPONENTS profiler events av QUIET)
find_package(ignition-transport8 QUIET)
find_package(ignition-msgs5 QUIET)
find_package(ignition-math6-eigen3 QUIET)

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

set(IGNITION_FOUND FALSE)
if(ignition-common3_FOUND
   AND ignition-transport8_FOUND
   AND ignition-msgs5_FOUND
   AND ignition-math6-eigen3_FOUND)
  set(IGNITION_FOUND TRUE)
  message(STATUS "Ignition Visualization Library will be built!")
endif()

# Load variable for clang tidy args, compiler options and cxx version
tesseract_variables()

initialize_code_coverage(ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
set(COVERAGE_EXCLUDE
    /usr/*
    /opt/*
    ${CMAKE_CURRENT_LIST_DIR}/test/*
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
if(IGNITION_FOUND)
  add_library(${PROJECT_NAME}_ignition src/ignition/entity_manager.cpp src/ignition/conversions.cpp)
  target_link_libraries(
    ${PROJECT_NAME}_ignition
    PUBLIC tesseract::tesseract_scene_graph
           tesseract::tesseract_common
           ${IGNITION-COMMON_LIBRARIES}
           ${IGNITION-MSGS_LIBRARIES}
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
  target_include_directories(${PROJECT_NAME}_ignition SYSTEM PUBLIC ${IGNITION-COMMON_INCLUDE_DIRS}
                                                                    ${IGNITION-MSGS_INCLUDE_DIRS})

  add_library(${PROJECT_NAME}_ignition_visualization src/ignition/tesseract_ignition_visualization.cpp)
  target_link_libraries(
    ${PROJECT_NAME}_ignition_visualization
    PUBLIC ${PROJECT_NAME}
           ${PROJECT_NAME}_ignition
           tesseract::tesseract_scene_graph
           tesseract::tesseract_common
           ${IGNITION-COMMON_LIBRARIES}
           ${IGNITION-TRANSPORT_LIBRARIES}
           ${IGNITION-MSGS_LIBRARIES}
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
  target_include_directories(
    ${PROJECT_NAME}_ignition_visualization SYSTEM
    PUBLIC ${IGNITION-COMMON_INCLUDE_DIRS} ${IGNITION-TRANSPORT_INCLUDE_DIRS} ${IGNITION-MSGS_INCLUDE_DIRS})

  list(
    APPEND
    PACKAGE_LIBRARIES
    ${PROJECT_NAME}_ignition
    ${PROJECT_NAME}_ignition_visualization)
else()
  list(APPEND IGNITION_MISSING "")
  if(NOT ignition-common3_FOUND)
    list(APPEND IGNITION_MISSING "ignition-common3")
  endif()
  if(NOT ignition-transport8_FOUND)
    list(APPEND IGNITION_MISSING "ignition-transport8")
  endif()
  if(NOT ignition-msgs5_FOUND)
    list(APPEND IGNITION_MISSING "ignition-msgs5")
  endif()
  if(NOT ignition-math6-eigen3_FOUND)
    list(APPEND IGNITION_MISSING "ignition-math6-eigen3")
  endif()
  message(STATUS "Ignition Visualization Library will not be built! Missing: ${IGNITION_MISSING}")
endif()

configure_package(NAMESPACE tesseract TARGETS ${PACKAGE_LIBRARIES})

# Mark cpp header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)

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
      "${TESSERACT_PACKAGE_PREFIX}tesseract-collision"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-common"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-environment"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-scene-graph"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-state-solver"
    WINDOWS_DEPENDS
      "console-bridge"
      "Eigen3"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-collision"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-common"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-environment"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-scene-graph"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-state-solver")
endif()
