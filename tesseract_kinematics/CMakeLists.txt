cmake_minimum_required(VERSION 3.5.0)

# Extract package name and version
find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)
project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES CXX)

if(WIN32)
  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

# System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)
find_package(console_bridge REQUIRED)
find_package(tesseract_scene_graph REQUIRED)
find_package(tesseract_state_solver REQUIRED)
find_package(tesseract_common REQUIRED)
find_package(yaml-cpp REQUIRED)

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

# Define compile-time default variables
if(MSVC)
  set(TESSERACT_KINEMATICS_PLUGIN_PATH ${CMAKE_INSTALL_PREFIX}/bin)
else()
  set(TESSERACT_KINEMATICS_PLUGIN_PATH ${CMAKE_INSTALL_PREFIX}/lib)
endif()

# Variable for kinematics plugins
set(KINEMATICS_PLUGINS "")

# Core
add_subdirectory(core)

# IKFast
option(TESSERACT_BUILD_IKFAST "Build IKFast components" ON)
if(TESSERACT_BUILD_IKFAST)
  message("Building IKFast components")
  add_subdirectory(ikfast)
endif()

# KDL
option(TESSERACT_BUILD_KDL "Build KDL components" ON)
if(TESSERACT_BUILD_KDL)
  message("Building KDL components")
  add_subdirectory(kdl)
endif()

# OPW
option(TESSERACT_BUILD_OPW "Build OPW components" ON)
if(TESSERACT_BUILD_OPW)
  message("Building OPW components")
  add_subdirectory(opw)
endif()

# UR
option(TESSERACT_BUILD_UR "Build UR components" ON)
if(TESSERACT_BUILD_UR)
  message("Building UR components")
  add_subdirectory(ur)
endif()

# Add compiler definition to core so it can find all plugins produced
string(
  REPLACE ";"
          ":"
          KINEMATICS_PLUGINS_STRING
          "${KINEMATICS_PLUGINS}")
target_compile_definitions(${PROJECT_NAME}_core PRIVATE TESSERACT_KINEMATICS_PLUGINS="${KINEMATICS_PLUGINS_STRING}")

# Testing
if(TESSERACT_ENABLE_TESTING
   AND TESSERACT_BUILD_IKFAST
   AND TESSERACT_BUILD_KDL
   AND TESSERACT_BUILD_OPW
   AND TESSERACT_BUILD_UR)
  enable_testing()
  add_run_tests_target(ENABLE ${TESSERACT_ENABLE_RUN_TESTING})
  add_subdirectory(test)
endif()

configure_package(NAMESPACE tesseract)

if(TESSERACT_PACKAGE)
  tesseract_cpack(
    VERSION ${pkg_extracted_version}
    MAINTAINER <https://github.com/ros-industrial-consortium/tesseract>
    DESCRIPTION ${pkg_extracted_description}
    LICENSE_FILE ${CMAKE_CURRENT_LIST_DIR}/../LICENSE
    README_FILE ${CMAKE_CURRENT_LIST_DIR}/../README.md
    LINUX_DEPENDS
      "libconsole-bridge-dev"
      "liborocos-kdl-dev"
      "libeigen3-dev"
      "libyaml-cpp-dev"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-common"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-scene-graph"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-state-solver"
      "${TESSERACT_PACKAGE_PREFIX}opw-kinematics"
    WINDOWS_DEPENDS
      "console_bridge"
      "orocos-kdl"
      "Eigen3"
      "yaml-cpp"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-common"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-scene-graph"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-state-solver"
      "${TESSERACT_PACKAGE_PREFIX}opw-kinematics")
endif()
