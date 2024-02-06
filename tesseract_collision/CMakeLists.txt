cmake_minimum_required(VERSION 3.15.0)

# Extract package name and version
find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)
project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES CXX)

if(WIN32)
  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

# All of these are used by all components so to avoid having them in each they are include here
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system program_options serialization)
find_package(octomap REQUIRED)
find_package(console_bridge REQUIRED)
find_package(tesseract_geometry REQUIRED)
find_package(tesseract_common REQUIRED)
find_package(yaml-cpp REQUIRED)

# These targets are necessary for 16.04 builds. Remove when Kinetic support is dropped
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

if(NOT TARGET octomap)
  add_library(octomap INTERFACE IMPORTED)
  set_target_properties(octomap PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${OCTOMAP_INCLUDE_DIRS}")
  set_target_properties(octomap PROPERTIES INTERFACE_LINK_LIBRARIES "${OCTOMAP_LIBRARIES}")
endif()
if(NOT TARGET octomath)
  add_library(octomath INTERFACE IMPORTED)
  set_target_properties(octomath PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${OCTOMAP_INCLUDE_DIRS}")
  set_target_properties(octomath PROPERTIES INTERFACE_LINK_LIBRARIES "${OCTOMAP_LIBRARIES}")
endif()

# Load variable for clang tidy args, compiler options and cxx version
tesseract_variables()

initialize_code_coverage(ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
set(COVERAGE_EXCLUDE
    /usr/*
    /opt/*
    ${CMAKE_CURRENT_LIST_DIR}/test/*
    ${CMAKE_CURRENT_LIST_DIR}/include/tesseract_collision/test_suite/*
    ${CMAKE_CURRENT_LIST_DIR}/include/tesseract_collision/vhacd/*
    /*/gtest/*
    /*/bullet/LinearMath/*
    /*/bullet/BulletCollision/*
    /*/include/ccd/*
    /*/include/fcl/*)
add_code_coverage_all_targets(EXCLUDE ${COVERAGE_EXCLUDE} ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})

# Define compile-time default variables
if(MSVC)
  set(TESSERACT_CONTACT_MANAGERS_PLUGIN_PATH ${CMAKE_INSTALL_PREFIX}/bin)
else()
  set(TESSERACT_CONTACT_MANAGERS_PLUGIN_PATH ${CMAKE_INSTALL_PREFIX}/lib)
endif()

# Variable for kinematics plugins
set(CONTACT_MANAGERS_PLUGINS "")

# Core
set(SUPPORTED_COMPONENTS core)
add_subdirectory(core)

# Bullet (currently required for creation of convex hulls)
list(APPEND SUPPORTED_COMPONENTS bullet)
add_subdirectory(bullet)

# Test Suite
option(TESSERACT_BUILD_TEST_SUITE "Build Test Suite components" ON)
if(TESSERACT_BUILD_TEST_SUITE)
  message("Building Test Suite components")
  list(APPEND SUPPORTED_COMPONENTS test_suite)
  add_subdirectory(test_suite)
endif()

# FCL
option(TESSERACT_BUILD_FCL "Build FCL components" ON)
if(TESSERACT_BUILD_FCL)
  message("Building FCL components")
  list(APPEND SUPPORTED_COMPONENTS fcl)
  add_subdirectory(fcl)
endif()

# HPP-FCL
option(TESSERACT_BUILD_HPP_FCL "Build HPP-FCL components" ON)
if(TESSERACT_BUILD_HPP_FCL)
  message("Building HPP-FCL components")
  list(APPEND SUPPORTED_COMPONENTS hpp_fcl)
  add_subdirectory(hpp_fcl)
endif()

# VHACD
option(TESSERACT_BUILD_VHACD "Build VHACD components" ON)
if(TESSERACT_BUILD_VHACD)
  message("Building VHACD components")
  list(APPEND SUPPORTED_COMPONENTS vhacd)
  add_subdirectory(vhacd)
endif()

# Add compiler definition to core so it can find all plugins produced
string(
  REPLACE ";"
          ":"
          CONTACT_MANAGERS_PLUGINS_STRING
          "${CONTACT_MANAGERS_PLUGINS}")
target_compile_definitions(${PROJECT_NAME}_core
                           PRIVATE TESSERACT_CONTACT_MANAGERS_PLUGINS="${CONTACT_MANAGERS_PLUGINS_STRING}")

# Testing
if((TESSERACT_ENABLE_TESTING OR TESSERACT_COLLISION_ENABLE_TESTING) AND TESSERACT_BUILD_FCL AND TESSERACT_BUILD_HPP_FCL)
  enable_testing()
  add_run_tests_target(ENABLE ${TESSERACT_ENABLE_RUN_TESTING})
  add_subdirectory(test)
endif()

# Benchmarks
if((TESSERACT_ENABLE_BENCHMARKING OR TESSERACT_COLLISION_ENABLE_BENCHMARKING)
   AND TESSERACT_BUILD_FCL
   AND TESSERACT_BUILD_HPP_FCL
   AND TESSERACT_BUILD_TEST_SUITE)
  add_subdirectory(test/benchmarks)
endif()

# Examples
if((TESSERACT_ENABLE_EXAMPLES OR TESSERACT_COLLISION_ENABLE_EXAMPLES) AND TESSERACT_BUILD_FCL
   AND TESSERACT_BUILD_HPP_FCL)
  add_subdirectory(examples)
endif()

configure_package(COMPONENT core SUPPORTED_COMPONENTS ${SUPPORTED_COMPONENTS})

if(TESSERACT_PACKAGE)
  cpack_component_package(
    VERSION ${pkg_extracted_version}
    MAINTAINER_NAME ${pkg_extracted_maintainer_name}
    MAINTAINER_EMAIL ${pkg_extracted_maintainer_email}
    DESCRIPTION ${pkg_extracted_description}
    LICENSE_FILE ${CMAKE_CURRENT_LIST_DIR}/../LICENSE
    README_FILE ${CMAKE_CURRENT_LIST_DIR}/../README.md
    COMPONENT_DEPENDS ${SUPPORTED_COMPONENTS})

  if(UNIX AND TESSERACT_PACKAGE_SOURCE)
    cpack_debian_source_package(
      CHANGLELOG ${CMAKE_CURRENT_LIST_DIR}/CHANGELOG.rst
      UPLOAD ${TESSERACT_PACKAGE_SOURCE_UPLOAD}
      DPUT_HOST ${TESSERACT_PACKAGE_SOURCE_DPUT_HOST}
      DEBIAN_INCREMENT ${TESSERACT_PACKAGE_SOURCE_DEBIAN_INCREMENT}
      DISTRIBUTIONS ${TESSERACT_PACKAGE_SOURCE_DISTRIBUTIONS})
  endif()
endif()
