cmake_minimum_required(VERSION 3.6.0)

# Extract package name and version
find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)
project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES CXX)

if(WIN32)
  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system program_options)
find_package(octomap REQUIRED)
find_package(console_bridge REQUIRED)
find_package(tesseract_geometry REQUIRED)
find_package(tesseract_common REQUIRED)
find_package(tesseract_support REQUIRED)
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
add_subdirectory(core)

# Bullet (currently required for creation of convex hulls)
add_subdirectory(bullet)

# FCL
option(TESSERACT_BUILD_FCL "Build FCL components" ON)
if(TESSERACT_BUILD_FCL)
  message("Building FCL components")
  add_subdirectory(fcl)
endif()

# VHACD
option(TESSERACT_BUILD_VHACD "Build VHACD components" ON)
if(TESSERACT_BUILD_VHACD)
  message("Building VHACD components")
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
if(TESSERACT_ENABLE_TESTING AND TESSERACT_BUILD_FCL)
  enable_testing()
  add_run_tests_target(ENABLE ${TESSERACT_ENABLE_RUN_TESTING})
  add_subdirectory(test)
endif()

# Benchmarks
if(TESSERACT_ENABLE_BENCHMARKING AND TESSERACT_BUILD_FCL)
  add_subdirectory(test/benchmarks)
endif()

# Examples
if(TESSERACT_ENABLE_EXAMPLES AND TESSERACT_BUILD_FCL)
  add_subdirectory(examples)
endif()

configure_package(NAMESPACE tesseract)

if(TESSERACT_PACKAGE)
  set(LINUX_DEPENDS
      "libbullet-dev"
      "libbullet-extras-dev"
      "libeigen3-dev"
      "libconsole-bridge-dev"
      "libyaml-cpp-dev"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-geometry"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-common"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-support")

  set(WINDOWS_DEPENDS
      "Eigen3"
      "console_bridge"
      "bullet"
      "bullet_extras"
      "yaml-cpp"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-geometry"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-common"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-support")

  if(TESSERACT_BUILD_FCL)
    list(APPEND LINUX_DEPENDS "libfcl-dev (>= 0.6.0) | ${TESSERACT_PACKAGE_PREFIX}fcl")
    list(APPEND LINUX_DEPENDS "fcl (>= 0.6.0) | ${TESSERACT_PACKAGE_PREFIX}fcl")
  endif()

  if(TESSERACT_BUILD_VHACD)
    list(APPEND LINUX_DEPENDS "libomp-dev")
    list(APPEND WINDOWS_DEPENDS "omp")

    if(OpenCL_FOUND)
      list(APPEND LINUX_DEPENDS "ocl-icd-opencl-dev")
      list(APPEND WINDOWS_DEPENDS "ocl-icd-opencl")
    endif()
  endif()

  tesseract_cpack(
    VERSION ${pkg_extracted_version}
    MAINTAINER <https://github.com/ros-industrial-consortium/tesseract>
    DESCRIPTION ${pkg_extracted_description}
    LICENSE_FILE ${CMAKE_CURRENT_LIST_DIR}/../LICENSE
    README_FILE ${CMAKE_CURRENT_LIST_DIR}/../README.md
    LINUX_DEPENDS ${LINUX_DEPENDS}
    WINDOWS_DEPENDS ${WINDOWS_DEPENDS})
endif()
