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

# These targets are necessary for 16.04 builds. Remove when Kinetic support is dropped
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

# Core
add_subdirectory(core)

# Bullet
option(TESSERACT_BUILD_BULLET "Build Bullet components" ON)
if(TESSERACT_BUILD_BULLET)
  message("Building Bullet components")
  add_subdirectory(bullet)
endif()

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

# Testing
if(TESSERACT_ENABLE_TESTING AND TESSERACT_BUILD_BULLET AND TESSERACT_BUILD_FCL)
  message(AUTHOR_WARNING "COLLISION CMAKE VERSION: ${CMAKE_VERSION}")
  enable_testing()
  add_run_tests_target(ENABLE ${TESSERACT_ENABLE_RUN_TESTING})
  add_subdirectory(test)
endif()

# Benchmarks
if(TESSERACT_ENABLE_BENCHMARKING AND TESSERACT_BUILD_BULLET AND TESSERACT_BUILD_FCL)
  add_subdirectory(test/benchmarks)
endif()

# Profiler
if(TESSERACT_BUILD_BULLET AND TESSERACT_BUILD_FCL)
  # Examples
  add_subdirectory(examples)

  # Create target that profiles the collision checkers.
  add_executable(${PROJECT_NAME}_profile core/src/collision_profile.cpp)
  target_link_libraries(
    ${PROJECT_NAME}_profile
    PRIVATE ${PROJECT_NAME}_bullet
            ${PROJECT_NAME}_fcl
            tesseract::tesseract_geometry
            tesseract::tesseract_support
            ${BULLET_LIBRARIES}
            ${Boost_LIBRARIES}
            ${OCTOMAP_LIBRARIES}
            ${LIBFCL_LIBRARIES})
  target_compile_options(${PROJECT_NAME}_profile PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
  target_compile_definitions(${PROJECT_NAME}_profile INTERFACE ${TESSERACT_COMPILE_DEFINITIONS})
  target_clang_tidy(${PROJECT_NAME}_profile ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
  target_cxx_version(${PROJECT_NAME}_profile PRIVATE VERSION ${TESSERACT_CXX_VERSION})
  target_code_coverage(
    ${PROJECT_NAME}_profile
    PRIVATE
    ALL
    EXCLUDE ${COVERAGE_EXCLUDE}
    ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})

  install_targets(TARGETS ${PROJECT_NAME}_profile)
endif()

# Package configuration
install(EXPORT ${PROJECT_NAME}-targets NAMESPACE "tesseract::" DESTINATION lib/cmake/${PROJECT_NAME})
export(EXPORT ${PROJECT_NAME}-targets NAMESPACE "tesseract::"
       FILE ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-targets.cmake)
configure_package()

if(TESSERACT_PACKAGE)
  tesseract_cpack(
    VERSION ${pkg_extracted_version}
    MAINTAINER <https://github.com/ros-industrial-consortium/tesseract>
    DESCRIPTION ${pkg_extracted_description}
    LICENSE_FILE ${CMAKE_CURRENT_LIST_DIR}/../LICENSE
    README_FILE ${CMAKE_CURRENT_LIST_DIR}/../README.md
    LINUX_DEPENDS
      "${TESSERACT_PACKAGE_PREFIX}tesseract-geometry"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-common"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-support"
      "libbullet-dev"
      "libfcl-dev (>= 0.6.0) | ${TESSERACT_PACKAGE_PREFIX}fcl"
      "libconsole-bridge-dev"
      "libeigen3-dev"
    WINDOWS_DEPENDS
      "${TESSERACT_PACKAGE_PREFIX}tesseract-geometry"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-common"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-support"
      "Eigen3")
endif()
