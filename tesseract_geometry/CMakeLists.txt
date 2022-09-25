cmake_minimum_required(VERSION 3.5.0)

# Extract package name and version
find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)
project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES CXX)

if(WIN32)
  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/")

find_package(Eigen3 REQUIRED)
find_package(console_bridge REQUIRED)
find_package(tesseract_common REQUIRED)
find_package(octomap REQUIRED)
find_package(assimp REQUIRED)
link_directories(${assimp_LIBRARY_DIRS})

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

find_file(ASSIMP_PBR_MATERIAL_FOUND assimp/pbrmaterial.h HINTS ${assimp_INCLUDE_DIRS})

if(ASSIMP_PBR_MATERIAL_FOUND)
  set(TESSERACT_ASSIMP_USE_PBRMATERIAL TESSERACT_ASSIMP_USE_PBRMATERIAL=1)
else()
  set(TESSERACT_ASSIMP_USE_PBRMATERIAL)
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

add_library(
  ${PROJECT_NAME} SHARED
  src/geometry.cpp
  src/geometries/box.cpp
  src/geometries/capsule.cpp
  src/geometries/cone.cpp
  src/geometries/convex_mesh.cpp
  src/geometries/cylinder.cpp
  src/geometries/mesh.cpp
  src/geometries/octree.cpp
  src/geometries/plane.cpp
  src/geometries/polygon_mesh.cpp
  src/geometries/sdf_mesh.cpp
  src/geometries/sphere.cpp)
target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC Eigen3::Eigen
         tesseract::tesseract_common
         console_bridge::console_bridge
         octomap
         octomath
         ${assimp_LIBRARIES})
target_compile_options(${PROJECT_NAME} PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME} PUBLIC ${TESSERACT_COMPILE_DEFINITIONS} ${TESSERACT_ASSIMP_USE_PBRMATERIAL})
target_clang_tidy(${PROJECT_NAME} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME} PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                  "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME} SYSTEM INTERFACE ${assimp_INCLUDE_DIRS})

configure_package(NAMESPACE tesseract TARGETS ${PROJECT_NAME})

# Mark cpp header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)

install(FILES "${CMAKE_CURRENT_LIST_DIR}/cmake/Findassimp.cmake" DESTINATION lib/cmake/${PROJECT_NAME})

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
      "libassimp-dev"
      "libconsole-bridge-dev"
      "liboctomap-dev (>= 1.9.3) | ${TESSERACT_PACKAGE_PREFIX}octomap"
      "libeigen3-dev"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-common"
    WINDOWS_DEPENDS
      "Assimp"
      "console_bridge"
      "octomap | ${TESSERACT_PACKAGE_PREFIX}octomap"
      "Eigen3"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-common")
endif()
