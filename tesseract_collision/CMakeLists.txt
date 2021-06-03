cmake_minimum_required(VERSION 3.6.0)

# Extract package name and version
find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)
project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES CXX)

if(WIN32)
  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/vhacd_common.cmake")

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system program_options)
find_package(octomap REQUIRED)
find_package(console_bridge REQUIRED)
find_package(tesseract_geometry REQUIRED)
find_package(tesseract_common REQUIRED)
find_package(tesseract_support REQUIRED)
find_package(fcl 0.6 REQUIRED)

find_package(
  Bullet
  REQUIRED
  CONFIGS
  BulletConfig-float64.cmake
  BulletConfig.cmake)
if(NOT
   ${BULLET_DEFINITIONS}
   MATCHES
   ".*-DBT_USE_DOUBLE_PRECISION.*")
  message(
    WARNING "Bullet does not appear to be build with double precision, current definitions: ${BULLET_DEFINITIONS}")
endif()

include_directories(BEFORE ${FCL_INCLUDE_DIRS})
link_directories(BEFORE ${FCL_LIBRARY_DIRS})

include_directories(BEFORE "${BULLET_ROOT_DIR}/${BULLET_INCLUDE_DIRS}")
link_directories(BEFORE "${BULLET_ROOT_DIR}/${BULLET_LIBRARY_DIRS}")

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

# Create interface for core
add_library(${PROJECT_NAME}_core INTERFACE)
target_link_libraries(
  ${PROJECT_NAME}_core
  INTERFACE Eigen3::Eigen
            tesseract::tesseract_common
            tesseract::tesseract_geometry
            ${BULLET_LIBRARIES}
            Boost::boost
            Boost::system)
target_compile_options(${PROJECT_NAME}_core INTERFACE ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_core INTERFACE ${TESSERACT_COMPILE_DEFINITIONS} ${BULLET_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_core ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_core INTERFACE VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}_core
  INTERFACE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(${PROJECT_NAME}_core INTERFACE "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                          "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_core SYSTEM INTERFACE "${BULLET_ROOT_DIR}/${BULLET_INCLUDE_DIRS}")

# Create target for Bullet implementation
add_library(
  ${PROJECT_NAME}_bullet
  src/bullet/bullet_cast_bvh_manager.cpp
  src/bullet/bullet_cast_simple_manager.cpp
  src/bullet/bullet_discrete_bvh_manager.cpp
  src/bullet/bullet_discrete_simple_manager.cpp
  src/bullet/bullet_utils.cpp
  src/bullet/tesseract_compound_collision_algorithm.cpp
  src/bullet/tesseract_compound_compound_collision_algorithm.cpp
  src/bullet/tesseract_collision_configuration.cpp
  src/bullet/tesseract_convex_convex_algorithm.cpp
  src/bullet/tesseract_gjk_pair_detector.cpp)
target_link_libraries(
  ${PROJECT_NAME}_bullet
  PUBLIC ${PROJECT_NAME}_core
         Eigen3::Eigen
         tesseract::tesseract_geometry
         console_bridge::console_bridge
         octomap
         octomath)
target_link_libraries(${PROJECT_NAME}_bullet PUBLIC ${BULLET_LIBRARIES})
target_compile_options(${PROJECT_NAME}_bullet PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(${PROJECT_NAME}_bullet PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_bullet PUBLIC ${TESSERACT_COMPILE_DEFINITIONS} ${BULLET_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_bullet ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_bullet PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}_bullet
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(${PROJECT_NAME}_bullet PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                         "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_bullet SYSTEM PUBLIC "${BULLET_ROOT_DIR}/${BULLET_INCLUDE_DIRS}")

# Create target for FCL implementation
add_library(${PROJECT_NAME}_fcl src/fcl/fcl_discrete_managers.cpp src/fcl/fcl_utils.cpp
                                src/fcl/fcl_collision_object_wrapper.cpp)
target_link_libraries(
  ${PROJECT_NAME}_fcl
  PUBLIC ${PROJECT_NAME}_core
         Eigen3::Eigen
         tesseract::tesseract_geometry
         fcl
         ${BULLET_LIBRARIES}
         console_bridge::console_bridge
         octomap
         octomath)
target_compile_options(${PROJECT_NAME}_fcl PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(${PROJECT_NAME}_fcl PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_fcl PUBLIC ${TESSERACT_COMPILE_DEFINITIONS} ${BULLET_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_fcl ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_fcl PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}_fcl
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(${PROJECT_NAME}_fcl PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                      "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_fcl SYSTEM PUBLIC "${BULLET_ROOT_DIR}/${BULLET_INCLUDE_DIRS}")

# Create target that profiles the collision checkers.
add_executable(${PROJECT_NAME}_profile src/collision_profile.cpp)
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
target_clang_tidy(${PROJECT_NAME}_profile ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_profile PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}_profile
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})

# Third party vhacd
find_package(OpenMP REQUIRED)

option(NO_OPENCL "NO_OPENCL" OFF)
message("NO_OPENCL " ${NO_OPENCL})

if(NOT NO_OPENCL)
  find_package(OpenCL)
endif()

if(NOT TARGET OpenMP::OpenMP_CXX)
  find_package(Threads REQUIRED)
  add_library(OpenMP::OpenMP_CXX IMPORTED INTERFACE)
  set_property(TARGET OpenMP::OpenMP_CXX PROPERTY INTERFACE_COMPILE_OPTIONS ${OpenMP_CXX_FLAGS})
  # Only works if the same flag is passed to the linker; use CMake 3.9+ otherwise (Intel, AppleClang)
  set_property(TARGET OpenMP::OpenMP_CXX PROPERTY INTERFACE_LINK_LIBRARIES ${OpenMP_CXX_FLAGS} Threads::Threads)
endif()

add_library(
  ${PROJECT_NAME}_vhacd
  ${VHACD_CPP_FILES}
  ${VHACD_C_FILES}
  ${VHACD_INC_FILES}
  ${VHACD_INL_FILES}
  ${VHACD_CL_FILES})
target_cxx_version(${PROJECT_NAME}_vhacd PUBLIC VERSION ${TESSERACT_CXX_VERSION})

if(OpenCL_FOUND)
  target_include_directories(${PROJECT_NAME}_vhacd PRIVATE "${OpenCL_INCLUDE_DIR}")
  target_link_libraries(${PROJECT_NAME}_vhacd PRIVATE "${OpenCL_LIBRARY}")
  target_compile_definitions(${PROJECT_NAME}_vhacd PRIVATE -DOPENCL_FOUND=1)
  target_compile_definitions(${PROJECT_NAME}_vhacd PRIVATE -DOPENCL_CL_FILES="${PROJECT_CL_FILES}")
endif()

target_compile_definitions(${PROJECT_NAME}_vhacd PUBLIC ${BULLET_DEFINITIONS})
target_link_libraries(
  ${PROJECT_NAME}_vhacd
  PUBLIC OpenMP::OpenMP_CXX
         tesseract::tesseract_common
         Eigen3::Eigen
         ${BULLET_LIBRARIES})
target_include_directories(${PROJECT_NAME}_vhacd PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                        "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_vhacd SYSTEM PUBLIC "${BULLET_ROOT_DIR}/${BULLET_INCLUDE_DIRS}")

# Convex decomposition libraries
find_library(CONVEX_DECOMPOSITION_LIBRARY ConvexDecomposition HINTS ${BULLET_ROOT_DIR}/${BULLET_LIBRARY_DIRS})
find_library(HACD_LIBRARY HACD HINTS ${BULLET_ROOT_DIR}/${BULLET_LIBRARY_DIRS})

add_library(${PROJECT_NAME}_convex_decomposition src/convex_decomposition/convex_decomposition_vhacd.cpp
                                                 src/convex_decomposition/convex_decomposition_hacd.cpp)
target_link_libraries(
  ${PROJECT_NAME}_convex_decomposition
  PUBLIC ${PROJECT_NAME}_vhacd
         Eigen3::Eigen
         tesseract::tesseract_geometry
         console_bridge::console_bridge
         ${BULLET_LIBRARIES}
         ${HACD_LIBRARY})
target_compile_options(${PROJECT_NAME}_convex_decomposition PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(${PROJECT_NAME}_convex_decomposition PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_convex_decomposition PUBLIC ${TESSERACT_COMPILE_DEFINITIONS}
                                                                       ${BULLET_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_convex_decomposition ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS}
                  ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_convex_decomposition PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}_convex_decomposition
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(
  ${PROJECT_NAME}_convex_decomposition PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                              "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_convex_decomposition SYSTEM
                           PUBLIC "${BULLET_ROOT_DIR}/${BULLET_INCLUDE_DIRS}")

# Create test suite interface
add_library(${PROJECT_NAME}_test_suite INTERFACE)
target_link_libraries(
  ${PROJECT_NAME}_test_suite
  INTERFACE Eigen3::Eigen
            tesseract::tesseract_support
            tesseract::tesseract_geometry
            ${PROJECT_NAME}_core)
target_compile_options(${PROJECT_NAME}_test_suite INTERFACE ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_test_suite INTERFACE ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_test_suite ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS}
                  ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_test_suite INTERFACE VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}_test_suite
  INTERFACE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(${PROJECT_NAME}_test_suite INTERFACE "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                                "$<INSTALL_INTERFACE:include>")

# Create test suite interface
add_library(${PROJECT_NAME}_test_suite_benchmarks INTERFACE)
target_link_libraries(${PROJECT_NAME}_test_suite_benchmarks INTERFACE Eigen3::Eigen tesseract::tesseract_support
                                                                      tesseract::tesseract_geometry)
target_compile_options(${PROJECT_NAME}_test_suite_benchmarks INTERFACE ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_clang_tidy(${PROJECT_NAME}_test_suite_benchmarks ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS}
                  ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_test_suite_benchmarks INTERFACE VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}_test_suite_benchmarks
  INTERFACE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(
  ${PROJECT_NAME}_test_suite_benchmarks INTERFACE "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                  "$<INSTALL_INTERFACE:include>")

set(PACKAGE_LIBRARIES
    ${PROJECT_NAME}_core
    ${PROJECT_NAME}_bullet
    ${PROJECT_NAME}_fcl
    ${PROJECT_NAME}_test_suite
    ${PROJECT_NAME}_test_suite_benchmarks
    ${PROJECT_NAME}_profile
    ${PROJECT_NAME}_vhacd
    ${PROJECT_NAME}_convex_decomposition)

if(NOT MSVC)
  # Create target for creating convex hull's from meshes
  add_executable(create_convex_hull src/create_convex_hull.cpp)
  target_link_libraries(
    create_convex_hull
    PUBLIC ${PROJECT_NAME}_core
           ${BULLET_LIBRARIES}
           Boost::boost
           Boost::program_options
           Eigen3::Eigen
           tesseract::tesseract_common
           tesseract::tesseract_geometry
           console_bridge::console_bridge
           octomap
           octomath)
  target_compile_options(create_convex_hull PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                    ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
  target_compile_definitions(create_convex_hull PRIVATE ${TESSERACT_COMPILE_DEFINITIONS} ${BULLET_DEFINITIONS})
  target_clang_tidy(create_convex_hull ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
  target_cxx_version(create_convex_hull PRIVATE VERSION ${TESSERACT_CXX_VERSION})
  target_include_directories(create_convex_hull PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
  target_include_directories(create_convex_hull SYSTEM PUBLIC "${BULLET_ROOT_DIR}/${BULLET_INCLUDE_DIRS}")

  list(APPEND PACKAGE_LIBRARIES create_convex_hull)
endif()

configure_package(NAMESPACE tesseract TARGETS ${PACKAGE_LIBRARIES})

# Mark cpp header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN "*.hpp"
  PATTERN "*.inl"
  PATTERN ".svn" EXCLUDE)

# Add Examples
add_subdirectory(examples)

if(TESSERACT_ENABLE_TESTING)
  message(AUTHOR_WARNING "COLLISION CMAKE VERSION: ${CMAKE_VERSION}")
  enable_testing()
  add_run_tests_target(ENABLE ${TESSERACT_ENABLE_RUN_TESTING})
  add_subdirectory(test)
endif()

if(TESSERACT_ENABLE_BENCHMARKING)
  add_subdirectory(test/benchmarks)
endif()

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
