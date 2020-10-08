cmake_minimum_required(VERSION 3.6.0)
project(tesseract_collision VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_VISIBILITY_PRESET hidden)
set(CMAKE_VISIBILITY_INLINES_HIDDEN 1)

find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS system thread program_options REQUIRED)
find_package(octomap REQUIRED)
find_package(console_bridge REQUIRED)
find_package(tesseract_geometry REQUIRED)
find_package(tesseract_common REQUIRED)
find_package(tesseract_support REQUIRED)
find_package(fcl REQUIRED)
find_package(cmake_common_scripts REQUIRED)

# We need to find ccd ourselves because FCL doesn't do it in their fclConfig.cmake
find_package(ccd REQUIRED)

# Find Bullet using PkgConfig which is the recommended way
find_package(PkgConfig REQUIRED)
pkg_check_modules(bullet REQUIRED IMPORTED_TARGET bullet)

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

set(COVERAGE_EXCLUDE /usr/* /opt/* ${CMAKE_CURRENT_LIST_DIR}/test/* ${CMAKE_CURRENT_LIST_DIR}/include/tesseract_collision/test_suite/* /*/gtest/* /*/bullet/LinearMath/* /*/bullet/BulletCollision/* /*/include/ccd/* /*/include/fcl/*)

add_code_coverage_all_targets(EXCLUDE ${COVERAGE_EXCLUDE})

# Load variable for clang tidy args, compiler options and cxx version
tesseract_variables()

# Create interface for core
add_library(${PROJECT_NAME}_core INTERFACE)
target_process_export(${PROJECT_NAME}_core INTERFACE)
target_link_libraries(${PROJECT_NAME}_core INTERFACE tesseract::tesseract_common tesseract::tesseract_geometry PkgConfig::bullet ${Boost_LIBRARIES})
target_compile_options(${PROJECT_NAME}_core INTERFACE ${TESSERACT_COMPILE_OPTIONS})
target_compile_definitions(${PROJECT_NAME}_core INTERFACE ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_core ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_core INTERFACE VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(${PROJECT_NAME}_core INTERFACE ALL EXCLUDE ${COVERAGE_EXCLUDE} ENABLE ${TESSERACT_ENABLE_TESTING})
target_include_directories(${PROJECT_NAME}_core INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_core SYSTEM INTERFACE
    ${EIGEN3_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS})

# Create target for Bullet implementation
add_library(${PROJECT_NAME}_bullet
  src/bullet/bullet_cast_bvh_manager.cpp
  src/bullet/bullet_cast_simple_manager.cpp
  src/bullet/bullet_discrete_bvh_manager.cpp
  src/bullet/bullet_discrete_simple_manager.cpp
  src/bullet/bullet_utils.cpp
  src/bullet/tesseract_compound_collision_algorithm.cpp
  src/bullet/tesseract_compound_compound_collision_algorithm.cpp
  src/bullet/tesseract_collision_configuration.cpp
  src/bullet/tesseract_convex_convex_algorithm.cpp
  src/bullet/tesseract_gjk_pair_detector.cpp
)
target_process_export(${PROJECT_NAME}_bullet)
target_link_libraries(${PROJECT_NAME}_bullet PUBLIC ${PROJECT_NAME}_core tesseract::tesseract_geometry PkgConfig::bullet console_bridge octomap octomath)
target_compile_options(${PROJECT_NAME}_bullet PRIVATE ${TESSERACT_COMPILE_OPTIONS})
target_compile_definitions(${PROJECT_NAME}_bullet PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_bullet ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_bullet PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(${PROJECT_NAME}_bullet ALL EXCLUDE ${COVERAGE_EXCLUDE} ENABLE ${TESSERACT_ENABLE_TESTING})
target_include_directories(${PROJECT_NAME}_bullet PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_bullet SYSTEM PUBLIC
    ${EIGEN3_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS})

# Create target for FCL implementation
add_library(${PROJECT_NAME}_fcl
  src/fcl/fcl_discrete_managers.cpp
  src/fcl/fcl_utils.cpp
  src/fcl/fcl_collision_object_wrapper.cpp)
target_process_export(${PROJECT_NAME}_fcl)
target_link_libraries(${PROJECT_NAME}_fcl PUBLIC ${PROJECT_NAME}_core tesseract::tesseract_geometry PkgConfig::bullet console_bridge octomap octomath fcl)
target_compile_options(${PROJECT_NAME}_fcl PRIVATE ${TESSERACT_COMPILE_OPTIONS})
target_compile_definitions(${PROJECT_NAME}_fcl PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_fcl ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_fcl PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(${PROJECT_NAME}_fcl ALL EXCLUDE ${COVERAGE_EXCLUDE} ENABLE ${TESSERACT_ENABLE_TESTING})
target_include_directories(${PROJECT_NAME}_fcl PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_fcl SYSTEM PUBLIC
    ${EIGEN3_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS})

# Create target for creating convex hull's from meshes
add_executable(create_convex_hull src/create_convex_hull.cpp)
target_link_libraries(create_convex_hull PUBLIC tesseract::tesseract_common tesseract::tesseract_geometry console_bridge PkgConfig::bullet ${Boost_LIBRARIES} octomap octomath)
target_compile_options(create_convex_hull PRIVATE ${TESSERACT_COMPILE_OPTIONS})
target_compile_definitions(create_convex_hull PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(create_convex_hull ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(create_convex_hull PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_include_directories(create_convex_hull PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
target_include_directories(create_convex_hull SYSTEM PUBLIC
    ${EIGEN3_INCLUDE_DIRS}
    ${LIBFCL_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS})

# Create test suite interface
add_library(${PROJECT_NAME}_test_suite INTERFACE)
target_link_libraries(${PROJECT_NAME}_test_suite INTERFACE tesseract::tesseract_support tesseract::tesseract_geometry ${PROJECT_NAME}_core)
target_compile_options(${PROJECT_NAME}_test_suite INTERFACE ${TESSERACT_COMPILE_OPTIONS})
target_compile_definitions(${PROJECT_NAME}_test_suite INTERFACE ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_test_suite ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_test_suite INTERFACE VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(${PROJECT_NAME}_test_suite INTERFACE ALL EXCLUDE ${COVERAGE_EXCLUDE} ENABLE ${TESSERACT_ENABLE_TESTING})
target_include_directories(${PROJECT_NAME}_test_suite INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_test_suite SYSTEM INTERFACE
    ${EIGEN3_INCLUDE_DIRS})

# Create test suite interface
add_library(${PROJECT_NAME}_test_suite_benchmarks INTERFACE)
target_link_libraries(${PROJECT_NAME}_test_suite_benchmarks INTERFACE tesseract::tesseract_support tesseract::tesseract_geometry)
target_compile_options(${PROJECT_NAME}_test_suite_benchmarks INTERFACE ${TESSERACT_COMPILE_OPTIONS})
target_clang_tidy(${PROJECT_NAME}_test_suite_benchmarks ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_test_suite_benchmarks INTERFACE VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(${PROJECT_NAME}_test_suite_benchmarks INTERFACE ALL EXCLUDE ${COVERAGE_EXCLUDE} ENABLE ${TESSERACT_ENABLE_TESTING})
target_include_directories(${PROJECT_NAME}_test_suite_benchmarks INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_test_suite_benchmarks SYSTEM INTERFACE
    ${EIGEN3_INCLUDE_DIRS})

configure_package(NAMESPACE tesseract TARGETS ${PROJECT_NAME}_core ${PROJECT_NAME}_bullet ${PROJECT_NAME}_fcl ${PROJECT_NAME}_test_suite ${PROJECT_NAME}_test_suite_benchmarks create_convex_hull)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
 )

# Add Examples
add_subdirectory(examples)

if (TESSERACT_ENABLE_TESTING)
  enable_testing()
  add_run_tests_target(ENABLE ${TESSERACT_ENABLE_RUN_TESTING})
  add_subdirectory(test)
endif()

if (TESSERACT_ENABLE_BENCHMARKING)
  add_subdirectory(test/benchmarks)
endif()

