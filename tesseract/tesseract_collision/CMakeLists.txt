cmake_minimum_required(VERSION 3.5.0)
project(tesseract_collision VERSION 0.1.0 LANGUAGES CXX)

find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS system thread program_options REQUIRED)
find_package(octomap REQUIRED)
find_package(console_bridge REQUIRED)
find_package(tesseract_geometry REQUIRED)
find_package(tesseract_common REQUIRED)
find_package(Bullet REQUIRED)
find_package(fcl REQUIRED)

# We need to find ccd ourselves because FCL doesn't do it in their fclConfig.cmake
find_package(ccd REQUIRED)

set(COVERAGE_EXCLUDE /usr/* /opt/* ${CMAKE_CURRENT_LIST_DIR}/test/* ${CMAKE_CURRENT_LIST_DIR}/include/tesseract_collision/test_suite/* /*/gtest/* /*/bullet/LinearMath/* /*/bullet/BulletCollision/* /*/include/ccd/* /*/include/fcl/*)

add_code_coverage_all_targets(EXCLUDE ${COVERAGE_EXCLUDE})

# Create interface for core
add_library(${PROJECT_NAME}_core INTERFACE)
target_link_libraries(${PROJECT_NAME}_core INTERFACE tesseract::tesseract_geometry ${Boost_LIBRARIES} ${BULLET_LIBRARIES})
tesseract_target_compile_options(${PROJECT_NAME}_core INTERFACE)
tesseract_clang_tidy(${PROJECT_NAME}_core)
tesseract_code_coverage(${PROJECT_NAME}_core ALL EXCLUDE ${COVERAGE_EXCLUDE})
target_compile_definitions(${PROJECT_NAME}_core INTERFACE "-DBT_USE_DOUBLE_PRECISION")
target_include_directories(${PROJECT_NAME}_core INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_core SYSTEM INTERFACE
    ${EIGEN3_INCLUDE_DIRS}
    ${BULLET_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    $<TARGET_PROPERTY:tesseract::tesseract_common,INTERFACE_INCLUDE_DIRECTORIES>) #tesseract::tesseract_common Due to bug in catkin, there is an open PR

# Create target for Bullet implementation
add_library(${PROJECT_NAME}_bullet SHARED
  src/bullet/bullet_cast_bvh_manager.cpp
  src/bullet/bullet_cast_simple_manager.cpp
  src/bullet/bullet_discrete_bvh_manager.cpp
  src/bullet/bullet_discrete_simple_manager.cpp
  src/bullet/bullet_utils.cpp
  src/bullet/tesseract_compound_collision_algorithm.cpp
  src/bullet/tesseract_compound_compound_collision_algorithm.cpp
  src/bullet/tesseract_collision_configuration.cpp
)
target_link_libraries(${PROJECT_NAME}_bullet PUBLIC ${PROJECT_NAME}_core tesseract::tesseract_geometry console_bridge ${OCTOMAP_LIBRARIES})
tesseract_target_compile_options(${PROJECT_NAME}_bullet PUBLIC)
tesseract_clang_tidy(${PROJECT_NAME}_bullet)
tesseract_code_coverage(${PROJECT_NAME}_bullet ALL EXCLUDE ${COVERAGE_EXCLUDE})
target_include_directories(${PROJECT_NAME}_bullet PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_bullet SYSTEM PUBLIC
    ${EIGEN3_INCLUDE_DIRS}
    ${BULLET_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${OCTOMAP_INCLUDE_DIRS})

# Create target for FCL implementation
add_library(${PROJECT_NAME}_fcl SHARED src/fcl/fcl_discrete_managers.cpp src/fcl/fcl_utils.cpp)
target_link_libraries(${PROJECT_NAME}_fcl PUBLIC ${PROJECT_NAME}_core tesseract::tesseract_geometry console_bridge ${OCTOMAP_LIBRARIES} fcl)
tesseract_target_compile_options(${PROJECT_NAME}_fcl PUBLIC)
tesseract_clang_tidy(${PROJECT_NAME}_fcl)
tesseract_code_coverage(${PROJECT_NAME}_fcl ALL EXCLUDE ${COVERAGE_EXCLUDE})
target_include_directories(${PROJECT_NAME}_fcl PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_fcl SYSTEM PUBLIC
    ${EIGEN3_INCLUDE_DIRS}
    ${BULLET_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${OCTOMAP_INCLUDE_DIRS})

# Create target for creating convex hull's from meshes
add_executable(create_convex_hull src/create_convex_hull.cpp)
target_link_libraries(create_convex_hull PUBLIC tesseract::tesseract_geometry console_bridge ${BULLET_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES})
tesseract_target_compile_options(create_convex_hull PUBLIC)
tesseract_clang_tidy(create_convex_hull)
target_compile_definitions(create_convex_hull PRIVATE "-DBT_USE_DOUBLE_PRECISION")
target_include_directories(create_convex_hull PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
target_include_directories(create_convex_hull SYSTEM PUBLIC
    ${EIGEN3_INCLUDE_DIRS}
    ${BULLET_INCLUDE_DIRS}
    ${LIBFCL_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${OCTOMAP_INCLUDE_DIRS})

# Create test suite interface
add_library(${PROJECT_NAME}_test_suite INTERFACE)
target_link_libraries(${PROJECT_NAME}_test_suite INTERFACE tesseract::tesseract_geometry ${PROJECT_NAME}_core)
tesseract_target_compile_options(${PROJECT_NAME}_test_suite INTERFACE)
tesseract_clang_tidy(${PROJECT_NAME}_test_suite)
tesseract_code_coverage(${PROJECT_NAME}_test_suite ALL EXCLUDE ${COVERAGE_EXCLUDE})
target_compile_definitions(${PROJECT_NAME}_test_suite INTERFACE TEST_SUITE_DATA_DIR="${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/test_suite/data")
target_include_directories(${PROJECT_NAME}_test_suite INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_test_suite SYSTEM INTERFACE
    ${EIGEN3_INCLUDE_DIRS}
    $<TARGET_PROPERTY:tesseract::tesseract_common,INTERFACE_INCLUDE_DIRECTORIES>) #tesseract::tesseract_common Due to bug in catkin, there is an open PR

tesseract_configure_package(${PROJECT_NAME}_core ${PROJECT_NAME}_bullet ${PROJECT_NAME}_fcl ${PROJECT_NAME}_test_suite create_convex_hull)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
 )

install(DIRECTORY include/${PROJECT_NAME}/test_suite/data/
    DESTINATION share/${PROJECT_NAME}/test_suite/data)

# Create an ament_index resource file to allow ROS2 ament_index_cpp to locate the installed path to this package.
# This is a workaround to let the ROS2 version of pluginlib find tesseract_collision's plugins, since tesseract_collision is a non-ROS CMake package.
# ADDITIONAL REQUIREMENT: The installed path must be added to the AMENT_PREFIX_PATH environment variable at runtime, which is outside the scope of CMakeLists.txt.
file(WRITE ${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages/${PROJECT_NAME} "")

# ADD Examples
add_subdirectory(examples)

if (TESSERACT_ENABLE_TESTING)
  enable_testing()
  tesseract_add_run_tests_target()
  add_subdirectory(test)
endif()
