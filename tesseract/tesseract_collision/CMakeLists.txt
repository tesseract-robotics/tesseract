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

# Create interface for core
add_library(${PROJECT_NAME}_core INTERFACE)
target_link_libraries(${PROJECT_NAME}_core INTERFACE tesseract::tesseract_geometry ${Boost_LIBRARIES} ${BULLET_LIBRARIES})
tesseract_target_compile_options(${PROJECT_NAME}_core INTERFACE)
tesseract_clang_tidy(${PROJECT_NAME}_core)
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
)
target_link_libraries(${PROJECT_NAME}_bullet PUBLIC ${PROJECT_NAME}_core tesseract::tesseract_geometry console_bridge ${OCTOMAP_LIBRARIES})
tesseract_target_compile_options(${PROJECT_NAME}_bullet PUBLIC)
tesseract_clang_tidy(${PROJECT_NAME}_bullet)
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
target_include_directories(create_convex_hull PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
target_include_directories(create_convex_hull SYSTEM PUBLIC
    ${EIGEN3_INCLUDE_DIRS}
    ${BULLET_INCLUDE_DIRS}
    ${LIBFCL_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${OCTOMAP_INCLUDE_DIRS})

tesseract_configure_package(${PROJECT_NAME}_core ${PROJECT_NAME}_bullet ${PROJECT_NAME}_fcl create_convex_hull)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
 )

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
