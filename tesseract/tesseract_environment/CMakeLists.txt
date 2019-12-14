cmake_minimum_required(VERSION 3.5.0)
project(tesseract_environment VERSION 0.1.0 LANGUAGES CXX)

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(console_bridge REQUIRED)
find_package(tesseract_collision REQUIRED)
find_package(tesseract_scene_graph REQUIRED)
find_package(tesseract_common REQUIRED)

# Create interface for core
add_library(${PROJECT_NAME}_core SHARED src/core/environment.cpp)
target_link_libraries(${PROJECT_NAME}_core PUBLIC tesseract::tesseract_collision_core tesseract::tesseract_scene_graph)
tesseract_target_compile_options(${PROJECT_NAME}_core PUBLIC)
tesseract_clang_tidy(${PROJECT_NAME}_core)
target_include_directories(${PROJECT_NAME}_core PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_core SYSTEM PUBLIC
    ${EIGEN3_INCLUDE_DIRS}
    $<TARGET_PROPERTY:tesseract::tesseract_common,INTERFACE_INCLUDE_DIRECTORIES>) #tesseract::tesseract_common Due to bug in catkin, there is an open PR

add_library(${PROJECT_NAME}_kdl SHARED src/kdl/kdl_env.cpp src/kdl/kdl_state_solver.cpp)
target_link_libraries(${PROJECT_NAME}_kdl PUBLIC ${PROJECT_NAME}_core tesseract::tesseract_collision_fcl tesseract::tesseract_collision_bullet tesseract::tesseract_scene_graph orocos-kdl console_bridge ${BULLET_LIBRARIES})
tesseract_target_compile_options(${PROJECT_NAME}_kdl PUBLIC)
tesseract_clang_tidy(${PROJECT_NAME}_kdl)
target_include_directories(${PROJECT_NAME}_kdl PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_kdl SYSTEM PUBLIC
    ${EIGEN3_INCLUDE_DIRS}
    ${BULLET_INCLUDE_DIRS}
    ${orocos_kdl_INCLUDE_DIRS}) # Orocos did not export target correctly

tesseract_configure_package(${PROJECT_NAME}_core ${PROJECT_NAME}_kdl)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

if (TESSERACT_ENABLE_TESTING)
  enable_testing()
  tesseract_add_run_tests_target()
  add_subdirectory(test)
endif()
