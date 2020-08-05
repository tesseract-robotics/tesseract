cmake_minimum_required(VERSION 3.5.0)
project(tesseract_process_managers VERSION 0.1.0 LANGUAGES CXX)

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)
find_package(Taskflow REQUIRED)
find_package(console_bridge REQUIRED)
find_package(tesseract_common REQUIRED)
find_package(tesseract REQUIRED)
find_package(tesseract_support REQUIRED)
find_package(tesseract_command_language REQUIRED)
find_package(tesseract_motion_planners REQUIRED)

set(COVERAGE_EXCLUDE /usr/* /opt/* ${CMAKE_CURRENT_LIST_DIR}/test/* /*/gtest/*)
add_code_coverage_all_targets(EXCLUDE ${COVERAGE_EXCLUDE})

# Create interface
add_library(${PROJECT_NAME} SHARED
    src/process_input.cpp
    src/process_generators/random_process_generator.cpp
    src/process_generators/motion_planner_process_generator.cpp
    src/process_managers/freespace_process_manager.cpp
    src/process_managers/raster_process_manager.cpp
    src/taskflow_generators/sequential_failure_tree_taskflow.cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC
    console_bridge
    Taskflow::Taskflow
    tesseract::tesseract_command_language
    tesseract::tesseract_motion_planners_simple
    tesseract::tesseract_motion_planners_trajopt
    tesseract::tesseract_motion_planners_descartes
    tesseract::tesseract_motion_planners_ompl)
tesseract_target_compile_options(${PROJECT_NAME} PUBLIC)
tesseract_clang_tidy(${PROJECT_NAME})
tesseract_code_coverage(${PROJECT_NAME} ALL EXCLUDE ${COVERAGE_EXCLUDE})
target_include_directories(${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
    ${EIGEN3_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_freespace_manager_example src/examples/freespace_manager_example.cpp)
target_link_libraries(${PROJECT_NAME}_freespace_manager_example console_bridge ${PROJECT_NAME} Taskflow::Taskflow tesseract::tesseract tesseract::tesseract_command_language tesseract::tesseract_support tesseract::tesseract_motion_planners_trajopt ${CMAKE_THREAD_LIBS_INIT})
tesseract_target_compile_options(${PROJECT_NAME}_freespace_manager_example PRIVATE)
tesseract_clang_tidy(${PROJECT_NAME}_freespace_manager_example)
target_include_directories(${PROJECT_NAME}_freespace_manager_example PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_freespace_manager_example SYSTEM PRIVATE
    ${EIGEN3_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_raster_manager_example src/examples/raster_manager_example.cpp)
target_link_libraries(${PROJECT_NAME}_raster_manager_example console_bridge ${PROJECT_NAME} Taskflow::Taskflow tesseract::tesseract tesseract::tesseract_command_language tesseract::tesseract_support tesseract::tesseract_motion_planners_trajopt ${CMAKE_THREAD_LIBS_INIT})
tesseract_target_compile_options(${PROJECT_NAME}_raster_manager_example PRIVATE)
tesseract_clang_tidy(${PROJECT_NAME}_raster_manager_example)
target_include_directories(${PROJECT_NAME}_raster_manager_example PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_raster_manager_example SYSTEM PRIVATE
    ${EIGEN3_INCLUDE_DIRS})

tesseract_configure_package(${PROJECT_NAME} ${PROJECT_NAME}_freespace_manager_example ${PROJECT_NAME}_raster_manager_example)

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
