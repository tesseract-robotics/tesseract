cmake_minimum_required(VERSION 3.5.0)
project(tesseract_process_managers VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_VISIBILITY_PRESET hidden)
set(CMAKE_VISIBILITY_INLINES_HIDDEN 1)

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)
find_package(Taskflow REQUIRED)
find_package(console_bridge REQUIRED)
find_package(tesseract_common REQUIRED)
find_package(tesseract REQUIRED)
find_package(tesseract_support REQUIRED)
find_package(tesseract_command_language REQUIRED)
find_package(tesseract_motion_planners REQUIRED)
find_package(tesseract_time_parameterization REQUIRED)
find_package(cmake_common_scripts REQUIRED)

set(COVERAGE_EXCLUDE /usr/* /opt/* ${CMAKE_CURRENT_LIST_DIR}/test/* /*/gtest/*)
add_code_coverage_all_targets(EXCLUDE ${COVERAGE_EXCLUDE})

# Load variable for clang tidy args, compiler options and cxx version
tesseract_variables()

include(GenerateExportHeader)

# Create interface
add_library(${PROJECT_NAME}
    src/process_input.cpp
    src/debug_observer.cpp
    src/process_generators/continuous_contact_check_process_generator.cpp
    src/process_generators/discrete_contact_check_process_generator.cpp
    src/process_generators/fix_state_bounds_process_generator.cpp
    src/process_generators/fix_state_collision_process_generator.cpp
    src/process_generators/iterative_spline_parameterization_process_generator.cpp
    src/process_generators/motion_planner_process_generator.cpp
    src/process_managers/raster_process_manager.cpp
    src/process_managers/raster_dt_process_manager.cpp
    src/process_managers/raster_waad_process_manager.cpp
    src/process_managers/raster_waad_dt_process_manager.cpp
    src/process_managers/simple_process_manager.cpp
    src/taskflow_generators/sequential_taskflow.cpp
    src/taskflow_generators/graph_taskflow.cpp
    src/taskflows/descartes_taskflow.cpp
    src/taskflows/ompl_taskflow.cpp
    src/taskflows/trajopt_taskflow.cpp
    src/taskflows/cartesian_taskflow.cpp
    src/taskflows/freespace_taskflow.cpp)
target_process_export(${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME} PUBLIC
    console_bridge
    Taskflow::Taskflow
    tesseract::tesseract_common
    tesseract::tesseract_command_language
    tesseract::tesseract_motion_planners_simple
    tesseract::tesseract_motion_planners_trajopt
    tesseract::tesseract_motion_planners_descartes
    tesseract::tesseract_motion_planners_ompl
    tesseract::tesseract_time_parameterization)
target_compile_options(${PROJECT_NAME} PRIVATE ${TESSERACT_COMPILE_OPTIONS})
target_compile_definitions(${PROJECT_NAME} PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME} ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME} PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(${PROJECT_NAME} ALL EXCLUDE ${COVERAGE_EXCLUDE} ENABLE ${TESSERACT_ENABLE_TESTING})
target_include_directories(${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
    ${EIGEN3_INCLUDE_DIRS})

add_subdirectory(examples)

configure_package(NAMESPACE tesseract TARGETS ${PROJECT_NAME})

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

if (TESSERACT_ENABLE_TESTING)
  enable_testing()
  add_run_tests_target(ENABLE ${TESSERACT_ENABLE_RUN_TESTING})
  add_subdirectory(test)
endif()
