cmake_minimum_required(VERSION 3.5.0)
project(tesseract VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_VISIBILITY_PRESET hidden)
set(CMAKE_VISIBILITY_INLINES_HIDDEN 1)

find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS system REQUIRED)
find_package(console_bridge REQUIRED)
find_package(tesseract_scene_graph REQUIRED)
find_package(tesseract_collision REQUIRED)
find_package(tesseract_environment REQUIRED)
find_package(tesseract_kinematics REQUIRED)
find_package(tesseract_common REQUIRED)
find_package(tesseract_urdf REQUIRED)
find_package(tesseract_command_language REQUIRED)
find_package(cmake_common_scripts REQUIRED)

set(COVERAGE_EXCLUDE /usr/* /opt/* ${CMAKE_CURRENT_LIST_DIR}/test/* /*/gtest/*)

add_code_coverage_all_targets(EXCLUDE ${COVERAGE_EXCLUDE})

# Load variable for clang tidy args, compiler options and cxx version
tesseract_variables()

# Create target
add_library(${PROJECT_NAME} src/tesseract.cpp src/manipulator_manager.cpp)
target_process_export(${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME} PUBLIC
  tesseract::tesseract_urdf
  tesseract::tesseract_scene_graph
  tesseract::tesseract_collision_bullet
  tesseract::tesseract_environment_kdl
  tesseract::tesseract_environment_ofkt
  tesseract::tesseract_kinematics_kdl
  tesseract::tesseract_kinematics_opw
  tesseract::tesseract_command_language
  console_bridge)
target_compile_options(${PROJECT_NAME} PRIVATE ${TESSERACT_COMPILE_OPTIONS})
target_compile_definitions(${PROJECT_NAME} PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME} ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME} PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(${PROJECT_NAME} ALL EXCLUDE ${COVERAGE_EXCLUDE} ENABLE ${TESSERACT_ENABLE_TESTING})
target_include_directories(${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
    ${Boost_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS})

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

if (TESSERACT_ENABLE_BENCHMARKING)
  add_subdirectory(test/benchmarks)
endif()

