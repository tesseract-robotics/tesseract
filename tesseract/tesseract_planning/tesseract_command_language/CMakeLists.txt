cmake_minimum_required(VERSION 3.5.0)
project(tesseract_command_language VERSION 0.1.0 LANGUAGES CXX)

find_package(console_bridge REQUIRED)
find_package(tesseract_common REQUIRED)
find_package(Eigen3 REQUIRED)

set(COVERAGE_EXCLUDE /usr/* /opt/* ${CMAKE_CURRENT_LIST_DIR}/test/* /*/gtest/*)

add_code_coverage_all_targets(EXCLUDE ${COVERAGE_EXCLUDE})

add_library(${PROJECT_NAME} SHARED
  src/plan_instruction.cpp
  src/move_instruction.cpp
  src/composite_instruction.cpp
  src/command_language_utils.cpp
  src/instruction_type.cpp
  src/waypoint_type.cpp
  src/serialize.cpp
  src/deserialize.cpp
  src/manipulator_info.cpp
  src/null_waypoint.cpp
  src/state_waypoint.cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC console_bridge tesseract::tesseract_common)
tesseract_target_compile_options(${PROJECT_NAME} PUBLIC)
tesseract_clang_tidy(${PROJECT_NAME})
tesseract_code_coverage(${PROJECT_NAME} ALL EXCLUDE ${COVERAGE_EXCLUDE})
target_include_directories(${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
 	${EIGEN3_INCLUDE_DIRS})
   
tesseract_configure_package(${PROJECT_NAME}) # ${PROJECT_NAME}_trajopt ${PROJECT_NAME}_demo ${PROJECT_NAME}_trajopt_demo )

# Mark header files for installation
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

