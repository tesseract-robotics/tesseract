cmake_minimum_required(VERSION 3.5.0)
project(tesseract_command_language VERSION 0.1.0 LANGUAGES CXX)

find_package(tesseract_common REQUIRED)
find_package(tesseract_motion_planners REQUIRED)
find_package(tesseract REQUIRED)
find_package(Eigen3 REQUIRED)

set(COVERAGE_EXCLUDE /usr/* /opt/* ${CMAKE_CURRENT_LIST_DIR}/test/* /*/gtest/*)

add_code_coverage_all_targets(EXCLUDE ${COVERAGE_EXCLUDE})

add_library(${PROJECT_NAME} SHARED
  src/plan_instruction.cpp
  src/move_instruction.cpp
  src/composite_instruction.cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC tesseract::tesseract_common)
tesseract_target_compile_options(${PROJECT_NAME} PUBLIC)
tesseract_clang_tidy(${PROJECT_NAME})
tesseract_code_coverage(${PROJECT_NAME} ALL EXCLUDE ${COVERAGE_EXCLUDE})
target_include_directories(${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
 	${EIGEN3_INCLUDE_DIRS})

add_library(${PROJECT_NAME}_trajopt src/trajopt_planner_universal_config.cpp)
target_link_libraries(${PROJECT_NAME}_trajopt PUBLIC ${PROJECT_NAME} tesseract::tesseract_motion_planners_trajopt tesseract::tesseract)
tesseract_target_compile_options(${PROJECT_NAME}_trajopt PUBLIC)
tesseract_clang_tidy(${PROJECT_NAME}_trajopt)
tesseract_code_coverage(${PROJECT_NAME}_trajopt ALL EXCLUDE ${COVERAGE_EXCLUDE})
target_include_directories(${PROJECT_NAME}_trajopt PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_trajopt SYSTEM PUBLIC
        ${EIGEN3_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_demo src/example_program.cpp)
target_link_libraries(${PROJECT_NAME}_demo PUBLIC ${PROJECT_NAME} ${PROJECT_NAME}_trajopt)
tesseract_target_compile_options(${PROJECT_NAME}_demo PUBLIC)
tesseract_clang_tidy(${PROJECT_NAME}_demo)
tesseract_code_coverage(${PROJECT_NAME}_demo ALL EXCLUDE ${COVERAGE_EXCLUDE})
target_include_directories(${PROJECT_NAME}_demo PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_demo SYSTEM PUBLIC
        ${EIGEN3_INCLUDE_DIRS})  


find_package(tesseract_support REQUIRED)
find_package(tesseract REQUIRED)
add_executable(${PROJECT_NAME}_trajopt_demo src/example_trajopt.cpp)
target_link_libraries(${PROJECT_NAME}_trajopt_demo PUBLIC ${PROJECT_NAME} ${PROJECT_NAME}_trajopt tesseract::tesseract_support tesseract::tesseract)
tesseract_target_compile_options(${PROJECT_NAME}_trajopt_demo PUBLIC)
tesseract_clang_tidy(${PROJECT_NAME}_trajopt_demo)
tesseract_code_coverage(${PROJECT_NAME}_trajopt_demo ALL EXCLUDE ${COVERAGE_EXCLUDE})
target_include_directories(${PROJECT_NAME}_trajopt_demo PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_trajopt_demo SYSTEM PUBLIC
        ${EIGEN3_INCLUDE_DIRS})
   
tesseract_configure_package(${PROJECT_NAME} ${PROJECT_NAME}_trajopt ${PROJECT_NAME}_demo ${PROJECT_NAME}_trajopt_demo )

# Mark header files for installation
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

#if (TESSERACT_ENABLE_TESTING)
#  enable_testing()
#  tesseract_add_run_tests_target()
#  add_subdirectory(test)
#endif()

