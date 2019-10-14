cmake_minimum_required(VERSION 3.5.0)
project(tesseract_process_planners VERSION 0.1.0 LANGUAGES CXX)

find_package(tesseract_common REQUIRED)
find_package(tesseract_motion_planners REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/${PROJECT_NAME}/process_definition.cpp
  src/${PROJECT_NAME}/process_segment_definition.cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC tesseract::tesseract_common tesseract::tesseract_motion_planners_core)
tesseract_target_compile_options(${PROJECT_NAME} PUBLIC)
target_include_directories(${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
 	${EIGEN3_INCLUDE_DIRS})
   
tesseract_configure_package(${PROJECT_NAME})

# Mark header files for installation
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

if (ENABLE_TESTS)
  enable_testing()
  add_custom_target(run_tests ALL
      WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
      COMMAND ${CMAKE_CTEST_COMMAND} -V -O "/tmp/${PROJECT_NAME}_ctest.log" -C $<CONFIGURATION>)

  add_subdirectory(test)
endif()

