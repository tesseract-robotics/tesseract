cmake_minimum_required(VERSION 3.5.0)
project(tesseract_common VERSION 0.1.0 LANGUAGES CXX)
include(cmake/tesseract_macros.cmake)
include(cmake/tesseract_code_coverage.cmake)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/")

find_package(Eigen3 REQUIRED)
find_package(TinyXML2 REQUIRED)

set(COVERAGE_EXCLUDE /usr/* /opt/* ${CMAKE_CURRENT_LIST_DIR}/test/* /*/gtest/*)

add_code_coverage_all_targets(EXCLUDE ${COVERAGE_EXCLUDE})

add_library(${PROJECT_NAME} INTERFACE)
target_link_libraries(${PROJECT_NAME} INTERFACE ${TinyXML2_LIBRARIES})
tesseract_target_compile_options(${PROJECT_NAME} INTERFACE)
tesseract_clang_tidy(${PROJECT_NAME})
tesseract_code_coverage(${PROJECT_NAME} ALL EXCLUDE ${COVERAGE_EXCLUDE})
target_include_directories(${PROJECT_NAME} INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME} SYSTEM INTERFACE
    ${EIGEN3_INCLUDE_DIRS}
    ${TinyXML2_INCLUDE_DIRS})

tesseract_configure_package(${PROJECT_NAME})

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(FILES
  "${CMAKE_CURRENT_LIST_DIR}/cmake/tesseract_macros.cmake"
  "${CMAKE_CURRENT_LIST_DIR}/cmake/tesseract_code_coverage.cmake"
  "${CMAKE_CURRENT_LIST_DIR}/cmake/FindTinyXML2.cmake"
  DESTINATION lib/cmake/${PROJECT_NAME})

if (TESSERACT_ENABLE_TESTING)
  enable_testing()
  tesseract_add_run_tests_target()
  add_subdirectory(test)
endif()
