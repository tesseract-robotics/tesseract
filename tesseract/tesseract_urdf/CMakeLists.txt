cmake_minimum_required(VERSION 3.5.0)
project(tesseract_urdf VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_VISIBILITY_PRESET hidden)
set(CMAKE_VISIBILITY_INLINES_HIDDEN 1)

find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(console_bridge REQUIRED)
find_package(PCL REQUIRED)
find_package(tesseract_geometry REQUIRED)
find_package(tesseract_common REQUIRED)
find_package(tesseract_scene_graph REQUIRED)
find_package(tesseract_collision REQUIRED)
find_package(cmake_common_scripts REQUIRED)

set(COVERAGE_EXCLUDE /usr/* /opt/* ${CMAKE_CURRENT_LIST_DIR}/test/* /*/gtest/* /*/bullet/LinearMath/*)

add_code_coverage_all_targets(EXCLUDE ${COVERAGE_EXCLUDE})

# Load variable for clang tidy args, compiler options and cxx version
tesseract_variables()

add_library(${PROJECT_NAME} src/urdf_parser.cpp)
target_process_export(${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME} PUBLIC tesseract::tesseract_common tesseract::tesseract_geometry tesseract::tesseract_scene_graph tesseract::tesseract_collision_core console_bridge ${Boost_LIBRARIES})
target_compile_options(${PROJECT_NAME} PRIVATE ${TESSERACT_COMPILE_OPTIONS})
target_compile_definitions(${PROJECT_NAME} PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME} ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME} PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(${PROJECT_NAME} ALL EXCLUDE ${COVERAGE_EXCLUDE} ENABLE ${TESSERACT_ENABLE_TESTING})

# PCL does not support c++11 on Xenial so cannot include point cloud parsing from urdf
# Boost version number is in XYYYZZ format such that:
#   (BOOST_VERSION % 100) is the sub-minor version
#   ((BOOST_VERSION / 100) % 1000) is the minor version
#   (BOOST_VERSION / 100000) is the major version.
if(Boost_VERSION VERSION_GREATER "106000")
    target_link_libraries(${PROJECT_NAME} PUBLIC ${PCL_LIBRARIES})
    target_compile_definitions(${PROJECT_NAME} PUBLIC TESSERACT_PARSE_POINT_CLOUDS="ON")
    foreach(DEF ${PCL_DEFINITIONS})
      string(STRIP ${DEF} DEF)
      if (NOT "${DEF}" STREQUAL "")
         string(SUBSTRING "${DEF}" 0 2 DEF_PREFIX)
         if ("${DEF_PREFIX}" STREQUAL "-m")
             string(REPLACE " " ";" DEF ${DEF})
             foreach(OPTION_DEF ${DEF})
                 target_compile_options(${PROJECT_NAME} PUBLIC ${OPTION_DEF})
             endforeach()
         else()
            target_compile_definitions(${PROJECT_NAME} PUBLIC ${DEF})
         endif()
      endif()
    endforeach()
    target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${PCL_INCLUDE_DIRS})
endif()

target_include_directories(${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
    ${EIGEN3_INCLUDE_DIRS}
    ${console_bridge_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS})

configure_package(NAMESPACE tesseract TARGETS ${PROJECT_NAME})

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

# ADD Examples
add_subdirectory(examples)

if (TESSERACT_ENABLE_TESTING)
  enable_testing()
  add_run_tests_target(ENABLE ${TESSERACT_ENABLE_RUN_TESTING})
  add_subdirectory(test)
endif()
