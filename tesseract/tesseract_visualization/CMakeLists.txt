cmake_minimum_required(VERSION 3.5.0)
project(tesseract_visualization VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_VISIBILITY_PRESET hidden)
set(CMAKE_VISIBILITY_INLINES_HIDDEN 1)

find_package(Eigen3 REQUIRED)
find_package(console_bridge REQUIRED)
find_package(tesseract_collision REQUIRED)
find_package(tesseract_common REQUIRED)
find_package(tesseract_environment REQUIRED)
find_package(tesseract_scene_graph REQUIRED)
find_package(tesseract REQUIRED)
find_package(tesseract_command_language REQUIRED)
find_package(class_loader REQUIRED)
find_package(cmake_common_scripts REQUIRED)

# Optional to build ignition visualization
find_package(ignition-common3 COMPONENTS profiler events av QUIET)
find_package(ignition-transport8 QUIET)
find_package(ignition-msgs5 QUIET)
find_package(ignition-math6-eigen3 QUIET)

set(IGNITION_FOUND FALSE)
if (ignition-common3_FOUND AND ignition-transport8_FOUND AND ignition-msgs5_FOUND AND ignition-math6-eigen3_FOUND)
  set(IGNITION_FOUND TRUE)
  message(STATUS "Ignition Visualization Library will be built!")
endif()

set(COVERAGE_EXCLUDE /usr/* /opt/* ${CMAKE_CURRENT_LIST_DIR}/test/* /*/gtest/*)

add_code_coverage_all_targets(EXCLUDE ${COVERAGE_EXCLUDE})

# Load variable for clang tidy args, compiler options and cxx version
tesseract_variables()

#####################################
# Define compile-time default variables
if(MSVC)
  set(TESSERACT_VISUALIZATION_PLUGIN_PATH ${CMAKE_INSTALL_PREFIX}/bin)
else()
  set(TESSERACT_VISUALIZATION_PLUGIN_PATH ${CMAKE_INSTALL_PREFIX}/lib)
endif()

add_library(${PROJECT_NAME}
    src/visualization_loader.cpp
    src/trajectory_interpolator.cpp
    src/trajectory_player.cpp)
target_process_export(${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME} PUBLIC tesseract::tesseract tesseract::tesseract_environment_core tesseract::tesseract_scene_graph tesseract::tesseract_command_language tesseract::tesseract_collision_core tesseract::tesseract_common console_bridge ${class_loader_LIBRARIES})
target_compile_options(${PROJECT_NAME} PRIVATE ${TESSERACT_COMPILE_OPTIONS})
target_compile_definitions(${PROJECT_NAME} PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME} ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME} PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(${PROJECT_NAME} ALL EXCLUDE ${COVERAGE_EXCLUDE} ENABLE ${TESSERACT_ENABLE_TESTING})
target_compile_definitions(${PROJECT_NAME} PRIVATE TESSERACT_VISUALIZATION_PLUGIN_PATH="${TESSERACT_VISUALIZATION_PLUGIN_PATH}")
if(class_loader_VERSION VERSION_LESS "0.4.0")
  target_compile_definitions(${PROJECT_NAME} PRIVATE CLASS_LOADER_NOT_SUPPORTED=ON)
elseif(class_loader_VERSION VERSION_LESS "1.2.0")
  target_compile_definitions(${PROJECT_NAME} PRIVATE CLASS_LOADER_SHARED_INSTANCE=ON)
else()
  target_compile_definitions(${PROJECT_NAME} PRIVATE CLASS_LOADER_INSTANCE=ON)
endif()
target_include_directories(${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
    ${EIGEN3_INCLUDE_DIRS}
    ${class_loader_INCLUDE_DIRS})

list(APPEND PACKAGE_LIBRARIES ${PROJECT_NAME})
if (IGNITION_FOUND)
  add_library(${PROJECT_NAME}_ignition
      src/ignition/entity_manager.cpp
      src/ignition/conversions.cpp)
  target_process_export(${PROJECT_NAME}_ignition)
  target_link_libraries(${PROJECT_NAME}_ignition PUBLIC
    tesseract::tesseract_scene_graph
    tesseract::tesseract_common
    ${IGNITION-COMMON_LIBRARIES}
    ${IGNITION-MSGS_LIBRARIES}
    console_bridge)
  target_compile_options(${PROJECT_NAME}_ignition PRIVATE ${TESSERACT_COMPILE_OPTIONS})
  target_compile_definitions(${PROJECT_NAME}_ignition PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
  target_clang_tidy(${PROJECT_NAME}_ignition ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
  target_cxx_version(${PROJECT_NAME}_ignition PUBLIC VERSION ${TESSERACT_CXX_VERSION})
  target_code_coverage(${PROJECT_NAME}_ignition ALL EXCLUDE ${COVERAGE_EXCLUDE} ENABLE ${TESSERACT_ENABLE_TESTING})
  target_include_directories(${PROJECT_NAME}_ignition PUBLIC
      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
      "$<INSTALL_INTERFACE:include>")
  target_include_directories(${PROJECT_NAME}_ignition SYSTEM PUBLIC
    ${IGNITION-COMMON_INCLUDE_DIRS}
    ${IGNITION-MSGS_INCLUDE_DIRS})

  add_library(${PROJECT_NAME}_ignition_visualization
      src/ignition/tesseract_ignition_visualization.cpp)
  target_process_export(${PROJECT_NAME}_ignition_visualization)
  target_link_libraries(${PROJECT_NAME}_ignition_visualization PUBLIC
    ${PROJECT_NAME}
    ${PROJECT_NAME}_ignition
    tesseract::tesseract_scene_graph
    tesseract::tesseract_common
    tesseract::tesseract_command_language
    ${IGNITION-COMMON_LIBRARIES}
    ${IGNITION-TRANSPORT_LIBRARIES}
    ${IGNITION-MSGS_LIBRARIES}
    console_bridge)
  target_compile_options(${PROJECT_NAME}_ignition_visualization PRIVATE ${TESSERACT_COMPILE_OPTIONS})
  target_compile_definitions(${PROJECT_NAME}_ignition_visualization PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
  target_clang_tidy(${PROJECT_NAME}_ignition_visualization ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
  target_cxx_version(${PROJECT_NAME}_ignition_visualization PUBLIC VERSION ${TESSERACT_CXX_VERSION})
  target_code_coverage(${PROJECT_NAME}_ignition_visualization ALL EXCLUDE ${COVERAGE_EXCLUDE} ENABLE ${TESSERACT_ENABLE_TESTING})
  target_include_directories(${PROJECT_NAME}_ignition_visualization PUBLIC
      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
      "$<INSTALL_INTERFACE:include>")
  target_include_directories(${PROJECT_NAME}_ignition_visualization SYSTEM PUBLIC
    ${IGNITION-COMMON_INCLUDE_DIRS}
    ${IGNITION-TRANSPORT_INCLUDE_DIRS}
    ${IGNITION-MSGS_INCLUDE_DIRS})

  add_library(${PROJECT_NAME}_ignition_visualization_plugin
      src/ignition/tesseract_ignition_visualization_plugin.cpp)
  target_link_libraries(${PROJECT_NAME}_ignition_visualization_plugin PUBLIC
    ${PROJECT_NAME}_ignition_visualization ${class_loader_LIBRARIES})
  target_compile_options(${PROJECT_NAME}_ignition_visualization_plugin PRIVATE ${TESSERACT_COMPILE_OPTIONS})
  target_compile_definitions(${PROJECT_NAME}_ignition_visualization_plugin PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
  target_cxx_version(${PROJECT_NAME}_ignition_visualization_plugin PUBLIC VERSION ${TESSERACT_CXX_VERSION})
  target_include_directories(${PROJECT_NAME}_ignition_visualization_plugin PUBLIC
      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
      "$<INSTALL_INTERFACE:include>")
  target_include_directories(${PROJECT_NAME}_ignition_visualization SYSTEM PUBLIC
    ${class_loader_INCLUDE_DIRS})

  list(APPEND PACKAGE_LIBRARIES ${PROJECT_NAME}_ignition ${PROJECT_NAME}_ignition_visualization ${PROJECT_NAME}_ignition_visualization_plugin)
else()
  message(WARNING "Ignition Visualization Library will not be built!")
endif()

configure_package(NAMESPACE tesseract TARGETS ${PACKAGE_LIBRARIES})

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
