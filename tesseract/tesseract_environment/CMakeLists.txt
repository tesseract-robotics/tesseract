cmake_minimum_required(VERSION 3.5.0)
project(tesseract_environment VERSION 0.1.0 LANGUAGES CXX)

if(WIN32)
  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(console_bridge REQUIRED)
find_package(tesseract_collision REQUIRED)
find_package(tesseract_kinematics REQUIRED)
find_package(tesseract_scene_graph REQUIRED)
find_package(tesseract_common REQUIRED)
find_package(cmake_common_scripts REQUIRED)

if(NOT TARGET console_bridge::console_bridge)
  add_library(console_bridge::console_bridge INTERFACE IMPORTED)
  set_target_properties(console_bridge::console_bridge PROPERTIES INTERFACE_INCLUDE_DIRECTORIES ${console_bridge_INCLUDE_DIRS})
  set_target_properties(console_bridge::console_bridge PROPERTIES INTERFACE_LINK_LIBRARIES ${console_bridge_LIBRARIES})
else()
  get_target_property(CHECK_INCLUDE_DIRECTORIES console_bridge::console_bridge INTERFACE_INCLUDE_DIRECTORIES)
  if (NOT ${CHECK_INCLUDE_DIRECTORIES})
    set_target_properties(console_bridge::console_bridge PROPERTIES INTERFACE_INCLUDE_DIRECTORIES ${console_bridge_INCLUDE_DIRS})
  endif()
endif()

set(COVERAGE_EXCLUDE /usr/* /opt/* ${CMAKE_CURRENT_LIST_DIR}/test/* /*/gtest/* /*/bullet/LinearMath/* /*/bullet/BulletCollision/*)

add_code_coverage_all_targets(EXCLUDE ${COVERAGE_EXCLUDE})

# Load variable for clang tidy args, compiler options and cxx version
tesseract_variables()

# Create interface for core
add_library(${PROJECT_NAME}_core src/core/environment.cpp src/core/manipulator_manager.cpp)
target_link_libraries(${PROJECT_NAME}_core PUBLIC
  tesseract::tesseract_common
  tesseract::tesseract_collision_core
  tesseract::tesseract_scene_graph
  tesseract::tesseract_kinematics_kdl
  tesseract::tesseract_kinematics_opw)
target_compile_options(${PROJECT_NAME}_core PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(${PROJECT_NAME}_core PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_core PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_core ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_core PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(${PROJECT_NAME}_core ALL EXCLUDE ${COVERAGE_EXCLUDE} ENABLE ${TESSERACT_ENABLE_TESTING})
target_include_directories(${PROJECT_NAME}_core PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_core SYSTEM PUBLIC
    ${EIGEN3_INCLUDE_DIRS})

add_library(${PROJECT_NAME}_kdl src/kdl/kdl_state_solver.cpp)
target_link_libraries(${PROJECT_NAME}_kdl PUBLIC ${PROJECT_NAME}_core tesseract::tesseract_scene_graph orocos-kdl console_bridge::console_bridge)
target_compile_options(${PROJECT_NAME}_kdl PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(${PROJECT_NAME}_kdl PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_kdl PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_kdl ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_kdl PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(${PROJECT_NAME}_kdl ALL EXCLUDE ${COVERAGE_EXCLUDE} ENABLE ${TESSERACT_ENABLE_TESTING})
target_include_directories(${PROJECT_NAME}_kdl PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_kdl SYSTEM PUBLIC
    ${EIGEN3_INCLUDE_DIRS}
    ${orocos_kdl_INCLUDE_DIRS}) # Orocos did not export target correctly

add_library(${PROJECT_NAME}_ofkt src/ofkt/ofkt_state_solver.cpp src/ofkt/ofkt_nodes.cpp)
target_link_libraries(${PROJECT_NAME}_ofkt PUBLIC ${PROJECT_NAME}_core tesseract::tesseract_scene_graph console_bridge::console_bridge)
target_compile_options(${PROJECT_NAME}_ofkt PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(${PROJECT_NAME}_ofkt PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_ofkt PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_ofkt ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_ofkt PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(${PROJECT_NAME}_ofkt ALL EXCLUDE ${COVERAGE_EXCLUDE} ENABLE ${TESSERACT_ENABLE_TESTING})
target_include_directories(${PROJECT_NAME}_ofkt PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_ofkt SYSTEM PUBLIC
    ${EIGEN3_INCLUDE_DIRS})

  configure_package(NAMESPACE tesseract TARGETS ${PROJECT_NAME}_core ${PROJECT_NAME}_kdl ${PROJECT_NAME}_ofkt)

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
