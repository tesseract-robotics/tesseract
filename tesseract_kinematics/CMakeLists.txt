cmake_minimum_required(VERSION 3.5.0)

# Extract package name and version
find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)
project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES CXX)

if(WIN32)
  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

# System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(console_bridge REQUIRED)
find_package(tesseract_scene_graph REQUIRED)
find_package(tesseract_state_solver REQUIRED)
find_package(tesseract_srdf REQUIRED)
find_package(tesseract_common REQUIRED)
find_package(opw_kinematics REQUIRED)
find_package(yaml-cpp REQUIRED)

if(NOT TARGET console_bridge::console_bridge)
  add_library(console_bridge::console_bridge INTERFACE IMPORTED)
  set_target_properties(console_bridge::console_bridge PROPERTIES INTERFACE_INCLUDE_DIRECTORIES
                                                                  ${console_bridge_INCLUDE_DIRS})
  set_target_properties(console_bridge::console_bridge PROPERTIES INTERFACE_LINK_LIBRARIES ${console_bridge_LIBRARIES})
else()
  get_target_property(CHECK_INCLUDE_DIRECTORIES console_bridge::console_bridge INTERFACE_INCLUDE_DIRECTORIES)
  if(NOT ${CHECK_INCLUDE_DIRECTORIES})
    set_target_properties(console_bridge::console_bridge PROPERTIES INTERFACE_INCLUDE_DIRECTORIES
                                                                    ${console_bridge_INCLUDE_DIRS})
  endif()
endif()

# Load variable for clang tidy args, compiler options and cxx version
tesseract_variables()

initialize_code_coverage(ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
set(COVERAGE_EXCLUDE
    /usr/*
    /opt/*
    ${CMAKE_CURRENT_LIST_DIR}/test/*
    /*/gtest/*
    /*/bullet/LinearMath/*)
add_code_coverage_all_targets(EXCLUDE ${COVERAGE_EXCLUDE} ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})

# Define compile-time default variables
if(MSVC)
  set(TESSERACT_KINEMATICS_PLUGIN_PATH ${CMAKE_INSTALL_PREFIX}/bin)
else()
  set(TESSERACT_KINEMATICS_PLUGIN_PATH ${CMAKE_INSTALL_PREFIX}/lib)
endif()

set(KINEMATICS_PLUGINS "")

# Create interface for core
add_library(${PROJECT_NAME}_core
  src/core/rop_inv_kin.cpp
  src/core/rep_inv_kin.cpp
  src/core/kinematic_group.cpp
  src/core/kinematics_plugin_factory.cpp
  )
target_link_libraries(
  ${PROJECT_NAME}_core
  PUBLIC Eigen3::Eigen
         tesseract::tesseract_common
         tesseract::tesseract_scene_graph
         tesseract::tesseract_state_solver_kdl
         tesseract::tesseract_srdf
         console_bridge::console_bridge
         yaml-cpp)
target_compile_options(${PROJECT_NAME}_core PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(${PROJECT_NAME}_core PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_core PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_compile_definitions(${PROJECT_NAME}_core
                           PRIVATE TESSERACT_KINEMATICS_PLUGIN_PATH="${TESSERACT_KINEMATICS_PLUGIN_PATH}")
target_clang_tidy(${PROJECT_NAME}_core ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_core PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}_core
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(${PROJECT_NAME}_core PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                       "$<INSTALL_INTERFACE:include>")

# Add KDL kinematics factories
add_library(
 ${PROJECT_NAME}_core_factories
 src/core/rop_factory.cpp
 src/core/rep_factory.cpp
 )
target_link_libraries(
 ${PROJECT_NAME}_core_factories
 PUBLIC ${PROJECT_NAME}_core
        console_bridge::console_bridge)
target_compile_options(${PROJECT_NAME}_core_factories PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(${PROJECT_NAME}_core_factories PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_core_factories PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_core_factories ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_core_factories PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
 ${PROJECT_NAME}_core_factories
 ALL
 EXCLUDE ${COVERAGE_EXCLUDE}
 ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(${PROJECT_NAME}_core_factories PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                       "$<INSTALL_INTERFACE:include>")


# Add factory library so kinematic_factory can find these factories by defauult
list(APPEND KINEMATICS_PLUGINS "${PROJECT_NAME}_core_factories")

# Create ikfast kinematics interface
add_library(${PROJECT_NAME}_ikfast INTERFACE)
target_link_libraries(${PROJECT_NAME}_ikfast INTERFACE ${PROJECT_NAME}_core Eigen3::Eigen
                                                       console_bridge::console_bridge)
target_compile_definitions(${PROJECT_NAME}_ikfast INTERFACE -DIKFAST_NO_MAIN -DIKFAST_CLIBRARY -DIKFAST_HAS_LIBRARY)
target_compile_options(${PROJECT_NAME}_ikfast INTERFACE ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_ikfast INTERFACE ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_ikfast ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_ikfast INTERFACE VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}_ikfast
  INTERFACE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(${PROJECT_NAME}_ikfast INTERFACE "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                            "$<INSTALL_INTERFACE:include>")

# Add KDL implementation of tesseract_kinematics
add_library(
  ${PROJECT_NAME}_kdl
  src/kdl/kdl_fwd_kin_chain.cpp
  src/kdl/kdl_fwd_kin_tree.cpp
  src/kdl/kdl_inv_kin_chain_lma.cpp
  src/kdl/kdl_inv_kin_chain_nr.cpp
  src/kdl/kdl_utils.cpp)
target_link_libraries(
  ${PROJECT_NAME}_kdl
  PUBLIC ${PROJECT_NAME}_core
         Eigen3::Eigen
         tesseract::tesseract_scene_graph
         tesseract::tesseract_common
         orocos-kdl
         console_bridge::console_bridge)
target_compile_options(${PROJECT_NAME}_kdl PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(${PROJECT_NAME}_kdl PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_kdl PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_kdl ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_kdl PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}_kdl
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
if(orocos_kdl_VERSION VERSION_LESS "1.4.0")
  target_compile_definitions(${PROJECT_NAME}_kdl PUBLIC KDL_LESS_1_4_0=ON)
endif()
target_include_directories(${PROJECT_NAME}_kdl PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                      "$<INSTALL_INTERFACE:include>")

# Add KDL kinematics factories
add_library(
  ${PROJECT_NAME}_kdl_factories
  src/kdl/kdl_factories.cpp)
target_link_libraries(
  ${PROJECT_NAME}_kdl_factories
  PUBLIC ${PROJECT_NAME}_kdl
         tesseract::tesseract_scene_graph
         tesseract::tesseract_common
         console_bridge::console_bridge)
target_compile_options(${PROJECT_NAME}_kdl_factories PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(${PROJECT_NAME}_kdl_factories PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_kdl_factories PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_kdl_factories ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_kdl_factories PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}_kdl_factories
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(${PROJECT_NAME}_kdl_factories PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                        "$<INSTALL_INTERFACE:include>")


# Add factory library so kinematic_factory can find these factories by defauult
list(APPEND KINEMATICS_PLUGINS "${PROJECT_NAME}_kdl_factories")

# Add OPW implementation of tesseract_kinematics
add_library(${PROJECT_NAME}_opw src/opw/opw_inv_kin.cpp)
target_link_libraries(
  ${PROJECT_NAME}_opw
  PUBLIC ${PROJECT_NAME}_core
         Eigen3::Eigen
         tesseract::tesseract_scene_graph
         opw_kinematics::opw_kinematics
         console_bridge::console_bridge)
target_compile_options(${PROJECT_NAME}_opw PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(${PROJECT_NAME}_opw PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_opw PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_opw ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_opw PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}_opw
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(${PROJECT_NAME}_opw PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                      "$<INSTALL_INTERFACE:include>")

add_library(${PROJECT_NAME}_opw_factory src/opw/opw_factory.cpp)
target_link_libraries(
  ${PROJECT_NAME}_opw_factory
  PUBLIC ${PROJECT_NAME}_opw
         tesseract::tesseract_scene_graph
         console_bridge::console_bridge)
target_compile_options(${PROJECT_NAME}_opw_factory PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(${PROJECT_NAME}_opw_factory PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_opw_factory PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_opw_factory ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_opw_factory PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}_opw_factory
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(${PROJECT_NAME}_opw_factory PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                      "$<INSTALL_INTERFACE:include>")

# Add factory library so kinematic_factory can find these factories by defauult
list(APPEND KINEMATICS_PLUGINS "${PROJECT_NAME}_opw_factory")

# Add Universal Robot implementation of tesseract_kinematics
add_library(${PROJECT_NAME}_ur src/ur/ur_inv_kin.cpp)
target_link_libraries(
  ${PROJECT_NAME}_ur
  PUBLIC ${PROJECT_NAME}_core
         Eigen3::Eigen
         tesseract::tesseract_scene_graph
         console_bridge::console_bridge)
target_compile_options(${PROJECT_NAME}_ur PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(${PROJECT_NAME}_ur PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_ur PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_ur ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_ur PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}_ur
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(${PROJECT_NAME}_ur PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                     "$<INSTALL_INTERFACE:include>")

add_library(${PROJECT_NAME}_ur_factory src/ur/ur_factory.cpp)
target_link_libraries(
 ${PROJECT_NAME}_ur_factory
 PUBLIC ${PROJECT_NAME}_ur
        tesseract::tesseract_scene_graph
        console_bridge::console_bridge)
target_compile_options(${PROJECT_NAME}_ur_factory PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(${PROJECT_NAME}_ur_factory PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_ur_factory PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_ur_factory ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_ur_factory PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
 ${PROJECT_NAME}_ur_factory
 ALL
 EXCLUDE ${COVERAGE_EXCLUDE}
 ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(${PROJECT_NAME}_ur_factory PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                    "$<INSTALL_INTERFACE:include>")

# Add factory library so kinematic_factory can find these factories by defauult
list(APPEND KINEMATICS_PLUGINS "${PROJECT_NAME}_ur_factory")

# Add compiler definition to core so it can find all plugins produced
string(REPLACE ";" ":" KINEMATICS_PLUGINS_STRING "${KINEMATICS_PLUGINS}")
target_compile_definitions(${PROJECT_NAME}_core PRIVATE TESSERACT_KINEMATICS_PLUGINS="${KINEMATICS_PLUGINS_STRING}")

configure_package(
  NAMESPACE tesseract
  TARGETS ${PROJECT_NAME}_core
          ${PROJECT_NAME}_kdl
          ${PROJECT_NAME}_ikfast
          ${PROJECT_NAME}_opw
          ${PROJECT_NAME}_ur
          ${PROJECT_NAME}_core_factories
          ${PROJECT_NAME}_kdl_factories
          ${PROJECT_NAME}_opw_factory
          ${PROJECT_NAME}_ur_factory
          )

# Mark cpp header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE)

#if(TESSERACT_ENABLE_TESTING)
  enable_testing()
  add_run_tests_target(ENABLE ${TESSERACT_ENABLE_RUN_TESTING})
  add_subdirectory(test)
#endif()

if(TESSERACT_PACKAGE)
  tesseract_cpack(
    VERSION ${pkg_extracted_version}
    MAINTAINER <https://github.com/ros-industrial-consortium/tesseract>
    DESCRIPTION ${pkg_extracted_description}
    LICENSE_FILE ${CMAKE_CURRENT_LIST_DIR}/../LICENSE
    README_FILE ${CMAKE_CURRENT_LIST_DIR}/../README.md
    LINUX_DEPENDS
      "${TESSERACT_PACKAGE_PREFIX}tesseract-common"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-scene-graph" tesseract_state_solver
      "${TESSERACT_PACKAGE_PREFIX}tesseract_state_solver"
      "${TESSERACT_PACKAGE_PREFIX}opw-kinematics"
      "libconsole-bridge-dev"
      "liborocos-kdl-dev"
      "libeigen3-dev"
    WINDOWS_DEPENDS
      "${TESSERACT_PACKAGE_PREFIX}tesseract-common"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-scene-graph"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-state-solver"
      "${TESSERACT_PACKAGE_PREFIX}opw-kinematics"
      "Eigen3")
endif()
