cmake_minimum_required(VERSION 3.15.0)

# Extract package name and version
find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)
project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES C CXX)

if(WIN32)
  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

# System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)
find_package(console_bridge REQUIRED)
find_package(tesseract_collision REQUIRED COMPONENTS core bullet)
find_package(tesseract_kinematics REQUIRED COMPONENTS core)
find_package(tesseract_scene_graph REQUIRED)
find_package(tesseract_state_solver REQUIRED)
find_package(tesseract_srdf REQUIRED)
find_package(tesseract_urdf REQUIRED)
find_package(tesseract_common REQUIRED)
find_package(cereal REQUIRED)

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
    /*/install/*
    /*/devel/*
    /*/gtest/*
    /*/bullet/LinearMath/*
    /*/bullet/BulletCollision/*)
add_code_coverage_all_targets(EXCLUDE ${COVERAGE_EXCLUDE} ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})

# Create interface for core
add_library(
  ${PROJECT_NAME}
  src/environment.cpp
  src/environment_cache.cpp
  src/environment_monitor_interface.cpp
  src/environment_monitor.cpp
  src/events.cpp
  src/utils.cpp)
target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC Eigen3::Eigen
         cereal::cereal
         tesseract::tesseract_common
         tesseract::tesseract_collision_core
         tesseract::tesseract_scene_graph
         tesseract::tesseract_state_solver_ofkt
         tesseract::tesseract_srdf
         tesseract::tesseract_urdf
         tesseract::tesseract_kinematics_core
         ${PROJECT_NAME}_commands)
target_link_libraries(${PROJECT_NAME} PRIVATE tesseract::tesseract_collision_bullet)
target_compile_options(${PROJECT_NAME} PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(${PROJECT_NAME} PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME} PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME} PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                  "$<INSTALL_INTERFACE:include>")

# Create interface for core
add_library(
  ${PROJECT_NAME}_commands
  src/command.cpp
  src/commands/add_contact_managers_plugin_info_command.cpp
  src/commands/add_kinematics_information_command.cpp
  src/commands/add_link_command.cpp
  src/commands/add_scene_graph_command.cpp
  src/commands/add_trajectory_link_command.cpp
  src/commands/change_collision_margins_command.cpp
  src/commands/change_joint_acceleration_limits_command.cpp
  src/commands/change_joint_origin_command.cpp
  src/commands/change_joint_position_limits_command.cpp
  src/commands/change_joint_velocity_limits_command.cpp
  src/commands/change_link_collision_enabled_command.cpp
  src/commands/change_link_origin_command.cpp
  src/commands/change_link_visibility_command.cpp
  src/commands/modify_allowed_collisions_command.cpp
  src/commands/move_joint_command.cpp
  src/commands/move_link_command.cpp
  src/commands/remove_allowed_collision_link_command.cpp
  src/commands/remove_joint_command.cpp
  src/commands/remove_link_command.cpp
  src/commands/replace_joint_command.cpp
  src/commands/set_active_continuous_contact_manager_command.cpp
  src/commands/set_active_discrete_contact_manager_command.cpp)
target_link_libraries(
  ${PROJECT_NAME}_commands
  PUBLIC Eigen3::Eigen
         cereal::cereal
         tesseract::tesseract_common
         tesseract::tesseract_collision_core
         tesseract::tesseract_scene_graph
         tesseract::tesseract_state_solver_ofkt
         tesseract::tesseract_srdf
         tesseract::tesseract_urdf
         tesseract::tesseract_kinematics_core)
target_compile_options(${PROJECT_NAME}_commands PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(${PROJECT_NAME}_commands PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_commands PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_commands ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_commands PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}_commands
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(${PROJECT_NAME}_commands PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                           "$<INSTALL_INTERFACE:include>")

# Configure Package
configure_package(
  NAMESPACE tesseract
  TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_commands
  DEPENDENCIES
    Eigen3
    cereal
    console_bridge
    "tesseract_collision COMPONENTS core"
    tesseract_scene_graph
    tesseract_state_solver
    tesseract_srdf
    "tesseract_kinematics COMPONENTS core"
    tesseract_urdf
    tesseract_common
  CFG_EXTRAS cmake/tesseract_environment-extras.cmake)

# Mark cpp header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)

if(TESSERACT_ENABLE_TESTING OR TESSERACT_ENVIRONMENT_ENABLE_TESTING)
  enable_testing()
  add_run_tests_target(ENABLE ${TESSERACT_ENABLE_RUN_TESTING})
  add_subdirectory(test)
endif()

if(TESSERACT_ENABLE_BENCHMARKING OR TESSERACT_ENVIRONMENT_ENABLE_BENCHMARKING)
  add_subdirectory(test/benchmarks)
endif()

if(TESSERACT_PACKAGE)
  cpack(
    VERSION ${pkg_extracted_version}
    MAINTAINER_NAME ${pkg_extracted_maintainer_name}
    MAINTAINER_EMAIL ${pkg_extracted_maintainer_email}
    DESCRIPTION ${pkg_extracted_description}
    LICENSE_FILE ${CMAKE_CURRENT_LIST_DIR}/../LICENSE
    README_FILE ${CMAKE_CURRENT_LIST_DIR}/../README.md
    LINUX_DEPENDS
      "libconsole-bridge-dev"
      "libeigen3-dev"
      "cereal"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-collision-core"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-geometry"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-kinematics-core"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-scene-graph"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-state-solver"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-common"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-urdf"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-srdf"
    WINDOWS_DEPENDS
      "console_bridge"
      "Eigen3"
      "cereal"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-collision-core"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-geometry"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-kinematics-core"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-scene-graph"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-state-solver"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-common"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-urdf"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-srdf")

  if(UNIX AND TESSERACT_PACKAGE_SOURCE)
    cpack_debian_source_package(
      CHANGLELOG ${CMAKE_CURRENT_LIST_DIR}/CHANGELOG.rst
      UPLOAD ${TESSERACT_PACKAGE_SOURCE_UPLOAD}
      DPUT_HOST ${TESSERACT_PACKAGE_SOURCE_DPUT_HOST}
      DEBIAN_INCREMENT ${TESSERACT_PACKAGE_SOURCE_DEBIAN_INCREMENT}
      DISTRIBUTIONS ${TESSERACT_PACKAGE_SOURCE_DISTRIBUTIONS})
  endif()
endif()
