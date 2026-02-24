set(SUPPORTED_COMPONENTS ${SUPPORTED_COMPONENTS} "environment" PARENT_SCOPE)

add_library(
  environment
  src/cereal_serialization.cpp
  src/environment.cpp
  src/environment_cache.cpp
  src/environment_monitor_interface.cpp
  src/environment_monitor.cpp
  src/events.cpp
  src/utils.cpp
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
add_library(tesseract::environment ALIAS environment)
set_target_properties(environment PROPERTIES OUTPUT_NAME tesseract_environment)
target_link_libraries(
  environment
  PUBLIC Eigen3::Eigen
         cereal::cereal
         tesseract::common
         tesseract::collision
         tesseract::scene_graph
         tesseract::state_solver_ofkt
         tesseract::srdf
         tesseract::urdf
         tesseract::kinematics)
target_link_libraries(environment PRIVATE tesseract::collision_bullet)
target_compile_options(environment PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(environment PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(environment PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(environment ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(environment PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  environment
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(environment PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                              "$<INSTALL_INTERFACE:include>")

# Configure Package
configure_component(
  COMPONENT environment
  NAMESPACE tesseract
  TARGETS environment
  DEPENDENCIES
    Eigen3
    cereal
    console_bridge
    "tesseract COMPONENTS common collision kinematics scene_graph state_solver srdf urdf")

# Mark cpp header files for installation
install(
  DIRECTORY include/tesseract
  DESTINATION include
  COMPONENT environment
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)

if(TESSERACT_ENABLE_TESTING OR TESSERACT_ENVIRONMENT_ENABLE_TESTING)
  enable_testing()
  add_subdirectory(test)
endif()

if(TESSERACT_ENABLE_BENCHMARKING OR TESSERACT_ENVIRONMENT_ENABLE_BENCHMARKING)
  add_subdirectory(test/benchmarks)
endif()

if(TESSERACT_PACKAGE)
  cpack(
    COMPONENT environment
    VERSION ${pkg_extracted_version}
    DESCRIPTION "Tesseract Environment Components"
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
