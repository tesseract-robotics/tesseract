add_library(
  kinematics
  src/forward_kinematics.cpp
  src/inverse_kinematics.cpp
  src/rop_inv_kin.cpp
  src/rep_inv_kin.cpp
  src/joint_group.cpp
  src/kinematic_group.cpp
  src/kinematics_plugin_factory.cpp
  src/utils.cpp
  src/validate.cpp)
add_library(tesseract::kinematics ALIAS kinematics)
set_target_properties(kinematics PROPERTIES OUTPUT_NAME tesseract_kinematics)
target_link_libraries(
  kinematics
  PUBLIC Eigen3::Eigen
         tesseract::common
         tesseract::scene_graph
         tesseract::state_solver_kdl
         console_bridge::console_bridge
         boost_plugin_loader::boost_plugin_loader
         yaml-cpp)
target_compile_options(kinematics PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(kinematics PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(kinematics PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_compile_definitions(kinematics PRIVATE TESSERACT_KINEMATICS_PLUGIN_PATH="${TESSERACT_KINEMATICS_PLUGIN_PATH}")
target_clang_tidy(kinematics ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(kinematics PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  kinematics
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(kinematics PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                             "$<INSTALL_INTERFACE:include>")

# Add KDL kinematics factories
add_library(kinematics_factories src/rop_factory.cpp src/rep_factory.cpp)
set_target_properties(kinematics_factories PROPERTIES OUTPUT_NAME tesseract_kinematics_factories)
target_link_libraries(kinematics_factories PUBLIC kinematics console_bridge::console_bridge)
target_compile_options(kinematics_factories PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(kinematics_factories PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(kinematics_factories PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(kinematics_factories ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(kinematics_factories PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  kinematics_factories
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(kinematics_factories PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                       "$<INSTALL_INTERFACE:include>")

# Add factory library so kinematic_factory can find these factories by defauult
set(KINEMATICS_PLUGINS ${KINEMATICS_PLUGINS} "tesseract_kinematics_factories" PARENT_SCOPE)

install(
  DIRECTORY include/tesseract
  DESTINATION include
  COMPONENT kinematics
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN "*.hpp")

configure_component(
  COMPONENT kinematics
  NAMESPACE tesseract
  TARGETS kinematics kinematics_factories
  DEPENDENCIES
    Eigen3
    console_bridge
    "tesseract COMPONENTS common scene_graph state_solver"
    boost_plugin_loader
    yaml-cpp)

if(TESSERACT_PACKAGE)
  cpack_component(
    COMPONENT kinematics
    VERSION ${pkg_extracted_version}
    DESCRIPTION "Tesseract Kinematics core components"
    LINUX_DEPENDS
      "libconsole-bridge-dev"
      "libeigen3-dev"
      "libyaml-cpp-dev"
      "boost-plugin-loader"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-common"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-scene-graph"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-state-solver"
    WINDOWS_DEPENDS
      "console_bridge"
      "Eigen3"
      "yaml-cpp"
      "boost-plugin-loader"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-common"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-scene-graph"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-state-solver")
endif()
