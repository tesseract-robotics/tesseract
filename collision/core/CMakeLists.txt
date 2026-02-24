# Create interface for core
add_library(
  collision
  src/cereal_serialization.cpp
  src/common.cpp
  src/contact_managers_plugin_factory.cpp
  src/continuous_contact_manager.cpp
  src/discrete_contact_manager.cpp
  src/types.cpp
  src/utils.cpp)
add_library(tesseract::collision ALIAS collision)
set_target_properties(collision PROPERTIES OUTPUT_NAME tesseract_collision)
target_link_libraries(
  collision
  PUBLIC Eigen3::Eigen
         tesseract::common
         tesseract::geometry
         boost_plugin_loader::boost_plugin_loader
         Boost::boost
         Boost::system
         cereal::cereal
         yaml-cpp
         octomap)
target_compile_options(collision PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(collision PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(collision PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_compile_definitions(collision
                           PRIVATE TESSERACT_CONTACT_MANAGERS_PLUGIN_PATH="${TESSERACT_CONTACT_MANAGERS_PLUGIN_PATH}")
target_cxx_version(collision PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_clang_tidy(collision ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_code_coverage(
  collision
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(collision PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                            "$<INSTALL_INTERFACE:include>")

# Mark cpp header files for installation
install(
  DIRECTORY include/tesseract
  DESTINATION include
  COMPONENT collision
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN "*.hpp"
  PATTERN "*.inl"
  PATTERN ".svn" EXCLUDE)

configure_component(
  COMPONENT collision
  NAMESPACE tesseract
  TARGETS collision
  DEPENDENCIES
    Eigen3
    console_bridge
    "tesseract COMPONENTS geometry common"
    yaml-cpp
    octomap
    boost_plugin_loader
    cereal
    "Boost COMPONENTS system program_options")

if(TESSERACT_PACKAGE)
  cpack_component(
    COMPONENT core
    VERSION ${pkg_extracted_version}
    DESCRIPTION "Tesseract Collision core components"
    LINUX_DEPENDS
      "libconsole-bridge-dev"
      "libeigen3-dev"
      "libyaml-cpp-dev"
      "liboctomap-dev"
      "libboost-system-dev"
      "libboost-program-options-dev"
      "cereal"
      "boost-plugin-loader"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-common"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-scene-graph"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-state-solver"
    WINDOWS_DEPENDS
      "console_bridge"
      "Eigen3"
      "yaml-cpp"
      "liboctomap-dev"
      "libboost-system-dev"
      "libboost-program-options-dev"
      "cereal"
      "boost-plugin-loader"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-common"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-scene-graph"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-state-solver")
endif()
