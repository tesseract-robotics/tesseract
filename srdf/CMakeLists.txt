find_package(Boost REQUIRED)
find_package(yaml-cpp REQUIRED)

set(SUPPORTED_COMPONENTS ${SUPPORTED_COMPONENTS} "srdf" PARENT_SCOPE)

add_library(
  srdf
  src/collision_margins.cpp
  src/configs.cpp
  src/disabled_collisions.cpp
  src/group_states.cpp
  src/group_tool_center_points.cpp
  src/groups.cpp
  src/kinematics_information.cpp
  src/srdf_model.cpp
  src/utils.cpp)
add_library(tesseract::srdf ALIAS srdf)
set_target_properties(srdf PROPERTIES OUTPUT_NAME tesseract_srdf)
target_link_libraries(
  srdf
  PUBLIC Eigen3::Eigen
         tesseract::common
         tesseract::scene_graph
         yaml-cpp
         cereal::cereal)
target_link_libraries(srdf PRIVATE Boost::boost console_bridge::console_bridge)
target_compile_options(srdf PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(srdf PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(srdf PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(srdf ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(srdf PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  srdf
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})

target_include_directories(srdf PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                       "$<INSTALL_INTERFACE:include>")

# Configure Package
configure_component(
  COMPONENT srdf
  NAMESPACE tesseract
  TARGETS srdf
  DEPENDENCIES
    Boost
    Eigen3
    console_bridge
    "tesseract COMPONENTS common scene_graph"
    yaml-cpp
    cereal)

# Mark cpp header files for installation
install(
  FILES include/tesseract/srdf/cereal_serialization.h
        include/tesseract/srdf/srdf_model.h
        include/tesseract/srdf/kinematics_information.h
        include/tesseract/srdf/utils.h
        include/tesseract/srdf/fwd.h
  DESTINATION include/tesseract/srdf
  COMPONENT srdf)

# ADD Examples
if(TESSERACT_ENABLE_EXAMPLES OR TESSERACT_SRDF_ENABLE_EXAMPLES)
  add_subdirectory(examples)
endif()

if(TESSERACT_ENABLE_TESTING OR TESSERACT_SRDF_ENABLE_TESTING)
  enable_testing()
  add_subdirectory(test)
endif()

if(TESSERACT_PACKAGE)
  cpack(
    COMPONENT srdf
    VERSION ${pkg_extracted_version}
    DESCRIPTION "Tesseract SRDF Components"
    LINUX_DEPENDS
      "libconsole-bridge-dev"
      "libeigen3-dev"
      "libyaml-cpp-dev"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-common"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-scene-graph"
    WINDOWS_DEPENDS
      "console-bridge"
      "Eigen3"
      "yaml-cpp"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-common"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-scene-graph")

  if(UNIX AND TESSERACT_PACKAGE_SOURCE)
    cpack_debian_source_package(
      CHANGLELOG ${CMAKE_CURRENT_LIST_DIR}/CHANGELOG.rst
      UPLOAD ${TESSERACT_PACKAGE_SOURCE_UPLOAD}
      DPUT_HOST ${TESSERACT_PACKAGE_SOURCE_DPUT_HOST}
      DEBIAN_INCREMENT ${TESSERACT_PACKAGE_SOURCE_DEBIAN_INCREMENT}
      DISTRIBUTIONS ${TESSERACT_PACKAGE_SOURCE_DISTRIBUTIONS})
  endif()
endif()
