find_package(Boost REQUIRED)
find_package(PCL REQUIRED COMPONENTS io)

set(SUPPORTED_COMPONENTS ${SUPPORTED_COMPONENTS} "urdf" PARENT_SCOPE)

add_library(
  urdf
  src/box.cpp
  src/calibration.cpp
  src/capsule.cpp
  src/collision.cpp
  src/cone.cpp
  src/cylinder.cpp
  src/dynamics.cpp
  src/geometry.cpp
  src/inertial.cpp
  src/joint.cpp
  src/limits.cpp
  src/link.cpp
  src/material.cpp
  src/mesh.cpp
  src/mimic.cpp
  src/octomap.cpp
  src/octree.cpp
  src/origin.cpp
  src/point_cloud.cpp
  src/safety_controller.cpp
  src/sdf_mesh.cpp
  src/sphere.cpp
  src/urdf_parser.cpp
  src/visual.cpp
  src/utils.cpp)
add_library(tesseract::urdf ALIAS urdf)
set_target_properties(urdf PROPERTIES OUTPUT_NAME tesseract_urdf)
target_link_libraries(
  urdf
  PUBLIC Boost::boost
         Eigen3::Eigen
         tesseract::common
         tesseract::geometry
         tesseract::scene_graph
         tesseract::collision_bullet
         console_bridge::console_bridge)
target_link_libraries(urdf PRIVATE Boost::boost console_bridge::console_bridge)
target_compile_options(urdf PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(urdf PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(urdf PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})

target_link_libraries(urdf PRIVATE ${PCL_IO_LIBRARIES})
target_include_directories(urdf PRIVATE ${PCL_INCLUDE_DIRS})
target_compile_definitions(urdf PUBLIC TESSERACT_PARSE_POINT_CLOUDS="ON")

target_include_directories(urdf PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                       "$<INSTALL_INTERFACE:include>")

target_clang_tidy(urdf ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(urdf PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  urdf
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})

# Configure Package
configure_component(
  COMPONENT urdf
  NAMESPACE tesseract
  TARGETS urdf
  DEPENDENCIES
    Boost
    Eigen3
    console_bridge
    "tesseract COMPONENTS common geometry scene_graph collision_bullet")

# Mark cpp header files for installation
install(FILES include/tesseract/urdf/urdf_parser.h include/tesseract/urdf/utils.h DESTINATION include/tesseract/urdf
        COMPONENT urdf)

# ADD Examples
if(TESSERACT_ENABLE_EXAMPLES OR TESSERACT_URDF_ENABLE_EXAMPLES)
  add_subdirectory(examples)
endif()

if(TESSERACT_ENABLE_TESTING OR TESSERACT_URDF_ENABLE_TESTING)
  enable_testing()
  add_subdirectory(test)
endif()

if(TESSERACT_PACKAGE)
  cpack(
    COMPONENT urdf
    VERSION ${pkg_extracted_version}
    DESCRIPTION "Tesseract URDF Components"
    LINUX_DEPENDS
      "libconsole-bridge-dev"
      "libeigen3-dev"
      "libpcl-dev"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-collision-bullet"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-common"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-geometry"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-scene-graph"
    WINDOWS_DEPENDS
      "console-bridge"
      "Eigen3"
      "pcl"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-collision-bullet"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-common"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-geometry"
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
