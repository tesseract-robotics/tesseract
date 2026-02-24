find_package(Boost REQUIRED COMPONENTS graph)
find_package(orocos_kdl REQUIRED)

set(SUPPORTED_COMPONENTS ${SUPPORTED_COMPONENTS} "scene_graph" PARENT_SCOPE)

add_library(
  scene_graph
  src/graph.cpp
  src/joint.cpp
  src/kdl_parser.cpp
  src/link.cpp
  src/scene_state.cpp)
add_library(tesseract::scene_graph ALIAS scene_graph)
set_target_properties(scene_graph PROPERTIES OUTPUT_NAME tesseract_scene_graph)
target_link_libraries(
  scene_graph
  PUBLIC Boost::boost
         Boost::graph
         Eigen3::Eigen
         tesseract::geometry
         tesseract::common
         orocos-kdl
         console_bridge::console_bridge)
target_compile_options(scene_graph PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(scene_graph PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(
  scene_graph
  PUBLIC ${TESSERACT_COMPILE_DEFINITIONS}
         Boost_USE_STATIC_LIBS=OFF
         Boost_USE_MULTITHREADED=ON
         Boost_USE_STATIC_RUNTIME=OFF)
target_clang_tidy(scene_graph ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(scene_graph PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  scene_graph
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(scene_graph PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                              "$<INSTALL_INTERFACE:include>")

# Configure Package
configure_component(
  COMPONENT scene_graph
  NAMESPACE tesseract
  TARGETS scene_graph
  DEPENDENCIES
    "Boost COMPONENTS graph"
    Eigen3
    console_bridge
    orocos_kdl
    "tesseract COMPONENTS geometry common")

# Mark cpp header files for installation
install(
  DIRECTORY include/tesseract
  DESTINATION include
  COMPONENT scene_graph
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)

# ADD Examples
if(TESSERACT_ENABLE_EXAMPLES OR TESSERACT_SCENE_GRAPH_ENABLE_EXAMPLES)
  add_subdirectory(examples)
endif()

if(TESSERACT_ENABLE_TESTING OR TESSERACT_SCENE_GRAPH_ENABLE_TESTING)
  enable_testing()
  add_subdirectory(test)
endif()

if(TESSERACT_PACKAGE)
  cpack(
    COMPONENT scene_graph
    VERSION ${pkg_extracted_version}
    DESCRIPTION "Tesseract Scene Graph Components"
    LINUX_DEPENDS
      "libboost-all-dev"
      "liborocos-kdl-dev"
      "libeigen3-dev"
      "libconsole-bridge-dev"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-common"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-geometry"
    WINDOWS_DEPENDS
      "boost-all"
      "orocos-kdl"
      "Eigen3"
      "console-bridge"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-common"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-geometry")

  if(UNIX AND TESSERACT_PACKAGE_SOURCE)
    cpack_debian_source_package(
      CHANGLELOG ${CMAKE_CURRENT_LIST_DIR}/CHANGELOG.rst
      UPLOAD ${TESSERACT_PACKAGE_SOURCE_UPLOAD}
      DPUT_HOST ${TESSERACT_PACKAGE_SOURCE_DPUT_HOST}
      DEBIAN_INCREMENT ${TESSERACT_PACKAGE_SOURCE_DEBIAN_INCREMENT}
      DISTRIBUTIONS ${TESSERACT_PACKAGE_SOURCE_DISTRIBUTIONS})
  endif()
endif()
