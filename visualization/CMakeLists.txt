find_package(boost_plugin_loader REQUIRED)

# Optional to build gazebo visualization
find_package(gz-common5 COMPONENTS profiler events av QUIET)
find_package(gz-transport11 QUIET)
find_package(gz-msgs8 QUIET)
find_package(gz-math7-eigen3 QUIET)

set(GAZEBO_FOUND FALSE)
if(gz-common5_FOUND
   AND gz-transport11_FOUND
   AND gz-msgs8_FOUND
   AND gz-math7-eigen3_FOUND)
  set(GAZEBO_FOUND TRUE)
  message(STATUS "Gazebo Visualization Library will be built!")
endif()

# ######################################################################################################################
# Define compile-time default variables
if(MSVC)
  set(TESSERACT_VISUALIZATION_PLUGIN_PATH ${CMAKE_INSTALL_PREFIX}/bin)
else()
  set(TESSERACT_VISUALIZATION_PLUGIN_PATH ${CMAKE_INSTALL_PREFIX}/lib)
endif()

set(SUPPORTED_COMPONENTS ${SUPPORTED_COMPONENTS} "visualization" PARENT_SCOPE)

add_library(
  visualization
  src/visualization_loader.cpp
  src/trajectory_interpolator.cpp
  src/trajectory_player.cpp
  src/markers/marker.cpp)
add_library(tesseract::visualization ALIAS visualization)
set_target_properties(visualization PROPERTIES OUTPUT_NAME tesseract_visualization)
target_link_libraries(
  visualization
  PUBLIC Eigen3::Eigen
         tesseract::environment
         tesseract::scene_graph
         tesseract::state_solver
         tesseract::collision
         tesseract::common
         boost_plugin_loader::boost_plugin_loader
         console_bridge::console_bridge)
target_compile_options(visualization PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(visualization PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(visualization PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(visualization ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(visualization PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  visualization
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_compile_definitions(visualization
                           PRIVATE TESSERACT_VISUALIZATION_PLUGIN_PATH="${TESSERACT_VISUALIZATION_PLUGIN_PATH}")
target_include_directories(visualization PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                "$<INSTALL_INTERFACE:include>")

list(APPEND PACKAGE_LIBRARIES visualization)
option(TESSERACT_BUILD_GZ_VISUALIZATION "Build Gazebo visualization plugin" ON)
if(GAZEBO_FOUND AND TESSERACT_BUILD_GZ_VISUALIZATION)
  add_library(visualization_ignition src/ignition/entity_manager.cpp src/ignition/conversions.cpp)
  set_target_properties(visualization_ignition PROPERTIES OUTPUT_NAME tesseract_visualization_ignition)
  target_link_libraries(
    visualization_ignition
    PUBLIC tesseract::scene_graph
           tesseract::common
           gz-common5::gz-common5-profiler
           gz-common5::gz-common5-events
           gz-common5::gz-common5-av
           gz-msg8::gz-msg8
           console_bridge::console_bridge)
  target_compile_options(visualization_ignition PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
  target_compile_options(visualization_ignition PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
  target_compile_definitions(visualization_ignition PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
  target_clang_tidy(visualization_ignition ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
  target_cxx_version(visualization_ignition PUBLIC VERSION ${TESSERACT_CXX_VERSION})
  target_code_coverage(
    visualization_ignition
    PRIVATE
    ALL
    EXCLUDE ${COVERAGE_EXCLUDE}
    ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
  target_include_directories(visualization_ignition PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                           "$<INSTALL_INTERFACE:include>")

  add_library(visualization_ignition_visualization src/ignition/tesseract_ignition_visualization.cpp)
  set_target_properties(visualization_ignition_visualization PROPERTIES OUTPUT_NAME
                                                                        tesseract_visualization_ignition_visualization)
  target_link_libraries(
    visualization_ignition_visualization
    PUBLIC visualization
           visualization_ignition
           tesseract::scene_graph
           tesseract::common
           gz-common5::gz-common5-profiler
           gz-common5::gz-common5-events
           gz-common5::gz-common5-av
           gz-transport11::gz-transport11
           gz-msg8::gz-msg8
           console_bridge::console_bridge)
  target_compile_options(visualization_ignition_visualization PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
  target_compile_options(visualization_ignition_visualization PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
  target_compile_definitions(visualization_ignition_visualization PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
  target_clang_tidy(visualization_ignition_visualization ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
  target_cxx_version(visualization_ignition_visualization PUBLIC VERSION ${TESSERACT_CXX_VERSION})
  target_code_coverage(
    visualization_ignition_visualization
    PRIVATE
    ALL
    EXCLUDE ${COVERAGE_EXCLUDE}
    ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
  target_include_directories(
    visualization_ignition_visualization PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                "$<INSTALL_INTERFACE:include>")

  list(
    APPEND
    PACKAGE_LIBRARIES
    visualization_ignition
    visualization_ignition_visualization)
else()
  list(APPEND GAZEBO_MISSING "")
  if(NOT gz-common5_FOUND)
    list(APPEND GAZEBO_MISSING "gz-common5")
  endif()
  if(NOT ignition-transport11_FOUND)
    list(APPEND GAZEBO_MISSING "gz-transport11")
  endif()
  if(NOT gz-msgs8_FOUND)
    list(APPEND GAZEBO_MISSING "gz-msgs8")
  endif()
  if(NOT gz-math7-eigen3_FOUND)
    list(APPEND GAZEBO_MISSING "gz-math7-eigen3")
  endif()
  message(STATUS "Gazebo Visualization Library will not be built! Missing: ${GAZEBO_MISSING}")
endif()

# Configure Package
configure_component(
  COMPONENT visualization
  NAMESPACE tesseract
  TARGETS ${PACKAGE_LIBRARIES}
  DEPENDENCIES Eigen3 console_bridge "tesseract COMPONENTS common environment scene_graph state_solver collision")

# Mark cpp header files for installation
install(
  DIRECTORY include/tesseract
  DESTINATION include
  COMPONENT visualization
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)

if(TESSERACT_ENABLE_TESTING OR TESSERACT_VISUALIZATION_ENABLE_TESTING)
  enable_testing()
  add_subdirectory(test)
endif()

if(TESSERACT_PACKAGE)
  cpack(
    COMPONENT visualization
    VERSION ${pkg_extracted_version}
    DESCRIPTION "Tesseract Visualization Components"
    LINUX_DEPENDS
      "libconsole-bridge-dev"
      "libeigen3-dev"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-collision-core"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-common"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-environment"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-scene-graph"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-state-solver"
    WINDOWS_DEPENDS
      "console-bridge"
      "Eigen3"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-collision-core"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-common"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-environment"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-scene-graph"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-state-solver")

  if(UNIX AND TESSERACT_PACKAGE_SOURCE)
    cpack_debian_source_package(
      CHANGLELOG ${CMAKE_CURRENT_LIST_DIR}/CHANGELOG.rst
      UPLOAD ${TESSERACT_PACKAGE_SOURCE_UPLOAD}
      DPUT_HOST ${TESSERACT_PACKAGE_SOURCE_DPUT_HOST}
      DEBIAN_INCREMENT ${TESSERACT_PACKAGE_SOURCE_DEBIAN_INCREMENT}
      DISTRIBUTIONS ${TESSERACT_PACKAGE_SOURCE_DISTRIBUTIONS})
  endif()
endif()
