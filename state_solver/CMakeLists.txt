find_package(orocos_kdl REQUIRED)

set(SUPPORTED_COMPONENTS ${SUPPORTED_COMPONENTS} "state_solver" PARENT_SCOPE)

add_library(state_solver INTERFACE)
add_library(tesseract::state_solver ALIAS state_solver)
target_link_libraries(
  state_solver
  INTERFACE Eigen3::Eigen
            tesseract::common
            tesseract::scene_graph
            console_bridge::console_bridge)
target_compile_options(state_solver INTERFACE ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(state_solver INTERFACE ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(state_solver ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(state_solver INTERFACE VERSION ${TESSERACT_CXX_VERSION})
target_include_directories(state_solver INTERFACE "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                  "$<INSTALL_INTERFACE:include>")

add_library(state_solver_kdl src/kdl_state_solver.cpp)
add_library(tesseract::state_solver_kdl ALIAS state_solver_kdl)
set_target_properties(state_solver_kdl PROPERTIES OUTPUT_NAME tesseract_state_solver_kdl)
target_link_libraries(
  state_solver_kdl
  PUBLIC state_solver
         Eigen3::Eigen
         tesseract::common
         tesseract::scene_graph
         orocos-kdl
         console_bridge::console_bridge)
target_compile_options(state_solver_kdl PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(state_solver_kdl PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(state_solver_kdl PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(state_solver_kdl ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(state_solver_kdl PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  state_solver_kdl
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(state_solver_kdl PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                   "$<INSTALL_INTERFACE:include>")

add_library(state_solver_ofkt src/ofkt_state_solver.cpp src/ofkt_nodes.cpp)
add_library(tesseract::state_solver_ofkt ALIAS state_solver_ofkt)
set_target_properties(state_solver_ofkt PROPERTIES OUTPUT_NAME tesseract_state_solver_ofkt)
target_link_libraries(
  state_solver_ofkt
  PUBLIC state_solver
         Eigen3::Eigen
         tesseract::common
         tesseract::scene_graph
         console_bridge::console_bridge)
target_compile_options(state_solver_ofkt PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(state_solver_ofkt PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(state_solver_ofkt PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(state_solver_ofkt ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(state_solver_ofkt PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  state_solver_ofkt
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(state_solver_ofkt PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                    "$<INSTALL_INTERFACE:include>")

# Configure Package
configure_component(
  COMPONENT state_solver
  NAMESPACE tesseract
  TARGETS state_solver state_solver_kdl state_solver_ofkt
  DEPENDENCIES
    Eigen3
    orocos_kdl
    console_bridge
    "tesseract COMPONENTS common scene_graph")

# Mark cpp header files for installation
install(
  DIRECTORY include/tesseract
  DESTINATION include
  COMPONENT state_solver
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)

if(TESSERACT_ENABLE_TESTING OR TESSERACT_STATE_SOLVER_ENABLE_TESTING)
  enable_testing()
  add_subdirectory(test)
endif()

if(TESSERACT_PACKAGE)
  cpack(
    COMPONENT state_solver
    VERSION ${pkg_extracted_version}
    DESCRIPTION "Tesseract State Solver Components"
    LINUX_DEPENDS
      "libconsole-bridge-dev"
      "liborocos-kdl-dev"
      "libeigen3-dev"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-scene-graph"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-common"
    WINDOWS_DEPENDS
      "console_bridge"
      "orocos-kdl"
      "Eigen3"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-scene-graph"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-common")

  if(UNIX AND TESSERACT_PACKAGE_SOURCE)
    cpack_debian_source_package(
      CHANGLELOG ${CMAKE_CURRENT_LIST_DIR}/CHANGELOG.rst
      UPLOAD ${TESSERACT_PACKAGE_SOURCE_UPLOAD}
      DPUT_HOST ${TESSERACT_PACKAGE_SOURCE_DPUT_HOST}
      DEBIAN_INCREMENT ${TESSERACT_PACKAGE_SOURCE_DEBIAN_INCREMENT}
      DISTRIBUTIONS ${TESSERACT_PACKAGE_SOURCE_DISTRIBUTIONS})
  endif()
endif()
