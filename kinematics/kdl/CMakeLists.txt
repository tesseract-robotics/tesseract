find_package(orocos_kdl REQUIRED)

add_library(
  kinematics_kdl
  src/kdl_fwd_kin_chain.cpp
  src/kdl_inv_kin_chain_lma.cpp
  src/kdl_inv_kin_chain_nr.cpp
  src/kdl_inv_kin_chain_nr_jl.cpp
  src/kdl_utils.cpp)
add_library(tesseract::kinematics_kdl ALIAS kinematics_kdl)
set_target_properties(kinematics_kdl PROPERTIES OUTPUT_NAME tesseract_kinematics_kdl)
target_link_libraries(
  kinematics_kdl
  PUBLIC kinematics
         Eigen3::Eigen
         tesseract::scene_graph
         tesseract::common
         orocos-kdl
         console_bridge::console_bridge)
target_compile_options(kinematics_kdl PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(kinematics_kdl PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(kinematics_kdl PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(kinematics_kdl ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(kinematics_kdl PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  kinematics_kdl
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
if(orocos_kdl_VERSION VERSION_LESS "1.4.0")
  target_compile_definitions(kinematics_kdl PUBLIC KDL_LESS_1_4_0=ON)
endif()
target_include_directories(kinematics_kdl PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                 "$<INSTALL_INTERFACE:include>")

# Add KDL kinematics factories
add_library(kinematics_kdl_factories src/kdl_factories.cpp)
set_target_properties(kinematics_kdl_factories PROPERTIES OUTPUT_NAME tesseract_kinematics_kdl_factories)
target_link_libraries(
  kinematics_kdl_factories
  PUBLIC kinematics_kdl
         tesseract::scene_graph
         tesseract::common
         console_bridge::console_bridge)
target_compile_options(kinematics_kdl_factories PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(kinematics_kdl_factories PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(kinematics_kdl_factories PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(kinematics_kdl_factories ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(kinematics_kdl_factories PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  kinematics_kdl_factories
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(kinematics_kdl_factories PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                           "$<INSTALL_INTERFACE:include>")

# Add factory library so kinematic_factory can find these factories by defauult
set(KINEMATICS_PLUGINS ${KINEMATICS_PLUGINS} "tesseract_kinematics_kdl_factories" PARENT_SCOPE)

install(
  DIRECTORY include/tesseract
  DESTINATION include
  COMPONENT kinematics_kdl
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN "*.hpp")

# Plugin should not be export only installed
install(
  TARGETS kinematics_kdl_factories
  COMPONENT kinematics_kdl
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

configure_component(
  COMPONENT kinematics_kdl
  NAMESPACE tesseract
  TARGETS kinematics_kdl
  DEPENDENCIES orocos_kdl "tesseract COMPONENTS kinematics")

if(TESSERACT_PACKAGE)
  cpack_component(
    COMPONENT kinematics_kdl
    VERSION ${pkg_extracted_version}
    DESCRIPTION "Tesseract Kinematics kdl implementation"
    COMPONENT_DEPENDS kinematics
    LINUX_DEPENDS "liborocos-kdl-dev"
    WINDOWS_DEPENDS "orocos-kdl")
endif()
