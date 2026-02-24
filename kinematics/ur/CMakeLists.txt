add_library(kinematics_ur src/ur_inv_kin.cpp)
add_library(tesseract::kinematics_ur ALIAS kinematics_ur)
set_target_properties(kinematics_ur PROPERTIES OUTPUT_NAME tesseract_kinematics_ur)
target_link_libraries(
  kinematics_ur
  PUBLIC kinematics
         Eigen3::Eigen
         tesseract::scene_graph
         console_bridge::console_bridge)
target_compile_options(kinematics_ur PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(kinematics_ur PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(kinematics_ur PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(kinematics_ur ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(kinematics_ur PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  kinematics_ur
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(kinematics_ur PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                "$<INSTALL_INTERFACE:include>")

add_library(kinematics_ur_factory src/ur_factory.cpp)
set_target_properties(kinematics_ur_factory PROPERTIES OUTPUT_NAME tesseract_kinematics_ur_factory)
target_link_libraries(kinematics_ur_factory PUBLIC kinematics_ur tesseract::scene_graph console_bridge::console_bridge)
target_compile_options(kinematics_ur_factory PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(kinematics_ur_factory PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(kinematics_ur_factory PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(kinematics_ur_factory ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(kinematics_ur_factory PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  kinematics_ur_factory
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(kinematics_ur_factory PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                        "$<INSTALL_INTERFACE:include>")

# Add factory library so kinematic_factory can find these factories by defauult
set(KINEMATICS_PLUGINS ${KINEMATICS_PLUGINS} "tesseract_kinematics_ur_factory" PARENT_SCOPE)

install(
  DIRECTORY include/tesseract
  DESTINATION include
  COMPONENT kinematics_ur
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN "*.hpp")

# Plugin should not be export only installed
install(
  TARGETS kinematics_ur_factory
  COMPONENT kinematics_ur
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

configure_component(
  COMPONENT kinematics_ur
  NAMESPACE tesseract
  TARGETS kinematics_ur
  DEPENDENCIES "tesseract COMPONENTS kinematics")

if(TESSERACT_PACKAGE)
  cpack_component(
    COMPONENT kinematics_ur
    VERSION ${pkg_extracted_version}
    DESCRIPTION "Tesseract Kinematics ur implementation"
    COMPONENT_DEPENDS kinematics)
endif()
