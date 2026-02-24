find_package(opw_kinematics REQUIRED)

add_library(kinematics_opw src/opw_inv_kin.cpp)
add_library(tesseract::kinematics_opw ALIAS kinematics_opw)
set_target_properties(kinematics_opw PROPERTIES OUTPUT_NAME tesseract_kinematics_opw)
target_link_libraries(
  kinematics_opw
  PUBLIC kinematics
         Eigen3::Eigen
         tesseract::scene_graph
         opw_kinematics::opw_kinematics
         console_bridge::console_bridge)
target_compile_options(kinematics_opw PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(kinematics_opw PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(kinematics_opw PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(kinematics_opw ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(kinematics_opw PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  kinematics_opw
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(kinematics_opw PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                 "$<INSTALL_INTERFACE:include>")

add_library(kinematics_opw_factory src/opw_factory.cpp)
set_target_properties(kinematics_opw_factory PROPERTIES OUTPUT_NAME tesseract_kinematics_opw_factory)
target_link_libraries(kinematics_opw_factory PUBLIC kinematics_opw tesseract::scene_graph
                                                    console_bridge::console_bridge)
target_compile_options(kinematics_opw_factory PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(kinematics_opw_factory PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(kinematics_opw_factory PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(kinematics_opw_factory ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(kinematics_opw_factory PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  kinematics_opw_factory
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(kinematics_opw_factory PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                         "$<INSTALL_INTERFACE:include>")

# Add factory library so kinematic_factory can find these factories by defauult
set(KINEMATICS_PLUGINS ${KINEMATICS_PLUGINS} "tesseract_kinematics_opw_factory" PARENT_SCOPE)

install(
  DIRECTORY include/tesseract
  DESTINATION include
  COMPONENT kinematics_opw
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN "*.hpp")

# Plugin should not be export only installed
install(
  TARGETS kinematics_opw_factory
  COMPONENT kinematics_opw
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

configure_component(
  COMPONENT kinematics_opw
  NAMESPACE tesseract
  TARGETS kinematics_opw
  DEPENDENCIES opw_kinematics "tesseract COMPONENTS kinematics")

if(TESSERACT_PACKAGE)
  cpack_component(
    COMPONENT kinematics_opw
    VERSION ${pkg_extracted_version}
    DESCRIPTION "Tesseract Kinematics opw implementation"
    COMPONENT_DEPENDS kinematics
    LINUX_DEPENDS "${TESSERACT_PACKAGE_PREFIX}opw-kinematics"
    WINDOWS_DEPENDS "${TESSERACT_PACKAGE_PREFIX}opw-kinematics")
endif()
