add_library(kinematics_ikfast INTERFACE)
add_library(tesseract::kinematics_ikfast ALIAS kinematics_ikfast)
target_link_libraries(kinematics_ikfast INTERFACE kinematics Eigen3::Eigen console_bridge::console_bridge)
target_compile_definitions(kinematics_ikfast INTERFACE -DIKFAST_NO_MAIN -DIKFAST_CLIBRARY -DIKFAST_HAS_LIBRARY)
target_compile_options(kinematics_ikfast INTERFACE ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(kinematics_ikfast INTERFACE ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(kinematics_ikfast ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(kinematics_ikfast INTERFACE VERSION ${TESSERACT_CXX_VERSION})
target_include_directories(kinematics_ikfast INTERFACE "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                       "$<INSTALL_INTERFACE:include>")

install(
  DIRECTORY include/tesseract
  DESTINATION include
  COMPONENT kinematics_ikfast
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN "*.hpp")

configure_component(
  COMPONENT kinematics_ikfast
  NAMESPACE tesseract
  TARGETS kinematics_ikfast
  DEPENDENCIES "tesseract COMPONENTS kinematics")

if(TESSERACT_PACKAGE)
  cpack_component(
    COMPONENT kinematics_ikfast
    VERSION ${pkg_extracted_version}
    DESCRIPTION "Tesseract Kinematics ikfast implementation"
    COMPONENT_DEPENDS kinematics)
endif()
