add_library(${PROJECT_NAME}_ikfast INTERFACE)
target_link_libraries(${PROJECT_NAME}_ikfast INTERFACE ${PROJECT_NAME}_core Eigen3::Eigen
                                                       console_bridge::console_bridge)
target_compile_definitions(${PROJECT_NAME}_ikfast INTERFACE -DIKFAST_NO_MAIN -DIKFAST_CLIBRARY -DIKFAST_HAS_LIBRARY)
target_compile_options(${PROJECT_NAME}_ikfast INTERFACE ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_ikfast INTERFACE ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_ikfast ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_ikfast INTERFACE VERSION ${TESSERACT_CXX_VERSION})
target_include_directories(${PROJECT_NAME}_ikfast INTERFACE "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                            "$<INSTALL_INTERFACE:include>")

install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  COMPONENT ikfast
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN "*.hpp")

configure_component(
  COMPONENT ikfast
  NAMESPACE tesseract
  TARGETS ${PROJECT_NAME}_ikfast
  DEPENDENCIES "tesseract_kinematics COMPONENTS core")

if(TESSERACT_PACKAGE)
  cpack_component(
    COMPONENT ikfast
    VERSION ${pkg_extracted_version}
    DESCRIPTION "Tesseract Kinematics ikfast implementation"
    COMPONENT_DEPENDS core)
endif()
