# Create test suite interface
add_library(common_test_suite INTERFACE)
add_library(tesseract::common_test_suite ALIAS common_test_suite)
target_link_libraries(common_test_suite INTERFACE common)
target_compile_options(common_test_suite INTERFACE ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(common_test_suite INTERFACE ${TESSERACT_COMPILE_DEFINITIONS_PUBLIC})
target_cxx_version(common_test_suite INTERFACE VERSION ${TESSERACT_CXX_VERSION})
target_clang_tidy(common_test_suite ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_include_directories(common_test_suite INTERFACE "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                       "$<INSTALL_INTERFACE:include>")

# Mark cpp header files for installation
install(
  DIRECTORY include/tesseract
  DESTINATION include
  COMPONENT common_test_suite
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN "*.hpp"
  PATTERN "*.inl"
  PATTERN ".svn" EXCLUDE)

configure_component(
  COMPONENT common_test_suite
  NAMESPACE tesseract
  TARGETS common_test_suite
  DEPENDENCIES "tesseract COMPONENTS common")

if(TESSERACT_PACKAGE)
  cpack_component(
    COMPONENT common_test_suite
    VERSION ${pkg_extracted_version}
    DESCRIPTION "Tesseract Common test_suite components"
    COMPONENT_DEPENDS common)
endif()
