# Create test suite interface
add_library(collision_test_suite INTERFACE)
add_library(tesseract::collision_test_suite ALIAS collision_test_suite)
target_link_libraries(collision_test_suite INTERFACE Eigen3::Eigen tesseract::geometry collision)
target_compile_options(collision_test_suite INTERFACE ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(collision_test_suite INTERFACE ${TESSERACT_COMPILE_DEFINITIONS})
target_cxx_version(collision_test_suite INTERFACE VERSION ${TESSERACT_CXX_VERSION})
target_clang_tidy(collision_test_suite ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_include_directories(collision_test_suite INTERFACE "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                          "$<INSTALL_INTERFACE:include>")

# Create test suite benchmarks interface
add_library(collision_test_suite_benchmarks INTERFACE)
add_library(tesseract::collision_test_suite_benchmarks ALIAS collision_test_suite_benchmarks)
target_link_libraries(collision_test_suite_benchmarks INTERFACE Eigen3::Eigen tesseract::geometry)
target_compile_options(collision_test_suite_benchmarks INTERFACE ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_cxx_version(collision_test_suite_benchmarks INTERFACE VERSION ${TESSERACT_CXX_VERSION})
target_clang_tidy(collision_test_suite_benchmarks ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_include_directories(
  collision_test_suite_benchmarks INTERFACE "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                            "$<INSTALL_INTERFACE:include>")
# Mark cpp header files for installation
install(
  DIRECTORY include/tesseract
  DESTINATION include
  COMPONENT collision_test_suite
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN "*.hpp"
  PATTERN "*.inl"
  PATTERN ".svn" EXCLUDE)

configure_component(
  COMPONENT collision_test_suite
  NAMESPACE tesseract
  TARGETS collision_test_suite collision_test_suite_benchmarks
  DEPENDENCIES "tesseract COMPONENTS collision")

if(TESSERACT_PACKAGE)
  cpack_component(
    COMPONENT collision_test_suite
    VERSION ${pkg_extracted_version}
    DESCRIPTION "Tesseract Collision test_suite components"
    COMPONENT_DEPENDS collision
    LINUX_DEPENDS "${TESSERACT_PACKAGE_PREFIX}tesseract-support"
    WINDOWS_DEPENDS "${TESSERACT_PACKAGE_PREFIX}tesseract-support")
endif()
