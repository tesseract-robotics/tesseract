find_gtest()
find_package(boost_plugin_loader REQUIRED)

add_executable(tesseract_common_serialization_unit tesseract_common_serialization_unit.cpp)
target_link_libraries(tesseract_common_serialization_unit PRIVATE GTest::GTest GTest::Main tesseract::common)
target_compile_options(tesseract_common_serialization_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                                   ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(tesseract_common_serialization_unit PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(tesseract_common_serialization_unit ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(tesseract_common_serialization_unit PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  tesseract_common_serialization_unit
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
add_gtest_discover_tests(tesseract_common_serialization_unit)
add_dependencies(tesseract_common_serialization_unit tesseract::common)

add_executable(tesseract_common_unit tesseract_common_unit.cpp)
target_link_libraries(tesseract_common_unit PRIVATE GTest::GTest GTest::Main tesseract::common)
target_compile_options(tesseract_common_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                     ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(tesseract_common_unit PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(tesseract_common_unit ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(tesseract_common_unit PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  tesseract_common_unit
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
add_gtest_discover_tests(tesseract_common_unit)
add_dependencies(tesseract_common_unit tesseract::common)

add_executable(tesseract_common_clone_cache_unit clone_cache_unit.cpp)
target_link_libraries(
  tesseract_common_clone_cache_unit
  PRIVATE GTest::GTest
          GTest::Main
          tesseract::common
          console_bridge::console_bridge)
target_compile_options(tesseract_common_clone_cache_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                                 ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(tesseract_common_clone_cache_unit PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(tesseract_common_clone_cache_unit ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(tesseract_common_clone_cache_unit PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  tesseract_common_clone_cache_unit
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
add_gtest_discover_tests(tesseract_common_clone_cache_unit)
add_dependencies(tesseract_common_clone_cache_unit tesseract::common)

add_library(tesseract_common_test_plugins test_plugin_multiply.cpp test_profile_factory.cpp)
target_link_libraries(tesseract_common_test_plugins PUBLIC tesseract::common boost_plugin_loader::boost_plugin_loader)
target_compile_options(tesseract_common_test_plugins PUBLIC ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                            ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(tesseract_common_test_plugins PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(tesseract_common_test_plugins ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(tesseract_common_test_plugins PUBLIC VERSION ${TESSERACT_CXX_VERSION})

add_executable(tesseract_common_plugin_loader_unit plugin_loader_unit.cpp)
target_link_libraries(
  tesseract_common_plugin_loader_unit
  PRIVATE GTest::GTest
          GTest::Main
          tesseract::common
          console_bridge::console_bridge
          boost_plugin_loader::boost_plugin_loader)
target_compile_options(tesseract_common_plugin_loader_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                                   ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(tesseract_common_plugin_loader_unit PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
target_compile_definitions(tesseract_common_plugin_loader_unit PRIVATE TEST_PLUGIN_DIR="${CMAKE_CURRENT_BINARY_DIR}")
target_clang_tidy(tesseract_common_plugin_loader_unit ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(tesseract_common_plugin_loader_unit PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  tesseract_common_plugin_loader_unit
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
add_gtest_discover_tests(tesseract_common_plugin_loader_unit)
add_dependencies(tesseract_common_plugin_loader_unit tesseract::common)

add_executable(tesseract_common_profile_plugin_factory_unit profile_plugin_factory_unit.cpp)
target_link_libraries(
  tesseract_common_profile_plugin_factory_unit
  PRIVATE GTest::GTest
          GTest::Main
          tesseract::common
          console_bridge::console_bridge
          boost_plugin_loader::boost_plugin_loader)
target_compile_options(tesseract_common_profile_plugin_factory_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                                            ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(tesseract_common_profile_plugin_factory_unit PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
target_compile_definitions(tesseract_common_profile_plugin_factory_unit
                           PRIVATE TEST_PLUGIN_DIR="${CMAKE_CURRENT_BINARY_DIR}")
target_clang_tidy(tesseract_common_profile_plugin_factory_unit ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(tesseract_common_profile_plugin_factory_unit PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  tesseract_common_profile_plugin_factory_unit
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
add_gtest_discover_tests(tesseract_common_profile_plugin_factory_unit)
add_dependencies(tesseract_common_profile_plugin_factory_unit tesseract::common)

add_executable(tesseract_common_resource_unit resource_locator_unit.cpp)
target_link_libraries(tesseract_common_resource_unit PRIVATE GTest::GTest GTest::Main tesseract::common)
target_compile_options(tesseract_common_resource_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                              ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(tesseract_common_resource_unit PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
target_compile_definitions(tesseract_common_resource_unit PRIVATE TESSERACT_INSTALL_PREFIX="${CMAKE_INSTALL_PREFIX}")
target_clang_tidy(tesseract_common_resource_unit ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(tesseract_common_resource_unit PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  tesseract_common_resource_unit
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
add_gtest_discover_tests(tesseract_common_resource_unit)
add_dependencies(tesseract_common_resource_unit tesseract::common)

add_executable(tesseract_common_utils_unit utils_unit.cpp)
target_link_libraries(tesseract_common_utils_unit PRIVATE GTest::GTest GTest::Main tesseract::common)
target_compile_options(tesseract_common_utils_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                           ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_clang_tidy(tesseract_common_utils_unit ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(tesseract_common_utils_unit PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  tesseract_common_utils_unit
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
add_gtest_discover_tests(tesseract_common_utils_unit)
add_dependencies(tesseract_common_utils_unit tesseract::common)
