find_package(GTest REQUIRED)

# Utils Tests
add_executable(${PROJECT_NAME}_utils_unit utils_test.cpp)
target_link_libraries(${PROJECT_NAME}_utils_unit PRIVATE GTest::GTest GTest::Main ${PROJECT_NAME})
target_compile_options(${PROJECT_NAME}_utils_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS})
target_clang_tidy(${PROJECT_NAME}_utils_unit ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_utils_unit PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(${PROJECT_NAME}_utils_unit ALL EXCLUDE ${COVERAGE_EXCLUDE} ENABLE ${TESSERACT_ENABLE_TESTING})
add_gtest_discover_tests(${PROJECT_NAME}_utils_unit)
add_dependencies(run_tests ${PROJECT_NAME}_utils_unit)
add_dependencies(${PROJECT_NAME}_utils_unit ${PROJECT_NAME})

# Serialize Tests
add_executable(${PROJECT_NAME}_serialize_unit serialize_test.cpp)
target_link_libraries(${PROJECT_NAME}_serialize_unit PRIVATE GTest::GTest GTest::Main ${PROJECT_NAME})
target_compile_options(${PROJECT_NAME}_serialize_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS})
target_clang_tidy(${PROJECT_NAME}_serialize_unit ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_serialize_unit PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(${PROJECT_NAME}_serialize_unit ALL EXCLUDE ${COVERAGE_EXCLUDE} ENABLE ${TESSERACT_ENABLE_TESTING})
add_gtest_discover_tests(${PROJECT_NAME}_serialize_unit)
add_dependencies(run_tests ${PROJECT_NAME}_serialize_unit)
add_dependencies(${PROJECT_NAME}_serialize_unit ${PROJECT_NAME})
