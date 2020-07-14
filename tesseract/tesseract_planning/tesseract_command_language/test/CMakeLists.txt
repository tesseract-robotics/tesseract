find_package(GTest REQUIRED)

# Utils Tests
add_executable(${PROJECT_NAME}_utils_unit utils_test.cpp)
target_link_libraries(${PROJECT_NAME}_utils_unit PRIVATE GTest::GTest GTest::Main ${PROJECT_NAME})
tesseract_target_compile_options(${PROJECT_NAME}_utils_unit PRIVATE)
tesseract_clang_tidy(${PROJECT_NAME}_utils_unit)
tesseract_code_coverage(${PROJECT_NAME}_utils_unit ALL EXCLUDE ${COVERAGE_EXCLUDE})
tesseract_gtest_discover_tests(${PROJECT_NAME}_utils_unit)
add_dependencies(run_tests ${PROJECT_NAME}_utils_unit)
add_dependencies(${PROJECT_NAME}_utils_unit ${PROJECT_NAME})

