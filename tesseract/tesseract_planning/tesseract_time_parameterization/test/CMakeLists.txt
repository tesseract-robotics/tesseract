find_package(GTest REQUIRED)

# Iterative Spline Time Parameterization Test
add_executable(${PROJECT_NAME}_iterative_spline iterative_spline_tests.cpp)
target_link_libraries(${PROJECT_NAME}_iterative_spline PRIVATE GTest::GTest GTest::Main ${PROJECT_NAME})
tesseract_target_compile_options(${PROJECT_NAME}_iterative_spline PRIVATE)
tesseract_clang_tidy(${PROJECT_NAME}_iterative_spline)
tesseract_code_coverage(${PROJECT_NAME}_iterative_spline ALL EXCLUDE ${COVERAGE_EXCLUDE})
tesseract_gtest_discover_tests(${PROJECT_NAME}_iterative_spline)
add_dependencies(${PROJECT_NAME}_iterative_spline ${PROJECT_NAME})
add_dependencies(run_tests ${PROJECT_NAME}_iterative_spline)

