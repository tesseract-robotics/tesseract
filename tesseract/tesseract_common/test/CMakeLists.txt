find_package(GTest REQUIRED)

add_executable(${PROJECT_NAME}_unit tesseract_common_unit.cpp)
target_link_libraries(${PROJECT_NAME}_unit GTest::GTest GTest::Main ${PROJECT_NAME})
tesseract_target_compile_options(${PROJECT_NAME}_unit PRIVATE)
tesseract_clang_tidy(${PROJECT_NAME}_unit)
tesseract_gtest_discover_tests(${PROJECT_NAME}_unit)
add_dependencies(${PROJECT_NAME}_unit ${PROJECT_NAME})
add_dependencies(run_tests ${PROJECT_NAME}_unit)
