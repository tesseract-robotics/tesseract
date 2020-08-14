find_package(GTest REQUIRED)

add_executable(${PROJECT_NAME}_player_unit trajectory_player_unit.cpp)
target_link_libraries(${PROJECT_NAME}_player_unit PRIVATE GTest::GTest GTest::Main ${PROJECT_NAME})
target_include_directories(${PROJECT_NAME}_player_unit PRIVATE ${EIGEN3_INCLUDE_DIRS})
tesseract_target_compile_options(${PROJECT_NAME}_player_unit PRIVATE)
tesseract_clang_tidy(${PROJECT_NAME}_player_unit)
tesseract_code_coverage(${PROJECT_NAME}_player_unit ALL EXCLUDE ${COVERAGE_EXCLUDE})
tesseract_gtest_discover_tests(${PROJECT_NAME}_player_unit)
add_dependencies(${PROJECT_NAME}_player_unit ${PROJECT_NAME})
add_dependencies(run_tests ${PROJECT_NAME}_player_unit)

