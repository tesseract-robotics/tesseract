find_gtest()

add_executable(${PROJECT_NAME}_player_unit trajectory_player_unit.cpp)
target_link_libraries(
  ${PROJECT_NAME}_player_unit
  PRIVATE Eigen3::Eigen
          GTest::GTest
          GTest::Main
          ${PROJECT_NAME})
target_compile_options(${PROJECT_NAME}_player_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                           ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_player_unit PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_player_unit ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_player_unit PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}_player_unit
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
add_gtest_discover_tests(${PROJECT_NAME}_player_unit)
add_dependencies(${PROJECT_NAME}_player_unit ${PROJECT_NAME})
add_dependencies(run_tests ${PROJECT_NAME}_player_unit)
