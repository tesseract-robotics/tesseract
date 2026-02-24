find_gtest()

add_executable(tesseract_visualization_player_unit trajectory_player_unit.cpp)
target_link_libraries(
  tesseract_visualization_player_unit
  PRIVATE Eigen3::Eigen
          GTest::GTest
          GTest::Main
          tesseract::visualization)
target_compile_options(tesseract_visualization_player_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                                   ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(tesseract_visualization_player_unit PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(tesseract_visualization_player_unit ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(tesseract_visualization_player_unit PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  tesseract_visualization_player_unit
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
add_gtest_discover_tests(tesseract_visualization_player_unit)
add_dependencies(tesseract_visualization_player_unit tesseract::visualization)
