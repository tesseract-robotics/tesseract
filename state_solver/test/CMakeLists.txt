find_gtest()

add_executable(tesseract_state_solver_unit tesseract_state_solver_ofkt_unit.cpp)
target_link_libraries(
  tesseract_state_solver_unit
  PRIVATE GTest::GTest
          GTest::Main
          tesseract::state_solver_kdl
          tesseract::state_solver_ofkt
          tesseract::urdf)
target_compile_options(tesseract_state_solver_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                           ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(tesseract_state_solver_unit PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(tesseract_state_solver_unit ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(tesseract_state_solver_unit PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  tesseract_state_solver_unit
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
add_gtest_discover_tests(tesseract_state_solver_unit)
add_dependencies(tesseract_state_solver_unit tesseract::state_solver_kdl)
add_dependencies(tesseract_state_solver_unit tesseract::state_solver_ofkt)
