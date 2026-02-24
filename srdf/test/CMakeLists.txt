find_gtest()

add_executable(tesseract_srdf_serialization_unit tesseract_srdf_serialization_unit.cpp)
target_link_libraries(
  tesseract_srdf_serialization_unit
  PRIVATE GTest::GTest
          GTest::Main
          tesseract::srdf
          tesseract::geometry)
target_compile_options(tesseract_srdf_serialization_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                                 ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(tesseract_srdf_serialization_unit PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(tesseract_srdf_serialization_unit ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(tesseract_srdf_serialization_unit PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  tesseract_srdf_serialization_unit
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
add_gtest_discover_tests(tesseract_srdf_serialization_unit)
add_dependencies(tesseract_srdf_serialization_unit tesseract::srdf)

add_executable(tesseract_srdf_unit tesseract_srdf_unit.cpp)
target_link_libraries(
  tesseract_srdf_unit
  PRIVATE GTest::GTest
          GTest::Main
          tesseract::srdf
          tesseract::geometry)
target_compile_options(tesseract_srdf_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                   ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(tesseract_srdf_unit PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(tesseract_srdf_unit ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(tesseract_srdf_unit PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  tesseract_srdf_unit
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
add_gtest_discover_tests(tesseract_srdf_unit)
add_dependencies(tesseract_srdf_unit tesseract::srdf)
