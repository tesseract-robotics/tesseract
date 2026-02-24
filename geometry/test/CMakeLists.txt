find_gtest()

add_executable(tesseract_geometry_serialization_unit tesseract_geometry_serialization_unit.cpp)
target_link_libraries(tesseract_geometry_serialization_unit PRIVATE GTest::GTest GTest::Main tesseract::geometry)
target_compile_options(tesseract_geometry_serialization_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                                     ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(tesseract_geometry_serialization_unit PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(tesseract_geometry_serialization_unit ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(tesseract_geometry_serialization_unit PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  tesseract_geometry_serialization_unit
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
add_gtest_discover_tests(tesseract_geometry_serialization_unit)
add_dependencies(tesseract_geometry_serialization_unit tesseract::geometry)

add_executable(tesseract_geometry_unit tesseract_geometry_unit.cpp)
target_link_libraries(tesseract_geometry_unit PRIVATE GTest::GTest GTest::Main tesseract::geometry)
target_compile_options(tesseract_geometry_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                       ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(tesseract_geometry_unit PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(tesseract_geometry_unit ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(tesseract_geometry_unit PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  tesseract_geometry_unit
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
add_gtest_discover_tests(tesseract_geometry_unit)
add_dependencies(tesseract_geometry_unit tesseract::geometry)
