find_gtest()
find_package(tesseract_support REQUIRED)
find_package(tesseract_urdf REQUIRED)

add_executable(${PROJECT_NAME}_unit tesseract_state_solver_ofkt_unit.cpp)
target_link_libraries(
  ${PROJECT_NAME}_unit
  PRIVATE GTest::GTest
          GTest::Main
          ${PROJECT_NAME}_kdl
          ${PROJECT_NAME}_ofkt
          tesseract::tesseract_urdf
          tesseract::tesseract_support)
target_compile_options(${PROJECT_NAME}_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                    ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_unit PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_unit ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_unit PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}_unit
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
add_gtest_discover_tests(${PROJECT_NAME}_unit)
add_dependencies(${PROJECT_NAME}_unit ${PROJECT_NAME}_kdl)
add_dependencies(run_tests ${PROJECT_NAME}_unit)
