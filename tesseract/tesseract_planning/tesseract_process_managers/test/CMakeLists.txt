find_package(GTest REQUIRED)
find_package(tesseract_support REQUIRED)
find_package(tesseract_kinematics REQUIRED)

add_executable(${PROJECT_NAME}_unit tesseract_process_managers_unit.cpp)
target_link_libraries(${PROJECT_NAME}_unit PRIVATE GTest::GTest GTest::Main tesseract::tesseract_support tesseract::tesseract_kinematics_opw ${PROJECT_NAME})
target_include_directories(${PROJECT_NAME}_unit PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/examples>")
target_compile_options(${PROJECT_NAME}_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS})
target_clang_tidy(${PROJECT_NAME}_unit ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_unit PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(${PROJECT_NAME}_unit ALL EXCLUDE ${COVERAGE_EXCLUDE} ENABLE ${TESSERACT_ENABLE_TESTING})
add_gtest_discover_tests(${PROJECT_NAME}_unit)
add_dependencies(run_tests ${PROJECT_NAME}_unit)
