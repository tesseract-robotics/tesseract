find_package(GTest REQUIRED)

if(NOT TARGET GTest::GTest)
  add_library(GTest::GTest INTERFACE IMPORTED)
  set_target_properties(GTest::GTest PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${GTEST_INCLUDE_DIRS}")
  if(${GTEST_LIBRARIES})
    set_target_properties(GTest::GTest PROPERTIES INTERFACE_LINK_LIBRARIES "${GTEST_LIBRARIES}")
  else()
    if(MSVC)
      set_target_properties(GTest::GTest PROPERTIES INTERFACE_LINK_LIBRARIES "gtest.lib")
    else()
      set_target_properties(GTest::GTest PROPERTIES INTERFACE_LINK_LIBRARIES "libgtest.so")
    endif()
  endif()
endif()

if(NOT TARGET GTest::Main)
  add_library(GTest::Main INTERFACE IMPORTED)
  set_target_properties(GTest::Main PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${GTEST_INCLUDE_DIRS}")
  if(${GTEST_MAIN_LIBRARIES})
    set_target_properties(GTest::Main PROPERTIES INTERFACE_LINK_LIBRARIES "${GTEST_MAIN_LIBRARIES}")
  else()
    if(MSVC)
      set_target_properties(GTest::Main PROPERTIES INTERFACE_LINK_LIBRARIES "gtest_main.lib")
    else()
      set_target_properties(GTest::Main PROPERTIES INTERFACE_LINK_LIBRARIES "libgtest_main.so")
    endif()
  endif()
endif()

include(GoogleTest)

add_executable(${PROJECT_NAME}_unit tesseract_common_unit.cpp)
target_link_libraries(${PROJECT_NAME}_unit PRIVATE GTest::GTest GTest::Main ${PROJECT_NAME})
target_compile_options(${PROJECT_NAME}_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                    ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_unit PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_unit ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_unit PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}_unit
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
add_gtest_discover_tests(${PROJECT_NAME}_unit)
add_dependencies(${PROJECT_NAME}_unit ${PROJECT_NAME})
add_dependencies(run_tests ${PROJECT_NAME}_unit)

add_executable(${PROJECT_NAME}_clone_cache_unit clone_cache_unit.cpp)
target_link_libraries(
  ${PROJECT_NAME}_clone_cache_unit
  PRIVATE GTest::GTest
          GTest::Main
          ${PROJECT_NAME}
          console_bridge::console_bridge)
target_compile_options(${PROJECT_NAME}_clone_cache_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                                ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_clone_cache_unit PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_clone_cache_unit ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS}
                  ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_clone_cache_unit PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}_clone_cache_unit
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
add_gtest_discover_tests(${PROJECT_NAME}_clone_cache_unit)
add_dependencies(${PROJECT_NAME}_clone_cache_unit ${PROJECT_NAME})
add_dependencies(run_tests ${PROJECT_NAME}_clone_cache_unit)

add_library(${PROJECT_NAME}_test_plugin_multiply test_plugin_multiply.cpp)
target_link_libraries(${PROJECT_NAME}_test_plugin_multiply PUBLIC ${PROJECT_NAME} Boost::boost)
target_compile_options(${PROJECT_NAME}_test_plugin_multiply PUBLIC ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                                   ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_test_plugin_multiply PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_test_plugin_multiply ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS}
                  ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_test_plugin_multiply PUBLIC VERSION ${TESSERACT_CXX_VERSION})

add_executable(${PROJECT_NAME}_plugin_loader_unit plugin_loader_unit.cpp)
target_link_libraries(
  ${PROJECT_NAME}_plugin_loader_unit
  PRIVATE GTest::GTest
          GTest::Main
          ${PROJECT_NAME}
          console_bridge::console_bridge)
target_compile_options(${PROJECT_NAME}_plugin_loader_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                                  ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_plugin_loader_unit PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
target_compile_definitions(${PROJECT_NAME}_plugin_loader_unit PRIVATE TEST_PLUGIN_DIR="${CMAKE_CURRENT_BINARY_DIR}")
target_clang_tidy(${PROJECT_NAME}_plugin_loader_unit ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS}
                  ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_plugin_loader_unit PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}_plugin_loader_unit
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
add_gtest_discover_tests(${PROJECT_NAME}_plugin_loader_unit)
add_dependencies(${PROJECT_NAME}_plugin_loader_unit ${PROJECT_NAME})
add_dependencies(run_tests ${PROJECT_NAME}_plugin_loader_unit)
