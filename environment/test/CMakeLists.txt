find_gtest()

find_package(OpenMP REQUIRED)
if(NOT TARGET OpenMP::OpenMP_CXX)
  find_package(Threads REQUIRED)
  add_library(OpenMP::OpenMP_CXX IMPORTED INTERFACE)
  set_property(TARGET OpenMP::OpenMP_CXX PROPERTY INTERFACE_COMPILE_OPTIONS ${OpenMP_CXX_FLAGS})
  # Only works if the same flag is passed to the linker; use CMake 3.9+ otherwise (Intel, AppleClang)
  set_property(TARGET OpenMP::OpenMP_CXX PROPERTY INTERFACE_LINK_LIBRARIES ${OpenMP_CXX_FLAGS} Threads::Threads)
endif()

add_executable(tesseract_environment_unit tesseract_environment_unit.cpp)
target_link_libraries(
  tesseract_environment_unit
  PRIVATE GTest::GTest
          GTest::Main
          OpenMP::OpenMP_CXX
          tesseract::environment
          tesseract::urdf
          tesseract::collision_bullet
          tesseract::collision_fcl
          tesseract::scene_graph)
target_compile_options(tesseract_environment_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                          ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(tesseract_environment_unit PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(tesseract_environment_unit ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(tesseract_environment_unit PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  tesseract_environment_unit
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
add_gtest_discover_tests(tesseract_environment_unit)
add_dependencies(tesseract_environment_unit tesseract::environment)

add_executable(tesseract_environment_collision tesseract_environment_collision.cpp)
target_link_libraries(tesseract_environment_collision PRIVATE GTest::GTest GTest::Main tesseract::environment)
target_compile_options(tesseract_environment_collision PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                               ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(tesseract_environment_collision PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(tesseract_environment_collision ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(tesseract_environment_collision PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  tesseract_environment_collision
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
add_gtest_discover_tests(tesseract_environment_collision)
add_dependencies(tesseract_environment_collision tesseract::environment)

add_executable(tesseract_environment_utils tesseract_environment_utils.cpp)
target_link_libraries(tesseract_environment_utils PRIVATE GTest::GTest GTest::Main tesseract::environment)
target_compile_options(tesseract_environment_utils PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                           ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(tesseract_environment_utils PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(tesseract_environment_utils ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(tesseract_environment_utils PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  tesseract_environment_utils
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
add_gtest_discover_tests(tesseract_environment_utils)
add_dependencies(tesseract_environment_utils tesseract::environment)

# Serialization Tests
add_executable(tesseract_environment_serialization_unit tesseract_environment_serialization_unit.cpp)
target_link_libraries(
  tesseract_environment_serialization_unit
  PRIVATE GTest::GTest
          GTest::Main
          tesseract::environment
          tesseract::common
          tesseract::urdf)
target_compile_options(tesseract_environment_serialization_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                                        ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_clang_tidy(tesseract_environment_serialization_unit ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(tesseract_environment_serialization_unit PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  tesseract_environment_serialization_unit
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
add_gtest_discover_tests(tesseract_environment_serialization_unit)
add_dependencies(tesseract_environment_serialization_unit tesseract::environment)

add_executable(tesseract_environment_cache_unit tesseract_environment_cache_unit.cpp)
target_link_libraries(tesseract_environment_cache_unit PRIVATE GTest::GTest GTest::Main tesseract::environment)
target_compile_options(tesseract_environment_cache_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                                ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(tesseract_environment_cache_unit PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(tesseract_environment_cache_unit ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(tesseract_environment_cache_unit PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  tesseract_environment_cache_unit
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
add_gtest_discover_tests(tesseract_environment_cache_unit)
add_dependencies(tesseract_environment_cache_unit tesseract::environment)
