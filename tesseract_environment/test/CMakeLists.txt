find_gtest()
find_package(tesseract_support REQUIRED)
find_package(OpenMP REQUIRED)
if(NOT TARGET OpenMP::OpenMP_CXX)
  find_package(Threads REQUIRED)
  add_library(OpenMP::OpenMP_CXX IMPORTED INTERFACE)
  set_property(TARGET OpenMP::OpenMP_CXX PROPERTY INTERFACE_COMPILE_OPTIONS ${OpenMP_CXX_FLAGS})
  # Only works if the same flag is passed to the linker; use CMake 3.9+ otherwise (Intel, AppleClang)
  set_property(TARGET OpenMP::OpenMP_CXX PROPERTY INTERFACE_LINK_LIBRARIES ${OpenMP_CXX_FLAGS} Threads::Threads)
endif()

add_executable(${PROJECT_NAME}_unit tesseract_environment_unit.cpp)
target_link_libraries(
  ${PROJECT_NAME}_unit
  PRIVATE GTest::GTest
          GTest::Main
          ${PROJECT_NAME}
          OpenMP::OpenMP_CXX
          tesseract::tesseract_support
          tesseract::tesseract_urdf
          tesseract::tesseract_collision_bullet
          tesseract::tesseract_collision_fcl
          tesseract::tesseract_scene_graph)
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
add_dependencies(${PROJECT_NAME}_unit ${PROJECT_NAME})
add_dependencies(run_tests ${PROJECT_NAME}_unit)

add_executable(${PROJECT_NAME}_collision tesseract_environment_collision.cpp)
target_link_libraries(
  ${PROJECT_NAME}_collision
  PRIVATE GTest::GTest
          GTest::Main
          ${PROJECT_NAME}
          tesseract::tesseract_support)
target_compile_options(${PROJECT_NAME}_collision PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                         ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_collision PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_collision ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_collision PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}_collision
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
add_gtest_discover_tests(${PROJECT_NAME}_collision)
add_dependencies(${PROJECT_NAME}_collision ${PROJECT_NAME})
add_dependencies(run_tests ${PROJECT_NAME}_collision)

add_executable(${PROJECT_NAME}_utils tesseract_environment_utils.cpp)
target_link_libraries(
  ${PROJECT_NAME}_utils
  PRIVATE GTest::GTest
          GTest::Main
          ${PROJECT_NAME}
          tesseract::tesseract_support)
target_compile_options(${PROJECT_NAME}_utils PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                     ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_utils PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_utils ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_utils PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}_utils
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
add_gtest_discover_tests(${PROJECT_NAME}_utils)
add_dependencies(${PROJECT_NAME}_utils ${PROJECT_NAME})
add_dependencies(run_tests ${PROJECT_NAME}_utils)

# Serialization Tests
add_executable(${PROJECT_NAME}_serialization_unit tesseract_environment_serialization_unit.cpp)
target_link_libraries(
  ${PROJECT_NAME}_serialization_unit
  PRIVATE GTest::GTest
          GTest::Main
          ${PROJECT_NAME}
          ${PROJECT_NAME}_commands
          tesseract::tesseract_common
          tesseract::tesseract_support
          tesseract::tesseract_urdf)
target_compile_options(${PROJECT_NAME}_serialization_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                                  ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_clang_tidy(${PROJECT_NAME}_serialization_unit ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_serialization_unit PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}_serialization_unit
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
add_gtest_discover_tests(${PROJECT_NAME}_serialization_unit)
add_dependencies(${PROJECT_NAME}_serialization_unit ${PROJECT_NAME}_commands)
add_dependencies(run_tests ${PROJECT_NAME}_serialization_unit)
