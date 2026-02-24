find_gtest()

add_executable(tesseract_scene_graph_unit tesseract_scene_graph_unit.cpp)
target_link_libraries(
  tesseract_scene_graph_unit
  PRIVATE GTest::GTest
          GTest::Main
          tesseract::scene_graph
          tesseract::geometry)
target_compile_options(tesseract_scene_graph_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                          ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(tesseract_scene_graph_unit PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(tesseract_scene_graph_unit ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(tesseract_scene_graph_unit PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  tesseract_scene_graph_unit
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
add_gtest_discover_tests(tesseract_scene_graph_unit)
add_dependencies(tesseract_scene_graph_unit tesseract::scene_graph)

add_executable(tesseract_scene_graph_joint_unit tesseract_scene_graph_joint_unit.cpp)
target_link_libraries(
  tesseract_scene_graph_joint_unit
  PRIVATE GTest::GTest
          GTest::Main
          tesseract::scene_graph
          tesseract::geometry)
target_compile_options(tesseract_scene_graph_joint_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                                ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_clang_tidy(tesseract_scene_graph_joint_unit ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(tesseract_scene_graph_joint_unit PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  tesseract_scene_graph_joint_unit
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
add_gtest_discover_tests(tesseract_scene_graph_joint_unit)
add_dependencies(tesseract_scene_graph_joint_unit tesseract::scene_graph)

add_executable(tesseract_scene_graph_link_unit tesseract_scene_graph_link_unit.cpp)
target_link_libraries(
  tesseract_scene_graph_link_unit
  PRIVATE GTest::GTest
          GTest::Main
          tesseract::scene_graph
          tesseract::geometry)
target_compile_options(tesseract_scene_graph_link_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                               ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_clang_tidy(tesseract_scene_graph_link_unit ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(tesseract_scene_graph_link_unit PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  tesseract_scene_graph_link_unit
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
add_gtest_discover_tests(tesseract_scene_graph_link_unit)
add_dependencies(tesseract_scene_graph_link_unit tesseract::scene_graph)

add_executable(tesseract_scene_graph_serialization_unit tesseract_scene_graph_serialization_unit.cpp)
target_link_libraries(
  tesseract_scene_graph_serialization_unit
  PRIVATE GTest::GTest
          GTest::Main
          tesseract::scene_graph
          tesseract::geometry)
target_compile_options(tesseract_scene_graph_serialization_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                                        ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_clang_tidy(tesseract_scene_graph_serialization_unit ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(tesseract_scene_graph_serialization_unit PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  tesseract_scene_graph_serialization_unit
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
add_gtest_discover_tests(tesseract_scene_graph_serialization_unit)
add_dependencies(tesseract_scene_graph_serialization_unit tesseract::scene_graph)
