find_gtest()

find_package(LAPACK REQUIRED) # Required for ikfast

add_executable(tesseract_kinematics_core_unit kinematics_core_unit.cpp)
target_link_libraries(
  tesseract_kinematics_core_unit
  PRIVATE GTest::GTest
          GTest::Main
          tesseract::kinematics
          tesseract::kinematics_kdl
          tesseract::urdf
          tesseract::scene_graph
          tesseract::common)
target_compile_options(tesseract_kinematics_core_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                              ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(tesseract_kinematics_core_unit PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(tesseract_kinematics_core_unit ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(tesseract_kinematics_core_unit PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  tesseract_kinematics_core_unit
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
add_gtest_discover_tests(tesseract_kinematics_core_unit)
add_dependencies(tesseract_kinematics_core_unit tesseract::kinematics)

add_executable(tesseract_kinematics_kdl_unit kdl_kinematics_unit.cpp)
target_link_libraries(
  tesseract_kinematics_kdl_unit
  PRIVATE GTest::GTest
          GTest::Main
          tesseract::kinematics_kdl
          tesseract::urdf
          tesseract::scene_graph
          tesseract::common)
target_compile_options(tesseract_kinematics_kdl_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                             ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(tesseract_kinematics_kdl_unit PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(tesseract_kinematics_kdl_unit ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(tesseract_kinematics_kdl_unit PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  tesseract_kinematics_kdl_unit
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
add_gtest_discover_tests(tesseract_kinematics_kdl_unit)
add_dependencies(tesseract_kinematics_kdl_unit tesseract::kinematics_kdl)

add_executable(tesseract_kinematics_opw_unit opw_kinematics_unit.cpp)
target_link_libraries(
  tesseract_kinematics_opw_unit
  PRIVATE GTest::GTest
          GTest::Main
          tesseract::kinematics_kdl
          tesseract::kinematics_opw
          tesseract::urdf
          tesseract::scene_graph)
target_compile_options(tesseract_kinematics_opw_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                             ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(tesseract_kinematics_opw_unit PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(tesseract_kinematics_opw_unit ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(tesseract_kinematics_opw_unit PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  tesseract_kinematics_opw_unit
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
add_gtest_discover_tests(tesseract_kinematics_opw_unit)
add_dependencies(tesseract_kinematics_opw_unit tesseract::kinematics_kdl tesseract::kinematics_opw)

add_library(tesseract_kinematics_ikfast_abb_irb2400 abb_irb2400_ikfast_kinematics.cpp)
add_library(tesseract::kinematics_ikfast_abb_irb2400 ALIAS tesseract_kinematics_ikfast_abb_irb2400)
target_link_libraries(
  tesseract_kinematics_ikfast_abb_irb2400
  PRIVATE GTest::GTest
          GTest::Main
          tesseract::kinematics_ikfast
          tesseract::urdf
          tesseract::scene_graph)
target_compile_options(tesseract_kinematics_ikfast_abb_irb2400 PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                                       ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(tesseract_kinematics_ikfast_abb_irb2400 PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
target_cxx_version(tesseract_kinematics_ikfast_abb_irb2400 PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  tesseract_kinematics_ikfast_abb_irb2400
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})

add_executable(tesseract_kinematics_ikfast_unit ikfast_kinematics_unit.cpp)
target_link_libraries(
  tesseract_kinematics_ikfast_unit
  PRIVATE tesseract::kinematics_ikfast_abb_irb2400
          GTest::GTest
          GTest::Main
          tesseract::kinematics_kdl
          tesseract::kinematics_ikfast
          tesseract::urdf
          tesseract::scene_graph)
target_compile_options(tesseract_kinematics_ikfast_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                                ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(tesseract_kinematics_ikfast_unit PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
target_cxx_version(tesseract_kinematics_ikfast_unit PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  tesseract_kinematics_ikfast_unit
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
add_gtest_discover_tests(tesseract_kinematics_ikfast_unit)
add_dependencies(tesseract_kinematics_ikfast_unit tesseract::kinematics_kdl tesseract::kinematics_ikfast)

add_executable(tesseract_kinematics_ikfast_7dof_unit ikfast_kinematics_7dof_unit.cpp iiwa7_ikfast_kinematics.cpp)
target_link_libraries(
  tesseract_kinematics_ikfast_7dof_unit
  PRIVATE GTest::GTest
          GTest::Main
          tesseract::kinematics_kdl
          tesseract::kinematics_ikfast
          tesseract::urdf
          tesseract::scene_graph)
target_compile_options(tesseract_kinematics_ikfast_7dof_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                                     ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(tesseract_kinematics_ikfast_7dof_unit PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
target_cxx_version(tesseract_kinematics_ikfast_7dof_unit PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  tesseract_kinematics_ikfast_7dof_unit
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
add_gtest_discover_tests(tesseract_kinematics_ikfast_7dof_unit)
add_dependencies(tesseract_kinematics_ikfast_7dof_unit tesseract::kinematics_kdl tesseract::kinematics_ikfast)

add_executable(tesseract_kinematics_rop_unit rop_kinematics_unit.cpp)
target_link_libraries(
  tesseract_kinematics_rop_unit
  PRIVATE GTest::GTest
          GTest::Main
          tesseract::kinematics_kdl
          tesseract::kinematics_opw
          tesseract::urdf
          tesseract::scene_graph)
target_compile_options(tesseract_kinematics_rop_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                             ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(tesseract_kinematics_rop_unit PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(tesseract_kinematics_rop_unit ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(tesseract_kinematics_rop_unit PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  tesseract_kinematics_rop_unit
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
add_gtest_discover_tests(tesseract_kinematics_rop_unit)
add_dependencies(tesseract_kinematics_rop_unit tesseract::kinematics_kdl tesseract::kinematics_opw)

add_executable(tesseract_kinematics_rep_unit rep_kinematics_unit.cpp)
target_link_libraries(
  tesseract_kinematics_rep_unit
  PRIVATE GTest::GTest
          GTest::Main
          tesseract::kinematics_kdl
          tesseract::kinematics_opw
          tesseract::urdf
          tesseract::scene_graph)
target_compile_options(tesseract_kinematics_rep_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                             ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(tesseract_kinematics_rep_unit PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(tesseract_kinematics_rep_unit ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(tesseract_kinematics_rep_unit PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  tesseract_kinematics_rep_unit
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
add_gtest_discover_tests(tesseract_kinematics_rep_unit)
add_dependencies(tesseract_kinematics_rep_unit tesseract::kinematics_kdl tesseract::kinematics_opw)

add_executable(tesseract_kinematics_ur_unit ur_kinematics_unit.cpp)
target_link_libraries(
  tesseract_kinematics_ur_unit
  PRIVATE GTest::GTest
          GTest::Main
          tesseract::kinematics_kdl
          tesseract::kinematics_ur
          tesseract::urdf
          tesseract::scene_graph)
target_compile_options(tesseract_kinematics_ur_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                            ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(tesseract_kinematics_ur_unit PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(tesseract_kinematics_ur_unit ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(tesseract_kinematics_ur_unit PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  tesseract_kinematics_ur_unit
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
add_gtest_discover_tests(tesseract_kinematics_ur_unit)
add_dependencies(tesseract_kinematics_ur_unit tesseract::kinematics_kdl)

add_executable(tesseract_kinematics_factory_unit kinematics_factory_unit.cpp)
target_link_libraries(
  tesseract_kinematics_factory_unit
  PRIVATE GTest::GTest
          GTest::Main
          tesseract::kinematics
          tesseract::urdf
          tesseract::scene_graph)
target_compile_options(tesseract_kinematics_factory_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                                 ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(tesseract_kinematics_factory_unit PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
target_compile_definitions(tesseract_kinematics_factory_unit PRIVATE PLUGIN_DIR="${CMAKE_CURRENT_BINARY_DIR}")

target_clang_tidy(tesseract_kinematics_factory_unit ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(tesseract_kinematics_factory_unit PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  tesseract_kinematics_factory_unit
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
add_gtest_discover_tests(tesseract_kinematics_factory_unit)
add_dependencies(tesseract_kinematics_factory_unit tesseract::kinematics)
