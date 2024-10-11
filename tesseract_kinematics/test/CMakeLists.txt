find_gtest()

find_package(tesseract_urdf REQUIRED)
find_package(LAPACK REQUIRED) # Required for ikfast

add_executable(${PROJECT_NAME}_core_unit kinematics_core_unit.cpp)
target_link_libraries(
  ${PROJECT_NAME}_core_unit
  PRIVATE GTest::GTest
          GTest::Main
          ${PROJECT_NAME}_core
          ${PROJECT_NAME}_kdl
          tesseract::tesseract_urdf
          tesseract::tesseract_scene_graph
          tesseract::tesseract_common)
target_compile_options(${PROJECT_NAME}_core_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                         ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_core_unit PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_core_unit ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_core_unit PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}_core_unit
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
add_gtest_discover_tests(${PROJECT_NAME}_core_unit)
add_dependencies(${PROJECT_NAME}_core_unit ${PROJECT_NAME}_core)
add_dependencies(run_tests ${PROJECT_NAME}_core_unit)

add_executable(${PROJECT_NAME}_kdl_unit kdl_kinematics_unit.cpp)
target_link_libraries(
  ${PROJECT_NAME}_kdl_unit
  PRIVATE GTest::GTest
          GTest::Main
          ${PROJECT_NAME}_kdl
          tesseract::tesseract_urdf
          tesseract::tesseract_scene_graph
          tesseract::tesseract_common)
target_compile_options(${PROJECT_NAME}_kdl_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                        ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_kdl_unit PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_kdl_unit ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_kdl_unit PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}_kdl_unit
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
add_gtest_discover_tests(${PROJECT_NAME}_kdl_unit)
add_dependencies(${PROJECT_NAME}_kdl_unit ${PROJECT_NAME}_kdl)
add_dependencies(run_tests ${PROJECT_NAME}_kdl_unit)

add_executable(${PROJECT_NAME}_opw_unit opw_kinematics_unit.cpp)
target_link_libraries(
  ${PROJECT_NAME}_opw_unit
  PRIVATE GTest::GTest
          GTest::Main
          ${PROJECT_NAME}_kdl
          ${PROJECT_NAME}_opw
          tesseract::tesseract_urdf
          tesseract::tesseract_scene_graph)
target_compile_options(${PROJECT_NAME}_opw_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                        ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_opw_unit PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_opw_unit ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_opw_unit PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}_opw_unit
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
add_gtest_discover_tests(${PROJECT_NAME}_opw_unit)
add_dependencies(${PROJECT_NAME}_opw_unit ${PROJECT_NAME}_kdl ${PROJECT_NAME}_opw)
add_dependencies(run_tests ${PROJECT_NAME}_opw_unit)

add_executable(${PROJECT_NAME}_ikfast_unit ikfast_kinematics_unit.cpp abb_irb2400_ikfast_kinematics.cpp)
target_link_libraries(
  ${PROJECT_NAME}_ikfast_unit
  PRIVATE GTest::GTest
          GTest::Main
          ${PROJECT_NAME}_kdl
          ${PROJECT_NAME}_ikfast
          tesseract::tesseract_urdf
          tesseract::tesseract_scene_graph)
target_compile_options(${PROJECT_NAME}_ikfast_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                           ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_ikfast_unit PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
target_cxx_version(${PROJECT_NAME}_ikfast_unit PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}_ikfast_unit
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
add_gtest_discover_tests(${PROJECT_NAME}_ikfast_unit)
add_dependencies(${PROJECT_NAME}_ikfast_unit ${PROJECT_NAME}_kdl ${PROJECT_NAME}_ikfast)
add_dependencies(run_tests ${PROJECT_NAME}_ikfast_unit)

add_executable(${PROJECT_NAME}_ikfast_7dof_unit ikfast_kinematics_7dof_unit.cpp iiwa7_ikfast_kinematics.cpp)
target_link_libraries(
  ${PROJECT_NAME}_ikfast_7dof_unit
  PRIVATE GTest::GTest
          GTest::Main
          ${PROJECT_NAME}_kdl
          ${PROJECT_NAME}_ikfast
          tesseract::tesseract_urdf
          tesseract::tesseract_scene_graph)
target_compile_options(${PROJECT_NAME}_ikfast_7dof_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                                ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_ikfast_7dof_unit PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
target_cxx_version(${PROJECT_NAME}_ikfast_7dof_unit PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}_ikfast_7dof_unit
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
add_gtest_discover_tests(${PROJECT_NAME}_ikfast_7dof_unit)
add_dependencies(${PROJECT_NAME}_ikfast_7dof_unit ${PROJECT_NAME}_kdl ${PROJECT_NAME}_ikfast)
add_dependencies(run_tests ${PROJECT_NAME}_ikfast_7dof_unit)

add_executable(${PROJECT_NAME}_rop_unit rop_kinematics_unit.cpp)
target_link_libraries(
  ${PROJECT_NAME}_rop_unit
  PRIVATE GTest::GTest
          GTest::Main
          ${PROJECT_NAME}_kdl
          ${PROJECT_NAME}_opw
          tesseract::tesseract_urdf
          tesseract::tesseract_scene_graph)
target_compile_options(${PROJECT_NAME}_rop_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                        ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_rop_unit PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_rop_unit ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_rop_unit PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}_rop_unit
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
add_gtest_discover_tests(${PROJECT_NAME}_rop_unit)
add_dependencies(${PROJECT_NAME}_rop_unit ${PROJECT_NAME}_kdl ${PROJECT_NAME}_opw)
add_dependencies(run_tests ${PROJECT_NAME}_rop_unit)

add_executable(${PROJECT_NAME}_rep_unit rep_kinematics_unit.cpp)
target_link_libraries(
  ${PROJECT_NAME}_rep_unit
  PRIVATE GTest::GTest
          GTest::Main
          ${PROJECT_NAME}_kdl
          ${PROJECT_NAME}_opw
          tesseract::tesseract_urdf
          tesseract::tesseract_scene_graph)
target_compile_options(${PROJECT_NAME}_rep_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                        ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_rep_unit PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_rep_unit ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_rep_unit PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}_rep_unit
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
add_gtest_discover_tests(${PROJECT_NAME}_rep_unit)
add_dependencies(${PROJECT_NAME}_rep_unit ${PROJECT_NAME}_kdl ${PROJECT_NAME}_opw)
add_dependencies(run_tests ${PROJECT_NAME}_rep_unit)

add_executable(${PROJECT_NAME}_ur_unit ur_kinematics_unit.cpp)
target_link_libraries(
  ${PROJECT_NAME}_ur_unit
  PRIVATE GTest::GTest
          GTest::Main
          ${PROJECT_NAME}_kdl
          ${PROJECT_NAME}_ur
          tesseract::tesseract_urdf
          tesseract::tesseract_scene_graph)
target_compile_options(${PROJECT_NAME}_ur_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                       ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_ur_unit PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_ur_unit ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_ur_unit PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}_ur_unit
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
add_gtest_discover_tests(${PROJECT_NAME}_ur_unit)
add_dependencies(${PROJECT_NAME}_ur_unit ${PROJECT_NAME}_kdl)
add_dependencies(run_tests ${PROJECT_NAME}_ur_unit)

add_executable(${PROJECT_NAME}_factory_unit kinematics_factory_unit.cpp)
target_link_libraries(
  ${PROJECT_NAME}_factory_unit
  PRIVATE GTest::GTest
          GTest::Main
          ${PROJECT_NAME}_core
          tesseract::tesseract_urdf
          tesseract::tesseract_scene_graph)
target_compile_options(${PROJECT_NAME}_factory_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                            ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_factory_unit PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_factory_unit ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_factory_unit PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}_factory_unit
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
add_gtest_discover_tests(${PROJECT_NAME}_factory_unit)
add_dependencies(${PROJECT_NAME}_factory_unit ${PROJECT_NAME}_core)
add_dependencies(run_tests ${PROJECT_NAME}_factory_unit)
