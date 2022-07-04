find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
else(OPENMP_FOUND)
  message(STATUS "Not found OpenMP")
endif()

find_package(tesseract_scene_graph REQUIRED)

find_gtest()

macro(add_gtest test_name test_file)
  add_executable(${test_name} ${test_file})
  target_link_libraries(
    ${test_name}
    PRIVATE GTest::GTest
            GTest::Main
            ${PROJECT_NAME}_core
            ${PROJECT_NAME}_test_suite
            ${PROJECT_NAME}_bullet
            ${PROJECT_NAME}_fcl
            tesseract::tesseract_geometry
            tesseract::tesseract_scene_graph
            console_bridge::console_bridge
            ${Boost_LIBRARIES}
            octomap
            octomath
            ${LIBFCL_LIBRARIES})
  target_compile_options(${test_name} PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE} ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
  target_compile_definitions(${test_name} PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
  target_clang_tidy(${test_name} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
  target_cxx_version(${test_name} PRIVATE VERSION ${TESSERACT_CXX_VERSION})
  target_code_coverage(
    ${test_name}
    PRIVATE
    ALL
    EXCLUDE ${COVERAGE_EXCLUDE}
    ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
  add_gtest_discover_tests(${test_name})
  add_dependencies(
    ${test_name}
    ${PROJECT_NAME}_core
    ${PROJECT_NAME}_test_suite
    ${PROJECT_NAME}_bullet
    ${PROJECT_NAME}_fcl)
  add_dependencies(run_tests ${test_name})
endmacro()

add_gtest(${PROJECT_NAME}_box_sphere_unit collision_box_sphere_unit.cpp)
add_gtest(${PROJECT_NAME}_box_cylinder_unit collision_box_cylinder_unit.cpp)
add_gtest(${PROJECT_NAME}_box_cone_unit collision_box_cone_unit.cpp)
add_gtest(${PROJECT_NAME}_box_box_unit collision_box_box_unit.cpp)
add_gtest(${PROJECT_NAME}_box_capsule_unit collision_box_capsule_unit.cpp)
add_gtest(${PROJECT_NAME}_large_dataset_unit collision_large_dataset_unit.cpp)
add_gtest(${PROJECT_NAME}_sphere_sphere_unit collision_sphere_sphere_unit.cpp)
add_gtest(${PROJECT_NAME}_mesh_mesh_unit collision_mesh_mesh_unit.cpp)
add_gtest(${PROJECT_NAME}_multi_threaded_unit collision_multi_threaded_unit.cpp)
add_gtest(${PROJECT_NAME}_octomap_sphere_unit collision_octomap_sphere_unit.cpp)
add_gtest(${PROJECT_NAME}_octomap_mesh_unit collision_octomap_mesh_unit.cpp)
add_gtest(${PROJECT_NAME}_clone_unit collision_clone_unit.cpp)
add_gtest(${PROJECT_NAME}_box_box_cast_unit collision_box_box_cast_unit.cpp)
add_gtest(${PROJECT_NAME}_compound_compound_unit collision_compound_compound_unit.cpp)
add_gtest(${PROJECT_NAME}_sphere_sphere_cast_unit collision_sphere_sphere_cast_unit.cpp)
add_gtest(${PROJECT_NAME}_octomap_octomap_unit collision_octomap_octomap_unit.cpp)
add_gtest(${PROJECT_NAME}_collision_margin_data_unit collision_margin_data_unit.cpp)
add_gtest(${PROJECT_NAME}_factory_unit contact_managers_factory_unit.cpp)
add_gtest(${PROJECT_NAME}_core_unit collision_core_unit.cpp)

add_gtest(${PROJECT_NAME}_factory_static_unit contact_managers_factory_static_unit.cpp)
target_link_libraries(${PROJECT_NAME}_factory_static_unit PRIVATE ${PROJECT_NAME}_bullet_factories)
