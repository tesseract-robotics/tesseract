find_package(OpenMP REQUIRED)
if(NOT TARGET OpenMP::OpenMP_CXX)
  find_package(Threads REQUIRED)
  add_library(OpenMP::OpenMP_CXX IMPORTED INTERFACE)
  set_property(TARGET OpenMP::OpenMP_CXX PROPERTY INTERFACE_COMPILE_OPTIONS ${OpenMP_CXX_FLAGS})
  # Only works if the same flag is passed to the linker; use CMake 3.9+ otherwise (Intel, AppleClang)
  set_property(TARGET OpenMP::OpenMP_CXX PROPERTY INTERFACE_LINK_LIBRARIES ${OpenMP_CXX_FLAGS} Threads::Threads)
endif()

find_gtest()

macro(add_gtest test_name test_file)
  add_executable(${test_name} ${test_file})
  target_link_libraries(
    ${test_name}
    PRIVATE GTest::GTest
            GTest::Main
            tesseract::collision
            tesseract::collision_test_suite
            tesseract::collision_bullet
            tesseract::collision_fcl
            tesseract::geometry
            tesseract::scene_graph
            console_bridge::console_bridge
            OpenMP::OpenMP_CXX
            Eigen3::Eigen
            ${Boost_LIBRARIES}
            octomap
            octomath
            ${LIBFCL_LIBRARIES})
  target_compile_options(${test_name} PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE} ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
  target_compile_definitions(${test_name} PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
  target_cxx_version(${test_name} PRIVATE VERSION ${TESSERACT_CXX_VERSION})
  target_clang_tidy(${test_name} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
  target_code_coverage(
    ${test_name}
    PRIVATE
    ALL
    EXCLUDE ${COVERAGE_EXCLUDE}
    ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
  add_gtest_discover_tests(${test_name})
  add_dependencies(
    ${test_name}
    tesseract::collision
    tesseract::collision_test_suite
    tesseract::collision_bullet
    tesseract::collision_fcl)
endmacro()

add_gtest(collision_box_sphere_unit collision_box_sphere_unit.cpp)
add_gtest(collision_box_cylinder_unit collision_box_cylinder_unit.cpp)
add_gtest(collision_box_cone_unit collision_box_cone_unit.cpp)
add_gtest(collision_box_box_unit collision_box_box_unit.cpp)
add_gtest(collision_box_capsule_unit collision_box_capsule_unit.cpp)
add_gtest(collision_large_dataset_unit collision_large_dataset_unit.cpp)
add_gtest(collision_sphere_sphere_unit collision_sphere_sphere_unit.cpp)
add_gtest(collision_mesh_mesh_unit collision_mesh_mesh_unit.cpp)
add_gtest(collision_multi_threaded_unit collision_multi_threaded_unit.cpp)
add_gtest(collision_octomap_sphere_unit collision_octomap_sphere_unit.cpp)
add_gtest(collision_octomap_mesh_unit collision_octomap_mesh_unit.cpp)
add_gtest(collision_clone_unit collision_clone_unit.cpp)
add_gtest(collision_box_box_cast_unit collision_box_box_cast_unit.cpp)
add_gtest(collision_compound_compound_unit collision_compound_compound_unit.cpp)
add_gtest(collision_compound_mesh_sphere_unit collision_compound_mesh_sphere_unit.cpp)
add_gtest(collision_sphere_sphere_cast_unit collision_sphere_sphere_cast_unit.cpp)
add_gtest(collision_octomap_octomap_unit collision_octomap_octomap_unit.cpp)
add_gtest(collision_factory_unit contact_managers_factory_unit.cpp)
add_gtest(collision_core_unit collision_core_unit.cpp)
add_gtest(collision_config_unit contact_managers_config_unit.cpp)

add_gtest(collision_factory_static_unit contact_managers_factory_static_unit.cpp)
target_link_libraries(collision_factory_static_unit PRIVATE tesseract::collision_bullet_factories)
