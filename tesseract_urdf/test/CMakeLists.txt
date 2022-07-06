find_gtest()
find_package(tesseract_support REQUIRED)

add_executable(
  ${PROJECT_NAME}_unit
  tesseract_urdf_unit.cpp
  tesseract_urdf_box_unit.cpp
  tesseract_urdf_calibration_unit.cpp
  tesseract_urdf_capsule_unit.cpp
  tesseract_urdf_collision_unit.cpp
  tesseract_urdf_cone_unit.cpp
  tesseract_urdf_convex_mesh_unit.cpp
  tesseract_urdf_cylinder_unit.cpp
  tesseract_urdf_dynamics_unit.cpp
  tesseract_urdf_extra_delimeters_unit.cpp
  tesseract_urdf_geometry_unit.cpp
  tesseract_urdf_inertial_unit.cpp
  tesseract_urdf_joint_unit.cpp
  tesseract_urdf_limits_unit.cpp
  tesseract_urdf_link_unit.cpp
  tesseract_urdf_material_unit.cpp
  tesseract_urdf_mesh_unit.cpp
  tesseract_urdf_mesh_material_unit.cpp
  tesseract_urdf_mimic_unit.cpp
  tesseract_urdf_octree_unit.cpp
  tesseract_urdf_origin_unit.cpp
  tesseract_urdf_safety_controller_unit.cpp
  tesseract_urdf_sdf_mesh_unit.cpp
  tesseract_urdf_sphere_unit.cpp
  tesseract_urdf_urdf_unit.cpp
  tesseract_urdf_utils_unit.cpp
  tesseract_urdf_visual_unit.cpp)

target_link_libraries(
  ${PROJECT_NAME}_unit
  PRIVATE GTest::GTest
          GTest::Main
          ${PROJECT_NAME}
          tesseract::tesseract_support
          tesseract::tesseract_geometry)
target_include_directories(${PROJECT_NAME}_unit PRIVATE "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>")
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
