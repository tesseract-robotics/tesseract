find_package(GTest REQUIRED)
find_package(tesseract_support REQUIRED)

add_executable(${PROJECT_NAME}_unit
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
  tesseract_urdf_mimic_unit.cpp
  tesseract_urdf_octree_unit.cpp
  tesseract_urdf_origin_unit.cpp
  tesseract_urdf_safety_controller_unit.cpp
  tesseract_urdf_sdf_mesh_unit.cpp
  tesseract_urdf_sphere_unit.cpp
  tesseract_urdf_urdf_unit.cpp
  tesseract_urdf_visual_unit.cpp)

target_link_libraries(${PROJECT_NAME}_unit GTest::GTest GTest::Main ${PROJECT_NAME} tesseract::tesseract_support tesseract::tesseract_geometry)
tesseract_target_compile_options(${PROJECT_NAME}_unit PRIVATE)
tesseract_clang_tidy(${PROJECT_NAME}_unit)
target_include_directories(${PROJECT_NAME}_unit PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>")
tesseract_gtest_discover_tests(${PROJECT_NAME}_unit)
add_dependencies(${PROJECT_NAME}_unit ${PROJECT_NAME})
add_dependencies(run_tests ${PROJECT_NAME}_unit)
