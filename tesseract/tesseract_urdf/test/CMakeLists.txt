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

target_link_libraries(${PROJECT_NAME}_unit ${GTEST_BOTH_LIBRARIES} ${PROJECT_NAME} tesseract::tesseract_support tesseract::tesseract_geometry)
target_compile_options(${PROJECT_NAME}_unit PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)
if(CXX_FEATURE_FOUND EQUAL "-1")
    target_compile_options(${PROJECT_NAME}_unit PRIVATE -std=c++11)
else()
    target_compile_features(${PROJECT_NAME}_unit PRIVATE cxx_std_11)
endif()
target_compile_options(${PROJECT_NAME}_unit PRIVATE -mno-avx)
target_include_directories(${PROJECT_NAME}_unit PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>")
target_include_directories(${PROJECT_NAME}_unit PRIVATE ${GTEST_INCLUDE_DIRS})
if(${CMAKE_VERSION} VERSION_LESS "3.10.0")
    gtest_add_tests(${PROJECT_NAME}_unit "" AUTO)
else()
    gtest_discover_tests(${PROJECT_NAME}_unit)
endif()
add_dependencies(${PROJECT_NAME}_unit ${PACKAGE_LIBRARIES})
add_dependencies(run_tests ${PROJECT_NAME}_unit)
