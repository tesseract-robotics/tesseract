
find_package(GTest REQUIRED)
find_package(tesseract_support REQUIRED)
find_package(tesseract_urdf REQUIRED)

add_executable(${PROJECT_NAME}_kdl_unit kdl_kinematics_unit.cpp)
target_link_libraries(${PROJECT_NAME}_kdl_unit GTest::GTest GTest::Main ${PROJECT_NAME}_kdl tesseract::tesseract_support tesseract::tesseract_urdf tesseract::tesseract_scene_graph)
tesseract_target_compile_options(${PROJECT_NAME}_kdl_unit PRIVATE)
tesseract_clang_tidy(${PROJECT_NAME}_kdl_unit)
target_include_directories(${PROJECT_NAME}_kdl_unit PRIVATE ${orocos_kdl_INCLUDE_DIRS})
tesseract_gtest_discover_tests(${PROJECT_NAME}_kdl_unit)
add_dependencies(${PROJECT_NAME}_kdl_unit ${PROJECT_NAME}_kdl )
add_dependencies(run_tests ${PROJECT_NAME}_kdl_unit)
