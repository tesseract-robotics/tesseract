find_package(tesseract_support REQUIRED)

add_executable(${PROJECT_NAME}_create_geometries_example create_geometries_example.cpp)
target_link_libraries(${PROJECT_NAME}_create_geometries_example ${PROJECT_NAME} console_bridge tesseract::tesseract_support)
tesseract_target_compile_options(${PROJECT_NAME}_create_geometries_example PRIVATE)
tesseract_clang_tidy(${PROJECT_NAME}_create_geometries_example)
install(TARGETS ${PROJECT_NAME}_create_geometries_example DESTINATION bin)

add_executable(${PROJECT_NAME}_parse_mesh_example parse_mesh_example.cpp)
target_link_libraries(${PROJECT_NAME}_parse_mesh_example ${PROJECT_NAME} console_bridge tesseract::tesseract_support)
tesseract_target_compile_options(${PROJECT_NAME}_parse_mesh_example PRIVATE)
tesseract_clang_tidy(${PROJECT_NAME}_parse_mesh_example)
install(TARGETS ${PROJECT_NAME}_parse_mesh_example DESTINATION bin)
