find_package(tesseract_support REQUIRED)
find_package(tesseract_geometry REQUIRED)

add_executable(${PROJECT_NAME}_build_graph_example build_scene_graph_example.cpp)
target_link_libraries(${PROJECT_NAME}_build_graph_example ${PROJECT_NAME} console_bridge)
tesseract_target_compile_options(${PROJECT_NAME}_build_graph_example PRIVATE)
tesseract_clang_tidy(${PROJECT_NAME}_build_graph_example)
install(TARGETS ${PROJECT_NAME}_build_graph_example DESTINATION bin)

add_executable(${PROJECT_NAME}_parse_srdf_example parse_srdf_example.cpp)
target_link_libraries(${PROJECT_NAME}_parse_srdf_example ${PROJECT_NAME} console_bridge tesseract::tesseract_support)
tesseract_target_compile_options(${PROJECT_NAME}_parse_srdf_example PRIVATE)
tesseract_clang_tidy(${PROJECT_NAME}_parse_srdf_example)
install(TARGETS ${PROJECT_NAME}_parse_srdf_example DESTINATION bin)
