find_package(tesseract_support REQUIRED)
find_package(tesseract_geometry REQUIRED)

add_executable(${PROJECT_NAME}_load_urdf_example load_urdf_example.cpp)
target_link_libraries(${PROJECT_NAME}_load_urdf_example ${PROJECT_NAME} console_bridge tesseract::tesseract_support)
tesseract_target_compile_options(${PROJECT_NAME}_load_urdf_example PRIVATE)
tesseract_clang_tidy(${PROJECT_NAME}_load_urdf_example)
install(TARGETS ${PROJECT_NAME}_load_urdf_example DESTINATION bin)
