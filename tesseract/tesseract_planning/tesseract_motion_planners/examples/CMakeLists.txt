find_package(tesseract_support REQUIRED)

add_executable(${PROJECT_NAME}_raster_example raster_example.cpp)
target_link_libraries(${PROJECT_NAME}_raster_example ${PROJECT_NAME}_descartes ${PROJECT_NAME}_trajopt ${PROJECT_NAME}_simple tesseract::tesseract_support)
tesseract_target_compile_options(${PROJECT_NAME}_raster_example PRIVATE)
tesseract_clang_tidy(${PROJECT_NAME}_raster_example)
target_compile_definitions(${PROJECT_NAME}_raster_example PRIVATE DATA_DIR="${CMAKE_SOURCE_DIR}/test")

install(TARGETS ${PROJECT_NAME}_raster_example DESTINATION bin)
