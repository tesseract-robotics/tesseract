find_package(tesseract_support REQUIRED)
find_package(tesseract_visualization REQUIRED)

add_executable(${PROJECT_NAME}_chain_example chain_example.cpp)
target_link_libraries(${PROJECT_NAME}_chain_example ${PROJECT_NAME}_descartes ${PROJECT_NAME}_trajopt ${PROJECT_NAME}_simple tesseract::tesseract_support tesseract::tesseract_visualization)
tesseract_target_compile_options(${PROJECT_NAME}_chain_example PRIVATE)
tesseract_clang_tidy(${PROJECT_NAME}_chain_example)

add_executable(${PROJECT_NAME}_raster_example raster_example.cpp)
target_link_libraries(${PROJECT_NAME}_raster_example ${PROJECT_NAME}_descartes ${PROJECT_NAME}_trajopt ${PROJECT_NAME}_simple tesseract::tesseract_support tesseract::tesseract_visualization)
tesseract_target_compile_options(${PROJECT_NAME}_raster_example PRIVATE)
tesseract_clang_tidy(${PROJECT_NAME}_raster_example)

add_executable(${PROJECT_NAME}_freespace_example freespace_example.cpp)
target_link_libraries(${PROJECT_NAME}_freespace_example ${PROJECT_NAME}_ompl ${PROJECT_NAME}_trajopt ${PROJECT_NAME}_simple tesseract::tesseract_support tesseract::tesseract_visualization)
tesseract_target_compile_options(${PROJECT_NAME}_freespace_example PRIVATE)
tesseract_clang_tidy(${PROJECT_NAME}_freespace_example)

install(TARGETS ${PROJECT_NAME}_raster_example ${PROJECT_NAME}_freespace_example DESTINATION bin)
