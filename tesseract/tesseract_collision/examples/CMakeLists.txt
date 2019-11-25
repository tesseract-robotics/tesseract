add_executable(${PROJECT_NAME}_box_box_example box_box_example.cpp)
target_link_libraries(${PROJECT_NAME}_box_box_example ${PROJECT_NAME}_bullet ${PROJECT_NAME}_fcl tesseract::tesseract_geometry console_bridge ${BULLET_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES} ${${LIBFCL_LIBRARIES}})
tesseract_target_compile_options(${PROJECT_NAME}_box_box_example PRIVATE)
target_compile_definitions(${PROJECT_NAME}_box_box_example PRIVATE DATA_DIR="${CMAKE_SOURCE_DIR}/test")

install(TARGETS ${PROJECT_NAME}_box_box_example DESTINATION bin)
