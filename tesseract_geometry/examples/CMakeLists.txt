find_package(tesseract_support REQUIRED)

add_executable(${PROJECT_NAME}_create_geometries_example create_geometries_example.cpp)
target_link_libraries(
  ${PROJECT_NAME}_create_geometries_example
  ${PROJECT_NAME}
  console_bridge::console_bridge
  tesseract::tesseract_support)
target_compile_options(${PROJECT_NAME}_create_geometries_example PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                                         ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_clang_tidy(${PROJECT_NAME}_create_geometries_example ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_create_geometries_example PRIVATE VERSION ${TESSERACT_CXX_VERSION})
add_dependencies(${PROJECT_NAME}_create_geometries_example ${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}_create_geometries_example DESTINATION bin)

add_executable(${PROJECT_NAME}_parse_mesh_example parse_mesh_example.cpp)
target_link_libraries(
  ${PROJECT_NAME}_parse_mesh_example
  ${PROJECT_NAME}
  console_bridge::console_bridge
  tesseract::tesseract_support)
target_compile_options(${PROJECT_NAME}_parse_mesh_example PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                                  ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_clang_tidy(${PROJECT_NAME}_parse_mesh_example ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_parse_mesh_example PRIVATE VERSION ${TESSERACT_CXX_VERSION})
add_dependencies(${PROJECT_NAME}_create_geometries_example ${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}_parse_mesh_example DESTINATION bin)
