find_package(tesseract_support REQUIRED)
find_package(tesseract_geometry REQUIRED)

add_executable(${PROJECT_NAME}_load_urdf_example load_urdf_example.cpp)
target_link_libraries(
  ${PROJECT_NAME}_load_urdf_example
  ${PROJECT_NAME}
  console_bridge::console_bridge
  tesseract::tesseract_support)
target_compile_options(${PROJECT_NAME}_load_urdf_example PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                                 ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_clang_tidy(${PROJECT_NAME}_load_urdf_example ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_load_urdf_example PRIVATE VERSION ${TESSERACT_CXX_VERSION})
install(TARGETS ${PROJECT_NAME}_load_urdf_example DESTINATION bin)
