find_package(tesseract_support REQUIRED)
add_executable(${PROJECT_NAME}_box_box_example box_box_example.cpp)
target_link_libraries(
  ${PROJECT_NAME}_box_box_example
  ${PROJECT_NAME}_bullet
  ${PROJECT_NAME}_fcl
  tesseract::tesseract_support
  tesseract::tesseract_geometry
  console_bridge::console_bridge
  ${Boost_LIBRARIES}
  octomap
  octomath)
target_compile_options(${PROJECT_NAME}_box_box_example PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                               ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_cxx_version(${PROJECT_NAME}_box_box_example PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_clang_tidy(${PROJECT_NAME}_box_box_example ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_compile_definitions(${PROJECT_NAME}_box_box_example PRIVATE DATA_DIR="${CMAKE_SOURCE_DIR}/test")

install(TARGETS ${PROJECT_NAME}_box_box_example DESTINATION bin)
