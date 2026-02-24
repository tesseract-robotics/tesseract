add_executable(tesseract_scene_graph_build_graph_example build_scene_graph_example.cpp)
target_link_libraries(
  tesseract_scene_graph_build_graph_example
  tesseract::scene_graph
  tesseract::common
  console_bridge::console_bridge)
target_compile_options(tesseract_scene_graph_build_graph_example PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                                         ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_clang_tidy(tesseract_scene_graph_build_graph_example ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(tesseract_scene_graph_build_graph_example PRIVATE VERSION ${TESSERACT_CXX_VERSION})
add_dependencies(tesseract_scene_graph_build_graph_example tesseract::scene_graph)
install(TARGETS tesseract_scene_graph_build_graph_example DESTINATION bin COMPONENT scene_graph)
