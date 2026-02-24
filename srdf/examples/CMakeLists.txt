add_executable(tesseract_srdf_parse_srdf_example parse_srdf_example.cpp)
target_link_libraries(
  tesseract_srdf_parse_srdf_example
  tesseract::srdf
  tesseract::common
  console_bridge::console_bridge)
target_compile_options(tesseract_srdf_parse_srdf_example PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                                 ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_clang_tidy(tesseract_srdf_parse_srdf_example ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(tesseract_srdf_parse_srdf_example PRIVATE VERSION ${TESSERACT_CXX_VERSION})
add_dependencies(tesseract_srdf_parse_srdf_example tesseract::srdf)
install(TARGETS tesseract_srdf_parse_srdf_example DESTINATION bin COMPONENT srdf)
