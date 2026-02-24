add_executable(tesseract_urdf_load_urdf_example load_urdf_example.cpp)
target_link_libraries(tesseract_urdf_load_urdf_example tesseract::urdf console_bridge::console_bridge)
target_compile_options(tesseract_urdf_load_urdf_example PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                                ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_clang_tidy(tesseract_urdf_load_urdf_example ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(tesseract_urdf_load_urdf_example PRIVATE VERSION ${TESSERACT_CXX_VERSION})
install(TARGETS tesseract_urdf_load_urdf_example DESTINATION bin COMPONENT urdf)
