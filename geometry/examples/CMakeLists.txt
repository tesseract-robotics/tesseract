add_executable(tesseract_geometry_create_geometries_example create_geometries_example.cpp)
target_link_libraries(tesseract_geometry_create_geometries_example tesseract::geometry console_bridge::console_bridge)
target_compile_options(tesseract_geometry_create_geometries_example PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                                            ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_clang_tidy(tesseract_geometry_create_geometries_example ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(tesseract_geometry_create_geometries_example PRIVATE VERSION ${TESSERACT_CXX_VERSION})
add_dependencies(tesseract_geometry_create_geometries_example tesseract::geometry)

add_executable(tesseract_geometry_parse_mesh_example parse_mesh_example.cpp)
target_link_libraries(tesseract_geometry_parse_mesh_example tesseract::geometry console_bridge::console_bridge)
target_compile_options(tesseract_geometry_parse_mesh_example PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                                     ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_clang_tidy(tesseract_geometry_parse_mesh_example ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(tesseract_geometry_parse_mesh_example PRIVATE VERSION ${TESSERACT_CXX_VERSION})
add_dependencies(tesseract_geometry_parse_mesh_example tesseract::geometry)

install(TARGETS tesseract_geometry_parse_mesh_example tesseract_geometry_create_geometries_example DESTINATION bin
        COMPONENT geometry)
