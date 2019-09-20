find_package(tesseract_support REQUIRED)
find_package(tesseract_geometry REQUIRED)

add_executable(${PROJECT_NAME}_load_urdf_example load_urdf_example.cpp)
target_link_libraries(${PROJECT_NAME}_load_urdf_example ${PROJECT_NAME} console_bridge tesseract::tesseract_support)
target_compile_options(${PROJECT_NAME}_load_urdf_example PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)
if(CXX_FEATURE_FOUND EQUAL "-1")
    target_compile_options(${PROJECT_NAME}_load_urdf_example PRIVATE -std=c++11)
else()
    target_compile_features(${PROJECT_NAME}_load_urdf_example PRIVATE cxx_std_11)
endif()
target_compile_options(${PROJECT_NAME}_load_urdf_example PRIVATE -mno-avx)

install(TARGETS ${PROJECT_NAME}_load_urdf_example DESTINATION bin)
