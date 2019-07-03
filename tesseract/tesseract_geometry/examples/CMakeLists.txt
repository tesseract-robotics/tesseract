find_package(tesseract_support REQUIRED)

add_executable(${PROJECT_NAME}_create_geometries_example create_geometries_example.cpp)
target_link_libraries(${PROJECT_NAME}_create_geometries_example ${PROJECT_NAME} console_bridge)
target_compile_options(${PROJECT_NAME}_create_geometries_example PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)
if(CXX_FEATURE_FOUND EQUAL "-1")
    target_compile_options(${PROJECT_NAME}_create_geometries_example PUBLIC -std=c++11)
else()
    target_compile_features(${PROJECT_NAME}_create_geometries_example PRIVATE cxx_std_11)
endif()

install(TARGETS ${PROJECT_NAME}_create_geometries_example DESTINATION bin)
