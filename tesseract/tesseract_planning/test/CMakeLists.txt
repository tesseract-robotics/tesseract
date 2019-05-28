find_package(GTest REQUIRED)
find_package(tesseract_support REQUIRED)

# OMPL Planning Test/Example Program
add_executable(${PROJECT_NAME}_ompl_unit ompl_planner_tests.cpp)
target_link_libraries(${PROJECT_NAME}_ompl_unit ${GTEST_BOTH_LIBRARIES} ${PROJECT_NAME}_ompl tesseract_support)
target_compile_options(${PROJECT_NAME}_ompl_unit PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)
target_compile_features(${PROJECT_NAME}_ompl_unit PUBLIC cxx_std_11)
target_include_directories(${PROJECT_NAME}_ompl_unit PRIVATE ${GTEST_INCLUDE_DIRS})
gtest_discover_tests(${PROJECT_NAME}_ompl_unit)
add_dependencies(run_tests ${PROJECT_NAME}_ompl_unit)
