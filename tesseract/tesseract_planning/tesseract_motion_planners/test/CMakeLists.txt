find_package(GTest REQUIRED)
find_package(tesseract_support REQUIRED)

# Descartes Tesseract Kinematics Test
add_executable(${PROJECT_NAME}_descartes_kin_unit descartes_tesseract_kinematics_tests.cpp)
target_link_libraries(${PROJECT_NAME}_descartes_kin_unit GTest::GTest GTest::Main ${PROJECT_NAME}_descartes tesseract::tesseract_support)
tesseract_target_compile_options(${PROJECT_NAME}_descartes_kin_unit PRIVATE)
tesseract_clang_tidy(${PROJECT_NAME}_descartes_kin_unit)
tesseract_gtest_discover_tests(${PROJECT_NAME}_descartes_kin_unit)
add_dependencies(${PROJECT_NAME}_descartes_kin_unit ${PROJECT_NAME}_descartes)
add_dependencies(run_tests ${PROJECT_NAME}_descartes_kin_unit)

# OMPL Planning Test/Example Program
add_executable(${PROJECT_NAME}_ompl_unit ompl_planner_tests.cpp)
target_link_libraries(${PROJECT_NAME}_ompl_unit ${Boost_LIBRARIES} GTest::GTest GTest::Main ${PROJECT_NAME}_ompl_freespace tesseract::tesseract_support)
tesseract_target_compile_options(${PROJECT_NAME}_ompl_unit PRIVATE)
tesseract_clang_tidy(${PROJECT_NAME}_ompl_unit)
target_include_directories(${PROJECT_NAME}_ompl_unit SYSTEM PUBLIC
    ${Boost_INCLUDE_DIRS})
tesseract_gtest_discover_tests(${PROJECT_NAME}_ompl_unit)
add_dependencies(${PROJECT_NAME}_ompl_unit ${PROJECT_NAME}_ompl_freespace)
add_dependencies(run_tests ${PROJECT_NAME}_ompl_unit)

# TrajOpt Planner Tests
add_executable(${PROJECT_NAME}_trajopt_unit trajopt_planner_tests.cpp)
target_link_libraries(${PROJECT_NAME}_trajopt_unit GTest::GTest GTest::Main ${PROJECT_NAME}_trajopt tesseract::tesseract_support)
tesseract_target_compile_options(${PROJECT_NAME}_trajopt_unit PRIVATE)
tesseract_clang_tidy(${PROJECT_NAME}_trajopt_unit)
tesseract_gtest_discover_tests(${PROJECT_NAME}_trajopt_unit)
add_dependencies(${PROJECT_NAME}_trajopt_unit ${PROJECT_NAME}_trajopt)
add_dependencies(run_tests ${PROJECT_NAME}_trajopt_unit)

# Descartes Planner Tests
add_executable(${PROJECT_NAME}_descartes_unit descartes_planner_tests.cpp)
target_link_libraries(${PROJECT_NAME}_descartes_unit GTest::GTest GTest::Main ${PROJECT_NAME}_descartes tesseract::tesseract_support tesseract::tesseract_kinematics_opw)
tesseract_target_compile_options(${PROJECT_NAME}_descartes_unit PRIVATE)
tesseract_clang_tidy(${PROJECT_NAME}_descartes_unit)
tesseract_gtest_discover_tests(${PROJECT_NAME}_descartes_unit)
add_dependencies(${PROJECT_NAME}_trajopt_unit ${PROJECT_NAME}_descartes)
add_dependencies(run_tests ${PROJECT_NAME}_descartes_unit)

# OMPL TrajOpt Planning Test
add_executable(${PROJECT_NAME}_ompl_trajopt_unit ompl_trajopt_planner_test.cpp)
target_link_libraries(${PROJECT_NAME}_ompl_trajopt_unit ${Boost_LIBRARIES} GTest::GTest GTest::Main ${PROJECT_NAME}_ompl_trajopt_freespace tesseract::tesseract_support)
tesseract_target_compile_options(${PROJECT_NAME}_ompl_trajopt_unit PRIVATE)
tesseract_clang_tidy(${PROJECT_NAME}_ompl_trajopt_unit)
target_include_directories(${PROJECT_NAME}_ompl_trajopt_unit SYSTEM PUBLIC
    ${Boost_INCLUDE_DIRS})
tesseract_gtest_discover_tests(${PROJECT_NAME}_ompl_trajopt_unit)
add_dependencies(${PROJECT_NAME}_ompl_trajopt_unit ${PROJECT_NAME}_ompl_trajopt_freespace)
add_dependencies(run_tests ${PROJECT_NAME}_ompl_trajopt_unit)
