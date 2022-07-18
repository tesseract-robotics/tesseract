find_package(benchmark REQUIRED)
find_package(tesseract_scene_graph REQUIRED)

macro(add_benchmark benchmark_name benchmark_file)
  add_executable(${benchmark_name} ${benchmark_file})
  target_compile_definitions(${benchmark_name} PRIVATE BENCHMARK_ARGS="${BENCHMARK_ARGS}")
  target_compile_options(${benchmark_name} PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                   ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
  target_compile_definitions(${benchmark_name} PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
  target_cxx_version(${benchmark_name} PRIVATE VERSION ${TESSERACT_CXX_VERSION})
  target_link_libraries(
    ${benchmark_name}
    benchmark::benchmark
    ${PROJECT_NAME}_test_suite_benchmarks
    ${PROJECT_NAME}_bullet
    ${PROJECT_NAME}_fcl
    tesseract::tesseract_geometry
    tesseract::tesseract_scene_graph
    Bullet3::Bullet
    console_bridge
    ${Boost_LIBRARIES}
    octomap
    octomath
    ${LIBFCL_LIBRARIES})
  target_clang_tidy(${benchmark_name} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
  target_include_directories(${benchmark_name} PRIVATE "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>")
  add_dependencies(
    ${benchmark_name}
    ${PROJECT_NAME}_test_suite_benchmarks
    ${PROJECT_NAME}_bullet
    ${PROJECT_NAME}_fcl)
  add_run_benchmark_target(${benchmark_name})
endmacro()

add_benchmark(${PROJECT_NAME}_bullet_discrete_simple_benchmarks bullet_discrete_simple_benchmarks.cpp)
add_benchmark(${PROJECT_NAME}_bullet_discrete_bvh_benchmarks bullet_discrete_bvh_benchmarks.cpp)
add_benchmark(${PROJECT_NAME}_fcl_discrete_bvh_benchmarks fcl_discrete_bvh_benchmarks.cpp)

# Create target that profiles the collision checkers.
add_executable(${PROJECT_NAME}_profile collision_profile.cpp)
target_link_libraries(
  ${PROJECT_NAME}_profile
  PRIVATE ${PROJECT_NAME}_bullet
          ${PROJECT_NAME}_fcl
          tesseract::tesseract_geometry
          tesseract::tesseract_support
          Bullet3::Bullet
          ${Boost_LIBRARIES}
          ${OCTOMAP_LIBRARIES}
          ${LIBFCL_LIBRARIES})
target_compile_options(${PROJECT_NAME}_profile PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_definitions(${PROJECT_NAME}_profile INTERFACE ${TESSERACT_COMPILE_DEFINITIONS})
target_cxx_version(${PROJECT_NAME}_profile PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_clang_tidy(${PROJECT_NAME}_profile ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_code_coverage(
  ${PROJECT_NAME}_profile
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})

install_targets(TARGETS ${PROJECT_NAME}_profile)
