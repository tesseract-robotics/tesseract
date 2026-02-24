find_package(benchmark REQUIRED)

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
    tesseract::collision_test_suite_benchmarks
    tesseract::collision_bullet
    tesseract::collision_fcl
    tesseract::geometry
    tesseract::scene_graph
    console_bridge::console_bridge
    ${Boost_LIBRARIES}
    octomap
    octomath
    ${LIBFCL_LIBRARIES})
  target_clang_tidy(${benchmark_name} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
  target_include_directories(${benchmark_name} PRIVATE "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>")
  add_dependencies(
    ${benchmark_name}
    tesseract::collision_test_suite_benchmarks
    tesseract::collision_bullet
    tesseract::collision_fcl)
  if(TESSERACT_ENABLE_RUN_BENCHMARKING)
    message(STATUS "Running benchmark ${benchmark_name}")
    add_run_benchmark_target(${benchmark_name})
  endif()
endmacro()

add_benchmark(collision_bullet_discrete_simple_benchmarks bullet_discrete_simple_benchmarks.cpp)
add_benchmark(collision_bullet_discrete_bvh_benchmarks bullet_discrete_bvh_benchmarks.cpp)
add_benchmark(collision_fcl_discrete_bvh_benchmarks fcl_discrete_bvh_benchmarks.cpp)

# Create target that profiles the collision checkers.
add_executable(tesseract_collision_profile collision_profile.cpp)
target_link_libraries(
  tesseract_collision_profile
  PRIVATE tesseract::collision_bullet
          tesseract::collision_fcl
          tesseract::geometry
          ${Boost_LIBRARIES}
          ${OCTOMAP_LIBRARIES}
          ${LIBFCL_LIBRARIES})
target_compile_options(tesseract_collision_profile PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_definitions(tesseract_collision_profile INTERFACE ${TESSERACT_COMPILE_DEFINITIONS})
target_cxx_version(tesseract_collision_profile PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_clang_tidy(tesseract_collision_profile ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_code_coverage(
  tesseract_collision_profile
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})

install_targets(TARGETS tesseract_collision_profile COMPONENT collision)
