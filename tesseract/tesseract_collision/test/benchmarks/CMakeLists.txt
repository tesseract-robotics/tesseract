find_package(benchmark REQUIRED)
find_package(tesseract_scene_graph REQUIRED)

macro(add_benchmark benchmark_name benchmark_file)
  add_executable(${benchmark_name} ${benchmark_file})
  tesseract_target_compile_options(${benchmark_name} PRIVATE)
  tesseract_clang_tidy(${benchmark_name})
  target_link_libraries(${benchmark_name}

      benchmark::benchmark
      ${PROJECT_NAME}_test_suite
      ${PROJECT_NAME}_bullet
      ${PROJECT_NAME}_fcl
      tesseract::tesseract_geometry
      tesseract::tesseract_scene_graph
      console_bridge
      ${BULLET_LIBRARIES}
      ${Boost_LIBRARIES}
      ${OCTOMAP_LIBRARIES}
      ${LIBFCL_LIBRARIES}
      )
  target_include_directories(${benchmark_name} PRIVATE
      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>")
  add_dependencies(${benchmark_name}
    ${PROJECT_NAME}_bullet
    ${PROJECT_NAME}_fcl)
  tesseract_add_run_benchmark_target(${benchmark_name})
endmacro()

add_benchmark(${PROJECT_NAME}_bullet_discrete_simple_benchmarks bullet_discrete_simple_benchmarks.cpp)
