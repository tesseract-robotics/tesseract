find_package(benchmark REQUIRED)
find_package(tesseract_support REQUIRED)

macro(add_benchmark benchmark_name benchmark_file)
  add_executable(${benchmark_name} ${benchmark_file})
  target_compile_definitions(${benchmark_name} PRIVATE BENCHMARK_ARGS="${BENCHMARK_ARGS}")
  target_compile_options(${benchmark_name} PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                   ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
  target_compile_definitions(${benchmark_name} PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
  target_clang_tidy(${benchmark_name} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
  target_cxx_version(${benchmark_name} PRIVATE VERSION ${TESSERACT_CXX_VERSION})
  target_link_libraries(
    ${benchmark_name}
    benchmark::benchmark
    ${PROJECT_NAME}_core
    ${PROJECT_NAME}_ofkt
    tesseract::tesseract_urdf
    tesseract::tesseract_support
    console_bridge
    ${Boost_LIBRARIES})
  target_include_directories(${benchmark_name} PRIVATE "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>")
  add_run_benchmark_target(${benchmark_name})
endmacro()

add_benchmark(${PROJECT_NAME}_clone_benchmark environment_clone_benchmarks.cpp)
