find_package(benchmark REQUIRED)

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
    tesseract::kinematics
    tesseract::kinematics_kdl
    tesseract::kinematics_opw
    tesseract::urdf
    tesseract::scene_graph
    tesseract::state_solver
    tesseract::common
    console_bridge::console_bridge
    ${Boost_LIBRARIES})
  target_include_directories(${benchmark_name} PRIVATE "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
                                                       "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>")
  if(TESSERACT_ENABLE_RUN_BENCHMARKING)
    message(STATUS "Running benchmark ${benchmark_name}")
    add_run_benchmark_target(${benchmark_name})
  endif()
  add_dependencies(
    ${benchmark_name}
    tesseract::kinematics
    tesseract::kinematics_kdl
    tesseract::kinematics_opw)
endmacro()

add_benchmark(tesseract_kinematics_rtp_vs_kdl_benchmark rtp_vs_kdl_benchmarks.cpp)
