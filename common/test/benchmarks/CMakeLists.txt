find_package(benchmark REQUIRED)

add_executable(tesseract_common_identity_benchmarks identity_benchmarks.cpp)
target_link_libraries(tesseract_common_identity_benchmarks PRIVATE benchmark::benchmark tesseract::common)
target_compile_options(tesseract_common_identity_benchmarks PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                                    ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(tesseract_common_identity_benchmarks PRIVATE ${TESSERACT_COMPILE_DEFINITIONS_PUBLIC})
target_cxx_version(tesseract_common_identity_benchmarks PUBLIC VERSION ${TESSERACT_CXX_VERSION})
add_dependencies(tesseract_common_identity_benchmarks tesseract::common)
target_clang_tidy(tesseract_common_identity_benchmarks ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
