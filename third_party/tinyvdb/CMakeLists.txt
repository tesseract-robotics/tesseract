# TinyVDB is vendored for the OpenVDB and NanoVDB persistence bridge in geometry. Keep this target private to Tesseract:
# it is not installed or exported.
add_library(
  tinyvdb STATIC
  tinyvdb_io.c
  lz4.c
  miniz.c
  tinyvdb_sparse.c
  tinyvdb_sparse_tree.c
  tinyvdb_lifecycle.c
  tvdb_memory.c
  tinyvdb_to_nanovdb.c
  tinyvdb_nanovdb_impl.c)
add_library(tesseract::tinyvdb ALIAS tinyvdb)

target_include_directories(tinyvdb SYSTEM PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_definitions(tinyvdb PRIVATE MINIZ_NO_STDIO)
set_source_files_properties(tinyvdb_io.c PROPERTIES COMPILE_DEFINITIONS TINYVDB_IO_IMPLEMENTATION)
set_target_properties(
  tinyvdb
  PROPERTIES C_STANDARD 11
             C_STANDARD_REQUIRED ON
             C_EXTENSIONS OFF
             POSITION_INDEPENDENT_CODE ON)
if(NOT WIN32)
  target_link_libraries(tinyvdb PUBLIC m)
endif()
