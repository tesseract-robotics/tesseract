# TinyVDB is vendored for the OpenVDB and NanoVDB persistence bridge in geometry. Keep this target private to Tesseract:
# it is not installed or exported.
add_library(
  tinyvdb STATIC
  tinyvdb_io.c
  lz4.c
  miniz.c
  tinyvdb_sparse.c
  tinyvdb_sparse_tree.c
  tinyvdb_tree.c
  tinyvdb_sdf_tree.c
  tinyvdb_thread.c
  tinyvdb_lifecycle.c
  tvdb_memory.c
  tinyvdb_to_nanovdb.c
  tinyvdb_nanovdb.c)
add_library(tesseract::tinyvdb ALIAS tinyvdb)

target_include_directories(tinyvdb SYSTEM PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
# Serial task backend: Tesseract does not use TinyVDB's worker threads, so the build needs no thread library.
target_compile_definitions(tinyvdb PRIVATE MINIZ_NO_STDIO TINYVDB_THREAD_NONE)
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

# NanoVDB files written by NanoVDB/fVDB are commonly Blosc-compressed; decode them when Blosc is available.
find_path(BLOSC_INCLUDE_DIR blosc.h)
find_library(BLOSC_LIBRARY blosc)
if(BLOSC_INCLUDE_DIR AND BLOSC_LIBRARY)
  target_compile_definitions(tinyvdb PRIVATE TVDB_HAVE_BLOSC)
  target_include_directories(tinyvdb PRIVATE "${BLOSC_INCLUDE_DIR}")
  target_link_libraries(tinyvdb PRIVATE "${BLOSC_LIBRARY}")
  set(TESSERACT_TINYVDB_HAS_BLOSC ON CACHE INTERNAL "tinyvdb decodes BLOSC-compressed NanoVDB")
else()
  message(STATUS "Blosc not found: BLOSC-compressed NanoVDB files will be rejected")
  set(TESSERACT_TINYVDB_HAS_BLOSC OFF CACHE INTERNAL "tinyvdb decodes BLOSC-compressed NanoVDB")
endif()
