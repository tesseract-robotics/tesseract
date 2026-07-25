/* Compile TinyVDB I/O implementation as C. */
/* Ensure POSIX functions (ftruncate, etc.) are declared. */
#if !defined(_WIN32) && !defined(_POSIX_C_SOURCE)
#define _POSIX_C_SOURCE 200809L
#endif
#ifndef TINYVDB_IO_IMPLEMENTATION
#define TINYVDB_IO_IMPLEMENTATION
#endif
#include "tinyvdb_io.h"
