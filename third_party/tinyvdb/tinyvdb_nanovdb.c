/* Compile NanoVDB support as C. */
#if !defined(_WIN32) && !defined(_POSIX_C_SOURCE)
#define _POSIX_C_SOURCE 200809L
#endif
#ifndef TINYVDB_NANOVDB_IMPLEMENTATION
#define TINYVDB_NANOVDB_IMPLEMENTATION
#endif
#include "tinyvdb_nanovdb.h"
