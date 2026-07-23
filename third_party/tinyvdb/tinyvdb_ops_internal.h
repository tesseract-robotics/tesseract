#pragma once

// Internal helpers shared by tinyvdb_ops.c. Not part of the public API.

#include <stddef.h>
#include "tinyvdb_mesh.h"  // tvdb_dense_grid

#ifdef __cplusplus
extern "C" {
#endif

static inline int tvdb_clamp_i(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }

static inline size_t tvdb_idx(const tvdb_dense_grid* g, int ix, int iy, int iz)
{
  // size_t arithmetic throughout so the index doesn't overflow int for grids
  // with > INT_MAX voxels.
  return ((size_t)iz * g->ny + iy) * g->nx + ix;
}

// Read with edge clamp.
static inline float tvdb_at(const tvdb_dense_grid* g, int ix, int iy, int iz)
{
  ix = tvdb_clamp_i(ix, 0, g->nx - 1);
  iy = tvdb_clamp_i(iy, 0, g->ny - 1);
  iz = tvdb_clamp_i(iz, 0, g->nz - 1);
  return g->data[tvdb_idx(g, ix, iy, iz)];
}

// fp64 variants. Declared after `tvdb_dense_grid_d` so we include ops.h.
#include "tinyvdb_ops.h"
static inline size_t tvdb_idx_d(const tvdb_dense_grid_d* g, int ix, int iy, int iz)
{
  return ((size_t)iz * g->ny + iy) * g->nx + ix;
}
static inline double tvdb_at_d(const tvdb_dense_grid_d* g, int ix, int iy, int iz)
{
  ix = tvdb_clamp_i(ix, 0, g->nx - 1);
  iy = tvdb_clamp_i(iy, 0, g->ny - 1);
  iz = tvdb_clamp_i(iz, 0, g->nz - 1);
  return g->data[tvdb_idx_d(g, ix, iy, iz)];
}

#ifdef __cplusplus
}
#endif
