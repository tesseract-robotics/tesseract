// Coordinate utilities and point/coordinate spatial queries. See
// tinyvdb_grid_index.h.

#include "tinyvdb_grid_index.h"

#include <math.h>
#include <stdlib.h>

#define GI_BIAS (1 << 20)          // 2^20; coords in [-2^20, 2^20-1]
#define GI_MASK21 ((1u << 21) - 1)

// ---- coordinate <-> world ---------------------------------------------------

void tvdb_world_to_ijk(const float* points, size_t n,
                       const float voxel_size[3], const float origin[3],
                       int32_t* out_ijk) {
  for (size_t i = 0; i < n; ++i)
    for (int a = 0; a < 3; ++a)
      // Guard against a non-positive voxel size: dividing would yield inf/nan
      // and casting that to int is UB.
      out_ijk[3*i+a] = voxel_size[a] > 0.0f
          ? (int32_t)floorf((points[3*i+a] - origin[a]) / voxel_size[a]) : 0;
}

void tvdb_ijk_to_world(const int32_t* ijk, size_t n,
                       const float voxel_size[3], const float origin[3],
                       float* out_points) {
  for (size_t i = 0; i < n; ++i)
    for (int a = 0; a < 3; ++a)
      out_points[3*i+a] = origin[a] + ((float)ijk[3*i+a] + 0.5f) * voxel_size[a];
}

// ---- Morton (Z-order) -------------------------------------------------------

static uint64_t gi_split3(uint64_t a) {            // spread 21 low bits to every 3rd
  a &= 0x1fffffULL;
  a = (a | a << 32) & 0x1f00000000ffffULL;
  a = (a | a << 16) & 0x1f0000ff0000ffULL;
  a = (a | a << 8)  & 0x100f00f00f00f00fULL;
  a = (a | a << 4)  & 0x10c30c30c30c30c3ULL;
  a = (a | a << 2)  & 0x1249249249249249ULL;
  return a;
}
static uint64_t gi_compact3(uint64_t a) {          // inverse of gi_split3
  a &= 0x1249249249249249ULL;
  a = (a | a >> 2)  & 0x10c30c30c30c30c3ULL;
  a = (a | a >> 4)  & 0x100f00f00f00f00fULL;
  a = (a | a >> 8)  & 0x1f0000ff0000ffULL;
  a = (a | a >> 16) & 0x1f00000000ffffULL;
  a = (a | a >> 32) & 0x1fffffULL;
  return a;
}

uint64_t tvdb_morton_encode(int32_t x, int32_t y, int32_t z) {
  uint64_t bx = (uint64_t)((int64_t)x + GI_BIAS) & GI_MASK21;
  uint64_t by = (uint64_t)((int64_t)y + GI_BIAS) & GI_MASK21;
  uint64_t bz = (uint64_t)((int64_t)z + GI_BIAS) & GI_MASK21;
  return gi_split3(bx) | (gi_split3(by) << 1) | (gi_split3(bz) << 2);
}

void tvdb_morton_decode(uint64_t code, int32_t* x, int32_t* y, int32_t* z) {
  *x = (int32_t)((int64_t)gi_compact3(code)       - GI_BIAS);
  *y = (int32_t)((int64_t)gi_compact3(code >> 1)  - GI_BIAS);
  *z = (int32_t)((int64_t)gi_compact3(code >> 2)  - GI_BIAS);
}

// ---- int3 -> index hash set -------------------------------------------------

static uint64_t gi_pack(int32_t x, int32_t y, int32_t z) {
  uint64_t ux = (uint64_t)((int64_t)x + GI_BIAS) & GI_MASK21;
  uint64_t uy = (uint64_t)((int64_t)y + GI_BIAS) & GI_MASK21;
  uint64_t uz = (uint64_t)((int64_t)z + GI_BIAS) & GI_MASK21;
  return (ux << 42) | (uy << 21) | uz;             // 63-bit key
}
static uint64_t gi_mix(uint64_t x) {
  x ^= x >> 33; x *= 0xff51afd7ed558ccdULL;
  x ^= x >> 33; x *= 0xc4ceb9fe1a85ec53ULL;
  x ^= x >> 33; return x;
}

typedef struct { uint64_t key; int64_t idx; } gi_entry;  // key stored as packed+1 (0 = empty)
typedef struct { gi_entry* e; size_t mask; } gi_hash;

static bool gi_hash_build(gi_hash* h, const int32_t* coords, size_t n) {
  size_t cap = 16;
  while (cap < (n + 1) * 2) cap <<= 1;
  h->e = (gi_entry*)calloc(cap, sizeof(gi_entry));
  h->mask = cap - 1;
  if (!h->e) return false;
  for (size_t i = 0; i < n; ++i) {
    uint64_t key = gi_pack(coords[3*i], coords[3*i+1], coords[3*i+2]) + 1;
    size_t s = (size_t)gi_mix(key) & h->mask;
    while (h->e[s].key) {
      if (h->e[s].key == key) break;               // already present (keep first index)
      s = (s + 1) & h->mask;
    }
    if (!h->e[s].key) { h->e[s].key = key; h->e[s].idx = (int64_t)i; }
  }
  return true;
}
static int64_t gi_hash_get(const gi_hash* h, int32_t x, int32_t y, int32_t z) {
  uint64_t key = gi_pack(x, y, z) + 1;
  size_t s = (size_t)gi_mix(key) & h->mask;
  while (h->e[s].key) {
    if (h->e[s].key == key) return h->e[s].idx;
    s = (s + 1) & h->mask;
  }
  return -1;
}
static void gi_hash_free(gi_hash* h) { free(h->e); h->e = NULL; }

// ---- queries ----------------------------------------------------------------

bool tvdb_coords_in_set(const int32_t* active, size_t na,
                        const int32_t* query, size_t nq, uint8_t* out) {
  gi_hash h;
  if (!gi_hash_build(&h, active, na)) return false;
  for (size_t i = 0; i < nq; ++i)
    out[i] = gi_hash_get(&h, query[3*i], query[3*i+1], query[3*i+2]) >= 0 ? 1 : 0;
  gi_hash_free(&h);
  return true;
}

bool tvdb_points_in_set(const float* points, size_t np,
                        const float voxel_size[3], const float origin[3],
                        const int32_t* active, size_t na, uint8_t* out) {
  if (voxel_size[0] <= 0.0f || voxel_size[1] <= 0.0f || voxel_size[2] <= 0.0f) return false;
  gi_hash h;
  if (!gi_hash_build(&h, active, na)) return false;
  for (size_t i = 0; i < np; ++i) {
    int32_t ijk[3];
    for (int a = 0; a < 3; ++a)
      ijk[a] = (int32_t)floorf((points[3*i+a] - origin[a]) / voxel_size[a]);
    out[i] = gi_hash_get(&h, ijk[0], ijk[1], ijk[2]) >= 0 ? 1 : 0;
  }
  gi_hash_free(&h);
  return true;
}

bool tvdb_ijk_to_index(const int32_t* active, size_t na,
                       const int32_t* query, size_t nq, int64_t* out) {
  gi_hash h;
  if (!gi_hash_build(&h, active, na)) return false;
  for (size_t i = 0; i < nq; ++i)
    out[i] = gi_hash_get(&h, query[3*i], query[3*i+1], query[3*i+2]);
  gi_hash_free(&h);
  return true;
}

bool tvdb_neighbor_counts(const int32_t* active, size_t na,
                          int connectivity, int32_t* out_counts) {
  gi_hash h;
  if (!gi_hash_build(&h, active, na)) return false;
  int off[26][3]; int noff;
  if (connectivity == 26) {
    noff = 0;
    for (int dz = -1; dz <= 1; ++dz) for (int dy = -1; dy <= 1; ++dy) for (int dx = -1; dx <= 1; ++dx) {
      if (dx == 0 && dy == 0 && dz == 0) continue;
      off[noff][0] = dx; off[noff][1] = dy; off[noff][2] = dz; ++noff;
    }
  } else {
    static const int o6[6][3] = { {1,0,0},{-1,0,0},{0,1,0},{0,-1,0},{0,0,1},{0,0,-1} };
    for (int t = 0; t < 6; ++t) { off[t][0]=o6[t][0]; off[t][1]=o6[t][1]; off[t][2]=o6[t][2]; }
    noff = 6;
  }
  for (size_t i = 0; i < na; ++i) {
    int32_t x = active[3*i], y = active[3*i+1], z = active[3*i+2];
    int32_t c = 0;
    for (int t = 0; t < noff; ++t)
      if (gi_hash_get(&h, x+off[t][0], y+off[t][1], z+off[t][2]) >= 0) ++c;
    out_counts[i] = c;
  }
  gi_hash_free(&h);
  return true;
}

bool tvdb_voxelize_points(const float* points, size_t n,
                          const float voxel_size[3], const float origin[3],
                          int32_t** out_coords, size_t* out_count) {
  *out_coords = NULL; *out_count = 0;
  if (voxel_size[0] <= 0.0f || voxel_size[1] <= 0.0f || voxel_size[2] <= 0.0f) return false;
  if (n == 0) return true;
  gi_hash h;
  size_t cap = 16; while (cap < (n + 1) * 2) cap <<= 1;
  h.e = (gi_entry*)calloc(cap, sizeof(gi_entry)); h.mask = cap - 1;
  if (!h.e) return false;
  int32_t* coords = (int32_t*)malloc(n * 3 * sizeof(int32_t));
  if (!coords) { free(h.e); return false; }
  size_t cnt = 0;
  for (size_t i = 0; i < n; ++i) {
    int32_t ijk[3];
    for (int a = 0; a < 3; ++a)
      ijk[a] = (int32_t)floorf((points[3*i+a] - origin[a]) / voxel_size[a]);
    uint64_t key = gi_pack(ijk[0], ijk[1], ijk[2]) + 1;
    size_t s = (size_t)gi_mix(key) & h.mask;
    int found = 0;
    while (h.e[s].key) { if (h.e[s].key == key) { found = 1; break; } s = (s + 1) & h.mask; }
    if (!found) {
      h.e[s].key = key;
      coords[3*cnt+0] = ijk[0]; coords[3*cnt+1] = ijk[1]; coords[3*cnt+2] = ijk[2];
      ++cnt;
    }
  }
  free(h.e);
  *out_coords = coords;
  *out_count = cnt;
  return true;
}
