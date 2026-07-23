// CPU autograd VJPs. See tinyvdb_autograd.h for design notes.

#include "tinyvdb_autograd.h"
#include "tinyvdb_ops_internal.h"
#include "tinyvdb_sample.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

// ---------------------------------------------------------------------------
// Trilinear sample VJPs.
// ---------------------------------------------------------------------------

static inline void world_to_voxel_idx(const tvdb_dense_grid* g,
                                      float wx, float wy, float wz,
                                      float* vx, float* vy, float* vz) {
  *vx = (wx - g->ox) / g->voxel_size - 0.5f;
  *vy = (wy - g->oy) / g->voxel_size - 0.5f;
  *vz = (wz - g->oz) / g->voxel_size - 0.5f;
}

void tvdb_sample_trilinear_dense_vjp_grid(const tvdb_dense_grid* grid,
                                          const tvdb_vec3f* pts,
                                          size_t n,
                                          const float* grad_out,
                                          tvdb_dense_grid* grad_grid) {
  if (!grid || !grad_grid || !grad_grid->data) return;
  if (grid->nx != grad_grid->nx || grid->ny != grad_grid->ny ||
      grid->nz != grad_grid->nz) return;
  // grad_grid is *accumulated into*, so we can't reuse splat (which assumes
  // zero-init for normalized splat). The math is identical: scatter
  // grad_out[j] back to the 8-neighborhood of pts[j] with the same weights.
  #pragma omp parallel for schedule(static)
  for (long long pp = 0; pp < (long long)n; ++pp) {
    size_t p = (size_t)pp;
    float vx, vy, vz;
    world_to_voxel_idx(grid, pts[p].x, pts[p].y, pts[p].z, &vx, &vy, &vz);
    int ix = (int)floorf(vx), iy = (int)floorf(vy), iz = (int)floorf(vz);
    float fx = vx - (float)ix, fy = vy - (float)iy, fz = vz - (float)iz;
    if (ix < -1 || iy < -1 || iz < -1) continue;
    if (ix >= grid->nx || iy >= grid->ny || iz >= grid->nz) continue;

    const float go = grad_out[p];
    for (int dz = 0; dz < 2; ++dz) {
      int z = iz + dz; if (z < 0 || z >= grid->nz) continue;
      float wz = (dz == 0) ? (1.0f - fz) : fz;
      for (int dy = 0; dy < 2; ++dy) {
        int y = iy + dy; if (y < 0 || y >= grid->ny) continue;
        float wy = (dy == 0) ? (1.0f - fy) : fy;
        for (int dx = 0; dx < 2; ++dx) {
          int x = ix + dx; if (x < 0 || x >= grid->nx) continue;
          float wx = (dx == 0) ? (1.0f - fx) : fx;
          float w = wx * wy * wz;
          size_t idx = tvdb_idx(grid, x, y, z);
          float contrib = w * go;
          #pragma omp atomic update
          grad_grid->data[idx] += contrib;
        }
      }
    }
  }
}

void tvdb_sample_trilinear_dense_vjp_pts(const tvdb_dense_grid* grid,
                                         const tvdb_vec3f* pts,
                                         size_t n,
                                         const float* grad_out,
                                         tvdb_vec3f* grad_pts) {
  if (!grid || !grid->data || !grad_pts) return;
  // Analytic d/dvx of the trilinear interpolation. Let
  //   c00 = c000 (1-fx) + c100 fx, c10 = c010 (1-fx) + c110 fx, ...
  //   c0  = c00 (1-fy) + c10 fy,   c1  = c01 (1-fy) + c11 fy
  //   out = c0 (1-fz) + c1 fz
  // d out/d fx = (c100 - c000)(1-fy)(1-fz) + (c110 - c010) fy (1-fz)
  //            + (c101 - c001)(1-fy) fz    + (c111 - c011) fy fz
  // d out/d fy = (c10 - c00)(1-fz) + (c11 - c01) fz
  // d out/d fz = c1 - c0
  // d fx/d wx = 1/voxel_size  (since vx = (wx - ox)/vs - 0.5).
  const float inv_vs = 1.0f / grid->voxel_size;
  #pragma omp parallel for schedule(static)
  for (long long pp = 0; pp < (long long)n; ++pp) {
    size_t p = (size_t)pp;
    float vx, vy, vz;
    world_to_voxel_idx(grid, pts[p].x, pts[p].y, pts[p].z, &vx, &vy, &vz);
    int ix = (int)floorf(vx), iy = (int)floorf(vy), iz = (int)floorf(vz);
    float fx = vx - (float)ix, fy = vy - (float)iy, fz = vz - (float)iz;
    float c000 = tvdb_at(grid, ix,     iy,     iz);
    float c100 = tvdb_at(grid, ix + 1, iy,     iz);
    float c010 = tvdb_at(grid, ix,     iy + 1, iz);
    float c110 = tvdb_at(grid, ix + 1, iy + 1, iz);
    float c001 = tvdb_at(grid, ix,     iy,     iz + 1);
    float c101 = tvdb_at(grid, ix + 1, iy,     iz + 1);
    float c011 = tvdb_at(grid, ix,     iy + 1, iz + 1);
    float c111 = tvdb_at(grid, ix + 1, iy + 1, iz + 1);
    float c00 = c000 * (1.0f - fx) + c100 * fx;
    float c10 = c010 * (1.0f - fx) + c110 * fx;
    float c01 = c001 * (1.0f - fx) + c101 * fx;
    float c11 = c011 * (1.0f - fx) + c111 * fx;
    float c0 = c00 * (1.0f - fy) + c10 * fy;
    float c1 = c01 * (1.0f - fy) + c11 * fy;

    float dfx = (c100 - c000) * (1.0f - fy) * (1.0f - fz)
              + (c110 - c010) * fy           * (1.0f - fz)
              + (c101 - c001) * (1.0f - fy) * fz
              + (c111 - c011) * fy           * fz;
    float dfy = (c10 - c00) * (1.0f - fz) + (c11 - c01) * fz;
    float dfz = c1 - c0;
    const float go = grad_out[p];
    grad_pts[p].x += go * dfx * inv_vs;
    grad_pts[p].y += go * dfy * inv_vs;
    grad_pts[p].z += go * dfz * inv_vs;
  }
}

void tvdb_splat_trilinear_dense_vjp_values(const tvdb_dense_grid* grad_grid,
                                           const tvdb_vec3f* pts,
                                           size_t n,
                                           float* grad_values) {
  if (!grad_grid || !grad_grid->data || !grad_values) return;
  // Splat-vs-sample are transposes; gradient w.r.t. values is sample(grad_grid).
  #pragma omp parallel for schedule(static)
  for (long long i = 0; i < (long long)n; ++i) {
    grad_values[(size_t)i] += tvdb_sample_trilinear_dense(
        grad_grid, pts[i].x, pts[i].y, pts[i].z);
  }
}

// ---------------------------------------------------------------------------
// CSG VJPs.
// ---------------------------------------------------------------------------

static inline int same_shape3(const tvdb_dense_grid* a, const tvdb_dense_grid* b) {
  return a->nx == b->nx && a->ny == b->ny && a->nz == b->nz;
}

void tvdb_csg_union_vjp(const tvdb_dense_grid* a, const tvdb_dense_grid* b,
                        const tvdb_dense_grid* grad_out,
                        tvdb_dense_grid* grad_a, tvdb_dense_grid* grad_b) {
  if (!a || !b || !grad_out || !grad_a || !grad_b) return;
  if (!same_shape3(a, b) || !same_shape3(a, grad_out) ||
      !same_shape3(a, grad_a) || !same_shape3(a, grad_b)) return;
  size_t n = (size_t)a->nx * a->ny * a->nz;
  #pragma omp parallel for schedule(static)
  for (long long i = 0; i < (long long)n; ++i) {
    float av = a->data[i], bv = b->data[i], g = grad_out->data[i];
    if (av < bv)      { grad_a->data[i] += g; }
    else if (bv < av) { grad_b->data[i] += g; }
    else              { grad_a->data[i] += 0.5f * g; grad_b->data[i] += 0.5f * g; }
  }
}

void tvdb_csg_intersection_vjp(const tvdb_dense_grid* a, const tvdb_dense_grid* b,
                               const tvdb_dense_grid* grad_out,
                               tvdb_dense_grid* grad_a, tvdb_dense_grid* grad_b) {
  if (!a || !b || !grad_out || !grad_a || !grad_b) return;
  if (!same_shape3(a, b) || !same_shape3(a, grad_out) ||
      !same_shape3(a, grad_a) || !same_shape3(a, grad_b)) return;
  size_t n = (size_t)a->nx * a->ny * a->nz;
  #pragma omp parallel for schedule(static)
  for (long long i = 0; i < (long long)n; ++i) {
    float av = a->data[i], bv = b->data[i], g = grad_out->data[i];
    if (av > bv)      { grad_a->data[i] += g; }
    else if (bv > av) { grad_b->data[i] += g; }
    else              { grad_a->data[i] += 0.5f * g; grad_b->data[i] += 0.5f * g; }
  }
}

void tvdb_csg_difference_vjp(const tvdb_dense_grid* a, const tvdb_dense_grid* b,
                             const tvdb_dense_grid* grad_out,
                             tvdb_dense_grid* grad_a, tvdb_dense_grid* grad_b) {
  if (!a || !b || !grad_out || !grad_a || !grad_b) return;
  if (!same_shape3(a, b) || !same_shape3(a, grad_out) ||
      !same_shape3(a, grad_a) || !same_shape3(a, grad_b)) return;
  // c = max(a, -b). dc/da = 1{a > -b}, dc/db = -1{-b > a}.
  size_t n = (size_t)a->nx * a->ny * a->nz;
  #pragma omp parallel for schedule(static)
  for (long long i = 0; i < (long long)n; ++i) {
    float av = a->data[i], bv = b->data[i], g = grad_out->data[i];
    float nb = -bv;
    if (av > nb)      { grad_a->data[i] += g; }
    else if (nb > av) { grad_b->data[i] += -g; }
    else              { grad_a->data[i] += 0.5f * g; grad_b->data[i] += -0.5f * g; }
  }
}

// ---------------------------------------------------------------------------
// Sparse conv3d VJPs.
//
// Reuse the same hash table machinery as tvdb_sparse_conv3d. Implementation
// is small and self-contained so we just build a local hash inline.
// ---------------------------------------------------------------------------

typedef struct { int x, y, z; int idx_plus_one; } sc_he_t;

static inline size_t sc_pow2_(size_t v) {
  size_t p = 1; while (p < v) p <<= 1; return p;
}
static inline uint64_t sc_pack_(int x, int y, int z) {
  uint64_t ux = ((uint64_t)(int64_t)x) & 0x1FFFFFu;
  uint64_t uy = ((uint64_t)(int64_t)y) & 0x1FFFFFu;
  uint64_t uz = ((uint64_t)(int64_t)z) & 0x1FFFFFu;
  return ux | (uy << 21) | (uz << 42);
}
static inline uint64_t sc_mix_(uint64_t x) {
  x ^= x >> 33; x *= 0xff51afd7ed558ccdULL;
  x ^= x >> 33; x *= 0xc4ceb9fe1a85ec53ULL;
  x ^= x >> 33; return x;
}

static int sc_hash_build(const tvdb_sparse_grid* g, sc_he_t** out_tbl, size_t* out_mask) {
  size_t cap = sc_pow2_(g->count * 2 + 16);
  sc_he_t* tbl = (sc_he_t*)calloc(cap, sizeof(sc_he_t));
  if (!tbl) return 0;
  size_t mask = cap - 1;
  for (size_t i = 0; i < g->count; ++i) {
    uint64_t h = sc_mix_(sc_pack_(g->coords[i].x, g->coords[i].y, g->coords[i].z));
    size_t k = (size_t)(h & mask);
    while (tbl[k].idx_plus_one) k = (k + 1) & mask;
    tbl[k].x = g->coords[i].x; tbl[k].y = g->coords[i].y; tbl[k].z = g->coords[i].z;
    tbl[k].idx_plus_one = (int)(i + 1);
  }
  *out_tbl = tbl; *out_mask = mask; return 1;
}

static int sc_hash_get(const sc_he_t* tbl, size_t mask, int x, int y, int z) {
  uint64_t h = sc_mix_(sc_pack_(x, y, z));
  size_t k = (size_t)(h & mask);
  while (tbl[k].idx_plus_one) {
    if (tbl[k].x == x && tbl[k].y == y && tbl[k].z == z) return tbl[k].idx_plus_one - 1;
    k = (k + 1) & mask;
  }
  return -1;
}

bool tvdb_sparse_conv3d_vjp_values(const tvdb_sparse_grid* in_topo,
                                   const float* grad_out_values,
                                   const float* kernel,
                                   int kx, int ky, int kz,
                                   float* grad_in_values) {
  if (!in_topo || !grad_out_values || !kernel || !grad_in_values) return false;
  if (kx <= 0 || ky <= 0 || kz <= 0) return false;
  if (in_topo->count == 0) return true;
  const int ax = kx / 2, ay = ky / 2, az = kz / 2;
  sc_he_t* tbl = NULL; size_t mask = 0;
  if (!sc_hash_build(in_topo, &tbl, &mask)) return false;

  // For each output coord i (== in_topo[i]), and each kernel tap, find the
  // input neighbor j; accumulate kernel[tap] * grad_out[i] into grad_in[j].
  // Multiple i's can scatter to the same j (atomic).
  #pragma omp parallel for schedule(static)
  for (long long ii = 0; ii < (long long)in_topo->count; ++ii) {
    int cx = in_topo->coords[ii].x;
    int cy = in_topo->coords[ii].y;
    int cz = in_topo->coords[ii].z;
    float go = grad_out_values[(size_t)ii];
    for (int dk = 0; dk < kz; ++dk) {
      int oz = cz + (dk - az);
      for (int dj = 0; dj < ky; ++dj) {
        int oy = cy + (dj - ay);
        for (int di = 0; di < kx; ++di) {
          int ox = cx + (di - ax);
          float w = kernel[((dk * ky) + dj) * kx + di];
          if (w == 0.0f) continue;
          int j = sc_hash_get(tbl, mask, ox, oy, oz);
          if (j < 0) continue;
          float c = w * go;
          #pragma omp atomic update
          grad_in_values[j] += c;
        }
      }
    }
  }
  free(tbl);
  return true;
}

bool tvdb_sparse_conv3d_vjp_kernel(const tvdb_sparse_grid* in_with_values,
                                   const float* grad_out_values,
                                   int kx, int ky, int kz,
                                   float* grad_kernel) {
  if (!in_with_values || !grad_out_values || !grad_kernel) return false;
  if (kx <= 0 || ky <= 0 || kz <= 0) return false;
  if (in_with_values->count == 0) return true;
  const int ax = kx / 2, ay = ky / 2, az = kz / 2;
  sc_he_t* tbl = NULL; size_t mask = 0;
  if (!sc_hash_build(in_with_values, &tbl, &mask)) return false;

  // For each (output_coord i, tap), accumulate grad_out[i] * in_values[j]
  // into grad_kernel[tap].
  // Parallelize over i with atomics on grad_kernel (only kx*ky*kz entries).
  #pragma omp parallel for schedule(static)
  for (long long ii = 0; ii < (long long)in_with_values->count; ++ii) {
    int cx = in_with_values->coords[ii].x;
    int cy = in_with_values->coords[ii].y;
    int cz = in_with_values->coords[ii].z;
    float go = grad_out_values[(size_t)ii];
    for (int dk = 0; dk < kz; ++dk) {
      int oz = cz + (dk - az);
      for (int dj = 0; dj < ky; ++dj) {
        int oy = cy + (dj - ay);
        for (int di = 0; di < kx; ++di) {
          int ox = cx + (di - ax);
          int j = sc_hash_get(tbl, mask, ox, oy, oz);
          if (j < 0) continue;
          int tap = ((dk * ky) + dj) * kx + di;
          float c = go * in_with_values->values[j];
          #pragma omp atomic update
          grad_kernel[tap] += c;
        }
      }
    }
  }
  free(tbl);
  return true;
}
