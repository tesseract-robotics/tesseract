#include "tinyvdb_sparse.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

void tvdb_sparse_grid_init(tvdb_sparse_grid* sg) {
  sg->coords = NULL;
  sg->values = NULL;
  sg->count = 0;
  sg->capacity = 0;
  sg->voxel_size = 1.0f;
  sg->ox = sg->oy = sg->oz = 0.0f;
}

void tvdb_sparse_grid_free(tvdb_sparse_grid* sg) {
  // capacity==0 with non-NULL coords is a non-owning view (e.g. tvdb_grid_batch_view):
  // its arrays point into another allocation, so reset the handle without freeing.
  if (sg->capacity > 0) {
    free(sg->coords);
    free(sg->values);
  }
  sg->coords = NULL;
  sg->values = NULL;
  sg->count = 0;
  sg->capacity = 0;
}

bool tvdb_sparse_grid_reserve(tvdb_sparse_grid* sg, size_t capacity) {
  if (capacity <= sg->capacity) return true;
  // Refuse to grow a non-owning view (capacity==0 but arrays already set):
  // realloc'ing its interior pointer would corrupt the backing allocation.
  if (sg->capacity == 0 && sg->coords != NULL) return false;
  void* nc = realloc(sg->coords, capacity * sizeof(tvdb_vec3i));
  void* nv = realloc(sg->values, capacity * sizeof(float));
  if (!nc || !nv) {
    if (nc) sg->coords = (tvdb_vec3i*)nc;
    if (nv) sg->values = (float*)nv;
    return false;
  }
  sg->coords = (tvdb_vec3i*)nc;
  sg->values = (float*)nv;
  sg->capacity = capacity;
  return true;
}

static bool tvdb_sparse_push(tvdb_sparse_grid* sg, int x, int y, int z, float v) {
  if (sg->count == sg->capacity) {
    size_t cap = sg->capacity ? sg->capacity * 2 : 256;
    if (!tvdb_sparse_grid_reserve(sg, cap)) return false;
  }
  sg->coords[sg->count].x = x;
  sg->coords[sg->count].y = y;
  sg->coords[sg->count].z = z;
  sg->values[sg->count] = v;
  ++sg->count;
  return true;
}

bool tvdb_dense_to_sparse(const tvdb_dense_grid* dense,
                          float background,
                          float tolerance,
                          tvdb_sparse_grid* out) {
  if (!dense || !dense->data || !out) return false;
  out->count = 0;
  out->voxel_size = dense->voxel_size;
  out->ox = dense->ox; out->oy = dense->oy; out->oz = dense->oz;

  const int nx = dense->nx, ny = dense->ny, nz = dense->nz;
  for (int iz = 0; iz < nz; ++iz) {
    for (int iy = 0; iy < ny; ++iy) {
      for (int ix = 0; ix < nx; ++ix) {
        float v = dense->data[(size_t)((iz * ny + iy) * nx + ix)];
        if (fabsf(v - background) > tolerance) {
          if (!tvdb_sparse_push(out, ix, iy, iz, v)) return false;
        }
      }
    }
  }
  return true;
}

bool tvdb_active_grid_coords(const tvdb_dense_grid* dense,
                             float background,
                             tvdb_sparse_grid* out) {
  return tvdb_dense_to_sparse(dense, background, 0.0f, out);
}

bool tvdb_sparse_to_dense(const tvdb_sparse_grid* sparse,
                          float background,
                          tvdb_dense_grid* out) {
  if (!sparse || !out || !out->data) return false;
  size_t n = (size_t)out->nx * (size_t)out->ny * (size_t)out->nz;
  for (size_t i = 0; i < n; ++i) out->data[i] = background;

  // Map sparse -> dense; assumes sparse.coords are in the same voxel-index
  // frame as the dense grid (caller is responsible for any origin offsets).
  for (size_t k = 0; k < sparse->count; ++k) {
    int x = sparse->coords[k].x;
    int y = sparse->coords[k].y;
    int z = sparse->coords[k].z;
    if (x < 0 || y < 0 || z < 0) continue;
    if (x >= out->nx || y >= out->ny || z >= out->nz) continue;
    out->data[(size_t)((z * out->ny + y) * out->nx + x)] = sparse->values[k];
  }
  return true;
}

// -------------------------------------------------------------------------
// Hashed lookup over sparse coords (linear probing). Used by CSG / morphology.
// -------------------------------------------------------------------------

typedef struct {
  uint64_t key;
  uint32_t idx_plus_one;  // 0 = empty
} tvdb_hash_entry;

static uint64_t tvdb_pack_ijk(int x, int y, int z) {
  // Treat as 21-bit signed-shifted unsigned. Sufficient for grids up to
  // ±1M voxels per axis.
  uint64_t ux = (uint64_t)((int64_t)x + (1LL << 20)) & ((1ULL << 21) - 1);
  uint64_t uy = (uint64_t)((int64_t)y + (1LL << 20)) & ((1ULL << 21) - 1);
  uint64_t uz = (uint64_t)((int64_t)z + (1LL << 20)) & ((1ULL << 21) - 1);
  return (ux << 42) | (uy << 21) | uz;
}

static uint64_t tvdb_mix64(uint64_t k) {
  k ^= k >> 33; k *= 0xff51afd7ed558ccdULL;
  k ^= k >> 33; k *= 0xc4ceb9fe1a85ec53ULL;
  k ^= k >> 33; return k;
}

static size_t tvdb_next_pow2(size_t v) {
  size_t p = 1; while (p < v) p <<= 1; return p;
}

static bool tvdb_hash_build(const tvdb_sparse_grid* g, tvdb_hash_entry** table_out, size_t* mask_out) {
  size_t cap = tvdb_next_pow2(g->count * 2 + 16);
  tvdb_hash_entry* tbl = (tvdb_hash_entry*)calloc(cap, sizeof(tvdb_hash_entry));
  if (!tbl) return false;
  size_t mask = cap - 1;
  for (size_t i = 0; i < g->count; ++i) {
    uint64_t key = tvdb_pack_ijk(g->coords[i].x, g->coords[i].y, g->coords[i].z);
    size_t h = (size_t)(tvdb_mix64(key) & mask);
    while (tbl[h].idx_plus_one) {
      if (tbl[h].key == key) break;
      h = (h + 1) & mask;
    }
    tbl[h].key = key;
    tbl[h].idx_plus_one = (uint32_t)(i + 1);
  }
  *table_out = tbl; *mask_out = mask;
  return true;
}

static int tvdb_hash_get(const tvdb_hash_entry* tbl, size_t mask,
                         int x, int y, int z) {
  uint64_t key = tvdb_pack_ijk(x, y, z);
  size_t h = (size_t)(tvdb_mix64(key) & mask);
  while (tbl[h].idx_plus_one) {
    if (tbl[h].key == key) return (int)(tbl[h].idx_plus_one - 1);
    h = (h + 1) & mask;
  }
  return -1;
}

// -------------------------------------------------------------------------
// CSG
// -------------------------------------------------------------------------

static bool tvdb_check_same_frame(const tvdb_sparse_grid* a, const tvdb_sparse_grid* b) {
  return fabsf(a->voxel_size - b->voxel_size) < 1e-6f
      && fabsf(a->ox - b->ox) < 1e-6f
      && fabsf(a->oy - b->oy) < 1e-6f
      && fabsf(a->oz - b->oz) < 1e-6f;
}

static bool tvdb_csg_sparse_impl(const tvdb_sparse_grid* a, const tvdb_sparse_grid* b,
                                 float background, tvdb_sparse_grid* out, int op) {
  // op: 0 = union (min), 1 = intersection (max), 2 = difference (max(a,-b))
  if (!a || !b || !out) return false;
  if (!tvdb_check_same_frame(a, b)) return false;
  out->count = 0;
  out->voxel_size = a->voxel_size;
  out->ox = a->ox; out->oy = a->oy; out->oz = a->oz;

  tvdb_hash_entry *ha = NULL, *hb = NULL;
  size_t ma = 0, mb = 0;
  if (!tvdb_hash_build(a, &ha, &ma)) return false;
  if (!tvdb_hash_build(b, &hb, &mb)) { free(ha); return false; }

  // Walk a, looking up b
  for (size_t i = 0; i < a->count; ++i) {
    int x = a->coords[i].x, y = a->coords[i].y, z = a->coords[i].z;
    int j = tvdb_hash_get(hb, mb, x, y, z);
    float va = a->values[i];
    float vb = (j >= 0) ? b->values[j] : background;
    float v = 0.0f;
    if (op == 0)      v = va < vb ? va : vb;
    else if (op == 1) v = va > vb ? va : vb;
    else              v = va > -vb ? va : -vb;
    if (!tvdb_sparse_push(out, x, y, z, v)) { free(ha); free(hb); return false; }
  }
  // Walk b, adding entries not in a (only meaningful for union)
  if (op == 0) {
    for (size_t i = 0; i < b->count; ++i) {
      int x = b->coords[i].x, y = b->coords[i].y, z = b->coords[i].z;
      if (tvdb_hash_get(ha, ma, x, y, z) >= 0) continue;
      float vb = b->values[i];
      float v = background < vb ? background : vb;
      if (!tvdb_sparse_push(out, x, y, z, v)) { free(ha); free(hb); return false; }
    }
  }
  free(ha); free(hb);
  return true;
}

bool tvdb_csg_union_sparse(const tvdb_sparse_grid* a, const tvdb_sparse_grid* b,
                           float background, tvdb_sparse_grid* out) {
  return tvdb_csg_sparse_impl(a, b, background, out, 0);
}
bool tvdb_csg_intersection_sparse(const tvdb_sparse_grid* a, const tvdb_sparse_grid* b,
                                  float background, tvdb_sparse_grid* out) {
  return tvdb_csg_sparse_impl(a, b, background, out, 1);
}
bool tvdb_csg_difference_sparse(const tvdb_sparse_grid* a, const tvdb_sparse_grid* b,
                                float background, tvdb_sparse_grid* out) {
  return tvdb_csg_sparse_impl(a, b, background, out, 2);
}

// -------------------------------------------------------------------------
// Morphology
// -------------------------------------------------------------------------

static bool tvdb_dilate_sparse_step(const tvdb_sparse_grid* in,
                                    float background,
                                    tvdb_sparse_grid* out) {
  out->count = 0;
  out->voxel_size = in->voxel_size;
  out->ox = in->ox; out->oy = in->oy; out->oz = in->oz;

  tvdb_hash_entry* hin = NULL; size_t mask = 0;
  if (!tvdb_hash_build(in, &hin, &mask)) return false;

  // For each input voxel, emit (self) plus each missing 6-neighbor.
  // The output value at each voxel = min over (self/contributing neighbors).
  // Use a temporary growing hash for the output.
  size_t guess = tvdb_next_pow2(in->count * 8 + 16);
  tvdb_hash_entry* hout = (tvdb_hash_entry*)calloc(guess, sizeof(tvdb_hash_entry));
  if (!hout) { free(hin); return false; }
  size_t hout_mask = guess - 1;

  static const int N[6][3] = {{1,0,0},{-1,0,0},{0,1,0},{0,-1,0},{0,0,1},{0,0,-1}};
  for (size_t i = 0; i < in->count; ++i) {
    int x = in->coords[i].x, y = in->coords[i].y, z = in->coords[i].z;
    float v_self = in->values[i];

    // self
    {
      uint64_t key = tvdb_pack_ijk(x, y, z);
      size_t h = (size_t)(tvdb_mix64(key) & hout_mask);
      while (hout[h].idx_plus_one) {
        if (hout[h].key == key) break;
        h = (h + 1) & hout_mask;
      }
      if (hout[h].idx_plus_one == 0) {
        if (!tvdb_sparse_push(out, x, y, z, v_self)) { free(hin); free(hout); return false; }
        hout[h].key = key; hout[h].idx_plus_one = (uint32_t)out->count;
      } else {
        size_t idx = hout[h].idx_plus_one - 1;
        if (v_self < out->values[idx]) out->values[idx] = v_self;
      }
    }
    // 6 neighbors
    for (int n = 0; n < 6; ++n) {
      int nx = x + N[n][0], ny = y + N[n][1], nz = z + N[n][2];
      // The neighbor's value, based on the dense kernel: if the neighbor is
      // active, its own value is unchanged here (it'll process itself);
      // we only contribute v_self to the neighbor's "min" because the
      // neighbor inherits the inside of `i` shifted by one voxel.
      uint64_t key = tvdb_pack_ijk(nx, ny, nz);
      size_t h = (size_t)(tvdb_mix64(key) & hout_mask);
      while (hout[h].idx_plus_one) {
        if (hout[h].key == key) break;
        h = (h + 1) & hout_mask;
      }
      if (hout[h].idx_plus_one == 0) {
        // neighbor not yet in output
        int j = tvdb_hash_get(hin, mask, nx, ny, nz);
        float v_existing = (j >= 0) ? in->values[j] : background;
        float v = v_self < v_existing ? v_self : v_existing;
        if (!tvdb_sparse_push(out, nx, ny, nz, v)) { free(hin); free(hout); return false; }
        hout[h].key = key; hout[h].idx_plus_one = (uint32_t)out->count;
      } else {
        size_t idx = hout[h].idx_plus_one - 1;
        if (v_self < out->values[idx]) out->values[idx] = v_self;
      }
    }
  }
  free(hin);
  free(hout);
  return true;
}

bool tvdb_dilate_sparse(const tvdb_sparse_grid* in,
                        float background, int iterations,
                        tvdb_sparse_grid* out) {
  if (!in || !out || iterations <= 0) return false;
  // Copy in -> out as the initial state, then iterate using a scratch.
  tvdb_sparse_grid scratch; tvdb_sparse_grid_init(&scratch);
  if (!tvdb_sparse_grid_reserve(out, in->count)) return false;
  out->count = 0;
  out->voxel_size = in->voxel_size;
  out->ox = in->ox; out->oy = in->oy; out->oz = in->oz;
  for (size_t i = 0; i < in->count; ++i) {
    if (!tvdb_sparse_push(out, in->coords[i].x, in->coords[i].y, in->coords[i].z, in->values[i])) {
      tvdb_sparse_grid_free(&scratch);
      return false;
    }
  }
  for (int it = 0; it < iterations; ++it) {
    if (!tvdb_dilate_sparse_step(out, background, &scratch)) {
      tvdb_sparse_grid_free(&scratch); return false;
    }
    // swap out <-> scratch
    tvdb_sparse_grid tmp = *out; *out = scratch; scratch = tmp;
  }
  tvdb_sparse_grid_free(&scratch);
  return true;
}

static bool tvdb_erode_sparse_step(const tvdb_sparse_grid* in,
                                   tvdb_sparse_grid* out) {
  out->count = 0;
  out->voxel_size = in->voxel_size;
  out->ox = in->ox; out->oy = in->oy; out->oz = in->oz;

  tvdb_hash_entry* hin = NULL; size_t mask = 0;
  if (!tvdb_hash_build(in, &hin, &mask)) return false;

  static const int N[6][3] = {{1,0,0},{-1,0,0},{0,1,0},{0,-1,0},{0,0,1},{0,0,-1}};
  for (size_t i = 0; i < in->count; ++i) {
    int x = in->coords[i].x, y = in->coords[i].y, z = in->coords[i].z;
    float v_self = in->values[i];
    float r = v_self;
    bool keep = true;
    for (int n = 0; n < 6; ++n) {
      int nx = x + N[n][0], ny = y + N[n][1], nz = z + N[n][2];
      int j = tvdb_hash_get(hin, mask, nx, ny, nz);
      if (j < 0) { keep = false; break; }
      float v = in->values[j];
      if (v > r) r = v;
    }
    if (keep) {
      if (!tvdb_sparse_push(out, x, y, z, r)) { free(hin); return false; }
    }
  }
  free(hin);
  return true;
}

bool tvdb_erode_sparse(const tvdb_sparse_grid* in, int iterations,
                       tvdb_sparse_grid* out) {
  if (!in || !out || iterations <= 0) return false;
  tvdb_sparse_grid scratch; tvdb_sparse_grid_init(&scratch);
  out->count = 0;
  out->voxel_size = in->voxel_size;
  out->ox = in->ox; out->oy = in->oy; out->oz = in->oz;
  for (size_t i = 0; i < in->count; ++i) {
    if (!tvdb_sparse_push(out, in->coords[i].x, in->coords[i].y, in->coords[i].z, in->values[i])) {
      tvdb_sparse_grid_free(&scratch);
      return false;
    }
  }
  for (int it = 0; it < iterations; ++it) {
    if (!tvdb_erode_sparse_step(out, &scratch)) {
      tvdb_sparse_grid_free(&scratch); return false;
    }
    tvdb_sparse_grid tmp = *out; *out = scratch; scratch = tmp;
  }
  tvdb_sparse_grid_free(&scratch);
  return true;
}

// -------------------------------------------------------------------------
// 3D convolution (same-topology)
// -------------------------------------------------------------------------

bool tvdb_sparse_conv3d(const tvdb_sparse_grid* in,
                        const float* kernel,
                        int kx, int ky, int kz,
                        float pad_value,
                        tvdb_sparse_grid* out) {
  if (!in || !kernel || !out) return false;
  if (kx <= 0 || ky <= 0 || kz <= 0) return false;

  out->count = 0;
  out->voxel_size = in->voxel_size;
  out->ox = in->ox; out->oy = in->oy; out->oz = in->oz;
  if (in->count == 0) return true;
  if (!tvdb_sparse_grid_reserve(out, in->count)) return false;

  // Anchor (matches numpy/scipy: floor(k/2) regardless of parity).
  const int ax = kx / 2, ay = ky / 2, az = kz / 2;

  // Hash input coords for O(1) neighbor lookup.
  tvdb_hash_entry* tbl = NULL; size_t mask = 0;
  if (!tvdb_hash_build(in, &tbl, &mask)) return false;

  #pragma omp parallel for schedule(static)
  for (long long i = 0; i < (long long)in->count; ++i) {
    int cx = in->coords[i].x;
    int cy = in->coords[i].y;
    int cz = in->coords[i].z;
    float acc = 0.0f;
    // K[di,dj,dk] convolves over neighbors offset by (di-ax, dj-ay, dk-az).
    for (int dk = 0; dk < kz; ++dk) {
      int oz = cz + (dk - az);
      for (int dj = 0; dj < ky; ++dj) {
        int oy = cy + (dj - ay);
        for (int di = 0; di < kx; ++di) {
          int ox = cx + (di - ax);
          float w = kernel[((dk * ky) + dj) * kx + di];
          if (w == 0.0f) continue;
          int j = tvdb_hash_get(tbl, mask, ox, oy, oz);
          float v = (j >= 0) ? in->values[j] : pad_value;
          acc += w * v;
        }
      }
    }
    out->coords[i].x = cx;
    out->coords[i].y = cy;
    out->coords[i].z = cz;
    out->values[i] = acc;
  }
  out->count = in->count;
  free(tbl);
  return true;
}

// -------------------------------------------------------------------------
// Multi-channel sparse 3D convolution
// -------------------------------------------------------------------------

bool tvdb_sparse_conv3d_mc(const tvdb_sparse_grid* in,
                           const float* in_values, int c_in,
                           const float* kernel, int kx, int ky, int kz,
                           int c_out,
                           float pad_value,
                           tvdb_sparse_grid* out,
                           float** out_values_mc) {
  if (!in || !in_values || !kernel || !out || !out_values_mc) return false;
  if (kx <= 0 || ky <= 0 || kz <= 0 || c_in <= 0 || c_out <= 0) return false;

  out->count = 0;
  out->voxel_size = in->voxel_size;
  out->ox = in->ox; out->oy = in->oy; out->oz = in->oz;
  *out_values_mc = NULL;
  if (in->count == 0) return true;
  if (!tvdb_sparse_grid_reserve(out, in->count)) return false;

  *out_values_mc = (float *)calloc(in->count * (size_t)c_out, sizeof(float));
  if (!*out_values_mc) return false;

  const int ax = kx / 2, ay = ky / 2, az = kz / 2;
  const size_t spatial_stride = (size_t)c_out * (size_t)c_in;

  tvdb_hash_entry* tbl = NULL; size_t mask = 0;
  if (!tvdb_hash_build(in, &tbl, &mask)) {
    free(*out_values_mc); *out_values_mc = NULL;
    return false;
  }

  #pragma omp parallel for schedule(static)
  for (long long i = 0; i < (long long)in->count; ++i) {
    int cx = in->coords[i].x;
    int cy = in->coords[i].y;
    int cz = in->coords[i].z;
    out->coords[i].x = cx;
    out->coords[i].y = cy;
    out->coords[i].z = cz;
    float* acc = (*out_values_mc) + (size_t)i * (size_t)c_out;

    for (int dk = 0; dk < kz; ++dk) {
      int oz = cz + (dk - az);
      for (int dj = 0; dj < ky; ++dj) {
        int oy = cy + (dj - ay);
        for (int di = 0; di < kx; ++di) {
          int ox = cx + (di - ax);
          const float* W = kernel +
              (((size_t)dk * (size_t)ky + (size_t)dj) * (size_t)kx + (size_t)di) * spatial_stride;
          int j = tvdb_hash_get(tbl, mask, ox, oy, oz);
          if (j >= 0) {
            const float* in_v = in_values + (size_t)j * (size_t)c_in;
            for (int co = 0; co < c_out; ++co) {
              float s = 0.0f;
              for (int ci = 0; ci < c_in; ++ci) {
                s += W[(size_t)co * (size_t)c_in + (size_t)ci] * in_v[ci];
              }
              acc[co] += s;
            }
          } else {
            for (int co = 0; co < c_out; ++co) {
              float s = 0.0f;
              for (int ci = 0; ci < c_in; ++ci) {
                s += W[(size_t)co * (size_t)c_in + (size_t)ci] * pad_value;
              }
              acc[co] += s;
            }
          }
        }
      }
    }
  }
  out->count = in->count;
  free(tbl);
  return true;
}
