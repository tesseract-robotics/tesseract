// Analytic level-set primitive generators + SDF utilities. See
// tinyvdb_levelset.h. Pure-C, dependency-free, CPU. The dense data layout is
// x-fastest: idx(i,j,k) = (k*ny + j)*nx + i, matching the rest of tinyvdb.

#include "tinyvdb_levelset.h"
#include "tinyvdb_sample.h"  // tvdb_sample_trilinear_dense (rebuild re-signing)

#include <math.h>
#include <stdlib.h>

static inline float lvl_clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

static inline float lvl_minf(float a, float b) { return a < b ? a : b; }
static inline float lvl_maxf(float a, float b) { return a > b ? a : b; }

// Resolve a half_width argument (<=0 means default), returning the background
// distance bg = half_width * voxel_size and writing the effective half_width.
static inline float lvl_background(float half_width, float voxel_size,
                                   float* eff_half_width) {
  float hw = (half_width > 0.0f) ? half_width : TVDB_LEVEL_SET_HALF_WIDTH;
  if (eff_half_width) *eff_half_width = hw;
  return hw * voxel_size;
}

// Allocate `out` covering world AABB [lo, hi] with isotropic voxel_size, placing
// the origin so voxel-0 center == lo (cell-center convention) and voxel (n-1)
// center >= hi. Returns false on bad args / OOM.
static bool lvl_make_grid(const float lo[3], const float hi[3], float vs,
                          tvdb_dense_grid* out) {
  if (!out || vs <= 0.0f) return false;
  int n[3];
  for (int a = 0; a < 3; ++a) {
    float span = hi[a] - lo[a];
    if (span < 0.0f) return false;
    n[a] = (int)ceilf(span / vs) + 1;  // (n-1)*vs >= span
    if (n[a] < 1) n[a] = 1;
  }
  tvdb_dense_grid_init(out, n[0], n[1], n[2]);  // mallocs + zeroes data
  if (!out->data) return false;
  out->voxel_size = vs;
  out->ox = lo[0] - 0.5f * vs;
  out->oy = lo[1] - 0.5f * vs;
  out->oz = lo[2] - 0.5f * vs;
  return true;
}

// Fill every voxel with sdf(world_center), clamped to [-bg, bg].
typedef float (*lvl_sdf_fn)(float wx, float wy, float wz, const void* params);

static void lvl_fill(tvdb_dense_grid* g, float bg, lvl_sdf_fn fn,
                     const void* params) {
  const float vs = g->voxel_size;
  for (int k = 0; k < g->nz; ++k) {
    float wz = g->oz + ((float)k + 0.5f) * vs;
    for (int j = 0; j < g->ny; ++j) {
      float wy = g->oy + ((float)j + 0.5f) * vs;
      size_t row = ((size_t)k * g->ny + j) * g->nx;
      for (int i = 0; i < g->nx; ++i) {
        float wx = g->ox + ((float)i + 0.5f) * vs;
        float d = fn(wx, wy, wz, params);
        g->data[row + i] = lvl_clampf(d, -bg, bg);
      }
    }
  }
}

// ---- Sphere ----------------------------------------------------------------

typedef struct { float c[3], r; } lvl_sphere_t;

static float lvl_sphere_sdf(float wx, float wy, float wz, const void* p) {
  const lvl_sphere_t* s = (const lvl_sphere_t*)p;
  float dx = wx - s->c[0], dy = wy - s->c[1], dz = wz - s->c[2];
  return sqrtf(dx * dx + dy * dy + dz * dz) - s->r;
}

bool tvdb_level_set_sphere(float radius, const float center[3],
                           float voxel_size, float half_width,
                           tvdb_dense_grid* out) {
  if (!center || radius <= 0.0f || voxel_size <= 0.0f) return false;
  float bg = lvl_background(half_width, voxel_size, NULL);
  float ext = radius + bg;
  float lo[3], hi[3];
  for (int a = 0; a < 3; ++a) { lo[a] = center[a] - ext; hi[a] = center[a] + ext; }
  if (!lvl_make_grid(lo, hi, voxel_size, out)) return false;
  lvl_sphere_t s = { { center[0], center[1], center[2] }, radius };
  lvl_fill(out, bg, lvl_sphere_sdf, &s);
  return true;
}

// ---- Box -------------------------------------------------------------------

typedef struct { float c[3], he[3]; } lvl_box_t;

static float lvl_box_sdf(float wx, float wy, float wz, const void* p) {
  const lvl_box_t* b = (const lvl_box_t*)p;
  float qx = fabsf(wx - b->c[0]) - b->he[0];
  float qy = fabsf(wy - b->c[1]) - b->he[1];
  float qz = fabsf(wz - b->c[2]) - b->he[2];
  float ox = lvl_maxf(qx, 0.0f), oy = lvl_maxf(qy, 0.0f), oz = lvl_maxf(qz, 0.0f);
  float outside = sqrtf(ox * ox + oy * oy + oz * oz);
  float inside = lvl_minf(lvl_maxf(qx, lvl_maxf(qy, qz)), 0.0f);
  return outside + inside;
}

bool tvdb_level_set_box(const float half_extents[3], const float center[3],
                        float voxel_size, float half_width,
                        tvdb_dense_grid* out) {
  if (!half_extents || !center || voxel_size <= 0.0f) return false;
  if (half_extents[0] <= 0.0f || half_extents[1] <= 0.0f || half_extents[2] <= 0.0f)
    return false;
  float bg = lvl_background(half_width, voxel_size, NULL);
  float lo[3], hi[3];
  for (int a = 0; a < 3; ++a) {
    lo[a] = center[a] - half_extents[a] - bg;
    hi[a] = center[a] + half_extents[a] + bg;
  }
  if (!lvl_make_grid(lo, hi, voxel_size, out)) return false;
  lvl_box_t b = { { center[0], center[1], center[2] },
                  { half_extents[0], half_extents[1], half_extents[2] } };
  lvl_fill(out, bg, lvl_box_sdf, &b);
  return true;
}

// ---- Torus (XZ plane, axis Y) ----------------------------------------------

typedef struct { float c[3], R, r; } lvl_torus_t;

static float lvl_torus_sdf(float wx, float wy, float wz, const void* p) {
  const lvl_torus_t* t = (const lvl_torus_t*)p;
  float dx = wx - t->c[0], dy = wy - t->c[1], dz = wz - t->c[2];
  float qx = sqrtf(dx * dx + dz * dz) - t->R;  // distance in XZ from ring
  return sqrtf(qx * qx + dy * dy) - t->r;
}

bool tvdb_level_set_torus(float major_radius, float minor_radius,
                          const float center[3], float voxel_size,
                          float half_width, tvdb_dense_grid* out) {
  if (!center || major_radius <= 0.0f || minor_radius <= 0.0f ||
      voxel_size <= 0.0f)
    return false;
  float bg = lvl_background(half_width, voxel_size, NULL);
  float ext_xz = major_radius + minor_radius + bg;
  float ext_y = minor_radius + bg;
  float lo[3] = { center[0] - ext_xz, center[1] - ext_y, center[2] - ext_xz };
  float hi[3] = { center[0] + ext_xz, center[1] + ext_y, center[2] + ext_xz };
  if (!lvl_make_grid(lo, hi, voxel_size, out)) return false;
  lvl_torus_t t = { { center[0], center[1], center[2] }, major_radius, minor_radius };
  lvl_fill(out, bg, lvl_torus_sdf, &t);
  return true;
}

// ---- Capsule (segment p0->p1, radius) --------------------------------------

typedef struct { float a[3], ba[3], baba, r; } lvl_capsule_t;

static float lvl_capsule_sdf(float wx, float wy, float wz, const void* p) {
  const lvl_capsule_t* c = (const lvl_capsule_t*)p;
  float pax = wx - c->a[0], pay = wy - c->a[1], paz = wz - c->a[2];
  float h = 0.0f;
  if (c->baba > 0.0f) {
    h = (pax * c->ba[0] + pay * c->ba[1] + paz * c->ba[2]) / c->baba;
    h = lvl_clampf(h, 0.0f, 1.0f);
  }
  float dx = pax - c->ba[0] * h;
  float dy = pay - c->ba[1] * h;
  float dz = paz - c->ba[2] * h;
  return sqrtf(dx * dx + dy * dy + dz * dz) - c->r;
}

bool tvdb_level_set_capsule(const float p0[3], const float p1[3], float radius,
                            float voxel_size, float half_width,
                            tvdb_dense_grid* out) {
  if (!p0 || !p1 || radius <= 0.0f || voxel_size <= 0.0f) return false;
  float bg = lvl_background(half_width, voxel_size, NULL);
  float ext = radius + bg;
  float lo[3], hi[3];
  for (int a = 0; a < 3; ++a) {
    lo[a] = lvl_minf(p0[a], p1[a]) - ext;
    hi[a] = lvl_maxf(p0[a], p1[a]) + ext;
  }
  if (!lvl_make_grid(lo, hi, voxel_size, out)) return false;
  lvl_capsule_t c;
  for (int a = 0; a < 3; ++a) { c.a[a] = p0[a]; c.ba[a] = p1[a] - p0[a]; }
  c.baba = c.ba[0] * c.ba[0] + c.ba[1] * c.ba[1] + c.ba[2] * c.ba[2];
  c.r = radius;
  lvl_fill(out, bg, lvl_capsule_sdf, &c);
  return true;
}

// ---- Platonic solids (convex half-space SDF) -------------------------------

// Fill `out` with `count` unit outward face normals for the given face_count and
// report the inradius/circumradius ratio. Returns the face count, or 0 if
// face_count is not one of {4,6,8,12,20}. `out` must hold up to 20*3 floats.
static int lvl_platonic_normals(int face_count, float* out, float* in_over_circ) {
  // Raw (un-normalized) normals; normalized below. PHI = golden ratio.
  const float PHI = 1.6180339887498949f;
  const float IPHI = 0.6180339887498949f;  // 1/PHI
  float raw[60];
  int n = 0;
  switch (face_count) {
    case 4: {  // tetrahedron: normals opposite the (1,1,1)-type vertices
      const float t[4][3] = { {-1,-1,-1}, {-1,1,1}, {1,-1,1}, {1,1,-1} };
      for (int i = 0; i < 4; ++i) for (int a = 0; a < 3; ++a) raw[3*i+a] = t[i][a];
      n = 4; *in_over_circ = 1.0f / 3.0f;
      break;
    }
    case 6: {  // cube: +/- axis normals
      const float c[6][3] = { {1,0,0},{-1,0,0},{0,1,0},{0,-1,0},{0,0,1},{0,0,-1} };
      for (int i = 0; i < 6; ++i) for (int a = 0; a < 3; ++a) raw[3*i+a] = c[i][a];
      n = 6; *in_over_circ = 0.57735026918962573f;  // 1/sqrt(3)
      break;
    }
    case 8: {  // octahedron: (+/-1,+/-1,+/-1)
      for (int sx = 0; sx < 2; ++sx) for (int sy = 0; sy < 2; ++sy)
        for (int sz = 0; sz < 2; ++sz) {
          raw[3*n+0] = sx ? -1.0f : 1.0f;
          raw[3*n+1] = sy ? -1.0f : 1.0f;
          raw[3*n+2] = sz ? -1.0f : 1.0f;
          ++n;
        }
      *in_over_circ = 0.57735026918962573f;  // 1/sqrt(3)
      break;
    }
    case 12: {  // dodecahedron faces = icosahedron vertex directions
      const float v[12][3] = {
        {0,1,PHI},{0,1,-PHI},{0,-1,PHI},{0,-1,-PHI},
        {1,PHI,0},{1,-PHI,0},{-1,PHI,0},{-1,-PHI,0},
        {PHI,0,1},{PHI,0,-1},{-PHI,0,1},{-PHI,0,-1} };
      for (int i = 0; i < 12; ++i) for (int a = 0; a < 3; ++a) raw[3*i+a] = v[i][a];
      n = 12; *in_over_circ = 0.79465447229176612f;
      break;
    }
    case 20: {  // icosahedron faces = dodecahedron vertex directions
      for (int sx = 0; sx < 2; ++sx) for (int sy = 0; sy < 2; ++sy)
        for (int sz = 0; sz < 2; ++sz) {  // 8 cube-corner directions
          raw[3*n+0] = sx ? -1.0f : 1.0f;
          raw[3*n+1] = sy ? -1.0f : 1.0f;
          raw[3*n+2] = sz ? -1.0f : 1.0f;
          ++n;
        }
      const float v[12][3] = {
        {0,IPHI,PHI},{0,IPHI,-PHI},{0,-IPHI,PHI},{0,-IPHI,-PHI},
        {IPHI,PHI,0},{IPHI,-PHI,0},{-IPHI,PHI,0},{-IPHI,-PHI,0},
        {PHI,0,IPHI},{PHI,0,-IPHI},{-PHI,0,IPHI},{-PHI,0,-IPHI} };
      for (int i = 0; i < 12; ++i, ++n)
        for (int a = 0; a < 3; ++a) raw[3*n+a] = v[i][a];
      *in_over_circ = 0.79465447229176612f;
      break;
    }
    default:
      return 0;
  }
  for (int i = 0; i < n; ++i) {
    float x = raw[3*i+0], y = raw[3*i+1], z = raw[3*i+2];
    float len = sqrtf(x*x + y*y + z*z);
    out[3*i+0] = x/len; out[3*i+1] = y/len; out[3*i+2] = z/len;
  }
  return n;
}

typedef struct { float c[3]; const float* n; int count; float offset; } lvl_platonic_t;

static float lvl_platonic_sdf(float wx, float wy, float wz, const void* p) {
  const lvl_platonic_t* s = (const lvl_platonic_t*)p;
  float px = wx - s->c[0], py = wy - s->c[1], pz = wz - s->c[2];
  float m = -3.4e38f;
  for (int i = 0; i < s->count; ++i) {
    float d = s->n[3*i+0]*px + s->n[3*i+1]*py + s->n[3*i+2]*pz;
    if (d > m) m = d;
  }
  return m - s->offset;
}

bool tvdb_level_set_platonic(int face_count, float radius, const float center[3],
                             float voxel_size, float half_width,
                             tvdb_dense_grid* out) {
  if (!center || radius <= 0.0f || voxel_size <= 0.0f) return false;
  float normals[60];
  float ratio = 0.0f;
  int count = lvl_platonic_normals(face_count, normals, &ratio);
  if (count == 0) return false;
  float bg = lvl_background(half_width, voxel_size, NULL);
  // The +bg band reaches farthest along vertex directions: |p| = (r_in+bg)/ratio.
  float ext = radius + bg / ratio;
  float lo[3], hi[3];
  for (int a = 0; a < 3; ++a) { lo[a] = center[a] - ext; hi[a] = center[a] + ext; }
  if (!lvl_make_grid(lo, hi, voxel_size, out)) return false;
  lvl_platonic_t s = { { center[0], center[1], center[2] }, normals, count,
                       radius * ratio };
  lvl_fill(out, bg, lvl_platonic_sdf, &s);
  return true;
}

// ---- SDF utilities ---------------------------------------------------------

static bool lvl_clone_shape(const tvdb_dense_grid* in, tvdb_dense_grid* out) {
  if (!in || !out || !in->data) return false;
  tvdb_dense_grid_init(out, in->nx, in->ny, in->nz);
  if (!out->data && (size_t)in->nx * in->ny * in->nz > 0) return false;
  out->voxel_size = in->voxel_size;
  out->ox = in->ox; out->oy = in->oy; out->oz = in->oz;
  return true;
}

bool tvdb_sdf_to_fog_volume(const tvdb_dense_grid* sdf, float half_width,
                            tvdb_dense_grid* out) {
  if (!lvl_clone_shape(sdf, out)) return false;
  float gamma = lvl_background(half_width, sdf->voxel_size, NULL);
  if (gamma <= 0.0f) gamma = sdf->voxel_size;
  size_t n = (size_t)sdf->nx * sdf->ny * sdf->nz;
  for (size_t i = 0; i < n; ++i)
    out->data[i] = lvl_clampf(-sdf->data[i] / gamma, 0.0f, 1.0f);
  return true;
}

bool tvdb_sdf_interior_mask(const tvdb_dense_grid* sdf, float isovalue,
                            tvdb_dense_grid* out) {
  if (!lvl_clone_shape(sdf, out)) return false;
  size_t n = (size_t)sdf->nx * sdf->ny * sdf->nz;
  for (size_t i = 0; i < n; ++i)
    out->data[i] = (sdf->data[i] < isovalue) ? 1.0f : 0.0f;
  return true;
}

// ---- Connected-component helpers (segmentation / enclosed regions) ---------

// Fill `off` with the neighbor offsets for the connectivity (6 = face, 26 =
// face+edge+vertex; anything else falls back to 6). Returns the offset count.
static int lvl_neighbor_offsets(int connectivity, int off[26][3]) {
  if (connectivity == 26) {
    int n = 0;
    for (int dz = -1; dz <= 1; ++dz)
      for (int dy = -1; dy <= 1; ++dy)
        for (int dx = -1; dx <= 1; ++dx) {
          if (dx == 0 && dy == 0 && dz == 0) continue;
          off[n][0] = dx; off[n][1] = dy; off[n][2] = dz; ++n;
        }
    return 26;
  }
  static const int o6[6][3] = { {1,0,0},{-1,0,0},{0,1,0},{0,-1,0},{0,0,1},{0,0,-1} };
  for (int i = 0; i < 6; ++i) { off[i][0]=o6[i][0]; off[i][1]=o6[i][1]; off[i][2]=o6[i][2]; }
  return 6;
}

bool tvdb_sdf_segmentation(const tvdb_dense_grid* sdf, float isovalue,
                           int connectivity, tvdb_dense_grid** out_grids,
                           int* out_count) {
  if (out_grids) *out_grids = NULL;
  if (out_count) *out_count = 0;
  if (!sdf || !sdf->data || !out_grids || !out_count) return false;
  int nx = sdf->nx, ny = sdf->ny, nz = sdf->nz;
  size_t n = (size_t)nx * ny * nz;
  if (n == 0) return true;  // no components

  int off[26][3];
  int noff = lvl_neighbor_offsets(connectivity, off);

  // Background fill (largest value = the exterior clamp for a narrow band).
  float bg = -3.4e38f;
  for (size_t i = 0; i < n; ++i) if (sdf->data[i] > bg) bg = sdf->data[i];

  int32_t* labels = (int32_t*)calloc(n, sizeof(int32_t));
  size_t* stack = (size_t*)malloc(n * sizeof(size_t));
  if (!labels || !stack) { free(labels); free(stack); return false; }

  int count = 0;
  for (size_t s = 0; s < n; ++s) {
    if (sdf->data[s] >= isovalue || labels[s] != 0) continue;
    ++count;
    size_t sp = 0;
    stack[sp++] = s; labels[s] = count;
    while (sp > 0) {
      size_t v = stack[--sp];
      int vi = (int)(v % nx), vj = (int)((v / nx) % ny), vk = (int)(v / ((size_t)nx * ny));
      for (int t = 0; t < noff; ++t) {
        int ni = vi + off[t][0], nj = vj + off[t][1], nk = vk + off[t][2];
        if (ni < 0 || ni >= nx || nj < 0 || nj >= ny || nk < 0 || nk >= nz) continue;
        size_t nidx = ((size_t)nk * ny + nj) * nx + ni;
        if (sdf->data[nidx] < isovalue && labels[nidx] == 0) {
          labels[nidx] = count;
          stack[sp++] = nidx;
        }
      }
    }
  }
  free(stack);

  if (count == 0) { free(labels); return true; }

  tvdb_dense_grid* grids = (tvdb_dense_grid*)malloc((size_t)count * sizeof(tvdb_dense_grid));
  if (!grids) { free(labels); return false; }
  for (int c = 1; c <= count; ++c) {
    tvdb_dense_grid* g = &grids[c - 1];
    tvdb_dense_grid_init(g, nx, ny, nz);
    if (!g->data) {  // OOM: tear down what we built
      for (int p = 0; p < c - 1; ++p) tvdb_dense_grid_free(&grids[p]);
      free(grids); free(labels);
      return false;
    }
    g->voxel_size = sdf->voxel_size; g->ox = sdf->ox; g->oy = sdf->oy; g->oz = sdf->oz;
    for (size_t i = 0; i < n; ++i) {
      if (labels[i] == 0 || labels[i] == c) g->data[i] = sdf->data[i];  // exterior or this object
      else g->data[i] = bg;                                              // fill other objects solid
    }
  }
  free(labels);
  *out_grids = grids;
  *out_count = count;
  return true;
}

bool tvdb_sdf_extract_enclosed_regions(const tvdb_dense_grid* sdf, float isovalue,
                                       int connectivity, tvdb_dense_grid* out) {
  if (!lvl_clone_shape(sdf, out)) return false;
  int nx = sdf->nx, ny = sdf->ny, nz = sdf->nz;
  size_t n = (size_t)nx * ny * nz;
  if (n == 0) return true;

  int off[26][3];
  int noff = lvl_neighbor_offsets(connectivity, off);

  uint8_t* vis = (uint8_t*)calloc(n, 1);
  size_t* stack = (size_t*)malloc(n * sizeof(size_t));
  if (!vis || !stack) { free(vis); free(stack); tvdb_dense_grid_free(out); return false; }

  // Seed: every exterior voxel on the grid boundary (connected to "infinity").
  size_t sp = 0;
  for (int k = 0; k < nz; ++k)
    for (int j = 0; j < ny; ++j)
      for (int i = 0; i < nx; ++i) {
        if (i != 0 && i != nx - 1 && j != 0 && j != ny - 1 && k != 0 && k != nz - 1) continue;
        size_t idx = ((size_t)k * ny + j) * nx + i;
        if (sdf->data[idx] >= isovalue && !vis[idx]) { vis[idx] = 1; stack[sp++] = idx; }
      }
  // Flood the open exterior.
  while (sp > 0) {
    size_t v = stack[--sp];
    int vi = (int)(v % nx), vj = (int)((v / nx) % ny), vk = (int)(v / ((size_t)nx * ny));
    for (int t = 0; t < noff; ++t) {
      int ni = vi + off[t][0], nj = vj + off[t][1], nk = vk + off[t][2];
      if (ni < 0 || ni >= nx || nj < 0 || nj >= ny || nk < 0 || nk >= nz) continue;
      size_t nidx = ((size_t)nk * ny + nj) * nx + ni;
      if (sdf->data[nidx] >= isovalue && !vis[nidx]) { vis[nidx] = 1; stack[sp++] = nidx; }
    }
  }
  // Enclosed = exterior voxels the boundary flood never reached.
  for (size_t i = 0; i < n; ++i)
    out->data[i] = (sdf->data[i] >= isovalue && !vis[i]) ? 1.0f : 0.0f;

  free(vis); free(stack);
  return true;
}

// ---- Topological measures (Euler characteristic / genus) -------------------

// Count connected components of the interior (sdf < isovalue) with the given
// connectivity. Returns -1 on OOM.
static int lvl_count_components(const tvdb_dense_grid* sdf, float isovalue,
                               int connectivity) {
  int nx = sdf->nx, ny = sdf->ny, nz = sdf->nz;
  size_t n = (size_t)nx * ny * nz;
  if (n == 0) return 0;
  int off[26][3];
  int noff = lvl_neighbor_offsets(connectivity, off);
  uint8_t* seen = (uint8_t*)calloc(n, 1);
  size_t* stack = (size_t*)malloc(n * sizeof(size_t));
  if (!seen || !stack) { free(seen); free(stack); return -1; }
  int comp = 0;
  for (size_t s = 0; s < n; ++s) {
    if (sdf->data[s] >= isovalue || seen[s]) continue;
    ++comp;
    size_t sp = 0; stack[sp++] = s; seen[s] = 1;
    while (sp > 0) {
      size_t v = stack[--sp];
      int vi = (int)(v % nx), vj = (int)((v / nx) % ny), vk = (int)(v / ((size_t)nx * ny));
      for (int t = 0; t < noff; ++t) {
        int ni = vi + off[t][0], nj = vj + off[t][1], nk = vk + off[t][2];
        if (ni < 0 || ni >= nx || nj < 0 || nj >= ny || nk < 0 || nk >= nz) continue;
        size_t nidx = ((size_t)nk * ny + nj) * nx + ni;
        if (sdf->data[nidx] < isovalue && !seen[nidx]) { seen[nidx] = 1; stack[sp++] = nidx; }
      }
    }
  }
  free(seen); free(stack);
  return comp;
}

#define LVL_MARK(arr, idx, cnt) do { if (!(arr)[(idx)]) { (arr)[(idx)] = 1; ++(cnt); } } while (0)

// Euler characteristic of the interior solid (sdf < isovalue) as a cubical
// complex: chi = V - E + F - cubes over the union of unit cubes. Writes the
// result to *chi. Returns false on OOM.
static bool lvl_chi_solid(const tvdb_dense_grid* sdf, float isovalue, long* chi) {
  int nx = sdf->nx, ny = sdf->ny, nz = sdf->nz;
  size_t n = (size_t)nx * ny * nz;
  if (n == 0) { *chi = 0; return true; }
  size_t vn  = (size_t)(nx+1) * (ny+1) * (nz+1);
  size_t exn = (size_t)nx     * (ny+1) * (nz+1);
  size_t eyn = (size_t)(nx+1) * ny     * (nz+1);
  size_t ezn = (size_t)(nx+1) * (ny+1) * nz;
  size_t fxn = (size_t)(nx+1) * ny     * nz;
  size_t fyn = (size_t)nx     * (ny+1) * nz;
  size_t fzn = (size_t)nx     * ny     * (nz+1);
  uint8_t* V  = (uint8_t*)calloc(vn, 1);
  uint8_t* EX = (uint8_t*)calloc(exn, 1);
  uint8_t* EY = (uint8_t*)calloc(eyn, 1);
  uint8_t* EZ = (uint8_t*)calloc(ezn, 1);
  uint8_t* FX = (uint8_t*)calloc(fxn, 1);
  uint8_t* FY = (uint8_t*)calloc(fyn, 1);
  uint8_t* FZ = (uint8_t*)calloc(fzn, 1);
  if (!V || !EX || !EY || !EZ || !FX || !FY || !FZ) {
    free(V); free(EX); free(EY); free(EZ); free(FX); free(FY); free(FZ);
    return false;
  }
  long nv = 0, ne = 0, nf = 0, nc = 0;
  for (int k = 0; k < nz; ++k)
    for (int j = 0; j < ny; ++j)
      for (int i = 0; i < nx; ++i) {
        if (sdf->data[((size_t)k*ny + j)*nx + i] >= isovalue) continue;
        ++nc;
        for (int dz = 0; dz < 2; ++dz)
          for (int dy = 0; dy < 2; ++dy)
            for (int dx = 0; dx < 2; ++dx) {
              size_t vi = ((size_t)(k+dz)*(ny+1) + (j+dy))*(nx+1) + (i+dx);
              LVL_MARK(V, vi, nv);
            }
        for (int dz = 0; dz < 2; ++dz)
          for (int dy = 0; dy < 2; ++dy) {
            size_t e = ((size_t)(k+dz)*(ny+1) + (j+dy))*nx + i;            // x-edge
            LVL_MARK(EX, e, ne);
          }
        for (int dz = 0; dz < 2; ++dz)
          for (int dx = 0; dx < 2; ++dx) {
            size_t e = ((size_t)(k+dz)*ny + j)*(nx+1) + (i+dx);            // y-edge
            LVL_MARK(EY, e, ne);
          }
        for (int dy = 0; dy < 2; ++dy)
          for (int dx = 0; dx < 2; ++dx) {
            size_t e = ((size_t)k*(ny+1) + (j+dy))*(nx+1) + (i+dx);        // z-edge
            LVL_MARK(EZ, e, ne);
          }
        for (int dx = 0; dx < 2; ++dx) {
          size_t f = ((size_t)k*ny + j)*(nx+1) + (i+dx);                   // face perp x
          LVL_MARK(FX, f, nf);
        }
        for (int dy = 0; dy < 2; ++dy) {
          size_t f = ((size_t)k*(ny+1) + (j+dy))*nx + i;                   // face perp y
          LVL_MARK(FY, f, nf);
        }
        for (int dz = 0; dz < 2; ++dz) {
          size_t f = ((size_t)(k+dz)*ny + j)*nx + i;                       // face perp z
          LVL_MARK(FZ, f, nf);
        }
      }
  free(V); free(EX); free(EY); free(EZ); free(FX); free(FY); free(FZ);
  *chi = nv - ne + nf - nc;
  return true;
}

#undef LVL_MARK

double tvdb_level_set_euler_characteristic(const tvdb_dense_grid* sdf,
                                           float isovalue) {
  if (!sdf || !sdf->data) return 0.0;
  long chi = 0;
  if (!lvl_chi_solid(sdf, isovalue, &chi)) return 0.0;
  return 2.0 * (double)chi;  // chi(surface) = 2 * chi(solid)
}

int tvdb_level_set_genus(const tvdb_dense_grid* sdf, float isovalue) {
  if (!sdf || !sdf->data) return 0;
  long chi = 0;
  if (!lvl_chi_solid(sdf, isovalue, &chi)) return 0;
  int comps = lvl_count_components(sdf, isovalue, 26);
  if (comps < 0) return 0;
  return comps - (int)chi;  // total genus = components - chi(solid)
}

// ---- Rebuild from isosurface (mesh round-trip) -----------------------------

bool tvdb_level_set_rebuild(const tvdb_dense_grid* sdf, float isovalue,
                            float voxel_size, float half_width,
                            int sign_method, tvdb_dense_grid* out) {
  if (!sdf || !sdf->data || !out) return false;
  float vs = (voxel_size > 0.0f) ? voxel_size : sdf->voxel_size;
  float hw = (half_width > 0.0f) ? half_width : TVDB_LEVEL_SET_HALF_WIDTH;
  if (vs <= 0.0f) return false;

  // 1) Extract the isosurface as a world-space triangle mesh.
  tvdb_triangle_mesh mesh;
  tvdb_triangle_mesh_init(&mesh);
  if (!tvdb_sdf_to_mesh(sdf, isovalue, &mesh, NULL) || mesh.face_count == 0) {
    tvdb_triangle_mesh_free(&mesh);
    return false;
  }
  // 2) Reconvert the mesh to a clean narrow-band SDF (optionally resampled).
  //    mesh_to_sdf's band_width is in WORLD units, so convert from voxels.
  bool ok = tvdb_mesh_to_sdf_vdb(&mesh, vs, hw * vs, out,
                                 (tvdb_sign_method)sign_method, NULL);
  tvdb_triangle_mesh_free(&mesh);
  if (!ok) return false;

  // 3) Re-sign from the input. The mesh-to-SDF sign is normal-based and depends
  //    on the marching-cubes winding (which can disagree); the inside/outside
  //    sign is invariant under the input's (possibly damaged/scaled) values, so
  //    take it directly: out = sign(input(p) - isovalue) * |out|.
  for (int k = 0; k < out->nz; ++k)
    for (int j = 0; j < out->ny; ++j)
      for (int i = 0; i < out->nx; ++i) {
        float px = out->ox + ((float)i + 0.5f) * out->voxel_size;
        float py = out->oy + ((float)j + 0.5f) * out->voxel_size;
        float pz = out->oz + ((float)k + 0.5f) * out->voxel_size;
        float in_val = tvdb_sample_trilinear_dense(sdf, px, py, pz) - isovalue;
        size_t idx = ((size_t)k * out->ny + j) * out->nx + i;
        float mag = fabsf(out->data[idx]);
        out->data[idx] = (in_val < 0.0f) ? -mag : mag;
      }
  return true;
}
