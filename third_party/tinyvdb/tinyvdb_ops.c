#include "tinyvdb_ops.h"
#include "tinyvdb_ops_internal.h"
#include "tinyvdb_simd.h"

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

// -------------------------------------------------------------------------
// dense vec grid lifecycle
// -------------------------------------------------------------------------

void tvdb_dense_vec_grid_init(tvdb_dense_vec_grid* grid, int nx, int ny, int nz) {
  grid->nx = nx;
  grid->ny = ny;
  grid->nz = nz;
  grid->ox = grid->oy = grid->oz = 0.0f;
  grid->voxel_size = 1.0f;
  grid->data = (float*)malloc((size_t)nx * (size_t)ny * (size_t)nz * 3u * sizeof(float));
  if (grid->data) {
    memset(grid->data, 0, (size_t)nx * (size_t)ny * (size_t)nz * 3u * sizeof(float));
  }
}

void tvdb_dense_vec_grid_free(tvdb_dense_vec_grid* grid) {
  if (grid->data) free(grid->data);
  grid->data = NULL;
}

// -------------------------------------------------------------------------
// helpers
// -------------------------------------------------------------------------

static size_t tvdb_grid_voxels(const tvdb_dense_grid* g) {
  return (size_t)g->nx * g->ny * g->nz;
}

static int tvdb_grid_same_shape(const tvdb_dense_grid* a, const tvdb_dense_grid* b) {
  return a->nx == b->nx && a->ny == b->ny && a->nz == b->nz;
}

// -------------------------------------------------------------------------
// Phase 1: morphology
// -------------------------------------------------------------------------

// SDF convention: f < 0 = inside, f > 0 = outside.
//   dilate (grow inside)  = pointwise 6-neighbor min
//   erode  (shrink inside)= pointwise 6-neighbor max
// One iteration moves the zero-isosurface by ~one voxel.

static void tvdb_morph_step(const tvdb_dense_grid* in, tvdb_dense_grid* out, int is_dilate) {
  const int nx = in->nx, ny = in->ny, nz = in->nz;
  for (int iz = 0; iz < nz; ++iz) {
    for (int iy = 0; iy < ny; ++iy) {
      for (int ix = 0; ix < nx; ++ix) {
        float c = in->data[tvdb_idx(in, ix, iy, iz)];
        float xm = tvdb_at(in, ix - 1, iy, iz);
        float xp = tvdb_at(in, ix + 1, iy, iz);
        float ym = tvdb_at(in, ix, iy - 1, iz);
        float yp = tvdb_at(in, ix, iy + 1, iz);
        float zm = tvdb_at(in, ix, iy, iz - 1);
        float zp = tvdb_at(in, ix, iy, iz + 1);
        float r = c;
        if (is_dilate) {
          if (xm < r) r = xm; if (xp < r) r = xp;
          if (ym < r) r = ym; if (yp < r) r = yp;
          if (zm < r) r = zm; if (zp < r) r = zp;
        } else {
          if (xm > r) r = xm; if (xp > r) r = xp;
          if (ym > r) r = ym; if (yp > r) r = yp;
          if (zm > r) r = zm; if (zp > r) r = zp;
        }
        out->data[tvdb_idx(out, ix, iy, iz)] = r;
      }
    }
  }
}

static void tvdb_morph_iter(tvdb_dense_grid* grid, int iterations, int is_dilate) {
  if (iterations <= 0 || grid->data == NULL) return;
  const size_t nv = (size_t)tvdb_grid_voxels(grid);
  float* tmp = (float*)malloc(nv * sizeof(float));
  if (!tmp) return;

  tvdb_dense_grid scratch = *grid;
  scratch.data = tmp;

  for (int it = 0; it < iterations; ++it) {
    tvdb_morph_step(grid, &scratch, is_dilate);
    // swap data pointers
    float* swap = grid->data;
    grid->data = scratch.data;
    scratch.data = swap;
  }
  // After even iterations, grid->data == original buffer. After odd, grid->data == tmp.
  // We need the result in the caller's original buffer. If pointer was swapped to tmp,
  // copy back and free tmp.
  if (grid->data == tmp) {
    memcpy(scratch.data, tmp, nv * sizeof(float));
    grid->data = scratch.data;
    free(tmp);
  } else {
    free(tmp);
  }
}

void tvdb_dilate(tvdb_dense_grid* grid, int iterations) {
  tvdb_morph_iter(grid, iterations, /*is_dilate=*/1);
}
void tvdb_erode(tvdb_dense_grid* grid, int iterations) {
  tvdb_morph_iter(grid, iterations, /*is_dilate=*/0);
}
void tvdb_open(tvdb_dense_grid* grid, int iterations) {
  tvdb_erode(grid, iterations);
  tvdb_dilate(grid, iterations);
}
void tvdb_close(tvdb_dense_grid* grid, int iterations) {
  tvdb_dilate(grid, iterations);
  tvdb_erode(grid, iterations);
}

// -------------------------------------------------------------------------
// Phase 1: filtering
// -------------------------------------------------------------------------

// Apply a separable 1-D kernel along one axis.
// axis: 0 = x, 1 = y, 2 = z.
static void tvdb_separable_pass(const tvdb_dense_grid* in, tvdb_dense_grid* out,
                                const float* kernel, int radius, int axis) {
  const int nx = in->nx, ny = in->ny, nz = in->nz;
  for (int iz = 0; iz < nz; ++iz) {
    for (int iy = 0; iy < ny; ++iy) {
      for (int ix = 0; ix < nx; ++ix) {
        float acc = 0.0f;
        for (int k = -radius; k <= radius; ++k) {
          int sx = ix, sy = iy, sz = iz;
          if (axis == 0) sx = ix + k;
          else if (axis == 1) sy = iy + k;
          else sz = iz + k;
          acc += kernel[k + radius] * tvdb_at(in, sx, sy, sz);
        }
        out->data[tvdb_idx(out, ix, iy, iz)] = acc;
      }
    }
  }
}

static void tvdb_apply_separable(tvdb_dense_grid* grid, const float* kernel,
                                 int radius, int iterations) {
  if (iterations <= 0 || radius <= 0 || grid->data == NULL) return;
  const size_t nv = (size_t)tvdb_grid_voxels(grid);
  float* buf_a = (float*)malloc(nv * sizeof(float));
  float* buf_b = (float*)malloc(nv * sizeof(float));
  if (!buf_a || !buf_b) {
    free(buf_a);
    free(buf_b);
    return;
  }

  tvdb_dense_grid ping = *grid; ping.data = buf_a;
  tvdb_dense_grid pong = *grid; pong.data = buf_b;

  for (int it = 0; it < iterations; ++it) {
    tvdb_separable_pass(grid, &ping, kernel, radius, 0);
    tvdb_separable_pass(&ping, &pong, kernel, radius, 1);
    tvdb_separable_pass(&pong, grid, kernel, radius, 2);
  }
  free(buf_a);
  free(buf_b);
}

void tvdb_gaussian_filter(tvdb_dense_grid* grid, int width, int iterations) {
  if (width <= 0) return;
  const int radius = width;
  const int len = 2 * radius + 1;
  const float sigma = (float)width * 0.5f;
  const float two_s2 = 2.0f * sigma * sigma;
  float* k = (float*)malloc((size_t)len * sizeof(float));
  if (!k) return;
  float sum = 0.0f;
  for (int i = -radius; i <= radius; ++i) {
    float v = expf(-((float)(i * i)) / two_s2);
    k[i + radius] = v;
    sum += v;
  }
  for (int i = 0; i < len; ++i) k[i] /= sum;
  tvdb_apply_separable(grid, k, radius, iterations);
  free(k);
}

void tvdb_mean_filter(tvdb_dense_grid* grid, int width, int iterations) {
  if (width <= 0) return;
  const int radius = width;
  const int len = 2 * radius + 1;
  float* k = (float*)malloc((size_t)len * sizeof(float));
  if (!k) return;
  const float w = 1.0f / (float)len;
  for (int i = 0; i < len; ++i) k[i] = w;
  tvdb_apply_separable(grid, k, radius, iterations);
  free(k);
}

// laplacian_filter: explicit-Euler diffusion step using a 7-point stencil.
// dt = h^2 / 8 (CFL stable for 3-D heat eq, margin below h^2/6).
void tvdb_laplacian_filter(tvdb_dense_grid* grid, int iterations) {
  if (iterations <= 0 || grid->data == NULL) return;
  const size_t nv = (size_t)tvdb_grid_voxels(grid);
  float* tmp = (float*)malloc(nv * sizeof(float));
  if (!tmp) return;

  tvdb_dense_grid scratch = *grid;
  scratch.data = tmp;

  const float dt = 1.0f / 8.0f;  // h^2 cancels in the discrete laplacian below
  // discrete laplacian L = (sum6 - 6 c) / h^2; update u += dt * h^2 * L = dt*(sum6 - 6 c)

  for (int it = 0; it < iterations; ++it) {
    const int nx = grid->nx, ny = grid->ny, nz = grid->nz;
    for (int iz = 0; iz < nz; ++iz) {
      for (int iy = 0; iy < ny; ++iy) {
        for (int ix = 0; ix < nx; ++ix) {
          float c  = grid->data[tvdb_idx(grid, ix, iy, iz)];
          float s  = tvdb_at(grid, ix - 1, iy, iz)
                   + tvdb_at(grid, ix + 1, iy, iz)
                   + tvdb_at(grid, ix, iy - 1, iz)
                   + tvdb_at(grid, ix, iy + 1, iz)
                   + tvdb_at(grid, ix, iy, iz - 1)
                   + tvdb_at(grid, ix, iy, iz + 1);
          tmp[tvdb_idx(grid, ix, iy, iz)] = c + dt * (s - 6.0f * c);
        }
      }
    }
    memcpy(grid->data, tmp, nv * sizeof(float));
  }
  free(tmp);
}

// -------------------------------------------------------------------------
// Phase 1: CSG
// -------------------------------------------------------------------------

// SDF convention: union = min, intersection = max, difference = max(a, -b).
// Requires identical grid dimensions; result must be pre-allocated to same shape.

void tvdb_csg_union(const tvdb_dense_grid* a, const tvdb_dense_grid* b, tvdb_dense_grid* result) {
  if (!tvdb_grid_same_shape(a, b) || !tvdb_grid_same_shape(a, result)) return;
  const size_t nv = (size_t)tvdb_grid_voxels(a);
  for (size_t i = 0; i < nv; ++i) {
    float va = a->data[i], vb = b->data[i];
    result->data[i] = va < vb ? va : vb;
  }
}

void tvdb_csg_intersection(const tvdb_dense_grid* a, const tvdb_dense_grid* b, tvdb_dense_grid* result) {
  if (!tvdb_grid_same_shape(a, b) || !tvdb_grid_same_shape(a, result)) return;
  const size_t nv = (size_t)tvdb_grid_voxels(a);
  for (size_t i = 0; i < nv; ++i) {
    float va = a->data[i], vb = b->data[i];
    result->data[i] = va > vb ? va : vb;
  }
}

void tvdb_csg_difference(const tvdb_dense_grid* a, const tvdb_dense_grid* b, tvdb_dense_grid* result) {
  if (!tvdb_grid_same_shape(a, b) || !tvdb_grid_same_shape(a, result)) return;
  const size_t nv = (size_t)tvdb_grid_voxels(a);
  for (size_t i = 0; i < nv; ++i) {
    float va = a->data[i], nb = -b->data[i];
    result->data[i] = va > nb ? va : nb;
  }
}

// -------------------------------------------------------------------------
// Phase 1: measurement
// -------------------------------------------------------------------------

// Surface area: count zero-crossing faces (per +x/+y/+z neighbor pair),
// scale by voxel_size^2. Simple, monotonic estimator.
float tvdb_surface_area(const tvdb_dense_grid* grid) {
  if (!grid->data) return 0.0f;
  const int nx = grid->nx, ny = grid->ny, nz = grid->nz;
  size_t crossings = 0;
  for (int iz = 0; iz < nz; ++iz) {
    for (int iy = 0; iy < ny; ++iy) {
      for (int ix = 0; ix < nx; ++ix) {
        float c = grid->data[tvdb_idx(grid, ix, iy, iz)];
        if (ix + 1 < nx) {
          float n = grid->data[tvdb_idx(grid, ix + 1, iy, iz)];
          if ((c <= 0.0f) != (n <= 0.0f)) ++crossings;
        }
        if (iy + 1 < ny) {
          float n = grid->data[tvdb_idx(grid, ix, iy + 1, iz)];
          if ((c <= 0.0f) != (n <= 0.0f)) ++crossings;
        }
        if (iz + 1 < nz) {
          float n = grid->data[tvdb_idx(grid, ix, iy, iz + 1)];
          if ((c <= 0.0f) != (n <= 0.0f)) ++crossings;
        }
      }
    }
  }
  return (float)crossings * grid->voxel_size * grid->voxel_size;
}

// Volume: integrate inside-region (f < 0). Each voxel contributes voxel_size^3.
float tvdb_volume(const tvdb_dense_grid* grid) {
  if (!grid->data) return 0.0f;
  const size_t nv = (size_t)tvdb_grid_voxels(grid);
  size_t inside = 0;
  for (size_t i = 0; i < nv; ++i) {
    if (grid->data[i] < 0.0f) ++inside;
  }
  const float h = grid->voxel_size;
  return (float)inside * h * h * h;
}

// -------------------------------------------------------------------------
// Phase 2: differential operators
// -------------------------------------------------------------------------

float tvdb_central_diff_x(const tvdb_dense_grid* g, int ix, int iy, int iz) {
  return (tvdb_at(g, ix + 1, iy, iz) - tvdb_at(g, ix - 1, iy, iz)) / (2.0f * g->voxel_size);
}
float tvdb_central_diff_y(const tvdb_dense_grid* g, int ix, int iy, int iz) {
  return (tvdb_at(g, ix, iy + 1, iz) - tvdb_at(g, ix, iy - 1, iz)) / (2.0f * g->voxel_size);
}
float tvdb_central_diff_z(const tvdb_dense_grid* g, int ix, int iy, int iz) {
  return (tvdb_at(g, ix, iy, iz + 1) - tvdb_at(g, ix, iy, iz - 1)) / (2.0f * g->voxel_size);
}

void tvdb_gradient(const tvdb_dense_grid* scalar, tvdb_dense_vec_grid* grad) {
  if (!scalar->data || !grad->data) return;
  const int nx = scalar->nx, ny = scalar->ny, nz = scalar->nz;
  #pragma omp parallel for collapse(2) schedule(static)
  for (int iz = 0; iz < nz; ++iz) {
    for (int iy = 0; iy < ny; ++iy) {
      for (int ix = 0; ix < nx; ++ix) {
        size_t i = tvdb_idx(scalar, ix, iy, iz) * 3u;
        grad->data[i + 0] = tvdb_central_diff_x(scalar, ix, iy, iz);
        grad->data[i + 1] = tvdb_central_diff_y(scalar, ix, iy, iz);
        grad->data[i + 2] = tvdb_central_diff_z(scalar, ix, iy, iz);
      }
    }
  }
}

static inline float tvdb_vec_at(const tvdb_dense_vec_grid* v, int ix, int iy, int iz, int c) {
  ix = tvdb_clamp_i(ix, 0, v->nx - 1);
  iy = tvdb_clamp_i(iy, 0, v->ny - 1);
  iz = tvdb_clamp_i(iz, 0, v->nz - 1);
  size_t base = (size_t)((iz * v->ny + iy) * v->nx + ix) * 3u;
  return v->data[base + (size_t)c];
}

void tvdb_divergence(const tvdb_dense_vec_grid* vec, tvdb_dense_grid* div) {
  if (!vec->data || !div->data) return;
  const int nx = vec->nx, ny = vec->ny, nz = vec->nz;
  const float h2 = 2.0f * vec->voxel_size;
  #pragma omp parallel for collapse(2) schedule(static)
  for (int iz = 0; iz < nz; ++iz) {
    for (int iy = 0; iy < ny; ++iy) {
      for (int ix = 0; ix < nx; ++ix) {
        float dvx_dx = (tvdb_vec_at(vec, ix + 1, iy, iz, 0) - tvdb_vec_at(vec, ix - 1, iy, iz, 0)) / h2;
        float dvy_dy = (tvdb_vec_at(vec, ix, iy + 1, iz, 1) - tvdb_vec_at(vec, ix, iy - 1, iz, 1)) / h2;
        float dvz_dz = (tvdb_vec_at(vec, ix, iy, iz + 1, 2) - tvdb_vec_at(vec, ix, iy, iz - 1, 2)) / h2;
        div->data[tvdb_idx(div, ix, iy, iz)] = dvx_dx + dvy_dy + dvz_dz;
      }
    }
  }
}

void tvdb_laplacian(const tvdb_dense_grid* scalar, tvdb_dense_grid* laplacian) {
  if (!scalar->data || !laplacian->data) return;
  const int nx = scalar->nx, ny = scalar->ny, nz = scalar->nz;
  const float h = scalar->voxel_size;
  const float inv_h2 = 1.0f / (h * h);
  #pragma omp parallel for collapse(2) schedule(static)
  for (int iz = 0; iz < nz; ++iz) {
    for (int iy = 0; iy < ny; ++iy) {
      for (int ix = 0; ix < nx; ++ix) {
        float c = scalar->data[tvdb_idx(scalar, ix, iy, iz)];
        float s = tvdb_at(scalar, ix - 1, iy, iz)
                + tvdb_at(scalar, ix + 1, iy, iz)
                + tvdb_at(scalar, ix, iy - 1, iz)
                + tvdb_at(scalar, ix, iy + 1, iz)
                + tvdb_at(scalar, ix, iy, iz - 1)
                + tvdb_at(scalar, ix, iy, iz + 1);
        laplacian->data[tvdb_idx(laplacian, ix, iy, iz)] = (s - 6.0f * c) * inv_h2;
      }
    }
  }
}

void tvdb_curl(const tvdb_dense_vec_grid* vec, tvdb_dense_vec_grid* curl) {
  if (!vec->data || !curl->data) return;
  const int nx = vec->nx, ny = vec->ny, nz = vec->nz;
  const float h2 = 2.0f * vec->voxel_size;
  #pragma omp parallel for collapse(2) schedule(static)
  for (int iz = 0; iz < nz; ++iz) {
    for (int iy = 0; iy < ny; ++iy) {
      for (int ix = 0; ix < nx; ++ix) {
        float dvz_dy = (tvdb_vec_at(vec, ix, iy + 1, iz, 2) - tvdb_vec_at(vec, ix, iy - 1, iz, 2)) / h2;
        float dvy_dz = (tvdb_vec_at(vec, ix, iy, iz + 1, 1) - tvdb_vec_at(vec, ix, iy, iz - 1, 1)) / h2;
        float dvx_dz = (tvdb_vec_at(vec, ix, iy, iz + 1, 0) - tvdb_vec_at(vec, ix, iy, iz - 1, 0)) / h2;
        float dvz_dx = (tvdb_vec_at(vec, ix + 1, iy, iz, 2) - tvdb_vec_at(vec, ix - 1, iy, iz, 2)) / h2;
        float dvy_dx = (tvdb_vec_at(vec, ix + 1, iy, iz, 1) - tvdb_vec_at(vec, ix - 1, iy, iz, 1)) / h2;
        float dvx_dy = (tvdb_vec_at(vec, ix, iy + 1, iz, 0) - tvdb_vec_at(vec, ix, iy - 1, iz, 0)) / h2;
        size_t i = (size_t)((iz * ny + iy) * nx + ix) * 3u;
        curl->data[i + 0] = dvz_dy - dvy_dz;
        curl->data[i + 1] = dvx_dz - dvz_dx;
        curl->data[i + 2] = dvy_dx - dvx_dy;
      }
    }
  }
}

// -------------------------------------------------------------------------
// Vector-grid operators (GridOperators): magnitude, normalize, cpt
// -------------------------------------------------------------------------

void tvdb_magnitude(const tvdb_dense_vec_grid* vec, tvdb_dense_grid* out) {
  if (!vec->data || !out->data) return;
  if (vec->nx != out->nx || vec->ny != out->ny || vec->nz != out->nz) return;
  const size_t nv = (size_t)vec->nx * vec->ny * vec->nz;
  for (size_t i = 0; i < nv; ++i) {
    float x = vec->data[i*3+0], y = vec->data[i*3+1], z = vec->data[i*3+2];
    out->data[i] = sqrtf(x*x + y*y + z*z);
  }
}

void tvdb_normalize_vec(const tvdb_dense_vec_grid* vec, tvdb_dense_vec_grid* out) {
  if (!vec->data || !out->data) return;
  if (vec->nx != out->nx || vec->ny != out->ny || vec->nz != out->nz) return;
  const size_t nv = (size_t)vec->nx * vec->ny * vec->nz;
  for (size_t i = 0; i < nv; ++i) {
    float x = vec->data[i*3+0], y = vec->data[i*3+1], z = vec->data[i*3+2];
    float m = sqrtf(x*x + y*y + z*z);
    if (m > 0.0f) {
      out->data[i*3+0] = x/m; out->data[i*3+1] = y/m; out->data[i*3+2] = z/m;
    } else {
      out->data[i*3+0] = out->data[i*3+1] = out->data[i*3+2] = 0.0f;
    }
  }
}

void tvdb_cpt(const tvdb_dense_grid* sdf, tvdb_dense_vec_grid* out) {
  if (!sdf->data || !out->data) return;
  if (sdf->nx != out->nx || sdf->ny != out->ny || sdf->nz != out->nz) return;
  const int nx = sdf->nx, ny = sdf->ny, nz = sdf->nz;
  const float vs = sdf->voxel_size;
  for (int iz = 0; iz < nz; ++iz)
    for (int iy = 0; iy < ny; ++iy)
      for (int ix = 0; ix < nx; ++ix) {
        size_t i = (size_t)(iz * ny + iy) * nx + ix;
        float px = sdf->ox + ((float)ix + 0.5f) * vs;
        float py = sdf->oy + ((float)iy + 0.5f) * vs;
        float pz = sdf->oz + ((float)iz + 0.5f) * vs;
        float gx = tvdb_central_diff_x(sdf, ix, iy, iz);
        float gy = tvdb_central_diff_y(sdf, ix, iy, iz);
        float gz = tvdb_central_diff_z(sdf, ix, iy, iz);
        float d = sdf->data[i];
        out->data[i*3+0] = px - d*gx;
        out->data[i*3+1] = py - d*gy;
        out->data[i*3+2] = pz - d*gz;
      }
}

// -------------------------------------------------------------------------
// Composite (per-voxel binary ops)
// -------------------------------------------------------------------------

void tvdb_comp_max(const tvdb_dense_grid* a, const tvdb_dense_grid* b, tvdb_dense_grid* result) {
  if (!tvdb_grid_same_shape(a, b) || !tvdb_grid_same_shape(a, result)) return;
  const size_t nv = (size_t)tvdb_grid_voxels(a);
  for (size_t i = 0; i < nv; ++i) { float va = a->data[i], vb = b->data[i]; result->data[i] = va > vb ? va : vb; }
}
void tvdb_comp_min(const tvdb_dense_grid* a, const tvdb_dense_grid* b, tvdb_dense_grid* result) {
  if (!tvdb_grid_same_shape(a, b) || !tvdb_grid_same_shape(a, result)) return;
  const size_t nv = (size_t)tvdb_grid_voxels(a);
  for (size_t i = 0; i < nv; ++i) { float va = a->data[i], vb = b->data[i]; result->data[i] = va < vb ? va : vb; }
}
void tvdb_comp_sum(const tvdb_dense_grid* a, const tvdb_dense_grid* b, tvdb_dense_grid* result) {
  if (!tvdb_grid_same_shape(a, b) || !tvdb_grid_same_shape(a, result)) return;
  const size_t nv = (size_t)tvdb_grid_voxels(a);
  for (size_t i = 0; i < nv; ++i) result->data[i] = a->data[i] + b->data[i];
}
void tvdb_comp_mult(const tvdb_dense_grid* a, const tvdb_dense_grid* b, tvdb_dense_grid* result) {
  if (!tvdb_grid_same_shape(a, b) || !tvdb_grid_same_shape(a, result)) return;
  const size_t nv = (size_t)tvdb_grid_voxels(a);
  for (size_t i = 0; i < nv; ++i) result->data[i] = a->data[i] * b->data[i];
}

// -------------------------------------------------------------------------
// In-place filters: median, mean-curvature flow
// -------------------------------------------------------------------------

static int tvdb_cmp_float(const void* pa, const void* pb) {
  float a = *(const float*)pa, b = *(const float*)pb;
  return a < b ? -1 : (a > b ? 1 : 0);
}

void tvdb_median_filter(tvdb_dense_grid* grid, int radius, int iterations) {
  if (!grid->data || radius < 1 || iterations < 1) return;
  const int nx = grid->nx, ny = grid->ny, nz = grid->nz;
  const size_t nv = (size_t)nx * ny * nz;
  float* tmp = (float*)malloc(nv * sizeof(float));
  int w = 2 * radius + 1;
  float* win = (float*)malloc((size_t)w * w * w * sizeof(float));
  if (!tmp || !win) { free(tmp); free(win); return; }
  for (int it = 0; it < iterations; ++it) {
    memcpy(tmp, grid->data, nv * sizeof(float));
    for (int iz = 0; iz < nz; ++iz)
      for (int iy = 0; iy < ny; ++iy)
        for (int ix = 0; ix < nx; ++ix) {
          int n = 0;
          for (int dz = -radius; dz <= radius; ++dz)
            for (int dy = -radius; dy <= radius; ++dy)
              for (int dx = -radius; dx <= radius; ++dx) {
                int x = ix + dx, y = iy + dy, z = iz + dz;       // clamp to border
                if (x < 0) x = 0; else if (x >= nx) x = nx - 1;
                if (y < 0) y = 0; else if (y >= ny) y = ny - 1;
                if (z < 0) z = 0; else if (z >= nz) z = nz - 1;
                win[n++] = tmp[(size_t)(z * ny + y) * nx + x];
              }
          qsort(win, (size_t)n, sizeof(float), tvdb_cmp_float);
          grid->data[(size_t)(iz * ny + iy) * nx + ix] = win[n / 2];
        }
  }
  free(tmp); free(win);
}

void tvdb_mean_curvature_flow(tvdb_dense_grid* grid, float dt, int iterations) {
  if (!grid->data || iterations < 1) return;
  const int nx = grid->nx, ny = grid->ny, nz = grid->nz;
  if (nx < 3 || ny < 3 || nz < 3) return;
  const size_t nv = (size_t)nx * ny * nz;
  const size_t sl = (size_t)nx * ny;
  const float h = grid->voxel_size, h2 = grid->voxel_size * grid->voxel_size;
  float* tmp = (float*)malloc(nv * sizeof(float));
  if (!tmp) return;
  for (int it = 0; it < iterations; ++it) {
    memcpy(tmp, grid->data, nv * sizeof(float));
    for (int iz = 1; iz < nz - 1; ++iz)
      for (int iy = 1; iy < ny - 1; ++iy)
        for (int ix = 1; ix < nx - 1; ++ix) {
          size_t i = (size_t)(iz * ny + iy) * nx + ix;
          float px = (tmp[i+1] - tmp[i-1]) / (2.0f*h);
          float py = (tmp[i+nx] - tmp[i-nx]) / (2.0f*h);
          float pz = (tmp[i+sl] - tmp[i-sl]) / (2.0f*h);
          float g2 = px*px + py*py + pz*pz;
          if (g2 < 1e-12f) continue;
          float pxx = (tmp[i+1] - 2.0f*tmp[i] + tmp[i-1]) / h2;
          float pyy = (tmp[i+nx] - 2.0f*tmp[i] + tmp[i-nx]) / h2;
          float pzz = (tmp[i+sl] - 2.0f*tmp[i] + tmp[i-sl]) / h2;
          float pxy = (tmp[i+1+nx] - tmp[i-1+nx] - tmp[i+1-nx] + tmp[i-1-nx]) / (4.0f*h2);
          float pyz = (tmp[i+nx+sl] - tmp[i-nx+sl] - tmp[i+nx-sl] + tmp[i-nx-sl]) / (4.0f*h2);
          float pxz = (tmp[i+1+sl] - tmp[i-1+sl] - tmp[i+1-sl] + tmp[i-1-sl]) / (4.0f*h2);
          float num = pxx*(py*py + pz*pz) + pyy*(px*px + pz*pz) + pzz*(px*px + py*py)
                    - 2.0f*(pxy*px*py + pyz*py*pz + pxz*px*pz);
          grid->data[i] = tmp[i] + dt * (num / g2);   // dt * |grad| * kappa
        }
  }
  free(tmp);
}

// -------------------------------------------------------------------------
// Signed flood fill
// -------------------------------------------------------------------------

void tvdb_signed_flood_fill(tvdb_dense_grid* grid, float band_world) {
  if (!grid->data || band_world <= 0.0f) return;
  const int nx = grid->nx, ny = grid->ny, nz = grid->nz;
  const size_t n = (size_t)nx * ny * nz, sl = (size_t)nx * ny;
  const float thresh = band_world - 1e-5f;          // |value| >= thresh => "far"
  uint8_t* vis = (uint8_t*)calloc(n, 1);
  size_t* stack = (size_t*)malloc(n * sizeof(size_t));
  if (!vis || !stack) { free(vis); free(stack); return; }

  // Seed: far voxels on the grid boundary (connected to "infinity" = exterior).
  size_t sp = 0;
  for (int k = 0; k < nz; ++k)
    for (int j = 0; j < ny; ++j)
      for (int i = 0; i < nx; ++i) {
        if (i != 0 && i != nx-1 && j != 0 && j != ny-1 && k != 0 && k != nz-1) continue;
        size_t idx = ((size_t)k * ny + j) * nx + i;
        if (fabsf(grid->data[idx]) >= thresh && !vis[idx]) { vis[idx] = 1; stack[sp++] = idx; }
      }
  // Flood the exterior through far voxels only (the band blocks it).
  while (sp > 0) {
    size_t v = stack[--sp];
    int vi = (int)(v % nx), vj = (int)((v / nx) % ny), vk = (int)(v / sl);
    const int off[6][3] = { {1,0,0},{-1,0,0},{0,1,0},{0,-1,0},{0,0,1},{0,0,-1} };
    for (int t = 0; t < 6; ++t) {
      int ni = vi+off[t][0], nj = vj+off[t][1], nk = vk+off[t][2];
      if (ni<0||ni>=nx||nj<0||nj>=ny||nk<0||nk>=nz) continue;
      size_t nidx = ((size_t)nk * ny + nj) * nx + ni;
      if (fabsf(grid->data[nidx]) >= thresh && !vis[nidx]) { vis[nidx] = 1; stack[sp++] = nidx; }
    }
  }
  // Assign signs: reached far = exterior (+band), unreached far = interior (-band).
  for (size_t i = 0; i < n; ++i)
    if (fabsf(grid->data[i]) >= thresh)
      grid->data[i] = vis[i] ? band_world : -band_world;

  free(vis); free(stack);
}

// -------------------------------------------------------------------------
// Phase 2: trilinear sampler in voxel space (used by advection)
// -------------------------------------------------------------------------

// Sample a dense scalar grid at (vx, vy, vz) given in voxel-index coordinates
// (origin-relative, in units of voxel cells, NOT world-space).
static float tvdb_sample_dense_voxel(const tvdb_dense_grid* g, float vx, float vy, float vz) {
  int ix = (int)floorf(vx), iy = (int)floorf(vy), iz = (int)floorf(vz);
  float fx = vx - (float)ix, fy = vy - (float)iy, fz = vz - (float)iz;
  float c000 = tvdb_at(g, ix,     iy,     iz);
  float c100 = tvdb_at(g, ix + 1, iy,     iz);
  float c010 = tvdb_at(g, ix,     iy + 1, iz);
  float c110 = tvdb_at(g, ix + 1, iy + 1, iz);
  float c001 = tvdb_at(g, ix,     iy,     iz + 1);
  float c101 = tvdb_at(g, ix + 1, iy,     iz + 1);
  float c011 = tvdb_at(g, ix,     iy + 1, iz + 1);
  float c111 = tvdb_at(g, ix + 1, iy + 1, iz + 1);
  float c00 = c000 * (1.0f - fx) + c100 * fx;
  float c10 = c010 * (1.0f - fx) + c110 * fx;
  float c01 = c001 * (1.0f - fx) + c101 * fx;
  float c11 = c011 * (1.0f - fx) + c111 * fx;
  float c0 = c00 * (1.0f - fy) + c10 * fy;
  float c1 = c01 * (1.0f - fy) + c11 * fy;
  return c0 * (1.0f - fz) + c1 * fz;
}

// -------------------------------------------------------------------------
// Phase 2: semi-Lagrangian advection
// -------------------------------------------------------------------------

void tvdb_advect_semi_lagrangian(const tvdb_dense_grid* field,
                                 const tvdb_dense_vec_grid* velocity,
                                 float dt,
                                 tvdb_dense_grid* result) {
  if (!field->data || !velocity->data || !result->data) return;
  if (field->nx != velocity->nx || field->ny != velocity->ny || field->nz != velocity->nz) return;
  if (!tvdb_grid_same_shape(field, result)) return;

  const int nx = field->nx, ny = field->ny, nz = field->nz;
  const float inv_h = 1.0f / field->voxel_size;
  #pragma omp parallel for collapse(2) schedule(static)
  for (int iz = 0; iz < nz; ++iz) {
    for (int iy = 0; iy < ny; ++iy) {
      for (int ix = 0; ix < nx; ++ix) {
        size_t vi = (size_t)((iz * ny + iy) * nx + ix) * 3u;
        float vx = velocity->data[vi + 0];
        float vy = velocity->data[vi + 1];
        float vz = velocity->data[vi + 2];
        // back-trace in voxel coordinates: x_back = x - dt*v / h
        float bx = (float)ix - dt * vx * inv_h;
        float by = (float)iy - dt * vy * inv_h;
        float bz = (float)iz - dt * vz * inv_h;
        result->data[tvdb_idx(result, ix, iy, iz)] =
            tvdb_sample_dense_voxel(field, bx, by, bz);
      }
    }
  }
}

// -------------------------------------------------------------------------
// Higher-order advection (RK1-4, MacCormack, BFECC)
// -------------------------------------------------------------------------

// Trilinear sample of a vector grid at voxel coordinates (clamped at borders).
static void tvdb_sample_vec_voxel(const tvdb_dense_vec_grid* g, float vx, float vy, float vz,
                                  float out[3]) {
  int ix = (int)floorf(vx), iy = (int)floorf(vy), iz = (int)floorf(vz);
  float fx = vx - ix, fy = vy - iy, fz = vz - iz;
  for (int c = 0; c < 3; ++c) {
    float c00 = tvdb_vec_at(g, ix, iy, iz, c) * (1-fx) + tvdb_vec_at(g, ix+1, iy, iz, c) * fx;
    float c10 = tvdb_vec_at(g, ix, iy+1, iz, c) * (1-fx) + tvdb_vec_at(g, ix+1, iy+1, iz, c) * fx;
    float c01 = tvdb_vec_at(g, ix, iy, iz+1, c) * (1-fx) + tvdb_vec_at(g, ix+1, iy, iz+1, c) * fx;
    float c11 = tvdb_vec_at(g, ix, iy+1, iz+1, c) * (1-fx) + tvdb_vec_at(g, ix+1, iy+1, iz+1, c) * fx;
    float c0 = c00 * (1-fy) + c10 * fy, c1 = c01 * (1-fy) + c11 * fy;
    out[c] = c0 * (1-fz) + c1 * fz;
  }
}

// Backtrace a voxel position by `dt` (negative dt = forward trace) under the
// steady velocity field, integrating dx/ds = -v(x)/h with an RK scheme of the
// given order (1..4).
static void tvdb_rk_backtrace(const tvdb_dense_vec_grid* vel, float inv_h, float dt, int order,
                              float x, float y, float z, float* bx, float* by, float* bz) {
  float v[3];
  tvdb_sample_vec_voxel(vel, x, y, z, v);
  float g1x = -v[0]*inv_h, g1y = -v[1]*inv_h, g1z = -v[2]*inv_h;
  if (order <= 1) { *bx = x + dt*g1x; *by = y + dt*g1y; *bz = z + dt*g1z; return; }
  if (order == 2) {  // midpoint
    tvdb_sample_vec_voxel(vel, x + 0.5f*dt*g1x, y + 0.5f*dt*g1y, z + 0.5f*dt*g1z, v);
    *bx = x - dt*v[0]*inv_h; *by = y - dt*v[1]*inv_h; *bz = z - dt*v[2]*inv_h; return;
  }
  if (order == 3) {  // Kutta's third order
    tvdb_sample_vec_voxel(vel, x + 0.5f*dt*g1x, y + 0.5f*dt*g1y, z + 0.5f*dt*g1z, v);
    float g2x = -v[0]*inv_h, g2y = -v[1]*inv_h, g2z = -v[2]*inv_h;
    tvdb_sample_vec_voxel(vel, x - dt*g1x + 2*dt*g2x, y - dt*g1y + 2*dt*g2y, z - dt*g1z + 2*dt*g2z, v);
    float g3x = -v[0]*inv_h, g3y = -v[1]*inv_h, g3z = -v[2]*inv_h;
    *bx = x + dt*(g1x + 4*g2x + g3x)/6.0f;
    *by = y + dt*(g1y + 4*g2y + g3y)/6.0f;
    *bz = z + dt*(g1z + 4*g2z + g3z)/6.0f;
    return;
  }
  // RK4
  tvdb_sample_vec_voxel(vel, x + 0.5f*dt*g1x, y + 0.5f*dt*g1y, z + 0.5f*dt*g1z, v);
  float g2x = -v[0]*inv_h, g2y = -v[1]*inv_h, g2z = -v[2]*inv_h;
  tvdb_sample_vec_voxel(vel, x + 0.5f*dt*g2x, y + 0.5f*dt*g2y, z + 0.5f*dt*g2z, v);
  float g3x = -v[0]*inv_h, g3y = -v[1]*inv_h, g3z = -v[2]*inv_h;
  tvdb_sample_vec_voxel(vel, x + dt*g3x, y + dt*g3y, z + dt*g3z, v);
  float g4x = -v[0]*inv_h, g4y = -v[1]*inv_h, g4z = -v[2]*inv_h;
  *bx = x + dt*(g1x + 2*g2x + 2*g3x + g4x)/6.0f;
  *by = y + dt*(g1y + 2*g2y + 2*g3y + g4y)/6.0f;
  *bz = z + dt*(g1z + 2*g2z + 2*g3z + g4z)/6.0f;
}

// Single semi-Lagrangian pass: result[x] = field(backtrace(x)).
static void tvdb_advect_sl(const tvdb_dense_grid* field, const tvdb_dense_vec_grid* vel,
                           float dt, int order, tvdb_dense_grid* result) {
  const int nx = field->nx, ny = field->ny, nz = field->nz;
  const float inv_h = 1.0f / field->voxel_size;
  for (int iz = 0; iz < nz; ++iz)
    for (int iy = 0; iy < ny; ++iy)
      for (int ix = 0; ix < nx; ++ix) {
        float bx, by, bz;
        tvdb_rk_backtrace(vel, inv_h, dt, order, (float)ix, (float)iy, (float)iz, &bx, &by, &bz);
        result->data[tvdb_idx(result, ix, iy, iz)] = tvdb_sample_dense_voxel(field, bx, by, bz);
      }
}

// Min/max of the trilinear stencil of `field` at voxel coords (for clamping).
static void tvdb_stencil_minmax(const tvdb_dense_grid* g, float vx, float vy, float vz,
                                float* mn, float* mx) {
  int ix = (int)floorf(vx), iy = (int)floorf(vy), iz = (int)floorf(vz);
  *mn = 3.4e38f; *mx = -3.4e38f;
  for (int dz = 0; dz < 2; ++dz)
    for (int dy = 0; dy < 2; ++dy)
      for (int dx = 0; dx < 2; ++dx) {
        float s = tvdb_at(g, ix+dx, iy+dy, iz+dz);
        if (s < *mn) *mn = s;
        if (s > *mx) *mx = s;
      }
}

void tvdb_advect(const tvdb_dense_grid* field, const tvdb_dense_vec_grid* velocity,
                 float dt, int scheme, int clamp, tvdb_dense_grid* result) {
  if (!field->data || !velocity->data || !result->data) return;
  if (field->nx != velocity->nx || field->ny != velocity->ny || field->nz != velocity->nz) return;
  if (!tvdb_grid_same_shape(field, result)) return;

  if (scheme <= TVDB_ADVECT_RK4) {           // pure semi-Lagrangian, RK order = scheme+1
    tvdb_advect_sl(field, velocity, dt, scheme + 1, result);
    return;
  }

  const int nx = field->nx, ny = field->ny, nz = field->nz;
  const size_t n = (size_t)nx * ny * nz;
  const float inv_h = 1.0f / field->voxel_size;
  tvdb_dense_grid phat, pstar;
  tvdb_dense_grid_init(&phat, nx, ny, nz); phat.voxel_size = field->voxel_size;
  tvdb_dense_grid_init(&pstar, nx, ny, nz); pstar.voxel_size = field->voxel_size;
  if (!phat.data || !pstar.data) { tvdb_dense_grid_free(&phat); tvdb_dense_grid_free(&pstar); return; }

  // Forward then backward advect (RK2 internally), giving a 2nd-order estimate
  // of the round-trip error (field - pstar).
  tvdb_advect_sl(field, velocity, dt, 2, &phat);    // phat = A(field)
  tvdb_advect_sl(&phat, velocity, -dt, 2, &pstar);  // pstar = A^-1(phat)

  if (scheme == TVDB_ADVECT_MACCORMACK) {
    for (int iz = 0; iz < nz; ++iz)
      for (int iy = 0; iy < ny; ++iy)
        for (int ix = 0; ix < nx; ++ix) {
          size_t i = tvdb_idx(field, ix, iy, iz);
          float val = phat.data[i] + 0.5f * (field->data[i] - pstar.data[i]);
          if (clamp) {
            float bx, by, bz, mn, mx;
            tvdb_rk_backtrace(velocity, inv_h, dt, 2, (float)ix, (float)iy, (float)iz, &bx, &by, &bz);
            tvdb_stencil_minmax(field, bx, by, bz, &mn, &mx);
            if (val < mn) val = mn; else if (val > mx) val = mx;
          }
          result->data[i] = val;
        }
  } else {  // BFECC: advect the error-corrected field forward.
    tvdb_dense_grid corr;
    tvdb_dense_grid_init(&corr, nx, ny, nz); corr.voxel_size = field->voxel_size;
    if (!corr.data) { tvdb_dense_grid_free(&phat); tvdb_dense_grid_free(&pstar); return; }
    for (size_t i = 0; i < n; ++i)
      corr.data[i] = field->data[i] + 0.5f * (field->data[i] - pstar.data[i]);
    tvdb_advect_sl(&corr, velocity, dt, 2, result);
    if (clamp) {
      for (int iz = 0; iz < nz; ++iz)
        for (int iy = 0; iy < ny; ++iy)
          for (int ix = 0; ix < nx; ++ix) {
            size_t i = tvdb_idx(field, ix, iy, iz);
            float bx, by, bz, mn, mx;
            tvdb_rk_backtrace(velocity, inv_h, dt, 2, (float)ix, (float)iy, (float)iz, &bx, &by, &bz);
            tvdb_stencil_minmax(field, bx, by, bz, &mn, &mx);
            if (result->data[i] < mn) result->data[i] = mn;
            else if (result->data[i] > mx) result->data[i] = mx;
          }
    }
    tvdb_dense_grid_free(&corr);
  }
  tvdb_dense_grid_free(&phat); tvdb_dense_grid_free(&pstar);
}

// -------------------------------------------------------------------------
// Phase 2: Poisson solver via Jacobi-preconditioned Conjugate Gradient
// -------------------------------------------------------------------------

// Solves L x = rhs, where L is the 7-point discrete Laplacian on a uniform
// grid with spacing h, using homogeneous Dirichlet (clamp-to-edge) boundaries.
// The diagonal is -6/h^2; off-diagonals are +1/h^2 each.
//
// Convergence on coarse uniform grids with this stencil is generally good;
// use Jacobi (diagonal) preconditioning for a small constant-factor speedup.

static void tvdb_apply_laplacian(const tvdb_dense_grid* x, tvdb_dense_grid* y) {
  // y = L x (7-point, edge-clamped)
  tvdb_laplacian(x, y);
}

static double tvdb_dot(const float* a, const float* b, size_t n) {
#if defined(TINYVDB_SIMD) && defined(__AVX2__) && !defined(TINYVDB_OPENMP_ENABLED)
  // AVX2 reduction (single-threaded). When OpenMP is enabled we keep the
  // OpenMP-reduction path because mixing nested SIMD with omp reduction
  // produces unstable summation order across threads.
  return tvdb_simd_dot_f32(a, b, n);
#else
  double s = 0.0;
  #pragma omp parallel for reduction(+:s) schedule(static)
  for (long long i = 0; i < (long long)n; ++i) s += (double)a[i] * (double)b[i];
  return s;
#endif
}

// fp64 7-point laplacian: out[i] = (sum_6_neighbors - 6*self) / h^2.
// Reads from `in` (fp64 buffer) of shape (nx, ny, nz); writes to `out`.
// Boundary cells use Dirichlet zero (out-of-bounds neighbors treated as 0).
static void apply_laplacian_d(const double* in, double* out,
                              int nx, int ny, int nz, double h) {
  const double inv_h2 = 1.0 / (h * h);
  #pragma omp parallel for collapse(2) schedule(static)
  for (int z = 0; z < nz; ++z) {
    for (int y = 0; y < ny; ++y) {
      for (int x = 0; x < nx; ++x) {
        size_t i = (size_t)((z * ny + y) * nx + x);
        double c = in[i];
        double xn = (x > 0)      ? in[i - 1]                  : 0.0;
        double xp = (x + 1 < nx) ? in[i + 1]                  : 0.0;
        double yn = (y > 0)      ? in[i - nx]                 : 0.0;
        double yp = (y + 1 < ny) ? in[i + nx]                 : 0.0;
        double zn = (z > 0)      ? in[i - (size_t)nx * ny]    : 0.0;
        double zp = (z + 1 < nz) ? in[i + (size_t)nx * ny]    : 0.0;
        out[i] = (xn + xp + yn + yp + zn + zp - 6.0 * c) * inv_h2;
      }
    }
  }
}

static double tvdb_dot_d(const double* a, const double* b, size_t n) {
  double s = 0.0;
  #pragma omp parallel for reduction(+:s) schedule(static)
  for (long long i = 0; i < (long long)n; ++i) s += a[i] * b[i];
  return s;
}

int tvdb_solve_poisson_d(const tvdb_dense_grid* rhs,
                         tvdb_dense_grid* x,
                         int max_iters,
                         double tolerance) {
  if (!rhs->data || !x->data) return 0;
  if (!tvdb_grid_same_shape(rhs, x)) return 0;
  const size_t n = (size_t)tvdb_grid_voxels(rhs);
  if (n == 0) return 0;

  // Allocate fp64 workspace.
  double* xd = (double*)malloc(n * sizeof(double));
  double* rd = (double*)malloc(n * sizeof(double));
  double* pd = (double*)malloc(n * sizeof(double));
  double* Apd = (double*)malloc(n * sizeof(double));
  double* zd = (double*)malloc(n * sizeof(double));
  double* rhsd = (double*)malloc(n * sizeof(double));
  if (!xd || !rd || !pd || !Apd || !zd || !rhsd) {
    free(xd); free(rd); free(pd); free(Apd); free(zd); free(rhsd);
    return 0;
  }
  #pragma omp parallel for schedule(static)
  for (long long i = 0; i < (long long)n; ++i) xd[i]   = (double)x->data[i];
  #pragma omp parallel for schedule(static)
  for (long long i = 0; i < (long long)n; ++i) rhsd[i] = (double)rhs->data[i];

  const int nx = rhs->nx, ny = rhs->ny, nz = rhs->nz;
  const double h = (double)rhs->voxel_size;
  const double Minv = -(h * h) / 6.0;

  // r = rhs - L x
  apply_laplacian_d(xd, Apd, nx, ny, nz, h);
  #pragma omp parallel for schedule(static)
  for (long long i = 0; i < (long long)n; ++i) rd[i] = rhsd[i] - Apd[i];
  #pragma omp parallel for schedule(static)
  for (long long i = 0; i < (long long)n; ++i) zd[i] = Minv * rd[i];
  memcpy(pd, zd, n * sizeof(double));

  double rz = tvdb_dot_d(rd, zd, n);
  double r0 = sqrt(tvdb_dot_d(rd, rd, n));
  if (r0 == 0.0) {
    for (size_t i = 0; i < n; ++i) x->data[i] = (float)xd[i];
    free(xd); free(rd); free(pd); free(Apd); free(zd); free(rhsd);
    return 0;
  }
  const double tol2 = tolerance * tolerance * r0 * r0;

  int it = 0;
  for (it = 0; it < max_iters; ++it) {
    apply_laplacian_d(pd, Apd, nx, ny, nz, h);
    double pAp = tvdb_dot_d(pd, Apd, n);
    if (pAp == 0.0) break;
    double alpha = rz / pAp;
    #pragma omp parallel for schedule(static)
    for (long long i = 0; i < (long long)n; ++i) xd[i] += alpha * pd[i];
    #pragma omp parallel for schedule(static)
    for (long long i = 0; i < (long long)n; ++i) rd[i] -= alpha * Apd[i];

    double rr = tvdb_dot_d(rd, rd, n);
    if (rr < tol2) { ++it; break; }

    #pragma omp parallel for schedule(static)
    for (long long i = 0; i < (long long)n; ++i) zd[i] = Minv * rd[i];
    double rz_new = tvdb_dot_d(rd, zd, n);
    double beta = rz_new / rz;
    #pragma omp parallel for schedule(static)
    for (long long i = 0; i < (long long)n; ++i) pd[i] = zd[i] + beta * pd[i];
    rz = rz_new;
  }

  #pragma omp parallel for schedule(static)
  for (long long i = 0; i < (long long)n; ++i) x->data[i] = (float)xd[i];
  free(xd); free(rd); free(pd); free(Apd); free(zd); free(rhsd);
  return it;
}

int tvdb_solve_poisson(const tvdb_dense_grid* rhs,
                       tvdb_dense_grid* x,
                       int max_iters,
                       float tolerance) {
  if (!rhs->data || !x->data) return 0;
  if (!tvdb_grid_same_shape(rhs, x)) return 0;

  const size_t n = (size_t)tvdb_grid_voxels(rhs);
  if (n == 0) return 0;

  // workspace
  float* r = (float*)malloc(n * sizeof(float));
  float* p = (float*)malloc(n * sizeof(float));
  float* Ap = (float*)malloc(n * sizeof(float));
  float* z = (float*)malloc(n * sizeof(float));
  if (!r || !p || !Ap || !z) {
    free(r); free(p); free(Ap); free(z);
    return 0;
  }

  tvdb_dense_grid xg = *x;
  tvdb_dense_grid Apg = *x; Apg.data = Ap;
  tvdb_dense_grid pg  = *x; pg.data  = p;

  // r = rhs - L x
  tvdb_apply_laplacian(&xg, &Apg);
  #pragma omp parallel for schedule(static)
  for (long long i = 0; i < (long long)n; ++i) r[i] = rhs->data[i] - Ap[i];

  // M^-1 = 1 / diag(L) = -h^2 / 6
  const float h = rhs->voxel_size;
  const float Minv = -(h * h) / 6.0f;

  #pragma omp parallel for schedule(static)
  for (long long i = 0; i < (long long)n; ++i) z[i] = Minv * r[i];
  memcpy(p, z, n * sizeof(float));

  double rz = tvdb_dot(r, z, n);
  double r0 = sqrt(tvdb_dot(r, r, n));
  if (r0 == 0.0) {
    free(r); free(p); free(Ap); free(z);
    return 0;
  }
  const double tol2 = (double)tolerance * (double)tolerance * r0 * r0;

  int it = 0;
  for (it = 0; it < max_iters; ++it) {
    tvdb_apply_laplacian(&pg, &Apg);  // Ap = L p
    double pAp = tvdb_dot(p, Ap, n);
    if (pAp == 0.0) break;
    double alpha = rz / pAp;
    #pragma omp parallel for schedule(static)
    for (long long i = 0; i < (long long)n; ++i) x->data[i] += (float)alpha * p[i];
    #pragma omp parallel for schedule(static)
    for (long long i = 0; i < (long long)n; ++i) r[i]      -= (float)alpha * Ap[i];

    double rr = tvdb_dot(r, r, n);
    if (rr < tol2) { ++it; break; }

    #pragma omp parallel for schedule(static)
    for (long long i = 0; i < (long long)n; ++i) z[i] = Minv * r[i];
    double rz_new = tvdb_dot(r, z, n);
    double beta = rz_new / rz;
    #pragma omp parallel for schedule(static)
    for (long long i = 0; i < (long long)n; ++i) p[i] = z[i] + (float)beta * p[i];
    rz = rz_new;
  }

  free(r); free(p); free(Ap); free(z);
  return it;
}

// ---- Fast Sweeping (3D Eikonal solver, Zhao 2005) ----
//
// Solves (D^+x phi)^2 + (D^-x phi)^2 + ... = h^2 with upwind selection.
// Per-voxel: pick min(|x-|, |x+|), min(|y-|, |y+|), min(|z-|, |z+|), sort
// ascending as a,b,c. Try 1-D update, then 2-D, then 3-D solution; choose the
// smallest consistent root (first that satisfies the upwind condition).

static inline float tvdb__min2f(float a, float b) { return a < b ? a : b; }

static float tvdb__godunov_solve(float a, float b, float c, float h) {
    // a <= b <= c (sorted, all non-negative finite). h is voxel size.
    // 1D: x = a + h
    float x = a + h;
    if (x <= b) return x;
    // 2D: solve (x-a)^2 + (x-b)^2 = h^2  =>  2x^2 - 2(a+b)x + a^2+b^2-h^2 = 0
    float ab = a + b;
    float disc = 2.0f * h * h - (a - b) * (a - b);
    if (disc < 0.0f) return x;  // shouldn't happen if a<=b
    x = 0.5f * (ab + sqrtf(disc));
    if (x <= c) return x;
    // 3D: solve (x-a)^2 + (x-b)^2 + (x-c)^2 = h^2
    float abc = a + b + c;
    float sumsq = a * a + b * b + c * c;
    float disc3 = abc * abc - 3.0f * (sumsq - h * h);
    if (disc3 < 0.0f) return x;
    x = (abc + sqrtf(disc3)) / 3.0f;
    return x;
}

int tvdb_fast_sweeping(tvdb_dense_grid* grid, float frozen_band,
                       int max_iters, float tol) {
    if (!grid || !grid->data) return 0;
    if (max_iters <= 0) max_iters = 1;
    if (frozen_band < 0.0f) frozen_band = 0.0f;
    const int nx = grid->nx, ny = grid->ny, nz = grid->nz;
    const float h = grid->voxel_size > 0.0f ? grid->voxel_size : 1.0f;
    const size_t N = (size_t)nx * ny * nz;

    // Capture sign from the input field; voxels at exactly 0 are treated as
    // positive half-space.
    uint8_t* sign_pos = (uint8_t*)malloc(N);
    uint8_t* frozen   = (uint8_t*)malloc(N);
    float*   absphi   = (float*)malloc(N * sizeof(float));
    if (!sign_pos || !frozen || !absphi) {
        free(sign_pos); free(frozen); free(absphi); return 0;
    }
    const float HUGE_VAL_F = 1e30f;
    for (size_t i = 0; i < N; ++i) {
        float v = grid->data[i];
        sign_pos[i] = (v >= 0.0f) ? 1u : 0u;
        float av = fabsf(v);
        if (av <= frozen_band) {
            frozen[i] = 1u;
            absphi[i] = av;
        } else {
            frozen[i] = 0u;
            absphi[i] = HUGE_VAL_F;
        }
    }

    // 8 sweep directions over (x,y,z) ranges.
    const int dirs[8][3] = {
        { 1, 1, 1}, {-1, 1, 1}, { 1,-1, 1}, {-1,-1, 1},
        { 1, 1,-1}, {-1, 1,-1}, { 1,-1,-1}, {-1,-1,-1}
    };

    int iter = 0;
    for (; iter < max_iters; ++iter) {
        float max_change = 0.0f;
        for (int d = 0; d < 8; ++d) {
            int sx = dirs[d][0], sy = dirs[d][1], sz = dirs[d][2];
            int x0 = (sx > 0) ? 0 : nx - 1, x1 = (sx > 0) ? nx : -1;
            int y0 = (sy > 0) ? 0 : ny - 1, y1 = (sy > 0) ? ny : -1;
            int z0 = (sz > 0) ? 0 : nz - 1, z1 = (sz > 0) ? nz : -1;
            for (int z = z0; z != z1; z += sz) {
                for (int y = y0; y != y1; y += sy) {
                    for (int x = x0; x != x1; x += sx) {
                        size_t idx = tvdb_idx(grid, x, y, z);
                        if (frozen[idx]) continue;
                        // Upwind neighbor min on each axis.
                        float ax = HUGE_VAL_F;
                        if (x > 0)        ax = tvdb__min2f(ax, absphi[tvdb_idx(grid, x - 1, y, z)]);
                        if (x + 1 < nx)   ax = tvdb__min2f(ax, absphi[tvdb_idx(grid, x + 1, y, z)]);
                        float ay = HUGE_VAL_F;
                        if (y > 0)        ay = tvdb__min2f(ay, absphi[tvdb_idx(grid, x, y - 1, z)]);
                        if (y + 1 < ny)   ay = tvdb__min2f(ay, absphi[tvdb_idx(grid, x, y + 1, z)]);
                        float az = HUGE_VAL_F;
                        if (z > 0)        az = tvdb__min2f(az, absphi[tvdb_idx(grid, x, y, z - 1)]);
                        if (z + 1 < nz)   az = tvdb__min2f(az, absphi[tvdb_idx(grid, x, y, z + 1)]);
                        // Sort a<=b<=c.
                        float a = ax, b = ay, c = az;
                        if (a > b) { float t = a; a = b; b = t; }
                        if (b > c) { float t = b; b = c; c = t; }
                        if (a > b) { float t = a; a = b; b = t; }
                        if (a >= HUGE_VAL_F * 0.5f) continue;
                        float new_v = tvdb__godunov_solve(a, b, c, h);
                        if (new_v < absphi[idx]) {
                            float ch = absphi[idx] - new_v;
                            if (ch > max_change) max_change = ch;
                            absphi[idx] = new_v;
                        }
                    }
                }
            }
        }
        if (max_change <= tol) { ++iter; break; }
    }

    // Write back signed values.
    for (size_t i = 0; i < N; ++i) {
        if (frozen[i]) continue;
        float a = absphi[i];
        grid->data[i] = sign_pos[i] ? a : -a;
    }
    free(sign_pos); free(frozen); free(absphi);
    return iter;
}

// =============================================================================
// fp64 dense grid: lifecycle, conversion, and ops parallel to the fp32 path.
// =============================================================================

void tvdb_dense_grid_d_init(tvdb_dense_grid_d* g, int nx, int ny, int nz) {
  g->nx = nx; g->ny = ny; g->nz = nz;
  g->voxel_size = 1.0;
  g->ox = g->oy = g->oz = 0.0;
  size_t n = (size_t)nx * (size_t)ny * (size_t)nz;
  g->data = (double*)calloc(n, sizeof(double));
}

void tvdb_dense_grid_d_free(tvdb_dense_grid_d* g) {
  if (!g) return;
  free(g->data); g->data = NULL;
  g->nx = g->ny = g->nz = 0;
}

void tvdb_dense_grid_f_to_d(const tvdb_dense_grid* in, tvdb_dense_grid_d* out) {
  tvdb_dense_grid_d_init(out, in->nx, in->ny, in->nz);
  out->voxel_size = (double)in->voxel_size;
  out->ox = (double)in->ox; out->oy = (double)in->oy; out->oz = (double)in->oz;
  size_t n = (size_t)in->nx * in->ny * in->nz;
  for (size_t i = 0; i < n; ++i) out->data[i] = (double)in->data[i];
}

void tvdb_dense_grid_d_to_f(const tvdb_dense_grid_d* in, tvdb_dense_grid* out) {
  tvdb_dense_grid_init(out, in->nx, in->ny, in->nz);
  out->voxel_size = (float)in->voxel_size;
  out->ox = (float)in->ox; out->oy = (float)in->oy; out->oz = (float)in->oz;
  size_t n = (size_t)in->nx * in->ny * in->nz;
  for (size_t i = 0; i < n; ++i) out->data[i] = (float)in->data[i];
}

double tvdb_sample_trilinear_dense_d(const tvdb_dense_grid_d* g,
                                     double wx, double wy, double wz) {
  if (!g->data) return 0.0;
  // Cell-center convention: voxel `i` stores its sample at world position
  // `ox + (i + 0.5) * vs`. Same as the fp32 sampler.
  double vx = (wx - g->ox) / g->voxel_size - 0.5;
  double vy = (wy - g->oy) / g->voxel_size - 0.5;
  double vz = (wz - g->oz) / g->voxel_size - 0.5;
  int ix = (int)floor(vx), iy = (int)floor(vy), iz = (int)floor(vz);
  double fx = vx - (double)ix, fy = vy - (double)iy, fz = vz - (double)iz;

  double c000 = tvdb_at_d(g, ix,     iy,     iz);
  double c100 = tvdb_at_d(g, ix + 1, iy,     iz);
  double c010 = tvdb_at_d(g, ix,     iy + 1, iz);
  double c110 = tvdb_at_d(g, ix + 1, iy + 1, iz);
  double c001 = tvdb_at_d(g, ix,     iy,     iz + 1);
  double c101 = tvdb_at_d(g, ix + 1, iy,     iz + 1);
  double c011 = tvdb_at_d(g, ix,     iy + 1, iz + 1);
  double c111 = tvdb_at_d(g, ix + 1, iy + 1, iz + 1);
  double c00 = c000 * (1.0 - fx) + c100 * fx;
  double c10 = c010 * (1.0 - fx) + c110 * fx;
  double c01 = c001 * (1.0 - fx) + c101 * fx;
  double c11 = c011 * (1.0 - fx) + c111 * fx;
  double c0 = c00 * (1.0 - fy) + c10 * fy;
  double c1 = c01 * (1.0 - fy) + c11 * fy;
  return c0 * (1.0 - fz) + c1 * fz;
}

void tvdb_laplacian_d(const tvdb_dense_grid_d* g, tvdb_dense_grid_d* out) {
  // 7-point Laplacian with edge clamp. h^2 normalization.
  double inv_h2 = 1.0 / (g->voxel_size * g->voxel_size);
  #pragma omp parallel for collapse(2) schedule(static)
  for (int z = 0; z < g->nz; ++z) {
    for (int y = 0; y < g->ny; ++y) {
      for (int x = 0; x < g->nx; ++x) {
        double c = g->data[tvdb_idx_d(g, x, y, z)];
        double s = tvdb_at_d(g, x - 1, y,     z)
                 + tvdb_at_d(g, x + 1, y,     z)
                 + tvdb_at_d(g, x,     y - 1, z)
                 + tvdb_at_d(g, x,     y + 1, z)
                 + tvdb_at_d(g, x,     y,     z - 1)
                 + tvdb_at_d(g, x,     y,     z + 1);
        out->data[tvdb_idx_d(out, x, y, z)] = (s - 6.0 * c) * inv_h2;
      }
    }
  }
}

static inline double dmin(double a, double b) { return a < b ? a : b; }
static inline double dmax(double a, double b) { return a > b ? a : b; }

void tvdb_csg_union_d(const tvdb_dense_grid_d* a, const tvdb_dense_grid_d* b,
                      tvdb_dense_grid_d* out) {
  size_t n = (size_t)out->nx * out->ny * out->nz;
  #pragma omp parallel for schedule(static)
  for (long long i = 0; i < (long long)n; ++i) {
    out->data[i] = dmin(a->data[i], b->data[i]);
  }
}

void tvdb_csg_intersection_d(const tvdb_dense_grid_d* a, const tvdb_dense_grid_d* b,
                             tvdb_dense_grid_d* out) {
  size_t n = (size_t)out->nx * out->ny * out->nz;
  #pragma omp parallel for schedule(static)
  for (long long i = 0; i < (long long)n; ++i) {
    out->data[i] = dmax(a->data[i], b->data[i]);
  }
}

void tvdb_csg_difference_d(const tvdb_dense_grid_d* a, const tvdb_dense_grid_d* b,
                           tvdb_dense_grid_d* out) {
  size_t n = (size_t)out->nx * out->ny * out->nz;
  #pragma omp parallel for schedule(static)
  for (long long i = 0; i < (long long)n; ++i) {
    out->data[i] = dmax(a->data[i], -b->data[i]);
  }
}

double tvdb_volume_d(const tvdb_dense_grid_d* g) {
  // Sum of voxel cells whose value is < 0.
  double cell = g->voxel_size * g->voxel_size * g->voxel_size;
  double vol = 0.0;
  size_t n = (size_t)g->nx * g->ny * g->nz;
  #pragma omp parallel for reduction(+:vol) schedule(static)
  for (long long i = 0; i < (long long)n; ++i) {
    if (g->data[i] < 0.0) vol += cell;
  }
  return vol;
}

double tvdb_surface_area_d(const tvdb_dense_grid_d* g) {
  // Count zero-crossings over 6-neighbor edges; weight by voxel_size^2.
  double face = g->voxel_size * g->voxel_size;
  double area = 0.0;
  #pragma omp parallel for collapse(2) reduction(+:area) schedule(static)
  for (int z = 0; z < g->nz; ++z) {
    for (int y = 0; y < g->ny; ++y) {
      for (int x = 0; x < g->nx; ++x) {
        double c = g->data[tvdb_idx_d(g, x, y, z)];
        if (x + 1 < g->nx) {
          double n2 = g->data[tvdb_idx_d(g, x + 1, y, z)];
          if ((c < 0.0) != (n2 < 0.0)) area += face;
        }
        if (y + 1 < g->ny) {
          double n2 = g->data[tvdb_idx_d(g, x, y + 1, z)];
          if ((c < 0.0) != (n2 < 0.0)) area += face;
        }
        if (z + 1 < g->nz) {
          double n2 = g->data[tvdb_idx_d(g, x, y, z + 1)];
          if ((c < 0.0) != (n2 < 0.0)) area += face;
        }
      }
    }
  }
  return area;
}

// FastSweeping: fp64 8-direction Eikonal solver. Logic mirrors the fp32
// path but accumulates everything in double.

static inline double tvdb__godunov_solve_d(double a, double b, double c, double h) {
  double x = a + h;
  if (x <= b) return x;
  double ab = a + b;
  double disc = 2.0 * h * h - (a - b) * (a - b);
  if (disc < 0.0) return x;
  x = 0.5 * (ab + sqrt(disc));
  if (x <= c) return x;
  double abc = a + b + c;
  double sumsq = a * a + b * b + c * c;
  double disc3 = abc * abc - 3.0 * (sumsq - h * h);
  if (disc3 < 0.0) return x;
  return (abc + sqrt(disc3)) / 3.0;
}

int tvdb_fast_sweeping_d(tvdb_dense_grid_d* grid, double frozen_band,
                         int max_iters, double tol) {
  if (!grid || !grid->data) return 0;
  if (max_iters <= 0) max_iters = 1;
  if (frozen_band < 0.0) frozen_band = 0.0;
  const int nx = grid->nx, ny = grid->ny, nz = grid->nz;
  const double h = grid->voxel_size > 0.0 ? grid->voxel_size : 1.0;
  const size_t N = (size_t)nx * ny * nz;
  uint8_t* sign_pos = (uint8_t*)malloc(N);
  uint8_t* frozen   = (uint8_t*)malloc(N);
  double*  absphi   = (double*)malloc(N * sizeof(double));
  if (!sign_pos || !frozen || !absphi) {
    free(sign_pos); free(frozen); free(absphi); return 0;
  }
  const double HUGE_D = 1e30;
  for (size_t i = 0; i < N; ++i) {
    double v = grid->data[i];
    sign_pos[i] = (v >= 0.0) ? 1u : 0u;
    double av = fabs(v);
    if (av <= frozen_band) { frozen[i] = 1u; absphi[i] = av; }
    else { frozen[i] = 0u; absphi[i] = HUGE_D; }
  }
  const int dirs[8][3] = {
    { 1, 1, 1}, {-1, 1, 1}, { 1,-1, 1}, {-1,-1, 1},
    { 1, 1,-1}, {-1, 1,-1}, { 1,-1,-1}, {-1,-1,-1}
  };
  int iter = 0;
  for (; iter < max_iters; ++iter) {
    double max_change = 0.0;
    for (int d = 0; d < 8; ++d) {
      int sx = dirs[d][0], sy = dirs[d][1], sz = dirs[d][2];
      int x0 = (sx > 0) ? 0 : nx - 1, x1 = (sx > 0) ? nx : -1;
      int y0 = (sy > 0) ? 0 : ny - 1, y1 = (sy > 0) ? ny : -1;
      int z0 = (sz > 0) ? 0 : nz - 1, z1 = (sz > 0) ? nz : -1;
      for (int z = z0; z != z1; z += sz) {
        for (int y = y0; y != y1; y += sy) {
          for (int x = x0; x != x1; x += sx) {
            size_t idx = tvdb_idx_d(grid, x, y, z);
            if (frozen[idx]) continue;
            double ax = HUGE_D;
            if (x > 0)        ax = dmin(ax, absphi[tvdb_idx_d(grid, x - 1, y, z)]);
            if (x + 1 < nx)   ax = dmin(ax, absphi[tvdb_idx_d(grid, x + 1, y, z)]);
            double ay = HUGE_D;
            if (y > 0)        ay = dmin(ay, absphi[tvdb_idx_d(grid, x, y - 1, z)]);
            if (y + 1 < ny)   ay = dmin(ay, absphi[tvdb_idx_d(grid, x, y + 1, z)]);
            double az = HUGE_D;
            if (z > 0)        az = dmin(az, absphi[tvdb_idx_d(grid, x, y, z - 1)]);
            if (z + 1 < nz)   az = dmin(az, absphi[tvdb_idx_d(grid, x, y, z + 1)]);
            double a = ax, b = ay, c = az;
            if (a > b) { double t = a; a = b; b = t; }
            if (b > c) { double t = b; b = c; c = t; }
            if (a > b) { double t = a; a = b; b = t; }
            if (a >= HUGE_D * 0.5) continue;
            double new_v = tvdb__godunov_solve_d(a, b, c, h);
            if (new_v < absphi[idx]) {
              double ch = absphi[idx] - new_v;
              if (ch > max_change) max_change = ch;
              absphi[idx] = new_v;
            }
          }
        }
      }
    }
    if (max_change <= tol) { ++iter; break; }
  }
  for (size_t i = 0; i < N; ++i) {
    if (frozen[i]) continue;
    grid->data[i] = sign_pos[i] ? absphi[i] : -absphi[i];
  }
  free(sign_pos); free(frozen); free(absphi);
  return iter;
}

// fp64 Poisson (PCG with Jacobi preconditioner; identical structure to the
// fp32 path but doubles throughout).
static inline double tvdb__lap_apply_d(const tvdb_dense_grid_d* g, int x, int y, int z) {
  double c = g->data[tvdb_idx_d(g, x, y, z)];
  double s = tvdb_at_d(g, x - 1, y, z) + tvdb_at_d(g, x + 1, y, z)
           + tvdb_at_d(g, x, y - 1, z) + tvdb_at_d(g, x, y + 1, z)
           + tvdb_at_d(g, x, y, z - 1) + tvdb_at_d(g, x, y, z + 1);
  return s - 6.0 * c;
}

int tvdb_solve_poisson_dd(const tvdb_dense_grid_d* rhs, tvdb_dense_grid_d* x,
                          int max_iters, double tolerance) {
  if (!rhs || !x || !rhs->data || !x->data) return 0;
  if (rhs->nx != x->nx || rhs->ny != x->ny || rhs->nz != x->nz) return 0;
  size_t n = (size_t)rhs->nx * rhs->ny * rhs->nz;
  double inv_h2 = 1.0 / (x->voxel_size * x->voxel_size);

  double* r  = (double*)malloc(n * sizeof(double));
  double* p  = (double*)malloc(n * sizeof(double));
  double* Ap = (double*)malloc(n * sizeof(double));
  double* z  = (double*)malloc(n * sizeof(double));
  if (!r || !p || !Ap || !z) {
    free(r); free(p); free(Ap); free(z); return 0;
  }

  // Working grid for x with its current values; A*x uses x->data directly.
  // r = b - A*x
  #pragma omp parallel for collapse(2) schedule(static)
  for (int zi = 0; zi < rhs->nz; ++zi) {
    for (int yi = 0; yi < rhs->ny; ++yi) {
      for (int xi = 0; xi < rhs->nx; ++xi) {
        size_t i = tvdb_idx_d(rhs, xi, yi, zi);
        r[i] = rhs->data[i] - inv_h2 * tvdb__lap_apply_d(x, xi, yi, zi);
      }
    }
  }
  // Jacobi preconditioner: M = -6/h^2  =>  z = r / (-6/h^2) = -r * h^2 / 6
  double inv_diag = -1.0 / (6.0 * inv_h2);
  #pragma omp parallel for schedule(static)
  for (long long i = 0; i < (long long)n; ++i) { z[i] = r[i] * inv_diag; p[i] = z[i]; }

  double rz = 0.0;
  #pragma omp parallel for reduction(+:rz) schedule(static)
  for (long long i = 0; i < (long long)n; ++i) rz += r[i] * z[i];
  double tol2 = tolerance * tolerance;
  int it = 0;
  for (; it < max_iters; ++it) {
    // Ap = A*p  (apply Laplacian over `p` viewed as grid).
    // Treat p as a temporary dense_grid_d with the same shape/origin as x.
    tvdb_dense_grid_d pgrid;
    pgrid.nx = x->nx; pgrid.ny = x->ny; pgrid.nz = x->nz;
    pgrid.voxel_size = x->voxel_size;
    pgrid.ox = x->ox; pgrid.oy = x->oy; pgrid.oz = x->oz;
    pgrid.data = p;
    #pragma omp parallel for collapse(2) schedule(static)
    for (int zi = 0; zi < pgrid.nz; ++zi) {
      for (int yi = 0; yi < pgrid.ny; ++yi) {
        for (int xi = 0; xi < pgrid.nx; ++xi) {
          size_t i = tvdb_idx_d(&pgrid, xi, yi, zi);
          Ap[i] = inv_h2 * tvdb__lap_apply_d(&pgrid, xi, yi, zi);
        }
      }
    }
    double pAp = 0.0;
    #pragma omp parallel for reduction(+:pAp) schedule(static)
    for (long long i = 0; i < (long long)n; ++i) pAp += p[i] * Ap[i];
    if (pAp == 0.0) break;
    double alpha = rz / pAp;
    #pragma omp parallel for schedule(static)
    for (long long i = 0; i < (long long)n; ++i) {
      x->data[i] += alpha * p[i];
      r[i]       -= alpha * Ap[i];
    }
    double rr = 0.0;
    #pragma omp parallel for reduction(+:rr) schedule(static)
    for (long long i = 0; i < (long long)n; ++i) rr += r[i] * r[i];
    if (rr <= tol2) { ++it; break; }
    #pragma omp parallel for schedule(static)
    for (long long i = 0; i < (long long)n; ++i) z[i] = r[i] * inv_diag;
    double rz_new = 0.0;
    #pragma omp parallel for reduction(+:rz_new) schedule(static)
    for (long long i = 0; i < (long long)n; ++i) rz_new += r[i] * z[i];
    double beta = rz_new / rz;
    rz = rz_new;
    #pragma omp parallel for schedule(static)
    for (long long i = 0; i < (long long)n; ++i) p[i] = z[i] + beta * p[i];
  }
  free(r); free(p); free(Ap); free(z);
  return it;
}
