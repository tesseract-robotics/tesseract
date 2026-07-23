#include "tinyvdb_ray.h"
#include "tinyvdb_sample.h"

#include <math.h>
#include <stddef.h>

// Ray-AABB slab test in voxel-index coordinates. Returns whether the ray
// intersects the AABB [0,nx)x[0,ny)x[0,nz), and if so writes the entry/exit
// t values clamped to [tmin, tmax].
static bool tvdb_ray_voxel_aabb(const tvdb_ray* ray, const tvdb_dense_grid* g,
                                float* t_enter, float* t_exit) {
  // Convert ray origin and dir to voxel-index space.
  float vs = g->voxel_size;
  float ox = (ray->origin.x - g->ox) / vs;
  float oy = (ray->origin.y - g->oy) / vs;
  float oz = (ray->origin.z - g->oz) / vs;
  float dx = ray->dir.x / vs;
  float dy = ray->dir.y / vs;
  float dz = ray->dir.z / vs;

  float t0 = ray->tmin, t1 = ray->tmax;
  for (int axis = 0; axis < 3; ++axis) {
    float o = (axis == 0) ? ox : (axis == 1) ? oy : oz;
    float d = (axis == 0) ? dx : (axis == 1) ? dy : dz;
    float lo = 0.0f;
    float hi = (float)((axis == 0) ? g->nx : (axis == 1) ? g->ny : g->nz);
    if (fabsf(d) < 1e-30f) {
      if (o < lo || o > hi) return false;
      continue;
    }
    float a = (lo - o) / d;
    float b = (hi - o) / d;
    if (a > b) { float t = a; a = b; b = t; }
    if (a > t0) t0 = a;
    if (b < t1) t1 = b;
    if (t0 > t1) return false;
  }
  *t_enter = t0;
  *t_exit = t1;
  return true;
}

size_t tvdb_voxels_along_ray_dense(const tvdb_dense_grid* g,
                                   const tvdb_ray* ray,
                                   tvdb_vec3i* out_voxels,
                                   size_t cap) {
  if (!g || !g->data || !ray) return 0;
  float t_enter, t_exit;
  if (!tvdb_ray_voxel_aabb(ray, g, &t_enter, &t_exit)) return 0;

  // Amanatides-Woo DDA in voxel-index space.
  float vs = g->voxel_size;
  float ox = (ray->origin.x - g->ox) / vs;
  float oy = (ray->origin.y - g->oy) / vs;
  float oz = (ray->origin.z - g->oz) / vs;
  float dx = ray->dir.x / vs;
  float dy = ray->dir.y / vs;
  float dz = ray->dir.z / vs;

  float ex = ox + t_enter * dx;
  float ey = oy + t_enter * dy;
  float ez = oz + t_enter * dz;

  int ix = (int)floorf(ex);
  int iy = (int)floorf(ey);
  int iz = (int)floorf(ez);
  if (ix == g->nx) ix = g->nx - 1;
  if (iy == g->ny) iy = g->ny - 1;
  if (iz == g->nz) iz = g->nz - 1;

  int sx = (dx > 0) ? 1 : (dx < 0 ? -1 : 0);
  int sy = (dy > 0) ? 1 : (dy < 0 ? -1 : 0);
  int sz = (dz > 0) ? 1 : (dz < 0 ? -1 : 0);

  float t_max_x = (sx != 0) ? (((float)ix + (sx > 0 ? 1.0f : 0.0f)) - ox) / dx : INFINITY;
  float t_max_y = (sy != 0) ? (((float)iy + (sy > 0 ? 1.0f : 0.0f)) - oy) / dy : INFINITY;
  float t_max_z = (sz != 0) ? (((float)iz + (sz > 0 ? 1.0f : 0.0f)) - oz) / dz : INFINITY;
  float t_delta_x = (sx != 0) ? fabsf(1.0f / dx) : INFINITY;
  float t_delta_y = (sy != 0) ? fabsf(1.0f / dy) : INFINITY;
  float t_delta_z = (sz != 0) ? fabsf(1.0f / dz) : INFINITY;

  size_t written = 0;
  size_t total = 0;
  while (ix >= 0 && ix < g->nx && iy >= 0 && iy < g->ny && iz >= 0 && iz < g->nz) {
    if (out_voxels && written < cap) {
      out_voxels[written].x = ix;
      out_voxels[written].y = iy;
      out_voxels[written].z = iz;
      ++written;
    }
    ++total;

    if (t_max_x < t_max_y && t_max_x < t_max_z) {
      ix += sx; t_max_x += t_delta_x;
      if (t_max_x > t_exit && t_max_y > t_exit && t_max_z > t_exit) break;
    } else if (t_max_y < t_max_z) {
      iy += sy; t_max_y += t_delta_y;
      if (t_max_x > t_exit && t_max_y > t_exit && t_max_z > t_exit) break;
    } else {
      iz += sz; t_max_z += t_delta_z;
      if (t_max_x > t_exit && t_max_y > t_exit && t_max_z > t_exit) break;
    }
  }
  return out_voxels ? written : total;
}

void tvdb_uniform_ray_samples(const tvdb_ray* ray,
                              size_t n_samples,
                              tvdb_vec3f* out_points,
                              float* out_t) {
  if (!ray || n_samples == 0) return;
  float lo = ray->tmin, hi = ray->tmax;
  for (size_t i = 0; i < n_samples; ++i) {
    float a = (n_samples == 1) ? 0.0f : (float)i / (float)(n_samples - 1);
    float t = lo + (hi - lo) * a;
    if (out_t) out_t[i] = t;
    if (out_points) {
      out_points[i].x = ray->origin.x + t * ray->dir.x;
      out_points[i].y = ray->origin.y + t * ray->dir.y;
      out_points[i].z = ray->origin.z + t * ray->dir.z;
    }
  }
}

size_t tvdb_segments_along_ray(const tvdb_dense_grid* g,
                               const tvdb_ray* ray,
                               float isovalue,
                               size_t step_count,
                               float* out_t_pairs,
                               size_t cap) {
  if (!g || !g->data || !ray || step_count < 2) return 0;
  size_t pairs = 0;
  bool inside = false;
  float t_enter = 0.0f;
  float t_prev = ray->tmin;
  float v_prev = tvdb_sample_trilinear_dense(g,
      ray->origin.x + t_prev * ray->dir.x,
      ray->origin.y + t_prev * ray->dir.y,
      ray->origin.z + t_prev * ray->dir.z) - isovalue;
  if (v_prev < 0.0f) { inside = true; t_enter = ray->tmin; }

  for (size_t i = 1; i < step_count; ++i) {
    float a = (float)i / (float)(step_count - 1);
    float t = ray->tmin + (ray->tmax - ray->tmin) * a;
    float v = tvdb_sample_trilinear_dense(g,
        ray->origin.x + t * ray->dir.x,
        ray->origin.y + t * ray->dir.y,
        ray->origin.z + t * ray->dir.z) - isovalue;

    if (v_prev * v < 0.0f) {
      // Linear-interp the crossing
      float frac = v_prev / (v_prev - v);
      float t_cross = t_prev + frac * (t - t_prev);
      if (!inside) {
        t_enter = t_cross;
        inside = true;
      } else {
        if (out_t_pairs && pairs < cap) {
          out_t_pairs[2 * pairs + 0] = t_enter;
          out_t_pairs[2 * pairs + 1] = t_cross;
        }
        ++pairs;
        inside = false;
      }
    }
    v_prev = v; t_prev = t;
  }
  if (inside) {
    if (out_t_pairs && pairs < cap) {
      out_t_pairs[2 * pairs + 0] = t_enter;
      out_t_pairs[2 * pairs + 1] = ray->tmax;
    }
    ++pairs;
  }
  return pairs;
}

bool tvdb_marching_cubes_batch(const tvdb_dense_grid* grids,
                               size_t n_grids,
                               float isovalue,
                               tvdb_triangle_mesh* meshes,
                               tvdb_arena_allocator_t* arena) {
  if (!grids || !meshes) return false;
  bool all_ok = true;
  for (size_t i = 0; i < n_grids; ++i) {
    if (arena) {
      tvdb_triangle_mesh_init_arena(&meshes[i], arena);
    } else {
      tvdb_triangle_mesh_init(&meshes[i]);
    }
    bool ok = tvdb_sdf_to_mesh(&grids[i], isovalue, &meshes[i], arena);
    if (!ok) all_ok = false;
  }
  return all_ok;
}
