// Minimal emission-absorption volume renderer. See tinyvdb_render.h.

#include "tinyvdb_render.h"
#include "tinyvdb_sample.h"  // tvdb_sample_trilinear_dense

#include <math.h>

static void v_sub(const float a[3], const float b[3], float o[3]) {
  o[0]=a[0]-b[0]; o[1]=a[1]-b[1]; o[2]=a[2]-b[2];
}
static void v_cross(const float a[3], const float b[3], float o[3]) {
  o[0]=a[1]*b[2]-a[2]*b[1]; o[1]=a[2]*b[0]-a[0]*b[2]; o[2]=a[0]*b[1]-a[1]*b[0];
}
static void v_norm(float a[3]) {
  float l = sqrtf(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]);
  if (l > 0.0f) { a[0]/=l; a[1]/=l; a[2]/=l; }
}

// Ray-AABB slab intersection. Returns false if no hit; else [t0,t1] (t1>=t0>=0
// clamped to the forward ray).
static bool ray_aabb(const float o[3], const float d[3],
                     const float lo[3], const float hi[3], float* t0, float* t1) {
  float tmin = 0.0f, tmax = 1e30f;
  for (int a = 0; a < 3; ++a) {
    if (fabsf(d[a]) < 1e-12f) {
      if (o[a] < lo[a] || o[a] > hi[a]) return false;
    } else {
      float inv = 1.0f / d[a];
      float ta = (lo[a] - o[a]) * inv, tb = (hi[a] - o[a]) * inv;
      if (ta > tb) { float t = ta; ta = tb; tb = t; }
      if (ta > tmin) tmin = ta;
      if (tb < tmax) tmax = tb;
      if (tmin > tmax) return false;
    }
  }
  *t0 = tmin; *t1 = tmax;
  return true;
}

bool tvdb_volume_render(const tvdb_dense_grid* density,
                        const float eye[3], const float center[3], const float up[3],
                        float fov_y, int width, int height,
                        float sigma, float step, float background,
                        float* out_image) {
  if (!density || !density->data || !out_image || width < 1 || height < 1 ||
      step <= 0.0f || fov_y <= 0.0f)
    return false;

  // Camera basis. If `up` is (nearly) parallel to the view direction, the cross
  // product collapses — fall back to an alternate up axis.
  float fwd[3]; v_sub(center, eye, fwd); v_norm(fwd);
  float right[3]; v_cross(fwd, up, right);
  if (right[0]*right[0] + right[1]*right[1] + right[2]*right[2] < 1e-12f) {
    float alt[3] = { 1.0f, 0.0f, 0.0f };
    if (fabsf(fwd[0]) > 0.9f) { alt[0] = 0.0f; alt[1] = 1.0f; }   // fwd ~ x -> use y
    v_cross(fwd, alt, right);
  }
  v_norm(right);
  float cup[3]; v_cross(right, fwd, cup);  // already unit
  float tan_half = tanf(0.5f * fov_y);
  float aspect = (float)width / (float)height;

  // Grid world AABB.
  float lo[3] = { density->ox, density->oy, density->oz };
  float hi[3] = {
    density->ox + density->nx * density->voxel_size,
    density->oy + density->ny * density->voxel_size,
    density->oz + density->nz * density->voxel_size,
  };

  #pragma omp parallel for schedule(static)
  for (int py = 0; py < height; ++py) {
    for (int px = 0; px < width; ++px) {
      // Pixel ray direction (px center; +x right, +y up, top row first).
      float sx = (2.0f * ((float)px + 0.5f) / (float)width - 1.0f) * aspect * tan_half;
      float sy = (1.0f - 2.0f * ((float)py + 0.5f) / (float)height) * tan_half;
      float dir[3] = {
        fwd[0] + sx*right[0] + sy*cup[0],
        fwd[1] + sx*right[1] + sy*cup[1],
        fwd[2] + sx*right[2] + sy*cup[2],
      };
      v_norm(dir);

      float t0, t1, transmit = 1.0f;
      if (ray_aabb(eye, dir, lo, hi, &t0, &t1) && t1 > t0) {
        for (float t = t0 + 0.5f*step; t < t1 && transmit > 1e-3f; t += step) {
          float wx = eye[0] + t*dir[0], wy = eye[1] + t*dir[1], wz = eye[2] + t*dir[2];
          float d = tvdb_sample_trilinear_dense(density, wx, wy, wz);
          if (d <= 0.0f) continue;
          float alpha = 1.0f - expf(-d * sigma * step);
          transmit *= (1.0f - alpha);
        }
      }
      float opacity = 1.0f - transmit;
      out_image[py*width + px] = opacity + transmit * background;
    }
  }
  return true;
}
