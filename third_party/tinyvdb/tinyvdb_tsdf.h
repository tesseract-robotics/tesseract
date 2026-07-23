#pragma once

// Truncated signed distance field (TSDF) fusion from a single depth frame.
// Loosely follows the integration step from KinectFusion / fvdb's
// IntegrateTSDF, on the dense-grid representation.

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "tinyvdb_mesh.h"  // tvdb_dense_grid, tvdb_vec3f
#include "tinyvdb_ops.h"   // tvdb_dense_vec_grid

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  int width;
  int height;
  const float* depth;    // row-major, size width*height
  float fx, fy, cx, cy;  // pinhole intrinsics (pixels)
  float pose[12];        // world<-camera 4x3 row-major (R | t)
  float trunc_distance;  // band width in world units
  float depth_min;       // valid depth range
  float depth_max;
} tvdb_depth_frame;

// Compute the camera-from-world transform from world-from-camera (rigid only:
// inverse = transpose(R) | -R^T * t). Returns the inverse pose into out[12].
void tvdb_invert_rigid_pose(const float pose_wc[12], float pose_cw_out[12]);

// Integrate a depth frame into an existing TSDF grid + weight grid.
// Both grids must be the same shape / origin / voxel_size and pre-allocated.
// On the first call, callers should initialize tsdf to trunc_distance and
// weights to 0.
bool tvdb_integrate_tsdf(tvdb_dense_grid* tsdf, tvdb_dense_grid* weights, const tvdb_depth_frame* frame);

// Same as above but also fuses per-pixel RGB into a 3-channel color volume.
// `rgb` is row-major HxWx3 in [0..255]; the color volume stores running
// floating-point averages.
bool tvdb_integrate_tsdf_with_color(tvdb_dense_grid* tsdf,
                                    tvdb_dense_grid* weights,
                                    tvdb_dense_vec_grid* color,
                                    const tvdb_depth_frame* frame,
                                    const uint8_t* rgb);

#ifdef __cplusplus
}
#endif
