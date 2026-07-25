#pragma once

// Minimal volume renderer: emission-absorption alpha compositing of a dense
// density grid along pinhole-camera rays (parallels fvdb VolumeRender; a
// library-level version of the vdbrender example).

#include <stdbool.h>
#include "tinyvdb_mesh.h"  // tvdb_dense_grid

#ifdef __cplusplus
extern "C" {
#endif

// Render `density` (a fog/density grid; values clamped at 0) with a pinhole
// camera looking from `eye` toward `center` with `up`, vertical field of view
// `fov_y` (radians). Each ray is clipped to the grid's world AABB and marched
// front-to-back with world step `step`: per sample alpha = 1 - exp(-density *
// sigma * step), transmittance T *= (1-alpha). The output grayscale pixel is
// the accumulated opacity (1-T) composited over `background`:
//   pixel = (1 - T) + T * background.
// `out_image` is a pre-allocated width*height float buffer (row-major, top row
// first). Returns false on bad args.
bool tvdb_volume_render(const tvdb_dense_grid* density,
                        const float eye[3],
                        const float center[3],
                        const float up[3],
                        float fov_y,
                        int width,
                        int height,
                        float sigma,
                        float step,
                        float background,
                        float* out_image);

#ifdef __cplusplus
}
#endif
