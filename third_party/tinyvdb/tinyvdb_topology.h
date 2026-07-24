#pragma once

// Topology / pooling operations on dense grids.
// Sparse-tree variants live separately (see tinyvdb_ops.h, Phase 3 surface).

#include <stdbool.h>
#include <stddef.h>
#include "tinyvdb_mesh.h"  // tvdb_dense_grid
#include "tvdb_memory.h"

#ifdef __cplusplus
extern "C" {
#endif

// Coarsen by an integer factor: out dims = ceil(in dims / factor); each output
// voxel is the average of its `factor^3` source voxels.
// `out` may be uninitialized; if `arena` is NULL, malloc is used.
bool tvdb_coarsen_grid(const tvdb_dense_grid* in, int factor, tvdb_dense_grid* out, tvdb_arena_allocator_t* arena);

// Refine by an integer factor via trilinear upsampling.
bool tvdb_refine_grid(const tvdb_dense_grid* in, int factor, tvdb_dense_grid* out, tvdb_arena_allocator_t* arena);

// Resample `in` to an arbitrary (non-integer) `voxel_size`, covering the same
// world AABB and origin. `order` selects the sampler: 0 = nearest (point),
// 1 = trilinear (box), 2 = triquadratic. Generalizes coarsen/refine to any
// scale factor (parallels OpenVDB GridTransformer::resampleToMatch).
// `out` may be uninitialized; if `arena` is NULL, malloc is used.
bool tvdb_resample_grid(const tvdb_dense_grid* in,
                        float voxel_size,
                        int order,
                        tvdb_dense_grid* out,
                        tvdb_arena_allocator_t* arena);

// Snap voxels within `tolerance` of `background` to exactly `background`.
// (For dense grids this is mostly a quantization/cleanup step; the sparse
// analogue would also drop those voxels.)
void tvdb_prune_grid(tvdb_dense_grid* g, float background, float tolerance);

// Clip to a world-space bounding box; output dims and origin set to match
// the clipped region. Tightly aligned to voxel grid.
bool tvdb_clip_grid(const tvdb_dense_grid* in,
                    const float bbox_min[3],
                    const float bbox_max[3],
                    tvdb_dense_grid* out,
                    tvdb_arena_allocator_t* arena);

// Merge two grids of the same voxel_size: the output spans the union of
// their world-space bounding boxes; overlapping voxels take the min (SDF
// union). Background values fill non-covered region.
bool tvdb_merge_grids(const tvdb_dense_grid* a,
                      const tvdb_dense_grid* b,
                      float background,
                      tvdb_dense_grid* out,
                      tvdb_arena_allocator_t* arena);

// kx * ky * kz pooling; output dims = ceil(in / k). Voxel_size is scaled
// by k along each axis (out->voxel_size = in->voxel_size; coarse units).
void tvdb_max_pool(const tvdb_dense_grid* in,
                   int kx,
                   int ky,
                   int kz,
                   tvdb_dense_grid* out,
                   tvdb_arena_allocator_t* arena);
void tvdb_avg_pool(const tvdb_dense_grid* in,
                   int kx,
                   int ky,
                   int kz,
                   tvdb_dense_grid* out,
                   tvdb_arena_allocator_t* arena);

#ifdef __cplusplus
}
#endif
