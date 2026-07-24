#pragma once

// Coordinate utilities and point/coordinate spatial queries (parallels fvdb
// IjkToIndex / MortonHilbertFromIjk / PointsInGrid / CoordsInGrid and OpenVDB
// point voxelization). Pure-C, operates on flat int32 coord triples and float
// world points.
//
// NOTE: the hash-backed queries (coords_in_set / points_in_set / ijk_to_index /
// voxelize) pack each axis into 21 biased bits, so coordinates must lie in
// [-2^20, 2^20-1]. The same range applies to morton_encode/decode.

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Batch world->voxel ijk (the cell containing each point):
//   ijk = floor((world - origin) / voxel_size), per axis.
// `points` is `n` xyz triples; `out_ijk` is `n` int32 triples.
void tvdb_world_to_ijk(const float* points,
                       size_t n,
                       const float voxel_size[3],
                       const float origin[3],
                       int32_t* out_ijk);

// Batch voxel ijk -> world voxel-CENTER position:
//   world = origin + (ijk + 0.5) * voxel_size, per axis.
void tvdb_ijk_to_world(const int32_t* ijk,
                       size_t n,
                       const float voxel_size[3],
                       const float origin[3],
                       float* out_points);

// 3D Morton (Z-order) code, 21 bits per axis (biased by 2^20 for signed coords).
uint64_t tvdb_morton_encode(int32_t x, int32_t y, int32_t z);
void tvdb_morton_decode(uint64_t code, int32_t* x, int32_t* y, int32_t* z);

// Voxelize a point cloud: world `points` (n triples) -> the set of unique
// occupied voxel coords, in first-seen order. `*out_coords` is malloc'd (caller
// frees), `*out_count` is the number of unique voxels. Returns false on OOM.
bool tvdb_voxelize_points(const float* points,
                          size_t n,
                          const float voxel_size[3],
                          const float origin[3],
                          int32_t** out_coords,
                          size_t* out_count);

// Membership of `query` coords (nq triples) in the `active` coord set (na
// triples): out[i] = 1 if present else 0. Returns false on OOM.
bool tvdb_coords_in_set(const int32_t* active, size_t na, const int32_t* query, size_t nq, uint8_t* out);

// For each world point, 1 if its voxel (floor) is in the active set.
bool tvdb_points_in_set(const float* points,
                        size_t np,
                        const float voxel_size[3],
                        const float origin[3],
                        const int32_t* active,
                        size_t na,
                        uint8_t* out);

// For each `query` coord, its first-seen index in `active` (0..na-1), or -1 if
// absent. Returns false on OOM.
bool tvdb_ijk_to_index(const int32_t* active, size_t na, const int32_t* query, size_t nq, int64_t* out);

// For each active voxel, the number of its neighbors that are also active.
// `connectivity` is 6 (face) or 26 (face+edge+vertex). Returns false on OOM.
bool tvdb_neighbor_counts(const int32_t* active, size_t na, int connectivity, int32_t* out_counts);

#ifdef __cplusplus
}
#endif
