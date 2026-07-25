#pragma once

// Jagged (variable-length batch) containers, the CPU analogues of fvdb's
// JaggedTensor and GridBatch. A JaggedTensor stores a batch of variable-length
// lists of fixed-channel float elements as one flat data array plus a list of
// element offsets (the "jagged offsets"). A GridBatch is a batch of sparse
// grids stored in the same jagged layout, with per-grid transforms, plus a
// bridge to JaggedTensor and read-only per-grid views that can feed the
// existing batched GPU ops (tvdb_gpu_sparse_conv3d_batched).

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "tinyvdb_ray.h"     // tvdb_vec3i
#include "tinyvdb_sparse.h"  // tvdb_sparse_grid

#ifdef __cplusplus
extern "C" {
#endif

// ---- JaggedTensor -----------------------------------------------------------

typedef struct
{
  float* data;       // total * channels floats, row-major [element][channel]
  int64_t* offsets;  // num_lists + 1 element offsets; offsets[0]=0, offsets[L]=total
  int64_t num_lists;
  int channels;  // features per element (>= 1)
} tvdb_jagged_t;

void tvdb_jagged_init(tvdb_jagged_t* jt);
void tvdb_jagged_free(tvdb_jagged_t* jt);

// Allocate a jagged tensor from per-list element counts; data is zeroed.
bool tvdb_jagged_create(tvdb_jagged_t* jt, int64_t num_lists, const int64_t* list_sizes, int channels);

// Build from `num_lists` separate contiguous arrays; list_data[i] holds
// list_sizes[i]*channels floats (may be NULL when its size is 0).
bool tvdb_jagged_from_lists(tvdb_jagged_t* jt,
                            int64_t num_lists,
                            const float* const* list_data,
                            const int64_t* list_sizes,
                            int channels);

int64_t tvdb_jagged_total(const tvdb_jagged_t* jt);                 // total elements
int64_t tvdb_jagged_list_count(const tvdb_jagged_t* jt);            // num lists
int64_t tvdb_jagged_list_size(const tvdb_jagged_t* jt, int64_t i);  // elements in list i

// Pointer to list i's (channel-interleaved) elements; *out_size gets its
// element count. Returns NULL on a bad index.
float* tvdb_jagged_list_ptr(const tvdb_jagged_t* jt, int64_t i, int64_t* out_size);

// Concatenate along the list dimension: out has sum(parts[k].num_lists) lists.
// All parts must share `channels`.
bool tvdb_jagged_concat(tvdb_jagged_t* out, const tvdb_jagged_t* const* parts, int64_t num_parts);

// Per-list reductions over elements into out[num_lists * channels] (channel-
// interleaved). Empty lists yield 0 (sum/mean) or 0 (max/min, documented).
bool tvdb_jagged_sum(const tvdb_jagged_t* jt, float* out);
bool tvdb_jagged_mean(const tvdb_jagged_t* jt, float* out);
bool tvdb_jagged_max(const tvdb_jagged_t* jt, float* out);
bool tvdb_jagged_min(const tvdb_jagged_t* jt, float* out);

// ---- GridBatch --------------------------------------------------------------

typedef struct
{
  tvdb_vec3i* coords;  // total active voxels
  float* values;       // total
  int64_t* offsets;    // num_grids + 1
  int64_t num_grids;
  float* voxel_size;  // num_grids
  float* origin;      // num_grids * 3 (ox, oy, oz)
} tvdb_grid_batch_t;

void tvdb_grid_batch_init(tvdb_grid_batch_t* gb);
void tvdb_grid_batch_free(tvdb_grid_batch_t* gb);

// Build a jagged concatenation of `n` sparse grids (copies coords/values and
// per-grid transforms).
bool tvdb_grid_batch_from_grids(tvdb_grid_batch_t* gb, const tvdb_sparse_grid* grids, int64_t n);

int64_t tvdb_grid_batch_size(const tvdb_grid_batch_t* gb);  // num grids
int64_t tvdb_grid_batch_total_voxels(const tvdb_grid_batch_t* gb);
int64_t tvdb_grid_batch_grid_size(const tvdb_grid_batch_t* gb, int64_t i);

// Read-only view of grid i as a tvdb_sparse_grid whose coords/values point into
// the batch (capacity = 0). DO NOT free or grow the view; it is valid only
// while the batch lives. Returns false on a bad index.
bool tvdb_grid_batch_view(const tvdb_grid_batch_t* gb, int64_t i, tvdb_sparse_grid* out_view);

// Fill out_views[num_grids] with read-only views (e.g. to feed a batched GPU
// op). Same lifetime/ownership rules as tvdb_grid_batch_view.
bool tvdb_grid_batch_views(const tvdb_grid_batch_t* gb, tvdb_sparse_grid* out_views);

// Bridge to JaggedTensor: a channels=1 jagged tensor over the batch values,
// sharing the batch's per-grid offsets (values are copied).
bool tvdb_grid_batch_values_jagged(const tvdb_grid_batch_t* gb, tvdb_jagged_t* out);

// Spatial query: local index of `ijk` within grid `b` (brute-force scan,
// mirroring tvdb_ijk_to_index), or -1 if absent / bad index.
int64_t tvdb_grid_batch_ijk_to_index(const tvdb_grid_batch_t* gb, int64_t b, const int32_t ijk[3]);

#ifdef __cplusplus
}
#endif
