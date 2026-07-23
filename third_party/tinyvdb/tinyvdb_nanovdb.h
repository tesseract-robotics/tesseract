/*
 * TinyVDB NanoVDB I/O — NanoVDB file format support for TinyVDB.
 *
 * Copyright (c) 2026 - Present Syoyo Fujita
 * Based on NanoVDB by Ken Museth
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * NanoVDB (by Ken Museth) is a read-only sparse GPU (and CPU) friendly
 * data structure. This module provides read/write support for NanoVDB files.
 *
 * Usage:
 *   #define TINYVDB_NANOVDB_IMPLEMENTATION
 *   #include "tinyvdb_nanovdb.h"
 */
#ifndef TINYVDB_NANOVDB_H_
#define TINYVDB_NANOVDB_H_

#include <stdint.h>
#include <stddef.h>

/* Forward declare types from tinyvdb_io.h to avoid circular dependency */
#ifndef TVDB_STATUS_T_DEFINED
#define TVDB_STATUS_T_DEFINED
typedef enum tvdb_status
{
  TVDB_OK = 0,
  TVDB_ERROR_INVALID_FILE,
  TVDB_ERROR_INVALID_HEADER,
  TVDB_ERROR_INVALID_DATA,
  TVDB_ERROR_INVALID_ARGUMENT,
  TVDB_ERROR_UNSUPPORTED_VERSION,
  TVDB_ERROR_UNSUPPORTED_GRID_TYPE,
  TVDB_ERROR_UNSUPPORTED_COMPRESSION,
  TVDB_ERROR_UNSUPPORTED_TRANSFORM,
  TVDB_ERROR_DECOMPRESSION_FAILED,
  TVDB_ERROR_OUT_OF_MEMORY,
  TVDB_ERROR_IO,
  TVDB_ERROR_MMAP_FAILED,
  TVDB_ERROR_PATH_CONVERSION,
  TVDB_ERROR_UNIMPLEMENTED
} tvdb_status_t;

typedef struct tvdb_error
{
  tvdb_status_t status;
  char message[512];
  uint64_t byte_offset;
  int32_t grid_index;
} tvdb_error_t;

typedef struct tvdb_allocator
{
  void* (*malloc_fn)(size_t size, void* user_ctx);
  void* (*realloc_fn)(void* ptr, size_t old_size, size_t new_size, void* user_ctx);
  void (*free_fn)(void* ptr, size_t size, void* user_ctx);
  void* user_ctx;
} tvdb_allocator_t;

#define TVDB_MAX_ERROR_MSG 512
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*  Constants                                                                 */
/* ========================================================================== */

/* NanoVDB magic numbers */
#define TVDB_NANOVDB_MAGIC_FILE 0x324244566f6e614eULL /* "NanoVDB2" */
#define TVDB_NANOVDB_MAGIC_GRID 0x314244566f6e614eULL /* "NanoVDB1" */
#define TVDB_NANOVDB_MAGIC_NUMB 0x304244566f6e614eULL /* "NanoVDB0" */

#define TVDB_NANOVDB_VERSION_MAJOR 32
#define TVDB_NANOVDB_VERSION_MINOR 6
#define TVDB_NANOVDB_VERSION_PATCH 0

/* Memory alignment (32 bytes) */
#define TVDB_NANOVDB_DATA_ALIGNMENT 32

/* Node dimensions (fixed for NanoVDB) */
#define TVDB_NANOVDB_LEAF_DIM 8
#define TVDB_NANOVDB_LEAF_LOG2DIM 3
#define TVDB_NANOVDB_LEAF_VOXELS 512 /* 8^3 */
#define TVDB_NANOVDB_INTERIOR_DIM 16
#define TVDB_NANOVDB_INTERIOR_LOG2DIM 4
#define TVDB_NANOVDB_INTERIOR_TILES 4096 /* 16^3 */
#define TVDB_NANOVDB_UPPER_DIM 32
#define TVDB_NANOVDB_UPPER_LOG2DIM 5
#define TVDB_NANOVDB_UPPER_TILES 32768 /* 32^3 */

/* ========================================================================== */
/*  Enums                                                                     */
/* ========================================================================== */

typedef enum tvdb_nanovdb_codec
{
  TVDB_NANOVDB_CODEC_NONE = 0,
  TVDB_NANOVDB_CODEC_ZIP = 1,
  TVDB_NANOVDB_CODEC_BLOSC = 2
} tvdb_nanovdb_codec_t;

typedef enum tvdb_nanovdb_grid_type
{
  TVDB_NANOVDB_GRID_TYPE_UNKNOWN = 0,
  TVDB_NANOVDB_GRID_TYPE_FLOAT = 1,
  TVDB_NANOVDB_GRID_TYPE_DOUBLE = 2,
  TVDB_NANOVDB_GRID_TYPE_INT16 = 3,
  TVDB_NANOVDB_GRID_TYPE_INT32 = 4,
  TVDB_NANOVDB_GRID_TYPE_INT64 = 5,
  TVDB_NANOVDB_GRID_TYPE_VEC3F = 6,
  TVDB_NANOVDB_GRID_TYPE_VEC3D = 7,
  TVDB_NANOVDB_GRID_TYPE_MASK = 8,
  TVDB_NANOVDB_GRID_TYPE_HALF = 9,
  TVDB_NANOVDB_GRID_TYPE_UINT32 = 10,
  TVDB_NANOVDB_GRID_TYPE_BOOLEAN = 11,
  TVDB_NANOVDB_GRID_TYPE_RGBA8 = 12,
  TVDB_NANOVDB_GRID_TYPE_FP4 = 13,
  TVDB_NANOVDB_GRID_TYPE_FP8 = 14,
  TVDB_NANOVDB_GRID_TYPE_FP16 = 15,
  TVDB_NANOVDB_GRID_TYPE_FPN = 16,
  TVDB_NANOVDB_GRID_TYPE_VEC4F = 17,
  TVDB_NANOVDB_GRID_TYPE_VEC4D = 18,
  TVDB_NANOVDB_GRID_TYPE_INDEX = 19,
  TVDB_NANOVDB_GRID_TYPE_ONINDEX = 20,
  TVDB_NANOVDB_GRID_TYPE_POINT_INDEX = 23,
  TVDB_NANOVDB_GRID_TYPE_VEC3U8 = 24,
  TVDB_NANOVDB_GRID_TYPE_VEC3U16 = 25,
  TVDB_NANOVDB_GRID_TYPE_UINT8 = 26
} tvdb_nanovdb_grid_type_t;

typedef enum tvdb_nanovdb_grid_class
{
  TVDB_NANOVDB_GRID_CLASS_UNKNOWN = 0,
  TVDB_NANOVDB_GRID_CLASS_LEVEL_SET = 1,
  TVDB_NANOVDB_GRID_CLASS_FOG_VOLUME = 2,
  TVDB_NANOVDB_GRID_CLASS_STAGGERED = 3,
  TVDB_NANOVDB_GRID_CLASS_COLLISION = 4,
  TVDB_NANOVDB_GRID_CLASS_POINT_INDEX = 5,
  TVDB_NANOVDB_GRID_CLASS_POINT_DATA = 6,
  TVDB_NANOVDB_GRID_CLASS_STUB = 7,
  TVDB_NANOVDB_GRID_CLASS_MACCABE = 8,
  TVDB_NANOVDB_GRID_CLASS_SDF_FUZZY = 9
} tvdb_nanovdb_grid_class_t;

typedef enum tvdb_nanovdb_checksum_mode
{
  TVDB_NANOVDB_CHECKSUM_NONE = 0,
  TVDB_NANOVDB_CHECKSUM_EASTWOOD = 1,
  TVDB_NANOVDB_CHECKSUM_DEFAULT = 2
} tvdb_nanovdb_checksum_mode_t;

/* ========================================================================== */
/*  Structures                                                                 */
/* ========================================================================== */

/* File header (16 bytes) */
typedef struct tvdb_nanovdb_file_header
{
  uint64_t magic;
  uint32_t version;
  uint16_t grid_count;
  uint16_t codec;
} tvdb_nanovdb_file_header_t;

/* File metadata (176 bytes) */
typedef struct tvdb_nanovdb_file_meta
{
  uint64_t grid_size;
  uint64_t file_size;
  uint64_t name_key;
  uint64_t voxel_count;
  uint32_t grid_type;
  uint32_t grid_class;
  double world_bbox_min[3];
  double world_bbox_max[3];
  int32_t index_bbox_min[3];
  int32_t index_bbox_max[3];
  double voxel_size[3];
  uint32_t name_size;
  uint32_t node_count[4];
  uint32_t tile_count[3];
  uint16_t codec;
  uint16_t blind_data_count;
  uint32_t version;
} tvdb_nanovdb_file_meta_t;

/* Grid data (672 bytes, 32-byte aligned) */
typedef struct tvdb_nanovdb_grid_data
{
  uint64_t magic;
  uint64_t checksum;
  uint32_t version;
  uint32_t flags;
  uint32_t grid_index;
  uint32_t grid_count;
  uint64_t grid_size;
  char grid_name[256];
  double map[12];
  double world_bbox_min[3];
  double world_bbox_max[3];
  double voxel_size[3];
  uint32_t grid_class;
  uint32_t grid_type;
  int64_t blind_metadata_offset;
  uint32_t blind_metadata_count;
  uint32_t data0;
  uint64_t data1;
  uint64_t data2;
} tvdb_nanovdb_grid_data_t;

/* Tree data (64 bytes, 32-byte aligned) */
typedef struct tvdb_nanovdb_tree_data
{
  int64_t node_offset[4];
  uint32_t node_count[3];
  uint32_t tile_count[3];
  uint64_t voxel_count;
} tvdb_nanovdb_tree_data_t;

/* Node types (used for traversal) */
typedef enum tvdb_nanovdb_node_type
{
  TVDB_NANOVDB_NODE_LEAF = 0,
  TVDB_NANOVDB_NODE_LOWER = 1,
  TVDB_NANOVDB_NODE_UPPER = 2,
  TVDB_NANOVDB_NODE_ROOT = 3
} tvdb_nanovdb_node_type_t;

/* Blind metadata entry (40 bytes) */
typedef struct tvdb_nanovdb_blind_meta
{
  int64_t byte_offset;
  uint64_t byte_size;
  uint64_t name_key;
  uint32_t value_count;
  uint32_t grid_type;
  uint32_t semantic;
  uint32_t flags;
} tvdb_nanovdb_blind_meta_t;

/* Grid structure for accessing NanoVDB data */
typedef struct tvdb_nanovdb_grid
{
  char* name;
  uint64_t size;
  uint32_t grid_type;
  uint32_t grid_class;
  double voxel_size[3];
  double world_bbox_min[3];
  double world_bbox_max[3];
  int32_t index_bbox_min[3];
  int32_t index_bbox_max[3];
  /* 3x4 row-major affine: [m00 m01 m02 tx, m10 m11 m12 ty, m20 m21 m22 tz].
   * Populated at load time from the grid's GridData.map; for grids built
   * by tvdb_nanovdb_create_grid the caller may need to populate this
   * before calling tvdb_nanovdb_world_to_index / _index_to_world. */
  double map[12];
  uint64_t active_voxel_count;
  uint32_t node_count[4];
  uint32_t tile_count[3];

  /* Raw data pointers (relative to grid start) */
  int64_t tree_data_offset;
  int64_t root_data_offset;
  int64_t leaf_data_offset;
  int64_t lower_data_offset;
  int64_t upper_data_offset;

  uint8_t* data;
  int owns_data;
} tvdb_nanovdb_grid_t;

/* Top-level file context */
typedef struct tvdb_nanovdb_file
{
  uint32_t version;
  uint16_t grid_count;
  uint16_t codec;
  tvdb_nanovdb_grid_t* grids;
  size_t num_grids;
  uint64_t file_size;
  const uint8_t* mmap_data;
  uint8_t* buffer;
  tvdb_allocator_t alloc;
} tvdb_nanovdb_file_t;

/* ========================================================================== */
/*  Gaussian Splat Structures                                                 */
/* ========================================================================== */

#define TVDB_GAUSSIAN_SPLAT_VERSION "fvdb_ply 1.0.0"

typedef struct tvdb_gaussian_splat
{
  uint32_t version;
  uint32_t num_gaussians;
  uint32_t sh_degree;
  uint32_t sh_dim;

  float* means;
  float* quats;
  float* log_scales;
  float* logit_opacities;
  float* sh_coeffs;

  char* metadata_keys;
  float* metadata_values;
  uint32_t* metadata_types;
  uint32_t* metadata_counts;
  uint32_t num_metadata;

  uint8_t owns_data;
} tvdb_gaussian_splat_t;

void tvdb_gaussian_splat_destroy(tvdb_gaussian_splat_t* splat);

/* ========================================================================== */
/*  Gaussian Splat Rasterization Structures                                   */
/* ========================================================================== */

#define TVDB_GAUSSIAN_RASTER_MAX_FEATURES 256
#define TVDB_GAUSSIAN_RASTER_TILE_SIZE 16
#define TVDB_GAUSSIAN_RASTER_DEFAULT_ALPHA_THRESHOLD 0.005f

typedef enum tvdb_camera_type
{
  TVDB_CAMERA_PERSPECTIVE = 0,
  TVDB_CAMERA_ORTHOGRAPHIC = 1
} tvdb_camera_type_t;

typedef struct tvdb_camera
{
  tvdb_camera_type_t type;
  float fx, fy;
  float cx, cy;
  float width, height;
  float near, far;
  float extrinsics[16];
  float intrinsics[9];
  int is_identity_extrinsics;
} tvdb_camera_t;

typedef struct tvdb_projected_gaussian
{
  float x, y;
  float conic_a, conic_b, conic_c;
  float opacity;
  float depth;
  float radius;
  float feature[3];
} tvdb_projected_gaussian_t;

typedef struct tvdb_tile_intersection
{
  int32_t gaussian_id;
  int32_t tile_x, tile_y;
} tvdb_tile_intersection_t;

typedef struct tvdb_raster_output
{
  float* image;
  float* alpha;
  int32_t* last_ids;
  uint32_t width;
  uint32_t height;
  uint32_t num_features;
  uint32_t owns_data;
} tvdb_raster_output_t;

void tvdb_raster_output_destroy(tvdb_raster_output_t* out);

/* ========================================================================== */
/*  Public API                                                                */
/* ========================================================================== */

/* File operations */
tvdb_status_t tvdb_nanovdb_file_open(tvdb_nanovdb_file_t* file,
                                     const char* filepath_utf8,
                                     const tvdb_allocator_t* alloc,
                                     tvdb_error_t* err);

tvdb_status_t tvdb_nanovdb_file_open_memory(tvdb_nanovdb_file_t* file,
                                            const uint8_t* data,
                                            size_t data_len,
                                            const tvdb_allocator_t* alloc,
                                            tvdb_error_t* err);

void tvdb_nanovdb_file_close(tvdb_nanovdb_file_t* file);

/* Grid access */
size_t tvdb_nanovdb_grid_count(const tvdb_nanovdb_file_t* file);
const char* tvdb_nanovdb_grid_name(const tvdb_nanovdb_file_t* file, size_t idx);
uint32_t tvdb_nanovdb_grid_type(const tvdb_nanovdb_file_t* file, size_t idx);
uint32_t tvdb_nanovdb_grid_class(const tvdb_nanovdb_file_t* file, size_t idx);
double tvdb_nanovdb_grid_voxel_size(const tvdb_nanovdb_file_t* file, size_t idx, int axis);

/* Value access (Root → Upper → Lower → Leaf, byte-exact NanoVDB layout
   via vendored PNanoVDB.h). Returns the active value at active voxels and
   the encoded inactive/background value at inactive voxels. */
float tvdb_nanovdb_get_voxel_f(const tvdb_nanovdb_grid_t* grid, int x, int y, int z);
double tvdb_nanovdb_get_voxel_d(const tvdb_nanovdb_grid_t* grid, int x, int y, int z);
int tvdb_nanovdb_is_voxel_active(const tvdb_nanovdb_grid_t* grid, int x, int y, int z);

/* Convert a loaded tinyvdb FloatGrid (Tree_float_5_4_3) into an in-memory
 * NanoVDB FloatGrid byte buffer. The output is malloc'd; caller must
 * `free(*out_data)`.
 *
 * Inputs:
 *   grid  — a tvdb_grid_t loaded from a .vdb file (e.g. via
 *           tvdb_file_open + tvdb_read_all_grids), with Tree_float_5_4_3
 *           layout. Other types return TVDB_ERROR_UNSUPPORTED_VERSION.
 *
 * Output:
 *   *out_data / *out_size — a complete NanoVDB FloatGrid byte stream
 *           starting at GridData (i.e. raw grid bytes, no codec/file
 *           header), suitable to wrap in tvdb_nanovdb_grid_t.data or
 *           write to disk preceded by a NanoVDB file header.
 *
 * Semantics: all active voxel values, leaf topology, internal-node tile
 * values, and root background are preserved. Per-leaf and per-tree
 * bounding boxes are recomputed; min/max/ave/stddev are derived from
 * active voxels. Checksum is set to "empty" (0xFFFFFFFFFFFFFFFF), which
 * NanoVDB treats as "skip validation".
 */
struct tvdb_grid;
typedef struct tvdb_grid tvdb_grid_t;
tvdb_status_t
tvdb_grid_to_nanovdb_float(const struct tvdb_grid* grid, uint8_t** out_data, size_t* out_size, tvdb_error_t* err);

/* Trilinear sample for FloatGrid. Cell-center convention: integer voxel
   (i,j,k) sits at sample point (i+0.5, j+0.5, k+0.5), so a sample at
   (i+0.5, j+0.5, k+0.5) returns get_voxel_f(i,j,k) exactly. Out-of-bounds
   taps fall back through the standard accessor (background value). */
float tvdb_nanovdb_sample_trilinear_f(const tvdb_nanovdb_grid_t* grid, float ix, float iy, float iz);

/* World <-> index space transforms.
 *
 * The grid stores a 3x4 row-major affine `map`: [m00 m01 m02 tx, m10 m11
 * m12 ty, m20 m21 m22 tz]. World coordinates are obtained from index
 * coordinates as `world = M * index + t`. The inverse is computed via
 * adjugate / determinant for general 3x3 matrices; uniform-scale grids
 * (the common case) reduce to a per-axis divide.
 *
 * `out` is set to the transformed point. Returns TVDB_OK on success,
 * TVDB_ERROR_INVALID_ARGUMENT on null inputs, or
 * TVDB_ERROR_INVALID_DATA if the matrix is singular (determinant ~ 0).
 */
tvdb_status_t
tvdb_nanovdb_index_to_world(const tvdb_nanovdb_grid_t* grid, double ix, double iy, double iz, double out[3]);
tvdb_status_t
tvdb_nanovdb_world_to_index(const tvdb_nanovdb_grid_t* grid, double wx, double wy, double wz, double out[3]);

/* Writing API */
tvdb_status_t tvdb_nanovdb_write_to_memory(const tvdb_nanovdb_file_t* file,
                                           uint32_t compression_flags,
                                           uint8_t** out_data,
                                           size_t* out_size,
                                           tvdb_error_t* err);

tvdb_status_t tvdb_nanovdb_file_save(const tvdb_nanovdb_file_t* file,
                                     const char* filepath_utf8,
                                     uint32_t compression_flags,
                                     int use_mmap,
                                     tvdb_error_t* err);

/* Grid creation API */
uint64_t tvdb_nanovdb_leaf_node_size(uint32_t grid_type);
uint64_t tvdb_nanovdb_lower_node_size(uint32_t grid_type);
uint64_t tvdb_nanovdb_upper_node_size(uint32_t grid_type);
uint64_t tvdb_nanovdb_root_tile_size(void);

tvdb_status_t tvdb_nanovdb_create_grid(tvdb_nanovdb_grid_t* grid,
                                       const char* name,
                                       uint32_t grid_type,
                                       uint32_t grid_class,
                                       int32_t min_coord[3],
                                       int32_t max_coord[3],
                                       const tvdb_allocator_t* alloc,
                                       tvdb_error_t* err);

void tvdb_nanovdb_destroy_grid(tvdb_nanovdb_grid_t* grid, const tvdb_allocator_t* alloc);

/* Utility */
const char* tvdb_nanovdb_grid_type_name(uint32_t grid_type);
const char* tvdb_nanovdb_grid_class_name(uint32_t grid_class);
int tvdb_nanovdb_is_big_endian_file(const tvdb_nanovdb_file_t* file);
uint32_t tvdb_nanovdb_value_size(uint32_t grid_type);

/* CRC32 of a contiguous byte range. Polynomial 0xEDB88320, initial state
 * `crc`, returns ~final. Matches NanoVDB's util::crc32 byte-for-byte. */
uint32_t tvdb_nanovdb_crc32(const void* data, size_t size, uint32_t crc);

/* Compute the head CRC32 of a grid: bytes [16, sizeof(GridData)+sizeof(TreeData))
 * for grid file_version > 32.6.0, i.e. the GridData+TreeData blob excluding
 * the 16-byte magic + stored-checksum prefix. Returns 0 for older versions
 * (head split is per-node and not yet implemented).
 *
 * `grid->data` must point to the beginning of the GridData blob. */
uint32_t tvdb_nanovdb_compute_head_checksum(const tvdb_nanovdb_grid_t* grid);

/* Compute the tail CRC32 of a grid: blocked-CRC32 (4KB blocks; see
 * NANOVDB_CRC32_LOG2_BLOCK_SIZE) over bytes
 * [sizeof(GridData)+sizeof(TreeData), grid_size). Returns 0 if the grid
 * is too small or the version is older than 32.6.0. */
uint32_t tvdb_nanovdb_compute_tail_checksum(const tvdb_nanovdb_grid_t* grid);

/* Validate the stored 64-bit checksum (`grid_data.checksum`) against the
 * recomputed value.
 *
 * `mode` chooses how much to recompute:
 *   TVDB_NANOVDB_CHECKSUM_NONE       — accept any stored value (no-op)
 *   TVDB_NANOVDB_CHECKSUM_EASTWOOD   — head only (32-bit "Half" check)
 *   TVDB_NANOVDB_CHECKSUM_DEFAULT    — full check (head + tail)
 *
 * Returns 1 if the stored checksum matches, 0 if it doesn't, and
 * 1 if the stored checksum is the "empty" sentinel (0xFFFFFFFFFFFFFFFF —
 * meaning no checksum was written). */
int tvdb_nanovdb_validate_checksum(const tvdb_nanovdb_grid_t* grid, tvdb_nanovdb_checksum_mode_t mode);

/* ========================================================================== */
/*  Gaussian Splat PLY I/O                                                    */
/* ========================================================================== */

tvdb_gaussian_splat_t* tvdb_gaussian_splat_load(const char* filepath_utf8, tvdb_error_t* err);

tvdb_status_t tvdb_gaussian_splat_save(const char* filepath_utf8,
                                       const tvdb_gaussian_splat_t* splat,
                                       tvdb_error_t* err);

uint32_t tvdb_gaussian_splat_count(const tvdb_gaussian_splat_t* splat);
void tvdb_gaussian_splat_get(const tvdb_gaussian_splat_t* splat,
                             uint32_t idx,
                             float out_means[3],
                             float out_quats[4],
                             float out_scales[3],
                             float* out_opacity);

/* ========================================================================== */
/*  Gaussian Splat CPU Rasterization                                          */
/* ========================================================================== */

tvdb_camera_t* tvdb_camera_create_perspective(float fx,
                                              float fy,
                                              float cx,
                                              float cy,
                                              float width,
                                              float height,
                                              float near,
                                              float far,
                                              const float extrinsics[16]);
tvdb_camera_t* tvdb_camera_create_orthographic(float scale,
                                               float cx,
                                               float cy,
                                               float width,
                                               float height,
                                               float near,
                                               float far,
                                               const float extrinsics[16]);
void tvdb_camera_destroy(tvdb_camera_t* cam);

tvdb_projected_gaussian_t* tvdb_gaussian_project(const tvdb_gaussian_splat_t* splats,
                                                 const tvdb_camera_t* cam,
                                                 uint32_t* out_count,
                                                 tvdb_error_t* err);

tvdb_status_t tvdb_gaussian_rasterize_forward(const tvdb_projected_gaussian_t* gaussians,
                                              uint32_t num_gaussians,
                                              uint32_t width,
                                              uint32_t height,
                                              uint32_t num_features,
                                              float background[3],
                                              float alpha_threshold,
                                              tvdb_raster_output_t* out,
                                              tvdb_error_t* err);

/* CPU backward pass for tvdb_gaussian_rasterize_forward. Reverses the
 * per-tile depth-sorted alpha blend, replaying T and the post-i color
 * accumulator analytically (no per-pixel intersection list saved).
 * Caller pre-zeros the accumulator arrays in grad_out (or calls
 * tvdb_gaussian_grad_init); this routine accumulates contributions. */
typedef struct tvdb_gaussian_grad
{
  float* grad_x;
  float* grad_y;
  float* grad_conic_a;
  float* grad_conic_b;
  float* grad_conic_c;
  float* grad_opacity;
  float* grad_feature; /* [num_gaussians * num_features] */
  uint32_t num_gaussians;
  uint32_t num_features;
  uint8_t owns_data;
} tvdb_gaussian_grad_t;

tvdb_status_t tvdb_gaussian_grad_init(tvdb_gaussian_grad_t* g, uint32_t num_gaussians, uint32_t num_features);
void tvdb_gaussian_grad_destroy(tvdb_gaussian_grad_t* g);

tvdb_status_t tvdb_gaussian_rasterize_backward(const tvdb_projected_gaussian_t* gaussians,
                                               uint32_t num_gaussians,
                                               const tvdb_raster_output_t* forward_out,
                                               const float* dL_dC,
                                               const float* dL_dA,
                                               float background[3],
                                               float alpha_threshold,
                                               tvdb_gaussian_grad_t* grad_out,
                                               tvdb_error_t* err);

void tvdb_projected_gaussian_destroy(tvdb_projected_gaussian_t* gaussians);

/* Evaluate view-dependent RGB color from spherical-harmonics coefficients
 * (3D Gaussian Splatting convention). `degree` in [0,3]; `sh_coeffs` holds
 * num_gaussians * K * 3 floats with K=(degree+1)^2 basis functions per
 * Gaussian, laid out [(g*K + k)*3 + c] (basis-major, RGB inner). `dirs` holds
 * num_gaussians * 3 view directions (normalized internally; a ~zero direction
 * leaves only the DC term). `out_colors` receives num_gaussians * 3 =
 * max(sh_result + 0.5, 0). */
tvdb_status_t tvdb_gaussian_sh_eval(uint32_t num_gaussians,
                                    uint32_t degree,
                                    const float* sh_coeffs,
                                    const float* dirs,
                                    float* out_colors,
                                    tvdb_error_t* err);

/* MCMC densification (Kheradmand et al. "3D Gaussian Splatting as MCMC").
 *
 * Relocation parameter update: when a Gaussian is split/cloned into `ratios[g]`
 * copies that share its location, recompute each copy's opacity and scale so the
 * combined rendered contribution is preserved. `opacities` are actual opacities
 * in [0,1] (apply sigmoid first); `scales` are actual per-axis scales
 * (num_gaussians*3, apply exp first). `ratios[g]` is clamped to [1, 51].
 * new_opacity = 1 - (1 - op)^(1/ratio); new_scale = op/denom * scale where denom
 * is the MCMC binomial series in new_opacity. */
tvdb_status_t tvdb_gaussian_mcmc_relocation(uint32_t num_gaussians,
                                            const float* opacities,
                                            const float* scales,
                                            const int32_t* ratios,
                                            float* new_opacities,
                                            float* new_scales,
                                            tvdb_error_t* err);

/* MCMC exploration noise: add covariance-aware, opacity-gated noise to the
 * means. `means`/`out_means` are num_gaussians*3; `quats` num_gaussians*4
 * (xyzw, normalized internally); `log_scales` num_gaussians*3 (exp'd);
 * `opacities_logit` num_gaussians (sigmoid'd); `rand` num_gaussians*3 is
 * caller-supplied standard-normal noise (deterministic given it). For each
 * Gaussian: noise = Cov * (rand * gate * lr), gate = 1/(1+exp(-100*(0.005-op))),
 * Cov = R diag(s^2) R^T. `out_means` may alias `means`. */
tvdb_status_t tvdb_gaussian_mcmc_add_noise(uint32_t num_gaussians,
                                           const float* means,
                                           const float* quats,
                                           const float* log_scales,
                                           const float* opacities_logit,
                                           const float* rand,
                                           float lr,
                                           float* out_means,
                                           tvdb_error_t* err);

#ifdef __cplusplus
}
#endif

/* ========================================================================== */
/*  IMPLEMENTATION                                                            */
/* ========================================================================== */

#ifdef TINYVDB_NANOVDB_IMPLEMENTATION

#define TVDB_GAUSSIAN_VERSION "fvdb_ply 1.0.0"
#define TVDB_GAUSSIAN_MAGIC "fvdb_ply_af_8198767135"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include <math.h>

/* Vendored PNanoVDB.h provides byte-exact NanoVDB hierarchical accessors
   (Root → Upper → Lower → Leaf) in pure C, matching nanovdb::ReadAccessor.
   We use it for tvdb_nanovdb_get_voxel_f / _d / sample_trilinear_f. All
   PNANOVDB_FORCE_INLINE symbols are static inline, so multiple TUs that
   include this header don't collide. */
#define PNANOVDB_C
#include "PNanoVDB.h"

#ifndef TVDB_ASSERT
#define TVDB_ASSERT(x) assert(x)
#endif

#if !defined(_WIN32) && !defined(_POSIX_C_SOURCE)
#define _POSIX_C_SOURCE 200809L
#endif

#if !defined(TVDB_NO_MMAP)
#if defined(_WIN32)
#include <windows.h>
#ifdef near
#undef near
#endif
#ifdef far
#undef far
#endif
#else
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#endif
#endif

#include "lz4.h"

#if !defined(TVDB_USE_SYSTEM_ZLIB)
#ifndef MINIZ_NO_STDIO
#define MINIZ_NO_STDIO
#endif
#include "miniz.h"
#else
#include <zlib.h>
#endif

/* ========================================================================== */
/*  Internal helpers                                                          */
/* ========================================================================== */

static uint16_t tvdb__nnvdb_swap16(uint16_t v) { return (uint16_t)((v >> 8) | (v << 8)); }

static uint32_t tvdb__nnvdb_swap32(uint32_t v)
{
  return ((v >> 24) & 0x000000FF) | ((v >> 8) & 0x0000FF00) | ((v << 8) & 0x00FF0000) | ((v << 24) & 0xFF000000);
}

static uint64_t tvdb__nnvdb_swap64(uint64_t v)
{
  return ((v >> 56) & 0x00000000000000FFULL) | ((v >> 40) & 0x000000000000FF00ULL) |
         ((v >> 24) & 0x0000000000FF0000ULL) | ((v >> 8) & 0x00000000FF000000ULL) | ((v << 8) & 0x000000FF00000000ULL) |
         ((v << 24) & 0x0000FF0000000000ULL) | ((v << 40) & 0x00FF000000000000ULL) |
         ((v << 56) & 0xFF00000000000000ULL);
}

static double tvdb__nnvdb_swap_double(double v)
{
  union
  {
    double d;
    uint64_t u;
  } u;
  u.d = v;
  u.u = tvdb__nnvdb_swap64(u.u);
  return u.d;
}

static int tvdb__nnvdb_is_little_endian(void)
{
  const uint16_t x = 1;
  return ((const uint8_t*)&x)[0] == 1;
}

static void tvdb__nnvdb_set_error(tvdb_error_t* err, tvdb_status_t status, const char* msg)
{
  if (!err)
    return;
  err->status = status;
  if (msg)
  {
    size_t len = strlen(msg);
    if (len >= TVDB_MAX_ERROR_MSG)
      len = TVDB_MAX_ERROR_MSG - 1;
    memcpy(err->message, msg, len);
    err->message[len] = '\0';
  }
  else
  {
    err->message[0] = '\0';
  }
}

/* ========================================================================== */
/*  Stream reader for NanoVDB                                                 */
/* ========================================================================== */

typedef struct
{
  const uint8_t* data;
  uint64_t pos;
  uint64_t length;
  int swap_endian;
} tvdb__nnvdb_sr_t;

static void tvdb__nnvdb_sr_init(tvdb__nnvdb_sr_t* sr, const uint8_t* data, uint64_t length, int swap)
{
  sr->data = data;
  sr->pos = 0;
  sr->length = length;
  sr->swap_endian = swap;
}

static int tvdb__nnvdb_sr_read(tvdb__nnvdb_sr_t* sr, size_t n, void* dst)
{
  if (sr->pos + n > sr->length)
    return 0;
  memcpy(dst, sr->data + sr->pos, n);
  sr->pos += n;
  return 1;
}

static int tvdb__nnvdb_sr_seek(tvdb__nnvdb_sr_t* sr, uint64_t pos)
{
  if (pos > sr->length)
    return 0;
  sr->pos = pos;
  return 1;
}

static int tvdb__nnvdb_sr_skip(tvdb__nnvdb_sr_t* sr, int64_t offset)
{
  uint64_t new_pos = (offset < 0) ? (sr->pos < (uint64_t)(-offset) ? 0 : sr->pos + offset) : sr->pos + offset;
  if (new_pos > sr->length)
    return 0;
  sr->pos = new_pos;
  return 1;
}

static uint64_t tvdb__nnvdb_sr_pos(tvdb__nnvdb_sr_t* sr) { return sr->pos; }

static uint8_t tvdb__nnvdb_sr_read_u8(tvdb__nnvdb_sr_t* sr)
{
  uint8_t v = 0;
  tvdb__nnvdb_sr_read(sr, 1, &v);
  return v;
}

static uint16_t tvdb__nnvdb_sr_read_u16(tvdb__nnvdb_sr_t* sr)
{
  uint16_t v = 0;
  tvdb__nnvdb_sr_read(sr, 2, &v);
  return sr->swap_endian ? tvdb__nnvdb_swap16(v) : v;
}

static uint32_t tvdb__nnvdb_sr_read_u32(tvdb__nnvdb_sr_t* sr)
{
  uint32_t v = 0;
  tvdb__nnvdb_sr_read(sr, 4, &v);
  return sr->swap_endian ? tvdb__nnvdb_swap32(v) : v;
}

static uint64_t tvdb__nnvdb_sr_read_u64(tvdb__nnvdb_sr_t* sr)
{
  uint64_t v = 0;
  tvdb__nnvdb_sr_read(sr, 8, &v);
  return sr->swap_endian ? tvdb__nnvdb_swap64(v) : v;
}

static int32_t tvdb__nnvdb_sr_read_i32(tvdb__nnvdb_sr_t* sr) { return (int32_t)tvdb__nnvdb_sr_read_u32(sr); }

static int64_t tvdb__nnvdb_sr_read_i64(tvdb__nnvdb_sr_t* sr) { return (int64_t)tvdb__nnvdb_sr_read_u64(sr); }

static double tvdb__nnvdb_sr_read_double(tvdb__nnvdb_sr_t* sr)
{
  union
  {
    double d;
    uint64_t u;
  } v;
  v.u = tvdb__nnvdb_sr_read_u64(sr);
  return sr->swap_endian ? tvdb__nnvdb_swap_double(v.d) : v.d;
}

static void tvdb__nnvdb_sr_read_str(tvdb__nnvdb_sr_t* sr, size_t len, char* dst)
{
  tvdb__nnvdb_sr_read(sr, len, dst);
  dst[len] = '\0';
}

/* ========================================================================== */
/*  Stream writer for NanoVDB                                                 */
/* ========================================================================== */

typedef struct
{
  uint8_t* data;
  uint64_t pos;
  uint64_t capacity;
  int swap_endian;
} tvdb__nnvdb_sw_t;

static int tvdb__nnvdb_sw_init(tvdb__nnvdb_sw_t* sw, uint8_t* data, uint64_t capacity)
{
  sw->data = data;
  sw->pos = 0;
  sw->capacity = capacity;
  sw->swap_endian = 0;
  return 1;
}

static int tvdb__nnvdb_sw_ensure(tvdb__nnvdb_sw_t* sw, uint64_t needed)
{
  (void)sw;
  (void)needed;
  return 1;
}

static int tvdb__nnvdb_sw_write(tvdb__nnvdb_sw_t* sw, size_t n, const void* src)
{
  if (sw->pos + n > sw->capacity)
    return 0;
  memcpy(sw->data + sw->pos, src, n);
  sw->pos += n;
  return 1;
}

static void tvdb__nnvdb_sw_write_u8(tvdb__nnvdb_sw_t* sw, uint8_t v) { tvdb__nnvdb_sw_write(sw, 1, &v); }

static void tvdb__nnvdb_sw_write_u16(tvdb__nnvdb_sw_t* sw, uint16_t v)
{
  if (sw->swap_endian)
    v = tvdb__nnvdb_swap16(v);
  tvdb__nnvdb_sw_write(sw, 2, &v);
}

static void tvdb__nnvdb_sw_write_u32(tvdb__nnvdb_sw_t* sw, uint32_t v)
{
  if (sw->swap_endian)
    v = tvdb__nnvdb_swap32(v);
  tvdb__nnvdb_sw_write(sw, 4, &v);
}

static void tvdb__nnvdb_sw_write_u64(tvdb__nnvdb_sw_t* sw, uint64_t v)
{
  if (sw->swap_endian)
    v = tvdb__nnvdb_swap64(v);
  tvdb__nnvdb_sw_write(sw, 8, &v);
}

static void tvdb__nnvdb_sw_write_i32(tvdb__nnvdb_sw_t* sw, int32_t v) { tvdb__nnvdb_sw_write_u32(sw, (uint32_t)v); }

static void tvdb__nnvdb_sw_write_i64(tvdb__nnvdb_sw_t* sw, int64_t v) { tvdb__nnvdb_sw_write_u64(sw, (uint64_t)v); }

static void tvdb__nnvdb_sw_write_double(tvdb__nnvdb_sw_t* sw, double v)
{
  union
  {
    double d;
    uint64_t u;
  } u;
  u.d = v;
  if (sw->swap_endian)
    u.u = tvdb__nnvdb_swap64(u.u);
  tvdb__nnvdb_sw_write(sw, 8, &u.u);
}

static void tvdb__nnvdb_sw_write_str(tvdb__nnvdb_sw_t* sw, const char* s, size_t len)
{
  tvdb__nnvdb_sw_write(sw, len, s);
  if (len > 0 && s[len - 1] != '\0')
  {
    tvdb__nnvdb_sw_write_u8(sw, 0);
  }
}

/* ========================================================================== */
/*  BLOSC decompression for NanoVDB                                            */
/* ========================================================================== */

#ifdef TVDB_HAVE_BLOSC
#include <blosc.h>
#endif

static int tvdb__nnvdb_decompress_blosc(void* dst, size_t dst_size, const void* src, size_t src_size)
{
  if (!dst || !src)
    return 0;
#ifdef TVDB_HAVE_BLOSC
  /* Real BLOSC1 decompression. NanoVDB writes one chunk per grid (each
     up to 1GB uncompressed), and each chunk is a full Blosc1 frame. */
  int rc = blosc_decompress_ctx(src, dst, dst_size, /*numinternalthreads=*/1);
  return rc > 0 ? 1 : 0;
#else
  /* Fallback path retained only for backwards compatibility with the
     earlier "fake LZ4 with 12-byte prefix" framing produced by older
     tinyvdb writes. Real BLOSC-encoded NanoVDB files (e.g. produced by
     libnanovdb's nanovdb_convert --blosc) require building with
     TINYVDB_USE_SYSTEM_BLOSC=ON. */
  (void)src_size;
  if (src_size < 12)
    return 0;
  int rc = LZ4_decompress_safe((const char*)src + 12, (char*)dst, (int)(src_size - 12), (int)dst_size);
  return rc >= 0 ? 1 : 0;
#endif
}

/* ========================================================================== */
/*  ZIP decompression for NanoVDB                                              */
/* ========================================================================== */

static int tvdb__nnvdb_decompress_zip(void* dst, size_t dst_size, const void* src, size_t src_size)
{
  if (!dst || !src)
    return 0;

#if !defined(TVDB_USE_SYSTEM_ZLIB)
  mz_ulong decomp_size = (mz_ulong)dst_size;
  int rc = mz_uncompress((uint8_t*)dst, &decomp_size, src, src_size);
  return (rc == MZ_OK) ? 1 : 0;
#else
  uLongf decomp_size = (uLongf)dst_size;
  int rc = uncompress((uint8_t*)dst, &decomp_size, src, src_size);
  return (rc == Z_OK) ? 1 : 0;
#endif
}

/* ========================================================================== */
/*  Decompression wrapper                                                      */
/* ========================================================================== */

static int
tvdb__nnvdb_decompress(void* dst, size_t dst_size, const void* src, size_t src_size, tvdb_nanovdb_codec_t codec)
{
  switch (codec)
  {
    case TVDB_NANOVDB_CODEC_BLOSC:
      return tvdb__nnvdb_decompress_blosc(dst, dst_size, src, src_size);
    case TVDB_NANOVDB_CODEC_ZIP:
      return tvdb__nnvdb_decompress_zip(dst, dst_size, src, src_size);
    default:
      if (dst_size == src_size)
      {
        memcpy(dst, src, src_size);
        return 1;
      }
      return 0;
  }
}

/* ========================================================================== */
/*  File reading                                                              */
/* ========================================================================== */

static tvdb_status_t tvdb__nnvdb_read_header(tvdb__nnvdb_sr_t* sr, tvdb_nanovdb_file_header_t* hdr, tvdb_error_t* err)
{
  hdr->magic = tvdb__nnvdb_sr_read_u64(sr);
  hdr->version = tvdb__nnvdb_sr_read_u32(sr);
  hdr->grid_count = tvdb__nnvdb_sr_read_u16(sr);
  hdr->codec = tvdb__nnvdb_sr_read_u16(sr);

  if (hdr->magic == 0)
  {
    tvdb__nnvdb_set_error(err, TVDB_ERROR_INVALID_HEADER, "Empty or invalid NanoVDB file");
    return TVDB_ERROR_INVALID_HEADER;
  }

  if (hdr->magic == tvdb__nnvdb_swap64(TVDB_NANOVDB_MAGIC_FILE) ||
      hdr->magic == tvdb__nnvdb_swap64(TVDB_NANOVDB_MAGIC_NUMB))
  {
    sr->swap_endian = 1;
    hdr->magic = tvdb__nnvdb_swap64(hdr->magic);
    hdr->version = tvdb__nnvdb_swap32(hdr->version);
    hdr->grid_count = tvdb__nnvdb_swap16(hdr->grid_count);
    hdr->codec = tvdb__nnvdb_swap16(hdr->codec);
  }

  if (hdr->magic != TVDB_NANOVDB_MAGIC_FILE && hdr->magic != TVDB_NANOVDB_MAGIC_GRID &&
      hdr->magic != TVDB_NANOVDB_MAGIC_NUMB)
  {
    tvdb__nnvdb_set_error(err, TVDB_ERROR_INVALID_HEADER, "Not a NanoVDB file");
    return TVDB_ERROR_INVALID_HEADER;
  }

  return TVDB_OK;
}

static tvdb_status_t tvdb__nnvdb_read_file_meta(tvdb__nnvdb_sr_t* sr, tvdb_nanovdb_file_meta_t* meta, tvdb_error_t* err)
{
  meta->grid_size = tvdb__nnvdb_sr_read_u64(sr);
  meta->file_size = tvdb__nnvdb_sr_read_u64(sr);
  meta->name_key = tvdb__nnvdb_sr_read_u64(sr);
  meta->voxel_count = tvdb__nnvdb_sr_read_u64(sr);
  meta->grid_type = tvdb__nnvdb_sr_read_u32(sr);
  meta->grid_class = tvdb__nnvdb_sr_read_u32(sr);

  for (int i = 0; i < 3; i++)
    meta->world_bbox_min[i] = tvdb__nnvdb_sr_read_double(sr);
  for (int i = 0; i < 3; i++)
    meta->world_bbox_max[i] = tvdb__nnvdb_sr_read_double(sr);
  for (int i = 0; i < 3; i++)
    meta->index_bbox_min[i] = tvdb__nnvdb_sr_read_i32(sr);
  for (int i = 0; i < 3; i++)
    meta->index_bbox_max[i] = tvdb__nnvdb_sr_read_i32(sr);
  for (int i = 0; i < 3; i++)
    meta->voxel_size[i] = tvdb__nnvdb_sr_read_double(sr);

  meta->name_size = tvdb__nnvdb_sr_read_u32(sr);
  for (int i = 0; i < 4; i++)
    meta->node_count[i] = tvdb__nnvdb_sr_read_u32(sr);
  for (int i = 0; i < 3; i++)
    meta->tile_count[i] = tvdb__nnvdb_sr_read_u32(sr);
  meta->codec = tvdb__nnvdb_sr_read_u16(sr);
  meta->blind_data_count = tvdb__nnvdb_sr_read_u16(sr);
  meta->version = tvdb__nnvdb_sr_read_u32(sr);

  return TVDB_OK;
}

static tvdb_status_t tvdb__nnvdb_read_grid_data(tvdb__nnvdb_sr_t* sr, tvdb_nanovdb_grid_data_t* gd, tvdb_error_t* err)
{
  gd->magic = tvdb__nnvdb_sr_read_u64(sr);
  gd->checksum = tvdb__nnvdb_sr_read_u64(sr);
  gd->version = tvdb__nnvdb_sr_read_u32(sr);
  gd->flags = tvdb__nnvdb_sr_read_u32(sr);
  gd->grid_index = tvdb__nnvdb_sr_read_u32(sr);
  gd->grid_count = tvdb__nnvdb_sr_read_u32(sr);
  gd->grid_size = tvdb__nnvdb_sr_read_u64(sr);
  tvdb__nnvdb_sr_read_str(sr, 256, gd->grid_name);

  /* NanoVDB Map is 264 bytes: float matrix/inverse/translation/taper,
     then double matrix/inverse/translation/taper. Keep the double affine
     transform in Tesseract's row-major 3x4 representation. */
  tvdb__nnvdb_sr_skip(sr, 88);
  for (int row = 0; row < 3; ++row)
    for (int column = 0; column < 3; ++column)
      gd->map[4 * row + column] = tvdb__nnvdb_sr_read_double(sr);
  tvdb__nnvdb_sr_skip(sr, 72);
  for (int row = 0; row < 3; ++row)
    gd->map[4 * row + 3] = tvdb__nnvdb_sr_read_double(sr);
  tvdb__nnvdb_sr_skip(sr, 8);
  for (int i = 0; i < 3; i++)
    gd->world_bbox_min[i] = tvdb__nnvdb_sr_read_double(sr);
  for (int i = 0; i < 3; i++)
    gd->world_bbox_max[i] = tvdb__nnvdb_sr_read_double(sr);
  for (int i = 0; i < 3; i++)
    gd->voxel_size[i] = tvdb__nnvdb_sr_read_double(sr);

  gd->grid_class = tvdb__nnvdb_sr_read_u32(sr);
  gd->grid_type = tvdb__nnvdb_sr_read_u32(sr);
  gd->blind_metadata_offset = tvdb__nnvdb_sr_read_i64(sr);
  gd->blind_metadata_count = tvdb__nnvdb_sr_read_u32(sr);
  gd->data0 = tvdb__nnvdb_sr_read_u32(sr);
  gd->data1 = tvdb__nnvdb_sr_read_u64(sr);
  gd->data2 = tvdb__nnvdb_sr_read_u64(sr);

  return TVDB_OK;
}

static tvdb_status_t tvdb__nnvdb_read_tree_data(tvdb__nnvdb_sr_t* sr, tvdb_nanovdb_tree_data_t* td, tvdb_error_t* err)
{
  for (int i = 0; i < 4; i++)
    td->node_offset[i] = tvdb__nnvdb_sr_read_i64(sr);
  for (int i = 0; i < 3; i++)
    td->node_count[i] = tvdb__nnvdb_sr_read_u32(sr);
  for (int i = 0; i < 3; i++)
    td->tile_count[i] = tvdb__nnvdb_sr_read_u32(sr);
  td->voxel_count = tvdb__nnvdb_sr_read_u64(sr);

  return TVDB_OK;
}

/* ========================================================================== */
/*  Public API implementation                                                  */
/* ========================================================================== */

tvdb_status_t tvdb_nanovdb_file_open(tvdb_nanovdb_file_t* file,
                                     const char* filepath_utf8,
                                     const tvdb_allocator_t* alloc,
                                     tvdb_error_t* err)
{
  if (!file || !filepath_utf8)
  {
    tvdb__nnvdb_set_error(err, TVDB_ERROR_INVALID_ARGUMENT, "Invalid argument");
    return TVDB_ERROR_INVALID_ARGUMENT;
  }

  memset(file, 0, sizeof(*file));

  if (alloc)
  {
    file->alloc = *alloc;
  }
  else
  {
    file->alloc.malloc_fn = (void* (*)(size_t, void*))malloc;
    file->alloc.realloc_fn = (void* (*)(void*, size_t, size_t, void*))realloc;
    file->alloc.free_fn = (void (*)(void*, size_t, void*))free;
    file->alloc.user_ctx = NULL;
  }

#if !defined(TVDB_NO_MMAP) && !defined(_WIN32)
  int fd = open(filepath_utf8, O_RDONLY);
  if (fd >= 0)
  {
    struct stat st;
    if (fstat(fd, &st) == 0)
    {
      file->file_size = (uint64_t)st.st_size;
      file->mmap_data = (const uint8_t*)mmap(NULL, st.st_size, PROT_READ, MAP_PRIVATE, fd, 0);
      if (file->mmap_data)
      {
        close(fd);
        return tvdb_nanovdb_file_open_memory(file, file->mmap_data, file->file_size, alloc, err);
      }
    }
    close(fd);
  }
#endif

  FILE* fp = fopen(filepath_utf8, "rb");
  if (!fp)
  {
    tvdb__nnvdb_set_error(err, TVDB_ERROR_IO, "Failed to open file");
    return TVDB_ERROR_IO;
  }

  fseek(fp, 0, SEEK_END);
  long fsize = ftell(fp);
  fseek(fp, 0, SEEK_SET);

  uint8_t* data = (uint8_t*)malloc((size_t)fsize);
  if (!data)
  {
    fclose(fp);
    tvdb__nnvdb_set_error(err, TVDB_ERROR_OUT_OF_MEMORY, "OOM");
    return TVDB_ERROR_OUT_OF_MEMORY;
  }

  if (fread(data, 1, (size_t)fsize, fp) != (size_t)fsize)
  {
    free(data);
    fclose(fp);
    tvdb__nnvdb_set_error(err, TVDB_ERROR_IO, "Failed to read file");
    return TVDB_ERROR_IO;
  }
  fclose(fp);

  file->buffer = data;
  tvdb_status_t st = tvdb_nanovdb_file_open_memory(file, data, (size_t)fsize, alloc, err);
  return st;
}

tvdb_status_t tvdb_nanovdb_file_open_memory(tvdb_nanovdb_file_t* file,
                                            const uint8_t* data,
                                            size_t data_len,
                                            const tvdb_allocator_t* alloc,
                                            tvdb_error_t* err)
{
  if (!file || !data)
  {
    tvdb__nnvdb_set_error(err, TVDB_ERROR_INVALID_ARGUMENT, "Invalid argument");
    return TVDB_ERROR_INVALID_ARGUMENT;
  }

  memset(file, 0, sizeof(*file));
  file->file_size = data_len;

  if (alloc)
  {
    file->alloc = *alloc;
  }
  else
  {
    file->alloc.malloc_fn = (void* (*)(size_t, void*))malloc;
    file->alloc.realloc_fn = (void* (*)(void*, size_t, size_t, void*))realloc;
    file->alloc.free_fn = (void (*)(void*, size_t, void*))free;
    file->alloc.user_ctx = NULL;
  }

  tvdb__nnvdb_sr_t sr;
  tvdb__nnvdb_sr_init(&sr, data, data_len, 0);

  tvdb_nanovdb_file_header_t hdr;
  tvdb_status_t st = tvdb__nnvdb_read_header(&sr, &hdr, err);
  if (st != TVDB_OK)
    return st;

  file->version = hdr.version;
  file->grid_count = hdr.grid_count;
  file->codec = hdr.codec;

  if (hdr.magic == TVDB_NANOVDB_MAGIC_GRID)
  {
    file->num_grids = 1;
    file->grids = (tvdb_nanovdb_grid_t*)calloc(1, sizeof(tvdb_nanovdb_grid_t));
    if (!file->grids)
    {
      tvdb__nnvdb_set_error(err, TVDB_ERROR_OUT_OF_MEMORY, "OOM");
      return TVDB_ERROR_OUT_OF_MEMORY;
    }
    file->grids[0].data = (uint8_t*)data;
    file->grids[0].size = data_len;
    file->grids[0].owns_data = 0;
    return TVDB_OK;
  }

  file->num_grids = hdr.grid_count;
  file->grids = (tvdb_nanovdb_grid_t*)calloc(file->num_grids, sizeof(tvdb_nanovdb_grid_t));
  if (!file->grids)
  {
    tvdb__nnvdb_set_error(err, TVDB_ERROR_OUT_OF_MEMORY, "OOM");
    return TVDB_ERROR_OUT_OF_MEMORY;
  }

  for (uint16_t i = 0; i < hdr.grid_count; i++)
  {
    tvdb_nanovdb_file_meta_t meta;
    if (tvdb__nnvdb_read_file_meta(&sr, &meta, err) != TVDB_OK)
      return TVDB_ERROR_INVALID_DATA;

    file->grids[i].name = (char*)malloc(meta.name_size + 1);
    if (file->grids[i].name)
    {
      tvdb__nnvdb_sr_read(&sr, meta.name_size, file->grids[i].name);
      file->grids[i].name[meta.name_size] = '\0';
    }
    /* NanoVDB stores a null terminator after each variable-length name. */
    if (!tvdb__nnvdb_sr_skip(&sr, 1))
      return TVDB_ERROR_INVALID_DATA;

    file->grids[i].grid_type = meta.grid_type;
    file->grids[i].grid_class = meta.grid_class;
    file->grids[i].voxel_size[0] = meta.voxel_size[0];
    file->grids[i].voxel_size[1] = meta.voxel_size[1];
    file->grids[i].voxel_size[2] = meta.voxel_size[2];
    file->grids[i].world_bbox_min[0] = meta.world_bbox_min[0];
    file->grids[i].world_bbox_min[1] = meta.world_bbox_min[1];
    file->grids[i].world_bbox_min[2] = meta.world_bbox_min[2];
    file->grids[i].world_bbox_max[0] = meta.world_bbox_max[0];
    file->grids[i].world_bbox_max[1] = meta.world_bbox_max[1];
    file->grids[i].world_bbox_max[2] = meta.world_bbox_max[2];
    file->grids[i].index_bbox_min[0] = meta.index_bbox_min[0];
    file->grids[i].index_bbox_min[1] = meta.index_bbox_min[1];
    file->grids[i].index_bbox_min[2] = meta.index_bbox_min[2];
    file->grids[i].index_bbox_max[0] = meta.index_bbox_max[0];
    file->grids[i].index_bbox_max[1] = meta.index_bbox_max[1];
    file->grids[i].index_bbox_max[2] = meta.index_bbox_max[2];
    file->grids[i].active_voxel_count = meta.voxel_count;
    file->grids[i].node_count[0] = meta.node_count[0];
    file->grids[i].node_count[1] = meta.node_count[1];
    file->grids[i].node_count[2] = meta.node_count[2];
    file->grids[i].node_count[3] = meta.node_count[3];
    file->grids[i].tile_count[0] = meta.tile_count[0];
    file->grids[i].tile_count[1] = meta.tile_count[1];
    file->grids[i].tile_count[2] = meta.tile_count[2];
    file->grids[i].size = meta.grid_size;
  }

  for (uint16_t i = 0; i < hdr.grid_count; i++)
  {
    uint64_t grid_data_start = tvdb__nnvdb_sr_pos(&sr);
    uint64_t grid_size = file->grids[i].size;

    tvdb__nnvdb_sr_t gr;
    tvdb__nnvdb_sr_init(&gr, sr.data + grid_data_start, sr.length - grid_data_start, sr.swap_endian);

    tvdb_nanovdb_grid_data_t gd;
    if (tvdb__nnvdb_read_grid_data(&gr, &gd, err) != TVDB_OK)
      return TVDB_ERROR_INVALID_DATA;

    tvdb_nanovdb_tree_data_t td;
    if (tvdb__nnvdb_read_tree_data(&gr, &td, err) != TVDB_OK)
      return TVDB_ERROR_INVALID_DATA;

    if (hdr.codec != TVDB_NANOVDB_CODEC_NONE && grid_size > 0)
    {
      uint64_t compressed_size = tvdb__nnvdb_sr_read_u64(&sr);

      uint8_t* decompressed = (uint8_t*)malloc((size_t)grid_size);
      if (!decompressed)
      {
        tvdb__nnvdb_set_error(err, TVDB_ERROR_OUT_OF_MEMORY, "OOM");
        return TVDB_ERROR_OUT_OF_MEMORY;
      }

      if (compressed_size > 0 && compressed_size < (1ULL << 30))
      {
        uint8_t* compressed = (uint8_t*)malloc((size_t)compressed_size);
        if (!compressed)
        {
          free(decompressed);
          tvdb__nnvdb_set_error(err, TVDB_ERROR_OUT_OF_MEMORY, "OOM");
          return TVDB_ERROR_OUT_OF_MEMORY;
        }

        tvdb__nnvdb_sr_read(&sr, (size_t)compressed_size, compressed);

        tvdb_nanovdb_codec_t codec =
            (hdr.codec == TVDB_NANOVDB_CODEC_BLOSC) ? TVDB_NANOVDB_CODEC_BLOSC : TVDB_NANOVDB_CODEC_ZIP;

        if (!tvdb__nnvdb_decompress(decompressed, (size_t)grid_size, compressed, (size_t)compressed_size, codec))
        {
          free(compressed);
          free(decompressed);
          tvdb__nnvdb_set_error(err, TVDB_ERROR_DECOMPRESSION_FAILED, "Failed to decompress grid data");
          return TVDB_ERROR_DECOMPRESSION_FAILED;
        }
        free(compressed);
      }
      else
      {
        tvdb__nnvdb_sr_read(&sr, (size_t)grid_size, decompressed);
      }

      file->grids[i].data = decompressed;
      file->grids[i].size = grid_size;
      file->grids[i].owns_data = 1;
    }
    else
    {
      file->grids[i].data = (uint8_t*)(sr.data + grid_data_start);
      file->grids[i].owns_data = 0;
    }

    file->grids[i].tree_data_offset = 672;
    file->grids[i].leaf_data_offset = td.node_offset[0];
    file->grids[i].lower_data_offset = td.node_offset[1];
    file->grids[i].upper_data_offset = td.node_offset[2];
    file->grids[i].root_data_offset = td.node_offset[3];
    for (int m = 0; m < 12; ++m)
      file->grids[i].map[m] = gd.map[m];
  }

  return TVDB_OK;
}

void tvdb_nanovdb_file_close(tvdb_nanovdb_file_t* file)
{
  if (!file)
    return;

  for (size_t i = 0; i < file->num_grids; i++)
  {
    if (file->grids[i].name)
      free(file->grids[i].name);
    if (file->grids[i].owns_data && file->grids[i].data)
      free(file->grids[i].data);
  }
  if (file->grids)
    free(file->grids);

  if (file->buffer)
    free(file->buffer);

#if !defined(TVDB_NO_MMAP) && !defined(_WIN32)
  if (file->mmap_data)
    munmap((void*)file->mmap_data, file->file_size);
#endif

  memset(file, 0, sizeof(*file));
}

size_t tvdb_nanovdb_grid_count(const tvdb_nanovdb_file_t* file) { return file ? file->num_grids : 0; }

const char* tvdb_nanovdb_grid_name(const tvdb_nanovdb_file_t* file, size_t idx)
{
  if (!file || idx >= file->num_grids)
    return NULL;
  return file->grids[idx].name ? file->grids[idx].name : "";
}

uint32_t tvdb_nanovdb_grid_type(const tvdb_nanovdb_file_t* file, size_t idx)
{
  if (!file || idx >= file->num_grids)
    return 0;
  return file->grids[idx].grid_type;
}

uint32_t tvdb_nanovdb_grid_class(const tvdb_nanovdb_file_t* file, size_t idx)
{
  if (!file || idx >= file->num_grids)
    return 0;
  return file->grids[idx].grid_class;
}

double tvdb_nanovdb_grid_voxel_size(const tvdb_nanovdb_file_t* file, size_t idx, int axis)
{
  if (!file || idx >= file->num_grids || axis < 0 || axis > 2)
    return 0.0;
  return file->grids[idx].voxel_size[axis];
}

/* Real Root → Upper → Lower → Leaf accessor backed by PNanoVDB.h. */
static pnanovdb_buf_t tvdb__nanovdb_make_buf(const tvdb_nanovdb_grid_t* grid)
{
  /* PNanoVDB treats the buffer as uint32_t* with grid handle at address 0.
     Grid->data is byte-aligned (mmap or malloc), so casting to uint32_t*
     is safe — NanoVDB requires 8-byte alignment minimum. */
  return pnanovdb_make_buf((uint32_t*)grid->data,
                           /*size_in_words=*/(uint64_t)(grid->size / 4));
}

float tvdb_nanovdb_get_voxel_f(const tvdb_nanovdb_grid_t* grid, int x, int y, int z)
{
  if (!grid || !grid->data)
    return 0.0f;
  if (grid->grid_type != TVDB_NANOVDB_GRID_TYPE_FLOAT)
    return 0.0f;

  pnanovdb_buf_t buf = tvdb__nanovdb_make_buf(grid);
  pnanovdb_grid_handle_t gh = { { 0u } };
  pnanovdb_root_handle_t root = pnanovdb_tree_get_root(buf, pnanovdb_grid_get_tree(buf, gh));
  pnanovdb_coord_t ijk = { x, y, z };
  pnanovdb_address_t addr = pnanovdb_root_get_value_address(PNANOVDB_GRID_TYPE_FLOAT, buf, root, PNANOVDB_REF(ijk));
  return pnanovdb_read_float(buf, addr);
}

double tvdb_nanovdb_get_voxel_d(const tvdb_nanovdb_grid_t* grid, int x, int y, int z)
{
  if (!grid || !grid->data)
    return 0.0;
  if (grid->grid_type != TVDB_NANOVDB_GRID_TYPE_DOUBLE)
    return 0.0;

  pnanovdb_buf_t buf = tvdb__nanovdb_make_buf(grid);
  pnanovdb_grid_handle_t gh = { { 0u } };
  pnanovdb_root_handle_t root = pnanovdb_tree_get_root(buf, pnanovdb_grid_get_tree(buf, gh));
  pnanovdb_coord_t ijk = { x, y, z };
  pnanovdb_address_t addr = pnanovdb_root_get_value_address(PNANOVDB_GRID_TYPE_DOUBLE, buf, root, PNANOVDB_REF(ijk));
  return pnanovdb_read_double(buf, addr);
}

float tvdb_nanovdb_sample_trilinear_f(const tvdb_nanovdb_grid_t* grid, float ix, float iy, float iz)
{
  if (!grid || !grid->data)
    return 0.0f;
  if (grid->grid_type != TVDB_NANOVDB_GRID_TYPE_FLOAT)
    return 0.0f;

  /* Cell-center convention: voxel (i,j,k) is centered at (i+0.5, j+0.5,
     k+0.5). Subtract 0.5 to get the lattice coordinate. */
  float fx = ix - 0.5f, fy = iy - 0.5f, fz = iz - 0.5f;
  int x0 = (int)floorf(fx), y0 = (int)floorf(fy), z0 = (int)floorf(fz);
  float tx = fx - (float)x0, ty = fy - (float)y0, tz = fz - (float)z0;

  pnanovdb_buf_t buf = tvdb__nanovdb_make_buf(grid);
  pnanovdb_grid_handle_t gh = { { 0u } };
  pnanovdb_root_handle_t root = pnanovdb_tree_get_root(buf, pnanovdb_grid_get_tree(buf, gh));

  float v[2][2][2];
  for (int dz = 0; dz < 2; ++dz)
  {
    for (int dy = 0; dy < 2; ++dy)
    {
      for (int dx = 0; dx < 2; ++dx)
      {
        pnanovdb_coord_t ijk = { x0 + dx, y0 + dy, z0 + dz };
        pnanovdb_address_t addr =
            pnanovdb_root_get_value_address(PNANOVDB_GRID_TYPE_FLOAT, buf, root, PNANOVDB_REF(ijk));
        v[dz][dy][dx] = pnanovdb_read_float(buf, addr);
      }
    }
  }
  float c00 = v[0][0][0] * (1 - tx) + v[0][0][1] * tx;
  float c01 = v[0][1][0] * (1 - tx) + v[0][1][1] * tx;
  float c10 = v[1][0][0] * (1 - tx) + v[1][0][1] * tx;
  float c11 = v[1][1][0] * (1 - tx) + v[1][1][1] * tx;
  float c0 = c00 * (1 - ty) + c01 * ty;
  float c1 = c10 * (1 - ty) + c11 * ty;
  return c0 * (1 - tz) + c1 * tz;
}

tvdb_status_t
tvdb_nanovdb_index_to_world(const tvdb_nanovdb_grid_t* grid, double ix, double iy, double iz, double out[3])
{
  if (!grid || !out)
    return TVDB_ERROR_INVALID_ARGUMENT;
  /* world = M * index + t. Row-major 3x4. */
  out[0] = grid->map[0] * ix + grid->map[1] * iy + grid->map[2] * iz + grid->map[3];
  out[1] = grid->map[4] * ix + grid->map[5] * iy + grid->map[6] * iz + grid->map[7];
  out[2] = grid->map[8] * ix + grid->map[9] * iy + grid->map[10] * iz + grid->map[11];
  return TVDB_OK;
}

tvdb_status_t
tvdb_nanovdb_world_to_index(const tvdb_nanovdb_grid_t* grid, double wx, double wy, double wz, double out[3])
{
  if (!grid || !out)
    return TVDB_ERROR_INVALID_ARGUMENT;
  const double* m = grid->map;
  /* Cofactor expansion of the 3x3 linear part. */
  double a = m[0], b = m[1], c = m[2];
  double d = m[4], e = m[5], f = m[6];
  double g = m[8], h = m[9], i = m[10];
  double det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
  if (det == 0.0 || (det > -1e-30 && det < 1e-30))
    return TVDB_ERROR_INVALID_DATA;
  double inv = 1.0 / det;
  /* Adjugate-transpose / det. */
  double inv00 = (e * i - f * h) * inv;
  double inv01 = (c * h - b * i) * inv;
  double inv02 = (b * f - c * e) * inv;
  double inv10 = (f * g - d * i) * inv;
  double inv11 = (a * i - c * g) * inv;
  double inv12 = (c * d - a * f) * inv;
  double inv20 = (d * h - e * g) * inv;
  double inv21 = (b * g - a * h) * inv;
  double inv22 = (a * e - b * d) * inv;
  double dx = wx - m[3], dy = wy - m[7], dz = wz - m[11];
  out[0] = inv00 * dx + inv01 * dy + inv02 * dz;
  out[1] = inv10 * dx + inv11 * dy + inv12 * dz;
  out[2] = inv20 * dx + inv21 * dy + inv22 * dz;
  return TVDB_OK;
}

int tvdb_nanovdb_is_voxel_active(const tvdb_nanovdb_grid_t* grid, int x, int y, int z)
{
  if (!grid || !grid->data)
    return 0;
  /* Quick bbox reject: voxels outside the active bbox are guaranteed
     inactive. PNanoVDB would return false at higher levels too, but the
     bbox check avoids descending the tree for the common all-outside
     case in renderers. */
  if (x < grid->index_bbox_min[0] || x > grid->index_bbox_max[0] || y < grid->index_bbox_min[1] ||
      y > grid->index_bbox_max[1] || z < grid->index_bbox_min[2] || z > grid->index_bbox_max[2])
  {
    return 0;
  }
  pnanovdb_buf_t buf = tvdb__nanovdb_make_buf(grid);
  pnanovdb_grid_handle_t gh = { { 0u } };
  pnanovdb_root_handle_t root = pnanovdb_tree_get_root(buf, pnanovdb_grid_get_tree(buf, gh));
  pnanovdb_readaccessor_t acc;
  pnanovdb_readaccessor_init(PNANOVDB_REF(acc), root);
  pnanovdb_coord_t ijk = { x, y, z };
  return pnanovdb_readaccessor_is_active(grid->grid_type, buf, PNANOVDB_REF(acc), PNANOVDB_REF(ijk)) ? 1 : 0;
}

/* ----- CRC32 + grid checksum (matches NanoVDB tools/GridChecksum.h) ----- */

uint32_t tvdb_nanovdb_crc32(const void* data, size_t size, uint32_t crc)
{
  if (!data)
    return crc;
  crc = ~crc;
  const uint8_t* p = (const uint8_t*)data;
  for (size_t i = 0; i < size; ++i)
  {
    crc ^= p[i];
    for (int j = 0; j < 8; ++j)
    {
      crc = (crc >> 1) ^ (0xEDB88320u & (uint32_t) - (int32_t)(crc & 1u));
    }
  }
  return ~crc;
}

#define TVDB__NVDB_GRID_DATA_BYTES 672u
#define TVDB__NVDB_TREE_DATA_BYTES 64u
#define TVDB__NVDB_CRC32_BLOCK_LOG2 12 /* 4 KB blocks */

/* Decode the grid's encoded version word into (major, minor, patch). The
 * NanoVDB encoding is `(major << 21) | (minor << 10) | patch` for v32.6.0+
 * and `(major << 24) | (minor << 16) | patch_low_16` for older. We follow
 * the modern encoding here since we already require v32+. */
static int tvdb__nvdb_version_gt_32_6_0(uint32_t enc)
{
  uint32_t major = (enc >> 21) & 0x7FFu;
  uint32_t minor = (enc >> 10) & 0x7FFu;
  if (major > 32u)
    return 1;
  if (major == 32u && minor > 6u)
    return 1;
  return 0;
}

uint32_t tvdb_nanovdb_compute_head_checksum(const tvdb_nanovdb_grid_t* grid)
{
  if (!grid || !grid->data || grid->size < 16u + TVDB__NVDB_TREE_DATA_BYTES)
  {
    return 0u;
  }
  /* Read the encoded version from the grid's GridData (offset 16). */
  uint32_t enc;
  memcpy(&enc, grid->data + 16, 4);
  if (!tvdb__nvdb_version_gt_32_6_0(enc))
    return 0u;

  size_t head_end = TVDB__NVDB_GRID_DATA_BYTES + TVDB__NVDB_TREE_DATA_BYTES;
  if (head_end > grid->size)
    return 0u;
  return tvdb_nanovdb_crc32(grid->data + 16, head_end - 16, 0u);
}

uint32_t tvdb_nanovdb_compute_tail_checksum(const tvdb_nanovdb_grid_t* grid)
{
  if (!grid || !grid->data)
    return 0u;
  uint32_t enc;
  memcpy(&enc, grid->data + 16, 4);
  if (!tvdb__nvdb_version_gt_32_6_0(enc))
    return 0u;

  size_t head_end = TVDB__NVDB_GRID_DATA_BYTES + TVDB__NVDB_TREE_DATA_BYTES;
  if (grid->size <= head_end)
    return 0u;
  size_t tail_size = grid->size - head_end;
  const uint8_t* tail = grid->data + head_end;

  /* blockedCrc32: 4KB blocks, then CRC32 of the resulting CRCs. The last
   * block absorbs the leftover bytes (matching NanoVDB's behavior of
   * extending blockSize on the final iteration when blockCount<<LOG2 <
   * size). If size < 4096, blockCount==0 and there's nothing to hash. */
  size_t block_size = (size_t)1 << TVDB__NVDB_CRC32_BLOCK_LOG2;
  size_t block_count = tail_size >> TVDB__NVDB_CRC32_BLOCK_LOG2;
  if (block_count == 0)
    return ~(uint32_t)0;

  uint32_t* checksums = (uint32_t*)malloc(block_count * sizeof(uint32_t));
  if (!checksums)
    return 0u;
  for (size_t i = 0; i < block_count; ++i)
  {
    size_t bs = block_size;
    if (i + 1 == block_count)
    {
      bs += tail_size - (block_count << TVDB__NVDB_CRC32_BLOCK_LOG2);
    }
    checksums[i] = tvdb_nanovdb_crc32(tail + (i << TVDB__NVDB_CRC32_BLOCK_LOG2), bs, 0u);
  }
  uint32_t out = tvdb_nanovdb_crc32(checksums, block_count * sizeof(uint32_t), 0u);
  free(checksums);
  return out;
}

int tvdb_nanovdb_validate_checksum(const tvdb_nanovdb_grid_t* grid, tvdb_nanovdb_checksum_mode_t mode)
{
  if (!grid || !grid->data)
    return 0;
  uint64_t stored;
  memcpy(&stored, grid->data + 8, 8);

  /* Empty/uninitialized checksum sentinel is all-1s. NanoVDB treats this
   * as "skip validation". */
  if (stored == 0xFFFFFFFFFFFFFFFFull)
    return 1;
  if (mode == TVDB_NANOVDB_CHECKSUM_NONE)
    return 1;

  uint32_t stored_head = (uint32_t)(stored & 0xFFFFFFFFu);
  uint32_t stored_tail = (uint32_t)((stored >> 32) & 0xFFFFFFFFu);

  uint32_t head = tvdb_nanovdb_compute_head_checksum(grid);
  if (head != stored_head)
    return 0;

  /* If the stored tail half is 0xFFFFFFFF, the file was written in
   * "Half" mode (head-only). Don't fail on tail mismatch in that case. */
  if (stored_tail == 0xFFFFFFFFu || mode == TVDB_NANOVDB_CHECKSUM_EASTWOOD)
  {
    return 1;
  }
  uint32_t tail = tvdb_nanovdb_compute_tail_checksum(grid);
  return tail == stored_tail ? 1 : 0;
}

const char* tvdb_nanovdb_grid_type_name(uint32_t grid_type)
{
  switch (grid_type)
  {
    case TVDB_NANOVDB_GRID_TYPE_FLOAT:
      return "Float";
    case TVDB_NANOVDB_GRID_TYPE_DOUBLE:
      return "Double";
    case TVDB_NANOVDB_GRID_TYPE_INT16:
      return "Int16";
    case TVDB_NANOVDB_GRID_TYPE_INT32:
      return "Int32";
    case TVDB_NANOVDB_GRID_TYPE_INT64:
      return "Int64";
    case TVDB_NANOVDB_GRID_TYPE_VEC3F:
      return "Vec3f";
    case TVDB_NANOVDB_GRID_TYPE_VEC3D:
      return "Vec3d";
    case TVDB_NANOVDB_GRID_TYPE_MASK:
      return "Mask";
    case TVDB_NANOVDB_GRID_TYPE_HALF:
      return "Half";
    case TVDB_NANOVDB_GRID_TYPE_UINT32:
      return "UInt32";
    case TVDB_NANOVDB_GRID_TYPE_BOOLEAN:
      return "Boolean";
    case TVDB_NANOVDB_GRID_TYPE_RGBA8:
      return "RGBA8";
    case TVDB_NANOVDB_GRID_TYPE_FP4:
      return "Fp4";
    case TVDB_NANOVDB_GRID_TYPE_FP8:
      return "Fp8";
    case TVDB_NANOVDB_GRID_TYPE_FP16:
      return "Fp16";
    case TVDB_NANOVDB_GRID_TYPE_FPN:
      return "FpN";
    case TVDB_NANOVDB_GRID_TYPE_VEC4F:
      return "Vec4f";
    case TVDB_NANOVDB_GRID_TYPE_VEC4D:
      return "Vec4d";
    case TVDB_NANOVDB_GRID_TYPE_INDEX:
      return "Index";
    case TVDB_NANOVDB_GRID_TYPE_POINT_INDEX:
      return "PointIndex";
    default:
      return "Unknown";
  }
}

const char* tvdb_nanovdb_grid_class_name(uint32_t grid_class)
{
  switch (grid_class)
  {
    case TVDB_NANOVDB_GRID_CLASS_LEVEL_SET:
      return "LevelSet";
    case TVDB_NANOVDB_GRID_CLASS_FOG_VOLUME:
      return "FogVolume";
    case TVDB_NANOVDB_GRID_CLASS_STAGGERED:
      return "Staggered";
    case TVDB_NANOVDB_GRID_CLASS_COLLISION:
      return "Collision";
    case TVDB_NANOVDB_GRID_CLASS_POINT_INDEX:
      return "PointIndex";
    case TVDB_NANOVDB_GRID_CLASS_POINT_DATA:
      return "PointData";
    default:
      return "Unknown";
  }
}

int tvdb_nanovdb_is_big_endian_file(const tvdb_nanovdb_file_t* file)
{
  (void)file;
  return !tvdb__nnvdb_is_little_endian();
}

/* ========================================================================== */
/*  BLOSC compression for NanoVDB write                                        */
/* ========================================================================== */

static int tvdb__nnvdb_compress_blosc(void** dst, size_t* dst_size, const void* src, size_t src_size)
{
  if (!src || !dst)
    return 0;

  size_t max_size = (size_t)LZ4_compressBound((int)src_size) + 12;
  *dst = malloc(max_size);
  if (!*dst)
    return 0;

  uint8_t* out = (uint8_t*)*dst;

  out[0] = 0;
  out[1] = (src_size & 0xFF);
  out[2] = (src_size >> 8) & 0xFF;
  out[3] = (src_size >> 16) & 0xFF;
  out[4] = (src_size >> 24) & 0xFF;

  int compressed = LZ4_compress_default((const char*)src, (char*)out + 12, (int)src_size, (int)(max_size - 12));
  if (compressed <= 0)
  {
    free(*dst);
    *dst = NULL;
    return 0;
  }

  *dst_size = (size_t)compressed + 12;
  return 1;
}

/* ========================================================================== */
/*  ZIP compression for NanoVDB write                                           */
/* ========================================================================== */

static int tvdb__nnvdb_compress_zip(void** dst, size_t* dst_size, const void* src, size_t src_size)
{
  if (!src || !dst)
    return 0;

#if !defined(TVDB_USE_SYSTEM_ZLIB)
  mz_ulong dest_size = mz_compressBound((mz_ulong)src_size);
  *dst = malloc(dest_size);
  if (!*dst)
    return 0;

  int rc = mz_compress((uint8_t*)*dst, &dest_size, src, src_size);
  if (rc != MZ_OK)
  {
    free(*dst);
    *dst = NULL;
    return 0;
  }
  *dst_size = (size_t)dest_size;
  return 1;
#else
  uLongf dest_size = compressBound((uLongf)src_size);
  *dst = malloc(dest_size);
  if (!*dst)
    return 0;

  int rc = compress((uint8_t*)*dst, &dest_size, src, src_size);
  if (rc != Z_OK)
  {
    free(*dst);
    *dst = NULL;
    return 0;
  }
  *dst_size = (size_t)dest_size;
  return 1;
#endif
}

/* ========================================================================== */
/*  Internal write helpers                                                     */
/* ========================================================================== */

/* Grid data constants */
#define TVDB_NANOVDB_GRID_DATA_SIZE 672
#define TVDB_NANOVDB_TREE_DATA_SIZE 64
#define TVDB_NANOVDB_ROOT_DATA_ALIGN 32
#define TVDB_NANOVDB_LEAF_NODE_ALIGN 32

static void tvdb__nnvdb_write_file_header(tvdb__nnvdb_sw_t* sw, const tvdb_nanovdb_file_t* file)
{
  tvdb__nnvdb_sw_write_u64(sw, TVDB_NANOVDB_MAGIC_FILE);
  uint32_t version =
      (TVDB_NANOVDB_VERSION_MAJOR << 24) | (TVDB_NANOVDB_VERSION_MINOR << 16) | TVDB_NANOVDB_VERSION_PATCH;
  tvdb__nnvdb_sw_write_u32(sw, version);
  tvdb__nnvdb_sw_write_u16(sw, (uint16_t)file->grid_count);
  tvdb__nnvdb_sw_write_u16(sw, (uint16_t)file->codec);
}

static void tvdb__nnvdb_write_file_meta(tvdb__nnvdb_sw_t* sw, const tvdb_nanovdb_grid_t* grid)
{
  tvdb__nnvdb_sw_write_u64(sw, grid->size);
  tvdb__nnvdb_sw_write_u64(sw, grid->size);
  tvdb__nnvdb_sw_write_u64(sw, 0);
  tvdb__nnvdb_sw_write_u64(sw, grid->active_voxel_count);
  tvdb__nnvdb_sw_write_u32(sw, grid->grid_type);
  tvdb__nnvdb_sw_write_u32(sw, grid->grid_class);

  for (int i = 0; i < 3; i++)
    tvdb__nnvdb_sw_write_double(sw, grid->world_bbox_min[i]);
  for (int i = 0; i < 3; i++)
    tvdb__nnvdb_sw_write_double(sw, grid->world_bbox_max[i]);
  for (int i = 0; i < 3; i++)
    tvdb__nnvdb_sw_write_i32(sw, grid->index_bbox_min[i]);
  for (int i = 0; i < 3; i++)
    tvdb__nnvdb_sw_write_i32(sw, grid->index_bbox_max[i]);
  for (int i = 0; i < 3; i++)
    tvdb__nnvdb_sw_write_double(sw, grid->voxel_size[i]);

  size_t name_len = grid->name ? strlen(grid->name) : 0;
  tvdb__nnvdb_sw_write_u32(sw, (uint32_t)name_len);

  for (int i = 0; i < 4; i++)
    tvdb__nnvdb_sw_write_u32(sw, grid->node_count[i]);
  for (int i = 0; i < 3; i++)
    tvdb__nnvdb_sw_write_u32(sw, grid->tile_count[i]);

  tvdb__nnvdb_sw_write_u16(sw, 0);
  tvdb__nnvdb_sw_write_u16(sw, 0);

  uint32_t version =
      (TVDB_NANOVDB_VERSION_MAJOR << 24) | (TVDB_NANOVDB_VERSION_MINOR << 16) | TVDB_NANOVDB_VERSION_PATCH;
  tvdb__nnvdb_sw_write_u32(sw, version);
}

static void tvdb__nnvdb_write_grid_name(tvdb__nnvdb_sw_t* sw, const char* name)
{
  size_t len = name ? strlen(name) : 0;
  if (len > 0)
  {
    tvdb__nnvdb_sw_write(sw, len, name);
  }
  tvdb__nnvdb_sw_write_u8(sw, 0);
}

static int tvdb__nnvdb_align(tvdb__nnvdb_sw_t* sw, size_t alignment)
{
  size_t pos = sw->pos;
  size_t mod = pos % alignment;
  if (mod > 0)
  {
    size_t pad = alignment - mod;
    if (sw->pos + pad > sw->capacity)
      return 0;
    for (size_t i = 0; i < pad; i++)
    {
      sw->data[sw->pos++] = 0;
    }
  }
  return 1;
}

/* ========================================================================== */
/*  Write implementations                                                      */
/* ========================================================================== */

tvdb_status_t tvdb_nanovdb_write_to_memory(const tvdb_nanovdb_file_t* file,
                                           uint32_t compression_flags,
                                           uint8_t** out_data,
                                           size_t* out_size,
                                           tvdb_error_t* err)
{
  if (!file || !out_data || !out_size)
  {
    tvdb__nnvdb_set_error(err, TVDB_ERROR_INVALID_ARGUMENT, "Invalid argument");
    return TVDB_ERROR_INVALID_ARGUMENT;
  }

  *out_data = NULL;
  *out_size = 0;

  if (file->num_grids == 0)
  {
    tvdb__nnvdb_set_error(err, TVDB_ERROR_INVALID_DATA, "No grids to write");
    return TVDB_ERROR_INVALID_DATA;
  }

  tvdb_nanovdb_codec_t codec = TVDB_NANOVDB_CODEC_NONE;
  if (compression_flags & TVDB_NANOVDB_CODEC_BLOSC)
  {
    codec = TVDB_NANOVDB_CODEC_BLOSC;
  }
  else if (compression_flags & TVDB_NANOVDB_CODEC_ZIP)
  {
    codec = TVDB_NANOVDB_CODEC_ZIP;
  }

  size_t initial_capacity = 1024 * 1024;
  uint8_t* buffer = (uint8_t*)malloc(initial_capacity);
  if (!buffer)
  {
    tvdb__nnvdb_set_error(err, TVDB_ERROR_OUT_OF_MEMORY, "OOM");
    return TVDB_ERROR_OUT_OF_MEMORY;
  }

  tvdb__nnvdb_sw_t sw;
  tvdb__nnvdb_sw_init(&sw, buffer, initial_capacity);

  tvdb_nanovdb_file_t wf;
  memset(&wf, 0, sizeof(wf));
  wf.version = (TVDB_NANOVDB_VERSION_MAJOR << 24) | (TVDB_NANOVDB_VERSION_MINOR << 16) | TVDB_NANOVDB_VERSION_PATCH;
  wf.grid_count = (uint16_t)file->num_grids;
  wf.codec = codec;

  tvdb__nnvdb_write_file_header(&sw, &wf);

  for (size_t i = 0; i < file->num_grids; i++)
  {
    tvdb__nnvdb_write_file_meta(&sw, &file->grids[i]);
    tvdb__nnvdb_write_grid_name(&sw, file->grids[i].name);
  }

  for (size_t i = 0; i < file->num_grids; i++)
  {
    const tvdb_nanovdb_grid_t* grid = &file->grids[i];

    if (!grid->data || grid->size == 0)
    {
      continue;
    }

    if (codec != TVDB_NANOVDB_CODEC_NONE)
    {
      void* compressed = NULL;
      size_t compressed_size = 0;

      int ok = 0;
      if (codec == TVDB_NANOVDB_CODEC_BLOSC)
      {
        ok = tvdb__nnvdb_compress_blosc(&compressed, &compressed_size, grid->data, grid->size);
      }
      else if (codec == TVDB_NANOVDB_CODEC_ZIP)
      {
        ok = tvdb__nnvdb_compress_zip(&compressed, &compressed_size, grid->data, grid->size);
      }

      if (ok && compressed && compressed_size < grid->size)
      {
        tvdb__nnvdb_sw_write_u64(&sw, compressed_size);
        if (sw.pos + compressed_size > sw.capacity)
        {
          size_t new_cap = sw.capacity * 2 + compressed_size;
          uint8_t* new_buf = (uint8_t*)realloc(sw.data, new_cap);
          if (!new_buf)
          {
            free(compressed);
            free(buffer);
            tvdb__nnvdb_set_error(err, TVDB_ERROR_OUT_OF_MEMORY, "OOM");
            return TVDB_ERROR_OUT_OF_MEMORY;
          }
          sw.data = new_buf;
          sw.capacity = new_cap;
        }
        memcpy(sw.data + sw.pos, compressed, compressed_size);
        sw.pos += compressed_size;
        free(compressed);
      }
      else
      {
        if (compressed)
          free(compressed);
        if (sw.pos + grid->size > sw.capacity)
        {
          size_t new_cap = sw.capacity * 2 + grid->size;
          uint8_t* new_buf = (uint8_t*)realloc(sw.data, new_cap);
          if (!new_buf)
          {
            free(buffer);
            tvdb__nnvdb_set_error(err, TVDB_ERROR_OUT_OF_MEMORY, "OOM");
            return TVDB_ERROR_OUT_OF_MEMORY;
          }
          sw.data = new_buf;
          sw.capacity = new_cap;
        }
        memcpy(sw.data + sw.pos, grid->data, grid->size);
        sw.pos += grid->size;
      }
    }
    else
    {
      if (sw.pos + grid->size > sw.capacity)
      {
        size_t new_cap = sw.capacity * 2 + grid->size;
        uint8_t* new_buf = (uint8_t*)realloc(sw.data, new_cap);
        if (!new_buf)
        {
          free(buffer);
          tvdb__nnvdb_set_error(err, TVDB_ERROR_OUT_OF_MEMORY, "OOM");
          return TVDB_ERROR_OUT_OF_MEMORY;
        }
        sw.data = new_buf;
        sw.capacity = new_cap;
      }
      memcpy(sw.data + sw.pos, grid->data, grid->size);
      sw.pos += grid->size;
    }
  }

  *out_data = sw.data;
  *out_size = sw.pos;
  return TVDB_OK;
}

tvdb_status_t tvdb_nanovdb_file_save(const tvdb_nanovdb_file_t* file,
                                     const char* filepath_utf8,
                                     uint32_t compression_flags,
                                     int use_mmap,
                                     tvdb_error_t* err)
{
  (void)use_mmap;

  uint8_t* data = NULL;
  size_t data_size = 0;

  tvdb_status_t st = tvdb_nanovdb_write_to_memory(file, compression_flags, &data, &data_size, err);
  if (st != TVDB_OK)
    return st;

#if defined(_WIN32)
  FILE* fp = fopen(filepath_utf8, "wb");
#else
  FILE* fp = fopen(filepath_utf8, "wb");
#endif
  if (!fp)
  {
    free(data);
    tvdb__nnvdb_set_error(err, TVDB_ERROR_IO, "Failed to open file");
    return TVDB_ERROR_IO;
  }

  if (fwrite(data, 1, data_size, fp) != data_size)
  {
    fclose(fp);
    free(data);
    tvdb__nnvdb_set_error(err, TVDB_ERROR_IO, "Failed to write file");
    return TVDB_ERROR_IO;
  }

  fclose(fp);
  free(data);
  return TVDB_OK;
}

/* ========================================================================== */
/*  Node size calculations                                                     */
/* ========================================================================== */

uint32_t tvdb_nanovdb_value_size(uint32_t grid_type)
{
  switch (grid_type)
  {
    case TVDB_NANOVDB_GRID_TYPE_FLOAT:
    case TVDB_NANOVDB_GRID_TYPE_UINT32:
    case TVDB_NANOVDB_GRID_TYPE_INT32:
      return 4;
    case TVDB_NANOVDB_GRID_TYPE_DOUBLE:
    case TVDB_NANOVDB_GRID_TYPE_INT64:
      return 8;
    case TVDB_NANOVDB_GRID_TYPE_INT16:
    case TVDB_NANOVDB_GRID_TYPE_HALF:
      return 2;
    case TVDB_NANOVDB_GRID_TYPE_VEC3F:
      return 12;
    case TVDB_NANOVDB_GRID_TYPE_VEC3D:
      return 24;
    case TVDB_NANOVDB_GRID_TYPE_BOOLEAN:
    case TVDB_NANOVDB_GRID_TYPE_MASK:
      return 1;
    case TVDB_NANOVDB_GRID_TYPE_RGBA8:
      return 4;
    case TVDB_NANOVDB_GRID_TYPE_VEC4F:
      return 16;
    case TVDB_NANOVDB_GRID_TYPE_VEC4D:
      return 32;
    case TVDB_NANOVDB_GRID_TYPE_UINT8:
      return 1;
    default:
      return 4;
  }
}

uint64_t tvdb_nanovdb_leaf_node_size(uint32_t grid_type)
{
  uint32_t value_size = tvdb_nanovdb_value_size(grid_type);
  uint32_t value_bytes = (value_size <= 4) ? 4 : 8;
  return 32 + 64 + 16 + (uint64_t)(512 * value_bytes);
}

uint64_t tvdb_nanovdb_lower_node_size(uint32_t grid_type)
{
  uint32_t value_size = tvdb_nanovdb_value_size(grid_type);
  uint32_t value_bytes = (value_size <= 4) ? 4 : 8;
  return 32 + 1024 + 1024 + 16 + (uint64_t)(4096 * value_bytes);
}

uint64_t tvdb_nanovdb_upper_node_size(uint32_t grid_type)
{
  uint32_t value_size = tvdb_nanovdb_value_size(grid_type);
  uint32_t value_bytes = (value_size <= 4) ? 4 : 8;
  return 32 + 8192 + 8192 + 16 + (uint64_t)(32768 * value_bytes);
}

uint64_t tvdb_nanovdb_root_tile_size(void) { return 32; }

/* ========================================================================== */
/*  Grid creation and destruction                                             */
/* ========================================================================== */

tvdb_status_t tvdb_nanovdb_create_grid(tvdb_nanovdb_grid_t* grid,
                                       const char* name,
                                       uint32_t grid_type,
                                       uint32_t grid_class,
                                       int32_t min_coord[3],
                                       int32_t max_coord[3],
                                       const tvdb_allocator_t* alloc,
                                       tvdb_error_t* err)
{
  if (!grid || !name)
  {
    tvdb__nnvdb_set_error(err, TVDB_ERROR_INVALID_ARGUMENT, "Invalid argument");
    return TVDB_ERROR_INVALID_ARGUMENT;
  }

  memset(grid, 0, sizeof(*grid));

  grid->name = (char*)malloc(strlen(name) + 1);
  if (!grid->name)
  {
    tvdb__nnvdb_set_error(err, TVDB_ERROR_OUT_OF_MEMORY, "OOM");
    return TVDB_ERROR_OUT_OF_MEMORY;
  }
  strcpy(grid->name, name);

  grid->grid_type = grid_type;
  grid->grid_class = grid_class;

  for (int i = 0; i < 3; i++)
  {
    grid->voxel_size[i] = 1.0;
    grid->index_bbox_min[i] = min_coord[i];
    grid->index_bbox_max[i] = max_coord[i];
    grid->world_bbox_min[i] = (double)min_coord[i];
    grid->world_bbox_max[i] = (double)max_coord[i];
  }
  /* Default map = identity affine (voxel_size = 1, origin at 0). */
  grid->map[0] = 1.0;
  grid->map[1] = 0.0;
  grid->map[2] = 0.0;
  grid->map[3] = 0.0;
  grid->map[4] = 0.0;
  grid->map[5] = 1.0;
  grid->map[6] = 0.0;
  grid->map[7] = 0.0;
  grid->map[8] = 0.0;
  grid->map[9] = 0.0;
  grid->map[10] = 1.0;
  grid->map[11] = 0.0;

  grid->node_count[0] = 0;
  grid->node_count[1] = 0;
  grid->node_count[2] = 0;
  grid->node_count[3] = 1;
  grid->tile_count[0] = 0;
  grid->tile_count[1] = 0;
  grid->tile_count[2] = 0;
  grid->active_voxel_count = 0;
  grid->owns_data = 1;

  grid->data = NULL;
  grid->size = 0;

  return TVDB_OK;
}

void tvdb_nanovdb_destroy_grid(tvdb_nanovdb_grid_t* grid, const tvdb_allocator_t* alloc)
{
  (void)alloc;
  if (!grid)
    return;
  if (grid->name)
    free(grid->name);
  if (grid->owns_data && grid->data)
    free(grid->data);
  memset(grid, 0, sizeof(*grid));
}

/* ========================================================================== */
/*  Gaussian Splat PLY I/O Implementation                                      */
/* ========================================================================== */

static void tvdb__gaussian_splat_destroy(tvdb_gaussian_splat_t* splat)
{
  if (!splat)
    return;
  if (splat->owns_data)
  {
    if (splat->means)
      free(splat->means);
    if (splat->quats)
      free(splat->quats);
    if (splat->log_scales)
      free(splat->log_scales);
    if (splat->logit_opacities)
      free(splat->logit_opacities);
    if (splat->sh_coeffs)
      free(splat->sh_coeffs);
    if (splat->metadata_keys)
      free(splat->metadata_keys);
    if (splat->metadata_values)
      free(splat->metadata_values);
    if (splat->metadata_types)
      free(splat->metadata_types);
    if (splat->metadata_counts)
      free(splat->metadata_counts);
  }
  memset(splat, 0, sizeof(*splat));
}

void tvdb_gaussian_splat_destroy(tvdb_gaussian_splat_t* splat)
{
  tvdb__gaussian_splat_destroy(splat);
  free(splat);
}

static int tvdb__skip_whitespace_and_comments(FILE* fp)
{
  int c;
  while ((c = fgetc(fp)) != EOF)
  {
    if (c == '#')
    {
      while ((c = fgetc(fp)) != EOF && c != '\n')
        ;
      if (c == EOF)
        return 0;
    }
    else if (c > ' ')
    {
      ungetc(c, fp);
      return 1;
    }
  }
  return 0;
}

static int tvdb__read_line(FILE* fp, char* buf, size_t bufsize)
{
  if (!fgets(buf, (int)bufsize, fp))
    return 0;
  size_t len = strlen(buf);
  while (len > 0 && (buf[len - 1] == '\n' || buf[len - 1] == '\r'))
  {
    buf[--len] = '\0';
  }
  return 1;
}

static int tvdb__parse_property(const char* line, char* name, size_t namelen, char* type, size_t typelen)
{
  const char* p = line;
  while (*p == ' ' || *p == '\t')
    p++;
  const char* start = p;
  while (*p && *p != ' ' && *p != '\t')
    p++;
  if ((size_t)(p - start) >= typelen)
    return 0;
  strncpy(type, start, (size_t)(p - start));
  type[p - start] = '\0';
  while (*p == ' ' || *p == '\t')
    p++;
  start = p;
  while (*p && *p != ' ' && *p != '\t' && *p != '[')
    p++;
  if ((size_t)(p - start) >= namelen)
    return 0;
  strncpy(name, start, (size_t)(p - start));
  name[p - start] = '\0';
  return 1;
}

static int tvdb__match_property(const char* prop, const char* expected)
{
  return strncmp(prop, expected, strlen(expected)) == 0;
}

tvdb_gaussian_splat_t* tvdb_gaussian_splat_load(const char* filepath_utf8, tvdb_error_t* err)
{
  FILE* fp = fopen(filepath_utf8, "rb");
  if (!fp)
  {
    if (err)
    {
      snprintf(err->message, sizeof(err->message), "Failed to open file: %s", filepath_utf8);
    }
    return NULL;
  }

  char line[1024];
  char magic[64] = { 0 };
  int num_vertices = 0;
  int num_properties = 0;
  int has_means = 0, has_quats = 0, has_scales = 0, has_opacities = 0;
  int has_sh0 = 0, has_shN = 0;
  int sh_degree = 0, sh_dim = 0;

  if (!tvdb__read_line(fp, magic, sizeof(magic)))
  {
    fclose(fp);
    if (err)
      snprintf(err->message, sizeof(err->message), "Empty file");
    return NULL;
  }

  if (strncmp(magic, "ply", 3) != 0)
  {
    fclose(fp);
    if (err)
      snprintf(err->message, sizeof(err->message), "Not a PLY file");
    return NULL;
  }

  while (tvdb__skip_whitespace_and_comments(fp) && tvdb__read_line(fp, line, sizeof(line)))
  {
    if (strncmp(line, "end_header", 10) == 0)
      break;

    if (strncmp(line, "element vertex", 14) == 0)
    {
      sscanf(line + 14, "%d", &num_vertices);
    }
    else if (strncmp(line, "property", 8) == 0)
    {
      num_properties++;
      char prop_name[64] = { 0 }, prop_type[32] = { 0 };
      if (tvdb__parse_property(line + 8, prop_name, sizeof(prop_name), prop_type, sizeof(prop_type)))
      {
        if (tvdb__match_property(prop_name, "x"))
          has_means = 1;
        else if (tvdb__match_property(prop_name, "quat") || tvdb__match_property(prop_name, "rot"))
          has_quats = 1;
        else if (tvdb__match_property(prop_name, "scale"))
          has_scales = 1;
        else if (tvdb__match_property(prop_name, "opacity"))
          has_opacities = 1;
        else if (tvdb__match_property(prop_name, "f_dc_0"))
          has_sh0 = 1;
        else if (tvdb__match_property(prop_name, "f_rest_"))
          has_shN = 1;
      }
    }
  }

  if (num_vertices <= 0)
  {
    fclose(fp);
    if (err)
      snprintf(err->message, sizeof(err->message), "No vertices found");
    return NULL;
  }

  tvdb_gaussian_splat_t* splat = (tvdb_gaussian_splat_t*)calloc(1, sizeof(tvdb_gaussian_splat_t));
  if (!splat)
  {
    fclose(fp);
    if (err)
      snprintf(err->message, sizeof(err->message), "Out of memory");
    return NULL;
  }

  splat->owns_data = 1;
  splat->num_gaussians = (uint32_t)num_vertices;
  splat->sh_degree = sh_degree;
  splat->sh_dim = sh_dim;

  splat->means = (float*)malloc((size_t)num_vertices * 3 * sizeof(float));
  splat->quats = (float*)malloc((size_t)num_vertices * 4 * sizeof(float));
  splat->log_scales = (float*)malloc((size_t)num_vertices * 3 * sizeof(float));
  splat->logit_opacities = (float*)malloc((size_t)num_vertices * sizeof(float));
  splat->sh_coeffs = (float*)malloc((size_t)num_vertices * (has_sh0 ? 3 : 0) * sizeof(float));

  if (!splat->means || !splat->quats || !splat->log_scales || !splat->logit_opacities)
  {
    tvdb__gaussian_splat_destroy(splat);
    fclose(fp);
    if (err)
      snprintf(err->message, sizeof(err->message), "Out of memory");
    return NULL;
  }

  for (int i = 0; i < num_vertices; i++)
  {
    float x = 0, y = 0, z = 0;
    float qw = 1, qx = 0, qy = 0, qz = 0;
    float sx = 0, sy = 0, sz = 0;
    float opacity = 0;
    float sh0_r = 0, sh0_g = 0, sh0_b = 0;

    int parsed = fscanf(fp, "%f %f %f", &x, &y, &z);

    if (has_quats)
    {
      float quat_arr[4] = { 0 };
      for (int q = 0; q < 4; q++)
      {
        if (fscanf(fp, "%f", &quat_arr[q]) != 1)
          break;
      }
      if (parsed >= 3)
      {
        qw = quat_arr[0];
        qx = quat_arr[1];
        qy = quat_arr[2];
        qz = quat_arr[3];
      }
    }

    if (has_scales)
    {
      for (int s = 0; s < 3; s++)
      {
        if (fscanf(fp, "%f", &sx) != 1)
          break;
        if (s == 0)
          sy = sx;
        if (s <= 1)
          sz = sx;
      }
    }

    if (has_opacities)
    {
      fscanf(fp, "%f", &opacity);
    }

    if (has_sh0)
    {
      for (int c = 0; c < 3; c++)
      {
        float val = 0;
        if (fscanf(fp, "%f", &val) != 1)
          break;
        if (c == 0)
          sh0_r = val;
        else if (c == 1)
          sh0_g = val;
        else
          sh0_b = val;
      }
    }

    fgetc(fp);

    splat->means[i * 3 + 0] = x;
    splat->means[i * 3 + 1] = y;
    splat->means[i * 3 + 2] = z;
    splat->quats[i * 4 + 0] = qw;
    splat->quats[i * 4 + 1] = qx;
    splat->quats[i * 4 + 2] = qy;
    splat->quats[i * 4 + 3] = qz;
    splat->log_scales[i * 3 + 0] = sx;
    splat->log_scales[i * 3 + 1] = sy;
    splat->log_scales[i * 3 + 2] = sz;
    splat->logit_opacities[i] = opacity;
  }

  fclose(fp);
  return splat;
}

tvdb_status_t tvdb_gaussian_splat_save(const char* filepath_utf8, const tvdb_gaussian_splat_t* splat, tvdb_error_t* err)
{
  if (!filepath_utf8 || !splat)
  {
    if (err)
      snprintf(err->message, sizeof(err->message), "Invalid argument");
    return TVDB_ERROR_INVALID_ARGUMENT;
  }

  FILE* fp = fopen(filepath_utf8, "w");
  if (!fp)
  {
    if (err)
      snprintf(err->message, sizeof(err->message), "Failed to open file: %s", filepath_utf8);
    return TVDB_ERROR_IO;
  }

  fprintf(fp, "ply\n");
  fprintf(fp, "format ascii 1.0\n");
  fprintf(fp, "%s\n", TVDB_GAUSSIAN_MAGIC);
  fprintf(fp, TVDB_GAUSSIAN_VERSION "\n");
  fprintf(fp, "element vertex %u\n", splat->num_gaussians);

  fprintf(fp, "property float x\n");
  fprintf(fp, "property float y\n");
  fprintf(fp, "property float z\n");
  fprintf(fp, "property float quat_0\n");
  fprintf(fp, "property float quat_1\n");
  fprintf(fp, "property float quat_2\n");
  fprintf(fp, "property float quat_3\n");
  fprintf(fp, "property float scale_0\n");
  fprintf(fp, "property float scale_1\n");
  fprintf(fp, "property float scale_2\n");
  fprintf(fp, "property float opacity\n");
  fprintf(fp, "property float f_dc_0\n");
  fprintf(fp, "property float f_dc_1\n");
  fprintf(fp, "property float f_dc_2\n");

  fprintf(fp, "end_header\n");

  for (uint32_t i = 0; i < splat->num_gaussians; i++)
  {
    const float* means = splat->means + i * 3;
    const float* quats = splat->quats + i * 4;
    const float* scales = splat->log_scales + i * 3;
    float opacity = splat->logit_opacities ? splat->logit_opacities[i] : 0.0f;
    float sh0_r = 0, sh0_g = 0, sh0_b = 0;
    if (splat->sh_coeffs)
    {
      const float* sh = splat->sh_coeffs + i * 3;
      sh0_r = sh[0];
      sh0_g = sh[1];
      sh0_b = sh[2];
    }

    fprintf(fp,
            "%g %g %g %g %g %g %g %g %g %g %g %g %g %g\n",
            (double)means[0],
            (double)means[1],
            (double)means[2],
            (double)quats[0],
            (double)quats[1],
            (double)quats[2],
            (double)quats[3],
            (double)scales[0],
            (double)scales[1],
            (double)scales[2],
            (double)opacity,
            (double)sh0_r,
            (double)sh0_g,
            (double)sh0_b);
  }

  fclose(fp);
  return TVDB_OK;
}

uint32_t tvdb_gaussian_splat_count(const tvdb_gaussian_splat_t* splat) { return splat ? splat->num_gaussians : 0; }

void tvdb_gaussian_splat_get(const tvdb_gaussian_splat_t* splat,
                             uint32_t idx,
                             float out_means[3],
                             float out_quats[4],
                             float out_scales[3],
                             float* out_opacity)
{
  if (!splat || idx >= splat->num_gaussians)
    return;

  if (out_means)
  {
    out_means[0] = splat->means[idx * 3 + 0];
    out_means[1] = splat->means[idx * 3 + 1];
    out_means[2] = splat->means[idx * 3 + 2];
  }
  if (out_quats)
  {
    out_quats[0] = splat->quats[idx * 4 + 0];
    out_quats[1] = splat->quats[idx * 4 + 1];
    out_quats[2] = splat->quats[idx * 4 + 2];
    out_quats[3] = splat->quats[idx * 4 + 3];
  }
  if (out_scales)
  {
    out_scales[0] = splat->log_scales[idx * 3 + 0];
    out_scales[1] = splat->log_scales[idx * 3 + 1];
    out_scales[2] = splat->log_scales[idx * 3 + 2];
  }
  if (out_opacity)
  {
    *out_opacity = splat->logit_opacities ? splat->logit_opacities[idx] : 0.0f;
  }
}

/* ========================================================================== */
/*  Gaussian Splat Rasterization Implementation                               */
/* ========================================================================== */

/* Math helpers — note that despite the legacy name, this is now a
   correctness-first single-precision exp2 (delegating to libm).
   The earlier hand-rolled bit-twiddle was numerically broken and
   produced ±inf for typical Gaussian sigma values. */
static float tvdb__fast_exp2(float x) { return exp2f(x); }

static float tvdb__fast_sigmoid(float x) { return 1.0f / (1.0f + tvdb__fast_exp2(-x)); }

static float tvdb__fast_sqrt(float x)
{
  union
  {
    float f;
    uint32_t i;
  } u;
  u.f = x;
  u.i = (u.i >> 1) + 0x1fbc0000;
  return u.f;
}

static void tvdb__mat4_mul_vec4(float out[4], const float m[16], const float v[4])
{
  out[0] = m[0] * v[0] + m[4] * v[1] + m[8] * v[2] + m[12] * v[3];
  out[1] = m[1] * v[0] + m[5] * v[1] + m[9] * v[2] + m[13] * v[3];
  out[2] = m[2] * v[0] + m[6] * v[1] + m[10] * v[2] + m[14] * v[3];
  out[3] = m[3] * v[0] + m[7] * v[1] + m[11] * v[2] + m[15] * v[3];
}

static void tvdb__quat_rotate(float out[3], const float q[4], const float v[3])
{
  float qx = q[0], qy = q[1], qz = q[2], qw = q[3];
  float ix = qw * v[0] + qy * v[2] - qz * v[1];
  float iy = qw * v[1] + qz * v[0] - qx * v[2];
  float iz = qw * v[2] + qx * v[1] - qy * v[0];
  float iw = -qx * v[0] - qy * v[1] - qz * v[2];
  out[0] = ix * qw + iw * (-qx) + iy * (-qz) - iz * (-qy);
  out[1] = iy * qw + iw * (-qy) + iz * (-qx) - ix * (-qz);
  out[2] = iz * qw + iw * (-qz) + ix * (-qy) - iy * (-qx);
}

static float tvdb__inverse_sigmoid(float x) { return logf(x / (1.0f - x)); }

static float tvdb__sigmoid(float x) { return 1.0f / (1.0f + tvdb__fast_exp2(-x)); }

/* Camera creation */
tvdb_camera_t* tvdb_camera_create_perspective(float fx,
                                              float fy,
                                              float cx,
                                              float cy,
                                              float width,
                                              float height,
                                              float near,
                                              float far,
                                              const float extrinsics[16])
{
  tvdb_camera_t* cam = (tvdb_camera_t*)calloc(1, sizeof(tvdb_camera_t));
  if (!cam)
    return NULL;

  cam->type = TVDB_CAMERA_PERSPECTIVE;
  cam->fx = fx;
  cam->fy = fy;
  cam->cx = cx;
  cam->cy = cy;
  cam->width = width;
  cam->height = height;
  cam->near = near;
  cam->far = far;

  if (extrinsics)
  {
    memcpy(cam->extrinsics, extrinsics, 16 * sizeof(float));
    cam->is_identity_extrinsics = 0;
  }
  else
  {
    for (int i = 0; i < 16; i++)
    {
      cam->extrinsics[i] = (i == 0 || i == 5 || i == 10 || i == 15) ? 1.0f : 0.0f;
    }
    cam->is_identity_extrinsics = 1;
  }

  cam->intrinsics[0] = fx;
  cam->intrinsics[1] = 0;
  cam->intrinsics[2] = cx;
  cam->intrinsics[3] = 0;
  cam->intrinsics[4] = fy;
  cam->intrinsics[5] = cy;
  cam->intrinsics[6] = 0;
  cam->intrinsics[7] = 0;
  cam->intrinsics[8] = 1;

  return cam;
}

tvdb_camera_t* tvdb_camera_create_orthographic(float scale,
                                               float cx,
                                               float cy,
                                               float width,
                                               float height,
                                               float near,
                                               float far,
                                               const float extrinsics[16])
{
  tvdb_camera_t* cam = (tvdb_camera_t*)calloc(1, sizeof(tvdb_camera_t));
  if (!cam)
    return NULL;

  cam->type = TVDB_CAMERA_ORTHOGRAPHIC;
  cam->fx = scale * width;
  cam->fy = scale * height;
  cam->cx = cx;
  cam->cy = cy;
  cam->width = width;
  cam->height = height;
  cam->near = near;
  cam->far = far;

  if (extrinsics)
  {
    memcpy(cam->extrinsics, extrinsics, 16 * sizeof(float));
    cam->is_identity_extrinsics = 0;
  }
  else
  {
    for (int i = 0; i < 16; i++)
    {
      cam->extrinsics[i] = (i == 0 || i == 5 || i == 10 || i == 15) ? 1.0f : 0.0f;
    }
    cam->is_identity_extrinsics = 1;
  }

  cam->intrinsics[0] = cam->fx;
  cam->intrinsics[1] = 0;
  cam->intrinsics[2] = cx;
  cam->intrinsics[3] = 0;
  cam->intrinsics[4] = cam->fy;
  cam->intrinsics[5] = cy;
  cam->intrinsics[6] = 0;
  cam->intrinsics[7] = 0;
  cam->intrinsics[8] = 1;

  return cam;
}

void tvdb_camera_destroy(tvdb_camera_t* cam) { free(cam); }

/* Gaussian projection: 3D -> 2D with covariance */
tvdb_projected_gaussian_t* tvdb_gaussian_project(const tvdb_gaussian_splat_t* splats,
                                                 const tvdb_camera_t* cam,
                                                 uint32_t* out_count,
                                                 tvdb_error_t* err)
{
  if (!splats || !cam || !out_count)
  {
    if (err)
      snprintf(err->message, sizeof(err->message), "Invalid argument");
    return NULL;
  }

  uint32_t N = splats->num_gaussians;
  tvdb_projected_gaussian_t* gaussians = (tvdb_projected_gaussian_t*)malloc(N * sizeof(tvdb_projected_gaussian_t));
  if (!gaussians)
  {
    if (err)
      snprintf(err->message, sizeof(err->message), "Out of memory");
    return NULL;
  }

  const float eps2d = 0.3f;

  for (uint32_t i = 0; i < N; i++)
  {
    const float* mean3d = splats->means + i * 3;
    const float* quat = splats->quats + i * 4;
    const float* log_scale = splats->log_scales + i * 3;
    float opacity = splats->logit_opacities ? splats->logit_opacities[i] : 0.0f;

    float scale[3] = { tvdb__fast_exp2(log_scale[0]), tvdb__fast_exp2(log_scale[1]), tvdb__fast_exp2(log_scale[2]) };

    float mean_cam[4] = { mean3d[0], mean3d[1], mean3d[2], 1.0f };
    float proj[4];
    tvdb__mat4_mul_vec4(proj, cam->extrinsics, mean_cam);

    float depth = proj[2];
    if (depth <= cam->near || depth >= cam->far)
    {
      gaussians[i].radius = 0.0f;
      gaussians[i].opacity = 0.0f;
      continue;
    }

    float cam_x = proj[0] / proj[3];
    float cam_y = proj[1] / proj[3];

    float inv_depth = 1.0f / depth;
    float fx = cam->intrinsics[0];
    float fy = cam->intrinsics[4];
    float cx = cam->intrinsics[2];
    float cy = cam->intrinsics[5];

    float x = fx * cam_x * inv_depth + cx;
    float y = fy * cam_y * inv_depth + cy;
    gaussians[i].x = x;
    gaussians[i].y = y;
    gaussians[i].depth = depth;

    float sx = scale[0], sy = scale[1], sz = scale[2];

    float covar_3d[6];
    float sqx = sx * sx, sqy = sy * sy, sqz = sz * sz;
    covar_3d[0] = sqx;
    covar_3d[1] = 0.0f;
    covar_3d[2] = 0.0f;
    covar_3d[3] = sqy;
    covar_3d[4] = 0.0f;
    covar_3d[5] = sqz;

    float R[9];
    float qx = quat[0], qy = quat[1], qz = quat[2], qw = quat[3];
    R[0] = 1 - 2 * (qy * qy + qz * qz);
    R[1] = 2 * (qx * qy - qw * qz);
    R[2] = 2 * (qx * qz + qw * qy);
    R[3] = 2 * (qx * qy + qw * qz);
    R[4] = 1 - 2 * (qx * qx + qz * qz);
    R[5] = 2 * (qy * qz - qw * qx);
    R[6] = 2 * (qx * qz - qw * qy);
    R[7] = 2 * (qy * qz + qw * qx);
    R[8] = 1 - 2 * (qx * qx + qy * qy);

    float covar_rot[6];
    for (int r = 0; r < 3; r++)
    {
      for (int c = r; c < 3; c++)
      {
        covar_rot[r * 3 + c] =
            R[r * 3 + 0] * R[c * 3 + 0] * sqx + R[r * 3 + 1] * R[c * 3 + 1] * sqy + R[r * 3 + 2] * R[c * 3 + 2] * sqz;
        if (r != c)
          covar_rot[c * 3 + r] = covar_rot[r * 3 + c];
      }
    }

    float inv_fx = 1.0f / fx;
    float inv_fy = 1.0f / fy;
    float d_x = x - cx;
    float d_y = y - cy;

    float covar_2d[3];
    covar_2d[0] = inv_fx * inv_fx * covar_rot[0] + d_x * d_x * inv_fx * inv_fx * inv_depth * inv_depth * covar_rot[2];
    covar_2d[1] = d_x * d_y * inv_fx * inv_fy * inv_depth * inv_depth * covar_rot[2];
    covar_2d[2] = inv_fy * inv_fy * covar_rot[3] + d_y * d_y * inv_fy * inv_fy * inv_depth * inv_depth * covar_rot[5];

    covar_2d[0] += eps2d;
    covar_2d[2] += eps2d;

    float det = covar_2d[0] * covar_2d[2] - covar_2d[1] * covar_2d[1];
    if (det > 1e-10f)
    {
      float inv_det = 1.0f / det;
      gaussians[i].conic_a = covar_2d[2] * inv_det;
      gaussians[i].conic_b = -covar_2d[1] * inv_det;
      gaussians[i].conic_c = covar_2d[0] * inv_det;
    }
    else
    {
      gaussians[i].conic_a = 1.0f;
      gaussians[i].conic_b = 0.0f;
      gaussians[i].conic_c = 1.0f;
    }

    float det2d = gaussians[i].conic_a * gaussians[i].conic_c - gaussians[i].conic_b * gaussians[i].conic_b;
    float eig_max =
        0.5f *
        (gaussians[i].conic_a + gaussians[i].conic_c +
         tvdb__fast_sqrt((gaussians[i].conic_a - gaussians[i].conic_c) * (gaussians[i].conic_a - gaussians[i].conic_c) +
                         4.0f * gaussians[i].conic_b * gaussians[i].conic_b));
    if (eig_max > 1e-10f)
    {
      gaussians[i].radius = 3.0f * tvdb__fast_sqrt(1.0f / eig_max);
    }
    else
    {
      gaussians[i].radius = 0.0f;
    }

    gaussians[i].opacity = tvdb__sigmoid(opacity);

    if (splats->sh_coeffs)
    {
      const float* sh = splats->sh_coeffs + i * 3;
      gaussians[i].feature[0] = sh[0];
      gaussians[i].feature[1] = sh[1];
      gaussians[i].feature[2] = sh[2];
    }
    else
    {
      gaussians[i].feature[0] = 1.0f;
      gaussians[i].feature[1] = 0.0f;
      gaussians[i].feature[2] = 0.0f;
    }
  }

  *out_count = N;
  return gaussians;
}

void tvdb_projected_gaussian_destroy(tvdb_projected_gaussian_t* gaussians) { free(gaussians); }

/* Spherical-harmonics basis constants (real SH, 3DGS convention). */
tvdb_status_t tvdb_gaussian_sh_eval(uint32_t num_gaussians,
                                    uint32_t degree,
                                    const float* sh_coeffs,
                                    const float* dirs,
                                    float* out_colors,
                                    tvdb_error_t* err)
{
  if (degree > 3 || (num_gaussians && (!sh_coeffs || !dirs || !out_colors)))
  {
    if (err)
    {
      err->status = TVDB_ERROR_INVALID_ARGUMENT;
      snprintf(err->message, sizeof(err->message), "invalid sh_eval arguments");
    }
    return TVDB_ERROR_INVALID_ARGUMENT;
  }
  const float C0 = 0.28209479177387814f;
  const float C1 = 0.4886025119029199f;
  const float C2[5] = {
    1.0925484305920792f, -1.0925484305920792f, 0.31539156525252005f, -1.0925484305920792f, 0.5462742152960396f
  };
  const float C3[7] = { -0.5900435899266435f, 2.890611442640554f, -0.4570457994644658f, 0.3731763325901154f,
                        -0.4570457994644658f, 1.445305721320277f, -0.5900435899266435f };
  uint32_t K = (degree + 1u) * (degree + 1u);
  for (uint32_t g = 0; g < num_gaussians; ++g)
  {
    float dx = dirs[3 * g + 0], dy = dirs[3 * g + 1], dz = dirs[3 * g + 2];
    float len = sqrtf(dx * dx + dy * dy + dz * dz); /* exact, to match GPU length() */
    if (len > 1e-8f)
    {
      dx /= len;
      dy /= len;
      dz /= len;
    }
    else
    {
      dx = dy = dz = 0.0f;
    }
    for (int c = 0; c < 3; ++c)
    {
      const float* sh = sh_coeffs + ((size_t)g * K) * 3 + (size_t)c; /* sh[k] = sh[k*3] */
      float r = C0 * sh[0];
      if (degree >= 1)
      {
        r += C1 * (-dy * sh[3 * 1] + dz * sh[3 * 2] - dx * sh[3 * 3]);
        if (degree >= 2)
        {
          float xx = dx * dx, yy = dy * dy, zz = dz * dz, xy = dx * dy, yz = dy * dz, xz = dx * dz;
          r += C2[0] * xy * sh[3 * 4] + C2[1] * yz * sh[3 * 5] + C2[2] * (2.0f * zz - xx - yy) * sh[3 * 6] +
               C2[3] * xz * sh[3 * 7] + C2[4] * (xx - yy) * sh[3 * 8];
          if (degree >= 3)
          {
            r += C3[0] * dy * (3.0f * xx - yy) * sh[3 * 9] + C3[1] * xy * dz * sh[3 * 10] +
                 C3[2] * dy * (4.0f * zz - xx - yy) * sh[3 * 11] +
                 C3[3] * dz * (2.0f * zz - 3.0f * xx - 3.0f * yy) * sh[3 * 12] +
                 C3[4] * dx * (4.0f * zz - xx - yy) * sh[3 * 13] + C3[5] * dz * (xx - yy) * sh[3 * 14] +
                 C3[6] * dx * (xx - 3.0f * yy) * sh[3 * 15];
          }
        }
      }
      r += 0.5f;
      out_colors[3 * g + c] = r > 0.0f ? r : 0.0f;
    }
  }
  if (err)
    err->status = TVDB_OK;
  return TVDB_OK;
}

#define TVDB_MCMC_NMAX 51

/* Pascal's triangle C(i,k) for i,k in [0, TVDB_MCMC_NMAX). */
static void tvdb__mcmc_binoms(float B[TVDB_MCMC_NMAX][TVDB_MCMC_NMAX])
{
  for (int i = 0; i < TVDB_MCMC_NMAX; ++i)
  {
    for (int k = 0; k < TVDB_MCMC_NMAX; ++k)
      B[i][k] = 0.0f;
    B[i][0] = 1.0f;
    for (int k = 1; k <= i; ++k)
      B[i][k] = B[i - 1][k - 1] + B[i - 1][k];
  }
}

tvdb_status_t tvdb_gaussian_mcmc_relocation(uint32_t num_gaussians,
                                            const float* opacities,
                                            const float* scales,
                                            const int32_t* ratios,
                                            float* new_opacities,
                                            float* new_scales,
                                            tvdb_error_t* err)
{
  if (num_gaussians && (!opacities || !scales || !ratios || !new_opacities || !new_scales))
  {
    if (err)
    {
      err->status = TVDB_ERROR_INVALID_ARGUMENT;
      snprintf(err->message, sizeof(err->message), "invalid mcmc_relocation arguments");
    }
    return TVDB_ERROR_INVALID_ARGUMENT;
  }
  float B[TVDB_MCMC_NMAX][TVDB_MCMC_NMAX];
  tvdb__mcmc_binoms(B);
  for (uint32_t g = 0; g < num_gaussians; ++g)
  {
    int ratio = ratios[g];
    if (ratio < 1)
      ratio = 1;
    if (ratio > TVDB_MCMC_NMAX)
      ratio = TVDB_MCMC_NMAX;
    float op = opacities[g];
    float new_op = 1.0f - powf(1.0f - op, 1.0f / (float)ratio);
    new_opacities[g] = new_op;
    float denom = 0.0f;
    for (int i = 1; i <= ratio; ++i)
      for (int k = 0; k <= i - 1; ++k)
      {
        float sign = (k & 1) ? -1.0f : 1.0f;
        denom += B[i - 1][k] * (sign / sqrtf((float)(k + 1))) * powf(new_op, (float)(k + 1));
      }
    float coeff = (denom != 0.0f) ? (op / denom) : 1.0f;
    for (int d = 0; d < 3; ++d)
      new_scales[3 * g + d] = coeff * scales[3 * g + d];
  }
  if (err)
    err->status = TVDB_OK;
  return TVDB_OK;
}

tvdb_status_t tvdb_gaussian_mcmc_add_noise(uint32_t num_gaussians,
                                           const float* means,
                                           const float* quats,
                                           const float* log_scales,
                                           const float* opacities_logit,
                                           const float* rand,
                                           float lr,
                                           float* out_means,
                                           tvdb_error_t* err)
{
  if (num_gaussians && (!means || !quats || !log_scales || !opacities_logit || !rand || !out_means))
  {
    if (err)
    {
      err->status = TVDB_ERROR_INVALID_ARGUMENT;
      snprintf(err->message, sizeof(err->message), "invalid mcmc_add_noise arguments");
    }
    return TVDB_ERROR_INVALID_ARGUMENT;
  }
  for (uint32_t g = 0; g < num_gaussians; ++g)
  {
    float op = 1.0f / (1.0f + expf(-opacities_logit[g]));
    float gate = 1.0f / (1.0f + expf(-100.0f * (0.005f - op)));
    float sx = expf(log_scales[3 * g + 0]), sy = expf(log_scales[3 * g + 1]), sz = expf(log_scales[3 * g + 2]);
    float sqx = sx * sx, sqy = sy * sy, sqz = sz * sz;
    float qx = quats[4 * g + 0], qy = quats[4 * g + 1], qz = quats[4 * g + 2], qw = quats[4 * g + 3];
    float ql = sqrtf(qx * qx + qy * qy + qz * qz + qw * qw);
    if (ql > 1e-8f)
    {
      qx /= ql;
      qy /= ql;
      qz /= ql;
      qw /= ql;
    }
    float R0 = 1.0f - 2.0f * (qy * qy + qz * qz), R1 = 2.0f * (qx * qy - qw * qz), R2 = 2.0f * (qx * qz + qw * qy);
    float R3 = 2.0f * (qx * qy + qw * qz), R4 = 1.0f - 2.0f * (qx * qx + qz * qz), R5 = 2.0f * (qy * qz - qw * qx);
    float R6 = 2.0f * (qx * qz - qw * qy), R7 = 2.0f * (qy * qz + qw * qx), R8 = 1.0f - 2.0f * (qx * qx + qy * qy);
    /* Cov = R diag(s^2) R^T (symmetric). */
    float c00 = R0 * R0 * sqx + R1 * R1 * sqy + R2 * R2 * sqz;
    float c01 = R0 * R3 * sqx + R1 * R4 * sqy + R2 * R5 * sqz;
    float c02 = R0 * R6 * sqx + R1 * R7 * sqy + R2 * R8 * sqz;
    float c11 = R3 * R3 * sqx + R4 * R4 * sqy + R5 * R5 * sqz;
    float c12 = R3 * R6 * sqx + R4 * R7 * sqy + R5 * R8 * sqz;
    float c22 = R6 * R6 * sqx + R7 * R7 * sqy + R8 * R8 * sqz;
    float gx = rand[3 * g + 0] * gate * lr, gy = rand[3 * g + 1] * gate * lr, gz = rand[3 * g + 2] * gate * lr;
    out_means[3 * g + 0] = means[3 * g + 0] + (c00 * gx + c01 * gy + c02 * gz);
    out_means[3 * g + 1] = means[3 * g + 1] + (c01 * gx + c11 * gy + c12 * gz);
    out_means[3 * g + 2] = means[3 * g + 2] + (c02 * gx + c12 * gy + c22 * gz);
  }
  if (err)
    err->status = TVDB_OK;
  return TVDB_OK;
}

/* Forward rasterization */
tvdb_status_t tvdb_gaussian_rasterize_forward(const tvdb_projected_gaussian_t* gaussians,
                                              uint32_t num_gaussians,
                                              uint32_t width,
                                              uint32_t height,
                                              uint32_t num_features,
                                              float background[3],
                                              float alpha_threshold,
                                              tvdb_raster_output_t* out,
                                              tvdb_error_t* err)
{
  if (!gaussians || !out || width == 0 || height == 0)
  {
    if (err)
      snprintf(err->message, sizeof(err->message), "Invalid argument");
    return TVDB_ERROR_INVALID_ARGUMENT;
  }

  if (num_features == 0)
    num_features = 3;
  if (alpha_threshold <= 0)
    alpha_threshold = TVDB_GAUSSIAN_RASTER_DEFAULT_ALPHA_THRESHOLD;

  memset(out, 0, sizeof(*out));
  out->width = width;
  out->height = height;
  out->num_features = num_features;
  out->owns_data = 1;

  size_t pixel_count = (size_t)width * height;
  out->image = (float*)calloc(pixel_count * num_features, sizeof(float));
  out->alpha = (float*)calloc(pixel_count, sizeof(float));
  out->last_ids = (int32_t*)malloc(pixel_count * sizeof(int32_t));

  if (!out->image || !out->alpha || !out->last_ids)
  {
    if (out->image)
      free(out->image);
    if (out->alpha)
      free(out->alpha);
    if (out->last_ids)
      free(out->last_ids);
    if (err)
      snprintf(err->message, sizeof(err->message), "Out of memory");
    return TVDB_ERROR_OUT_OF_MEMORY;
  }

  for (size_t i = 0; i < pixel_count; i++)
  {
    for (uint32_t f = 0; f < num_features; f++)
    {
      out->image[i * num_features + f] = background ? background[f] : 0.0f;
    }
  }

  uint32_t tile_size = TVDB_GAUSSIAN_RASTER_TILE_SIZE;
  uint32_t num_tiles_x = (width + tile_size - 1) / tile_size;
  uint32_t num_tiles_y = (height + tile_size - 1) / tile_size;

  typedef struct
  {
    uint32_t gaussian_id;
    float depth;
    int32_t tile_x, tile_y;
  } tile_entry_t;

  size_t max_entries = num_gaussians * 16;
  tile_entry_t* entries = (tile_entry_t*)malloc(max_entries * sizeof(tile_entry_t));
  if (!entries)
  {
    free(out->image);
    free(out->alpha);
    free(out->last_ids);
    if (err)
      snprintf(err->message, sizeof(err->message), "Out of memory");
    return TVDB_ERROR_OUT_OF_MEMORY;
  }
  size_t num_entries = 0;

  for (uint32_t i = 0; i < num_gaussians; i++)
  {
    if (gaussians[i].radius <= 0.0f || gaussians[i].opacity <= 0.0f)
      continue;

    int32_t center_x = (int32_t)gaussians[i].x;
    int32_t center_y = (int32_t)gaussians[i].y;
    int32_t radius = (int32_t)(gaussians[i].radius + 0.5f);

    int32_t tile_x0 = center_x / (int32_t)tile_size;
    int32_t tile_y0 = center_y / (int32_t)tile_size;
    int32_t tile_x1 = (center_x + radius) / (int32_t)tile_size;
    int32_t tile_y1 = (center_y + radius) / (int32_t)tile_size;

    for (int32_t ty = tile_y0; ty <= tile_y1; ty++)
    {
      for (int32_t tx = tile_x0; tx <= tile_x1; tx++)
      {
        if (tx < 0 || ty < 0 || (uint32_t)tx >= num_tiles_x || (uint32_t)ty >= num_tiles_y)
          continue;
        if (num_entries >= max_entries)
          continue;
        entries[num_entries].gaussian_id = i;
        entries[num_entries].depth = gaussians[i].depth;
        entries[num_entries].tile_x = tx;
        entries[num_entries].tile_y = ty;
        num_entries++;
      }
    }
  }

  for (size_t e = 0; e < num_entries; e++)
  {
    for (size_t k = e + 1; k < num_entries; k++)
    {
      if (entries[e].tile_x > entries[k].tile_x ||
          (entries[e].tile_x == entries[k].tile_x && entries[e].tile_y > entries[k].tile_y) ||
          (entries[e].tile_x == entries[k].tile_x && entries[e].tile_y == entries[k].tile_y &&
           entries[e].depth > entries[k].depth))
      {
        tile_entry_t tmp = entries[e];
        entries[e] = entries[k];
        entries[k] = tmp;
      }
    }
  }

  for (size_t e = 0; e < num_entries; e++)
  {
    uint32_t gid = entries[e].gaussian_id;
    const tvdb_projected_gaussian_t* g = &gaussians[gid];
    int32_t tile_x = entries[e].tile_x;
    int32_t tile_y = entries[e].tile_y;

    uint32_t px0 = (uint32_t)(tile_x * tile_size);
    uint32_t py0 = (uint32_t)(tile_y * tile_size);
    uint32_t px1 = px0 + tile_size;
    uint32_t py1 = py0 + tile_size;
    if (px1 > width)
      px1 = width;
    if (py1 > height)
      py1 = height;

    float conic_a = g->conic_a, conic_b = g->conic_b, conic_c = g->conic_c;
    float gx = g->x, gy = g->y;
    float alpha = g->opacity;

    for (uint32_t py = py0; py < py1; py++)
    {
      for (uint32_t px = px0; px < px1; px++)
      {
        float dx = (float)px - gx;
        float dy = (float)py - gy;
        float sigma = 0.5f * (conic_a * dx * dx + 2.0f * conic_b * dx * dy + conic_c * dy * dy);
        if (sigma > 10.0f)
          continue;

        /* α = opacity · exp(-σ): natural exp matches the standard
           Gaussian-splatting density; the backward uses the same. */
        float gaussian_alpha = alpha * expf(-sigma);
        if (gaussian_alpha < alpha_threshold)
          continue;

        size_t pixel_idx = (size_t)py * width + px;

        float T = 1.0f - out->alpha[pixel_idx];
        if (T < 0.001f)
          continue;

        out->alpha[pixel_idx] += gaussian_alpha * T;
        for (uint32_t f = 0; f < num_features; f++)
        {
          out->image[pixel_idx * num_features + f] += g->feature[f] * gaussian_alpha * T;
        }
        out->last_ids[pixel_idx] = (int32_t)gid;
      }
    }
  }

  free(entries);
  return TVDB_OK;
}

void tvdb_raster_output_destroy(tvdb_raster_output_t* out)
{
  if (!out)
    return;
  if (out->owns_data)
  {
    if (out->image)
      free(out->image);
    if (out->alpha)
      free(out->alpha);
    if (out->last_ids)
      free(out->last_ids);
  }
  memset(out, 0, sizeof(*out));
}

/* ----- Gaussian-splat rasterizer backward (CPU autograd) ----- */

tvdb_status_t tvdb_gaussian_grad_init(tvdb_gaussian_grad_t* g, uint32_t num_gaussians, uint32_t num_features)
{
  if (!g)
    return TVDB_ERROR_INVALID_ARGUMENT;
  memset(g, 0, sizeof(*g));
  if (num_gaussians == 0)
    return TVDB_OK;
  if (num_features == 0)
    num_features = 3;
  g->num_gaussians = num_gaussians;
  g->num_features = num_features;
  g->grad_x = (float*)calloc(num_gaussians, sizeof(float));
  g->grad_y = (float*)calloc(num_gaussians, sizeof(float));
  g->grad_conic_a = (float*)calloc(num_gaussians, sizeof(float));
  g->grad_conic_b = (float*)calloc(num_gaussians, sizeof(float));
  g->grad_conic_c = (float*)calloc(num_gaussians, sizeof(float));
  g->grad_opacity = (float*)calloc(num_gaussians, sizeof(float));
  g->grad_feature = (float*)calloc((size_t)num_gaussians * num_features, sizeof(float));
  g->owns_data = 1;
  if (!g->grad_x || !g->grad_y || !g->grad_conic_a || !g->grad_conic_b || !g->grad_conic_c || !g->grad_opacity ||
      !g->grad_feature)
  {
    tvdb_gaussian_grad_destroy(g);
    return TVDB_ERROR_OUT_OF_MEMORY;
  }
  return TVDB_OK;
}

void tvdb_gaussian_grad_destroy(tvdb_gaussian_grad_t* g)
{
  if (!g)
    return;
  if (g->owns_data)
  {
    free(g->grad_x);
    free(g->grad_y);
    free(g->grad_conic_a);
    free(g->grad_conic_b);
    free(g->grad_conic_c);
    free(g->grad_opacity);
    free(g->grad_feature);
  }
  memset(g, 0, sizeof(*g));
}

tvdb_status_t tvdb_gaussian_rasterize_backward(const tvdb_projected_gaussian_t* gaussians,
                                               uint32_t num_gaussians,
                                               const tvdb_raster_output_t* fwd,
                                               const float* dL_dC,
                                               const float* dL_dA,
                                               float background[3],
                                               float alpha_threshold,
                                               tvdb_gaussian_grad_t* grad_out,
                                               tvdb_error_t* err)
{
  if (!gaussians || !fwd || !dL_dC || !grad_out)
  {
    if (err)
      snprintf(err->message, sizeof(err->message), "Invalid argument");
    return TVDB_ERROR_INVALID_ARGUMENT;
  }
  if (grad_out->num_gaussians != num_gaussians || grad_out->num_features != fwd->num_features)
  {
    if (err)
      snprintf(err->message, sizeof(err->message), "grad_out shape mismatch");
    return TVDB_ERROR_INVALID_ARGUMENT;
  }

  uint32_t width = fwd->width;
  uint32_t height = fwd->height;
  uint32_t F = fwd->num_features;
  if (alpha_threshold <= 0)
    alpha_threshold = TVDB_GAUSSIAN_RASTER_DEFAULT_ALPHA_THRESHOLD;

  /* Re-derive the same per-tile, depth-sorted entry list as the forward
     pass. We must traverse it in REVERSE order. */
  uint32_t tile_size = TVDB_GAUSSIAN_RASTER_TILE_SIZE;
  uint32_t num_tiles_x = (width + tile_size - 1) / tile_size;
  uint32_t num_tiles_y = (height + tile_size - 1) / tile_size;

  typedef struct
  {
    uint32_t gaussian_id;
    float depth;
    int32_t tile_x, tile_y;
  } tile_entry_t;

  size_t max_entries = (size_t)num_gaussians * 16;
  tile_entry_t* entries = (tile_entry_t*)malloc(max_entries * sizeof(*entries));
  if (!entries)
  {
    if (err)
      snprintf(err->message, sizeof(err->message), "OOM");
    return TVDB_ERROR_OUT_OF_MEMORY;
  }
  size_t num_entries = 0;
  for (uint32_t i = 0; i < num_gaussians; ++i)
  {
    if (gaussians[i].radius <= 0.0f || gaussians[i].opacity <= 0.0f)
      continue;
    int32_t cx = (int32_t)gaussians[i].x;
    int32_t cy = (int32_t)gaussians[i].y;
    int32_t r = (int32_t)(gaussians[i].radius + 0.5f);
    int32_t tx0 = cx / (int32_t)tile_size, ty0 = cy / (int32_t)tile_size;
    int32_t tx1 = (cx + r) / (int32_t)tile_size;
    int32_t ty1 = (cy + r) / (int32_t)tile_size;
    for (int32_t ty = ty0; ty <= ty1; ++ty)
    {
      for (int32_t tx = tx0; tx <= tx1; ++tx)
      {
        if (tx < 0 || ty < 0 || (uint32_t)tx >= num_tiles_x || (uint32_t)ty >= num_tiles_y)
          continue;
        if (num_entries >= max_entries)
          continue;
        entries[num_entries].gaussian_id = i;
        entries[num_entries].depth = gaussians[i].depth;
        entries[num_entries].tile_x = tx;
        entries[num_entries].tile_y = ty;
        ++num_entries;
      }
    }
  }
  /* Same naive O(N²) sort as forward (so order matches). */
  for (size_t e = 0; e < num_entries; ++e)
  {
    for (size_t k = e + 1; k < num_entries; ++k)
    {
      int swap = (entries[e].tile_x > entries[k].tile_x) ||
                 (entries[e].tile_x == entries[k].tile_x && entries[e].tile_y > entries[k].tile_y) ||
                 (entries[e].tile_x == entries[k].tile_x && entries[e].tile_y == entries[k].tile_y &&
                  entries[e].depth > entries[k].depth);
      if (swap)
      {
        tile_entry_t tmp = entries[e];
        entries[e] = entries[k];
        entries[k] = tmp;
      }
    }
  }

  size_t pixel_count = (size_t)width * height;
  float* T_curr = (float*)malloc(pixel_count * sizeof(float));
  float* S_curr = (float*)calloc(pixel_count * F, sizeof(float));
  if (!T_curr || !S_curr)
  {
    free(entries);
    free(T_curr);
    free(S_curr);
    if (err)
      snprintf(err->message, sizeof(err->message), "OOM");
    return TVDB_ERROR_OUT_OF_MEMORY;
  }
  /* Initialize: T_curr[p] = T_final = 1 - alpha_final.
   *             S_curr[p,f] = T_final * bg[f] (post-all-gaussians color). */
  for (size_t p = 0; p < pixel_count; ++p)
  {
    float Tfin = 1.0f - fwd->alpha[p];
    if (Tfin < 0.0f)
      Tfin = 0.0f;
    T_curr[p] = Tfin;
    for (uint32_t f = 0; f < F; ++f)
    {
      S_curr[p * F + f] = Tfin * (background ? background[f] : 0.0f);
    }
  }

  /* Walk entries in REVERSE forward order. */
  for (size_t e = num_entries; e > 0; --e)
  {
    const tile_entry_t* en = &entries[e - 1];
    const tvdb_projected_gaussian_t* g = &gaussians[en->gaussian_id];
    uint32_t gid = en->gaussian_id;

    uint32_t px0 = (uint32_t)(en->tile_x * (int32_t)tile_size);
    uint32_t py0 = (uint32_t)(en->tile_y * (int32_t)tile_size);
    uint32_t px1 = px0 + tile_size;
    uint32_t py1 = py0 + tile_size;
    if (px1 > width)
      px1 = width;
    if (py1 > height)
      py1 = height;

    float a_c = g->conic_a, b_c = g->conic_b, c_c = g->conic_c;
    float gx = g->x, gy = g->y;
    float opac = g->opacity;

    for (uint32_t py = py0; py < py1; ++py)
    {
      for (uint32_t px = px0; px < px1; ++px)
      {
        float dx = (float)px - gx;
        float dy = (float)py - gy;
        float sigma = 0.5f * (a_c * dx * dx + 2.0f * b_c * dx * dy + c_c * dy * dy);
        if (sigma > 10.0f)
          continue;
        float G = (float)exp(-(double)sigma);
        float alpha = opac * G;
        if (alpha < alpha_threshold)
          continue;

        /* T_pre is the transmittance entering this gaussian during
         * the forward pass (i.e. what the forward called `T`). The
         * forward skipped if T_pre < 0.001, so we replicate that
         * test on T_pre, not T_curr. */
        float one_m_alpha = 1.0f - alpha;
        if (one_m_alpha < 1e-7f)
          one_m_alpha = 1e-7f;
        float T_pre = T_curr[(size_t)py * width + px] / one_m_alpha;
        if (T_pre < 0.001f)
          continue;

        size_t pidx = (size_t)py * width + px;

        /* dL/df_i [p] += dL/dC[p] * (T_pre * alpha) */
        float w = T_pre * alpha;
        /* Also compute:
         *   dotCf  = Σ_f dL/dC[p,f] * f_i[f]  (for dα via C)
         *   dotCS  = Σ_f dL/dC[p,f] * S_curr[p,f]  (for dα via C)
         * dα/dα = T_pre*f_i - S_curr/(1-α). */
        float dotCf = 0.0f, dotCS = 0.0f;
        for (uint32_t f = 0; f < F; ++f)
        {
          float dLdCf = dL_dC[pidx * F + f];
          float feat = (f < 3) ? g->feature[f] : 0.0f;
          dotCf += dLdCf * feat;
          dotCS += dLdCf * S_curr[pidx * F + f];
          grad_out->grad_feature[(size_t)gid * F + f] += dLdCf * w;
        }
        float Tfin = 1.0f - fwd->alpha[pidx];
        if (Tfin < 0.0f)
          Tfin = 0.0f;
        float dL_dalpha = T_pre * dotCf - dotCS / one_m_alpha;
        if (dL_dA)
        {
          /* dA_final/dα_i = T_N / (1 - α_i) */
          dL_dalpha += dL_dA[pidx] * Tfin / one_m_alpha;
        }

        /* α = opacity * G  →  dα/dopacity = G, dα/dG = opacity,
         * G = exp(-σ) → dG/dσ = -G → dα/dσ = -α. */
        grad_out->grad_opacity[gid] += dL_dalpha * G;
        float dL_dsigma = -alpha * dL_dalpha;

        /* σ-grads:
         *   ∂σ/∂gx = -(a*dx + b*dy)
         *   ∂σ/∂gy = -(b*dx + c*dy)
         *   ∂σ/∂a  = 0.5 * dx²
         *   ∂σ/∂b  = dx * dy
         *   ∂σ/∂c  = 0.5 * dy² */
        grad_out->grad_x[gid] += dL_dsigma * -(a_c * dx + b_c * dy);
        grad_out->grad_y[gid] += dL_dsigma * -(b_c * dx + c_c * dy);
        grad_out->grad_conic_a[gid] += dL_dsigma * 0.5f * dx * dx;
        grad_out->grad_conic_b[gid] += dL_dsigma * dx * dy;
        grad_out->grad_conic_c[gid] += dL_dsigma * 0.5f * dy * dy;

        /* Update running state: S_curr[p,f] += w * f_i[f];
         *                       T_curr[p]    = T_pre. */
        for (uint32_t f = 0; f < F; ++f)
        {
          float feat = (f < 3) ? g->feature[f] : 0.0f;
          S_curr[pidx * F + f] += w * feat;
        }
        T_curr[pidx] = T_pre;
      }
    }
  }

  free(entries);
  free(T_curr);
  free(S_curr);
  return TVDB_OK;
}

#endif /* TINYVDB_NANOVDB_IMPLEMENTATION */

#endif /* TINYVDB_NANOVDB_H_ */
