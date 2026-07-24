#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include "tvdb_memory.h"

// Data types
typedef struct
{
  float x, y, z;
} tvdb_vec3f;

typedef struct
{
  uint32_t v0, v1, v2;
} tvdb_triangle;

typedef struct
{
  tvdb_vec3f* vertices;
  size_t vertex_count;
  size_t vertex_capacity;
  tvdb_triangle* faces;
  size_t face_count;
  size_t face_capacity;
} tvdb_triangle_mesh;

typedef struct
{
  int nx, ny, nz;
  float ox, oy, oz;
  float voxel_size;
  float* data;
} tvdb_dense_grid;

// API
bool tvdb_mesh_to_sdf(const tvdb_triangle_mesh* mesh,
                      float voxel_size,
                      float band_width,
                      tvdb_dense_grid* grid,
                      tvdb_arena_allocator_t* arena);
bool tvdb_sdf_to_mesh(const tvdb_dense_grid* grid,
                      float isovalue,
                      tvdb_triangle_mesh* mesh,
                      tvdb_arena_allocator_t* arena);

// Marching-cubes lookup tables (edge: 256 ints; triangle: flat 256*16 ints).
const int* tvdb_mc_edge_table(void);
const int* tvdb_mc_tri_table_flat(void);
bool tvdb_make_manifold(const tvdb_triangle_mesh* input,
                        double resolution,
                        double isovalue,
                        tvdb_triangle_mesh* output,
                        tvdb_arena_allocator_t* arena);

typedef enum
{
  TVDB_SIGN_FLOOD_FILL = 0,
  TVDB_SIGN_SWEEP = 1,
} tvdb_sign_method;

bool tvdb_mesh_to_sdf_vdb(const tvdb_triangle_mesh* mesh,
                          float voxel_size,
                          float band_width,
                          tvdb_dense_grid* grid,
                          tvdb_sign_method sign_method,
                          tvdb_arena_allocator_t* arena);
bool tvdb_make_manifold_vdb(const tvdb_triangle_mesh* input,
                            double resolution,
                            double isovalue,
                            tvdb_triangle_mesh* output,
                            tvdb_sign_method sign_method,
                            tvdb_arena_allocator_t* arena);

// Keep original memory management for compatibility
void tvdb_triangle_mesh_init(tvdb_triangle_mesh* mesh);
void tvdb_triangle_mesh_free(tvdb_triangle_mesh* mesh);
void tvdb_dense_grid_init(tvdb_dense_grid* grid, int nx, int ny, int nz);
void tvdb_dense_grid_free(tvdb_dense_grid* grid);

// Arena-based init
void tvdb_triangle_mesh_init_arena(tvdb_triangle_mesh* mesh, tvdb_arena_allocator_t* arena);
void tvdb_dense_grid_init_arena(tvdb_dense_grid* grid, int nx, int ny, int nz, tvdb_arena_allocator_t* arena);

#ifdef __cplusplus
}
#endif
