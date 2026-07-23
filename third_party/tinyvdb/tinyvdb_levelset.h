#pragma once

// Analytic level-set (signed distance field) primitive generators and SDF
// utilities, parallel to OpenVDB's LevelSetSphere / LevelSetPlatonic /
// LevelSetTubes and LevelSetUtil. All operate on the dense grid representation
// (tvdb_dense_grid); the result can be written to .vdb via the existing
// dense/sparse writers or fed to the dense ops (CSG, fast sweeping, etc).

#include "tinyvdb_mesh.h"  // tvdb_dense_grid

#ifdef __cplusplus
extern "C" {
#endif

// Default narrow-band half width in voxels (matches OpenVDB's
// LEVEL_SET_HALF_WIDTH). Pass half_width <= 0 to any generator to use this.
#define TVDB_LEVEL_SET_HALF_WIDTH 3.0f

// --- Primitive generators ---------------------------------------------------
//
// Each fills `out` (allocated via tvdb_dense_grid_init; free with
// tvdb_dense_grid_free) with a dense narrow-band SDF: every voxel holds the
// true signed distance to the surface, clamped to +/- (half_width*voxel_size)
// (the background fill). The grid is sized to enclose the primitive plus the
// background margin, with an isotropic `voxel_size`. Negative = inside. Voxel
// (i,j,k) stores the distance at world position
// (ox + (i+0.5)*vs, oy + (j+0.5)*vs, oz + (k+0.5)*vs) (cell-center convention,
// matching the samplers). Returns true on success, false on bad args / OOM.

// Sphere of `radius` centered at `center`.
bool tvdb_level_set_sphere(float radius,
                           const float center[3],
                           float voxel_size,
                           float half_width,
                           tvdb_dense_grid* out);

// Axis-aligned box; `half_extents` are the half side lengths along x/y/z.
bool tvdb_level_set_box(const float half_extents[3],
                        const float center[3],
                        float voxel_size,
                        float half_width,
                        tvdb_dense_grid* out);

// Torus lying in the XZ plane (symmetry axis = Y), `major_radius` ring radius,
// `minor_radius` tube radius, centered at `center`.
bool tvdb_level_set_torus(float major_radius,
                          float minor_radius,
                          const float center[3],
                          float voxel_size,
                          float half_width,
                          tvdb_dense_grid* out);

// Capsule: the line segment p0->p1 swept by `radius` (a round-ended cylinder).
bool tvdb_level_set_capsule(const float p0[3],
                            const float p1[3],
                            float radius,
                            float voxel_size,
                            float half_width,
                            tvdb_dense_grid* out);

// Platonic solid, selected by `face_count`: 4 = tetrahedron, 6 = cube,
// 8 = octahedron, 12 = dodecahedron, 20 = icosahedron. `radius` is the
// circumradius (center-to-vertex distance). The field is the convex
// half-space SDF (max over the face planes): its zero isosurface is exact;
// off-surface values within the narrow band near edges/vertices are a
// conservative underestimate of the true distance (the standard convex-polytope
// SDF). Returns false for an unsupported face_count or bad args.
bool tvdb_level_set_platonic(int face_count,
                             float radius,
                             const float center[3],
                             float voxel_size,
                             float half_width,
                             tvdb_dense_grid* out);

// --- SDF utilities (operate on an existing dense SDF grid) ------------------

// Convert an SDF to a fog volume: per voxel density =
// clamp(-sdf / (half_width*voxel_size), 0, 1). The interior (sdf<0) ramps from
// 0 at the surface to 1 at `half_width` voxels deep; the exterior is 0. `out`
// is allocated and inherits `sdf`'s transform. Pass half_width <= 0 for the
// default. Returns true on success.
bool tvdb_sdf_to_fog_volume(const tvdb_dense_grid* sdf, float half_width, tvdb_dense_grid* out);

// Interior mask: out voxel = 1.0 where sdf < isovalue, else 0.0. `out` is
// allocated and inherits `sdf`'s transform. Returns true on success.
bool tvdb_sdf_interior_mask(const tvdb_dense_grid* sdf, float isovalue, tvdb_dense_grid* out);

// Segment the interior (sdf < isovalue) into connected components (parallels
// OpenVDB LevelSetUtil::sdfSegmentation). `connectivity` is 6 (face) or 26
// (face+edge+vertex). Returns one SDF grid per component via `*out_grids` (a
// malloc'd array of `*out_count` tvdb_dense_grid, each owning its data and
// inheriting `sdf`'s transform): grid i is a copy of `sdf` with every *other*
// component's interior filled to the background (its largest value), so only
// component i remains a distinct object. Free each grid's data with
// tvdb_dense_grid_free, then free(*out_grids). `*out_count` may be 0 (no
// interior), in which case `*out_grids` is NULL. Returns false on error.
bool tvdb_sdf_segmentation(const tvdb_dense_grid* sdf,
                           float isovalue,
                           int connectivity,
                           tvdb_dense_grid** out_grids,
                           int* out_count);

// Extract enclosed regions / cavities (parallels OpenVDB
// LevelSetUtil::extractEnclosedRegions): exterior voxels (sdf >= isovalue) that
// are not connected to the grid boundary by a path of exterior voxels — i.e.
// voids sealed inside the surface. `out` is a mask grid (1.0 inside a cavity,
// 0.0 elsewhere) inheriting `sdf`'s transform. `connectivity` is 6 or 26.
// Returns true on success.
bool tvdb_sdf_extract_enclosed_regions(const tvdb_dense_grid* sdf,
                                       float isovalue,
                                       int connectivity,
                                       tvdb_dense_grid* out);

// Topological measures of the level set's surface (the isosurface at
// `isovalue`), parallel to OpenVDB LevelSetMeasure. Computed exactly from the
// cubical complex of the interior region (sdf < isovalue): chi_solid =
// V - E + F - cubes over the unit-cube complex, and for a closed surface
// chi_surface = 2 * chi_solid. These assume the interior does not touch the
// grid boundary (the surface is closed); for tinyvdb's narrow-band generators
// that holds. Return 0.0 / 0 on a NULL/empty grid.

// Euler characteristic of the surface: 2 for a sphere, 0 for a torus, 4 for two
// spheres, -2 for a genus-2 surface, etc.
double tvdb_level_set_euler_characteristic(const tvdb_dense_grid* sdf, float isovalue);

// Total genus of the surface: (number of connected interior components) -
// chi_solid. 0 for a sphere or any number of disjoint spheres, 1 for a torus,
// 2 for a double torus. (Components are counted with face-corner / 26-cube
// adjacency to match the closed-cube complex.)
int tvdb_level_set_genus(const tvdb_dense_grid* sdf, float isovalue);

// Rebuild a clean narrow-band signed distance field from a level set's
// isosurface (parallels OpenVDB LevelSetRebuild): extract the `isovalue`
// isosurface as a mesh (marching cubes) and reconvert it to an SDF. This
// renormalizes a damaged / non-Eikonal / arbitrarily-scaled level set, can
// resample to a new `voxel_size`, and recenters a non-zero isovalue at 0.
// `voxel_size` <= 0 reuses the input's voxel size; `half_width` <= 0 uses the
// default band. `sign_method` is 0 (flood fill) or 1 (sweep). `out` is
// allocated by mesh-to-SDF (sized to the surface bbox + band) — free with
// tvdb_dense_grid_free. Returns false on a NULL/empty grid or an empty
// isosurface.
bool tvdb_level_set_rebuild(const tvdb_dense_grid* sdf,
                            float isovalue,
                            float voxel_size,
                            float half_width,
                            int sign_method,
                            tvdb_dense_grid* out);

#ifdef __cplusplus
}
#endif
