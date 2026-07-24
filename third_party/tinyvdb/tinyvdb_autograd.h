#pragma once

// CPU autograd: per-op vector-Jacobian-product (VJP) functions.
//
// Design:
//   tinyvdb has no tape, no Variable type, no operator overloading. Each
//   forward op has companion `_vjp_*` functions that take the upstream
//   gradient (`grad_out`) and produce gradients with respect to each input.
//   This matches fvdb's PyTorch custom Function backwards: it's the minimum
//   surface needed to compose differentiable pipelines, while keeping the
//   library framework-free.
//
// Pattern:
//   forward:    out = f(a, b, ...)
//   backward:   dL/da, dL/db, ... = f_vjp(a, b, ..., grad_out)
//
// All ops here are linear or piecewise-linear in their inputs, so VJPs are
// either transposes of forward ops or simple routing. The user is
// responsible for chaining VJPs in topological order.
//
// Sign convention: gradients are *added* to the output buffers (so a
// caller can accumulate from multiple upstreams). Zero the output buffer
// before calling if you want absolute (not accumulated) gradients.

#include <stdbool.h>
#include <stddef.h>

#include "tinyvdb_mesh.h"    // tvdb_dense_grid, tvdb_vec3f
#include "tinyvdb_ops.h"     // tvdb_dense_grid_d
#include "tinyvdb_sparse.h"  // tvdb_sparse_grid

#ifdef __cplusplus
extern "C" {
#endif

// ---------------------------------------------------------------------------
// Trilinear sample / splat VJPs
//
// Forward `out_j = sum_i w_ij(p_j) * g_i` where `w_ij` is the trilinear
// weight from voxel i to point j. Splat is the transpose: `g_i += sum_j
// w_ij * v_j`. Each op's VJP is therefore the *other* op:
//
//   sample.vjp(grid)  = splat(grad_out, points)         => grad_grid
//   splat.vjp(values) = sample(grad_grid, points)       => grad_values
//
// VJP w.r.t. the points is `grad_out * grad(grid)(points)`, which is just
// the gradient of the field sampled at each point times the upstream grad.
// We compute it by finite-differencing the forward sampler in each axis.
// ---------------------------------------------------------------------------

// VJP of `sample(grid, points)` w.r.t. `grid`. Equivalent to splatting
// `grad_out` onto `grad_grid` with the same trilinear weights. `grad_grid`
// is *accumulated into* (caller zeros if needed).
void tvdb_sample_trilinear_dense_vjp_grid(const tvdb_dense_grid* grid,
                                          const tvdb_vec3f* pts,
                                          size_t n,
                                          const float* grad_out,
                                          tvdb_dense_grid* grad_grid);

// VJP of `sample(grid, points)` w.r.t. `points`. For each point, computes
// `grad_pts[j] = grad_out[j] * grad_grid_at_point(j)` using analytic
// trilinear weight derivatives. `grad_pts` is *accumulated into*.
void tvdb_sample_trilinear_dense_vjp_pts(const tvdb_dense_grid* grid,
                                         const tvdb_vec3f* pts,
                                         size_t n,
                                         const float* grad_out,
                                         tvdb_vec3f* grad_pts);

// VJP of `splat(grid, points, values)` w.r.t. `values`. Equivalent to
// sampling `grad_grid` at the same points. `grad_values` is *accumulated*.
void tvdb_splat_trilinear_dense_vjp_values(const tvdb_dense_grid* grad_grid,
                                           const tvdb_vec3f* pts,
                                           size_t n,
                                           float* grad_values);

// ---------------------------------------------------------------------------
// CSG VJPs.
//
// Forward csg_union     : c[i] = min(a[i], b[i])
// Forward csg_intersect : c[i] = max(a[i], b[i])
// Forward csg_difference: c[i] = max(a[i], -b[i])
//
// VJP routes grad_out to whichever input was the active branch. Ties go
// 50/50 to keep the VJP a valid subgradient. Outputs are *accumulated*.
// ---------------------------------------------------------------------------

void tvdb_csg_union_vjp(const tvdb_dense_grid* a,
                        const tvdb_dense_grid* b,
                        const tvdb_dense_grid* grad_out,
                        tvdb_dense_grid* grad_a,
                        tvdb_dense_grid* grad_b);
void tvdb_csg_intersection_vjp(const tvdb_dense_grid* a,
                               const tvdb_dense_grid* b,
                               const tvdb_dense_grid* grad_out,
                               tvdb_dense_grid* grad_a,
                               tvdb_dense_grid* grad_b);
void tvdb_csg_difference_vjp(const tvdb_dense_grid* a,
                             const tvdb_dense_grid* b,
                             const tvdb_dense_grid* grad_out,
                             tvdb_dense_grid* grad_a,
                             tvdb_dense_grid* grad_b);

// ---------------------------------------------------------------------------
// Sparse conv3d VJPs.
//
// Forward `tvdb_sparse_conv3d` computes:
//   out[oc] = sum_{di,dj,dk} kernel[((dk*ky+dj)*kx+di)] * in[oc - (di,dj,dk)]
// where coords-out coincides with coords-in (same-topology) and out-of-set
// neighbors contribute 0.
//
// VJPs are linear in the relevant inputs:
//   d/d_in_value  = correlate(grad_out, kernel) (transposed kernel application)
//   d/d_kernel    = sum_oc grad_out[oc] * in[oc - offset] for each kernel tap
//
// Both VJPs preserve same-topology layout and accumulate into outputs.
// Returns false on shape / topology mismatch.
// ---------------------------------------------------------------------------

bool tvdb_sparse_conv3d_vjp_values(const tvdb_sparse_grid* in_topo,
                                   const float* grad_out_values,
                                   const float* kernel,
                                   int kx,
                                   int ky,
                                   int kz,
                                   float* grad_in_values /* same length as in_topo */);

bool tvdb_sparse_conv3d_vjp_kernel(const tvdb_sparse_grid* in_with_values,
                                   const float* grad_out_values,
                                   int kx,
                                   int ky,
                                   int kz,
                                   float* grad_kernel /* length kx*ky*kz */);

#ifdef __cplusplus
}
#endif
