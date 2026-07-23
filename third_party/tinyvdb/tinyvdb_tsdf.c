#include "tinyvdb_tsdf.h"
#include "tinyvdb_ops_internal.h"

#include <math.h>
#include <string.h>

void tvdb_invert_rigid_pose(const float pose_wc[12], float pose_cw_out[12]) {
  // pose_wc = [R | t]; inverse = [R^T | -R^T t]
  float R[9] = {
    pose_wc[0], pose_wc[1], pose_wc[2],
    pose_wc[4], pose_wc[5], pose_wc[6],
    pose_wc[8], pose_wc[9], pose_wc[10],
  };
  float t[3] = { pose_wc[3], pose_wc[7], pose_wc[11] };
  // R^T (rows become columns)
  float Rt[9] = {
    R[0], R[3], R[6],
    R[1], R[4], R[7],
    R[2], R[5], R[8],
  };
  float nt[3] = {
    -(Rt[0]*t[0] + Rt[1]*t[1] + Rt[2]*t[2]),
    -(Rt[3]*t[0] + Rt[4]*t[1] + Rt[5]*t[2]),
    -(Rt[6]*t[0] + Rt[7]*t[1] + Rt[8]*t[2]),
  };
  pose_cw_out[0]  = Rt[0]; pose_cw_out[1]  = Rt[1]; pose_cw_out[2]  = Rt[2]; pose_cw_out[3]  = nt[0];
  pose_cw_out[4]  = Rt[3]; pose_cw_out[5]  = Rt[4]; pose_cw_out[6]  = Rt[5]; pose_cw_out[7]  = nt[1];
  pose_cw_out[8]  = Rt[6]; pose_cw_out[9]  = Rt[7]; pose_cw_out[10] = Rt[8]; pose_cw_out[11] = nt[2];
}

static inline int tvdb_grid_same_shape3(const tvdb_dense_grid* a, const tvdb_dense_grid* b) {
  return a->nx == b->nx && a->ny == b->ny && a->nz == b->nz;
}

// Shared inner kernel: project a voxel into the camera, return whether it
// integrated successfully and write color/sdf into out_sdf, out_iu, out_iv.
static bool tvdb_tsdf_project_voxel(int ix, int iy, int iz,
                                    const tvdb_dense_grid* tsdf,
                                    const tvdb_depth_frame* frame,
                                    const float pose_cw[12],
                                    float* out_sdf, int* out_iu, int* out_iv) {
  const float vs = tsdf->voxel_size;
  float wx = tsdf->ox + ((float)ix + 0.5f) * vs;
  float wy = tsdf->oy + ((float)iy + 0.5f) * vs;
  float wz = tsdf->oz + ((float)iz + 0.5f) * vs;
  float cx = pose_cw[0] * wx + pose_cw[1] * wy + pose_cw[2]  * wz + pose_cw[3];
  float cy = pose_cw[4] * wx + pose_cw[5] * wy + pose_cw[6]  * wz + pose_cw[7];
  float cz = pose_cw[8] * wx + pose_cw[9] * wy + pose_cw[10] * wz + pose_cw[11];
  if (cz <= 0.0f) return false;
  float u = frame->fx * (cx / cz) + frame->cx;
  float v = frame->fy * (cy / cz) + frame->cy;
  int iu = (int)floorf(u + 0.5f);
  int iv = (int)floorf(v + 0.5f);
  if (iu < 0 || iu >= frame->width || iv < 0 || iv >= frame->height) return false;
  float d = frame->depth[(size_t)iv * (size_t)frame->width + (size_t)iu];
  if (!(d >= frame->depth_min && d <= frame->depth_max)) return false;
  float sdf = d - cz;
  if (sdf < -frame->trunc_distance) return false;
  if (sdf > frame->trunc_distance) sdf = frame->trunc_distance;
  *out_sdf = sdf; *out_iu = iu; *out_iv = iv;
  return true;
}

bool tvdb_integrate_tsdf(tvdb_dense_grid* tsdf,
                         tvdb_dense_grid* weights,
                         const tvdb_depth_frame* frame) {
  if (!tsdf || !weights || !frame) return false;
  if (!tsdf->data || !weights->data || !frame->depth) return false;
  if (!tvdb_grid_same_shape3(tsdf, weights)) return false;

  float pose_cw[12];
  tvdb_invert_rigid_pose(frame->pose, pose_cw);

  const int nx = tsdf->nx, ny = tsdf->ny, nz = tsdf->nz;

  #pragma omp parallel for collapse(2) schedule(static)
  for (int iz = 0; iz < nz; ++iz) {
    for (int iy = 0; iy < ny; ++iy) {
      for (int ix = 0; ix < nx; ++ix) {
        float sdf; int iu, iv;
        if (!tvdb_tsdf_project_voxel(ix, iy, iz, tsdf, frame, pose_cw, &sdf, &iu, &iv))
          continue;
        size_t idx = tvdb_idx(tsdf, ix, iy, iz);
        float w_old = weights->data[idx];
        float t_old = tsdf->data[idx];
        float w_new = w_old + 1.0f;
        tsdf->data[idx]    = (t_old * w_old + sdf) / w_new;
        weights->data[idx] = w_new;
      }
    }
  }
  return true;
}

bool tvdb_integrate_tsdf_with_color(tvdb_dense_grid* tsdf,
                                    tvdb_dense_grid* weights,
                                    tvdb_dense_vec_grid* color,
                                    const tvdb_depth_frame* frame,
                                    const uint8_t* rgb) {
  if (!tsdf || !weights || !color || !frame || !rgb) return false;
  if (!tsdf->data || !weights->data || !color->data || !frame->depth) return false;
  if (!tvdb_grid_same_shape3(tsdf, weights)) return false;
  if (color->nx != tsdf->nx || color->ny != tsdf->ny || color->nz != tsdf->nz) return false;

  float pose_cw[12];
  tvdb_invert_rigid_pose(frame->pose, pose_cw);

  const int nx = tsdf->nx, ny = tsdf->ny, nz = tsdf->nz;

  #pragma omp parallel for collapse(2) schedule(static)
  for (int iz = 0; iz < nz; ++iz) {
    for (int iy = 0; iy < ny; ++iy) {
      for (int ix = 0; ix < nx; ++ix) {
        float sdf; int iu, iv;
        if (!tvdb_tsdf_project_voxel(ix, iy, iz, tsdf, frame, pose_cw, &sdf, &iu, &iv))
          continue;
        size_t idx = tvdb_idx(tsdf, ix, iy, iz);
        float w_old = weights->data[idx];
        float t_old = tsdf->data[idx];
        float w_new = w_old + 1.0f;
        tsdf->data[idx]    = (t_old * w_old + sdf) / w_new;
        weights->data[idx] = w_new;

        size_t pix = ((size_t)iv * (size_t)frame->width + (size_t)iu) * 3u;
        float r = (float)rgb[pix + 0];
        float g = (float)rgb[pix + 1];
        float b = (float)rgb[pix + 2];
        size_t cidx = idx * 3u;
        color->data[cidx + 0] = (color->data[cidx + 0] * w_old + r) / w_new;
        color->data[cidx + 1] = (color->data[cidx + 1] * w_old + g) / w_new;
        color->data[cidx + 2] = (color->data[cidx + 2] * w_old + b) / w_new;
      }
    }
  }
  return true;
}
