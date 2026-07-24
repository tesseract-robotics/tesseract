// Grid statistics and diagnostics. See tinyvdb_stats.h. Dense data is x-fastest:
// idx(i,j,k) = (k*ny + j)*nx + i.

#include "tinyvdb_stats.h"

#include <math.h>

bool tvdb_grid_statistics(const tvdb_dense_grid* grid, tvdb_grid_stats_t* out) {
  if (!out) return false;
  out->min = out->max = out->mean = out->stddev = out->sum = 0.0;
  out->count = 0;
  if (!grid || !grid->data) return false;
  size_t n = (size_t)grid->nx * grid->ny * grid->nz;
  if (n == 0) return false;

  double mn = grid->data[0], mx = grid->data[0], sum = 0.0;
  for (size_t i = 0; i < n; ++i) {
    double v = grid->data[i];
    if (v < mn) mn = v;
    if (v > mx) mx = v;
    sum += v;
  }
  double mean = sum / (double)n;
  // Second pass for a numerically stable variance.
  double var = 0.0;
  for (size_t i = 0; i < n; ++i) {
    double d = (double)grid->data[i] - mean;
    var += d * d;
  }
  var /= (double)n;
  out->min = mn; out->max = mx; out->mean = mean;
  out->stddev = sqrt(var); out->sum = sum; out->count = n;
  return true;
}

bool tvdb_grid_histogram(const tvdb_dense_grid* grid, double range_min,
                         double range_max, int nbins, size_t* out_counts) {
  if (!grid || !grid->data || !out_counts || nbins < 1 || range_max <= range_min)
    return false;
  for (int b = 0; b < nbins; ++b) out_counts[b] = 0;
  size_t n = (size_t)grid->nx * grid->ny * grid->nz;
  double inv = (double)nbins / (range_max - range_min);
  for (size_t i = 0; i < n; ++i) {
    double v = grid->data[i];
    int b = (int)((v - range_min) * inv);
    if (b < 0) b = 0;
    if (b >= nbins) b = nbins - 1;
    ++out_counts[b];
  }
  return true;
}

bool tvdb_check_level_set(const tvdb_dense_grid* grid, double band_world,
                          double tol, tvdb_level_set_check_t* out) {
  if (!out) return false;
  out->mean_grad_mag = 0.0; out->max_grad_error = 0.0;
  out->bad_fraction = 0.0; out->band_count = 0;
  if (!grid || !grid->data) return false;
  int nx = grid->nx, ny = grid->ny, nz = grid->nz;
  if (nx < 3 || ny < 3 || nz < 3) return false;
  double inv2vs = 1.0 / (2.0 * (double)grid->voxel_size);

  size_t band = 0, bad = 0;
  double sum_mag = 0.0, max_err = 0.0;
  for (int k = 1; k < nz - 1; ++k)
    for (int j = 1; j < ny - 1; ++j)
      for (int i = 1; i < nx - 1; ++i) {
        size_t c = ((size_t)k * ny + j) * nx + i;
        if (band_world > 0.0 && fabs((double)grid->data[c]) > band_world) continue;
        double gx = ((double)grid->data[c + 1] - grid->data[c - 1]) * inv2vs;
        double gy = ((double)grid->data[c + nx] - grid->data[c - nx]) * inv2vs;
        size_t sl = (size_t)nx * ny;
        double gz = ((double)grid->data[c + sl] - grid->data[c - sl]) * inv2vs;
        double mag = sqrt(gx * gx + gy * gy + gz * gz);
        double err = fabs(mag - 1.0);
        sum_mag += mag;
        if (err > max_err) max_err = err;
        if (err > tol) ++bad;
        ++band;
      }
  out->band_count = band;
  if (band > 0) {
    out->mean_grad_mag = sum_mag / (double)band;
    out->bad_fraction = (double)bad / (double)band;
    out->max_grad_error = max_err;
  }
  return true;
}

bool tvdb_check_fog_volume(const tvdb_dense_grid* grid, double eps,
                           int* out_valid, double* out_min, double* out_max) {
  if (!grid || !grid->data) return false;
  size_t n = (size_t)grid->nx * grid->ny * grid->nz;
  if (n == 0) return false;
  double mn = grid->data[0], mx = grid->data[0];
  for (size_t i = 0; i < n; ++i) {
    double v = grid->data[i];
    if (v < mn) mn = v;
    if (v > mx) mx = v;
  }
  if (out_min) *out_min = mn;
  if (out_max) *out_max = mx;
  if (out_valid) *out_valid = (mn >= -eps && mx <= 1.0 + eps) ? 1 : 0;
  return true;
}
