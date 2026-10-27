/**
 * @file signed_distance_field.cpp
 * @brief Tesseract Signed Distance Field Geometry
 *
 * @author Joel Kang
 * @date June 10, 2026
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <algorithm>
#include <array>
#include <boost/uuid/random_generator.hpp>
#include <cmath>
#include <cstring>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <string>

extern "C" {
#include <tinyvdb_nanovdb.h>
}
// PNanoVDB's cached read accessor (static inline, C mode): a query's voxels mostly share a
// leaf, so after the first lookup the rest are served from the accessor's cache. Fields can arrive
// from outside (files, ROS messages), so every buffer read is bounds checked: a malformed tree
// reads zeros instead of past the buffer.
#define PNANOVDB_C
#define PNANOVDB_BUF_BOUNDS_CHECK
#include <third_party/nanovdb/PNanoVDB.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/common/utils.h>
#include <tesseract/geometry/impl/signed_distance_field_utils.h>
#include <tesseract/geometry/impl/signed_distance_field.h>

namespace tesseract::geometry
{
namespace
{
/// Largest |index| a query may reach before it is treated as outside every grid (keeps int casts safe).
constexpr double MAX_INDEX_MAGNITUDE = 1e9;
/// A read within this fraction of a grid's band clamp counts as saturated: float-error sized, so
/// genuine interpolants just inside the clamp keep their meaning.
constexpr double SATURATION_RELATIVE_TOLERANCE = 1e-4;

/**
 * @brief One axis of an interpolation stencil: the samples it reads and their weights.
 * @details Samples first .. first + taps - 1 along the axis; @c weight[i] and @c slope[i] are the
 * weight of sample first + i and its derivative with respect to the position (index units).
 */
struct Stencil
{
  int first{ 0 };
  int taps{ 0 };
  std::array<double, 4> weight{};
  std::array<double, 4> slope{};
};

/**
 * @brief The stencil of @p method at position @p x (index units). With @p count > 0 the axis has
 * that many samples, and trilinear keeps its cell inside them (the last sample is a cell's far
 * corner, not a new cell); wider stencils clamp their indices when the samples are read.
 */
Stencil makeStencil(SignedDistanceFieldInterpolation method, double x, int count)
{
  Stencil s;
  switch (method)
  {
    case SignedDistanceFieldInterpolation::TRICUBIC:
    {
      // Catmull-Rom: passes through the samples, continuous first derivative.
      s.first = static_cast<int>(std::floor(x)) - 1;
      s.taps = 4;
      const double t = x - (s.first + 1);
      const double t2 = t * t;
      const double t3 = t2 * t;
      s.weight = {
        0.5 * (-t3 + (2 * t2) - t), 0.5 * ((3 * t3) - (5 * t2) + 2), 0.5 * ((-3 * t3) + (4 * t2) + t), 0.5 * (t3 - t2)
      };
      s.slope = { 0.5 * ((-3 * t2) + (4 * t) - 1),
                  0.5 * ((9 * t2) - (10 * t)),
                  0.5 * ((-9 * t2) + (8 * t) + 1),
                  0.5 * ((3 * t2) - (2 * t)) };
      return s;
    }
    case SignedDistanceFieldInterpolation::TRILINEAR:
    default:
    {
      s.first = static_cast<int>(std::floor(x));
      if (count > 0)
        s.first = std::min(s.first, count - 2);
      s.taps = 2;
      const double t = x - s.first;
      s.weight = { 1 - t, t, 0.0, 0.0 };
      s.slope = { -1.0, 1.0, 0.0, 0.0 };
      return s;
    }
  }
}

/**
 * @brief Tensor-product value and gradient (index units) of the TAPS^3 samples @p v
 * (index a + TAPS * (b + TAPS * c)) under the stencils @p sx, @p sy, @p sz.
 */
template <int TAPS>
double tensorInterpolate(const std::array<double, 64>& v,
                         const Stencil& sx,
                         const Stencil& sy,
                         const Stencil& sz,
                         Eigen::Vector3d* gradient)
{
  double value = 0;
  double gx = 0;
  double gy = 0;
  double gz = 0;
  for (int c = 0; c < TAPS; ++c)
  {
    double plane = 0;
    double plane_dx = 0;
    double plane_dy = 0;
    for (int b = 0; b < TAPS; ++b)
    {
      double row = 0;
      double row_dx = 0;
      for (int a = 0; a < TAPS; ++a)
      {
        const int i = a + (TAPS * (b + (TAPS * c)));
        const double sample = v[static_cast<std::size_t>(i)];
        row += sx.weight[static_cast<std::size_t>(a)] * sample;
        row_dx += sx.slope[static_cast<std::size_t>(a)] * sample;
      }
      plane += sy.weight[static_cast<std::size_t>(b)] * row;
      plane_dx += sy.weight[static_cast<std::size_t>(b)] * row_dx;
      plane_dy += sy.slope[static_cast<std::size_t>(b)] * row;
    }
    value += sz.weight[static_cast<std::size_t>(c)] * plane;
    gx += sz.weight[static_cast<std::size_t>(c)] * plane_dx;
    gy += sz.weight[static_cast<std::size_t>(c)] * plane_dy;
    gz += sz.slope[static_cast<std::size_t>(c)] * plane;
  }
  if (gradient != nullptr)
    *gradient = Eigen::Vector3d(gx, gy, gz);
  return value;
}

/**
 * @brief Interpolate at index-space position @p x, reading samples through @p fetch(i, j, k, value),
 * which returns false when a sample is unavailable (the read then fails).
 * @return false if @p fetch refused a sample
 */
template <typename Fetch>
bool interpolate(SignedDistanceFieldInterpolation method,
                 const Eigen::Vector3d& x,
                 const Eigen::Vector3i& count,
                 const Fetch& fetch,
                 double& value,
                 Eigen::Vector3d* gradient)
{
  const Stencil sx = makeStencil(method, x.x(), count.x());
  const Stencil sy = makeStencil(method, x.y(), count.y());
  const Stencil sz = makeStencil(method, x.z(), count.z());
  const int taps = sx.taps;
  std::array<double, 64> v{};
  for (int c = 0; c < taps; ++c)
    for (int b = 0; b < taps; ++b)
      for (int a = 0; a < taps; ++a)
      {
        const int i = a + (taps * (b + (taps * c)));
        if (!fetch(sx.first + a, sy.first + b, sz.first + c, v[static_cast<std::size_t>(i)]))
          return false;
      }

  switch (taps)
  {
    case 2:
      value = tensorInterpolate<2>(v, sx, sy, sz, gradient);
      break;
    default:
      value = tensorInterpolate<4>(v, sx, sy, sz, gradient);
      break;
  }
  return true;
}

/** @brief The index range a stencil of @p method reads at @p x, per axis (for bounds checks). */
void stencilBounds(SignedDistanceFieldInterpolation method,
                   const Eigen::Vector3d& x,
                   Eigen::Vector3i& lo,
                   Eigen::Vector3i& hi)
{
  for (int a = 0; a < 3; ++a)
  {
    const Stencil s = makeStencil(method, x[a], 0);
    lo[a] = s.first;
    hi[a] = s.first + s.taps - 1;
  }
}
}  // namespace

/**
 * @brief The parsed, immutable grid of a sparse field, shared by every copy of it.
 *
 * The grid is read with PNanoVDB straight from its bytes (the bare grid buffer), so there is one copy
 * of it and nothing is ever densified. Not copyable: the parsed grid points into the bytes.
 */
class SparseSignedDistanceFieldGrid
{
public:
  SparseSignedDistanceFieldGrid(const SparseSignedDistanceFieldGrid&) = delete;
  SparseSignedDistanceFieldGrid& operator=(const SparseSignedDistanceFieldGrid&) = delete;
  SparseSignedDistanceFieldGrid(SparseSignedDistanceFieldGrid&&) = delete;
  SparseSignedDistanceFieldGrid& operator=(SparseSignedDistanceFieldGrid&&) = delete;
  ~SparseSignedDistanceFieldGrid() = default;

  SparseSignedDistanceFieldGrid(std::vector<std::uint8_t> nvdb, double band_clamp) : nvdb_(std::move(nvdb))
  {
    grid_ = parse(nvdb_, band_clamp);
    domain_.setEmpty();
    domain_.extend(grid_.origin + (grid_.active_min.cast<double>() * grid_.voxel_size));
    domain_.extend(grid_.origin + (grid_.active_max.cast<double>() * grid_.voxel_size));
  }

  const std::vector<std::uint8_t>& nvdb() const { return nvdb_; }
  double bandClamp() const { return grid_.band_clamp; }
  const Eigen::AlignedBox3d& domain() const { return domain_; }
  double voxelSize() const { return grid_.voxel_size; }

  /**
   * @brief The field's distance at @p point (local frame), read with @p distance_method, and its
   * gradient when @p gradient is set, read with @p gradient_method.
   * @details +infinity, with a zero gradient, where the grid cannot answer or answers at or past its
   * band clamp outside the surface (see the sparse constructor). Where @p gradient_method cannot read
   * (a wider stencil reaching past the data), the direction falls back to @p distance_method there.
   */
  double query(const Eigen::Vector3d& point,
               Eigen::Vector3d* gradient,
               SignedDistanceFieldInterpolation distance_method,
               SignedDistanceFieldInterpolation gradient_method) const
  {
    double distance{ 0 };
    if (domain_.contains(point) && answer(point, gradient, distance_method, gradient_method, distance) &&
        !saturatedOutside(distance))
      return distance;
    if (gradient != nullptr)
      gradient->setZero();
    return std::numeric_limits<double>::infinity();
  }

private:
  struct Grid
  {
    /// The bare grid buffer. A std::vector's storage is aligned for any fundamental type, as
    /// PNanoVDB's 64-bit reads need.
    const std::uint8_t* data{ nullptr };
    std::uint64_t size_in_words{ 0 };  ///< Buffer length in 32-bit words (PNanoVDB's bounds check)
    double voxel_size{ 0 };            ///< Uniform voxel size (local units)
    Eigen::Vector3d origin;            ///< Local position of voxel (0, 0, 0)
    Eigen::Vector3i active_min;        ///< Active voxel index bounds (inclusive)
    Eigen::Vector3i active_max;
    double band_clamp{ 0 };
    /// A level set's inactive voxels and tiles hold its background (+ outside, - inside): data.
    bool level_set{ false };

    pnanovdb_buf_t buffer() const
    {
      // PNanoVDB reads through a non-const pointer; it never writes.
      // NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast, cppcoreguidelines-pro-type-reinterpret-cast)
      return pnanovdb_make_buf(reinterpret_cast<std::uint32_t*>(const_cast<std::uint8_t*>(data)), size_in_words);
    }
  };

  /**
   * @brief Whether @p bytes are a bare NanoVDB grid buffer rather than a NanoVDB file.
   * @details Grids carry "NanoVDB1", or "NanoVDB0" from older writers. "NanoVDB0" also opened old-format
   * files, so a buffer with it counts as a grid only when its GridData records a FloatGrid whose size
   * fits the buffer.
   */
  static bool isBareGrid(const std::vector<std::uint8_t>& bytes)
  {
    if (bytes.size() < PNANOVDB_GRID_SIZE + PNANOVDB_TREE_SIZE)
      return false;
    std::uint64_t magic{ 0 };
    std::uint64_t grid_size{ 0 };
    std::uint32_t grid_type{ 0 };
    std::memcpy(&magic, bytes.data() + PNANOVDB_GRID_OFF_MAGIC, sizeof(magic));
    std::memcpy(&grid_size, bytes.data() + PNANOVDB_GRID_OFF_GRID_SIZE, sizeof(grid_size));
    std::memcpy(&grid_type, bytes.data() + PNANOVDB_GRID_OFF_GRID_TYPE, sizeof(grid_type));
    if (magic == PNANOVDB_MAGIC_GRID)
      return true;
    return magic == PNANOVDB_MAGIC_NUMBER && grid_type == PNANOVDB_GRID_TYPE_FLOAT &&
           grid_size >= PNANOVDB_GRID_SIZE + PNANOVDB_TREE_SIZE && grid_size <= bytes.size();
  }

  /**
   * @brief Parse one grid, and rewrite @p nvdb in the canonical encoding, which the returned
   * grid then reads in place.
   *
   * Everything is read from the grid buffer itself (not the file's per-grid metadata), so a bare
   * grid buffer is accepted too. The canonical encoding is the bare, uncompressed grid buffer with no
   * checksum, recorded as a lone grid, its name kept. Two fields holding the same grid therefore
   * compare equal byte for byte, and a field written and read back is unchanged.
   */
  static Grid parse(std::vector<std::uint8_t>& nvdb, double band_clamp)
  {
    const std::string where = "SignedDistanceField: ";
    if (!std::isfinite(band_clamp) || band_clamp < 0.0)
      throw std::runtime_error(where + "band_clamp must be finite and non-negative");
    if (nvdb.empty())
      throw std::runtime_error(where + "the NanoVDB buffer is empty");

    if (!isBareGrid(nvdb))
    {
      tvdb_nanovdb_file_t file{};
      tvdb_error_t error{};
      const tvdb_status_t status = tvdb_nanovdb_file_open_memory(&file, nvdb.data(), nvdb.size(), nullptr, &error);
      // Closing also frees what a failed open allocated.
      const std::unique_ptr<tvdb_nanovdb_file_t, void (*)(tvdb_nanovdb_file_t*)> file_owner(&file,
                                                                                            &tvdb_nanovdb_file_close);
      if (status != TVDB_OK)
        throw std::runtime_error(where + (error.message[0] == '\0' ? "not a NanoVDB file" : error.message));
      if (file.num_grids != 1)
        throw std::runtime_error(where + "exactly one grid is required (readSparseSignedDistanceFieldNVDB "
                                         "selects one from a file holding several)");
      const tvdb_nanovdb_grid_t& g = file.grids[0];
      if (g.data == nullptr || g.size < PNANOVDB_GRID_SIZE + PNANOVDB_TREE_SIZE)
        throw std::runtime_error(where + "the grid holds no data");
      nvdb.assign(g.data, g.data + g.size);
    }

    Grid grid;
    grid.data = nvdb.data();
    grid.size_in_words = nvdb.size() / 4;
    pnanovdb_buf_t buf = grid.buffer();
    const pnanovdb_grid_handle_t handle = { { 0U } };
    const pnanovdb_uint64_t magic = pnanovdb_grid_get_magic(buf, handle);
    if (magic != PNANOVDB_MAGIC_GRID && magic != PNANOVDB_MAGIC_NUMBER)
      throw std::runtime_error(where + "the grid buffer does not start with NanoVDB grid data");
    const pnanovdb_uint64_t grid_size = pnanovdb_grid_get_grid_size(buf, handle);
    if (grid_size < PNANOVDB_GRID_SIZE + PNANOVDB_TREE_SIZE || grid_size > grid.size_in_words * 4)
      throw std::runtime_error(where + "the grid's recorded size does not match its buffer");
    nvdb.resize(static_cast<std::size_t>(grid_size));
    grid.data = nvdb.data();
    grid.size_in_words = grid_size / 4;
    buf = grid.buffer();
    if (pnanovdb_grid_get_grid_type(buf, handle) != PNANOVDB_GRID_TYPE_FLOAT)
      throw std::runtime_error(where + "only FloatGrids are supported");
    if (pnanovdb_version_get_major(pnanovdb_grid_get_version(buf, handle)) != PNANOVDB_MAJOR_VERSION_NUMBER)
      throw std::runtime_error(where + "unsupported NanoVDB version (major " +
                               std::to_string(PNANOVDB_MAJOR_VERSION_NUMBER) + " is required)");
    // A fog volume's values are densities, not distances. Other classes are accepted: writers differ
    // (fVDB marks its distance grids as index grids, with inactive voxels holding 0).
    const pnanovdb_uint32_t grid_class = pnanovdb_grid_get_grid_class(buf, handle);
    if (grid_class == PNANOVDB_GRID_CLASS_FOG_VOLUME)
      throw std::runtime_error(where + "a fog volume holds densities, not signed distances");
    grid.level_set = (grid_class == PNANOVDB_GRID_CLASS_LEVEL_SET);

    // Index-to-local transform: a uniform, axis-aligned scale plus a translation.
    const pnanovdb_map_handle_t map = pnanovdb_grid_get_map(buf, handle);
    const double voxel = pnanovdb_map_get_matd(buf, map, 0);
    if (!std::isfinite(voxel) || voxel <= 0.0)
      throw std::runtime_error(where + "the voxel size must be finite and positive");
    for (pnanovdb_uint32_t r = 0; r < 3; ++r)
    {
      for (pnanovdb_uint32_t c = 0; c < 3; ++c)
      {
        const double expected = (r == c) ? voxel : 0.0;
        if (std::abs(pnanovdb_map_get_matd(buf, map, (3 * r) + c) - expected) > 1e-9 * voxel)
          throw std::runtime_error(where + "only axis-aligned, uniformly scaled transforms are supported");
      }
    }
    grid.voxel_size = voxel;
    grid.origin = Eigen::Vector3d(
        pnanovdb_map_get_vecd(buf, map, 0), pnanovdb_map_get_vecd(buf, map, 1), pnanovdb_map_get_vecd(buf, map, 2));
    if (!grid.origin.allFinite())
      throw std::runtime_error(where + "the grid's translation must be finite");

    const pnanovdb_tree_handle_t tree = pnanovdb_grid_get_tree(buf, handle);
    // Every node array must lie inside the buffer: PNanoVDB's bounds-checked reads would otherwise
    // return zeros, which read as surface everywhere.
    const auto fits = [&](pnanovdb_uint64_t offset, pnanovdb_uint64_t count, pnanovdb_uint64_t node_size) {
      const pnanovdb_uint64_t start = tree.address.byte_offset + offset;
      return count == 0 || (start >= tree.address.byte_offset && count <= grid_size / node_size &&
                            start + (count * node_size) <= grid_size);
    };
    if (!fits(pnanovdb_tree_get_node_offset_leaf(buf, tree),
              pnanovdb_tree_get_node_count_leaf(buf, tree),
              PNANOVDB_GRID_TYPE_GET(PNANOVDB_GRID_TYPE_FLOAT, leaf_size)) ||
        !fits(pnanovdb_tree_get_node_offset_lower(buf, tree),
              pnanovdb_tree_get_node_count_lower(buf, tree),
              PNANOVDB_GRID_TYPE_GET(PNANOVDB_GRID_TYPE_FLOAT, lower_size)) ||
        !fits(pnanovdb_tree_get_node_offset_upper(buf, tree),
              pnanovdb_tree_get_node_count_upper(buf, tree),
              PNANOVDB_GRID_TYPE_GET(PNANOVDB_GRID_TYPE_FLOAT, upper_size)) ||
        !fits(pnanovdb_tree_get_node_offset_root(buf, tree),
              1,
              PNANOVDB_GRID_TYPE_GET(PNANOVDB_GRID_TYPE_FLOAT, root_size)))
      throw std::runtime_error(where + "the grid's nodes do not fit its buffer");
    const pnanovdb_root_handle_t root = pnanovdb_tree_get_root(buf, tree);
    const pnanovdb_coord_t bbox_min = pnanovdb_root_get_bbox_min(buf, root);
    const pnanovdb_coord_t bbox_max = pnanovdb_root_get_bbox_max(buf, root);
    grid.active_min = Eigen::Vector3i(bbox_min.x, bbox_min.y, bbox_min.z);
    grid.active_max = Eigen::Vector3i(bbox_max.x, bbox_max.y, bbox_max.z);
    const pnanovdb_uint64_t voxel_count = pnanovdb_tree_get_voxel_count(buf, tree);
    if (voxel_count == 0 || (grid.active_max.array() < grid.active_min.array()).any())
      throw std::runtime_error(where + "the grid has no active voxels");
    tightenActiveBounds(grid, tree, grid_size, where);
    grid.band_clamp = band_clamp;

    // Canonical header: no checksum (NanoVDB's "empty" value) and a lone grid; the name is kept.
    // The root's bounding box becomes the exact active bounds, so a reader that sizes a dense copy
    // from it (or from the file metadata written from it) spans the data and no more.
    const pnanovdb_coord_t tight_min = { grid.active_min.x(), grid.active_min.y(), grid.active_min.z() };
    const pnanovdb_coord_t tight_max = { grid.active_max.x(), grid.active_max.y(), grid.active_max.z() };
    pnanovdb_root_set_bbox_min(buf, root, &tight_min);
    pnanovdb_root_set_bbox_max(buf, root, &tight_max);
    pnanovdb_write_uint64(buf, pnanovdb_address_offset(handle.address, PNANOVDB_GRID_OFF_CHECKSUM), ~0ULL);
    pnanovdb_write_uint32(buf, pnanovdb_address_offset(handle.address, PNANOVDB_GRID_OFF_GRID_INDEX), 0U);
    pnanovdb_write_uint32(buf, pnanovdb_address_offset(handle.address, PNANOVDB_GRID_OFF_GRID_COUNT), 1U);
    return grid;
  }

  /**
   * @brief Shrink @p grid's active bounds to the active voxels themselves.
   * @details The root's bounding box may be loose (some writers record whole
   * internal nodes), and the field's domain is built from these bounds. When every active value lives
   * in a leaf, the leaves' value masks give the exact bounds: per leaf, a 64-bit word per x slice
   * with one byte per y row and one bit per z. Active tiles (in the root table or internal nodes)
   * hold values outside any leaf, so a tree with them keeps the root's box.
   */
  static void tightenActiveBounds(Grid& grid,
                                  const pnanovdb_tree_handle_t& tree,
                                  pnanovdb_uint64_t grid_size,
                                  const std::string& where)
  {
    const pnanovdb_buf_t buf = grid.buffer();
    if (pnanovdb_tree_get_tile_count_lower(buf, tree) != 0 || pnanovdb_tree_get_tile_count_upper(buf, tree) != 0)
      return;
    // The tree's root tile count is not reliably "active tiles" across writers, so read the root
    // table: an entry without a child is a tile, and its state says whether it is active.
    const pnanovdb_root_handle_t root = pnanovdb_tree_get_root(buf, tree);
    const pnanovdb_uint32_t root_entries = pnanovdb_root_get_tile_count(buf, root);
    for (pnanovdb_uint32_t n = 0; n < root_entries; ++n)
    {
      const pnanovdb_root_tile_handle_t tile = pnanovdb_root_get_tile(PNANOVDB_GRID_TYPE_FLOAT, root, n);
      if (pnanovdb_root_tile_get_child(buf, tile) == 0 && pnanovdb_root_tile_get_state(buf, tile) != 0)
        return;
    }

    const pnanovdb_uint64_t leaf_count = pnanovdb_tree_get_node_count_leaf(buf, tree);
    const pnanovdb_uint64_t leaf_size = PNANOVDB_GRID_TYPE_GET(PNANOVDB_GRID_TYPE_FLOAT, leaf_size);
    const pnanovdb_uint64_t first_leaf = tree.address.byte_offset + pnanovdb_tree_get_node_offset_leaf(buf, tree);
    if (leaf_count == 0 || first_leaf + (leaf_count * leaf_size) > grid_size)
      throw std::runtime_error(where + "the grid's leaf nodes do not fit its buffer");

    Eigen::Vector3i lo = Eigen::Vector3i::Constant(std::numeric_limits<int>::max());
    Eigen::Vector3i hi = Eigen::Vector3i::Constant(std::numeric_limits<int>::min());
    for (pnanovdb_uint64_t n = 0; n < leaf_count; ++n)
    {
      const pnanovdb_leaf_handle_t leaf = { pnanovdb_address_offset64(pnanovdb_address_null(),
                                                                      first_leaf + (n * leaf_size)) };
      const pnanovdb_coord_t corner = pnanovdb_leaf_get_bbox_min(buf, leaf);
      const Eigen::Vector3i origin(corner.x & ~7, corner.y & ~7, corner.z & ~7);
      for (int x = 0; x < 8; ++x)
      {
        const pnanovdb_uint64_t word = pnanovdb_read_uint64(
            buf, pnanovdb_address_offset(leaf.address, PNANOVDB_LEAF_OFF_VALUE_MASK + (8U * static_cast<unsigned>(x))));
        if (word == 0)
          continue;
        std::uint32_t z_bits = 0;
        for (int y = 0; y < 8; ++y)
        {
          const auto row = static_cast<std::uint32_t>((word >> (8U * static_cast<unsigned>(y))) & 0xFFU);
          if (row == 0)
            continue;
          z_bits |= row;
          lo.y() = std::min(lo.y(), origin.y() + y);
          hi.y() = std::max(hi.y(), origin.y() + y);
        }
        lo.x() = std::min(lo.x(), origin.x() + x);
        hi.x() = std::max(hi.x(), origin.x() + x);
        for (int z = 0; z < 8; ++z)
        {
          if ((z_bits & (1U << static_cast<unsigned>(z))) == 0)
            continue;
          lo.z() = std::min(lo.z(), origin.z() + z);
          hi.z() = std::max(hi.z(), origin.z() + z);
        }
      }
    }
    if ((hi.array() < lo.array()).any())
      throw std::runtime_error(where + "the grid has no active voxels");
    grid.active_min = grid.active_min.cwiseMax(lo);
    grid.active_max = grid.active_max.cwiseMin(hi);
  }

  /**
   * @brief Read the grid with @p method. Outside a level set, false unless every voxel the read uses
   * is active: there an inactive voxel holds no distance (fVDB writes 0), and blending it in would read
   * as surface. A level set's inactive voxels hold its background and are read like active ones.
   */
  static bool sample(const Grid& grid,
                     const Eigen::Vector3d& point,
                     SignedDistanceFieldInterpolation method,
                     double& distance,
                     Eigen::Vector3d* gradient)
  {
    const Eigen::Vector3d index = (point - grid.origin) / grid.voxel_size;
    if (!index.allFinite() || index.cwiseAbs().maxCoeff() > MAX_INDEX_MAGNITUDE)
      return false;
    Eigen::Vector3i lo;
    Eigen::Vector3i hi;
    stencilBounds(method, index, lo, hi);
    if ((lo.array() < grid.active_min.array()).any() || (hi.array() > grid.active_max.array()).any())
      return false;

    const pnanovdb_buf_t buf = grid.buffer();
    const pnanovdb_grid_handle_t grid_handle = { { 0U } };
    const pnanovdb_root_handle_t root = pnanovdb_tree_get_root(buf, pnanovdb_grid_get_tree(buf, grid_handle));
    pnanovdb_readaccessor_t accessor;
    pnanovdb_readaccessor_init(&accessor, root);
    const bool level_set = grid.level_set;
    const auto fetch = [&buf, &accessor, level_set](int i, int j, int k, double& value) {
      pnanovdb_coord_t ijk = { i, j, k };
      if (!level_set && pnanovdb_readaccessor_is_active(PNANOVDB_GRID_TYPE_FLOAT, buf, &accessor, &ijk) == 0)
        return false;
      const pnanovdb_address_t address =
          pnanovdb_readaccessor_get_value_address(PNANOVDB_GRID_TYPE_FLOAT, buf, &accessor, &ijk);
      value = static_cast<double>(pnanovdb_read_float(buf, address));
      return true;
    };

    if (!interpolate(method, index, Eigen::Vector3i::Zero(), fetch, distance, gradient))
      return false;
    if (gradient != nullptr)
      *gradient /= grid.voxel_size;
    return true;
  }

  /** @brief The grid's answer at @p point (see @ref query); false where it has no data. */
  bool answer(const Eigen::Vector3d& point,
              Eigen::Vector3d* gradient,
              SignedDistanceFieldInterpolation distance_method,
              SignedDistanceFieldInterpolation gradient_method,
              double& distance) const
  {
    const Grid& grid = grid_;
    const bool same_method = (distance_method == gradient_method);
    if (!sample(grid, point, distance_method, distance, same_method ? gradient : nullptr))
      return false;
    if (gradient != nullptr && !same_method)
    {
      double unused{ 0 };
      if (!sample(grid, point, gradient_method, unused, gradient))
        sample(grid, point, distance_method, unused, gradient);
    }
    return true;
  }

  /** @brief Whether @p distance says only "at least the band clamp away, outside". */
  bool saturatedOutside(double distance) const
  {
    return grid_.band_clamp > 0.0 && distance >= grid_.band_clamp * (1.0 - SATURATION_RELATIVE_TOLERANCE);
  }

  std::vector<std::uint8_t> nvdb_;
  Grid grid_;
  Eigen::AlignedBox3d domain_;
};

/** @brief Sample a batched distance function on the regular lattice spanning the domain. */
static std::vector<double> sampleGrid(const BatchedSignedDistanceFunction& sampler,
                                      const Eigen::AlignedBox3d& domain,
                                      const Eigen::Vector3i& dims)
{
  const Eigen::Vector3d step = (domain.max() - domain.min()).cwiseQuotient((dims.cast<double>().array() - 1).matrix());

  std::vector<Eigen::Vector3d> positions;
  positions.reserve(static_cast<std::size_t>(dims.x()) * static_cast<std::size_t>(dims.y()) *
                    static_cast<std::size_t>(dims.z()));
  // Row-major: index = i + nx*(j + ny*k)
  for (int k = 0; k < dims.z(); ++k)
    for (int j = 0; j < dims.y(); ++j)
      for (int i = 0; i < dims.x(); ++i)
        positions.emplace_back(
            domain.min().x() + (step.x() * i), domain.min().y() + (step.y() * j), domain.min().z() + (step.z() * k));

  std::vector<double> distances = sampler(positions);
  if (distances.size() != positions.size())
    throw std::runtime_error("SignedDistanceField: sampler returned " + std::to_string(distances.size()) +
                             " values for " + std::to_string(positions.size()) + " sample points");
  return distances;
}

SignedDistanceField::SignedDistanceField(const Eigen::AlignedBox3d& domain,
                                         const Eigen::Vector3i& dimensions,  // NOLINT
                                         std::vector<double> distances,
                                         const Eigen::Vector3d& scale)  // NOLINT
  : Geometry(GeometryType::SIGNED_DISTANCE_FIELD)
  , domain_(domain)
  , dimensions_(dimensions)
  , distances_(std::move(distances))
  , scale_(scale)
{
  validate();
}

SignedDistanceField::SignedDistanceField(const Eigen::AlignedBox3d& domain,
                                         const Eigen::Vector3i& dimensions,  // NOLINT
                                         BatchedSignedDistanceFunction sampler,
                                         const Eigen::Vector3d& scale)  // NOLINT
  : Geometry(GeometryType::SIGNED_DISTANCE_FIELD)
  , domain_(domain)
  , dimensions_(dimensions)
  , sampler_(std::move(sampler))
  , scale_(scale)
{
  validateDomainAndDimensions();
  if (sampler_ == nullptr)
    throw std::runtime_error("SignedDistanceField: sampler must not be null");
}

SignedDistanceField::SignedDistanceField(std::vector<std::uint8_t> nvdb,
                                         double band_clamp,
                                         const Eigen::Vector3d& scale)  // NOLINT
  : Geometry(GeometryType::SIGNED_DISTANCE_FIELD), scale_(scale)
{
  initializeSparse(std::move(nvdb), band_clamp);
}

void SignedDistanceField::initializeSparse(std::vector<std::uint8_t> nvdb, double band_clamp)
{
  auto sparse = std::make_shared<const SparseSignedDistanceFieldGrid>(std::move(nvdb), band_clamp);
  domain_ = sparse->domain();
  // Nominal sample counts at the grid's spacing; nothing is ever sampled at them, they only
  // describe the field's resolution to code that asks.
  const Eigen::Vector3d extent = domain_.max() - domain_.min();
  for (int a = 0; a < 3; ++a)
  {
    const double samples = std::round(extent[a] / sparse->voxelSize()) + 1.0;
    dimensions_[a] = static_cast<int>(std::clamp(samples, 2.0, static_cast<double>(std::numeric_limits<int>::max())));
  }
  distances_.clear();
  sampler_ = nullptr;
  sparse_ = std::move(sparse);
  validateDomainAndDimensions();
}

void SignedDistanceField::validateDomainAndDimensions() const
{
  if (dimensions_.x() < 2 || dimensions_.y() < 2 || dimensions_.z() < 2)
    throw std::runtime_error("SignedDistanceField: dimensions must be >= 2 on every axis");

  if (!domain_.min().allFinite() || !domain_.max().allFinite())
    throw std::runtime_error("SignedDistanceField: domain bounds must be finite");

  if ((domain_.max().array() <= domain_.min().array()).any())
    throw std::runtime_error("SignedDistanceField: domain max must exceed domain min on every axis");

  if (!scale_.allFinite() || (scale_.array() <= 0.0).any())
    throw std::runtime_error("SignedDistanceField: scale must be finite and positive on every axis");
}

void SignedDistanceField::validate() const
{
  validateDomainAndDimensions();
  if (sparse_ != nullptr)
    return;  // the grid was validated when parsed; a sparse field has no dense grid

  const auto expected = static_cast<std::size_t>(dimensions_.x()) * static_cast<std::size_t>(dimensions_.y()) *
                        static_cast<std::size_t>(dimensions_.z());
  if (distances_.size() != expected)
    throw std::runtime_error("SignedDistanceField: distances has " + std::to_string(distances_.size()) +
                             " values but the grid requires " + std::to_string(expected));
}

const Eigen::AlignedBox3d& SignedDistanceField::getDomain() const { return domain_; }

const Eigen::Vector3i& SignedDistanceField::getDimensions() const { return dimensions_; }

const std::vector<double>& SignedDistanceField::getDistances() const
{
  if (sparse_ != nullptr)
    throw std::runtime_error("SignedDistanceField: a sparse field has no dense grid; query it with getDistance / "
                             "getGradient, or encode it with writeSignedDistanceFieldNVDB");
  discretize();
  return distances_;
}

const Eigen::Vector3d& SignedDistanceField::getScale() const { return scale_; }

bool SignedDistanceField::isDiscretized() const { return !distances_.empty(); }

bool SignedDistanceField::isSparse() const { return sparse_ != nullptr; }

const std::vector<std::uint8_t>& SignedDistanceField::getNVDB() const
{
  static const std::vector<std::uint8_t> no_grid;
  return (sparse_ != nullptr) ? sparse_->nvdb() : no_grid;
}

double SignedDistanceField::getBandClamp() const { return (sparse_ != nullptr) ? sparse_->bandClamp() : 0.0; }

void SignedDistanceField::discretize() const
{
  if (sampler_ == nullptr)
    return;  // grid-backed (or empty): nothing to materialize

  // Discretizing is rare (serialization / comparison), so a single shared lock is cheaper than per-object
  // state and keeps the class trivially copyable. Once discretized, distances_ is never rewritten, so
  // later concurrent reads are safe.
  static std::mutex discretize_mutex;
  const std::lock_guard<std::mutex> lock(discretize_mutex);
  if (!distances_.empty())
    return;  // already discretized
  distances_ = sampleGrid(sampler_, domain_, dimensions_);
}

double SignedDistanceField::getDistance(const Eigen::Vector3d& point) const
{
  if (sparse_ != nullptr)
    return sparse_->query(point, nullptr, interpolation_, gradient_interpolation_);

  // A function-backed field evaluates its sampler directly (exact, no resampling).
  if (sampler_ != nullptr)
  {
    const std::vector<double> sampled = sampler_({ point });
    if (sampled.size() != 1)
      throw std::runtime_error("SignedDistanceField: sampler returned " + std::to_string(sampled.size()) +
                               " values for a single point");
    return sampled.front();
  }

  return interpolateGrid(point, nullptr, interpolation_);
}

Eigen::Vector3d SignedDistanceField::getGradient(const Eigen::Vector3d& point) const
{
  Eigen::Vector3d gradient = Eigen::Vector3d::Zero();
  if (sparse_ != nullptr)
  {
    sparse_->query(point, &gradient, interpolation_, gradient_interpolation_);
    return gradient;
  }

  if (sampler_ != nullptr)
  {
    // Central differences of the sampler. The step matches what the implicit collision solver used
    // before fields carried their own gradient: 1e-5 of the shortest domain extent, floored at 1e-6
    // in the scaled frame. A probe never steps out of the domain from a point inside it (the sampler
    // may be defined only there): at a boundary it is one-sided, divided by the spacing it actually spans.
    const double step = std::max(1e-6 / scale_.x(), (domain_.max() - domain_.min()).minCoeff() * 1e-5);  // local units
    std::vector<Eigen::Vector3d> probes;
    probes.reserve(6);
    Eigen::Vector3d spacing;
    for (Eigen::Index axis = 0; axis < 3; ++axis)
    {
      Eigen::Vector3d hi = point;
      Eigen::Vector3d lo = point;
      hi[axis] = std::min(point[axis] + step, std::max(point[axis], domain_.max()[axis]));
      lo[axis] = std::max(point[axis] - step, std::min(point[axis], domain_.min()[axis]));
      spacing[axis] = hi[axis] - lo[axis];
      probes.push_back(hi);
      probes.push_back(lo);
    }
    const std::vector<double> sampled = sampler_(probes);
    if (sampled.size() != probes.size())
      throw std::runtime_error("SignedDistanceField: sampler returned " + std::to_string(sampled.size()) +
                               " values for " + std::to_string(probes.size()) + " sample points");
    for (Eigen::Index axis = 0; axis < 3; ++axis)
      gradient[axis] = (sampled[static_cast<std::size_t>(2 * axis)] - sampled[static_cast<std::size_t>(2 * axis) + 1]) /
                       spacing[axis];
    return gradient;
  }

  interpolateGrid(point, &gradient, gradient_interpolation_);
  return gradient;
}

double SignedDistanceField::getDistanceAndGradient(const Eigen::Vector3d& point, Eigen::Vector3d& gradient) const
{
  if (sparse_ != nullptr)
    return sparse_->query(point, &gradient, interpolation_, gradient_interpolation_);
  if (sampler_ != nullptr)
  {
    gradient = getGradient(point);
    return getDistance(point);
  }
  if (interpolation_ == gradient_interpolation_)
    return interpolateGrid(point, &gradient, interpolation_);
  interpolateGrid(point, &gradient, gradient_interpolation_);
  return interpolateGrid(point, nullptr, interpolation_);
}

double SignedDistanceField::interpolateGrid(const Eigen::Vector3d& point,
                                            Eigen::Vector3d* gradient,
                                            SignedDistanceFieldInterpolation method) const
{
  const Eigen::Vector3d extent = domain_.max() - domain_.min();

  // Continuous grid coordinates in [0, dims-1] on each axis (clamped to the domain).
  Eigen::Vector3d grid;
  for (int a = 0; a < 3; ++a)
  {
    const double t = std::clamp((point[a] - domain_.min()[a]) / extent[a], 0.0, 1.0);
    grid[a] = t * static_cast<double>(dimensions_[a] - 1);
  }

  // Stencils wider than a cell reach past the first and last samples near the domain's faces;
  // those reads repeat the edge sample.
  const int nx = dimensions_.x();
  const int ny = dimensions_.y();
  const int nz = dimensions_.z();
  const auto fetch = [&](int i, int j, int k, double& value) {
    i = std::clamp(i, 0, nx - 1);
    j = std::clamp(j, 0, ny - 1);
    k = std::clamp(k, 0, nz - 1);
    value = distances_[static_cast<std::size_t>(i) +
                       static_cast<std::size_t>(nx) *
                           (static_cast<std::size_t>(j) + static_cast<std::size_t>(ny) * static_cast<std::size_t>(k))];
    return true;
  };

  double distance{ 0 };
  interpolate(method, grid, dimensions_, fetch, distance, gradient);
  if (gradient != nullptr)
  {
    // Index units -> local units: one cell spans extent / (dims - 1) on each axis.
    for (int a = 0; a < 3; ++a)
      (*gradient)[a] *= static_cast<double>(dimensions_[a] - 1) / extent[a];
  }
  return distance;
}

void SignedDistanceField::setInterpolation(SignedDistanceFieldInterpolation interpolation)
{
  interpolation_ = interpolation;
  gradient_interpolation_ = interpolation;
}

SignedDistanceFieldInterpolation SignedDistanceField::getInterpolation() const { return interpolation_; }

void SignedDistanceField::setGradientInterpolation(SignedDistanceFieldInterpolation interpolation)
{
  gradient_interpolation_ = interpolation;
}

SignedDistanceFieldInterpolation SignedDistanceField::getGradientInterpolation() const
{
  return gradient_interpolation_;
}

Geometry::Ptr SignedDistanceField::clone() const
{
  if (sparse_ != nullptr)
  {
    // The parsed grids are immutable, so the clone shares them; like any clone it gets its own UUID.
    auto copy = std::make_shared<SignedDistanceField>(*this);
    copy->setUUID(boost::uuids::random_generator()());
    return copy;
  }
  if (sampler_ != nullptr)
    return std::make_shared<SignedDistanceField>(domain_, dimensions_, sampler_, scale_);
  auto copy = std::make_shared<SignedDistanceField>(domain_, dimensions_, distances_, scale_);
  copy->interpolation_ = interpolation_;
  copy->gradient_interpolation_ = gradient_interpolation_;
  return copy;
}

bool SignedDistanceField::operator==(const SignedDistanceField& rhs) const
{
  // Sparse fields are equal when their grids and clamps are.
  if (sparse_ != nullptr || rhs.sparse_ != nullptr)
  {
    if (sparse_ == nullptr || rhs.sparse_ == nullptr)
      return false;
    return Geometry::operator==(rhs) && sparse_->nvdb() == rhs.sparse_->nvdb() &&
           sparse_->bandClamp() == rhs.sparse_->bandClamp() && scale_.isApprox(rhs.scale_, 1e-5) &&
           interpolation_ == rhs.interpolation_ && gradient_interpolation_ == rhs.gradient_interpolation_;
  }

  // Equality is defined on the materialized grids (samplers cannot be compared).
  discretize();
  rhs.discretize();

  bool equal = true;
  equal &= Geometry::operator==(rhs);
  equal &= dimensions_ == rhs.dimensions_;
  equal &= domain_.min().isApprox(rhs.domain_.min(), 1e-5);
  equal &= domain_.max().isApprox(rhs.domain_.max(), 1e-5);
  equal &= distances_ == rhs.distances_;
  equal &= scale_.isApprox(rhs.scale_, 1e-5);
  equal &= interpolation_ == rhs.interpolation_;
  equal &= gradient_interpolation_ == rhs.gradient_interpolation_;
  return equal;
}
bool SignedDistanceField::operator!=(const SignedDistanceField& rhs) const { return !operator==(rhs); }

}  // namespace tesseract::geometry
