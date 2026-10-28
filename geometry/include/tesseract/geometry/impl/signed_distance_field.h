/**
 * @file signed_distance_field.h
 * @brief Tesseract Signed Distance Field Geometry
 *
 * @author Joel Kang
 * @date June 10, 2026
 *
 * @copyright Copyright (c) 2026, Tesseract
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
#ifndef TESSERACT_GEOMETRY_SIGNED_DISTANCE_FIELD_H
#define TESSERACT_GEOMETRY_SIGNED_DISTANCE_FIELD_H

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstdint>
#include <functional>
#include <memory>
#include <vector>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/common/fwd.h>
#include <tesseract/geometry/geometry.h>

namespace tesseract::geometry
{
class SignedDistanceField;
template <class Archive>
void serialize(Archive& ar, SignedDistanceField& obj);

/** @brief Evaluate the signed distance at a single point in the field's local frame. */
using SignedDistanceFunction = std::function<double(const Eigen::Vector3d&)>;

/**
 * @brief Evaluate the signed distance at many points at once.
 *
 * Receives every sample point in one call and must return one distance per point, in the same
 * order. Use this overload to vectorize / offload the evaluation (e.g. a batched nvblox ESDF GPU
 * query) instead of paying a per-point call.
 */
using BatchedSignedDistanceFunction = std::function<std::vector<double>(const std::vector<Eigen::Vector3d>&)>;

class SparseSignedDistanceFieldGrid;

/**
 * @brief How a grid-backed or sparse field is read between its samples.
 * @details Both pass through the samples and reproduce a linear field exactly. They differ in how many
 * samples a read uses and whether the gradient is continuous across cell faces.
 */
enum class SignedDistanceFieldInterpolation : std::uint8_t
{
  /** @brief 2x2x2 samples. Passes through the samples; the gradient jumps at cell faces. The default. */
  TRILINEAR,
  /** @brief 4x4x4 samples, Catmull-Rom. Passes through the samples; the gradient is continuous. */
  TRICUBIC
};

/**
 * @brief A volumetric signed distance field geometry.
 *
 * Stores the field in a backend-neutral form: the signed distance to a surface (negative inside)
 * sampled on a regular axis-aligned grid over a local-frame domain. This is the canonical,
 * engine-agnostic representation; collision backends query it in place through @ref getDistance and
 * @ref getGradient rather than copying it into a native form. Hold a distance function with
 * @ref createSignedDistanceField (discretized on demand), sample it onto a grid up front with
 * @ref createDiscreteSignedDistanceField, construct it directly from sampled data, or load a
 * standard OpenVDB field with @ref readSignedDistanceFieldVDB.
 *
 * @note A function-backed field calls its function at every collision query. For an expensive
 * function (e.g. distance to a mesh), sample it up front with @ref createDiscreteSignedDistanceField.
 *
 * @note This is the signed-distance-field geometry; use it when the field is your native
 * representation (sensor-fused / analytic) or you need volumetric discrete collision.
 *
 * @note Distinct from the Gazebo "SDF" Simulation Description Format.
 *
 * A field can instead be SPARSE: one NanoVDB grid kept in its stored form (see the sparse
 * constructor). Memory then follows the surface rather than a box around it,
 * and no backend resamples it: queries read the grid directly through @ref getDistance
 * and @ref getGradient. A sparse field has no dense grid, so @ref getDistances throws.
 *
 * @note The Bullet and FCL discrete collision backends consume this geometry through the shared
 * implicit collision solver. It is concave, so it is not supported by continuous/cast managers.
 */
class SignedDistanceField : public Geometry
{
public:
  using Ptr = std::shared_ptr<SignedDistanceField>;
  using ConstPtr = std::shared_ptr<const SignedDistanceField>;

  /**
   * @brief Construct from a dense grid of signed distances sampled on a regular lattice.
   * @param domain The axis-aligned domain (local frame) spanned by the sample grid
   * @param dimensions The number of samples along each axis (>= 2 on every axis)
   * @param distances The sampled signed distances, row-major: index = i + nx*(j + ny*k), with
   * size == dimensions.prod(). Sample (i,j,k) sits at domain.min() + (i,j,k)/(dimensions-1) *
   * (domain.max() - domain.min())
   *
   * @param scale Finite, strictly positive local scaling applied to the field
   */
  SignedDistanceField(const Eigen::AlignedBox3d& domain,
                      const Eigen::Vector3i& dimensions,
                      std::vector<double> distances,
                      const Eigen::Vector3d& scale = Eigen::Vector3d(1, 1, 1));

  /**
   * @brief Construct a lazily-evaluated field backed by a distance function.
   *
   * No grid is sampled up front: @p sampler is the field's source of truth, so queries and each
   * collision backend evaluate it directly (exact, no resampling). The dense grid is materialized
   * (via @ref discretize) only when concrete data is required — serialization or equality comparison.
   *
   * @note A lazy field cannot be serialized or compared until discretized. @ref discretize is invoked
   * automatically at those boundaries; call it yourself to pin the snapshot at a defined time or to
   * run a costly/thread-unsafe @p sampler on your own thread. @p sampler must be thread-safe.
   *
   * @param domain The axis-aligned domain (local frame) to sample over when discretizing
   * @param dimensions The number of samples along each axis to discretize (>= 2 on every axis)
   * @param sampler The batched signed distance function (negative inside) in the field's local frame
   *
   * @param scale Finite, strictly positive local scaling applied to the field
   */
  SignedDistanceField(const Eigen::AlignedBox3d& domain,
                      const Eigen::Vector3i& dimensions,
                      BatchedSignedDistanceFunction sampler,
                      const Eigen::Vector3d& scale = Eigen::Vector3d(1, 1, 1));

  /**
   * @brief Construct a sparse field from one NanoVDB grid, kept in its stored sparse form.
   *
   * A query at a point reads the voxels around it. What an inactive voxel holds depends on the grid's
   * class. In a level set (NanoVDB grid class LevelSet) inactive voxels and tiles hold the
   * background, positive outside and negative inside, so they are read like active ones. In any
   * other grid an inactive voxel holds no distance, so the grid answers only where every voxel the
   * read uses is active (eight for trilinear).
   *
   * Where the grid cannot answer, and wherever it answers at or past @p band_clamp outside the
   * surface, it knows only that the surface is at least that far away: the distance is +infinity,
   * with a zero gradient. The field has no answer there; collision checks measure from where it
   * does, and another geometry on the same link, or on another link, still answers. A read at or
   * past the clamp inside the surface is returned as read: a contact, at least that deep.
   *
   * The domain is the grid's active voxel bounds. Values are read with the field's interpolation
   * (trilinear unless @ref setInterpolation says otherwise), and @ref getGradient returns the
   * interpolant's own gradient.
   *
   * @param nvdb One FloatGrid with an axis-aligned, uniformly scaled transform: a NanoVDB file holding
   * exactly that grid, as written by NanoVDB / fVDB (any codec the reader supports), or the bare grid
   * buffer. Voxel values are signed distances in the grid's own units, negative inside. To use one
   * grid of a file holding several, see @ref readSparseSignedDistanceFieldNVDB.
   * @param band_clamp The distance at which the grid's values saturate (a narrow band's clamp), in
   * grid units; for a level set usually its background. 0 means the values never saturate.
   * @param scale Finite, strictly positive local scaling applied to the field (e.g. 1e-3 for a grid
   * stored in millimetres and used in metres)
   * @throw std::runtime_error if @p nvdb is not a single uniform, axis-aligned NanoVDB FloatGrid of the
   * supported major version, is a fog volume, has nodes outside its buffer, or has no active voxels,
   * or if @p band_clamp is negative or not finite
   */
  SignedDistanceField(std::vector<std::uint8_t> nvdb,
                      double band_clamp,
                      const Eigen::Vector3d& scale = Eigen::Vector3d(1, 1, 1));

  /**
   * @brief Stack sparse fields of one surface as levels by resolution: a fallback, not a union.
   *
   * The levels are ordered finest first: level 0 is the finest, as in OpenVDB's tools::MultiResGrid.
   * A query at a point asks each level in turn and the first
   * that can answer does. A level other than the last also declines a read at or past its band clamp,
   * inside or outside the surface, which says only "at least this far", so a coarser level with real
   * range answers instead. The last level answers as it would alone (see the sparse constructor).
   * Typical use: a fine grid cropped around a region of interest over a coarse grid of the whole
   * object, so accuracy is paid for only where it is needed.
   *
   * The domain is the union of the levels' domains, and the field reads every level with its own
   * interpolation settings (@ref setInterpolation), not the levels'.
   *
   * @warning Every level must describe the same surface, in the same frame, at a different
   * resolution. The field answers from the first level that can, not from the nearest surface across
   * levels, so a level holding a different object would be hidden wherever an earlier level has
   * data. Give separate objects separate fields; several fields on a link combine as any geometries do.
   *
   * @param levels Sparse single-grid fields (see the sparse constructor), finest first, with equal scales
   * @throw std::runtime_error if @p levels is empty, a level is null, not a sparse single-grid field,
   * or has a different scale from the first, or a level has a smaller voxel size than the level before it
   */
  explicit SignedDistanceField(std::vector<SignedDistanceField::ConstPtr> levels);

  SignedDistanceField() = default;
  ~SignedDistanceField() override = default;

  /** @brief Get the axis-aligned domain spanned by the sample grid (local frame) */
  const Eigen::AlignedBox3d& getDomain() const;

  /** @brief Get the number of samples along each axis */
  const Eigen::Vector3i& getDimensions() const;

  /**
   * @brief Get the sampled signed distances (row-major: index = i + nx*(j + ny*k)).
   * @note Triggers @ref discretize on a lazy field (materializes the grid from its sampler).
   * @throw std::runtime_error on a sparse field, which has no dense grid
   */
  const std::vector<double>& getDistances() const;

  /** @brief Get the local scaling applied to the field */
  const Eigen::Vector3d& getScale() const;

  /**
   * @brief Get the signed distance at a point in the field's local frame.
   * @details A function-backed field evaluates its sampler directly (exact); a grid-backed field
   * interpolates the samples (see @ref setInterpolation), with the point clamped to the domain. A sparse field
   * answers from its grid (see the sparse constructor), and +infinity where the grid knows only
   * that the surface is at least its band clamp away.
   */
  double getDistance(const Eigen::Vector3d& point) const;

  /**
   * @brief Get the gradient of the signed distance at a point in the field's local frame.
   * @details Grid-backed and sparse fields return the exact gradient of their interpolant (see
   * @ref setInterpolation); a function-backed field takes central differences of its sampler. Not normalized.
   * Zero where a sparse field has no data. The local @c scale does not change the direction.
   */
  Eigen::Vector3d getGradient(const Eigen::Vector3d& point) const;

  /**
   * @brief Distance and gradient in one call: the same answers as @ref getDistance and
   * @ref getGradient, with one lookup instead of two in a sparse field.
   */
  double getDistanceAndGradient(const Eigen::Vector3d& point, Eigen::Vector3d& gradient) const;

  /** @brief Whether the dense grid has been materialized (true for grid-backed and discretized fields) */
  bool isDiscretized() const;

  /** @brief Whether this is a sparse field: one NanoVDB grid, or levels of them (@ref getLevels) */
  bool isSparse() const;

  /**
   * @brief Choose how @ref getDistance and @ref getGradient read between samples (grid-backed and
   * sparse fields; a function-backed field is always evaluated exactly). Sets both; follow with
   * @ref setGradientInterpolation to read the direction differently.
   * @details Near the edge of a sparse grid's active voxels, a wider stencil
   * (tricubic) needs more active neighbours, so it answers in a slightly smaller region.
   */
  void setInterpolation(SignedDistanceFieldInterpolation interpolation);
  /** @brief How the distance is read (@ref setInterpolation). */
  SignedDistanceFieldInterpolation getInterpolation() const;

  /**
   * @brief Read the gradient with a different method from the distance.
   * @details For example trilinear distance (it never overshoots the samples) with a tricubic
   * gradient (continuous across cell faces, for gradient-based optimisers). The gradient is then no
   * longer exactly the derivative of the reported distance. In a sparse field, where this method
   * cannot read (a wider stencil reaching past the active voxels), the gradient falls back to the
   * distance's method there.
   */
  void setGradientInterpolation(SignedDistanceFieldInterpolation interpolation);
  /** @brief How the gradient is read (defaults to the distance's method). */
  SignedDistanceFieldInterpolation getGradientInterpolation() const;

  /**
   * @brief A sparse field's grid, as its bare, uncompressed grid buffer (what the field reads in
   * place); empty for a dense, function-backed or multi-level field (a multi-level field's grids are its
   * levels').
   */
  const std::vector<std::uint8_t>& getNVDB() const;

  /** @brief A sparse field's band clamp (see the sparse constructor); 0 for a dense, function-backed or multi-level
   * field.
   */
  double getBandClamp() const;

  /** @brief A multi-level field's levels, finest first (see the multi-level constructor); empty otherwise. */
  const std::vector<SignedDistanceField::ConstPtr>& getLevels() const;

  /**
   * @brief Materialize the dense sample grid from the sampler (no-op if already discretized or grid-backed).
   * @details Thread-safe and idempotent. A lazy field must be discretized before it can be serialized or
   * compared; this happens automatically at those boundaries but may be called explicitly.
   */
  void discretize() const;

  Geometry::Ptr clone() const override final;

  bool operator==(const SignedDistanceField& rhs) const;
  bool operator!=(const SignedDistanceField& rhs) const;

private:
  /** @brief Throw if the domain is degenerate or any dimension is < 2. */
  void validateDomainAndDimensions() const;

  /** @brief Throw if the domain/dimensions are invalid or distances.size() != dimensions.prod(). */
  void validate() const;

  Eigen::AlignedBox3d domain_;
  Eigen::Vector3i dimensions_{ 0, 0, 0 };
  /// Sampled grid. Empty on a lazy field until discretized; @c mutable so discretize() can memoize on a const field.
  mutable std::vector<double> distances_;
  /// Source of truth for a lazy field; null on a grid-backed field. Not serialized.
  BatchedSignedDistanceFunction sampler_;
  Eigen::Vector3d scale_{ 1, 1, 1 };
  SignedDistanceFieldInterpolation interpolation_{ SignedDistanceFieldInterpolation::TRILINEAR };
  SignedDistanceFieldInterpolation gradient_interpolation_{ SignedDistanceFieldInterpolation::TRILINEAR };
  /// The parsed grid of a sparse field (immutable, shared between copies); null otherwise.
  std::shared_ptr<const SparseSignedDistanceFieldGrid> sparse_;
  /// The levels of a multi-level field (sparse single-grid fields, finest first); empty otherwise.
  std::vector<SignedDistanceField::ConstPtr> levels_;

  /** @brief Parse @p nvdb into @ref sparse_ and derive the domain and dimensions from it. */
  void initializeSparse(std::vector<std::uint8_t> nvdb, double band_clamp);

  /** @brief Check @p levels and derive the multi-level field's domain, dimensions and scale from them. */
  void initializeLevels(std::vector<SignedDistanceField::ConstPtr> levels);

  /** @brief A sparse or multi-level field's distance at @p point, and its gradient when @p gradient is set. */
  double querySparse(const Eigen::Vector3d& point, Eigen::Vector3d* gradient) const;

  /** @brief Read of the dense grid (point clamped to the domain), and its gradient when asked. */
  double interpolateGrid(const Eigen::Vector3d& point,
                         Eigen::Vector3d* gradient,
                         SignedDistanceFieldInterpolation method) const;

  template <class Archive>
  friend void ::tesseract::geometry::serialize(Archive& ar, SignedDistanceField& obj);
};
}  // namespace tesseract::geometry

#endif  // TESSERACT_GEOMETRY_SIGNED_DISTANCE_FIELD_H
