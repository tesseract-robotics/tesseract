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

/**
 * @brief A volumetric signed distance field geometry.
 *
 * Stores the field in a backend-neutral form: the signed distance to a surface (negative inside)
 * sampled on a regular axis-aligned grid over a local-frame domain. This is the canonical,
 * engine-agnostic representation; each collision backend's geometry-to-shape converter transcodes
 * it into its own native form (e.g. the Bullet backend builds a btMiniSDF blob for
 * @c btSdfCollisionShape::initializeSDF; a VDB / GPU backend would build its own). Hold a distance
 * function with @ref createSignedDistanceField (discretized on demand), sample it onto a grid up
 * front with @ref createDiscreteSignedDistanceField, construct it directly from sampled data, or load
 * a serialized field with @ref readSignedDistanceFieldData.
 *
 * @note This is the signed-distance-field geometry; use it when the field is your native
 * representation (sensor-fused / analytic) or you need volumetric discrete collision.
 *
 * @note Distinct from the Gazebo "SDF" Simulation Description Format.
 *
 * @note Currently only the Bullet discrete collision backend consumes this geometry. It is concave,
 * so it is not supported by the continuous/cast managers or the FCL backend.
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
   * @param scale Local scaling applied to the field
   * @param margin Collision margin
   */
  SignedDistanceField(const Eigen::AlignedBox3d& domain,
                      const Eigen::Vector3i& dimensions,
                      std::vector<double> distances,
                      const Eigen::Vector3d& scale = Eigen::Vector3d(1, 1, 1),
                      double margin = 0.0);

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
   * @param scale Local scaling applied to the field
   * @param margin Collision margin
   */
  SignedDistanceField(const Eigen::AlignedBox3d& domain,
                      const Eigen::Vector3i& dimensions,
                      BatchedSignedDistanceFunction sampler,
                      const Eigen::Vector3d& scale = Eigen::Vector3d(1, 1, 1),
                      double margin = 0.0);

  SignedDistanceField() = default;
  ~SignedDistanceField() override = default;

  /** @brief Get the axis-aligned domain spanned by the sample grid (local frame) */
  const Eigen::AlignedBox3d& getDomain() const;

  /** @brief Get the number of samples along each axis */
  const Eigen::Vector3i& getDimensions() const;

  /**
   * @brief Get the sampled signed distances (row-major: index = i + nx*(j + ny*k)).
   * @note Triggers @ref discretize on a lazy field (materializes the grid from its sampler).
   */
  const std::vector<double>& getDistances() const;

  /** @brief Get the local scaling applied to the field */
  const Eigen::Vector3d& getScale() const;

  /** @brief Get the collision margin */
  double getMargin() const;

  /**
   * @brief Get the signed distance at a point in the field's local frame.
   * @details A function-backed field evaluates its sampler directly (exact); a grid-backed field
   * trilinearly interpolates the samples. The point is clamped to the domain.
   */
  double getDistance(const Eigen::Vector3d& point) const;

  /** @brief Whether the dense grid has been materialized (true for grid-backed and discretized fields) */
  bool isDiscretized() const;

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
  double margin_{ 0.0 };

  template <class Archive>
  friend void ::tesseract::geometry::serialize(Archive& ar, SignedDistanceField& obj);
};
}  // namespace tesseract::geometry

#endif  // TESSERACT_GEOMETRY_SIGNED_DISTANCE_FIELD_H
