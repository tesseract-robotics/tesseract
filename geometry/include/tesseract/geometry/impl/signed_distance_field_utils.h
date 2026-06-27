/**
 * @file signed_distance_field_utils.h
 * @brief Utilities for discretizing a SignedDistanceField from a sampled distance function
 *
 * @author Joel Kang
 * @date June 15, 2026
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
#ifndef TESSERACT_GEOMETRY_SIGNED_DISTANCE_FIELD_UTILS_H
#define TESSERACT_GEOMETRY_SIGNED_DISTANCE_FIELD_UTILS_H

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <Eigen/Core>
#include <cstdint>
#include <functional>
#include <memory>
#include <vector>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/geometry/impl/signed_distance_field.h>

namespace tesseract::geometry
{

/**
 * @brief Discretize a @ref SignedDistanceField by sampling a signed distance function on a dense grid.
 *
 * Evaluates @p sdf at `dimensions.x() * dimensions.y() * dimensions.z()` points on a regular
 * axis-aligned lattice spanning [@p domain_min, @p domain_max] (inclusive of both corners) and
 * stores them as the field's backend-neutral grid. Each collision backend transcodes this grid into
 * its own native form when the geometry is added to a contact manager.
 *
 * @param sdf Signed distance function (negative inside, positive outside) in the field's local frame
 * @param domain_min Minimum corner of the sampled domain
 * @param domain_max Maximum corner of the sampled domain (must exceed @p domain_min on every axis)
 * @param dimensions Number of samples along each axis (each must be >= 2)
 * @param scale Local scaling applied to the field
 * @param margin Collision margin
 * @return A SignedDistanceField holding the sampled grid
 * @throw std::runtime_error on an invalid domain or dimensions
 */
SignedDistanceField::Ptr createDiscreteSignedDistanceField(const SignedDistanceFunction& sdf,
                                                           const Eigen::Vector3d& domain_min,
                                                           const Eigen::Vector3d& domain_max,
                                                           const Eigen::Vector3i& dimensions,
                                                           const Eigen::Vector3d& scale = Eigen::Vector3d(1, 1, 1),
                                                           double margin = 0.0);

/** @brief Batched overload — every grid sample position is handed to @p sdf in one call. */
SignedDistanceField::Ptr createDiscreteSignedDistanceField(const BatchedSignedDistanceFunction& sdf,
                                                           const Eigen::Vector3d& domain_min,
                                                           const Eigen::Vector3d& domain_max,
                                                           const Eigen::Vector3i& dimensions,
                                                           const Eigen::Vector3d& scale = Eigen::Vector3d(1, 1, 1),
                                                           double margin = 0.0);

/**
 * @brief Create a lazily-evaluated @ref SignedDistanceField backed by a distance function.
 *
 * Unlike @ref createDiscreteSignedDistanceField, the grid is not sampled up front: @p sdf remains the
 * field's source of truth, so queries and each collision backend evaluate it directly (exact, no
 * resampling). The dense grid is materialized only when concrete data is required (serialization or
 * comparison). The field cannot be serialized or compared from multiple threads concurrently until
 * discretized — call @ref SignedDistanceField::discretize up front if needed. @p sdf must be thread-safe.
 *
 * @param dimensions Number of samples along each axis to discretize when materialized (each must be >= 2)
 */
SignedDistanceField::Ptr createSignedDistanceField(const SignedDistanceFunction& sdf,
                                                   const Eigen::Vector3d& domain_min,
                                                   const Eigen::Vector3d& domain_max,
                                                   const Eigen::Vector3i& dimensions,
                                                   const Eigen::Vector3d& scale = Eigen::Vector3d(1, 1, 1),
                                                   double margin = 0.0);

/** @brief Batched overload — the sampler receives every grid sample position in one call. */
SignedDistanceField::Ptr createSignedDistanceField(const BatchedSignedDistanceFunction& sdf,
                                                   const Eigen::Vector3d& domain_min,
                                                   const Eigen::Vector3d& domain_max,
                                                   const Eigen::Vector3i& dimensions,
                                                   const Eigen::Vector3d& scale = Eigen::Vector3d(1, 1, 1),
                                                   double margin = 0.0);

/**
 * @brief Serialize a field's grid (domain, dimensions, distances) to a backend-neutral byte blob.
 * @details The local @c scale and @c margin are NOT stored; they are carried alongside the field
 * (e.g. as URDF attributes) and supplied on load. This is the format written/read for @c .sdf files.
 */
std::vector<std::uint8_t> writeSignedDistanceFieldData(const SignedDistanceField& sdf);

/**
 * @brief Reconstruct a @ref SignedDistanceField from a byte blob produced by
 * @ref writeSignedDistanceFieldData.
 * @param data The serialized grid blob
 * @param scale Local scaling applied to the field
 * @param margin Collision margin
 * @throw std::runtime_error if the blob is malformed
 */
SignedDistanceField::Ptr readSignedDistanceFieldData(const std::vector<std::uint8_t>& data,
                                                     const Eigen::Vector3d& scale = Eigen::Vector3d(1, 1, 1),
                                                     double margin = 0.0);
}  // namespace tesseract::geometry
#endif  // TESSERACT_GEOMETRY_SIGNED_DISTANCE_FIELD_UTILS_H
