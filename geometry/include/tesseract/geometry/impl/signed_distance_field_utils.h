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
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
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
 * stores them as the field's backend-neutral grid. @p sdf is not kept: queries and the collision
 * backends interpolate the grid, so an expensive function is paid for once, here.
 *
 * @param sdf Signed distance function (negative inside, positive outside) in the field's local frame
 * @param domain_min Minimum corner of the sampled domain
 * @param domain_max Maximum corner of the sampled domain (must exceed @p domain_min on every axis)
 * @param dimensions Number of samples along each axis (each must be >= 2)
 * @param scale Local scaling applied to the field
 * @return A grid-backed SignedDistanceField holding the sampled grid
 * @throw std::runtime_error on an invalid domain or dimensions
 */
SignedDistanceField::Ptr createDiscreteSignedDistanceField(const SignedDistanceFunction& sdf,
                                                           const Eigen::Vector3d& domain_min,
                                                           const Eigen::Vector3d& domain_max,
                                                           const Eigen::Vector3i& dimensions,
                                                           const Eigen::Vector3d& scale = Eigen::Vector3d(1, 1, 1));

/** @brief Batched overload — every grid sample position is handed to @p sdf in one call. */
SignedDistanceField::Ptr createDiscreteSignedDistanceField(const BatchedSignedDistanceFunction& sdf,
                                                           const Eigen::Vector3d& domain_min,
                                                           const Eigen::Vector3d& domain_max,
                                                           const Eigen::Vector3i& dimensions,
                                                           const Eigen::Vector3d& scale = Eigen::Vector3d(1, 1, 1));

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
                                                   const Eigen::Vector3d& scale = Eigen::Vector3d(1, 1, 1));

/** @brief Batched overload — the sampler receives every grid sample position in one call. */
SignedDistanceField::Ptr createSignedDistanceField(const BatchedSignedDistanceFunction& sdf,
                                                   const Eigen::Vector3d& domain_min,
                                                   const Eigen::Vector3d& domain_max,
                                                   const Eigen::Vector3i& dimensions,
                                                   const Eigen::Vector3d& scale = Eigen::Vector3d(1, 1, 1));

/**
 * @brief Serialize a field as a standard OpenVDB FloatGrid.
 * @details The grid transform maps lattice index (0, 0, 0) to the field domain minimum and uses
 * the field sample spacing as its uniform voxel size. The local @c scale is not stored; it is
 * carried alongside the field (e.g. as a URDF attribute) and supplied on load.
 * @throws std::runtime_error if the field has non-uniform voxel spacing or cannot be encoded, or is
 * sparse (a sparse field has no single dense grid; use @ref writeSignedDistanceFieldNVDB).
 */
std::vector<std::uint8_t> writeSignedDistanceFieldVDB(const SignedDistanceField& sdf);

/**
 * @brief Reconstruct a @ref SignedDistanceField from a standard OpenVDB FloatGrid byte buffer.
 * @details Exactly one FloatGrid with an axis-aligned, uniformly scaled transform is supported.
 * Its active voxel bounding box defines the finite Tesseract field domain; inactive voxels within
 * that domain use the grid background value.
 * @param data Pointer to the serialized grid blob
 * @param size Number of bytes in @p data
 * @param scale Local scaling applied to the field
 * @throw std::runtime_error if the VDB is malformed or uses an unsupported grid or transform
 */
SignedDistanceField::Ptr readSignedDistanceFieldVDB(const std::uint8_t* data,
                                                    std::size_t size,
                                                    const Eigen::Vector3d& scale = Eigen::Vector3d(1, 1, 1));

/**
 * @brief Reconstruct a @ref SignedDistanceField from a standard OpenVDB FloatGrid byte vector.
 * @details Convenience overload for an owning byte vector. Use the pointer-and-size overload to
 * read from other contiguous storage without a copy.
 */
SignedDistanceField::Ptr readSignedDistanceFieldVDB(const std::vector<std::uint8_t>& data,
                                                    const Eigen::Vector3d& scale = Eigen::Vector3d(1, 1, 1));

/**
 * @brief Serialize a field as a standard NanoVDB FloatGrid file.
 * @details The field's local @c scale is not stored and must be supplied separately when loading.
 * A sparse field is written as its FloatGrid, as stored. Its band clamp is not stored either: pass
 * it to @ref readSparseSignedDistanceFieldNVDB to read the file back as an equal sparse field.
 * @throws std::runtime_error if the field cannot be represented as a uniform FloatGrid.
 */
std::vector<std::uint8_t> writeSignedDistanceFieldNVDB(const SignedDistanceField& sdf);

/**
 * @brief Reconstruct a dense @ref SignedDistanceField from a NanoVDB FloatGrid byte buffer.
 * @details Exactly one FloatGrid with an axis-aligned, uniformly scaled transform is supported, and
 * its active bounds are read into a dense grid. To keep a file sparse, use
 * @ref readSparseSignedDistanceFieldNVDB.
 * @param data Pointer to the serialized NanoVDB file
 * @param size Number of bytes in @p data
 * @param scale Local scaling applied to the field
 * @throw std::runtime_error if the NanoVDB file is malformed or uses an unsupported grid or transform
 */
SignedDistanceField::Ptr readSignedDistanceFieldNVDB(const std::uint8_t* data,
                                                     std::size_t size,
                                                     const Eigen::Vector3d& scale = Eigen::Vector3d(1, 1, 1));

/**
 * @brief Reconstruct a @ref SignedDistanceField from a NanoVDB FloatGrid byte vector.
 * @details Convenience overload for an owning byte vector. Use the pointer-and-size overload to
 * read from other contiguous storage without a copy.
 */
SignedDistanceField::Ptr readSignedDistanceFieldNVDB(const std::vector<std::uint8_t>& data,
                                                     const Eigen::Vector3d& scale = Eigen::Vector3d(1, 1, 1));

/**
 * @brief The names of the grids in a NanoVDB file, in file order.
 * @details A file can hold several grids: level sets of different objects, or anything else. Read
 * one as a field with @ref readSparseSignedDistanceFieldNVDB; how several relate is the caller's to say.
 * @throw std::runtime_error if the NanoVDB file is malformed
 */
std::vector<std::string> getSignedDistanceFieldNVDBGridNames(const std::uint8_t* data, std::size_t size);

/** @brief Convenience overload of @ref getSignedDistanceFieldNVDBGridNames for an owning byte vector. */
std::vector<std::string> getSignedDistanceFieldNVDBGridNames(const std::vector<std::uint8_t>& data);

/**
 * @brief Read one FloatGrid of a NanoVDB file as a sparse @ref SignedDistanceField, kept in its stored
 * sparse form (see the sparse @ref SignedDistanceField constructor).
 * @param data Pointer to the serialized NanoVDB file
 * @param size Number of bytes in @p data
 * @param band_clamp The grid's band clamp, in grid units (0: the values never saturate)
 * @param scale Local scaling applied to the field
 * @param grid_name The grid to read; may be empty only when the file holds exactly one grid
 * @throw std::runtime_error if the file is malformed, @p grid_name is empty and the file holds more
 * than one grid, no grid has that name (or more than one does), or the grid is not a valid sparse field
 */
SignedDistanceField::Ptr readSparseSignedDistanceFieldNVDB(const std::uint8_t* data,
                                                           std::size_t size,
                                                           double band_clamp,
                                                           const Eigen::Vector3d& scale = Eigen::Vector3d(1, 1, 1),
                                                           const std::string& grid_name = "");

/** @brief Convenience overload of @ref readSparseSignedDistanceFieldNVDB for an owning byte vector. */
SignedDistanceField::Ptr readSparseSignedDistanceFieldNVDB(const std::vector<std::uint8_t>& data,
                                                           double band_clamp,
                                                           const Eigen::Vector3d& scale = Eigen::Vector3d(1, 1, 1),
                                                           const std::string& grid_name = "");
}  // namespace tesseract::geometry
#endif  // TESSERACT_GEOMETRY_SIGNED_DISTANCE_FIELD_UTILS_H
