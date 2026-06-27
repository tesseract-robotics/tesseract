/**
 * @file signed_distance_field_utils.cpp
 * @brief Utilities for discretizing and (de)serializing a SignedDistanceField grid
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

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <array>
#include <stdexcept>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/common/utils.h>
#include <tesseract/geometry/impl/signed_distance_field_utils.h>

namespace tesseract::geometry
{
/** @brief Magic header and version tag for the backend-neutral grid blob. */
static constexpr std::array<std::uint8_t, 4> SDF_MAGIC = { 'T', 'S', 'D', 'F' };
static constexpr std::uint32_t SDF_VERSION = 1;

/** @brief Adapt a per-point distance function to the batched interface. */
static BatchedSignedDistanceFunction toBatched(const SignedDistanceFunction& sdf)
{
  return [sdf](const std::vector<Eigen::Vector3d>& points) {
    std::vector<double> distances;
    distances.reserve(points.size());
    for (const auto& point : points)
      distances.push_back(sdf(point));
    return distances;
  };
}

SignedDistanceField::Ptr createDiscreteSignedDistanceField(const SignedDistanceFunction& sdf,
                                                           const Eigen::Vector3d& domain_min,
                                                           const Eigen::Vector3d& domain_max,
                                                           const Eigen::Vector3i& dimensions,
                                                           const Eigen::Vector3d& scale,
                                                           double margin)
{
  return createDiscreteSignedDistanceField(toBatched(sdf), domain_min, domain_max, dimensions, scale, margin);
}

SignedDistanceField::Ptr createDiscreteSignedDistanceField(const BatchedSignedDistanceFunction& sdf,
                                                           const Eigen::Vector3d& domain_min,
                                                           const Eigen::Vector3d& domain_max,
                                                           const Eigen::Vector3i& dimensions,
                                                           const Eigen::Vector3d& scale,
                                                           double margin)
{
  // Eager: a pre-discretized lazy field, i.e. sample the grid up front so it is immediately serializable.
  auto field = createSignedDistanceField(sdf, domain_min, domain_max, dimensions, scale, margin);
  field->discretize();
  return field;
}

SignedDistanceField::Ptr createSignedDistanceField(const SignedDistanceFunction& sdf,
                                                   const Eigen::Vector3d& domain_min,
                                                   const Eigen::Vector3d& domain_max,
                                                   const Eigen::Vector3i& dimensions,
                                                   const Eigen::Vector3d& scale,
                                                   double margin)
{
  return createSignedDistanceField(toBatched(sdf), domain_min, domain_max, dimensions, scale, margin);
}

SignedDistanceField::Ptr createSignedDistanceField(const BatchedSignedDistanceFunction& sdf,
                                                   const Eigen::Vector3d& domain_min,
                                                   const Eigen::Vector3d& domain_max,
                                                   const Eigen::Vector3i& dimensions,
                                                   const Eigen::Vector3d& scale,
                                                   double margin)
{
  return std::make_shared<SignedDistanceField>(
      Eigen::AlignedBox3d(domain_min, domain_max), dimensions, sdf, scale, margin);
}

std::vector<std::uint8_t> writeSignedDistanceFieldData(const SignedDistanceField& sdf)
{
  std::vector<std::uint8_t> blob;

  blob.insert(blob.end(), SDF_MAGIC.begin(), SDF_MAGIC.end());
  tesseract::common::appendBytes(blob, SDF_VERSION);

  for (int i = 0; i < 3; ++i)
    tesseract::common::appendBytes(blob, sdf.getDomain().min()[i]);
  for (int i = 0; i < 3; ++i)
    tesseract::common::appendBytes(blob, sdf.getDomain().max()[i]);
  for (int i = 0; i < 3; ++i)
    tesseract::common::appendBytes(blob, static_cast<std::int32_t>(sdf.getDimensions()[i]));

  // getDistances() discretizes a lazy field, so a serialized blob always carries concrete samples.
  const std::vector<double>& distances = sdf.getDistances();
  tesseract::common::appendBytes(blob, static_cast<std::uint64_t>(distances.size()));
  for (const double value : distances)
    tesseract::common::appendBytes(blob, value);

  return blob;
}

SignedDistanceField::Ptr readSignedDistanceFieldData(const std::vector<std::uint8_t>& data,
                                                     const Eigen::Vector3d& scale,
                                                     double margin)
{
  std::size_t offset = 0;

  if (data.size() < SDF_MAGIC.size() || !std::equal(SDF_MAGIC.begin(), SDF_MAGIC.end(), data.begin()))
    throw std::runtime_error("readSignedDistanceFieldData: not a tesseract signed distance field blob");
  offset += SDF_MAGIC.size();

  const auto version = tesseract::common::readBytes<std::uint32_t>(data, offset);
  if (version != SDF_VERSION)
    throw std::runtime_error("readSignedDistanceFieldData: unsupported version " + std::to_string(version));

  Eigen::Vector3d domain_min;
  Eigen::Vector3d domain_max;
  for (int i = 0; i < 3; ++i)
    domain_min[i] = tesseract::common::readBytes<double>(data, offset);
  for (int i = 0; i < 3; ++i)
    domain_max[i] = tesseract::common::readBytes<double>(data, offset);

  Eigen::Vector3i dimensions;
  for (int i = 0; i < 3; ++i)
    dimensions[i] = static_cast<int>(tesseract::common::readBytes<std::int32_t>(data, offset));

  const auto count = tesseract::common::readBytes<std::uint64_t>(data, offset);
  std::vector<double> distances(static_cast<std::size_t>(count));
  for (auto& value : distances)
    value = tesseract::common::readBytes<double>(data, offset);

  return std::make_shared<SignedDistanceField>(
      Eigen::AlignedBox3d(domain_min, domain_max), dimensions, std::move(distances), scale, margin);
}

}  // namespace tesseract::geometry
