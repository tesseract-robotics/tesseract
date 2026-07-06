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
#include <mutex>
#include <stdexcept>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/common/utils.h>
#include <tesseract/geometry/impl/signed_distance_field.h>

namespace tesseract::geometry
{
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
                                         const Eigen::Vector3d& scale,  // NOLINT
                                         double margin)
  : Geometry(GeometryType::SIGNED_DISTANCE_FIELD)
  , domain_(domain)
  , dimensions_(dimensions)
  , distances_(std::move(distances))
  , scale_(scale)
  , margin_(margin)
{
  validate();
}

SignedDistanceField::SignedDistanceField(const Eigen::AlignedBox3d& domain,
                                         const Eigen::Vector3i& dimensions,  // NOLINT
                                         BatchedSignedDistanceFunction sampler,
                                         const Eigen::Vector3d& scale,  // NOLINT
                                         double margin)
  : Geometry(GeometryType::SIGNED_DISTANCE_FIELD)
  , domain_(domain)
  , dimensions_(dimensions)
  , sampler_(std::move(sampler))
  , scale_(scale)
  , margin_(margin)
{
  validateDomainAndDimensions();
  if (sampler_ == nullptr)
    throw std::runtime_error("SignedDistanceField: sampler must not be null");
}

void SignedDistanceField::validateDomainAndDimensions() const
{
  if (dimensions_.x() < 2 || dimensions_.y() < 2 || dimensions_.z() < 2)
    throw std::runtime_error("SignedDistanceField: dimensions must be >= 2 on every axis");

  if ((domain_.max().array() <= domain_.min().array()).any())
    throw std::runtime_error("SignedDistanceField: domain max must exceed domain min on every axis");
}

void SignedDistanceField::validate() const
{
  validateDomainAndDimensions();

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
  discretize();
  return distances_;
}

const Eigen::Vector3d& SignedDistanceField::getScale() const { return scale_; }

double SignedDistanceField::getMargin() const { return margin_; }

bool SignedDistanceField::isDiscretized() const { return !distances_.empty(); }

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
  // A function-backed field evaluates its sampler directly (exact, no resampling).
  if (sampler_ != nullptr)
  {
    const std::vector<double> sampled = sampler_({ point });
    if (sampled.size() != 1)
      throw std::runtime_error("SignedDistanceField: sampler returned " + std::to_string(sampled.size()) +
                               " values for a single point");
    return sampled.front();
  }

  // Grid-backed: trilinear interpolation of the samples.
  const Eigen::Vector3d extent = domain_.max() - domain_.min();

  // Continuous grid coordinates in [0, dims-1] on each axis (clamped to the domain).
  Eigen::Vector3d grid;
  for (int a = 0; a < 3; ++a)
  {
    const double t = std::clamp((point[a] - domain_.min()[a]) / extent[a], 0.0, 1.0);
    grid[a] = t * static_cast<double>(dimensions_[a] - 1);
  }

  // Lower corner of the enclosing cell and the fractional position within it.
  const auto i0 = std::min(static_cast<int>(grid.x()), dimensions_.x() - 2);
  const auto j0 = std::min(static_cast<int>(grid.y()), dimensions_.y() - 2);
  const auto k0 = std::min(static_cast<int>(grid.z()), dimensions_.z() - 2);
  const double fx = grid.x() - i0;
  const double fy = grid.y() - j0;
  const double fz = grid.z() - k0;

  const int nx = dimensions_.x();
  const int ny = dimensions_.y();
  const auto at = [&](int i, int j, int k) -> double {
    return distances_[static_cast<std::size_t>(i) +
                      static_cast<std::size_t>(nx) *
                          (static_cast<std::size_t>(j) + static_cast<std::size_t>(ny) * static_cast<std::size_t>(k))];
  };

  // Trilinear blend of the 8 cell corners.
  const double c00 = at(i0, j0, k0) * (1 - fx) + at(i0 + 1, j0, k0) * fx;
  const double c10 = at(i0, j0 + 1, k0) * (1 - fx) + at(i0 + 1, j0 + 1, k0) * fx;
  const double c01 = at(i0, j0, k0 + 1) * (1 - fx) + at(i0 + 1, j0, k0 + 1) * fx;
  const double c11 = at(i0, j0 + 1, k0 + 1) * (1 - fx) + at(i0 + 1, j0 + 1, k0 + 1) * fx;
  const double c0 = c00 * (1 - fy) + c10 * fy;
  const double c1 = c01 * (1 - fy) + c11 * fy;
  return c0 * (1 - fz) + c1 * fz;
}

Geometry::Ptr SignedDistanceField::clone() const
{
  if (sampler_ != nullptr)
    return std::make_shared<SignedDistanceField>(domain_, dimensions_, sampler_, scale_, margin_);
  return std::make_shared<SignedDistanceField>(domain_, dimensions_, distances_, scale_, margin_);
}

bool SignedDistanceField::operator==(const SignedDistanceField& rhs) const
{
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
  equal &= tesseract::common::almostEqualRelativeAndAbs(margin_, rhs.margin_);
  return equal;
}
bool SignedDistanceField::operator!=(const SignedDistanceField& rhs) const { return !operator==(rhs); }

}  // namespace tesseract::geometry
