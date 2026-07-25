/**
 * @file implicit_sdf_collision_solver.h
 * @brief Backend-neutral collision queries between implicit signed-distance shapes.
 *
 * @copyright Copyright (c) 2026, Tesseract Robotics
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
#ifndef TESSERACT_COLLISION_IMPLICIT_SDF_COLLISION_SOLVER_H
#define TESSERACT_COLLISION_IMPLICIT_SDF_COLLISION_SOLVER_H

#include <Eigen/Geometry>
#include <array>
#include <functional>
#include <vector>

namespace tesseract::geometry
{
class Geometry;
}

namespace tesseract::collision
{
/** @brief An implicit shape whose signed-distance queries and bounds are expressed in world coordinates. */
struct ImplicitSDFShape
{
  using DistanceFunction = std::function<double(const Eigen::Vector3d&)>;
  using GradientFunction = std::function<Eigen::Vector3d(const Eigen::Vector3d&)>;

  DistanceFunction distance;
  /** @brief Distance gradient; it may be non-unit or zero at nondifferentiable points. */
  GradientFunction gradient;
  Eigen::AlignedBox3d aabb;

  /** @brief Return whether the shape has all data required by the solver. */
  bool isValid() const;
};

/** @brief Configuration for the multi-start implicit SDF collision solver. */
struct ImplicitSDFCollisionConfig
{
  int initial_sample_count{ 16 };
  int max_iterations{ 20 };
  int max_contacts{ 4 };
  double contact_margin{ 0.0 };
  double minimum_step{ 1e-4 };
  double duplicate_tolerance{ 1e-5 };
};

/** @brief Contact produced by the implicit SDF collision solver. */
struct ImplicitSDFContact
{
  double distance{ 0.0 };
  Eigen::Vector3d point{ Eigen::Vector3d::Zero() };
  Eigen::Vector3d normal{ Eigen::Vector3d::UnitX() };
  std::array<Eigen::Vector3d, 2> nearest_points{ Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero() };
};

/**
 * @brief Find contacts between two implicit signed-distance shapes.
 *
 * This adapts MuJoCo's multi-start SDF collision strategy: deterministic Halton seeds are optimized
 * over the overlap of the shapes' margin-expanded AABBs using a composite collision objective and
 * backtracking gradient descent. Contact distance and nearest points are then recovered by projecting
 * the converged point onto both zero level sets.
 *
 * @param shape0 First implicit shape
 * @param shape1 Second implicit shape
 * @param config Solver configuration
 * @return Contacts with normals directed from @p shape0 toward @p shape1
 * @throws std::invalid_argument If either shape or any solver configuration value is invalid
 */
std::vector<ImplicitSDFContact> collideImplicitSDF(const ImplicitSDFShape& shape0,
                                                   const ImplicitSDFShape& shape1,
                                                   const ImplicitSDFCollisionConfig& config = {});

/**
 * @brief Create a world-coordinate implicit representation of a supported Tesseract geometry.
 *
 * Supported geometries are box, sphere, cylinder, cone, capsule, and signed distance field.
 * This adapter is backend-neutral and is used by FCL; other backends may provide callbacks around
 * their native distance representations and call @ref collideImplicitSDF directly.
 *
 * @param geometry Geometry to represent
 * @param pose Geometry pose in world coordinates
 * @return A valid implicit shape when the geometry is supported, otherwise an invalid shape
 * @throws std::invalid_argument If the pose or supported geometry data is invalid, or if an SDF has nonuniform scale
 */
ImplicitSDFShape makeImplicitSDFShape(const tesseract::geometry::Geometry& geometry, const Eigen::Isometry3d& pose);

}  // namespace tesseract::collision

#endif  // TESSERACT_COLLISION_IMPLICIT_SDF_COLLISION_SOLVER_H
