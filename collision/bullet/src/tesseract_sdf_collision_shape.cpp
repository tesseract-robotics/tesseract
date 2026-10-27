/**
 * @file tesseract_sdf_collision_shape.cpp
 * @brief Bullet collision shape that queries a Tesseract SignedDistanceField directly.
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

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <BulletCollision/BroadphaseCollision/btBroadphaseProxy.h>
#include <BulletCollision/CollisionShapes/btSdfCollisionShape.h>
#include <cmath>
#include <limits>
#include <stdexcept>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/bullet/bullet_utils.h>
#include <tesseract/collision/bullet/tesseract_sdf_collision_shape.h>

namespace tesseract::collision::bullet_internal
{
TesseractSdfCollisionShape::TesseractSdfCollisionShape(tesseract::geometry::SignedDistanceField::ConstPtr field)
  : field_(std::move(field))
{
  if (field_ == nullptr)
    throw std::invalid_argument("TesseractSdfCollisionShape: field must not be null");
  m_shapeType = SDF_SHAPE_PROXYTYPE;
}

void TesseractSdfCollisionShape::getAabb(const btTransform& t, btVector3& aabbMin, btVector3& aabbMax) const
{
  const Eigen::Vector3d& scale = field_->getScale();
  const btVector3 local_min = convertEigenToBt(Eigen::Vector3d(field_->getDomain().min().cwiseProduct(scale)));
  const btVector3 local_max = convertEigenToBt(Eigen::Vector3d(field_->getDomain().max().cwiseProduct(scale)));
  btTransformAabb(local_min, local_max, getMargin(), t, aabbMin, aabbMax);
}

void TesseractSdfCollisionShape::setLocalScaling(const btVector3& scaling)
{
  if ((scaling - btVector3(1, 1, 1)).fuzzyZero())
    return;
  throw std::invalid_argument("TesseractSdfCollisionShape: scale the field (SignedDistanceField scale), not the shape");
}

const btVector3& TesseractSdfCollisionShape::getLocalScaling() const { return local_scaling_; }

void TesseractSdfCollisionShape::calculateLocalInertia(btScalar /*mass*/, btVector3& inertia) const
{
  inertia.setValue(0, 0, 0);
}

const char* TesseractSdfCollisionShape::getName() const { return "TesseractSdfCollisionShape"; }

void TesseractSdfCollisionShape::processAllTriangles(btTriangleCallback* /*callback*/,
                                                     const btVector3& /*aabbMin*/,
                                                     const btVector3& /*aabbMax*/) const
{
}

bool TesseractSdfCollisionShape::queryPoint(const btVector3& local_point, btScalar& distance, btVector3& normal) const
{
  // Uniform scale s (enforced by the implicit solver): distance s * d(p / s), gradient unchanged.
  const Eigen::Vector3d& scale = field_->getScale();
  const Eigen::Vector3d point = convertBtToEigen(local_point).cwiseQuotient(scale);
  if (!field_->getDomain().contains(point))
    return false;
  Eigen::Vector3d gradient;
  const double d = field_->getDistanceAndGradient(point, gradient) * scale.x();
  if (!std::isfinite(d))
    return false;
  if (gradient.squaredNorm() > 0.0)
    gradient.normalize();
  distance = static_cast<btScalar>(d);
  normal = convertEigenToBt(gradient);
  return true;
}

bool TesseractSdfCollisionShape::queryDistance(const btVector3& local_point, btScalar& distance) const
{
  const Eigen::Vector3d& scale = field_->getScale();
  const Eigen::Vector3d point = convertBtToEigen(local_point).cwiseQuotient(scale);
  if (!field_->getDomain().contains(point))
    return false;
  const double d = field_->getDistance(point) * scale.x();
  if (!std::isfinite(d))
    return false;
  distance = static_cast<btScalar>(d);
  return true;
}

const tesseract::geometry::SignedDistanceField& TesseractSdfCollisionShape::getField() const { return *field_; }

namespace
{
/** @brief The solver's view of Bullet's own btMiniSDF-backed shape, through its queryPoint. */
ImplicitSDFShape makeBulletMiniSdfShape(const btSdfCollisionShape& shape, const btTransform& world_transform)
{
  const btTransform inverse_transform = world_transform.inverse();
  ImplicitSDFShape result;
  result.distance = [&shape, inverse_transform](const Eigen::Vector3d& point) {
    btScalar distance{ 0 };
    btVector3 normal;
    if (!querySdfShapePoint(shape, inverse_transform * convertEigenToBt(point), distance, normal))
      return std::numeric_limits<double>::infinity();
    return static_cast<double>(distance);
  };
  result.gradient = [&shape, inverse_transform, world_transform](const Eigen::Vector3d& point) -> Eigen::Vector3d {
    btScalar distance{ 0 };
    btVector3 normal;
    if (!querySdfShapePoint(shape, inverse_transform * convertEigenToBt(point), distance, normal))
      return Eigen::Vector3d::Zero();
    normal = world_transform.getBasis() * normal;
    return convertBtToEigen(normal);
  };
  btVector3 aabb_min;
  btVector3 aabb_max;
  shape.getAabb(world_transform, aabb_min, aabb_max);
  result.aabb = Eigen::AlignedBox3d(convertBtToEigen(aabb_min), convertBtToEigen(aabb_max));
  return result;
}
}  // namespace

ImplicitSDFShape makeImplicitSdfShape(const btCollisionShape& shape, const btTransform& world_transform)
{
  if (const auto* field_shape = dynamic_cast<const TesseractSdfCollisionShape*>(&shape))
    return makeImplicitSDFShape(field_shape->getField(), convertBtToEigen(world_transform));
  return makeBulletMiniSdfShape(static_cast<const btSdfCollisionShape&>(shape), world_transform);
}

bool querySdfShapeContact(const btCollisionShape& shape,
                          const btVector3& local_point,
                          btScalar max_distance,
                          btScalar& distance,
                          btVector3& normal)
{
  if (const auto* field_shape = dynamic_cast<const TesseractSdfCollisionShape*>(&shape))
  {
    // Distance first: the normal is the expensive read when the gradient uses a wider stencil.
    if (!field_shape->queryDistance(local_point, distance) || distance > max_distance)
      return false;
    return field_shape->queryPoint(local_point, distance, normal);
  }
  // btMiniSDF reads distance and normal together; there is nothing to skip.
  return querySdfShapePoint(shape, local_point, distance, normal) && distance <= max_distance;
}

bool querySdfShapePoint(const btCollisionShape& shape,
                        const btVector3& local_point,
                        btScalar& distance,
                        btVector3& normal)
{
  if (const auto* field_shape = dynamic_cast<const TesseractSdfCollisionShape*>(&shape))
    return field_shape->queryPoint(local_point, distance, normal);
  // queryPoint is a non-const member on btSdfCollisionShape even though it does not mutate the
  // field, hence the const_cast (stock Bullet C-casts away constness in the same spot).
  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast)
  auto& mini_sdf = const_cast<btSdfCollisionShape&>(static_cast<const btSdfCollisionShape&>(shape));
  return mini_sdf.queryPoint(local_point, distance, normal);
}

}  // namespace tesseract::collision::bullet_internal
