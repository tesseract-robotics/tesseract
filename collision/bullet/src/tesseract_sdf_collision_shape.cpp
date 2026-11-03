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
#include <cmath>
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
/** @brief Every SDF_SHAPE_PROXYTYPE shape Tesseract creates is this one (createShapePrimitive). */
const TesseractSdfCollisionShape& fieldShape(const btCollisionShape& shape)
{
  return static_cast<const TesseractSdfCollisionShape&>(shape);
}
}  // namespace

ImplicitSDFShape makeImplicitSdfShape(const btCollisionShape& shape, const btTransform& world_transform)
{
  return makeImplicitSDFShape(fieldShape(shape).getField(), convertBtToEigen(world_transform));
}

bool querySdfShapeContact(const btCollisionShape& shape,
                          const btVector3& local_point,
                          btScalar max_distance,
                          btScalar& distance,
                          btVector3& normal)
{
  // Distance first: the normal is the expensive read when the gradient uses a wider stencil.
  const TesseractSdfCollisionShape& field_shape = fieldShape(shape);
  if (!field_shape.queryDistance(local_point, distance) || distance > max_distance)
    return false;
  return field_shape.queryPoint(local_point, distance, normal);
}

bool querySdfShapePoint(const btCollisionShape& shape,
                        const btVector3& local_point,
                        btScalar& distance,
                        btVector3& normal)
{
  return fieldShape(shape).queryPoint(local_point, distance, normal);
}

}  // namespace tesseract::collision::bullet_internal
