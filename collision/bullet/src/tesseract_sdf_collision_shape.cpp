/**
 * @file tesseract_sdf_collision_shape.cpp
 * @brief Bullet collision shape that refers to a Tesseract signed distance field.
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
#include <LinearMath/btAabbUtil2.h>
#include <utility>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/bullet/bullet_utils.h>
#include <tesseract/collision/bullet/tesseract_sdf_collision_shape.h>

namespace tesseract::collision::bullet_internal
{
TesseractSdfCollisionShape::TesseractSdfCollisionShape(tesseract::geometry::SignedDistanceField::ConstPtr field)
  : field_(std::move(field))
  , local_aabb_min_(convertEigenToBt(Eigen::Vector3d(field_->getDomain().min().cwiseProduct(field_->getScale()))))
  , local_aabb_max_(convertEigenToBt(Eigen::Vector3d(field_->getDomain().max().cwiseProduct(field_->getScale()))))
{
  m_shapeType = SDF_SHAPE_PROXYTYPE;
}

const tesseract::geometry::SignedDistanceField& TesseractSdfCollisionShape::getField() const { return *field_; }

void TesseractSdfCollisionShape::getAabb(const btTransform& t, btVector3& aabbMin, btVector3& aabbMax) const
{
  btTransformAabb(local_aabb_min_, local_aabb_max_, getMargin(), t, aabbMin, aabbMax);
}

void TesseractSdfCollisionShape::setLocalScaling(const btVector3& /*scaling*/) {}

const btVector3& TesseractSdfCollisionShape::getLocalScaling() const { return local_scaling_; }

void TesseractSdfCollisionShape::calculateLocalInertia(btScalar /*mass*/, btVector3& inertia) const
{
  inertia.setValue(0, 0, 0);
}

const char* TesseractSdfCollisionShape::getName() const { return "TesseractSDF"; }

void TesseractSdfCollisionShape::processAllTriangles(btTriangleCallback* /*callback*/,
                                                     const btVector3& /*aabbMin*/,
                                                     const btVector3& /*aabbMax*/) const
{
}
}  // namespace tesseract::collision::bullet_internal
