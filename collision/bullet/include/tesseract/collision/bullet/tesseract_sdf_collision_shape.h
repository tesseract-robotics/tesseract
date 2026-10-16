/**
 * @file tesseract_sdf_collision_shape.h
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
#ifndef TESSERACT_COLLISION_TESSERACT_SDF_COLLISION_SHAPE_H
#define TESSERACT_COLLISION_TESSERACT_SDF_COLLISION_SHAPE_H

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <BulletCollision/CollisionShapes/btConcaveShape.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/geometry/impl/signed_distance_field.h>

namespace tesseract::collision::bullet_internal
{
/**
 * @brief A Bullet shape for a Tesseract signed distance field, which it refers to rather than copies.
 *
 * It carries Bullet's SDF shape type, so the dispatcher routes its pairs to the Tesseract SDF algorithms, and those
 * query the field through the backend-neutral implicit solver. Bullet itself only reads its bounding box. The
 * field's scale is applied by the solver, so the shape's own local scaling stays at one.
 */
class TesseractSdfCollisionShape : public btConcaveShape
{
public:
  explicit TesseractSdfCollisionShape(tesseract::geometry::SignedDistanceField::ConstPtr field);

  /** @brief The field this shape refers to */
  const tesseract::geometry::SignedDistanceField& getField() const;

  void getAabb(const btTransform& t, btVector3& aabbMin, btVector3& aabbMax) const override;

  /** @brief Ignored: the field's own scale is applied when it is queried */
  void setLocalScaling(const btVector3& scaling) override;
  const btVector3& getLocalScaling() const override;

  void calculateLocalInertia(btScalar mass, btVector3& inertia) const override;
  const char* getName() const override;

  /** @brief No-op: a field has no triangles */
  void processAllTriangles(btTriangleCallback* callback,
                           const btVector3& aabbMin,
                           const btVector3& aabbMax) const override;

private:
  tesseract::geometry::SignedDistanceField::ConstPtr field_;
  btVector3 local_aabb_min_;
  btVector3 local_aabb_max_;
  btVector3 local_scaling_{ 1, 1, 1 };
};
}  // namespace tesseract::collision::bullet_internal

#endif  // TESSERACT_COLLISION_TESSERACT_SDF_COLLISION_SHAPE_H
