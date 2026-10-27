/**
 * @file tesseract_sdf_collision_shape.h
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
#ifndef TESSERACT_COLLISION_TESSERACT_SDF_COLLISION_SHAPE_H
#define TESSERACT_COLLISION_TESSERACT_SDF_COLLISION_SHAPE_H

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <BulletCollision/CollisionShapes/btConcaveShape.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/implicit_sdf_collision_solver.h>
#include <tesseract/geometry/impl/signed_distance_field.h>

namespace tesseract::collision::bullet_internal
{
/**
 * @brief A Bullet SDF shape that reads a @ref tesseract::geometry::SignedDistanceField in place.
 *
 * Bullet's shape for every field (grid-backed or function-backed): it queries the field itself,
 * with no copy into Bullet's @c btMiniSDF. It reports @c SDF_SHAPE_PROXYTYPE, so the existing SDF
 * pair algorithms handle it; they reach the field through @ref makeImplicitSdfShape and
 * @ref querySdfShapePoint, which accept this shape and @c btSdfCollisionShape alike. The field's own
 * @c scale is applied here; Bullet local scaling is not supported.
 */
class TesseractSdfCollisionShape : public btConcaveShape
{
public:
  BT_DECLARE_ALIGNED_ALLOCATOR();

  explicit TesseractSdfCollisionShape(tesseract::geometry::SignedDistanceField::ConstPtr field);

  /** @brief The field's scaled domain under @p t, grown by the margin. */
  void getAabb(const btTransform& t, btVector3& aabbMin, btVector3& aabbMax) const override;

  /** @brief Only unit scaling is accepted (the field carries its own scale). */
  void setLocalScaling(const btVector3& scaling) override;
  const btVector3& getLocalScaling() const override;

  /** @brief Static geometry: zero inertia. */
  void calculateLocalInertia(btScalar mass, btVector3& inertia) const override;

  const char* getName() const override;

  /** @brief No triangles: this shape is only collided through the SDF pair algorithms. */
  void processAllTriangles(btTriangleCallback* callback,
                           const btVector3& aabbMin,
                           const btVector3& aabbMax) const override;

  /**
   * @brief Scaled signed distance and outward normal at a point in the shape's frame.
   * @return false outside the field's domain
   */
  bool queryPoint(const btVector3& local_point, btScalar& distance, btVector3& normal) const;

  /**
   * @brief Scaled signed distance only, at a point in the shape's frame. Cheaper than
   * @ref queryPoint when the normal may not be needed.
   * @return false outside the field's domain
   */
  bool queryDistance(const btVector3& local_point, btScalar& distance) const;

  const tesseract::geometry::SignedDistanceField& getField() const;

private:
  tesseract::geometry::SignedDistanceField::ConstPtr field_;
  btVector3 local_scaling_{ 1, 1, 1 };
};

/**
 * @brief The implicit solver's view of a Bullet SDF shape (@c SDF_SHAPE_PROXYTYPE) placed at
 * @p world_transform: a @ref TesseractSdfCollisionShape or a @c btSdfCollisionShape.
 * @details The shape must outlive the result.
 */
ImplicitSDFShape makeImplicitSdfShape(const btCollisionShape& shape, const btTransform& world_transform);

/**
 * @brief Distance and normal at a point in an SDF shape's frame, but only if the distance is within
 * @p max_distance (a contact threshold). For a @ref TesseractSdfCollisionShape the distance is read
 * first and the normal only for points that pass; a btSdfCollisionShape reads both at once.
 * @return true if the shape has a value there and it is within @p max_distance
 */
bool querySdfShapeContact(const btCollisionShape& shape,
                          const btVector3& local_point,
                          btScalar max_distance,
                          btScalar& distance,
                          btVector3& normal);

/**
 * @brief Distance and normal at a point in an SDF shape's frame, for either Bullet SDF shape type.
 * @return false where the shape has no value
 */
bool querySdfShapePoint(const btCollisionShape& shape,
                        const btVector3& local_point,
                        btScalar& distance,
                        btVector3& normal);

}  // namespace tesseract::collision::bullet_internal

#endif  // TESSERACT_COLLISION_TESSERACT_SDF_COLLISION_SHAPE_H
