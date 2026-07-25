/**
 * @file tesseract_sdf_sdf_algorithm.cpp
 * @brief Bullet narrowphase algorithm for signed-distance-field pairs.
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
#include <BulletCollision/CollisionDispatch/btCollisionObject.h>
#include <BulletCollision/CollisionDispatch/btCollisionObjectWrapper.h>
#include <BulletCollision/CollisionDispatch/btManifoldResult.h>
#include <BulletCollision/CollisionShapes/btSdfCollisionShape.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/bullet/bullet_utils.h>
#include <tesseract/collision/bullet/tesseract_sdf_sdf_algorithm.h>
#include <tesseract/collision/implicit_sdf_collision_solver.h>

#include <limits>

namespace tesseract::collision::bullet_internal
{
namespace
{
ImplicitSDFShape makeBulletSDFShape(btSdfCollisionShape& shape, const btTransform& world_transform)
{
  const btTransform inverse_transform = world_transform.inverse();
  ImplicitSDFShape result;
  result.distance = [&shape, inverse_transform](const Eigen::Vector3d& point) {
    btScalar distance{ 0 };
    btVector3 normal;
    if (!shape.queryPoint(inverse_transform * convertEigenToBt(point), distance, normal))
      return std::numeric_limits<double>::infinity();
    return static_cast<double>(distance);
  };
  result.gradient = [&shape, inverse_transform, world_transform](const Eigen::Vector3d& point) -> Eigen::Vector3d {
    btScalar distance{ 0 };
    btVector3 normal;
    if (!shape.queryPoint(inverse_transform * convertEigenToBt(point), distance, normal))
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

TesseractSdfSdfAlgorithm::TesseractSdfSdfAlgorithm(btPersistentManifold* mf,
                                                   const btCollisionAlgorithmConstructionInfo& ci)
  : btActivatingCollisionAlgorithm(ci), m_manifoldPtr(mf)
{
}

TesseractSdfSdfAlgorithm::~TesseractSdfSdfAlgorithm()
{
  if (m_ownManifold && m_manifoldPtr != nullptr)
    m_dispatcher->releaseManifold(m_manifoldPtr);
}

void TesseractSdfSdfAlgorithm::processCollision(const btCollisionObjectWrapper* body0Wrap,
                                                const btCollisionObjectWrapper* body1Wrap,
                                                const btDispatcherInfo& /*dispatchInfo*/,
                                                btManifoldResult* resultOut)
{
  // queryPoint is non-const even though it does not mutate the field.
  // NOLINTBEGIN(cppcoreguidelines-pro-type-const-cast)
  auto* sdf_shape0 =
      const_cast<btSdfCollisionShape*>(static_cast<const btSdfCollisionShape*>(body0Wrap->getCollisionShape()));
  auto* sdf_shape1 =
      const_cast<btSdfCollisionShape*>(static_cast<const btSdfCollisionShape*>(body1Wrap->getCollisionShape()));
  // NOLINTEND(cppcoreguidelines-pro-type-const-cast)

  const ImplicitSDFShape implicit_sdf0 = makeBulletSDFShape(*sdf_shape0, body0Wrap->getWorldTransform());
  const ImplicitSDFShape implicit_sdf1 = makeBulletSDFShape(*sdf_shape1, body1Wrap->getWorldTransform());
  ImplicitSDFCollisionConfig config;
  config.contact_margin = static_cast<double>(resultOut->m_closestPointDistanceThreshold);
  const std::vector<ImplicitSDFContact> contacts = collideImplicitSDF(implicit_sdf0, implicit_sdf1, config);

  if (m_manifoldPtr == nullptr)
  {
    m_manifoldPtr = m_dispatcher->getNewManifold(body0Wrap->getCollisionObject(), body1Wrap->getCollisionObject());
    m_ownManifold = true;
  }
  resultOut->setPersistentManifold(m_manifoldPtr);

  for (const auto& contact : contacts)
  {
    // Bullet expects the normal on body B directed toward body A. The shared solver returns the
    // opposite direction: from shape 0 toward shape 1.
    resultOut->addContactPoint(convertEigenToBt(Eigen::Vector3d(-contact.normal)),
                               convertEigenToBt(contact.nearest_points[1]),
                               static_cast<btScalar>(contact.distance));
  }
  resultOut->refreshContactPoints();
}

btScalar TesseractSdfSdfAlgorithm::calculateTimeOfImpact(btCollisionObject* /*body0*/,
                                                         btCollisionObject* /*body1*/,
                                                         const btDispatcherInfo& /*dispatchInfo*/,
                                                         btManifoldResult* /*resultOut*/)
{
  return btScalar(1.);
}

}  // namespace tesseract::collision::bullet_internal
