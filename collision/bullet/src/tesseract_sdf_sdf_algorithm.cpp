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
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/bullet/bullet_utils.h>
#include <tesseract/collision/bullet/tesseract_sdf_collision_shape.h>
#include <tesseract/collision/bullet/tesseract_sdf_sdf_algorithm.h>
#include <tesseract/collision/implicit_sdf_collision_solver.h>

namespace tesseract::collision::bullet_internal
{
namespace
{
ImplicitSDFShape makeSDFShape(const btCollisionObjectWrapper& wrap)
{
  const auto& shape = static_cast<const TesseractSdfCollisionShape&>(*wrap.getCollisionShape());
  return makeImplicitSDFShape(shape.getField(), convertBtToEigen(wrap.getWorldTransform()));
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
  const ImplicitSDFShape implicit_sdf0 = makeSDFShape(*body0Wrap);
  const ImplicitSDFShape implicit_sdf1 = makeSDFShape(*body1Wrap);
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
