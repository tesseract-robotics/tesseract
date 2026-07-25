/**
 * @file tesseract_sdf_sdf_algorithm.h
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
#ifndef TESSERACT_COLLISION_TESSERACT_SDF_SDF_ALGORITHM_H
#define TESSERACT_COLLISION_TESSERACT_SDF_SDF_ALGORITHM_H

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <BulletCollision/CollisionDispatch/btActivatingCollisionAlgorithm.h>
#include <BulletCollision/CollisionDispatch/btCollisionCreateFunc.h>
#include <BulletCollision/CollisionDispatch/btCollisionDispatcher.h>
#include <BulletCollision/NarrowPhaseCollision/btPersistentManifold.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract::collision::bullet_internal
{
/** @brief SDF-vs-SDF narrowphase backed by the backend-neutral implicit collision solver. */
class TesseractSdfSdfAlgorithm : public btActivatingCollisionAlgorithm
{
  bool m_ownManifold{ false };
  btPersistentManifold* m_manifoldPtr;

public:
  TesseractSdfSdfAlgorithm(btPersistentManifold* mf, const btCollisionAlgorithmConstructionInfo& ci);

  ~TesseractSdfSdfAlgorithm() override;
  TesseractSdfSdfAlgorithm(const TesseractSdfSdfAlgorithm&) = delete;
  TesseractSdfSdfAlgorithm& operator=(const TesseractSdfSdfAlgorithm&) = delete;
  TesseractSdfSdfAlgorithm(TesseractSdfSdfAlgorithm&&) = delete;
  TesseractSdfSdfAlgorithm& operator=(TesseractSdfSdfAlgorithm&&) = delete;

  void processCollision(const btCollisionObjectWrapper* body0Wrap,
                        const btCollisionObjectWrapper* body1Wrap,
                        const btDispatcherInfo& dispatchInfo,
                        btManifoldResult* resultOut) override;

  btScalar calculateTimeOfImpact(btCollisionObject* body0,
                                 btCollisionObject* body1,
                                 const btDispatcherInfo& dispatchInfo,
                                 btManifoldResult* resultOut) override;

  void getAllContactManifolds(btManifoldArray& manifoldArray) override
  {
    if (m_manifoldPtr != nullptr && m_ownManifold)
      manifoldArray.push_back(m_manifoldPtr);
  }

  struct CreateFunc : public btCollisionAlgorithmCreateFunc
  {
    btCollisionAlgorithm* CreateCollisionAlgorithm(btCollisionAlgorithmConstructionInfo& ci,
                                                   const btCollisionObjectWrapper* /*body0Wrap*/,
                                                   const btCollisionObjectWrapper* /*body1Wrap*/) override
    {
      void* mem = ci.m_dispatcher1->allocateCollisionAlgorithm(sizeof(TesseractSdfSdfAlgorithm));
      return new (mem) TesseractSdfSdfAlgorithm(ci.m_manifold, ci);  // NOLINT
    }
  };
};

}  // namespace tesseract::collision::bullet_internal

#endif  // TESSERACT_COLLISION_TESSERACT_SDF_SDF_ALGORITHM_H
