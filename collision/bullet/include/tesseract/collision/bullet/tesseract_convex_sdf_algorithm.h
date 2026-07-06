/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If
you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not
required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original
software.
3. This notice may not be removed or altered from any source distribution.
*/
#ifndef TESSERACT_COLLISION_TESSERACT_CONVEX_SDF_ALGORITHM_H
#define TESSERACT_COLLISION_TESSERACT_CONVEX_SDF_ALGORITHM_H

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <BulletCollision/CollisionDispatch/btActivatingCollisionAlgorithm.h>
#include <BulletCollision/CollisionDispatch/btCollisionCreateFunc.h>
#include <BulletCollision/CollisionDispatch/btCollisionDispatcher.h>
#include <BulletCollision/NarrowPhaseCollision/btPersistentManifold.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

// LCOV_EXCL_START
namespace tesseract::collision::bullet_internal
{
/**
 * @brief Convex vs btSdfCollisionShape narrowphase that honors the contact distance threshold
 *
 * Stock Bullet handles convex-vs-SDF inside btConvexConcaveCollisionAlgorithm by sampling the
 * field at the convex shape's vertices (or a sphere's center) and only emitting a contact when
 * the sampled distance is within the shape's surface (dist <= radius). It ignores
 * btManifoldResult::m_closestPointDistanceThreshold, so positive-distance (separated) contacts
 * inside the collision margin band are never reported. Tesseract's margin-based consumers
 * (TrajOpt collision costs, contact-check gates) require those contacts, so this algorithm
 * replicates Bullet's SDF vertex query but widens the acceptance band by the pair's contact
 * distance threshold. It also adds capsule support (absent in stock Bullet) by sampling the
 * field along the capsule's axis segment with a radius offset.
 */
class TesseractConvexSdfAlgorithm : public btActivatingCollisionAlgorithm
{
  bool m_isSwapped;
  bool m_ownManifold{ false };
  btPersistentManifold* m_manifoldPtr;
  /** @brief Ensures the unsupported-shape warning is only emitted once for this pair */
  bool m_warned_unsupported{ false };

public:
  TesseractConvexSdfAlgorithm(btPersistentManifold* mf,
                              const btCollisionAlgorithmConstructionInfo& ci,
                              const btCollisionObjectWrapper* body0Wrap,
                              const btCollisionObjectWrapper* body1Wrap,
                              bool isSwapped);

  ~TesseractConvexSdfAlgorithm() override;
  TesseractConvexSdfAlgorithm(const TesseractConvexSdfAlgorithm&) = delete;
  TesseractConvexSdfAlgorithm& operator=(const TesseractConvexSdfAlgorithm&) = delete;
  TesseractConvexSdfAlgorithm(TesseractConvexSdfAlgorithm&&) = delete;
  TesseractConvexSdfAlgorithm& operator=(TesseractConvexSdfAlgorithm&&) = delete;

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
                                                   const btCollisionObjectWrapper* body0Wrap,
                                                   const btCollisionObjectWrapper* body1Wrap) override
    {
      void* mem = ci.m_dispatcher1->allocateCollisionAlgorithm(sizeof(TesseractConvexSdfAlgorithm));
      return new (mem) TesseractConvexSdfAlgorithm(ci.m_manifold, ci, body0Wrap, body1Wrap, false);  // NOLINT
    }
  };

  struct SwappedCreateFunc : public btCollisionAlgorithmCreateFunc
  {
    btCollisionAlgorithm* CreateCollisionAlgorithm(btCollisionAlgorithmConstructionInfo& ci,
                                                   const btCollisionObjectWrapper* body0Wrap,
                                                   const btCollisionObjectWrapper* body1Wrap) override
    {
      void* mem = ci.m_dispatcher1->allocateCollisionAlgorithm(sizeof(TesseractConvexSdfAlgorithm));
      return new (mem) TesseractConvexSdfAlgorithm(ci.m_manifold, ci, body0Wrap, body1Wrap, true);  // NOLINT
    }
  };
};

}  // namespace tesseract::collision::bullet_internal
// LCOV_EXCL_STOP
#endif  // TESSERACT_COLLISION_TESSERACT_CONVEX_SDF_ALGORITHM_H
