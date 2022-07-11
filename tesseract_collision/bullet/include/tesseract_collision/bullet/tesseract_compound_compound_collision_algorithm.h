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
#ifndef TESSERACT_COLLISION_TESSERACT_COMPOUND_COMPOUND_COLLISION_ALGORITHM_H
#define TESSERACT_COLLISION_TESSERACT_COMPOUND_COMPOUND_COLLISION_ALGORITHM_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <BulletCollision/BroadphaseCollision/btDispatcher.h>
#include <BulletCollision/BroadphaseCollision/btBroadphaseInterface.h>
#include <BulletCollision/CollisionDispatch/btActivatingCollisionAlgorithm.h>
#include <BulletCollision/NarrowPhaseCollision/btPersistentManifold.h>
#include <BulletCollision/BroadphaseCollision/btBroadphaseProxy.h>
#include <BulletCollision/CollisionDispatch/btCollisionCreateFunc.h>
#include <LinearMath/btAlignedObjectArray.h>
#include <BulletCollision/BroadphaseCollision/btDbvt.h>
#include <BulletCollision/CollisionDispatch/btHashedSimplePairCache.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_collision/bullet/tesseract_compound_collision_algorithm.h>

class btCollisionObject;
class btCollisionShape;

// LCOV_EXCL_START
namespace tesseract_collision::tesseract_collision_bullet
{
/**
 * @brief Supports collision between two btCompoundCollisionShape shapes
 *
 * The original implementation would check all collision objects before exiting the bvh of the compound shape. The
 * original code had a callback, but it only passed in the collision shape and no the collision object which is where
 * the user data is located. This was modifed to check if collision is done for the contact test type FIRST during the
 * internal broadphase of the compound shapes and exit early.
 *
 * Note: This could be removed in the future but the callback need to be modifed to accept the collision object along
 * with the collision shape. I don't believe this will be an issue since all of the other callback in Bullet accept
 * both.
 */
class TesseractCompoundCompoundCollisionAlgorithm : public TesseractCompoundCollisionAlgorithm  // NOLINT
{
  class btHashedSimplePairCache* m_childCollisionAlgorithmCache;
  btSimplePairArray m_removePairs;

  int m_compoundShapeRevision0;  // to keep track of changes, so that childAlgorithm array can be updated
  int m_compoundShapeRevision1;

  void removeChildAlgorithms();

public:
  TesseractCompoundCompoundCollisionAlgorithm(const btCollisionAlgorithmConstructionInfo& ci,
                                              const btCollisionObjectWrapper* body0Wrap,
                                              const btCollisionObjectWrapper* body1Wrap,
                                              bool isSwapped);

  ~TesseractCompoundCompoundCollisionAlgorithm() override;
  TesseractCompoundCompoundCollisionAlgorithm(const TesseractCompoundCompoundCollisionAlgorithm&) = default;
  TesseractCompoundCompoundCollisionAlgorithm& operator=(const TesseractCompoundCompoundCollisionAlgorithm&) = default;
  TesseractCompoundCompoundCollisionAlgorithm(TesseractCompoundCompoundCollisionAlgorithm&&) = default;
  TesseractCompoundCompoundCollisionAlgorithm& operator=(TesseractCompoundCompoundCollisionAlgorithm&&) = default;

  void processCollision(const btCollisionObjectWrapper* body0Wrap,
                        const btCollisionObjectWrapper* body1Wrap,
                        const btDispatcherInfo& dispatchInfo,
                        btManifoldResult* resultOut) override;

  btScalar calculateTimeOfImpact(btCollisionObject* body0,
                                 btCollisionObject* body1,
                                 const btDispatcherInfo& dispatchInfo,
                                 btManifoldResult* resultOut) override;

  void getAllContactManifolds(btManifoldArray& manifoldArray) override;

  struct CreateFunc : public btCollisionAlgorithmCreateFunc
  {
    btCollisionAlgorithm* CreateCollisionAlgorithm(btCollisionAlgorithmConstructionInfo& ci,
                                                   const btCollisionObjectWrapper* body0Wrap,
                                                   const btCollisionObjectWrapper* body1Wrap) override
    {
      void* mem = ci.m_dispatcher1->allocateCollisionAlgorithm(sizeof(TesseractCompoundCompoundCollisionAlgorithm));
      return new (mem) TesseractCompoundCompoundCollisionAlgorithm(ci, body0Wrap, body1Wrap, false);
    }
  };

  struct SwappedCreateFunc : public btCollisionAlgorithmCreateFunc
  {
    btCollisionAlgorithm* CreateCollisionAlgorithm(btCollisionAlgorithmConstructionInfo& ci,
                                                   const btCollisionObjectWrapper* body0Wrap,
                                                   const btCollisionObjectWrapper* body1Wrap) override
    {
      void* mem = ci.m_dispatcher1->allocateCollisionAlgorithm(sizeof(TesseractCompoundCompoundCollisionAlgorithm));
      return new (mem) TesseractCompoundCompoundCollisionAlgorithm(ci, body0Wrap, body1Wrap, true);
    }
  };
};
}  // namespace tesseract_collision::tesseract_collision_bullet
// LCOV_EXCL_STOP
#endif  // TESSERACT_COLLISION_TESSERACT_COMPOUND_COMPOUND_COLLISION_ALGORITHM_H
