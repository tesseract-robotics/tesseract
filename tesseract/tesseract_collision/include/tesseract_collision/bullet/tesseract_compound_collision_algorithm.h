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
#ifndef TESSERACT_COLLISION_TESSERACT_COMPOUND_COLLISION_ALGORITHM_H
#define TESSERACT_COLLISION_TESSERACT_COMPOUND_COLLISION_ALGORITHM_H

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
TESSERACT_COMMON_IGNORE_WARNINGS_POP

class btDispatcher;
class btCollisionObject;
class btCollisionShape;

// LCOV_EXCL_START
namespace tesseract_collision
{
namespace tesseract_collision_bullet
{
/// btCompoundCollisionAlgorithm  supports collision between CompoundCollisionShapes and other collision shapes
class TesseractCompoundCollisionAlgorithm : public btActivatingCollisionAlgorithm  // NOLINT
{
  btNodeStack stack2;
  btManifoldArray manifoldArray;

protected:
  btAlignedObjectArray<btCollisionAlgorithm*> m_childCollisionAlgorithms;
  bool m_isSwapped;

  class btPersistentManifold* m_sharedManifold;
  bool m_ownsManifold;

  int m_compoundShapeRevision;  // to keep track of changes, so that childAlgorithm array can be updated

  void removeChildAlgorithms();

  void preallocateChildAlgorithms(const btCollisionObjectWrapper* body0Wrap, const btCollisionObjectWrapper* body1Wrap);

public:
  TesseractCompoundCollisionAlgorithm(const btCollisionAlgorithmConstructionInfo& ci,
                                      const btCollisionObjectWrapper* body0Wrap,
                                      const btCollisionObjectWrapper* body1Wrap,
                                      bool isSwapped);

  ~TesseractCompoundCollisionAlgorithm() override;

  btCollisionAlgorithm* getChildAlgorithm(int n) const { return m_childCollisionAlgorithms[n]; }

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
    int i;
    for (i = 0; i < m_childCollisionAlgorithms.size(); i++)
    {
      if (m_childCollisionAlgorithms[i])
        m_childCollisionAlgorithms[i]->getAllContactManifolds(manifoldArray);
    }
  }

  struct CreateFunc : public btCollisionAlgorithmCreateFunc
  {
    btCollisionAlgorithm* CreateCollisionAlgorithm(btCollisionAlgorithmConstructionInfo& ci,
                                                   const btCollisionObjectWrapper* body0Wrap,
                                                   const btCollisionObjectWrapper* body1Wrap) override
    {
      void* mem = ci.m_dispatcher1->allocateCollisionAlgorithm(sizeof(TesseractCompoundCollisionAlgorithm));
      return new (mem) TesseractCompoundCollisionAlgorithm(ci, body0Wrap, body1Wrap, false);
    }
  };

  struct SwappedCreateFunc : public btCollisionAlgorithmCreateFunc
  {
    btCollisionAlgorithm* CreateCollisionAlgorithm(btCollisionAlgorithmConstructionInfo& ci,
                                                   const btCollisionObjectWrapper* body0Wrap,
                                                   const btCollisionObjectWrapper* body1Wrap) override
    {
      void* mem = ci.m_dispatcher1->allocateCollisionAlgorithm(sizeof(TesseractCompoundCollisionAlgorithm));
      return new (mem) TesseractCompoundCollisionAlgorithm(ci, body0Wrap, body1Wrap, true);
    }
  };
};
}  // namespace tesseract_collision_bullet
}  // namespace tesseract_collision
// LCOV_EXCL_STOP
#endif  // TESSERACT_COLLISION_TESSERACT_COMPOUND_COLLISION_ALGORITHM_H
