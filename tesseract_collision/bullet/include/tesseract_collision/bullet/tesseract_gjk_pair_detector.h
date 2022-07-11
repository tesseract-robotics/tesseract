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
#ifndef TESSERACT_COLLISION_TESSERACT_GJK_PAIR_DETECTOR_H
#define TESSERACT_COLLISION_TESSERACT_GJK_PAIR_DETECTOR_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <BulletCollision/NarrowPhaseCollision/btDiscreteCollisionDetectorInterface.h>
#include <BulletCollision/CollisionShapes/btCollisionMargin.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

class btConvexShape;
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <BulletCollision/NarrowPhaseCollision/btSimplexSolverInterface.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

class btConvexPenetrationDepthSolver;

#include <tesseract_collision/core/types.h>

namespace tesseract_collision::tesseract_collision_bullet
{
/**
 * @brief This is a modifed Convex to Convex collision algorithm
 *
 * This was modified to leverage the Tesseract contact request to enable and disable different parts of the algorithm.
 * The algorithm first does a quick binary check if the two objects are in collision. If in collision it runs
 * the penetration algorithm to get the nearest points and penetration depths. If not in collision it runs the an
 * algorithm to get the nearest points and distance. The Tesseract contact request allow you to now decide if you need
 * all three. Example, in the case of OMPL if you have a contact distance of zero you can get a performance increase,
 * by disabling the penetration and distance calculation because they add no value.
 *
 * Note: This will not be able to be removed.
 */
class TesseractGjkPairDetector : public btDiscreteCollisionDetectorInterface
{
  btVector3 m_cachedSeparatingAxis;
  btConvexPenetrationDepthSolver* m_penetrationDepthSolver;
  btSimplexSolverInterface* m_simplexSolver;
  const btConvexShape* m_minkowskiA;
  const btConvexShape* m_minkowskiB;
  int m_shapeTypeA;
  int m_shapeTypeB;
  btScalar m_marginA;
  btScalar m_marginB;

  bool m_ignoreMargin;
  btScalar m_cachedSeparatingDistance{ 0 };

  const ContactTestData* m_cdata;

public:
  // some debugging to fix degeneracy problems
  int m_lastUsedMethod;
  int m_curIter{ 0 };
  int m_degenerateSimplex{ 0 };
  int m_catchDegeneracies;
  int m_fixContactNormalDirection;

  TesseractGjkPairDetector(const btConvexShape* objectA,
                           const btConvexShape* objectB,
                           btSimplexSolverInterface* simplexSolver,
                           btConvexPenetrationDepthSolver* penetrationDepthSolver,
                           const ContactTestData* cdata);

  TesseractGjkPairDetector(const btConvexShape* objectA,
                           const btConvexShape* objectB,
                           int shapeTypeA,
                           int shapeTypeB,
                           btScalar marginA,
                           btScalar marginB,
                           btSimplexSolverInterface* simplexSolver,
                           btConvexPenetrationDepthSolver* penetrationDepthSolver,
                           const ContactTestData* cdata);

  void getClosestPoints(const ClosestPointInput& input,
                        Result& output,
                        class btIDebugDraw* debugDraw,
                        bool swapResults = false) override;

  void getClosestPointsNonVirtual(const ClosestPointInput& input, Result& output, class btIDebugDraw* debugDraw);

  void setMinkowskiA(const btConvexShape* minkA) { m_minkowskiA = minkA; }

  void setMinkowskiB(const btConvexShape* minkB) { m_minkowskiB = minkB; }
  void setCachedSeparatingAxis(const btVector3& separatingAxis) { m_cachedSeparatingAxis = separatingAxis; }

  const btVector3& getCachedSeparatingAxis() const { return m_cachedSeparatingAxis; }
  btScalar getCachedSeparatingDistance() const { return m_cachedSeparatingDistance; }

  void setPenetrationDepthSolver(btConvexPenetrationDepthSolver* penetrationDepthSolver)
  {
    m_penetrationDepthSolver = penetrationDepthSolver;
  }

  /// don't use setIgnoreMargin, it's for Bullet's internal use
  void setIgnoreMargin(bool ignoreMargin) { m_ignoreMargin = ignoreMargin; }
};
}  // namespace tesseract_collision::tesseract_collision_bullet
#endif  // TESSERACT_COLLISION_TESSERACT_GJK_PAIR_DETECTOR_H
