/**
 * @file bullet_utils.h
 * @brief Tesseract ROS Bullet environment utility function.
 *
 * @author John Schulman
 * @author Levi Armstrong
 * @date Dec 18, 2017
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
 * @copyright Copyright (c) 2013, John Schulman
 *
 * @par License
 * Software License Agreement (BSD-2-Clause)
 * @par
 * All rights reserved.
 * @par
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * @par
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 * @par
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef TESSERACT_COLLISION_BULLET_UTILS_H
#define TESSERACT_COLLISION_BULLET_UTILS_H

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wall"
#pragma GCC diagnostic ignored "-Wint-to-pointer-cast"
#include <btBulletCollisionCommon.h>
#pragma GCC diagnostic pop

#include <tesseract_core/basic_types.h>
#include <tesseract_collision/contact_checker_common.h>
#include <geometric_shapes/mesh_operations.h>
#include <ros/console.h>

namespace tesseract
{
namespace tesseract_bullet
{
#define METERS

const float BULLET_MARGIN = 0;
const float BULLET_SUPPORT_FUNC_TOLERANCE = .01 METERS;
const float BULLET_LENGTH_TOLERANCE = .001 METERS;
const float BULLET_EPSILON = 1e-3;
const double BULLET_DEFAULT_CONTACT_DISTANCE = 0.05;
const bool BULLET_COMPOUND_USE_DYNAMIC_AABB = true;

inline btVector3 convertEigenToBt(const Eigen::Vector3d& v) { return btVector3(v[0], v[1], v[2]); }
inline Eigen::Vector3d convertBtToEigen(const btVector3& v) { return Eigen::Vector3d(v.x(), v.y(), v.z()); }
inline btQuaternion convertEigenToBt(const Eigen::Quaterniond& q) { return btQuaternion(q.x(), q.y(), q.z(), q.w()); }
inline btMatrix3x3 convertEigenToBt(const Eigen::Matrix3d& r)
{
  return btMatrix3x3(r(0, 0), r(0, 1), r(0, 2), r(1, 0), r(1, 1), r(1, 2), r(2, 0), r(2, 1), r(2, 2));
}

inline btTransform convertEigenToBt(const Eigen::Isometry3d& t)
{
  const Eigen::Matrix3d& rot = t.matrix().block<3, 3>(0, 0);
  const Eigen::Vector3d& tran = t.translation();

  return btTransform(convertEigenToBt(rot), convertEigenToBt(tran));
}

/**
 * @brief This is a tesseract bullet collsion object.
 *
 * It is a wrapper around bullet's collision object which
 * contains specific information related to tesseract
 */
class CollisionObjectWrapper : public btCollisionObject
{
public:
  CollisionObjectWrapper(const std::string& name,
                         const int& type_id,
                         const std::vector<shapes::ShapeConstPtr>& shapes,
                         const VectorIsometry3d& shape_poses,
                         const CollisionObjectTypeVector& collision_object_types);

  short int m_collisionFilterGroup;
  short int m_collisionFilterMask;
  bool m_enabled;

  /** @brief Get the collision object name */
  const std::string& getName() const { return m_name; }
  /** @brief Get a user defined type */
  const int& getTypeID() const { return m_type_id; }
  /** \brief Check if two CollisionObjectWrapper objects point to the same source object */
  bool sameObject(const CollisionObjectWrapper& other) const
  {
    return m_name == other.m_name && m_type_id == other.m_type_id && m_shapes.size() == other.m_shapes.size() &&
           m_shape_poses.size() == other.m_shape_poses.size() &&
           std::equal(m_shapes.begin(), m_shapes.end(), other.m_shapes.begin()) &&
           std::equal(m_shape_poses.begin(),
                      m_shape_poses.end(),
                      other.m_shape_poses.begin(),
                      [](const Eigen::Isometry3d& t1, const Eigen::Isometry3d& t2) { return t1.isApprox(t2); });
  }

  /**
   * @brief This clones the collision objects but not the collision shape wich is const.
   * @return Shared Pointer to the cloned collision object
   */
  std::shared_ptr<CollisionObjectWrapper> clone()
  {
    std::shared_ptr<CollisionObjectWrapper> clone_cow(
        new CollisionObjectWrapper(m_name, m_type_id, m_shapes, m_shape_poses, m_collision_object_types, m_data));
    clone_cow->setCollisionShape(getCollisionShape());
    clone_cow->setWorldTransform(getWorldTransform());
    clone_cow->m_collisionFilterGroup = m_collisionFilterGroup;
    clone_cow->m_collisionFilterMask = m_collisionFilterMask;
    clone_cow->m_enabled = m_enabled;
    return clone_cow;
  }

  template <class T>
  void manage(T* t)
  {  // manage memory of this object
    m_data.push_back(std::shared_ptr<T>(t));
  }
  template <class T>
  void manage(std::shared_ptr<T> t)
  {
    m_data.push_back(t);
  }

protected:
  /** @brief This is a special constructor used by the clone method */
  CollisionObjectWrapper(const std::string& name,
                         const int& type_id,
                         const std::vector<shapes::ShapeConstPtr>& shapes,
                         const VectorIsometry3d& shape_poses,
                         const CollisionObjectTypeVector& collision_object_types,
                         const std::vector<std::shared_ptr<void>>& data);

  std::string m_name;                                 /**< @brief The name of the collision object */
  int m_type_id;                                      /**< @brief A user defined type id */
  std::vector<shapes::ShapeConstPtr> m_shapes;        /**< @brief The shapes that define the collison object */
  VectorIsometry3d m_shape_poses;                     /**< @brief The shpaes poses information */
  CollisionObjectTypeVector m_collision_object_types; /**< @brief The shape collision object type to be used */

  std::vector<std::shared_ptr<void>>
      m_data; /**< @brief This manages the collision shape pointer so they get destroyed */
};

typedef CollisionObjectWrapper COW;
typedef std::shared_ptr<CollisionObjectWrapper> COWPtr;
typedef std::shared_ptr<const CollisionObjectWrapper> COWConstPtr;
typedef std::map<std::string, COWPtr> Link2Cow;
typedef std::map<std::string, COWConstPtr> Link2ConstCow;

/** @brief This is a casted collision shape used for checking if an object is collision free between two transforms */
struct CastHullShape : public btConvexShape
{
public:
  btConvexShape* m_shape;
  btTransform m_t01;

  CastHullShape(btConvexShape* shape, const btTransform& t01) : m_shape(shape), m_t01(t01)
  {
    m_shapeType = CUSTOM_CONVEX_SHAPE_TYPE;
  }

  void updateCastTransform(const btTransform& t01) { m_t01 = t01; }
  btVector3 localGetSupportingVertex(const btVector3& vec) const override
  {
    btVector3 sv0 = m_shape->localGetSupportingVertex(vec);
    btVector3 sv1 = m_t01 * m_shape->localGetSupportingVertex(vec * m_t01.getBasis());
    return (vec.dot(sv0) > vec.dot(sv1)) ? sv0 : sv1;
  }

  // notice that the vectors should be unit length
  void batchedUnitVectorGetSupportingVertexWithoutMargin(const btVector3* /*vectors*/,
                                                         btVector3* /*supportVerticesOut*/,
                                                         int /*numVectors*/) const override
  {
    throw std::runtime_error("not implemented");
  }

  /// getAabb's default implementation is brute force, expected derived classes to implement a fast dedicated version
  void getAabb(const btTransform& t_w0, btVector3& aabbMin, btVector3& aabbMax) const override
  {
    m_shape->getAabb(t_w0, aabbMin, aabbMax);
    btVector3 min1, max1;
    m_shape->getAabb(t_w0 * m_t01, min1, max1);
    aabbMin.setMin(min1);
    aabbMax.setMax(max1);
  }

  virtual void getAabbSlow(const btTransform& /*t*/, btVector3& /*aabbMin*/, btVector3& /*aabbMax*/) const override
  {
    throw std::runtime_error("shouldn't happen");
  }

  virtual void setLocalScaling(const btVector3& /*scaling*/) override {}
  virtual const btVector3& getLocalScaling() const override
  {
    static btVector3 out(1, 1, 1);
    return out;
  }

  virtual void setMargin(btScalar /*margin*/) override {}
  virtual btScalar getMargin() const override { return 0; }
  virtual int getNumPreferredPenetrationDirections() const override { return 0; }
  virtual void getPreferredPenetrationDirection(int /*index*/, btVector3& /*penetrationVector*/) const override
  {
    throw std::runtime_error("not implemented");
  }

  virtual void calculateLocalInertia(btScalar, btVector3&) const { throw std::runtime_error("not implemented"); }
  virtual const char* getName() const { return "CastHull"; }
  virtual btVector3 localGetSupportingVertexWithoutMargin(const btVector3& v) const
  {
    return localGetSupportingVertex(v);
  }
};

inline void
GetAverageSupport(const btConvexShape* shape, const btVector3& localNormal, float& outsupport, btVector3& outpt)
{
  btVector3 ptSum(0, 0, 0);
  float ptCount = 0;
  float maxSupport = -1000;

  const btPolyhedralConvexShape* pshape = dynamic_cast<const btPolyhedralConvexShape*>(shape);
  if (pshape)
  {
    int nPts = pshape->getNumVertices();

    for (int i = 0; i < nPts; ++i)
    {
      btVector3 pt;
      pshape->getVertex(i, pt);

      float sup = pt.dot(localNormal);
      if (sup > maxSupport + BULLET_EPSILON)
      {
        ptCount = 1;
        ptSum = pt;
        maxSupport = sup;
      }
      else if (sup < maxSupport - BULLET_EPSILON)
      {
      }
      else
      {
        ptCount += 1;
        ptSum += pt;
      }
    }
    outsupport = maxSupport;
    outpt = ptSum / ptCount;
  }
  else
  {
    outpt = shape->localGetSupportingVertexWithoutMargin(localNormal);
    outsupport = localNormal.dot(outpt);
  }
}

/**
 * @brief This is used to check if a collision check is required between the provided two collision objects
 * @param cow1 The first collision object
 * @param cow2 The second collision object
 * @param acm  The contact allowed function pointer
 * @param verbose Indicate if verbose information should be printed to the terminal
 * @return True if the two collision objects should be checked for collision, otherwise false
 */
inline bool needsCollisionCheck(const COW& cow1, const COW& cow2, const IsContactAllowedFn acm, bool verbose = false)
{
  return cow1.m_enabled && cow2.m_enabled && (cow2.m_collisionFilterGroup & cow1.m_collisionFilterMask) &&
         (cow1.m_collisionFilterGroup & cow2.m_collisionFilterMask) &&
         !isContactAllowed(cow1.getName(), cow2.getName(), acm, verbose);
}

inline btScalar addDiscreteSingleResult(btManifoldPoint& cp,
                                        const btCollisionObjectWrapper* colObj0Wrap,
                                        const btCollisionObjectWrapper* colObj1Wrap,
                                        ContactDistanceData& collisions)
{
  const CollisionObjectWrapper* cd0 = static_cast<const CollisionObjectWrapper*>(colObj0Wrap->getCollisionObject());
  const CollisionObjectWrapper* cd1 = static_cast<const CollisionObjectWrapper*>(colObj1Wrap->getCollisionObject());

  ObjectPairKey pc = getObjectPairKey(cd0->getName(), cd1->getName());

  const auto& it = collisions.res->find(pc);
  bool found = (it != collisions.res->end());

  //    size_t l = 0;
  //    if (found)
  //    {
  //      l = it->second.size();
  //      if (m_collisions.req->type == DistanceRequestType::LIMITED && l >= m_collisions.req->max_contacts_per_body)
  //          return 0;

  //    }

  ContactResult contact;
  contact.link_names[0] = cd0->getName();
  contact.link_names[1] = cd1->getName();
  contact.nearest_points[0] = convertBtToEigen(cp.m_positionWorldOnA);
  contact.nearest_points[1] = convertBtToEigen(cp.m_positionWorldOnB);
  contact.type_id[0] = cd0->getTypeID();
  contact.type_id[1] = cd1->getTypeID();
  contact.distance = cp.m_distance1;
  contact.normal = convertBtToEigen(-1 * cp.m_normalWorldOnB);

  if (!processResult(collisions, contact, pc, found))
  {
    return 0;
  }

  return 1;
}

inline btScalar addCastSingleResult(btManifoldPoint& cp,
                                    const btCollisionObjectWrapper* colObj0Wrap,
                                    int index0,
                                    const btCollisionObjectWrapper* colObj1Wrap,
                                    int index1,
                                    ContactDistanceData& collisions,
                                    bool castShapeIsFirst)
{
  const CollisionObjectWrapper* cd0 = static_cast<const CollisionObjectWrapper*>(colObj0Wrap->getCollisionObject());
  const CollisionObjectWrapper* cd1 = static_cast<const CollisionObjectWrapper*>(colObj1Wrap->getCollisionObject());

  const std::pair<std::string, std::string>& pc = cd0->getName() < cd1->getName() ?
                                                      std::make_pair(cd0->getName(), cd1->getName()) :
                                                      std::make_pair(cd1->getName(), cd0->getName());

  ContactResultMap::iterator it = collisions.res->find(pc);
  bool found = it != collisions.res->end();

  //    size_t l = 0;
  //    if (found)
  //    {
  //      l = it->second.size();
  //      if (m_collisions.req->type == DistanceRequestType::LIMITED && l >= m_collisions.req->max_contacts_per_body)
  //          return 0;
  //    }

  ContactResult contact;
  contact.link_names[0] = cd0->getName();
  contact.link_names[1] = cd1->getName();
  contact.nearest_points[0] = convertBtToEigen(cp.m_positionWorldOnA);
  contact.nearest_points[1] = convertBtToEigen(cp.m_positionWorldOnB);
  contact.type_id[0] = cd0->getTypeID();
  contact.type_id[1] = cd1->getTypeID();
  contact.distance = cp.m_distance1;
  contact.normal = convertBtToEigen(-1 * cp.m_normalWorldOnB);

  ContactResult* col = processResult(collisions, contact, pc, found);
  if (!col)
  {
    return 0;
  }

  btVector3 normalWorldFromCast = -(castShapeIsFirst ? 1 : -1) * cp.m_normalWorldOnB;
  const btCollisionObjectWrapper* firstColObjWrap = (castShapeIsFirst ? colObj0Wrap : colObj1Wrap);
  int shapeIndex = (castShapeIsFirst ? index0 : index1);

  if (castShapeIsFirst)
  {
    std::swap(col->nearest_points[0], col->nearest_points[1]);
    std::swap(col->link_names[0], col->link_names[1]);
    std::swap(col->type_id[0], col->type_id[1]);
    col->normal *= -1;
  }

  btTransform tfWorld0, tfWorld1;
  const CastHullShape* shape;
  if (btBroadphaseProxy::isConvex(firstColObjWrap->getCollisionObject()->getCollisionShape()->getShapeType()))
  {
    shape = static_cast<const CastHullShape*>(firstColObjWrap->getCollisionObject()->getCollisionShape());
    tfWorld0 = firstColObjWrap->getWorldTransform();
    tfWorld1 = firstColObjWrap->getWorldTransform() * shape->m_t01;
  }
  else if (btBroadphaseProxy::isCompound(firstColObjWrap->getCollisionObject()->getCollisionShape()->getShapeType()))
  {
    const btCompoundShape* compound =
        static_cast<const btCompoundShape*>(firstColObjWrap->getCollisionObject()->getCollisionShape());
    shape = static_cast<const CastHullShape*>(compound->getChildShape(shapeIndex));
    tfWorld0 = firstColObjWrap->getWorldTransform() * compound->getChildTransform(shapeIndex);
    tfWorld1 = tfWorld0 * shape->m_t01;
  }
  else
  {
    throw std::runtime_error("I can only continuous collision check convex shapes and compound shapes made of convex "
                             "shapes");
  }
  assert(!!shape);

  btVector3 normalLocal0 = normalWorldFromCast * tfWorld0.getBasis();
  btVector3 normalLocal1 = normalWorldFromCast * tfWorld1.getBasis();

  btVector3 ptLocal0;
  float localsup0;
  GetAverageSupport(shape->m_shape, normalLocal0, localsup0, ptLocal0);
  btVector3 ptWorld0 = tfWorld0 * ptLocal0;
  btVector3 ptLocal1;
  float localsup1;
  GetAverageSupport(shape->m_shape, normalLocal1, localsup1, ptLocal1);
  btVector3 ptWorld1 = tfWorld1 * ptLocal1;

  float sup0 = normalWorldFromCast.dot(ptWorld0);
  float sup1 = normalWorldFromCast.dot(ptWorld1);

  // TODO: this section is potentially problematic. think hard about the math
  if (sup0 - sup1 > BULLET_SUPPORT_FUNC_TOLERANCE)
  {
    col->cc_time = 0;
    col->cc_type = ContinouseCollisionType::CCType_Time0;
  }
  else if (sup1 - sup0 > BULLET_SUPPORT_FUNC_TOLERANCE)
  {
    col->cc_time = 1;
    col->cc_type = ContinouseCollisionType::CCType_Time1;
  }
  else
  {
    const btVector3& ptOnCast = castShapeIsFirst ? cp.m_positionWorldOnA : cp.m_positionWorldOnB;
    float l0c = (ptOnCast - ptWorld0).length(), l1c = (ptOnCast - ptWorld1).length();

    col->cc_nearest_points[0] = col->nearest_points[1];
    col->nearest_points[1] = convertBtToEigen(ptWorld0);

    col->cc_nearest_points[1] = convertBtToEigen(ptWorld1);
    col->cc_type = ContinouseCollisionType::CCType_Between;

    if (l0c + l1c < BULLET_LENGTH_TOLERANCE)
    {
      col->cc_time = .5;
    }
    else
    {
      col->cc_time = l0c / (l0c + l1c);
    }
  }

  return 1;
}

/** @brief This is copied directly out of BulletWorld */
struct TesseractBridgedManifoldResult : public btManifoldResult
{
  btCollisionWorld::ContactResultCallback& m_resultCallback;

  TesseractBridgedManifoldResult(const btCollisionObjectWrapper* obj0Wrap,
                                 const btCollisionObjectWrapper* obj1Wrap,
                                 btCollisionWorld::ContactResultCallback& resultCallback)
    : btManifoldResult(obj0Wrap, obj1Wrap), m_resultCallback(resultCallback)
  {
  }

  virtual void addContactPoint(const btVector3& normalOnBInWorld, const btVector3& pointInWorld, btScalar depth)
  {
    bool isSwapped = m_manifoldPtr->getBody0() != m_body0Wrap->getCollisionObject();
    btVector3 pointA = pointInWorld + normalOnBInWorld * depth;
    btVector3 localA;
    btVector3 localB;
    if (isSwapped)
    {
      localA = m_body1Wrap->getCollisionObject()->getWorldTransform().invXform(pointA);
      localB = m_body0Wrap->getCollisionObject()->getWorldTransform().invXform(pointInWorld);
    }
    else
    {
      localA = m_body0Wrap->getCollisionObject()->getWorldTransform().invXform(pointA);
      localB = m_body1Wrap->getCollisionObject()->getWorldTransform().invXform(pointInWorld);
    }

    btManifoldPoint newPt(localA, localB, normalOnBInWorld, depth);
    newPt.m_positionWorldOnA = pointA;
    newPt.m_positionWorldOnB = pointInWorld;

    // BP mod, store contact triangles.
    if (isSwapped)
    {
      newPt.m_partId0 = m_partId1;
      newPt.m_partId1 = m_partId0;
      newPt.m_index0 = m_index1;
      newPt.m_index1 = m_index0;
    }
    else
    {
      newPt.m_partId0 = m_partId0;
      newPt.m_partId1 = m_partId1;
      newPt.m_index0 = m_index0;
      newPt.m_index1 = m_index1;
    }

    // experimental feature info, for per-triangle material etc.
    const btCollisionObjectWrapper* obj0Wrap = isSwapped ? m_body1Wrap : m_body0Wrap;
    const btCollisionObjectWrapper* obj1Wrap = isSwapped ? m_body0Wrap : m_body1Wrap;
    m_resultCallback.addSingleResult(
        newPt, obj0Wrap, newPt.m_partId0, newPt.m_index0, obj1Wrap, newPt.m_partId1, newPt.m_index1);
  }
};

struct TesseractBroadphaseBridgedManifoldResult : public btManifoldResult
{
  ContactDistanceData& collisions_;

  TesseractBroadphaseBridgedManifoldResult(const btCollisionObjectWrapper* obj0Wrap,
                                           const btCollisionObjectWrapper* obj1Wrap,
                                           ContactDistanceData& collisions)
    : btManifoldResult(obj0Wrap, obj1Wrap), collisions_(collisions)
  {
  }

  virtual void addContactPoint(const btVector3& normalOnBInWorld, const btVector3& pointInWorld, btScalar depth)
  {
    if (depth > collisions_.req->contact_distance)
      return;

    bool isSwapped = m_manifoldPtr->getBody0() != m_body0Wrap->getCollisionObject();
    btVector3 pointA = pointInWorld + normalOnBInWorld * depth;
    btVector3 localA;
    btVector3 localB;
    if (isSwapped)
    {
      localA = m_body1Wrap->getCollisionObject()->getWorldTransform().invXform(pointA);
      localB = m_body0Wrap->getCollisionObject()->getWorldTransform().invXform(pointInWorld);
    }
    else
    {
      localA = m_body0Wrap->getCollisionObject()->getWorldTransform().invXform(pointA);
      localB = m_body1Wrap->getCollisionObject()->getWorldTransform().invXform(pointInWorld);
    }

    btManifoldPoint newPt(localA, localB, normalOnBInWorld, depth);
    newPt.m_positionWorldOnA = pointA;
    newPt.m_positionWorldOnB = pointInWorld;

    // BP mod, store contact triangles.
    if (isSwapped)
    {
      newPt.m_partId0 = m_partId1;
      newPt.m_partId1 = m_partId0;
      newPt.m_index0 = m_index1;
      newPt.m_index1 = m_index0;
    }
    else
    {
      newPt.m_partId0 = m_partId0;
      newPt.m_partId1 = m_partId1;
      newPt.m_index0 = m_index0;
      newPt.m_index1 = m_index1;
    }

    // experimental feature info, for per-triangle material etc.
    const btCollisionObjectWrapper* obj0Wrap = isSwapped ? m_body1Wrap : m_body0Wrap;
    const btCollisionObjectWrapper* obj1Wrap = isSwapped ? m_body0Wrap : m_body1Wrap;
    //    addDiscreteSingleResult(newPt,obj0Wrap,newPt.m_partId0,newPt.m_index0,obj1Wrap,newPt.m_partId1,newPt.m_index1);
    addDiscreteSingleResult(newPt, obj0Wrap, obj1Wrap, collisions_);
  }
};

/**
 * @brief This is copied directly out of BulletWorld
 *
 * This is currently not used but will remain because it is needed
 * to check a collision object not in the broadphase to the broadphase
 * which may eventually be exposed.
*/
struct TesseractSingleContactCallback : public btBroadphaseAabbCallback
{
  btCollisionObject* m_collisionObject;    /**< @brief The bullet collision object */
  btCollisionDispatcher* m_dispatcher;     /**< @brief The bullet collision dispatcher used for getting object to object
                                              collison algorithm */
  const btDispatcherInfo& m_dispatch_info; /**< @brief The bullet collision dispatcher configuration information */
  btCollisionWorld::ContactResultCallback& m_resultCallback;

  TesseractSingleContactCallback(btCollisionObject* collisionObject,
                                 btCollisionDispatcher* dispatcher,
                                 const btDispatcherInfo& dispatch_info,
                                 btCollisionWorld::ContactResultCallback& resultCallback)
    : m_collisionObject(collisionObject)
    , m_dispatcher(dispatcher)
    , m_dispatch_info(dispatch_info)
    , m_resultCallback(resultCallback)
  {
  }

  virtual bool process(const btBroadphaseProxy* proxy)
  {
    btCollisionObject* collisionObject = (btCollisionObject*)proxy->m_clientObject;
    if (collisionObject == m_collisionObject)
      return true;

    if (m_resultCallback.needsCollision(collisionObject->getBroadphaseHandle()))
    {
      btCollisionObjectWrapper ob0(
          0, m_collisionObject->getCollisionShape(), m_collisionObject, m_collisionObject->getWorldTransform(), -1, -1);
      btCollisionObjectWrapper ob1(
          0, collisionObject->getCollisionShape(), collisionObject, collisionObject->getWorldTransform(), -1, -1);

      btCollisionAlgorithm* algorithm = m_dispatcher->findAlgorithm(&ob0, &ob1, 0, BT_CLOSEST_POINT_ALGORITHMS);
      if (algorithm)
      {
        TesseractBridgedManifoldResult contactPointResult(&ob0, &ob1, m_resultCallback);
        contactPointResult.m_closestPointDistanceThreshold = m_resultCallback.m_closestDistanceThreshold;

        // discrete collision detection query
        algorithm->processCollision(&ob0, &ob1, m_dispatch_info, &contactPointResult);

        algorithm->~btCollisionAlgorithm();
        m_dispatcher->freeCollisionAlgorithm(algorithm);
      }
    }
    return true;
  }
};

/**
 * @brief A callback function that is called as part of the broadphase collision checking.
 *
 * If the AABB of two collision objects are overlapping the processOverlap method is called
 * and they are checked for collision/distance and the results are stored in collision_.
 */
class TesseractCollisionPairCallback : public btOverlapCallback
{
  const btDispatcherInfo& dispatch_info_;
  btCollisionDispatcher* dispatcher_;
  ContactDistanceData& collisions_;

public:
  TesseractCollisionPairCallback(const btDispatcherInfo& dispatchInfo,
                                 btCollisionDispatcher* dispatcher,
                                 ContactDistanceData& collisions)
    : dispatch_info_(dispatchInfo), dispatcher_(dispatcher), collisions_(collisions)
  {
  }

  virtual ~TesseractCollisionPairCallback() {}
  virtual bool processOverlap(btBroadphasePair& pair)
  {
    const CollisionObjectWrapper* cow1 = static_cast<const CollisionObjectWrapper*>(pair.m_pProxy0->m_clientObject);
    const CollisionObjectWrapper* cow2 = static_cast<const CollisionObjectWrapper*>(pair.m_pProxy1->m_clientObject);

    bool needs_collision = needsCollisionCheck(*cow1, *cow2, collisions_.req->isContactAllowed, false);

    if (needs_collision)
    {
      btCollisionObjectWrapper obj0Wrap(0, cow1->getCollisionShape(), cow1, cow1->getWorldTransform(), -1, -1);
      btCollisionObjectWrapper obj1Wrap(0, cow2->getCollisionShape(), cow2, cow2->getWorldTransform(), -1, -1);

      // dispatcher will keep algorithms persistent in the collision pair
      if (!pair.m_algorithm)
      {
        pair.m_algorithm = dispatcher_->findAlgorithm(&obj0Wrap, &obj1Wrap, 0, BT_CLOSEST_POINT_ALGORITHMS);
      }

      if (pair.m_algorithm)
      {
        TesseractBroadphaseBridgedManifoldResult contactPointResult(&obj0Wrap, &obj1Wrap, collisions_);
        contactPointResult.m_closestPointDistanceThreshold = collisions_.req->contact_distance;

        // discrete collision detection query
        pair.m_algorithm->processCollision(&obj0Wrap, &obj1Wrap, dispatch_info_, &contactPointResult);
      }
    }
    return false;
  }
};

btCollisionShape* createShapePrimitive(const shapes::ShapeConstPtr& geom,
                                       const CollisionObjectType& collision_object_type,
                                       CollisionObjectWrapper* cow);

/**
 * @brief updateCollisionObjectsWithRequest
 * @param req
 * @param link2cow
 */
inline void updateCollisionObjectWithRequest(const ContactRequest& req, COW& cow)
{
  // For descrete checks we can check static to kinematic and kinematic to
  // kinematic
  cow.m_collisionFilterGroup = btBroadphaseProxy::KinematicFilter;
  if (!req.link_names.empty())
  {
    bool check = (std::find_if(req.link_names.begin(), req.link_names.end(), [&cow](const std::string& link) {
                    return link == cow.getName();
                  }) == req.link_names.end());
    if (check)
    {
      cow.m_collisionFilterGroup = btBroadphaseProxy::StaticFilter;
    }
  }

  if (cow.m_collisionFilterGroup == btBroadphaseProxy::StaticFilter)
  {
    cow.m_collisionFilterMask = btBroadphaseProxy::KinematicFilter;
  }
  else
  {
    cow.m_collisionFilterMask = btBroadphaseProxy::StaticFilter | btBroadphaseProxy::KinematicFilter;
  }

  if (cow.getBroadphaseHandle())
  {
    cow.getBroadphaseHandle()->m_collisionFilterGroup = cow.m_collisionFilterGroup;
    cow.getBroadphaseHandle()->m_collisionFilterMask = cow.m_collisionFilterMask;
  }
  cow.setContactProcessingThreshold(req.contact_distance);
}

inline COWPtr createCollisionObject(const std::string& name,
                                    const int& type_id,
                                    const std::vector<shapes::ShapeConstPtr>& shapes,
                                    const VectorIsometry3d& shape_poses,
                                    const CollisionObjectTypeVector& collision_object_types,
                                    bool enabled = true)
{
  // dont add object that does not have geometry
  if (shapes.empty() || shape_poses.empty() || (shapes.size() != shape_poses.size()))
  {
    ROS_DEBUG("ignoring link %s", name.c_str());
    return nullptr;
  }

  COWPtr new_cow(new COW(name, type_id, shapes, shape_poses, collision_object_types));

  new_cow->m_enabled = enabled;
  new_cow->setContactProcessingThreshold(BULLET_DEFAULT_CONTACT_DISTANCE);

  ROS_DEBUG("Created collision object for link %s", new_cow->getName().c_str());
  return new_cow;
}

struct DiscreteCollisionCollector : public btCollisionWorld::ContactResultCallback
{
  ContactDistanceData& collisions_;
  const COWPtr cow_;
  double contact_distance_;
  bool verbose_;

  DiscreteCollisionCollector(ContactDistanceData& collisions,
                             const COWPtr cow,
                             double contact_distance,
                             bool verbose = false)
    : collisions_(collisions), cow_(cow), contact_distance_(contact_distance), verbose_(verbose)
  {
    m_closestDistanceThreshold = contact_distance;
    m_collisionFilterGroup = cow->m_collisionFilterGroup;
    m_collisionFilterMask = cow->m_collisionFilterMask;
  }

  virtual btScalar addSingleResult(btManifoldPoint& cp,
                                   const btCollisionObjectWrapper* colObj0Wrap,
                                   int /*partId0*/,
                                   int /*index0*/,
                                   const btCollisionObjectWrapper* colObj1Wrap,
                                   int /*partId1*/,
                                   int /*index1*/)
  {
    if (cp.m_distance1 > contact_distance_)
      return 0;

    return addDiscreteSingleResult(cp, colObj0Wrap, colObj1Wrap, collisions_);
  }

  bool needsCollision(btBroadphaseProxy* proxy0) const
  {
    return !collisions_.done && needsCollisionCheck(*cow_,
                                                    *(static_cast<CollisionObjectWrapper*>(proxy0->m_clientObject)),
                                                    collisions_.req->isContactAllowed,
                                                    verbose_);
  }
};

struct CastCollisionCollector : public btCollisionWorld::ContactResultCallback
{
  ContactDistanceData& collisions_;
  const COWPtr cow_;
  double contact_distance_;
  bool verbose_;

  CastCollisionCollector(ContactDistanceData& collisions,
                         const COWPtr cow,
                         double contact_distance,
                         bool verbose = false)
    : collisions_(collisions), cow_(cow), contact_distance_(contact_distance), verbose_(verbose)
  {
    m_closestDistanceThreshold = contact_distance;
    m_collisionFilterGroup = cow->m_collisionFilterGroup;
    m_collisionFilterMask = cow->m_collisionFilterMask;
  }

  virtual btScalar addSingleResult(btManifoldPoint& cp,
                                   const btCollisionObjectWrapper* colObj0Wrap,
                                   int /*partId0*/,
                                   int index0,
                                   const btCollisionObjectWrapper* colObj1Wrap,
                                   int /*partId1*/,
                                   int index1)
  {
    if (cp.m_distance1 > contact_distance_)
      return 0;

    return addCastSingleResult(
        cp, colObj0Wrap, index0, colObj1Wrap, index1, collisions_, (colObj0Wrap->getCollisionObject() == cow_.get()));
  }

  bool needsCollision(btBroadphaseProxy* proxy0) const
  {
    return !collisions_.done && needsCollisionCheck(*cow_,
                                                    *(static_cast<CollisionObjectWrapper*>(proxy0->m_clientObject)),
                                                    collisions_.req->isContactAllowed,
                                                    verbose_);
  }
};

inline COWPtr makeCastCollisionObject(const COWPtr& cow)
{
  COWPtr new_cow = cow->clone();

  btTransform tf;
  tf.setIdentity();

  if (btBroadphaseProxy::isConvex(new_cow->getCollisionShape()->getShapeType()))
  {
    btConvexShape* convex = static_cast<btConvexShape*>(new_cow->getCollisionShape());
    assert(convex != NULL);
    assert(convex->getShapeType() !=
           CUSTOM_CONVEX_SHAPE_TYPE);  // This checks if the collision object is already a cast collision object

    CastHullShape* shape = new CastHullShape(convex, tf);
    assert(shape != NULL);

    new_cow->manage(shape);
    new_cow->setCollisionShape(shape);
  }
  else if (btBroadphaseProxy::isCompound(new_cow->getCollisionShape()->getShapeType()))
  {
    btCompoundShape* compound = static_cast<btCompoundShape*>(new_cow->getCollisionShape());
    btCompoundShape* new_compound =
        new btCompoundShape(/*dynamicAABBtree=*/BULLET_COMPOUND_USE_DYNAMIC_AABB, compound->getNumChildShapes());

    for (int i = 0; i < compound->getNumChildShapes(); ++i)
    {
      btConvexShape* convex = static_cast<btConvexShape*>(compound->getChildShape(i));
      assert(convex != NULL);
      assert(convex->getShapeType() !=
             CUSTOM_CONVEX_SHAPE_TYPE);  // This checks if the collision object is already a cast collision object

      btTransform geomTrans = compound->getChildTransform(i);

      btCollisionShape* subshape = new CastHullShape(convex, tf);
      assert(subshape != NULL);

      if (subshape != NULL)
      {
        new_cow->manage(subshape);
        subshape->setMargin(BULLET_MARGIN);
        new_compound->addChildShape(geomTrans, subshape);
      }
    }

    new_compound->setMargin(BULLET_MARGIN);  // margin: compound. seems to
                                             // have no effect when positive
                                             // but has an effect when
                                             // negative
    new_cow->manage(new_compound);
    new_cow->setCollisionShape(new_compound);
    new_cow->setWorldTransform(cow->getWorldTransform());
  }
  else
  {
    ROS_ERROR("I can only continuous collision check convex shapes and "
              "compound shapes made of convex shapes");
  }

  return new_cow;
}

}
}
#endif  // TESSERACT_COLLISION_BULLET_UTILS_H
