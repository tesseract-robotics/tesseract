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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <BulletCollision/CollisionShapes/btCollisionShape.h>
#include <BulletCollision/CollisionDispatch/btManifoldResult.h>
#include <btBulletCollisionCommon.h>
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_collision/core/types.h>
#include <tesseract_collision/core/common.h>

namespace tesseract_collision::tesseract_collision_bullet
{
#define METERS

const btScalar BULLET_MARGIN = btScalar(0.0);
const btScalar BULLET_SUPPORT_FUNC_TOLERANCE = btScalar(0.01) METERS;
const btScalar BULLET_LENGTH_TOLERANCE = btScalar(0.001) METERS;
const btScalar BULLET_EPSILON = btScalar(1e-3);
const btScalar BULLET_DEFAULT_CONTACT_DISTANCE = btScalar(0.05);
const bool BULLET_COMPOUND_USE_DYNAMIC_AABB = true;

btVector3 convertEigenToBt(const Eigen::Vector3d& v);

Eigen::Vector3d convertBtToEigen(const btVector3& v);

btQuaternion convertEigenToBt(const Eigen::Quaterniond& q);

btMatrix3x3 convertEigenToBt(const Eigen::Matrix3d& r);

Eigen::Matrix3d convertBtToEigen(const btMatrix3x3& r);

btTransform convertEigenToBt(const Eigen::Isometry3d& t);

Eigen::Isometry3d convertBtToEigen(const btTransform& t);

/**
 * @brief This is a tesseract bullet collsion object.
 *
 * It is a wrapper around bullet's collision object which
 * contains specific information related to tesseract
 */
class CollisionObjectWrapper : public btCollisionObject
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<CollisionObjectWrapper>;
  using ConstPtr = std::shared_ptr<const CollisionObjectWrapper>;

  CollisionObjectWrapper() = default;
  CollisionObjectWrapper(std::string name,
                         const int& type_id,
                         CollisionShapesConst shapes,
                         tesseract_common::VectorIsometry3d shape_poses);

  short int m_collisionFilterGroup{ btBroadphaseProxy::KinematicFilter };
  short int m_collisionFilterMask{ btBroadphaseProxy::StaticFilter | btBroadphaseProxy::KinematicFilter };
  bool m_enabled{ true };

  /** @brief Get the collision object name */
  const std::string& getName() const;
  /** @brief Get a user defined type */
  const int& getTypeID() const;
  /** \brief Check if two CollisionObjectWrapper objects point to the same source object */
  bool sameObject(const CollisionObjectWrapper& other) const;

  const CollisionShapesConst& getCollisionGeometries() const;

  const tesseract_common::VectorIsometry3d& getCollisionGeometriesTransforms() const;

  /**
   * @brief Get the collision objects axis aligned bounding box
   * @param aabb_min The minimum point
   * @param aabb_max The maximum point
   */
  void getAABB(btVector3& aabb_min, btVector3& aabb_max) const;

  /**
   * @brief This clones the collision objects but not the collision shape wich is const.
   * @return Shared Pointer to the cloned collision object
   */
  std::shared_ptr<CollisionObjectWrapper> clone();

  void manage(const std::shared_ptr<btCollisionShape>& t);

  void manageReserve(std::size_t s);

protected:
  /** @brief The name of the collision object */
  std::string m_name;
  /** @brief A user defined type id */
  int m_type_id{ -1 };
  /* @brief The shapes that define the collision object */
  CollisionShapesConst m_shapes{};
  /**< @brief The shapes poses information */
  tesseract_common::VectorIsometry3d m_shape_poses{};
  /** @brief This manages the collision shape pointer so they get destroyed */
  std::vector<std::shared_ptr<btCollisionShape>> m_data{};
};

using COW = CollisionObjectWrapper;
using Link2Cow = std::map<std::string, COW::Ptr>;
using Link2ConstCow = std::map<std::string, COW::ConstPtr>;

/** @brief This is a casted collision shape used for checking if an object is collision free between two transforms */
struct CastHullShape : public btConvexShape
{
public:
  btConvexShape* m_shape;
  btTransform m_t01;

  CastHullShape(btConvexShape* shape, const btTransform& t01);

  void updateCastTransform(const btTransform& t01);
  btVector3 localGetSupportingVertex(const btVector3& vec) const override;

  /// getAabb's default implementation is brute force, expected derived classes to implement a fast dedicated version
  void getAabb(const btTransform& t_w0, btVector3& aabbMin, btVector3& aabbMax) const override;

  const char* getName() const override;
  btVector3 localGetSupportingVertexWithoutMargin(const btVector3& v) const override;

  // LCOV_EXCL_START
  // notice that the vectors should be unit length
  void batchedUnitVectorGetSupportingVertexWithoutMargin(const btVector3* vectors,
                                                         btVector3* supportVerticesOut,
                                                         int numVectors) const override;

  void getAabbSlow(const btTransform& t, btVector3& aabbMin, btVector3& aabbMax) const override;

  void setLocalScaling(const btVector3& scaling) override;

  const btVector3& getLocalScaling() const override;

  void setMargin(btScalar margin) override;

  btScalar getMargin() const override;

  int getNumPreferredPenetrationDirections() const override;

  void getPreferredPenetrationDirection(int index, btVector3& penetrationVector) const override;

  void calculateLocalInertia(btScalar, btVector3&) const override;
  // LCOV_EXCL_STOP
};

void GetAverageSupport(const btConvexShape* shape,
                       const btVector3& localNormal,
                       btScalar& outsupport,
                       btVector3& outpt);

/**
 * @brief This transversus the parent tree to find the base object and return its world transform
 * This should be the links transform and it should only need to transverse twice at max.
 * @param cow Bullet collision object wrapper.
 * @return Transform of link in world coordinates
 */
btTransform getLinkTransformFromCOW(const btCollisionObjectWrapper* cow);

/**
 * @brief This is used to check if a collision check is required between the provided two collision objects
 * @param cow1 The first collision object
 * @param cow2 The second collision object
 * @param acm  The contact allowed function pointer
 * @param verbose Indicate if verbose information should be printed to the terminal
 * @return True if the two collision objects should be checked for collision, otherwise false
 */
bool needsCollisionCheck(const COW& cow1, const COW& cow2, const IsContactAllowedFn& acm, bool verbose = false);

btScalar addDiscreteSingleResult(btManifoldPoint& cp,
                                 const btCollisionObjectWrapper* colObj0Wrap,
                                 const btCollisionObjectWrapper* colObj1Wrap,
                                 ContactTestData& collisions);

/**
 * @brief Calculate the continuous contact data for casted collision shape
 * @param col Contact results
 * @param cow Bullet Collision Object Wrapper
 * @param pt_world Casted contact point in world coordinates
 * @param normal_world Casted normal to move shape out of collision in world coordinates
 * @param link_tf_inv The links world transform inverse
 * @param link_index The link index in teh ContactResults the shape is associated with
 */
void calculateContinuousData(ContactResult* col,
                             const btCollisionObjectWrapper* cow,
                             const btVector3& pt_world,
                             const btVector3& normal_world,
                             const btTransform& link_tf_inv,
                             size_t link_index);

btScalar addCastSingleResult(btManifoldPoint& cp,
                             const btCollisionObjectWrapper* colObj0Wrap,
                             int index0,
                             const btCollisionObjectWrapper* colObj1Wrap,
                             int index1,
                             ContactTestData& collisions);

/** @brief This is copied directly out of BulletWorld */
struct TesseractBridgedManifoldResult : public btManifoldResult
{
  btCollisionWorld::ContactResultCallback& m_resultCallback;

  TesseractBridgedManifoldResult(const btCollisionObjectWrapper* obj0Wrap,
                                 const btCollisionObjectWrapper* obj1Wrap,
                                 btCollisionWorld::ContactResultCallback& resultCallback);

  void addContactPoint(const btVector3& normalOnBInWorld, const btVector3& pointInWorld, btScalar depth) override;
};

/** @brief The BroadphaseContactResultCallback is used to report contact points */
struct BroadphaseContactResultCallback
{
  ContactTestData& collisions_;
  double contact_distance_;
  bool verbose_;

  BroadphaseContactResultCallback(ContactTestData& collisions, double contact_distance, bool verbose = false);

  virtual ~BroadphaseContactResultCallback() = default;
  BroadphaseContactResultCallback(const BroadphaseContactResultCallback&) = default;
  BroadphaseContactResultCallback& operator=(const BroadphaseContactResultCallback&) = delete;
  BroadphaseContactResultCallback(BroadphaseContactResultCallback&&) = default;
  BroadphaseContactResultCallback& operator=(BroadphaseContactResultCallback&&) = delete;

  virtual bool needsCollision(const CollisionObjectWrapper* cow0, const CollisionObjectWrapper* cow1) const;

  virtual btScalar addSingleResult(btManifoldPoint& cp,
                                   const btCollisionObjectWrapper* colObj0Wrap,
                                   int partId0,
                                   int index0,
                                   const btCollisionObjectWrapper* colObj1Wrap,
                                   int partId1,
                                   int index1) = 0;
};

struct DiscreteBroadphaseContactResultCallback : public BroadphaseContactResultCallback
{
  DiscreteBroadphaseContactResultCallback(ContactTestData& collisions, double contact_distance, bool verbose = false);

  btScalar addSingleResult(btManifoldPoint& cp,
                           const btCollisionObjectWrapper* colObj0Wrap,
                           int partId0,
                           int index0,
                           const btCollisionObjectWrapper* colObj1Wrap,
                           int partId1,
                           int index1) override;
};

struct CastBroadphaseContactResultCallback : public BroadphaseContactResultCallback
{
  CastBroadphaseContactResultCallback(ContactTestData& collisions, double contact_distance, bool verbose = false);

  btScalar addSingleResult(btManifoldPoint& cp,
                           const btCollisionObjectWrapper* colObj0Wrap,
                           int partId0,
                           int index0,
                           const btCollisionObjectWrapper* colObj1Wrap,
                           int partId1,
                           int index1) override;
};

struct TesseractBroadphaseBridgedManifoldResult : public btManifoldResult
{
  BroadphaseContactResultCallback& result_callback_;

  TesseractBroadphaseBridgedManifoldResult(const btCollisionObjectWrapper* obj0Wrap,
                                           const btCollisionObjectWrapper* obj1Wrap,
                                           BroadphaseContactResultCallback& result_callback);

  void addContactPoint(const btVector3& normalOnBInWorld, const btVector3& pointInWorld, btScalar depth) override;
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
  BroadphaseContactResultCallback& results_callback_;

public:
  TesseractCollisionPairCallback(const btDispatcherInfo& dispatchInfo,
                                 btCollisionDispatcher* dispatcher,
                                 BroadphaseContactResultCallback& results_callback);

  ~TesseractCollisionPairCallback() override = default;
  TesseractCollisionPairCallback(const TesseractCollisionPairCallback&) = default;
  TesseractCollisionPairCallback& operator=(const TesseractCollisionPairCallback&) = delete;
  TesseractCollisionPairCallback(TesseractCollisionPairCallback&&) = default;
  TesseractCollisionPairCallback& operator=(TesseractCollisionPairCallback&&) = delete;

  bool processOverlap(btBroadphasePair& pair) override;
};

/** @brief This class is used to filter broadphase */
class TesseractOverlapFilterCallback : public btOverlapFilterCallback
{
public:
  TesseractOverlapFilterCallback(bool verbose = false);

  bool needBroadphaseCollision(btBroadphaseProxy* proxy0, btBroadphaseProxy* proxy1) const override;

private:
  bool verbose_{ false };
};

/**
 * @brief Create a bullet collision shape from tesseract collision shape
 * @param geom Tesseract collision shape
 * @param cow The collision object wrapper the collision shape is associated with
 * @param shape_index The collision shapes index within the collision shape wrapper. This can be accessed from the
 * bullet collision shape by calling getUserIndex function.
 * @return Bullet collision shape.
 */
std::shared_ptr<btCollisionShape> createShapePrimitive(const CollisionShapeConstPtr& geom,
                                                       CollisionObjectWrapper* cow,
                                                       int shape_index);

/**
 * @brief Update a collision objects filters
 * @param active A list of active collision objects
 * @param cow The collision object to update.
 */
void updateCollisionObjectFilters(const std::vector<std::string>& active, const COW::Ptr& cow);

COW::Ptr createCollisionObject(const std::string& name,
                               const int& type_id,
                               const CollisionShapesConst& shapes,
                               const tesseract_common::VectorIsometry3d& shape_poses,
                               bool enabled = true);

struct DiscreteCollisionCollector : public btCollisionWorld::ContactResultCallback
{
  ContactTestData& collisions_;
  const COW::Ptr cow_;
  double contact_distance_;
  bool verbose_;

  DiscreteCollisionCollector(ContactTestData& collisions,
                             COW::Ptr cow,
                             btScalar contact_distance,
                             bool verbose = false);

  btScalar addSingleResult(btManifoldPoint& cp,
                           const btCollisionObjectWrapper* colObj0Wrap,
                           int partId0,
                           int index0,
                           const btCollisionObjectWrapper* colObj1Wrap,
                           int partId1,
                           int index1) override;

  bool needsCollision(btBroadphaseProxy* proxy0) const override;
};

struct CastCollisionCollector : public btCollisionWorld::ContactResultCallback
{
  ContactTestData& collisions_;
  const COW::Ptr cow_;
  double contact_distance_;
  bool verbose_;

  CastCollisionCollector(ContactTestData& collisions, COW::Ptr cow, double contact_distance, bool verbose = false);

  btScalar addSingleResult(btManifoldPoint& cp,
                           const btCollisionObjectWrapper* colObj0Wrap,
                           int partId0,
                           int index0,
                           const btCollisionObjectWrapper* colObj1Wrap,
                           int partId1,
                           int index1) override;

  bool needsCollision(btBroadphaseProxy* proxy0) const override;
};

COW::Ptr makeCastCollisionObject(const COW::Ptr& cow);

/**
 * @brief Update the Broadphase AABB for the input collision object
 * @param cow The collision objects
 * @param broadphase The bullet broadphase interface
 * @param dispatcher The bullet collision dispatcher
 */
void updateBroadphaseAABB(const COW::Ptr& cow,
                          const std::unique_ptr<btBroadphaseInterface>& broadphase,
                          const std::unique_ptr<btCollisionDispatcher>& dispatcher);

/**
 * @brief Remove the collision object from broadphase
 * @param cow The collision objects
 * @param broadphase The bullet broadphase interface
 * @param dispatcher The bullet collision dispatcher
 */
void removeCollisionObjectFromBroadphase(const COW::Ptr& cow,
                                         const std::unique_ptr<btBroadphaseInterface>& broadphase,
                                         const std::unique_ptr<btCollisionDispatcher>& dispatcher);

/**
 * @brief Add the collision object to broadphase
 * @param cow The collision objects
 * @param broadphase The bullet broadphase interface
 * @param dispatcher The bullet collision dispatcher
 */
void addCollisionObjectToBroadphase(const COW::Ptr& cow,
                                    const std::unique_ptr<btBroadphaseInterface>& broadphase,
                                    const std::unique_ptr<btCollisionDispatcher>& dispatcher);

/**
 * @brief Update a collision objects filters for broadphase
 * @param active A list of active collision objects
 * @param cow The collision object to update.
 * @param broadphase The collision object to update.
 * @param dispatcher The collision object to update.
 */
void updateCollisionObjectFilters(const std::vector<std::string>& active,
                                  const COW::Ptr& cow,
                                  const std::unique_ptr<btBroadphaseInterface>& broadphase,
                                  const std::unique_ptr<btCollisionDispatcher>& dispatcher);

/**
 * @brief Refresh the broadphase data structure
 * @details When change certain properties of a collision object the broadphase is not aware so this function can be
 * called to trigger an update. For example, when changing active links this changes internal flags which may require
 * moving a collision object from the static BVH to the dynamic BVH so this function must be called.
 * @param cow The collision object to update.
 * @param broadphase The broadphase to update.
 * @param dispatcher The dispatcher.
 */
void refreshBroadphaseProxy(const COW::Ptr& cow,
                            const std::unique_ptr<btBroadphaseInterface>& broadphase,
                            const std::unique_ptr<btCollisionDispatcher>& dispatcher);
}  // namespace tesseract_collision::tesseract_collision_bullet
#endif  // TESSERACT_COLLISION_BULLET_UTILS_H
