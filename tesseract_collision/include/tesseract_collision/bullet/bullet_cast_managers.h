/**
 * @file bullet_cast_managers.h
 * @brief Tesseract ROS Bullet cast(continuous) collision managers.
 *
 * @author Levi Armstrong
 * @date Dec 18, 2017
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
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

#include <tesseract_collision/bullet/bullet_utils.h>
#include <tesseract_core/continuous_contact_manager_base.h>

#ifndef TESSERACT_COLLISION_BULLET_CAST_MANAGERS_H
#define TESSERACT_COLLISION_BULLET_CAST_MANAGERS_H

namespace tesseract
{
namespace tesseract_bullet
{
/** @brief A simple implementaiton of a tesseract manager which does not use BHV */
class BulletCastSimpleManager : public ContinuousContactManagerBase
{
public:
  BulletCastSimpleManager();

  ContinuousContactManagerBasePtr clone() const override;

  bool addCollisionObject(const std::string& name,
                          const int& mask_id,
                          const std::vector<shapes::ShapeConstPtr>& shapes,
                          const VectorIsometry3d& shape_poses,
                          const CollisionObjectTypeVector& collision_object_types,
                          bool enabled = true) override;

  bool hasCollisionObject(const std::string& name) const override;

  bool removeCollisionObject(const std::string& name) override;

  bool enableCollisionObject(const std::string& name) override;

  bool disableCollisionObject(const std::string& name) override;

  void setCollisionObjectsTransform(const std::string& name, const Eigen::Isometry3d& pose) override;

  void setCollisionObjectsTransform(const std::vector<std::string>& names, const VectorIsometry3d& poses) override;

  void setCollisionObjectsTransform(const TransformMap& transforms) override;

  void setCollisionObjectsTransform(const std::string& name,
                                    const Eigen::Isometry3d& pose1,
                                    const Eigen::Isometry3d& pose2) override;

  void setCollisionObjectsTransform(const std::vector<std::string>& names,
                                    const VectorIsometry3d& pose1,
                                    const VectorIsometry3d& pose2) override;

  void setCollisionObjectsTransform(const TransformMap& pose1, const TransformMap& pose2) override;

  void setContactRequest(const ContactRequest& req) override;

  const ContactRequest& getContactRequest() const override;

  void contactTest(ContactResultMap& collisions) override;

  /**
   * @brief A a bullet collision object to the manager
   * @param cow The tesseract bullet collision object
   */
  void addCollisionObject(const COWPtr& cow);

private:
  ContactRequest request_;                            /**< @brief The active contact request message */
  std::unique_ptr<btCollisionDispatcher> dispatcher_; /**< @brief The bullet collision dispatcher used for getting
                                                         object to object collison algorithm */
  btDispatcherInfo dispatch_info_;              /**< @brief The bullet collision dispatcher configuration information */
  btDefaultCollisionConfiguration coll_config_; /**< @brief The bullet collision configuration */
  Link2Cow link2cow_;        /**< @brief A map of all (static and active) collision objects being managed */
  std::vector<COWPtr> cows_; /**< @brief A vector of collision objects (active followed by static) */
  Link2Cow link2castcow_;    /**< @brief A map of cast (active) collision objects being managed. */
};
typedef std::shared_ptr<BulletCastSimpleManager> BulletCastSimpleManagerPtr;

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

/** @brief A BVH implementaiton of a tesseract contact manager */
class BulletCastBVHManager : public ContinuousContactManagerBase
{
public:
  BulletCastBVHManager();
  ~BulletCastBVHManager();

  ContinuousContactManagerBasePtr clone() const override;

  bool addCollisionObject(const std::string& name,
                          const int& mask_id,
                          const std::vector<shapes::ShapeConstPtr>& shapes,
                          const VectorIsometry3d& shape_poses,
                          const CollisionObjectTypeVector& collision_object_types,
                          bool enabled = true) override;

  bool hasCollisionObject(const std::string& name) const override;

  bool removeCollisionObject(const std::string& name) override;

  bool enableCollisionObject(const std::string& name) override;

  bool disableCollisionObject(const std::string& name) override;

  void setCollisionObjectsTransform(const std::string& name, const Eigen::Isometry3d& pose) override;

  void setCollisionObjectsTransform(const std::vector<std::string>& names, const VectorIsometry3d& poses) override;

  void setCollisionObjectsTransform(const TransformMap& transforms) override;

  void setCollisionObjectsTransform(const std::string& name,
                                    const Eigen::Isometry3d& pose1,
                                    const Eigen::Isometry3d& pose2) override;

  void setCollisionObjectsTransform(const std::vector<std::string>& names,
                                    const VectorIsometry3d& pose1,
                                    const VectorIsometry3d& pose2) override;

  void setCollisionObjectsTransform(const TransformMap& pose1, const TransformMap& pose2) override;

  void setContactRequest(const ContactRequest& req) override;

  const ContactRequest& getContactRequest() const override;

  void contactTest(ContactResultMap& collisions) override;

  /**
   * @brief A a bullet collision object to the manager
   * @param cow The tesseract bullet collision object
   */
  void addCollisionObject(const COWPtr& cow);

private:
  ContactRequest request_;                            /**< @brief The active contact request message */
  std::unique_ptr<btCollisionDispatcher> dispatcher_; /**< @brief The bullet collision dispatcher used for getting
                                                         object to object collison algorithm */
  btDispatcherInfo dispatch_info_;              /**< @brief The bullet collision dispatcher configuration information */
  btDefaultCollisionConfiguration coll_config_; /**< @brief The bullet collision configuration */
  std::unique_ptr<btBroadphaseInterface> broadphase_; /**< @brief The bullet broadphase interface */
  Link2Cow link2cow_;                                 /**< @brief A map of all collision objects being managed */
  Link2Cow link2castcow_;                             /**< @brief A map of cast collision objects being managed. */

  /**
   * @brief Perform a contact test for the provided object which is not part of the manager
   * @param cow The Collision object
   * @param collisions The collision results
   */
  void contactTest(const COWPtr& cow, ContactDistanceData& collisions);
};
typedef std::shared_ptr<BulletCastBVHManager> BulletCastBVHManagerPtr;

template <typename T>
void constructCastContactManager(T& manager,
                                 const Link2Cow& link2cow,
                                 const ContactRequest& req,
                                 const TransformMap& transforms1,
                                 const TransformMap& transforms2)
{
  assert(transforms1.size() == transforms2.size());

  auto it1 = transforms1.begin();
  auto it2 = transforms2.begin();
  while (it1 != transforms1.end())
  {
    COWPtr new_cow = link2cow.at(it1->first)->clone();
    if (!new_cow || !new_cow->m_enabled)
    {
      std::advance(it1, 1);
      std::advance(it2, 1);
      continue;
    }

    assert(new_cow->getCollisionShape());
    assert(transforms2.find(it1->first) != transforms2.end());

    new_cow->m_collisionFilterGroup = btBroadphaseProxy::KinematicFilter;
    if (!req.link_names.empty())
    {
      bool check = (std::find_if(req.link_names.begin(), req.link_names.end(), [&](std::string link) {
                      return link == it1->first;
                    }) == req.link_names.end());
      if (check)
      {
        new_cow->m_collisionFilterGroup = btBroadphaseProxy::StaticFilter;
      }
    }

    if (new_cow->m_collisionFilterGroup == btBroadphaseProxy::StaticFilter)
    {
      new_cow->setWorldTransform(convertEigenToBt(it1->second));
      new_cow->m_collisionFilterMask = btBroadphaseProxy::KinematicFilter;
    }
    else
    {
      if (btBroadphaseProxy::isConvex(new_cow->getCollisionShape()->getShapeType()))
      {
        btConvexShape* convex = static_cast<btConvexShape*>(new_cow->getCollisionShape());
        assert(convex != NULL);

        btTransform tf1 = convertEigenToBt(it1->second);
        btTransform tf2 = convertEigenToBt(it2->second);

        CastHullShape* shape = new CastHullShape(convex, tf1.inverseTimes(tf2));
        assert(shape != NULL);

        new_cow->manage(shape);
        new_cow->setCollisionShape(shape);
        new_cow->setWorldTransform(tf1);
      }
      else if (btBroadphaseProxy::isCompound(new_cow->getCollisionShape()->getShapeType()))
      {
        btCompoundShape* compound = static_cast<btCompoundShape*>(new_cow->getCollisionShape());
        const Eigen::Isometry3d& tf1 = it1->second;
        const Eigen::Isometry3d& tf2 = it2->second;

        btCompoundShape* new_compound =
            new btCompoundShape(/*dynamicAABBtree=*/BULLET_COMPOUND_USE_DYNAMIC_AABB, compound->getNumChildShapes());

        for (int i = 0; i < compound->getNumChildShapes(); ++i)
        {
          btConvexShape* convex = static_cast<btConvexShape*>(compound->getChildShape(i));
          assert(convex != NULL);

          btTransform geomTrans = compound->getChildTransform(i);
          btTransform child_tf1 = convertEigenToBt(tf1) * geomTrans;
          btTransform child_tf2 = convertEigenToBt(tf2) * geomTrans;

          btCollisionShape* subshape = new CastHullShape(convex, child_tf1.inverseTimes(child_tf2));
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
        new_cow->setWorldTransform(convertEigenToBt(tf1));
      }
      else
      {
        ROS_ERROR("I can only continuous collision check convex shapes and "
                  "compound shapes made of convex shapes");
      }

      new_cow->m_collisionFilterMask = btBroadphaseProxy::StaticFilter;
    }

    new_cow->setContactProcessingThreshold(req.contact_distance);
    manager.addCollisionObject(new_cow);
    std::advance(it1, 1);
    std::advance(it2, 1);
  }
}
}
}

#endif  // TESSERACT_COLLISION_BULLET_CAST_MANAGERS_H
