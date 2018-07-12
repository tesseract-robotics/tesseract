/**
 * @file bullet_discrete_managers.h
 * @brief Tesseract ROS Bullet discrete collision managers.
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
#ifndef TESSERACT_COLLISION_BULLET_DISCRETE_MANAGERS_H
#define TESSERACT_COLLISION_BULLET_DISCRETE_MANAGERS_H

#include <tesseract_collision/bullet/bullet_utils.h>
#include <tesseract_core/discrete_contact_manager_base.h>
namespace tesseract
{

/** @brief A simple implementaiton of a bullet manager which does not use BHV */
class BulletDiscreteSimpleManager : public DiscreteContactManagerBase
{
public:

  BulletDiscreteSimpleManager();
  ~BulletDiscreteSimpleManager() {}

  DiscreteContactManagerBasePtr clone() const override;

  bool addCollisionObject(const std::string& name,
                          const int& mask_id,
                          const std::vector<shapes::ShapeConstPtr>& shapes,
                          const EigenSTL::vector_Affine3d& shape_poses,
                          const CollisionObjectTypeVector& collision_object_types,
                          bool enabled = true) override;

  bool hasCollisionObject(const std::string& name) const override;

  bool removeCollisionObject(const std::string& name) override;

  bool enableCollisionObject(const std::string& name) override;

  bool disableCollisionObject(const std::string& name) override;

  void setCollisionObjectsTransform(const std::string& name, const Eigen::Affine3d& pose) override;

  void setCollisionObjectsTransform(const std::vector<std::string>& names, const EigenSTL::vector_Affine3d& poses) override;

  void setCollisionObjectsTransform(const TransformMap& transforms) override;

  void contactTest(ContactResultMap &collisions) override;

  void setContactRequest(const ContactRequest& req) override;

  const ContactRequest& getContactRequest() const override;

  /**
   * @brief A a bullet collision object to the manager
   * @param cow The tesseract bullet collision object
   */
  void addCollisionObject(COWPtr& cow);

  /**
   * @brief Return collision objects
   * @return A map of collision objects <name, collision object>
   */
  const Link2Cow& getCollisionObjects() const;

private:
  ContactRequest request_;                            /**< @brief The active contact request message */
  std::unique_ptr<btCollisionDispatcher> dispatcher_; /**< @brief The bullet collision dispatcher used for getting object to object collison algorithm */
  btDispatcherInfo dispatch_info_;                    /**< @brief The bullet collision dispatcher configuration information */
  btDefaultCollisionConfiguration coll_config_;       /**< @brief The bullet collision configuration */
  Link2Cow link2cow_;                                 /**< @brief A map of all (static and active) collision objects being managed */
  std::vector<COWPtr> cows_;                          /**< @brief A vector of collision objects (active followed by static) */

};
typedef std::shared_ptr<BulletDiscreteSimpleManager> BulletDiscreteSimpleManagerPtr;


struct DiscreteCollisionCollector : public btCollisionWorld::ContactResultCallback
{
  ContactDistanceData& collisions_;
  const COWPtr cow_;
  double contact_distance_;
  bool verbose_;

  DiscreteCollisionCollector(ContactDistanceData& collisions, const COWPtr cow, double contact_distance, bool verbose = false)
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
    return !collisions_.done &&
        needsCollisionCheck(*cow_,
                            *(static_cast<CollisionObjectWrapper*>(proxy0->m_clientObject)),
                            collisions_.req->isContactAllowed,
                            verbose_);
  }
};

/** @brief A BVH implementaiton of a bullet manager */
class BulletDiscreteBVHManager : public DiscreteContactManagerBase
{
public:

  BulletDiscreteBVHManager();
  ~BulletDiscreteBVHManager();

  DiscreteContactManagerBasePtr clone() const override;

  bool addCollisionObject(const std::string& name,
                          const int& mask_id,
                          const std::vector<shapes::ShapeConstPtr>& shapes,
                          const EigenSTL::vector_Affine3d& shape_poses,
                          const CollisionObjectTypeVector& collision_object_types,
                          bool enabled = true) override;

  bool hasCollisionObject(const std::string& name) const override;

  bool removeCollisionObject(const std::string& name) override;

  bool enableCollisionObject(const std::string& name) override;

  bool disableCollisionObject(const std::string& name) override;

  void setCollisionObjectsTransform(const std::string& name, const Eigen::Affine3d& pose) override;

  void setCollisionObjectsTransform(const std::vector<std::string>& names, const EigenSTL::vector_Affine3d& poses) override;

  void setCollisionObjectsTransform(const TransformMap& transforms) override;

  void contactTest(ContactResultMap& collisions) override;

  void setContactRequest(const ContactRequest& req) override;

  const ContactRequest& getContactRequest() const override;

  /**
   * @brief A a bullet collision object to the manager
   * @param cow The tesseract bullet collision object
   */
  void addCollisionObject(COWPtr& cow);

  /**
   * @brief Return collision objects
   * @return A map of collision objects <name, collision object>
   */
  const Link2Cow& getCollisionObjects() const;

private:
  ContactRequest request_;                            /**< @brief The active contact request message */
  std::unique_ptr<btCollisionDispatcher> dispatcher_; /**< @brief The bullet collision dispatcher used for getting object to object collison algorithm */
  btDispatcherInfo dispatch_info_;                    /**< @brief The bullet collision dispatcher configuration information */
  btDefaultCollisionConfiguration coll_config_;       /**< @brief The bullet collision configuration */
  std::unique_ptr<btBroadphaseInterface> broadphase_; /**< @brief The bullet broadphase interface */
  Link2Cow link2cow_;                                 /**< @brief A map of all (static and active) collision objects being managed */

  /**
   * @brief Perform a contact test for the provided object which is not part of the manager
   * @param cow The Collision object
   * @param collisions The collision results
   */
  void contactTest(const COWPtr& cow, ContactDistanceData& collisions);

};

typedef std::shared_ptr<BulletDiscreteBVHManager> BulletDiscreteBVHManagerPtr;

template<typename T>
void constructDiscreteContactManager(T& manager,
                                     const Link2Cow& link2cow,
                                     const ContactRequest& req,
                                     const TransformMap& transforms)
{
  for (const auto& transform : transforms)
  {
    COWPtr new_cow = link2cow.at(transform.first)->clone();
    if (!new_cow || !new_cow->m_enabled)
      continue;

    assert(new_cow->getCollisionShape());
    assert(new_cow->getCollisionShape()->getShapeType() != CUSTOM_CONVEX_SHAPE_TYPE);

    new_cow->setWorldTransform(convertEigenToBt(transform.second));

    // For descrete checks we can check static to kinematic and kinematic to
    // kinematic
    new_cow->m_collisionFilterGroup = btBroadphaseProxy::KinematicFilter;
    if (!req.link_names.empty())
    {
      bool check = (std::find_if(req.link_names.begin(), req.link_names.end(), [&](std::string link) {
                      return link == transform.first;
                    }) == req.link_names.end());
      if (check)
      {
        new_cow->m_collisionFilterGroup = btBroadphaseProxy::StaticFilter;
      }
    }

    if (new_cow->m_collisionFilterGroup == btBroadphaseProxy::StaticFilter)
    {
      new_cow->m_collisionFilterMask = btBroadphaseProxy::KinematicFilter;
    }
    else
    {
      new_cow->m_collisionFilterMask = btBroadphaseProxy::StaticFilter | btBroadphaseProxy::KinematicFilter;
    }

    new_cow->setContactProcessingThreshold(req.contact_distance);
    manager.addCollisionObject(new_cow);
  }
}

}
#endif // TESSERACT_COLLISION_BULLET_DISCRETE_MANAGERS_H
