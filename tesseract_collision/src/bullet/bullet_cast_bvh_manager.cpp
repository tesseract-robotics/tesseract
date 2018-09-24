/**
 * @file bullet_cast_bvh_manager.cpp
 * @brief Tesseract ROS Bullet Cast(continuous) BVH Manager implementation.
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

#include "tesseract_collision/bullet/bullet_cast_bvh_manager.h"

namespace tesseract
{
namespace tesseract_bullet
{

BulletCastBVHManager::BulletCastBVHManager()
{
  dispatcher_.reset(new btCollisionDispatcher(&coll_config_));

  dispatcher_->registerCollisionCreateFunc(
      BOX_SHAPE_PROXYTYPE,
      BOX_SHAPE_PROXYTYPE,
      coll_config_.getCollisionAlgorithmCreateFunc(CONVEX_SHAPE_PROXYTYPE, CONVEX_SHAPE_PROXYTYPE));

  dispatcher_->setDispatcherFlags(dispatcher_->getDispatcherFlags() &
                                  ~btCollisionDispatcher::CD_USE_RELATIVE_CONTACT_BREAKING_THRESHOLD);

  broadphase_.reset(new btDbvtBroadphase());
}

BulletCastBVHManager::~BulletCastBVHManager()
{
  // clean up remaining objects
  for (auto& co : link2cow_)
  {
    btCollisionObject* collisionObject = co.second.get();

    btBroadphaseProxy* bp = collisionObject->getBroadphaseHandle();
    if (bp)
    {
      // only clear the cached algorithms
      broadphase_->getOverlappingPairCache()->cleanProxyFromPairs(bp, dispatcher_.get());
      broadphase_->destroyProxy(bp, dispatcher_.get());
      collisionObject->setBroadphaseHandle(0);
    }
  }

  // clean up remaining objects
  for (auto& co : link2castcow_)
  {
    btCollisionObject* collisionObject = co.second.get();

    btBroadphaseProxy* bp = collisionObject->getBroadphaseHandle();
    if (bp)
    {
      // only clear the cached algorithms
      broadphase_->getOverlappingPairCache()->cleanProxyFromPairs(bp, dispatcher_.get());
      broadphase_->destroyProxy(bp, dispatcher_.get());
      collisionObject->setBroadphaseHandle(0);
    }
  }
}

ContinuousContactManagerBasePtr BulletCastBVHManager::clone() const
{
  BulletCastBVHManagerPtr manager(new BulletCastBVHManager());

  for (const auto& cow : link2cow_)
  {
    COWPtr new_cow = cow.second->clone();

    assert(new_cow->getCollisionShape());
    assert(new_cow->getCollisionShape()->getShapeType() != CUSTOM_CONVEX_SHAPE_TYPE);

    new_cow->setWorldTransform(cow.second->getWorldTransform());

    new_cow->setContactProcessingThreshold(request_.contact_distance);
    manager->addCollisionObject(new_cow);
  }
  manager->setContactRequest(request_);
  return manager;
}

bool BulletCastBVHManager::addCollisionObject(const std::string& name,
                                              const int& mask_id,
                                              const std::vector<shapes::ShapeConstPtr>& shapes,
                                              const VectorIsometry3d& shape_poses,
                                              const CollisionObjectTypeVector& collision_object_types,
                                              bool enabled)
{
  COWPtr new_cow = createCollisionObject(name, mask_id, shapes, shape_poses, collision_object_types, enabled);
  if (new_cow != nullptr)
  {
    addCollisionObject(new_cow);
    return true;
  }
  else
  {
    return false;
  }
}

bool BulletCastBVHManager::hasCollisionObject(const std::string& name) const
{
  return (link2cow_.find(name) != link2cow_.end());
}

bool BulletCastBVHManager::removeCollisionObject(const std::string& name)
{
  bool removed = false;
  auto it = link2castcow_.find(name);
  if (it != link2castcow_.end())
  {
    btBroadphaseProxy* bp = it->second->getBroadphaseHandle();
    if (bp)
    {
      // only clear the cached algorithms
      broadphase_->getOverlappingPairCache()->cleanProxyFromPairs(bp, dispatcher_.get());
      broadphase_->destroyProxy(bp, dispatcher_.get());
      it->second->setBroadphaseHandle(0);
    }

    link2castcow_.erase(name);
    removed = true;
  }

  auto it2 = link2cow_.find(name);
  if (it2 != link2cow_.end())
  {
    btBroadphaseProxy* bp = it2->second->getBroadphaseHandle();
    if (bp)
    {
      // only clear the cached algorithms
      broadphase_->getOverlappingPairCache()->cleanProxyFromPairs(bp, dispatcher_.get());
      broadphase_->destroyProxy(bp, dispatcher_.get());
      it2->second->setBroadphaseHandle(0);
    }

    link2cow_.erase(name);
    removed = true;
  }

  return removed;
}

bool BulletCastBVHManager::enableCollisionObject(const std::string& name)
{
  bool enabled = false;
  auto it = link2cow_.find(name);
  if (it != link2cow_.end())
  {
    it->second->m_enabled = true;
    enabled = true;
  }

  auto it2 = link2castcow_.find(name);
  if (it2 != link2castcow_.end())
  {
    it2->second->m_enabled = true;
    enabled = true;
  }

  return enabled;
}

bool BulletCastBVHManager::disableCollisionObject(const std::string& name)
{
  bool disabled = false;
  auto it = link2cow_.find(name);
  if (it != link2cow_.end())
  {
    it->second->m_enabled = false;
    disabled = true;
  }

  auto it2 = link2castcow_.find(name);
  if (it2 != link2castcow_.end())
  {
    it2->second->m_enabled = true;
    disabled = true;
  }

  return disabled;
}

void BulletCastBVHManager::setCollisionObjectsTransform(const std::string& name, const Eigen::Isometry3d& pose)
{
  // TODO: Find a way to remove this check. Need to store information in Tesseract EnvState indicating transforms with
  // geometry
  auto it = link2cow_.find(name);
  if (it != link2cow_.end())
    it->second->setWorldTransform(convertEigenToBt(pose));
}

void BulletCastBVHManager::setCollisionObjectsTransform(const std::vector<std::string>& names,
                                                        const VectorIsometry3d& poses)
{
  assert(names.size() == poses.size());
  for (auto i = 0u; i < names.size(); ++i)
    setCollisionObjectsTransform(names[i], poses[i]);
}

void BulletCastBVHManager::setCollisionObjectsTransform(const TransformMap& transforms)
{
  for (const auto& transform : transforms)
    setCollisionObjectsTransform(transform.first, transform.second);
}

void BulletCastBVHManager::setCollisionObjectsTransform(const std::string& name,
                                                        const Eigen::Isometry3d& pose1,
                                                        const Eigen::Isometry3d& pose2)
{
  // TODO: Find a way to remove this check. Need to store information in Tesseract EnvState indicating transforms with
  // geometry
  auto it = link2castcow_.find(name);
  if (it != link2castcow_.end())
  {
    COWPtr& cow = it->second;
    assert(cow->m_collisionFilterGroup == btBroadphaseProxy::KinematicFilter);

    btTransform tf1 = convertEigenToBt(pose1);
    btTransform tf2 = convertEigenToBt(pose2);

    static_cast<CastHullShape*>(cow->getCollisionShape())->updateCastTransform(tf1.inverseTimes(tf2));
    cow->setWorldTransform(tf1);

    // Now update Broadphase AABB (Copied from BulletWorld updateSingleAabb function
    btVector3 minAabb, maxAabb;
    cow->getCollisionShape()->getAabb(cow->getWorldTransform(), minAabb, maxAabb);
    // need to increase the aabb for contact thresholds
    btVector3 contactThreshold(cow->getContactProcessingThreshold(),
                               cow->getContactProcessingThreshold(),
                               cow->getContactProcessingThreshold());
    minAabb -= contactThreshold;
    maxAabb += contactThreshold;

    if (dispatch_info_.m_useContinuous && cow->getInternalType() == btCollisionObject::CO_RIGID_BODY &&
        !cow->isStaticOrKinematicObject())
    {
      btVector3 minAabb2, maxAabb2;
      cow->getCollisionShape()->getAabb(cow->getInterpolationWorldTransform(), minAabb2, maxAabb2);
      minAabb2 -= contactThreshold;
      maxAabb2 += contactThreshold;
      minAabb.setMin(minAabb2);
      maxAabb.setMax(maxAabb2);
    }

    // moving objects should be moderately sized, probably something wrong if not
    if (cow->isStaticObject() || ((maxAabb - minAabb).length2() < btScalar(1e12)))
    {
      broadphase_->setAabb(cow->getBroadphaseHandle(), minAabb, maxAabb, dispatcher_.get());
    }
    else
    {
      // something went wrong, investigate
      // this assert is unwanted in 3D modelers (danger of loosing work)
      cow->setActivationState(DISABLE_SIMULATION);
    }
  }
}

void BulletCastBVHManager::setCollisionObjectsTransform(const std::vector<std::string>& names,
                                                        const VectorIsometry3d& pose1,
                                                        const VectorIsometry3d& pose2)
{
  assert(names.size() == pose1.size() == pose2.size());
  for (auto i = 0u; i < names.size(); ++i)
    setCollisionObjectsTransform(names[i], pose1[i], pose2[i]);
}

void BulletCastBVHManager::setCollisionObjectsTransform(const TransformMap& pose1, const TransformMap& pose2)
{
  assert(pose1.size() == pose2.size());
  auto it1 = pose1.begin();
  auto it2 = pose2.begin();
  while (it1 != pose1.end())
  {
    assert(pose1.find(it1->first) != pose2.end());
    setCollisionObjectsTransform(it1->first, it1->second, it2->second);
    std::advance(it1, 1);
    std::advance(it2, 1);
  }
}

void BulletCastBVHManager::contactTest(ContactResultMap& collisions)
{
  ContactDistanceData cdata(&request_, &collisions);

  broadphase_->calculateOverlappingPairs(dispatcher_.get());

  btOverlappingPairCache* pairCache = broadphase_->getOverlappingPairCache();

  TesseractCollisionPairCallback collisionCallback(dispatch_info_, dispatcher_.get(), cdata);

  pairCache->processAllOverlappingPairs(&collisionCallback, dispatcher_.get());
}

void BulletCastBVHManager::setContactRequest(const ContactRequest& req)
{
  request_ = req;

  // Now need to update the broadphase with correct aabb
  for (auto& co : link2cow_)
  {
    COWPtr& cow = co.second;

    // Need to check if a collision object is still active
    if (cow->m_collisionFilterGroup == btBroadphaseProxy::KinematicFilter)
    {
      // Update with request
      updateCollisionObjectWithRequest(request_, *cow);

      bool still_active = (std::find_if(req.link_names.begin(), req.link_names.end(), [&cow](const std::string& link) {
                             return link == cow->getName();
                           }) != req.link_names.end());

      if (still_active)
      {
        // Get the existing active collision object
        COWPtr active_cow = link2castcow_[cow->getName()];

        // Update with request
        updateCollisionObjectWithRequest(request_, *active_cow);

        // Calculate the aabb
        btVector3 minAabb, maxAabb;
        active_cow->getCollisionShape()->getAabb(cow->getWorldTransform(), minAabb, maxAabb);
        btVector3 contactThreshold(active_cow->getContactProcessingThreshold(),
                                   active_cow->getContactProcessingThreshold(),
                                   active_cow->getContactProcessingThreshold());
        minAabb -= contactThreshold;
        maxAabb += contactThreshold;

        // Update the broadphase aabb
        broadphase_->setAabb(active_cow->getBroadphaseHandle(), minAabb, maxAabb, dispatcher_.get());
      }
      else
      {
        COWPtr active_cow = link2castcow_[cow->getName()];

        // Remove the active collision object from the broadphase
        btBroadphaseProxy* bp = active_cow->getBroadphaseHandle();
        if (bp)
        {
          // only clear the cached algorithms
          broadphase_->getOverlappingPairCache()->cleanProxyFromPairs(bp, dispatcher_.get());
          broadphase_->destroyProxy(bp, dispatcher_.get());
          active_cow->setBroadphaseHandle(0);
        }

        // Remove the collision object from active map
        link2castcow_.erase(cow->getName());

        // Calculate broadphase aabb
        btVector3 minAabb, maxAabb;
        cow->getCollisionShape()->getAabb(cow->getWorldTransform(), minAabb, maxAabb);
        btVector3 contactThreshold(cow->getContactProcessingThreshold(),
                                   cow->getContactProcessingThreshold(),
                                   cow->getContactProcessingThreshold());
        minAabb -= contactThreshold;
        maxAabb += contactThreshold;

        // Add the static collision object to the broadphase
        int type = cow->getCollisionShape()->getShapeType();
        cow->setBroadphaseHandle(broadphase_->createProxy(minAabb,
                                                          maxAabb,
                                                          type,
                                                          cow.get(),
                                                          cow->m_collisionFilterGroup,
                                                          cow->m_collisionFilterMask,
                                                          dispatcher_.get()));
      }
    }
    else
    {
      // Update with request
      updateCollisionObjectWithRequest(request_, *cow);

      bool now_active = (std::find_if(req.link_names.begin(), req.link_names.end(), [&cow](const std::string& link) {
                           return link == cow->getName();
                         }) != req.link_names.end());
      if (now_active)
      {
        // Create active collision object
        COWPtr active_cow = makeCastCollisionObject(cow);

        // Update with request
        updateCollisionObjectWithRequest(request_, *active_cow);

        // Remove the static collision object from the broadphase
        btBroadphaseProxy* bp = cow->getBroadphaseHandle();
        if (bp)
        {
          // only clear the cached algorithms
          broadphase_->getOverlappingPairCache()->cleanProxyFromPairs(bp, dispatcher_.get());
          broadphase_->destroyProxy(bp, dispatcher_.get());
          cow->setBroadphaseHandle(0);
        }

        // Calculate broadphase aabb
        btVector3 minAabb, maxAabb;
        active_cow->getCollisionShape()->getAabb(active_cow->getWorldTransform(), minAabb, maxAabb);
        btVector3 contactThreshold(active_cow->getContactProcessingThreshold(),
                                   active_cow->getContactProcessingThreshold(),
                                   active_cow->getContactProcessingThreshold());
        minAabb -= contactThreshold;
        maxAabb += contactThreshold;

        // Add the active collision object to the broadphase
        int type = active_cow->getCollisionShape()->getShapeType();
        active_cow->setBroadphaseHandle(broadphase_->createProxy(minAabb,
                                                                 maxAabb,
                                                                 type,
                                                                 active_cow.get(),
                                                                 active_cow->m_collisionFilterGroup,
                                                                 active_cow->m_collisionFilterMask,
                                                                 dispatcher_.get()));

        // Add it to the active map
        link2castcow_[active_cow->getName()] = active_cow;
      }
      else
      {
        // Calculate the aabb
        btVector3 minAabb, maxAabb;
        cow->getCollisionShape()->getAabb(cow->getWorldTransform(), minAabb, maxAabb);
        btVector3 contactThreshold(cow->getContactProcessingThreshold(),
                                   cow->getContactProcessingThreshold(),
                                   cow->getContactProcessingThreshold());
        minAabb -= contactThreshold;
        maxAabb += contactThreshold;

        // Update the broadphase aabb
        broadphase_->setAabb(cow->getBroadphaseHandle(), minAabb, maxAabb, dispatcher_.get());
      }
    }
  }
}

const ContactRequest& BulletCastBVHManager::getContactRequest() const { return request_; }
void BulletCastBVHManager::addCollisionObject(const COWPtr& cow)
{
  link2cow_[cow->getName()] = cow;

  // calculate new AABB
  btTransform trans = cow->getWorldTransform();

  btVector3 minAabb;
  btVector3 maxAabb;
  cow->getCollisionShape()->getAabb(trans, minAabb, maxAabb);

  // need to increase the aabb for contact thresholds
  btVector3 contactThreshold(
      cow->getContactProcessingThreshold(), cow->getContactProcessingThreshold(), cow->getContactProcessingThreshold());
  minAabb -= contactThreshold;
  maxAabb += contactThreshold;

  int type = cow->getCollisionShape()->getShapeType();
  cow->setBroadphaseHandle(broadphase_->createProxy(
      minAabb, maxAabb, type, cow.get(), cow->m_collisionFilterGroup, cow->m_collisionFilterMask, dispatcher_.get()));
}

void BulletCastBVHManager::contactTest(const COWPtr& cow, ContactDistanceData& collisions)
{
  btVector3 aabbMin, aabbMax;
  cow->getCollisionShape()->getAabb(cow->getWorldTransform(), aabbMin, aabbMax);

  // need to increase the aabb for contact thresholds
  btVector3 contactThreshold1(
      cow->getContactProcessingThreshold(), cow->getContactProcessingThreshold(), cow->getContactProcessingThreshold());
  aabbMin -= contactThreshold1;
  aabbMax += contactThreshold1;

  CastCollisionCollector cc(collisions, cow, cow->getContactProcessingThreshold());

  TesseractSingleContactCallback contactCB(cow.get(), dispatcher_.get(), dispatch_info_, cc);

  broadphase_->aabbTest(aabbMin, aabbMax, contactCB);
}
}
}
