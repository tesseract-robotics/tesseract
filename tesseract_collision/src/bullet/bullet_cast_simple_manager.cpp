/**
 * @file bullet_cast_simple_manager.cpp
 * @brief Tesseract ROS Bullet Cast(continuous) Simple Manager implementation.
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

#include "tesseract_collision/bullet/bullet_cast_simple_manager.h"

namespace tesseract
{
namespace tesseract_bullet
{

BulletCastSimpleManager::BulletCastSimpleManager()
{
  dispatcher_.reset(new btCollisionDispatcher(&coll_config_));

  dispatcher_->registerCollisionCreateFunc(
      BOX_SHAPE_PROXYTYPE,
      BOX_SHAPE_PROXYTYPE,
      coll_config_.getCollisionAlgorithmCreateFunc(CONVEX_SHAPE_PROXYTYPE, CONVEX_SHAPE_PROXYTYPE));

  dispatcher_->setDispatcherFlags(dispatcher_->getDispatcherFlags() &
                                  ~btCollisionDispatcher::CD_USE_RELATIVE_CONTACT_BREAKING_THRESHOLD);
}

ContinuousContactManagerBasePtr BulletCastSimpleManager::clone() const
{
  BulletCastSimpleManagerPtr manager(new BulletCastSimpleManager());

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

bool BulletCastSimpleManager::addCollisionObject(const std::string& name,
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

bool BulletCastSimpleManager::hasCollisionObject(const std::string& name) const
{
  return (link2cow_.find(name) != link2cow_.end());
}

bool BulletCastSimpleManager::removeCollisionObject(const std::string& name)
{
  auto it = link2cow_.find(name);
  if (it != link2cow_.end())
  {
    cows_.erase(std::find(cows_.begin(), cows_.end(), it->second));
    link2cow_.erase(name);
    link2castcow_.erase(name);
    return true;
  }

  return false;
}

bool BulletCastSimpleManager::enableCollisionObject(const std::string& name)
{
  auto it = link2cow_.find(name);
  if (it != link2cow_.end())
  {
    it->second->m_enabled = true;
    link2castcow_[name]->m_enabled = true;
    return true;
  }

  return false;
}

bool BulletCastSimpleManager::disableCollisionObject(const std::string& name)
{
  auto it = link2cow_.find(name);
  if (it != link2cow_.end())
  {
    it->second->m_enabled = false;
    link2castcow_[name]->m_enabled = false;
    return true;
  }

  return false;
}

void BulletCastSimpleManager::setCollisionObjectsTransform(const std::string& name, const Eigen::Isometry3d& pose)
{
  // TODO: Find a way to remove this check. Need to store information in Tesseract EnvState indicating transforms with
  // geometry
  auto it = link2cow_.find(name);
  if (it != link2cow_.end())
    it->second->setWorldTransform(convertEigenToBt(pose));
}

void BulletCastSimpleManager::setCollisionObjectsTransform(const std::vector<std::string>& names,
                                                           const VectorIsometry3d& poses)
{
  assert(names.size() == poses.size());
  for (auto i = 0u; i < names.size(); ++i)
    setCollisionObjectsTransform(names[i], poses[i]);
}

void BulletCastSimpleManager::setCollisionObjectsTransform(const TransformMap& transforms)
{
  for (const auto& transform : transforms)
    setCollisionObjectsTransform(transform.first, transform.second);
}

void BulletCastSimpleManager::setCollisionObjectsTransform(const std::string& name,
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

    if (btBroadphaseProxy::isConvex(cow->getCollisionShape()->getShapeType()))
    {
      assert(dynamic_cast<CastHullShape*>(cow->getCollisionShape()) != nullptr);
      static_cast<CastHullShape*>(cow->getCollisionShape())->updateCastTransform(tf1.inverseTimes(tf2));
    }
    else if (btBroadphaseProxy::isCompound(cow->getCollisionShape()->getShapeType()))
    {
      assert(dynamic_cast<btCompoundShape*>(cow->getCollisionShape()) != nullptr);
      btCompoundShape* compound = static_cast<btCompoundShape*>(cow->getCollisionShape());
      for (int i = 0; i < compound->getNumChildShapes(); ++i)
      {
        assert(!btBroadphaseProxy::isCompound(compound->getChildShape(i)->getShapeType()));
        assert(dynamic_cast<CastHullShape*>(compound->getChildShape(i)) != nullptr);
        static_cast<CastHullShape*>(compound->getChildShape(i))->updateCastTransform(tf1.inverseTimes(tf2));
        compound->updateChildTransform(i, compound->getChildTransform(i), false); // This is required to update the BVH tree
      }
      compound->recalculateLocalAabb();
    }

    cow->setWorldTransform(tf1);
    link2cow_[name]->setWorldTransform(tf1);
  }
}

void BulletCastSimpleManager::setCollisionObjectsTransform(const std::vector<std::string>& names,
                                                           const VectorIsometry3d& pose1,
                                                           const VectorIsometry3d& pose2)
{
  assert(names.size() == pose1.size());
  assert(names.size() == pose2.size());
  for (auto i = 0u; i < names.size(); ++i)
    setCollisionObjectsTransform(names[i], pose1[i], pose2[i]);
}

void BulletCastSimpleManager::setCollisionObjectsTransform(const TransformMap& pose1, const TransformMap& pose2)
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

void BulletCastSimpleManager::contactTest(ContactResultMap& collisions)
{
  ContactDistanceData cdata(&request_, &collisions);

  for (auto cow1_iter = cows_.begin(); cow1_iter != (cows_.end() - 1); cow1_iter++)
  {
    const COWPtr& cow1 = *cow1_iter;

    if (cow1->m_collisionFilterGroup != btBroadphaseProxy::KinematicFilter)
      break;

    if (!cow1->m_enabled)
      continue;

    btVector3 aabbMin[2], aabbMax[2];
    cow1->getCollisionShape()->getAabb(cow1->getWorldTransform(), aabbMin[0], aabbMax[0]);

    // need to increase the aabb for contact thresholds
    btVector3 contactThreshold1(cow1->getContactProcessingThreshold(),
                                cow1->getContactProcessingThreshold(),
                                cow1->getContactProcessingThreshold());
    aabbMin[0] -= contactThreshold1;
    aabbMax[0] += contactThreshold1;

    btCollisionObjectWrapper obA(0, cow1->getCollisionShape(), cow1.get(), cow1->getWorldTransform(), -1, -1);

    CastCollisionCollector cc(cdata, cow1, cow1->getContactProcessingThreshold());
    for (auto cow2_iter = cow1_iter + 1; cow2_iter != cows_.end(); cow2_iter++)
    {
      assert(!cdata.done);

      const COWPtr& cow2 = *cow2_iter;

      cow2->getCollisionShape()->getAabb(cow2->getWorldTransform(), aabbMin[1], aabbMax[1]);

      // need to increase the aabb for contact thresholds
      btVector3 contactThreshold2(cow2->getContactProcessingThreshold(),
                                  cow2->getContactProcessingThreshold(),
                                  cow2->getContactProcessingThreshold());
      aabbMin[1] -= contactThreshold2;
      aabbMax[1] += contactThreshold2;

      bool aabb_check = (aabbMin[0][0] <= aabbMax[1][0] && aabbMax[0][0] >= aabbMin[1][0]) &&
                        (aabbMin[0][1] <= aabbMax[1][1] && aabbMax[0][1] >= aabbMin[1][1]) &&
                        (aabbMin[0][2] <= aabbMax[1][2] && aabbMax[0][2] >= aabbMin[1][2]);

      if (aabb_check)
      {
        bool needs_collision = needsCollisionCheck(*cow1, *cow2, request_.isContactAllowed, false);

        if (needs_collision)
        {
          btCollisionObjectWrapper obB(0, cow2->getCollisionShape(), cow2.get(), cow2->getWorldTransform(), -1, -1);

          btCollisionAlgorithm* algorithm = dispatcher_->findAlgorithm(&obA, &obB, 0, BT_CLOSEST_POINT_ALGORITHMS);
          assert(algorithm != nullptr);
          if (algorithm)
          {
            TesseractBridgedManifoldResult contactPointResult(&obA, &obB, cc);
            contactPointResult.m_closestPointDistanceThreshold = cc.m_closestDistanceThreshold;

            // discrete collision detection query
            algorithm->processCollision(&obA, &obB, dispatch_info_, &contactPointResult);

            algorithm->~btCollisionAlgorithm();
            dispatcher_->freeCollisionAlgorithm(algorithm);
          }
        }
      }

      if (cdata.done)
        break;
    }
  }
}

void BulletCastSimpleManager::setContactRequest(const ContactRequest& req)
{
  request_ = req;
  cows_.clear();
  cows_.reserve(link2cow_.size());

  // Now need to update the broadphase with correct aabb
  for (auto& co : link2cow_)
  {
    COWPtr& cow = co.second;

    // Update with request
    updateCollisionObjectWithRequest(request_, *cow);

    // Get the cast collision object
    COWPtr cast_cow = link2castcow_[cow->getName()];

    // Update with request
    updateCollisionObjectWithRequest(request_, *cast_cow);

    // Add to collision object vector
    if (cow->m_collisionFilterGroup == btBroadphaseProxy::KinematicFilter)
    {
      cows_.insert(cows_.begin(), cast_cow);
    }
    else
    {
      cows_.push_back(cow);
    }
  }
}

const ContactRequest& BulletCastSimpleManager::getContactRequest() const { return request_; }
void BulletCastSimpleManager::addCollisionObject(const COWPtr& cow)
{
  link2cow_[cow->getName()] = cow;

  // Create cast collision object
  COWPtr cast_cow = makeCastCollisionObject(cow);

  // Add it to the cast map
  link2castcow_[cast_cow->getName()] = cast_cow;

  if (cow->m_collisionFilterGroup == btBroadphaseProxy::KinematicFilter)
    cows_.insert(cows_.begin(), cast_cow);
  else
    cows_.push_back(cow);
}

}
}
