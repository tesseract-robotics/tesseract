/**
 * @file bullet_discrete_manager.cpp
 * @brief Tesseract ROS Bullet Manager implementation.
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

#include "tesseract_collision/bullet/bullet_discrete_managers.h"

namespace tesseract
{
namespace tesseract_bullet
{
////////////////////////////////////////////////
/////// BulletDiscreteManagerSimple ////////////
////////////////////////////////////////////////

BulletDiscreteSimpleManager::BulletDiscreteSimpleManager()
{
  dispatcher_.reset(new btCollisionDispatcher(&coll_config_));

  dispatcher_->registerCollisionCreateFunc(
      BOX_SHAPE_PROXYTYPE,
      BOX_SHAPE_PROXYTYPE,
      coll_config_.getCollisionAlgorithmCreateFunc(CONVEX_SHAPE_PROXYTYPE, CONVEX_SHAPE_PROXYTYPE));

  dispatcher_->setDispatcherFlags(dispatcher_->getDispatcherFlags() &
                                  ~btCollisionDispatcher::CD_USE_RELATIVE_CONTACT_BREAKING_THRESHOLD);
}

DiscreteContactManagerBasePtr BulletDiscreteSimpleManager::clone() const
{
  BulletDiscreteSimpleManagerPtr manager(new BulletDiscreteSimpleManager());

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

bool BulletDiscreteSimpleManager::addCollisionObject(const std::string& name,
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

bool BulletDiscreteSimpleManager::hasCollisionObject(const std::string& name) const
{
  return (link2cow_.find(name) != link2cow_.end());
}

bool BulletDiscreteSimpleManager::removeCollisionObject(const std::string& name)
{
  auto it = link2cow_.find(name);
  if (it != link2cow_.end())
  {
    cows_.erase(std::find(cows_.begin(), cows_.end(), it->second));
    link2cow_.erase(name);
    return true;
  }

  return false;
}

bool BulletDiscreteSimpleManager::enableCollisionObject(const std::string& name)
{
  auto it = link2cow_.find(name);
  if (it != link2cow_.end())
  {
    it->second->m_enabled = true;
    return true;
  }
  return false;
}

bool BulletDiscreteSimpleManager::disableCollisionObject(const std::string& name)
{
  auto it = link2cow_.find(name);
  if (it != link2cow_.end())
  {
    it->second->m_enabled = false;
    return true;
  }
  return false;
}

void BulletDiscreteSimpleManager::setCollisionObjectsTransform(const std::string& name, const Eigen::Isometry3d& pose)
{
  // TODO: Find a way to remove this check. Need to store information in Tesseract EnvState indicating transforms with
  // geometry
  auto it = link2cow_.find(name);
  if (it != link2cow_.end())
    it->second->setWorldTransform(convertEigenToBt(pose));
}

void BulletDiscreteSimpleManager::setCollisionObjectsTransform(const std::vector<std::string>& names,
                                                               const VectorIsometry3d& poses)
{
  assert(names.size() == poses.size());
  for (auto i = 0u; i < names.size(); ++i)
    setCollisionObjectsTransform(names[i], poses[i]);
}

void BulletDiscreteSimpleManager::setCollisionObjectsTransform(const TransformMap& transforms)
{
  for (const auto& transform : transforms)
    setCollisionObjectsTransform(transform.first, transform.second);
}

void BulletDiscreteSimpleManager::contactTest(ContactResultMap& collisions)
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

    DiscreteCollisionCollector cc(cdata, cow1, cow1->getContactProcessingThreshold());
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

void BulletDiscreteSimpleManager::setContactRequest(const ContactRequest& req)
{
  request_ = req;
  cows_.clear();
  cows_.reserve(link2cow_.size());

  for (auto& element : link2cow_)
  {
    updateCollisionObjectWithRequest(request_, *element.second);

    // Update collision object vector
    if (element.second->m_collisionFilterGroup == btBroadphaseProxy::KinematicFilter)
      cows_.insert(cows_.begin(), element.second);
    else
      cows_.push_back(element.second);
  }
}

const ContactRequest& BulletDiscreteSimpleManager::getContactRequest() const { return request_; }
void BulletDiscreteSimpleManager::addCollisionObject(const COWPtr& cow)
{
  link2cow_[cow->getName()] = cow;

  if (cow->m_collisionFilterGroup == btBroadphaseProxy::KinematicFilter)
    cows_.insert(cows_.begin(), cow);
  else
    cows_.push_back(cow);
}

const Link2Cow& BulletDiscreteSimpleManager::getCollisionObjects() const { return link2cow_; }
////////////////////////////////////////////////
////////// BulletDiscreteBVHManager ////////////
////////////////////////////////////////////////

BulletDiscreteBVHManager::BulletDiscreteBVHManager()
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

BulletDiscreteBVHManager::~BulletDiscreteBVHManager()
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
}

DiscreteContactManagerBasePtr BulletDiscreteBVHManager::clone() const
{
  BulletDiscreteBVHManagerPtr manager(new BulletDiscreteBVHManager());

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

bool BulletDiscreteBVHManager::addCollisionObject(const std::string& name,
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

bool BulletDiscreteBVHManager::hasCollisionObject(const std::string& name) const
{
  return (link2cow_.find(name) != link2cow_.end());
}

bool BulletDiscreteBVHManager::removeCollisionObject(const std::string& name)
{
  auto it = link2cow_.find(name);  // Levi TODO: Should these check be removed?
  if (it != link2cow_.end())
  {
    btBroadphaseProxy* bp = it->second->getBroadphaseHandle();
    if (bp)
    {
      // only clear the cached algorithms
      broadphase_->getOverlappingPairCache()->cleanProxyFromPairs(bp, dispatcher_.get());
      broadphase_->destroyProxy(bp, dispatcher_.get());
      it->second->setBroadphaseHandle(0);
    }

    link2cow_.erase(name);
    return true;
  }

  return false;
}

bool BulletDiscreteBVHManager::enableCollisionObject(const std::string& name)
{
  auto it = link2cow_.find(name);  // Levi TODO: Should these check be removed?
  if (it != link2cow_.end())
  {
    it->second->m_enabled = true;
    return true;
  }
  return false;
}

bool BulletDiscreteBVHManager::disableCollisionObject(const std::string& name)
{
  auto it = link2cow_.find(name);  // Levi TODO: Should these check be removed?
  if (it != link2cow_.end())
  {
    it->second->m_enabled = false;
    return true;
  }
  return false;
}

void BulletDiscreteBVHManager::setCollisionObjectsTransform(const std::string& name, const Eigen::Isometry3d& pose)
{
  // TODO: Find a way to remove this check. Need to store information in Tesseract EnvState indicating transforms with
  // geometry
  auto it = link2cow_.find(name);
  if (it != link2cow_.end())
  {
    COWPtr& cow = it->second;
    cow->setWorldTransform(convertEigenToBt(pose));

    // Now update Broadphase AABB (Copied from BulletWorld updateSingleAabb function)
    btVector3 minAabb, maxAabb;
    cow->getCollisionShape()->getAabb(cow->getWorldTransform(), minAabb, maxAabb);
    // need to increase the aabb for contact thresholds
    btVector3 contactThreshold(cow->getContactProcessingThreshold(),
                               cow->getContactProcessingThreshold(),
                               cow->getContactProcessingThreshold());
    minAabb -= contactThreshold;
    maxAabb += contactThreshold;

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

void BulletDiscreteBVHManager::setCollisionObjectsTransform(const std::vector<std::string>& names,
                                                            const VectorIsometry3d& poses)
{
  assert(names.size() == poses.size());
  for (auto i = 0u; i < names.size(); ++i)
    setCollisionObjectsTransform(names[i], poses[i]);
}

void BulletDiscreteBVHManager::setCollisionObjectsTransform(const TransformMap& transforms)
{
  for (const auto& transform : transforms)
    setCollisionObjectsTransform(transform.first, transform.second);
}

void BulletDiscreteBVHManager::setContactRequest(const ContactRequest& req)
{
  request_ = req;

  // Now need to update the broadphase with correct aabb
  for (auto& co : link2cow_)
  {
    COWPtr& cow = co.second;

    updateCollisionObjectWithRequest(request_, *cow);

    btVector3 minAabb, maxAabb;
    cow->getCollisionShape()->getAabb(cow->getWorldTransform(), minAabb, maxAabb);
    // need to increase the aabb for contact thresholds
    btVector3 contactThreshold(cow->getContactProcessingThreshold(),
                               cow->getContactProcessingThreshold(),
                               cow->getContactProcessingThreshold());
    minAabb -= contactThreshold;
    maxAabb += contactThreshold;

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

const ContactRequest& BulletDiscreteBVHManager::getContactRequest() const { return request_; }
void BulletDiscreteBVHManager::contactTest(ContactResultMap& collisions)
{
  ContactDistanceData cdata(&request_, &collisions);

  broadphase_->calculateOverlappingPairs(dispatcher_.get());

  btOverlappingPairCache* pairCache = broadphase_->getOverlappingPairCache();

  TesseractCollisionPairCallback collisionCallback(dispatch_info_, dispatcher_.get(), cdata);

  pairCache->processAllOverlappingPairs(&collisionCallback, dispatcher_.get());
}

void BulletDiscreteBVHManager::addCollisionObject(const COWPtr& cow)
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

const Link2Cow& BulletDiscreteBVHManager::getCollisionObjects() const { return link2cow_; }
void BulletDiscreteBVHManager::contactTest(const COWPtr& cow, ContactDistanceData& collisions)
{
  btVector3 aabbMin, aabbMax;
  cow->getCollisionShape()->getAabb(cow->getWorldTransform(), aabbMin, aabbMax);

  // need to increase the aabb for contact thresholds
  btVector3 contactThreshold1(
      cow->getContactProcessingThreshold(), cow->getContactProcessingThreshold(), cow->getContactProcessingThreshold());
  aabbMin -= contactThreshold1;
  aabbMax += contactThreshold1;

  DiscreteCollisionCollector cc(collisions, cow, cow->getContactProcessingThreshold());

  TesseractSingleContactCallback contactCB(cow.get(), dispatcher_.get(), dispatch_info_, cc);

  broadphase_->aabbTest(aabbMin, aabbMax, contactCB);
}
}
}
