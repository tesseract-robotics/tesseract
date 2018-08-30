/**
 * @file bullet_cast_manager.cpp
 * @brief Tesseract ROS Bullet Cast(continuous) Manager implementation.
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

#include "tesseract_collision/bullet/bullet_cast_managers.h"

namespace tesseract
{
COWPtr makeCastCollisionObject(const COWPtr& cow)
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
    btCompoundShape* new_compound = new btCompoundShape(/*dynamicAABBtree=*/false);

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
////////////////////////////////////////////////
/////// BulletCastManagerSimple ////////////
////////////////////////////////////////////////
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
  bool removed = false;
  auto it = link2cow_.find(name);
  if (it != link2cow_.end())
  {
    cows_.erase(std::find(cows_.begin(), cows_.end(), it->second));
    link2cow_.erase(name);
    removed = true;
  }

  auto it2 = link2castcow_.find(name);
  if (it2 != link2castcow_.end())
  {
    cows_.erase(std::find(cows_.begin(), cows_.end(), it2->second));
    link2castcow_.erase(name);
    removed = true;
  }

  return removed;
}

bool BulletCastSimpleManager::enableCollisionObject(const std::string& name)
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

bool BulletCastSimpleManager::disableCollisionObject(const std::string& name)
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

void BulletCastSimpleManager::setCollisionObjectsTransform(const std::string& name, const Eigen::Isometry3d& pose)
{
  // TODO: Find a way to remove this check. Need to store information in Tesseract EnvState indicating transforms with
  // geometry
  auto it = link2cow_.find(name);
  assert(it->second->getCollisionShape()->getShapeType() != CUSTOM_CONVEX_SHAPE_TYPE);
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
    assert(it->second->getCollisionShape()->getShapeType() == CUSTOM_CONVEX_SHAPE_TYPE);

    btTransform tf1 = convertEigenToBt(pose1);
    btTransform tf2 = convertEigenToBt(pose2);

    static_cast<tesseract::CastHullShape*>(it->second->getCollisionShape())->updateCastTransform(tf1.inverseTimes(tf2));
    it->second->setWorldTransform(tf1);
  }
}

void BulletCastSimpleManager::setCollisionObjectsTransform(const std::vector<std::string>& names,
                                                           const VectorIsometry3d& pose1,
                                                           const VectorIsometry3d& pose2)
{
  assert(names.size() == pose1.size() == pose2.size());
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

void BulletCastSimpleManager::setContactRequest(const ContactRequest& req)
{
  request_ = req;
  cows_.clear();
  cows_.reserve(link2cow_.size());

  // Now need to update the broadphase with correct aabb
  for (auto& co : link2cow_)
  {
    COWPtr& cow = co.second;

    // Need to check if a collision object is still active
    if (cow->m_collisionFilterGroup == btBroadphaseProxy::KinematicFilter)
    {
      // Update with request
      updateCollisionObjectWithRequest(request_, *cow);

      bool still_active = (std::find_if(req.link_names.begin(), req.link_names.end(), [&](std::string link) {
                             return link == cow->getName();
                           }) == req.link_names.end());

      if (still_active)
      {
        // Get the existing active collision object
        COWPtr active_cow = link2castcow_[cow->getName()];

        // Update with request
        updateCollisionObjectWithRequest(request_, *active_cow);

        // Add to collision object vector
        cows_.insert(cows_.begin(), active_cow);
      }
      else
      {
        // Remove the collision object from active map
        link2castcow_.erase(cow->getName());

        // Add to collision object vector
        cows_.push_back(cow);
      }
    }
    else
    {
      // Update with request
      updateCollisionObjectWithRequest(request_, *cow);

      bool now_active = (std::find_if(req.link_names.begin(), req.link_names.end(), [&](std::string link) {
                           return link == cow->getName();
                         }) == req.link_names.end());
      if (now_active)
      {
        // Create active collision object
        COWPtr active_cow = makeCastCollisionObject(cow);

        // Update with request
        updateCollisionObjectWithRequest(request_, *active_cow);

        // Add it to the active map
        link2castcow_[active_cow->getName()] = active_cow;

        // Add to collision object vector
        cows_.insert(cows_.begin(), active_cow);
      }
      else
      {
        // Add to collision object vector
        cows_.push_back(cow);
      }
    }
  }
}

const ContactRequest& BulletCastSimpleManager::getContactRequest() const { return request_; }
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
        bool needs_collision = needsCollisionCheck(*cow1, *cow2, cdata.req->isContactAllowed, false);

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

void BulletCastSimpleManager::addCollisionObject(COWPtr& cow)
{
  link2cow_[cow->getName()] = cow;

  if (cow->m_collisionFilterGroup == btBroadphaseProxy::KinematicFilter)
    cows_.insert(cows_.begin(), cow);
  else
    cows_.push_back(cow);
}

////////////////////////////////////////////////
////////// BulletCastBVHManager ////////////
////////////////////////////////////////////////

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
  {
    assert(it->second->getCollisionShape()->getShapeType() != CUSTOM_CONVEX_SHAPE_TYPE);
    it->second->setWorldTransform(convertEigenToBt(pose));
  }
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
    assert(cow->getCollisionShape()->getShapeType() == CUSTOM_CONVEX_SHAPE_TYPE);

    btTransform tf1 = convertEigenToBt(pose1);
    btTransform tf2 = convertEigenToBt(pose2);

    static_cast<tesseract::CastHullShape*>(cow->getCollisionShape())->updateCastTransform(tf1.inverseTimes(tf2));
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

      bool still_active = (std::find_if(req.link_names.begin(), req.link_names.end(), [&](std::string link) {
                             return link == cow->getName();
                           }) == req.link_names.end());

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

      bool now_active = (std::find_if(req.link_names.begin(), req.link_names.end(), [&](std::string link) {
                           return link == cow->getName();
                         }) == req.link_names.end());
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
void BulletCastBVHManager::addCollisionObject(COWPtr& cow)
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
