/**
 * @file bullet_discrete_bvh_manager.cpp
 * @brief Tesseract ROS Bullet Discrete BVH Manager implementation.
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

#include "tesseract_collision/bullet/bullet_discrete_bvh_manager.h"

namespace tesseract_collision
{
namespace tesseract_collision_bullet
{
static const CollisionShapesConst EMPTY_COLLISION_SHAPES_CONST;
static const tesseract_common::VectorIsometry3d EMPTY_COLLISION_SHAPES_TRANSFORMS;

BulletDiscreteBVHManager::BulletDiscreteBVHManager()
{
  dispatcher_ = std::make_unique<btCollisionDispatcher>(&coll_config_);

  dispatcher_->registerCollisionCreateFunc(
      BOX_SHAPE_PROXYTYPE,
      BOX_SHAPE_PROXYTYPE,
      coll_config_.getCollisionAlgorithmCreateFunc(CONVEX_SHAPE_PROXYTYPE, CONVEX_SHAPE_PROXYTYPE));

  dispatcher_->setDispatcherFlags(dispatcher_->getDispatcherFlags() &
                                  ~btCollisionDispatcher::CD_USE_RELATIVE_CONTACT_BREAKING_THRESHOLD);

  broadphase_ = std::make_unique<btDbvtBroadphase>();

  contact_distance_ = 0;
}

BulletDiscreteBVHManager::~BulletDiscreteBVHManager()
{
  // clean up remaining objects
  for (auto& co : link2cow_)
    removeCollisionObjectFromBroadphase(co.second, broadphase_, dispatcher_);
}

DiscreteContactManager::Ptr BulletDiscreteBVHManager::clone() const
{
  BulletDiscreteBVHManager::Ptr manager(new BulletDiscreteBVHManager());

  for (const auto& cow : link2cow_)
  {
    COW::Ptr new_cow = cow.second->clone();

    assert(new_cow->getCollisionShape());
    assert(new_cow->getCollisionShape()->getShapeType() != CUSTOM_CONVEX_SHAPE_TYPE);

    new_cow->setWorldTransform(cow.second->getWorldTransform());

    new_cow->setContactProcessingThreshold(static_cast<btScalar>(contact_distance_));
    manager->addCollisionObject(new_cow);
  }

  manager->setActiveCollisionObjects(active_);
  manager->setContactDistanceThreshold(contact_distance_);
  manager->setIsContactAllowedFn(fn_);

  return std::move(manager);
}

bool BulletDiscreteBVHManager::addCollisionObject(const std::string& name,
                                                  const int& mask_id,
                                                  const CollisionShapesConst& shapes,
                                                  const tesseract_common::VectorIsometry3d& shape_poses,
                                                  bool enabled)
{
  COW::Ptr new_cow = createCollisionObject(name, mask_id, shapes, shape_poses, enabled);
  if (new_cow != nullptr)
  {
    addCollisionObject(new_cow);
    return true;
  }

  return false;
}

const CollisionShapesConst& BulletDiscreteBVHManager::getCollisionObjectGeometries(const std::string& name) const
{
  auto cow = link2cow_.find(name);
  return (link2cow_.find(name) != link2cow_.end()) ? cow->second->getCollisionGeometries() :
                                                     EMPTY_COLLISION_SHAPES_CONST;
}

const tesseract_common::VectorIsometry3d&
BulletDiscreteBVHManager::getCollisionObjectGeometriesTransforms(const std::string& name) const
{
  auto cow = link2cow_.find(name);
  return (link2cow_.find(name) != link2cow_.end()) ? cow->second->getCollisionGeometriesTransforms() :
                                                     EMPTY_COLLISION_SHAPES_TRANSFORMS;
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
    removeCollisionObjectFromBroadphase(it->second, broadphase_, dispatcher_);
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
    COW::Ptr& cow = it->second;
    cow->setWorldTransform(convertEigenToBt(pose));

    // Update Collision Object Broadphase AABB
    updateBroadphaseAABB(cow, broadphase_, dispatcher_);
  }
}

void BulletDiscreteBVHManager::setCollisionObjectsTransform(const std::vector<std::string>& names,
                                                            const tesseract_common::VectorIsometry3d& poses)
{
  assert(names.size() == poses.size());
  for (auto i = 0u; i < names.size(); ++i)
    setCollisionObjectsTransform(names[i], poses[i]);
}

void BulletDiscreteBVHManager::setCollisionObjectsTransform(const tesseract_common::TransformMap& transforms)
{
  for (const auto& transform : transforms)
    setCollisionObjectsTransform(transform.first, transform.second);
}

void BulletDiscreteBVHManager::setActiveCollisionObjects(const std::vector<std::string>& names)
{
  active_ = names;

  // Now need to update the broadphase with correct aabb
  for (auto& co : link2cow_)
  {
    COW::Ptr& cow = co.second;

    updateCollisionObjectFilters(active_, *cow);
  }
}

const std::vector<std::string>& BulletDiscreteBVHManager::getActiveCollisionObjects() const { return active_; }
void BulletDiscreteBVHManager::setContactDistanceThreshold(double contact_distance)
{
  contact_distance_ = contact_distance;

  for (auto& co : link2cow_)
  {
    COW::Ptr& cow = co.second;
    cow->setContactProcessingThreshold(static_cast<btScalar>(contact_distance));
    assert(cow->getBroadphaseHandle() != nullptr);
    updateBroadphaseAABB(cow, broadphase_, dispatcher_);
  }
}

double BulletDiscreteBVHManager::getContactDistanceThreshold() const { return contact_distance_; }
void BulletDiscreteBVHManager::setIsContactAllowedFn(IsContactAllowedFn fn) { fn_ = fn; }
IsContactAllowedFn BulletDiscreteBVHManager::getIsContactAllowedFn() const { return fn_; }
void BulletDiscreteBVHManager::contactTest(ContactResultMap& collisions, const ContactTestType& type)
{
  ContactTestData cdata(active_, contact_distance_, fn_, type, collisions);

  broadphase_->calculateOverlappingPairs(dispatcher_.get());

  btOverlappingPairCache* pairCache = broadphase_->getOverlappingPairCache();

  DiscreteBroadphaseContactResultCallback cc(cdata, contact_distance_);

  TesseractCollisionPairCallback collisionCallback(dispatch_info_, dispatcher_.get(), cc);

  pairCache->processAllOverlappingPairs(&collisionCallback, dispatcher_.get());
}

void BulletDiscreteBVHManager::addCollisionObject(const COW::Ptr& cow)
{
  link2cow_[cow->getName()] = cow;

  // Add collision object to broadphase
  addCollisionObjectToBroadphase(cow, broadphase_, dispatcher_);
}

const Link2Cow& BulletDiscreteBVHManager::getCollisionObjects() const { return link2cow_; }
void BulletDiscreteBVHManager::contactTest(const COW::Ptr& cow, ContactTestData& collisions)
{
  btVector3 min_aabb, max_aabb;
  cow->getAABB(min_aabb, max_aabb);

  DiscreteCollisionCollector cc(collisions, cow, cow->getContactProcessingThreshold());

  TesseractSingleContactCallback contactCB(cow.get(), dispatcher_.get(), dispatch_info_, cc);

  broadphase_->aabbTest(min_aabb, max_aabb, contactCB);
}
}  // namespace tesseract_collision_bullet
}  // namespace tesseract_collision
