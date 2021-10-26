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

#include <tesseract_collision/bullet/bullet_discrete_bvh_manager.h>

extern btScalar gDbvtMargin;  // NOLINT

namespace tesseract_collision::tesseract_collision_bullet
{
static const CollisionShapesConst EMPTY_COLLISION_SHAPES_CONST;
static const tesseract_common::VectorIsometry3d EMPTY_COLLISION_SHAPES_TRANSFORMS;

BulletDiscreteBVHManager::BulletDiscreteBVHManager(std::string name) : name_(std::move(name))
{
  // Bullet adds a margin of 5cm to which is an extern variable, so we set it to zero.
  gDbvtMargin = 0;

  dispatcher_ = std::make_unique<btCollisionDispatcher>(&coll_config_);

  dispatcher_->registerCollisionCreateFunc(
      BOX_SHAPE_PROXYTYPE,
      BOX_SHAPE_PROXYTYPE,
      coll_config_.getCollisionAlgorithmCreateFunc(CONVEX_SHAPE_PROXYTYPE, CONVEX_SHAPE_PROXYTYPE));

  dispatcher_->setDispatcherFlags(dispatcher_->getDispatcherFlags() &
                                  ~btCollisionDispatcher::CD_USE_RELATIVE_CONTACT_BREAKING_THRESHOLD);

  broadphase_ = std::make_unique<btDbvtBroadphase>();
  broadphase_->getOverlappingPairCache()->setOverlapFilterCallback(&broadphase_overlap_cb_);

  contact_test_data_.collision_margin_data = CollisionMarginData(0);
}

BulletDiscreteBVHManager::~BulletDiscreteBVHManager()
{
  // clean up remaining objects
  for (auto& co : link2cow_)
    removeCollisionObjectFromBroadphase(co.second, broadphase_, dispatcher_);
}

std::string BulletDiscreteBVHManager::getName() const { return name_; }

DiscreteContactManager::UPtr BulletDiscreteBVHManager::clone() const
{
  auto manager = std::make_unique<BulletDiscreteBVHManager>();

  auto margin = static_cast<btScalar>(contact_test_data_.collision_margin_data.getMaxCollisionMargin());

  for (const auto& cow : link2cow_)
  {
    COW::Ptr new_cow = cow.second->clone();

    assert(new_cow->getCollisionShape());
    assert(new_cow->getCollisionShape()->getShapeType() != CUSTOM_CONVEX_SHAPE_TYPE);

    new_cow->setWorldTransform(cow.second->getWorldTransform());
    new_cow->setContactProcessingThreshold(margin);

    manager->addCollisionObject(new_cow);
  }

  manager->setActiveCollisionObjects(active_);
  manager->setCollisionMarginData(contact_test_data_.collision_margin_data);
  manager->setIsContactAllowedFn(contact_test_data_.fn);

  return manager;
}

bool BulletDiscreteBVHManager::addCollisionObject(const std::string& name,
                                                  const int& mask_id,
                                                  const CollisionShapesConst& shapes,
                                                  const tesseract_common::VectorIsometry3d& shape_poses,
                                                  bool enabled)
{
  if (link2cow_.find(name) != link2cow_.end())
    removeCollisionObject(name);

  COW::Ptr new_cow = createCollisionObject(name, mask_id, shapes, shape_poses, enabled);
  if (new_cow != nullptr)
  {
    auto margin = static_cast<btScalar>(contact_test_data_.collision_margin_data.getMaxCollisionMargin());
    new_cow->setContactProcessingThreshold(margin);
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
    collision_objects_.erase(std::find(collision_objects_.begin(), collision_objects_.end(), name));
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

    // Need to clean the proxy from broadphase cache so BroadPhaseFilter gets called again.
    // The BroadPhaseFilter only gets called once, so if you change when two objects can be in collision, like filters
    // this must be called or contacts between shapes will be missed.
    broadphase_->getOverlappingPairCache()->cleanProxyFromPairs(it->second->getBroadphaseHandle(), dispatcher_.get());
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

    // Need to clean the proxy from broadphase cache so BroadPhaseFilter gets called again.
    // The BroadPhaseFilter only gets called once, so if you change when two objects can be in collision, like filters
    // this must be called or contacts between shapes will be missed.
    broadphase_->getOverlappingPairCache()->cleanProxyFromPairs(it->second->getBroadphaseHandle(), dispatcher_.get());
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
  for (auto i = 0U; i < names.size(); ++i)
    setCollisionObjectsTransform(names[i], poses[i]);
}

void BulletDiscreteBVHManager::setCollisionObjectsTransform(const tesseract_common::TransformMap& transforms)
{
  for (const auto& transform : transforms)
    setCollisionObjectsTransform(transform.first, transform.second);
}

const std::vector<std::string>& BulletDiscreteBVHManager::getCollisionObjects() const { return collision_objects_; }

void BulletDiscreteBVHManager::setActiveCollisionObjects(const std::vector<std::string>& names)
{
  active_ = names;
  contact_test_data_.active = &active_;

  // Now need to update the broadphase with correct aabb
  for (auto& co : link2cow_)
  {
    COW::Ptr& cow = co.second;
    updateCollisionObjectFilters(active_, cow, broadphase_, dispatcher_);
    refreshBroadphaseProxy(cow, broadphase_, dispatcher_);
  }
}

const std::vector<std::string>& BulletDiscreteBVHManager::getActiveCollisionObjects() const { return active_; }
void BulletDiscreteBVHManager::setCollisionMarginData(CollisionMarginData collision_margin_data,
                                                      CollisionMarginOverrideType override_type)
{
  contact_test_data_.collision_margin_data.apply(collision_margin_data, override_type);
  onCollisionMarginDataChanged();
}

void BulletDiscreteBVHManager::setDefaultCollisionMarginData(double default_collision_margin)
{
  contact_test_data_.collision_margin_data.setDefaultCollisionMargin(default_collision_margin);
  onCollisionMarginDataChanged();
}

void BulletDiscreteBVHManager::setPairCollisionMarginData(const std::string& name1,
                                                          const std::string& name2,
                                                          double collision_margin)
{
  contact_test_data_.collision_margin_data.setPairCollisionMargin(name1, name2, collision_margin);
  onCollisionMarginDataChanged();
}

const CollisionMarginData& BulletDiscreteBVHManager::getCollisionMarginData() const
{
  return contact_test_data_.collision_margin_data;
}
void BulletDiscreteBVHManager::setIsContactAllowedFn(IsContactAllowedFn fn) { contact_test_data_.fn = fn; }
IsContactAllowedFn BulletDiscreteBVHManager::getIsContactAllowedFn() const { return contact_test_data_.fn; }
void BulletDiscreteBVHManager::contactTest(ContactResultMap& collisions, const ContactRequest& request)
{
  contact_test_data_.res = &collisions;
  contact_test_data_.req = request;
  contact_test_data_.done = false;

  btOverlappingPairCache* pairCache = broadphase_->getOverlappingPairCache();

  broadphase_->calculateOverlappingPairs(dispatcher_.get());

  DiscreteBroadphaseContactResultCallback cc(contact_test_data_,
                                             contact_test_data_.collision_margin_data.getMaxCollisionMargin());

  TesseractCollisionPairCallback collisionCallback(dispatch_info_, dispatcher_.get(), cc);

  pairCache->processAllOverlappingPairs(&collisionCallback, dispatcher_.get());
}

void BulletDiscreteBVHManager::addCollisionObject(const COW::Ptr& cow)
{
  cow->setUserPointer(&contact_test_data_);
  link2cow_[cow->getName()] = cow;
  collision_objects_.push_back(cow->getName());

  // Add collision object to broadphase
  addCollisionObjectToBroadphase(cow, broadphase_, dispatcher_);
}

void BulletDiscreteBVHManager::onCollisionMarginDataChanged()
{
  auto margin = static_cast<btScalar>(contact_test_data_.collision_margin_data.getMaxCollisionMargin());
  for (auto& co : link2cow_)
  {
    COW::Ptr& cow = co.second;
    cow->setContactProcessingThreshold(margin);
    assert(cow->getBroadphaseHandle() != nullptr);
    updateBroadphaseAABB(cow, broadphase_, dispatcher_);
  }
}
}  // namespace tesseract_collision::tesseract_collision_bullet
