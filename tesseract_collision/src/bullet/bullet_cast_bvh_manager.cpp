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

extern btScalar gDbvtMargin;

namespace tesseract_collision
{
namespace tesseract_collision_bullet
{
static const CollisionShapesConst EMPTY_COLLISION_SHAPES_CONST;
static const tesseract_common::VectorIsometry3d EMPTY_COLLISION_SHAPES_TRANSFORMS;

BulletCastBVHManager::BulletCastBVHManager()
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

BulletCastBVHManager::~BulletCastBVHManager()
{
  // clean up remaining objects
  for (auto& co : link2cow_)
    removeCollisionObjectFromBroadphase(co.second, broadphase_, dispatcher_);

  // clean up remaining objects
  for (auto& co : link2castcow_)
    removeCollisionObjectFromBroadphase(co.second, broadphase_, dispatcher_);
}

ContinuousContactManager::Ptr BulletCastBVHManager::clone() const
{
  auto manager = std::make_shared<BulletCastBVHManager>();

  btScalar margin = static_cast<btScalar>(contact_test_data_.collision_margin_data.getMaxCollisionMargin());

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

bool BulletCastBVHManager::addCollisionObject(const std::string& name,
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
    btScalar margin = static_cast<btScalar>(contact_test_data_.collision_margin_data.getMaxCollisionMargin());
    new_cow->setContactProcessingThreshold(margin);
    addCollisionObject(new_cow);
    return true;
  }

  return false;
}

const CollisionShapesConst& BulletCastBVHManager::getCollisionObjectGeometries(const std::string& name) const
{
  auto cow = link2cow_.find(name);
  return (link2cow_.find(name) != link2cow_.end()) ? cow->second->getCollisionGeometries() :
                                                     EMPTY_COLLISION_SHAPES_CONST;
}

const tesseract_common::VectorIsometry3d&
BulletCastBVHManager::getCollisionObjectGeometriesTransforms(const std::string& name) const
{
  auto cow = link2cow_.find(name);
  return (link2cow_.find(name) != link2cow_.end()) ? cow->second->getCollisionGeometriesTransforms() :
                                                     EMPTY_COLLISION_SHAPES_TRANSFORMS;
}

bool BulletCastBVHManager::hasCollisionObject(const std::string& name) const
{
  return (link2cow_.find(name) != link2cow_.end());
}

bool BulletCastBVHManager::removeCollisionObject(const std::string& name)
{
  auto it = link2cow_.find(name);
  if (it != link2cow_.end())
  {
    COW::Ptr& cow1 = it->second;
    collision_objects_.erase(std::find(collision_objects_.begin(), collision_objects_.end(), name));
    removeCollisionObjectFromBroadphase(cow1, broadphase_, dispatcher_);
    link2cow_.erase(name);

    COW::Ptr& cow2 = link2castcow_[name];
    removeCollisionObjectFromBroadphase(cow2, broadphase_, dispatcher_);
    link2castcow_.erase(name);

    return true;
  }

  return false;
}

bool BulletCastBVHManager::enableCollisionObject(const std::string& name)
{
  auto it = link2cow_.find(name);
  if (it != link2cow_.end())
  {
    it->second->m_enabled = true;

    // Need to clean the proxy from broadphase cache so BroadPhaseFilter gets called again.
    // The BroadPhaseFilter only gets called once, so if you change when two objects can be in collision, like filters
    // this must be called or contacts between shapes will be missed.
    if (it->second->getBroadphaseHandle())
      broadphase_->getOverlappingPairCache()->cleanProxyFromPairs(it->second->getBroadphaseHandle(), dispatcher_.get());

    auto cast_cow = link2castcow_[name];
    cast_cow->m_enabled = true;

    // Need to clean the proxy from broadphase cache so BroadPhaseFilter gets called again.
    // The BroadPhaseFilter only gets called once, so if you change when two objects can be in collision, like filters
    // this must be called or contacts between shapes will be missed.
    if (cast_cow->getBroadphaseHandle())
      broadphase_->getOverlappingPairCache()->cleanProxyFromPairs(cast_cow->getBroadphaseHandle(), dispatcher_.get());

    return true;
  }

  return false;
}

bool BulletCastBVHManager::disableCollisionObject(const std::string& name)
{
  auto it = link2cow_.find(name);
  if (it != link2cow_.end())
  {
    it->second->m_enabled = false;

    // Need to clean the proxy from broadphase cache so BroadPhaseFilter gets called again.
    // The BroadPhaseFilter only gets called once, so if you change when two objects can be in collision, like filters
    // this must be called or contacts between shapes will be missed.
    if (it->second->getBroadphaseHandle())
      broadphase_->getOverlappingPairCache()->cleanProxyFromPairs(it->second->getBroadphaseHandle(), dispatcher_.get());

    auto cast_cow = link2castcow_[name];
    cast_cow->m_enabled = false;

    // Need to clean the proxy from broadphase cache so BroadPhaseFilter gets called again.
    // The BroadPhaseFilter only gets called once, so if you change when two objects can be in collision, like filters
    // this must be called or contacts between shapes will be missed.
    if (cast_cow->getBroadphaseHandle())
      broadphase_->getOverlappingPairCache()->cleanProxyFromPairs(cast_cow->getBroadphaseHandle(), dispatcher_.get());

    return true;
  }

  return false;
}

void BulletCastBVHManager::setCollisionObjectsTransform(const std::string& name, const Eigen::Isometry3d& pose)
{
  // TODO: Find a way to remove this check. Need to store information in Tesseract EnvState indicating transforms with
  // geometry
  auto it = link2cow_.find(name);
  if (it != link2cow_.end())
  {
    COW::Ptr& cow = it->second;
    btTransform tf = convertEigenToBt(pose);
    cow->setWorldTransform(tf);
    link2castcow_[name]->setWorldTransform(tf);

    // Now update Broadphase AABB (See BulletWorld updateSingleAabb function)
    if (cow->getBroadphaseHandle())
      updateBroadphaseAABB(cow, broadphase_, dispatcher_);
  }
}

void BulletCastBVHManager::setCollisionObjectsTransform(const std::vector<std::string>& names,
                                                        const tesseract_common::VectorIsometry3d& poses)
{
  assert(names.size() == poses.size());
  for (auto i = 0u; i < names.size(); ++i)
    setCollisionObjectsTransform(names[i], poses[i]);
}

void BulletCastBVHManager::setCollisionObjectsTransform(const tesseract_common::TransformMap& transforms)
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
    COW::Ptr& cow = it->second;
    assert(cow->m_collisionFilterGroup == btBroadphaseProxy::KinematicFilter);

    btTransform tf1 = convertEigenToBt(pose1);
    btTransform tf2 = convertEigenToBt(pose2);

    cow->setWorldTransform(tf1);
    link2cow_[name]->setWorldTransform(tf1);

    // If collision object is disabled dont proceed
    if (cow->m_enabled)
    {
      if (btBroadphaseProxy::isConvex(cow->getCollisionShape()->getShapeType()))
      {
        assert(dynamic_cast<CastHullShape*>(cow->getCollisionShape()) != nullptr);
        static_cast<CastHullShape*>(cow->getCollisionShape())->updateCastTransform(tf1.inverseTimes(tf2));
      }
      else if (btBroadphaseProxy::isCompound(cow->getCollisionShape()->getShapeType()))
      {
        assert(dynamic_cast<btCompoundShape*>(cow->getCollisionShape()) != nullptr);
        auto* compound = static_cast<btCompoundShape*>(cow->getCollisionShape());
        for (int i = 0; i < compound->getNumChildShapes(); ++i)
        {
          if (btBroadphaseProxy::isConvex(compound->getChildShape(i)->getShapeType()))
          {
            assert(dynamic_cast<CastHullShape*>(compound->getChildShape(i)) != nullptr);
            const btTransform& local_tf = compound->getChildTransform(i);

            btTransform delta_tf = (tf1 * local_tf).inverseTimes(tf2 * local_tf);
            static_cast<CastHullShape*>(compound->getChildShape(i))->updateCastTransform(delta_tf);
            compound->updateChildTransform(i, local_tf, false);  // This is required to update the BVH tree
          }
          else if (btBroadphaseProxy::isCompound(compound->getChildShape(i)->getShapeType()))
          {
            assert(dynamic_cast<btCompoundShape*>(compound->getChildShape(i)) != nullptr);
            auto* second_compound = static_cast<btCompoundShape*>(compound->getChildShape(i));

            for (int j = 0; j < second_compound->getNumChildShapes(); ++j)
            {
              assert(!btBroadphaseProxy::isCompound(second_compound->getChildShape(j)->getShapeType()));
              assert(dynamic_cast<CastHullShape*>(second_compound->getChildShape(j)) != nullptr);
              const btTransform& local_tf = second_compound->getChildTransform(j);

              btTransform delta_tf = (tf1 * local_tf).inverseTimes(tf2 * local_tf);
              static_cast<CastHullShape*>(second_compound->getChildShape(j))->updateCastTransform(delta_tf);
              second_compound->updateChildTransform(j, local_tf, false);  // This is required to update the BVH tree
            }
            second_compound->recalculateLocalAabb();
          }
        }
        compound->recalculateLocalAabb();
      }
      else
      {
        throw std::runtime_error("I can only continuous collision check convex shapes and compound shapes made of "
                                 "convex "
                                 "shapes");
      }

      // Now update Broadphase AABB (See BulletWorld updateSingleAabb function)
      updateBroadphaseAABB(cow, broadphase_, dispatcher_);
    }
  }
}

void BulletCastBVHManager::setCollisionObjectsTransform(const std::vector<std::string>& names,
                                                        const tesseract_common::VectorIsometry3d& pose1,
                                                        const tesseract_common::VectorIsometry3d& pose2)
{
  assert(names.size() == pose1.size());
  assert(names.size() == pose2.size());
  for (auto i = 0u; i < names.size(); ++i)
    setCollisionObjectsTransform(names[i], pose1[i], pose2[i]);
}

void BulletCastBVHManager::setCollisionObjectsTransform(const tesseract_common::TransformMap& pose1,
                                                        const tesseract_common::TransformMap& pose2)
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

const std::vector<std::string>& BulletCastBVHManager::getCollisionObjects() const { return collision_objects_; }

void BulletCastBVHManager::setActiveCollisionObjects(const std::vector<std::string>& names)
{
  active_ = names;
  contact_test_data_.active = &active_;

  // Now need to update the broadphase with correct aabb
  for (auto& co : link2cow_)
  {
    COW::Ptr& cow = co.second;

    // Need to check if a collision object is still active
    if (cow->m_collisionFilterGroup == btBroadphaseProxy::KinematicFilter)
    {
      // Update with active
      updateCollisionObjectFilters(active_, cow, broadphase_, dispatcher_);

      // Get the active collision object
      COW::Ptr& active_cow = link2castcow_[cow->getName()];

      // Update with active
      updateCollisionObjectFilters(active_, active_cow, broadphase_, dispatcher_);

      // Check if the link is still active.
      if (!isLinkActive(active_, cow->getName()))
      {
        // Remove the active collision object from the broadphase
        removeCollisionObjectFromBroadphase(active_cow, broadphase_, dispatcher_);

        // Add the active collision object to the broadphase
        addCollisionObjectToBroadphase(cow, broadphase_, dispatcher_);
      }
    }
    else
    {
      // Update with active
      updateCollisionObjectFilters(active_, cow, broadphase_, dispatcher_);

      // Get the active collision object
      COW::Ptr& active_cow = link2castcow_[cow->getName()];

      // Update with active
      updateCollisionObjectFilters(active_, active_cow, broadphase_, dispatcher_);

      // Check if link is now active
      if (isLinkActive(active_, cow->getName()))
      {
        // Remove the static collision object from the broadphase
        removeCollisionObjectFromBroadphase(cow, broadphase_, dispatcher_);

        // Add the active collision object to the broadphase
        addCollisionObjectToBroadphase(active_cow, broadphase_, dispatcher_);
      }
    }
  }
}

const std::vector<std::string>& BulletCastBVHManager::getActiveCollisionObjects() const { return active_; }

void BulletCastBVHManager::setCollisionMarginData(CollisionMarginData collision_margin_data,
                                                  CollisionMarginOverrideType override_type)
{
  contact_test_data_.collision_margin_data.apply(collision_margin_data, override_type);
  onCollisionMarginDataChanged();
}

void BulletCastBVHManager::setDefaultCollisionMarginData(double default_collision_margin)
{
  contact_test_data_.collision_margin_data.setDefaultCollisionMargin(default_collision_margin);
  onCollisionMarginDataChanged();
}

void BulletCastBVHManager::setPairCollisionMarginData(const std::string& name1,
                                                      const std::string& name2,
                                                      double collision_margin)
{
  contact_test_data_.collision_margin_data.setPairCollisionMargin(name1, name2, collision_margin);
  onCollisionMarginDataChanged();
}

const CollisionMarginData& BulletCastBVHManager::getCollisionMarginData() const
{
  return contact_test_data_.collision_margin_data;
}
void BulletCastBVHManager::setIsContactAllowedFn(IsContactAllowedFn fn) { contact_test_data_.fn = fn; }
IsContactAllowedFn BulletCastBVHManager::getIsContactAllowedFn() const { return contact_test_data_.fn; }
void BulletCastBVHManager::contactTest(ContactResultMap& collisions, const ContactRequest& request)
{
  contact_test_data_.res = &collisions;
  contact_test_data_.req = request;
  contact_test_data_.done = false;

  broadphase_->calculateOverlappingPairs(dispatcher_.get());

  btOverlappingPairCache* pairCache = broadphase_->getOverlappingPairCache();

  CastBroadphaseContactResultCallback cc(contact_test_data_,
                                         contact_test_data_.collision_margin_data.getMaxCollisionMargin());

  TesseractCollisionPairCallback collisionCallback(dispatch_info_, dispatcher_.get(), cc);

  pairCache->processAllOverlappingPairs(&collisionCallback, dispatcher_.get());
}

void BulletCastBVHManager::addCollisionObject(COW::Ptr cow)
{
  cow->setUserPointer(&contact_test_data_);
  link2cow_[cow->getName()] = cow;
  collision_objects_.push_back(cow->getName());

  // Create cast collision object
  COW::Ptr cast_cow = makeCastCollisionObject(cow);
  cast_cow->setUserPointer(&contact_test_data_);

  // Add it to the cast map
  link2castcow_[cast_cow->getName()] = cast_cow;

  const COW::Ptr& selected_cow = (cow->m_collisionFilterGroup == btBroadphaseProxy::KinematicFilter) ? cast_cow : cow;

  btVector3 aabb_min, aabb_max;
  selected_cow->getAABB(aabb_min, aabb_max);

  int type = selected_cow->getCollisionShape()->getShapeType();
  selected_cow->setBroadphaseHandle(broadphase_->createProxy(aabb_min,
                                                             aabb_max,
                                                             type,
                                                             selected_cow.get(),
                                                             selected_cow->m_collisionFilterGroup,
                                                             selected_cow->m_collisionFilterMask,
                                                             dispatcher_.get()));
}

void BulletCastBVHManager::onCollisionMarginDataChanged()
{
  btScalar margin = static_cast<btScalar>(contact_test_data_.collision_margin_data.getMaxCollisionMargin());
  for (auto& co : link2cow_)
  {
    COW::Ptr& cow = co.second;
    cow->setContactProcessingThreshold(margin);
    if (cow->getBroadphaseHandle())
      updateBroadphaseAABB(cow, broadphase_, dispatcher_);
  }

  for (auto& co : link2castcow_)
  {
    COW::Ptr& cow = co.second;
    cow->setContactProcessingThreshold(margin);
    if (cow->getBroadphaseHandle())
      updateBroadphaseAABB(cow, broadphase_, dispatcher_);
  }
}

}  // namespace tesseract_collision_bullet
}  // namespace tesseract_collision
