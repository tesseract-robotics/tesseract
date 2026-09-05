/**
 * @file fcl_discrete_managers.cpp
 * @brief Tesseract ROS FCL contact checker implementation.
 *
 * @author Levi Armstrong
 * @date Dec 18, 2017
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (BSD)
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

#include <algorithm>
#include <cassert>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <tesseract/collision/fcl/fcl_discrete_managers.h>
#include <tesseract/common/contact_allowed_validator.h>
#include <tesseract/common/utils.h>

using namespace tesseract::collision::fcl_internal;

namespace tesseract::collision
{
static const CollisionShapesConst EMPTY_COLLISION_SHAPES_CONST;
static const tesseract::common::VectorIsometry3d EMPTY_COLLISION_SHAPES_TRANSFORMS;

FCLDiscreteBVHManager::FCLDiscreteBVHManager(std::string name) : name_(std::move(name))
{
  static_manager_ = std::make_unique<fcl::DynamicAABBTreeCollisionManagerd>();
  dynamic_manager_ = std::make_unique<fcl::DynamicAABBTreeCollisionManagerd>();
  collision_margin_data_ = CollisionMarginData(0);
}

std::string FCLDiscreteBVHManager::getName() const { return name_; }

DiscreteContactManager::UPtr FCLDiscreteBVHManager::clone() const
{
  auto manager = std::make_unique<FCLDiscreteBVHManager>();

  std::vector<COW::Ptr> cows;
  cows.reserve(collision_objects_.size());
  for (const auto& id : collision_objects_)
    cows.push_back(link2cow_.at(id)->clone());

  // The refit inside addCollisionObjects must not be deferred to the setActiveCollisionObjects below, which is why
  // this backend offers no way to defer it. Deferring changes the broadphase tree, so the pair reaches the
  // narrowphase in the opposite order, and fcl::distance is not order-symmetric: the GJK witness mirrors instead of
  // negating. The clone then reports the two small off-axis components of the contact normal with the sign opposite
  // to its source, 1.7e-3 and 3.0e-3 apart against the clone suite's 1e-3 tolerance. The distance, the nearest
  // points and the dominant normal component still agree, as does the order getCollisionObjects() reports.
  manager->addCollisionObjects(cows);

  manager->setActiveCollisionObjects(active_);
  manager->setCollisionMarginData(collision_margin_data_);
  manager->setContactAllowedValidator(validator_);

  return manager;
}

bool FCLDiscreteBVHManager::addCollisionObject(const tesseract::common::LinkId& id,
                                               const int& mask_id,
                                               const CollisionShapesConst& shapes,
                                               const tesseract::common::VectorIsometry3d& shape_poses,
                                               bool enabled)
{
  if (link2cow_.find(id) != link2cow_.end())
    removeCollisionObject(id);

  COW::Ptr new_cow = createFCLCollisionObject(id, mask_id, shapes, shape_poses, enabled);
  if (new_cow != nullptr)
  {
    addCollisionObject(new_cow);
    return true;
  }

  return false;
}

bool FCLDiscreteBVHManager::addCollisionObjects(const std::vector<CollisionObjectSpec>& objects)
{
  std::vector<COW::Ptr> cows;
  cows.reserve(objects.size());

  // Reproduce the per-id semantics of adding the specs one at a time: a repeated id displaces its earlier entry,
  // so the last spec naming an id decides both the wrapper and its position.
  std::unordered_map<tesseract::common::LinkId, std::size_t> batch_index;
  batch_index.reserve(objects.size());

  bool success{ true };
  for (const auto& obj : objects)
  {
    // The hole is compacted away below rather than erased here, which would be O(n) per repeat.
    const auto it = batch_index.find(obj.id);
    if (it != batch_index.end())
    {
      cows[it->second] = nullptr;
      batch_index.erase(it);
    }

    const COW::Ptr new_cow = createFCLCollisionObject(obj.id, obj.mask_id, obj.shapes, obj.shape_poses, obj.enabled);
    if (new_cow == nullptr)
    {
      success = false;
      continue;
    }

    batch_index[obj.id] = cows.size();
    cows.push_back(new_cow);
  }

  cows.erase(std::remove(cows.begin(), cows.end(), nullptr), cows.end());

  // The primitive does not displace an already-registered object, so do here what the single-object entry point
  // does. Skipping this orphans the old object's broadphase proxy. Every id the batch names is removed, including
  // one whose spec failed to build: the single-object form removes before it creates, so a failed spec leaves that
  // id unregistered. Bulk removal skips the ids it does not hold, so the batch is passed whole and its return,
  // which reports only those absences, is not the return of this call.
  std::vector<tesseract::common::LinkId> displaced;
  displaced.reserve(objects.size());
  for (const auto& obj : objects)
    displaced.push_back(obj.id);

  removeCollisionObjects(displaced);

  if (!cows.empty())
    addCollisionObjects(cows);

  return success;
}

const CollisionShapesConst&
FCLDiscreteBVHManager::getCollisionObjectGeometries(const tesseract::common::LinkId& id) const
{
  auto cow = link2cow_.find(id);
  return (cow != link2cow_.end()) ? cow->second->getCollisionGeometries() : EMPTY_COLLISION_SHAPES_CONST;
}

const tesseract::common::VectorIsometry3d&
FCLDiscreteBVHManager::getCollisionObjectGeometriesTransforms(const tesseract::common::LinkId& id) const
{
  auto cow = link2cow_.find(id);
  return (cow != link2cow_.end()) ? cow->second->getCollisionGeometriesTransforms() : EMPTY_COLLISION_SHAPES_TRANSFORMS;
}

bool FCLDiscreteBVHManager::hasCollisionObject(const tesseract::common::LinkId& id) const
{
  return (link2cow_.find(id) != link2cow_.end());
}

bool FCLDiscreteBVHManager::removeCollisionObject(const tesseract::common::LinkId& id)
{
  auto it = link2cow_.find(id);
  if (it != link2cow_.end())
  {
    std::vector<CollisionObjectPtr>& objects = it->second->getCollisionObjects();
    fcl_co_count_ -= objects.size();

    std::vector<fcl::CollisionObject<double>*> static_objs;
    static_manager_->getObjects(static_objs);

    std::vector<fcl::CollisionObject<double>*> dynamic_objs;
    dynamic_manager_->getObjects(dynamic_objs);

    // Must check if object exists in the manager before calling unregister.
    // If it does not exist and unregister is called it is undefined behavior
    for (auto& co : objects)
    {
      auto static_it = std::find(static_objs.begin(), static_objs.end(), co.get());
      if (static_it != static_objs.end())
        static_manager_->unregisterObject(co.get());

      auto dynamic_it = std::find(dynamic_objs.begin(), dynamic_objs.end(), co.get());
      if (dynamic_it != dynamic_objs.end())
        dynamic_manager_->unregisterObject(co.get());
    }

    collision_objects_.erase(std::find(collision_objects_.begin(), collision_objects_.end(), id));
    link2cow_.erase(it);
    active_.erase(id);
    return true;
  }
  return false;
}

bool FCLDiscreteBVHManager::removeCollisionObjects(const std::vector<tesseract::common::LinkId>& ids)
{
  // Materialising both trees is the expensive part, so skip it for a batch that holds none of the named ids. A
  // batch naming only links without collision geometry is normal at the environment call site.
  const bool any_held = std::any_of(ids.begin(), ids.end(), [this](const tesseract::common::LinkId& id) {
    return link2cow_.find(id) != link2cow_.end();
  });
  // An empty batch succeeds, as the per-id loop does; a non-empty batch naming only absent ids does not.
  if (!any_held)
    return ids.empty();

  // Materialise both trees once for the batch, not once per link, and index them for O(1) membership.
  std::vector<fcl::CollisionObject<double>*> static_objs;
  static_manager_->getObjects(static_objs);
  std::vector<fcl::CollisionObject<double>*> dynamic_objs;
  dynamic_manager_->getObjects(dynamic_objs);

  const std::unordered_set<fcl::CollisionObject<double>*> in_static(static_objs.begin(), static_objs.end());
  const std::unordered_set<fcl::CollisionObject<double>*> in_dynamic(dynamic_objs.begin(), dynamic_objs.end());

  std::unordered_set<tesseract::common::LinkId> removed;
  removed.reserve(ids.size());

  bool success{ true };
  for (const auto& id : ids)
  {
    auto it = link2cow_.find(id);
    if (it == link2cow_.end())
    {
      success = false;
      continue;
    }

    std::vector<CollisionObjectPtr>& objects = it->second->getCollisionObjects();
    fcl_co_count_ -= objects.size();

    // Must check membership before unregistering: unregistering an object a manager does not hold is undefined
    // behaviour. The sets are built before any unregistering and are not refreshed, which is sound because an
    // object belongs to exactly one link, so no later iteration can consult an entry this call already removed.
    for (auto& co : objects)
    {
      if (in_static.find(co.get()) != in_static.end())
        static_manager_->unregisterObject(co.get());

      if (in_dynamic.find(co.get()) != in_dynamic.end())
        dynamic_manager_->unregisterObject(co.get());
    }

    removed.insert(id);
    link2cow_.erase(it);
    active_.erase(id);
  }

  // One pass over collision_objects_, preserving the order of the survivors.
  collision_objects_.erase(
      std::remove_if(collision_objects_.begin(),
                     collision_objects_.end(),
                     [&removed](const tesseract::common::LinkId& id) { return removed.find(id) != removed.end(); }),
      collision_objects_.end());

  return success;
}

bool FCLDiscreteBVHManager::enableCollisionObject(const tesseract::common::LinkId& id)
{
  auto it = link2cow_.find(id);
  if (it != link2cow_.end())
  {
    it->second->m_enabled = true;
    return true;
  }
  return false;
}

bool FCLDiscreteBVHManager::disableCollisionObject(const tesseract::common::LinkId& id)
{
  auto it = link2cow_.find(id);
  if (it != link2cow_.end())
  {
    it->second->m_enabled = false;
    return true;
  }
  return false;
}

bool FCLDiscreteBVHManager::isCollisionObjectEnabled(const tesseract::common::LinkId& id) const
{
  auto it = link2cow_.find(id);
  if (it != link2cow_.end())
    return it->second->m_enabled;

  return false;
}

void FCLDiscreteBVHManager::setCollisionObjectsTransform(const tesseract::common::LinkId& id,
                                                         const Eigen::Isometry3d& pose)
{
  auto it = link2cow_.find(id);
  if (it != link2cow_.end())
  {
    const Eigen::Isometry3d& cur_tf = it->second->getCollisionObjectsTransform();
    // Note: If the transform has not changed do not update to prevent unnecessary re-balancing of the BVH tree
    if (!tesseract::common::almostEqualRelativeAndAbs(cur_tf, pose))
    {
      it->second->setCollisionObjectsTransform(pose);
      if (it->second->m_collisionFilterGroup == CollisionFilterGroups::StaticFilter)
      {
        // Note: Calling update causes a re-balance of the AABB tree, which is expensive
        static_manager_->update(it->second->getCollisionObjectsRaw());
      }
      else
      {
        // Note: Calling update causes a re-balance of the AABB tree, which is expensive
        dynamic_manager_->update(it->second->getCollisionObjectsRaw());
      }
    }
  }
}

Eigen::Isometry3d FCLDiscreteBVHManager::getCollisionObjectsTransform(const tesseract::common::LinkId& id) const
{
  return link2cow_.at(id)->getCollisionObjectsTransform();
}

void FCLDiscreteBVHManager::setCollisionObjectsTransform(const tesseract::common::LinkIdTransformMap& transforms)
{
  static_update_.clear();
  dynamic_update_.clear();
  for (const auto& [id, tf] : transforms)
  {
    auto it = link2cow_.find(id);
    if (it != link2cow_.end())
    {
      const Eigen::Isometry3d& cur_tf = it->second->getCollisionObjectsTransform();
      // Note: If the transform has not changed do not update to prevent unnecessary re-balancing of the BVH tree
      if (!tesseract::common::almostEqualRelativeAndAbs(cur_tf, tf))
      {
        it->second->setCollisionObjectsTransform(tf);
        std::vector<CollisionObjectRawPtr>& co = it->second->getCollisionObjectsRaw();
        if (it->second->m_collisionFilterGroup == CollisionFilterGroups::StaticFilter)
        {
          static_update_.insert(static_update_.end(), co.begin(), co.end());
        }
        else
        {
          dynamic_update_.insert(dynamic_update_.end(), co.begin(), co.end());
        }
      }
    }
  }

  // This is because FCL supports batch update which only re-balances the tree once
  if (!static_update_.empty())
    static_manager_->update(static_update_);

  if (!dynamic_update_.empty())
    dynamic_manager_->update(dynamic_update_);
}

void FCLDiscreteBVHManager::setCollisionObjectsTransform(const std::vector<tesseract::common::LinkId>& ids,
                                                         const tesseract::common::VectorIsometry3d& poses)
{
  if (ids.size() != poses.size())
    throw std::runtime_error("FCLDiscreteBVHManager, setCollisionObjectsTransform received " +
                             std::to_string(ids.size()) + " ids but " + std::to_string(poses.size()) + " poses!");

  static_update_.clear();
  dynamic_update_.clear();
  for (std::size_t i = 0; i < ids.size(); ++i)
  {
    auto it = link2cow_.find(ids[i]);
    if (it == link2cow_.end())
      continue;

    const Eigen::Isometry3d& cur_tf = it->second->getCollisionObjectsTransform();
    // Note: If the transform has not changed do not update to prevent unnecessary re-balancing of the BVH tree
    if (tesseract::common::almostEqualRelativeAndAbs(cur_tf, poses[i]))
      continue;

    it->second->setCollisionObjectsTransform(poses[i]);
    std::vector<CollisionObjectRawPtr>& co = it->second->getCollisionObjectsRaw();
    if (it->second->m_collisionFilterGroup == CollisionFilterGroups::StaticFilter)
      static_update_.insert(static_update_.end(), co.begin(), co.end());
    else
      dynamic_update_.insert(dynamic_update_.end(), co.begin(), co.end());
  }

  // This is because FCL supports batch update which only re-balances the tree once
  if (!static_update_.empty())
    static_manager_->update(static_update_);

  if (!dynamic_update_.empty())
    dynamic_manager_->update(dynamic_update_);
}

const std::vector<tesseract::common::LinkId>& FCLDiscreteBVHManager::getCollisionObjects() const
{
  return collision_objects_;
}

void FCLDiscreteBVHManager::setActiveCollisionObjects(const std::unordered_set<tesseract::common::LinkId>& ids)
{
  active_ = ids;

  for (auto& co : link2cow_)
    updateCollisionObjectFilters(active_, co.second, static_manager_, dynamic_manager_);

  // This causes a refit on the bvh tree.
  dynamic_manager_->update();
  static_manager_->update();
}

const std::unordered_set<tesseract::common::LinkId>& FCLDiscreteBVHManager::getActiveCollisionObjects() const
{
  return active_;
}

void FCLDiscreteBVHManager::setCollisionMarginData(CollisionMarginData collision_margin_data)
{
  collision_margin_data_ = std::move(collision_margin_data);
  onCollisionMarginDataChanged();
}

const CollisionMarginData& FCLDiscreteBVHManager::getCollisionMarginData() const { return collision_margin_data_; }

void FCLDiscreteBVHManager::setCollisionMarginPairData(const CollisionMarginPairData& pair_margin_data,
                                                       CollisionMarginPairOverrideType override_type)
{
  collision_margin_data_.apply(pair_margin_data, override_type);
  onCollisionMarginDataChanged();
}

void FCLDiscreteBVHManager::setDefaultCollisionMargin(double default_collision_margin)
{
  collision_margin_data_.setDefaultCollisionMargin(default_collision_margin);
  onCollisionMarginDataChanged();
}

void FCLDiscreteBVHManager::setCollisionMarginPair(const tesseract::common::LinkId& id1,
                                                   const tesseract::common::LinkId& id2,
                                                   double collision_margin)
{
  collision_margin_data_.setCollisionMargin(id1, id2, collision_margin);
  onCollisionMarginDataChanged();
}

void FCLDiscreteBVHManager::incrementCollisionMargin(double increment)
{
  collision_margin_data_.incrementMargins(increment);
  onCollisionMarginDataChanged();
}

void FCLDiscreteBVHManager::setContactAllowedValidator(
    std::shared_ptr<const tesseract::common::ContactAllowedValidator> validator)
{
  validator_ = std::move(validator);
}
std::shared_ptr<const tesseract::common::ContactAllowedValidator>
FCLDiscreteBVHManager::getContactAllowedValidator() const
{
  return validator_;
}

void FCLDiscreteBVHManager::contactTest(ContactResultMap& collisions, const ContactRequest& request)
{
  ContactTestData cdata(collision_margin_data_, validator_, request, collisions);
  if (collision_margin_data_.getMaxCollisionMargin() > 0)
  {
    // TODO: Should the order be flipped?
    if (!static_manager_->empty())
      static_manager_->collide(dynamic_manager_.get(), &cdata, &distanceCallback);

    // It looks like the self check is as fast as selfDistanceContactTest even though it is N^2
    if (!cdata.done && !dynamic_manager_->empty())
      dynamic_manager_->collide(&cdata, &distanceCallback);
  }
  else
  {
    // TODO: Should the order be flipped?
    if (!static_manager_->empty())
      static_manager_->collide(dynamic_manager_.get(), &cdata, &collisionCallback);

    // It looks like the self check is as fast as selfDistanceContactTest even though it is N^2
    if (!cdata.done && !dynamic_manager_->empty())
      dynamic_manager_->collide(&cdata, &collisionCallback);
  }
}

void FCLDiscreteBVHManager::addCollisionObject(const COW::Ptr& cow)
{
  addCollisionObjects(std::vector<COW::Ptr>{ cow });
}

void FCLDiscreteBVHManager::addCollisionObjects(const std::vector<COW::Ptr>& cows)
{
  for (const auto& cow : cows)
  {
    const std::size_t cnt = cow->getCollisionObjectsRaw().size();
    fcl_co_count_ += cnt;
    link2cow_[cow->getLinkId()] = cow;
    collision_objects_.push_back(cow->getLinkId());

    std::vector<CollisionObjectPtr>& objects = cow->getCollisionObjects();
    if (cow->m_collisionFilterGroup == CollisionFilterGroups::StaticFilter)
    {
      // If static add to static manager
      for (auto& co : objects)
        static_manager_->registerObject(co.get());
    }
    else
    {
      for (auto& co : objects)
        dynamic_manager_->registerObject(co.get());
    }

    // If active links is not empty update filters to replace the active links list
    if (!active_.empty())
      updateCollisionObjectFilters(active_, cow, static_manager_, dynamic_manager_);
  }

  static_update_.reserve(fcl_co_count_);
  dynamic_update_.reserve(fcl_co_count_);

  // This causes a refit on the bvh tree.
  dynamic_manager_->update();
  static_manager_->update();
}

void FCLDiscreteBVHManager::onCollisionMarginDataChanged()
{
  static_update_.clear();
  dynamic_update_.clear();

  for (auto& cow : link2cow_)
  {
    cow.second->setContactDistanceThreshold(collision_margin_data_.getMaxCollisionMargin(cow.second->getLinkId()));
    std::vector<CollisionObjectRawPtr>& co = cow.second->getCollisionObjectsRaw();
    if (cow.second->m_collisionFilterGroup == CollisionFilterGroups::StaticFilter)
    {
      static_update_.insert(static_update_.end(), co.begin(), co.end());
    }
    else
    {
      dynamic_update_.insert(dynamic_update_.end(), co.begin(), co.end());
    }
  }

  if (!static_update_.empty())
    static_manager_->update(static_update_);

  if (!dynamic_update_.empty())
    dynamic_manager_->update(dynamic_update_);
}
}  // namespace tesseract::collision
