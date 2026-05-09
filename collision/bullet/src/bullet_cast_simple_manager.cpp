/**
 * @file bullet_cast_simple_manager.cpp
 * @brief Tesseract ROS Bullet Cast(continuous) Simple Manager implementation.
 *
 * @author Levi Armstrong
 * @date Dec 18, 2017
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

#include <tesseract/collision/bullet/bullet_cast_simple_manager.h>
#include <tesseract/common/contact_allowed_validator.h>

#include <cassert>

using namespace tesseract::collision::bullet_internal;

namespace tesseract::collision
{
static const CollisionShapesConst EMPTY_COLLISION_SHAPES_CONST;
static const tesseract::common::VectorIsometry3d EMPTY_COLLISION_SHAPES_TRANSFORMS;

BulletCastSimpleManager::BulletCastSimpleManager(std::string name, TesseractCollisionConfigurationInfo config_info)
  : name_(std::move(name)), config_info_(std::move(config_info)), coll_config_(config_info_)
{
  dispatcher_ = std::make_unique<btCollisionDispatcher>(&coll_config_);

  dispatcher_->registerCollisionCreateFunc(
      BOX_SHAPE_PROXYTYPE,
      BOX_SHAPE_PROXYTYPE,
      coll_config_.getCollisionAlgorithmCreateFunc(CONVEX_SHAPE_PROXYTYPE, CONVEX_SHAPE_PROXYTYPE));

  dispatcher_->setDispatcherFlags(dispatcher_->getDispatcherFlags() &
                                  ~btCollisionDispatcher::CD_USE_RELATIVE_CONTACT_BREAKING_THRESHOLD);
  contact_test_data_.collision_margin_data = CollisionMarginData(0);
}

std::string BulletCastSimpleManager::getName() const { return name_; }

ContinuousContactManager::UPtr BulletCastSimpleManager::clone() const
{
  auto manager = std::make_unique<BulletCastSimpleManager>(name_, config_info_.clone());

  for (const auto& id : collision_objects_)
  {
    const auto& orig = link2cow_.at(id);
    COW::Ptr new_cow = orig->clone();

    assert(new_cow->getCollisionShape());
    assert(new_cow->getCollisionShape()->getShapeType() != CUSTOM_CONVEX_SHAPE_TYPE);

    new_cow->setWorldTransform(orig->getWorldTransform());
    auto margin =
        static_cast<btScalar>(contact_test_data_.collision_margin_data.getMaxCollisionMargin(new_cow->getLinkId()));
    new_cow->setContactProcessingThreshold(margin);

    manager->addCollisionObject(new_cow);
  }

  manager->setActiveCollisionObjects(active_);
  manager->setCollisionMarginData(contact_test_data_.collision_margin_data);
  manager->setContactAllowedValidator(contact_test_data_.validator);

  return manager;
}

bool BulletCastSimpleManager::addCollisionObject(const tesseract::common::LinkId& id,
                                                 const int& mask_id,
                                                 const CollisionShapesConst& shapes,
                                                 const tesseract::common::VectorIsometry3d& shape_poses,
                                                 bool enabled)
{
  if (link2cow_.find(id) != link2cow_.end())
    removeCollisionObject(id);

  COW::Ptr new_cow = createCollisionObject(id, mask_id, shapes, shape_poses, enabled);
  if (new_cow != nullptr)
  {
    auto margin =
        static_cast<btScalar>(contact_test_data_.collision_margin_data.getMaxCollisionMargin(new_cow->getLinkId()));
    new_cow->setContactProcessingThreshold(margin);
    addCollisionObject(new_cow);
    return true;
  }

  return false;
}

const CollisionShapesConst&
BulletCastSimpleManager::getCollisionObjectGeometries(const tesseract::common::LinkId& id) const
{
  auto cow = link2cow_.find(id);
  return (cow != link2cow_.end()) ? cow->second->getCollisionGeometries() : EMPTY_COLLISION_SHAPES_CONST;
}

const tesseract::common::VectorIsometry3d&
BulletCastSimpleManager::getCollisionObjectGeometriesTransforms(const tesseract::common::LinkId& id) const
{
  auto cow = link2cow_.find(id);
  return (cow != link2cow_.end()) ? cow->second->getCollisionGeometriesTransforms() : EMPTY_COLLISION_SHAPES_TRANSFORMS;
}

bool BulletCastSimpleManager::hasCollisionObject(const tesseract::common::LinkId& id) const
{
  return (link2cow_.find(id) != link2cow_.end());
}

bool BulletCastSimpleManager::removeCollisionObject(const tesseract::common::LinkId& id)
{
  auto it = link2cow_.find(id);
  if (it != link2cow_.end())
  {
    cows_.erase(std::find_if(cows_.begin(), cows_.end(), [&id](const auto& p) { return p->getLinkId() == id; }));
    collision_objects_.erase(std::find(collision_objects_.begin(), collision_objects_.end(), id));
    link2cow_.erase(it);
    link2castcow_.erase(id);
    active_.erase(id);
    return true;
  }

  return false;
}

bool BulletCastSimpleManager::enableCollisionObject(const tesseract::common::LinkId& id)
{
  auto it = link2cow_.find(id);
  if (it != link2cow_.end())
  {
    it->second->m_enabled = true;
    link2castcow_[id]->m_enabled = true;
    return true;
  }

  return false;
}

bool BulletCastSimpleManager::disableCollisionObject(const tesseract::common::LinkId& id)
{
  auto it = link2cow_.find(id);
  if (it != link2cow_.end())
  {
    it->second->m_enabled = false;
    link2castcow_[id]->m_enabled = false;
    return true;
  }

  return false;
}

bool BulletCastSimpleManager::isCollisionObjectEnabled(const tesseract::common::LinkId& id) const
{
  auto it = link2cow_.find(id);
  if (it != link2cow_.end())
    return it->second->m_enabled;

  return false;
}

void BulletCastSimpleManager::setCollisionObjectsTransform(const tesseract::common::LinkId& id,
                                                           const Eigen::Isometry3d& pose)
{
  auto it = link2cow_.find(id);
  if (it != link2cow_.end())
  {
    btTransform tf = convertEigenToBt(pose);
    it->second->setWorldTransform(tf);
    link2castcow_[id]->setWorldTransform(tf);
  }
}

Eigen::Isometry3d BulletCastSimpleManager::getCollisionObjectsTransform(const tesseract::common::LinkId& id) const
{
  // Returns pose1 (start) — link2cow_ tracks the start pose; pose2 is encoded in the cast hull and unrecoverable.
  return convertBtToEigen(link2cow_.at(id)->getWorldTransform());
}

void BulletCastSimpleManager::setCollisionObjectsTransform(const tesseract::common::LinkIdTransformMap& transforms)
{
  for (const auto& [id, tf] : transforms)
  {
    auto it = link2cow_.find(id);
    if (it != link2cow_.end())
    {
      btTransform bt_tf = convertEigenToBt(tf);
      it->second->setWorldTransform(bt_tf);
      link2castcow_[id]->setWorldTransform(bt_tf);
    }
  }
}

void BulletCastSimpleManager::setCollisionObjectsTransform(const tesseract::common::LinkId& id,
                                                           const Eigen::Isometry3d& pose1,
                                                           const Eigen::Isometry3d& pose2)
{
  auto it = link2castcow_.find(id);
  if (it != link2castcow_.end())
  {
    COW::Ptr& cow = it->second;
    assert(cow->m_collisionFilterGroup == btBroadphaseProxy::KinematicFilter);

    btTransform tf1 = convertEigenToBt(pose1);
    btTransform tf2 = convertEigenToBt(pose2);

    cow->setWorldTransform(tf1);
    link2cow_[id]->setWorldTransform(tf1);

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
        throw std::runtime_error("I can only collision check convex shapes and compound shapes made of convex shapes");
      }
    }
  }
}

void BulletCastSimpleManager::setCollisionObjectsTransform(const tesseract::common::LinkIdTransformMap& pose1,
                                                           const tesseract::common::LinkIdTransformMap& pose2)
{
  for (const auto& id : getCollisionObjects())
  {
    auto it1 = pose1.find(id);
    auto it2 = pose2.find(id);
    if (it1 != pose1.end() && it2 != pose2.end())
      setCollisionObjectsTransform(id, it1->second, it2->second);
  }
}

const std::vector<tesseract::common::LinkId>& BulletCastSimpleManager::getCollisionObjects() const
{
  return collision_objects_;
}

void BulletCastSimpleManager::setActiveCollisionObjects(const std::unordered_set<tesseract::common::LinkId>& ids)
{
  active_ = ids;

  cows_.clear();
  cows_.reserve(link2cow_.size());

  // Now need to update the broadphase with correct aabb
  for (auto& co : link2cow_)
  {
    COW::Ptr& cow = co.second;

    // Update with request
    updateCollisionObjectFilters(active_, cow);

    // Get the cast collision object
    COW::Ptr cast_cow = link2castcow_[cow->getLinkId()];

    // Update with request
    updateCollisionObjectFilters(active_, cast_cow);

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

const std::unordered_set<tesseract::common::LinkId>& BulletCastSimpleManager::getActiveCollisionObjectIds() const
{
  return active_;
}

void BulletCastSimpleManager::setCollisionMarginData(CollisionMarginData collision_margin_data)
{
  contact_test_data_.collision_margin_data = std::move(collision_margin_data);
  onCollisionMarginDataChanged();
}

const CollisionMarginData& BulletCastSimpleManager::getCollisionMarginData() const
{
  return contact_test_data_.collision_margin_data;
}

void BulletCastSimpleManager::setCollisionMarginPairData(const CollisionMarginPairData& pair_margin_data,
                                                         CollisionMarginPairOverrideType override_type)
{
  contact_test_data_.collision_margin_data.apply(pair_margin_data, override_type);
  onCollisionMarginDataChanged();
}

void BulletCastSimpleManager::setDefaultCollisionMargin(double default_collision_margin)
{
  contact_test_data_.collision_margin_data.setDefaultCollisionMargin(default_collision_margin);
  onCollisionMarginDataChanged();
}

void BulletCastSimpleManager::setCollisionMarginPair(const tesseract::common::LinkId& id1,
                                                     const tesseract::common::LinkId& id2,
                                                     double collision_margin)
{
  contact_test_data_.collision_margin_data.setCollisionMargin(id1, id2, collision_margin);
  onCollisionMarginDataChanged();
}

void BulletCastSimpleManager::incrementCollisionMargin(double increment)
{
  contact_test_data_.collision_margin_data.incrementMargins(increment);
  onCollisionMarginDataChanged();
}

void BulletCastSimpleManager::setContactAllowedValidator(
    std::shared_ptr<const tesseract::common::ContactAllowedValidator> validator)
{
  contact_test_data_.validator = std::move(validator);
}
std::shared_ptr<const tesseract::common::ContactAllowedValidator>
BulletCastSimpleManager::getContactAllowedValidator() const
{
  return contact_test_data_.validator;
}
void BulletCastSimpleManager::contactTest(ContactResultMap& collisions, const ContactRequest& request)
{
  contact_test_data_.res = &collisions;
  contact_test_data_.req = request;
  contact_test_data_.done = false;

  for (auto cow1_iter = cows_.begin(); cow1_iter != (cows_.end() - 1); cow1_iter++)
  {
    const COW::Ptr& cow1 = *cow1_iter;

    if (cow1->m_collisionFilterGroup != btBroadphaseProxy::KinematicFilter)
      break;

    if (!cow1->m_enabled)
      continue;

    btVector3 min_aabb[2], max_aabb[2];  // NOLINT
    cow1->getAABB(min_aabb[0], max_aabb[0]);

    btCollisionObjectWrapper obA(nullptr, cow1->getCollisionShape(), cow1.get(), cow1->getWorldTransform(), -1, -1);

    CastCollisionCollector cc(contact_test_data_, cow1);
    for (auto cow2_iter = cow1_iter + 1; cow2_iter != cows_.end(); cow2_iter++)
    {
      assert(!contact_test_data_.done);

      const COW::Ptr& cow2 = *cow2_iter;
      cow2->getAABB(min_aabb[1], max_aabb[1]);

      bool aabb_check = (min_aabb[0][0] <= max_aabb[1][0] && max_aabb[0][0] >= min_aabb[1][0]) &&
                        (min_aabb[0][1] <= max_aabb[1][1] && max_aabb[0][1] >= min_aabb[1][1]) &&
                        (min_aabb[0][2] <= max_aabb[1][2] && max_aabb[0][2] >= min_aabb[1][2]);

      if (aabb_check)
      {
        const tesseract::common::LinkIdPair link_pair(cow1->getLinkId(), cow2->getLinkId());
        bool needs_collision = needsCollisionCheck(*cow1, *cow2, contact_test_data_.validator, false);

        if (needs_collision)
        {
          btCollisionObjectWrapper obB(
              nullptr, cow2->getCollisionShape(), cow2.get(), cow2->getWorldTransform(), -1, -1);

          btCollisionAlgorithm* algorithm =
              dispatcher_->findAlgorithm(&obA, &obB, nullptr, BT_CLOSEST_POINT_ALGORITHMS);
          assert(algorithm != nullptr);
          if (algorithm != nullptr)
          {
            // Update the contact threshold to be pair specific
            cc.m_closestDistanceThreshold = contact_test_data_.collision_margin_data.getCollisionMargin(link_pair);
            TesseractBridgedManifoldResult contactPointResult(&obA, &obB, cc);

            // discrete collision detection query
            algorithm->processCollision(&obA, &obB, dispatch_info_, &contactPointResult);

            algorithm->~btCollisionAlgorithm();
            dispatcher_->freeCollisionAlgorithm(algorithm);
          }
        }
      }

      if (contact_test_data_.done)
        break;
    }

    if (contact_test_data_.done)
      break;
  }
}

void BulletCastSimpleManager::addCollisionObject(const COW::Ptr& cow)
{
  cow->setUserPointer(&contact_test_data_);
  const auto lid = cow->getLinkId();
  link2cow_[lid] = cow;
  collision_objects_.push_back(cow->getLinkId());

  // Create cast collision object
  COW::Ptr cast_cow = makeCastCollisionObject(cow);
  cast_cow->setUserPointer(&contact_test_data_);

  // Add it to the cast map
  link2castcow_[lid] = cast_cow;

  if (cow->m_collisionFilterGroup == btBroadphaseProxy::KinematicFilter)
    cows_.insert(cows_.begin(), cast_cow);
  else
    cows_.push_back(cow);
}

void BulletCastSimpleManager::onCollisionMarginDataChanged()
{
  for (auto& co : link2cow_)
  {
    auto margin =
        static_cast<btScalar>(contact_test_data_.collision_margin_data.getMaxCollisionMargin(co.second->getLinkId()));
    co.second->setContactProcessingThreshold(margin);
  }

  for (auto& co : link2castcow_)
  {
    auto margin =
        static_cast<btScalar>(contact_test_data_.collision_margin_data.getMaxCollisionMargin(co.second->getLinkId()));
    co.second->setContactProcessingThreshold(margin);
  }
}

}  // namespace tesseract::collision
