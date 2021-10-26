/**
 * @file bullet_discrete_simple_manager.cpp
 * @brief Tesseract ROS Bullet Discrete Simple Manager implementation.
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

#include "tesseract_collision/bullet/bullet_discrete_simple_manager.h"

namespace tesseract_collision::tesseract_collision_bullet
{
static const CollisionShapesConst EMPTY_COLLISION_SHAPES_CONST;
static const tesseract_common::VectorIsometry3d EMPTY_COLLISION_SHAPES_TRANSFORMS;

BulletDiscreteSimpleManager::BulletDiscreteSimpleManager(std::string name) : name_(std::move(name))
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

std::string BulletDiscreteSimpleManager::getName() const { return name_; }

DiscreteContactManager::UPtr BulletDiscreteSimpleManager::clone() const
{
  auto manager = std::make_unique<BulletDiscreteSimpleManager>();

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

bool BulletDiscreteSimpleManager::addCollisionObject(const std::string& name,
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

const CollisionShapesConst& BulletDiscreteSimpleManager::getCollisionObjectGeometries(const std::string& name) const
{
  auto cow = link2cow_.find(name);
  return (link2cow_.find(name) != link2cow_.end()) ? cow->second->getCollisionGeometries() :
                                                     EMPTY_COLLISION_SHAPES_CONST;
}

const tesseract_common::VectorIsometry3d&
BulletDiscreteSimpleManager::getCollisionObjectGeometriesTransforms(const std::string& name) const
{
  auto cow = link2cow_.find(name);
  return (link2cow_.find(name) != link2cow_.end()) ? cow->second->getCollisionGeometriesTransforms() :
                                                     EMPTY_COLLISION_SHAPES_TRANSFORMS;
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
    collision_objects_.erase(std::find(collision_objects_.begin(), collision_objects_.end(), name));
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
                                                               const tesseract_common::VectorIsometry3d& poses)
{
  assert(names.size() == poses.size());
  for (auto i = 0U; i < names.size(); ++i)
    setCollisionObjectsTransform(names[i], poses[i]);
}

void BulletDiscreteSimpleManager::setCollisionObjectsTransform(const tesseract_common::TransformMap& transforms)
{
  for (const auto& transform : transforms)
    setCollisionObjectsTransform(transform.first, transform.second);
}

const std::vector<std::string>& BulletDiscreteSimpleManager::getCollisionObjects() const { return collision_objects_; }

void BulletDiscreteSimpleManager::setActiveCollisionObjects(const std::vector<std::string>& names)
{
  active_ = names;
  contact_test_data_.active = &active_;
  cows_.clear();
  cows_.reserve(link2cow_.size());

  for (auto& co : link2cow_)
  {
    COW::Ptr& cow = co.second;

    updateCollisionObjectFilters(active_, cow);

    // Update collision object vector
    if (cow->m_collisionFilterGroup == btBroadphaseProxy::KinematicFilter)
      cows_.insert(cows_.begin(), cow);
    else
      cows_.push_back(cow);
  }
}

const std::vector<std::string>& BulletDiscreteSimpleManager::getActiveCollisionObjects() const { return active_; }

void BulletDiscreteSimpleManager::setCollisionMarginData(CollisionMarginData collision_margin_data,
                                                         CollisionMarginOverrideType override_type)
{
  contact_test_data_.collision_margin_data.apply(collision_margin_data, override_type);
  onCollisionMarginDataChanged();
}

void BulletDiscreteSimpleManager::setDefaultCollisionMarginData(double default_collision_margin)
{
  contact_test_data_.collision_margin_data.setDefaultCollisionMargin(default_collision_margin);
  onCollisionMarginDataChanged();
}

void BulletDiscreteSimpleManager::setPairCollisionMarginData(const std::string& name1,
                                                             const std::string& name2,
                                                             double collision_margin)
{
  contact_test_data_.collision_margin_data.setPairCollisionMargin(name1, name2, collision_margin);
  onCollisionMarginDataChanged();
}

const CollisionMarginData& BulletDiscreteSimpleManager::getCollisionMarginData() const
{
  return contact_test_data_.collision_margin_data;
}
void BulletDiscreteSimpleManager::setIsContactAllowedFn(IsContactAllowedFn fn) { contact_test_data_.fn = fn; }
IsContactAllowedFn BulletDiscreteSimpleManager::getIsContactAllowedFn() const { return contact_test_data_.fn; }
void BulletDiscreteSimpleManager::contactTest(ContactResultMap& collisions, const ContactRequest& request)
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

    DiscreteCollisionCollector cc(contact_test_data_, cow1, cow1->getContactProcessingThreshold());
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
        bool needs_collision = needsCollisionCheck(*cow1, *cow2, contact_test_data_.fn, false);

        if (needs_collision)
        {
          btCollisionObjectWrapper obB(
              nullptr, cow2->getCollisionShape(), cow2.get(), cow2->getWorldTransform(), -1, -1);

          btCollisionAlgorithm* algorithm =
              dispatcher_->findAlgorithm(&obA, &obB, nullptr, BT_CLOSEST_POINT_ALGORITHMS);
          assert(algorithm != nullptr);
          if (algorithm != nullptr)
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

      if (contact_test_data_.done)
        break;
    }

    if (contact_test_data_.done)
      break;
  }
}

void BulletDiscreteSimpleManager::addCollisionObject(const COW::Ptr& cow)
{
  cow->setUserPointer(&contact_test_data_);
  link2cow_[cow->getName()] = cow;
  collision_objects_.push_back(cow->getName());

  if (cow->m_collisionFilterGroup == btBroadphaseProxy::KinematicFilter)
    cows_.insert(cows_.begin(), cow);
  else
    cows_.push_back(cow);
}

void BulletDiscreteSimpleManager::onCollisionMarginDataChanged()
{
  auto margin = static_cast<btScalar>(contact_test_data_.collision_margin_data.getMaxCollisionMargin());
  for (auto& co : link2cow_)
    co.second->setContactProcessingThreshold(margin);
}

}  // namespace tesseract_collision::tesseract_collision_bullet
