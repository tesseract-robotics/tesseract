/**
 * @file fcl_discrete_managers.cpp
 * @brief Tesseract ROS FCL contact checker implementation.
 *
 * @author Levi Armstrong
 * @date Dec 18, 2017
 * @version TODO
 * @bug No known bugs
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

#include "tesseract_collision/fcl/fcl_discrete_managers.h"

namespace tesseract_collision
{
namespace tesseract_collision_fcl
{
static const CollisionShapesConst EMPTY_COLLISION_SHAPES_CONST;
static const tesseract_common::VectorIsometry3d EMPTY_COLLISION_SHAPES_TRANSFORMS;

FCLDiscreteBVHManager::FCLDiscreteBVHManager()
{
  manager_ = std::unique_ptr<fcl::BroadPhaseCollisionManagerd>(new fcl::DynamicAABBTreeCollisionManagerd());
  contact_distance_ = 0;
}

DiscreteContactManager::Ptr FCLDiscreteBVHManager::clone() const
{
  FCLDiscreteBVHManager::Ptr manager(new FCLDiscreteBVHManager());

  for (const auto& cow : link2cow_)
  {
    COW::Ptr new_cow = cow.second->clone();

    // TODO LEVI: Should this happen as part of the clone?
    new_cow->setCollisionObjectsTransform(cow.second->getCollisionObjectsTransform());
    manager->addCollisionObject(new_cow);
  }

  manager->setActiveCollisionObjects(active_);
  manager->setContactDistanceThreshold(contact_distance_);
  manager->setIsContactAllowedFn(fn_);

  return std::move(manager);
}

bool FCLDiscreteBVHManager::addCollisionObject(const std::string& name,
                                               const int& mask_id,
                                               const CollisionShapesConst& shapes,
                                               const tesseract_common::VectorIsometry3d& shape_poses,
                                               bool enabled)
{
  COW::Ptr new_cow = createFCLCollisionObject(name, mask_id, shapes, shape_poses, enabled);
  if (new_cow != nullptr)
  {
    addCollisionObject(new_cow);
    return true;
  }

  return false;
}

const CollisionShapesConst& FCLDiscreteBVHManager::getCollisionObjectGeometries(const std::string& name) const
{
  auto cow = link2cow_.find(name);
  return (link2cow_.find(name) != link2cow_.end()) ? cow->second->getCollisionGeometries() :
                                                     EMPTY_COLLISION_SHAPES_CONST;
}

const tesseract_common::VectorIsometry3d&
FCLDiscreteBVHManager::getCollisionObjectGeometriesTransforms(const std::string& name) const
{
  auto cow = link2cow_.find(name);
  return (link2cow_.find(name) != link2cow_.end()) ? cow->second->getCollisionGeometriesTransforms() :
                                                     EMPTY_COLLISION_SHAPES_TRANSFORMS;
}

bool FCLDiscreteBVHManager::hasCollisionObject(const std::string& name) const
{
  return (link2cow_.find(name) != link2cow_.end());
}

bool FCLDiscreteBVHManager::removeCollisionObject(const std::string& name)
{
  auto it = link2cow_.find(name);
  if (it != link2cow_.end())
  {
    std::vector<CollisionObjectPtr>& objects = it->second->getCollisionObjects();
    for (auto& co : objects)
      manager_->unregisterObject(co.get());

    link2cow_.erase(name);
    return true;
  }
  return false;
}

bool FCLDiscreteBVHManager::enableCollisionObject(const std::string& name)
{
  auto it = link2cow_.find(name);
  if (it != link2cow_.end())
  {
    it->second->m_enabled = true;
    return true;
  }
  return false;
}

bool FCLDiscreteBVHManager::disableCollisionObject(const std::string& name)
{
  auto it = link2cow_.find(name);
  if (it != link2cow_.end())
  {
    it->second->m_enabled = false;
    return true;
  }
  return false;
}

void FCLDiscreteBVHManager::setCollisionObjectsTransform(const std::string& name, const Eigen::Isometry3d& pose)
{
  auto it = link2cow_.find(name);
  if (it != link2cow_.end())
    it->second->setCollisionObjectsTransform(pose);
}

void FCLDiscreteBVHManager::setCollisionObjectsTransform(const std::vector<std::string>& names,
                                                         const tesseract_common::VectorIsometry3d& poses)
{
  assert(names.size() == poses.size());
  for (auto i = 0u; i < names.size(); ++i)
    setCollisionObjectsTransform(names[i], poses[i]);
}

void FCLDiscreteBVHManager::setCollisionObjectsTransform(const tesseract_common::TransformMap& transforms)
{
  for (const auto& transform : transforms)
    setCollisionObjectsTransform(transform.first, transform.second);
}

void FCLDiscreteBVHManager::setActiveCollisionObjects(const std::vector<std::string>& names)
{
  active_ = names;

  for (auto& co : link2cow_)
  {
    COW::Ptr& cow = co.second;

    updateCollisionObjectFilters(active_, *cow);
  }
}

const std::vector<std::string>& FCLDiscreteBVHManager::getActiveCollisionObjects() const { return active_; }
void FCLDiscreteBVHManager::setContactDistanceThreshold(double contact_distance)
{
  contact_distance_ = contact_distance;
}

double FCLDiscreteBVHManager::getContactDistanceThreshold() const { return contact_distance_; }
void FCLDiscreteBVHManager::setIsContactAllowedFn(IsContactAllowedFn fn) { fn_ = fn; }
IsContactAllowedFn FCLDiscreteBVHManager::getIsContactAllowedFn() const { return fn_; }
void FCLDiscreteBVHManager::contactTest(ContactResultMap& collisions, const ContactTestType& type)
{
  ContactTestData cdata(active_, contact_distance_, fn_, type, collisions);
  if (contact_distance_ > 0)
  {
    manager_->distance(&cdata, &distanceCallback);
  }
  else
  {
    manager_->collide(&cdata, &collisionCallback);
  }
}

void FCLDiscreteBVHManager::addCollisionObject(const COW::Ptr& cow)
{
  link2cow_[cow->getName()] = cow;

  std::vector<CollisionObjectPtr>& objects = cow->getCollisionObjects();
  for (auto& co : objects)
    manager_->registerObject(co.get());
}

const Link2COW& FCLDiscreteBVHManager::getCollisionObjects() const { return link2cow_; }
}  // namespace tesseract_collision_fcl
}  // namespace tesseract_collision
