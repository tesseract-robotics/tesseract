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

#include <tesseract_core/macros.h>
TESSERACT_IGNORE_WARNINGS_PUSH
#include <eigen_conversions/eigen_msg.h>
#include <geometric_shapes/shape_operations.h>
TESSERACT_IGNORE_WARNINGS_POP

#include "tesseract_collision/fcl/fcl_discrete_managers.h"

namespace tesseract
{
namespace tesseract_fcl
{
FCLDiscreteBVHManager::FCLDiscreteBVHManager()
{
  manager_ = std::unique_ptr<fcl::BroadPhaseCollisionManagerd>(new fcl::DynamicAABBTreeCollisionManagerd());
  contact_distance_ = 0;
}

DiscreteContactManagerBasePtr FCLDiscreteBVHManager::clone() const
{
  FCLDiscreteBVHManagerPtr manager(new FCLDiscreteBVHManager());

  for (const auto& cow : link2cow_)
  {
    COWPtr new_cow = cow.second->clone();

    // TODO LEVI: Should this happen as part of the clone?
    new_cow->setCollisionObjectsTransform(cow.second->getCollisionObjectsTransform());
    manager->addCollisionObject(new_cow);
  }

  manager->setActiveCollisionObjects(active_);
  manager->setContactDistanceThreshold(contact_distance_);
  manager->setIsContactAllowedFn(fn_);

  return manager;
}

bool FCLDiscreteBVHManager::addCollisionObject(const std::string& name,
                                               const int& mask_id,
                                               const std::vector<shapes::ShapeConstPtr>& shapes,
                                               const VectorIsometry3d& shape_poses,
                                               const CollisionObjectTypeVector& collision_object_types,
                                               bool enabled)
{
  COWPtr new_cow = createFCLCollisionObject(name, mask_id, shapes, shape_poses, collision_object_types, enabled);
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
                                                         const VectorIsometry3d& poses)
{
  assert(names.size() == poses.size());
  for (auto i = 0u; i < names.size(); ++i)
    setCollisionObjectsTransform(names[i], poses[i]);
}

void FCLDiscreteBVHManager::setCollisionObjectsTransform(const TransformMap& transforms)
{
  for (const auto& transform : transforms)
    setCollisionObjectsTransform(transform.first, transform.second);
}

void FCLDiscreteBVHManager::setActiveCollisionObjects(const std::vector<std::string>& names)
{
  active_ = names;

  for (auto& co : link2cow_)
  {
    COWPtr& cow = co.second;

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

void FCLDiscreteBVHManager::addCollisionObject(const COWPtr& cow)
{
  link2cow_[cow->getName()] = cow;

  std::vector<CollisionObjectPtr>& objects = cow->getCollisionObjects();
  for (auto& co : objects)
    manager_->registerObject(co.get());
}

const Link2COW& FCLDiscreteBVHManager::getCollisionObjects() const { return link2cow_; }
}
}
