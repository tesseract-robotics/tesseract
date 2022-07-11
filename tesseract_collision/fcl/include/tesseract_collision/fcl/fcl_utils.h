/**
 * @file fcl_utils.h
 * @brief Tesseract ROS FCL Utility Functions.
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

#ifndef TESSERACT_COLLISION_FCL_UTILS_H
#define TESSERACT_COLLISION_FCL_UTILS_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <fcl/broadphase/broadphase_dynamic_AABB_tree-inl.h>
#include <fcl/narrowphase/collision-inl.h>
#include <fcl/narrowphase/distance-inl.h>
#include <memory>
#include <set>
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_collision/core/types.h>
#include <tesseract_collision/core/common.h>
#include <tesseract_collision/fcl/fcl_collision_object_wrapper.h>

namespace tesseract_collision::tesseract_collision_fcl
{
using CollisionGeometryPtr = std::shared_ptr<fcl::CollisionGeometryd>;
using CollisionObjectPtr = std::shared_ptr<FCLCollisionObjectWrapper>;
using CollisionObjectRawPtr = fcl::CollisionObjectd*;
using CollisionObjectConstPtr = std::shared_ptr<const fcl::CollisionObjectd>;

enum CollisionFilterGroups
{
  DefaultFilter = 1,
  StaticFilter = 2,
  KinematicFilter = 4,
  AllFilter = -1  // all bits sets: DefaultFilter | StaticFilter | KinematicFilter
};

/**
 * @brief This is a Tesseract link collision object wrapper which add items specific to tesseract. It is a wrapper
 * around a tesseract link which may contain several collision objects.
 */
class CollisionObjectWrapper
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<CollisionObjectWrapper>;
  using ConstPtr = std::shared_ptr<const CollisionObjectWrapper>;

  CollisionObjectWrapper() = default;
  CollisionObjectWrapper(std::string name,
                         const int& type_id,
                         CollisionShapesConst shapes,
                         tesseract_common::VectorIsometry3d shape_poses);

  short int m_collisionFilterGroup{ CollisionFilterGroups::KinematicFilter };
  short int m_collisionFilterMask{ CollisionFilterGroups::StaticFilter | CollisionFilterGroups::KinematicFilter };
  bool m_enabled{ true };

  const std::string& getName() const { return name_; }
  const int& getTypeID() const { return type_id_; }
  /** \brief Check if two objects point to the same source object */
  bool sameObject(const CollisionObjectWrapper& other) const
  {
    return name_ == other.name_ && type_id_ == other.type_id_ && shapes_.size() == other.shapes_.size() &&
           shape_poses_.size() == other.shape_poses_.size() &&
           std::equal(shapes_.begin(), shapes_.end(), other.shapes_.begin()) &&
           std::equal(shape_poses_.begin(),
                      shape_poses_.end(),
                      other.shape_poses_.begin(),
                      [](const Eigen::Isometry3d& t1, const Eigen::Isometry3d& t2) { return t1.isApprox(t2); });
  }

  const CollisionShapesConst& getCollisionGeometries() const { return shapes_; }

  const tesseract_common::VectorIsometry3d& getCollisionGeometriesTransforms() const { return shape_poses_; }

  void setCollisionObjectsTransform(const Eigen::Isometry3d& pose)
  {
    world_pose_ = pose;
    for (unsigned i = 0; i < collision_objects_.size(); ++i)
    {
      CollisionObjectPtr& co = collision_objects_[i];
      co->setTransform(pose * shape_poses_[i]);
      co->updateAABB();  // This a tesseract function that updates abb to take into account contact distance
    }
  }

  void setContactDistanceThreshold(double contact_distance)
  {
    contact_distance_ = contact_distance;
    for (auto& co : collision_objects_)
      co->setContactDistanceThreshold(contact_distance_);
  }

  double getContactDistanceThreshold() const { return contact_distance_; }
  const Eigen::Isometry3d& getCollisionObjectsTransform() const { return world_pose_; }
  const std::vector<CollisionObjectPtr>& getCollisionObjects() const { return collision_objects_; }
  std::vector<CollisionObjectPtr>& getCollisionObjects() { return collision_objects_; }
  const std::vector<CollisionObjectRawPtr>& getCollisionObjectsRaw() const { return collision_objects_raw_; }
  std::vector<CollisionObjectRawPtr>& getCollisionObjectsRaw() { return collision_objects_raw_; }
  std::shared_ptr<CollisionObjectWrapper> clone() const
  {
    auto clone_cow = std::make_shared<CollisionObjectWrapper>();
    clone_cow->name_ = name_;
    clone_cow->type_id_ = type_id_;
    clone_cow->shapes_ = shapes_;
    clone_cow->shape_poses_ = shape_poses_;
    clone_cow->collision_geometries_ = collision_geometries_;

    clone_cow->collision_objects_.reserve(collision_objects_.size());
    clone_cow->collision_objects_raw_.reserve(collision_objects_.size());
    for (const auto& co : collision_objects_)
    {
      assert(std::dynamic_pointer_cast<FCLCollisionObjectWrapper>(co) != nullptr);
      auto collObj =
          std::make_shared<FCLCollisionObjectWrapper>(*std::static_pointer_cast<FCLCollisionObjectWrapper>(co));
      collObj->setUserData(clone_cow.get());
      collObj->setTransform(co->getTransform());
      collObj->updateAABB();
      clone_cow->collision_objects_.push_back(collObj);
      clone_cow->collision_objects_raw_.push_back(collObj.get());
    }

    clone_cow->m_collisionFilterGroup = m_collisionFilterGroup;
    clone_cow->m_collisionFilterMask = m_collisionFilterMask;
    clone_cow->m_enabled = m_enabled;
    return clone_cow;
  }

  /**
   * @brief Given fcl collision shape get the index to the links collision shape
   * @param co fcl collision shape
   * @return links collision shape index
   */
  int getShapeIndex(const fcl::CollisionObjectd* co) const;

protected:
  std::string name_;                                              // name of the collision object
  int type_id_{ -1 };                                             // user defined type id
  Eigen::Isometry3d world_pose_{ Eigen::Isometry3d::Identity() }; /**< @brief Collision Object World Transformation */
  CollisionShapesConst shapes_;
  tesseract_common::VectorIsometry3d shape_poses_;
  std::vector<CollisionGeometryPtr> collision_geometries_;
  std::vector<CollisionObjectPtr> collision_objects_;
  /**
   * @brief The raw pointer is also stored because FCL accepts vectors for batch process.
   * Note: They are updating the API to Shared Pointers but the broadphase has not been updated yet.
   */
  std::vector<CollisionObjectRawPtr> collision_objects_raw_;

  double contact_distance_{ 0 }; /**< @brief The contact distance threshold */
};

CollisionGeometryPtr createShapePrimitive(const CollisionShapeConstPtr& geom);

using COW = CollisionObjectWrapper;
using Link2COW = std::map<std::string, COW::Ptr>;
using Link2ConstCOW = std::map<std::string, COW::ConstPtr>;

inline COW::Ptr createFCLCollisionObject(const std::string& name,
                                         const int& type_id,
                                         const CollisionShapesConst& shapes,
                                         const tesseract_common::VectorIsometry3d& shape_poses,
                                         bool enabled)
{
  // dont add object that does not have geometry
  if (shapes.empty() || shape_poses.empty() || (shapes.size() != shape_poses.size()))
  {
    CONSOLE_BRIDGE_logDebug("ignoring link %s", name.c_str());
    return nullptr;
  }

  auto new_cow = std::make_shared<COW>(name, type_id, shapes, shape_poses);

  new_cow->m_enabled = enabled;
  CONSOLE_BRIDGE_logDebug("Created collision object for link %s", new_cow->getName().c_str());
  return new_cow;
}

/**
 * @brief Update collision objects filters
 * @param active The active collision objects
 * @param cow The collision object to update
 * @param static_manager Broadphasse manager for static objects
 * @param dynamic_manager Broadphase manager for dynamic objects
 */
inline void updateCollisionObjectFilters(const std::vector<std::string>& active,
                                         const COW::Ptr& cow,
                                         const std::unique_ptr<fcl::BroadPhaseCollisionManagerd>& static_manager,
                                         const std::unique_ptr<fcl::BroadPhaseCollisionManagerd>& dynamic_manager)
{
  // For descrete checks we can check static to kinematic and kinematic to
  // kinematic
  if (!isLinkActive(active, cow->getName()))
  {
    if (cow->m_collisionFilterGroup != CollisionFilterGroups::StaticFilter)
    {
      std::vector<CollisionObjectPtr>& objects = cow->getCollisionObjects();
      // This link was dynamic but is now static
      for (auto& co : objects)
        dynamic_manager->unregisterObject(co.get());

      for (auto& co : objects)
        static_manager->registerObject(co.get());
    }
    cow->m_collisionFilterGroup = CollisionFilterGroups::StaticFilter;
  }
  else
  {
    if (cow->m_collisionFilterGroup != CollisionFilterGroups::KinematicFilter)
    {
      std::vector<CollisionObjectPtr>& objects = cow->getCollisionObjects();
      // This link was static but is now dynamic
      for (auto& co : objects)
        static_manager->unregisterObject(co.get());

      for (auto& co : objects)
        dynamic_manager->registerObject(co.get());
    }
    cow->m_collisionFilterGroup = CollisionFilterGroups::KinematicFilter;
  }

  // If the group is StaticFilter then the Mask is KinematicFilter, then StaticFilter groups can only collide with
  // KinematicFilter groups. If the group is KinematicFilter then the mask is StaticFilter and KinematicFilter meaning
  // that KinematicFilter groups can collide with both StaticFilter and KinematicFilter groups.
  if (cow->m_collisionFilterGroup == CollisionFilterGroups::StaticFilter)
  {
    cow->m_collisionFilterMask = CollisionFilterGroups::KinematicFilter;
  }
  else
  {
    cow->m_collisionFilterMask = CollisionFilterGroups::StaticFilter | CollisionFilterGroups::KinematicFilter;
  }
}

bool collisionCallback(fcl::CollisionObjectd* o1, fcl::CollisionObjectd* o2, void* data);

bool distanceCallback(fcl::CollisionObjectd* o1, fcl::CollisionObjectd* o2, void* data);

}  // namespace tesseract_collision::tesseract_collision_fcl
#endif  // TESSERACT_COLLISION_FCL_UTILS_H
