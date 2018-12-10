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

#include <tesseract_core/macros.h>
TESSERACT_IGNORE_WARNINGS_PUSH
#include <fcl/broadphase/broadphase_dynamic_AABB_tree.h>
#include <fcl/narrowphase/collision.h>
#include <fcl/narrowphase/distance.h>
#include <memory>
#include <set>
#include <geometric_shapes/mesh_operations.h>
#include <ros/console.h>
TESSERACT_IGNORE_WARNINGS_POP

#include <tesseract_core/basic_types.h>
#include <tesseract_collision/contact_checker_common.h>

namespace tesseract
{
namespace tesseract_fcl
{
typedef std::shared_ptr<fcl::CollisionGeometryd> CollisionGeometryPtr;
typedef std::shared_ptr<fcl::CollisionObjectd> CollisionObjectPtr;
typedef std::shared_ptr<const fcl::CollisionObjectd> CollisionObjectConstPtr;

enum CollisionFilterGroups
{
  DefaultFilter = 1,
  StaticFilter = 2,
  KinematicFilter = 4,
  AllFilter = -1  // all bits sets: DefaultFilter | StaticFilter | KinematicFilter
};

class CollisionObjectWrapper
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  CollisionObjectWrapper(const std::string& name,
                         const int& type_id,
                         const std::vector<shapes::ShapeConstPtr>& shapes,
                         const VectorIsometry3d& shape_poses,
                         const CollisionObjectTypeVector& collision_object_types);

  short int m_collisionFilterGroup;
  short int m_collisionFilterMask;
  bool m_enabled;

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

  void setCollisionObjectsTransform(const Eigen::Isometry3d& pose)
  {
    world_pose_ = pose;
    for (unsigned i = 0; i < collision_objects_.size(); ++i)
    {
      CollisionObjectPtr& co = collision_objects_[i];
      co->setTransform(pose * shape_poses_[i]);
      co->computeAABB();
    }
  }

  const Eigen::Isometry3d& getCollisionObjectsTransform() const { return world_pose_; }
  const std::vector<CollisionObjectPtr>& getCollisionObjects() const { return collision_objects_; }
  std::vector<CollisionObjectPtr>& getCollisionObjects() { return collision_objects_; }
  std::shared_ptr<CollisionObjectWrapper> clone() const
  {
    std::shared_ptr<CollisionObjectWrapper> clone_cow(new CollisionObjectWrapper(
        name_, type_id_, shapes_, shape_poses_, collision_object_types_, collision_geometries_, collision_objects_));
    clone_cow->m_collisionFilterGroup = m_collisionFilterGroup;
    clone_cow->m_collisionFilterMask = m_collisionFilterMask;
    clone_cow->m_enabled = m_enabled;
    return clone_cow;
  }

protected:
  CollisionObjectWrapper(const std::string& name,
                         const int& type_id,
                         const std::vector<shapes::ShapeConstPtr>& shapes,
                         const VectorIsometry3d& shape_poses,
                         const CollisionObjectTypeVector& collision_object_types,
                         const std::vector<CollisionGeometryPtr>& collision_geometries,
                         const std::vector<CollisionObjectPtr>& collision_objects);

  std::string name_;             // name of the collision object
  int type_id_;                  // user defined type id
  Eigen::Isometry3d world_pose_; /**< @brief Collision Object World Transformation */
  std::vector<shapes::ShapeConstPtr> shapes_;
  VectorIsometry3d shape_poses_;
  CollisionObjectTypeVector collision_object_types_;
  std::vector<CollisionGeometryPtr> collision_geometries_;
  std::vector<CollisionObjectPtr> collision_objects_;
};

CollisionGeometryPtr createShapePrimitive(const shapes::ShapeConstPtr& geom,
                                          const CollisionObjectType& collision_object_type);

typedef CollisionObjectWrapper COW;
typedef std::shared_ptr<CollisionObjectWrapper> COWPtr;
typedef std::shared_ptr<const CollisionObjectWrapper> COWConstPtr;
typedef std::map<std::string, COWPtr> Link2COW;
typedef std::map<std::string, COWConstPtr> Link2ConstCOW;

inline COWPtr createFCLCollisionObject(const std::string& name,
                                       const int& type_id,
                                       const std::vector<shapes::ShapeConstPtr>& shapes,
                                       const VectorIsometry3d& shape_poses,
                                       const CollisionObjectTypeVector& collision_object_types,
                                       bool enabled)
{
  // dont add object that does not have geometry
  if (shapes.empty() || shape_poses.empty() || (shapes.size() != shape_poses.size()))
  {
    ROS_DEBUG("ignoring link %s", name.c_str());
    return nullptr;
  }

  COWPtr new_cow(new COW(name, type_id, shapes, shape_poses, collision_object_types));

  new_cow->m_enabled = enabled;
  ROS_DEBUG("Created collision object for link %s", new_cow->getName().c_str());
  return new_cow;
}

/**
 * @brief Update collision objects filters
 * @param active The active collision objects
 * @param cow The collision object to update
 */
inline void updateCollisionObjectFilters(const std::vector<std::string>& active, COW& cow)
{
  // For descrete checks we can check static to kinematic and kinematic to
  // kinematic
  cow.m_collisionFilterGroup = CollisionFilterGroups::KinematicFilter;
  if (!isLinkActive(active, cow.getName()))
  {
    cow.m_collisionFilterGroup = CollisionFilterGroups::StaticFilter;
  }

  if (cow.m_collisionFilterGroup == CollisionFilterGroups::StaticFilter)
  {
    cow.m_collisionFilterMask = CollisionFilterGroups::KinematicFilter;
  }
  else
  {
    cow.m_collisionFilterMask = CollisionFilterGroups::StaticFilter | CollisionFilterGroups::KinematicFilter;
  }
}

bool collisionCallback(fcl::CollisionObjectd* o1, fcl::CollisionObjectd* o2, void* data);

bool distanceCallback(fcl::CollisionObjectd* o1, fcl::CollisionObjectd* o2, void* data, double& min_dist);
}
}
#endif  // TESSERACT_COLLISION_FCL_UTILS_H
