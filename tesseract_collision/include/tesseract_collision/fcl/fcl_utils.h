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

//#include <fcl/broadphase/broadphase_collision_manager.h>
#include <fcl/broadphase/broadphase_dynamic_AABB_tree.h>
#include <fcl/narrowphase/collision.h>
#include <fcl/narrowphase/distance.h>
#include <memory>
#include <set>
#include <tesseract_core/basic_types.h>
#include <tesseract_collision/contact_checker_common.h>
#include <geometric_shapes/mesh_operations.h>
#include <ros/console.h>

namespace tesseract
{
typedef std::shared_ptr<fcl::CollisionGeometryd> FCLCollisionGeometryPtr;
typedef std::shared_ptr<fcl::CollisionObjectd> FCLCollisionObjectPtr;
typedef std::shared_ptr<const fcl::CollisionObjectd> FCLCollisionObjectConstPtr;

enum FCLCollisionFilterGroups
{
  DefaultFilter = 1,
  StaticFilter = 2,
  KinematicFilter = 4,
  AllFilter = -1 //all bits sets: DefaultFilter | StaticFilter | KinematicFilter
};

class FCLCollisionObjectWrapper
{
public:
  FCLCollisionObjectWrapper(const std::string& name,
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
  bool sameObject(const FCLCollisionObjectWrapper& other) const
  {
    return name_ == other.name_ && type_id_ == other.type_id_ && &shapes_ == &(other.shapes_) &&
           &shape_poses_ == &(other.shape_poses_);
  }

  void setCollisionObjectsTransform(const Eigen::Isometry3d& pose)
  {
    world_pose_ = pose;
    for (unsigned i = 0; i < collision_objects_.size(); ++i)
    {
      FCLCollisionObjectPtr& co = collision_objects_[i];
      co->setTransform(pose * shape_poses_[i]);
      co->computeAABB();
    }
  }

  const Eigen::Isometry3d& getCollisionObjectsTransform() const { return world_pose_; }

  const std::vector<FCLCollisionObjectPtr>& getCollisionObjects() const
  {
    return collision_objects_;
  }

  std::vector<FCLCollisionObjectPtr>& getCollisionObjects()
  {
    return collision_objects_;
  }

  std::shared_ptr<FCLCollisionObjectWrapper> clone()
  {
    std::shared_ptr<FCLCollisionObjectWrapper> clone_cow(
        new FCLCollisionObjectWrapper(name_, type_id_, shapes_, shape_poses_, collision_object_types_, collision_geometries_, collision_objects_));
    clone_cow->m_collisionFilterGroup = m_collisionFilterGroup;
    clone_cow->m_collisionFilterMask = m_collisionFilterMask;
    clone_cow->m_enabled = m_enabled;
    return clone_cow;
  }

protected:

  FCLCollisionObjectWrapper(const std::string& name,
                            const int& type_id,
                            const std::vector<shapes::ShapeConstPtr>& shapes,
                            const VectorIsometry3d& shape_poses,
                            const CollisionObjectTypeVector& collision_object_types,
                            const std::vector<FCLCollisionGeometryPtr>& collision_geometries,
                            const std::vector<FCLCollisionObjectPtr>& collision_objects);

  std::string name_;  // name of the collision object
  int type_id_;       // user defined type id
  Eigen::Isometry3d world_pose_; /**< @brief Collision Object World Transformation */
  std::vector<shapes::ShapeConstPtr> shapes_;
  VectorIsometry3d shape_poses_;
  CollisionObjectTypeVector collision_object_types_;
  std::vector<FCLCollisionGeometryPtr> collision_geometries_;
  std::vector<FCLCollisionObjectPtr> collision_objects_;
};

FCLCollisionGeometryPtr createShapePrimitive(const shapes::ShapeConstPtr& geom,
                                             const CollisionObjectType& collision_object_type);

typedef FCLCollisionObjectWrapper FCLCOW;
typedef std::shared_ptr<FCLCollisionObjectWrapper> FCLCOWPtr;
typedef std::shared_ptr<const FCLCollisionObjectWrapper> FCLCOWConstPtr;
typedef std::map<std::string, FCLCOWPtr> Link2FCLCOW;
typedef std::map<std::string, FCLCOWConstPtr> Link2ConstFCLCOW;

inline FCLCOWPtr createFCLCollisionObject(const std::string& name,
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
    return false;
  }

  FCLCOWPtr new_cow(new FCLCOW(name, type_id, shapes, shape_poses, collision_object_types));

  if (new_cow)
  {
    new_cow->m_enabled = enabled;
    ROS_DEBUG("Created collision object for link %s", new_cow->getName().c_str());
    return new_cow;
  }
  else
  {
    ROS_DEBUG("Failed to create collision object for link %s", name.c_str());
    return nullptr;
  }
}

/**
 * @brief updateCollisionObjectsWithRequest
 * @param req
 * @param cow
 */
inline void updateCollisionObjectWithRequest(const ContactRequest& req, FCLCOW& cow)
{
  // For descrete checks we can check static to kinematic and kinematic to
  // kinematic
  cow.m_collisionFilterGroup = FCLCollisionFilterGroups::KinematicFilter;
  if (!req.link_names.empty())
  {
    bool check = (std::find_if(req.link_names.begin(), req.link_names.end(), [&](std::string link) {
                    return link == cow.getName();
                  }) == req.link_names.end());
    if (check)
    {
      cow.m_collisionFilterGroup = FCLCollisionFilterGroups::StaticFilter;
    }
  }

  if (cow.m_collisionFilterGroup == FCLCollisionFilterGroups::StaticFilter)
  {
    cow.m_collisionFilterMask = FCLCollisionFilterGroups::KinematicFilter;
  }
  else
  {
    cow.m_collisionFilterMask = FCLCollisionFilterGroups::StaticFilter | FCLCollisionFilterGroups::KinematicFilter;
  }
}

bool collisionCallback(fcl::CollisionObjectd* o1, fcl::CollisionObjectd* o2, void* data);

bool distanceCallback(fcl::CollisionObjectd* o1, fcl::CollisionObjectd* o2, void* data, double& min_dist);

}
#endif // TESSERACT_COLLISION_FCL_UTILS_H
