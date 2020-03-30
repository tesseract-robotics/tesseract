/**
 * @file bullet3_discrete_bvh_manager.h
 * @brief Tesseract ROS Bullet3 discrete BVH collision manager.
 *
 * @author Levi Armstrong
 * @date March 7, 2020
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2020, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef TESSERACT_COLLISION_BULLET3_DISCRETE_BVH_MANAGERS_H
#define TESSERACT_COLLISION_BULLET3_DISCRETE_BVH_MANAGERS_H

#include <tesseract_collision/bullet3/bullet3_utils.h>
#include <tesseract_collision/core/discrete_contact_manager.h>
#include <Bullet3Collision/NarrowPhaseCollision/b3CpuNarrowPhase.h>
#include <Bullet3Collision/BroadPhaseCollision/b3DynamicBvhBroadphase.h>

namespace tesseract_collision
{
namespace tesseract_collision_bullet3
{
/** @brief A BVH implementaiton of a bullet manager */
class Bullet3DiscreteBVHManager : public DiscreteContactManager
{
public:
  using Ptr = std::shared_ptr<Bullet3DiscreteBVHManager>;
  using ConstPtr = std::shared_ptr<const Bullet3DiscreteBVHManager>;

  Bullet3DiscreteBVHManager();
  ~Bullet3DiscreteBVHManager() override;

  static std::string name() { return "Bullet3DiscreteBVHManager"; }
  static DiscreteContactManager::Ptr create() { return std::make_shared<Bullet3DiscreteBVHManager>(); }

  DiscreteContactManager::Ptr clone() const override;

  bool addCollisionObject(const std::string& name,
                          const int& mask_id,
                          const CollisionShapesConst& shapes,
                          const tesseract_common::VectorIsometry3d& shape_poses,
                          bool enabled = true) override;

  const CollisionShapesConst& getCollisionObjectGeometries(const std::string& name) const;

  const tesseract_common::VectorIsometry3d& getCollisionObjectGeometriesTransforms(const std::string& name) const;

  bool hasCollisionObject(const std::string& name) const override;

  bool removeCollisionObject(const std::string& name) override;

  bool enableCollisionObject(const std::string& name) override;

  bool disableCollisionObject(const std::string& name) override;

  void setCollisionObjectsTransform(const std::string& name, const Eigen::Isometry3d& pose) override;

  void setCollisionObjectsTransform(const std::vector<std::string>& names,
                                    const tesseract_common::VectorIsometry3d& poses) override;

  void setCollisionObjectsTransform(const tesseract_common::TransformMap& transforms) override;

  void setActiveCollisionObjects(const std::vector<std::string>& names) override;

  const std::vector<std::string>& getActiveCollisionObjects() const override;

  void setContactDistanceThreshold(double contact_distance) override;

  double getContactDistanceThreshold() const override;

  void setIsContactAllowedFn(IsContactAllowedFn fn) override;

  IsContactAllowedFn getIsContactAllowedFn() const override;

  void contactTest(ContactResultMap& collisions, const ContactTestType& type) override;

  /**
   * @brief A a bullet collision object to the manager
   * @param cow The tesseract bullet collision object
   */
  void addCollisionObject(const COW::Ptr& cow);

  /**
   * @brief Return collision objects
   * @return A map of collision objects <name, collision object>
   */
  const Link2Cow& getCollisionObjects() const;

private:
  std::vector<std::string> active_; /**< @brief A list of the active collision objects */
  double contact_distance_;         /**< @brief The contact distance threshold */
  IsContactAllowedFn fn_;           /**< @brief The is allowed collision function */

  long proxy_capacity_ = 50 /**< @brief The broadphase capacity */
      std::unique_ptr<b3OverlappingPairCache>
          pair_cache_;                                 /**< @brief The overlapping pair cache */
  std::unique_ptr<b3DynamicBvhBroadphase> broadphase_; /**< @brief The bullet3 broadphase interface */
  std::unique_ptr<b3CpuNarrowPhase> narrowphase_;      /**< @brief The bullet3 CPU narrowphase interface */
  b3Config coll_config_;

  Link2Cow link2cow_; /**< @brief A map of all (static and active) collision objects being managed */

  /**
   * @brief Perform a contact test for the provided object which is not part of the manager
   * @param cow The Collision object
   * @param collisions The collision results
   */
  void contactTest(const COW::Ptr& cow, ContactTestData& collisions);
};

}  // namespace tesseract_collision_bullet3
}  // namespace tesseract_collision
#endif  // TESSERACT_COLLISION_BULLET3_DISCRETE_BVH_MANAGERS_H
