/**
 * @file bullet_discrete_bvh_manager.h
 * @brief Tesseract ROS Bullet discrete BVH collision manager.
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
#ifndef TESSERACT_COLLISION_BULLET_DISCRETE_BVH_MANAGERS_H
#define TESSERACT_COLLISION_BULLET_DISCRETE_BVH_MANAGERS_H

#include <tesseract_collision/bullet/bullet_utils.h>
#include <tesseract_collision/core/discrete_contact_manager.h>
namespace tesseract_collision
{
namespace tesseract_collision_bullet
{
/** @brief A BVH implementaiton of a bullet manager */
class BulletDiscreteBVHManager : public DiscreteContactManager
{
public:
  using Ptr = std::shared_ptr<BulletDiscreteBVHManager>;
  using ConstPtr = std::shared_ptr<const BulletDiscreteBVHManager>;

  BulletDiscreteBVHManager();
  ~BulletDiscreteBVHManager() override;
  BulletDiscreteBVHManager(const BulletDiscreteBVHManager&) = delete;
  BulletDiscreteBVHManager& operator=(const BulletDiscreteBVHManager&) = delete;
  BulletDiscreteBVHManager(BulletDiscreteBVHManager&&) = delete;
  BulletDiscreteBVHManager& operator=(BulletDiscreteBVHManager&&) = delete;

  static std::string name() { return "BulletDiscreteBVHManager"; }
  static DiscreteContactManager::Ptr create() { return std::make_shared<BulletDiscreteBVHManager>(); }

  DiscreteContactManager::Ptr clone() const override;

  bool addCollisionObject(const std::string& name,
                          const int& mask_id,
                          const CollisionShapesConst& shapes,
                          const tesseract_common::VectorIsometry3d& shape_poses,
                          bool enabled = true) override;

  const CollisionShapesConst& getCollisionObjectGeometries(const std::string& name) const override;

  const tesseract_common::VectorIsometry3d&
  getCollisionObjectGeometriesTransforms(const std::string& name) const override;

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

  std::unique_ptr<btCollisionDispatcher> dispatcher_; /**< @brief The bullet collision dispatcher used for getting
                                                         object to object collison algorithm */
  btDispatcherInfo dispatch_info_;              /**< @brief The bullet collision dispatcher configuration information */
  btDefaultCollisionConfiguration coll_config_; /**< @brief The bullet collision configuration */
  std::unique_ptr<btBroadphaseInterface> broadphase_; /**< @brief The bullet broadphase interface */
  Link2Cow link2cow_; /**< @brief A map of all (static and active) collision objects being managed */

  /**
   * @brief Perform a contact test for the provided object which is not part of the manager
   * @param cow The Collision object
   * @param collisions The collision results
   */
  void contactTest(const COW::Ptr& cow, ContactTestData& collisions);
};

}  // namespace tesseract_collision_bullet
}  // namespace tesseract_collision
#endif  // TESSERACT_COLLISION_BULLET_DISCRETE_BVH_MANAGERS_H
