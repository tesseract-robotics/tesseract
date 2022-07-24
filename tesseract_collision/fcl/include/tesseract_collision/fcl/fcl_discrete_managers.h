/**
 * @file fcl_discrete_managers.h
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

#ifndef TESSERACT_COLLISION_FCL_DISCRETE_MANAGERS_H
#define TESSERACT_COLLISION_FCL_DISCRETE_MANAGERS_H

#include <tesseract_collision/core/discrete_contact_manager.h>
#include <tesseract_collision/fcl/fcl_utils.h>

namespace tesseract_collision::tesseract_collision_fcl
{
/** @brief A FCL implementation of the discrete contact manager */
class FCLDiscreteBVHManager : public DiscreteContactManager
{
public:
  using Ptr = std::shared_ptr<FCLDiscreteBVHManager>;
  using ConstPtr = std::shared_ptr<const FCLDiscreteBVHManager>;
  using UPtr = std::unique_ptr<FCLDiscreteBVHManager>;
  using ConstUPtr = std::unique_ptr<const FCLDiscreteBVHManager>;

  FCLDiscreteBVHManager(std::string name = "FCLDiscreteBVHManager");
  ~FCLDiscreteBVHManager() override = default;
  FCLDiscreteBVHManager(const FCLDiscreteBVHManager&) = delete;
  FCLDiscreteBVHManager& operator=(const FCLDiscreteBVHManager&) = delete;
  FCLDiscreteBVHManager(FCLDiscreteBVHManager&&) = delete;
  FCLDiscreteBVHManager& operator=(FCLDiscreteBVHManager&&) = delete;

  std::string getName() const override final;

  DiscreteContactManager::UPtr clone() const override final;

  bool addCollisionObject(const std::string& name,
                          const int& mask_id,
                          const CollisionShapesConst& shapes,
                          const tesseract_common::VectorIsometry3d& shape_poses,
                          bool enabled = true) override final;

  const CollisionShapesConst& getCollisionObjectGeometries(const std::string& name) const override final;

  const tesseract_common::VectorIsometry3d&
  getCollisionObjectGeometriesTransforms(const std::string& name) const override final;

  bool hasCollisionObject(const std::string& name) const override final;

  bool removeCollisionObject(const std::string& name) override final;

  bool enableCollisionObject(const std::string& name) override final;

  bool disableCollisionObject(const std::string& name) override final;

  bool isCollisionObjectEnabled(const std::string& name) const override final;

  void setCollisionObjectsTransform(const std::string& name, const Eigen::Isometry3d& pose) override final;

  void setCollisionObjectsTransform(const std::vector<std::string>& names,
                                    const tesseract_common::VectorIsometry3d& poses) override final;

  void setCollisionObjectsTransform(const tesseract_common::TransformMap& transforms) override final;

  const std::vector<std::string>& getCollisionObjects() const override final;

  void setActiveCollisionObjects(const std::vector<std::string>& names) override final;

  const std::vector<std::string>& getActiveCollisionObjects() const override final;

  void setCollisionMarginData(
      CollisionMarginData collision_margin_data,
      CollisionMarginOverrideType override_type = CollisionMarginOverrideType::REPLACE) override final;

  void setDefaultCollisionMarginData(double default_collision_margin) override final;

  void setPairCollisionMarginData(const std::string& name1,
                                  const std::string& name2,
                                  double collision_margin) override final;

  const CollisionMarginData& getCollisionMarginData() const override final;

  void setIsContactAllowedFn(IsContactAllowedFn fn) override final;

  IsContactAllowedFn getIsContactAllowedFn() const override final;

  void contactTest(ContactResultMap& collisions, const ContactRequest& request) override final;

  /**
   * @brief Add a fcl collision object to the manager
   * @param cow The tesseract fcl collision object
   */
  void addCollisionObject(const COW::Ptr& cow);

private:
  std::string name_;

  /** @brief Broad-phase Collision Manager for active collision objects */
  std::unique_ptr<fcl::BroadPhaseCollisionManagerd> static_manager_;

  /** @brief Broad-phase Collision Manager for active collision objects */
  std::unique_ptr<fcl::BroadPhaseCollisionManagerd> dynamic_manager_;

  Link2COW link2cow_;               /**< @brief A map of all (static and active) collision objects being managed */
  std::vector<std::string> active_; /**< @brief A list of the active collision objects */
  std::vector<std::string> collision_objects_; /**< @brief A list of the collision objects */
  CollisionMarginData collision_margin_data_;  /**< @brief The contact distance threshold */
  IsContactAllowedFn fn_;                      /**< @brief The is allowed collision function */
  std::size_t fcl_co_count_{ 0 };              /**< @brief The number fcl collision objects */

  /** @brief This is used to store static collision objects to update */
  std::vector<CollisionObjectRawPtr> static_update_;

  /** @brief This is used to store dynamic collision objects to update */
  std::vector<CollisionObjectRawPtr> dynamic_update_;

  /** @brief This function will update internal data when margin data has changed */
  void onCollisionMarginDataChanged();
};

}  // namespace tesseract_collision::tesseract_collision_fcl
#endif  // TESSERACT_COLLISION_FCL_DISCRETE_MANAGERS_H
