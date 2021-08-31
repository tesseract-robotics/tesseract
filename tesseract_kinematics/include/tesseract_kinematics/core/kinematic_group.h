/**
 * @file kinematic_group.h
 * @brief A kinematic group with forward and inverse kinematics methods.
 *
 * @author Levi Armstrong
 * @date Aug 20, 2021
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2021, Southwest Research Institute
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
#ifndef TESSERACT_KINEMATICS_KINEMATIC_GROUP_H
#define TESSERACT_KINEMATICS_KINEMATIC_GROUP_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_kinematics/core/joint_group.h>
#include <tesseract_kinematics/core/inverse_kinematics.h>

namespace tesseract_kinematics
{
/**
 * @brief The Kinematic Group Inverse Kinematics Input Data
 * @details For simple case your inverse kinetics object only requires a single input to solve for
 * but imagine the case where you have two robots and a positioner. Now each robot requires an
 * input to solve IK for. This structure is to support the ability to provide multiple inputs for
 * kinematic arragements involving multiple robots.
 */
struct KinGroupIKInput
{
  // LCOV_EXCL_START
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // LCOV_EXCL_STOP

  KinGroupIKInput(const Eigen::Isometry3d& p, std::string wf, std::string tl)
    : pose(p), working_frame(std::move(wf)), tip_link_name(std::move(tl))
  {
  }

  /** @brief The desired inverse kinematic pose */
  Eigen::Isometry3d pose;

  /**
   * @brief The link name the pose is relative to
   * @details The provided working frame must be listed in InverseKinematics::getWorkingFrames()
   */
  std::string working_frame;

  /**
   * @brief The tip link of the kinematic object to solve IK
   * @details The provided tip link name must be listed in InverseKinematics::getTipLinkNames()
   */
  std::string tip_link_name;  // This defines the internal kinematic group the information belongs to
};

using KinGroupIKInputs = tesseract_common::AlignedVector<KinGroupIKInput>;

class KinematicGroup : public JointGroup
{
public:
  // LCOV_EXCL_START
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // LCOV_EXCL_STOP

  using Ptr = std::shared_ptr<KinematicGroup>;
  using ConstPtr = std::shared_ptr<const KinematicGroup>;
  using UPtr = std::unique_ptr<KinematicGroup>;
  using ConstUPtr = std::unique_ptr<const KinematicGroup>;

  ~KinematicGroup() = default;
  KinematicGroup(const KinematicGroup& other);
  KinematicGroup& operator=(const KinematicGroup& other);
  KinematicGroup(KinematicGroup&&) = default;
  KinematicGroup& operator=(KinematicGroup&&) = default;

  /**
   * @brief Create a kinematics group with inverse kinematics
   * @param inv_kin The inverse kinematics object to create kinematic group from
   * @param scene_graph The scene graph
   * @param scene_state The scene state
   */
  KinematicGroup(InverseKinematics::UPtr inv_kin,
                 const tesseract_scene_graph::SceneGraph& scene_graph,
                 const tesseract_scene_graph::SceneState& scene_state);

  /**
   * @brief Calculates joint solutions given a pose.
   * @details If redundant solutions are needed see utility funciton getRedundantSolutions.
   * @param solutions A vector of solutions, so check the size of the vector to determine the number of solutions
   * @param tip_link_poses The input information to solve inverse kinematics for. There must be an input for each link
   * provided in getTipLinkNames
   * @param seed Vector of seed joint angles (size must match number of joints in robot chain)
   * @return A vector of solutions, If empty it failed to find a solution (including uninitialized)
   */
  IKSolutions calcInvKin(const KinGroupIKInputs& tip_link_poses, const Eigen::Ref<const Eigen::VectorXd>& seed) const;

  /** @brief Get the working frames */
  std::vector<std::string> getWorkingFrames() const;

  /** @brief Get the tip link name */
  std::vector<std::string> getTipLinkNames() const;

private:
  InverseKinematics::UPtr inv_kin_;
  Eigen::Isometry3d inv_to_fwd_base_{ Eigen::Isometry3d::Identity() };
  std::vector<std::string> working_frames_;
  std::vector<std::string> tip_link_names_;
  std::unordered_map<std::string, std::string> inv_working_frames_map_;
  std::unordered_map<std::string, std::string> inv_tip_links_map_;
};

}  // namespace tesseract_kinematics

#endif  // TESSERACT_KINEMATICS_KINEMATIC_GROUP_H
