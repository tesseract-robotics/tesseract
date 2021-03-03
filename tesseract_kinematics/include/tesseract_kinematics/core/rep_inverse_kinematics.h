/**
 * @file rep_inverse_kinematics.h
 * @brief Robot with External Positioner Inverse kinematics functions.
 *
 * @author Levi Armstrong
 * @date June 25, 2020
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
#ifndef TESSERACT_KINEMATICS_REP_INVERSE_KINEMATICS_H
#define TESSERACT_KINEMATICS_REP_INVERSE_KINEMATICS_H
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <unordered_map>
#include <console_bridge/console.h>

#include <tesseract_scene_graph/graph.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_kinematics/core/inverse_kinematics.h>
#include <tesseract_kinematics/core/forward_kinematics.h>

#ifdef SWIG
%shared_ptr(tesseract_kinematics::RobotWithExternalPositionerInvKin)
#endif  // SWIG

namespace tesseract_kinematics
{
/**
 * @brief Robot With External Positioner Inverse kinematic implementation.
 *
 * In this kinematic arrangement the base link is the tip link of the external positioner and the tip link is the
 * tip link of the manipulator. Therefore all provided target poses are expected to the tip link of the positioner.
 */
class RobotWithExternalPositionerInvKin : public InverseKinematics
{
public:
  // LCOV_EXCL_START
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // LCOV_EXCL_STOP

  using Ptr = std::shared_ptr<RobotWithExternalPositionerInvKin>;
  using ConstPtr = std::shared_ptr<const RobotWithExternalPositionerInvKin>;

  RobotWithExternalPositionerInvKin() = default;
  ~RobotWithExternalPositionerInvKin() override = default;
  RobotWithExternalPositionerInvKin(const RobotWithExternalPositionerInvKin&) = delete;
  RobotWithExternalPositionerInvKin& operator=(const RobotWithExternalPositionerInvKin&) = delete;
  RobotWithExternalPositionerInvKin(RobotWithExternalPositionerInvKin&&) = delete;
  RobotWithExternalPositionerInvKin& operator=(RobotWithExternalPositionerInvKin&&) = delete;

  InverseKinematics::Ptr clone() const override;

  bool update() override;

  IKSolutions calcInvKin(const Eigen::Isometry3d& pose, const Eigen::Ref<const Eigen::VectorXd>& seed) const override;

  IKSolutions calcInvKin(const Eigen::Isometry3d& pose,
                         const Eigen::Ref<const Eigen::VectorXd>& seed,
                         const std::string& link_name) const override;

  bool checkJoints(const Eigen::Ref<const Eigen::VectorXd>& vec) const override;

  const std::vector<std::string>& getJointNames() const override;

  const std::vector<std::string>& getLinkNames() const override;

  const std::vector<std::string>& getActiveLinkNames() const override;

  const tesseract_common::KinematicLimits& getLimits() const override;

  void setLimits(tesseract_common::KinematicLimits limits) override;

  tesseract_scene_graph::SceneGraph::ConstPtr getSceneGraph() const;
  unsigned int numJoints() const override;
  const std::string& getBaseLinkName() const override;
  const std::string& getTipLinkName() const override;
  const std::string& getName() const override;
  const std::string& getSolverName() const override;

  /**
   * @brief Initializes Inverse Kinematics for a robot on a positioner
   * @param scene_graph The Tesseract Scene Graph
   * @param manipulator
   * @param manipulator_reach
   * @param positioner
   * @param positioner_sample_resolution
   * @param name The name of the kinematic object
   * @return True if init() completes successfully
   */
  bool init(tesseract_scene_graph::SceneGraph::ConstPtr scene_graph,
            InverseKinematics::Ptr manipulator,
            double manipulator_reach,
            ForwardKinematics::Ptr positioner,
            Eigen::VectorXd positioner_sample_resolution,
            const tesseract_common::TransformMap& current_transforms,
            std::string name);

  /**
   * @brief Initializes Inverse Kinematics for a robot on a positioner
   * @param scene_graph The Tesseract Scene Graph
   * @param manipulator
   * @param manipulator_reach
   * @param positioner
   * @param positioner_sample_resolution
   * @param name The name of the kinematic object
   * @param solver_name The name given to the solver. This is exposed so you may have same solver with different
   * sampling resolutions
   * @return True if init() completes successfully
   */
  bool init(tesseract_scene_graph::SceneGraph::ConstPtr scene_graph,
            InverseKinematics::Ptr manipulator,
            double manipulator_reach,
            ForwardKinematics::Ptr positioner,
            Eigen::VectorXd positioner_sample_resolution,
            std::string name,
            std::string solver_name = "RobotWithExternalPositionerInvKin");

  /**
   * @brief Initializes Inverse Kinematics for a robot on a positioner
   * @param scene_graph The Tesseract Scene Graph
   * @param manipulator
   * @param manipulator_reach
   * @param positioner
   * @param positioner_sample_resolution
   * @param robot_to_positioner
   * @param name The name of the kinematic object
   * @param solver_name The name given to the solver. This is exposed so you may have same solver with different
   * sampling resolutions
   * @return True if init() completes successfully
   */
  bool init(tesseract_scene_graph::SceneGraph::ConstPtr scene_graph,
            InverseKinematics::Ptr manipulator,
            double manipulator_reach,
            ForwardKinematics::Ptr positioner,
            Eigen::VectorXd positioner_sample_resolution,
            const Eigen::Isometry3d& robot_to_positioner,
            std::string name,
            std::string solver_name = "RobotWithExternalPositionerInvKin");

  /**
   * @brief Checks if kinematics has been initialized
   * @return True if init() has completed successfully
   */
  bool checkInitialized() const;

private:
  bool initialized_{ false };                               /**< Identifies if the object has been initialized */
  tesseract_scene_graph::SceneGraph::ConstPtr scene_graph_; /**< Tesseract Scene Graph */
  InverseKinematics::Ptr manip_inv_kin_;
  double manip_reach_{ 0 };
  ForwardKinematics::Ptr positioner_fwd_kin_;
  Eigen::VectorXd positioner_sample_resolution_;
  Eigen::Isometry3d manip_base_to_positioner_base_;
  unsigned dof_;
  tesseract_common::KinematicLimits limits_;
  std::vector<std::string> joint_names_;
  std::vector<std::string> link_names_;
  std::vector<std::string> active_link_names_;
  std::vector<Eigen::VectorXd> dof_range_;
  std::string name_;                                               /**< Name of the kinematic chain */
  std::string solver_name_{ "RobotWithExternalPositionerInvKin" }; /**< Name of this solver */

  /**
   * @brief This used by the clone method
   * @return True if init() completes successfully
   */
  bool init(const RobotWithExternalPositionerInvKin& kin);

  /** @brief calcFwdKin helper function */
  IKSolutions calcInvKinHelper(const Eigen::Isometry3d& pose, const Eigen::Ref<const Eigen::VectorXd>& seed) const;

  void nested_ik(IKSolutions& solutions,
                 int loop_level,
                 const std::vector<Eigen::VectorXd>& dof_range,
                 const Eigen::Isometry3d& target_pose,
                 Eigen::VectorXd& positioner_pose,
                 const Eigen::Ref<const Eigen::VectorXd>& seed) const;

  void ikAt(IKSolutions& solutions,
            const Eigen::Isometry3d& target_pose,
            Eigen::VectorXd& positioner_pose,
            const Eigen::Ref<const Eigen::VectorXd>& seed) const;
};
}  // namespace tesseract_kinematics
#endif  // TESSERACT_KINEMATICS_REP_INVERSE_KINEMATICS_H
