/**
 * @file rop_inverse_kinematics.cpp
 * @brief Tesseract Robot on Positioner Inverse kinematics implementation.
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
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_kinematics/core/rop_inverse_kinematics.h>

namespace tesseract_kinematics
{
using Eigen::MatrixXd;
using Eigen::VectorXd;

InverseKinematics::Ptr RobotOnPositionerInvKin::clone() const
{
  auto cloned_invkin = std::make_shared<RobotOnPositionerInvKin>();
  cloned_invkin->init(*this);
  return std::move(cloned_invkin);
}

bool RobotOnPositionerInvKin::calcInvKinHelper(std::vector<double> &solutions,
                                         const Eigen::Isometry3d& pose,
                                         const Eigen::Ref<const Eigen::VectorXd>& seed) const
{
  Eigen::VectorXd positioner_pose(positioner_fwd_kin_->numJoints());
  nested_ik(solutions, 0, dof_range_, pose, positioner_pose, seed);

  return !solutions.empty();
}

void RobotOnPositionerInvKin::nested_ik(std::vector<double>& solutions,
    int loop_level,
    const std::vector<Eigen::VectorXd>& dof_range,
    const Eigen::Isometry3d& target_pose,
    Eigen::VectorXd& positioner_pose,
    const Eigen::Ref<const Eigen::VectorXd>& seed) const
{
  if (loop_level >= static_cast<int>(positioner_fwd_kin_->numJoints()))
  {
    ikAt(solutions, target_pose, positioner_pose, seed);
    return;
  }

  for (long i = 0; i < static_cast<long>(dof_range[static_cast<std::size_t>(loop_level)].size()); ++i)
  {
    positioner_pose(loop_level) = dof_range[static_cast<std::size_t>(loop_level)][i];
    nested_ik(solutions, loop_level + 1, dof_range, target_pose, positioner_pose, seed);
  }
}

bool RobotOnPositionerInvKin::ikAt(std::vector<double>& solutions,
                                   const Eigen::Isometry3d& target_pose,
                                   Eigen::VectorXd &positioner_pose,
                                   const Eigen::Ref<const Eigen::VectorXd> &seed) const
{
  Eigen::Isometry3d positioner_tf;
  if (!positioner_fwd_kin_->calcFwdKin(positioner_tf, positioner_pose))
    return false;

  Eigen::Isometry3d robot_target_pose = positioner_tf.inverse() * target_pose;
  if (robot_target_pose.translation().norm() > manip_reach_)
    return false;

  Eigen::VectorXd robot_solution_set;
  auto robot_dof = static_cast<int>(manip_inv_kin_->numJoints());
  if (!manip_inv_kin_->calcInvKin(robot_solution_set, robot_target_pose, seed.tail(robot_dof)))
    return false;

  long num_sols = robot_solution_set.size() / robot_dof;
  for (long i = 0; i < num_sols; i++)
  {
    double* sol = robot_solution_set.data() + robot_dof * i;

    std::vector<double> full_sol;
    full_sol.insert(end(full_sol), positioner_pose.data(), positioner_pose.data() + positioner_pose.size());
    full_sol.insert(end(full_sol), std::make_move_iterator(sol), std::make_move_iterator(sol + robot_dof));

    solutions.insert(
        begin(solutions), std::make_move_iterator(full_sol.begin()), std::make_move_iterator(full_sol.end()));
  }

  return !solutions.empty();
}

bool RobotOnPositionerInvKin::calcInvKin(Eigen::VectorXd& solutions,
                                   const Eigen::Isometry3d& pose,
                                   const Eigen::Ref<const Eigen::VectorXd>& seed) const
{
  assert(checkInitialized());
  std::vector<double> solution_set;
  if (!calcInvKinHelper(solution_set, pose, seed))
    return false;

  solutions = Eigen::Map<Eigen::VectorXd>(solution_set.data(), static_cast<long>(solution_set.size()));
  return true;
}

bool RobotOnPositionerInvKin::calcInvKin(Eigen::VectorXd& /*solutions*/,
                                   const Eigen::Isometry3d& /*pose*/,
                                   const Eigen::Ref<const Eigen::VectorXd>& /*seed*/,
                                   const std::string& /*link_name*/) const
{
  assert(checkInitialized());
  assert(false);

  CONSOLE_BRIDGE_logError("This method call is not supported by RobotOnPositionerInvKin yet.");

  return false;
}

bool RobotOnPositionerInvKin::checkJoints(const Eigen::Ref<const Eigen::VectorXd>& vec) const
{
  if (vec.size() != dof_)
  {
    CONSOLE_BRIDGE_logError("Number of joint angles (%d) don't match robot_model (%d)",
                            static_cast<int>(vec.size()),
                            dof_);
    return false;
  }

  for (int i = 0; i < vec.size(); ++i)
  {
    if ((vec[i] < joint_limits_(i, 0)) || (vec(i) > joint_limits_(i, 1)))
    {
      CONSOLE_BRIDGE_logDebug("Joint %s is out-of-range (%g < %g < %g)",
                              joint_names_[static_cast<size_t>(i)].c_str(),
                              joint_limits_(i, 0),
                              vec(i),
                              joint_limits_(i, 1));
    }
  }

  return true;
}

const std::vector<std::string>& RobotOnPositionerInvKin::getJointNames() const
{
  assert(checkInitialized());
  return joint_names_;
}

const std::vector<std::string>& RobotOnPositionerInvKin::getLinkNames() const
{
  assert(checkInitialized());
  return link_names_;
}

const std::vector<std::string>& RobotOnPositionerInvKin::getActiveLinkNames() const
{
  assert(checkInitialized());
  return active_link_names_;
}

const Eigen::MatrixX2d& RobotOnPositionerInvKin::getLimits() const { return joint_limits_; }

tesseract_scene_graph::SceneGraph::ConstPtr RobotOnPositionerInvKin::getSceneGraph() const { return scene_graph_; };
unsigned int RobotOnPositionerInvKin::numJoints() const { return dof_; }
const std::string& RobotOnPositionerInvKin::getBaseLinkName() const { return positioner_fwd_kin_->getBaseLinkName(); }
const std::string& RobotOnPositionerInvKin::getTipLinkName() const { return manip_inv_kin_->getTipLinkName(); }
const std::string& RobotOnPositionerInvKin::getName() const { return name_; }
const std::string& RobotOnPositionerInvKin::getSolverName() const { return solver_name_; }

bool RobotOnPositionerInvKin::checkInitialized() const
{
  if (!initialized_)
  {
    CONSOLE_BRIDGE_logError("Kinematics has not been initialized!");
  }

  return initialized_;
}

bool RobotOnPositionerInvKin::init(tesseract_scene_graph::SceneGraph::ConstPtr scene_graph,
          InverseKinematics::Ptr manipulator,
          double manipulator_reach,
          ForwardKinematics::Ptr positioner,
          Eigen::VectorXd positioner_sample_resolution,
          std::string name)
{
  initialized_ = false;

  if (scene_graph == nullptr)
  {
    CONSOLE_BRIDGE_logError("Null pointer to Scene Graph");
    return false;
  }

  if (!scene_graph_->getLink(scene_graph_->getRoot()))
  {
    CONSOLE_BRIDGE_logError("The scene graph has an invalid root.");
    return false;
  }

  if (manipulator == nullptr)
  {
    CONSOLE_BRIDGE_logError("Provided manipulator is a nullptr");
    return false;
  }

  if (manipulator == nullptr)
  {
    CONSOLE_BRIDGE_logError("Provided manipulator is a nullptr");
    return false;
  }

  if (!(manipulator_reach > 0))
  {
    CONSOLE_BRIDGE_logError("Manipulator reach is not greather than zero");
    return false;
  }

  if (positioner == nullptr)
  {
    CONSOLE_BRIDGE_logError("Provided positioner is a nullptr");
    return false;
  }

  if (positioner_sample_resolution.size() != positioner->numJoints())
  {
    CONSOLE_BRIDGE_logError("Positioner sample resolution must be same size as positioner number of joints");
    return false;
  }

  // TODO: Check if the manipulator base link is the child of the positioner tip link.

  for (long i = 0; i < positioner_sample_resolution.size(); ++i)
  {
    if (!(positioner_sample_resolution(i) > 0))
    {
      CONSOLE_BRIDGE_logError("Positioner sample resolution is not greather than zero");
      return false;
    }
  }

  scene_graph_ = std::move(scene_graph);
  name_ = std::move(name);
  manip_inv_kin_ = std::move(manipulator);
  manip_reach_ = manipulator_reach;
  positioner_fwd_kin_ = std::move(positioner);
  positioner_sample_resolution_ = positioner_sample_resolution;
  dof_ = positioner_fwd_kin_->numJoints() + manip_inv_kin_->numJoints();

  joint_limits_ = Eigen::MatrixX2d(dof_, 2);
  joint_limits_ << positioner_fwd_kin_->getLimits(), manip_inv_kin_->getLimits();

  joint_names_ = positioner_fwd_kin_->getJointNames();
  const auto& manip_joints = manip_inv_kin_->getJointNames();
  joint_names_.insert(joint_names_.end(), manip_joints.begin(), manip_joints.end());

  link_names_ = positioner_fwd_kin_->getLinkNames();
  const auto& manip_links = manip_inv_kin_->getLinkNames();
  link_names_.insert(link_names_.end(), manip_links.begin(), manip_links.end());

  active_link_names_ = positioner_fwd_kin_->getActiveLinkNames();
  active_link_names_.insert(active_link_names_.end(), manip_links.begin(), manip_links.end());

  auto positioner_num_joints = static_cast<int>(positioner_fwd_kin_->numJoints());
  const Eigen::MatrixX2d& positioner_limits = positioner_fwd_kin_->getLimits();

  // For the kinematics object to be sampled we need to create the joint values at the sampling resolution
  // The sampled joints results are stored in dof_range[joint index] to be used by the nested_ik function
  dof_range_.reserve(static_cast<std::size_t>(positioner_num_joints));
  for (int d = 0; d < positioner_num_joints; ++d)
  {
    // given the sampling resolution for the joint calculate the number of samples such that the resolution is not
    // exceeded.
    int cnt = static_cast<int>(std::ceil(std::abs(positioner_limits(d, 1) - positioner_limits(d, 0)) /
                                         positioner_sample_resolution_(d))) +
              1;
    dof_range_.push_back(Eigen::VectorXd::LinSpaced(cnt, positioner_limits(d, 0), positioner_limits(d, 1)));
  }

  initialized_ = true;
  return initialized_;
}

bool RobotOnPositionerInvKin::init(const RobotOnPositionerInvKin& kin)
{
  initialized_ = kin.initialized_;
  scene_graph_ = kin.scene_graph_;
  name_ = kin.name_;
  manip_inv_kin_ = kin.manip_inv_kin_->clone();
  manip_reach_ = kin.manip_reach_;
  positioner_fwd_kin_ = kin.positioner_fwd_kin_->clone();
  positioner_sample_resolution_ = kin.positioner_sample_resolution_;
  dof_ = kin.dof_;
  joint_limits_ = kin.joint_limits_;
  joint_names_ = kin.joint_names_;
  link_names_ = kin.link_names_;
  active_link_names_ = kin.active_link_names_;
  dof_range_ = kin.dof_range_;

  return initialized_;
}
}  // namespace tesseract_kinematics
