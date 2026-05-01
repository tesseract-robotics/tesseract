/**
 * @file rtp_inv_kin.cpp
 * @brief Robot with Tool Positioner Inverse kinematics implementation.
 */

#include <tesseract/kinematics/rtp_inv_kin.h>
#include <tesseract/kinematics/utils.h>
#include <tesseract/kinematics/forward_kinematics.h>
#include <tesseract/scene_graph/graph.h>
#include <tesseract/scene_graph/joint.h>
#include <tesseract/scene_graph/scene_state.h>

namespace tesseract::kinematics
{
using Eigen::MatrixXd;
using Eigen::VectorXd;

RTPInvKin::RTPInvKin(const tesseract::scene_graph::SceneGraph& scene_graph,
                     const tesseract::scene_graph::SceneState& scene_state,
                     InverseKinematics::UPtr manipulator,
                     double manipulator_reach,
                     std::unique_ptr<ForwardKinematics> tool_positioner,
                     const Eigen::VectorXd& tool_sample_resolution,
                     std::string solver_name)
{
  if (tool_positioner == nullptr)
    throw std::runtime_error("Provided tool positioner is a nullptr");

  if (!scene_graph.getLink(scene_graph.getRoot()))
    throw std::runtime_error("The scene graph has an invalid root.");

  std::vector<std::string> joint_names = tool_positioner->getJointNames();
  auto s = static_cast<Eigen::Index>(joint_names.size());
  Eigen::MatrixX2d tool_limits;
  tool_limits.resize(s, 2);
  for (Eigen::Index i = 0; i < s; ++i)
  {
    const auto& name = joint_names[static_cast<std::size_t>(i)];
    auto joint = scene_graph.getJoint(name);
    if (joint == nullptr)
      throw std::runtime_error("Tool positioner joint '" + name + "' not found in scene graph");
    if (joint->limits == nullptr)
      throw std::runtime_error("Tool positioner joint '" + name + "' has no limits");
    tool_limits(i, 0) = joint->limits->lower;
    tool_limits(i, 1) = joint->limits->upper;
  }

  init(scene_graph,
       scene_state,
       std::move(manipulator),
       manipulator_reach,
       std::move(tool_positioner),
       tool_limits,
       tool_sample_resolution,
       std::move(solver_name));
}

RTPInvKin::RTPInvKin(const tesseract::scene_graph::SceneGraph& scene_graph,
                     const tesseract::scene_graph::SceneState& scene_state,
                     InverseKinematics::UPtr manipulator,
                     double manipulator_reach,
                     std::unique_ptr<ForwardKinematics> tool_positioner,
                     const Eigen::MatrixX2d& tool_sample_range,
                     const Eigen::VectorXd& tool_sample_resolution,
                     std::string solver_name)
{
  init(scene_graph,
       scene_state,
       std::move(manipulator),
       manipulator_reach,
       std::move(tool_positioner),
       tool_sample_range,
       tool_sample_resolution,
       std::move(solver_name));
}

RTPInvKin::RTPInvKin(const tesseract::scene_graph::SceneGraph& scene_graph,
                     const tesseract::scene_graph::SceneState& scene_state,
                     InverseKinematics::UPtr manipulator,
                     std::unique_ptr<ForwardKinematics> tool_positioner,
                     const Eigen::VectorXd& tool_sample_resolution,
                     std::string solver_name)
{
  if (tool_positioner == nullptr)
    throw std::runtime_error("Provided tool positioner is a nullptr");
  if (manipulator == nullptr)
    throw std::runtime_error("Provided manipulator is a nullptr");
  if (manipulator->getTipLinkNames().empty())
    throw std::runtime_error("Provided manipulator has no tip links");

  std::vector<std::string> joint_names = tool_positioner->getJointNames();
  auto s = static_cast<Eigen::Index>(joint_names.size());
  Eigen::MatrixX2d tool_limits;
  tool_limits.resize(s, 2);
  for (Eigen::Index i = 0; i < s; ++i)
  {
    const auto& name = joint_names[static_cast<std::size_t>(i)];
    auto joint = scene_graph.getJoint(name);
    if (joint == nullptr)
      throw std::runtime_error("Tool positioner joint '" + name + "' not found in scene graph");
    if (joint->limits == nullptr)
      throw std::runtime_error("Tool positioner joint '" + name + "' has no limits");
    tool_limits(i, 0) = joint->limits->lower;
    tool_limits(i, 1) = joint->limits->upper;
  }

  const double auto_reach =
      computeChainReachUpperBound(scene_graph, manipulator->getBaseLinkName(), manipulator->getTipLinkNames()[0]);

  init(scene_graph,
       scene_state,
       std::move(manipulator),
       auto_reach,
       std::move(tool_positioner),
       tool_limits,
       tool_sample_resolution,
       std::move(solver_name));
}

RTPInvKin::RTPInvKin(const tesseract::scene_graph::SceneGraph& scene_graph,
                     const tesseract::scene_graph::SceneState& scene_state,
                     InverseKinematics::UPtr manipulator,
                     std::unique_ptr<ForwardKinematics> tool_positioner,
                     const Eigen::MatrixX2d& tool_sample_range,
                     const Eigen::VectorXd& tool_sample_resolution,
                     std::string solver_name)
{
  if (manipulator == nullptr)
    throw std::runtime_error("Provided manipulator is a nullptr");
  if (manipulator->getTipLinkNames().empty())
    throw std::runtime_error("Provided manipulator has no tip links");

  const double auto_reach =
      computeChainReachUpperBound(scene_graph, manipulator->getBaseLinkName(), manipulator->getTipLinkNames()[0]);

  init(scene_graph,
       scene_state,
       std::move(manipulator),
       auto_reach,
       std::move(tool_positioner),
       tool_sample_range,
       tool_sample_resolution,
       std::move(solver_name));
}

void RTPInvKin::init(const tesseract::scene_graph::SceneGraph& scene_graph,
                     const tesseract::scene_graph::SceneState& scene_state,
                     InverseKinematics::UPtr manipulator,
                     double manipulator_reach,
                     std::unique_ptr<ForwardKinematics> tool_positioner,
                     const Eigen::MatrixX2d& tool_sample_range,
                     const Eigen::VectorXd& tool_sample_resolution,
                     std::string solver_name)
{
  if (solver_name.empty())
    throw std::runtime_error("Solver name must not be empty.");

  if (!scene_graph.getLink(scene_graph.getRoot()))
    throw std::runtime_error("The scene graph has an invalid root.");

  if (manipulator == nullptr)
    throw std::runtime_error("Provided manipulator is a nullptr");

  if (!(manipulator_reach > 0))
    throw std::runtime_error("Manipulator reach is not greater than zero");

  if (tool_positioner == nullptr)
    throw std::runtime_error("Provided tool positioner is a nullptr");

  if (tool_sample_resolution.size() != tool_positioner->numJoints())
    throw std::runtime_error("Tool sample resolution must be same size as tool positioner number of joints");

  for (long i = 0; i < tool_sample_resolution.size(); ++i)
  {
    if (!(tool_sample_resolution(i) > 0))
      throw std::runtime_error("Tool sample resolution is not greater than zero");
  }

  // Static offset from the manipulator tip to the tool positioner's base link (usually identity
  // because the tool chain begins at the manipulator tip, but allow any fixed transform).
  manip_tip_to_tool_base_ = scene_state.link_transforms.at(manipulator->getTipLinkNames()[0]).inverse() *
                            scene_state.link_transforms.at(tool_positioner->getBaseLinkName());

  solver_name_ = std::move(solver_name);
  manip_tip_link_ = manipulator->getTipLinkNames()[0];
  manip_inv_kin_ = std::move(manipulator);
  tool_tip_link_ = tool_positioner->getTipLinkNames()[0];
  tool_fwd_kin_ = std::move(tool_positioner);
  manip_reach_ = manipulator_reach;
  dof_ = manip_inv_kin_->numJoints() + tool_fwd_kin_->numJoints();

  // Joint order: manipulator first, tool second.
  joint_names_ = manip_inv_kin_->getJointNames();
  const auto& tool_joints = tool_fwd_kin_->getJointNames();
  joint_names_.insert(joint_names_.end(), tool_joints.begin(), tool_joints.end());

  auto tool_num_joints = static_cast<int>(tool_fwd_kin_->numJoints());
  dof_range_.reserve(static_cast<std::size_t>(tool_num_joints));
  for (int d = 0; d < tool_num_joints; ++d)
  {
    int cnt = static_cast<int>(std::ceil(std::abs(tool_sample_range(d, 1) - tool_sample_range(d, 0)) /
                                         tool_sample_resolution(d))) +
              1;
    dof_range_.emplace_back(
        Eigen::VectorXd::LinSpaced(cnt, tool_sample_range(d, 0), tool_sample_range(d, 1)));
  }
}

InverseKinematics::UPtr RTPInvKin::clone() const { return std::make_unique<RTPInvKin>(*this); }

RTPInvKin::RTPInvKin(const RTPInvKin& other) { *this = other; }

RTPInvKin& RTPInvKin::operator=(const RTPInvKin& other)
{
  if (this == &other)
    return *this;

  manip_inv_kin_ = other.manip_inv_kin_->clone();
  tool_fwd_kin_ = other.tool_fwd_kin_->clone();
  manip_tip_link_ = other.manip_tip_link_;
  tool_tip_link_ = other.tool_tip_link_;
  manip_reach_ = other.manip_reach_;
  joint_names_ = other.joint_names_;
  manip_tip_to_tool_base_ = other.manip_tip_to_tool_base_;
  dof_ = other.dof_;
  dof_range_ = other.dof_range_;
  solver_name_ = other.solver_name_;
  return *this;
}

void RTPInvKin::calcInvKinHelper(IKSolutions& solutions,
                                 const tesseract::common::TransformMap& tip_link_poses,
                                 const Eigen::Ref<const Eigen::VectorXd>& seed) const
{
  Eigen::VectorXd tool_pose(tool_fwd_kin_->numJoints());
  nested_ik(solutions, 0, dof_range_, tip_link_poses, tool_pose, seed);
}

void RTPInvKin::nested_ik(IKSolutions& solutions,
                          int loop_level,
                          const std::vector<Eigen::VectorXd>& dof_range,
                          const tesseract::common::TransformMap& tip_link_poses,
                          Eigen::VectorXd& tool_pose,
                          const Eigen::Ref<const Eigen::VectorXd>& seed) const
{
  if (loop_level >= static_cast<int>(tool_fwd_kin_->numJoints()))
  {
    ikAt(solutions, tip_link_poses, tool_pose, seed);
    return;
  }

  for (long i = 0; i < static_cast<long>(dof_range[static_cast<std::size_t>(loop_level)].size()); ++i)
  {
    tool_pose(loop_level) = dof_range[static_cast<std::size_t>(loop_level)][i];
    nested_ik(solutions, loop_level + 1, dof_range, tip_link_poses, tool_pose, seed);
  }
}

void RTPInvKin::ikAt(IKSolutions& solutions,
                     const tesseract::common::TransformMap& tip_link_poses,
                     Eigen::VectorXd& tool_pose,
                     const Eigen::Ref<const Eigen::VectorXd>& seed) const
{
  TESSERACT_THREAD_LOCAL tesseract::common::TransformMap tool_poses;
  tool_poses.clear();
  tool_fwd_kin_->calcFwdKin(tool_poses, tool_pose);
  Eigen::Isometry3d tool_tf = tool_poses[tool_tip_link_];  // in tool_base frame

  // Target tool-tip pose (in manip base frame, since working_frame == manip base)
  // maps to a manip-tip target via the inverse of the tool-chain pose:
  //   T_manip_tip = T_target_tool_tip * (T_manip_tip_to_tool_base * T_tool_base_to_tool_tip(q_tool))^-1
  Eigen::Isometry3d robot_target_pose =
      tip_link_poses.at(tool_tip_link_) * (manip_tip_to_tool_base_ * tool_tf).inverse();

  if (robot_target_pose.translation().norm() > manip_reach_)
    return;

  tesseract::common::TransformMap robot_target_poses;
  robot_target_poses[manip_tip_link_] = robot_target_pose;

  auto robot_dof = manip_inv_kin_->numJoints();
  auto tool_dof = static_cast<Eigen::Index>(tool_pose.size());

  TESSERACT_THREAD_LOCAL IKSolutions robot_solution_set;
  robot_solution_set.clear();
  manip_inv_kin_->calcInvKin(robot_solution_set, robot_target_poses, seed.head(robot_dof));
  if (robot_solution_set.empty())
    return;

  for (const auto& robot_solution : robot_solution_set)
  {
    Eigen::VectorXd full_sol;
    full_sol.resize(robot_dof + tool_dof);
    full_sol.head(robot_dof) = robot_solution;
    full_sol.tail(tool_dof) = tool_pose;
    solutions.push_back(full_sol);
  }
}

void RTPInvKin::calcInvKin(IKSolutions& solutions,
                           const tesseract::common::TransformMap& tip_link_poses,
                           const Eigen::Ref<const Eigen::VectorXd>& seed) const
{
  assert(tip_link_poses.find(tool_tip_link_) != tip_link_poses.end());                      // NOLINT
  assert(std::abs(1.0 - tip_link_poses.at(tool_tip_link_).matrix().determinant()) < 1e-6);  // NOLINT

  return calcInvKinHelper(solutions, tip_link_poses, seed);  // NOLINT
}

std::vector<std::string> RTPInvKin::getJointNames() const { return joint_names_; }

Eigen::Index RTPInvKin::numJoints() const { return dof_; }

std::string RTPInvKin::getBaseLinkName() const { return manip_inv_kin_->getBaseLinkName(); }

std::string RTPInvKin::getWorkingFrame() const { return manip_inv_kin_->getBaseLinkName(); }

std::vector<std::string> RTPInvKin::getTipLinkNames() const { return { tool_tip_link_ }; }

std::string RTPInvKin::getSolverName() const { return solver_name_; }

}  // namespace tesseract::kinematics
