/**
 * @file rtp_inv_kin.cpp
 * @brief Robot with Tool Positioner Inverse kinematics implementation.
 *
 * @author Roelof Oomen
 * @date May 1, 2026
 *
 * @copyright Copyright (c) 2026
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

#include <cassert>
#include <tesseract/kinematics/rtp_inv_kin.h>
#include <tesseract/kinematics/utils.h>
#include <tesseract/kinematics/forward_kinematics.h>
#include <tesseract/scene_graph/graph.h>
#include <tesseract/scene_graph/joint.h>
#include <tesseract/scene_graph/scene_state.h>

namespace tesseract::kinematics
{
RTPInvKin::RTPInvKin(const tesseract::scene_graph::SceneGraph& scene_graph,
                     const tesseract::scene_graph::SceneState& scene_state,
                     InverseKinematics::UPtr manipulator,
                     double manipulator_reach,
                     std::unique_ptr<ForwardKinematics> tool_positioner,
                     const Eigen::VectorXd& tool_sample_resolution,
                     std::string solver_name)
{
  init(scene_graph,
       scene_state,
       std::move(manipulator),
       manipulator_reach,
       std::move(tool_positioner),
       std::nullopt,
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
  init(scene_graph,
       scene_state,
       std::move(manipulator),
       std::nullopt,
       std::move(tool_positioner),
       std::nullopt,
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
  init(scene_graph,
       scene_state,
       std::move(manipulator),
       std::nullopt,
       std::move(tool_positioner),
       tool_sample_range,
       tool_sample_resolution,
       std::move(solver_name));
}

void RTPInvKin::init(const tesseract::scene_graph::SceneGraph& scene_graph,
                     const tesseract::scene_graph::SceneState& scene_state,
                     InverseKinematics::UPtr manipulator,
                     std::optional<double> manipulator_reach,
                     std::unique_ptr<ForwardKinematics> tool_positioner,
                     const std::optional<Eigen::MatrixX2d>& tool_sample_range,
                     const Eigen::VectorXd& tool_sample_resolution,
                     std::string solver_name)
{
  // Upper bound on the combined sample count. buildSampleGrid() bounds each tool joint
  // individually; this bounds their product, which sets both the size of the tables built below
  // and the number of inner IK solves every calcInvKin() call performs. Not a modelling limit -
  // it turns a misconfigured discretisation into a diagnosable exception instead of a
  // multi-gigabyte allocation.
  constexpr std::size_t max_grid_samples = 1000000;

  if (tool_positioner == nullptr)
    throw std::runtime_error("Provided tool positioner is a nullptr");

  if (manipulator == nullptr)
    throw std::runtime_error("Provided manipulator is a nullptr");

  if (solver_name.empty())
    throw std::runtime_error("Solver name must not be empty.");

  if (!scene_graph.getLink(scene_graph.getRoot()))
    throw std::runtime_error("The scene graph has an invalid root.");

  if (manipulator->getTipLinkNames().size() != 1)
    throw std::runtime_error("RTPInvKin requires a manipulator with exactly one tip link");

  // The tool tip is the frame every target pose is interpreted in, so picking one of several
  // would silently change the meaning of the query.
  if (tool_positioner->getTipLinkNames().size() != 1)
    throw std::runtime_error("RTPInvKin requires a tool positioner with exactly one tip link");

  const std::string manip_tip = manipulator->getTipLinkNames()[0];
  const std::string tool_tip = tool_positioner->getTipLinkNames()[0];
  const std::string tool_base = tool_positioner->getBaseLinkName();

  // The graph and the state are independent parameters, so a state predating the tool chain has
  // these links in the graph but not in the transform map. Checked here so the static offset below
  // reports a runtime_error rather than letting map::at throw out_of_range.
  const auto require_in_state = [&scene_state](const std::string& link_name) {
    if (scene_state.link_transforms.find(link_name) == scene_state.link_transforms.end())
      throw std::runtime_error("Link '" + link_name + "' is not present in the provided scene state");
  };
  require_in_state(manip_tip);
  require_in_state(tool_base);

  // Derived from the manipulator's base->tip chain when the caller did not supply one.
  const double reach = manipulator_reach.has_value() ?
                           *manipulator_reach :
                           computeChainReachUpperBound(scene_graph, manipulator->getBaseLinkName(), manip_tip);

  if (!(reach > 0))
    throw std::runtime_error("Manipulator reach is not greater than zero");

  if (tool_sample_resolution.size() != tool_positioner->numJoints())
    throw std::runtime_error("Tool sample resolution must be same size as tool positioner number of joints");

  if (tool_sample_range.has_value() && tool_sample_range->rows() != tool_positioner->numJoints())
    throw std::runtime_error("Tool sample range must have one row per tool positioner joint");

  // buildSampleGrid owns the per-row contract: finite ordered bounds, finite positive resolution,
  // and a workable sample count. Built before any member is assigned, so a bad tool
  // discretisation is rejected up front alongside the checks above.
  const std::vector<Eigen::VectorXd> dof_range =
      buildSampleGrid(tool_sample_range.has_value() ? *tool_sample_range :
                                                      gatherJointLimits(scene_graph, tool_positioner->getJointNames()),
                      tool_sample_resolution);

  // The static-offset model below assumes the tool positioner's base is rigidly attached to the
  // manipulator tip. If an active joint sits between them, sampling the tool kinematics no longer
  // produces a deterministic wrist target and IK results would silently be wrong.
  if (manip_tip != tool_base)
  {
    if (scene_graph.getLink(tool_base) == nullptr)
      throw std::runtime_error("Tool positioner base link '" + tool_base + "' not found in scene graph");
    const auto path = scene_graph.getShortestPath(manip_tip, tool_base);
    if (path.links.size() < 2)
      throw std::runtime_error("Tool positioner base link '" + tool_base +
                               "' is not connected to manipulator tip link '" + manip_tip + "'");
    if (!path.active_joints.empty())
      throw std::runtime_error("Tool positioner base link '" + tool_base +
                               "' must be rigidly attached to manipulator tip link '" + manip_tip +
                               "'; found active joint '" + path.active_joints.front() + "' on path");
  }

  std::size_t grid_size = 1;
  for (const auto& samples : dof_range)
  {
    const auto count = static_cast<std::size_t>(samples.size());
    if (grid_size > max_grid_samples / count)
      throw std::runtime_error("Tool sample grid exceeds " + std::to_string(max_grid_samples) +
                               " combined samples; coarsen the tool sample resolution or narrow its range");
    grid_size *= count;
  }

  // Static offset from the manipulator tip to the tool positioner's base link (usually identity
  // because the tool chain begins at the manipulator tip, but allow any fixed transform).
  const Eigen::Isometry3d manip_tip_to_tool_base =
      scene_state.link_transforms.at(manip_tip).inverse() * scene_state.link_transforms.at(tool_base);

  // Each sample's transform is tool tip -> manipulator tip, i.e. relative, so it does not change
  // when the arm moves: the rigid-attachment check above makes manip_tip_to_tool_base invariant,
  // and the tool FK depends only on the tool joints. Only a change to the fixed manipulator-tip ->
  // tool-base mounting invalidates these, and that requires reconstructing the solver anyway.
  // Sample order matches a nested loop over dof_range with the last tool joint varying fastest.
  const Eigen::Index tool_dof = tool_positioner->numJoints();

  tool_samples_.resize(tool_dof, static_cast<Eigen::Index>(grid_size));
  sample_to_manip_tip_.resize(grid_size);

  tesseract::common::TransformMap tool_poses;
  std::vector<Eigen::Index> sample_index(static_cast<std::size_t>(tool_dof), 0);
  Eigen::VectorXd tool_pose(tool_dof);
  for (Eigen::Index d = 0; d < tool_dof; ++d)
    tool_pose(d) = dof_range[static_cast<std::size_t>(d)](0);

  for (std::size_t k = 0; k < grid_size; ++k)
  {
    tool_poses.clear();
    tool_positioner->calcFwdKin(tool_poses, tool_pose);

    tool_samples_.col(static_cast<Eigen::Index>(k)) = tool_pose;
    sample_to_manip_tip_[k] = (manip_tip_to_tool_base * tool_poses.at(tool_tip)).inverse();

    // Advance the odometer over the grid; the last tool joint varies fastest.
    for (Eigen::Index d = tool_dof - 1; d >= 0; --d)
    {
      const Eigen::VectorXd& samples = dof_range[static_cast<std::size_t>(d)];
      Eigen::Index& i = sample_index[static_cast<std::size_t>(d)];
      i = (i + 1 < samples.size()) ? i + 1 : 0;
      tool_pose(d) = samples(i);
      if (i != 0)
        break;
    }
  }

  solver_name_ = std::move(solver_name);
  manip_tip_link_ = manip_tip;
  tool_tip_link_ = tool_tip;
  manip_reach_ = reach;
  dof_ = manipulator->numJoints() + tool_dof;

  // Joint order: manipulator first, tool second.
  joint_names_ = manipulator->getJointNames();
  const std::vector<std::string> tool_joints = tool_positioner->getJointNames();
  joint_names_.insert(joint_names_.end(), tool_joints.begin(), tool_joints.end());

  manip_inv_kin_ = std::move(manipulator);
}

InverseKinematics::UPtr RTPInvKin::clone() const { return std::make_unique<RTPInvKin>(*this); }

RTPInvKin::RTPInvKin(const RTPInvKin& other)
  : joint_names_(other.joint_names_)
  , manip_inv_kin_(other.manip_inv_kin_->clone())
  , manip_tip_link_(other.manip_tip_link_)
  , tool_tip_link_(other.tool_tip_link_)
  , manip_reach_(other.manip_reach_)
  , dof_(other.dof_)
  , tool_samples_(other.tool_samples_)
  , sample_to_manip_tip_(other.sample_to_manip_tip_)
  , solver_name_(other.solver_name_)
{
}

RTPInvKin& RTPInvKin::operator=(const RTPInvKin& other)
{
  if (this == &other)
    return *this;

  joint_names_ = other.joint_names_;
  manip_inv_kin_ = other.manip_inv_kin_->clone();
  manip_tip_link_ = other.manip_tip_link_;
  tool_tip_link_ = other.tool_tip_link_;
  manip_reach_ = other.manip_reach_;
  dof_ = other.dof_;
  tool_samples_ = other.tool_samples_;
  sample_to_manip_tip_ = other.sample_to_manip_tip_;
  solver_name_ = other.solver_name_;
  return *this;
}

void RTPInvKin::calcInvKin(IKSolutions& solutions,
                           const tesseract::common::TransformMap& tip_link_poses,
                           const Eigen::Ref<const Eigen::VectorXd>& seed) const
{
  assert(tip_link_poses.find(tool_tip_link_) != tip_link_poses.end());                      // NOLINT
  assert(std::abs(1.0 - tip_link_poses.at(tool_tip_link_).matrix().determinant()) < 1e-6);  // NOLINT

  const Eigen::Isometry3d& target_tool_tip = tip_link_poses.at(tool_tip_link_);
  const Eigen::Index tool_dof = tool_samples_.rows();
  const Eigen::Index manip_dof = dof_ - tool_dof;
  const Eigen::Index sample_count = tool_samples_.cols();
  const double manip_reach_sq = manip_reach_ * manip_reach_;

  // One solution per sample is the common case; closed-form inner solvers return several and the
  // vector regrows, which only moves VectorXd handles. Reserving the OPW worst case instead would
  // over-allocate eightfold on every call for every other solver.
  solutions.reserve(static_cast<std::size_t>(sample_count));

  // Keyed once - the inner solver is handed the same map every sample, with only the value updated.
  tesseract::common::TransformMap manip_target_poses;
  Eigen::Isometry3d& manip_target = manip_target_poses[manip_tip_link_];
  const Eigen::Ref<const Eigen::VectorXd> manip_seed = seed.head(manip_dof);

  IKSolutions manip_solutions;
  for (Eigen::Index k = 0; k < sample_count; ++k)
  {
    // T_manip_tip = T_target_tool_tip * (T_manip_tip_to_tool_base * T_tool_base_to_tool_tip(q_k))^-1
    manip_target = target_tool_tip * sample_to_manip_tip_[static_cast<std::size_t>(k)];

    if (manip_target.translation().squaredNorm() > manip_reach_sq)
      continue;

    manip_solutions.clear();
    manip_inv_kin_->calcInvKin(manip_solutions, manip_target_poses, manip_seed);

    for (const auto& manip_solution : manip_solutions)
    {
      solutions.emplace_back(dof_);
      Eigen::VectorXd& full_sol = solutions.back();
      full_sol.head(manip_dof) = manip_solution;
      full_sol.tail(tool_dof) = tool_samples_.col(k);
    }
  }
}

std::vector<std::string> RTPInvKin::getJointNames() const { return joint_names_; }

Eigen::Index RTPInvKin::numJoints() const { return dof_; }

std::string RTPInvKin::getBaseLinkName() const { return manip_inv_kin_->getBaseLinkName(); }

std::string RTPInvKin::getWorkingFrame() const { return manip_inv_kin_->getBaseLinkName(); }

std::vector<std::string> RTPInvKin::getTipLinkNames() const { return { tool_tip_link_ }; }

std::string RTPInvKin::getSolverName() const { return solver_name_; }

}  // namespace tesseract::kinematics
