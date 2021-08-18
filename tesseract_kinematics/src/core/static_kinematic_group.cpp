#include <tesseract_kinematics/core/static_kinematic_group.h>
#include <tesseract_common/utils.h>
#include <console_bridge/console.h>
#include <tesseract_scene_graph/kdl_parser.h>

namespace tesseract_kinematics
{
StaticKinematicGroup::StaticKinematicGroup(InverseKinematics::UPtr inv_kin,
                                           const tesseract_scene_graph::SceneGraph& scene_graph,
                                           const tesseract_scene_graph::SceneState& scene_state)
  : state_(scene_state), inv_kin_(std::move(inv_kin))
{
  for (const auto& joint_name : inv_kin_->getJointNames())
  {
    if (scene_graph.getJoint(joint_name) == nullptr)
      throw std::runtime_error("Inverse kinematic joint name '" + joint_name +
                               "' does not exist in the provided scene graph!");
  }

  name_ = inv_kin_->getName();
  joint_names_ = inv_kin_->getJointNames();
  tip_link_names_ = inv_kin_->getTipLinkNames();

  tesseract_scene_graph::KDLTreeData data =
      tesseract_scene_graph::parseSceneGraph(scene_graph, joint_names_, scene_state.joints);
  state_solver_ = std::make_unique<tesseract_scene_graph::KDLStateSolver>(scene_graph, data);

  std::vector<std::string> active_link_names = state_solver_->getActiveLinkNames();
  for (const auto& link : scene_graph.getLinks())
  {
    auto it = std::find(active_link_names.begin(), active_link_names.end(), link->getName());
    if (it == active_link_names.end())
      static_link_names_.push_back(link->getName());
  }

  bool static_added{ false };
  for (const auto& working_frame : inv_kin_->getWorkingFrames())
  {
    auto it = std::find(active_link_names.begin(), active_link_names.end(), working_frame);
    if (it == active_link_names.end() && !static_added)
    {
      static_added = true;
      working_frames_.insert(working_frames_.end(), static_link_names_.begin(), static_link_names_.end());
      inv_working_frames_map_[working_frame] = working_frame;
      for (const auto& static_link_name : static_link_names_)
        inv_working_frames_map_[static_link_name] = working_frame;
    }
    else
    {
      std::vector<std::string> child_link_names = scene_graph.getLinkChildrenNames(working_frame);
      working_frames_.push_back(working_frame);
      working_frames_.insert(working_frames_.end(), child_link_names.begin(), child_link_names.end());

      inv_working_frames_map_[working_frame] = working_frame;
      for (const auto& child : child_link_names)
        inv_working_frames_map_[child] = working_frame;
    }
  }

  inv_to_fwd_base_ = state_.link_transforms.at(inv_kin_->getBaseLinkName()).inverse() *
                     state_.link_transforms.at(state_solver_->getBaseLinkName());

  limits_.resize(static_cast<Eigen::Index>(joint_names_.size()));
  for (Eigen::Index i = 0; i < static_cast<Eigen::Index>(joint_names_.size()); ++i)
  {
    auto joint = scene_graph.getJoint(joint_names_[static_cast<std::size_t>(i)]);

    // Set limits
    limits_.joint_limits(i, 0) = joint->limits->lower;
    limits_.joint_limits(i, 1) = joint->limits->upper;
    limits_.velocity_limits(i) = joint->limits->velocity;
    limits_.acceleration_limits(i) = joint->limits->acceleration;

    // Set redundancy indices
    switch (joint->type)
    {
      case tesseract_scene_graph::JointType::REVOLUTE:
      case tesseract_scene_graph::JointType::CONTINUOUS:
        redundancy_indices_.push_back(static_cast<Eigen::Index>(i));
        break;
      default:
        break;
    }
  }
}

StaticKinematicGroup::StaticKinematicGroup(const StaticKinematicGroup& other) { *this = other; }

StaticKinematicGroup& StaticKinematicGroup::operator=(const StaticKinematicGroup& other)
{
  name_ = other.name_;
  state_ = other.state_;
  inv_kin_ = other.inv_kin_->clone();
  inv_to_fwd_base_ = other.inv_to_fwd_base_;
  joint_names_ = other.joint_names_;
  static_link_names_ = other.static_link_names_;
  working_frames_ = other.working_frames_;
  tip_link_names_ = other.tip_link_names_;
  limits_ = other.limits_;
  redundancy_indices_ = other.redundancy_indices_;
  inv_working_frames_map_ = other.inv_working_frames_map_;
  return *this;
}

tesseract_common::TransformMap
StaticKinematicGroup::calcFwdKin(const Eigen::Ref<const Eigen::VectorXd>& joint_angles) const
{
  return state_solver_->getState(joint_names_, joint_angles).link_transforms;
}

IKSolutions StaticKinematicGroup::calcInvKin(const Eigen::Isometry3d& pose,
                                             const std::string& working_frame,
                                             const std::string& tip_link_name,
                                             const Eigen::Ref<const Eigen::VectorXd>& seed) const
{
  assert(std::find(tip_link_names_.begin(), tip_link_names_.end(), tip_link_name) != tip_link_names_.end());
  assert(std::find(working_frames_.begin(), working_frames_.end(), working_frame) != working_frames_.end());

  std::string inv_working_frame = inv_working_frames_map_.at(working_frame);
  Eigen::Isometry3d inv_pose =
      state_.link_transforms.at(inv_working_frame).inverse() * state_.link_transforms.at(working_frame) * pose;
  IKSolutions solutions = inv_kin_->calcInvKin(inv_pose, inv_working_frame, tip_link_name, seed);

  IKSolutions solutions_filtered;
  solutions_filtered.reserve(solutions.size());
  for (auto& solution : solutions)
  {
    if (tesseract_common::satisfiesPositionLimits(solution, limits_.joint_limits))
      solutions_filtered.push_back(solution);
  }

  return solutions_filtered;
}

Eigen::MatrixXd StaticKinematicGroup::calcJacobian(const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
                                                   const std::string& link_name) const
{
  return state_solver_->getJacobian(joint_names_, joint_angles, link_name);
}

bool StaticKinematicGroup::checkJoints(const Eigen::Ref<const Eigen::VectorXd>& vec) const
{
  if (vec.size() != static_cast<Eigen::Index>(joint_names_.size()))
  {
    CONSOLE_BRIDGE_logError(
        "Number of joint angles (%d) don't match robot_model (%d)", static_cast<int>(vec.size()), numJoints());
    return false;
  }

  for (int i = 0; i < vec.size(); ++i)
  {
    if ((vec[i] < limits_.joint_limits(i, 0)) || (vec(i) > limits_.joint_limits(i, 1)))
    {
      CONSOLE_BRIDGE_logDebug("Joint %s is out-of-range (%g < %g < %g)",
                              joint_names_[static_cast<size_t>(i)].c_str(),
                              limits_.joint_limits(i, 0),
                              vec(i),
                              limits_.joint_limits(i, 1));
      return false;
    }
  }

  return true;
}

const std::vector<std::string>& StaticKinematicGroup::getJointNames() const { return joint_names_; }

const std::vector<std::string>& StaticKinematicGroup::getLinkNames() const { return state_solver_->getLinkNames(); }

const std::vector<std::string>& StaticKinematicGroup::getActiveLinkNames() const
{
  return state_solver_->getActiveLinkNames();
}

const tesseract_common::KinematicLimits& StaticKinematicGroup::getLimits() const { return limits_; }

void StaticKinematicGroup::setLimits(tesseract_common::KinematicLimits limits)
{
  Eigen::Index nj = numJoints();
  if (limits.joint_limits.rows() != nj || limits.velocity_limits.size() != nj ||
      limits.acceleration_limits.size() != nj)
    throw std::runtime_error("Kinematics Group limits assigned are invalid!");

  limits_ = limits;
}

const std::vector<Eigen::Index>& StaticKinematicGroup::getRedundancyCapableJointIndices() const
{
  return redundancy_indices_;
}

Eigen::Index StaticKinematicGroup::numJoints() const { return inv_kin_->numJoints(); }

const std::string& StaticKinematicGroup::getBaseLinkName() const { return state_solver_->getBaseLinkName(); }

const std::vector<std::string>& StaticKinematicGroup::getWorkingFrames() const { return working_frames_; }

const std::vector<std::string>& StaticKinematicGroup::getTipLinkNames() const { return tip_link_names_; }

const std::string& StaticKinematicGroup::getName() const { return name_; }

std::unique_ptr<KinematicGroup> StaticKinematicGroup::clone() const
{
  return std::make_unique<StaticKinematicGroup>(*this);
}
}  // namespace tesseract_kinematics
