#include <tesseract_kinematics/core/kinematic_group.h>
#include <tesseract_common/utils.h>
#include <console_bridge/console.h>

namespace tesseract_kinematics
{
KinematicGroup::KinematicGroup(ForwardKinematics::UPtr fwd_kin,
                               InverseKinematics::UPtr inv_kin,
                               const tesseract_scene_graph::SceneGraph& scene_graph,
                               const tesseract_common::TransformMap& state)
  : fwd_kin_(std::move(fwd_kin)), inv_kin_(std::move(inv_kin))
{
  if (fwd_kin_->numJoints() != inv_kin_->numJoints())
    throw std::runtime_error("Tried to synchronize kinematics objects with different number of joints!");

  if (!tesseract_common::isIdentical(fwd_kin_->getJointNames(), inv_kin_->getJointNames(), false))
    throw std::runtime_error("Tried to synchronize kinematics objects with different joint names!");

  name_ = fwd_kin_->getName();
  joint_names_ = fwd_kin_->getJointNames();
  joint_link_names_ = fwd_kin_->getJointLinkNames();
  base_link_name_ = fwd_kin_->getBaseLinkName();
  tip_link_names_ = fwd_kin_->getTipLinkNames();

  link_names_ = scene_graph.getLinkChildrenNames(base_link_name_);
  link_names_.insert(link_names_.begin(), base_link_name_);

  inv_to_fwd_base_ = state.at(inv_kin_->getBaseLinkName()).inverse() * state.at(fwd_kin_->getBaseLinkName());
  adjacency_map_ = std::make_unique<tesseract_scene_graph::AdjacencyMap>(scene_graph, joint_link_names_, state);

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

  std::vector<std::string> inv_kin_joint_names = inv_kin_->getJointNames();
  if (fwd_kin_->getJointNames() != inv_kin_joint_names)
  {
    for (std::size_t i = 0; i < inv_kin_joint_names.size(); ++i)
    {
      auto it = std::find(joint_names_.begin(), joint_names_.end(), inv_kin_joint_names[i]);
      Eigen::Index idx = std::distance(joint_names_.begin(), it);
      inv_joint_map_.push_back(idx);
    }
  }
}

KinematicGroup::KinematicGroup(const KinematicGroup& other) { *this = other; }

KinematicGroup& KinematicGroup::operator=(const KinematicGroup& other)
{
  name_ = other.name_;
  fwd_kin_ = other.fwd_kin_->clone();
  inv_kin_ = other.inv_kin_->clone();
  inv_to_fwd_base_ = other.inv_to_fwd_base_;
  inv_joint_map_ = other.inv_joint_map_;
  adjacency_map_ = std::make_unique<tesseract_scene_graph::AdjacencyMap>(*other.adjacency_map_);
  joint_names_ = other.joint_names_;
  joint_link_names_ = other.joint_link_names_;
  link_names_ = other.link_names_;
  base_link_name_ = other.base_link_name_;
  tip_link_names_ = other.tip_link_names_;
  limits_ = other.limits_;
  redundancy_indices_ = other.redundancy_indices_;
  return *this;
}

tesseract_common::TransformMap KinematicGroup::calcFwdKin(const Eigen::Ref<const Eigen::VectorXd>& joint_angles) const
{
  return fwd_kin_->calcFwdKin(joint_angles);
}

IKSolutions KinematicGroup::calcInvKin(const Eigen::Isometry3d& pose,
                                       const std::string& working_frame,
                                       const std::string& tip_link_name,
                                       const Eigen::Ref<const Eigen::VectorXd>& seed) const
{
  IKSolutions solutions;
  const std::vector<std::string>& aln = adjacency_map_->getActiveLinkNames();
  if (std::find(aln.begin(), aln.end(), working_frame) != aln.end())
  {
    const auto& pair = adjacency_map_->getLinkMapping(working_frame);
    solutions = inv_kin_->calcInvKin(pair.transform * pose, pair.link_name, tip_link_name, seed);
  }
  else if (inv_kin_->getBaseLinkName() == working_frame)
  {
    solutions = inv_kin_->calcInvKin(pose, working_frame, tip_link_name, seed);
  }
  else if (fwd_kin_->getBaseLinkName() == working_frame)
  {
    solutions = inv_kin_->calcInvKin(inv_to_fwd_base_ * pose, inv_kin_->getBaseLinkName(), tip_link_name, seed);
  }
  else
  {
    const auto& pair = adjacency_map_->getLinkMapping(working_frame);
    solutions = inv_kin_->calcInvKin(pair.transform * pose, pair.link_name, tip_link_name, seed);
  }

  IKSolutions solutions_filtered;
  solutions_filtered.reserve(solutions.size());
  for (auto& solution : solutions)
  {
    if (tesseract_common::satisfiesPositionLimits(solution, limits_.joint_limits))
    {
      if (!inv_joint_map_.empty())
        tesseract_common::reorder(solution, inv_joint_map_);

      solutions_filtered.push_back(solution);
    }
  }

  return solutions_filtered;
}

Eigen::MatrixXd KinematicGroup::calcJacobian(const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
                                             const std::string& link_name) const
{
  return fwd_kin_->calcJacobian(joint_angles, link_name);
}

bool KinematicGroup::checkJoints(const Eigen::Ref<const Eigen::VectorXd>& vec) const
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

const std::vector<std::string>& KinematicGroup::getJointNames() const { return joint_names_; }

const std::vector<std::string>& KinematicGroup::getLinkNames() const { return link_names_; }

const std::vector<std::string>& KinematicGroup::getActiveLinkNames() const
{
  return adjacency_map_->getActiveLinkNames();
}

const tesseract_common::KinematicLimits& KinematicGroup::getLimits() const { return limits_; }

void KinematicGroup::setLimits(tesseract_common::KinematicLimits limits)
{
  Eigen::Index nj = numJoints();
  if (limits.joint_limits.rows() != nj || limits.velocity_limits.size() != nj ||
      limits.acceleration_limits.size() != nj)
    throw std::runtime_error("Kinematics Group limits assigned are invalid!");

  limits_ = limits;
}

const std::vector<Eigen::Index>& KinematicGroup::getRedundancyCapableJointIndices() const
{
  return redundancy_indices_;
}

Eigen::Index KinematicGroup::numJoints() const { return fwd_kin_->numJoints(); }

const std::string& KinematicGroup::getBaseLinkName() const { return base_link_name_; }

const std::vector<std::string>& KinematicGroup::getTipLinkNames() const { return tip_link_names_; }

const std::string& KinematicGroup::getName() const { return name_; }

std::unique_ptr<KinematicGroup> KinematicGroup::clone() const { return std::make_unique<KinematicGroup>(*this); }
}  // namespace tesseract_kinematics
