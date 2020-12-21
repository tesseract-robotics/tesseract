/**
 * @file commands.h
 * @brief This contains classes for recording operations applied to the environment
 *        for tracking changes. This is mainly support distributed systems to keep
 *        multiple instances up-to-date.
 *
 * @author Levi Armstrong
 * @date Dec 18, 2017
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
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
#ifndef TESSERACT_ENVIRONMENT_COMMANDS_H
#define TESSERACT_ENVIRONMENT_COMMANDS_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_scene_graph/joint.h>
#include <tesseract_scene_graph/link.h>
#include <tesseract_scene_graph/graph.h>
#include <tesseract_scene_graph/kinematics_information.h>
#include <tesseract_collision/core/types.h>

#ifdef SWIG

%shared_ptr(tesseract_environment::Command)
%shared_ptr(tesseract_environment::AddCommand)
%shared_ptr(tesseract_environment::MoveLinkCommand)
%shared_ptr(tesseract_environment::MoveJointCommand)
%shared_ptr(tesseract_environment::RemoveLinkCommand)
%shared_ptr(tesseract_environment::RemoveJointCommand)
%shared_ptr(tesseract_environment::ChangeLinkOriginCommand)
%shared_ptr(tesseract_environment::ChangeJointOriginCommand)
%shared_ptr(tesseract_environment::ChangeLinkCollisionEnabledCommand)
%shared_ptr(tesseract_environment::ChangeLinkVisibilityCommand)
%shared_ptr(tesseract_environment::AddAllowedCollisionCommand)
%shared_ptr(tesseract_environment::RemoveAllowedCollisionCommand)
%shared_ptr(tesseract_environment::RemoveAllowedCollisionLinkCommand)
%shared_ptr(tesseract_environment::AddSceneGraphCommand)
%shared_ptr(tesseract_environment::AddSceneGraphCommand)
%shared_ptr(tesseract_environment::AddKinematicsInformationCommand)
%shared_ptr(tesseract_environment::ChangeJointPositionLimitsCommand)
%shared_ptr(tesseract_environment::ChangeJointVelocityLimitsCommand)
%shared_ptr(tesseract_environment::ChangeJointAccelerationLimitsCommand)
%shared_ptr(tesseract_environment::ChangeDefaultContactMarginCommand)
%shared_ptr(tesseract_environment::ChangePairContactMarginCommand)

%shared_factory(
  tesseract_environment::Command,
  tesseract_environment::AddCommand,
	tesseract_environment::MoveLinkCommand,
	tesseract_environment::MoveJointCommand,
	tesseract_environment::RemoveLinkCommand,
	tesseract_environment::RemoveJointCommand,
	tesseract_environment::ChangeLinkOriginCommand,
	tesseract_environment::ChangeJointOriginCommand,
	tesseract_environment::ChangeLinkCollisionEnabledCommand,
	tesseract_environment::ChangeLinkVisibilityCommand,
	tesseract_environment::AddAllowedCollisionCommand,
  tesseract_environment::RemoveAllowedCollisionCommand,
  tesseract_environment::RemoveAllowedCollisionLinkCommand,
  tesseract_environment::AddSceneGraphCommand,
  tesseract_environment::AddKinematicsInformationCommand,
  tesseract_environment::ChangeJointPositionLimitsCommand,
  tesseract_environment::ChangeJointVelocityLimitsCommand,
  tesseract_environment::ChangeJointAccelerationLimitsCommand,
  tesseract_environment::ChangeDefaultContactMarginCommand,
  tesseract_environment::ChangePairContactMarginCommand
)

%template(Commands) std::vector<tesseract_environment::Command::ConstPtr>;

#endif  // SWIG

namespace tesseract_environment
{
enum class CommandType
{
  ADD = 0,
  MOVE_LINK = 1,
  MOVE_JOINT = 2,
  REMOVE_LINK = 3,
  REMOVE_JOINT = 4,
  CHANGE_LINK_ORIGIN = 5,
  CHANGE_JOINT_ORIGIN = 6,
  CHANGE_LINK_COLLISION_ENABLED = 7,
  CHANGE_LINK_VISIBILITY = 8,
  ADD_ALLOWED_COLLISION = 9,
  REMOVE_ALLOWED_COLLISION = 10,
  REMOVE_ALLOWED_COLLISION_LINK = 11,
  ADD_SCENE_GRAPH = 12,
  CHANGE_JOINT_POSITION_LIMITS = 13,
  CHANGE_JOINT_VELOCITY_LIMITS = 14,
  CHANGE_JOINT_ACCELERATION_LIMITS = 15,
  ADD_KINEMATICS_INFORMATION = 16,
  CHANGE_DEFAULT_CONTACT_MARGIN = 17,
  CHANGE_PAIR_CONTACT_MARGIN = 18
};

class Command
{
public:
  using Ptr = std::shared_ptr<Command>;
  using ConstPtr = std::shared_ptr<const Command>;

  explicit Command(CommandType type) : type_(type) {}
  virtual ~Command() = default;
  Command(const Command&) = delete;
  Command& operator=(const Command&) = delete;
  Command(Command&&) = delete;
  Command& operator=(Command&&) = delete;

  CommandType getType() const { return type_; }

private:
  /** \brief The type of the shape */
  CommandType type_;
};

using Commands = std::vector<Command::ConstPtr>;

class AddCommand : public Command
{
public:
  AddCommand(tesseract_scene_graph::Link::ConstPtr link, tesseract_scene_graph::Joint::ConstPtr joint)
    : Command(CommandType::ADD), link_(std::move(link)), joint_(std::move(joint))
  {
  }

  const tesseract_scene_graph::Link::ConstPtr& getLink() const { return link_; }
  const tesseract_scene_graph::Joint::ConstPtr& getJoint() const { return joint_; }

private:
  tesseract_scene_graph::Link::ConstPtr link_;
  tesseract_scene_graph::Joint::ConstPtr joint_;
};

class MoveLinkCommand : public Command
{
public:
  MoveLinkCommand(tesseract_scene_graph::Joint::ConstPtr joint)
    : Command(CommandType::MOVE_LINK), joint_(std::move(joint))
  {
  }

  const tesseract_scene_graph::Joint::ConstPtr& getJoint() const { return joint_; }

private:
  tesseract_scene_graph::Joint::ConstPtr joint_;
};

class MoveJointCommand : public Command
{
public:
  MoveJointCommand(std::string joint_name, std::string parent_link)
    : Command(CommandType::MOVE_JOINT), joint_name_(std::move(joint_name)), parent_link_(std::move(parent_link))
  {
  }

  const std::string& getJointName() const { return joint_name_; }
  const std::string& getParentLink() const { return parent_link_; }

private:
  std::string joint_name_;
  std::string parent_link_;
};

class RemoveLinkCommand : public Command
{
public:
  RemoveLinkCommand(std::string link_name) : Command(CommandType::REMOVE_LINK), link_name_(std::move(link_name)) {}

  const std::string& getLinkName() const { return link_name_; }

private:
  std::string link_name_;
};

class RemoveJointCommand : public Command
{
public:
  RemoveJointCommand(std::string joint_name) : Command(CommandType::REMOVE_JOINT), joint_name_(std::move(joint_name)) {}

  const std::string& getJointName() const { return joint_name_; }

private:
  std::string joint_name_;
};

class ChangeLinkOriginCommand : public Command
{
public:
  ChangeLinkOriginCommand(std::string link_name, const Eigen::Isometry3d& origin)
    : Command(CommandType::CHANGE_LINK_ORIGIN), link_name_(std::move(link_name)), origin_(origin)
  {
  }

  const std::string& getLinkName() const { return link_name_; }
  const Eigen::Isometry3d& getOrigin() const { return origin_; }

private:
  std::string link_name_;
  Eigen::Isometry3d origin_;
};

class ChangeJointOriginCommand : public Command
{
public:
  ChangeJointOriginCommand(std::string joint_name, const Eigen::Isometry3d& origin)
    : Command(CommandType::CHANGE_JOINT_ORIGIN), joint_name_(std::move(joint_name)), origin_(origin)
  {
  }

  const std::string& getJointName() const { return joint_name_; }
  const Eigen::Isometry3d& getOrigin() const { return origin_; }

private:
  std::string joint_name_;
  Eigen::Isometry3d origin_;
};

class ChangeLinkCollisionEnabledCommand : public Command
{
public:
  ChangeLinkCollisionEnabledCommand(std::string link_name, bool enabled)
    : Command(CommandType::CHANGE_LINK_COLLISION_ENABLED), link_name_(std::move(link_name)), enabled_(enabled)
  {
  }

  const std::string& getLinkName() const { return link_name_; }
  bool getEnabled() const { return enabled_; }

private:
  std::string link_name_;
  bool enabled_;
};

class ChangeLinkVisibilityCommand : public Command
{
public:
  ChangeLinkVisibilityCommand(std::string link_name, bool enabled)
    : Command(CommandType::CHANGE_LINK_VISIBILITY), link_name_(std::move(link_name)), enabled_(enabled)
  {
  }

  const std::string& getLinkName() const { return link_name_; }
  bool getEnabled() const { return enabled_; }

private:
  std::string link_name_;
  bool enabled_;
};

class AddAllowedCollisionCommand : public Command
{
public:
  AddAllowedCollisionCommand(std::string link_name1, std::string link_name2, std::string reason)
    : Command(CommandType::ADD_ALLOWED_COLLISION)
    , link_name1_(std::move(link_name1))
    , link_name2_(std::move(link_name2))
    , reason_(std::move(reason))
  {
  }

  const std::string& getLinkName1() const { return link_name1_; }
  const std::string& getLinkName2() const { return link_name2_; }
  const std::string& getReason() const { return reason_; }

private:
  std::string link_name1_;
  std::string link_name2_;
  std::string reason_;
};

class RemoveAllowedCollisionCommand : public Command
{
public:
  RemoveAllowedCollisionCommand(std::string link_name1, std::string link_name2)
    : Command(CommandType::REMOVE_ALLOWED_COLLISION)
    , link_name1_(std::move(link_name1))
    , link_name2_(std::move(link_name2))
  {
  }

  const std::string& getLinkName1() const { return link_name1_; }
  const std::string& getLinkName2() const { return link_name2_; }

private:
  std::string link_name1_;
  std::string link_name2_;
};

class RemoveAllowedCollisionLinkCommand : public Command
{
public:
  RemoveAllowedCollisionLinkCommand(std::string link_name)
    : Command(CommandType::REMOVE_ALLOWED_COLLISION_LINK), link_name_(std::move(link_name))
  {
  }

  const std::string& getLinkName() const { return link_name_; }

private:
  std::string link_name_;
};

class AddSceneGraphCommand : public Command
{
public:
  AddSceneGraphCommand(const tesseract_scene_graph::SceneGraph& scene_graph,
                       tesseract_scene_graph::Joint::ConstPtr joint,
                       std::string prefix)
    : Command(CommandType::ADD_SCENE_GRAPH)
    , scene_graph_(scene_graph.clone())
    , joint_(std::move(joint))
    , prefix_(std::move(prefix))
  {
  }

  const tesseract_scene_graph::SceneGraph::ConstPtr& getSceneGraph() const { return scene_graph_; }
  const tesseract_scene_graph::Joint::ConstPtr& getJoint() const { return joint_; }
  const std::string& getPrefix() const { return prefix_; }

private:
  tesseract_scene_graph::SceneGraph::ConstPtr scene_graph_;
  tesseract_scene_graph::Joint::ConstPtr joint_;
  std::string prefix_;
};

class AddKinematicsInformationCommand : public Command
{
public:
  AddKinematicsInformationCommand(tesseract_scene_graph::KinematicsInformation kinematics_information)
    : Command(CommandType::ADD_KINEMATICS_INFORMATION), kinematics_information_(std::move(kinematics_information))
  {
  }

  const tesseract_scene_graph::KinematicsInformation& getKinematicsInformation() const
  {
    return kinematics_information_;
  }

private:
  tesseract_scene_graph::KinematicsInformation kinematics_information_;
};

class ChangeJointPositionLimitsCommand : public Command
{
public:
  ChangeJointPositionLimitsCommand(std::string joint_name, double lower, double upper)
    : Command(CommandType::CHANGE_JOINT_POSITION_LIMITS)
    , limits_({ std::make_pair(joint_name, std::make_pair(lower, upper)) })
  {
    assert(upper > lower);
  }

  ChangeJointPositionLimitsCommand(std::unordered_map<std::string, std::pair<double, double>> limits)
    : Command(CommandType::CHANGE_JOINT_POSITION_LIMITS), limits_(std::move(limits))
  {
    assert(std::all_of(limits_.begin(), limits_.end(), [](const auto& p) { return p.second.second > p.second.first; }));
  }

  const std::unordered_map<std::string, std::pair<double, double>>& getLimits() const { return limits_; }

private:
  std::unordered_map<std::string, std::pair<double, double>> limits_;
};

class ChangeJointVelocityLimitsCommand : public Command
{
public:
  ChangeJointVelocityLimitsCommand(std::string joint_name, double limit)
    : Command(CommandType::CHANGE_JOINT_VELOCITY_LIMITS), limits_({ std::make_pair(joint_name, limit) })
  {
    assert(limit > 0);
  }

  ChangeJointVelocityLimitsCommand(std::unordered_map<std::string, double> limits)
    : Command(CommandType::CHANGE_JOINT_VELOCITY_LIMITS), limits_(std::move(limits))
  {
    assert(std::all_of(limits_.begin(), limits_.end(), [](const auto& p) { return p.second > 0; }));
  }

  const std::unordered_map<std::string, double>& getLimits() const { return limits_; }

private:
  std::unordered_map<std::string, double> limits_;
};

class ChangeJointAccelerationLimitsCommand : public Command
{
public:
  ChangeJointAccelerationLimitsCommand(std::string joint_name, double limit)
    : Command(CommandType::CHANGE_JOINT_ACCELERATION_LIMITS), limits_({ std::make_pair(joint_name, limit) })
  {
    assert(limit > 0);
  }

  ChangeJointAccelerationLimitsCommand(std::unordered_map<std::string, double> limits)
    : Command(CommandType::CHANGE_JOINT_ACCELERATION_LIMITS), limits_(std::move(limits))
  {
    assert(std::all_of(limits_.begin(), limits_.end(), [](const auto& p) { return p.second > 0; }));
  }

  const std::unordered_map<std::string, double>& getLimits() const { return limits_; }

private:
  std::unordered_map<std::string, double> limits_;
};

class ChangeDefaultContactMarginCommand : public Command
{
public:
  ChangeDefaultContactMarginCommand(double default_margin)
    : Command(CommandType::CHANGE_DEFAULT_CONTACT_MARGIN), default_margin_(default_margin)
  {
  }

  double getDefaultCollisionMargin() const { return default_margin_; }

private:
  double default_margin_{ 0 };
};

class ChangePairContactMarginCommand : public Command
{
public:
  ChangePairContactMarginCommand(
      std::unordered_map<tesseract_common::LinkNamesPair, double, tesseract_common::PairHash> link_pair_margin)
    : Command(CommandType::CHANGE_PAIR_CONTACT_MARGIN), link_pair_margin_(std::move(link_pair_margin))
  {
  }

  const std::unordered_map<tesseract_common::LinkNamesPair, double, tesseract_common::PairHash>&
  getPairCollisionMarginData() const
  {
    return link_pair_margin_;
  }

private:
  std::unordered_map<tesseract_common::LinkNamesPair, double, tesseract_common::PairHash> link_pair_margin_;
};

}  // namespace tesseract_environment

#endif  // TESSERACT_ENVIRONMENT_COMMANDS_H
