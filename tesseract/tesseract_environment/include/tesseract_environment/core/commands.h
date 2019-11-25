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

#include <memory>
#include <tesseract_scene_graph/joint.h>
#include <tesseract_scene_graph/link.h>
#include <Eigen/Geometry>

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
  REMOVE_ALLOWED_COLLISION_LINK = 11
};

class Command
{
public:
  using Ptr = std::shared_ptr<Command>;
  using ConstPtr = std::shared_ptr<const Command>;

  explicit Command(CommandType type) : type_(type) {}
  virtual ~Command() = default;
  Command(const Command&) = default;
  Command& operator=(const Command&) = default;
  Command(Command&&) = default;
  Command& operator=(Command&&) = default;

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

}  // namespace tesseract_environment

#endif  // TESSERACT_ENVIRONMENT_COMMANDS_H
