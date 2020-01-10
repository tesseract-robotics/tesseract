/**
 * @file commands.i
 * @brief SWIG interface file for tesseract_environment/core/commands.h
 *
 * @author John Wason
 * @date December 10, 2019
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2019, Wason Technology, LLC
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

%{
#include <tesseract_environment/core/commands.h>
%}

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

%template(Commands) std::vector<tesseract_environment::Command::ConstPtr>;

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

  explicit Command(CommandType type);
  virtual ~Command();

  CommandType getType() const;
};

using Commands = std::vector<Command::ConstPtr>;

class AddCommand : public Command
{
public:
  AddCommand(tesseract_scene_graph::Link::ConstPtr link, tesseract_scene_graph::Joint::ConstPtr joint);

  const tesseract_scene_graph::Link::ConstPtr getLink() const;
  const tesseract_scene_graph::Joint::ConstPtr getJoint() const;
};

class MoveLinkCommand : public Command
{
public:
  MoveLinkCommand(tesseract_scene_graph::Joint::ConstPtr joint);

  const tesseract_scene_graph::Joint::ConstPtr getJoint() const;
};

class MoveJointCommand : public Command
{
public:
  MoveJointCommand(const std::string& joint_name, const std::string& parent_link);

  const std::string getJointName() const;
  const std::string getParentLink() const;
};

class RemoveLinkCommand : public Command
{
public:
  RemoveLinkCommand(const std::string& link_name);

  const std::string getLinkName() const;
};

class RemoveJointCommand : public Command
{
public:
  RemoveJointCommand(const std::string& joint_name);

  const std::string getJointName() const;

private:
  std::string joint_name_;
};

class ChangeLinkOriginCommand : public Command
{
public:
  ChangeLinkOriginCommand(const std::string& link_name, const Eigen::Isometry3d& origin);

  const std::string getLinkName() const;
  const Eigen::Isometry3d getOrigin() const;
};

class ChangeJointOriginCommand : public Command
{
public:
  ChangeJointOriginCommand(const std::string& joint_name, const Eigen::Isometry3d& origin);

  const std::string getJointName();
  const Eigen::Isometry3d getOrigin();
};

class ChangeLinkCollisionEnabledCommand : public Command
{
public:
  ChangeLinkCollisionEnabledCommand(const std::string& link_name, bool enabled);

  const std::string getLinkName() const;
  bool getEnabled() const;
};

class ChangeLinkVisibilityCommand : public Command
{
public:
  ChangeLinkVisibilityCommand(const std::string& link_name, bool enabled);

  const std::string getLinkName() const;
  bool getEnabled() const;
};

class AddAllowedCollisionCommand : public Command
{
public:
  AddAllowedCollisionCommand(const std::string& link_name1, const std::string& link_name2, const std::string& reason);

  const std::string getLinkName1() const;
  const std::string getLinkName2() const;
  const std::string getReason() const;
};

class RemoveAllowedCollisionCommand : public Command
{
public:
  RemoveAllowedCollisionCommand(const std::string& link_name1, const std::string& link_name2);

  const std::string getLinkName1() const;
  const std::string getLinkName2() const;
};

class RemoveAllowedCollisionLinkCommand : public Command
{
public:
  RemoveAllowedCollisionLinkCommand(const std::string& link_name);

  const std::string getLinkName() const;
};

}  // namespace tesseract_environment