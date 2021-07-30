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

#include <tesseract_environment/core/commands/add_allowed_collision_command.h>
#include <tesseract_environment/core/commands/add_link_command.h>
#include <tesseract_environment/core/commands/add_kinematics_information_command.h>
#include <tesseract_environment/core/commands/add_scene_graph_command.h>
#include <tesseract_environment/core/commands/change_joint_acceleration_limits_command.h>
#include <tesseract_environment/core/commands/change_joint_origin_command.h>
#include <tesseract_environment/core/commands/change_joint_position_limits_command.h>
#include <tesseract_environment/core/commands/change_joint_velocity_limits_command.h>
#include <tesseract_environment/core/commands/change_link_collision_enabled_command.h>
#include <tesseract_environment/core/commands/change_link_origin_command.h>
#include <tesseract_environment/core/commands/change_link_visibility_command.h>
#include <tesseract_environment/core/commands/move_joint_command.h>
#include <tesseract_environment/core/commands/move_link_command.h>
#include <tesseract_environment/core/commands/remove_allowed_collision_command.h>
#include <tesseract_environment/core/commands/remove_allowed_collision_link_command.h>
#include <tesseract_environment/core/commands/remove_joint_command.h>
#include <tesseract_environment/core/commands/remove_link_command.h>
#include <tesseract_environment/core/commands/replace_joint_command.h>
#include <tesseract_environment/core/commands/change_collision_margins_command.h>

#ifdef SWIG

%shared_ptr(tesseract_environment::Command)
%shared_ptr(tesseract_environment::AddLinkCommand)
%shared_ptr(tesseract_environment::MoveLinkCommand)
%shared_ptr(tesseract_environment::MoveJointCommand)
%shared_ptr(tesseract_environment::RemoveLinkCommand)
%shared_ptr(tesseract_environment::RemoveJointCommand)
%shared_ptr(tesseract_environment::ReplaceJointCommand)
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
%shared_ptr(tesseract_environment::ChangeCollisionMarginsCommand)

%shared_factory(
  tesseract_environment::Command,
  tesseract_environment::AddLinkCommand,
  tesseract_environment::MoveLinkCommand,
  tesseract_environment::MoveJointCommand,
  tesseract_environment::RemoveLinkCommand,
  tesseract_environment::RemoveJointCommand,
  tesseract_environment::ReplaceJointCommand,
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
  tesseract_environment::ChangeCollisionMarginsCommand
)

%template(Commands) std::vector<tesseract_environment::Command::ConstPtr>;

#endif  // SWIG

#endif  // TESSERACT_ENVIRONMENT_COMMANDS_H
