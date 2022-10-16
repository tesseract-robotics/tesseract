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

#include <tesseract_environment/commands/add_contact_managers_plugin_info_command.h>
#include <tesseract_environment/commands/add_link_command.h>
#include <tesseract_environment/commands/add_kinematics_information_command.h>
#include <tesseract_environment/commands/add_scene_graph_command.h>
#include <tesseract_environment/commands/change_joint_acceleration_limits_command.h>
#include <tesseract_environment/commands/change_joint_origin_command.h>
#include <tesseract_environment/commands/change_joint_position_limits_command.h>
#include <tesseract_environment/commands/change_joint_velocity_limits_command.h>
#include <tesseract_environment/commands/change_link_collision_enabled_command.h>
#include <tesseract_environment/commands/change_link_origin_command.h>
#include <tesseract_environment/commands/change_link_visibility_command.h>
#include <tesseract_environment/commands/modify_allowed_collisions_command.h>
#include <tesseract_environment/commands/move_joint_command.h>
#include <tesseract_environment/commands/move_link_command.h>
#include <tesseract_environment/commands/remove_allowed_collision_link_command.h>
#include <tesseract_environment/commands/remove_joint_command.h>
#include <tesseract_environment/commands/remove_link_command.h>
#include <tesseract_environment/commands/replace_joint_command.h>
#include <tesseract_environment/commands/change_collision_margins_command.h>
#include <tesseract_environment/commands/set_active_continuous_contact_manager_command.h>
#include <tesseract_environment/commands/set_active_discrete_contact_manager_command.h>

#endif  // TESSERACT_ENVIRONMENT_COMMANDS_H
