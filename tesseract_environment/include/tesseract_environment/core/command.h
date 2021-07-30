/**
 * @file command.h
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
#ifndef TESSERACT_ENVIRONMENT_COMMAND_H
#define TESSERACT_ENVIRONMENT_COMMAND_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
#include <vector>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_environment
{
enum class CommandType
{
  ADD_LINK = 0,
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
  REPLACE_JOINT = 17,
  CHANGE_COLLISION_MARGINS = 18
};

class Command
{
public:
  using Ptr = std::shared_ptr<Command>;
  using ConstPtr = std::shared_ptr<const Command>;

  Command() = default;
  virtual ~Command() = default;
  Command(const Command&) = default;
  Command& operator=(const Command&) = default;
  Command(Command&&) = default;
  Command& operator=(Command&&) = default;

  virtual CommandType getType() const = 0;
};

using Commands = std::vector<Command::ConstPtr>;
}  // namespace tesseract_environment

#endif  // COMMAND_H
