/**
 * @file modify_allowed_collision_command.cpp
 * @brief Used to modify an allowed collisions to the environment
 *
 * @author Levi Armstrong
 * @author Matthew Powelson
 * @date March 18, 2022
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

#include <tesseract_environment/commands/modify_allowed_collisions_command.h>

namespace tesseract::environment
{
ModifyAllowedCollisionsCommand::ModifyAllowedCollisionsCommand() : Command(CommandType::MODIFY_ALLOWED_COLLISIONS) {}

ModifyAllowedCollisionsCommand::ModifyAllowedCollisionsCommand(tesseract::common::AllowedCollisionMatrix acm,
                                                               ModifyAllowedCollisionsType type)
  : Command(CommandType::MODIFY_ALLOWED_COLLISIONS), type_(type), acm_(std::move(acm))
{
}

ModifyAllowedCollisionsType ModifyAllowedCollisionsCommand::getModifyType() const { return type_; }
const tesseract::common::AllowedCollisionMatrix& ModifyAllowedCollisionsCommand::getAllowedCollisionMatrix() const
{
  return acm_;
}

bool ModifyAllowedCollisionsCommand::operator==(const ModifyAllowedCollisionsCommand& rhs) const
{
  bool equal = true;
  equal &= Command::operator==(rhs);
  equal &= type_ == rhs.type_;
  equal &= acm_ == rhs.acm_;
  return equal;
}
bool ModifyAllowedCollisionsCommand::operator!=(const ModifyAllowedCollisionsCommand& rhs) const
{
  return !operator==(rhs);
}

}  // namespace tesseract::environment
