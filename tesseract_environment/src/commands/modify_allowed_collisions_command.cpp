/**
 * @file modify_allowed_collision_command.cpp
 * @brief Used to modify an allowed collisions to the environment
 *
 * @author Levi Armstrong
 * @author Matthew Powelson
 * @date March 18, 2022
 * @version TODO
 * @bug No known bugs
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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/serialization/access.hpp>
#include <boost/serialization/nvp.hpp>
#include <memory>
#include <string>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/commands/modify_allowed_collisions_command.h>

namespace tesseract_environment
{
ModifyAllowedCollisionsCommand::ModifyAllowedCollisionsCommand() : Command(CommandType::MODIFY_ALLOWED_COLLISIONS) {}

ModifyAllowedCollisionsCommand::ModifyAllowedCollisionsCommand(tesseract_common::AllowedCollisionMatrix acm,
                                                               ModifyAllowedCollisionsType type)
  : Command(CommandType::MODIFY_ALLOWED_COLLISIONS), type_(type), acm_(std::move(acm))
{
}

ModifyAllowedCollisionsType ModifyAllowedCollisionsCommand::getModifyType() const { return type_; }
const tesseract_common::AllowedCollisionMatrix& ModifyAllowedCollisionsCommand::getAllowedCollisionMatrix() const
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

template <class Archive>
void ModifyAllowedCollisionsCommand::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Command);
  ar& BOOST_SERIALIZATION_NVP(type_);
  ar& BOOST_SERIALIZATION_NVP(acm_);
}
}  // namespace tesseract_environment

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_environment::ModifyAllowedCollisionsCommand)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_environment::ModifyAllowedCollisionsCommand)
