/**
 * @file change_collision_margins_command.cpp
 * @brief Used to change collision margins
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

#include <tesseract_common/utils.h>
#include <tesseract_environment/commands/change_collision_margins_command.h>

namespace tesseract_environment
{
ChangeCollisionMarginsCommand::ChangeCollisionMarginsCommand() : Command(CommandType::CHANGE_COLLISION_MARGINS){};

ChangeCollisionMarginsCommand::ChangeCollisionMarginsCommand(double default_margin,
                                                             CollisionMarginOverrideType override_type)
  : Command(CommandType::CHANGE_COLLISION_MARGINS)
  , collision_margin_data_(CollisionMarginData(default_margin))
  , collision_margin_override_(override_type)
{
}

ChangeCollisionMarginsCommand::ChangeCollisionMarginsCommand(PairsCollisionMarginData pairs_margin,
                                                             CollisionMarginOverrideType override_type)
  : Command(CommandType::CHANGE_COLLISION_MARGINS)
  , collision_margin_data_(CollisionMarginData(std::move(pairs_margin)))
  , collision_margin_override_(override_type)
{
}

ChangeCollisionMarginsCommand::ChangeCollisionMarginsCommand(CollisionMarginData collision_margin_data,
                                                             CollisionMarginOverrideType override_type)
  : Command(CommandType::CHANGE_COLLISION_MARGINS)
  , collision_margin_data_(std::move(collision_margin_data))
  , collision_margin_override_(override_type)
{
}

tesseract_common::CollisionMarginData ChangeCollisionMarginsCommand::getCollisionMarginData() const
{
  return collision_margin_data_;
}
tesseract_common::CollisionMarginOverrideType ChangeCollisionMarginsCommand::getCollisionMarginOverrideType() const
{
  return collision_margin_override_;
}

bool ChangeCollisionMarginsCommand::operator==(const ChangeCollisionMarginsCommand& rhs) const
{
  bool equal = true;
  equal &= Command::operator==(rhs);
  equal &= collision_margin_data_ == rhs.collision_margin_data_;
  equal &= collision_margin_override_ == rhs.collision_margin_override_;
  return equal;
}
bool ChangeCollisionMarginsCommand::operator!=(const ChangeCollisionMarginsCommand& rhs) const
{
  return !operator==(rhs);
}

template <class Archive>
void ChangeCollisionMarginsCommand::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Command);
  ar& BOOST_SERIALIZATION_NVP(collision_margin_data_);
  ar& BOOST_SERIALIZATION_NVP(collision_margin_override_);
}
}  // namespace tesseract_environment

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_environment::ChangeCollisionMarginsCommand)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_environment::ChangeCollisionMarginsCommand)
