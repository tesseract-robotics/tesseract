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
#include <boost/serialization/split_member.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/utils.h>
#include <tesseract_environment/commands/change_collision_margins_command.h>

namespace tesseract_environment
{
ChangeCollisionMarginsCommand::ChangeCollisionMarginsCommand() : Command(CommandType::CHANGE_COLLISION_MARGINS) {}

ChangeCollisionMarginsCommand::ChangeCollisionMarginsCommand(double default_margin)
  : Command(CommandType::CHANGE_COLLISION_MARGINS), default_margin_(default_margin)
{
}

ChangeCollisionMarginsCommand::ChangeCollisionMarginsCommand(CollisionMarginPairData pair_margins,
                                                             CollisionMarginPairOverrideType pair_override_type)
  : Command(CommandType::CHANGE_COLLISION_MARGINS)
  , pair_margins_(std::move(pair_margins))
  , pair_override_type_(pair_override_type)
{
}

ChangeCollisionMarginsCommand::ChangeCollisionMarginsCommand(double default_margin,
                                                             CollisionMarginPairData pair_margins,
                                                             CollisionMarginPairOverrideType pair_override_type)
  : Command(CommandType::CHANGE_COLLISION_MARGINS)
  , default_margin_(default_margin)
  , pair_margins_(std::move(pair_margins))
  , pair_override_type_(pair_override_type)
{
}

std::optional<double> ChangeCollisionMarginsCommand::getDefaultCollisionMargin() const { return default_margin_; }

tesseract_common::CollisionMarginPairData ChangeCollisionMarginsCommand::getCollisionMarginPairData() const
{
  return pair_margins_;
}

tesseract_common::CollisionMarginPairOverrideType
ChangeCollisionMarginsCommand::getCollisionMarginPairOverrideType() const
{
  return pair_override_type_;
}

bool ChangeCollisionMarginsCommand::operator==(const ChangeCollisionMarginsCommand& rhs) const
{
  bool equal = true;
  equal &= Command::operator==(rhs);
  equal &= default_margin_.has_value() == rhs.default_margin_.has_value();
  if (!equal)
    return equal;

  if (default_margin_.has_value() && rhs.default_margin_.has_value())
    equal &= tesseract_common::almostEqualRelativeAndAbs(default_margin_.value(), rhs.default_margin_.value());

  equal &= pair_margins_ == rhs.pair_margins_;
  equal &= pair_override_type_ == rhs.pair_override_type_;
  return equal;
}
bool ChangeCollisionMarginsCommand::operator!=(const ChangeCollisionMarginsCommand& rhs) const
{
  return !operator==(rhs);
}

template <class Archive>
void ChangeCollisionMarginsCommand::load(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Command);
  bool has_default_margin{ false };
  ar& boost::serialization::make_nvp("has_default_margin", has_default_margin);
  if (has_default_margin)
  {
    double default_margin{ 0 };
    ar& boost::serialization::make_nvp("default_margin", default_margin);
    default_margin_ = default_margin;
  }
  ar& BOOST_SERIALIZATION_NVP(pair_margins_);
  ar& BOOST_SERIALIZATION_NVP(pair_override_type_);
}

template <class Archive>
void ChangeCollisionMarginsCommand::save(Archive& ar, const unsigned int /*version*/) const
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Command);

  bool has_default_margin{ default_margin_.has_value() };
  double default_margin{ 0 };
  if (default_margin_.has_value())
    default_margin = default_margin_.value();

  ar& boost::serialization::make_nvp("has_default_margin", has_default_margin);
  ar& boost::serialization::make_nvp("default_margin", default_margin);
  ar& BOOST_SERIALIZATION_NVP(pair_margins_);
  ar& BOOST_SERIALIZATION_NVP(pair_override_type_);
}

template <class Archive>
void ChangeCollisionMarginsCommand::serialize(Archive& ar, const unsigned int version)
{
  boost::serialization::split_member(ar, *this, version);
}
}  // namespace tesseract_environment

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_environment::ChangeCollisionMarginsCommand)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_environment::ChangeCollisionMarginsCommand)
