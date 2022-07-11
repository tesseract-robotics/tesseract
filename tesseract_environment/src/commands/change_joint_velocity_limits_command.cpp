/**
 * @file change_joint_velocity_limits_command.cpp
 * @brief Used to change joint velocity limis in the environment
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
#include <boost/serialization/unordered_map.hpp>
#include <memory>
#include <string>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/utils.h>
#include <tesseract_environment/commands/change_joint_velocity_limits_command.h>

namespace tesseract_environment
{
ChangeJointVelocityLimitsCommand::ChangeJointVelocityLimitsCommand()
  : Command(CommandType::CHANGE_JOINT_VELOCITY_LIMITS){};

ChangeJointVelocityLimitsCommand::ChangeJointVelocityLimitsCommand(std::string joint_name, double limit)
  : Command(CommandType::CHANGE_JOINT_VELOCITY_LIMITS), limits_({ std::make_pair(std::move(joint_name), limit) })
{
  assert(limit > 0);
}

ChangeJointVelocityLimitsCommand::ChangeJointVelocityLimitsCommand(std::unordered_map<std::string, double> limits)
  : Command(CommandType::CHANGE_JOINT_VELOCITY_LIMITS), limits_(std::move(limits))
{
  assert(std::all_of(limits_.begin(), limits_.end(), [](const auto& p) { return p.second > 0; }));
}

const std::unordered_map<std::string, double>& ChangeJointVelocityLimitsCommand::getLimits() const { return limits_; }

bool ChangeJointVelocityLimitsCommand::operator==(const ChangeJointVelocityLimitsCommand& rhs) const
{
  bool equal = true;
  equal &= Command::operator==(rhs);
  equal &= tesseract_common::isIdenticalMap<std::unordered_map<std::string, double>, double>(limits_, rhs.limits_);
  return equal;
}
bool ChangeJointVelocityLimitsCommand::operator!=(const ChangeJointVelocityLimitsCommand& rhs) const
{
  return !operator==(rhs);
}

template <class Archive>
void ChangeJointVelocityLimitsCommand::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Command);
  ar& BOOST_SERIALIZATION_NVP(limits_);
}
}  // namespace tesseract_environment

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_environment::ChangeJointVelocityLimitsCommand)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_environment::ChangeJointVelocityLimitsCommand)
