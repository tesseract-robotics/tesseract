/**
 * @file change_joint_acceleration_limits_command.cpp
 * @brief Used to change joint accelerations limis in the environment
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
#include <tesseract_environment/commands/change_joint_acceleration_limits_command.h>

namespace tesseract_environment
{
ChangeJointAccelerationLimitsCommand::ChangeJointAccelerationLimitsCommand()
  : Command(CommandType::CHANGE_JOINT_ACCELERATION_LIMITS){};

ChangeJointAccelerationLimitsCommand::ChangeJointAccelerationLimitsCommand(std::string joint_name, double limit)
  : Command(CommandType::CHANGE_JOINT_ACCELERATION_LIMITS), limits_({ std::make_pair(std::move(joint_name), limit) })
{
  assert(limit > 0);
}

ChangeJointAccelerationLimitsCommand::ChangeJointAccelerationLimitsCommand(
    std::unordered_map<std::string, double> limits)
  : Command(CommandType::CHANGE_JOINT_ACCELERATION_LIMITS), limits_(std::move(limits))
{
  assert(std::all_of(limits_.begin(), limits_.end(), [](const auto& p) { return p.second > 0; }));
}

const std::unordered_map<std::string, double>& ChangeJointAccelerationLimitsCommand::getLimits() const
{
  return limits_;
}

bool ChangeJointAccelerationLimitsCommand::operator==(const ChangeJointAccelerationLimitsCommand& rhs) const
{
  bool equal = true;
  equal &= Command::operator==(rhs);
  equal &= tesseract_common::isIdenticalMap<std::unordered_map<std::string, double>, double>(limits_, rhs.limits_);
  return equal;
}
bool ChangeJointAccelerationLimitsCommand::operator!=(const ChangeJointAccelerationLimitsCommand& rhs) const
{
  return !operator==(rhs);
}

template <class Archive>
void ChangeJointAccelerationLimitsCommand::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Command);
  ar& BOOST_SERIALIZATION_NVP(limits_);
}
}  // namespace tesseract_environment

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_environment::ChangeJointAccelerationLimitsCommand)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_environment::ChangeJointAccelerationLimitsCommand)
