/**
 * @file change_joint_position_limits_command.cpp
 * @brief Used to change joint position limis in the environment
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
#include <tesseract_environment/commands/change_joint_position_limits_command.h>

namespace tesseract_environment
{
ChangeJointPositionLimitsCommand::ChangeJointPositionLimitsCommand()
  : Command(CommandType::CHANGE_JOINT_POSITION_LIMITS){};

ChangeJointPositionLimitsCommand::ChangeJointPositionLimitsCommand(std::string joint_name, double lower, double upper)
  : Command(CommandType::CHANGE_JOINT_POSITION_LIMITS)
  , limits_({ std::make_pair(std::move(joint_name), std::make_pair(lower, upper)) })
{
  assert(upper > lower);
}

ChangeJointPositionLimitsCommand::ChangeJointPositionLimitsCommand(
    std::unordered_map<std::string, std::pair<double, double>> limits)
  : Command(CommandType::CHANGE_JOINT_POSITION_LIMITS), limits_(std::move(limits))
{
  assert(std::all_of(limits_.begin(), limits_.end(), [](const auto& p) { return p.second.second > p.second.first; }));
}

const std::unordered_map<std::string, std::pair<double, double>>& ChangeJointPositionLimitsCommand::getLimits() const
{
  return limits_;
}

bool ChangeJointPositionLimitsCommand::operator==(const ChangeJointPositionLimitsCommand& rhs) const
{
  auto fn = [](const std::pair<double, double>& p1, const std::pair<double, double>& p2) {
    return tesseract_common::almostEqualRelativeAndAbs(p1.first, p2.first) &&
           tesseract_common::almostEqualRelativeAndAbs(p1.second, p2.second);
  };
  bool equal = true;
  equal &= Command::operator==(rhs);
  equal &= tesseract_common::isIdenticalMap<std::unordered_map<std::string, std::pair<double, double>>,
                                            std::pair<double, double>>(limits_, rhs.limits_, fn);
  return equal;
}
bool ChangeJointPositionLimitsCommand::operator!=(const ChangeJointPositionLimitsCommand& rhs) const
{
  return !operator==(rhs);
}

template <class Archive>
void ChangeJointPositionLimitsCommand::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Command);
  ar& BOOST_SERIALIZATION_NVP(limits_);
}
}  // namespace tesseract_environment

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_environment::ChangeJointPositionLimitsCommand)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_environment::ChangeJointPositionLimitsCommand)
