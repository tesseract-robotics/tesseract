/**
 * @file change_joint_velocity_limits_command.h
 * @brief Used to change a joints velocity limits in environment
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
#ifndef TESSERACT_ENVIRONMENT_CHANGE_JOINT_VELOCITY_LIMITS_COMMAND_H
#define TESSERACT_ENVIRONMENT_CHANGE_JOINT_VELOCITY_LIMITS_COMMAND_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
#include <unordered_map>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/command.h>

namespace tesseract_environment
{
class ChangeJointVelocityLimitsCommand : public Command
{
public:
  using Ptr = std::shared_ptr<ChangeJointVelocityLimitsCommand>;
  using ConstPtr = std::shared_ptr<const ChangeJointVelocityLimitsCommand>;

  /**
   * @brief Changes the velocity limits associated with a joint
   * @param joint_name Name of the joint to be updated
   * @param limits New velocity limits to be set as the joint limits
   */
  ChangeJointVelocityLimitsCommand(std::string joint_name, double limit)
    : limits_({ std::make_pair(std::move(joint_name), limit) })
  {
    assert(limit > 0);
  }

  /**
   * @brief Changes the velocity limits associated with a joint
   * @param limits A map of joint names to new velocity limits
   */
  ChangeJointVelocityLimitsCommand(std::unordered_map<std::string, double> limits) : limits_(std::move(limits))
  {
    assert(std::all_of(limits_.begin(), limits_.end(), [](const auto& p) { return p.second > 0; }));
  }

  CommandType getType() const final { return CommandType::CHANGE_JOINT_VELOCITY_LIMITS; }
  const std::unordered_map<std::string, double>& getLimits() const { return limits_; }

private:
  std::unordered_map<std::string, double> limits_;
};
}  // namespace tesseract_environment

#endif  // TESSERACT_ENVIRONMENT_CHANGE_JOINT_VELOCITY_LIMITS_COMMAND_H
