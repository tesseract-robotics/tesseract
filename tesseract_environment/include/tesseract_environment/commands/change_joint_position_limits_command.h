/**
 * @file change_joint_position_limits_command.h
 * @brief Used to change a joints position limits in environment
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
#ifndef TESSERACT_ENVIRONMENT_CHANGE_JOINT_POSITION_LIMITS_COMMAND_H
#define TESSERACT_ENVIRONMENT_CHANGE_JOINT_POSITION_LIMITS_COMMAND_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/serialization/access.hpp>
#include <memory>
#include <unordered_map>
#include <cassert>
#include <algorithm>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/command.h>

namespace tesseract_environment
{
class ChangeJointPositionLimitsCommand : public Command
{
public:
  using Ptr = std::shared_ptr<ChangeJointPositionLimitsCommand>;
  using ConstPtr = std::shared_ptr<const ChangeJointPositionLimitsCommand>;

  ChangeJointPositionLimitsCommand();

  /**
   * @brief Changes the position limits associated with a joint
   * @param joint_name Name of the joint to be updated
   * @param limits New position limits to be set as the joint limits
   */
  ChangeJointPositionLimitsCommand(std::string joint_name, double lower, double upper);

  /**
   * @brief Changes the position limits associated with one or more joints
   * @param limits A map of joint names to new position limits.
   * For each limit pair, first is the lower limit second is the upper limit
   */
  ChangeJointPositionLimitsCommand(std::unordered_map<std::string, std::pair<double, double>> limits);

  const std::unordered_map<std::string, std::pair<double, double>>& getLimits() const;

  bool operator==(const ChangeJointPositionLimitsCommand& rhs) const;
  bool operator!=(const ChangeJointPositionLimitsCommand& rhs) const;

private:
  std::unordered_map<std::string, std::pair<double, double>> limits_;

  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};
}  // namespace tesseract_environment

#include <boost/serialization/export.hpp>
#include <boost/serialization/tracking.hpp>
BOOST_CLASS_EXPORT_KEY2(tesseract_environment::ChangeJointPositionLimitsCommand, "ChangeJointPositionLimitsCommand")
#endif  // TESSERACT_ENVIRONMENT_CHANGE_JOINT_POSITION_LIMITS_COMMAND_H
