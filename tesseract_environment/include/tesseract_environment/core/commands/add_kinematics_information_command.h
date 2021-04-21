/**
 * @file add_kinematics_information_command.h
 * @brief Used to kinematics information to the environment
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
#ifndef TESSERACT_ENVIRONMENT_ADD_KINEMATICS_INFORMATION_COMMAND_H
#define TESSERACT_ENVIRONMENT_ADD_KINEMATICS_INFORMATION_COMMAND_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/core/command.h>
#include <tesseract_srdf/kinematics_information.h>

namespace tesseract_environment
{
class AddKinematicsInformationCommand : public Command
{
public:
  using Ptr = std::shared_ptr<AddKinematicsInformationCommand>;
  using ConstPtr = std::shared_ptr<const AddKinematicsInformationCommand>;

  /**
   * @brief Add kinematics information to the environment
   * @param kin_info The kinematics information
   */
  AddKinematicsInformationCommand(tesseract_srdf::KinematicsInformation kinematics_information)
    : kinematics_information_(std::move(kinematics_information))
  {
  }

  CommandType getType() const final { return CommandType::ADD_KINEMATICS_INFORMATION; }
  const tesseract_srdf::KinematicsInformation& getKinematicsInformation() const { return kinematics_information_; }

private:
  tesseract_srdf::KinematicsInformation kinematics_information_;
};
}  // namespace tesseract_environment

#endif  // TESSERACT_ENVIRONMENT_ADD_KINEMATICS_INFORMATION_COMMAND_H
