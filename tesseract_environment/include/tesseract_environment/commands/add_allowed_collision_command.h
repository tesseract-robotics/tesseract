/**
 * @file add_allowed_collision_command.h
 * @brief Used to add an allowed collision to the environment
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
#ifndef TESSERACT_ENVIRONMENT_ADD_ALLOWED_COLLISION_COMMAND_H
#define TESSERACT_ENVIRONMENT_ADD_ALLOWED_COLLISION_COMMAND_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
#include <string>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/command.h>

namespace tesseract_environment
{
class AddAllowedCollisionCommand : public Command
{
public:
  using Ptr = std::shared_ptr<AddAllowedCollisionCommand>;
  using ConstPtr = std::shared_ptr<const AddAllowedCollisionCommand>;

  /**
   * @brief Disable collision between two collision objects
   * @param link_name1 Collision object name
   * @param link_name2 Collision object name
   * @param reason The reason for disabling collison
   */
  AddAllowedCollisionCommand(std::string link_name1, std::string link_name2, std::string reason)
    : link_name1_(std::move(link_name1)), link_name2_(std::move(link_name2)), reason_(std::move(reason))
  {
  }

  CommandType getType() const final { return CommandType::ADD_ALLOWED_COLLISION; }
  const std::string& getLinkName1() const { return link_name1_; }
  const std::string& getLinkName2() const { return link_name2_; }
  const std::string& getReason() const { return reason_; }

private:
  std::string link_name1_;
  std::string link_name2_;
  std::string reason_;
};
}  // namespace tesseract_environment

#endif  // TESSERACT_ENVIRONMENT_ADD_ALLOWED_COLLISION_COMMAND_H
