/**
 * @file remove_allowed_collision_command.h
 * @brief Used to remove all allowed collision for a given link from the environment
 *
 * @author Levi Armstrong
 * @date Dec 18, 2017
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
#ifndef TESSERACT_ENVIRONMENT_REMOVE_ALLOWED_COLLISION_LINK_COMMAND_H
#define TESSERACT_ENVIRONMENT_REMOVE_ALLOWED_COLLISION_LINK_COMMAND_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
#include <string>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/command.h>

namespace tesseract_environment
{
class RemoveAllowedCollisionLinkCommand;
template <class Archive>
void serialize(Archive& ar, RemoveAllowedCollisionLinkCommand& obj);

class RemoveAllowedCollisionLinkCommand : public Command
{
public:
  using Ptr = std::shared_ptr<RemoveAllowedCollisionLinkCommand>;
  using ConstPtr = std::shared_ptr<const RemoveAllowedCollisionLinkCommand>;

  RemoveAllowedCollisionLinkCommand();

  /**
   * @brief Remove disabled collision for any pair with link_name from allowed collision matrix
   * @param link_name Collision object name
   */
  RemoveAllowedCollisionLinkCommand(std::string link_name);

  const std::string& getLinkName() const;

  bool operator==(const RemoveAllowedCollisionLinkCommand& rhs) const;
  bool operator!=(const RemoveAllowedCollisionLinkCommand& rhs) const;

private:
  std::string link_name_;

  template <class Archive>
  friend void ::tesseract_environment::serialize(Archive& ar, RemoveAllowedCollisionLinkCommand& obj);
};
}  // namespace tesseract_environment

#endif  // TESSERACT_ENVIRONMENT_REMOVE_ALLOWED_COLLISION_LINK_COMMAND_H
