/**
 * @file change_link_collision_enabled_command.h
 * @brief Used to change a links collision enabled in environment
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
#ifndef TESSERACT_ENVIRONMENT_CHANGE_LINK_COLLISION_ENABLED_COMMAND_H
#define TESSERACT_ENVIRONMENT_CHANGE_LINK_COLLISION_ENABLED_COMMAND_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/serialization/access.hpp>
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/command.h>

namespace tesseract_environment
{
class ChangeLinkCollisionEnabledCommand : public Command
{
public:
  using Ptr = std::shared_ptr<ChangeLinkCollisionEnabledCommand>;
  using ConstPtr = std::shared_ptr<const ChangeLinkCollisionEnabledCommand>;

  ChangeLinkCollisionEnabledCommand() : Command(CommandType::CHANGE_LINK_COLLISION_ENABLED){};

  /**
   * @brief Set whether a link should be considered during collision checking
   * @param link_name The link name to modify collision enabled
   * @param enabled True if should be condisdered during collision checking, otherwise false
   */
  ChangeLinkCollisionEnabledCommand(std::string link_name, bool enabled)
    : Command(CommandType::CHANGE_LINK_COLLISION_ENABLED), link_name_(std::move(link_name)), enabled_(enabled)
  {
  }

  const std::string& getLinkName() const { return link_name_; }
  bool getEnabled() const { return enabled_; }

  bool operator==(const ChangeLinkCollisionEnabledCommand& rhs) const;
  bool operator!=(const ChangeLinkCollisionEnabledCommand& rhs) const;

private:
  std::string link_name_;
  bool enabled_{ false };

  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};
}  // namespace tesseract_environment

#include <boost/serialization/export.hpp>
#include <boost/serialization/tracking.hpp>
BOOST_CLASS_EXPORT_KEY2(tesseract_environment::ChangeLinkCollisionEnabledCommand, "ChangeLinkCollisionEnabledCommand")
#endif  // TESSERACT_ENVIRONMENT_CHANGE_LINK_COLLISION_ENABLED_COMMAND_H
