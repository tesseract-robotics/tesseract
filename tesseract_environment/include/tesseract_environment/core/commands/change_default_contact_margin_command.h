/**
 * @file change_default_contact_margin_command.h
 * @brief Used to change default contact margin in environment
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
#ifndef TESSERACT_ENVIRONMENT_CHANGE_DEFAULT_CONTACT_MARGIN_COMMAND_H
#define TESSERACT_ENVIRONMENT_CHANGE_DEFAULT_CONTACT_MARGIN_COMMAND_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
#include <unordered_map>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/core/command.h>

namespace tesseract_environment
{
class ChangeDefaultContactMarginCommand : public Command
{
public:
  using Ptr = std::shared_ptr<ChangeDefaultContactMarginCommand>;
  using ConstPtr = std::shared_ptr<const ChangeDefaultContactMarginCommand>;

  ChangeDefaultContactMarginCommand(double default_margin) : default_margin_(default_margin) {}

  CommandType getType() const final { return CommandType::CHANGE_DEFAULT_CONTACT_MARGIN; }
  double getDefaultCollisionMargin() const { return default_margin_; }

private:
  double default_margin_{ 0 };
};
}  // namespace tesseract_environment

#endif  // TESSERACT_ENVIRONMENT_CHANGE_DEFAULT_CONTACT_MARGIN_COMMAND_H
