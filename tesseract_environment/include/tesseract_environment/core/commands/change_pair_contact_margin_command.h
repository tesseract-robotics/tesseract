/**
 * @file change_pair_contact_margin_command.h
 * @brief Used to change pair contact margin in environment
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
#ifndef TESSERACT_ENVIRONMENT_CHANGE_PAIR_CONTACT_MARGIN_COMMAND_H
#define TESSERACT_ENVIRONMENT_CHANGE_PAIR_CONTACT_MARGIN_COMMAND_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
#include <unordered_map>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/core/command.h>
#include <tesseract_common/types.h>

namespace tesseract_environment
{
class ChangePairContactMarginCommand : public Command
{
public:
  using Ptr = std::shared_ptr<ChangePairContactMarginCommand>;
  using ConstPtr = std::shared_ptr<const ChangePairContactMarginCommand>;

  ChangePairContactMarginCommand(
      std::unordered_map<tesseract_common::LinkNamesPair, double, tesseract_common::PairHash> link_pair_margin)
    : link_pair_margin_(std::move(link_pair_margin))
  {
  }

  CommandType getType() const final { return CommandType::CHANGE_PAIR_CONTACT_MARGIN; }

  const std::unordered_map<tesseract_common::LinkNamesPair, double, tesseract_common::PairHash>&
  getPairCollisionMarginData() const
  {
    return link_pair_margin_;
  }

private:
  std::unordered_map<tesseract_common::LinkNamesPair, double, tesseract_common::PairHash> link_pair_margin_;
};
}  // namespace tesseract_environment

#endif  // TESSERACT_ENVIRONMENT_CHANGE_PAIR_CONTACT_MARGIN_COMMAND_H
