/**
 * @file change_contact_margins_command.h
 * @brief Used to change contact margin data in environment
 *
 * @author Levi Armstrong
 * @date March 15, 2021
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2021, Southwest Research Institute
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
#ifndef TESSERACT_ENVIRONMENT_CHANGE_COLLISION_MARGINS_COMMAND_H
#define TESSERACT_ENVIRONMENT_CHANGE_COLLISION_MARGINS_COMMAND_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
#include <unordered_map>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/core/command.h>
#include <tesseract_common/collision_margin_data.h>

namespace tesseract_environment
{
using CollisionMarginData = tesseract_common::CollisionMarginData;
using CollisionMarginOverrideType = tesseract_common::CollisionMarginOverrideType;
using PairsCollisionMarginData = tesseract_common::PairsCollisionMarginData;

class ChangeCollisionMarginsCommand : public Command
{
public:
  using Ptr = std::shared_ptr<ChangeCollisionMarginsCommand>;
  using ConstPtr = std::shared_ptr<const ChangeCollisionMarginsCommand>;

  ChangeCollisionMarginsCommand(
      double default_margin,
      CollisionMarginOverrideType override_type = CollisionMarginOverrideType::OVERRIDE_DEFAULT_MARGIN)
    : collision_margin_data_(CollisionMarginData(default_margin)), collision_margin_override_(override_type)
  {
  }

  ChangeCollisionMarginsCommand(
      PairsCollisionMarginData pairs_margin,
      CollisionMarginOverrideType override_type = CollisionMarginOverrideType::OVERRIDE_PAIR_MARGIN)
    : collision_margin_data_(CollisionMarginData(std::move(pairs_margin))), collision_margin_override_(override_type)
  {
  }

  ChangeCollisionMarginsCommand(CollisionMarginData collision_margin_data,
                                CollisionMarginOverrideType override_type = CollisionMarginOverrideType::REPLACE)
    : collision_margin_data_(std::move(collision_margin_data)), collision_margin_override_(override_type)
  {
  }

  CommandType getType() const final { return CommandType::CHANGE_COLLISION_MARGINS; }
  tesseract_common::CollisionMarginData getCollisionMarginData() const { return collision_margin_data_; }
  tesseract_common::CollisionMarginOverrideType getCollisionMarginOverrideType() const
  {
    return collision_margin_override_;
  }

private:
  CollisionMarginData collision_margin_data_;
  CollisionMarginOverrideType collision_margin_override_;
};
}  // namespace tesseract_environment

#endif  // TESSERACT_ENVIRONMENT_CHANGE_COLLISION_MARGINS_COMMAND_H
