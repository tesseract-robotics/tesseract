/**
 * @file change_contact_margins_command.h
 * @brief Used to change contact margin data in environment
 *
 * @author Levi Armstrong
 * @date March 15, 2021
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
#include <optional>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/command.h>
#include <tesseract_common/collision_margin_data.h>

namespace tesseract::environment
{
using CollisionMarginPairData = tesseract::common::CollisionMarginPairData;
using CollisionMarginPairOverrideType = tesseract::common::CollisionMarginPairOverrideType;

class ChangeCollisionMarginsCommand;
template <class Archive>
void serialize(Archive& ar, ChangeCollisionMarginsCommand& obj);

class ChangeCollisionMarginsCommand : public Command
{
public:
  using Ptr = std::shared_ptr<ChangeCollisionMarginsCommand>;
  using ConstPtr = std::shared_ptr<const ChangeCollisionMarginsCommand>;

  ChangeCollisionMarginsCommand();

  ChangeCollisionMarginsCommand(double default_margin);

  ChangeCollisionMarginsCommand(
      CollisionMarginPairData pair_margins,
      CollisionMarginPairOverrideType pair_override_type = CollisionMarginPairOverrideType::MODIFY);

  ChangeCollisionMarginsCommand(
      double default_margin,
      CollisionMarginPairData pair_margins,
      CollisionMarginPairOverrideType pair_override_type = CollisionMarginPairOverrideType::MODIFY);

  std::optional<double> getDefaultCollisionMargin() const;
  CollisionMarginPairData getCollisionMarginPairData() const;
  CollisionMarginPairOverrideType getCollisionMarginPairOverrideType() const;

  bool operator==(const ChangeCollisionMarginsCommand& rhs) const;
  bool operator!=(const ChangeCollisionMarginsCommand& rhs) const;

private:
  std::optional<double> default_margin_;
  CollisionMarginPairData pair_margins_;
  CollisionMarginPairOverrideType pair_override_type_{ CollisionMarginPairOverrideType::NONE };

  template <class Archive>
  friend void ::tesseract::environment::serialize(Archive& ar, ChangeCollisionMarginsCommand& obj);
};
}  // namespace tesseract::environment

#endif  // TESSERACT_ENVIRONMENT_CHANGE_COLLISION_MARGINS_COMMAND_H
