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
#include <boost/serialization/access.hpp>
#include <memory>
#include <unordered_map>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/command.h>
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

  ChangeCollisionMarginsCommand();

  ChangeCollisionMarginsCommand(
      double default_margin,
      CollisionMarginOverrideType override_type = CollisionMarginOverrideType::OVERRIDE_DEFAULT_MARGIN);

  ChangeCollisionMarginsCommand(
      PairsCollisionMarginData pairs_margin,
      CollisionMarginOverrideType override_type = CollisionMarginOverrideType::OVERRIDE_PAIR_MARGIN);

  ChangeCollisionMarginsCommand(CollisionMarginData collision_margin_data,
                                CollisionMarginOverrideType override_type = CollisionMarginOverrideType::REPLACE);

  tesseract_common::CollisionMarginData getCollisionMarginData() const;
  tesseract_common::CollisionMarginOverrideType getCollisionMarginOverrideType() const;

  bool operator==(const ChangeCollisionMarginsCommand& rhs) const;
  bool operator!=(const ChangeCollisionMarginsCommand& rhs) const;

private:
  CollisionMarginData collision_margin_data_;
  CollisionMarginOverrideType collision_margin_override_{ CollisionMarginOverrideType::OVERRIDE_DEFAULT_MARGIN };

  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};
}  // namespace tesseract_environment

#include <boost/serialization/export.hpp>
#include <boost/serialization/tracking.hpp>
BOOST_CLASS_EXPORT_KEY2(tesseract_environment::ChangeCollisionMarginsCommand, "ChangeCollisionMarginsCommand")
#endif  // TESSERACT_ENVIRONMENT_CHANGE_COLLISION_MARGINS_COMMAND_H
