/**
 * @file modify_allowed_collision_command.h
 * @brief Used to modify an allowed collision to the environment
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
#ifndef TESSERACT_ENVIRONMENT_MODIFY_ALLOWED_COLLISIONS_MATRIX_COMMAND_H
#define TESSERACT_ENVIRONMENT_MODIFY_ALLOWED_COLLISIONS_MATRIX_COMMAND_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/command.h>
#include <tesseract_common/allowed_collision_matrix.h>

namespace tesseract::environment
{
enum class ModifyAllowedCollisionsType
{
  ADD,
  REMOVE,
  REPLACE
};

class ModifyAllowedCollisionsCommand;
template <class Archive>
void serialize(Archive& ar, ModifyAllowedCollisionsCommand& obj);

class ModifyAllowedCollisionsCommand : public Command
{
public:
  using Ptr = std::shared_ptr<ModifyAllowedCollisionsCommand>;
  using ConstPtr = std::shared_ptr<const ModifyAllowedCollisionsCommand>;

  ModifyAllowedCollisionsCommand();

  ModifyAllowedCollisionsCommand(tesseract::common::AllowedCollisionMatrix acm, ModifyAllowedCollisionsType type);

  ModifyAllowedCollisionsType getModifyType() const;
  const tesseract::common::AllowedCollisionMatrix& getAllowedCollisionMatrix() const;

  bool operator==(const ModifyAllowedCollisionsCommand& rhs) const;
  bool operator!=(const ModifyAllowedCollisionsCommand& rhs) const;

private:
  ModifyAllowedCollisionsType type_{ ModifyAllowedCollisionsType::ADD };
  tesseract::common::AllowedCollisionMatrix acm_;

  template <class Archive>
  friend void ::tesseract::environment::serialize(Archive& ar, ModifyAllowedCollisionsCommand& obj);
};
}  // namespace tesseract::environment

#endif  // TESSERACT_ENVIRONMENT_MODIFY_ALLOWED_COLLISIONS_MATRIX_COMMAND_H
