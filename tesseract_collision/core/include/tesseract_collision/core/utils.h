/**
 * @file utils.h
 * @brief Tesseract Collision utils
 *
 * @author Levi Armstrong
 * @author Matthew Powelson
 * @date December 1, 2021
 * @version TODO
 * @bug No known bugs
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
#ifndef TESSERACT_COLLISION_CORE_UTILS_H
#define TESSERACT_COLLISION_CORE_UTILS_H

#include <tesseract_collision/core/types.h>
#include <tesseract_collision/core/contact_result_validator.h>
#include <tesseract_common/contact_allowed_validator.h>

namespace tesseract_collision
{
/**
 * @brief Combines two ContactAllowedValidator using the override type
 * @param original Original ContactAllowedValidator. This will be returned if ACMOverrideType is None
 * @param override Overriding ContactAllowedValidator. This will be returned if ACMOverrideType is ASSIGN
 * @param type Override type used to combine the ContactAllowedValidator
 * @return One ContactAllowedValidator that combines the two
 */
tesseract_common::ContactAllowedValidator::ConstPtr
combineContactAllowedValidators(tesseract_common::ContactAllowedValidator::ConstPtr original,
                                tesseract_common::ContactAllowedValidator::ConstPtr override,
                                ACMOverrideType type = ACMOverrideType::OR);

/**
 * @brief Applies ACM to contact manager using override type
 * @param manager Manager whose ContactAllowedValidator will be overwritten
 * @param acm ACM used to create ContactAllowedValidator
 * @param type Determines how the two ContactAllowedValidator are combined
 */
template <typename ManagerType>
inline void applyContactAllowedValidatorOverride(ManagerType& manager,
                                                 const tesseract_common::AllowedCollisionMatrix& acm,
                                                 ACMOverrideType type)
{
  tesseract_common::ContactAllowedValidator::ConstPtr original = manager.getContactAllowedValidator();
  auto override = std::make_shared<tesseract_common::ACMContactAllowedValidator>(acm);
  manager.setContactAllowedValidator(combineContactAllowedValidators(original, override, type));
}

/**
 * @brief Loops over the map and for every object string either enables or disables it based on the value (true=enable,
 * false=disable)
 * @param manager Manager that will be modified
 * @param modify_object_enabled Map of [key]:value = [object name]:disable or enable
 */
template <typename ManagerType>
inline void applyModifyObjectEnabled(ManagerType& manager,
                                     const std::unordered_map<std::string, bool>& modify_object_enabled)
{
  for (const auto& entry : modify_object_enabled)
  {
    if (entry.second)
      manager.enableCollisionObject(entry.first);
    else
      manager.disableCollisionObject(entry.first);
  }
}
}  // namespace tesseract_collision

#endif  // TESSERACT_COLLISION_TYPES_H
