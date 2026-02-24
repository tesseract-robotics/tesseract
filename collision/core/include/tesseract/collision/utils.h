/**
 * @file utils.h
 * @brief Tesseract Collision utils
 *
 * @author Levi Armstrong
 * @author Matthew Powelson
 * @date December 1, 2021
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
#ifndef TESSERACT_COLLISION_UTILS_H
#define TESSERACT_COLLISION_UTILS_H

#include <tesseract/collision/types.h>
#include <tesseract/collision/contact_result_validator.h>
#include <tesseract/common/contact_allowed_validator.h>

namespace tesseract::collision
{
/**
 * @brief Combines two ContactAllowedValidator using the override type
 * @param original Original ContactAllowedValidator. This will be returned if ACMOverrideType is None
 * @param override Overriding ContactAllowedValidator. This will be returned if ACMOverrideType is ASSIGN
 * @param type Override type used to combine the ContactAllowedValidator
 * @return One ContactAllowedValidator that combines the two
 */
tesseract::common::ContactAllowedValidator::ConstPtr
combineContactAllowedValidators(tesseract::common::ContactAllowedValidator::ConstPtr original,
                                tesseract::common::ContactAllowedValidator::ConstPtr override,
                                ACMOverrideType type = ACMOverrideType::OR);

/**
 * @brief Applies ACM to contact manager using override type
 * @param manager Manager whose ContactAllowedValidator will be overwritten
 * @param acm ACM used to create ContactAllowedValidator
 * @param type Determines how the two ContactAllowedValidator are combined
 */
template <typename ManagerType>
inline void applyContactAllowedValidatorOverride(ManagerType& manager,
                                                 const tesseract::common::AllowedCollisionMatrix& acm,
                                                 ACMOverrideType type)
{
  tesseract::common::ContactAllowedValidator::ConstPtr original = manager.getContactAllowedValidator();
  auto override = std::make_shared<tesseract::common::ACMContactAllowedValidator>(acm);
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
}  // namespace tesseract::collision

#endif  // TESSERACT_COLLISION_UTILS_H
