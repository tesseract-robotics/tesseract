/**
 * @file contact_allowed_validators.cpp
 * @brief The contact allowed validator
 *
 * @author Levi Armstrong
 * @date Jan 2, 2025
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2025, Levi Armstrong
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

#include <tesseract_common/contact_allowed_validator.h>

namespace tesseract_common
{
ACMContactAllowedValidator::ACMContactAllowedValidator(tesseract_common::AllowedCollisionMatrix acm)
  : acm_(std::move(acm))
{
}

bool ACMContactAllowedValidator::operator()(const std::string& link_name1, const std::string& link_name2) const
{
  return acm_.isCollisionAllowed(link_name1, link_name2);
}

CombinedContactAllowedValidator::CombinedContactAllowedValidator(
    std::vector<std::shared_ptr<const ContactAllowedValidator>> validators,
    CombinedContactAllowedValidatorType type)
  : validators_(std::move(validators)), type_(type)
{
}

bool CombinedContactAllowedValidator::operator()(const std::string& link_name1, const std::string& link_name2) const
{
  assert(!validators_.empty());
  if (type_ == CombinedContactAllowedValidatorType::OR)
  {
    bool value{ false };
    for (const auto& validator : validators_)
      value = value || (*validator)(link_name1, link_name2);

    return value;
  }

  bool value{ true };
  for (const auto& validator : validators_)
    value = value && (*validator)(link_name1, link_name2);

  return value;
}

}  // namespace tesseract_common
