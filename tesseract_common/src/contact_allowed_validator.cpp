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

#include <boost/serialization/nvp.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/base_object.hpp>

namespace tesseract_common
{
template <class Archive>
void ContactAllowedValidator::serialize(Archive& /*ar*/, const unsigned int /*version*/)
{
}

ACMContactAllowedValidator::ACMContactAllowedValidator(tesseract_common::AllowedCollisionMatrix acm)
  : acm_(std::move(acm))
{
}

bool ACMContactAllowedValidator::operator()(const std::string& link_name1, const std::string& link_name2) const
{
  return acm_.isCollisionAllowed(link_name1, link_name2);
}

template <class Archive>
void ACMContactAllowedValidator::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(ContactAllowedValidator);
  ar& BOOST_SERIALIZATION_NVP(acm_);
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

template <class Archive>
void CombinedContactAllowedValidator::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(ContactAllowedValidator);
  ar& BOOST_SERIALIZATION_NVP(validators_);
  ar& BOOST_SERIALIZATION_NVP(type_);
}

}  // namespace tesseract_common

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_common::ContactAllowedValidator)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_common::ContactAllowedValidator)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_common::ACMContactAllowedValidator)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_common::ACMContactAllowedValidator)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_common::CombinedContactAllowedValidator)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_common::CombinedContactAllowedValidator)
