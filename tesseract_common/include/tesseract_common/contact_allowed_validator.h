/**
 * @file contact_allowed_validators.h
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

#ifndef TESSERACT_COMMON_CONTACT_ALLOWED_VALIDATOR_H
#define TESSERACT_COMMON_CONTACT_ALLOWED_VALIDATOR_H

#include <string>
#include <memory>
#include <vector>

#include <tesseract_common/allowed_collision_matrix.h>

namespace tesseract_common
{
/** @brief Should return true if links are allowed to be in collision, otherwise false. */
class ContactAllowedValidator
{
public:
  using Ptr = std::shared_ptr<ContactAllowedValidator>;
  using ConstPtr = std::shared_ptr<const ContactAllowedValidator>;
  using UPtr = std::unique_ptr<ContactAllowedValidator>;
  using ConstUPtr = std::unique_ptr<const ContactAllowedValidator>;

  virtual ~ContactAllowedValidator() = default;

  virtual bool operator()(const std::string&, const std::string&) const = 0;

protected:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};

class ACMContactAllowedValidator : public ContactAllowedValidator
{
public:
  using Ptr = std::shared_ptr<ACMContactAllowedValidator>;
  using ConstPtr = std::shared_ptr<const ACMContactAllowedValidator>;
  using UPtr = std::unique_ptr<ACMContactAllowedValidator>;
  using ConstUPtr = std::unique_ptr<const ACMContactAllowedValidator>;

  ACMContactAllowedValidator() = default;  // Required for serialization
  ACMContactAllowedValidator(tesseract_common::AllowedCollisionMatrix acm);

  bool operator()(const std::string& link_name1, const std::string& link_name2) const override;

protected:
  tesseract_common::AllowedCollisionMatrix acm_;

  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};

/** @brief Identify how the two should be combined */
enum class CombinedContactAllowedValidatorType : std::uint8_t
{
  /** @brief Combines the two ContactAllowedValidator with AND operator */
  AND,
  /** @brief Combines the two ContactAllowedValidator with OR operator */
  OR,
};

class CombinedContactAllowedValidator : public ContactAllowedValidator
{
public:
  using Ptr = std::shared_ptr<CombinedContactAllowedValidator>;
  using ConstPtr = std::shared_ptr<const CombinedContactAllowedValidator>;
  using UPtr = std::unique_ptr<CombinedContactAllowedValidator>;
  using ConstUPtr = std::unique_ptr<const CombinedContactAllowedValidator>;

  CombinedContactAllowedValidator() = default;  // Required for serialization
  CombinedContactAllowedValidator(std::vector<std::shared_ptr<const ContactAllowedValidator>> validators,
                                  CombinedContactAllowedValidatorType type);

  bool operator()(const std::string& link_name1, const std::string& link_name2) const override;

protected:
  std::vector<std::shared_ptr<const ContactAllowedValidator>> validators_;
  CombinedContactAllowedValidatorType type_{ CombinedContactAllowedValidatorType::OR };

  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};

}  // namespace tesseract_common

BOOST_CLASS_EXPORT_KEY(tesseract_common::ContactAllowedValidator)
BOOST_CLASS_EXPORT_KEY(tesseract_common::ACMContactAllowedValidator)
BOOST_CLASS_EXPORT_KEY(tesseract_common::CombinedContactAllowedValidator)

#endif  // TESSERACT_COMMON_CONTACT_ALLOWED_VALIDATOR_H
