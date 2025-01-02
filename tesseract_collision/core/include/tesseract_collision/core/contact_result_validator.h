/**
 * @file contact_result_validator.h
 * @brief Contact result validator
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
#ifndef TESSERACT_COLLISION_CORE_CONTACT_RESULT_VALIDATORS_H
#define TESSERACT_COLLISION_CORE_CONTACT_RESULT_VALIDATORS_H

#include <memory>
#include <boost/serialization/access.hpp>
#include <boost/serialization/export.hpp>

namespace tesseract_collision
{
struct ContactResult;

/**
 * @brief Should return true if contact results are valid, otherwise false.
 *
 * This is used so users may provide a callback to reject/approve collision results in various algorithms.
 */
class ContactResultValidator
{
public:
  using Ptr = std::shared_ptr<ContactResultValidator>;
  using ConstPtr = std::shared_ptr<const ContactResultValidator>;
  using UPtr = std::unique_ptr<ContactResultValidator>;
  using ConstUPtr = std::unique_ptr<const ContactResultValidator>;

  virtual ~ContactResultValidator() = default;

  virtual bool operator()(const ContactResult&) const = 0;

protected:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};

}  // namespace tesseract_collision

BOOST_CLASS_EXPORT_KEY(tesseract_collision::ContactResultValidator)

#endif  // TESSERACT_COLLISION_CORE_CONTACT_RESULT_VALIDATORS_H
