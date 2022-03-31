/**
 * @file tesseract_support_resource_locator.h
 * @brief Locate and retrieve resource data in tesseract_support
 *
 * @author Levi Armstrong
 * @date March 31, 2022
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2022, Levi Armstrong
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
#ifndef TESSERACT_SUPPORT_TESSERACT_SUPPORT_RESOURCE_LOCATOR_H
#define TESSERACT_SUPPORT_TESSERACT_SUPPORT_RESOURCE_LOCATOR_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/serialization/access.hpp>
#include <string>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/resource_locator.h>
#include <tesseract_common/types.h>

namespace tesseract_common
{
/** @brief Abstract class for resource loaders */
class TesseractSupportResourceLocator : public ResourceLocator
{
public:
  using Ptr = std::shared_ptr<TesseractSupportResourceLocator>;
  using ConstPtr = std::shared_ptr<const TesseractSupportResourceLocator>;

  std::shared_ptr<Resource> locateResource(const std::string& url) const override final;

private:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};
}  // namespace tesseract_common

#include <boost/serialization/export.hpp>
#include <boost/serialization/tracking.hpp>
BOOST_CLASS_EXPORT_KEY2(tesseract_common::TesseractSupportResourceLocator, "TesseractSupportResourceLocator")

#endif  // TESSERACT_SUPPORT_TESSERACT_SUPPORT_RESOURCE_LOCATOR_H
