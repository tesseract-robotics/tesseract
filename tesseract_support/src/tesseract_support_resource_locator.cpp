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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/serialization/access.hpp>
#include <boost/serialization/nvp.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_support/tesseract_support_resource_locator.h>
#include <tesseract_common/filesystem.h>

namespace tesseract_common
{
std::shared_ptr<Resource> TesseractSupportResourceLocator::locateResource(const std::string& url) const
{
  std::string mod_url = url;
  if (url.find("package://tesseract_support") == 0)
  {
    mod_url.erase(0, strlen("package://tesseract_support"));
    size_t pos = mod_url.find('/');
    if (pos == std::string::npos)
      return nullptr;

    std::string package = mod_url.substr(0, pos);
    mod_url.erase(0, pos);
    std::string package_path = std::string(TESSERACT_SUPPORT_DIR);

    if (package_path.empty())
      return nullptr;

    mod_url = package_path + mod_url;
  }

  if (!tesseract_common::fs::path(mod_url).is_absolute())
    return nullptr;

  return std::make_shared<SimpleLocatedResource>(
      url, mod_url, std::make_shared<TesseractSupportResourceLocator>(*this));
}

template <class Archive>
void TesseractSupportResourceLocator::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(ResourceLocator);
}

}  // namespace tesseract_common

#include <tesseract_common/serialization.h>
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_common::TesseractSupportResourceLocator)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_common::TesseractSupportResourceLocator)
