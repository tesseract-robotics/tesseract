/**
 * @file resource.h
 * @brief Class to retrieve data from resources
 *
 * @author John Wason
 * @date October 25, 2019
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2019, Wason Technology, LLC
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

#ifndef TESSERACT_COMMON_RESOURCE_H
#define TESSERACT_COMMON_RESOURCE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <vector>
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_common
{
/**
 * @brief Represents resource data available from a file or url
 *
 */
class Resource
{
public:
  using Ptr = std::shared_ptr<Resource>;
  using ConstPtr = std::shared_ptr<const Resource>;

  /**
   * @brief Returns true if the located resource is a local file
   *
   * @return true if the resource is a local file, otherwise false
   */
  virtual bool isFile() = 0;

  /**
   * @brief Get the original URL used to locate the file
   *
   * @return The URL of the resource
   */
  virtual std::string getUrl() = 0;

  /**
   * @brief Get the file path of the resource. Only valid if isFile() is true.
   *
   * @return The file path to the resource
   */
  virtual std::string getFilePath() = 0;

  /**
   * @brief Get the resource as bytes. This function may block
   *
   * @return Resource bytes as a uint8_t vector
   */
  virtual std::vector<uint8_t> getResourceContents() = 0;

  /**
   * @brief Get the resource as a std::istream. This function and the returned stream may block
   *
   * @return A std::istream shared pointer for the resource data
   */
  virtual std::shared_ptr<std::istream> getResourceContentStream() = 0;
};

}  // namespace tesseract_common
#endif  // TESSERACT_COMMON_RESOURCE_H