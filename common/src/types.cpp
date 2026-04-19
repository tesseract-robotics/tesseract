/**
 * @file types.cpp
 * @brief Common Tesseract Types
 *
 * @author Levi Armstrong
 * @date January 18, 2018
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

#include <tesseract/common/types.h>

#include <stdexcept>
#include <string>

namespace tesseract::common
{
void checkPairHashCollision(const char* context,
                            const std::string& new_name1,
                            const std::string& new_name2,
                            const std::string& existing_name1,
                            const std::string& existing_name2)
{
  const bool names_match = (existing_name1 == new_name1 && existing_name2 == new_name2) ||
                           (existing_name1 == new_name2 && existing_name2 == new_name1);
  if (!names_match)
    throw std::runtime_error(std::string(context) + " LinkIdPair hash collision: ('" + new_name1 + "', '" + new_name2 +
                             "') collides with ('" + existing_name1 + "', '" + existing_name2 + "')");
}
}  // namespace tesseract::common
