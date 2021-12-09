/**
 * @file utils.cpp
 * @brief Tesseract Collision Utils
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

#include <tesseract_collision/core/utils.h>

namespace tesseract_collision
{
IsContactAllowedFn combineContactAllowedFn(const IsContactAllowedFn& original,
                                           const IsContactAllowedFn& override,
                                           ACMOverrideType type)
{
  switch (type)
  {
    case ACMOverrideType::NONE:
      return original;
    case ACMOverrideType::ASSIGN:
      return override;
    case ACMOverrideType::AND:
      return [original, override](const std::string& str1, const std::string& str2) {
        return original(str1, str2) && override(str1, str2);
      };
    case ACMOverrideType::OR:
      return [original, override](const std::string& str1, const std::string& str2) {
        return original(str1, str2) || override(str1, str2);
      };
    default:
      return original;
  }
}
}  // namespace tesseract_collision
