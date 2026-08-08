/**
 * @file types.cpp
 * @brief Common Tesseract Types
 *
 * @author Levi Armstrong
 * @author Roelof Oomen
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

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/functional/hash.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/common/types.h>

namespace tesseract::common
{
std::size_t nameIdHash(const std::string& name) noexcept { return boost::hash<std::string>{}(name); }

std::size_t combineNameIdHash(std::size_t f, std::size_t s) noexcept
{
  std::size_t seed{ 0 };
  boost::hash_combine(seed, f);
  boost::hash_combine(seed, s);
  return seed;
}

}  // namespace tesseract::common
