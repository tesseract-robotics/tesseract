/**
 * @file utils.h
 * @brief Common Tesseract Utility Functions
 *
 * @author Levi Armstrong
 * @date January 18, 2018
 * @version TODO
 * @bug No known bugs
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
#ifndef TESSERACT_COMMON_UTILS_H
#define TESSERACT_COMMON_UTILS_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <vector>
#include <string>
#include <sstream>
#include <stdexcept>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_common
{
/**
 * @brief Determine if a string is a number
 * @param s The string to evaluate
 * @return True if numeric, otherwise false.
 */
inline bool isNumeric(const std::string& s)
{
  if (s.empty())
    return false;

  std::stringstream ss;
  ss.imbue(std::locale::classic());

  ss << s;

  double out;
  ss >> out;

  return !(ss.fail() || !ss.eof());
}

inline bool isNumeric(const std::vector<std::string>& sv)
{
  for (const auto& s : sv)
  {
    if (!isNumeric(s))
      return false;
  }

  return true;
}

/**
 * @brief Convert a string to a numeric value type
 * @param s The string to be converted
 * @param value The value to be loaded with coverted string
 * @return True if successful, otherwise false
 */
template <typename FloatType>
inline bool toNumeric(const std::string& s, FloatType& value)
{
  if (s.empty())
    return false;

  std::stringstream ss;
  ss.imbue(std::locale::classic());

  ss << s;

  FloatType out;
  ss >> out;

  if (ss.fail() || !ss.eof())
    return false;

  value = out;
  return true;
}

}  // namespace tesseract_common
#endif  // TESSERACT_COMMON_UTILS_H
