/**
 * @file types.h
 * @brief Common Tesseract Types
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
#ifndef TESSERACT_COMMON_TYPES_H
#define TESSERACT_COMMON_TYPES_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <string>
#include <utility>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_common
{
using LinkNamesPair = std::pair<std::string, std::string>;

struct PairHash
{
  std::size_t operator()(const LinkNamesPair& pair) const;
};

/**
 * @brief Create a pair of strings, where the pair.first is always <= pair.second.
 *
 * This is commonly used along with PairHash as the key to an unordered_map<LinkNamesPair, Type, PairHash>
 * @param link_name1 First link name
 * @param link_name2 Second link name
 * @return LinkNamesPair a lexicographically sorted pair of strings
 */
LinkNamesPair makeOrderedLinkPair(const std::string& link_name1, const std::string& link_name2);

/**
 * @brief Populate a pair of strings, where the pair.first is always <= pair.second.
 *
 * This is used to avoid multiple memory application throughout the code base
 *
 * This is commonly used along with PairHash as the key to an unordered_map<LinkNamesPair, Type, PairHash>
 *
 * @param pair The link name pair to load a lexicographically sorted pair of strings
 * @param link_name1 First link name
 * @param link_name2 Second link nam
 */
void makeOrderedLinkPair(LinkNamesPair& pair, const std::string& link_name1, const std::string& link_name2);

}  // namespace tesseract_common

#endif  // TESSERACT_COMMON_TYPES_H
