/**
 * @file types.cpp
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

#include <tesseract_common/types.h>

namespace tesseract_common
{
const std::string KinematicsPluginInfo::CONFIG_KEY{ "kinematic_plugins" };

std::size_t PairHash::operator()(const LinkNamesPair& pair) const
{
  return std::hash<std::string>()(pair.first + pair.second);
}

LinkNamesPair makeOrderedLinkPair(const std::string& link_name1, const std::string& link_name2)
{
  if (link_name1 <= link_name2)
    return std::make_pair(link_name1, link_name2);

  return std::make_pair(link_name2, link_name1);
}

}  // namespace tesseract_common
