/**
 * @file utils.cpp
 * @brief Tesseract Scene Graph utility functions
 *
 * @author Levi Armstrong
 * @date Dec 18, 2017
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

#include <tesseract_scene_graph/utils.h>

namespace tesseract_scene_graph
{
std::vector<std::string> getAllowedCollisions(const std::vector<std::string>& link_names,
                                              const AllowedCollisionEntries& acm_entries,
                                              bool remove_duplicates)
{
  std::vector<std::string> results;
  results.reserve(acm_entries.size());

  for (const auto& entry : acm_entries)
  {
    const std::string link_1 = entry.first.first;
    const std::string link_2 = entry.first.second;

    // If the first entry is one of the links we were looking for
    if (std::find(link_names.begin(), link_names.end(), link_1) != link_names.end())
      // If it hasn't already been added or remove_duplicates is disabled
      if (!remove_duplicates || (std::find(results.begin(), results.end(), link_2) == results.end()))
        results.push_back(link_2);

    if (std::find(link_names.begin(), link_names.end(), link_2) != link_names.end())
      if (!remove_duplicates || (std::find(results.begin(), results.end(), link_1) == results.end()))
        results.push_back(link_1);
  }
  return results;
}
}  // namespace tesseract_scene_graph
