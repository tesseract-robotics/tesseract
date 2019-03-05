/**
 * @file utils.h
 * @brief Tesseract Environment Utility Functions.
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
#ifndef TESSERACT_ENVIRONMENT_UTILS_H
#define TESSERACT_ENVIRONMENT_UTILS_H
#include <tesseract_environment/core/macros.h>
TESSERACT_ENVIRONMENT_IGNORE_WARNINGS_PUSH
#include <tesseract_scene_graph/graph.h>
TESSERACT_ENVIRONMENT_IGNORE_WARNINGS_POP

namespace tesseract_environment
{
  /**
   * @brief Get the active Link Names Recursively
   *
   *        This currently only works for graphs that are trees. Need to create a generic method using boost::visitor
   *
   * @param active_links
   * @param scene_graph
   * @param current_link
   * @param active
   */
  inline void getActiveLinkNamesRecursive(std::vector<std::string>& active_links,
                                          const tesseract_scene_graph::SceneGraphConstPtr scene_graph,
                                          const std::string& current_link,
                                          bool active)
  {
    // recursively get all active links
    assert(scene_graph->isTree());
    if (active)
    {
      active_links.push_back(current_link);
      for (const auto& child_link : scene_graph->getAdjacentLinkNames(current_link))
        getActiveLinkNamesRecursive(active_links, scene_graph, child_link, active);
    }
    else
    {
      for (const auto& child_link : scene_graph->getAdjacentLinkNames(current_link))
        if (scene_graph->getInboundJoints(child_link)[0]->type != tesseract_scene_graph::JointType::FIXED)
          getActiveLinkNamesRecursive(active_links, scene_graph, child_link, true);
        else
          getActiveLinkNamesRecursive(active_links, scene_graph, child_link, active);
    }
  }
}
#endif // TESSERACT_ENVIRONMENT_UTILS_H
