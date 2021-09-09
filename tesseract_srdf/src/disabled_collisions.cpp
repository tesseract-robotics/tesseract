/**
 * @file disabled_collisions.cpp
 * @brief Parse disabled collision data from srdf file
 *
 * @author Levi Armstrong
 * @date March 13, 2021
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2021, Southwest Research Institute
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
#include <console_bridge/console.h>
#include <tinyxml2.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/utils.h>
#include <tesseract_srdf/disabled_collisions.h>
#include <tesseract_scene_graph/graph.h>
#include <tesseract_scene_graph/allowed_collision_matrix.h>

namespace tesseract_srdf
{
tesseract_scene_graph::AllowedCollisionMatrix
parseDisabledCollisions(const tesseract_scene_graph::SceneGraph& scene_graph,
                        const tinyxml2::XMLElement* srdf_xml,
                        const std::array<int, 3>& /*version*/)
{
  tesseract_scene_graph::AllowedCollisionMatrix acm;

  for (const tinyxml2::XMLElement* xml_element = srdf_xml->FirstChildElement("disable_collisions");
       xml_element != nullptr;
       xml_element = xml_element->NextSiblingElement("disable_collisions"))
  {
    std::string link1_name, link2_name, reason;
    tinyxml2::XMLError status = tesseract_common::QueryStringAttributeRequired(xml_element, "link1", link1_name);
    if (status != tinyxml2::XML_SUCCESS)
      std::throw_with_nested(std::runtime_error("DisabledCollisions: Missing or failed to parse attribute 'link1'!"));

    status = tesseract_common::QueryStringAttributeRequired(xml_element, "link2", link2_name);
    if (status != tinyxml2::XML_SUCCESS)
      std::throw_with_nested(std::runtime_error("DisabledCollisions: Missing or failed to parse attribute 'link2'!"));

    if (!scene_graph.getLink(link1_name))
    {
      CONSOLE_BRIDGE_logWarn("Link '%s' is not known to URDF. Cannot disable collisons.", link1_name.c_str());
      continue;
    }
    if (!scene_graph.getLink(link2_name))
    {
      CONSOLE_BRIDGE_logWarn("Link '%s' is not known to URDF. Cannot disable collisons.", link2_name.c_str());
      continue;
    }

    status = tesseract_common::QueryStringAttribute(xml_element, "reason", reason);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      std::throw_with_nested(std::runtime_error("DisabledCollisions: Missing or failed to parse attribute 'reason'!"));

    acm.addAllowedCollision(link1_name, link2_name, reason);
  }

  return acm;
}
}  // namespace tesseract_srdf
