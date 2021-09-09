/**
 * @file collision_margins.cpp
 * @brief Parse collision margin data from srdf file
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
#include <unordered_map>
#include <tinyxml2.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_scene_graph/graph.h>
#include <tesseract_srdf/collision_margins.h>
#include <tesseract_common/types.h>
#include <tesseract_common/utils.h>
#include <tesseract_common/collision_margin_data.h>

namespace tesseract_srdf
{
tesseract_common::CollisionMarginData::Ptr parseCollisionMargins(const tesseract_scene_graph::SceneGraph& scene_graph,
                                                                 const tinyxml2::XMLElement* srdf_xml,
                                                                 const std::array<int, 3>& /*version*/)
{
  double default_margin{ 0 };
  tesseract_common::PairsCollisionMarginData pair_margins;

  const tinyxml2::XMLElement* xml_element = srdf_xml->FirstChildElement("collision_margins");
  if (xml_element == nullptr)
    return nullptr;

  tinyxml2::XMLError status =
      tesseract_common::QueryDoubleAttributeRequired(xml_element, "default_margin", default_margin);
  if (status != tinyxml2::XML_SUCCESS)
    std::throw_with_nested(std::runtime_error("CollisionMargins: collision_margins missing attribute "
                                              "'default_margin'."));

  for (const tinyxml2::XMLElement* xml_pair_element = xml_element->FirstChildElement("pair_margin");
       xml_pair_element != nullptr;
       xml_pair_element = xml_pair_element->NextSiblingElement("pair_margin"))
  {
    std::string link1_name, link2_name;
    double link_pair_margin{ 0 };
    tinyxml2::XMLError status = tesseract_common::QueryStringAttributeRequired(xml_pair_element, "link1", link1_name);
    if (status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("parseCollisionMargins: Missing or failded to parse 'link1' attribute.");

    status = tesseract_common::QueryStringAttributeRequired(xml_pair_element, "link2", link2_name);
    if (status != tinyxml2::XML_SUCCESS)
      std::throw_with_nested(std::runtime_error("CollisionMargins: Missing or failded to parse 'link2' attribute."));

    if (!scene_graph.getLink(link1_name))
    {
      CONSOLE_BRIDGE_logWarn("parseCollisionMargins: Link '%s' is not known to URDF.", link1_name.c_str());
    }

    if (!scene_graph.getLink(link2_name))
    {
      CONSOLE_BRIDGE_logWarn("parseCollisionMargins: Link '%s' is not known to URDF.", link2_name.c_str());
    }

    status = tesseract_common::QueryDoubleAttributeRequired(xml_pair_element, "margin", link_pair_margin);
    if (status != tinyxml2::XML_SUCCESS)
      std::throw_with_nested(std::runtime_error("parseCollisionMargins: failed to parse link pair 'margin' "
                                                "attribute."));

    auto key = tesseract_common::makeOrderedLinkPair(link1_name, link2_name);
    pair_margins[key] = link_pair_margin;
  }

  return std::make_shared<tesseract_common::CollisionMarginData>(default_margin, pair_margins);
}
}  // namespace tesseract_srdf
