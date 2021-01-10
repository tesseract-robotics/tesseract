#ifndef TESSERACT_SCENE_GRAPH_SRDF_DISABLED_COLLISIONS_H
#define TESSERACT_SCENE_GRAPH_SRDF_DISABLED_COLLISIONS_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <array>
#include <console_bridge/console.h>
#include <tinyxml2.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/utils.h>
#include <tesseract_scene_graph/graph.h>
#include <tesseract_scene_graph/allowed_collision_matrix.h>

namespace tesseract_scene_graph
{
/**
 * @brief Parse allowed collisions from srdf xml element
 * @param scene_graph The tesseract scene graph
 * @param srdf_xml The xml element to parse
 * @param version The srdf version number
 * @return Allowed Collision Matrix
 */
inline AllowedCollisionMatrix parseDisabledCollisions(const tesseract_scene_graph::SceneGraph& scene_graph,
                                                      const tinyxml2::XMLElement* srdf_xml,
                                                      const std::array<int, 3>& /*version*/)
{
  AllowedCollisionMatrix acm;

  for (const tinyxml2::XMLElement* xml_element = srdf_xml->FirstChildElement("disable_collisions"); xml_element;
       xml_element = xml_element->NextSiblingElement("disable_collisions"))
  {
    std::string link1_name, link2_name, reason;
    tinyxml2::XMLError status = tesseract_common::QueryStringAttributeRequired(xml_element, "link1", link1_name);
    if (status != tinyxml2::XML_SUCCESS)
      continue;

    status = tesseract_common::QueryStringAttributeRequired(xml_element, "link2", link2_name);
    if (status != tinyxml2::XML_SUCCESS)
      continue;

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
    {
      CONSOLE_BRIDGE_logError("Invalid disable_collisions attribute 'reason'");
      continue;
    }

    acm.addAllowedCollision(link1_name, link2_name, reason);
  }

  return acm;
}

}  // namespace tesseract_scene_graph

#endif  // TESSERACT_SCENE_GRAPH_SRDF_DISABLED_COLLISIONS_H
