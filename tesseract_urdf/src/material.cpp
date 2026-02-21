/**
 * @file material.cpp
 * @brief Parse material from xml string
 *
 * @author Levi Armstrong
 * @date September 1, 2019
 *
 * @copyright Copyright (c) 2019, Southwest Research Institute
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
#include <stdexcept>
#include <unordered_map>

#include <boost/algorithm/string.hpp>
#include <console_bridge/console.h>
#include <Eigen/Geometry>
#include <tesseract_common/utils.h>
#include <tinyxml2.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_scene_graph/link.h>
#include <tesseract_urdf/material.h>

namespace tesseract::urdf
{
tesseract::scene_graph::Material::Ptr
parseMaterial(const tinyxml2::XMLElement* xml_element,
              std::unordered_map<std::string, tesseract::scene_graph::Material::Ptr>& available_materials,
              bool allow_anonymous)
{
  std::string material_name;
  if (tesseract::common::QueryStringAttribute(xml_element, "name", material_name) != tinyxml2::XML_SUCCESS)
    std::throw_with_nested(std::runtime_error("Material: Missing or failed parsing attribute 'name'!"));

  auto m = std::make_shared<tesseract::scene_graph::Material>(material_name);

  m->texture_filename = "";
  const tinyxml2::XMLElement* texture = xml_element->FirstChildElement("texture");
  if (texture != nullptr)
  {
    if (tesseract::common::QueryStringAttribute(texture, "filename", m->texture_filename) != tinyxml2::XML_SUCCESS)
      std::throw_with_nested(std::runtime_error("Material: Missing or failed parsing texture attribute 'filename'!"));
  }

  const tinyxml2::XMLElement* color = xml_element->FirstChildElement("color");
  if (color != nullptr)
  {
    std::string color_string;
    if (tesseract::common::QueryStringAttribute(color, "rgba", color_string) != tinyxml2::XML_SUCCESS)
      std::throw_with_nested(std::runtime_error("Material: Missing or failed parsing color attribute 'rgba'!"));

    if (!color_string.empty())
    {
      std::vector<std::string> tokens;
      boost::split(tokens, color_string, boost::is_any_of(" "), boost::token_compress_on);
      if (tokens.size() != 4 || !tesseract::common::isNumeric(tokens))
        std::throw_with_nested(std::runtime_error("Material: Failed to parse color attribute 'rgba' from string!"));

      double r{ 0 }, g{ 0 }, b{ 0 }, a{ 0 };
      // No need to check return values because the tokens are verified above
      tesseract::common::toNumeric<double>(tokens[0], r);
      tesseract::common::toNumeric<double>(tokens[1], g);
      tesseract::common::toNumeric<double>(tokens[2], b);
      tesseract::common::toNumeric<double>(tokens[3], a);

      m->color = Eigen::Vector4d(r, g, b, a);
    }
    else
    {
      std::throw_with_nested(std::runtime_error("Material: Missing or failed parsing color attribute 'rgba'!"));
    }
  }

  if (color == nullptr && texture == nullptr)
  {
    if (available_materials.empty())
      std::throw_with_nested(
          std::runtime_error("Material: Material name '" + material_name + "' only is not allowed!"));

    auto it = available_materials.find(material_name);
    if (it == available_materials.end())
      std::throw_with_nested(std::runtime_error("Material with name only '" + material_name +
                                                "' was not located in available materials!"));

    m = it->second;
  }
  else
  {
    if (!material_name.empty())
    {
      auto it = available_materials.find(material_name);
      if (it != available_materials.end())
        CONSOLE_BRIDGE_logDebug("Multiple materials with the same name '%s' exist!", material_name.c_str());

      available_materials[material_name] = m;
    }
    else if (!allow_anonymous)
    {
      std::throw_with_nested(std::runtime_error("Anonymous material names (empty string) not allowed!"));
    }
  }

  return m;
}

tinyxml2::XMLElement* writeMaterial(const std::shared_ptr<const tesseract::scene_graph::Material>& material,
                                    tinyxml2::XMLDocument& doc)
{
  if (material == nullptr)
    std::throw_with_nested(std::runtime_error("Material is nullptr and cannot be converted to XML"));
  tinyxml2::XMLElement* xml_element = doc.NewElement(MATERIAL_ELEMENT_NAME.data());
  Eigen::IOFormat eigen_format(Eigen::StreamPrecision, Eigen::DontAlignCols, " ", " ");

  xml_element->SetAttribute("name", material->getName().c_str());

  if (!material->texture_filename.empty())
  {
    tinyxml2::XMLElement* xml_texture = doc.NewElement("texture");
    xml_texture->SetAttribute("filename", material->texture_filename.c_str());
    xml_element->InsertEndChild(xml_texture);
  }

  tinyxml2::XMLElement* xml_color = doc.NewElement("color");
  std::stringstream color_string;
  color_string << material->color.format(eigen_format);
  xml_color->SetAttribute("rgba", color_string.str().c_str());
  xml_element->InsertEndChild(xml_color);

  return xml_element;
}

}  // namespace tesseract::urdf
