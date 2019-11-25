/**
 * @file material.h
 * @brief Parse material from xml string
 *
 * @author Levi Armstrong
 * @date September 1, 2019
 * @version TODO
 * @bug No known bugs
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
#ifndef TESSERACT_URDF_MATERIAL_H
#define TESSERACT_URDF_MATERIAL_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <tesseract_common/status_code.h>
#include <tesseract_common/utils.h>
#include <Eigen/Geometry>
#include <tinyxml2.h>
#include <boost/algorithm/string.hpp>
#include <unordered_map>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_urdf/utils.h>
#include <tesseract_scene_graph/link.h>

namespace tesseract_urdf
{
class MaterialStatusCategory : public tesseract_common::StatusCategory
{
public:
  MaterialStatusCategory() : name_("MaterialStatusCategory") {}
  const std::string& name() const noexcept override { return name_; }
  std::string message(int code) const override
  {
    switch (code)
    {
      case Success:
        return "Sucessfully parsed material element!";
      case MultipleMaterialsWithSameName:
        return "Multiple materials with the same name exist!";
      case ErrorAttributeName:
        return "Missing or failed parsing material attribute 'name'!";
      case ErrorTextureAttributeFilename:
        return "Missing or failed parsing material's texture attribute 'filename'!";
      case ErrorColorAttributeRGBA:
        return "Missing or failed parsing material's color attribute 'rgba'!";
      case ErrorParsingColorAttributeRGBA:
        return "Failed to parsing material's color attribute 'rgba' from string!";
      case ErrorNameOnlyIsNotAllowed:
        return "Material name only is not allowed!";
      case ErrorLocatingMaterialByName:
        return "Material with name only was not located in available materials!";
      case ErrorAnonymousMaterialNamesNotAllowed:
        return "Anonymous material names (empty string) not allowed!";
      default:
        return "Invalid error code for " + name_ + "!";
    }
  }

  enum
  {
    MultipleMaterialsWithSameName = 1,
    Success = 0,
    ErrorAttributeName = -1,
    ErrorTextureAttributeFilename = -2,
    ErrorColorAttributeRGBA = -3,
    ErrorParsingColorAttributeRGBA = -4,
    ErrorNameOnlyIsNotAllowed = -5,
    ErrorLocatingMaterialByName = -6,
    ErrorAnonymousMaterialNamesNotAllowed = -7

  };

private:
  std::string name_;
};

inline tesseract_common::StatusCode::Ptr
parse(tesseract_scene_graph::Material::Ptr& material,
      const tinyxml2::XMLElement* xml_element,
      std::unordered_map<std::string, tesseract_scene_graph::Material::Ptr>& available_materials,
      const bool allow_anonymous,
      const int /*version*/)
{
  material = tesseract_scene_graph::DEFAULT_TESSERACT_MATERIAL;
  auto status_cat = std::make_shared<MaterialStatusCategory>();
  auto success_status = std::make_shared<tesseract_common::StatusCode>(MaterialStatusCategory::Success, status_cat);

  std::string material_name;
  if (QueryStringAttribute(xml_element, "name", material_name) != tinyxml2::XML_SUCCESS)
    return std::make_shared<tesseract_common::StatusCode>(MaterialStatusCategory::ErrorAttributeName, status_cat);

  auto m = std::make_shared<tesseract_scene_graph::Material>(material_name);

  m->texture_filename = "";
  const tinyxml2::XMLElement* texture = xml_element->FirstChildElement("texture");
  if (texture != nullptr)
  {
    if (QueryStringAttribute(texture, "filename", m->texture_filename) != tinyxml2::XML_SUCCESS)
      return std::make_shared<tesseract_common::StatusCode>(MaterialStatusCategory::ErrorTextureAttributeFilename,
                                                            status_cat);
  }

  const tinyxml2::XMLElement* color = xml_element->FirstChildElement("color");
  if (color != nullptr)
  {
    std::string color_string;
    if (QueryStringAttribute(color, "rgba", color_string) != tinyxml2::XML_SUCCESS)
      return std::make_shared<tesseract_common::StatusCode>(MaterialStatusCategory::ErrorColorAttributeRGBA,
                                                            status_cat);

    if (!color_string.empty())
    {
      std::vector<std::string> tokens;
      boost::split(tokens, color_string, boost::is_any_of(" "), boost::token_compress_on);
      if (tokens.size() != 4 || !tesseract_common::isNumeric(tokens))
        return std::make_shared<tesseract_common::StatusCode>(MaterialStatusCategory::ErrorParsingColorAttributeRGBA,
                                                              status_cat);

      double r, g, b, a;
      if (!tesseract_common::toNumeric<double>(tokens[0], r))
        return std::make_shared<tesseract_common::StatusCode>(MaterialStatusCategory::ErrorParsingColorAttributeRGBA,
                                                              status_cat);

      if (!tesseract_common::toNumeric<double>(tokens[1], g))
        return std::make_shared<tesseract_common::StatusCode>(MaterialStatusCategory::ErrorParsingColorAttributeRGBA,
                                                              status_cat);

      if (!tesseract_common::toNumeric<double>(tokens[2], b))
        return std::make_shared<tesseract_common::StatusCode>(MaterialStatusCategory::ErrorParsingColorAttributeRGBA,
                                                              status_cat);

      if (!tesseract_common::toNumeric<double>(tokens[3], a))
        return std::make_shared<tesseract_common::StatusCode>(MaterialStatusCategory::ErrorParsingColorAttributeRGBA,
                                                              status_cat);

      m->color = Eigen::Vector4d(r, g, b, a);
    }
    else
    {
      return std::make_shared<tesseract_common::StatusCode>(MaterialStatusCategory::ErrorColorAttributeRGBA,
                                                            status_cat);
    }
  }

  if (color == nullptr && texture == nullptr)
  {
    if (available_materials.empty())
    {
      return std::make_shared<tesseract_common::StatusCode>(MaterialStatusCategory::ErrorNameOnlyIsNotAllowed,
                                                            status_cat);
    }

    auto it = available_materials.find(material_name);
    if (it == available_materials.end())
      return std::make_shared<tesseract_common::StatusCode>(MaterialStatusCategory::ErrorLocatingMaterialByName,
                                                            status_cat);
    m = it->second;
  }
  else
  {
    if (!material_name.empty())
    {
      auto it = available_materials.find(material_name);
      if (it != available_materials.end())
        success_status->setChild(std::make_shared<tesseract_common::StatusCode>(
            MaterialStatusCategory::MultipleMaterialsWithSameName, status_cat));

      available_materials[material_name] = m;
    }
    else if (!allow_anonymous)
    {
      return std::make_shared<tesseract_common::StatusCode>(
          MaterialStatusCategory::ErrorAnonymousMaterialNamesNotAllowed, status_cat);
    }
  }

  material = m;
  return success_status;
}

}  // namespace tesseract_urdf
#endif  // TESSERACT_URDF_MATERIAL_H
