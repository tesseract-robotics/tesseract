/**
 * @file trajopt_collision_config.cpp
 * @brief TrajOpt collision configuration settings
 *
 * @author Tyler Marr
 * @date August 20, 2020
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2020, Southwest Research Institute
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
#include <iostream>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/trajopt/trajopt_collision_config.h>
#include <tesseract_common/utils.h>

namespace tesseract_planning
{
CollisionCostConfig::CollisionCostConfig(const tinyxml2::XMLElement& xml_element)
{
  const tinyxml2::XMLElement* enabled_element = xml_element.FirstChildElement("Enabled");
  const tinyxml2::XMLElement* use_weighted_sum_element = xml_element.FirstChildElement("UseWeightedSum");
  const tinyxml2::XMLElement* type_element = xml_element.FirstChildElement("CollisionEvaluator");
  const tinyxml2::XMLElement* buffer_margin_element = xml_element.FirstChildElement("BufferMargin");
  const tinyxml2::XMLElement* safety_margin_buffer_element = xml_element.FirstChildElement("SafetyMarginBuffer");
  const tinyxml2::XMLElement* coeff_element = xml_element.FirstChildElement("Coefficient");

  if (!enabled_element)
    throw std::runtime_error("CollisionCostConfig: Must have Enabled element.");

  tinyxml2::XMLError status = enabled_element->QueryBoolText(&enabled);
  if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
    throw std::runtime_error("CollisionCostConfig: Error parsing Enabled string");

  if (use_weighted_sum_element)
  {
    status = use_weighted_sum_element->QueryBoolText(&use_weighted_sum);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("CollisionCostConfig: Error parsing UseWeightedSum string");
  }

  if (type_element)
  {
    int coll_type = static_cast<int>(trajopt::CollisionEvaluatorType::CAST_CONTINUOUS);
    status = type_element->QueryIntAttribute("type", &coll_type);
    if (status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("CollisionCostConfig: Error parsing CollisionEvaluator type attribute.");

    type = static_cast<trajopt::CollisionEvaluatorType>(coll_type);
  }

  if (buffer_margin_element)
  {
    std::string buffer_margin_string;
    status = tesseract_common::QueryStringText(buffer_margin_element, buffer_margin_string);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("CollisionCostConfig: Error parsing BufferMargin string");

    if (!tesseract_common::isNumeric(buffer_margin_string))
      throw std::runtime_error("CollisionCostConfig: BufferMargin is not a numeric values.");

    tesseract_common::toNumeric<double>(buffer_margin_string, safety_margin);
  }

  if (safety_margin_buffer_element)
  {
    std::string safety_margin_buffer_string;
    status = tesseract_common::QueryStringText(safety_margin_buffer_element, safety_margin_buffer_string);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("CollisionCostConfig: Error parsing SafetyMarginBuffer string");

    if (!tesseract_common::isNumeric(safety_margin_buffer_string))
      throw std::runtime_error("CollisionCostConfig: SafetyMarginBuffer is not a numeric values.");

    tesseract_common::toNumeric<double>(safety_margin_buffer_string, safety_margin_buffer);
  }

  if (coeff_element)
  {
    std::string coeff_string;
    status = tesseract_common::QueryStringText(coeff_element, coeff_string);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("CollisionCostConfig: Error parsing Coefficient string");

    if (!tesseract_common::isNumeric(coeff_string))
      throw std::runtime_error("CollisionCostConfig: Coefficient is not a numeric values.");

    tesseract_common::toNumeric<double>(coeff_string, coeff);
  }
}

tinyxml2::XMLElement* CollisionCostConfig::toXML(tinyxml2::XMLDocument& doc) const
{
  tinyxml2::XMLElement* xml_coll_cost_config = doc.NewElement("CollisionCostConfig");

  tinyxml2::XMLElement* xml_enabled = doc.NewElement("Enabled");
  xml_enabled->SetText(enabled);
  xml_coll_cost_config->InsertEndChild(xml_enabled);

  tinyxml2::XMLElement* xml_use_weighted_sum = doc.NewElement("UseWeightedSum");
  xml_use_weighted_sum->SetText(use_weighted_sum);
  xml_coll_cost_config->InsertEndChild(xml_use_weighted_sum);

  tinyxml2::XMLElement* xml_type = doc.NewElement("CollisionEvaluator");
  xml_type->SetAttribute("type", std::to_string(static_cast<int>(type)).c_str());
  xml_coll_cost_config->InsertEndChild(xml_type);

  tinyxml2::XMLElement* xml_buffer_margin = doc.NewElement("BufferMargin");
  xml_buffer_margin->SetText(safety_margin);
  xml_coll_cost_config->InsertEndChild(xml_buffer_margin);

  tinyxml2::XMLElement* xml_safety_margin_buffer = doc.NewElement("SafetyMarginBuffer");
  xml_safety_margin_buffer->SetText(safety_margin_buffer);
  xml_coll_cost_config->InsertEndChild(xml_safety_margin_buffer);

  tinyxml2::XMLElement* xml_coeff = doc.NewElement("Coefficient");
  xml_coeff->SetText(coeff);
  xml_coll_cost_config->InsertEndChild(xml_coeff);

  return xml_coll_cost_config;
}

CollisionConstraintConfig::CollisionConstraintConfig(const tinyxml2::XMLElement& xml_element)
{
  const tinyxml2::XMLElement* enabled_element = xml_element.FirstChildElement("Enabled");
  const tinyxml2::XMLElement* use_weighted_sum_element = xml_element.FirstChildElement("UseWeightedSum");
  const tinyxml2::XMLElement* type_element = xml_element.FirstChildElement("CollisionEvaluator");
  const tinyxml2::XMLElement* safety_margin_element = xml_element.FirstChildElement("SafetyMargin");
  const tinyxml2::XMLElement* safety_margin_buffer_element = xml_element.FirstChildElement("SafetyMarginBuffer");
  const tinyxml2::XMLElement* coeff_element = xml_element.FirstChildElement("Coefficient");

  if (!enabled_element)
    throw std::runtime_error("CollisionConstraintConfig: Must have Enabled element.");

  tinyxml2::XMLError status = enabled_element->QueryBoolText(&enabled);
  if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
    throw std::runtime_error("CollisionConstraintConfig: Error parsing Enabled string");

  if (use_weighted_sum_element)
  {
    status = use_weighted_sum_element->QueryBoolText(&use_weighted_sum);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("CollisionConstraintConfig: Error parsing UseWeightedSum string");
  }

  if (type_element)
  {
    int coll_type = static_cast<int>(trajopt::CollisionEvaluatorType::CAST_CONTINUOUS);
    status = type_element->QueryIntAttribute("type", &coll_type);
    if (status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("CollisionConstraintConfig: Error parsing CollisionEvaluator type attribute.");

    type = static_cast<trajopt::CollisionEvaluatorType>(coll_type);
  }

  if (safety_margin_element)
  {
    std::string safety_margin_string;
    status = tesseract_common::QueryStringText(safety_margin_element, safety_margin_string);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("CollisionConstraintConfig: Error parsing SafetyMargin string");

    if (!tesseract_common::isNumeric(safety_margin_string))
      throw std::runtime_error("CollisionConstraintConfig: SafetyMargin is not a numeric values.");

    tesseract_common::toNumeric<double>(safety_margin_string, safety_margin);
  }

  if (safety_margin_buffer_element)
  {
    std::string safety_margin_buffer_string;
    status = tesseract_common::QueryStringText(safety_margin_buffer_element, safety_margin_buffer_string);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("CollisionConstraintConfig: Error parsing SafetyMarginBuffer string");

    if (!tesseract_common::isNumeric(safety_margin_buffer_string))
      throw std::runtime_error("CollisionConstraintConfig: SafetyMarginBuffer is not a numeric values.");

    tesseract_common::toNumeric<double>(safety_margin_buffer_string, safety_margin_buffer);
  }

  if (coeff_element)
  {
    std::string coeff_string;
    status = tesseract_common::QueryStringText(coeff_element, coeff_string);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("CollisionConstraintConfig: Error parsing Coefficient string");

    if (!tesseract_common::isNumeric(coeff_string))
      throw std::runtime_error("CollisionConstraintConfig: Coefficient is not a numeric values.");

    tesseract_common::toNumeric<double>(coeff_string, coeff);
  }
}

tinyxml2::XMLElement* CollisionConstraintConfig::toXML(tinyxml2::XMLDocument& doc) const
{
  tinyxml2::XMLElement* xml_coll_cnt_config = doc.NewElement("CollisionConstraintConfig");

  tinyxml2::XMLElement* xml_enabled = doc.NewElement("Enabled");
  xml_enabled->SetText(enabled);
  xml_coll_cnt_config->InsertEndChild(xml_enabled);

  tinyxml2::XMLElement* xml_use_weighted_sum = doc.NewElement("UseWeightedSum");
  xml_use_weighted_sum->SetText(use_weighted_sum);
  xml_coll_cnt_config->InsertEndChild(xml_use_weighted_sum);

  tinyxml2::XMLElement* xml_type = doc.NewElement("CollisionEvaluator");
  xml_type->SetAttribute("type", std::to_string(static_cast<int>(type)).c_str());
  xml_coll_cnt_config->InsertEndChild(xml_type);

  tinyxml2::XMLElement* xml_safety_margin = doc.NewElement("SafetyMargin");
  xml_safety_margin->SetText(safety_margin);
  xml_coll_cnt_config->InsertEndChild(xml_safety_margin);

  tinyxml2::XMLElement* xml_safety_margin_buffer = doc.NewElement("SafetyMarginBuffer");
  xml_safety_margin_buffer->SetText(safety_margin_buffer);
  xml_coll_cnt_config->InsertEndChild(xml_safety_margin_buffer);

  tinyxml2::XMLElement* xml_coeff = doc.NewElement("Coefficient");
  xml_coeff->SetText(coeff);
  xml_coll_cnt_config->InsertEndChild(xml_coeff);

  return xml_coll_cnt_config;
}
}  // namespace tesseract_planning
