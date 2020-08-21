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
 tinyxml2::XMLElement* CollisionCostConfig::toXML(tinyxml2::XMLDocument& doc) const
 {
   tinyxml2::XMLElement* xml_coll_cost_config = doc.NewElement("CollisionCostConfig");

   tinyxml2::XMLElement* xml_enabled = doc.NewElement("Enabled");
   xml_enabled->SetText(enabled);
   xml_coll_cost_config->InsertEndChild(xml_enabled);

   tinyxml2::XMLElement* xml_use_weighted_sum = doc.NewElement("UseWeightedSum");
   xml_use_weighted_sum->SetText(use_weighted_sum);
   xml_coll_cost_config->InsertEndChild(xml_use_weighted_sum);

   tinyxml2::XMLElement* xml_type = doc.NewElement("Type");
   xml_type->SetText(static_cast<int>(type));
   xml_coll_cost_config->InsertEndChild(xml_type);

   tinyxml2::XMLElement* xml_buffer_margin = doc.NewElement("BufferMargin");
   xml_buffer_margin->SetText(buffer_margin);
   xml_coll_cost_config->InsertEndChild(xml_buffer_margin);

   tinyxml2::XMLElement* xml_safety_margin_buffer = doc.NewElement("SafetyMarginBuffer");
   xml_safety_margin_buffer->SetText(safety_margin_buffer);
   xml_coll_cost_config->InsertEndChild(xml_safety_margin_buffer);

   tinyxml2::XMLElement* xml_coeff = doc.NewElement("Coefficient");
   xml_coeff->SetText(coeff);
   xml_coll_cost_config->InsertEndChild(xml_coeff);

   return xml_coll_cost_config;
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

   tinyxml2::XMLElement* xml_type = doc.NewElement("Type");
   xml_type->SetText(static_cast<int>(type));
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
}


