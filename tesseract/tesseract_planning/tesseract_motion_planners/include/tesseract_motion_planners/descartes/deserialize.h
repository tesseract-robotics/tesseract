/**
 * @file deserialize.h
 * @brief Provide methods for deserialize descartes plans to xml
 *
 * @author Tyler Marr
 * @date August 25, 2020
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
#ifndef TESSERACT_MOTION_PLANNERS_DESCARTES_DESERIALIZE_H
#define TESSERACT_MOTION_PLANNERS_DESCARTES_DESERIALIZE_H
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <tinyxml2.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP
#include <tesseract_motion_planners/descartes/profile/descartes_default_plan_profile.h>

namespace tesseract_planning
{
DescartesDefaultPlanProfile<double> descartesPlanParser(const tinyxml2::XMLElement& xml_element);

DescartesDefaultPlanProfile<double> descartesPlanFromXMLElement(const tinyxml2::XMLElement* profile_xml);

DescartesDefaultPlanProfile<double> descartesPlanFromXMLDocument(const tinyxml2::XMLDocument& xml_doc);

DescartesDefaultPlanProfile<double> descartesPlanFromXMLFile(const std::string& file_path);

DescartesDefaultPlanProfile<double> descartesPlanFromXMLString(const std::string& xml_string);

}  // namespace tesseract_planning

#endif  // TESSERACT_MOTION_PLANNERS_DESCARTES_DESERIALIZE_H
