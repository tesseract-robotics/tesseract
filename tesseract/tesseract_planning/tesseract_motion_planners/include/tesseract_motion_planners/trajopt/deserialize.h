/**
 * @file deserialize.h
 * @brief Provide methods for deserialize instructions to xml and deserialization
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
#ifndef TESSERACT_MOTION_PLANNERS_TRAJOPT_DESERIALIZE_H
#define TESSERACT_MOTION_PLANNERS_TRAJOPT_DESERIALIZE_H
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <tinyxml2.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_plan_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_composite_profile.h>

namespace tesseract_planning
{
TrajOptDefaultPlanProfile trajOptPlanParser(const tinyxml2::XMLElement& xml_element);

TrajOptDefaultPlanProfile trajOptPlanFromXMLElement(const tinyxml2::XMLElement* profile_xml);

TrajOptDefaultPlanProfile trajOptPlanFromXMLDocument(const tinyxml2::XMLDocument& xml_doc);

TrajOptDefaultPlanProfile trajOptPlanFromXMLFile(const std::string& file_path);

TrajOptDefaultPlanProfile trajOptPlanFromXMLString(const std::string& xml_string);

TrajOptDefaultCompositeProfile trajOptCompositeParser(const tinyxml2::XMLElement& xml_element);

TrajOptDefaultCompositeProfile trajOptCompositeFromXMLElement(const tinyxml2::XMLElement* profile_xml);

TrajOptDefaultCompositeProfile trajOptCompositeFromXMLDocument(const tinyxml2::XMLDocument& xml_doc);

TrajOptDefaultCompositeProfile trajOptCompositeFromXMLFile(const std::string& file_path);

TrajOptDefaultCompositeProfile trajOptCompositeFromXMLString(const std::string& xml_string);

}  // namespace tesseract_planning

#endif  // TESSERACT_MOTION_PLANNERS_TRAJOPT_DESERIALIZE_H
