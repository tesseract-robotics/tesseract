/**
 * @file null_waypoint.cpp
 * @brief
 *
 * @author Levi Armstrong
 * @date June 15, 2020
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
#include <iostream>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/null_waypoint.h>
#include <tesseract_command_language/waypoint_type.h>

namespace tesseract_planning
{
NullWaypoint::NullWaypoint(const tinyxml2::XMLElement& /*xml_element*/) {}

int NullWaypoint::getType() const { return static_cast<int>(WaypointType::NULL_WAYPOINT); }

void NullWaypoint::print(const std::string& prefix) const { std::cout << prefix << "Null WP"; };

tinyxml2::XMLElement* NullWaypoint::toXML(tinyxml2::XMLDocument& doc) const
{
  tinyxml2::XMLElement* xml_waypoint = doc.NewElement("Waypoint");
  xml_waypoint->SetAttribute("type", std::to_string(getType()).c_str());

  tinyxml2::XMLElement* xml_null_waypoint = doc.NewElement("NullWaypoint");
  xml_waypoint->InsertEndChild(xml_null_waypoint);

  return xml_waypoint;
}

}  // namespace tesseract_planning
