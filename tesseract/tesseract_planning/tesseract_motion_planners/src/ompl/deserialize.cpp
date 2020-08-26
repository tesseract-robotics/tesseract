/**
 * @file deserialize.cpp
 * @brief Provide methods for deserialize instructions to xml and deserialization
 *
 * @author Tyler Marr
 * @date August 21, 2020
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
#include <string>
#include <fstream>
#include <console_bridge/console.h>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/utils.h>
#include <tesseract_motion_planners/ompl/deserialize.h>
#include <tesseract_motion_planners/ompl/profile/ompl_default_plan_profile.h>

namespace tesseract_planning
{
OMPLDefaultPlanProfile omplPlanParser(const tinyxml2::XMLElement& xml_element)
{
  const tinyxml2::XMLElement* ompl_plan_element = xml_element.FirstChildElement("OMPLPlanProfile");
  return OMPLDefaultPlanProfile(*ompl_plan_element);
}

OMPLDefaultPlanProfile omplPlanFromXMLElement(const tinyxml2::XMLElement* profile_xml)
{
  std::array<int, 3> version;
  std::string version_string;
  tinyxml2::XMLError status = tesseract_common::QueryStringAttribute(profile_xml, "version", version_string);
  if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
    throw std::runtime_error("fromXML: Error parsing robot attribute 'version'");

  if (status != tinyxml2::XML_NO_ATTRIBUTE)
  {
    std::vector<std::string> tokens;
    boost::split(tokens, version_string, boost::is_any_of("."), boost::token_compress_on);
    if (tokens.size() < 2 || tokens.size() > 3 || !tesseract_common::isNumeric(tokens))
      throw std::runtime_error("fromXML: Error parsing robot attribute 'version'");

    tesseract_common::toNumeric<int>(tokens[0], version[0]);
    tesseract_common::toNumeric<int>(tokens[1], version[1]);
    if (tokens.size() == 3)
      tesseract_common::toNumeric<int>(tokens[2], version[2]);
    else
      version[2] = 0;
  }
  else
  {
    CONSOLE_BRIDGE_logWarn("No version number was provided so latest parser will be used.");
  }

  const tinyxml2::XMLElement* planner_xml = profile_xml->FirstChildElement("Planner");
  if (!planner_xml)
    throw std::runtime_error("fromXML: Could not find the 'Planner' element in the xml file.");

  int type;
  status = planner_xml->QueryIntAttribute("type", &type);
  if (status != tinyxml2::XML_SUCCESS)
    throw std::runtime_error("fromXML: Failed to parse instruction type attribute.");

  return omplPlanParser(*planner_xml);
}

OMPLDefaultPlanProfile omplPlanFromXMLDocument(const tinyxml2::XMLDocument& xml_doc)
{
  const tinyxml2::XMLElement* planner_xml = xml_doc.FirstChildElement("Profile");
  if (!planner_xml)
    throw std::runtime_error("Could not find the 'Profile' element in the xml file");

  return omplPlanFromXMLElement(planner_xml);
}

OMPLDefaultPlanProfile omplPlanFromXMLString(const std::string& xml_string)
{
  tinyxml2::XMLDocument xml_doc;
  tinyxml2::XMLError status = xml_doc.Parse(xml_string.c_str());
  if (status != tinyxml2::XMLError::XML_SUCCESS)
    throw std::runtime_error("Could not parse the Planner Profile XML File.");

  return omplPlanFromXMLDocument(xml_doc);
}

OMPLDefaultPlanProfile omplPlanFromXMLFile(const std::string& file_path)
{
  // get the entire file
  std::string xml_string;
  std::fstream xml_file(file_path.c_str(), std::fstream::in);
  if (xml_file.is_open())
  {
    while (xml_file.good())
    {
      std::string line;
      std::getline(xml_file, line);
      xml_string += (line + "\n");
    }
    xml_file.close();
    return omplPlanFromXMLString(xml_string);
  }

  throw std::runtime_error("Could not open file " + file_path + "for parsing.");
}

}  // namespace tesseract_planning
