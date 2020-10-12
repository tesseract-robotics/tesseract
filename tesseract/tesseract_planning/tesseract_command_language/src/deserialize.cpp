/**
 * @file deserialize.cpp
 * @brief Provide methods for deserialize instructions to xml and deserialization
 *
 * @author Levi Armstrong
 * @date August 17, 2020
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
#include <array>
#include <console_bridge/console.h>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/utils.h>
#include <tesseract_command_language/command_language.h>
#include <tesseract_command_language/deserialize.h>

namespace tesseract_planning
{
Waypoint getNullWaypoint(const tinyxml2::XMLElement& /*waypoint_element*/) { return NullWaypoint(); }

Waypoint getCartesianWaypoint(const tinyxml2::XMLElement& waypoint_element)
{
  const tinyxml2::XMLElement* cwp_element = waypoint_element.FirstChildElement("CartesianWaypoint");
  return CartesianWaypoint(*cwp_element);
}

Waypoint getJointWaypoint(const tinyxml2::XMLElement& waypoint_element)
{
  const tinyxml2::XMLElement* jwp_element = waypoint_element.FirstChildElement("JointWaypoint");
  return JointWaypoint(*jwp_element);
}

Waypoint getStateWaypoint(const tinyxml2::XMLElement& waypoint_element)
{
  const tinyxml2::XMLElement* swp_element = waypoint_element.FirstChildElement("StateWaypoint");
  return StateWaypoint(*swp_element);
}

Instruction getNullInstruction(const tinyxml2::XMLElement& /*instruction_element*/) { return NullInstruction(); }

Instruction getPlanInstruction(const tinyxml2::XMLElement& instruction_element, WaypointParserFn waypoint_parser)
{
  const tinyxml2::XMLElement* plan_element = instruction_element.FirstChildElement("PlanInstruction");
  if (!plan_element)
    throw std::runtime_error("Missing Child Element MoveInstruction.");

  const tinyxml2::XMLElement* description_element = plan_element->FirstChildElement("Description");
  const tinyxml2::XMLElement* profile_element = plan_element->FirstChildElement("Profile");
  const tinyxml2::XMLElement* manip_info_element = plan_element->FirstChildElement("ManipulatorInfo");
  const tinyxml2::XMLElement* waypoint_element = plan_element->FirstChildElement("Waypoint");

  std::string description;
  if (description_element)
  {
    tinyxml2::XMLError status = tesseract_common::QueryStringText(description_element, description);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("fromXML: Error parsing Description string");
  }

  std::string profile;
  if (profile_element)
  {
    tinyxml2::XMLError status = tesseract_common::QueryStringText(profile_element, profile);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("fromXML: Error parsing Profile string");
  }

  int plan_type = -1;
  tinyxml2::XMLError status = plan_element->QueryIntAttribute("type", &plan_type);
  if (status != tinyxml2::XML_SUCCESS)
    throw std::runtime_error("fromXML: Failed to parse plan instruction type attribute.");

  // Get Waypoint
  int waypoint_type = -1;
  status = waypoint_element->QueryIntAttribute("type", &waypoint_type);
  if (status != tinyxml2::XML_SUCCESS)
    throw std::runtime_error("fromXML: Failed to parse plan instruction waypoint type attribute.");

  Waypoint wp = waypoint_parser(*waypoint_element, waypoint_type);

  PlanInstruction plan_instruction(wp, static_cast<PlanInstructionType>(plan_type), profile);

  // Get Manipulator Info
  if (manip_info_element)
    plan_instruction.setManipulatorInfo(ManipulatorInfo(*manip_info_element));

  plan_instruction.setDescription(description);
  return plan_instruction;
}

Instruction getMoveInstruction(const tinyxml2::XMLElement& instruction_element, WaypointParserFn waypoint_parser)
{
  const tinyxml2::XMLElement* move_element = instruction_element.FirstChildElement("MoveInstruction");
  if (!move_element)
    throw std::runtime_error("Missing Child Element MoveInstruction.");

  const tinyxml2::XMLElement* description_element = move_element->FirstChildElement("Description");
  const tinyxml2::XMLElement* profile_element = move_element->FirstChildElement("Profile");
  const tinyxml2::XMLElement* manip_info_element = move_element->FirstChildElement("ManipulatorInfo");
  const tinyxml2::XMLElement* waypoint_element = move_element->FirstChildElement("Waypoint");

  std::string description;
  if (description_element)
  {
    tinyxml2::XMLError status = tesseract_common::QueryStringText(description_element, description);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("fromXML: Error parsing Description string");
  }

  std::string profile;
  if (profile_element)
  {
    tinyxml2::XMLError status = tesseract_common::QueryStringText(profile_element, profile);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("fromXML: Error parsing Profile string");
  }

  int move_type = -1;
  tinyxml2::XMLError status = move_element->QueryIntAttribute("type", &move_type);
  if (status != tinyxml2::XML_SUCCESS)
    throw std::runtime_error("fromXML: Failed to parse move instruction type attribute.");

  // Get Waypoint
  int waypoint_type = -1;
  status = waypoint_element->QueryIntAttribute("type", &waypoint_type);
  if (status != tinyxml2::XML_SUCCESS)
    throw std::runtime_error("fromXML: Failed to parse move instruction waypoint type attribute.");

  Waypoint wp = waypoint_parser(*waypoint_element, waypoint_type);

  MoveInstruction move_instruction(wp, static_cast<MoveInstructionType>(move_type), profile);

  // Get Manipulator Info
  if (manip_info_element)
    move_instruction.setManipulatorInfo(ManipulatorInfo(*manip_info_element));

  move_instruction.setDescription(description);
  return move_instruction;
}

Instruction startInstructionfromXML(const tinyxml2::XMLElement& start_instruction_xml,
                                    InstructionParserFn instruction_parser,
                                    WaypointParserFn waypoint_parser)
{
  const tinyxml2::XMLElement* instruction_xml = start_instruction_xml.FirstChildElement("Instruction");
  if (!instruction_xml)
    throw std::runtime_error("Missing Child Element Instruction.");

  int type = -1;
  tinyxml2::XMLError status = instruction_xml->QueryIntAttribute("type", &type);
  if (status != tinyxml2::XML_SUCCESS)
    throw std::runtime_error("fromXML: Failed to parse start instruction type attribute.");

  return instruction_parser(*instruction_xml, type, waypoint_parser);
}

Instruction getCompositeInstruction(const tinyxml2::XMLElement& instruction_element,
                                    InstructionParserFn instruction_parser,
                                    WaypointParserFn waypoint_parser)
{
  const tinyxml2::XMLElement* composite_element = instruction_element.FirstChildElement("CompositeInstruction");
  if (!composite_element)
    throw std::runtime_error("Missing Child Element CompositeInstruction");

  const tinyxml2::XMLElement* description_element = composite_element->FirstChildElement("Description");
  const tinyxml2::XMLElement* profile_element = composite_element->FirstChildElement("Profile");
  const tinyxml2::XMLElement* manip_info_element = composite_element->FirstChildElement("ManipulatorInfo");
  const tinyxml2::XMLElement* start_instruction_element = composite_element->FirstChildElement("StartInstruction");

  std::string description;
  if (description_element)
  {
    tinyxml2::XMLError status = tesseract_common::QueryStringText(description_element, description);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("fromXML: Error parsing Description string");
  }

  std::string profile;
  if (profile_element)
  {
    tinyxml2::XMLError status = tesseract_common::QueryStringText(profile_element, profile);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("fromXML: Error parsing Profile string");
  }

  int order_type = -1;
  tinyxml2::XMLError status = composite_element->QueryIntAttribute("order", &order_type);
  if (status != tinyxml2::XML_SUCCESS)
    throw std::runtime_error("fromXML: Failed to parse instruction type attribute.");

  CompositeInstruction composite(profile, CompositeInstructionOrder(order_type));

  // Get Manipulator Info
  if (manip_info_element)
    composite.setManipulatorInfo(ManipulatorInfo(*manip_info_element));

  composite.setDescription(description);

  if (start_instruction_element)
    composite.setStartInstruction(
        startInstructionfromXML(*start_instruction_element, instruction_parser, waypoint_parser));

  for (const tinyxml2::XMLElement* child_instruction = composite_element->FirstChildElement("Instruction");
       child_instruction;
       child_instruction = child_instruction->NextSiblingElement("Instruction"))
  {
    int type = -1;
    status = child_instruction->QueryIntAttribute("type", &type);
    if (status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("fromXML: Failed to parse instruction type attribute.");

    composite.push_back(instruction_parser(*child_instruction, type, waypoint_parser));
  }

  return composite;
}

Instruction defaultInstructionParser(const tinyxml2::XMLElement& xml_element,
                                     int type,
                                     WaypointParserFn waypoint_parser)
{
  switch (type)
  {
    case static_cast<int>(InstructionType::NULL_INSTRUCTION):
    {
      return getNullInstruction(xml_element);
    }
    case static_cast<int>(InstructionType::PLAN_INSTRUCTION):
    {
      return getPlanInstruction(xml_element, waypoint_parser);
    }
    case static_cast<int>(InstructionType::MOVE_INSTRUCTION):
    {
      return getMoveInstruction(xml_element, waypoint_parser);
    }
    case static_cast<int>(InstructionType::COMPOSITE_INSTRUCTION):
    {
      return getCompositeInstruction(xml_element, defaultInstructionParser, waypoint_parser);
    }
    default:
    {
      throw std::runtime_error("Unsupported Instruction Type!");
    }
  }
}

Waypoint defaultWaypointParser(const tinyxml2::XMLElement& xml_element, int type)
{
  switch (type)
  {
    case static_cast<int>(WaypointType::NULL_WAYPOINT):
    {
      return getNullWaypoint(xml_element);
    }
    case static_cast<int>(WaypointType::CARTESIAN_WAYPOINT):
    {
      return getCartesianWaypoint(xml_element);
    }
    case static_cast<int>(WaypointType::JOINT_WAYPOINT):
    {
      return getJointWaypoint(xml_element);
    }
    case static_cast<int>(WaypointType::STATE_WAYPOINT):
    {
      return getStateWaypoint(xml_element);
    }
    default:
    {
      throw std::runtime_error("Unsupported Waypoint Type!");
    }
  }
}

Instruction fromXMLElement(const tinyxml2::XMLElement* cl_xml,
                           InstructionParserFn instruction_parser,
                           WaypointParserFn waypoint_parser)
{
  std::array<int, 3> version;
  std::string version_string;
  tinyxml2::XMLError status = tesseract_common::QueryStringAttribute(cl_xml, "version", version_string);
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

  const tinyxml2::XMLElement* instruction_xml = cl_xml->FirstChildElement("Instruction");
  if (!instruction_xml)
    throw std::runtime_error("fromXML: Could not find the 'Instruction' element in the xml file.");

  int type = static_cast<int>(InstructionType::NULL_INSTRUCTION);
  status = instruction_xml->QueryIntAttribute("type", &type);
  if (status != tinyxml2::XML_SUCCESS)
    throw std::runtime_error("fromXML: Failed to parse instruction type attribute.");

  return instruction_parser(*instruction_xml, type, waypoint_parser);
}

Instruction fromXMLDocument(const tinyxml2::XMLDocument& xml_doc,
                            InstructionParserFn instruction_parser,
                            WaypointParserFn waypoint_parser)
{
  const tinyxml2::XMLElement* cl_xml = xml_doc.FirstChildElement("CommandLanguage");
  if (!cl_xml)
    throw std::runtime_error("Could not find the 'CommandLanguage' element in the xml file");

  return fromXMLElement(cl_xml, instruction_parser, waypoint_parser);
}

Instruction fromXMLString(const std::string& xml_string,
                          InstructionParserFn instruction_parser,
                          WaypointParserFn waypoint_parser)
{
  tinyxml2::XMLDocument xml_doc;
  tinyxml2::XMLError status = xml_doc.Parse(xml_string.c_str());
  if (status != tinyxml2::XMLError::XML_SUCCESS)
    throw std::runtime_error("Could not parse the Instruction XML File.");

  return fromXMLDocument(xml_doc, instruction_parser, waypoint_parser);
}

Instruction fromXMLFile(const std::string& file_path,
                        InstructionParserFn instruction_parser,
                        WaypointParserFn waypoint_parser)
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
    return fromXMLString(xml_string, instruction_parser, waypoint_parser);
  }

  throw std::runtime_error("Could not open file " + file_path + "for parsing.");
}

}  // namespace tesseract_planning
