/**
 * @file deserialize.h
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
#ifndef TESSERACT_COMMAND_LANGUAGE_DESERAILIZE_H
#define TESSERACT_COMMAND_LANGUAGE_DESERAILIZE_H
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <tinyxml2.h>
#include <functional>
#include <map>
TESSERACT_COMMON_IGNORE_WARNINGS_POP
#include <tesseract_command_language/core/instruction.h>

namespace tesseract_planning
{
using WaypointParserFn = std::function<Waypoint(const tinyxml2::XMLElement&, int)>;
using InstructionParserFn = std::function<Instruction(const tinyxml2::XMLElement&, int, WaypointParserFn)>;

Waypoint defaultWaypointParser(const tinyxml2::XMLElement& xml_element, int type);
Instruction defaultInstructionParser(const tinyxml2::XMLElement& xml_element,
                                     int type,
                                     WaypointParserFn waypoint_parser);

Instruction fromXMLDocument(const tinyxml2::XMLDocument& xml_doc,
                            InstructionParserFn instruction_parser = defaultInstructionParser,
                            WaypointParserFn waypoint_parser = defaultWaypointParser);

Instruction fromXMLFile(const std::string& file_path,
                        InstructionParserFn instruction_parser = defaultInstructionParser,
                        WaypointParserFn waypoint_parser = defaultWaypointParser);

Instruction fromXMLString(const std::string& xml_string,
                          InstructionParserFn instruction_parser = defaultInstructionParser,
                          WaypointParserFn waypoint_parser = defaultWaypointParser);

}  // namespace tesseract_planning

#endif  // TESSERACT_COMMAND_LANGUAGE_DESERAILIZE_H
