/**
 * @file plan_instruction.cpp
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

#include <tesseract_command_language/plan_instruction.h>

namespace tesseract_planning
{
PlanInstruction::PlanInstruction(Waypoint waypoint,
                                 PlanInstructionType type,
                                 std::string profile,
                                 ManipulatorInfo manipulator_info)
  : plan_type_(type), waypoint_(std::move(waypoint)), profile_(std::move(profile)), manipulator_info_(manipulator_info)
{
}

void PlanInstruction::setWaypoint(Waypoint waypoint) { waypoint_ = waypoint; }
Waypoint& PlanInstruction::getWaypoint() { return waypoint_; }
const Waypoint& PlanInstruction::getWaypoint() const { return waypoint_; }

void PlanInstruction::setManipulatorInfo(ManipulatorInfo info) { manipulator_info_ = info; }
const ManipulatorInfo& PlanInstruction::getManipulatorInfo() const { return manipulator_info_; }
ManipulatorInfo& PlanInstruction::getManipulatorInfo() { return manipulator_info_; }

void PlanInstruction::setProfile(const std::string& profile)
{
  profile_ = (profile.empty()) ? DEFAULT_PROFILE_KEY : profile;
}
const std::string& PlanInstruction::getProfile() const { return profile_; }

int PlanInstruction::getType() const { return type_; }

const std::string& PlanInstruction::getDescription() const { return description_; }

void PlanInstruction::setDescription(const std::string& description) { description_ = description; }

void PlanInstruction::print(std::string prefix) const
{
  std::cout << prefix + "Plan Instruction, Type: " << getType() << ", Plan Type: " << static_cast<int>(plan_type_)
            << ", ";
  getWaypoint().print();
  std::cout << ", Description: " << getDescription() << std::endl;
}

PlanInstructionType PlanInstruction::getPlanType() const { return plan_type_; }

void PlanInstruction::setPlanType(PlanInstructionType type) { plan_type_ = type; }

bool PlanInstruction::isLinear() const { return (plan_type_ == PlanInstructionType::LINEAR); }

bool PlanInstruction::isFreespace() const { return (plan_type_ == PlanInstructionType::FREESPACE); }

bool PlanInstruction::isCircular() const { return (plan_type_ == PlanInstructionType::CIRCULAR); }

bool PlanInstruction::isStart() const { return (plan_type_ == PlanInstructionType::START); }

tinyxml2::XMLElement* PlanInstruction::toXML(tinyxml2::XMLDocument& doc) const
{
  tinyxml2::XMLElement* xml_instruction = doc.NewElement("Instruction");
  xml_instruction->SetAttribute("type", std::to_string(getType()).c_str());

  tinyxml2::XMLElement* xml_plan_instruction = doc.NewElement("PlanInstruction");
  xml_plan_instruction->SetAttribute("type", std::to_string(static_cast<int>(getPlanType())).c_str());

  tinyxml2::XMLElement* xml_description = doc.NewElement("Description");
  xml_description->SetText(getDescription().c_str());
  xml_plan_instruction->InsertEndChild(xml_description);

  tinyxml2::XMLElement* xml_profile = doc.NewElement("Profile");
  xml_profile->SetText(getProfile().c_str());
  xml_plan_instruction->InsertEndChild(xml_profile);

  if (!getManipulatorInfo().empty())
  {
    tinyxml2::XMLElement* xml_manip_info = getManipulatorInfo().toXML(doc);
    xml_plan_instruction->InsertEndChild(xml_manip_info);
  }

  tinyxml2::XMLElement* xml_waypoint = getWaypoint().toXML(doc);
  xml_plan_instruction->InsertEndChild(xml_waypoint);

  xml_instruction->InsertEndChild(xml_plan_instruction);

  return xml_instruction;
}

}  // namespace tesseract_planning
