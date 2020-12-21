/**
 * @file timer_instruction.h
 * @brief
 *
 * @author Levi Armstrong
 * @date November 15, 2020
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
#ifndef TESSERACT_COMMAND_LANGUAGE_TIMER_INSTRUCTION_H
#define TESSERACT_COMMAND_LANGUAGE_TIMER_INSTRUCTION_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <iostream>
#include <string>
#include <tinyxml2.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/instruction_type.h>

namespace tesseract_planning
{
enum class TimerInstructionType : int
{
  DIGITAL_OUTPUT_HIGH = 0,
  DIGITAL_OUTPUT_LOW = 1
};

/**
 * @brief This instruction indicates that a timer should be started and when the time expires it either sets a digital
 * output high(1) or low(0).
 *
 *   - DIGITAL_OUTPUT_HIGH : The digital output will be set to high(1) when the timer expires
 *   - DIGITAL_OUTPUT_LOW  : The digital output will be set to low(0) when the timer expires
 */
class TimerInstruction
{
public:
  TimerInstruction(TimerInstructionType type, double time, int io) : timer_type_(type), timer_time_(time), timer_io_(io)
  {
  }

  int getType() const { return static_cast<int>(InstructionType::TIMER_INSTRUCTION); }

  const std::string& getDescription() const { return description_; }

  void setDescription(const std::string& description) { description_ = description; }

  void print(std::string prefix = "") const  // NOLINT
  {
    std::cout << prefix + "Timer Instruction, Type: " << getType() << ", Timer Type: " << static_cast<int>(timer_type_)
              << ", Time: " << timer_time_ << ", IO: " << timer_io_;
    std::cout << ", Description: " << getDescription() << std::endl;
  }

  /**
   * @brief Get the timer type
   * @return The timer type
   */
  TimerInstructionType getTimerType() const { return timer_type_; }

  /**
   * @brief Set the timer type
   * @param type The timer type
   */
  void setTimerType(TimerInstructionType type) { timer_type_ = type; }

  /**
   * @brief Get timer time in second
   * @return The timer time in second
   */
  double getTimerTime() const { return timer_time_; }
  /**
   * @brief Set timer time in second
   * @param time The timer time in second
   */
  void setTimerTime(double time) { timer_time_ = time; }

  /**
   * @brief Get the timer IO
   * @return The timer IO
   */
  int getTimerIO() const { return timer_io_; }

  /**
   * @brief Set the timer IO
   * @param io The timer IO
   */
  void setTimerIO(int io) { timer_io_ = io; }

  tinyxml2::XMLElement* toXML(tinyxml2::XMLDocument& doc) const
  {
    tinyxml2::XMLElement* xml_instruction = doc.NewElement("Instruction");
    xml_instruction->SetAttribute("type", std::to_string(getType()).c_str());

    tinyxml2::XMLElement* xml_timer_instruction = doc.NewElement("TimerInstruction");
    xml_timer_instruction->SetAttribute("type", std::to_string(static_cast<int>(getTimerType())).c_str());
    xml_timer_instruction->SetAttribute("time", std::to_string(timer_time_).c_str());
    xml_timer_instruction->SetAttribute("io", std::to_string(timer_io_).c_str());

    tinyxml2::XMLElement* xml_description = doc.NewElement("Description");
    xml_description->SetText(getDescription().c_str());
    xml_timer_instruction->InsertEndChild(xml_description);

    xml_timer_instruction->InsertEndChild(xml_description);
    xml_instruction->InsertEndChild(xml_timer_instruction);

    return xml_instruction;
  }

private:
  /** @brief The description of the instruction */
  std::string description_{ "Tesseract Timer Instruction" };
  TimerInstructionType timer_type_;
  double timer_time_{ 0 };
  int timer_io_{ -1 };
};
}  // namespace tesseract_planning

#ifdef SWIG
%tesseract_command_language_add_instruction_type(TimerInstruction)
#endif  // SWIG

#endif  // TESSERACT_COMMAND_LANGUAGE_TIMER_INSTRUCTION_H
