/**
 * @file serialize_test.cpp
 * @brief
 *
 * @author Levi Armstrong
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
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/command_language.h>
#include <tesseract_command_language/utils/utils.h>
#include <tesseract_command_language/serialize.h>
#include <tesseract_command_language/deserialize.h>
#include <tesseract_common/utils.h>

using namespace tesseract_planning;

bool DEBUG = false;

CompositeInstruction getProgram()
{
  CompositeInstruction program("raster_program", CompositeInstructionOrder::ORDERED, ManipulatorInfo("manipulator"));

  // Start Joint Position for the program
  std::vector<std::string> joint_names = { "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6" };
  Waypoint wp0 = StateWaypoint(joint_names, Eigen::VectorXd::Zero(6));
  PlanInstruction start_instruction(wp0, PlanInstructionType::START);
  program.setStartInstruction(start_instruction);

  // Define raster poses
  Waypoint wp1 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.8, -0.3, 0.8) *
                                   Eigen::Quaterniond(0, 0, -1.0, 0));
  Waypoint wp2 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.8, -0.2, 0.8) *
                                   Eigen::Quaterniond(0, 0, -1.0, 0));
  Waypoint wp3 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.8, -0.1, 0.8) *
                                   Eigen::Quaterniond(0, 0, -1.0, 0));
  Waypoint wp4 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.8, 0.0, 0.8) *
                                   Eigen::Quaterniond(0, 0, -1.0, 0));
  Waypoint wp5 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.8, 0.1, 0.8) *
                                   Eigen::Quaterniond(0, 0, -1.0, 0));
  Waypoint wp6 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.8, 0.2, 0.8) *
                                   Eigen::Quaterniond(0, 0, -1.0, 0));
  Waypoint wp7 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.8, 0.3, 0.8) *
                                   Eigen::Quaterniond(0, 0, -1.0, 0));

  // Define raster move instruction
  PlanInstruction plan_c0(wp2, PlanInstructionType::LINEAR, "RASTER");
  PlanInstruction plan_c1(wp3, PlanInstructionType::LINEAR, "RASTER");
  PlanInstruction plan_c2(wp4, PlanInstructionType::LINEAR, "RASTER");
  PlanInstruction plan_c3(wp5, PlanInstructionType::LINEAR, "RASTER");
  PlanInstruction plan_c4(wp6, PlanInstructionType::LINEAR, "RASTER");
  PlanInstruction plan_c5(wp7, PlanInstructionType::LINEAR, "RASTER");

  PlanInstruction plan_f0(wp1, PlanInstructionType::FREESPACE, "freespace_profile");
  plan_f0.setDescription("from_start_plan");
  CompositeInstruction from_start;
  from_start.setDescription("from_start");
  from_start.push_back(plan_f0);
  program.push_back(from_start);

  {
    CompositeInstruction raster_segment;
    raster_segment.setDescription("raster_segment");
    raster_segment.push_back(plan_c0);
    raster_segment.push_back(plan_c1);
    raster_segment.push_back(plan_c2);
    raster_segment.push_back(plan_c3);
    raster_segment.push_back(plan_c4);
    raster_segment.push_back(plan_c5);
    program.push_back(raster_segment);
  }

  {
    PlanInstruction plan_f1(wp1, PlanInstructionType::FREESPACE, "freespace_profile");
    plan_f1.setDescription("transition_from_end_plan");
    CompositeInstruction transition_from_end;
    transition_from_end.setDescription("transition_from_end");
    transition_from_end.push_back(plan_f1);
    CompositeInstruction transition_from_start;
    transition_from_start.setDescription("transition_from_start");
    transition_from_start.push_back(plan_f1);

    CompositeInstruction transitions("DEFAULT", CompositeInstructionOrder::UNORDERED);
    transitions.setDescription("transitions");
    transitions.push_back(transition_from_start);
    transitions.push_back(transition_from_end);
    program.push_back(transitions);
  }

  {
    CompositeInstruction raster_segment;
    raster_segment.setDescription("raster_segment");
    raster_segment.push_back(plan_c0);
    raster_segment.push_back(plan_c1);
    raster_segment.push_back(plan_c2);
    raster_segment.push_back(plan_c3);
    raster_segment.push_back(plan_c4);
    raster_segment.push_back(plan_c5);
    program.push_back(raster_segment);
  }

  {
    PlanInstruction plan_f1(wp1, PlanInstructionType::FREESPACE, "freespace_profile");
    plan_f1.setDescription("transition_from_end_plan");
    CompositeInstruction transition_from_end;
    transition_from_end.setDescription("transition_from_end");
    transition_from_end.push_back(plan_f1);
    CompositeInstruction transition_from_start;
    transition_from_start.setDescription("transition_from_start");
    transition_from_start.push_back(plan_f1);

    CompositeInstruction transitions("DEFAULT", CompositeInstructionOrder::UNORDERED);
    transitions.setDescription("transitions");
    transitions.push_back(transition_from_start);
    transitions.push_back(transition_from_end);
    program.push_back(transitions);
  }

  {
    CompositeInstruction raster_segment;
    raster_segment.setDescription("raster_segment");
    raster_segment.push_back(plan_c0);
    raster_segment.push_back(plan_c1);
    raster_segment.push_back(plan_c2);
    raster_segment.push_back(plan_c3);
    raster_segment.push_back(plan_c4);
    raster_segment.push_back(plan_c5);
    program.push_back(raster_segment);
  }

  PlanInstruction plan_f2(wp1, PlanInstructionType::FREESPACE, "freespace_profile");
  plan_f2.setDescription("to_end_plan");
  CompositeInstruction to_end;
  to_end.setDescription("to_end");
  to_end.push_back(plan_f2);
  program.push_back(to_end);

  return program;
}

TEST(TesseractCommandLanguageSerializeUnit, SerializeToXml)  // NOLINT
{
  // Write program to file
  CompositeInstruction program = getProgram();
  toXMLFile(program, tesseract_common::getTempPath() + "raster_example_input.xml");

  Instruction imported_program = fromXMLFile(tesseract_common::getTempPath() + "raster_example_input.xml");

  EXPECT_TRUE(isCompositeInstruction(imported_program));
  const auto* ci = imported_program.cast_const<CompositeInstruction>();
  EXPECT_EQ(program.size(), ci->size());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
