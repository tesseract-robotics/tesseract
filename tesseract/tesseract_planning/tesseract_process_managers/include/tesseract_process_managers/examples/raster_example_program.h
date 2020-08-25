#ifndef TESSERACT_PROCESS_MANAGERS_EXAMPLE_PROGRAM_H
#define TESSERACT_PROCESS_MANAGERS_EXAMPLE_PROGRAM_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <tesseract_command_language/command_language.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_planning
{
inline CompositeInstruction rasterExampleProgram()
{
  CompositeInstruction program("raster_program", CompositeInstructionOrder::ORDERED, ManipulatorInfo("manipulator"));

  // Start Joint Position for the program
  std::vector<std::string> joint_names = { "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6" };
  StateWaypoint swp1 = StateWaypoint(joint_names, Eigen::VectorXd::Zero(6));
  PlanInstruction start_instruction(swp1, PlanInstructionType::START);
  program.setStartInstruction(start_instruction);

  Waypoint wp1 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.8, -0.3, 0.8) *
                                   Eigen::Quaterniond(0, 0, -1.0, 0));

  // Define from start composite instruction
  PlanInstruction plan_f0(wp1, PlanInstructionType::FREESPACE, "freespace_profile");
  plan_f0.setDescription("from_start_plan");
  CompositeInstruction from_start;
  from_start.setDescription("from_start");
  from_start.push_back(plan_f0);
  program.push_back(from_start);

  //
  for (int i = 0; i < 4; ++i)
  {
    double x = 0.8 + (i * 0.1);
    Waypoint wp1 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(x, -0.3, 0.8) *
                                     Eigen::Quaterniond(0, 0, -1.0, 0));
    Waypoint wp2 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(x, -0.2, 0.8) *
                                     Eigen::Quaterniond(0, 0, -1.0, 0));
    Waypoint wp3 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(x, -0.1, 0.8) *
                                     Eigen::Quaterniond(0, 0, -1.0, 0));
    Waypoint wp4 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(x, 0.0, 0.8) *
                                     Eigen::Quaterniond(0, 0, -1.0, 0));
    Waypoint wp5 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(x, 0.1, 0.8) *
                                     Eigen::Quaterniond(0, 0, -1.0, 0));
    Waypoint wp6 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(x, 0.2, 0.8) *
                                     Eigen::Quaterniond(0, 0, -1.0, 0));
    Waypoint wp7 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(x, 0.3, 0.8) *
                                     Eigen::Quaterniond(0, 0, -1.0, 0));

    CompositeInstruction raster_segment;
    raster_segment.setDescription("Raster #" + std::to_string(i + 1));
    if (i == 0 || i == 3)
    {
      raster_segment.push_back(PlanInstruction(wp2, PlanInstructionType::LINEAR, "RASTER"));
      raster_segment.push_back(PlanInstruction(wp3, PlanInstructionType::LINEAR, "RASTER"));
      raster_segment.push_back(PlanInstruction(wp4, PlanInstructionType::LINEAR, "RASTER"));
      raster_segment.push_back(PlanInstruction(wp5, PlanInstructionType::LINEAR, "RASTER"));
      raster_segment.push_back(PlanInstruction(wp6, PlanInstructionType::LINEAR, "RASTER"));
      raster_segment.push_back(PlanInstruction(wp7, PlanInstructionType::LINEAR, "RASTER"));
    }
    else
    {
      raster_segment.push_back(PlanInstruction(wp6, PlanInstructionType::LINEAR, "RASTER"));
      raster_segment.push_back(PlanInstruction(wp5, PlanInstructionType::LINEAR, "RASTER"));
      raster_segment.push_back(PlanInstruction(wp4, PlanInstructionType::LINEAR, "RASTER"));
      raster_segment.push_back(PlanInstruction(wp3, PlanInstructionType::LINEAR, "RASTER"));
      raster_segment.push_back(PlanInstruction(wp2, PlanInstructionType::LINEAR, "RASTER"));
      raster_segment.push_back(PlanInstruction(wp1, PlanInstructionType::LINEAR, "RASTER"));
    }
    program.push_back(raster_segment);

    // Add transition
    Instruction tranisiton_instruction = NullInstruction();
    if (i == 0)
    {
      Waypoint wp7 =
          CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.8 + ((i + 1) * 0.1), 0.3, 0.8) *
                            Eigen::Quaterniond(0, 0, -1.0, 0));

      PlanInstruction plan_f1(wp7, PlanInstructionType::FREESPACE, "freespace_profile");
      plan_f1.setDescription("transition_from_end_plan");
      tranisiton_instruction = plan_f1;
    }
    else if (i == 1)
    {
      Waypoint wp1 =
          CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.8 + ((i + 1) * 0.1), -0.3, 0.8) *
                            Eigen::Quaterniond(0, 0, -1.0, 0));

      PlanInstruction plan_f1(wp1, PlanInstructionType::FREESPACE, "freespace_profile");
      plan_f1.setDescription("transition_from_end_plan");
      tranisiton_instruction = plan_f1;
    }
    CompositeInstruction transition_from_end;
    transition_from_end.setDescription("transition_from_end");
    transition_from_end.push_back(tranisiton_instruction);

    CompositeInstruction transition_from_start;
    transition_from_start.setDescription("transition_from_start");
    transition_from_start.push_back(tranisiton_instruction);

    CompositeInstruction transitions("DEFAULT", CompositeInstructionOrder::UNORDERED);
    transitions.setDescription("transitions");
    transitions.push_back(transition_from_start);
    transitions.push_back(transition_from_end);
    program.push_back(transitions);
  }

  PlanInstruction plan_f2(swp1, PlanInstructionType::FREESPACE, "freespace_profile");
  plan_f2.setDescription("to_end_plan");
  CompositeInstruction to_end;
  to_end.setDescription("to_end");
  to_end.push_back(plan_f2);
  program.push_back(to_end);

  return program;
}

}  // namespace tesseract_planning

#endif
