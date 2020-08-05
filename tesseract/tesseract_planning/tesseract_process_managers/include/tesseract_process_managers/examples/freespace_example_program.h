#ifndef TESSERACT_PROCESS_MANAGER_EXAMPLE_PROGRAM_H
#define TESSERACT_PROCESS_MANAGER_EXAMPLE_PROGRAM_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <tesseract_command_language/command_language.h>

TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_planning
{
inline CompositeInstruction freespaceExampleProgram()
{
  CompositeInstruction program(
      "freespace_composite", CompositeInstructionOrder::ORDERED, ManipulatorInfo("manipulator"));

  // Start Joint Position for the program
  std::vector<std::string> joint_names = { "joint_a1", "joint_a2", "joint_a3", "joint_a4",
                                           "joint_a5", "joint_a6", "joint_a6" };
  Waypoint wp1 = StateWaypoint(joint_names, Eigen::VectorXd::Zero(7));
  PlanInstruction start_instruction(wp1, PlanInstructionType::START);
  program.setStartInstruction(start_instruction);

  // Define target pose
  Waypoint wp2 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.2, 0.2, 1.0));
  PlanInstruction plan_f0(wp2, PlanInstructionType::FREESPACE, "DEFAULT");
  plan_f0.setDescription("freespace_motion");
  program.push_back(plan_f0);

  return program;
}

}  // namespace tesseract_planning

#endif
