#include <tesseract_command_language/core/waypoint.h>
#include <tesseract_command_language/core/instruction.h>

#include <tesseract_command_language/joint_waypoint.h>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/plan_instruction.h>

int main(int argc, char* argv[])
{
  using namespace tesseract_planning;

  // Start Joint Position for the program
  Waypoint wp1 = JointWaypoint(Eigen::VectorXd::Ones(6));

  // Define raster posese
  Waypoint wp2 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(1, -1, 1));
  Waypoint wp3 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(1, -0.4, 1));
  Waypoint wp4 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(1, -0.2, 1));
  Waypoint wp5 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(1, 0.0, 1));
  Waypoint wp6 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(1, 0.2, 1));
  Waypoint wp7 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(1, 0.4, 1));

  JointWaypoint* cast_test1 = wp1.cast<JointWaypoint>();
  const CartesianWaypoint* cast_test2 = wp2.cast_const<CartesianWaypoint>();

  // Define raster move instruction
  PlanInstruction plan_c1(wp3, PlanInstructionType::LINEAR);
  PlanInstruction plan_c2(wp4, PlanInstructionType::LINEAR);
  PlanInstruction plan_c3(wp5, PlanInstructionType::LINEAR);
  PlanInstruction plan_c4(wp6, PlanInstructionType::LINEAR);
  PlanInstruction plan_c5(wp7, PlanInstructionType::LINEAR);

  // Create a program
  CompositeInstruction program;
  program.setStartWaypoint(wp1);

  PlanInstruction plan_f1(wp2, PlanInstructionType::FREESPACE);

  CompositeInstruction raster1;
  raster1.push_back(plan_f1);
  raster1.push_back(plan_c1);
  raster1.push_back(plan_c2);
  raster1.push_back(plan_c3);
  raster1.push_back(plan_c4);
  raster1.push_back(plan_c5);
  program.push_back(raster1);

  PlanInstruction plan_f2(wp2, PlanInstructionType::FREESPACE);
  CompositeInstruction raster2;
  raster2.push_back(plan_f2);
  raster2.push_back(plan_c1);
  raster2.push_back(plan_c2);
  raster2.push_back(plan_c3);
  raster2.push_back(plan_c4);
  raster2.push_back(plan_c5);
  program.push_back(raster2);

  PlanInstruction plan_f3(wp2, PlanInstructionType::FREESPACE);

  CompositeInstruction raster3;
  raster3.push_back(plan_f3);
  raster3.push_back(plan_c1);
  raster3.push_back(plan_c2);
  raster3.push_back(plan_c3);
  raster3.push_back(plan_c4);
  raster3.push_back(plan_c5);
  program.push_back(raster3);

  program.push_back(plan_f3);

  //  struct ProgramMetaData
  //  {
  //    Eigen::VectorXd current_position;
  //  };

  //  struct Program
  //  {
  //    ProgramMetaData metadata;
  //    CompositeInstruction instructions;
  //  };
}
