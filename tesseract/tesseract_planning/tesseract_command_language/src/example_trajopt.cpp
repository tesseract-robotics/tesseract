#include <tesseract_command_language/core/waypoint.h>
#include <tesseract_command_language/core/component_info.h>
#include <tesseract_command_language/core/instruction.h>

#include <tesseract_command_language/joint_waypoint.h>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/plan_instruction.h>
#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/component_info_impl.h>

#include <tesseract_command_language/trajopt_planner_universal_config.h>
#include <tesseract_motion_planners/trajopt/trajopt_motion_planner.h>
#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_common/utils.h>


std::string locateResource(const std::string& url)
{
  std::string mod_url = url;
  if (url.find("package://tesseract_support") == 0)
  {
    mod_url.erase(0, strlen("package://tesseract_support"));
    size_t pos = mod_url.find('/');
    if (pos == std::string::npos)
    {
      return std::string();
    }

    std::string package = mod_url.substr(0, pos);
    mod_url.erase(0, pos);
    std::string package_path = std::string(TESSERACT_SUPPORT_DIR);

    if (package_path.empty())
    {
      return std::string();
    }

    mod_url = package_path + mod_url;
  }

  return mod_url;
}

tesseract_planning::CompositeInstruction generateSeed(const tesseract_planning::CompositeInstruction& instructions,
                                                      const Eigen::VectorXd& current_state)
{
  tesseract_planning::CompositeInstruction seed;
  const tesseract_planning::PlanInstruction* prev_plan_instruction {nullptr};
  tesseract_planning::JointWaypoint current_jwp(current_state);

  for (const auto& instruction : instructions)
  {
    if (instruction.isPlan())
    {
      const tesseract_planning::PlanInstruction* plan_instruction = instruction.cast_const<tesseract_planning::PlanInstruction>();
      if (plan_instruction->isLinear())
      {
        tesseract_planning::CompositeInstruction composite;
        if (prev_plan_instruction)
        {
          /** @todo This should also handle if waypoint type is joint */
          const auto* pre_cwp = prev_plan_instruction->getWaypoint().cast_const<tesseract_planning::CartesianWaypoint>();
          const auto* cur_cwp = plan_instruction->getWaypoint().cast_const<tesseract_planning::CartesianWaypoint>();

          tesseract_common::VectorIsometry3d poses = tesseract_motion_planners::interpolate(*pre_cwp, *cur_cwp, 10);
          for (std::size_t p = 1; p < poses.size(); ++p)
          {
            tesseract_planning::MoveInstruction move_instruction(tesseract_planning::CartesianWaypoint(poses[p]), tesseract_planning::MoveInstructionType::LINEAR);
            move_instruction.setPosition(current_state);
            move_instruction.setTCP(plan_instruction->getTCP());
            move_instruction.setWorkingFrame(plan_instruction->getWorkingFrame());
            move_instruction.setDescription(plan_instruction->getDescription());
            composite.push_back(move_instruction);
          }
        }
        else
        {
          tesseract_planning::MoveInstruction move_instruction(plan_instruction->getWaypoint(), tesseract_planning::MoveInstructionType::LINEAR);
          move_instruction.setTCP(plan_instruction->getTCP());
          move_instruction.setWorkingFrame(plan_instruction->getWorkingFrame());
          move_instruction.setDescription(plan_instruction->getDescription());
          composite.push_back(move_instruction);
        }
        seed.push_back(composite);
      }
      else if (plan_instruction->isFreespace())
      {
        tesseract_planning::CompositeInstruction composite;
        if (prev_plan_instruction)
        {
          /** @todo This should also handle if waypoint type is cartesian */
          const auto* pre_cwp = prev_plan_instruction->getWaypoint().cast_const<tesseract_planning::JointWaypoint>();
          const auto* cur_cwp = plan_instruction->getWaypoint().cast_const<tesseract_planning::JointWaypoint>();

          Eigen::MatrixXd states = tesseract_motion_planners::interpolate(*pre_cwp, *cur_cwp, 10);
          for (long i = 1; i < states.cols(); ++i)
          {
            tesseract_planning::MoveInstruction move_instruction(tesseract_planning::JointWaypoint(states.col(i)), tesseract_planning::MoveInstructionType::FREESPACE);
            move_instruction.setPosition(states.col(i));
            move_instruction.setTCP(plan_instruction->getTCP());
            move_instruction.setWorkingFrame(plan_instruction->getWorkingFrame());
            move_instruction.setDescription(plan_instruction->getDescription());
            composite.push_back(move_instruction);
          }
        }
        else
        {
          tesseract_planning::MoveInstruction move_instruction(plan_instruction->getWaypoint(), tesseract_planning::MoveInstructionType::FREESPACE);
          move_instruction.setPosition(*(plan_instruction->getWaypoint().cast_const<tesseract_planning::JointWaypoint>()));
          move_instruction.setTCP(plan_instruction->getTCP());
          move_instruction.setWorkingFrame(plan_instruction->getWorkingFrame());
          move_instruction.setDescription(plan_instruction->getDescription());
          composite.push_back(move_instruction);
        }
        seed.push_back(composite);
      }
      else
      {
        throw std::runtime_error("Unsupported!");
      }
      prev_plan_instruction = plan_instruction;
    }
    else
    {
      seed.push_back(instruction);
    }
  }
  return seed;
}

int main (int argc, char *argv[])
{
  using namespace tesseract_planning;

  // Create Tesseract
  tesseract_scene_graph::ResourceLocator::Ptr locator =
      std::make_shared<tesseract_scene_graph::SimpleResourceLocator>(locateResource);
  tesseract::Tesseract::Ptr tesseract = std::make_shared<tesseract::Tesseract>();
  boost::filesystem::path urdf_path(std::string(TESSERACT_SUPPORT_DIR) + "/urdf/lbr_iiwa_14_r820.urdf");
  boost::filesystem::path srdf_path(std::string(TESSERACT_SUPPORT_DIR) + "/urdf/lbr_iiwa_14_r820.srdf");
  tesseract->init(urdf_path, srdf_path, locator);

  // Get Kinematics Object
  auto kin = tesseract->getFwdKinematicsManager()->getFwdKinematicSolver("manipulator");

  // Start Joint Position for the program
  JointWaypoint wp0(Eigen::VectorXd::Zero(kin->numJoints()));

  // Define goal pose
//  CartesianWaypoint wp1(Eigen::Isometry3d::Identity() * Eigen::Translation3d(1, -1, 1));
  JointWaypoint wp1(tesseract_common::generateRandomNumber(kin->getLimits()));

  // Define Start State
  PlanInstruction plan_f0(wp0, PlanInstructionType::FREESPACE);
  plan_f0.addConstraint(FixedComponentInfo());

  // Define freespace move instruction
  PlanInstruction plan_f1(wp1, PlanInstructionType::FREESPACE);
  plan_f1.addConstraint(FixedComponentInfo());

  // Create a program
  CompositeInstruction program;
  program.push_back(plan_f0);
  program.push_back(plan_f1);
  program.addCost(VelocitySmoothingComponentInfo());
  program.addCost(AccelerationSmoothingComponentInfo());
  program.addCost(JerkSmoothingComponentInfo());
  program.addCost(AvoidCollisionComponentInfo());

  auto config = std::make_shared<tesseract_planning::TrajOptPlannerUniversalConfig>(tesseract, "manipulator", "tool0", Eigen::Isometry3d::Identity());
  config->instructions = program;
  config->seed = generateSeed(program, wp0);

  tesseract_motion_planners::TrajOptMotionPlanner planner;
  planner.setConfiguration(config);

  tesseract_motion_planners::PlannerResponse response;
  auto post_check_type = tesseract_motion_planners::PostPlanCheckType::DISCRETE_CONTINUOUS_COLLISION;
  auto status = planner.solve(response, post_check_type, true);

  if (status)
    config->processResults(response.joint_trajectory);

  std::cout << status.message() << std::endl;
}
