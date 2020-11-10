/**
 * @file chain_example.cpp
 * @brief Chained motion planning example
 *
 * @author Levi Armstrong
 * @date August 31, 2020
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
#include <console_bridge/console.h>
#include <descartes_samplers/evaluators/euclidean_distance_edge_evaluator.h>
#include <class_loader/class_loader.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_kinematics/core/utils.h>

#include <tesseract_motion_planners/descartes/descartes_collision.h>
#include <tesseract_motion_planners/descartes/descartes_motion_planner.h>
#include <tesseract_motion_planners/descartes/descartes_utils.h>
#include <tesseract_motion_planners/descartes/problem_generators/default_problem_generator.h>
#include <tesseract_motion_planners/descartes/profile/descartes_default_plan_profile.h>

#include <tesseract_motion_planners/trajopt/trajopt_motion_planner.h>
#include <tesseract_motion_planners/trajopt/problem_generators/default_problem_generator.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_plan_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_composite_profile.h>

#include <tesseract_motion_planners/core/types.h>
#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_motion_planners/interface_utils.h>

#include <tesseract_visualization/visualization_loader.h>

using namespace tesseract_planning;

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

int main(int /*argc*/, char** /*argv*/)
{
  // Setup
  tesseract_scene_graph::ResourceLocator::Ptr locator =
      std::make_shared<tesseract_scene_graph::SimpleResourceLocator>(locateResource);
  auto tesseract = std::make_shared<tesseract::Tesseract>();
  boost::filesystem::path urdf_path(std::string(TESSERACT_SUPPORT_DIR) + "/urdf/abb_irb2400.urdf");
  boost::filesystem::path srdf_path(std::string(TESSERACT_SUPPORT_DIR) + "/urdf/abb_irb2400.srdf");
  tesseract->init(urdf_path, srdf_path, locator);

  // Dynamically load ignition visualizer if exist
  tesseract_visualization::VisualizationLoader loader;
  auto plotter = loader.get();

  if (plotter != nullptr)
  {
    plotter->init(tesseract);
    plotter->waitForConnection();
    plotter->plotEnvironment();
  }

  ManipulatorInfo manip;
  manip.manipulator = "manipulator";
  manip.manipulator_ik_solver = "OPWInvKin";

  auto fwd_kin = tesseract->getEnvironment()->getManipulatorManager()->getFwdKinematicSolver(manip.manipulator);
  auto inv_kin = tesseract->getEnvironment()->getManipulatorManager()->getInvKinematicSolver(manip.manipulator);
  auto cur_state = tesseract->getEnvironment()->getCurrentState();

  // Specify start location
  JointWaypoint wp0(fwd_kin->getJointNames(), Eigen::VectorXd::Zero(6));

  // Specify raster 1 start waypoint and end waypoint
  CartesianWaypoint wp1 =
      Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.8, -.20, 0.8) * Eigen::Quaterniond(0, 0, -1.0, 0);
  CartesianWaypoint wp2 =
      Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.8, .20, 0.8) * Eigen::Quaterniond(0, 0, -1.0, 0);

  // Specify raster 2 start waypoint and end waypoint
  CartesianWaypoint wp3 =
      Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.9, -.20, 0.8) * Eigen::Quaterniond(0, 0, -1.0, 0);
  CartesianWaypoint wp4 =
      Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.9, .20, 0.8) * Eigen::Quaterniond(0, 0, -1.0, 0);

  // Specify raster 4 start waypoint and end waypoint
  CartesianWaypoint wp5 =
      Eigen::Isometry3d::Identity() * Eigen::Translation3d(1.0, -.20, 0.8) * Eigen::Quaterniond(0, 0, -1.0, 0);
  CartesianWaypoint wp6 =
      Eigen::Isometry3d::Identity() * Eigen::Translation3d(1.0, .20, 0.8) * Eigen::Quaterniond(0, 0, -1.0, 0);

  // Define Plan Instructions
  PlanInstruction start_instruction(wp0, PlanInstructionType::START);
  PlanInstruction plan_f1(wp1, PlanInstructionType::FREESPACE, "DEFAULT");
  PlanInstruction plan_c1(wp2, PlanInstructionType::LINEAR, "DEFAULT");
  PlanInstruction plan_c2(wp3, PlanInstructionType::LINEAR, "DEFAULT");
  PlanInstruction plan_c3(wp4, PlanInstructionType::LINEAR, "DEFAULT");
  PlanInstruction plan_c4(wp5, PlanInstructionType::LINEAR, "DEFAULT");
  PlanInstruction plan_c5(wp6, PlanInstructionType::LINEAR, "DEFAULT");
  PlanInstruction plan_f3(wp0, PlanInstructionType::FREESPACE, "DEFAULT");

  // Create program
  CompositeInstruction program;
  program.setStartInstruction(start_instruction);
  program.setManipulatorInfo(manip);
  program.push_back(plan_f1);
  program.push_back(plan_c1);
  program.push_back(plan_c2);
  program.push_back(plan_c3);
  program.push_back(plan_c4);
  program.push_back(plan_c5);
  program.push_back(plan_f3);

  // Plot Program
  if (plotter)
  {
  }

  // Create Profiles
  auto descartes_plan_profile = std::make_shared<DescartesDefaultPlanProfileD>();
  auto trajopt_plan_profile = std::make_shared<TrajOptDefaultPlanProfile>();
  auto trajopt_composite_profile = std::make_shared<TrajOptDefaultCompositeProfile>();

  // Create a seed
  CompositeInstruction seed = generateSeed(program, cur_state, tesseract);

  // Create Planning Request
  PlannerRequest request;
  request.seed = seed;
  request.instructions = program;
  request.tesseract = tesseract;
  request.env_state = cur_state;

  // Solve Descartes Plan
  PlannerResponse descartes_response;
  DescartesMotionPlannerD descartes_planner;
  descartes_planner.plan_profiles["DEFAULT"] = descartes_plan_profile;
  descartes_planner.problem_generator = tesseract_planning::DefaultDescartesProblemGenerator<double>;
  auto descartes_status = descartes_planner.solve(request, descartes_response);
  assert(descartes_status);

  // Plot Descartes Trajectory
  if (plotter)
  {
    plotter->waitForInput();
    plotter->plotTrajectory(descartes_response.results);
  }

  // Update Seed
  request.seed = descartes_response.results;

  // Solve TrajOpt Plan
  PlannerResponse trajopt_response;
  TrajOptMotionPlanner trajopt_planner;
  trajopt_planner.problem_generator = tesseract_planning::DefaultTrajoptProblemGenerator;
  trajopt_planner.plan_profiles["DEFAULT"] = trajopt_plan_profile;
  trajopt_planner.composite_profiles["DEFAULT"] = trajopt_composite_profile;
  auto trajopt_status = trajopt_planner.solve(request, trajopt_response);
  assert(trajopt_status);

  if (plotter)
  {
    plotter->waitForInput();
    plotter->plotTrajectory(trajopt_response.results);
  }
}
