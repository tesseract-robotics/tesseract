/**
 * @file descartes_planner_tests.cpp
 * @brief This contains unit test for the tesseract descartes planner
 *
 * @author Levi Armstrong
 * @date September 16, 2019
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2019, Southwest Research Institute
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
#include <opw_kinematics/opw_parameters.h>
#include <tesseract_motion_planners/descartes/descartes_collision.h>
#include <descartes_samplers/evaluators/euclidean_distance_edge_evaluator.h>
#include <tesseract_kinematics/opw/opw_inv_kin.h>
#include <tesseract_kinematics/core/utils.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/command_language_utils.h>

#include <tesseract_motion_planners/descartes/descartes_motion_planner.h>
#include <tesseract_motion_planners/descartes/descartes_utils.h>
#include <tesseract_motion_planners/descartes/profile/descartes_default_plan_profile.h>
#include <tesseract_motion_planners/descartes/problem_generators/default_problem_generator.h>
#include <tesseract_motion_planners/core/types.h>
#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_motion_planners/interface_utils.h>

using namespace tesseract;
using namespace tesseract_environment;
using namespace tesseract_scene_graph;
using namespace tesseract_collision;
using namespace tesseract_planning;
using namespace tesseract_kinematics;
using namespace opw_kinematics;
using namespace descartes_light;
using namespace descartes_core;

const bool DEBUG = false;

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

class TesseractPlanningDescartesUnit : public ::testing::Test
{
protected:
  Tesseract::Ptr tesseract_ptr_;
  opw_kinematics::Parameters<double> opw_params_;
  ManipulatorInfo manip;

  void SetUp() override
  {
    tesseract_scene_graph::ResourceLocator::Ptr locator =
        std::make_shared<tesseract_scene_graph::SimpleResourceLocator>(locateResource);
    Tesseract::Ptr tesseract = std::make_shared<Tesseract>();
    boost::filesystem::path urdf_path(std::string(TESSERACT_SUPPORT_DIR) + "/urdf/abb_irb2400.urdf");
    boost::filesystem::path srdf_path(std::string(TESSERACT_SUPPORT_DIR) + "/urdf/abb_irb2400.srdf");
    EXPECT_TRUE(tesseract->init(urdf_path, srdf_path, locator));
    tesseract_ptr_ = tesseract;

    opw_params_.a1 = (0.100);
    opw_params_.a2 = (-0.135);
    opw_params_.b = (0.000);
    opw_params_.c1 = (0.615);
    opw_params_.c2 = (0.705);
    opw_params_.c3 = (0.755);
    opw_params_.c4 = (0.085);

    opw_params_.offsets[2] = -M_PI / 2.0;

    manip.manipulator = "manipulator";
    manip.manipulator_ik_solver = "OPWInvKin";

    auto robot_kin = tesseract_ptr_->getFwdKinematicsManagerConst()->getFwdKinematicSolver(manip.manipulator);
    auto opw_kin = std::make_shared<OPWInvKin>();
    opw_kin->init(manip.manipulator,
                  opw_params_,
                  robot_kin->getBaseLinkName(),
                  robot_kin->getTipLinkName(),
                  robot_kin->getJointNames(),
                  robot_kin->getLinkNames(),
                  robot_kin->getActiveLinkNames(),
                  robot_kin->getLimits());

    tesseract_ptr_->getInvKinematicsManager()->addInvKinematicSolver(opw_kin);
    tesseract_ptr_->getInvKinematicsManager()->setDefaultInvKinematicSolver(manip.manipulator,
                                                                            manip.manipulator_ik_solver);
  }
};

TEST_F(TesseractPlanningDescartesUnit, DescartesPlannerFixedPoses)  // NOLINT
{
  // Create the planner and the responses that will store the results
  PlannerResponse planning_response;

  auto fwd_kin = tesseract_ptr_->getFwdKinematicsManagerConst()->getFwdKinematicSolver("manipulator");
  auto inv_kin = tesseract_ptr_->getInvKinematicsManagerConst()->getInvKinematicSolver("manipulator");
  auto cur_state = tesseract_ptr_->getEnvironmentConst()->getCurrentState();

  // Specify a start waypoint
  CartesianWaypoint wp1 =
      Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.8, -.20, 0.8) * Eigen::Quaterniond(0, 0, -1.0, 0);

  // Specify a end waypoint
  CartesianWaypoint wp2 =
      Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.8, .20, 0.8) * Eigen::Quaterniond(0, 0, -1.0, 0);

  // Define Start Instruction
  MoveInstruction start_instruction(wp1, MoveInstructionType::START, "TEST_PROFILE", manip);
  start_instruction.getManipulatorInfo().working_frame = "base_link";

  // Define Plan Instructions
  PlanInstruction plan_f1(wp2, PlanInstructionType::LINEAR, "TEST_PROFILE", manip);
  plan_f1.getManipulatorInfo().working_frame = "base_link";

  // Create a program
  CompositeInstruction program;
  program.setStartInstruction(start_instruction);
  program.push_back(plan_f1);

  // Create a seed
  CompositeInstruction seed = generateSeed(program, cur_state, tesseract_ptr_);

  // Create Profiles
  auto plan_profile = std::make_shared<DescartesDefaultPlanProfileD>();

  // Create Planner
  DescartesMotionPlannerD single_descartes_planner;
  plan_profile->num_threads = 1;
  single_descartes_planner.plan_profiles["TEST_PROFILE"] = plan_profile;
  single_descartes_planner.problem_generator = &DefaultDescartesProblemGenerator<double>;

  // Create Planning Request
  PlannerRequest request;
  request.seed = seed;
  request.instructions = program;
  request.tesseract = tesseract_ptr_;
  request.env_state = tesseract_ptr_->getEnvironment()->getCurrentState();

  PlannerResponse single_planner_response;
  auto single_status = single_descartes_planner.solve(request, single_planner_response);
  EXPECT_TRUE(&single_status);

  CompositeInstruction official_results = single_planner_response.results;

  for (int i = 0; i < 10; ++i)
  {
    // Test the problem generator
    {
      auto problem = DefaultDescartesProblemGenerator<double>(request, single_descartes_planner.plan_profiles);
      EXPECT_EQ(problem->samplers.size(), 11);
      EXPECT_EQ(problem->timing_constraints.size(), 11);
      EXPECT_EQ(problem->edge_evaluators.size(), 10);
    }

    DescartesMotionPlannerD descartes_planner;
    plan_profile->num_threads = 4;
    descartes_planner.plan_profiles["TEST_PROFILE"] = plan_profile;
    descartes_planner.problem_generator = &DefaultDescartesProblemGenerator<double>;

    PlannerResponse planner_response;
    auto status = descartes_planner.solve(request, planner_response);

    if (DEBUG)
    {
      std::cout << "Request Instructions:" << std::endl;
      request.instructions.print();
      std::cout << "Request Seed:" << std::endl;
      request.seed.print();
      std::cout << "Single Planner Response Results:" << std::endl;
      single_planner_response.results.print();
      std::cout << "Threaded Planner Response Results:" << std::endl;
      planner_response.results.print();
    }

    EXPECT_TRUE(&status);
    EXPECT_EQ(official_results.size(), planner_response.results.size());
    for (std::size_t j = 0; j < official_results.size(); ++j)
    {
      if (isCompositeInstruction(official_results[j]))
      {
        const auto* sub_official = official_results[j].cast_const<CompositeInstruction>();
        const auto* sub = planner_response.results[j].cast_const<CompositeInstruction>();
        for (std::size_t k = 0; k < sub->size(); ++k)
        {
          if (isCompositeInstruction((*sub_official)[k]))
          {
            EXPECT_TRUE(false);
          }
          else if (isMoveInstruction((*sub_official)[k]))
          {
            const auto* mv_official = (*sub_official)[k].cast_const<MoveInstruction>();
            const auto* mv = (*sub)[k].cast_const<MoveInstruction>();
            EXPECT_TRUE(
                getJointPosition(mv_official->getWaypoint()).isApprox(getJointPosition(mv->getWaypoint()), 1e-5));
          }
        }
      }
      else if (isMoveInstruction(official_results[j]))
      {
        const auto* mv_official = official_results[j].cast_const<MoveInstruction>();
        const auto* mv = request.seed[j].cast_const<MoveInstruction>();
        EXPECT_TRUE(getJointPosition(mv_official->getWaypoint()).isApprox(getJointPosition(mv->getWaypoint()), 1e-5));
      }
    }
  }
}

TEST_F(TesseractPlanningDescartesUnit, DescartesPlannerAxialSymetric)  // NOLINT
{
  // Create the planner and the responses that will store the results
  PlannerResponse planning_response;

  auto fwd_kin = tesseract_ptr_->getFwdKinematicsManagerConst()->getFwdKinematicSolver(manip.manipulator);
  auto inv_kin = tesseract_ptr_->getInvKinematicsManagerConst()->getInvKinematicSolver(manip.manipulator);
  auto cur_state = tesseract_ptr_->getEnvironmentConst()->getCurrentState();

  // Specify a start waypoint
  CartesianWaypoint wp1 =
      Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.8, -.20, 0.8) * Eigen::Quaterniond(0, 0, -1.0, 0);

  // Specify a end waypoint
  CartesianWaypoint wp2 =
      Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.8, .20, 0.8) * Eigen::Quaterniond(0, 0, -1.0, 0);

  // Define Start Instruction
  MoveInstruction start_instruction(wp1, MoveInstructionType::START, "TEST_PROFILE", manip);
  start_instruction.getManipulatorInfo().working_frame = "base_link";

  // Define Plan Instructions
  PlanInstruction plan_f1(wp2, PlanInstructionType::LINEAR, "TEST_PROFILE", manip);
  plan_f1.getManipulatorInfo().working_frame = "base_link";

  // Create a program
  CompositeInstruction program;
  program.setStartInstruction(start_instruction);
  program.push_back(plan_f1);

  // Create a seed
  CompositeInstruction seed = generateSeed(program, cur_state, tesseract_ptr_);

  // Create Profiles
  auto plan_profile = std::make_shared<DescartesDefaultPlanProfileD>();
  // Make this a tool z-axis free sampler
  plan_profile->target_pose_sampler = [](const Eigen::Isometry3d& tool_pose) {
    return tesseract_planning::sampleToolAxis(tool_pose, M_PI_4, Eigen::Vector3d(0, 0, 1));
  };

  // Create Planner
  DescartesMotionPlannerD single_descartes_planner;
  plan_profile->num_threads = 1;
  single_descartes_planner.plan_profiles["TEST_PROFILE"] = plan_profile;
  single_descartes_planner.problem_generator = &DefaultDescartesProblemGenerator<double>;

  // Create Planning Request
  PlannerRequest request;
  request.seed = seed;
  request.instructions = program;
  request.tesseract = tesseract_ptr_;
  request.env_state = tesseract_ptr_->getEnvironment()->getCurrentState();

  auto problem = DefaultDescartesProblemGenerator<double>(request, single_descartes_planner.plan_profiles);
  problem->num_threads = 1;
  EXPECT_EQ(problem->samplers.size(), 11);
  EXPECT_EQ(problem->timing_constraints.size(), 11);
  EXPECT_EQ(problem->edge_evaluators.size(), 10);

  PlannerResponse single_planner_response;
  auto single_status = single_descartes_planner.solve(request, single_planner_response);
  EXPECT_TRUE(&single_status);

  CompositeInstruction official_results = request.seed;

  for (int i = 0; i < 10; ++i)
  {
    DescartesMotionPlannerD descartes_planner;
    plan_profile->num_threads = 4;
    descartes_planner.plan_profiles["TEST_PROFILE"] = plan_profile;
    descartes_planner.problem_generator = &DefaultDescartesProblemGenerator<double>;

    PlannerResponse planner_response;
    request.seed = seed;  // reset seed to the original seed
    auto status = descartes_planner.solve(request, planner_response);
    EXPECT_TRUE(&status);
    EXPECT_TRUE(official_results.size() == request.seed.size());
    for (std::size_t j = 0; j < official_results.size(); ++j)
    {
      if (isCompositeInstruction(official_results[j]))
      {
        const auto* sub_official = official_results[j].cast_const<CompositeInstruction>();
        const auto* sub = request.seed[j].cast_const<CompositeInstruction>();
        for (std::size_t k = 0; k < sub->size(); ++k)
        {
          if (isCompositeInstruction((*sub_official)[k]))
          {
            EXPECT_TRUE(false);
          }
          else if (isMoveInstruction((*sub_official)[k]))
          {
            const auto* mv_official = (*sub_official)[k].cast_const<MoveInstruction>();
            const auto* mv = (*sub)[k].cast_const<MoveInstruction>();
            EXPECT_TRUE(
                getJointPosition(mv_official->getWaypoint()).isApprox(getJointPosition(mv->getWaypoint()), 1e-5));
          }
        }
      }
      else if (isMoveInstruction(official_results[j]))
      {
        const auto* mv_official = official_results[j].cast_const<MoveInstruction>();
        const auto* mv = request.seed[j].cast_const<MoveInstruction>();
        EXPECT_TRUE(getJointPosition(mv_official->getWaypoint()).isApprox(getJointPosition(mv->getWaypoint()), 1e-5));
      }
    }
  }
}

TEST_F(TesseractPlanningDescartesUnit, DescartesPlannerCollisionEdgeEvaluator)  // NOLINT
{
  // Create the planner and the responses that will store the results
  PlannerResponse planning_response;

  auto fwd_kin = tesseract_ptr_->getFwdKinematicsManagerConst()->getFwdKinematicSolver(manip.manipulator);
  auto inv_kin = tesseract_ptr_->getInvKinematicsManagerConst()->getInvKinematicSolver(manip.manipulator);
  auto cur_state = tesseract_ptr_->getEnvironmentConst()->getCurrentState();

  // Specify a start waypoint
  CartesianWaypoint wp1 =
      Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.8, -.10, 0.8) * Eigen::Quaterniond(0, 0, -1.0, 0);

  // Specify a end waypoint
  CartesianWaypoint wp2 =
      Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.8, .10, 0.8) * Eigen::Quaterniond(0, 0, -1.0, 0);

  // Define Start Instruction
  MoveInstruction start_instruction(wp1, MoveInstructionType::START, "TEST_PROFILE", manip);
  start_instruction.getManipulatorInfo().working_frame = "base_link";

  // Define Plan Instructions
  PlanInstruction plan_f1(wp2, PlanInstructionType::LINEAR, "TEST_PROFILE", manip);
  plan_f1.getManipulatorInfo().working_frame = "base_link";

  // Create a program
  CompositeInstruction program;
  program.setStartInstruction(start_instruction);
  program.push_back(plan_f1);

  // Create a seed
  CompositeInstruction seed = generateSeed(program, cur_state, tesseract_ptr_);

  // Create Profiles
  auto plan_profile = std::make_shared<DescartesDefaultPlanProfileD>();
  // Make this a tool z-axis free sampler
  plan_profile->target_pose_sampler = [](const Eigen::Isometry3d& tool_pose) {
    return tesseract_planning::sampleToolAxis(tool_pose, 60 * M_PI * 180.0, Eigen::Vector3d(0, 0, 1));
  };
  plan_profile->enable_edge_collision = true;  // Add collision edge evaluator

  // Create Planning Request
  PlannerRequest request;
  request.seed = seed;
  request.instructions = program;
  request.tesseract = tesseract_ptr_;
  request.env_state = tesseract_ptr_->getEnvironment()->getCurrentState();

  // Create Planner
  DescartesMotionPlannerD single_descartes_planner;
  plan_profile->num_threads = 1;
  single_descartes_planner.plan_profiles["TEST_PROFILE"] = plan_profile;

  // Test Problem size - TODO: Make dedicated unit test for DefaultDescartesProblemGenerator
  auto problem = DefaultDescartesProblemGenerator<double>(request, single_descartes_planner.plan_profiles);
  EXPECT_EQ(problem->samplers.size(), 3);
  EXPECT_EQ(problem->timing_constraints.size(), 3);
  EXPECT_EQ(problem->edge_evaluators.size(), 2);
  EXPECT_EQ(problem->num_threads, 1);

  PlannerResponse single_planner_response;
  auto single_status = single_descartes_planner.solve(request, single_planner_response);
  EXPECT_TRUE(&single_status);

  CompositeInstruction official_results = request.seed;

  for (int i = 0; i < 10; ++i)
  {
    DescartesMotionPlannerD descartes_planner;
    plan_profile->num_threads = 4;
    descartes_planner.plan_profiles["TEST_PROFILE"] = plan_profile;
    descartes_planner.problem_generator = &DefaultDescartesProblemGenerator<double>;

    PlannerResponse planner_response;
    request.seed = seed;  // reset seed to the original seed
    auto status = descartes_planner.solve(request, planner_response);
    EXPECT_TRUE(&status);
    EXPECT_TRUE(official_results.size() == request.seed.size());
    for (std::size_t j = 0; j < official_results.size(); ++j)
    {
      if (isCompositeInstruction(official_results[j]))
      {
        const auto* sub_official = official_results[j].cast_const<CompositeInstruction>();
        const auto* sub = request.seed[j].cast_const<CompositeInstruction>();
        for (std::size_t k = 0; k < sub->size(); ++k)
        {
          if (isCompositeInstruction((*sub_official)[k]))
          {
            EXPECT_TRUE(false);
          }
          else if (isMoveInstruction((*sub_official)[k]))
          {
            const auto* mv_official = (*sub_official)[k].cast_const<MoveInstruction>();
            const auto* mv = (*sub)[k].cast_const<MoveInstruction>();
            EXPECT_TRUE(
                getJointPosition(mv_official->getWaypoint()).isApprox(getJointPosition(mv->getWaypoint()), 1e-5));
          }
        }
      }
      else if (isMoveInstruction(official_results[j]))
      {
        const auto* mv_official = official_results[j].cast_const<MoveInstruction>();
        const auto* mv = request.seed[j].cast_const<MoveInstruction>();
        EXPECT_TRUE(getJointPosition(mv_official->getWaypoint()).isApprox(getJointPosition(mv->getWaypoint()), 1e-5));
      }
    }
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
