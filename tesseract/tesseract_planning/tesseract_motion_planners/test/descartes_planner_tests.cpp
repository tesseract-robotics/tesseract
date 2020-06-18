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

#include <tesseract_motion_planners/descartes/descartes_motion_planner.h>
#include <tesseract_motion_planners/descartes/descartes_motion_planner_default_config.h>
#include <tesseract_motion_planners/descartes/descartes_utils.h>
#include <tesseract_motion_planners/descartes/profile/descartes_default_plan_profile.h>

#include <tesseract_motion_planners/core/types.h>
#include <tesseract_motion_planners/core/utils.h>

const int NUM_STEPS = 200;

using namespace tesseract;
using namespace tesseract_environment;
using namespace tesseract_scene_graph;
using namespace tesseract_collision;
using namespace tesseract_planning;
using namespace tesseract_kinematics;
using namespace opw_kinematics;
using namespace descartes_light;
using namespace descartes_core;

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

//DescartesMotionPlannerConfigD
//createDescartesPlannerConfig(const tesseract::Tesseract::ConstPtr& tesseract_ptr,
//                             const std::string& /*manip*/,
//                             const tesseract_kinematics::InverseKinematics::ConstPtr& kin,
//                             const Eigen::Isometry3d& tcp,
//                             const double robot_reach,
//                             const tesseract_environment::EnvState::ConstPtr& current_state,
//                             const std::vector<tesseract_motion_planners::Waypoint::Ptr>& waypoints,
//                             bool use_collision_edge_evaluator = false)
//{
//  const std::vector<std::string>& joint_names = kin->getJointNames();
//  const std::vector<std::string>& active_link_names = kin->getActiveLinkNames();

//  tesseract_environment::AdjacencyMap::Ptr adjacency_map = std::make_shared<tesseract_environment::AdjacencyMap>(
//      tesseract_ptr->getEnvironmentConst()->getSceneGraph(), active_link_names, current_state->link_transforms);

//  // Create Collision Interface
//  typename descartes_light::CollisionInterface<double>::Ptr coll_interface =
//      std::make_shared<tesseract_motion_planners::DescartesCollision<double>>(
//          tesseract_ptr->getEnvironmentConst(), adjacency_map->getActiveLinkNames(), joint_names);

//  // Create Timing Constraint
//  std::vector<descartes_core::TimingConstraint<double>> timing =
//      makeTiming<double>(waypoints, std::numeric_limits<double>::max());

//  // Create Edge Evaluator
//  descartes_light::EdgeEvaluator<double>::Ptr edge_computer;
//  if (!use_collision_edge_evaluator)
//  {
//    edge_computer = std::make_shared<descartes_light::EuclideanDistanceEdgeEvaluator<double>>(kin->numJoints());
//  }
//  else
//  {
//    edge_computer = std::make_shared<tesseract_motion_planners::DescartesCollisionEdgeEvaluator<double>>(
//        tesseract_ptr->getEnvironmentConst(), adjacency_map->getActiveLinkNames(), joint_names);
//  }

//  // Create is valid function
//  tesseract_motion_planners::DescartesIsValidFn<double> is_valid_fn =
//      std::bind(&tesseract_kinematics::isWithinLimits<double>, std::placeholders::_1, kin->getLimits());

//  // Create Position Samplers
//  std::vector<typename descartes_light::PositionSampler<double>::Ptr> position_samplers =
//      makeRobotSamplers<double>(waypoints, kin, coll_interface, current_state, tcp, robot_reach, true, is_valid_fn);

//  return tesseract_motion_planners::DescartesMotionPlannerConfigD(tesseract_ptr,
//                                                                  adjacency_map->getActiveLinkNames(),
//                                                                  joint_names,
//                                                                  edge_computer,
//                                                                  timing,
//                                                                  position_samplers,
//                                                                  waypoints);
//}

class TesseractPlanningDescartesUnit : public ::testing::Test
{
protected:
  Tesseract::Ptr tesseract_ptr_;
  opw_kinematics::Parameters<double> opw_params_;

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

    auto robot_kin = tesseract_ptr_->getFwdKinematicsManagerConst()->getFwdKinematicSolver("manipulator");
    auto opw_kin = std::make_shared<OPWInvKin>();
    opw_kin->init("manipulator",
                  opw_params_,
                  robot_kin->getBaseLinkName(),
                  robot_kin->getTipLinkName(),
                  robot_kin->getJointNames(),
                  robot_kin->getLinkNames(),
                  robot_kin->getActiveLinkNames(),
                  robot_kin->getLimits());

    tesseract_ptr_->getInvKinematicsManager()->addInvKinematicSolver(opw_kin);
    tesseract_ptr_->getInvKinematicsManager()->setDefaultInvKinematicSolver("manipulator", opw_kin->getSolverName());
  }
};

TEST_F(TesseractPlanningDescartesUnit, DescartesPlannerFixedPoses)  // NOLINT
{
  // Create the planner and the responses that will store the results
  PlannerResponse planning_response;

  auto fwd_kin = tesseract_ptr_->getFwdKinematicsManagerConst()->getFwdKinematicSolver("manipulator");
  auto inv_kin = tesseract_ptr_->getInvKinematicsManagerConst()->getInvKinematicSolver("manipulator");
  const std::vector<std::string>& joint_names = fwd_kin->getJointNames();
  auto cur_state = tesseract_ptr_->getEnvironmentConst()->getCurrentState();

  // Specify a start waypoint
  CartesianWaypoint wp1 = Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.8, -.20, 0.8) * Eigen::Quaterniond(0, 0, -1.0, 0);

  // Specify a end waypoint
  CartesianWaypoint wp2 = Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.8, .20, 0.8) * Eigen::Quaterniond(0, 0, -1.0, 0);

  // Define Plan Instructions
  PlanInstruction plan_f0(wp1, PlanInstructionType::FREESPACE, "TEST_PROFILE");
  plan_f0.setWorkingFrame("base_link");
  PlanInstruction plan_f1(wp2, PlanInstructionType::LINEAR, "TEST_PROFILE");
  plan_f1.setWorkingFrame("base_link");

  // Create a program
  CompositeInstruction program;
  program.push_back(plan_f0);
  program.push_back(plan_f1);

  // Create a seed
  CompositeInstruction seed = generateSeed(program, cur_state, fwd_kin, inv_kin);

  // Create Profiles
  auto plan_profile = std::make_shared<DescartesDefaultPlanProfileD>();

  // Create TrajOpt Config
  auto config = std::make_shared<DescartesMotionPlannerDefaultConfigD>(tesseract_ptr_,
                                                                       tesseract_ptr_->getEnvironmentConst()->getCurrentState(),
                                                                       "manipulator",
                                                                       1.5);
  config->configuration = DescartesProblemD::Configuration::ROBOT_ONLY;
  config->instructions = program;
  config->seed = seed;
  config->plan_profiles["TEST_PROFILE"] = plan_profile;
  config->prob.num_threads = 1;

  // Create Planner
  DescartesMotionPlannerD single_descartes_planner;
  single_descartes_planner.setConfiguration(config);

  PlannerResponse single_planner_response;
  auto single_status = single_descartes_planner.solve(single_planner_response);
  EXPECT_TRUE(&single_status);

  CompositeInstruction official_results = config->seed;

  for (int i = 0; i < 10; ++i)
  {
    config->prob.num_threads = 4;
    DescartesMotionPlannerD descartes_planner;
    PlannerResponse planner_response;
    config->seed = seed; // reset seed to the original seed
    descartes_planner.setConfiguration(config);
    EXPECT_EQ(config->prob.samplers.size(), 11);
    EXPECT_EQ(config->prob.timing_constraints.size(), 11);
    EXPECT_EQ(config->prob.edge_evaluators.size(), 10);

    auto status = descartes_planner.solve(planner_response);
    EXPECT_TRUE(&status);
    EXPECT_TRUE(official_results.size() == config->seed.size());
    for (std::size_t j = 0; j < official_results.size(); ++j)
    {
      if (official_results[j].isComposite())
      {
        const auto* sub_official = official_results[j].cast_const<CompositeInstruction>();
        const auto* sub = config->seed[j].cast_const<CompositeInstruction>();
        for (std::size_t k = 0; k < sub->size(); ++k)
        {
          if ((*sub_official)[k].isComposite())
          {
            EXPECT_TRUE(false);
          }
          else if ((*sub_official)[k].isMove())
          {
            const auto* mv_official = (*sub_official)[k].cast_const<MoveInstruction>();
            const auto* mv = (*sub)[k].cast_const<MoveInstruction>();
            EXPECT_TRUE(mv_official->getPosition().isApprox(mv->getPosition(), 1e-5));
          }
        }
      }
      else if (official_results[j].isMove())
      {
        const auto* mv_official = official_results[j].cast_const<MoveInstruction>();
        const auto* mv = config->seed[j].cast_const<MoveInstruction>();
        EXPECT_TRUE(mv_official->getPosition().isApprox(mv->getPosition(), 1e-5));
      }
    }
  }
}

TEST_F(TesseractPlanningDescartesUnit, DescartesPlannerAxialSymetric)  // NOLINT
{
  // Create the planner and the responses that will store the results
  PlannerResponse planning_response;

  auto fwd_kin = tesseract_ptr_->getFwdKinematicsManagerConst()->getFwdKinematicSolver("manipulator");
  auto inv_kin = tesseract_ptr_->getInvKinematicsManagerConst()->getInvKinematicSolver("manipulator");
  const std::vector<std::string>& joint_names = fwd_kin->getJointNames();
  auto cur_state = tesseract_ptr_->getEnvironmentConst()->getCurrentState();

  // Specify a start waypoint
  CartesianWaypoint wp1 = Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.8, -.20, 0.8) * Eigen::Quaterniond(0, 0, -1.0, 0);

  // Specify a end waypoint
  CartesianWaypoint wp2 = Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.8, .20, 0.8) * Eigen::Quaterniond(0, 0, -1.0, 0);

  // Define Plan Instructions
  PlanInstruction plan_f0(wp1, PlanInstructionType::FREESPACE, "TEST_PROFILE");
  plan_f0.setWorkingFrame("base_link");
  PlanInstruction plan_f1(wp2, PlanInstructionType::LINEAR, "TEST_PROFILE");
  plan_f1.setWorkingFrame("base_link");

  // Create a program
  CompositeInstruction program;
  program.push_back(plan_f0);
  program.push_back(plan_f1);

  // Create a seed
  CompositeInstruction seed = generateSeed(program, cur_state, fwd_kin, inv_kin);

  // Create Profiles
  auto plan_profile = std::make_shared<DescartesDefaultPlanProfileD>();
  // Make this a tool z-axis free sampler
  plan_profile->target_pose_sampler = [](const Eigen::Isometry3d& tool_pose) { return tesseract_planning::sampleToolAxis(tool_pose, M_PI_4, Eigen::Vector3d(0, 0, 1)); };

  // Create TrajOpt Config
  auto config = std::make_shared<DescartesMotionPlannerDefaultConfigD>(tesseract_ptr_,
                                                                       tesseract_ptr_->getEnvironmentConst()->getCurrentState(),
                                                                       "manipulator",
                                                                       1.5);
  config->configuration = DescartesProblemD::Configuration::ROBOT_ONLY;
  config->instructions = program;
  config->seed = seed;
  config->plan_profiles["TEST_PROFILE"] = plan_profile;
  config->prob.num_threads = 1;

  // Create Planner
  DescartesMotionPlannerD single_descartes_planner;
  single_descartes_planner.setConfiguration(config);
  EXPECT_EQ(config->prob.samplers.size(), 11);
  EXPECT_EQ(config->prob.timing_constraints.size(), 11);
  EXPECT_EQ(config->prob.edge_evaluators.size(), 10);

  PlannerResponse single_planner_response;
  auto single_status = single_descartes_planner.solve(single_planner_response);
  EXPECT_TRUE(&single_status);

  CompositeInstruction official_results = config->seed;

  for (int i = 0; i < 10; ++i)
  {
    config->prob.num_threads = 4;
    DescartesMotionPlannerD descartes_planner;
    PlannerResponse planner_response;
    config->seed = seed; // reset seed to the original seed
    descartes_planner.setConfiguration(config);
    auto status = descartes_planner.solve(planner_response);
    EXPECT_TRUE(&status);
    EXPECT_TRUE(official_results.size() == config->seed.size());
    for (std::size_t j = 0; j < official_results.size(); ++j)
    {
      if (official_results[j].isComposite())
      {
        const auto* sub_official = official_results[j].cast_const<CompositeInstruction>();
        const auto* sub = config->seed[j].cast_const<CompositeInstruction>();
        for (std::size_t k = 0; k < sub->size(); ++k)
        {
          if ((*sub_official)[k].isComposite())
          {
            EXPECT_TRUE(false);
          }
          else if ((*sub_official)[k].isMove())
          {
            const auto* mv_official = (*sub_official)[k].cast_const<MoveInstruction>();
            const auto* mv = (*sub)[k].cast_const<MoveInstruction>();
            EXPECT_TRUE(mv_official->getPosition().isApprox(mv->getPosition(), 1e-5));
          }
        }
      }
      else if (official_results[j].isMove())
      {
        const auto* mv_official = official_results[j].cast_const<MoveInstruction>();
        const auto* mv = config->seed[j].cast_const<MoveInstruction>();
        EXPECT_TRUE(mv_official->getPosition().isApprox(mv->getPosition(), 1e-5));
      }
    }
  }
}

TEST_F(TesseractPlanningDescartesUnit, DescartesPlannerCollisionEdgeEvaluator)  // NOLINT
{
  // Create the planner and the responses that will store the results
  PlannerResponse planning_response;

  auto fwd_kin = tesseract_ptr_->getFwdKinematicsManagerConst()->getFwdKinematicSolver("manipulator");
  auto inv_kin = tesseract_ptr_->getInvKinematicsManagerConst()->getInvKinematicSolver("manipulator");
  const std::vector<std::string>& joint_names = fwd_kin->getJointNames();
  auto cur_state = tesseract_ptr_->getEnvironmentConst()->getCurrentState();

  // Specify a start waypoint
  CartesianWaypoint wp1 = Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.8, -.10, 0.8) * Eigen::Quaterniond(0, 0, -1.0, 0);

  // Specify a end waypoint
  CartesianWaypoint wp2 = Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.8, .10, 0.8) * Eigen::Quaterniond(0, 0, -1.0, 0);

  // Define Plan Instructions
  PlanInstruction plan_f0(wp1, PlanInstructionType::FREESPACE, "TEST_PROFILE");
  plan_f0.setWorkingFrame("base_link");
  PlanInstruction plan_f1(wp2, PlanInstructionType::LINEAR, "TEST_PROFILE");
  plan_f1.setWorkingFrame("base_link");

  // Create a program
  CompositeInstruction program;
  program.push_back(plan_f0);
  program.push_back(plan_f1);

  // Create a seed
  CompositeInstruction seed = generateSeed(program, cur_state, fwd_kin, inv_kin);

  // Create Profiles
  auto plan_profile = std::make_shared<DescartesDefaultPlanProfileD>();
  // Make this a tool z-axis free sampler
  plan_profile->target_pose_sampler = [](const Eigen::Isometry3d& tool_pose) { return tesseract_planning::sampleToolAxis(tool_pose, M_PI_4, Eigen::Vector3d(0, 0, 1)); };
  plan_profile->enable_edge_collision = true; // Add collision edge evaluator

  // Create TrajOpt Config
  auto config = std::make_shared<DescartesMotionPlannerDefaultConfigD>(tesseract_ptr_,
                                                                       tesseract_ptr_->getEnvironmentConst()->getCurrentState(),
                                                                       "manipulator",
                                                                       1.5);
  config->configuration = DescartesProblemD::Configuration::ROBOT_ONLY;
  config->instructions = program;
  config->seed = seed;
  config->plan_profiles["TEST_PROFILE"] = plan_profile;
  config->prob.num_threads = 1;

  // Create Planner
  DescartesMotionPlannerD single_descartes_planner;
  single_descartes_planner.setConfiguration(config);
  EXPECT_EQ(config->prob.samplers.size(), 11);
  EXPECT_EQ(config->prob.timing_constraints.size(), 11);
  EXPECT_EQ(config->prob.edge_evaluators.size(), 10);

  PlannerResponse single_planner_response;
  auto single_status = single_descartes_planner.solve(single_planner_response);
  EXPECT_TRUE(&single_status);

  CompositeInstruction official_results = config->seed;

  for (int i = 0; i < 10; ++i)
  {
    config->prob.num_threads = 4;
    DescartesMotionPlannerD descartes_planner;
    PlannerResponse planner_response;
    config->seed = seed; // reset seed to the original seed
    descartes_planner.setConfiguration(config);
    auto status = descartes_planner.solve(planner_response);
    EXPECT_TRUE(&status);
    EXPECT_TRUE(official_results.size() == config->seed.size());
    for (std::size_t j = 0; j < official_results.size(); ++j)
    {
      if (official_results[j].isComposite())
      {
        const auto* sub_official = official_results[j].cast_const<CompositeInstruction>();
        const auto* sub = config->seed[j].cast_const<CompositeInstruction>();
        for (std::size_t k = 0; k < sub->size(); ++k)
        {
          if ((*sub_official)[k].isComposite())
          {
            EXPECT_TRUE(false);
          }
          else if ((*sub_official)[k].isMove())
          {
            const auto* mv_official = (*sub_official)[k].cast_const<MoveInstruction>();
            const auto* mv = (*sub)[k].cast_const<MoveInstruction>();
            EXPECT_TRUE(mv_official->getPosition().isApprox(mv->getPosition(), 1e-5));
          }
        }
      }
      else if (official_results[j].isMove())
      {
        const auto* mv_official = official_results[j].cast_const<MoveInstruction>();
        const auto* mv = config->seed[j].cast_const<MoveInstruction>();
        EXPECT_TRUE(mv_official->getPosition().isApprox(mv->getPosition(), 1e-5));
      }
    }
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
