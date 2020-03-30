/**
 * @file planning_dispatch_queue_tests.cpp
 * @brief This contains unit test planning dispatch queue library
 *
 * @author Levi Armstrong
 * @date February 26, 2020
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
#include <functional>
#include <gtest/gtest.h>
#include <taskflow/taskflow.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/tesseract.h>
#include <tesseract_environment/core/utils.h>
#include <tesseract_motion_planners/ompl/ompl_motion_planner.h>
#include <tesseract_motion_planners/ompl/ompl_planner_configurator.h>
#include <tesseract_motion_planners/ompl/config/ompl_planner_freespace_config.h>
#include <tesseract_motion_planners/trajopt/config/trajopt_planner_freespace_config.h>
#include <tesseract_motion_planners/trajopt/trajopt_motion_planner.h>
#include <tesseract_motion_planners/planning_dispatch_queue.h>

using namespace tesseract;
using namespace tesseract_scene_graph;
using namespace tesseract_collision;
using namespace tesseract_environment;
using namespace tesseract_geometry;
using namespace tesseract_kinematics;
using namespace tesseract_motion_planners;

const static std::vector<double> start_state = { -0.5, 0.5, 0.0, -1.3348, 0.0, 1.4959, 0.0 };
const static std::vector<double> end_state = { 0.5, 0.5, 0.0, -1.3348, 0.0, 1.4959, 0.0 };

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

static void addBox(tesseract_environment::Environment& env)
{
  Link link_1("box_attached");

  Visual::Ptr visual = std::make_shared<Visual>();
  visual->origin = Eigen::Isometry3d::Identity();
  visual->origin.translation() = Eigen::Vector3d(0.5, 0, 0.55);
  visual->geometry = std::make_shared<tesseract_geometry::Box>(0.4, 0.001, 0.4);
  link_1.visual.push_back(visual);

  Collision::Ptr collision = std::make_shared<Collision>();
  collision->origin = visual->origin;
  collision->geometry = visual->geometry;
  link_1.collision.push_back(collision);

  Joint joint_1("joint_n1");
  joint_1.parent_link_name = "base_link";
  joint_1.child_link_name = link_1.getName();
  joint_1.type = JointType::FIXED;

  env.addLink(std::move(link_1), std::move(joint_1));
}

Tesseract::Ptr getTesseract()
{
  tesseract_scene_graph::ResourceLocator::Ptr locator =
      std::make_shared<tesseract_scene_graph::SimpleResourceLocator>(locateResource);
  Tesseract::Ptr tesseract = std::make_shared<Tesseract>();
  boost::filesystem::path urdf_path(std::string(TESSERACT_SUPPORT_DIR) + "/urdf/lbr_iiwa_14_r820.urdf");
  boost::filesystem::path srdf_path(std::string(TESSERACT_SUPPORT_DIR) + "/urdf/lbr_iiwa_14_r820.srdf");
  EXPECT_TRUE(tesseract->init(urdf_path, srdf_path, locator));

  // Add box to environment
  addBox(*(tesseract->getEnvironment()));
  return tesseract;
}

TEST(PlanningDispatchQueue, PlanningDispatchQueueUnit)  // NOLINT
{
  // Step 1: Load scene and srdf
  Tesseract::Ptr tesseract = getTesseract();

  // Step 2: Create ompl planner config and populate it
  auto kin = tesseract->getFwdKinematicsManagerConst()->getFwdKinematicSolver("manipulator");
  std::vector<double> swp = start_state;
  std::vector<double> ewp = end_state;

  tesseract_motion_planners::PlannerResponse ompl_response;
  auto ompl_fn = [&tesseract, &kin, &ewp, &swp, &ompl_response]() {
    tesseract_motion_planners::OMPLMotionPlanner ompl_planner;
    std::vector<OMPLPlannerConfigurator::ConstPtr> planners = { std::make_shared<SBLConfigurator>(),
                                                                std::make_shared<RRTConnectConfigurator>() };
    auto ompl_config = std::make_shared<OMPLPlannerFreespaceConfig>(tesseract, "manipulator", planners);

    ompl_config->start_waypoint = std::make_shared<tesseract_motion_planners::JointWaypoint>(swp, kin->getJointNames());
    ompl_config->end_waypoint = std::make_shared<tesseract_motion_planners::JointWaypoint>(ewp, kin->getJointNames());
    ompl_config->collision_safety_margin = 0.02;
    ompl_config->planning_time = 5.0;
    ompl_config->max_solutions = 2;
    ompl_config->longest_valid_segment_fraction = 0.01;

    ompl_config->collision_continuous = true;
    ompl_config->collision_check = true;
    ompl_config->simplify = false;
    ompl_config->n_output_states = 50;

    ompl_planner.setConfiguration(ompl_config);
    ompl_planner.solve(ompl_response, false);
  };

  tesseract_motion_planners::PlannerResponse response;
  auto trajopt_fn = [&tesseract, &kin, &ewp, &swp, &response]() {
    TrajOptMotionPlanner trajopt_planner;
    auto trajopt_config = std::make_shared<TrajOptPlannerFreespaceConfig>(
        tesseract, "manipulator", "tool0", Eigen::Isometry3d::Identity());
    trajopt_config->target_waypoints.push_back(
        std::make_shared<tesseract_motion_planners::JointWaypoint>(swp, kin->getJointNames()));
    trajopt_config->target_waypoints.push_back(
        std::make_shared<tesseract_motion_planners::JointWaypoint>(ewp, kin->getJointNames()));
    trajopt_config->longest_valid_segment_fraction = 0.01;
    trajopt_config->collision_cost_config.enabled = false;
    trajopt_config->collision_constraint_config.enabled = true;
    trajopt_config->collision_constraint_config.safety_margin = 0.02;
    trajopt_config->collision_constraint_config.type = trajopt::CollisionEvaluatorType::CAST_CONTINUOUS;
    trajopt_config->smooth_velocities = true;
    trajopt_config->smooth_jerks = true;
    trajopt_config->smooth_accelerations = true;
    trajopt_config->num_steps = 50;

    // Set the ompl planner configuration
    trajopt_planner.setConfiguration(trajopt_config);
    trajopt_planner.solve(response, false);
  };

  tesseract_motion_planners::PlannerResponse response2;
  auto trajopt_fn2 = [&tesseract, &kin, &ewp, &swp, &response2]() {
    TrajOptMotionPlanner trajopt_planner;
    auto trajopt_config = std::make_shared<TrajOptPlannerFreespaceConfig>(
        tesseract, "manipulator", "tool0", Eigen::Isometry3d::Identity());
    trajopt_config->target_waypoints.push_back(
        std::make_shared<tesseract_motion_planners::JointWaypoint>(swp, kin->getJointNames()));
    trajopt_config->target_waypoints.push_back(
        std::make_shared<tesseract_motion_planners::JointWaypoint>(ewp, kin->getJointNames()));
    trajopt_config->longest_valid_segment_fraction = 0.01;
    trajopt_config->collision_cost_config.enabled = false;
    trajopt_config->collision_constraint_config.enabled = true;
    trajopt_config->collision_constraint_config.safety_margin = 0.02;
    trajopt_config->collision_constraint_config.type = trajopt::CollisionEvaluatorType::CAST_CONTINUOUS;
    trajopt_config->smooth_velocities = true;
    trajopt_config->smooth_jerks = true;
    trajopt_config->smooth_accelerations = true;
    trajopt_config->num_steps = 50;

    // Set the ompl planner configuration
    trajopt_planner.setConfiguration(trajopt_config);
    trajopt_planner.solve(response2, false);
  };

  // create an executor and a taskflow
  tf::Executor executor;
  tf::Taskflow taskflow("Planning Test");

  auto A = taskflow.emplace(trajopt_fn).name("A");
  auto B = taskflow.emplace(ompl_fn).name("B");
  auto C = taskflow.emplace(trajopt_fn2).name("C");
  UNUSED(A);
  UNUSED(B);
  UNUSED(C);

  executor.run(taskflow).wait();

  EXPECT_TRUE(response.status);
  EXPECT_TRUE(response2.status);
  EXPECT_TRUE(ompl_response.status);

  //  auto D = taskflow.emplace([&](){ std::cout << "TaskD\n"; }).name("D");

  //  A.precede(B, C);
  //  B.precede(D);
  //  C.precede(D);

  //  auto opml_id = planning_dispatch.dispatchPlanningRequest(ompl_fn);
  //  auto trajopt_id = planning_dispatch.dispatchPlanningRequest(trajopt_fn);
  //  auto trajopt_id2 = planning_dispatch.dispatchPlanningRequest(trajopt_fn2);

  //  UNUSED(opml_id);
  //  UNUSED(trajopt_id);
  //  UNUSED(trajopt_id2);

  //  while (!planning_dispatch.isIdle())
  //  {
  //    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  //  }

  //  EXPECT_TRUE(planning_dispatch.hasResults());

  //  PlanningDispatchQueue::TaskResult first;
  //  EXPECT_TRUE(planning_dispatch.fetchResult(first));

  //  PlanningDispatchQueue::TaskResult second;
  //  EXPECT_TRUE(planning_dispatch.fetchResult(second));

  //  EXPECT_FALSE(planning_dispatch.hasResults());

  //  EXPECT_TRUE(first.second.status);
  //  EXPECT_TRUE(second.second.status);
}

// TEST(PlanningDispatchQueue, PlanningDispatchQueueUnit)  // NOLINT
//{
//  // Step 1: Load scene and srdf
//  Tesseract::Ptr tesseract = getTesseract();

//  // Step 2: Create ompl planner config and populate it
//  auto kin = tesseract->getFwdKinematicsManagerConst()->getFwdKinematicSolver("manipulator");
//  std::vector<double> swp = start_state;
//  std::vector<double> ewp = end_state;

//  // Step 3: Create planning dispatch queue.
//  PlanningDispatchQueue planning_dispatch("unit_test", 4);

//  // Step 4: Create and setup ompl planner
//  tesseract_motion_planners::OMPLMotionPlanner ompl_planner;
//  std::vector<OMPLPlannerConfigurator::ConstPtr> planners = { std::make_shared<SBLConfigurator>(),
//                                                              std::make_shared<RRTConnectConfigurator>() };
//  auto ompl_config = std::make_shared<OMPLPlannerFreespaceConfig>(tesseract, "manipulator", planners);

//  ompl_config->start_waypoint = std::make_shared<tesseract_motion_planners::JointWaypoint>(swp, kin->getJointNames());
//  ompl_config->end_waypoint = std::make_shared<tesseract_motion_planners::JointWaypoint>(ewp, kin->getJointNames());
//  ompl_config->collision_safety_margin = 0.02;
//  ompl_config->planning_time = 5.0;
//  ompl_config->max_solutions = 2;
//  ompl_config->longest_valid_segment_fraction = 0.01;

//  ompl_config->collision_continuous = true;
//  ompl_config->collision_check = true;
//  ompl_config->simplify = false;
//  ompl_config->n_output_states = 50;

//  // Set the ompl planner configuration
//  ompl_planner.setConfiguration(ompl_config);

//  auto ompl_fn = [&ompl_planner](tesseract_motion_planners::PlannerResponse& response){
//    ompl_planner.solve(response, false);
//  };
//  UNUSED(ompl_fn);

//  auto trajopt_fn = [&tesseract, &kin, &ewp, &swp](tesseract_motion_planners::PlannerResponse& response){
//    TrajOptMotionPlanner trajopt_planner;
//    auto trajopt_config =
//        std::make_shared<TrajOptPlannerFreespaceConfig>(tesseract, "manipulator", "tool0",
//        Eigen::Isometry3d::Identity());
//    trajopt_config->target_waypoints.push_back(std::make_shared<tesseract_motion_planners::JointWaypoint>(swp,
//    kin->getJointNames()));
//    trajopt_config->target_waypoints.push_back(std::make_shared<tesseract_motion_planners::JointWaypoint>(ewp,
//    kin->getJointNames())); trajopt_config->longest_valid_segment_fraction = 0.01;
//    trajopt_config->collision_cost_config.enabled = false;
//    trajopt_config->collision_constraint_config.enabled = true;
//    trajopt_config->collision_constraint_config.safety_margin = 0.02;
//    trajopt_config->collision_constraint_config.type = trajopt::CollisionEvaluatorType::CAST_CONTINUOUS;
//    trajopt_config->smooth_velocities = true;
//    trajopt_config->smooth_jerks = true;
//    trajopt_config->smooth_accelerations = true;
//    trajopt_config->num_steps = 50;

//    // Set the ompl planner configuration
//    trajopt_planner.setConfiguration(trajopt_config);
//    trajopt_planner.solve(response, false);
//  };

//  auto trajopt_fn2 = [&tesseract, &kin, &ewp, &swp](tesseract_motion_planners::PlannerResponse& response){
//    TrajOptMotionPlanner trajopt_planner;
//    auto trajopt_config =
//        std::make_shared<TrajOptPlannerFreespaceConfig>(tesseract, "manipulator", "tool0",
//        Eigen::Isometry3d::Identity());
//    trajopt_config->target_waypoints.push_back(std::make_shared<tesseract_motion_planners::JointWaypoint>(swp,
//    kin->getJointNames()));
//    trajopt_config->target_waypoints.push_back(std::make_shared<tesseract_motion_planners::JointWaypoint>(ewp,
//    kin->getJointNames())); trajopt_config->longest_valid_segment_fraction = 0.01;
//    trajopt_config->collision_cost_config.enabled = false;
//    trajopt_config->collision_constraint_config.enabled = true;
//    trajopt_config->collision_constraint_config.safety_margin = 0.02;
//    trajopt_config->collision_constraint_config.type = trajopt::CollisionEvaluatorType::CAST_CONTINUOUS;
//    trajopt_config->smooth_velocities = true;
//    trajopt_config->smooth_jerks = true;
//    trajopt_config->smooth_accelerations = true;
//    trajopt_config->num_steps = 50;

//    // Set the ompl planner configuration
//    trajopt_planner.setConfiguration(trajopt_config);
//    trajopt_planner.solve(response, false);
//  };

////  auto opml_id = planning_dispatch.dispatchPlanningRequest(ompl_fn);
//  auto trajopt_id = planning_dispatch.dispatchPlanningRequest(trajopt_fn);
//  auto trajopt_id2 = planning_dispatch.dispatchPlanningRequest(trajopt_fn2);

////  UNUSED(opml_id);
//  UNUSED(trajopt_id);
//  UNUSED(trajopt_id2);

//  while (!planning_dispatch.isIdle())
//  {
//    std::this_thread::sleep_for(std::chrono::milliseconds(100));
//  }

//  EXPECT_TRUE(planning_dispatch.hasResults());

//  PlanningDispatchQueue::TaskResult first;
//  EXPECT_TRUE(planning_dispatch.fetchResult(first));

//  PlanningDispatchQueue::TaskResult second;
//  EXPECT_TRUE(planning_dispatch.fetchResult(second));

//  EXPECT_FALSE(planning_dispatch.hasResults());

//  EXPECT_TRUE(first.second.status);
//  EXPECT_TRUE(second.second.status);

//}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
