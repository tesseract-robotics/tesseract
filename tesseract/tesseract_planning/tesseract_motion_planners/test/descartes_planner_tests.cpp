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
#include <tesseract_motion_planners/descartes/descartes_collision_edge_evaluator.h>
#include <tesseract_motion_planners/descartes/utils.h>
#include <tesseract_motion_planners/core/types.h>
#include <tesseract_motion_planners/core/utils.h>

const int NUM_STEPS = 200;

using namespace tesseract;
using namespace tesseract_environment;
using namespace tesseract_scene_graph;
using namespace tesseract_collision;
using namespace tesseract_motion_planners;
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

DescartesMotionPlannerConfigD
createDescartesPlannerConfig(const tesseract::Tesseract::ConstPtr& tesseract_ptr,
                             const std::string& /*manip*/,
                             const tesseract_kinematics::InverseKinematics::ConstPtr& kin,
                             const Eigen::Isometry3d& tcp,
                             const double robot_reach,
                             const tesseract_environment::EnvState::ConstPtr& current_state,
                             const std::vector<tesseract_motion_planners::Waypoint::Ptr>& waypoints,
                             bool use_collision_edge_evaluator = false)
{
  const std::vector<std::string>& joint_names = kin->getJointNames();
  const std::vector<std::string>& active_link_names = kin->getActiveLinkNames();

  tesseract_environment::AdjacencyMap::Ptr adjacency_map = std::make_shared<tesseract_environment::AdjacencyMap>(
      tesseract_ptr->getEnvironmentConst()->getSceneGraph(), active_link_names, current_state->transforms);

  // Create Collision Interface
  typename descartes_light::CollisionInterface<double>::Ptr coll_interface =
      std::make_shared<tesseract_motion_planners::DescartesCollision<double>>(
          tesseract_ptr->getEnvironmentConst(), adjacency_map->getActiveLinkNames(), joint_names);

  // Create Timing Constraint
  std::vector<descartes_core::TimingConstraint<double>> timing =
      makeTiming<double>(waypoints, std::numeric_limits<double>::max());

  // Create Edge Evaluator
  descartes_light::EdgeEvaluator<double>::Ptr edge_computer;
  if (!use_collision_edge_evaluator)
  {
    edge_computer = std::make_shared<descartes_light::EuclideanDistanceEdgeEvaluator<double>>(kin->numJoints());
  }
  else
  {
    edge_computer = std::make_shared<tesseract_motion_planners::DescartesCollisionEdgeEvaluator<double>>(
        tesseract_ptr->getEnvironmentConst(), adjacency_map->getActiveLinkNames(), joint_names);
  }

  // Create is valid function
  tesseract_motion_planners::DescartesIsValidFn<double> is_valid_fn =
      std::bind(&tesseract_kinematics::isWithinLimits<double>, std::placeholders::_1, kin->getLimits());

  // Create Position Samplers
  std::vector<typename descartes_light::PositionSampler<double>::Ptr> position_samplers =
      makeRobotSamplers<double>(waypoints, kin, coll_interface, current_state, tcp, robot_reach, true, is_valid_fn);

  return tesseract_motion_planners::DescartesMotionPlannerConfigD(tesseract_ptr,
                                                                  adjacency_map->getActiveLinkNames(),
                                                                  joint_names,
                                                                  edge_computer,
                                                                  timing,
                                                                  position_samplers,
                                                                  waypoints);
}

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
  }
};

TEST_F(TesseractPlanningDescartesUnit, DescartesPlannerFixedPoses)  // NOLINT
{
  // These specify the series of points to be optimized
  std::vector<tesseract_motion_planners::Waypoint::Ptr> waypoints;
  Eigen::VectorXd x = Eigen::VectorXd::LinSpaced(NUM_STEPS, -0.2, 0.2);
  for (int i = 0; i < x.size(); ++i)
  {
    CartesianWaypoint::Ptr waypoint =
        std::make_shared<CartesianWaypoint>(Eigen::Vector3d(0.8, x[i], 0.8), Eigen::Quaterniond(0, 0, -1.0, 0));
    waypoint->setIsCritical(true);
    waypoints.push_back(waypoint);
  }

  auto robot_kin = tesseract_ptr_->getInvKinematicsManagerConst()->getInvKinematicSolver("manipulator", "OPWInvKin");
  auto current_state = tesseract_ptr_->getEnvironmentConst()->getCurrentState();
  DescartesMotionPlannerConfigD config = createDescartesPlannerConfig(
      tesseract_ptr_, "manipulator", robot_kin, Eigen::Isometry3d::Identity(), 1.5, current_state, waypoints);
  config.num_threads = 1;
  DescartesMotionPlanner<double> single_descartes_planner;
  PlannerResponse single_planner_response;
  single_descartes_planner.setConfiguration(config);
  auto single_status = single_descartes_planner.solve(single_planner_response);
  EXPECT_TRUE(single_status);

  for (int i = 0; i < 10; ++i)
  {
    config.num_threads = 4;
    DescartesMotionPlanner<double> descartes_planner;
    PlannerResponse planner_response;
    descartes_planner.setConfiguration(config);
    auto status = descartes_planner.solve(planner_response);
    EXPECT_TRUE(status);
    for (int j = 0; j < static_cast<int>(waypoints.size()); ++j)
    {
      EXPECT_TRUE(single_planner_response.joint_trajectory.trajectory.row(j).isApprox(
          planner_response.joint_trajectory.trajectory.row(j), 1e-5));
    }
  }
}

TEST_F(TesseractPlanningDescartesUnit, DescartesPlannerAxialSymetric)  // NOLINT
{
  // These specify the series of points to be optimized
  std::vector<tesseract_motion_planners::Waypoint::Ptr> waypoints;
  Eigen::VectorXd x = Eigen::VectorXd::LinSpaced(NUM_STEPS, -0.2, 0.2);
  for (int i = 0; i < x.size(); ++i)
  {
    CartesianWaypoint::Ptr waypoint =
        std::make_shared<CartesianWaypoint>(Eigen::Vector3d(0.8, x[i], 0.8), Eigen::Quaterniond(0, 0, -1.0, 0));
    waypoint->setIsCritical(true);
    Eigen::VectorXd c(6);
    c << 1, 1, 1, 1, 1, 0;
    waypoint->setCoefficients(c);
    waypoints.push_back(waypoint);
  }

  auto robot_kin = tesseract_ptr_->getInvKinematicsManagerConst()->getInvKinematicSolver("manipulator", "OPWInvKin");
  auto current_state = tesseract_ptr_->getEnvironmentConst()->getCurrentState();
  DescartesMotionPlannerConfigD config = createDescartesPlannerConfig(
      tesseract_ptr_, "manipulator", robot_kin, Eigen::Isometry3d::Identity(), 1.5, current_state, waypoints);
  config.num_threads = 1;
  DescartesMotionPlanner<double> single_descartes_planner;
  PlannerResponse single_planner_response;
  single_descartes_planner.setConfiguration(config);
  auto single_status = single_descartes_planner.solve(single_planner_response);
  EXPECT_TRUE(single_status);

  for (int i = 0; i < 10; ++i)
  {
    config.num_threads = descartes_light::SolverD::getMaxThreads();
    DescartesMotionPlanner<double> descartes_planner;
    PlannerResponse planner_response;
    descartes_planner.setConfiguration(config);
    auto status = descartes_planner.solve(planner_response);
    EXPECT_TRUE(status);
    for (int j = 0; j < static_cast<int>(waypoints.size()); ++j)
    {
      EXPECT_TRUE(single_planner_response.joint_trajectory.trajectory.row(j).isApprox(
          planner_response.joint_trajectory.trajectory.row(j), 1e-5));
    }
  }
}

TEST_F(TesseractPlanningDescartesUnit, DescartesPlannerCollisionEdgeEvaluator)  // NOLINT
{
  // These specify the series of points to be optimized
  std::vector<tesseract_motion_planners::Waypoint::Ptr> waypoints;
  Eigen::VectorXd x = Eigen::VectorXd::LinSpaced(4, -0.1, 0.1);  // Reduced num of steps because test takes to long
  for (int i = 0; i < x.size(); ++i)
  {
    CartesianWaypoint::Ptr waypoint =
        std::make_shared<CartesianWaypoint>(Eigen::Vector3d(0.8, x[i], 0.8), Eigen::Quaterniond(0, 0, -1.0, 0));
    waypoint->setIsCritical(true);
    Eigen::VectorXd c(6);
    c << 1, 1, 1, 1, 1, 0;
    waypoint->setCoefficients(c);
    waypoints.push_back(waypoint);
  }

  auto robot_kin = tesseract_ptr_->getInvKinematicsManagerConst()->getInvKinematicSolver("manipulator", "OPWInvKin");
  auto current_state = tesseract_ptr_->getEnvironmentConst()->getCurrentState();
  DescartesMotionPlannerConfigD config = createDescartesPlannerConfig(
      tesseract_ptr_, "manipulator", robot_kin, Eigen::Isometry3d::Identity(), 1.5, current_state, waypoints, true);
  config.num_threads = 1;
  DescartesMotionPlanner<double> single_descartes_planner;
  PlannerResponse single_planner_response;
  single_descartes_planner.setConfiguration(config);
  auto single_status = single_descartes_planner.solve(single_planner_response);
  EXPECT_TRUE(single_status);

  for (int i = 0; i < 10; ++i)
  {
    config.num_threads = descartes_light::SolverD::getMaxThreads();
    DescartesMotionPlanner<double> descartes_planner;
    PlannerResponse planner_response;
    descartes_planner.setConfiguration(config);
    auto status = descartes_planner.solve(planner_response);
    EXPECT_TRUE(status);
    for (int j = 0; j < static_cast<int>(waypoints.size()); ++j)
    {
      EXPECT_TRUE(single_planner_response.joint_trajectory.trajectory.row(j).isApprox(
          planner_response.joint_trajectory.trajectory.row(j), 1e-5));
    }
  }
}

TEST_F(TesseractPlanningDescartesUnit, DescartesPlannerMakeRobotSampler)  // NOLINT
{
  auto robot_kin = tesseract_ptr_->getInvKinematicsManagerConst()->getInvKinematicSolver("manipulator", "OPWInvKin");
  auto current_state = tesseract_ptr_->getEnvironmentConst()->getCurrentState();

  // These specify the series of points to be optimized
  std::vector<tesseract_motion_planners::Waypoint::Ptr> waypoints;
  Eigen::VectorXd x = Eigen::VectorXd::LinSpaced(NUM_STEPS, -0.2, 0.2);
  for (int i = 0; i < x.size(); ++i)
  {
    if (i == 0)
    {
      std::vector<std::string> joint_names = robot_kin->getJointNames();
      Eigen::VectorXd joint_positions = tesseract_ptr_->getEnvironmentConst()->getCurrentJointValues(joint_names);
      JointWaypoint::Ptr waypoint = std::make_shared<JointWaypoint>(joint_positions, joint_names);
      waypoint->setIsCritical(true);
      waypoints.push_back(waypoint);
    }
    else if (i == 1)
    {
      CartesianWaypoint::Ptr waypoint =
          std::make_shared<CartesianWaypoint>(Eigen::Vector3d(0.8, x[i], 0.8), Eigen::Quaterniond(0, 0, -1.0, 0));
      waypoint->setIsCritical(true);
      Eigen::VectorXd c(6);
      c << 1, 1, 1, 1, 1, 1;
      waypoint->setCoefficients(c);
      waypoints.push_back(waypoint);
    }
    else
    {
      CartesianWaypoint::Ptr waypoint =
          std::make_shared<CartesianWaypoint>(Eigen::Vector3d(0.8, x[i], 0.8), Eigen::Quaterniond(0, 0, -1.0, 0));
      waypoint->setIsCritical(true);
      Eigen::VectorXd c(6);
      c << 1, 1, 1, 1, 1, 0;
      waypoint->setCoefficients(c);
      waypoints.push_back(waypoint);
    }
  }

  DescartesMotionPlannerConfigD config = createDescartesPlannerConfig(
      tesseract_ptr_, "manipulator", robot_kin, Eigen::Isometry3d::Identity(), 1.5, current_state, waypoints);
  config.num_threads = 1;

  // Verify that the right sampler was choosen
  EXPECT_TRUE(config.samplers.size() == static_cast<std::size_t>(x.size()));
  EXPECT_TRUE(config.samplers[0] != nullptr);
  EXPECT_TRUE(config.samplers[1] != nullptr);
  EXPECT_TRUE(config.samplers[2] != nullptr);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
