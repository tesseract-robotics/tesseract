/**
 * @file ompl_constrained_planner_tests.cpp
 * @brief This contains unit test ompl constrained planning
 *
 * @author Levi Armstrong
 * @date February 24, 2020
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
#include <boost/filesystem/path.hpp>

#include <ompl/geometric/planners/sbl/SBL.h>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>
#include <ompl/geometric/planners/kpiece/BKPIECE1.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/TRRT.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/prm/LazyPRMstar.h>
#include <ompl/geometric/planners/prm/SPARS.h>

#include <ompl/util/RandomNumbers.h>

#include <functional>
#include <cmath>
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/tesseract.h>
#include <tesseract_environment/core/utils.h>
#include <tesseract_motion_planners/ompl/ompl_motion_planner.h>
#include <tesseract_motion_planners/ompl/ompl_planner_configurator.h>
#include <tesseract_motion_planners/ompl/config/ompl_planner_constrained_config.h>
#include <trajopt/problem_description.hpp>
#include <trajopt_utils/logging.hpp>

using namespace tesseract;
using namespace tesseract_scene_graph;
using namespace tesseract_collision;
using namespace tesseract_environment;
using namespace tesseract_geometry;
using namespace tesseract_kinematics;
using namespace tesseract_motion_planners;

const static int SEED = 1;
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

class GlassUprightConstraint : public ompl::base::Constraint
{
public:
  GlassUprightConstraint(const Eigen::Vector3d& normal, tesseract_kinematics::ForwardKinematics::Ptr fwd_kin)
    : ompl::base::Constraint(fwd_kin->numJoints(), 1), normal_(normal.normalized()), fwd_kin_(std::move(fwd_kin))
  {
  }

  ~GlassUprightConstraint() override = default;

  void function(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::Ref<Eigen::VectorXd> out) const override
  {
    Eigen::Isometry3d pose;
    fwd_kin_->calcFwdKin(pose, x);

    Eigen::Vector3d z_axis = pose.matrix().col(2).template head<3>().normalized();

    out[0] = std::atan2(z_axis.cross(normal_).norm(), z_axis.dot(normal_));
  }

private:
  Eigen::Vector3d normal_;
  tesseract_kinematics::ForwardKinematics::Ptr fwd_kin_;
};

TEST(OMPLConstraintPlanner, OMPLConstraintPlannerUnit)  // NOLINT
{
  EXPECT_EQ(ompl::RNG::getSeed(), SEED) << "Randomization seed does not match expected: " << ompl::RNG::getSeed()
                                        << " vs. " << SEED;

  // Step 1: Load scene and srdf
  tesseract_scene_graph::ResourceLocator::Ptr locator =
      std::make_shared<tesseract_scene_graph::SimpleResourceLocator>(locateResource);
  Tesseract::Ptr tesseract = std::make_shared<Tesseract>();
  boost::filesystem::path urdf_path(std::string(TESSERACT_SUPPORT_DIR) + "/urdf/lbr_iiwa_14_r820.urdf");
  boost::filesystem::path srdf_path(std::string(TESSERACT_SUPPORT_DIR) + "/urdf/lbr_iiwa_14_r820.srdf");
  EXPECT_TRUE(tesseract->init(urdf_path, srdf_path, locator));

  // Step 3: Create ompl planner config and populate it
  auto kin = tesseract->getEnvironment()->getManipulatorManager()->getFwdKinematicSolver("manipulator");
  std::vector<double> swp = start_state;
  std::vector<double> ewp = end_state;

  Eigen::Vector3d normal = -1.0 * Eigen::Vector3d::UnitZ();
  tesseract_motion_planners::OMPLMotionPlanner ompl_planner;

  std::vector<tesseract_motion_planners::OMPLPlannerConfigurator::ConstPtr> planners;

  auto rrtconnect_planner = std::make_shared<tesseract_motion_planners::RRTConnectConfigurator>();
  planners.push_back(rrtconnect_planner);

  auto ompl_config =
      std::make_shared<tesseract_motion_planners::OMPLPlannerConstrainedConfig>(tesseract, "manipulator", planners);

  ompl_config->start_waypoint = std::make_shared<tesseract_motion_planners::JointWaypoint>(swp, kin->getJointNames());
  ompl_config->end_waypoint = std::make_shared<tesseract_motion_planners::JointWaypoint>(ewp, kin->getJointNames());
  ompl_config->collision_safety_margin = 0.02;
  ompl_config->planning_time = 400.0;
  ompl_config->max_solutions = 2;
  ompl_config->longest_valid_segment_fraction = 0.01;

  ompl_config->collision_continuous = true;
  ompl_config->collision_check = true;
  ompl_config->simplify = false;
  ompl_config->n_output_states = 50;
  ompl_config->constraint = std::make_shared<GlassUprightConstraint>(normal, kin);

  // Set the planner configuration
  ompl_planner.setConfiguration(ompl_config);

  tesseract_motion_planners::PlannerResponse ompl_planning_response;
  tesseract_common::StatusCode status = ompl_planner.solve(ompl_planning_response);

  if (!status)
  {
    CONSOLE_BRIDGE_logError("CI Error: %s", status.message().c_str());
  }
  EXPECT_TRUE(status);
  CONSOLE_BRIDGE_logInform("Number of states: %d", ompl_planning_response.joint_trajectory.trajectory.rows());

  EXPECT_TRUE(ompl_planning_response.joint_trajectory.trajectory.rows() > 2);

  long cnt = 0;
  double max_angle = 0;
  for (long r = 0; r < ompl_planning_response.joint_trajectory.trajectory.rows(); ++r)
  {
    Eigen::Isometry3d pose;
    kin->calcFwdKin(pose, ompl_planning_response.joint_trajectory.trajectory.row(r));

    Eigen::Vector3d z_axis = pose.matrix().col(2).template head<3>().normalized();

    double angle = std::abs(std::atan2(z_axis.cross(normal).norm(), z_axis.dot(normal)));

    if (angle > max_angle)
      max_angle = angle;

    if (angle > 0.2)
      ++cnt;
  }

  CONSOLE_BRIDGE_logWarn("CI Warn: Max error found: %f radians", max_angle);

  if (cnt != 0)
  {
    CONSOLE_BRIDGE_logError("CI Error: %d out of %d did not satisfy constraint",
                            cnt,
                            ompl_planning_response.joint_trajectory.trajectory.rows());
  }

  EXPECT_TRUE(cnt == 0);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  // Set the randomization seed for the planners to get repeatable results
  ompl::RNG::setSeed(SEED);

  return RUN_ALL_TESTS();
}
