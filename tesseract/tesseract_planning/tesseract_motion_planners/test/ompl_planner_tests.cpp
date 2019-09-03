#include <tesseract_motion_planners/ompl/conversions.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/prm/LazyPRMstar.h>
#include <ompl/geometric/planners/prm/SPARS.h>

#include <tesseract/tesseract.h>
#include <tesseract_environment/core/utils.h>
#include <tesseract_motion_planners/ompl/continuous_motion_validator.h>
#include <tesseract_motion_planners/ompl/ompl_freespace_planner.h>
#include <tesseract_motion_planners/ompl/ompl_settings.h>

#include <functional>
#include <gtest/gtest.h>

using namespace tesseract;
using namespace tesseract_scene_graph;
using namespace tesseract_collision;
using namespace tesseract_environment;
using namespace tesseract_geometry;
using namespace tesseract_kinematics;
using namespace tesseract_motion_planners;

std::string locateResource(const std::string& url)
{
  std::string mod_url = url;
  if (url.find("package://tesseract_support") == 0)
  {
    mod_url.erase(0, strlen("package://tesseract_support"));
    size_t pos = mod_url.find("/");
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
  visual->origin.translation() = Eigen::Vector3d(0.4, 0, 0.55);
  visual->geometry = std::make_shared<tesseract_geometry::Box>(0.5, 0.001, 0.5);
  link_1.visual.push_back(visual);

  Collision::Ptr collision = std::make_shared<Collision>();
  collision->origin = visual->origin;
  collision->geometry = visual->geometry;
  link_1.collision.push_back(collision);

  Joint joint_1("joint_n1");
  joint_1.parent_link_name = "base_link";
  joint_1.child_link_name = link_1.getName();
  joint_1.type = JointType::FIXED;

  env.addLink(link_1, joint_1);
}

TEST(TesseractPlanningUnit, OMPLFreespacePlannerUnit)
{
  // Step 1: Load scene and srdf
  ResourceLocatorFn locator = locateResource;
  Tesseract::Ptr tesseract = std::make_shared<Tesseract>();
  boost::filesystem::path urdf_path(std::string(TESSERACT_SUPPORT_DIR) + "/urdf/lbr_iiwa_14_r820.urdf");
  boost::filesystem::path srdf_path(std::string(TESSERACT_SUPPORT_DIR) + "/urdf/lbr_iiwa_14_r820.srdf");
  EXPECT_TRUE(tesseract->init(urdf_path, srdf_path, locator));

  // Step 2: Add box to environment
  addBox(*(tesseract->getEnvironment()));

  // Step 3: Create ompl planner config and populate it
  auto kin = tesseract->getFwdKinematicsManagerConst()->getFwdKinematicSolver("manipulator");
  std::vector<double> swp = { -1.2, 0.5, 0.0, -1.3348, 0.0, 1.4959, 0.0 };
  std::vector<double> ewp = { 1.2, 0.2762, 0.0, -1.3348, 0.0, 1.4959, 0.0 };
  tesseract_motion_planners::OMPLFreespacePlannerConfig<RRTConnectConfig> rrt_connect_config;
  rrt_connect_config.start_waypoint =
      std::make_shared<tesseract_motion_planners::JointWaypoint>(swp, kin->getJointNames());
  rrt_connect_config.end_waypoint =
      std::make_shared<tesseract_motion_planners::JointWaypoint>(ewp, kin->getJointNames());
  rrt_connect_config.tesseract = tesseract;
  rrt_connect_config.manipulator = "manipulator";
  rrt_connect_config.collision_safety_margin = 0.01;
  rrt_connect_config.planning_time = 5;
  rrt_connect_config.num_threads = 4;
  rrt_connect_config.max_solutions = 4;
  rrt_connect_config.settings.range = 0.1;

  // RRTConnect Solve
  tesseract_motion_planners::OMPLFreespacePlanner<ompl::geometric::RRTConnect, RRTConnectConfig> rrt_connect_planner;
  rrt_connect_planner.setConfiguration(rrt_connect_config);

  tesseract_motion_planners::PlannerResponse rrt_connect_planning_response;
  tesseract_common::StatusCode status = rrt_connect_planner.solve(rrt_connect_planning_response);

  EXPECT_TRUE(status);

  // Check for start state in collision error
  swp = { 0, 0.7, 0.0, 0, 0.0, 0, 0.0 };
  rrt_connect_config.start_waypoint =
      std::make_shared<tesseract_motion_planners::JointWaypoint>(swp, kin->getJointNames());

  rrt_connect_planner.setConfiguration(rrt_connect_config);
  status = rrt_connect_planner.solve(rrt_connect_planning_response);

  EXPECT_FALSE(status);

  // Check for start state in collision error
  swp = { -1.2, 0.5, 0.0, -1.3348, 0.0, 1.4959, 0.0 };
  ewp = { 0, 0.7, 0.0, 0, 0.0, 0, 0.0 };
  rrt_connect_config.start_waypoint =
      std::make_shared<tesseract_motion_planners::JointWaypoint>(swp, kin->getJointNames());
  rrt_connect_config.end_waypoint =
      std::make_shared<tesseract_motion_planners::JointWaypoint>(ewp, kin->getJointNames());

  rrt_connect_planner.setConfiguration(rrt_connect_config);
  status = rrt_connect_planner.solve(rrt_connect_planning_response);

  EXPECT_FALSE(status);

  // Reset start and end waypoints
  swp = { -1.2, 0.5, 0.0, -1.3348, 0.0, 1.4959, 0.0 };
  ewp = { 1.2, 0.2762, 0.0, -1.3348, 0.0, 1.4959, 0.0 };
  rrt_connect_config.start_waypoint =
      std::make_shared<tesseract_motion_planners::JointWaypoint>(swp, kin->getJointNames());
  rrt_connect_config.end_waypoint =
      std::make_shared<tesseract_motion_planners::JointWaypoint>(ewp, kin->getJointNames());

  // PRM Solve
  tesseract_motion_planners::OMPLFreespacePlannerConfig<PRMConfig> prm_config;
  prm_config.start_waypoint = std::make_shared<tesseract_motion_planners::JointWaypoint>(swp, kin->getJointNames());
  prm_config.end_waypoint = std::make_shared<tesseract_motion_planners::JointWaypoint>(ewp, kin->getJointNames());
  prm_config.tesseract = tesseract;
  prm_config.manipulator = "manipulator";
  prm_config.collision_safety_margin = 0.01;
  prm_config.planning_time = 5;
  prm_config.num_threads = 4;
  prm_config.max_solutions = 4;
  prm_config.settings.max_nearest_neighbors = 5;

  tesseract_motion_planners::OMPLFreespacePlanner<ompl::geometric::PRM, PRMConfig> prm_planner;
  prm_planner.setConfiguration(prm_config);

  tesseract_motion_planners::PlannerResponse prm_planning_response;
  status = prm_planner.solve(prm_planning_response);

  EXPECT_TRUE(status);

  // PRMstar Solve
  tesseract_motion_planners::OMPLFreespacePlannerConfig<PRMstarConfig> prm_star_config;
  prm_star_config.start_waypoint =
      std::make_shared<tesseract_motion_planners::JointWaypoint>(swp, kin->getJointNames());
  prm_star_config.end_waypoint = std::make_shared<tesseract_motion_planners::JointWaypoint>(ewp, kin->getJointNames());
  prm_star_config.tesseract = tesseract;
  prm_star_config.manipulator = "manipulator";
  prm_star_config.collision_safety_margin = 0.01;
  prm_star_config.planning_time = 5;
  prm_star_config.num_threads = 4;
  prm_star_config.max_solutions = 4;

  tesseract_motion_planners::OMPLFreespacePlanner<ompl::geometric::PRMstar, PRMstarConfig> prm_star_planner;
  prm_star_planner.setConfiguration(prm_star_config);

  tesseract_motion_planners::PlannerResponse prm_star_planning_response;
  status = prm_star_planner.solve(prm_star_planning_response);

  EXPECT_TRUE(status);

  // LazyPRMstar Solve
  tesseract_motion_planners::OMPLFreespacePlannerConfig<LazyPRMstarConfig> lazy_prm_star_config;
  lazy_prm_star_config.start_waypoint =
      std::make_shared<tesseract_motion_planners::JointWaypoint>(swp, kin->getJointNames());
  lazy_prm_star_config.end_waypoint =
      std::make_shared<tesseract_motion_planners::JointWaypoint>(ewp, kin->getJointNames());
  lazy_prm_star_config.tesseract = tesseract;
  lazy_prm_star_config.manipulator = "manipulator";
  lazy_prm_star_config.collision_safety_margin = 0.01;
  lazy_prm_star_config.planning_time = 5;
  lazy_prm_star_config.num_threads = 4;
  lazy_prm_star_config.max_solutions = 4;

  tesseract_motion_planners::OMPLFreespacePlanner<ompl::geometric::LazyPRMstar, LazyPRMstarConfig>
      lazy_prm_star_planner;
  lazy_prm_star_planner.setConfiguration(lazy_prm_star_config);

  tesseract_motion_planners::PlannerResponse lazy_prm_star_planning_response;
  status = lazy_prm_star_planner.solve(lazy_prm_star_planning_response);

  EXPECT_TRUE(status);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
