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

#include <functional>
#include <gtest/gtest.h>

using namespace tesseract;
using namespace tesseract_scene_graph;
using namespace tesseract_collision;
using namespace tesseract_environment;
using namespace tesseract_geometry;
using namespace tesseract_kinematics;

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
  tesseract_motion_planners::OMPLFreespacePlannerConfig config;
  config.start_waypoint = std::make_shared<tesseract_motion_planners::JointWaypoint>(swp, kin->getJointNames());
  config.end_waypoint = std::make_shared<tesseract_motion_planners::JointWaypoint>(ewp, kin->getJointNames());
  config.tesseract = tesseract;
  config.manipulator = "manipulator";
  config.collision_safety_margin = 0.01;
  config.planning_time = 5;

  // RRTConnect Solve
  tesseract_motion_planners::OMPLFreespacePlanner<ompl::geometric::RRTConnect> rrt_connect_planner;
  rrt_connect_planner.setConfiguration(config);

  tesseract_motion_planners::PlannerResponse rrt_connect_planning_response;
  tesseract_common::StatusCode status = rrt_connect_planner.solve(rrt_connect_planning_response);

  EXPECT_TRUE(status);

  // PRM Solve
  tesseract_motion_planners::OMPLFreespacePlanner<ompl::geometric::PRM> prm_planner;
  prm_planner.setConfiguration(config);

  tesseract_motion_planners::PlannerResponse prm_planning_response;
  status = prm_planner.solve(prm_planning_response);

  EXPECT_TRUE(status);

  // PRMstar Solve
  tesseract_motion_planners::OMPLFreespacePlanner<ompl::geometric::PRMstar> prm_star_planner;
  prm_star_planner.setConfiguration(config);

  tesseract_motion_planners::PlannerResponse prm_star_planning_response;
  status = prm_star_planner.solve(prm_star_planning_response);

  EXPECT_TRUE(status);

  // LazyPRMstar Solve
  tesseract_motion_planners::OMPLFreespacePlanner<ompl::geometric::LazyPRMstar> lazy_prm_star_planner;
  lazy_prm_star_planner.setConfiguration(config);

  tesseract_motion_planners::PlannerResponse lazy_prm_star_planning_response;
  status = lazy_prm_star_planner.solve(lazy_prm_star_planning_response);

  EXPECT_TRUE(status);

  //  // SPARS Solve
  //  tesseract_motion_planners::OMPLFreespacePlanner<ompl::geometric::SPARS> spars_planner;
  //  spars_planner.setConfiguration(config);

  //  tesseract_motion_planners::PlannerResponse spars_planning_response;
  //  status = spars_planner.solve(spars_planning_response);

  //  EXPECT_TRUE(status);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
