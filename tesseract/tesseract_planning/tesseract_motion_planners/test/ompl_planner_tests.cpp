#include <tesseract_motion_planners/ompl/chain_ompl_interface.h>
#include <tesseract_motion_planners/ompl/conversions.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
//#include <ompl/geometric/planners/prm/PRM.h>    // These are other options for
// planners
//#include <ompl/geometric/planners/prm/PRMstar.h>
//#include <ompl/geometric/planners/prm/LazyPRMstar.h>
//#include <ompl/geometric/planners/prm/SPARS.h>

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <tesseract/tesseract.h>
#include <tesseract_environment/core/utils.h>
#include <tesseract_motion_planners/ompl/continuous_motion_validator.h>

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

TEST(TesseractPlanningUnit, OMPLPlannerUnit)
{
  // Step 1: Load scene and srdf
  ResourceLocatorFn locator = locateResource;
  Tesseract::Ptr tesseract = std::make_shared<Tesseract>();
  boost::filesystem::path urdf_path(std::string(TESSERACT_SUPPORT_DIR) + "/urdf/lbr_iiwa_14_r820.urdf");
  boost::filesystem::path srdf_path(std::string(TESSERACT_SUPPORT_DIR) + "/urdf/lbr_iiwa_14_r820.srdf");
  EXPECT_TRUE(tesseract->init(urdf_path, srdf_path, locator));

  // Step 2: Add box to environment
  addBox(*(tesseract->getEnvironment()));

  // A tesseract plotter makes generating and publishing visualization messages
  // easy
  //  tesseract::tesseract_ros::ROSBasicPlottingPtr plotter =
  //      std::make_shared<tesseract::tesseract_ros::ROSBasicPlotting>(env);

  // Step 4: Create a planning context for OMPL - this sets up the OMPL state environment for your given chain
  tesseract_motion_planners::ChainOmplInterface ompl_context(
      tesseract->getEnvironment(), tesseract->getFwdKinematicsManagerConst()->getFwdKinematicSolver("manipulator"));

  ompl::base::MotionValidatorPtr mv = std::make_shared<tesseract_motion_planners::ContinuousMotionValidator>(
      ompl_context.spaceInformation(),
      tesseract->getEnvironment(),
      tesseract->getFwdKinematicsManagerConst()->getFwdKinematicSolver("manipulator"));
  ompl_context.setMotionValidator(mv);

  // Step 5: Create an OMPL planner that we want to use
  ompl::base::PlannerPtr planner = std::make_shared<ompl::geometric::RRTConnect>(ompl_context.spaceInformation());

  // Step 6: Create a start and terminal state for the robot to move between
  tesseract_motion_planners::OmplPlanParameters params;
  std::vector<double> start = { -1.2, 0.5, 0.0, -1.3348, 0.0, 1.4959, 0.0 };
  std::vector<double> goal = { 1.2, 0.2762, 0.0, -1.3348, 0.0, 1.4959, 0.0 };

  // Step 7: Call plan. This returns an optional which is set if the plan
  // succeeded
  auto maybe_path = ompl_context.plan(planner, start, goal, params);

  EXPECT_TRUE(maybe_path);

  // Plot results
  //  if (maybe_path)
  //  {
  //    const ompl::geometric::PathGeometric& path = *maybe_path;
  //    const auto& names = env->getManipulator("manipulator")->getJointNames();
  //    plotter->plotTrajectory(names, tesseract::tesseract_motion_planners::toTrajArray(path));
  //  }
  //  else
  //  {
  //    ROS_WARN_STREAM("Planning failed");
  //  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
