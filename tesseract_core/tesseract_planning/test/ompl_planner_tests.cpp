#include <tesseract_planning/ompl/chain_ompl_interface.h>
#include <tesseract_planning/ompl/conversions.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
//#include <ompl/geometric/planners/prm/PRM.h>    // These are other options for
// planners
//#include <ompl/geometric/planners/prm/PRMstar.h>
//#include <ompl/geometric/planners/prm/LazyPRMstar.h>
//#include <ompl/geometric/planners/prm/SPARS.h>

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <tesseract_environment/kdl/kdl_env.h>
#include <tesseract_environment/core/utils.h>
#include <tesseract_scene_graph/graph.h>
#include <tesseract_scene_graph/utils.h>
#include <tesseract_scene_graph/parser/urdf_parser.h>
#include <tesseract_scene_graph/parser/srdf_parser.h>
#include <tesseract_collision/bullet/bullet_cast_bvh_manager.h>
#include <tesseract_collision/bullet/bullet_discrete_bvh_manager.h>
#include <tesseract_planning/ompl/continuous_motion_validator.h>
#include <tesseract_geometry/geometries.h>
#include <tesseract_kinematics/kdl/kdl_fwd_kin_chain.h>
#include <tesseract_kinematics/kdl/kdl_fwd_kin_tree.h>
#include <tesseract_kinematics/core/utils.h>

#include <functional>
#include <gtest/gtest.h>

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

SceneGraphPtr getSceneGraph()
{
std::string path = std::string(TESSERACT_SUPPORT_DIR) + "/urdf/lbr_iiwa_14_r820.urdf";

tesseract_scene_graph::ResourceLocatorFn locator = locateResource;
return tesseract_scene_graph::parseURDF(path, locator);
}

SRDFModelPtr getSRDFModel(const SceneGraph& scene_graph)
{
std::string path = std::string(TESSERACT_SUPPORT_DIR) + "/urdf/lbr_iiwa_14_r820.srdf";
SRDFModelPtr srdf = std::make_shared<SRDFModel>();
srdf->initFile(scene_graph, path);

return srdf;
}

static void addBox(tesseract_environment::Environment& env)
{
  Link link_1("box_attached");

  VisualPtr visual = std::make_shared<Visual>();
  visual->origin = Eigen::Isometry3d::Identity();
  visual->origin.translation() = Eigen::Vector3d(0.4, 0, 0.55);
  visual->geometry = std::make_shared<tesseract_geometry::Box>(0.5, 0.001, 0.5);
  link_1.visual.push_back(visual);

  CollisionPtr collision = std::make_shared<Collision>();
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
  SceneGraphPtr scene_graph = getSceneGraph();
  EXPECT_TRUE(scene_graph != nullptr);

  SRDFModelPtr srdf_model = getSRDFModel(*scene_graph);
  EXPECT_TRUE(srdf_model != nullptr);

  // Add allowed collision to the scene
  processSRDFAllowedCollisions(*scene_graph, *srdf_model);

  // Step 2: Create a "tesseract" environment
  KDLEnvPtr env = std::make_shared<KDLEnv>();
  EXPECT_TRUE(env != nullptr);

  EXPECT_TRUE(env->init(scene_graph));

  // Register contact manager
  env->registerDiscreteContactManager("bullet", &tesseract_collision_bullet::BulletDiscreteBVHManager::create);
  env->registerContinuousContactManager("bullet", &tesseract_collision_bullet::BulletCastBVHManager::create);

  // Set Active contact manager
  env->setActiveDiscreteContactManager("bullet");
  env->setActiveContinuousContactManager("bullet");

  addBox(*env);

  // Step 3: Get kinematics objects from the srdf model
  ForwardKinematicsConstPtrMap kin_map = createKinematicsMap<KDLFwdKinChain, KDLFwdKinTree>(scene_graph, *srdf_model);

  // A tesseract plotter makes generating and publishing visualization messages
  // easy
//  tesseract::tesseract_ros::ROSBasicPlottingPtr plotter =
//      std::make_shared<tesseract::tesseract_ros::ROSBasicPlotting>(env);

  // Step 4: Create a planning context for OMPL - this sets up the OMPL state environment for your given chain
  tesseract_planning::ChainOmplInterface ompl_context(env, kin_map["manipulator"]);

  ompl::base::MotionValidatorPtr mv = std::make_shared<tesseract_planning::ContinuousMotionValidator>(ompl_context.spaceInformation(), env, kin_map["manipulator"]);
  ompl_context.setMotionValidator(mv);

  // Step 5: Create an OMPL planner that we want to use
  ompl::base::PlannerPtr planner = std::make_shared<ompl::geometric::RRTConnect>(ompl_context.spaceInformation());

  // Step 6: Create a start and terminal state for the robot to move between
  tesseract_planning::OmplPlanParameters params;
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
//    plotter->plotTrajectory(names, tesseract::tesseract_planning::toTrajArray(path));
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
