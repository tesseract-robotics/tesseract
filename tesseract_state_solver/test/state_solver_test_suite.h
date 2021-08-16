#ifndef TESSERACT_STATE_SOLVER_STATE_SOLVER_TEST_SUITE_H
#define TESSERACT_STATE_SOLVER_STATE_SOLVER_TEST_SUITE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <algorithm>
#include <vector>
#include <tesseract_urdf/urdf_parser.h>
#include <tesseract_scene_graph/resource_locator.h>
#include <tesseract_geometry/impl/box.h>
#include <tesseract_common/utils.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_state_solver/kdl/kdl_state_solver.h>

namespace tesseract_scene_graph
{
namespace test_suite
{
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

SceneGraph::UPtr getSceneGraph()
{
  std::string path = std::string(TESSERACT_SUPPORT_DIR) + "/urdf/lbr_iiwa_14_r820.urdf";

  tesseract_scene_graph::ResourceLocator::Ptr locator =
      std::make_shared<tesseract_scene_graph::SimpleResourceLocator>(locateResource);
  return tesseract_urdf::parseURDFFile(path, locator);
}

SceneGraph::UPtr getSubSceneGraph()
{
  auto subgraph = std::make_unique<SceneGraph>();
  subgraph->setName("subgraph");

  // Now add a link to empty environment
  auto visual = std::make_shared<Visual>();
  visual->geometry = std::make_shared<tesseract_geometry::Box>(1, 1, 1);
  auto collision = std::make_shared<Collision>();
  collision->geometry = std::make_shared<tesseract_geometry::Box>(1, 1, 1);

  const std::string link_name1 = "subgraph_base_link";
  const std::string link_name2 = "subgraph_link_1";
  const std::string joint_name1 = "subgraph_joint1";
  Link link_1(link_name1);
  link_1.visual.push_back(visual);
  link_1.collision.push_back(collision);
  Link link_2(link_name2);

  Joint joint_1(joint_name1);
  joint_1.parent_to_joint_origin_transform.translation()(0) = 1.25;
  joint_1.parent_link_name = link_name1;
  joint_1.child_link_name = link_name2;
  joint_1.type = JointType::FIXED;

  subgraph->addLink(link_1);
  subgraph->addLink(link_2);
  subgraph->addJoint(joint_1);

  return subgraph;
}

void runCompareSceneStates(const SceneState& base_state, const SceneState& compare_state)
{
  EXPECT_EQ(base_state.joints.size(), compare_state.joints.size());
  EXPECT_EQ(base_state.joint_transforms.size(), compare_state.joint_transforms.size());
  EXPECT_EQ(base_state.link_transforms.size(), compare_state.link_transforms.size());

  for (const auto& pair : base_state.joints)
  {
    EXPECT_NEAR(pair.second, compare_state.joints.at(pair.first), 1e-6);
  }

  for (const auto& pair : base_state.joint_transforms)
  {
    EXPECT_TRUE(pair.second.isApprox(compare_state.joint_transforms.at(pair.first), 1e-6));
  }

  for (const auto& link_pair : base_state.link_transforms)
  {
    EXPECT_TRUE(link_pair.second.isApprox(compare_state.link_transforms.at(link_pair.first), 1e-6));
  }
}

void runCompareStateSolver(const StateSolver& base_solver, StateSolver& comp_solver)
{
  std::vector<std::string> base_joint_names = base_solver.getJointNames();
  std::vector<std::string> comp_joint_names = comp_solver.getJointNames();
  EXPECT_TRUE(tesseract_common::isIdentical(base_joint_names, comp_joint_names, false));
  // @todo compare joint limits

  for (int i = 0; i < 10; ++i)
  {
    SceneState base_random_state = base_solver.getRandomState();
    SceneState comp_state_const = comp_solver.getState(base_random_state.joints);
    comp_solver.setState(base_random_state.joints);
    const SceneState& comp_state = comp_solver.getState();

    runCompareSceneStates(base_random_state, comp_state_const);
    runCompareSceneStates(base_random_state, comp_state);
  }
}

void runCompareStateSolverLimits(const SceneGraph& scene_graph, const StateSolver& comp_solver)
{
  std::vector<std::string> comp_joint_names = comp_solver.getJointNames();
  tesseract_common::KinematicLimits limits = comp_solver.getLimits();

  for (Eigen::Index i = 0; i < static_cast<Eigen::Index>(comp_joint_names.size()); ++i)
  {
    const auto& scene_joint = scene_graph.getJoint(comp_joint_names[static_cast<std::size_t>(i)]);
    EXPECT_NEAR(limits.joint_limits(i, 0), scene_joint->limits->lower, 1e-5);
    EXPECT_NEAR(limits.joint_limits(i, 1), scene_joint->limits->upper, 1e-5);
    EXPECT_NEAR(limits.velocity_limits(i), scene_joint->limits->velocity, 1e-5);
    EXPECT_NEAR(limits.acceleration_limits(i), scene_joint->limits->acceleration, 1e-5);
  }
}

template <typename S>
void runAddandRemoveLinkTest()
{
  // Get the scene graph
  auto scene_graph = getSceneGraph();
  auto state_solver = S(*scene_graph);

  auto visual = std::make_shared<Visual>();
  visual->geometry = std::make_shared<tesseract_geometry::Box>(1, 1, 1);
  auto collision = std::make_shared<Collision>();
  collision->geometry = std::make_shared<tesseract_geometry::Box>(1, 1, 1);

  const std::string link_name1 = "link_n1";
  const std::string link_name2 = "link_n2";
  const std::string joint_name1 = "joint_n1";
  const std::string joint_name2 = "joint_n2";

  Link link_1(link_name1);
  link_1.visual.push_back(visual);
  link_1.collision.push_back(collision);

  Joint joint_1(joint_name1);
  joint_1.parent_link_name = scene_graph->getRoot();
  joint_1.child_link_name = link_name1;
  joint_1.type = JointType::FIXED;

  Link link_2(link_name2);

  Joint joint_2(joint_name2);
  joint_2.parent_to_joint_origin_transform.translation()(0) = 1.25;
  joint_2.parent_link_name = link_name1;
  joint_2.child_link_name = link_name2;
  joint_2.type = JointType::FIXED;

  // Test adding link

  EXPECT_TRUE(scene_graph->addLink(link_1, joint_1));
  EXPECT_TRUE(state_solver.addLink(link_1, joint_1));

  KDLStateSolver base_state_solver(*scene_graph);

  runCompareStateSolver(base_state_solver, state_solver);
  runCompareStateSolverLimits(*scene_graph, base_state_solver);

  std::vector<std::string> joint_names = state_solver.getJointNames();
  // Fixed joints are not listed
  EXPECT_TRUE(std::find(joint_names.begin(), joint_names.end(), joint_name1) == joint_names.end());
  EXPECT_TRUE(state_solver.getState().link_transforms.find(link_name1) !=
              state_solver.getState().link_transforms.end());
  EXPECT_TRUE(state_solver.getState().joint_transforms.find(joint_name1) !=
              state_solver.getState().joint_transforms.end());
  // Fixed joints are not listed
  EXPECT_TRUE(state_solver.getState().joints.find(joint_name1) == state_solver.getState().joints.end());

  EXPECT_TRUE(scene_graph->addLink(link_2, joint_2));
  EXPECT_TRUE(state_solver.addLink(link_2, joint_2));

  base_state_solver = KDLStateSolver(*scene_graph);

  runCompareStateSolver(base_state_solver, state_solver);
  runCompareStateSolverLimits(*scene_graph, base_state_solver);

  joint_names = state_solver.getJointNames();
  // Fixed joints are not listed
  EXPECT_TRUE(std::find(joint_names.begin(), joint_names.end(), joint_name2) == joint_names.end());
  EXPECT_TRUE(state_solver.getState().link_transforms.find(link_name2) !=
              state_solver.getState().link_transforms.end());
  EXPECT_TRUE(state_solver.getState().joint_transforms.find(joint_name2) !=
              state_solver.getState().joint_transforms.end());
  // Fixed joints are not listed
  EXPECT_TRUE(state_solver.getState().joints.find(joint_name2) == state_solver.getState().joints.end());

  scene_graph->saveDOT(tesseract_common::getTempPath() + "state_solver_before_remove_link_unit.dot");

  // Test removing link
  EXPECT_TRUE(scene_graph->removeLink(link_name1, true));
  EXPECT_TRUE(state_solver.removeLink(link_name1));

  base_state_solver = KDLStateSolver(*scene_graph);

  runCompareStateSolver(base_state_solver, state_solver);
  runCompareStateSolverLimits(*scene_graph, base_state_solver);

  joint_names = state_solver.getJointNames();
  EXPECT_TRUE(std::find(joint_names.begin(), joint_names.end(), joint_name1) == joint_names.end());
  EXPECT_TRUE(state_solver.getState().link_transforms.find(link_name1) ==
              state_solver.getState().link_transforms.end());
  EXPECT_TRUE(state_solver.getState().joint_transforms.find(joint_name1) ==
              state_solver.getState().joint_transforms.end());
  EXPECT_TRUE(state_solver.getState().joints.find(joint_name1) == state_solver.getState().joints.end());
  EXPECT_TRUE(state_solver.getState().link_transforms.find(link_name2) ==
              state_solver.getState().link_transforms.end());
  EXPECT_TRUE(state_solver.getState().joint_transforms.find(joint_name2) ==
              state_solver.getState().joint_transforms.end());
  EXPECT_TRUE(state_solver.getState().joints.find(joint_name2) == state_solver.getState().joints.end());

  scene_graph->saveDOT(tesseract_common::getTempPath() + "state_solver_after_remove_link_unit.dot");

  // Test against double removing

  EXPECT_FALSE(state_solver.removeLink(link_name1));
  runCompareStateSolver(base_state_solver, state_solver);
  runCompareStateSolverLimits(*scene_graph, base_state_solver);

  EXPECT_FALSE(state_solver.removeLink(link_name2));
  runCompareStateSolver(base_state_solver, state_solver);
  runCompareStateSolverLimits(*scene_graph, base_state_solver);

  EXPECT_FALSE(state_solver.removeJoint(joint_name1));
  runCompareStateSolver(base_state_solver, state_solver);
  runCompareStateSolverLimits(*scene_graph, base_state_solver);

  EXPECT_FALSE(state_solver.removeJoint(joint_name2));
  runCompareStateSolver(base_state_solver, state_solver);
  runCompareStateSolverLimits(*scene_graph, base_state_solver);

  /////////////////////////////////////////////////////////////////////////////////////

  // Test adding link

  EXPECT_TRUE(scene_graph->addLink(link_1, joint_1));
  EXPECT_TRUE(state_solver.addLink(link_1, joint_1));

  base_state_solver = KDLStateSolver(*scene_graph);

  runCompareStateSolver(base_state_solver, state_solver);
  runCompareStateSolverLimits(*scene_graph, base_state_solver);

  joint_names = state_solver.getJointNames();
  // Fixed joints are not listed
  EXPECT_TRUE(std::find(joint_names.begin(), joint_names.end(), joint_name1) == joint_names.end());
  EXPECT_TRUE(state_solver.getState().link_transforms.find(link_name1) !=
              state_solver.getState().link_transforms.end());
  EXPECT_TRUE(state_solver.getState().joint_transforms.find(joint_name1) !=
              state_solver.getState().joint_transforms.end());
  // Fixed joints are not listed
  EXPECT_TRUE(state_solver.getState().joints.find(joint_name1) == state_solver.getState().joints.end());

  EXPECT_TRUE(scene_graph->addLink(link_2, joint_2));
  EXPECT_TRUE(state_solver.addLink(link_2, joint_2));

  base_state_solver = KDLStateSolver(*scene_graph);

  runCompareStateSolver(base_state_solver, state_solver);
  runCompareStateSolverLimits(*scene_graph, base_state_solver);

  joint_names = state_solver.getJointNames();
  // Fixed joints are not listed
  EXPECT_TRUE(std::find(joint_names.begin(), joint_names.end(), joint_name2) == joint_names.end());
  EXPECT_TRUE(state_solver.getState().link_transforms.find(link_name2) !=
              state_solver.getState().link_transforms.end());
  EXPECT_TRUE(state_solver.getState().joint_transforms.find(joint_name2) !=
              state_solver.getState().joint_transforms.end());
  // Fixed joints are not listed
  EXPECT_TRUE(state_solver.getState().joints.find(joint_name2) == state_solver.getState().joints.end());

  scene_graph->saveDOT(tesseract_common::getTempPath() + "state_solver_before_remove_link_unit2.dot");

  // Test removing link
  EXPECT_TRUE(scene_graph->removeJoint(joint_name1, true));
  EXPECT_TRUE(state_solver.removeJoint(joint_name1));

  base_state_solver = KDLStateSolver(*scene_graph);

  runCompareStateSolver(base_state_solver, state_solver);
  runCompareStateSolverLimits(*scene_graph, base_state_solver);

  joint_names = state_solver.getJointNames();
  EXPECT_TRUE(std::find(joint_names.begin(), joint_names.end(), joint_name1) == joint_names.end());
  EXPECT_TRUE(state_solver.getState().link_transforms.find(link_name1) ==
              state_solver.getState().link_transforms.end());
  EXPECT_TRUE(state_solver.getState().joint_transforms.find(joint_name1) ==
              state_solver.getState().joint_transforms.end());
  EXPECT_TRUE(state_solver.getState().joints.find(joint_name1) == state_solver.getState().joints.end());
  EXPECT_TRUE(state_solver.getState().link_transforms.find(link_name2) ==
              state_solver.getState().link_transforms.end());
  EXPECT_TRUE(state_solver.getState().joint_transforms.find(joint_name2) ==
              state_solver.getState().joint_transforms.end());
  EXPECT_TRUE(state_solver.getState().joints.find(joint_name2) == state_solver.getState().joints.end());

  scene_graph->saveDOT(tesseract_common::getTempPath() + "state_solver_after_remove_link_unit2.dot");

  EXPECT_FALSE(state_solver.removeLink(link_name1));
  runCompareStateSolver(base_state_solver, state_solver);
  runCompareStateSolverLimits(*scene_graph, base_state_solver);

  EXPECT_FALSE(state_solver.removeLink(link_name2));
  runCompareStateSolver(base_state_solver, state_solver);
  runCompareStateSolverLimits(*scene_graph, base_state_solver);

  EXPECT_FALSE(state_solver.removeJoint(joint_name1));
  runCompareStateSolver(base_state_solver, state_solver);
  runCompareStateSolverLimits(*scene_graph, base_state_solver);

  EXPECT_FALSE(state_solver.removeJoint(joint_name2));
  runCompareStateSolver(base_state_solver, state_solver);
  runCompareStateSolverLimits(*scene_graph, base_state_solver);
}

template <typename S>
void runAddSceneGraphTest()
{
  // Get the scene graph
  auto scene_graph = getSceneGraph();
  auto state_solver = S(*scene_graph);

  auto subgraph = std::make_unique<SceneGraph>();
  subgraph->setName("subgraph");

  Joint joint_1_empty("provided_subgraph_joint");
  joint_1_empty.parent_link_name = "base_link";
  joint_1_empty.child_link_name = "prefix_subgraph_base_link";
  joint_1_empty.type = JointType::FIXED;

  EXPECT_FALSE(scene_graph->insertSceneGraph(*subgraph, joint_1_empty));
  EXPECT_FALSE(state_solver.insertSceneGraph(*subgraph, joint_1_empty));

  KDLStateSolver base_state_solver(*scene_graph);

  runCompareStateSolver(base_state_solver, state_solver);
  runCompareStateSolverLimits(*scene_graph, base_state_solver);

  subgraph = getSubSceneGraph();

  const std::string subgraph_joint_name = "attach_subgraph_joint";

  Joint joint(subgraph_joint_name);
  joint.parent_link_name = scene_graph->getRoot();
  joint.child_link_name = subgraph->getRoot();
  joint.type = JointType::FIXED;

  EXPECT_TRUE(scene_graph->insertSceneGraph(*subgraph, joint));
  EXPECT_TRUE(state_solver.insertSceneGraph(*subgraph, joint));

  base_state_solver = KDLStateSolver(*scene_graph);

  runCompareStateSolver(base_state_solver, state_solver);
  runCompareStateSolverLimits(*scene_graph, base_state_solver);

  std::vector<std::string> joint_names = state_solver.getJointNames();
  EXPECT_TRUE(std::find(joint_names.begin(), joint_names.end(), subgraph_joint_name) == joint_names.end());
  EXPECT_TRUE(state_solver.getState().link_transforms.find(subgraph->getRoot()) !=
              state_solver.getState().link_transforms.end());
  EXPECT_TRUE(state_solver.getState().joint_transforms.find(subgraph_joint_name) !=
              state_solver.getState().joint_transforms.end());
  EXPECT_TRUE(state_solver.getState().joints.find(subgraph_joint_name) == state_solver.getState().joints.end());

  // Adding twice with the same name should fail
  EXPECT_FALSE(scene_graph->insertSceneGraph(*subgraph, joint));
  EXPECT_FALSE(state_solver.insertSceneGraph(*subgraph, joint));

  base_state_solver = KDLStateSolver(*scene_graph);

  runCompareStateSolver(base_state_solver, state_solver);
  runCompareStateSolverLimits(*scene_graph, base_state_solver);

  // Add subgraph with prefix
  std::string prefix = "prefix_";
  Joint prefix_joint(prefix + subgraph_joint_name);
  prefix_joint.parent_link_name = scene_graph->getRoot();
  prefix_joint.child_link_name = prefix + subgraph->getRoot();
  prefix_joint.type = JointType::FIXED;

  EXPECT_TRUE(scene_graph->insertSceneGraph(*subgraph, prefix_joint, prefix));
  EXPECT_TRUE(state_solver.insertSceneGraph(*subgraph, prefix_joint, prefix));

  base_state_solver = KDLStateSolver(*scene_graph);

  runCompareStateSolver(base_state_solver, state_solver);
  runCompareStateSolverLimits(*scene_graph, base_state_solver);

  joint_names = state_solver.getJointNames();
  EXPECT_TRUE(std::find(joint_names.begin(), joint_names.end(), prefix + subgraph_joint_name) == joint_names.end());
  EXPECT_TRUE(state_solver.getState().link_transforms.find(prefix + subgraph->getRoot()) !=
              state_solver.getState().link_transforms.end());
  EXPECT_TRUE(state_solver.getState().joint_transforms.find(prefix + subgraph_joint_name) !=
              state_solver.getState().joint_transforms.end());
  EXPECT_TRUE(state_solver.getState().joints.find(prefix + subgraph_joint_name) ==
              state_solver.getState().joints.end());

  // Add subgraph with prefix and joint
  prefix = "prefix2_";
  Joint prefix_joint2(prefix + subgraph_joint_name);
  prefix_joint2.parent_link_name = scene_graph->getRoot();
  prefix_joint2.child_link_name = prefix + subgraph->getRoot();
  prefix_joint2.type = JointType::FIXED;

  EXPECT_TRUE(scene_graph->insertSceneGraph(*subgraph, prefix_joint2, prefix));
  EXPECT_TRUE(state_solver.insertSceneGraph(*subgraph, prefix_joint2, prefix));

  base_state_solver = KDLStateSolver(*scene_graph);

  runCompareStateSolver(base_state_solver, state_solver);
  runCompareStateSolverLimits(*scene_graph, base_state_solver);

  joint_names = state_solver.getJointNames();
  EXPECT_TRUE(std::find(joint_names.begin(), joint_names.end(), prefix + subgraph_joint_name) == joint_names.end());
  EXPECT_TRUE(state_solver.getState().link_transforms.find(prefix + subgraph->getRoot()) !=
              state_solver.getState().link_transforms.end());
  EXPECT_TRUE(state_solver.getState().joint_transforms.find(prefix + subgraph_joint_name) !=
              state_solver.getState().joint_transforms.end());
  EXPECT_TRUE(state_solver.getState().joints.find(prefix + subgraph_joint_name) ==
              state_solver.getState().joints.end());
}

template <typename S>
void runChangeJointOriginTest()
{
  // Get the scene graph
  auto scene_graph = getSceneGraph();
  auto state_solver = S(*scene_graph);

  const std::string link_name1 = "link_n1";
  const std::string joint_name1 = "joint_n1";
  Link link_1(link_name1);

  Joint joint_1(joint_name1);
  joint_1.parent_link_name = scene_graph->getRoot();
  joint_1.child_link_name = link_name1;
  joint_1.type = JointType::FIXED;

  EXPECT_TRUE(scene_graph->addLink(link_1, joint_1));
  EXPECT_TRUE(state_solver.addLink(link_1, joint_1));

  KDLStateSolver base_state_solver(*scene_graph);

  runCompareStateSolver(base_state_solver, state_solver);
  runCompareStateSolverLimits(*scene_graph, base_state_solver);

  EXPECT_TRUE(state_solver.getState().link_transforms.find(link_name1) !=
              state_solver.getState().link_transforms.end());
  EXPECT_TRUE(state_solver.getState().joint_transforms.find(joint_name1) !=
              state_solver.getState().joint_transforms.end());
  EXPECT_TRUE(state_solver.getState().joints.find(joint_name1) == state_solver.getState().joints.end());

  scene_graph->saveDOT(tesseract_common::getTempPath() + "state_solver_before_change_joint_origin_unit.dot");

  Eigen::Isometry3d new_origin = Eigen::Isometry3d::Identity();
  new_origin.translation()(0) += 1.234;

  EXPECT_TRUE(scene_graph->changeJointOrigin(joint_name1, new_origin));
  EXPECT_TRUE(state_solver.changeJointOrigin(joint_name1, new_origin));

  base_state_solver = KDLStateSolver(*scene_graph);

  runCompareStateSolver(base_state_solver, state_solver);
  runCompareStateSolverLimits(*scene_graph, base_state_solver);

  // Check that the origin got updated
  EXPECT_TRUE(state_solver.getState().link_transforms.at(link_name1).isApprox(new_origin));
  EXPECT_TRUE(state_solver.getState().joint_transforms.at(joint_name1).isApprox(new_origin));

  scene_graph->saveDOT(tesseract_common::getTempPath() + "state_solver_after_change_joint_origin_unit.dot");
}

template <typename S>
void runMoveJointTest()
{
  // Get the scene graph
  auto scene_graph = getSceneGraph();
  auto state_solver = S(*scene_graph);

  const std::string link_name1 = "link_n1";
  const std::string link_name2 = "link_n2";
  const std::string joint_name1 = "joint_n1";
  const std::string joint_name2 = "joint_n2";
  Link link_1(link_name1);
  Link link_2(link_name2);

  Joint joint_1(joint_name1);
  joint_1.parent_link_name = scene_graph->getRoot();
  joint_1.child_link_name = link_name1;
  joint_1.type = JointType::FIXED;

  Joint joint_2(joint_name2);
  joint_2.parent_to_joint_origin_transform.translation()(0) = 1.25;
  joint_2.parent_link_name = link_name1;
  joint_2.child_link_name = link_name2;
  joint_2.type = JointType::FIXED;

  EXPECT_TRUE(scene_graph->addLink(link_1, joint_1));
  EXPECT_TRUE(state_solver.addLink(link_1, joint_1));

  KDLStateSolver base_state_solver(*scene_graph);

  runCompareStateSolver(base_state_solver, state_solver);
  runCompareStateSolverLimits(*scene_graph, base_state_solver);

  EXPECT_TRUE(state_solver.getState().link_transforms.find(link_name1) !=
              state_solver.getState().link_transforms.end());
  EXPECT_TRUE(state_solver.getState().joint_transforms.find(joint_name1) !=
              state_solver.getState().joint_transforms.end());
  EXPECT_TRUE(state_solver.getState().joints.find(joint_name1) == state_solver.getState().joints.end());

  EXPECT_TRUE(scene_graph->addLink(link_2, joint_2));
  EXPECT_TRUE(state_solver.addLink(link_2, joint_2));

  base_state_solver = KDLStateSolver(*scene_graph);

  runCompareStateSolver(base_state_solver, state_solver);

  std::vector<std::string> joint_names = state_solver.getJointNames();
  EXPECT_TRUE(std::find(joint_names.begin(), joint_names.end(), joint_name1) == joint_names.end());
  EXPECT_TRUE(std::find(joint_names.begin(), joint_names.end(), joint_name2) == joint_names.end());
  EXPECT_TRUE(state_solver.getState().link_transforms.find(link_name1) !=
              state_solver.getState().link_transforms.end());
  EXPECT_TRUE(state_solver.getState().joint_transforms.find(joint_name1) !=
              state_solver.getState().joint_transforms.end());
  EXPECT_TRUE(state_solver.getState().joints.find(joint_name1) == state_solver.getState().joints.end());
  EXPECT_TRUE(state_solver.getState().link_transforms.find(link_name2) !=
              state_solver.getState().link_transforms.end());
  EXPECT_TRUE(state_solver.getState().joint_transforms.find(joint_name2) !=
              state_solver.getState().joint_transforms.end());
  EXPECT_TRUE(state_solver.getState().joints.find(joint_name2) == state_solver.getState().joints.end());

  scene_graph->saveDOT(tesseract_common::getTempPath() + "state_solver_before_move_joint_unit.dot");

  EXPECT_TRUE(scene_graph->moveJoint(joint_name1, "tool0"));
  EXPECT_TRUE(state_solver.moveJoint(joint_name1, "tool0"));

  base_state_solver = KDLStateSolver(*scene_graph);

  runCompareStateSolver(base_state_solver, state_solver);
  runCompareStateSolverLimits(*scene_graph, base_state_solver);

  joint_names = state_solver.getJointNames();
  EXPECT_TRUE(std::find(joint_names.begin(), joint_names.end(), joint_name1) == joint_names.end());
  EXPECT_TRUE(std::find(joint_names.begin(), joint_names.end(), joint_name2) == joint_names.end());
  EXPECT_TRUE(state_solver.getState().link_transforms.find(link_name1) !=
              state_solver.getState().link_transforms.end());
  EXPECT_TRUE(state_solver.getState().joint_transforms.find(joint_name1) !=
              state_solver.getState().joint_transforms.end());
  EXPECT_TRUE(state_solver.getState().joints.find(joint_name1) == state_solver.getState().joints.end());
  EXPECT_TRUE(state_solver.getState().link_transforms.find(link_name2) !=
              state_solver.getState().link_transforms.end());
  EXPECT_TRUE(state_solver.getState().joint_transforms.find(joint_name2) !=
              state_solver.getState().joint_transforms.end());
  EXPECT_TRUE(state_solver.getState().joints.find(joint_name2) == state_solver.getState().joints.end());

  scene_graph->saveDOT(tesseract_common::getTempPath() + "state_solver_after_move_joint_unit.dot");
}

template <typename S>
void runMoveLinkTest()
{
  // Get the scene graph
  auto scene_graph = getSceneGraph();
  auto state_solver = S(*scene_graph);

  const std::string link_name1 = "link_n1";
  const std::string link_name2 = "link_n2";
  const std::string joint_name1 = "joint_n1";
  const std::string joint_name2 = "joint_n2";
  Link link_1(link_name1);
  Link link_2(link_name2);

  Joint joint_1(joint_name1);
  joint_1.parent_link_name = scene_graph->getRoot();
  joint_1.child_link_name = link_name1;
  joint_1.type = JointType::FIXED;

  Joint joint_2(joint_name2);
  joint_2.parent_to_joint_origin_transform.translation()(0) = 1.25;
  joint_2.parent_link_name = link_name1;
  joint_2.child_link_name = link_name2;
  joint_2.type = JointType::FIXED;

  EXPECT_TRUE(scene_graph->addLink(link_1, joint_1));
  EXPECT_TRUE(state_solver.addLink(link_1, joint_1));

  KDLStateSolver base_state_solver(*scene_graph);

  runCompareStateSolver(base_state_solver, state_solver);
  runCompareStateSolverLimits(*scene_graph, base_state_solver);

  EXPECT_TRUE(state_solver.getState().link_transforms.find(link_name1) !=
              state_solver.getState().link_transforms.end());
  EXPECT_TRUE(state_solver.getState().joint_transforms.find(joint_name1) !=
              state_solver.getState().joint_transforms.end());
  EXPECT_TRUE(state_solver.getState().joints.find(joint_name1) == state_solver.getState().joints.end());

  EXPECT_TRUE(scene_graph->addLink(link_2, joint_2));
  EXPECT_TRUE(state_solver.addLink(link_2, joint_2));

  base_state_solver = KDLStateSolver(*scene_graph);

  runCompareStateSolver(base_state_solver, state_solver);
  runCompareStateSolverLimits(*scene_graph, base_state_solver);

  std::vector<std::string> joint_names = state_solver.getJointNames();
  EXPECT_TRUE(std::find(joint_names.begin(), joint_names.end(), joint_name1) == joint_names.end());
  EXPECT_TRUE(std::find(joint_names.begin(), joint_names.end(), joint_name2) == joint_names.end());
  EXPECT_TRUE(state_solver.getState().link_transforms.find(link_name1) !=
              state_solver.getState().link_transforms.end());
  EXPECT_TRUE(state_solver.getState().joint_transforms.find(joint_name1) !=
              state_solver.getState().joint_transforms.end());
  EXPECT_TRUE(state_solver.getState().joints.find(joint_name1) == state_solver.getState().joints.end());
  EXPECT_TRUE(state_solver.getState().link_transforms.find(link_name2) !=
              state_solver.getState().link_transforms.end());
  EXPECT_TRUE(state_solver.getState().joint_transforms.find(joint_name2) !=
              state_solver.getState().joint_transforms.end());
  EXPECT_TRUE(state_solver.getState().joints.find(joint_name2) == state_solver.getState().joints.end());

  scene_graph->saveDOT(tesseract_common::getTempPath() + "state_solver_before_move_link_unit.dot");

  std::string moved_joint_name = joint_name1 + "_moved";
  Joint move_link_joint = joint_1.clone(moved_joint_name);
  move_link_joint.parent_link_name = "tool0";

  EXPECT_TRUE(scene_graph->moveLink(move_link_joint));
  EXPECT_TRUE(state_solver.moveLink(move_link_joint));

  base_state_solver = KDLStateSolver(*scene_graph);

  runCompareStateSolver(base_state_solver, state_solver);
  runCompareStateSolverLimits(*scene_graph, base_state_solver);

  joint_names = state_solver.getJointNames();
  EXPECT_TRUE(std::find(joint_names.begin(), joint_names.end(), joint_name1) == joint_names.end());
  EXPECT_TRUE(std::find(joint_names.begin(), joint_names.end(), moved_joint_name) == joint_names.end());
  EXPECT_TRUE(std::find(joint_names.begin(), joint_names.end(), joint_name2) == joint_names.end());

  EXPECT_TRUE(state_solver.getState().link_transforms.find(link_name1) !=
              state_solver.getState().link_transforms.end());
  EXPECT_TRUE(state_solver.getState().joint_transforms.find(joint_name1) ==
              state_solver.getState().joint_transforms.end());
  EXPECT_TRUE(state_solver.getState().joint_transforms.find(moved_joint_name) !=
              state_solver.getState().joint_transforms.end());
  EXPECT_TRUE(state_solver.getState().joints.find(joint_name1) == state_solver.getState().joints.end());
  EXPECT_TRUE(state_solver.getState().link_transforms.find(link_name2) !=
              state_solver.getState().link_transforms.end());
  EXPECT_TRUE(state_solver.getState().joint_transforms.find(joint_name2) !=
              state_solver.getState().joint_transforms.end());
  EXPECT_TRUE(state_solver.getState().joints.find(joint_name2) == state_solver.getState().joints.end());

  scene_graph->saveDOT(tesseract_common::getTempPath() + "state_solver_after_move_link_unit.dot");
}

template <typename S>
void runChangeJointLimitsTest()
{
  // Get the scene graph
  auto scene_graph = getSceneGraph();
  auto state_solver = S(*scene_graph);

  double new_lower = 1.0;
  double new_upper = 2.0;
  double new_velocity = 3.0;
  double new_acceleration = 4.0;

  scene_graph->changeJointPositionLimits("joint_a1", new_lower, new_upper);
  scene_graph->changeJointVelocityLimits("joint_a1", new_velocity);
  scene_graph->changeJointAccelerationLimits("joint_a1", new_acceleration);

  state_solver.changeJointPositionLimits("joint_a1", new_lower, new_upper);
  state_solver.changeJointVelocityLimits("joint_a1", new_velocity);
  state_solver.changeJointAccelerationLimits("joint_a1", new_acceleration);

  std::vector<std::string> joint_names = state_solver.getJointNames();
  long idx = std::distance(joint_names.begin(), std::find(joint_names.begin(), joint_names.end(), "joint_a1"));
  auto limits = state_solver.getLimits();
  EXPECT_NEAR(limits.joint_limits(idx, 0), new_lower, 1e-5);
  EXPECT_NEAR(limits.joint_limits(idx, 1), new_upper, 1e-5);
  EXPECT_NEAR(limits.velocity_limits(idx), new_velocity, 1e-5);
  EXPECT_NEAR(limits.acceleration_limits(idx), new_acceleration, 1e-5);
}

template <typename S>
void runReplaceJointTest()
{
  {  // Replace joint with same type
    // Get the scene graph
    auto scene_graph = getSceneGraph();
    auto state_solver = S(*scene_graph);

    Joint joint_1("joint_a1");
    joint_1.parent_to_joint_origin_transform.translation()(0) = 1.25;
    joint_1.parent_link_name = "base_link";
    joint_1.child_link_name = "link_1";
    joint_1.type = JointType::FIXED;

    EXPECT_TRUE(scene_graph->removeJoint("joint_a1"));
    EXPECT_TRUE(scene_graph->addJoint(joint_1));
    EXPECT_TRUE(state_solver.replaceJoint(joint_1));

    KDLStateSolver base_state_solver(*scene_graph);

    runCompareStateSolver(base_state_solver, state_solver);
    runCompareStateSolverLimits(*scene_graph, base_state_solver);
  }

  {  // Replace joint which exist but the link does not which should fail
    // Get the scene graph
    auto scene_graph = getSceneGraph();
    auto state_solver = S(*scene_graph);

    Joint joint_1("joint_a1");
    joint_1.parent_to_joint_origin_transform.translation()(0) = 1.25;
    joint_1.parent_link_name = "base_link";
    joint_1.child_link_name = "link_2_does_not_exist";
    joint_1.type = JointType::FIXED;

    EXPECT_FALSE(state_solver.replaceJoint(joint_1));

    KDLStateSolver base_state_solver(*scene_graph);

    runCompareStateSolver(base_state_solver, state_solver);
    runCompareStateSolverLimits(*scene_graph, base_state_solver);
  }

  {  // Replace joint with same type but change transform
    // Get the scene graph
    auto scene_graph = getSceneGraph();
    auto state_solver = S(*scene_graph);

    Joint new_joint_a1 = scene_graph->getJoint("joint_a1")->clone();
    new_joint_a1.parent_to_joint_origin_transform.translation()(0) = 1.25;

    EXPECT_TRUE(scene_graph->removeJoint("joint_a1"));
    EXPECT_TRUE(scene_graph->addJoint(new_joint_a1));
    EXPECT_TRUE(state_solver.replaceJoint(new_joint_a1));

    KDLStateSolver base_state_solver(*scene_graph);

    runCompareStateSolver(base_state_solver, state_solver);
    runCompareStateSolverLimits(*scene_graph, base_state_solver);
  }

  {  // Replace joint with different type (Fixed)
    // Get the scene graph
    auto scene_graph = getSceneGraph();
    auto state_solver = S(*scene_graph);

    Joint new_joint_a1 = scene_graph->getJoint("joint_a1")->clone();
    new_joint_a1.parent_to_joint_origin_transform.translation()(0) = 1.25;
    new_joint_a1.type = JointType::FIXED;

    EXPECT_TRUE(scene_graph->removeJoint("joint_a1"));
    EXPECT_TRUE(scene_graph->addJoint(new_joint_a1));
    EXPECT_TRUE(state_solver.replaceJoint(new_joint_a1));

    KDLStateSolver base_state_solver(*scene_graph);

    runCompareStateSolver(base_state_solver, state_solver);
    runCompareStateSolverLimits(*scene_graph, base_state_solver);
  }

  {  // Replace joint with different type (Continuous)
    // Get the scene graph
    auto scene_graph = getSceneGraph();
    auto state_solver = S(*scene_graph);

    Joint new_joint_a1 = scene_graph->getJoint("joint_a1")->clone();
    new_joint_a1.parent_to_joint_origin_transform.translation()(0) = 1.25;
    new_joint_a1.type = JointType::CONTINUOUS;

    EXPECT_TRUE(scene_graph->removeJoint("joint_a1"));
    EXPECT_TRUE(scene_graph->addJoint(new_joint_a1));
    EXPECT_TRUE(state_solver.replaceJoint(new_joint_a1));

    KDLStateSolver base_state_solver(*scene_graph);

    runCompareStateSolver(base_state_solver, state_solver);
    runCompareStateSolverLimits(*scene_graph, base_state_solver);
  }

  {  // Replace joint with different type (Prismatic)
    // Get the scene graph
    auto scene_graph = getSceneGraph();
    auto state_solver = S(*scene_graph);

    Joint new_joint_a1 = scene_graph->getJoint("joint_a1")->clone();
    new_joint_a1.parent_to_joint_origin_transform.translation()(0) = 1.25;
    new_joint_a1.type = JointType::PRISMATIC;

    EXPECT_TRUE(scene_graph->removeJoint("joint_a1"));
    EXPECT_TRUE(scene_graph->addJoint(new_joint_a1));
    EXPECT_TRUE(state_solver.replaceJoint(new_joint_a1));

    KDLStateSolver base_state_solver(*scene_graph);

    runCompareStateSolver(base_state_solver, state_solver);
    runCompareStateSolverLimits(*scene_graph, base_state_solver);
  }

  {  // Replace joint and with different parent should fail
    // Get the scene graph
    auto scene_graph = getSceneGraph();
    auto state_solver = S(*scene_graph);

    Joint new_joint_a3 = scene_graph->getJoint("joint_a3")->clone();
    new_joint_a3.parent_link_name = "base_link";

    EXPECT_FALSE(state_solver.replaceJoint(new_joint_a3));

    KDLStateSolver base_state_solver(*scene_graph);

    runCompareStateSolver(base_state_solver, state_solver);
    runCompareStateSolverLimits(*scene_graph, base_state_solver);
  }
}
}  // namespace test_suite
}  // namespace tesseract_scene_graph

#endif  // TESSERACT_STATE_SOLVER_
