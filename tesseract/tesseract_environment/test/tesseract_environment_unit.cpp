#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <algorithm>
#include <vector>
#include <tesseract_urdf/urdf_parser.h>
#include <tesseract_collision/bullet/bullet_discrete_bvh_manager.h>
#include <tesseract_collision/bullet/bullet_cast_bvh_manager.h>
#include <tesseract_scene_graph/resource_locator.h>
#include <tesseract_common/utils.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/core/types.h>
#include <tesseract_environment/kdl/kdl_state_solver.h>
#include <tesseract_environment/ofkt/ofkt_state_solver.h>
#include <tesseract_environment/core/environment.h>

using namespace tesseract_scene_graph;
using namespace tesseract_collision;
using namespace tesseract_environment;

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

SceneGraph::Ptr getSceneGraph()
{
  std::string path = std::string(TESSERACT_SUPPORT_DIR) + "/urdf/lbr_iiwa_14_r820.urdf";

  tesseract_scene_graph::ResourceLocator::Ptr locator =
      std::make_shared<tesseract_scene_graph::SimpleResourceLocator>(locateResource);
  return tesseract_urdf::parseURDFFile(path, locator);
}

template <typename S>
Environment::Ptr getEnvironment()
{
  tesseract_scene_graph::SceneGraph::Ptr scene_graph = getSceneGraph();
  EXPECT_TRUE(scene_graph != nullptr);

  auto env = std::make_shared<Environment>();
  EXPECT_TRUE(env != nullptr);

  bool success = env->init<S>(*scene_graph);
  EXPECT_TRUE(success);

  // Register contact manager
  EXPECT_TRUE(env->registerDiscreteContactManager(tesseract_collision_bullet::BulletDiscreteBVHManager::name(),
                                                  &tesseract_collision_bullet::BulletDiscreteBVHManager::create));
  EXPECT_TRUE(env->registerContinuousContactManager(tesseract_collision_bullet::BulletCastBVHManager::name(),
                                                    &tesseract_collision_bullet::BulletCastBVHManager::create));

  // Set Active contact manager
  EXPECT_TRUE(env->setActiveDiscreteContactManager(tesseract_collision_bullet::BulletDiscreteBVHManager::name()));
  EXPECT_TRUE(env->setActiveContinuousContactManager(tesseract_collision_bullet::BulletCastBVHManager::name()));

  return env;
}

template <typename S>
void runContactManagerCloneTest()
{
  // Get the environment
  auto env = getEnvironment<S>();

  // Test after clone if active list correct
  tesseract_collision::DiscreteContactManager::Ptr discrete_manager = env->getDiscreteContactManager();
  const std::vector<std::string>& e_active_list = env->getActiveLinkNames();
  const std::vector<std::string>& d_active_list = discrete_manager->getActiveCollisionObjects();
  EXPECT_TRUE(std::equal(e_active_list.begin(), e_active_list.end(), d_active_list.begin()));

  tesseract_collision::ContinuousContactManager::Ptr cast_manager = env->getContinuousContactManager();
  const std::vector<std::string>& c_active_list = cast_manager->getActiveCollisionObjects();
  EXPECT_TRUE(std::equal(e_active_list.begin(), e_active_list.end(), c_active_list.begin()));
}

template <typename S>
void runAddandRemoveLinkTest()
{
  // Get the environment
  auto env = getEnvironment<S>();

  const std::string link_name1 = "link_n1";
  const std::string link_name2 = "link_n2";
  const std::string joint_name1 = "joint_n1";
  Link link_1(link_name1);
  Link link_2(link_name2);

  Joint joint_1(joint_name1);
  joint_1.parent_to_joint_origin_transform.translation()(0) = 1.25;
  joint_1.parent_link_name = link_name1;
  joint_1.child_link_name = link_name2;
  joint_1.type = JointType::FIXED;

  EXPECT_TRUE(env->addLink(std::move(link_1)));

  std::vector<std::string> link_names = env->getLinkNames();
  std::vector<std::string> joint_names = env->getJointNames();
  EXPECT_TRUE(std::find(link_names.begin(), link_names.end(), link_name1) != link_names.end());
  EXPECT_TRUE(std::find(joint_names.begin(), joint_names.end(), "joint_" + link_name1) != joint_names.end());
  EXPECT_TRUE(env->getCurrentState()->link_transforms.find(link_name1) !=
              env->getCurrentState()->link_transforms.end());
  EXPECT_TRUE(env->getCurrentState()->joint_transforms.find("joint_" + link_name1) !=
              env->getCurrentState()->joint_transforms.end());
  EXPECT_TRUE(env->getCurrentState()->joints.find("joint_" + link_name1) == env->getCurrentState()->joints.end());

  env->addLink(std::move(link_2), std::move(joint_1));
  link_names = env->getLinkNames();
  joint_names = env->getJointNames();
  EXPECT_TRUE(std::find(link_names.begin(), link_names.end(), link_name2) != link_names.end());
  EXPECT_TRUE(std::find(joint_names.begin(), joint_names.end(), joint_name1) != joint_names.end());
  EXPECT_TRUE(env->getCurrentState()->link_transforms.find(link_name2) !=
              env->getCurrentState()->link_transforms.end());
  EXPECT_TRUE(env->getCurrentState()->joint_transforms.find(joint_name1) !=
              env->getCurrentState()->joint_transforms.end());
  EXPECT_TRUE(env->getCurrentState()->joints.find(joint_name1) == env->getCurrentState()->joints.end());

  env->getSceneGraph()->saveDOT(tesseract_common::getTempPath() + "before_remove_link_unit.dot");

  env->removeLink(link_name1);
  link_names = env->getLinkNames();
  joint_names = env->getJointNames();
  EXPECT_TRUE(std::find(link_names.begin(), link_names.end(), link_name1) == link_names.end());
  EXPECT_TRUE(std::find(joint_names.begin(), joint_names.end(), "joint_" + link_name1) == joint_names.end());
  EXPECT_TRUE(std::find(link_names.begin(), link_names.end(), link_name2) == link_names.end());
  EXPECT_TRUE(std::find(joint_names.begin(), joint_names.end(), joint_name1) == joint_names.end());
  EXPECT_TRUE(env->getCurrentState()->link_transforms.find(link_name1) ==
              env->getCurrentState()->link_transforms.end());
  EXPECT_TRUE(env->getCurrentState()->joint_transforms.find("joint_" + link_name1) ==
              env->getCurrentState()->joint_transforms.end());
  EXPECT_TRUE(env->getCurrentState()->joints.find("joint_" + link_name1) == env->getCurrentState()->joints.end());
  EXPECT_TRUE(env->getCurrentState()->link_transforms.find(link_name2) ==
              env->getCurrentState()->link_transforms.end());
  EXPECT_TRUE(env->getCurrentState()->joint_transforms.find(joint_name1) ==
              env->getCurrentState()->joint_transforms.end());
  EXPECT_TRUE(env->getCurrentState()->joints.find(joint_name1) == env->getCurrentState()->joints.end());

  env->getSceneGraph()->saveDOT(tesseract_common::getTempPath() + "after_remove_link_unit.dot");

  // Test against double removing
  EXPECT_FALSE(env->removeLink(link_name1));
  EXPECT_FALSE(env->removeLink(link_name2));
  EXPECT_FALSE(env->removeJoint(joint_name1));
  EXPECT_FALSE(env->removeJoint("joint_" + link_name1));
}

template <typename S>
void runMoveLinkandJointTest()
{
  // Get the environment
  auto env = getEnvironment<S>();

  const std::string link_name1 = "link_n1";
  const std::string link_name2 = "link_n2";
  const std::string joint_name1 = "joint_n1";
  const std::string joint_name2 = "joint_n2";
  Link link_1(link_name1);
  Link link_2(link_name2);

  Joint joint_1(joint_name1);
  joint_1.parent_link_name = env->getRootLinkName();
  joint_1.child_link_name = link_name1;
  joint_1.type = JointType::FIXED;

  Joint joint_2(joint_name2);
  joint_2.parent_to_joint_origin_transform.translation()(0) = 1.25;
  joint_2.parent_link_name = link_name1;
  joint_2.child_link_name = link_name2;
  joint_2.type = JointType::FIXED;

  env->addLink(std::move(link_1), std::move(joint_1));
  EXPECT_TRUE(env->getCurrentState()->link_transforms.find(link_name1) !=
              env->getCurrentState()->link_transforms.end());
  EXPECT_TRUE(env->getCurrentState()->joint_transforms.find(joint_name1) !=
              env->getCurrentState()->joint_transforms.end());
  EXPECT_TRUE(env->getCurrentState()->joints.find(joint_name1) == env->getCurrentState()->joints.end());

  env->addLink(std::move(link_2), std::move(joint_2));
  std::vector<std::string> link_names = env->getLinkNames();
  std::vector<std::string> joint_names = env->getJointNames();
  EXPECT_TRUE(std::find(link_names.begin(), link_names.end(), link_name1) != link_names.end());
  EXPECT_TRUE(std::find(link_names.begin(), link_names.end(), link_name2) != link_names.end());
  EXPECT_TRUE(std::find(joint_names.begin(), joint_names.end(), joint_name1) != joint_names.end());
  EXPECT_TRUE(std::find(joint_names.begin(), joint_names.end(), joint_name2) != joint_names.end());
  EXPECT_TRUE(env->getCurrentState()->link_transforms.find(link_name1) !=
              env->getCurrentState()->link_transforms.end());
  EXPECT_TRUE(env->getCurrentState()->joint_transforms.find(joint_name1) !=
              env->getCurrentState()->joint_transforms.end());
  EXPECT_TRUE(env->getCurrentState()->joints.find(joint_name1) == env->getCurrentState()->joints.end());
  EXPECT_TRUE(env->getCurrentState()->link_transforms.find(link_name2) !=
              env->getCurrentState()->link_transforms.end());
  EXPECT_TRUE(env->getCurrentState()->joint_transforms.find(joint_name2) !=
              env->getCurrentState()->joint_transforms.end());
  EXPECT_TRUE(env->getCurrentState()->joints.find(joint_name2) == env->getCurrentState()->joints.end());

  env->getSceneGraph()->saveDOT(tesseract_common::getTempPath() + "before_move_joint_unit.dot");

  env->moveJoint(joint_name1, "tool0");
  link_names = env->getLinkNames();
  joint_names = env->getJointNames();
  EXPECT_TRUE(env->getJoint(joint_name1)->parent_link_name == "tool0");
  EXPECT_TRUE(std::find(link_names.begin(), link_names.end(), link_name1) != link_names.end());
  EXPECT_TRUE(std::find(link_names.begin(), link_names.end(), link_name2) != link_names.end());
  EXPECT_TRUE(std::find(joint_names.begin(), joint_names.end(), joint_name1) != joint_names.end());
  EXPECT_TRUE(std::find(joint_names.begin(), joint_names.end(), joint_name2) != joint_names.end());
  EXPECT_TRUE(env->getCurrentState()->link_transforms.find(link_name1) !=
              env->getCurrentState()->link_transforms.end());
  EXPECT_TRUE(env->getCurrentState()->joint_transforms.find(joint_name1) !=
              env->getCurrentState()->joint_transforms.end());
  EXPECT_TRUE(env->getCurrentState()->joints.find(joint_name1) == env->getCurrentState()->joints.end());
  EXPECT_TRUE(env->getCurrentState()->link_transforms.find(link_name2) !=
              env->getCurrentState()->link_transforms.end());
  EXPECT_TRUE(env->getCurrentState()->joint_transforms.find(joint_name2) !=
              env->getCurrentState()->joint_transforms.end());
  EXPECT_TRUE(env->getCurrentState()->joints.find(joint_name2) == env->getCurrentState()->joints.end());

  env->getSceneGraph()->saveDOT(tesseract_common::getTempPath() + "after_move_joint_unit.dot");
}

template <typename S>
void runChangeJointOriginTest()
{
  // Get the environment
  auto env = getEnvironment<S>();

  const std::string link_name1 = "link_n1";
  const std::string joint_name1 = "joint_n1";
  Link link_1(link_name1);

  Joint joint_1(joint_name1);
  joint_1.parent_link_name = env->getRootLinkName();
  joint_1.child_link_name = link_name1;
  joint_1.type = JointType::FIXED;

  env->addLink(std::move(link_1), std::move(joint_1));
  EXPECT_TRUE(env->getCurrentState()->link_transforms.find(link_name1) !=
              env->getCurrentState()->link_transforms.end());
  EXPECT_TRUE(env->getCurrentState()->joint_transforms.find(joint_name1) !=
              env->getCurrentState()->joint_transforms.end());
  EXPECT_TRUE(env->getCurrentState()->joints.find(joint_name1) == env->getCurrentState()->joints.end());

  env->getSceneGraph()->saveDOT(tesseract_common::getTempPath() + "before_change_joint_origin_unit.dot");

  Eigen::Isometry3d new_origin = Eigen::Isometry3d::Identity();
  new_origin.translation()(0) += 1.234;
  env->changeJointOrigin(joint_name1, new_origin);

  // Check that the origin got updated
  EXPECT_TRUE(env->getJoint(joint_name1)->parent_to_joint_origin_transform.isApprox(new_origin));
  EXPECT_TRUE(env->getCurrentState()->link_transforms.at(link_name1).isApprox(new_origin));
  EXPECT_TRUE(env->getCurrentState()->joint_transforms.at(joint_name1).isApprox(new_origin));

  env->getSceneGraph()->saveDOT(tesseract_common::getTempPath() + "after_change_joint_origin_unit.dot");
}

template <typename S>
void runCurrentStatePreservedWhenEnvChangesTest()
{
  // Get the environment
  auto env = getEnvironment<S>();

  // Set the initial state of the robot
  std::unordered_map<std::string, double> joint_states;
  joint_states["joint_a1"] = 0.0;
  joint_states["joint_a2"] = 0.0;
  joint_states["joint_a3"] = 0.0;
  joint_states["joint_a4"] = -1.57;
  joint_states["joint_a5"] = 0.0;
  joint_states["joint_a6"] = 0.0;
  joint_states["joint_a7"] = 0.0;
  env->setState(joint_states);

  EnvState::ConstPtr current_state = env->getCurrentState();
  for (auto& joint_state : joint_states)
  {
    EXPECT_NEAR(current_state->joints.at(joint_state.first), joint_state.second, 1e-5);
  }

  Link link("link_n1");

  Joint joint("joint_n1");
  joint.parent_link_name = env->getRootLinkName();
  joint.child_link_name = "link_n1";
  joint.type = JointType::FIXED;

  env->addLink(std::move(link), std::move(joint));

  current_state = env->getCurrentState();
  for (auto& joint_state : joint_states)
  {
    EXPECT_NEAR(current_state->joints.at(joint_state.first), joint_state.second, 1e-5);
  }
}

template <typename S>
void runAddSceneGraphTest()
{
  // Get the environment
  auto env = getEnvironment<S>();

  SceneGraph::Ptr subgraph = std::make_shared<SceneGraph>();
  subgraph->setName("subgraph");

  // Adding an empty scene graph which should fail
  EXPECT_FALSE(env->addSceneGraph(*subgraph));

  // Now add a link to empty environment
  Link link("subgraph_base_link");
  subgraph->addLink(std::move(link));

  EXPECT_TRUE(env->addSceneGraph(*subgraph));
  EXPECT_TRUE(env->getJoint("subgraph_joint") != nullptr);
  EXPECT_TRUE(env->getLink("subgraph_base_link") != nullptr);
  EXPECT_TRUE(env->getCurrentState()->link_transforms.find("subgraph_base_link") !=
              env->getCurrentState()->link_transforms.end());
  EXPECT_TRUE(env->getCurrentState()->joint_transforms.find("subgraph_joint") !=
              env->getCurrentState()->joint_transforms.end());
  EXPECT_TRUE(env->getCurrentState()->joints.find("subgraph_joint") == env->getCurrentState()->joints.end());

  // Adding twice with the same name should fail
  EXPECT_FALSE(env->addSceneGraph(*subgraph));
  EXPECT_TRUE(env->addSceneGraph(*subgraph, "prefix_"));
  EXPECT_TRUE(env->getJoint("prefix_subgraph_joint") != nullptr);
  EXPECT_TRUE(env->getLink("prefix_subgraph_base_link") != nullptr);
  EXPECT_TRUE(env->getCurrentState()->link_transforms.find("subgraph_base_link") !=
              env->getCurrentState()->link_transforms.end());
  EXPECT_TRUE(env->getCurrentState()->joint_transforms.find("subgraph_joint") !=
              env->getCurrentState()->joint_transforms.end());
  EXPECT_TRUE(env->getCurrentState()->joints.find("subgraph_joint") == env->getCurrentState()->joints.end());
  EXPECT_TRUE(env->getCurrentState()->link_transforms.find("prefix_subgraph_base_link") !=
              env->getCurrentState()->link_transforms.end());
  EXPECT_TRUE(env->getCurrentState()->joint_transforms.find("prefix_subgraph_joint") !=
              env->getCurrentState()->joint_transforms.end());
  EXPECT_TRUE(env->getCurrentState()->joints.find("prefix_subgraph_joint") == env->getCurrentState()->joints.end());
}

template <typename S>
void runApplyCommandsTest()
{
  // Get the environment
  auto env = getEnvironment<S>();

  const std::string link_name1 = "link_n1";
  const std::string link_name2 = "link_n2";
  const std::string joint_name1 = "joint_n1";
  auto link_1 = std::make_shared<Link>(link_name1);
  auto link_2 = std::make_shared<Link>(link_name2);

  auto joint_1 = std::make_shared<Joint>(joint_name1);
  joint_1->parent_to_joint_origin_transform.translation()(0) = 1.25;
  joint_1->parent_link_name = "base_link";
  joint_1->child_link_name = link_name1;
  joint_1->type = JointType::FIXED;

  // Empty or invalid
  {
    Commands commands;
    EXPECT_TRUE(env->applyCommands(commands));
    commands.push_back(nullptr);
    EXPECT_FALSE(env->applyCommands(commands));
  }

  // Add
  {
    {
      Commands commands{ std::make_shared<AddCommand>(link_1, joint_1) };
      EXPECT_TRUE(env->applyCommands(commands));
      EXPECT_FALSE(env->applyCommands(commands));

      std::vector<std::string> link_names = env->getLinkNames();
      std::vector<std::string> joint_names = env->getJointNames();
      EXPECT_TRUE(std::find(link_names.begin(), link_names.end(), link_name1) != link_names.end());
      EXPECT_TRUE(std::find(joint_names.begin(), joint_names.end(), joint_name1) != joint_names.end());
    }
    {
      Commands commands{ std::make_shared<AddCommand>(link_2, nullptr) };
      EXPECT_TRUE(env->applyCommands(commands));
      std::vector<std::string> link_names = env->getLinkNames();
      std::vector<std::string> joint_names = env->getJointNames();
      EXPECT_TRUE(std::find(link_names.begin(), link_names.end(), link_name2) != link_names.end());
      EXPECT_TRUE(std::find(joint_names.begin(), joint_names.end(), "joint_" + link_name2) != joint_names.end());
    }
    {
      Commands commands{ std::make_shared<AddCommand>(nullptr, joint_1) };
      EXPECT_FALSE(env->applyCommands(commands));
    }
    {
      Commands commands{ std::make_shared<AddCommand>(nullptr, nullptr) };
      EXPECT_FALSE(env->applyCommands(commands));
    }
  }
  /// @todo Add tests for applying commands to the environment
}

template <typename S>
void runEnvCloneTest()
{
  // Get the environment
  auto env = getEnvironment<S>();

  // Clone the environment
  auto clone = env->clone();

  // Check the basics
  EXPECT_EQ(clone->getName(), env->getName());
  EXPECT_EQ(clone->getRevision(), env->getRevision());

  // Check that all links got cloned
  std::vector<std::string> link_names = env->getLinkNames();
  std::vector<std::string> clone_link_names = clone->getLinkNames();
  for (const auto& name : link_names)
    EXPECT_TRUE(std::find(clone_link_names.begin(), clone_link_names.end(), name) != clone_link_names.end());

  // Check that all joints got cloned
  std::vector<std::string> joint_names = env->getJointNames();
  std::vector<std::string> clone_joint_names = clone->getJointNames();
  for (const auto& name : joint_names)
    EXPECT_TRUE(std::find(clone_joint_names.begin(), clone_joint_names.end(), name) != clone_joint_names.end());

  // Check that the command history is preserved
  auto history = env->getCommandHistory();
  auto clone_history = clone->getCommandHistory();
  ASSERT_EQ(history.size(), clone_history.size());
  for (std::size_t i = 0; i < history.size(); i++)
  {
    EXPECT_EQ(history[i]->getType(), clone_history[i]->getType());
  }

  // Check active links
  std::vector<std::string> active_link_names = env->getActiveLinkNames();
  std::vector<std::string> clone_active_link_names = clone->getActiveLinkNames();
  for (const auto& name : active_link_names)
    EXPECT_TRUE(std::find(clone_active_link_names.begin(), clone_active_link_names.end(), name) !=
                clone_active_link_names.end());

  // Check active joints
  std::vector<std::string> active_joint_names = env->getActiveJointNames();
  std::vector<std::string> clone_active_joint_names = clone->getActiveJointNames();
  for (const auto& name : active_joint_names)
    EXPECT_TRUE(std::find(clone_active_joint_names.begin(), clone_active_joint_names.end(), name) !=
                clone_active_joint_names.end());

  // Check that the state is preserved
  Eigen::VectorXd joint_vals = env->getCurrentState()->getJointValues(active_joint_names);
  Eigen::VectorXd clone_joint_vals = clone->getCurrentState()->getJointValues(active_joint_names);
  EXPECT_TRUE(joint_vals.isApprox(clone_joint_vals));
}

template <typename S>
void runEnvSetStateTest()
{
  // Get the environment
  auto env = getEnvironment<S>();

  //////////////////////////////////////////////////////////////////
  // Test forward kinematics when tip link is the base of the chain
  //////////////////////////////////////////////////////////////////
  Eigen::Isometry3d pose;
  Eigen::VectorXd jvals;
  jvals.resize(7);
  jvals.setZero();
  std::vector<std::string> joint_names = { "base_link-base", "joint_a1", "joint_a2", "joint_a3",      "joint_a4",
                                           "joint_a5",       "joint_a6", "joint_a7", "joint_a7-tool0" };
  std::vector<std::string> link_names = { "base",   "base_link", "link_1", "link_2", "link_3",
                                          "link_4", "link_5",    "link_6", "link_7", "tool0" };
  std::vector<std::string> active_joint_names = { "joint_a1", "joint_a2", "joint_a3", "joint_a4",
                                                  "joint_a5", "joint_a6", "joint_a7" };

  std::vector<EnvState::Ptr> states;

  env->setState(active_joint_names, jvals);
  states.push_back(std::make_shared<EnvState>(*(env->getCurrentState())));
  states.push_back(env->getState(active_joint_names, jvals));

  for (auto& current_state : states)
  {
    // Check joints and links size
    EXPECT_EQ(current_state->joint_transforms.size(), 9);
    EXPECT_EQ(current_state->link_transforms.size(), 10);
    EXPECT_EQ(current_state->joints.size(), 7);

    // Check joints and links names
    for (const auto& joint_name : joint_names)
    {
      EXPECT_TRUE(current_state->joint_transforms.find(joint_name) != current_state->joint_transforms.end());
    }

    for (const auto& link_name : link_names)
    {
      EXPECT_TRUE(current_state->link_transforms.find(link_name) != current_state->link_transforms.end());
    }

    for (const auto& joint_name : active_joint_names)
    {
      EXPECT_TRUE(current_state->joints.find(joint_name) != current_state->joints.end());
    }

    EXPECT_TRUE(current_state->link_transforms["base_link"].isApprox(Eigen::Isometry3d::Identity()));

    {
      Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
      EXPECT_TRUE(current_state->link_transforms["link_1"].isApprox(result));
    }

    {
      Eigen::Isometry3d result = Eigen::Isometry3d::Identity() * Eigen::Translation3d(-0.00043624, 0, 0.36);
      EXPECT_TRUE(current_state->link_transforms["link_2"].isApprox(result));
    }

    {
      Eigen::Isometry3d result = Eigen::Isometry3d::Identity() * Eigen::Translation3d(-0.00043624, 0, 0.36);
      EXPECT_TRUE(current_state->link_transforms["link_3"].isApprox(result));
    }

    {
      Eigen::Isometry3d result = Eigen::Isometry3d::Identity() * Eigen::Translation3d(0, 0, 0.36 + 0.42);
      EXPECT_TRUE(current_state->link_transforms["link_4"].isApprox(result));
    }

    {
      Eigen::Isometry3d result = Eigen::Isometry3d::Identity() * Eigen::Translation3d(0, 0, 0.36 + 0.42);
      EXPECT_TRUE(current_state->link_transforms["link_5"].isApprox(result));
    }

    {
      Eigen::Isometry3d result = Eigen::Isometry3d::Identity() * Eigen::Translation3d(0, 0, 0.36 + 0.42 + 0.4);
      EXPECT_TRUE(current_state->link_transforms["link_6"].isApprox(result));
    }

    {
      Eigen::Isometry3d result = Eigen::Isometry3d::Identity() * Eigen::Translation3d(0, 0, 0.36 + 0.42 + 0.4);
      EXPECT_TRUE(current_state->link_transforms["link_7"].isApprox(result));
    }

    {
      Eigen::Isometry3d result = Eigen::Isometry3d::Identity() * Eigen::Translation3d(0, 0, 1.306);
      EXPECT_TRUE(current_state->link_transforms["tool0"].isApprox(result));
    }
  }
}

template <typename S>
void runEnvSetStateTest2()
{
  // Get the environment
  auto env = getEnvironment<S>();

  //////////////////////////////////////////////////////////////////
  // Test forward kinematics when tip link is the base of the chain
  //////////////////////////////////////////////////////////////////
  Eigen::Isometry3d pose;
  Eigen::VectorXd jvals;
  jvals.resize(7);
  jvals.setZero();
  std::vector<std::string> joint_names = { "base_link-base", "joint_a1", "joint_a2", "joint_a3",      "joint_a4",
                                           "joint_a5",       "joint_a6", "joint_a7", "joint_a7-tool0" };
  std::vector<std::string> link_names = { "base",   "base_link", "link_1", "link_2", "link_3",
                                          "link_4", "link_5",    "link_6", "link_7", "tool0" };
  std::vector<std::string> active_joint_names = { "joint_a1", "joint_a2", "joint_a3", "joint_a4",
                                                  "joint_a5", "joint_a6", "joint_a7" };

  Eigen::Vector3d axis(0, 1, 0);
  jvals(1) = M_PI_2;
  std::vector<EnvState::Ptr> states;

  env->setState(active_joint_names, jvals);
  states.push_back(std::make_shared<EnvState>(*(env->getCurrentState())));
  states.push_back(env->getState(active_joint_names, jvals));

  for (auto& current_state : states)
  {
    // Check joints and links size
    EXPECT_EQ(current_state->joint_transforms.size(), 9);
    EXPECT_EQ(current_state->link_transforms.size(), 10);
    EXPECT_EQ(current_state->joints.size(), 7);

    // Check joints and links names
    for (const auto& joint_name : joint_names)
    {
      EXPECT_TRUE(current_state->joint_transforms.find(joint_name) != current_state->joint_transforms.end());
    }

    for (const auto& link_name : link_names)
    {
      EXPECT_TRUE(current_state->link_transforms.find(link_name) != current_state->link_transforms.end());
    }

    int cnt = 0;
    for (const auto& joint_name : active_joint_names)
    {
      EXPECT_TRUE(current_state->joints.find(joint_name) != current_state->joints.end());
      EXPECT_NEAR(current_state->joints[joint_name], jvals(cnt++), 1e-5);
    }

    EXPECT_TRUE(current_state->link_transforms["base_link"].isApprox(Eigen::Isometry3d::Identity()));
    EXPECT_TRUE(current_state->link_transforms["base"].isApprox(Eigen::Isometry3d::Identity()));

    {
      Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
      EXPECT_TRUE(current_state->link_transforms["link_1"].isApprox(result));
    }

    {
      Eigen::Isometry3d result = Eigen::Translation3d(-0.00043624, 0, 0.36) * Eigen::AngleAxisd(M_PI_2, axis);
      EXPECT_TRUE(current_state->link_transforms["link_2"].isApprox(result, 1e-4));
    }

    {
      Eigen::Isometry3d result = Eigen::Translation3d(-0.00043624, 0, 0.36) * Eigen::AngleAxisd(M_PI_2, axis);
      EXPECT_TRUE(current_state->link_transforms["link_3"].isApprox(result, 1e-4));
    }

    {
      Eigen::Isometry3d result =
          Eigen::Translation3d(0.42 - 0.00043624, 0, 0.36 - 0.00043624) * Eigen::AngleAxisd(M_PI_2, axis);
      EXPECT_TRUE(current_state->link_transforms["link_4"].isApprox(result, 1e-4));
    }

    {
      Eigen::Isometry3d result =
          Eigen::Translation3d(0.42 - 0.00043624, 0, 0.36 - 0.00043624) * Eigen::AngleAxisd(M_PI_2, axis);
      EXPECT_TRUE(current_state->link_transforms["link_5"].isApprox(result, 1e-4));
    }

    {
      Eigen::Isometry3d result =
          Eigen::Translation3d(0.42 + 0.4 - 0.00043624, 0, 0.36 - 0.00043624) * Eigen::AngleAxisd(M_PI_2, axis);
      EXPECT_TRUE(current_state->link_transforms["link_6"].isApprox(result, 1e-4));
    }

    {
      Eigen::Isometry3d result =
          Eigen::Translation3d(0.42 + 0.4 - 0.00043624, 0, 0.36 - 0.00043624) * Eigen::AngleAxisd(M_PI_2, axis);
      EXPECT_TRUE(current_state->link_transforms["link_7"].isApprox(result, 1e-4));
    }

    {
      Eigen::Isometry3d result =
          Eigen::Translation3d(1.306 - 0.36 - 0.00043624, 0, 0.36 - 0.00043624) * Eigen::AngleAxisd(M_PI_2, axis);
      EXPECT_TRUE(current_state->link_transforms["tool0"].isApprox(result, 1e-4));
    }
  }
}

TEST(TesseractEnvironmentUnit, EnvCloneContactManagerUnit)  // NOLINT
{
  runContactManagerCloneTest<KDLStateSolver>();
  runContactManagerCloneTest<OFKTStateSolver>();
}

TEST(TesseractEnvironmentUnit, EnvAddandRemoveLink)  // NOLINT
{
  runAddandRemoveLinkTest<KDLStateSolver>();
  runAddandRemoveLinkTest<OFKTStateSolver>();
}

TEST(TesseractEnvironmentUnit, EnvMoveLinkandJoint)  // NOLINT
{
  runMoveLinkandJointTest<KDLStateSolver>();
  runMoveLinkandJointTest<OFKTStateSolver>();
}

TEST(TesseractEnvironmentUnit, EnvChangeJointOrigin)  // NOLINT
{
  runChangeJointOriginTest<KDLStateSolver>();
  runChangeJointOriginTest<OFKTStateSolver>();
}

TEST(TesseractEnvironmentUnit, EnvCurrentStatePreservedWhenEnvChanges)  // NOLINT
{
  runCurrentStatePreservedWhenEnvChangesTest<KDLStateSolver>();
  runCurrentStatePreservedWhenEnvChangesTest<OFKTStateSolver>();
}

TEST(TesseractEnvironmentUnit, EnvAddSceneGraph)
{
  runAddSceneGraphTest<KDLStateSolver>();
  runAddSceneGraphTest<OFKTStateSolver>();
}

TEST(TesseractEnvironmentUnit, EnvApplyCommands)  // NOLINT
{
  runApplyCommandsTest<KDLStateSolver>();
  runApplyCommandsTest<OFKTStateSolver>();
}

TEST(TesseractEnvironmentUnit, EnvClone)  // NOLINT
{
  runEnvCloneTest<KDLStateSolver>();
  runEnvCloneTest<OFKTStateSolver>();
}

TEST(TesseractEnvironmentUnit, EnvSetState)  // NOLINT
{
  runEnvSetStateTest<KDLStateSolver>();
  runEnvSetStateTest<OFKTStateSolver>();

  runEnvSetStateTest2<KDLStateSolver>();
  runEnvSetStateTest2<OFKTStateSolver>();
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
