#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <algorithm>
#include <tesseract_urdf/urdf_parser.h>
#include <tesseract_collision/bullet/bullet_discrete_bvh_manager.h>
#include <tesseract_collision/bullet/bullet_cast_bvh_manager.h>
#include <tesseract_scene_graph/resource_locator.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/core/types.h>
#include "tesseract_environment/kdl/kdl_env.h"

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

void runContactManagerCloneTest(const tesseract_environment::Environment::Ptr& env)
{
  // Test after clone if active list correct
  tesseract_collision::DiscreteContactManager::Ptr discrete_manager = env->getDiscreteContactManager();
  const std::vector<std::string>& e_active_list = env->getActiveLinkNames();
  const std::vector<std::string>& d_active_list = discrete_manager->getActiveCollisionObjects();
  EXPECT_TRUE(std::equal(e_active_list.begin(), e_active_list.end(), d_active_list.begin()));

  tesseract_collision::ContinuousContactManager::Ptr cast_manager = env->getContinuousContactManager();
  const std::vector<std::string>& c_active_list = cast_manager->getActiveCollisionObjects();
  EXPECT_TRUE(std::equal(e_active_list.begin(), e_active_list.end(), c_active_list.begin()));
}

void runAddandRemoveLinkTest(const tesseract_environment::Environment::Ptr& env)
{
  Link link_1("link_n1");
  Link link_2("link_n2");

  Joint joint_1("joint_n1");
  joint_1.parent_to_joint_origin_transform.translation()(0) = 1.25;
  joint_1.parent_link_name = "link_n1";
  joint_1.child_link_name = "link_n2";
  joint_1.type = JointType::FIXED;

  env->addLink(link_1);

  std::vector<std::string> link_names = env->getLinkNames();
  std::vector<std::string> joint_names = env->getJointNames();
  EXPECT_TRUE(std::find(link_names.begin(), link_names.end(), link_1.getName()) != link_names.end());
  EXPECT_TRUE(std::find(joint_names.begin(), joint_names.end(), "joint_" + link_1.getName()) != joint_names.end());

  env->addLink(link_2, joint_1);
  link_names = env->getLinkNames();
  joint_names = env->getJointNames();
  EXPECT_TRUE(std::find(link_names.begin(), link_names.end(), link_2.getName()) != link_names.end());
  EXPECT_TRUE(std::find(joint_names.begin(), joint_names.end(), joint_1.getName()) != joint_names.end());

  env->getSceneGraph()->saveDOT("/tmp/before_remove_link_unit.dot");

  env->removeLink(link_1.getName());
  link_names = env->getLinkNames();
  joint_names = env->getJointNames();
  EXPECT_TRUE(std::find(link_names.begin(), link_names.end(), link_1.getName()) == link_names.end());
  EXPECT_TRUE(std::find(joint_names.begin(), joint_names.end(), "joint_" + link_1.getName()) == joint_names.end());
  EXPECT_TRUE(std::find(link_names.begin(), link_names.end(), link_2.getName()) == link_names.end());
  EXPECT_TRUE(std::find(joint_names.begin(), joint_names.end(), joint_1.getName()) == joint_names.end());

  env->getSceneGraph()->saveDOT("/tmp/after_remove_link_unit.dot");

  // Test against double removing
  EXPECT_FALSE(env->removeLink(link_1.getName()));
  EXPECT_FALSE(env->removeLink(link_2.getName()));
  EXPECT_FALSE(env->removeJoint(joint_1.getName()));
  EXPECT_FALSE(env->removeJoint("joint_" + link_1.getName()));
}

void runMoveLinkandJointTest(const tesseract_environment::Environment::Ptr& env)
{
  Link link_1("link_n1");
  Link link_2("link_n2");

  Joint joint_1("joint_n1");
  joint_1.parent_link_name = env->getRootLinkName();
  joint_1.child_link_name = "link_n1";
  joint_1.type = JointType::FIXED;

  Joint joint_2("joint_n2");
  joint_2.parent_to_joint_origin_transform.translation()(0) = 1.25;
  joint_2.parent_link_name = "link_n1";
  joint_2.child_link_name = "link_n2";
  joint_2.type = JointType::FIXED;

  env->addLink(link_1, joint_1);
  EnvState::ConstPtr state = env->getCurrentState();
  EXPECT_TRUE(state->transforms.find(link_1.getName()) != state->transforms.end());

  env->addLink(link_2, joint_2);
  std::vector<std::string> link_names = env->getLinkNames();
  std::vector<std::string> joint_names = env->getJointNames();
  EXPECT_TRUE(std::find(link_names.begin(), link_names.end(), link_1.getName()) != link_names.end());
  EXPECT_TRUE(std::find(link_names.begin(), link_names.end(), link_2.getName()) != link_names.end());
  EXPECT_TRUE(std::find(joint_names.begin(), joint_names.end(), joint_1.getName()) != joint_names.end());
  EXPECT_TRUE(std::find(joint_names.begin(), joint_names.end(), joint_2.getName()) != joint_names.end());

  env->getSceneGraph()->saveDOT("/tmp/before_move_joint_unit.dot");

  env->moveJoint("joint_n1", "tool0");
  link_names = env->getLinkNames();
  joint_names = env->getJointNames();
  EXPECT_TRUE(env->getJoint("joint_n1")->parent_link_name == "tool0");
  EXPECT_TRUE(std::find(link_names.begin(), link_names.end(), link_1.getName()) != link_names.end());
  EXPECT_TRUE(std::find(link_names.begin(), link_names.end(), link_2.getName()) != link_names.end());
  EXPECT_TRUE(std::find(joint_names.begin(), joint_names.end(), joint_1.getName()) != joint_names.end());
  EXPECT_TRUE(std::find(joint_names.begin(), joint_names.end(), joint_2.getName()) != joint_names.end());

  env->getSceneGraph()->saveDOT("/tmp/after_move_joint_unit.dot");
}

void runChangeJointOriginTest(const tesseract_environment::Environment::Ptr& env)
{
  Link link_1("link_n1");

  Joint joint_1("joint_n1");
  joint_1.parent_link_name = env->getRootLinkName();
  joint_1.child_link_name = "link_n1";
  joint_1.type = JointType::FIXED;

  env->addLink(link_1, joint_1);
  EnvState::ConstPtr state = env->getCurrentState();
  ASSERT_TRUE(state->transforms.find(link_1.getName()) != state->transforms.end());

  env->getSceneGraph()->saveDOT("/tmp/before_change_joint_origin_unit.dot");

  Eigen::Isometry3d new_origin = Eigen::Isometry3d::Identity();
  new_origin.translation()(0) += 1.234;
  env->changeJointOrigin("joint_n1", new_origin);

  // Check that the origin got updated
  EXPECT_TRUE(env->getJoint("joint_n1")->parent_to_joint_origin_transform.isApprox(new_origin));

  env->getSceneGraph()->saveDOT("/tmp/after_change_joint_origin_unit.dot");
}

void runCurrentStatePreservedWhenEnvChangesTest(const tesseract_environment::Environment::Ptr& env)
{
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

  env->addLink(link, joint);

  current_state = env->getCurrentState();
  for (auto& joint_state : joint_states)
  {
    EXPECT_NEAR(current_state->joints.at(joint_state.first), joint_state.second, 1e-5);
  }
}

TEST(TesseractEnvironmentUnit, KDLEnvCloneContactManagerUnit)  // NOLINT
{
  tesseract_scene_graph::SceneGraph::Ptr scene_graph = getSceneGraph();
  EXPECT_TRUE(scene_graph != nullptr);

  tesseract_environment::KDLEnv::Ptr env(new tesseract_environment::KDLEnv);
  EXPECT_TRUE(env != nullptr);

  bool success = env->init(scene_graph);
  EXPECT_TRUE(success);

  // Register contact manager
  EXPECT_TRUE(env->registerDiscreteContactManager(tesseract_collision_bullet::BulletDiscreteBVHManager::name(),
                                                  &tesseract_collision_bullet::BulletDiscreteBVHManager::create));
  EXPECT_TRUE(env->registerContinuousContactManager(tesseract_collision_bullet::BulletCastBVHManager::name(),
                                                    &tesseract_collision_bullet::BulletCastBVHManager::create));

  // Set Active contact manager
  EXPECT_TRUE(env->setActiveDiscreteContactManager(tesseract_collision_bullet::BulletDiscreteBVHManager::name()));
  EXPECT_TRUE(env->setActiveContinuousContactManager(tesseract_collision_bullet::BulletCastBVHManager::name()));

  runContactManagerCloneTest(env);
}

TEST(TesseractEnvironmentUnit, KDLEnvAddandRemoveLink)  // NOLINT
{
  SceneGraph::Ptr scene_graph = getSceneGraph();
  EXPECT_TRUE(scene_graph != nullptr);

  KDLEnv::Ptr env(new KDLEnv());
  EXPECT_TRUE(env != nullptr);

  bool success = env->init(scene_graph);
  EXPECT_TRUE(success);

  // Register contact manager
  EXPECT_TRUE(env->registerDiscreteContactManager(tesseract_collision_bullet::BulletDiscreteBVHManager::name(),
                                                  &tesseract_collision_bullet::BulletDiscreteBVHManager::create));
  EXPECT_TRUE(env->registerContinuousContactManager(tesseract_collision_bullet::BulletCastBVHManager::name(),
                                                    &tesseract_collision_bullet::BulletCastBVHManager::create));

  // Set Active contact manager
  EXPECT_TRUE(env->setActiveDiscreteContactManager(tesseract_collision_bullet::BulletDiscreteBVHManager::name()));
  EXPECT_TRUE(env->setActiveContinuousContactManager(tesseract_collision_bullet::BulletCastBVHManager::name()));

  runAddandRemoveLinkTest(env);
}

TEST(TesseractEnvironmentUnit, KDLEnvMoveLinkandJoint)  // NOLINT
{
  SceneGraph::Ptr scene_graph = getSceneGraph();
  EXPECT_TRUE(scene_graph != nullptr);

  KDLEnv::Ptr env(new KDLEnv());
  EXPECT_TRUE(env != nullptr);

  bool success = env->init(scene_graph);
  EXPECT_TRUE(success);

  // Register contact manager
  EXPECT_TRUE(env->registerDiscreteContactManager(tesseract_collision_bullet::BulletDiscreteBVHManager::name(),
                                                  &tesseract_collision_bullet::BulletDiscreteBVHManager::create));
  EXPECT_TRUE(env->registerContinuousContactManager(tesseract_collision_bullet::BulletCastBVHManager::name(),
                                                    &tesseract_collision_bullet::BulletCastBVHManager::create));

  // Set Active contact manager
  EXPECT_TRUE(env->setActiveDiscreteContactManager(tesseract_collision_bullet::BulletDiscreteBVHManager::name()));
  EXPECT_TRUE(env->setActiveContinuousContactManager(tesseract_collision_bullet::BulletCastBVHManager::name()));

  runMoveLinkandJointTest(env);
}

TEST(TesseractEnvironmentUnit, KDLEnvChangeJointOrigin)  // NOLINT
{
  SceneGraph::Ptr scene_graph = getSceneGraph();
  EXPECT_TRUE(scene_graph != nullptr);

  KDLEnv::Ptr env(new KDLEnv());
  EXPECT_TRUE(env != nullptr);

  bool success = env->init(scene_graph);
  EXPECT_TRUE(success);

  // Register contact manager
  EXPECT_TRUE(env->registerDiscreteContactManager(tesseract_collision_bullet::BulletDiscreteBVHManager::name(),
                                                  &tesseract_collision_bullet::BulletDiscreteBVHManager::create));
  EXPECT_TRUE(env->registerContinuousContactManager(tesseract_collision_bullet::BulletCastBVHManager::name(),
                                                    &tesseract_collision_bullet::BulletCastBVHManager::create));

  // Set Active contact manager
  EXPECT_TRUE(env->setActiveDiscreteContactManager(tesseract_collision_bullet::BulletDiscreteBVHManager::name()));
  EXPECT_TRUE(env->setActiveContinuousContactManager(tesseract_collision_bullet::BulletCastBVHManager::name()));

  runChangeJointOriginTest(env);
}

TEST(TesseractEnvironmentUnit, KDLEnvCurrentStatePreservedWhenEnvChanges)  // NOLINT
{
  SceneGraph::Ptr scene_graph = getSceneGraph();
  EXPECT_TRUE(scene_graph != nullptr);

  KDLEnv::Ptr env(new KDLEnv());
  EXPECT_TRUE(env != nullptr);

  bool success = env->init(scene_graph);
  EXPECT_TRUE(success);

  runCurrentStatePreservedWhenEnvChangesTest(env);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
