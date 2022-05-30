#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <algorithm>
#include <vector>
#include <tesseract_urdf/urdf_parser.h>
#include <tesseract_common/resource_locator.h>
#include <tesseract_geometry/impl/box.h>
#include <tesseract_common/utils.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/commands.h>
#include <tesseract_environment/environment.h>
#include <tesseract_support/tesseract_support_resource_locator.h>

using namespace tesseract_scene_graph;
using namespace tesseract_srdf;
using namespace tesseract_collision;
using namespace tesseract_environment;

SceneGraph::UPtr getSceneGraph()
{
  std::string path = std::string(TESSERACT_SUPPORT_DIR) + "/urdf/lbr_iiwa_14_r820.urdf";

  tesseract_common::TesseractSupportResourceLocator locator;
  return tesseract_urdf::parseURDFFile(path, locator);
}

SRDFModel::Ptr getSRDFModel(const SceneGraph& scene_graph)
{
  std::string path = std::string(TESSERACT_SUPPORT_DIR) + "/urdf/lbr_iiwa_14_r820.srdf";
  tesseract_common::TesseractSupportResourceLocator locator;

  auto srdf = std::make_unique<SRDFModel>();
  srdf->initFile(scene_graph, path, locator);

  return srdf;
}

tesseract_environment::Environment::UPtr getEnvironment()
{
  auto scene_graph = getSceneGraph();
  EXPECT_TRUE(scene_graph != nullptr);

  auto srdf = getSRDFModel(*scene_graph);
  EXPECT_TRUE(srdf != nullptr);

  auto env = std::make_unique<Environment>();
  bool success = env->init(*scene_graph, srdf);
  EXPECT_TRUE(success);

  Link link_1("link_n1");
  {
    Visual::Ptr v = std::make_shared<Visual>();
    v->origin.translation() = Eigen::Vector3d(0, 0, 0);
    v->material = std::make_shared<Material>("test_material");
    v->material->color = Eigen::Vector4d(1, 0, 0, 1);
    v->geometry = std::make_shared<tesseract_geometry::Box>(1, 1, 1);
    v->name = "link1_visual";
    link_1.visual.push_back(v);

    Collision::Ptr c = std::make_shared<Collision>();
    c->origin.translation() = v->origin.translation();
    c->geometry = v->geometry;
    c->name = "link1_collision";
    link_1.collision.push_back(c);
  }

  Link link_2("link_n2");
  {
    Visual::Ptr v = std::make_shared<Visual>();
    v->origin.translation() = Eigen::Vector3d(1.5, 0, 0);
    v->material = std::make_shared<Material>("test_material");
    v->material->color = Eigen::Vector4d(1, 1, 1, 1);
    v->geometry = std::make_shared<tesseract_geometry::Box>(1, 1, 1);
    v->name = "link2_visual";
    link_2.visual.push_back(v);

    Collision::Ptr c = std::make_shared<Collision>();
    c->origin.translation() = v->origin.translation();
    c->geometry = v->geometry;
    c->name = "link2_collision";
    link_2.collision.push_back(c);
  }

  auto cmd1 = std::make_shared<AddLinkCommand>(link_1, true);
  env->applyCommand(cmd1);

  auto cmd2 = std::make_shared<AddLinkCommand>(link_2, true);
  env->applyCommand(cmd2);

  return env;
}

TEST(TesseractEnvironmentCollisionUnit, runEnvironmentDiscreteCollisionTest)  // NOLINT
{
  // Get the environment
  auto env = getEnvironment();

  // Setup collision margin data
  CollisionCheckConfig mCollisionCheckConfig;
  mCollisionCheckConfig.longest_valid_segment_length = 0.1;
  mCollisionCheckConfig.contact_request.type = tesseract_collision::ContactTestType::FIRST;
  mCollisionCheckConfig.type = tesseract_collision::CollisionEvaluatorType::DISCRETE;
  tesseract_collision::CollisionMarginData margin_data(0.0);
  mCollisionCheckConfig.contact_manager_config.margin_data = margin_data;

  // Setup collision checker
  DiscreteContactManager::Ptr manager = env->getDiscreteContactManager();
  {  // Check for collisions
    tesseract_collision::ContactResultMap collision;
    std::vector<std::string> active_links = { "link_n1" };
    manager->setActiveCollisionObjects(active_links);
    manager->contactTest(collision, mCollisionCheckConfig.contact_request);
    EXPECT_FALSE(collision.empty());
  }

  auto pose = Eigen::Isometry3d::Identity();
  pose.translation() = Eigen::Vector3d(0, 0, 1.5);
  manager->setCollisionObjectsTransform("link_n1", pose);
  manager->setCollisionMarginData(mCollisionCheckConfig.contact_manager_config.margin_data);

  // Check for collisions
  tesseract_collision::ContactResultMap collision;
  manager->contactTest(collision, mCollisionCheckConfig.contact_request);

  EXPECT_FALSE(collision.empty());
}

TEST(TesseractEnvironmentCollisionUnit, runEnvironmentContinuousCollisionTest)  // NOLINT
{
  // Get the environment
  auto env = getEnvironment();

  // Setup collision margin data
  CollisionCheckConfig mCollisionCheckConfig;
  mCollisionCheckConfig.longest_valid_segment_length = 0.1;
  mCollisionCheckConfig.contact_request.type = tesseract_collision::ContactTestType::FIRST;
  mCollisionCheckConfig.type = tesseract_collision::CollisionEvaluatorType::CONTINUOUS;
  tesseract_collision::CollisionMarginData margin_data(0.0);
  mCollisionCheckConfig.contact_manager_config.margin_data = margin_data;

  // Setup collision checker
  ContinuousContactManager::Ptr manager = env->getContinuousContactManager();
  {  // Check for collisions
    tesseract_collision::ContactResultMap collision;
    std::vector<std::string> active_links = { "link_n1" };
    manager->setActiveCollisionObjects(active_links);
    manager->contactTest(collision, mCollisionCheckConfig.contact_request);
    EXPECT_FALSE(collision.empty());
  }

  auto pose1 = Eigen::Isometry3d::Identity();
  pose1.translation() = Eigen::Vector3d(0, -2, 1.5);
  auto pose2 = Eigen::Isometry3d::Identity();
  pose2.translation() = Eigen::Vector3d(0, 2, 1.5);
  manager->setCollisionObjectsTransform("link_n1", pose1, pose2);
  manager->setCollisionMarginData(mCollisionCheckConfig.contact_manager_config.margin_data);

  // Check for collisions
  tesseract_collision::ContactResultMap collision;
  manager->contactTest(collision, mCollisionCheckConfig.contact_request);

  EXPECT_FALSE(collision.empty());
}

TEST(TesseractEnvironmentCollisionUnit, runEnvironmentClearDiscreteCollisionTest)  // NOLINT
{
  // Get the environment
  auto env = getEnvironment();
  env->clearCachedDiscreteContactManager();

  // Setup collision margin data
  CollisionCheckConfig mCollisionCheckConfig;
  mCollisionCheckConfig.longest_valid_segment_length = 0.1;
  mCollisionCheckConfig.contact_request.type = tesseract_collision::ContactTestType::FIRST;
  mCollisionCheckConfig.type = tesseract_collision::CollisionEvaluatorType::DISCRETE;
  tesseract_collision::CollisionMarginData margin_data(0.0);
  mCollisionCheckConfig.contact_manager_config.margin_data = margin_data;

  // Setup collision checker
  DiscreteContactManager::Ptr manager = env->getDiscreteContactManager();
  {  // Check for collisions
    tesseract_collision::ContactResultMap collision;
    std::vector<std::string> active_links = { "link_n1" };
    manager->setActiveCollisionObjects(active_links);
    manager->contactTest(collision, mCollisionCheckConfig.contact_request);
    EXPECT_FALSE(collision.empty());
  }

  auto pose = Eigen::Isometry3d::Identity();
  pose.translation() = Eigen::Vector3d(0, 0, 1.5);
  manager->setCollisionObjectsTransform("link_n1", pose);
  manager->setCollisionMarginData(mCollisionCheckConfig.contact_manager_config.margin_data);

  // Check for collisions
  tesseract_collision::ContactResultMap collision;
  manager->contactTest(collision, mCollisionCheckConfig.contact_request);

  EXPECT_FALSE(collision.empty());
}

TEST(TesseractEnvironmentCollisionUnit, runEnvironmentClearContinuousCollisionTest)  // NOLINT
{
  // Get the environment
  auto env = getEnvironment();
  env->clearCachedContinuousContactManager();

  // Setup collision margin data
  CollisionCheckConfig mCollisionCheckConfig;
  mCollisionCheckConfig.longest_valid_segment_length = 0.1;
  mCollisionCheckConfig.contact_request.type = tesseract_collision::ContactTestType::FIRST;
  mCollisionCheckConfig.type = tesseract_collision::CollisionEvaluatorType::CONTINUOUS;
  tesseract_collision::CollisionMarginData margin_data(0.0);
  mCollisionCheckConfig.contact_manager_config.margin_data = margin_data;

  // Setup collision checker
  ContinuousContactManager::Ptr manager = env->getContinuousContactManager();
  {  // Check for collisions
    tesseract_collision::ContactResultMap collision;
    std::vector<std::string> active_links = { "link_n1" };
    manager->setActiveCollisionObjects(active_links);
    manager->contactTest(collision, mCollisionCheckConfig.contact_request);
    EXPECT_FALSE(collision.empty());
  }

  auto pose1 = Eigen::Isometry3d::Identity();
  pose1.translation() = Eigen::Vector3d(0, -2, 1.5);
  auto pose2 = Eigen::Isometry3d::Identity();
  pose2.translation() = Eigen::Vector3d(0, 2, 1.5);
  manager->setCollisionObjectsTransform("link_n1", pose1, pose2);
  manager->setCollisionMarginData(mCollisionCheckConfig.contact_manager_config.margin_data);

  // Check for collisions
  tesseract_collision::ContactResultMap collision;
  manager->contactTest(collision, mCollisionCheckConfig.contact_request);

  EXPECT_FALSE(collision.empty());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
