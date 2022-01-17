#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <algorithm>
#include <vector>
#include <tesseract_urdf/urdf_parser.h>
#include <tesseract_common/resource_locator.h>
#include <tesseract_common/utils.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/environment.h>
#include <tesseract_environment/utils.h>

using namespace tesseract_scene_graph;
using namespace tesseract_srdf;
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

SceneGraph::UPtr getSceneGraph()
{
  std::string path = std::string(TESSERACT_SUPPORT_DIR) + "/urdf/boxbot.urdf";

  tesseract_common::SimpleResourceLocator locator(locateResource);
  return tesseract_urdf::parseURDFFile(path, locator);
}

SRDFModel::Ptr getSRDFModel(const SceneGraph& scene_graph)
{
  std::string path = std::string(TESSERACT_SUPPORT_DIR) + "/urdf/boxbot.srdf";
  tesseract_common::SimpleResourceLocator locator(locateResource);

  auto srdf = std::make_unique<SRDFModel>();
  srdf->initFile(scene_graph, path, locator);

  return srdf;
}

template <typename ManagerType>
void checkIsAllowedFnOverride(std::unique_ptr<ManagerType> manager)
{
  CollisionCheckConfig config;

  // ASSIGN
  {
    config.contact_manager_config.acm.addAllowedCollision("allowed_link_1a", "allowed_link_2a", "Unit test");
    config.contact_manager_config.acm_override_type = ACMOverrideType::ASSIGN;
    manager->applyContactManagerConfig(config.contact_manager_config);
    auto fn = manager->getIsContactAllowedFn();
    EXPECT_TRUE(fn("allowed_link_1a", "allowed_link_2a"));
  }

  // NONE
  {
    // Manager currently allows: a
    config.contact_manager_config.acm.addAllowedCollision("allowed_link_1b", "allowed_link_2b", "Unit test");
    config.contact_manager_config.acm_override_type = ACMOverrideType::NONE;
    manager->applyContactManagerConfig(config.contact_manager_config);
    auto fn = manager->getIsContactAllowedFn();
    EXPECT_FALSE(fn("allowed_link_1b", "allowed_link_2b"));
  }

  // OR
  {
    // Manager currently allows: a
    config.contact_manager_config.acm.addAllowedCollision("allowed_link_1c", "allowed_link_2c", "Unit test");
    config.contact_manager_config.acm_override_type = ACMOverrideType::OR;
    manager->applyContactManagerConfig(config.contact_manager_config);
    auto fn = manager->getIsContactAllowedFn();
    EXPECT_TRUE(fn("allowed_link_1a", "allowed_link_2a"));
    EXPECT_TRUE(fn("allowed_link_1c", "allowed_link_2c"));
  }

  // AND
  {
    // Manager currently allows: a, c
    config.contact_manager_config.acm.removeAllowedCollision("allowed_link_1a", "allowed_link_2a");
    config.contact_manager_config.acm_override_type = ACMOverrideType::AND;
    manager->applyContactManagerConfig(config.contact_manager_config);
    auto fn = manager->getIsContactAllowedFn();
    EXPECT_FALSE(fn("allowed_link_1a", "allowed_link_2a"));
    EXPECT_TRUE(fn("allowed_link_1c", "allowed_link_2c"));
  }
}

TEST(TesseractEnvironmentUtils, applyContactManagerConfigIsAllowed)  // NOLINT
{
  auto scene_graph = getSceneGraph();
  EXPECT_TRUE(scene_graph != nullptr);

  auto srdf = getSRDFModel(*scene_graph);
  EXPECT_TRUE(srdf != nullptr);

  auto env = std::make_shared<Environment>();
  bool success = env->init(*scene_graph, srdf);
  EXPECT_TRUE(success);

  // Setup CollisionCheckConfig
  CollisionCheckConfig mCollisionCheckConfig;
  mCollisionCheckConfig.longest_valid_segment_length = 0.1;
  mCollisionCheckConfig.contact_request.type = tesseract_collision::ContactTestType::FIRST;
  mCollisionCheckConfig.type = tesseract_collision::CollisionEvaluatorType::DISCRETE;
  tesseract_collision::CollisionMarginData margin_data(0.0);
  mCollisionCheckConfig.contact_manager_config.margin_data = margin_data;

  checkIsAllowedFnOverride<DiscreteContactManager>(env->getDiscreteContactManager());
  checkIsAllowedFnOverride<ContinuousContactManager>(env->getContinuousContactManager());
}

TEST(TesseractEnvironmentUtils, applyContactManagerConfigObjectEnable)  // NOLINT
{
  auto scene_graph = getSceneGraph();
  EXPECT_TRUE(scene_graph != nullptr);

  auto srdf = getSRDFModel(*scene_graph);
  EXPECT_TRUE(srdf != nullptr);

  auto env = std::make_shared<Environment>();
  bool success = env->init(*scene_graph, srdf);
  EXPECT_TRUE(success);

  // Setup CollisionCheckConfig
  CollisionCheckConfig default_config;
  default_config.longest_valid_segment_length = 0.1;
  default_config.contact_request.type = tesseract_collision::ContactTestType::FIRST;
  default_config.type = tesseract_collision::CollisionEvaluatorType::DISCRETE;
  tesseract_collision::CollisionMarginData margin_data(0.0);
  default_config.contact_manager_config.margin_data = margin_data;
  default_config.contact_manager_config.margin_data_override_type =
      tesseract_common::CollisionMarginOverrideType::REPLACE;

  // Check Discrete
  {
    auto config = default_config;
    DiscreteContactManager::Ptr manager = env->getDiscreteContactManager();
    std::vector<std::string> active_links = { "boxbot_link", "test_box_link" };
    manager->setActiveCollisionObjects(active_links);

    // Put the boxes 0.1m in collision
    tesseract_common::TransformMap tmap;
    tmap["boxbot_link"] = Eigen::Isometry3d::Identity();
    tmap["test_box_link"] = Eigen::Isometry3d::Identity();
    tmap["test_box_link"].translate(Eigen::Vector3d(0.9, 0, 0));

    // In collision by default
    {
      tesseract_collision::ContactResultMap contacts = checkTrajectoryState(*manager, tmap, config);
      EXPECT_FALSE(contacts.empty());
    }

    // Not in collision if link disabled
    {
      config.contact_manager_config.modify_object_enabled["boxbot_link"] = false;
      manager->applyContactManagerConfig(config.contact_manager_config);
      tesseract_collision::ContactResultMap contacts = checkTrajectoryState(*manager, tmap, config);
      EXPECT_TRUE(contacts.empty());
    }

    // Re-enable it. Now in collision again
    {
      config.contact_manager_config.modify_object_enabled["boxbot_link"] = true;
      manager->applyContactManagerConfig(config.contact_manager_config);
      tesseract_collision::ContactResultMap contacts = checkTrajectoryState(*manager, tmap, config);
      EXPECT_FALSE(contacts.empty());
    }

    // Disable a link that doesn't exist. Still in collision
    {
      config.contact_manager_config.modify_object_enabled["nonexistant_link"] = false;
      manager->applyContactManagerConfig(config.contact_manager_config);
      tesseract_collision::ContactResultMap contacts = checkTrajectoryState(*manager, tmap, config);
      EXPECT_FALSE(contacts.empty());
    }
  }

  // Check Continuous
  {
    auto config = default_config;
    ContinuousContactManager::Ptr manager = env->getContinuousContactManager();
    std::vector<std::string> active_links = { "boxbot_link", "test_box_link" };
    manager->setActiveCollisionObjects(active_links);

    // Put the swept volume of the boxes 0.1m in collision
    tesseract_common::TransformMap tmap1;
    tmap1["boxbot_link"] = Eigen::Isometry3d::Identity();
    tmap1["test_box_link"] = Eigen::Isometry3d::Identity();
    tmap1["test_box_link"].translate(Eigen::Vector3d(0.9, 2, 0));

    tesseract_common::TransformMap tmap2;
    tmap2["boxbot_link"] = Eigen::Isometry3d::Identity();
    tmap2["test_box_link"] = Eigen::Isometry3d::Identity();
    tmap2["test_box_link"].translate(Eigen::Vector3d(0.9, -2, 0));

    {
      tesseract_collision::ContactResultMap contacts =
          checkTrajectorySegment(*manager, tmap1, tmap2, config.contact_request);
      // In collision by default
      EXPECT_FALSE(contacts.empty());
    }

    // Not in collision if link disabled
    {
      config.contact_manager_config.modify_object_enabled["boxbot_link"] = false;
      manager->applyContactManagerConfig(config.contact_manager_config);
      tesseract_collision::ContactResultMap contacts =
          checkTrajectorySegment(*manager, tmap1, tmap2, config.contact_request);
      EXPECT_TRUE(contacts.empty());
    }

    // Re-enable it. Now in collision again
    {
      config.contact_manager_config.modify_object_enabled["boxbot_link"] = true;
      manager->applyContactManagerConfig(config.contact_manager_config);
      tesseract_collision::ContactResultMap contacts =
          checkTrajectorySegment(*manager, tmap1, tmap2, config.contact_request);
      EXPECT_FALSE(contacts.empty());
    }

    // Disable a link that doesn't exist. Still in collision
    {
      config.contact_manager_config.modify_object_enabled["nonexistant_link"] = false;
      manager->applyContactManagerConfig(config.contact_manager_config);
      tesseract_collision::ContactResultMap contacts =
          checkTrajectorySegment(*manager, tmap1, tmap2, config.contact_request);
      EXPECT_FALSE(contacts.empty());
    }
  }
}

TEST(TesseractEnvironmentUtils, checkTrajectoryState)  // NOLINT
{
  auto scene_graph = getSceneGraph();
  EXPECT_TRUE(scene_graph != nullptr);

  auto srdf = getSRDFModel(*scene_graph);
  EXPECT_TRUE(srdf != nullptr);

  auto env = std::make_shared<Environment>();
  bool success = env->init(*scene_graph, srdf);
  EXPECT_TRUE(success);

  // Setup CollisionCheckConfig
  CollisionCheckConfig default_config;
  default_config.longest_valid_segment_length = 0.1;
  default_config.contact_request.type = tesseract_collision::ContactTestType::FIRST;
  default_config.type = tesseract_collision::CollisionEvaluatorType::DISCRETE;
  tesseract_collision::CollisionMarginData margin_data(0.0);
  default_config.contact_manager_config.margin_data = margin_data;
  default_config.contact_manager_config.margin_data_override_type =
      tesseract_common::CollisionMarginOverrideType::REPLACE;

  // Check Discrete
  {
    auto config = default_config;
    DiscreteContactManager::Ptr manager = env->getDiscreteContactManager();
    std::vector<std::string> active_links = { "boxbot_link", "test_box_link" };
    manager->setActiveCollisionObjects(active_links);

    // Put the boxes 0.05m away from each other
    tesseract_common::TransformMap tmap;
    tmap["boxbot_link"] = Eigen::Isometry3d::Identity();
    tmap["test_box_link"] = Eigen::Isometry3d::Identity();
    tmap["test_box_link"].translate(Eigen::Vector3d(1.05, 0, 0));

    // Not in collision
    {
      manager->applyContactManagerConfig(config.contact_manager_config);
      tesseract_collision::ContactResultMap contacts = checkTrajectoryState(*manager, tmap, config.contact_request);
      EXPECT_TRUE(contacts.empty());
    }
    // In collision if manager->applyContactManagerConfig works correctly
    {
      config.contact_manager_config.margin_data.setDefaultCollisionMargin(0.1);
      manager->applyContactManagerConfig(config.contact_manager_config);
      tesseract_collision::ContactResultMap contacts = checkTrajectoryState(*manager, tmap, config.contact_request);
      EXPECT_FALSE(contacts.empty());
    }
    // Not collision if checkTrajectoryState applies the config
    {
      config.contact_manager_config.margin_data.setDefaultCollisionMargin(0.0);
      tesseract_collision::ContactResultMap contacts = checkTrajectoryState(*manager, tmap, config);
      EXPECT_TRUE(contacts.empty());
    }
    // In collision if checkTrajectoryState applies the config
    {
      config.contact_manager_config.margin_data.setDefaultCollisionMargin(0.1);
      tesseract_collision::ContactResultMap contacts = checkTrajectoryState(*manager, tmap, config);
      EXPECT_FALSE(contacts.empty());
    }
  }

  // Check Continuous
  {
    auto config = default_config;
    ContinuousContactManager::Ptr manager = env->getContinuousContactManager();
    std::vector<std::string> active_links = { "boxbot_link", "test_box_link" };
    manager->setActiveCollisionObjects(active_links);

    // Put the swept volume of the boxes 0.05m away from each other
    tesseract_common::TransformMap tmap1;
    tmap1["boxbot_link"] = Eigen::Isometry3d::Identity();
    tmap1["test_box_link"] = Eigen::Isometry3d::Identity();
    tmap1["test_box_link"].translate(Eigen::Vector3d(1.05, 2, 0));

    tesseract_common::TransformMap tmap2;
    tmap2["boxbot_link"] = Eigen::Isometry3d::Identity();
    tmap2["test_box_link"] = Eigen::Isometry3d::Identity();
    tmap2["test_box_link"].translate(Eigen::Vector3d(1.05, -2, 0));

    // Not in collision
    {
      manager->applyContactManagerConfig(config.contact_manager_config);
      tesseract_collision::ContactResultMap contacts =
          checkTrajectorySegment(*manager, tmap1, tmap2, config.contact_request);
      EXPECT_TRUE(contacts.empty());
    }
    // In collision if manager->applyContactManagerConfig works correctly
    {
      config.contact_manager_config.margin_data.setDefaultCollisionMargin(0.1);
      manager->applyContactManagerConfig(config.contact_manager_config);
      tesseract_collision::ContactResultMap contacts =
          checkTrajectorySegment(*manager, tmap1, tmap2, config.contact_request);
      EXPECT_FALSE(contacts.empty());
    }
    // Not collision if checkTrajectoryState applies the config
    {
      config.contact_manager_config.margin_data.setDefaultCollisionMargin(0.0);
      tesseract_collision::ContactResultMap contacts = checkTrajectorySegment(*manager, tmap1, tmap2, config);
      EXPECT_TRUE(contacts.empty());
    }
    // In collision if checkTrajectoryState applies the config
    {
      config.contact_manager_config.margin_data.setDefaultCollisionMargin(0.1);
      tesseract_collision::ContactResultMap contacts = checkTrajectorySegment(*manager, tmap1, tmap2, config);
      EXPECT_FALSE(contacts.empty());
    }
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
