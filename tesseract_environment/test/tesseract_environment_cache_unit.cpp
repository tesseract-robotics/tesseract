#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <algorithm>
#include <vector>
#include <tesseract_urdf/urdf_parser.h>
#include <tesseract_common/resource_locator.h>
#include <tesseract_common/utils.h>
#include <tesseract_geometry/impl/box.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/environment.h>
#include <tesseract_environment/environment_cache.h>
#include <tesseract_environment/commands/add_link_command.h>

#include <tesseract_scene_graph/graph.h>
#include <tesseract_scene_graph/link.h>

#include <tesseract_srdf/srdf_model.h>

using namespace tesseract_scene_graph;
using namespace tesseract_srdf;
using namespace tesseract_environment;

SceneGraph::Ptr getSceneGraph(const tesseract_common::ResourceLocator& locator)
{
  std::string path = "package://tesseract_support/urdf/boxbot.urdf";
  return tesseract_urdf::parseURDFFile(locator.locateResource(path)->getFilePath(), locator);
}

SRDFModel::Ptr getSRDFModel(const SceneGraph& scene_graph, const tesseract_common::ResourceLocator& locator)
{
  std::string path = "package://tesseract_support/urdf/boxbot.srdf";

  auto srdf = std::make_shared<SRDFModel>();
  srdf->initFile(scene_graph, locator.locateResource(path)->getFilePath(), locator);

  return srdf;
}

void addLink(Environment& env)
{
  auto visual = std::make_shared<Visual>();
  visual->geometry = std::make_shared<tesseract_geometry::Box>(1, 1, 1);
  auto collision = std::make_shared<Collision>();
  collision->geometry = std::make_shared<tesseract_geometry::Box>(1, 1, 1);

  Link link_1("link_n1");
  link_1.visual.push_back(visual);
  link_1.collision.push_back(collision);

  auto cmd = std::make_shared<AddLinkCommand>(link_1);
  EXPECT_TRUE(cmd != nullptr);
  EXPECT_EQ(cmd->getType(), CommandType::ADD_LINK);
  EXPECT_TRUE(cmd->getLink() != nullptr);
  EXPECT_TRUE(cmd->getJoint() == nullptr);
  EXPECT_TRUE(env.applyCommand(cmd));
}

TEST(TesseractEnvironmentCache, defaultEnvironmentCacheTest)  // NOLINT
{
  tesseract_common::GeneralResourceLocator locator;
  auto scene_graph = getSceneGraph(locator);
  EXPECT_TRUE(scene_graph != nullptr);

  auto srdf = getSRDFModel(*scene_graph, locator);
  EXPECT_TRUE(srdf != nullptr);

  auto env = std::make_shared<Environment>();
  bool success = env->init(*scene_graph, srdf);
  EXPECT_TRUE(success);

  DefaultEnvironmentCache cache(env, 10);
  EXPECT_EQ(cache.getCacheSize(), 10);

  cache.setCacheSize(5);
  EXPECT_EQ(cache.getCacheSize(), 5);

  for (int i = 0; i < 20; ++i)
  {
    Environment::UPtr cached_env = cache.getCachedEnvironment();
    EXPECT_TRUE(cached_env != nullptr);
    EXPECT_EQ(cached_env->getRevision(), 3);
  }

  addLink(*env);

  EXPECT_EQ(cache.getCacheSize(), 5);

  for (int i = 0; i < 10; ++i)
  {
    Environment::UPtr cached_env = cache.getCachedEnvironment();
    EXPECT_TRUE(cached_env != nullptr);
    EXPECT_EQ(cached_env->getRevision(), 4);
  }

  cache.refreshCache();
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
