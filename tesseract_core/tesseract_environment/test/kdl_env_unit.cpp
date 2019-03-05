#include <tesseract_environment/core/macros.h>
TESSERACT_ENVIRONMENT_IGNORE_WARNINGS_PUSH
#include <ros/package.h>
#include <gtest/gtest.h>
#include <algorithm>
#include <tesseract_scene_graph/parser/urdf_parser.h>
#include <tesseract_collision/bullet/bullet_discrete_bvh_manager.h>
#include <tesseract_collision/bullet/bullet_cast_bvh_manager.h>
TESSERACT_ENVIRONMENT_IGNORE_WARNINGS_POP

#include "tesseract_environment/kdl/kdl_env.h"


std::string locateResource(const std::string& url)
{
  std::string mod_url = url;
  if (url.find("package://") == 0)
  {
    mod_url.erase(0, strlen("package://"));
    size_t pos = mod_url.find("/");
    if (pos == std::string::npos)
    {
      return std::string();
    }

    std::string package = mod_url.substr(0, pos);
    mod_url.erase(0, pos);
    std::string package_path = ros::package::getPath(package);

    if (package_path.empty())
    {
      return std::string();
    }

    mod_url = package_path + mod_url; // "file://" + package_path + mod_url;
  }

  return mod_url;
}

tesseract_scene_graph::SceneGraphPtr getSceneGraph()
{
  std::string path = std::string(DATA_DIR) + "/urdf/lbr_iiwa_14_r820.urdf";

  tesseract_scene_graph::ResourceLocatorFn locator = locateResource;
  return tesseract_scene_graph::parseURDF(path, locator);
}

void runTest(const tesseract_environment::EnvironmentPtr& env)
{
  // Test after clone if active list correct
  tesseract_collision::DiscreteContactManagerPtr discrete_manager = env->getDiscreteContactManager();
  const std::vector<std::string>& e_active_list = env->getActiveLinkNames();
  const std::vector<std::string>& d_active_list = discrete_manager->getActiveCollisionObjects();
  EXPECT_TRUE(std::equal(e_active_list.begin(), e_active_list.end(), d_active_list.begin()));

  tesseract_collision::ContinuousContactManagerPtr cast_manager = env->getContinuousContactManager();
  const std::vector<std::string>& c_active_list = cast_manager->getActiveCollisionObjects();
  EXPECT_TRUE(std::equal(e_active_list.begin(), e_active_list.end(), c_active_list.begin()));
}

TEST(TesseractROSUnit, KDLEnvActiveCollisionObjectUnit)
{
  tesseract_scene_graph::SceneGraphPtr scene_graph = getSceneGraph();
  assert(scene_graph != nullptr);

  tesseract_environment::KDLEnvPtr env(new tesseract_environment::KDLEnv);
  assert(env != nullptr);

  bool success = env->init(scene_graph);
  env->setDiscreteContactManager(tesseract_collision::tesseract_collision_bullet::BulletDiscreteBVHManagerPtr(new tesseract_collision::tesseract_collision_bullet::BulletDiscreteBVHManager()));
  env->setContinuousContactManager(tesseract_collision::tesseract_collision_bullet::BulletCastBVHManagerPtr(new tesseract_collision::tesseract_collision_bullet::BulletCastBVHManager()));
  assert(success);

  runTest(env);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
