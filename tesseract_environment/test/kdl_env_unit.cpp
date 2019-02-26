#include <tesseract_environment/core/macros.h>
TESSERACT_ENVIRONMENT_IGNORE_WARNINGS_PUSH
#include <ros/package.h>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <fstream>
#include <urdf_parser/urdf_parser.h>
#include <algorithm>
TESSERACT_ENVIRONMENT_IGNORE_WARNINGS_POP

#include "tesseract_environment/kdl/kdl_env.h"
#include "tesseract_collision/bullet/bullet_discrete_bvh_manager.h"
#include "tesseract_collision/bullet/bullet_cast_bvh_manager.h"

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
  std::string urdf_path = std::string(DATA_DIR) + "/urdf/lbr_iiwa_14_r820.urdf";
  std::ifstream urdf_ifs(urdf_path);
  std::string urdf_xml_string((std::istreambuf_iterator<char>(urdf_ifs)), (std::istreambuf_iterator<char>()));

  urdf::ModelInterfaceSharedPtr urdf_model = urdf::parseURDF(urdf_xml_string);
  assert(urdf_model != nullptr);

  tesseract_environment::KDLEnvPtr env(new tesseract_environment::KDLEnv);
  assert(env != nullptr);

  bool success = env->init(urdf_model);
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
