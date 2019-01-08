#include <tesseract_core/macros.h>
TESSERACT_IGNORE_WARNINGS_PUSH
#include <ros/package.h>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <fstream>
#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>
#include <algorithm>
TESSERACT_IGNORE_WARNINGS_POP

#include "tesseract_ros/kdl/kdl_env.h"

void runTest(const tesseract::tesseract_ros::ROSBasicEnvPtr& env)
{
  // Test after clone if active list correct
  tesseract::DiscreteContactManagerBasePtr discrete_manager = env->getDiscreteContactManager();
  const std::vector<std::string>& e_active_list = env->getActiveLinkNames();
  const std::vector<std::string>& d_active_list = discrete_manager->getActiveCollisionObjects();
  EXPECT_TRUE(std::equal(e_active_list.begin(), e_active_list.end(), d_active_list.begin()));

  tesseract::ContinuousContactManagerBasePtr cast_manager = env->getContinuousContactManager();
  const std::vector<std::string>& c_active_list = cast_manager->getActiveCollisionObjects();
  EXPECT_TRUE(std::equal(e_active_list.begin(), e_active_list.end(), c_active_list.begin()));
}

TEST(TesseractROSUnit, KDLEnvActiveCollisionObjectUnit)
{
  std::string urdf_path = ros::package::getPath("tesseract_ros") + "/test/urdf/lbr_iiwa_14_r820.urdf";
  std::ifstream urdf_ifs(urdf_path);
  std::string urdf_xml_string((std::istreambuf_iterator<char>(urdf_ifs)), (std::istreambuf_iterator<char>()));

  urdf::ModelInterfaceSharedPtr urdf_model = urdf::parseURDF(urdf_xml_string);
  assert(urdf_model != nullptr);

  std::string srdf_path = ros::package::getPath("tesseract_ros") + "/test/urdf/lbr_iiwa_14_r820.srdf";
  std::ifstream srdf_ifs(srdf_path);
  std::string srdf_xml_string((std::istreambuf_iterator<char>(srdf_ifs)), (std::istreambuf_iterator<char>()));

  srdf::ModelSharedPtr srdf_model = srdf::ModelSharedPtr(new srdf::Model);
  srdf_model->initString(*urdf_model, srdf_xml_string);
  assert(srdf_model != nullptr);

  tesseract::tesseract_ros::KDLEnvPtr env(new tesseract::tesseract_ros::KDLEnv);
  assert(env != nullptr);

  bool success = env->init(urdf_model, srdf_model);
  assert(success);

  runTest(env);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
