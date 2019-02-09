#include <tesseract_core/macros.h>
TESSERACT_IGNORE_WARNINGS_PUSH
#include <ros/package.h>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <fstream>
#include <urdf_parser/urdf_parser.h>
TESSERACT_IGNORE_WARNINGS_POP

#include <tesseract_msgs/TesseractState.h>
#include <tesseract_ros/kdl/kdl_env.h>
#include <tesseract_ros/ros_tesseract_utils.h>

class TesseractROSUtilsUnitBase
{
protected:
  void SetUpRobot(std::string robot_name)
  {
    ros::Time::init();
    std::string path = ros::package::getPath("tesseract_ros");
    std::ifstream urdf_ifs(path + "/test/urdf/" + robot_name + ".urdf");
    std::ifstream srdf_ifs(path + "/test/srdf/" + robot_name + ".srdf");
    std::string urdf_xml_string((std::istreambuf_iterator<char>(urdf_ifs)), (std::istreambuf_iterator<char>()));
    std::string srdf_xml_string((std::istreambuf_iterator<char>(srdf_ifs)), (std::istreambuf_iterator<char>()));

    urdf_model = urdf::parseURDF(urdf_xml_string);
    srdf_model = srdf::ModelSharedPtr(new srdf::Model);
    srdf_model->initString(*(urdf_model.get()), srdf_xml_string);

    env.reset(new tesseract::tesseract_ros::KDLEnv());
    env->init(urdf_model, srdf_model);
  }

  urdf::ModelInterfaceSharedPtr urdf_model;
  srdf::ModelSharedPtr srdf_model;
  tesseract::tesseract_ros::KDLEnvPtr env;
};

class TesseractROSUtilsUnitWithParam : public ::testing::TestWithParam<const char*>, public TesseractROSUtilsUnitBase
{
protected:
  void SetUp() override { SetUpRobot(GetParam()); }
};

class TesseractROSUtilsUnit : public ::testing::Test, public TesseractROSUtilsUnitBase
{
protected:
  void SetUp() override { SetUpRobot("pppbot"); }
};

// Tests that TesseractState messages correctly store joint state
TEST_F(TesseractROSUtilsUnit, TestTesseractStateMsgJointState)
{
  std::unordered_map<std::string, double> js1, js2, js3, js4;
  js1["p1"] = 0.3;
  js1["p2"] = 0.3;
  js1["p3"] = -0.3;
  env->setState(js1);

  tesseract::EnvStateConstPtr state;
  state = env->getState();
  js2 = state->joints;
  ASSERT_TRUE(js1 == js2);

  tesseract_msgs::TesseractState tesseract_state_msg;
  tesseract::tesseract_ros::tesseractToTesseractStateMsg(tesseract_state_msg, *env);

  // changing joint state before processing the TesseractState msg
  js1["p3"] = 0;
  env->setState(js1);
  state = env->getState();
  js3 = state->joints;
  ASSERT_FALSE(js2 == js3);

  tesseract::tesseract_ros::processTesseractStateMsg(env, tesseract_state_msg);
  // checking joint state has been correctly loaded
  state = env->getState();
  js4 = state->joints;
  EXPECT_TRUE(js2 == js4);
}

// Tests that TesseractState messages correctly store the AllowedCollisionMatrix
TEST_F(TesseractROSUtilsUnit, TestTesseractStateMsgAllowedCollisionMatrix)
{
  tesseract::AllowedCollisionMatrixPtr acm = env->getAllowedCollisionMatrixNonConst();
  EXPECT_FALSE(acm->isCollisionAllowed("link2", "link3"));
  acm->addAllowedCollision("link2", "link3", "adjacent");
  EXPECT_TRUE(acm->isCollisionAllowed("link2", "link3"));
  const size_t n_allowed = acm->getAllAllowedCollisions().size();

  tesseract_msgs::TesseractState tesseract_state_msg;
  tesseract::tesseract_ros::tesseractToTesseractStateMsg(tesseract_state_msg, *env);

  // changing ACM before processing the TesseractState msg
  acm->clearAllowedCollisions();
  EXPECT_FALSE(acm->isCollisionAllowed("link2", "link3"));
  ASSERT_EQ(acm->getAllAllowedCollisions().size(), 0);

  tesseract::tesseract_ros::processTesseractStateMsg(env, tesseract_state_msg);
  // checking ACM has been properly loaded
  EXPECT_TRUE(acm->isCollisionAllowed("link2", "link3"));
  EXPECT_EQ(acm->getAllAllowedCollisions().size(), n_allowed);
}

// Tests that TesseractState messages correctly store joint state
TEST_P(TesseractROSUtilsUnitWithParam, TestGetActiveLinkNamesRecursive)
{
  std::unordered_map<std::string, std::vector<std::string>> expected_link_names;
  expected_link_names["pppbot"] = { "link1", "link2", "link3" };
  expected_link_names["lbr_iiwa_14_r820"] = { "link_1", "link_2", "link_3", "link_4",
                                              "link_5", "link_6", "link_7", "tool0" };
  EXPECT_EQ(env->getActiveLinkNames(), expected_link_names[GetParam()]);
}

INSTANTIATE_TEST_CASE_P(AllRobots, TesseractROSUtilsUnitWithParam, testing::ValuesIn({ "pppbot", "lbr_iiwa_14_r820" }));

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
