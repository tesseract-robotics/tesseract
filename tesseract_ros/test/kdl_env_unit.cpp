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
#include "tesseract_ros/ros_tesseract_utils.h"

class TesseractKDLEnvUnit : public testing::Test
{
protected:
  void SetUp() override
  {
    std::string path = ros::package::getPath("tesseract_ros");
    std::ifstream urdf_ifs(path + "/test/urdf/pppbot.urdf");
    std::ifstream srdf_ifs(path + "/test/srdf/pppbot.srdf");
    std::string urdf_xml_string((std::istreambuf_iterator<char>(urdf_ifs)), (std::istreambuf_iterator<char>()));
    std::string srdf_xml_string((std::istreambuf_iterator<char>(srdf_ifs)), (std::istreambuf_iterator<char>()));

    urdf_model = urdf::parseURDF(urdf_xml_string);
    srdf_model = srdf::ModelSharedPtr(new srdf::Model);
    srdf_model->initString(*(urdf_model.get()), srdf_xml_string);

    env.reset(new tesseract::tesseract_ros::KDLEnv());
    env->init(urdf_model, srdf_model);
  }

  bool discreteCollisionCheck()
  {
    tesseract::DiscreteContactManagerBasePtr dcm;
    dcm = env->getDiscreteContactManager();
    tesseract::ContactResultMap collisions;
    dcm->contactTest(collisions, tesseract::ContactTestTypes::FIRST);
    return (collisions.size() > 0);
  }

  bool continuousCollisionCheck(const std::unordered_map<std::string, double>& start,
                                const std::unordered_map<std::string, double>& target)
  {
    std::vector<std::string> jointNames = env->getJointNames();
    tesseract::TrajArray traj;
    traj.resize(2, 3);
    traj.setZero();
    unsigned int i = 0;
    for (const std::string& jn : jointNames)
    {
      traj.row(0)[i] = start.at(jn);
      traj.row(1)[i] = target.at(jn);
      ++i;
    }
    tesseract::ContinuousContactManagerBasePtr ccm;
    ccm = env->getContinuousContactManager();
    std::vector<tesseract::ContactResultMap> collisions;
    tesseract::BasicKinConstPtr kin = env->getManipulator("manipulator");
    bool found = tesseract::continuousCollisionCheckTrajectory(*ccm, *env, *kin, traj, collisions);
    return found;
  }

  urdf::ModelInterfaceSharedPtr urdf_model;
  srdf::ModelSharedPtr srdf_model;
  tesseract::tesseract_ros::KDLEnvPtr env;
};

TEST_F(TesseractKDLEnvUnit, TestACMWorks)
{
  std::unordered_map<std::string, double> js1, js2, js3, js4;
  js1["p1"] = 0.0;
  js1["p2"] = 0.0;
  js1["p3"] = 0.0;

  // js3 in collision
  js3 = js1;
  js3["p2"] = -.01;

  // we expect the collision check to return True since js3 is in collision
  env->setState(js3);
  EXPECT_TRUE(discreteCollisionCheck());

  // used to enable and disable collisions
  tesseract::AllowedCollisionMatrixPtr acm;
  acm = env->getAllowedCollisionMatrixNonConst();

  // we allowed collision between link1 and link2, so we expect the collision
  // check to return False
  acm->addAllowedCollision("link2", "link3", "test");
  EXPECT_TRUE(acm->isCollisionAllowed("link2", "link3"));
  EXPECT_TRUE(env->getIsContactAllowedFn()("link2", "link3"));
  tesseract::DiscreteContactManagerBasePtr dcm;
  dcm = env->getDiscreteContactManager();
  EXPECT_TRUE(dcm->getIsContactAllowedFn()("link2", "link3"));
  ASSERT_FALSE(discreteCollisionCheck());

  acm->removeAllowedCollision("link2", "link3");
  ASSERT_TRUE(discreteCollisionCheck());

  // js1 and js2 not in collision, but swept-volume is in collision
  js2 = js1;
  js2["p2"] = 0.55;
  // in fact, when p2 goes from 0 to 0.55, a collision happens along the way
  js4 = js1;
  js4["p2"] = 0.4;

  env->setState(js4);
  ASSERT_TRUE(discreteCollisionCheck());
  env->setState(js2);
  ASSERT_FALSE(discreteCollisionCheck());
  env->setState(js1);
  ASSERT_FALSE(discreteCollisionCheck());

  tesseract::ContinuousContactManagerBasePtr ccm;

  ccm = env->getContinuousContactManager();
  EXPECT_FALSE(ccm->getIsContactAllowedFn()("link2", "obstacle1"));
  // we expect the continuous collision check to find collisions
  ASSERT_TRUE(continuousCollisionCheck(js1, js2));

  acm->addAllowedCollision("link1", "obstacle1", "test");
  acm->addAllowedCollision("link2", "obstacle1", "test");
  acm->addAllowedCollision("link3", "obstacle1", "test");
  ccm = env->getContinuousContactManager();
  EXPECT_TRUE(ccm->getIsContactAllowedFn()("link2", "obstacle1"));
  ASSERT_FALSE(continuousCollisionCheck(js1, js2));

  acm->removeAllowedCollision("link1", "obstacle1");
  acm->removeAllowedCollision("link2", "obstacle1");
  acm->removeAllowedCollision("link3", "obstacle1");
  ccm = env->getContinuousContactManager();
  EXPECT_FALSE(ccm->getIsContactAllowedFn()("link2", "obstacle1"));
  ASSERT_TRUE(continuousCollisionCheck(js1, js2));
}

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
