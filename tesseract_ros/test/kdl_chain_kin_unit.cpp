
#include "tesseract_ros/kdl/kdl_chain_kin.h"
#include <ros/package.h>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <fstream>
#include <urdf_parser/urdf_parser.h>

urdf::ModelInterfaceSharedPtr getURDFModel()
{
  std::string path = ros::package::getPath("tesseract_ros") + "/test/urdf/lbr_iiwa_14_r820.urdf";
  std::ifstream ifs(path);
  std::string urdf_xml_string((std::istreambuf_iterator<char>(ifs)), (std::istreambuf_iterator<char>()));

  return urdf::parseURDF(urdf_xml_string);
}

void runTest(tesseract::tesseract_ros::ROSBasicKin& kin)
{
  //////////////////////////////////////////////////////////////////
  // Test forward kinematics when tip link is the base of the chain
  //////////////////////////////////////////////////////////////////
  Eigen::Affine3d pose;
  Eigen::VectorXd jvals;
  tesseract::EnvState state;

  jvals.resize(7);
  jvals.setZero();

  EXPECT_TRUE(kin.calcFwdKin(pose, Eigen::Affine3d::Identity(), jvals, "base_link", state));
  EXPECT_TRUE(pose.isApprox(Eigen::Affine3d::Identity()));

  ///////////////////////////
  // Test forward kinematics
  ///////////////////////////
  pose.setIdentity();
  EXPECT_TRUE(kin.calcFwdKin(pose, Eigen::Affine3d::Identity(), jvals, "tool0", state));
  Eigen::Affine3d result;
  result.setIdentity();
  result.translation()[0] = 0;
  result.translation()[1] = 0;
  result.translation()[2] = 1.306;
  EXPECT_TRUE(pose.isApprox(result));
}

TEST(TesseractROSUnit, KDLKinChainUnit)
{
  tesseract::tesseract_ros::KDLChainKin kin;
  urdf::ModelInterfaceSharedPtr urdf_model = getURDFModel();
  EXPECT_TRUE(kin.init(urdf_model, "base_link", "tool0", "manip"));

  runTest(kin);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
