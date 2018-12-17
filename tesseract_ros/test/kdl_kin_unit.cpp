#include <tesseract_core/macros.h>
TESSERACT_IGNORE_WARNINGS_PUSH
#include <ros/package.h>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <fstream>
#include <urdf_parser/urdf_parser.h>
TESSERACT_IGNORE_WARNINGS_POP

#include "tesseract_ros/kdl/kdl_chain_kin.h"
#include "tesseract_ros/kdl/kdl_joint_kin.h"

urdf::ModelInterfaceSharedPtr getURDFModel()
{
  std::string path = ros::package::getPath("tesseract_ros") + "/test/urdf/lbr_iiwa_14_r820.urdf";
  std::ifstream ifs(path);
  std::string urdf_xml_string((std::istreambuf_iterator<char>(ifs)), (std::istreambuf_iterator<char>()));

  return urdf::parseURDF(urdf_xml_string);
}

void runFwdKinTest(tesseract::tesseract_ros::ROSBasicKin& kin)
{
  //////////////////////////////////////////////////////////////////
  // Test forward kinematics when tip link is the base of the chain
  //////////////////////////////////////////////////////////////////
  Eigen::Isometry3d pose;
  Eigen::VectorXd jvals;
  tesseract::EnvState state;
  state.joints.reserve(7);
  state.joints["joint_a1"] = 0;
  state.joints["joint_a2"] = 0;
  state.joints["joint_a3"] = 0;
  state.joints["joint_a4"] = 0;
  state.joints["joint_a5"] = 0;
  state.joints["joint_a6"] = 0;
  state.joints["joint_a7"] = 0;

  jvals.resize(7);
  jvals.setZero();

  EXPECT_TRUE(kin.calcFwdKin(pose, Eigen::Isometry3d::Identity(), jvals, "base_link", state));
  EXPECT_TRUE(pose.isApprox(Eigen::Isometry3d::Identity()));

  ///////////////////////////
  // Test forward kinematics
  ///////////////////////////
  pose.setIdentity();
  EXPECT_TRUE(kin.calcFwdKin(pose, Eigen::Isometry3d::Identity(), jvals, "tool0", state));
  Eigen::Isometry3d result;
  result.setIdentity();
  result.translation()[0] = 0;
  result.translation()[1] = 0;
  result.translation()[2] = 1.306;
  EXPECT_TRUE(pose.isApprox(result));
}

void runJacobianTest(tesseract::tesseract_ros::ROSBasicKin& kin)
{
  //////////////////////////////////////////////////////////////////
  // Test forward kinematics when tip link is the base of the chain
  //////////////////////////////////////////////////////////////////
  Eigen::MatrixXd jacobian;
  Eigen::VectorXd jvals;
  tesseract::EnvState state;
  state.joints.reserve(7);
  jvals.resize(7);

  jvals(0) = -0.785398;
  jvals(1) = 0.785398;
  jvals(2) = -0.785398;
  jvals(3) = 0.785398;
  jvals(4) = -0.785398;
  jvals(5) = 0.785398;
  jvals(6) = -0.785398;

  state.joints["joint_a1"] = jvals(0);
  state.joints["joint_a2"] = jvals(1);
  state.joints["joint_a3"] = jvals(2);
  state.joints["joint_a4"] = jvals(3);
  state.joints["joint_a5"] = jvals(4);
  state.joints["joint_a6"] = jvals(5);
  state.joints["joint_a7"] = jvals(6);

  ///////////////////////////
  // Test Jacobian
  ///////////////////////////
  jacobian.resize(6, 7);
  EXPECT_TRUE(kin.calcJacobian(jacobian, Eigen::Isometry3d::Identity(), jvals, "tool0", state));

  Eigen::Isometry3d pose;
  kin.calcFwdKin(pose, Eigen::Isometry3d::Identity(), jvals, "tool0", state);

  Eigen::VectorXd njvals;
  double delta = 0.001;
  for (int i = 0; i < static_cast<int>(jvals.size()); ++i)
  {
    njvals = jvals;
    njvals[i] += delta;
    Eigen::Isometry3d updated_pose;
    kin.calcFwdKin(updated_pose, Eigen::Isometry3d::Identity(), njvals, "tool0", state);
    double delta_x = (updated_pose.translation().x() - pose.translation().x()) / delta;
    double delta_y = (updated_pose.translation().y() - pose.translation().y()) / delta;
    double delta_z = (updated_pose.translation().z() - pose.translation().z()) / delta;
    EXPECT_NEAR(delta_x, jacobian(0, i), 1e-3);
    EXPECT_NEAR(delta_y, jacobian(1, i), 1e-3);
    EXPECT_NEAR(delta_z, jacobian(2, i), 1e-3);
    Eigen::AngleAxisd r12(pose.rotation().transpose() * updated_pose.rotation());  // rotation from p1 -> p2
    double theta = r12.angle();
    theta = copysign(fmod(fabs(theta), 2.0 * M_PI), theta);
    if (theta < -M_PI)
      theta = theta + 2. * M_PI;
    if (theta > M_PI)
      theta = theta - 2. * M_PI;
    Eigen::VectorXd omega = (pose.rotation() * r12.axis() * theta) / delta;
    EXPECT_NEAR(omega(0), jacobian(3, i), 1e-3);
    EXPECT_NEAR(omega(1), jacobian(4, i), 1e-3);
    EXPECT_NEAR(omega(2), jacobian(5, i), 1e-3);
  }

  ///////////////////////////
  // Test Jacobian at Point
  ///////////////////////////
  for (int k = 0; k < 3; ++k)
  {
    Eigen::Vector3d link_point;
    link_point.setZero();
    link_point[k] = 1;
    // calcJacobian requires the link point to be in the base frame for which the jacobian is calculated.
    EXPECT_TRUE(kin.calcJacobian(jacobian, Eigen::Isometry3d::Identity(), jvals, "tool0", state, pose * link_point));

    for (int i = 0; i < static_cast<int>(jvals.size()); ++i)
    {
      njvals = jvals;
      njvals[i] += delta;
      Eigen::Isometry3d updated_pose;
      kin.calcFwdKin(updated_pose, Eigen::Isometry3d::Identity(), njvals, "tool0", state);
      Eigen::Vector3d temp = pose * link_point;
      Eigen::Vector3d temp2 = updated_pose * link_point;
      double delta_x = (temp2.x() - temp.x()) / delta;
      double delta_y = (temp2.y() - temp.y()) / delta;
      double delta_z = (temp2.z() - temp.z()) / delta;
      EXPECT_NEAR(delta_x, jacobian(0, i), 1e-3);
      EXPECT_NEAR(delta_y, jacobian(1, i), 1e-3);
      EXPECT_NEAR(delta_z, jacobian(2, i), 1e-3);
      Eigen::AngleAxisd r12(pose.rotation().transpose() * updated_pose.rotation());  // rotation from p1 -> p2
      double theta = r12.angle();
      theta = copysign(fmod(fabs(theta), 2.0 * M_PI), theta);
      if (theta < -M_PI)
        theta = theta + 2. * M_PI;
      if (theta > M_PI)
        theta = theta - 2. * M_PI;
      Eigen::VectorXd omega = (pose.rotation() * r12.axis() * theta) / delta;
      EXPECT_NEAR(omega(0), jacobian(3, i), 1e-3);
      EXPECT_NEAR(omega(1), jacobian(4, i), 1e-3);
      EXPECT_NEAR(omega(2), jacobian(5, i), 1e-3);
    }
  }

  ///////////////////////////////////////////
  // Test Jacobian at point with change base
  ///////////////////////////////////////////

  for (int k = 0; k < 3; ++k)
  {
    Eigen::Vector3d link_point;
    link_point.setZero();
    link_point[k] = 1;

    Eigen::Isometry3d change_base;
    change_base.setIdentity();
    change_base(0, 0) = 0;
    change_base(1, 0) = 1;
    change_base(0, 1) = -1;
    change_base(1, 1) = 0;
    change_base.translation() = link_point;

    kin.calcFwdKin(pose, change_base, jvals, "tool0", state);
    // calcJacobian requires the link point to be in the base frame for which the jacobian is calculated.
    EXPECT_TRUE(kin.calcJacobian(jacobian, change_base, jvals, "tool0", state, pose * link_point));

    for (int i = 0; i < static_cast<int>(jvals.size()); ++i)
    {
      njvals = jvals;
      njvals[i] += delta;
      Eigen::Isometry3d updated_pose;
      kin.calcFwdKin(updated_pose, change_base, njvals, "tool0", state);
      Eigen::Vector3d temp = pose * link_point;
      Eigen::Vector3d temp2 = updated_pose * link_point;
      double delta_x = (temp2.x() - temp.x()) / delta;
      double delta_y = (temp2.y() - temp.y()) / delta;
      double delta_z = (temp2.z() - temp.z()) / delta;
      EXPECT_NEAR(delta_x, jacobian(0, i), 1e-3);
      EXPECT_NEAR(delta_y, jacobian(1, i), 1e-3);
      EXPECT_NEAR(delta_z, jacobian(2, i), 1e-3);
      Eigen::AngleAxisd r12(pose.rotation().transpose() * updated_pose.rotation());  // rotation from p1 -> p2
      double theta = r12.angle();
      theta = copysign(fmod(fabs(theta), 2.0 * M_PI), theta);
      if (theta < -M_PI)
        theta = theta + 2. * M_PI;
      if (theta > M_PI)
        theta = theta - 2. * M_PI;
      Eigen::VectorXd omega = (pose.rotation() * r12.axis() * theta) / delta;
      EXPECT_NEAR(omega(0), jacobian(3, i), 1e-3);
      EXPECT_NEAR(omega(1), jacobian(4, i), 1e-3);
      EXPECT_NEAR(omega(2), jacobian(5, i), 1e-3);
    }
  }
}

TEST(TesseractROSUnit, KDLKinChainForwardKinematicUnit)
{
  tesseract::tesseract_ros::KDLChainKin kin;
  urdf::ModelInterfaceSharedPtr urdf_model = getURDFModel();
  EXPECT_TRUE(kin.init(urdf_model, "base_link", "tool0", "manip"));

  runFwdKinTest(kin);
}

TEST(TesseractROSUnit, KDLKinJointForwardKinematicUnit)
{
  tesseract::tesseract_ros::KDLJointKin kin;
  urdf::ModelInterfaceSharedPtr urdf_model = getURDFModel();
  std::vector<std::string> joint_names = { "joint_a1", "joint_a2", "joint_a3", "joint_a4",
                                           "joint_a5", "joint_a6", "joint_a7" };
  EXPECT_TRUE(kin.init(urdf_model, joint_names, "manip"));

  runFwdKinTest(kin);
}

TEST(TesseractROSUnit, KDLKinChainJacobianUnit)
{
  tesseract::tesseract_ros::KDLChainKin kin;
  urdf::ModelInterfaceSharedPtr urdf_model = getURDFModel();
  EXPECT_TRUE(kin.init(urdf_model, "base_link", "tool0", "manip"));

  runJacobianTest(kin);
}

TEST(TesseractROSUnit, KDLKinJointJacobianUnit)
{
  tesseract::tesseract_ros::KDLJointKin kin;
  urdf::ModelInterfaceSharedPtr urdf_model = getURDFModel();
  std::vector<std::string> joint_names = { "joint_a1", "joint_a2", "joint_a3", "joint_a4",
                                           "joint_a5", "joint_a6", "joint_a7" };
  EXPECT_TRUE(kin.init(urdf_model, joint_names, "manip"));

  runJacobianTest(kin);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
