#include <tesseract_kinematics/core/macros.h>
TESSERACT_KINEMATICS_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <fstream>
#include <tesseract_scene_graph/parser/urdf_parser.h>
#include <ros/package.h>
TESSERACT_KINEMATICS_IGNORE_WARNINGS_POP

#include "tesseract_kinematics/kdl/kdl_fwd_kin_chain.h"
#include "tesseract_kinematics/kdl/kdl_fwd_kin_tree.h"
#include "tesseract_kinematics/core/utils.h"

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
  std::string path = std::string(DATA_DIR) + "/lbr_iiwa_14_r820.urdf";

  tesseract_scene_graph::ResourceLocatorFn locator = locateResource;
  return tesseract_scene_graph::parseURDF(path, locator);
}

void runFwdKinTest(tesseract_kinematics::ForwardKinematics& kin)
{
  //////////////////////////////////////////////////////////////////
  // Test forward kinematics when tip link is the base of the chain
  //////////////////////////////////////////////////////////////////
  Eigen::Isometry3d pose;
  Eigen::VectorXd jvals;
  jvals.resize(7);
  jvals.setZero();

  EXPECT_TRUE(kin.calcFwdKin(pose, jvals, "base_link"));
  EXPECT_TRUE(pose.isApprox(Eigen::Isometry3d::Identity()));

  ///////////////////////////
  // Test forward kinematics
  ///////////////////////////
  pose.setIdentity();
  EXPECT_TRUE(kin.calcFwdKin(pose, jvals, "tool0"));
  Eigen::Isometry3d result;
  result.setIdentity();
  result.translation()[0] = 0;
  result.translation()[1] = 0;
  result.translation()[2] = 1.306;
  EXPECT_TRUE(pose.isApprox(result));
}

void runJacobianTest(tesseract_kinematics::ForwardKinematics& kin)
{
  //////////////////////////////////////////////////////////////////
  // Test forward kinematics when tip link is the base of the chain
  //////////////////////////////////////////////////////////////////
  Eigen::MatrixXd jacobian;
  Eigen::VectorXd jvals;
  jvals.resize(7);

  jvals(0) = -0.785398;
  jvals(1) = 0.785398;
  jvals(2) = -0.785398;
  jvals(3) = 0.785398;
  jvals(4) = -0.785398;
  jvals(5) = 0.785398;
  jvals(6) = -0.785398;

  ///////////////////////////
  // Test Jacobian
  ///////////////////////////
  jacobian.resize(6, 7);
  EXPECT_TRUE(kin.calcJacobian(jacobian, jvals, "tool0"));

  Eigen::Isometry3d pose;
  kin.calcFwdKin(pose, jvals, "tool0");

  Eigen::VectorXd njvals;
  double delta = 0.001;
  for (int i = 0; i < static_cast<int>(jvals.size()); ++i)
  {
    njvals = jvals;
    njvals[i] += delta;
    Eigen::Isometry3d updated_pose;
    kin.calcFwdKin(updated_pose, njvals, "tool0");
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
    EXPECT_TRUE(kin.calcJacobian(jacobian, jvals, "tool0"));
    tesseract_kinematics::jacobianChangeRefPoint(jacobian, pose.linear() * link_point);

    for (int i = 0; i < static_cast<int>(jvals.size()); ++i)
    {
      njvals = jvals;
      njvals[i] += delta;
      Eigen::Isometry3d updated_pose;
      kin.calcFwdKin(updated_pose, njvals, "tool0");
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

    kin.calcFwdKin(pose, jvals, "tool0");
    // calcJacobian requires the link point to be in the base frame for which the jacobian is calculated.
    EXPECT_TRUE(kin.calcJacobian(jacobian, jvals, "tool0"));
    tesseract_kinematics::jacobianChangeBase(jacobian, change_base);
    tesseract_kinematics::jacobianChangeRefPoint(jacobian, (change_base * pose).linear() * link_point);

    for (int i = 0; i < static_cast<int>(jvals.size()); ++i)
    {
      njvals = jvals;
      njvals[i] += delta;
      Eigen::Isometry3d updated_pose;
      kin.calcFwdKin(updated_pose, njvals, "tool0");

      Eigen::Vector3d temp = change_base * pose * link_point;
      Eigen::Vector3d temp2 = change_base * updated_pose * link_point;
      double delta_x = (temp2.x() - temp.x()) / delta;
      double delta_y = (temp2.y() - temp.y()) / delta;
      double delta_z = (temp2.z() - temp.z()) / delta;
      EXPECT_NEAR(delta_x, jacobian(0, i), 1e-3);
      EXPECT_NEAR(delta_y, jacobian(1, i), 1e-3);
      EXPECT_NEAR(delta_z, jacobian(2, i), 1e-3);
      Eigen::AngleAxisd r12((change_base * pose).rotation().transpose() * (change_base * updated_pose).rotation());  // rotation from p1 -> p2
      double theta = r12.angle();
      theta = copysign(fmod(fabs(theta), 2.0 * M_PI), theta);
      if (theta < -M_PI)
        theta = theta + 2. * M_PI;
      if (theta > M_PI)
        theta = theta - 2. * M_PI;
      Eigen::VectorXd omega = ((change_base * pose).rotation() * r12.axis() * theta) / delta;
      EXPECT_NEAR(omega(0), jacobian(3, i), 1e-3);
      EXPECT_NEAR(omega(1), jacobian(4, i), 1e-3);
      EXPECT_NEAR(omega(2), jacobian(5, i), 1e-3);
    }
  }
}

TEST(TesseractROSUnit, KDLKinChainForwardKinematicUnit)
{
  tesseract_kinematics::KDLFwdKinChain kin;
  tesseract_scene_graph::SceneGraphPtr scene_graph = getSceneGraph();
  EXPECT_TRUE(kin.init(scene_graph, "base_link", "tool0", "manip"));

  runFwdKinTest(kin);
}

TEST(TesseractROSUnit, KDLKinTreeForwardKinematicUnit)
{
  tesseract_kinematics::KDLFwdKinTree kin;
  tesseract_scene_graph::SceneGraphPtr scene_graph = getSceneGraph();
  std::vector<std::string> joint_names = { "joint_a1", "joint_a2", "joint_a3", "joint_a4",
                                           "joint_a5", "joint_a6", "joint_a7" };

  std::unordered_map<std::string, double> start_state;
  start_state["joint_a1"] = 0;
  start_state["joint_a2"] = 0;
  start_state["joint_a3"] = 0;
  start_state["joint_a4"] = 0;
  start_state["joint_a5"] = 0;
  start_state["joint_a6"] = 0;
  start_state["joint_a7"] = 0;

  EXPECT_TRUE(kin.init(scene_graph, joint_names, start_state, "manip"));

  runFwdKinTest(kin);
}

TEST(TesseractROSUnit, KDLKinChainJacobianUnit)
{
  tesseract_kinematics::KDLFwdKinChain kin;
  tesseract_scene_graph::SceneGraphPtr scene_graph = getSceneGraph();
  EXPECT_TRUE(kin.init(scene_graph, "base_link", "tool0", "manip"));

  runJacobianTest(kin);
}

TEST(TesseractROSUnit, KDLKinTreeJacobianUnit)
{
  tesseract_kinematics::KDLFwdKinTree kin;
  tesseract_scene_graph::SceneGraphPtr scene_graph = getSceneGraph();
  std::vector<std::string> joint_names = { "joint_a1", "joint_a2", "joint_a3", "joint_a4",
                                           "joint_a5", "joint_a6", "joint_a7" };

  std::unordered_map<std::string, double> start_state;
  start_state["joint_a1"] = 0;
  start_state["joint_a2"] = 0;
  start_state["joint_a3"] = 0;
  start_state["joint_a4"] = 0;
  start_state["joint_a5"] = 0;
  start_state["joint_a6"] = 0;
  start_state["joint_a7"] = 0;

  EXPECT_TRUE(kin.init(scene_graph, joint_names, start_state, "manip"));

  runJacobianTest(kin);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
