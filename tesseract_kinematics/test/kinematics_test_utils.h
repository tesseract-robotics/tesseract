/**
 * @file kin_test_suite.h
 * @brief Tesseract kinematics test suite
 *
 * @author Levi Armstrong
 * @date Feb 4, 2021
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2021, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef TESSERACT_KINEMATICS_KIN_TEST_SUITE_H
#define TESSERACT_KINEMATICS_KIN_TEST_SUITE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <fstream>
#include <yaml-cpp/yaml.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_kinematics/core/forward_kinematics.h>
#include <tesseract_kinematics/core/inverse_kinematics.h>
#include <tesseract_kinematics/core/utils.h>
#include <tesseract_kinematics/core/types.h>
#include <tesseract_kinematics/core/kinematic_group.h>
#include <tesseract_kinematics/core/kinematics_plugin_factory.h>

#include <tesseract_scene_graph/graph.h>
#include <tesseract_state_solver/kdl/kdl_state_solver.h>

#include <tesseract_urdf/urdf_parser.h>
#include <tesseract_common/utils.h>

namespace tesseract_kinematics
{
namespace test_suite
{
inline std::string locateResource(const std::string& url)
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

    mod_url = package_path + mod_url;  // "file://" + package_path + mod_url;
  }

  return mod_url;
}

inline tesseract_scene_graph::SceneGraph::UPtr getSceneGraphIIWA()
{
  std::string path = std::string(TESSERACT_SUPPORT_DIR) + "/urdf/lbr_iiwa_14_r820.urdf";

  tesseract_common::SimpleResourceLocator locator(locateResource);
  return tesseract_urdf::parseURDFFile(path, locator);
}

inline tesseract_scene_graph::SceneGraph::UPtr getSceneGraphABBExternalPositioner()
{
  std::string path = std::string(TESSERACT_SUPPORT_DIR) + "/urdf/abb_irb2400_external_positioner.urdf";

  tesseract_common::SimpleResourceLocator locator(locateResource);
  return tesseract_urdf::parseURDFFile(path, locator);
}

inline tesseract_scene_graph::SceneGraph::UPtr getSceneGraphABBOnPositioner()
{
  std::string path = std::string(TESSERACT_SUPPORT_DIR) + "/urdf/abb_irb2400_on_positioner.urdf";

  tesseract_common::SimpleResourceLocator locator(locateResource);
  return tesseract_urdf::parseURDFFile(path, locator);
}

inline tesseract_scene_graph::SceneGraph::UPtr getSceneGraphABB()
{
  std::string path = std::string(TESSERACT_SUPPORT_DIR) + "/urdf/abb_irb2400.urdf";

  tesseract_common::SimpleResourceLocator locator(locateResource);
  return tesseract_urdf::parseURDFFile(path, locator);
}

inline tesseract_scene_graph::SceneGraph::UPtr getSceneGraphIIWA7()
{
  std::string path = std::string(TESSERACT_SUPPORT_DIR) + "/urdf/iiwa7.urdf";

  tesseract_common::SimpleResourceLocator locator(locateResource);
  return tesseract_urdf::parseURDFFile(path, locator);
}

inline tesseract_scene_graph::SceneGraph::UPtr getSceneGraphUR(const tesseract_kinematics::URParameters& params,
                                                               double shoulder_offset,
                                                               double elbow_offset)
{
  using namespace tesseract_scene_graph;

  auto sg = std::make_unique<SceneGraph>("universal_robot");
  sg->addLink(Link("base_link"));
  sg->addLink(Link("shoulder_link"));
  sg->addLink(Link("upper_arm_link"));
  sg->addLink(Link("forearm_link"));
  sg->addLink(Link("wrist_1_link"));
  sg->addLink(Link("wrist_2_link"));
  sg->addLink(Link("wrist_3_link"));
  sg->addLink(Link("ee_link"));
  sg->addLink(Link("tool0"));

  {
    Joint j("shoulder_pan_joint");
    j.type = JointType::REVOLUTE;
    j.parent_link_name = "base_link";
    j.child_link_name = "shoulder_link";
    j.axis = Eigen::Vector3d::UnitZ();
    j.parent_to_joint_origin_transform.translation() = Eigen::Vector3d(0, 0, params.d1);
    j.limits = std::make_shared<JointLimits>();
    j.limits->lower = -2.0 * M_PI;
    j.limits->upper = 2.0 * M_PI;
    j.limits->velocity = 2.16;
    j.limits->acceleration = 0.5 * j.limits->velocity;
    sg->addJoint(j);
  }

  {
    Joint j("shoulder_lift_joint");
    j.type = JointType::REVOLUTE;
    j.parent_link_name = "shoulder_link";
    j.child_link_name = "upper_arm_link";
    j.axis = Eigen::Vector3d::UnitY();
    j.parent_to_joint_origin_transform.translation() = Eigen::Vector3d(0, shoulder_offset, 0);
    j.parent_to_joint_origin_transform =
        j.parent_to_joint_origin_transform * Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY());
    j.limits = std::make_shared<JointLimits>();
    j.limits->lower = -2.0 * M_PI;
    j.limits->upper = 2.0 * M_PI;
    j.limits->velocity = 2.16;
    j.limits->acceleration = 0.5 * j.limits->velocity;
    sg->addJoint(j);
  }

  {
    Joint j("elbow_joint");
    j.type = JointType::REVOLUTE;
    j.parent_link_name = "upper_arm_link";
    j.child_link_name = "forearm_link";
    j.axis = Eigen::Vector3d::UnitY();
    j.parent_to_joint_origin_transform.translation() = Eigen::Vector3d(0, elbow_offset, -params.a2);
    j.limits = std::make_shared<JointLimits>();
    j.limits->lower = -2.0 * M_PI;
    j.limits->upper = 2.0 * M_PI;
    j.limits->velocity = 2.16;
    j.limits->acceleration = 0.5 * j.limits->velocity;
    sg->addJoint(j);
  }

  {
    Joint j("wrist_1_joint");
    j.type = JointType::REVOLUTE;
    j.parent_link_name = "forearm_link";
    j.child_link_name = "wrist_1_link";
    j.axis = Eigen::Vector3d::UnitY();
    j.parent_to_joint_origin_transform.translation() = Eigen::Vector3d(0, 0, -params.a3);
    j.parent_to_joint_origin_transform =
        j.parent_to_joint_origin_transform * Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY());
    j.limits = std::make_shared<JointLimits>();
    j.limits->lower = -2.0 * M_PI;
    j.limits->upper = 2.0 * M_PI;
    j.limits->velocity = 2.16;
    j.limits->acceleration = 0.5 * j.limits->velocity;
    sg->addJoint(j);
  }

  {
    Joint j("wrist_2_joint");
    j.type = JointType::REVOLUTE;
    j.parent_link_name = "wrist_1_link";
    j.child_link_name = "wrist_2_link";
    j.axis = Eigen::Vector3d::UnitZ();
    j.parent_to_joint_origin_transform.translation() =
        Eigen::Vector3d(0, params.d4 - elbow_offset - shoulder_offset, 0);
    j.limits = std::make_shared<JointLimits>();
    j.limits->lower = -2.0 * M_PI;
    j.limits->upper = 2.0 * M_PI;
    j.limits->velocity = 2.16;
    j.limits->acceleration = 0.5 * j.limits->velocity;
    sg->addJoint(j);
  }

  {
    Joint j("wrist_3_joint");
    j.type = JointType::REVOLUTE;
    j.parent_link_name = "wrist_2_link";
    j.child_link_name = "wrist_3_link";
    j.axis = Eigen::Vector3d::UnitY();
    j.parent_to_joint_origin_transform.translation() = Eigen::Vector3d(0, 0, params.d5);
    j.limits = std::make_shared<JointLimits>();
    j.limits->lower = -2.0 * M_PI;
    j.limits->upper = 2.0 * M_PI;
    j.limits->velocity = 2.16;
    j.limits->acceleration = 0.5 * j.limits->velocity;
    sg->addJoint(j);
  }

  {
    Joint j("ee_fixed_joint");
    j.type = JointType::FIXED;
    j.parent_link_name = "wrist_3_link";
    j.child_link_name = "ee_link";
    j.parent_to_joint_origin_transform.translation() = Eigen::Vector3d(0, params.d6, 0);
    j.parent_to_joint_origin_transform =
        j.parent_to_joint_origin_transform * Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ());
    sg->addJoint(j);
  }

  {
    Joint j("wrist_3_link-tool0_fixed_joint");
    j.type = JointType::FIXED;
    j.parent_link_name = "wrist_3_link";
    j.child_link_name = "tool0";
    j.parent_to_joint_origin_transform.translation() = Eigen::Vector3d(0, params.d6, 0);
    j.parent_to_joint_origin_transform =
        j.parent_to_joint_origin_transform * Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitX());
    sg->addJoint(j);
  }

  return sg;
}

inline tesseract_common::KinematicLimits getTargetLimits(const tesseract_scene_graph::SceneGraph& scene_graph,
                                                         const std::vector<std::string>& joint_names)
{
  auto s = static_cast<Eigen::Index>(joint_names.size());

  tesseract_common::KinematicLimits limits;
  limits.resize(s);

  for (Eigen::Index i = 0; i < s; ++i)
  {
    auto joint = scene_graph.getJoint(joint_names[static_cast<std::size_t>(i)]);
    limits.joint_limits(i, 0) = joint->limits->lower;
    limits.joint_limits(i, 1) = joint->limits->upper;
    limits.velocity_limits(i) = joint->limits->velocity;
    limits.acceleration_limits(i) = joint->limits->acceleration;
  }

  return limits;
}

/**
 * @brief Run a kinematic jacobian test
 * @param kin The kinematics object
 * @param jvals The joint values to calculate the jacobian about
 * @param link_name Name of link to calculate jacobian. If empty it will use the function that does not require link
 * name
 * @param link_point Is expressed in the same base frame of the jacobian and is a vector from the old point to the new
 * point.
 * @param change_base The transform from the desired frame to the current base frame of the jacobian
 */
inline void runJacobianTest(tesseract_kinematics::ForwardKinematics& kin,
                            const Eigen::VectorXd& jvals,
                            const std::string& link_name,
                            const Eigen::Vector3d& link_point,
                            const Eigen::Isometry3d& change_base)
{
  Eigen::MatrixXd jacobian, numerical_jacobian;
  tesseract_common::TransformMap poses;

  jacobian.resize(6, kin.numJoints());

  poses = kin.calcFwdKin(jvals);
  jacobian = kin.calcJacobian(jvals, link_name);
  tesseract_common::jacobianChangeBase(jacobian, change_base);
  tesseract_common::jacobianChangeRefPoint(jacobian, (change_base * poses[link_name]).linear() * link_point);

  numerical_jacobian.resize(6, kin.numJoints());
  tesseract_kinematics::numericalJacobian(numerical_jacobian, change_base, kin, jvals, link_name, link_point);

  for (int i = 0; i < 6; ++i)
    for (int j = 0; j < static_cast<int>(kin.numJoints()); ++j)
      EXPECT_NEAR(numerical_jacobian(i, j), jacobian(i, j), 1e-3);
}

/**
 * @brief Run a kinematic jacobian test
 * @param kin The kinematics object
 * @param jvals The joint values to calculate the jacobian about
 * @param link_name Name of link to calculate jacobian. If empty it will use the function that does not require link
 * name
 * @param link_point Is expressed in the same base frame of the jacobian and is a vector from the old point to the new
 * point.
 */
inline void runJacobianTest(tesseract_kinematics::KinematicGroup& kin_group,
                            const Eigen::VectorXd& jvals,
                            const std::string& link_name,
                            const Eigen::Vector3d& link_point)
{
  Eigen::MatrixXd jacobian, numerical_jacobian;
  jacobian.resize(6, kin_group.numJoints());

  tesseract_common::TransformMap poses = kin_group.calcFwdKin(jvals);
  {  // Test with all information
    jacobian = kin_group.calcJacobian(jvals, link_name, link_point);

    numerical_jacobian.resize(6, kin_group.numJoints());
    tesseract_kinematics::numericalJacobian(numerical_jacobian, kin_group, jvals, link_name, link_point);

    for (int i = 0; i < 6; ++i)
    {
      for (int j = 0; j < static_cast<int>(kin_group.numJoints()); ++j)
      {
        EXPECT_NEAR(numerical_jacobian(i, j), jacobian(i, j), 1e-3);
      }
    }
  }

  {  // Test don't use link_point
    jacobian = kin_group.calcJacobian(jvals, link_name);

    numerical_jacobian.resize(6, kin_group.numJoints());
    tesseract_kinematics::numericalJacobian(numerical_jacobian, kin_group, jvals, link_name, Eigen::Vector3d::Zero());

    for (int i = 0; i < 6; ++i)
    {
      for (int j = 0; j < static_cast<int>(kin_group.numJoints()); ++j)
      {
        EXPECT_NEAR(numerical_jacobian(i, j), jacobian(i, j), 1e-3);
      }
    }
  }
}

/**
 * @brief Run kinematic limits test
 * @param limits The limits to check
 * @param target_limits The target to compare to
 */
inline void runKinJointLimitsTest(const tesseract_common::KinematicLimits& limits,
                                  const tesseract_common::KinematicLimits& target_limits)
{
  //////////////////////////////////////////////////////////////////
  // Test forward kinematics joint limits
  //////////////////////////////////////////////////////////////////
  EXPECT_EQ(limits.joint_limits.rows(), target_limits.joint_limits.rows());
  EXPECT_EQ(limits.velocity_limits.rows(), target_limits.velocity_limits.rows());
  EXPECT_EQ(limits.acceleration_limits.rows(), target_limits.acceleration_limits.rows());

  // Check limits
  for (Eigen::Index i = 0; i < limits.joint_limits.rows(); ++i)
  {
    EXPECT_NEAR(limits.joint_limits(i, 0), target_limits.joint_limits(i, 0), 1e-6);
    EXPECT_NEAR(limits.joint_limits(i, 1), target_limits.joint_limits(i, 1), 1e-6);
    EXPECT_NEAR(limits.velocity_limits(i), target_limits.velocity_limits(i), 1e-6);
    EXPECT_NEAR(limits.acceleration_limits(i), target_limits.acceleration_limits(i), 1e-6);
  }
}

/**
 * @brief Run kinematics setJointLimits function test
 * @param kin Kinematic object to test
 */
inline void runKinSetJointLimitsTest(tesseract_kinematics::KinematicGroup& kin_group)
{
  //////////////////////////////////////////////////////////////////
  // Test setting kinematic group joint limits
  //////////////////////////////////////////////////////////////////
  tesseract_common::KinematicLimits limits = kin_group.getLimits();
  EXPECT_TRUE(limits.joint_limits.rows() > 0);
  EXPECT_TRUE(limits.velocity_limits.rows() > 0);
  EXPECT_TRUE(limits.acceleration_limits.rows() > 0);

  // Check limits
  for (Eigen::Index i = 0; i < limits.joint_limits.rows(); ++i)
  {
    limits.joint_limits(i, 0) = -5.0 - double(i);
    limits.joint_limits(i, 1) = 5.0 + double(i);
    limits.velocity_limits(i) = 10.0 + double(i);
    limits.acceleration_limits(i) = 5.0 + double(i);
  }

  kin_group.setLimits(limits);
  runKinJointLimitsTest(kin_group.getLimits(), limits);

  // Test failure
  tesseract_common::KinematicLimits limits_empty;
  EXPECT_ANY_THROW(kin_group.setLimits(limits_empty));  // NOLINT
}

/**
 * @brief Check if two vectors of strings are equal but ignore order
 * @param names Vector to check
 * @param target_names Target to compare against
 */
inline void runStringVectorEqualTest(const std::vector<std::string>& names,
                                     const std::vector<std::string>& target_names)
{
  EXPECT_EQ(names.size(), target_names.size());
  EXPECT_FALSE(names.empty());
  EXPECT_FALSE(target_names.empty());

  std::vector<std::string> v1 = names;
  std::vector<std::string> v2 = target_names;
  std::sort(v1.begin(), v1.end());
  std::sort(v2.begin(), v2.end());
  EXPECT_TRUE(std::equal(v1.begin(), v1.end(), v2.begin()));
  //  EXPECT_TRUE(tesseract_common::isIdentical(names, target_names, false));
}

/**
 * @brief Run inverse kinematics test comparing the inverse solution to the forward solution
 * @param inv_kin The inverse kinematics object
 * @param fwd_kin The forward kinematics object to compare to
 * @param target_pose The target pose to solve inverse kinematics for
 * @param seed The seed used for solving inverse kinematics
 */
inline void runInvKinTest(const tesseract_kinematics::InverseKinematics& inv_kin,
                          const tesseract_kinematics::ForwardKinematics& fwd_kin,
                          const Eigen::Isometry3d& target_pose,
                          const std::string& tip_link_name,
                          const Eigen::VectorXd& seed)
{
  ///////////////////////////
  // Test Inverse kinematics
  ///////////////////////////
  EXPECT_TRUE(inv_kin.getBaseLinkName() == fwd_kin.getBaseLinkName());  // This only works if they are equal
  tesseract_common::TransformMap input{ std::make_pair(tip_link_name, target_pose) };
  IKSolutions solutions = inv_kin.calcInvKin(input, seed);
  EXPECT_TRUE(!solutions.empty());

  for (const auto& sol : solutions)
  {
    tesseract_common::TransformMap result_poses = fwd_kin.calcFwdKin(sol);
    Eigen::Isometry3d result = result_poses[tip_link_name];
    EXPECT_TRUE(target_pose.translation().isApprox(result.translation(), 1e-4));

    Eigen::Quaterniond rot_pose(target_pose.rotation());
    Eigen::Quaterniond rot_result(result.rotation());
    EXPECT_TRUE(rot_pose.isApprox(rot_result, 1e-3));
  }
}

/**
 * @brief Run inverse kinematics test comparing the inverse solution to the forward solution
 * @param kin_group The kinematic group
 * @param target_pose The target pose to solve inverse kinematics for
 * @param seed The seed used for solving inverse kinematics
 */
inline void runInvKinTest(const tesseract_kinematics::KinematicGroup& kin_group,
                          const Eigen::Isometry3d& target_pose,
                          const std::string& working_frame,
                          const std::string& tip_link_name,
                          const Eigen::VectorXd& seed)
{
  ///////////////////////////
  // Test Inverse kinematics
  ///////////////////////////
  KinGroupIKInputs inputs{ KinGroupIKInput(target_pose, working_frame, tip_link_name) };
  IKSolutions solutions = kin_group.calcInvKin(inputs, seed);
  EXPECT_TRUE(!solutions.empty());

  for (const auto& sol : solutions)
  {
    tesseract_common::TransformMap result_poses = kin_group.calcFwdKin(sol);
    Eigen::Isometry3d result = result_poses.at(working_frame).inverse() * result_poses[tip_link_name];
    EXPECT_TRUE(target_pose.translation().isApprox(result.translation(), 1e-4));

    Eigen::Quaterniond rot_pose(target_pose.rotation());
    Eigen::Quaterniond rot_result(result.rotation());
    EXPECT_TRUE(rot_pose.isApprox(rot_result, 1e-3));
  }
}

inline void runFwdKinIIWATest(tesseract_kinematics::ForwardKinematics& kin)
{
  //////////////////////////////////////////////////////////////////
  // Test forward kinematics when tip link is the base of the chain
  //////////////////////////////////////////////////////////////////

  Eigen::VectorXd jvals;
  jvals.resize(7);
  jvals.setZero();

  tesseract_common::TransformMap poses = kin.calcFwdKin(jvals);

  EXPECT_EQ(poses.size(), 1);
  Eigen::Isometry3d pose = poses.at("tool0");
  Eigen::Isometry3d result;
  result.setIdentity();
  result.translation()[0] = 0;
  result.translation()[1] = 0;
  result.translation()[2] = 1.306;
  EXPECT_TRUE(pose.isApprox(result));
}

inline void runJacobianIIWATest(tesseract_kinematics::ForwardKinematics& kin, bool is_kin_tree = false)
{
  UNUSED(is_kin_tree);
  std::string tip_link = "tool0";
  std::string base_link = "base_link";

  //////////////////////////////////////////////////////////////////
  // Test forward kinematics when tip link is the base of the chain
  //////////////////////////////////////////////////////////////////
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
  Eigen::Vector3d link_point(0, 0, 0);
  runJacobianTest(kin, jvals, tip_link, link_point, Eigen::Isometry3d::Identity());

  EXPECT_ANY_THROW(runJacobianTest(kin, jvals, "", link_point, Eigen::Isometry3d::Identity()));  // NOLINT

  ///////////////////////////
  // Test Jacobian at Point
  ///////////////////////////
  for (int k = 0; k < 3; ++k)
  {
    Eigen::Vector3d link_point(0, 0, 0);
    link_point[k] = 1;

    runJacobianTest(kin, jvals, tip_link, link_point, Eigen::Isometry3d::Identity());

    EXPECT_ANY_THROW(runJacobianTest(kin, jvals, "", link_point, Eigen::Isometry3d::Identity()));  // NOLINT
  }

  ///////////////////////////////////////////
  // Test Jacobian with change base
  ///////////////////////////////////////////
  for (int k = 0; k < 3; ++k)
  {
    link_point = Eigen::Vector3d(0, 0, 0);
    Eigen::Isometry3d change_base;
    change_base.setIdentity();
    change_base(0, 0) = 0;
    change_base(1, 0) = 1;
    change_base(0, 1) = -1;
    change_base(1, 1) = 0;
    change_base.translation() = Eigen::Vector3d(0, 0, 0);
    change_base.translation()[k] = 1;

    runJacobianTest(kin, jvals, tip_link, link_point, change_base);

    EXPECT_ANY_THROW(runJacobianTest(kin, jvals, "", link_point, change_base));  // NOLINT
  }

  ///////////////////////////////////////////
  // Test Jacobian at point with change base
  ///////////////////////////////////////////
  for (int k = 0; k < 3; ++k)
  {
    Eigen::Vector3d link_point(0, 0, 0);
    link_point[k] = 1;

    Eigen::Isometry3d change_base;
    change_base.setIdentity();
    change_base(0, 0) = 0;
    change_base(1, 0) = 1;
    change_base(0, 1) = -1;
    change_base(1, 1) = 0;
    change_base.translation() = link_point;

    runJacobianTest(kin, jvals, tip_link, link_point, change_base);

    EXPECT_ANY_THROW(runJacobianTest(kin, jvals, "", link_point, change_base));  // NOLINT
  }
}

inline void runKinGroupJacobianIIWATest(tesseract_kinematics::KinematicGroup& kin_group)
{
  std::string base_link_name = "base_link";
  std::vector<std::string> link_names = { "base_link", "link_1", "link_2", "link_3", "link_4",
                                          "link_5",    "link_6", "link_7", "tool0" };

  //////////////////////////////////////////////////////////////////
  // Test forward kinematics when tip link is the base of the chain
  //////////////////////////////////////////////////////////////////
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
  // Test Jacobian at Point
  // This also runs a test without the link point
  ///////////////////////////
  for (int k = 0; k < 3; ++k)
  {
    Eigen::Vector3d link_point(0, 0, 0);
    link_point[k] = 1;

    for (const auto& link_name : link_names)
      runJacobianTest(kin_group, jvals, link_name, link_point);

    // NOLINTNEXTLINE
    EXPECT_ANY_THROW(runJacobianTest(kin_group, jvals, "", link_point));
  }
}

inline void runActiveLinkNamesIIWATest(const tesseract_kinematics::KinematicGroup& kin_group)
{
  EXPECT_FALSE(kin_group.checkJoints(Eigen::VectorXd::Zero(8)));
  EXPECT_FALSE(kin_group.checkJoints(Eigen::VectorXd::Constant(7, std::numeric_limits<double>::max())));
  EXPECT_FALSE(kin_group.checkJoints(Eigen::VectorXd::Constant(7, -std::numeric_limits<double>::max())));
  EXPECT_TRUE(kin_group.checkJoints(Eigen::VectorXd::Zero(7)));

  std::vector<std::string> target_active_link_names = { "link_1", "link_2", "link_3", "link_4",
                                                        "link_5", "link_6", "link_7", "tool0" };

  std::vector<std::string> target_static_link_names = { "base_link", "base" };

  std::vector<std::string> target_link_names = target_active_link_names;
  target_link_names.insert(target_link_names.end(), target_static_link_names.begin(), target_static_link_names.end());

  for (const auto& l : target_active_link_names)
  {
    EXPECT_TRUE(kin_group.isActiveLinkName(l));
  }

  for (const auto& l : target_link_names)
  {
    EXPECT_TRUE(kin_group.hasLinkName(l));
  }

  {
    std::vector<std::string> link_names = kin_group.getActiveLinkNames();
    runStringVectorEqualTest(link_names, target_active_link_names);
  }

  {
    std::vector<std::string> link_names = kin_group.getStaticLinkNames();
    runStringVectorEqualTest(link_names, target_static_link_names);
  }

  {
    std::vector<std::string> link_names = kin_group.getLinkNames();
    runStringVectorEqualTest(link_names, target_link_names);
  }
}

inline void runActiveLinkNamesABBTest(const tesseract_kinematics::KinematicGroup& kin_group)
{
  EXPECT_FALSE(kin_group.checkJoints(Eigen::VectorXd::Zero(7)));
  EXPECT_FALSE(kin_group.checkJoints(Eigen::VectorXd::Constant(6, std::numeric_limits<double>::max())));
  EXPECT_FALSE(kin_group.checkJoints(Eigen::VectorXd::Constant(6, -std::numeric_limits<double>::max())));
  EXPECT_TRUE(kin_group.checkJoints(Eigen::VectorXd::Zero(6)));

  std::vector<std::string> target_active_link_names = { "link_1", "link_2", "link_3", "link_4",
                                                        "link_5", "link_6", "tool0" };

  std::vector<std::string> target_static_link_names = { "base_link" };

  std::vector<std::string> target_link_names = target_active_link_names;
  target_link_names.insert(target_link_names.end(), target_static_link_names.begin(), target_static_link_names.end());

  for (const auto& l : target_active_link_names)
  {
    EXPECT_TRUE(kin_group.isActiveLinkName(l));
  }

  for (const auto& l : target_link_names)
  {
    EXPECT_TRUE(kin_group.hasLinkName(l));
  }

  {
    std::vector<std::string> link_names = kin_group.getActiveLinkNames();
    runStringVectorEqualTest(link_names, target_active_link_names);
  }

  {
    std::vector<std::string> link_names = kin_group.getStaticLinkNames();
    runStringVectorEqualTest(link_names, target_static_link_names);
  }

  {
    std::vector<std::string> link_names = kin_group.getLinkNames();
    runStringVectorEqualTest(link_names, target_link_names);
  }
}

inline void runActiveLinkNamesURTest(const tesseract_kinematics::KinematicGroup& kin_group)
{
  EXPECT_FALSE(kin_group.checkJoints(Eigen::VectorXd::Zero(7)));
  EXPECT_FALSE(kin_group.checkJoints(Eigen::VectorXd::Constant(6, std::numeric_limits<double>::max())));
  EXPECT_FALSE(kin_group.checkJoints(Eigen::VectorXd::Constant(6, -std::numeric_limits<double>::max())));
  EXPECT_TRUE(kin_group.checkJoints(Eigen::VectorXd::Zero(6)));

  std::vector<std::string> target_active_link_names = { "shoulder_link", "upper_arm_link", "forearm_link",
                                                        "wrist_1_link",  "wrist_2_link",   "wrist_3_link",
                                                        "tool0" };

  std::vector<std::string> target_static_link_names = { "base_link" };

  std::vector<std::string> target_link_names = target_active_link_names;
  target_link_names.insert(target_link_names.end(), target_static_link_names.begin(), target_static_link_names.end());

  for (const auto& l : target_active_link_names)
  {
    EXPECT_TRUE(kin_group.isActiveLinkName(l));
  }

  for (const auto& l : target_link_names)
  {
    EXPECT_TRUE(kin_group.hasLinkName(l));
  }

  {
    std::vector<std::string> link_names = kin_group.getActiveLinkNames();
    runStringVectorEqualTest(link_names, target_active_link_names);
  }

  {
    std::vector<std::string> link_names = kin_group.getStaticLinkNames();
    runStringVectorEqualTest(link_names, target_static_link_names);
  }

  {
    std::vector<std::string> link_names = kin_group.getLinkNames();
    runStringVectorEqualTest(link_names, target_link_names);
  }
}

inline void runKinGroupJacobianABBOnPositionerTest(tesseract_kinematics::KinematicGroup& kin_group)
{
  std::string base_link_name = "positioner_base_link";
  std::vector<std::string> link_names{ "positioner_base_link",
                                       "positioner_tool0",
                                       "base",
                                       "base_link",
                                       "link_1",
                                       "link_2",
                                       "link_3",
                                       "link_4",
                                       "link_5",
                                       "link_6",
                                       "tool0" };

  //////////////////////////////////////////////////////////////////
  // Test forward kinematics when tip link is the base of the chain
  //////////////////////////////////////////////////////////////////
  Eigen::MatrixXd jacobian, numerical_jacobian;
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
  // Test Jacobian at Point
  // Note: Also tests without the link point
  ///////////////////////////
  for (int k = 0; k < 3; ++k)
  {
    Eigen::Vector3d link_point(0, 0, 0);
    link_point[k] = 1;

    for (const auto& link_name : link_names)
      runJacobianTest(kin_group, jvals, link_name, link_point);

    // NOLINTNEXTLINE
    EXPECT_ANY_THROW(runJacobianTest(kin_group, jvals, "", link_point));
  }
}

inline void runActiveLinkNamesABBOnPositionerTest(const tesseract_kinematics::KinematicGroup& kin_group)
{
  EXPECT_FALSE(kin_group.checkJoints(Eigen::VectorXd::Zero(8)));
  EXPECT_FALSE(kin_group.checkJoints(Eigen::VectorXd::Constant(7, std::numeric_limits<double>::max())));
  EXPECT_FALSE(kin_group.checkJoints(Eigen::VectorXd::Constant(7, -std::numeric_limits<double>::max())));
  EXPECT_TRUE(kin_group.checkJoints(Eigen::VectorXd::Zero(7)));

  std::vector<std::string> target_active_link_names = { "positioner_tool0", "base_link", "base",   "link_1", "link_2",
                                                        "link_3",           "link_4",    "link_5", "link_6", "tool0" };

  std::vector<std::string> target_static_link_names = { "positioner_base_link" };

  std::vector<std::string> target_link_names = target_active_link_names;
  target_link_names.insert(target_link_names.end(), target_static_link_names.begin(), target_static_link_names.end());

  for (const auto& l : target_active_link_names)
  {
    EXPECT_TRUE(kin_group.isActiveLinkName(l));
  }

  for (const auto& l : target_link_names)
  {
    EXPECT_TRUE(kin_group.hasLinkName(l));
  }

  {
    std::vector<std::string> link_names = kin_group.getActiveLinkNames();
    runStringVectorEqualTest(link_names, target_active_link_names);
  }

  {
    std::vector<std::string> link_names = kin_group.getStaticLinkNames();
    runStringVectorEqualTest(link_names, target_static_link_names);
  }

  {
    std::vector<std::string> link_names = kin_group.getLinkNames();
    runStringVectorEqualTest(link_names, target_link_names);
  }
}

inline void runKinGroupJacobianABBExternalPositionerTest(tesseract_kinematics::KinematicGroup& kin_group)
{
  std::string base_link_name = "positioner_base_link";
  std::vector<std::string> link_names{ "positioner_base_link",
                                       "positioner_tool0",
                                       "positioner_link_1",
                                       "base_link",
                                       "link_1",
                                       "link_2",
                                       "link_3",
                                       "link_4",
                                       "link_5",
                                       "link_6",
                                       "tool0" };

  //////////////////////////////////////////////////////////////////
  // Test forward kinematics when tip link is the base of the chain
  //////////////////////////////////////////////////////////////////
  Eigen::MatrixXd jacobian, numerical_jacobian;
  Eigen::VectorXd jvals;
  jvals.resize(8);

  jvals(0) = -0.785398;
  jvals(1) = 0.785398;
  jvals(2) = -0.785398;
  jvals(3) = 0.785398;
  jvals(4) = -0.785398;
  jvals(5) = 0.785398;
  jvals(6) = -0.785398;
  jvals(7) = 0.785398;

  ///////////////////////////
  // Test Jacobian at Point
  // Note: Also tests without the link point
  ///////////////////////////
  for (int k = 0; k < 3; ++k)
  {
    Eigen::Vector3d link_point(0, 0, 0);
    link_point[k] = 1;

    for (const auto& link_name : link_names)
      runJacobianTest(kin_group, jvals, link_name, link_point);

    // NOLINTNEXTLINE
    EXPECT_ANY_THROW(runJacobianTest(kin_group, jvals, "", link_point));
  }
}

inline void runActiveLinkNamesABBExternalPositionerTest(const tesseract_kinematics::KinematicGroup& kin_group)
{
  EXPECT_FALSE(kin_group.checkJoints(Eigen::VectorXd::Zero(9)));
  EXPECT_FALSE(kin_group.checkJoints(Eigen::VectorXd::Constant(8, std::numeric_limits<double>::max())));
  EXPECT_FALSE(kin_group.checkJoints(Eigen::VectorXd::Constant(8, -std::numeric_limits<double>::max())));
  EXPECT_TRUE(kin_group.checkJoints(Eigen::VectorXd::Zero(8)));

  std::vector<std::string> target_active_link_names = {
    "positioner_tool0", "positioner_link_1", "link_1", "link_2", "link_3", "link_4", "link_5", "link_6", "tool0"
  };

  std::vector<std::string> target_static_link_names = { "world", "base_link", "base", "positioner_base_link" };

  std::vector<std::string> target_link_names = target_active_link_names;
  target_link_names.insert(target_link_names.end(), target_static_link_names.begin(), target_static_link_names.end());

  for (const auto& l : target_active_link_names)
  {
    EXPECT_TRUE(kin_group.isActiveLinkName(l));
  }

  for (const auto& l : target_link_names)
  {
    EXPECT_TRUE(kin_group.hasLinkName(l));
  }

  {
    std::vector<std::string> link_names = kin_group.getActiveLinkNames();
    runStringVectorEqualTest(link_names, target_active_link_names);
  }

  {
    std::vector<std::string> link_names = kin_group.getStaticLinkNames();
    runStringVectorEqualTest(link_names, target_static_link_names);
  }

  {
    std::vector<std::string> link_names = kin_group.getLinkNames();
    runStringVectorEqualTest(link_names, target_link_names);
  }
}

inline void runInvKinIIWATest(const tesseract_kinematics::KinematicsPluginFactory& factory,
                              const std::string& inv_factory_name,
                              const std::string& fwd_factory_name)
{
  tesseract_scene_graph::SceneGraph::Ptr scene_graph = getSceneGraphIIWA();
  std::string manip_name = "manip";
  std::string base_link_name = "base_link";
  std::string tip_link_name = "tool0";
  std::vector<std::string> joint_names{ "joint_a1", "joint_a2", "joint_a3", "joint_a4",
                                        "joint_a5", "joint_a6", "joint_a7" };
  std::vector<std::string> joint_link_names{ "link_1", "link_2", "link_3", "link_4", "link_5", "link_6", "link_7" };
  tesseract_common::KinematicLimits target_limits = getTargetLimits(*scene_graph, joint_names);

  tesseract_scene_graph::KDLStateSolver state_solver(*scene_graph);
  tesseract_scene_graph::SceneState scene_state = state_solver.getState();

  tesseract_common::PluginInfo fwd_plugin_info;
  fwd_plugin_info.class_name = fwd_factory_name;
  fwd_plugin_info.config["base_link"] = base_link_name;
  fwd_plugin_info.config["tip_link"] = tip_link_name;

  tesseract_common::PluginInfo inv_plugin_info;
  inv_plugin_info.class_name = inv_factory_name;
  inv_plugin_info.config = fwd_plugin_info.config;

  // Inverse target pose and seed
  Eigen::Isometry3d pose;
  pose.setIdentity();
  pose.translation()[0] = 0;
  pose.translation()[1] = 0;
  pose.translation()[2] = 1.306;

  Eigen::VectorXd seed;
  seed.resize(7);
  seed(0) = -0.785398;
  seed(1) = 0.785398;
  seed(2) = -0.785398;
  seed(3) = 0.785398;
  seed(4) = -0.785398;
  seed(5) = 0.785398;
  seed(6) = -0.785398;

  // Check create method with empty scene graph
  tesseract_scene_graph::SceneGraph scene_graph_empty;
  auto kin_empty = factory.createInvKin(inv_factory_name, inv_plugin_info, scene_graph_empty, scene_state);
  EXPECT_TRUE(kin_empty == nullptr);

  {  // Check create method using base_link and tool0
    auto fwd_kin = factory.createFwdKin(fwd_factory_name, fwd_plugin_info, *scene_graph, scene_state);
    EXPECT_TRUE(fwd_kin != nullptr);
    //    EXPECT_EQ(fwd_kin->getSolverName(), fwd_solver_name);
    EXPECT_EQ(fwd_kin->numJoints(), 7);
    EXPECT_EQ(fwd_kin->getBaseLinkName(), base_link_name);
    EXPECT_EQ(fwd_kin->getTipLinkNames().size(), 1);
    EXPECT_EQ(fwd_kin->getTipLinkNames()[0], tip_link_name);
    EXPECT_EQ(fwd_kin->getJointNames(), joint_names);

    runJacobianIIWATest(*fwd_kin);
    runFwdKinIIWATest(*fwd_kin);

    auto inv_kin = factory.createInvKin(inv_factory_name, inv_plugin_info, *scene_graph, scene_state);
    EXPECT_TRUE(inv_kin != nullptr);
    //    EXPECT_EQ(inv_kin->getSolverName(), inv_solver_name);
    EXPECT_EQ(inv_kin->numJoints(), 7);
    EXPECT_EQ(inv_kin->getBaseLinkName(), base_link_name);
    EXPECT_EQ(inv_kin->getWorkingFrame(), base_link_name);
    EXPECT_EQ(inv_kin->getTipLinkNames().size(), 1);
    EXPECT_EQ(inv_kin->getTipLinkNames()[0], tip_link_name);
    EXPECT_EQ(inv_kin->getJointNames(), joint_names);

    runInvKinTest(*inv_kin, *fwd_kin, pose, tip_link_name, seed);

    KinematicGroup kin_group(manip_name, joint_names, std::move(inv_kin), *scene_graph, scene_state);
    EXPECT_EQ(kin_group.getBaseLinkName(), scene_graph->getRoot());
    runInvKinTest(kin_group, pose, base_link_name, tip_link_name, seed);
    runKinGroupJacobianIIWATest(kin_group);
    runActiveLinkNamesIIWATest(kin_group);
    runKinJointLimitsTest(kin_group.getLimits(), target_limits);
    runKinSetJointLimitsTest(kin_group);
  }

  {  // Check cloned
    auto fwd_kin = factory.createFwdKin(fwd_factory_name, fwd_plugin_info, *scene_graph, scene_state);
    EXPECT_TRUE(fwd_kin != nullptr);
    auto fwd_kin3 = fwd_kin->clone();
    //    EXPECT_EQ(fwd_kin3->getSolverName(), fwd_solver_name);
    EXPECT_EQ(fwd_kin3->numJoints(), 7);
    EXPECT_EQ(fwd_kin3->getBaseLinkName(), base_link_name);
    EXPECT_EQ(fwd_kin3->getTipLinkNames().size(), 1);
    EXPECT_EQ(fwd_kin3->getTipLinkNames()[0], tip_link_name);
    EXPECT_EQ(fwd_kin3->getJointNames(), joint_names);

    runJacobianIIWATest(*fwd_kin3);
    runFwdKinIIWATest(*fwd_kin3);

    auto inv_kin = factory.createInvKin(inv_factory_name, inv_plugin_info, *scene_graph, scene_state);
    auto inv_kin3 = inv_kin->clone();
    EXPECT_TRUE(inv_kin3 != nullptr);
    //    EXPECT_EQ(inv_kin3->getSolverName(), inv_solver_name);
    EXPECT_EQ(inv_kin3->numJoints(), 7);
    EXPECT_EQ(inv_kin3->getBaseLinkName(), base_link_name);
    EXPECT_EQ(inv_kin3->getWorkingFrame(), base_link_name);
    EXPECT_EQ(inv_kin3->getTipLinkNames().size(), 1);
    EXPECT_EQ(inv_kin3->getTipLinkNames()[0], tip_link_name);
    EXPECT_EQ(inv_kin3->getJointNames(), joint_names);

    runInvKinTest(*inv_kin3, *fwd_kin3, pose, tip_link_name, seed);

    KinematicGroup kin_group(manip_name, joint_names, std::move(inv_kin3), *scene_graph, scene_state);
    EXPECT_EQ(kin_group.getBaseLinkName(), scene_graph->getRoot());
    runInvKinTest(kin_group, pose, base_link_name, tip_link_name, seed);
    runKinGroupJacobianIIWATest(kin_group);
    runActiveLinkNamesIIWATest(kin_group);
    runKinJointLimitsTest(kin_group.getLimits(), target_limits);
    runKinSetJointLimitsTest(kin_group);
  }

  fwd_plugin_info.config["base_link"] = "missing_link";
  fwd_plugin_info.config["tip_link"] = tip_link_name;
  inv_plugin_info.config = fwd_plugin_info.config;

  {  // Test forward kinematics failure

    auto fwd_kin = factory.createFwdKin(fwd_factory_name, fwd_plugin_info, *scene_graph, scene_state);
    EXPECT_TRUE(fwd_kin == nullptr);
  }

  {  // Inverse Kinematics Test failure
    auto inv_kin = factory.createInvKin(inv_factory_name, inv_plugin_info, *scene_graph, scene_state);
    EXPECT_TRUE(inv_kin == nullptr);
  }
}

}  // namespace test_suite
}  // namespace tesseract_kinematics
#endif  // TESSERACT_KINEMATICS_KIN_TEST_SUITE_H
