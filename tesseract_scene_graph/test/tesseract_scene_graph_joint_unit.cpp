#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <iostream>
#include <fstream>
#include <tesseract_geometry/geometries.h>
#include <tesseract_common/utils.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_scene_graph/graph.h>
#include <tesseract_scene_graph/utils.h>
#include <tesseract_scene_graph/resource_locator.h>

TEST(TesseractSceneGraphUnit, TesseractSceneGraphJointDynamicsUnit)  // NOLINT
{
  using namespace tesseract_scene_graph;
  JointDynamics j;

  EXPECT_NEAR(j.damping, 0, 1e-6);
  EXPECT_NEAR(j.friction, 0, 1e-6);

  j.damping = 10;
  j.friction = 5;
  j.clear();

  EXPECT_NEAR(j.damping, 0, 1e-6);
  EXPECT_NEAR(j.friction, 0, 1e-6);
}

TEST(TesseractSceneGraphUnit, TesseractSceneGraphJointLimitsUnit)  // NOLINT
{
  using namespace tesseract_scene_graph;
  JointLimits j;

  EXPECT_NEAR(j.lower, 0, 1e-6);
  EXPECT_NEAR(j.upper, 0, 1e-6);
  EXPECT_NEAR(j.effort, 0, 1e-6);
  EXPECT_NEAR(j.velocity, 0, 1e-6);
  EXPECT_NEAR(j.acceleration, 0, 1e-6);

  j.lower = 1;
  j.upper = 2;
  j.effort = 3;
  j.velocity = 4;
  j.acceleration = 5;
  j.clear();

  EXPECT_NEAR(j.lower, 0, 1e-6);
  EXPECT_NEAR(j.upper, 0, 1e-6);
  EXPECT_NEAR(j.effort, 0, 1e-6);
  EXPECT_NEAR(j.velocity, 0, 1e-6);
  EXPECT_NEAR(j.acceleration, 0, 1e-6);

  std::ostringstream s;
  s << j;
  EXPECT_FALSE(s.str().empty());
}

TEST(TesseractSceneGraphUnit, TesseractSceneGraphJointSafetyUnit)  // NOLINT
{
  using namespace tesseract_scene_graph;
  JointSafety j;

  EXPECT_NEAR(j.soft_upper_limit, 0, 1e-6);
  EXPECT_NEAR(j.soft_lower_limit, 0, 1e-6);
  EXPECT_NEAR(j.k_position, 0, 1e-6);
  EXPECT_NEAR(j.k_velocity, 0, 1e-6);

  j.soft_upper_limit = 1;
  j.soft_lower_limit = 2;
  j.k_position = 3;
  j.k_velocity = 4;
  j.clear();

  EXPECT_NEAR(j.soft_upper_limit, 0, 1e-6);
  EXPECT_NEAR(j.soft_lower_limit, 0, 1e-6);
  EXPECT_NEAR(j.k_position, 0, 1e-6);
  EXPECT_NEAR(j.k_velocity, 0, 1e-6);

  std::ostringstream s;
  s << j;
  EXPECT_FALSE(s.str().empty());
}

TEST(TesseractSceneGraphUnit, TesseractSceneGraphJointCalibrationUnit)  // NOLINT
{
  using namespace tesseract_scene_graph;
  JointCalibration j;

  EXPECT_NEAR(j.reference_position, 0, 1e-6);
  EXPECT_NEAR(j.rising, 0, 1e-6);
  EXPECT_NEAR(j.falling, 0, 1e-6);

  j.reference_position = 1;
  j.rising = 2;
  j.falling = 3;
  j.clear();

  EXPECT_NEAR(j.reference_position, 0, 1e-6);
  EXPECT_NEAR(j.rising, 0, 1e-6);
  EXPECT_NEAR(j.falling, 0, 1e-6);

  std::ostringstream s;
  s << j;
  EXPECT_FALSE(s.str().empty());
}

TEST(TesseractSceneGraphUnit, TesseractSceneGraphJointMimicUnit)  // NOLINT
{
  using namespace tesseract_scene_graph;
  JointMimic j;

  EXPECT_NEAR(j.offset, 0, 1e-6);
  EXPECT_NEAR(j.multiplier, 1, 1e-6);
  EXPECT_TRUE(j.joint_name.empty());

  j.offset = 1;
  j.multiplier = 2;
  j.joint_name = "joint_name";
  j.clear();

  EXPECT_NEAR(j.offset, 0, 1e-6);
  EXPECT_NEAR(j.multiplier, 1, 1e-6);
  EXPECT_TRUE(j.joint_name.empty());

  std::ostringstream s;
  s << j;
  EXPECT_FALSE(s.str().empty());
}

TEST(TesseractSceneGraphUnit, TesseractSceneGraphJointUnit)  // NOLINT
{
  using namespace tesseract_scene_graph;

  Joint joint_1("joint_n1");
  EXPECT_TRUE(joint_1.parent_to_joint_origin_transform.isApprox(Eigen::Isometry3d::Identity()));
  EXPECT_TRUE(joint_1.child_link_name.empty());
  EXPECT_TRUE(joint_1.parent_link_name.empty());
  EXPECT_TRUE(joint_1.child_link_name.empty());
  EXPECT_TRUE(joint_1.dynamics == nullptr);
  EXPECT_TRUE(joint_1.limits == nullptr);
  EXPECT_TRUE(joint_1.safety == nullptr);
  EXPECT_TRUE(joint_1.calibration == nullptr);
  EXPECT_TRUE(joint_1.mimic == nullptr);
  EXPECT_TRUE(joint_1.type == JointType::UNKNOWN);

  joint_1.parent_to_joint_origin_transform.translation() = Eigen::Vector3d(1, 2, 3);
  joint_1.parent_link_name = "link_n1";
  joint_1.child_link_name = "link_n2";
  joint_1.axis = Eigen::Vector3d::UnitZ();
  joint_1.type = JointType::PRISMATIC;
  joint_1.dynamics = std::make_shared<JointDynamics>();
  joint_1.dynamics->damping = 0.1;
  joint_1.dynamics->friction = 0.25;
  joint_1.limits = std::make_shared<JointLimits>();
  joint_1.limits->lower = -5;
  joint_1.limits->upper = 5;
  joint_1.limits->effort = 0.5;
  joint_1.limits->velocity = 2;
  joint_1.calibration = std::make_shared<JointCalibration>();
  joint_1.calibration->rising = 0.1;
  joint_1.calibration->falling = 0.1;
  joint_1.mimic = std::make_shared<JointMimic>();
  joint_1.mimic->offset = 0.5;
  joint_1.mimic->joint_name = "joint_0";
  joint_1.mimic->multiplier = 1.5;

  EXPECT_EQ(joint_1.getName(), "joint_n1");

  Joint joint_1_clone = joint_1.clone();
  EXPECT_EQ(joint_1_clone.getName(), "joint_n1");
  EXPECT_EQ(joint_1_clone.parent_link_name, "link_n1");
  EXPECT_EQ(joint_1_clone.child_link_name, "link_n2");
  EXPECT_TRUE(joint_1_clone.parent_to_joint_origin_transform.isApprox(joint_1.parent_to_joint_origin_transform));
  EXPECT_TRUE(joint_1_clone.axis.isApprox(Eigen::Vector3d::UnitZ()));
  EXPECT_EQ(joint_1_clone.type, JointType::PRISMATIC);
  EXPECT_TRUE(joint_1_clone.dynamics != joint_1.dynamics);
  EXPECT_NEAR(joint_1_clone.dynamics->damping, 0.1, 1e-6);
  EXPECT_NEAR(joint_1_clone.dynamics->friction, 0.25, 1e-6);
  EXPECT_TRUE(joint_1_clone.limits != joint_1.limits);
  EXPECT_NEAR(joint_1_clone.limits->lower, -5, 1e-6);
  EXPECT_NEAR(joint_1_clone.limits->upper, 5, 1e-6);
  EXPECT_NEAR(joint_1_clone.limits->effort, 0.5, 1e-6);
  EXPECT_NEAR(joint_1_clone.limits->velocity, 2, 1e-6);
  EXPECT_TRUE(joint_1_clone.calibration != joint_1.calibration);
  EXPECT_NEAR(joint_1_clone.calibration->rising, 0.1, 1e-6);
  EXPECT_NEAR(joint_1_clone.calibration->falling, 0.1, 1e-6);
  EXPECT_TRUE(joint_1_clone.mimic != joint_1.mimic);
  EXPECT_NEAR(joint_1_clone.mimic->offset, 0.5, 1e-6);
  EXPECT_EQ(joint_1_clone.mimic->joint_name, "joint_0");
  EXPECT_NEAR(joint_1_clone.mimic->multiplier, 1.5, 1e-6);

  std::ostringstream s1;
  s1 << JointType::FIXED;
  EXPECT_EQ(s1.str(), "Fixed");
  std::ostringstream s2;
  s2 << JointType::PLANAR;
  EXPECT_EQ(s2.str(), "Planar");
  std::ostringstream s3;
  s3 << JointType::FLOATING;
  EXPECT_EQ(s3.str(), "Floating");
  std::ostringstream s4;
  s4 << JointType::REVOLUTE;
  EXPECT_EQ(s4.str(), "Revolute");
  std::ostringstream s5;
  s5 << JointType::PRISMATIC;
  EXPECT_EQ(s5.str(), "Prismatic");
  std::ostringstream s6;
  s6 << JointType::CONTINUOUS;
  EXPECT_EQ(s6.str(), "Continuous");
  std::ostringstream s7;
  s7 << JointType::UNKNOWN;
  EXPECT_EQ(s7.str(), "Unknown");

  joint_1.clear();
  EXPECT_EQ(joint_1.getName(), "joint_n1");
  EXPECT_TRUE(joint_1.parent_to_joint_origin_transform.isApprox(Eigen::Isometry3d::Identity()));
  EXPECT_TRUE(joint_1.child_link_name.empty());
  EXPECT_TRUE(joint_1.parent_link_name.empty());
  EXPECT_TRUE(joint_1.child_link_name.empty());
  EXPECT_TRUE(joint_1.dynamics == nullptr);
  EXPECT_TRUE(joint_1.limits == nullptr);
  EXPECT_TRUE(joint_1.safety == nullptr);
  EXPECT_TRUE(joint_1.calibration == nullptr);
  EXPECT_TRUE(joint_1.mimic == nullptr);
  EXPECT_TRUE(joint_1.type == JointType::UNKNOWN);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
