#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_kinematics/core/forward_kinematics_factory.h>
#include <tesseract_kinematics/core/inverse_kinematics_factory.h>
#include <tesseract_kinematics/kdl/kdl_fwd_kin_chain.h>
#include <tesseract_kinematics/core/utils.h>
#include "kinematics_test_utils.h"

const static std::string FACTORY_NAME = "TestFactory";

class TestForwardKinematicsFactory : public tesseract_kinematics::ForwardKinematicsFactory
{
  const std::string& getName() const override { return FACTORY_NAME; }
  tesseract_kinematics::ForwardKinematicsFactoryType getType() const override
  {
    return tesseract_kinematics::ForwardKinematicsFactoryType::CHAIN;
  }
};

class TestInverseKinematicsFactory : public tesseract_kinematics::InverseKinematicsFactory
{
  const std::string& getName() const override { return FACTORY_NAME; }
  tesseract_kinematics::InverseKinematicsFactoryType getType() const override
  {
    return tesseract_kinematics::InverseKinematicsFactoryType::CHAIN;
  }
};

TEST(TesseractKinematicsUnit, CoreFactoryUnit)  // NOLINT
{
  using namespace tesseract_kinematics;
  TestForwardKinematicsFactory test_fwd_factory;
  EXPECT_TRUE(test_fwd_factory.create(nullptr, "", "", "") == nullptr);
  EXPECT_TRUE(test_fwd_factory.create(nullptr, std::vector<std::pair<std::string, std::string>>(), "") == nullptr);
  EXPECT_TRUE(test_fwd_factory.create(nullptr, std::vector<std::string>(), "") == nullptr);

  TestInverseKinematicsFactory test_inv_factory;
  EXPECT_TRUE(test_inv_factory.create(nullptr, "", "", "") == nullptr);
  EXPECT_TRUE(test_inv_factory.create(nullptr, std::vector<std::pair<std::string, std::string>>(), "") == nullptr);
  EXPECT_TRUE(test_inv_factory.create(nullptr, std::vector<std::string>(), "") == nullptr);
}

TEST(TesseractKinematicsUnit, CoreUtilsWithinLimitsUnit)  // NOLINT
{
  Eigen::VectorXd joint_values;
  joint_values.resize(2);
  joint_values(0) = 0;
  joint_values(1) = 0;

  Eigen::MatrixX2d limits;
  limits.resize(2, 2);
  limits(0, 0) = -1;
  limits(0, 1) = 1;
  limits(1, 0) = -1;
  limits(1, 1) = 1;

  EXPECT_TRUE(tesseract_kinematics::isWithinLimits<double>(joint_values, limits));

  limits(0, 0) = .5;
  limits(0, 1) = 1;
  limits(1, 0) = .5;
  limits(1, 1) = 1;
  EXPECT_FALSE(tesseract_kinematics::isWithinLimits<double>(joint_values, limits));
}

TEST(TesseractKinematicsUnit, UtilsHarmonizeUnit)  // NOLINT
{
  Eigen::VectorXd q(2);
  q[0] = (4 * M_PI) + M_PI_4;
  q[1] = -(4 * M_PI) - M_PI_4;

  tesseract_kinematics::harmonizeTowardZero<double>(q);
  EXPECT_NEAR(q[0], M_PI_4, 1e-6);
  EXPECT_NEAR(q[1], -M_PI_4, 1e-6);

  q[0] = M_PI_4;
  q[1] = -M_PI_4;

  tesseract_kinematics::harmonizeTowardZero<double>(q);
  EXPECT_NEAR(q[0], M_PI_4, 1e-6);
  EXPECT_NEAR(q[1], -M_PI_4, 1e-6);

  q[0] = 5 * M_PI_4;
  q[1] = -5 * M_PI_4;

  tesseract_kinematics::harmonizeTowardZero<double>(q);
  EXPECT_NEAR(q[0], -3 * M_PI_4, 1e-6);
  EXPECT_NEAR(q[1], 3 * M_PI_4, 1e-6);
}

TEST(TesseractKinematicsUnit, UtilsNearSingularityUnit)  // NOLINT
{
  tesseract_scene_graph::SceneGraph::Ptr scene_graph = tesseract_kinematics::test_suite::getSceneGraphABB();

  tesseract_kinematics::KDLFwdKinChain fwd_kin;
  fwd_kin.init(scene_graph, "base_link", "tool0", "manip");

  // First test joint 4, 5 and 6 at zero which should be in a singularity
  Eigen::VectorXd jv = Eigen::VectorXd::Zero(6);
  Eigen::MatrixXd jacobian = fwd_kin.calcJacobian(jv);
  EXPECT_TRUE(tesseract_kinematics::isNearSingularity(jacobian, 0.001));

  // Set joint 5 angle to 1 deg and it with the default threshold it should still be in singularity
  jv[4] = 1 * M_PI / 180.0;
  jacobian = fwd_kin.calcJacobian(jv);
  EXPECT_TRUE(tesseract_kinematics::isNearSingularity(jacobian));

  // Set joint 5 angle to 2 deg and it should no longer be in a singularity
  jv[4] = 2 * M_PI / 180.0;
  jacobian = fwd_kin.calcJacobian(jv);
  EXPECT_FALSE(tesseract_kinematics::isNearSingularity(jacobian));

  // Increase threshold and now with joint 5 at 2 deg it will now be considered in a singularity
  EXPECT_TRUE(tesseract_kinematics::isNearSingularity(jacobian, 0.02));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
