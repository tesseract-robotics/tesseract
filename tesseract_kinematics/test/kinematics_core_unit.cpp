#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_kinematics/kdl/kdl_fwd_kin_chain.h>
#include <tesseract_kinematics/core/utils.h>
#include "kinematics_test_utils.h"

const static std::string FACTORY_NAME = "TestFactory";

TEST(TesseractKinematicsUnit, UtilsHarmonizeUnit)  // NOLINT
{
  Eigen::VectorXd q(2);
  q[0] = (4 * M_PI) + M_PI_4;
  q[1] = -(4 * M_PI) - M_PI_4;

  tesseract_kinematics::harmonizeTowardZero<double>(q, { 0, 1 });
  EXPECT_NEAR(q[0], M_PI_4, 1e-6);
  EXPECT_NEAR(q[1], -M_PI_4, 1e-6);

  q[0] = M_PI_4;
  q[1] = -M_PI_4;

  tesseract_kinematics::harmonizeTowardZero<double>(q, { 0, 1 });
  EXPECT_NEAR(q[0], M_PI_4, 1e-6);
  EXPECT_NEAR(q[1], -M_PI_4, 1e-6);

  q[0] = 5 * M_PI_4;
  q[1] = -5 * M_PI_4;

  tesseract_kinematics::harmonizeTowardZero<double>(q, { 0, 1 });
  EXPECT_NEAR(q[0], -3 * M_PI_4, 1e-6);
  EXPECT_NEAR(q[1], 3 * M_PI_4, 1e-6);
}

template <typename FloatType>
void runRedundantSolutionsTest()
{
  // Helper function for checking if all redundant solutions are unique
  auto expect_unique_solutions = [](const std::vector<tesseract_kinematics::VectorX<FloatType>>& solutions) {
    for (auto sol_1 = solutions.begin(); sol_1 != solutions.end() - 1; ++sol_1)
    {
      for (auto sol_2 = sol_1 + 1; sol_2 != solutions.end(); ++sol_2)
      {
        EXPECT_FALSE(tesseract_common::almostEqualRelativeAndAbs(
            sol_1->template cast<double>(), sol_2->template cast<double>(), 1e-6));
      }
    }
  };

  {
    double max_diff = 1e-6;
    Eigen::MatrixX2d limits(3, 2);
    limits << 0, 2.0 * M_PI, 0, 2.0 * M_PI, 0, 2.0 * M_PI;
    std::vector<Eigen::Index> redundancy_capable_joints = { 0, 1, 2 };

    tesseract_kinematics::VectorX<FloatType> q(3);
    q << 0, 0, 0;

    {  // Test when initial solution is at the lower limit
      std::vector<tesseract_kinematics::VectorX<FloatType>> solutions =
          tesseract_kinematics::getRedundantSolutions<FloatType>(q, limits, redundancy_capable_joints);
      if (tesseract_common::satisfiesPositionLimits(q.template cast<double>(), limits, max_diff))
        solutions.push_back(q);

      EXPECT_EQ(solutions.size(), 8);
      expect_unique_solutions(solutions);
    }

    {  // Test when initial solution is within the limits
      limits << -2.0 * M_PI, 2.0 * M_PI, -2.0 * M_PI, 2.0 * M_PI, -2.0 * M_PI, 2.0 * M_PI;
      std::vector<tesseract_kinematics::VectorX<FloatType>> solutions =
          tesseract_kinematics::getRedundantSolutions<FloatType>(q, limits, redundancy_capable_joints);
      if (tesseract_common::satisfiesPositionLimits(q.template cast<double>(), limits, max_diff))
        solutions.push_back(q);

      EXPECT_EQ(solutions.size(), 27);
      expect_unique_solutions(solutions);
    }

    {  // Test when the initial solution outside the lower and upper limit
      limits << -2.0 * M_PI, 2.0 * M_PI, -2.0 * M_PI, 2.0 * M_PI, -2.0 * M_PI, 2.0 * M_PI;
      q << static_cast<FloatType>(-4.0 * M_PI), static_cast<FloatType>(-4.0 * M_PI), static_cast<FloatType>(4.0 * M_PI);

      std::vector<tesseract_kinematics::VectorX<FloatType>> solutions =
          tesseract_kinematics::getRedundantSolutions<FloatType>(q, limits, redundancy_capable_joints);
      if (tesseract_common::satisfiesPositionLimits(q.template cast<double>(), limits, max_diff))
        solutions.push_back(q);

      EXPECT_EQ(solutions.size(), 27);
      expect_unique_solutions(solutions);
    }
  }

  {  // Test case when not all joints are redundancy capable
    double max_diff = 1.0e-6;
    Eigen::MatrixX2d limits(4, 2);
    limits << -2.0 * M_PI, 2.0 * M_PI, -2.0 * M_PI, 2.0 * M_PI, -2.0 * M_PI, 2.0 * M_PI, -2.0 * M_PI, 2.0 * M_PI;

    tesseract_kinematics::VectorX<FloatType> q(4);
    q << static_cast<FloatType>(-4.0 * M_PI), static_cast<FloatType>(-4.0 * M_PI), static_cast<FloatType>(0.0),
        static_cast<FloatType>(4.0 * M_PI);

    // Arbitrarily decide that joint 2 is not redundancy capable
    std::vector<Eigen::Index> redundancy_capable_joints = { 0, 1, 3 };

    std::vector<tesseract_kinematics::VectorX<FloatType>> solutions =
        tesseract_kinematics::getRedundantSolutions<FloatType>(q, limits, redundancy_capable_joints);
    if (tesseract_common::satisfiesPositionLimits(q.template cast<double>(), limits, max_diff))
      solutions.push_back(q);

    EXPECT_EQ(solutions.size(), 27);
    expect_unique_solutions(solutions);
  }

  {  // Edge-case tests
    Eigen::MatrixX2d limits(4, 2);
    limits << -2.0 * M_PI, 2.0 * M_PI, -2.0 * M_PI, 2.0 * M_PI, -2.0 * M_PI, 2.0 * M_PI, -2.0 * M_PI, 2.0 * M_PI;

    tesseract_kinematics::VectorX<FloatType> q(4);
    q << static_cast<FloatType>(-4.0 * M_PI), static_cast<FloatType>(-4.0 * M_PI), static_cast<FloatType>(0.0),
        static_cast<FloatType>(4.0 * M_PI);

    std::vector<Eigen::Index> redundancy_capable_joints = {};
    std::vector<tesseract_kinematics::VectorX<FloatType>> solutions =
        tesseract_kinematics::getRedundantSolutions<FloatType>(q, limits, redundancy_capable_joints);

    EXPECT_EQ(solutions.size(), 0);

    redundancy_capable_joints = { 10 };

    // NOLINTNEXTLINE
    EXPECT_THROW(tesseract_kinematics::getRedundantSolutions<FloatType>(q, limits, redundancy_capable_joints),
                 std::runtime_error);
  }
}

TEST(TesseractKinematicsUnit, RedundantSolutionsUnit)  // NOLINT
{
  runRedundantSolutionsTest<float>();
  runRedundantSolutionsTest<double>();
}

TEST(TesseractKinematicsUnit, UtilsNearSingularityUnit)  // NOLINT
{
  tesseract_scene_graph::SceneGraph::Ptr scene_graph = tesseract_kinematics::test_suite::getSceneGraphABB();

  tesseract_kinematics::KDLFwdKinChain fwd_kin(*scene_graph, "base_link", "tool0");

  // First test joint 4, 5 and 6 at zero which should be in a singularity
  Eigen::VectorXd jv = Eigen::VectorXd::Zero(6);
  Eigen::MatrixXd jacobian = fwd_kin.calcJacobian(jv, "tool0");
  EXPECT_TRUE(tesseract_kinematics::isNearSingularity(jacobian, 0.001));

  // Set joint 5 angle to 1 deg and it with the default threshold it should still be in singularity
  jv[4] = 1 * M_PI / 180.0;
  jacobian = fwd_kin.calcJacobian(jv, "tool0");
  EXPECT_TRUE(tesseract_kinematics::isNearSingularity(jacobian));

  // Set joint 5 angle to 2 deg and it should no longer be in a singularity
  jv[4] = 2 * M_PI / 180.0;
  jacobian = fwd_kin.calcJacobian(jv, "tool0");
  EXPECT_FALSE(tesseract_kinematics::isNearSingularity(jacobian));

  // Increase threshold and now with joint 5 at 2 deg it will now be considered in a singularity
  EXPECT_TRUE(tesseract_kinematics::isNearSingularity(jacobian, 0.02));
}

TEST(TesseractKinematicsUnit, UtilscalcManipulabilityUnit)  // NOLINT
{
  tesseract_scene_graph::SceneGraph::Ptr scene_graph = tesseract_kinematics::test_suite::getSceneGraphABB();

  tesseract_kinematics::KDLFwdKinChain fwd_kin(*scene_graph, "base_link", "tool0");

  // First test joint 4, 5 and 6 at zero which should be in a singularity
  Eigen::VectorXd jv = Eigen::VectorXd::Zero(6);
  Eigen::MatrixXd jacobian = fwd_kin.calcJacobian(jv, "tool0");
  tesseract_kinematics::Manipulability m = tesseract_kinematics::calcManipulability(jacobian);
  EXPECT_EQ(m.m.eigen_values.size(), 6);
  EXPECT_NEAR(m.m.volume, 0, 1e-6);
  EXPECT_GT(m.m.condition, 1e+20);

  EXPECT_EQ(m.m_linear.eigen_values.size(), 3);
  EXPECT_NEAR(m.m_linear.eigen_values[0], 0.18153054745434696, 1e-6);
  EXPECT_NEAR(m.m_linear.eigen_values[1], 0.8835999999999999, 1e-6);
  EXPECT_NEAR(m.m_linear.eigen_values[2], 1.960719452545653, 1e-6);
  EXPECT_NEAR(m.m_linear.condition, 10.801044122002406, 1e-6);
  EXPECT_NEAR(m.m_linear.measure, 3.286494199295414, 1e-6);
  EXPECT_NEAR(m.m_linear.volume, 0.5608031457314142, 1e-6);

  EXPECT_EQ(m.m_angular.eigen_values.size(), 3);
  EXPECT_NEAR(m.m_angular.eigen_values[0], 1.0, 1e-6);
  EXPECT_NEAR(m.m_angular.eigen_values[1], 2.0, 1e-6);
  EXPECT_NEAR(m.m_angular.eigen_values[2], 3.0, 1e-6);
  EXPECT_NEAR(m.m_angular.condition, 3.0, 1e-6);
  EXPECT_NEAR(m.m_angular.measure, 1.7320508075688772, 1e-6);
  EXPECT_NEAR(m.m_angular.volume, 2.449489742783178, 1e-6);

  EXPECT_EQ(m.f.eigen_values.size(), 6);
  EXPECT_NEAR(m.m.volume, 0, 1e-6);
  EXPECT_GT(m.m.condition, 1e+20);

  EXPECT_EQ(m.f_linear.eigen_values.size(), 3);
  EXPECT_NEAR(m.f_linear.eigen_values[0], 0.5100168709509535, 1e-6);
  EXPECT_NEAR(m.f_linear.eigen_values[1], 1.1317338162064283, 1e-6);
  EXPECT_NEAR(m.f_linear.eigen_values[2], 5.508714726106856, 1e-6);
  EXPECT_NEAR(m.f_linear.condition, 10.801044122002406, 1e-6);
  EXPECT_NEAR(m.f_linear.measure, 3.286494199295414, 1e-6);
  EXPECT_NEAR(m.f_linear.volume, 1.783156902045858, 1e-6);

  EXPECT_EQ(m.f_angular.eigen_values.size(), 3);
  EXPECT_NEAR(m.f_angular.eigen_values[0], 0.3333333333333333, 1e-6);
  EXPECT_NEAR(m.f_angular.eigen_values[1], 0.5, 1e-6);
  EXPECT_NEAR(m.f_angular.eigen_values[2], 1.0, 1e-6);
  EXPECT_NEAR(m.f_angular.condition, 3.0, 1e-6);
  EXPECT_NEAR(m.f_angular.measure, 1.7320508075688774, 1e-6);
  EXPECT_NEAR(m.f_angular.volume, 0.408248290463863, 1e-6);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
