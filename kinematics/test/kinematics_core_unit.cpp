#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/kinematics/kdl/kdl_fwd_kin_chain.h>
#include <tesseract/kinematics/rtp_inv_kin.h>
#include <tesseract/kinematics/utils.h>
#include <tesseract/scene_graph/graph.h>
#include <tesseract/scene_graph/joint.h>
#include <tesseract/scene_graph/link.h>
#include "kinematics_test_utils.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <random>

const static std::string FACTORY_NAME = "TestFactory";

TEST(TesseractKinematicsUnit, UtilsHarmonizeTowardZeroUnit)  // NOLINT
{
  Eigen::VectorXd q(2);
  q[0] = (4 * M_PI) + M_PI_4;
  q[1] = -(4 * M_PI) - M_PI_4;

  tesseract::kinematics::harmonizeTowardZero<double>(q, { 0, 1 });
  EXPECT_NEAR(q[0], M_PI_4, 1e-6);
  EXPECT_NEAR(q[1], -M_PI_4, 1e-6);

  q[0] = M_PI_4;
  q[1] = -M_PI_4;

  tesseract::kinematics::harmonizeTowardZero<double>(q, { 0, 1 });
  EXPECT_NEAR(q[0], M_PI_4, 1e-6);
  EXPECT_NEAR(q[1], -M_PI_4, 1e-6);

  q[0] = 5 * M_PI_4;
  q[1] = -5 * M_PI_4;

  tesseract::kinematics::harmonizeTowardZero<double>(q, { 0, 1 });
  EXPECT_NEAR(q[0], -3 * M_PI_4, 1e-6);
  EXPECT_NEAR(q[1], 3 * M_PI_4, 1e-6);
}

TEST(TesseractKinematicsUnit, UtilsHarmonizeTowardMedianUnit)  // NOLINT
{
  Eigen::MatrixX2d c(2, 2);
  c(0, 0) = -M_PI;
  c(0, 1) = +M_PI;
  c(1, 0) = -M_PI;
  c(1, 1) = +M_PI;
  Eigen::VectorXd m(2);
  m[0] = (c(0, 0) + c(0, 1)) / 2.0;
  m[1] = (c(1, 0) + c(1, 1)) / 2.0;

  Eigen::VectorXd q(2);
  q[0] = (4 * M_PI) + M_PI_4;
  q[1] = -(4 * M_PI) - M_PI_4;

  tesseract::kinematics::harmonizeTowardMedian<double>(q, { 0, 1 }, c);
  EXPECT_NEAR(q[0], M_PI_4, 1e-6);
  EXPECT_NEAR(q[1], -M_PI_4, 1e-6);
  EXPECT_TRUE(std::abs(q[0] - m[0]) < (M_PI + 1e-6));
  EXPECT_TRUE(std::abs(q[1] - m[1]) < (M_PI + 1e-6));

  q[0] = M_PI_4;
  q[1] = -M_PI_4;

  tesseract::kinematics::harmonizeTowardMedian<double>(q, { 0, 1 }, c);
  EXPECT_NEAR(q[0], M_PI_4, 1e-6);
  EXPECT_NEAR(q[1], -M_PI_4, 1e-6);
  EXPECT_TRUE(std::abs(q[0] - m[0]) < (M_PI + 1e-6));
  EXPECT_TRUE(std::abs(q[1] - m[1]) < (M_PI + 1e-6));

  q[0] = 5 * M_PI_4;
  q[1] = -5 * M_PI_4;

  tesseract::kinematics::harmonizeTowardMedian<double>(q, { 0, 1 }, c);
  EXPECT_NEAR(q[0], -3 * M_PI_4, 1e-6);
  EXPECT_NEAR(q[1], 3 * M_PI_4, 1e-6);
  EXPECT_TRUE(std::abs(q[0] - m[0]) < (M_PI + 1e-6));
  EXPECT_TRUE(std::abs(q[1] - m[1]) < (M_PI + 1e-6));

  // NON Zero Positive Constant
  c(0, 0) = (10 * M_PI) + M_PI_4 - M_PI;
  c(0, 1) = (10 * M_PI) + M_PI_4 + M_PI;
  c(1, 0) = (10 * M_PI) + M_PI_4 - M_PI;
  c(1, 1) = (10 * M_PI) + M_PI_4 + M_PI;
  m[0] = (c(0, 0) + c(0, 1)) / 2.0;
  m[1] = (c(1, 0) + c(1, 1)) / 2.0;

  q[0] = (4 * M_PI) + M_PI_4;
  q[1] = -(4 * M_PI) - M_PI_4;

  tesseract::kinematics::harmonizeTowardMedian<double>(q, { 0, 1 }, c);
  EXPECT_NEAR(q[0], m[0], 1e-6);
  EXPECT_NEAR(q[1], m[1] - M_PI_2, 1e-6);
  EXPECT_TRUE(std::abs(q[0] - m[0]) < (M_PI + 1e-6));
  EXPECT_TRUE(std::abs(q[1] - m[1]) < (M_PI + 1e-6));

  q[0] = M_PI_4;
  q[1] = -M_PI_4;

  tesseract::kinematics::harmonizeTowardMedian<double>(q, { 0, 1 }, c);
  EXPECT_NEAR(q[0], m[0], 1e-6);
  EXPECT_NEAR(q[1], m[1] - M_PI_2, 1e-6);
  EXPECT_TRUE(std::abs(q[0] - m[0]) < (M_PI + 1e-6));
  EXPECT_TRUE(std::abs(q[1] - m[1]) < (M_PI + 1e-6));

  q[0] = 5 * M_PI_4;
  q[1] = -5 * M_PI_4;

  tesseract::kinematics::harmonizeTowardMedian<double>(q, { 0, 1 }, c);
  EXPECT_NEAR(q[0], m[0] - M_PI, 1e-6);
  EXPECT_NEAR(q[1], m[1] + M_PI_2, 1e-6);
  EXPECT_TRUE(std::abs(q[0] - m[0]) < (M_PI + 1e-6));
  EXPECT_TRUE(std::abs(q[1] - m[1]) < (M_PI + 1e-6));

  // NON Zero Negative Constant
  c(0, 0) = (-1 * ((10 * M_PI) + M_PI_4)) - M_PI;
  c(0, 1) = (-1 * ((10 * M_PI) + M_PI_4)) + M_PI;
  c(1, 0) = (-1 * ((10 * M_PI) + M_PI_4)) - M_PI;
  c(1, 1) = (-1 * ((10 * M_PI) + M_PI_4)) + M_PI;
  m[0] = (c(0, 0) + c(0, 1)) / 2.0;
  m[1] = (c(1, 0) + c(1, 1)) / 2.0;

  q[0] = (4 * M_PI) + M_PI_4;
  q[1] = -(4 * M_PI) - M_PI_4;

  tesseract::kinematics::harmonizeTowardMedian<double>(q, { 0, 1 }, c);
  EXPECT_NEAR(q[0], m[0] + M_PI_2, 1e-6);
  EXPECT_NEAR(q[1], m[1], 1e-6);
  EXPECT_TRUE(std::abs(q[0] - m[0]) < (M_PI + 1e-6));
  EXPECT_TRUE(std::abs(q[1] - m[1]) < (M_PI + 1e-6));

  q[0] = M_PI_4;
  q[1] = -M_PI_4;

  tesseract::kinematics::harmonizeTowardMedian<double>(q, { 0, 1 }, c);
  EXPECT_NEAR(q[0], m[0] + M_PI_2, 1e-6);
  EXPECT_NEAR(q[1], m[1], 1e-6);
  EXPECT_TRUE(std::abs(q[0] - m[0]) < (M_PI + 1e-6));
  EXPECT_TRUE(std::abs(q[1] - m[1]) < (M_PI + 1e-6));

  q[0] = 5 * M_PI_4;
  q[1] = -5 * M_PI_4;

  tesseract::kinematics::harmonizeTowardMedian<double>(q, { 0, 1 }, c);
  EXPECT_NEAR(q[0], m[0] - M_PI_2, 1e-6);
  EXPECT_NEAR(q[1], m[1] + M_PI, 1e-6);
  EXPECT_TRUE(std::abs(q[0] - m[0]) < (M_PI + 1e-6));
  EXPECT_TRUE(std::abs(q[1] - m[1]) < (M_PI + 1e-6));
}

template <typename FloatType>
void runRedundantSolutionsTest()
{
  // Helper function for checking if all redundant solutions are unique
  auto expect_unique_solutions = [](const std::vector<tesseract::kinematics::VectorX<FloatType>>& solutions) {
    for (auto sol_1 = solutions.begin(); sol_1 != solutions.end() - 1; ++sol_1)
    {
      for (auto sol_2 = sol_1 + 1; sol_2 != solutions.end(); ++sol_2)
      {
        EXPECT_FALSE(tesseract::common::almostEqualRelativeAndAbs(
            sol_1->template cast<double>(), sol_2->template cast<double>(), 1e-6));
      }
    }
  };

  {
    double max_diff = 1e-6;
    Eigen::MatrixX2d limits(3, 2);
    limits << 0, 2.0 * M_PI, 0, 2.0 * M_PI, 0, 2.0 * M_PI;
    std::vector<Eigen::Index> redundancy_capable_joints = { 0, 1, 2 };

    tesseract::kinematics::VectorX<FloatType> q(3);
    q << 0, 0, 0;

    {  // Test when initial solution is at the lower limit
      std::vector<tesseract::kinematics::VectorX<FloatType>> solutions =
          tesseract::kinematics::getRedundantSolutions<FloatType>(q, limits, redundancy_capable_joints);
      if (tesseract::common::satisfiesLimits<double>(q.template cast<double>(), limits, max_diff))
        solutions.push_back(q);

      EXPECT_EQ(solutions.size(), 8);
      expect_unique_solutions(solutions);
    }

    {  // Test when initial solution is within the limits
      limits << -2.0 * M_PI, 2.0 * M_PI, -2.0 * M_PI, 2.0 * M_PI, -2.0 * M_PI, 2.0 * M_PI;
      std::vector<tesseract::kinematics::VectorX<FloatType>> solutions =
          tesseract::kinematics::getRedundantSolutions<FloatType>(q, limits, redundancy_capable_joints);
      if (tesseract::common::satisfiesLimits<double>(q.template cast<double>(), limits, max_diff))
        solutions.push_back(q);

      EXPECT_EQ(solutions.size(), 27);
      expect_unique_solutions(solutions);
    }

    {  // Test when the initial solution outside the lower and upper limit
      limits << -2.0 * M_PI, 2.0 * M_PI, -2.0 * M_PI, 2.0 * M_PI, -2.0 * M_PI, 2.0 * M_PI;
      q << static_cast<FloatType>(-4.0 * M_PI), static_cast<FloatType>(-4.0 * M_PI), static_cast<FloatType>(4.0 * M_PI);

      std::vector<tesseract::kinematics::VectorX<FloatType>> solutions =
          tesseract::kinematics::getRedundantSolutions<FloatType>(q, limits, redundancy_capable_joints);
      if (tesseract::common::satisfiesLimits<double>(q.template cast<double>(), limits, max_diff))
        solutions.push_back(q);

      EXPECT_EQ(solutions.size(), 27);
      expect_unique_solutions(solutions);
    }
  }

  {  // Test case when not all joints are redundancy capable
    double max_diff = 1.0e-6;
    Eigen::MatrixX2d limits(4, 2);
    limits << -2.0 * M_PI, 2.0 * M_PI, -2.0 * M_PI, 2.0 * M_PI, -2.0 * M_PI, 2.0 * M_PI, -2.0 * M_PI, 2.0 * M_PI;

    tesseract::kinematics::VectorX<FloatType> q(4);
    q << static_cast<FloatType>(-4.0 * M_PI), static_cast<FloatType>(-4.0 * M_PI), static_cast<FloatType>(0.0),
        static_cast<FloatType>(4.0 * M_PI);

    // Arbitrarily decide that joint 2 is not redundancy capable
    std::vector<Eigen::Index> redundancy_capable_joints = { 0, 1, 3 };

    std::vector<tesseract::kinematics::VectorX<FloatType>> solutions =
        tesseract::kinematics::getRedundantSolutions<FloatType>(q, limits, redundancy_capable_joints);
    if (tesseract::common::satisfiesLimits<double>(q.template cast<double>(), limits, max_diff))
      solutions.push_back(q);

    EXPECT_EQ(solutions.size(), 27);
    expect_unique_solutions(solutions);
  }

  {  // Edge-case tests
    Eigen::MatrixX2d limits(4, 2);
    limits << -2.0 * M_PI, 2.0 * M_PI, -2.0 * M_PI, 2.0 * M_PI, -2.0 * M_PI, 2.0 * M_PI, -2.0 * M_PI, 2.0 * M_PI;

    tesseract::kinematics::VectorX<FloatType> q(4);
    q << static_cast<FloatType>(-4.0 * M_PI), static_cast<FloatType>(-4.0 * M_PI), static_cast<FloatType>(0.0),
        static_cast<FloatType>(4.0 * M_PI);

    std::vector<Eigen::Index> redundancy_capable_joints = {};
    std::vector<tesseract::kinematics::VectorX<FloatType>> solutions =
        tesseract::kinematics::getRedundantSolutions<FloatType>(q, limits, redundancy_capable_joints);

    EXPECT_EQ(solutions.size(), 0);

    redundancy_capable_joints = { 10 };

    // NOLINTNEXTLINE
    EXPECT_THROW(tesseract::kinematics::getRedundantSolutions<FloatType>(q, limits, redundancy_capable_joints),
                 std::runtime_error);
  }

  {  // Not finit lower
    Eigen::MatrixX2d limits(4, 2);
    limits << -std::numeric_limits<double>::infinity(), 2.0 * M_PI, -2.0 * M_PI, 2.0 * M_PI, -2.0 * M_PI, 2.0 * M_PI,
        -2.0 * M_PI, 2.0 * M_PI;

    tesseract::kinematics::VectorX<FloatType> q(4);
    q << static_cast<FloatType>(-4.0 * M_PI), static_cast<FloatType>(-4.0 * M_PI), static_cast<FloatType>(0.0),
        static_cast<FloatType>(4.0 * M_PI);

    std::vector<Eigen::Index> redundancy_capable_joints = { 0 };
    std::vector<tesseract::kinematics::VectorX<FloatType>> solutions =
        tesseract::kinematics::getRedundantSolutions<FloatType>(q, limits, redundancy_capable_joints);

    EXPECT_EQ(solutions.size(), 0);
  }

  {  // Not finit upper
    Eigen::MatrixX2d limits(4, 2);
    limits << -2.0 * M_PI, std::numeric_limits<double>::infinity(), -2.0 * M_PI, 2.0 * M_PI, -2.0 * M_PI, 2.0 * M_PI,
        -2.0 * M_PI, 2.0 * M_PI;

    tesseract::kinematics::VectorX<FloatType> q(4);
    q << static_cast<FloatType>(-4.0 * M_PI), static_cast<FloatType>(-4.0 * M_PI), static_cast<FloatType>(0.0),
        static_cast<FloatType>(4.0 * M_PI);

    std::vector<Eigen::Index> redundancy_capable_joints = { 0, 1, 3 };
    std::vector<tesseract::kinematics::VectorX<FloatType>> solutions =
        tesseract::kinematics::getRedundantSolutions<FloatType>(q, limits, redundancy_capable_joints);

    EXPECT_EQ(solutions.size(), 0);
  }
}

TEST(TesseractKinematicsUnit, RedundantSolutionsUnit)  // NOLINT
{
  runRedundantSolutionsTest<float>();
  runRedundantSolutionsTest<double>();
}

TEST(TesseractKinematicsUnit, UtilsNearSingularityUnit)  // NOLINT
{
  tesseract::common::GeneralResourceLocator locator;
  tesseract::scene_graph::SceneGraph::Ptr scene_graph = tesseract::kinematics::test_suite::getSceneGraphABB(locator);

  tesseract::kinematics::KDLFwdKinChain fwd_kin(*scene_graph, "base_link", "tool0");

  // First test joint 4, 5 and 6 at zero which should be in a singularity
  Eigen::VectorXd jv = Eigen::VectorXd::Zero(6);
  Eigen::MatrixXd jacobian(6, fwd_kin.numJoints());
  fwd_kin.calcJacobian(jacobian, jv, "tool0");
  EXPECT_TRUE(tesseract::kinematics::isNearSingularity(jacobian, 0.001));

  // Set joint 5 angle to 1 deg and it with the default threshold it should still be in singularity
  jv[4] = 1 * M_PI / 180.0;
  fwd_kin.calcJacobian(jacobian, jv, "tool0");
  EXPECT_TRUE(tesseract::kinematics::isNearSingularity(jacobian));

  // Set joint 5 angle to 2 deg and it should no longer be in a singularity
  jv[4] = 2 * M_PI / 180.0;
  fwd_kin.calcJacobian(jacobian, jv, "tool0");
  EXPECT_FALSE(tesseract::kinematics::isNearSingularity(jacobian));

  // Increase threshold and now with joint 5 at 2 deg it will now be considered in a singularity
  EXPECT_TRUE(tesseract::kinematics::isNearSingularity(jacobian, 0.02));
}

TEST(TesseractKinematicsUnit, UtilscalcManipulabilityUnit)  // NOLINT
{
  tesseract::common::GeneralResourceLocator locator;
  tesseract::scene_graph::SceneGraph::Ptr scene_graph = tesseract::kinematics::test_suite::getSceneGraphABB(locator);

  tesseract::kinematics::KDLFwdKinChain fwd_kin(*scene_graph, "base_link", "tool0");

  // First test joint 4, 5 and 6 at zero which should be in a singularity
  Eigen::VectorXd jv = Eigen::VectorXd::Zero(6);
  Eigen::MatrixXd jacobian(6, fwd_kin.numJoints());
  fwd_kin.calcJacobian(jacobian, jv, "tool0");
  tesseract::kinematics::Manipulability m = tesseract::kinematics::calcManipulability(jacobian);
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

TEST(TesseractKinematicsUnit, solvePInv_OverdeterminedSystem)
{
  Eigen::MatrixXd A(4, 2);
  A << 1, 2, 3, 4, 5, 6, 7, 8;

  Eigen::VectorXd b(4);
  b << 1, 2, 3, 4;

  Eigen::VectorXd x(A.cols());
  bool success = tesseract::kinematics::solvePInv(A, b, x);

  EXPECT_TRUE(success);
  EXPECT_EQ(x.size(), 2);

  // Check solution approximately satisfies Ax ≈ b
  Eigen::VectorXd b_approx = A * x;
  EXPECT_TRUE((b - b_approx).norm() < 1e-4);
}

TEST(TesseractKinematicsUnit, solvePInv_UnderdeterminedSystem)
{
  Eigen::MatrixXd A(2, 4);
  A << 1, 2, 3, 4, 5, 6, 7, 8;

  Eigen::VectorXd b(2);
  b << 1, 2;

  Eigen::VectorXd x(A.cols());
  bool success = tesseract::kinematics::solvePInv(A, b, x);

  EXPECT_TRUE(success);
  EXPECT_EQ(x.size(), 4);

  // Check solution approximately satisfies Ax ≈ b
  Eigen::VectorXd b_approx = A * x;
  EXPECT_TRUE((b - b_approx).norm() < 1e-4);
}

TEST(TesseractKinematicsUnit, solvePInv_SizeMismatch)
{
  Eigen::MatrixXd A(3, 3);
  A << 1, 2, 3, 4, 5, 6, 7, 8, 9;

  Eigen::VectorXd b(2);  // Wrong size

  Eigen::VectorXd x(3);
  bool success = tesseract::kinematics::solvePInv(A, b, x);
  EXPECT_FALSE(success);
}

TEST(TesseractKinematicsUnit, solvePInv_EmptyMatrix)
{
  Eigen::MatrixXd A(0, 0);
  Eigen::VectorXd b(0);
  Eigen::VectorXd x;

  bool success = tesseract::kinematics::solvePInv(A, b, x);
  EXPECT_FALSE(success);
}

// Helper function to validate Ax ≈ A(PA)x
bool isPseudoinverseValid(const Eigen::MatrixXd& A, const Eigen::MatrixXd& P, double tolerance = 1e-4)
{
  Eigen::MatrixXd approx = A * P * A;
  return (A - approx).norm() < tolerance;
}

TEST(TesseractKinematicsUnit, dampedPInv_FullRankSquareMatrix)
{
  Eigen::MatrixXd A(3, 3);
  A << 1, 2, 3, 4, 5, 6, 7, 8, 10;

  Eigen::MatrixXd P(3, 3);
  bool success = tesseract::kinematics::dampedPInv(A, P, 1e-5, 0.01);
  EXPECT_TRUE(success);
  EXPECT_EQ(P.rows(), 3);
  EXPECT_EQ(P.cols(), 3);
  EXPECT_TRUE(isPseudoinverseValid(A, P));
}

TEST(TesseractKinematicsUnit, dampedPInv_RankDeficientMatrix)
{
  Eigen::MatrixXd A(3, 3);
  A << 1, 2, 3, 2, 4, 6, 3, 6, 9;  // Rank deficient (linearly dependent rows)

  Eigen::MatrixXd P(3, 3);
  bool success = tesseract::kinematics::dampedPInv(A, P, 1e-5, 0.01);
  EXPECT_TRUE(success);
  EXPECT_EQ(P.rows(), 3);
  EXPECT_EQ(P.cols(), 3);
  EXPECT_TRUE(isPseudoinverseValid(A, P, 1e-2));  // Allow more slack
}

TEST(TesseractKinematicsUnit, dampedPInv_OverdeterminedMatrix)
{
  Eigen::MatrixXd A(4, 2);
  A << 1, 2, 3, 4, 5, 6, 7, 8;

  Eigen::MatrixXd P(2, 4);
  bool success = tesseract::kinematics::dampedPInv(A, P, 1e-5, 0.01);
  EXPECT_TRUE(success);
  EXPECT_EQ(P.rows(), 2);
  EXPECT_EQ(P.cols(), 4);
  EXPECT_TRUE(isPseudoinverseValid(A, P));
}

TEST(TesseractKinematicsUnit, dampedPInv_UnderdeterminedMatrix)
{
  Eigen::MatrixXd A(2, 4);
  A << 1, 2, 3, 4, 5, 6, 7, 8;

  Eigen::MatrixXd P(4, 2);
  bool success = tesseract::kinematics::dampedPInv(A, P, 1e-5, 0.01);
  EXPECT_TRUE(success);
  EXPECT_EQ(P.rows(), 4);
  EXPECT_EQ(P.cols(), 2);
  EXPECT_TRUE(isPseudoinverseValid(A, P));
}

TEST(TesseractKinematicsUnit, dampedPInv_EmptyMatrix)
{
  Eigen::MatrixXd A;
  Eigen::MatrixXd P;
  bool success = tesseract::kinematics::dampedPInv(A, P, 1e-5, 0.01);
  EXPECT_FALSE(success);
}

namespace
{
/**
 * @brief Build a single-joint 2-link scene graph used by the upper-bound tests.
 * The chain is: base (root) -- joint --> tip.
 */
tesseract::scene_graph::SceneGraph::UPtr makeSingleJointSceneGraph(tesseract::scene_graph::JointType type,
                                                                   const Eigen::Vector3d& offset,
                                                                   double lower,
                                                                   double upper)
{
  using namespace tesseract::scene_graph;
  auto sg = std::make_unique<SceneGraph>("test");
  sg->addLink(Link("base"));
  sg->addLink(Link("tip"));
  Joint j("j0");
  j.type = type;
  j.parent_link_name = "base";
  j.child_link_name = "tip";
  j.parent_to_joint_origin_transform.translation() = offset;
  j.axis = Eigen::Vector3d::UnitZ();
  j.limits = std::make_shared<JointLimits>();
  j.limits->lower = lower;
  j.limits->upper = upper;
  sg->addJoint(j);
  return sg;
}
}  // namespace

TEST(TesseractKinematicsUnit, ChainReachUpperBoundRevoluteOffset)  // NOLINT
{
  auto sg = makeSingleJointSceneGraph(
      tesseract::scene_graph::JointType::REVOLUTE, Eigen::Vector3d(0.0, 0.0, 0.1), -M_PI, M_PI);

  double bound = tesseract::kinematics::computeChainReachUpperBound(*sg, "base", "tip");

  EXPECT_NEAR(bound, 0.1, 1e-12);
}

TEST(TesseractKinematicsUnit, ChainReachUpperBoundMultiJointRevolute)  // NOLINT
{
  using namespace tesseract::scene_graph;
  SceneGraph sg("test");
  sg.addLink(Link("l0"));
  sg.addLink(Link("l1"));
  sg.addLink(Link("l2"));

  Joint j1("j1");
  j1.type = JointType::REVOLUTE;
  j1.parent_link_name = "l0";
  j1.child_link_name = "l1";
  j1.parent_to_joint_origin_transform.translation() = Eigen::Vector3d(0.0, 0.0, 1.0);
  j1.axis = Eigen::Vector3d::UnitY();
  j1.limits = std::make_shared<JointLimits>();
  j1.limits->lower = -M_PI;
  j1.limits->upper = M_PI;
  sg.addJoint(j1);

  Joint j2("j2");
  j2.type = JointType::REVOLUTE;
  j2.parent_link_name = "l1";
  j2.child_link_name = "l2";
  j2.parent_to_joint_origin_transform.translation() = Eigen::Vector3d(1.0, 2.0, 2.0);  // norm = 3
  j2.axis = Eigen::Vector3d::UnitZ();
  j2.limits = std::make_shared<JointLimits>();
  j2.limits->lower = -M_PI;
  j2.limits->upper = M_PI;
  sg.addJoint(j2);

  double bound = tesseract::kinematics::computeChainReachUpperBound(sg, "l0", "l2");

  EXPECT_NEAR(bound, 1.0 + 3.0, 1e-12);
}

TEST(TesseractKinematicsUnit, ChainReachUpperBoundPrismatic)  // NOLINT
{
  auto sg = makeSingleJointSceneGraph(
      tesseract::scene_graph::JointType::PRISMATIC, Eigen::Vector3d(0.1, 0.0, 0.0), -0.5, 1.5);

  double bound = tesseract::kinematics::computeChainReachUpperBound(*sg, "base", "tip");

  // 0.1 offset + max(|-0.5|, |1.5|) = 0.1 + 1.5 = 1.6
  EXPECT_NEAR(bound, 1.6, 1e-12);
}

TEST(TesseractKinematicsUnit, ChainReachUpperBoundZeroLength)  // NOLINT
{
  using namespace tesseract::scene_graph;
  SceneGraph sg("test");
  sg.addLink(Link("only"));

  double bound = tesseract::kinematics::computeChainReachUpperBound(sg, "only", "only");

  EXPECT_DOUBLE_EQ(bound, 0.0);
}

TEST(TesseractKinematicsUnit, ChainReachUpperBoundFloatingThrows)  // NOLINT
{
  auto sg =
      makeSingleJointSceneGraph(tesseract::scene_graph::JointType::FLOATING, Eigen::Vector3d(0.0, 0.0, 0.1), 0.0, 0.0);

  EXPECT_THROW(tesseract::kinematics::computeChainReachUpperBound(*sg, "base", "tip"), std::runtime_error);
}

TEST(TesseractKinematicsUnit, ChainReachUpperBoundMissingLinkThrows)  // NOLINT
{
  auto sg = makeSingleJointSceneGraph(
      tesseract::scene_graph::JointType::REVOLUTE, Eigen::Vector3d(0.0, 0.0, 0.1), -M_PI, M_PI);

  EXPECT_THROW(tesseract::kinematics::computeChainReachUpperBound(*sg, "base", "no_such_link"), std::runtime_error);
  EXPECT_THROW(tesseract::kinematics::computeChainReachUpperBound(*sg, "no_such_link", "tip"), std::runtime_error);
}

TEST(TesseractKinematicsUnit, ChainReachUpperBoundBranchedTree)  // NOLINT
{
  // Two siblings hanging off a shared parent. The function must walk only the
  // shortest path to the requested tip, not accumulate offsets from the unrelated branch.
  using namespace tesseract::scene_graph;
  SceneGraph sg("test");
  sg.addLink(Link("root"));

  namespace ts = tesseract::kinematics::test_suite;
  ts::addRevoluteChild(sg,
                       "j_root_hub",
                       "root",
                       "hub",
                       Eigen::Vector3d::UnitZ(),
                       Eigen::Isometry3d(Eigen::Translation3d(1.0, 0.0, 0.0)));  // norm = 1.0
  ts::addRevoluteChild(sg,
                       "j_hub_a",
                       "hub",
                       "tip_a",
                       Eigen::Vector3d::UnitZ(),
                       Eigen::Isometry3d(Eigen::Translation3d(0.0, 2.0, 0.0)));  // norm = 2.0
  ts::addRevoluteChild(sg,
                       "j_hub_b",
                       "hub",
                       "tip_b",
                       Eigen::Vector3d::UnitZ(),
                       Eigen::Isometry3d(Eigen::Translation3d(0.0, 0.0, 100.0)));  // norm = 100.0 (unrelated branch)

  EXPECT_NEAR(tesseract::kinematics::computeChainReachUpperBound(sg, "root", "tip_a"), 1.0 + 2.0, 1e-12);
  EXPECT_NEAR(tesseract::kinematics::computeChainReachUpperBound(sg, "root", "tip_b"), 1.0 + 100.0, 1e-12);
}

TEST(TesseractKinematicsUnit, ChainReachUpperBoundContinuousJoint)  // NOLINT
{
  auto sg = makeSingleJointSceneGraph(
      tesseract::scene_graph::JointType::CONTINUOUS, Eigen::Vector3d(0.0, 0.0, 0.25), 0.0, 0.0);

  double bound = tesseract::kinematics::computeChainReachUpperBound(*sg, "base", "tip");

  // Continuous joints rotate; only the offset contributes.
  EXPECT_NEAR(bound, 0.25, 1e-12);
}

TEST(TesseractKinematicsUnit, ChainReachUpperBoundPrismaticMimicThrows)  // NOLINT
{
  auto sg =
      makeSingleJointSceneGraph(tesseract::scene_graph::JointType::PRISMATIC, Eigen::Vector3d(0.0, 0.0, 0.1), 0.0, 1.0);
  auto j = std::const_pointer_cast<tesseract::scene_graph::Joint>(sg->getJoint("j0"));
  j->mimic = std::make_shared<tesseract::scene_graph::JointMimic>();

  EXPECT_THROW(tesseract::kinematics::computeChainReachUpperBound(*sg, "base", "tip"), std::runtime_error);
}

TEST(TesseractKinematicsUnit, ChainReachUpperBoundABBIRB2400FKSampled)  // NOLINT
{
  tesseract::common::GeneralResourceLocator locator;
  auto scene_graph = tesseract::kinematics::test_suite::getSceneGraphABB(locator);

  const std::string base_link("base_link");
  const std::string tip_link("tool0");
  const double bound = tesseract::kinematics::computeChainReachUpperBound(*scene_graph, base_link, tip_link);

  // Build FK to produce tool0 poses for random joint configs.
  auto fwd = std::make_unique<tesseract::kinematics::KDLFwdKinChain>(*scene_graph, base_link, tip_link);

  // Gather joint limits along the manipulator chain via shared helper.
  const auto joint_names = fwd->getJointNames();
  Eigen::MatrixX2d limits = tesseract::kinematics::test_suite::getTargetLimits(*scene_graph, joint_names).joint_limits;

  std::mt19937 rng(0xC0FFEE);  // deterministic
  constexpr int kNumSamples = 10000;
  constexpr double kEpsilon = 1e-9;

  double observed_max = 0.0;
  for (int i = 0; i < kNumSamples; ++i)
  {
    Eigen::VectorXd q(limits.rows());
    for (Eigen::Index k = 0; k < q.size(); ++k)
    {
      std::uniform_real_distribution<double> dist(limits(k, 0), limits(k, 1));
      q(k) = dist(rng);
    }

    tesseract::common::TransformMap poses;
    fwd->calcFwdKin(poses, q);
    const double r = poses.at(tip_link).translation().norm();
    observed_max = std::max(observed_max, r);
    ASSERT_LE(r, bound + kEpsilon) << "config " << i << " produced norm(tool0)=" << r << " > bound " << bound;
  }

  // Sanity: derived bound should not be wildly loose either - for a real 6-DOF arm the max
  // reach from uniform sampling should be within a few metres of the bound. Observed max on
  // IRB2400 is ~2.15 m; bound from the URDF sums to ~2.20 m (~2.4% slack).
  EXPECT_LT(bound, 4.0) << "Bound " << bound << " is suspiciously loose for IRB2400";
  EXPECT_GT(observed_max, 1.5) << "Did not sample a representative workspace (max=" << observed_max << ")";
}

TEST(KinematicsUtils, GatherJointLimits)  // NOLINT
{
  auto sg = std::make_shared<tesseract::scene_graph::SceneGraph>();
  sg->setName("test");
  sg->addLink(tesseract::scene_graph::Link("base"));

  // Joint A with limits [-1, 2]
  tesseract::scene_graph::Joint ja("a");
  ja.parent_link_name = "base";
  ja.child_link_name = "l1";
  ja.type = tesseract::scene_graph::JointType::REVOLUTE;
  ja.axis = Eigen::Vector3d::UnitZ();
  ja.limits = std::make_shared<tesseract::scene_graph::JointLimits>();
  ja.limits->lower = -1.0;
  ja.limits->upper = 2.0;
  sg->addLink(tesseract::scene_graph::Link("l1"));
  sg->addJoint(ja);

  // Joint B with limits [-3, 0.5]
  tesseract::scene_graph::Joint jb("b");
  jb.parent_link_name = "l1";
  jb.child_link_name = "l2";
  jb.type = tesseract::scene_graph::JointType::REVOLUTE;
  jb.axis = Eigen::Vector3d::UnitZ();
  jb.limits = std::make_shared<tesseract::scene_graph::JointLimits>();
  jb.limits->lower = -3.0;
  jb.limits->upper = 0.5;
  sg->addLink(tesseract::scene_graph::Link("l2"));
  sg->addJoint(jb);

  Eigen::MatrixX2d limits = tesseract::kinematics::gatherJointLimits(*sg, { "a", "b" });
  ASSERT_EQ(limits.rows(), 2);
  EXPECT_DOUBLE_EQ(limits(0, 0), -1.0);
  EXPECT_DOUBLE_EQ(limits(0, 1), 2.0);
  EXPECT_DOUBLE_EQ(limits(1, 0), -3.0);
  EXPECT_DOUBLE_EQ(limits(1, 1), 0.5);

  // Missing joint → throws
  EXPECT_THROW(tesseract::kinematics::gatherJointLimits(*sg, { "missing" }), std::runtime_error);

  // Joint without limits → throws
  tesseract::scene_graph::Joint jc("c");
  jc.parent_link_name = "l2";
  jc.child_link_name = "l3";
  jc.type = tesseract::scene_graph::JointType::FIXED;  // FIXED has no limits
  sg->addLink(tesseract::scene_graph::Link("l3"));
  sg->addJoint(jc);
  EXPECT_THROW(tesseract::kinematics::gatherJointLimits(*sg, { "c" }), std::runtime_error);
}

TEST(KinematicsUtils, BuildSampleGrid)  // NOLINT
{
  Eigen::MatrixX2d range(2, 2);
  range << 0.0, 1.0, -1.0, 1.0;
  Eigen::VectorXd res(2);
  res << 0.5, 1.0;

  auto grid = tesseract::kinematics::buildSampleGrid(range, res);
  ASSERT_EQ(grid.size(), 2U);
  // Joint 0: range 1.0, res 0.5 → ceil(1.0/0.5) + 1 = 3 samples (0, 0.5, 1)
  ASSERT_EQ(grid[0].size(), 3);
  EXPECT_DOUBLE_EQ(grid[0](0), 0.0);
  EXPECT_DOUBLE_EQ(grid[0](1), 0.5);
  EXPECT_DOUBLE_EQ(grid[0](2), 1.0);
  // Joint 1: range 2.0, res 1.0 → ceil(2.0/1.0) + 1 = 3 samples (-1, 0, 1)
  ASSERT_EQ(grid[1].size(), 3);
  EXPECT_DOUBLE_EQ(grid[1](0), -1.0);
  EXPECT_DOUBLE_EQ(grid[1](1), 0.0);
  EXPECT_DOUBLE_EQ(grid[1](2), 1.0);

  // Resolution larger than range → still produces 2 samples (endpoints)
  Eigen::MatrixX2d narrow(1, 2);
  narrow << 0.0, 0.1;
  Eigen::VectorXd coarse(1);
  coarse << 1.0;
  auto narrow_grid = tesseract::kinematics::buildSampleGrid(narrow, coarse);
  ASSERT_EQ(narrow_grid.size(), 1U);
  EXPECT_EQ(narrow_grid[0].size(), 2);
  EXPECT_DOUBLE_EQ(narrow_grid[0](0), 0.0);
  EXPECT_DOUBLE_EQ(narrow_grid[0](1), 0.1);
}

TEST(AddRevoluteChild, NonZeroDynamicLimitDefaults)  // NOLINT
{
  // Regression guard for kinematics_test_utils.h::addRevoluteChild —
  // velocity / acceleration / jerk must default to a strictly positive value
  // so getTargetLimits() on graphs built via this helper is usable.
  auto sg = std::make_shared<tesseract::scene_graph::SceneGraph>();
  sg->setName("test");
  sg->addLink(tesseract::scene_graph::Link("base"));
  sg->setRoot("base");

  namespace ts = tesseract::kinematics::test_suite;
  ts::addRevoluteChild(*sg, "j1", "base", "link1", Eigen::Vector3d::UnitZ());

  auto joint = sg->getJoint("j1");
  ASSERT_NE(joint, nullptr);
  ASSERT_NE(joint->limits, nullptr);
  EXPECT_GT(joint->limits->velocity, 0.0);
  EXPECT_GT(joint->limits->acceleration, 0.0);
  EXPECT_GT(joint->limits->jerk, 0.0);
}

TEST(RTPInvKin, CtorsRejectNullManipulator)  // NOLINT
{
  // All four public ctors must reject a null manipulator with std::runtime_error.
  // The explicit-reach ctors previously threw later from inside init() with a
  // less precise stack; the auto-reach ctors would have dereferenced before
  // reaching init(). This test pins the exception type for every ctor variant.
  auto sg = std::make_shared<tesseract::scene_graph::SceneGraph>();
  sg->setName("test");
  sg->addLink(tesseract::scene_graph::Link("world"));
  sg->setRoot("world");

  namespace ts = tesseract::kinematics::test_suite;
  ts::addRevoluteChild(*sg, "tool_j1", "world", "tool_tip", Eigen::Vector3d::UnitZ());

  tesseract::scene_graph::SceneState scene_state;
  scene_state.link_transforms["world"] = Eigen::Isometry3d::Identity();
  scene_state.link_transforms["tool_tip"] = Eigen::Isometry3d::Identity();

  auto make_tool = [&]() { return std::make_unique<tesseract::kinematics::KDLFwdKinChain>(*sg, "world", "tool_tip"); };

  Eigen::VectorXd res(1);
  res << 0.5;
  Eigen::MatrixX2d range(1, 2);
  range << -M_PI, M_PI;

  // Ctor variant: explicit reach, range derived from joint limits.
  EXPECT_THROW(tesseract::kinematics::RTPInvKin(*sg,
                                                scene_state,
                                                /*manipulator=*/nullptr,
                                                /*manipulator_reach=*/1.0,
                                                make_tool(),
                                                res),
               std::runtime_error);

  // Ctor variant: explicit reach, explicit range.
  EXPECT_THROW(tesseract::kinematics::RTPInvKin(*sg,
                                                scene_state,
                                                /*manipulator=*/nullptr,
                                                /*manipulator_reach=*/1.0,
                                                make_tool(),
                                                range,
                                                res),
               std::runtime_error);

  // Ctor variant: auto reach, range derived from joint limits.
  EXPECT_THROW(tesseract::kinematics::RTPInvKin(*sg,
                                                scene_state,
                                                /*manipulator=*/nullptr,
                                                make_tool(),
                                                res),
               std::runtime_error);

  // Ctor variant: auto reach, explicit range.
  EXPECT_THROW(tesseract::kinematics::RTPInvKin(*sg,
                                                scene_state,
                                                /*manipulator=*/nullptr,
                                                make_tool(),
                                                range,
                                                res),
               std::runtime_error);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
