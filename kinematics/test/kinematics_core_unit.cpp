#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/kinematics/kdl/kdl_fwd_kin_chain.h>
#include <tesseract/kinematics/kdl/kdl_inv_kin_chain_lma.h>
#include <tesseract/kinematics/inverse_kinematics.h>
#include <tesseract/kinematics/joint_group.h>
#include <tesseract/kinematics/kinematic_group.h>
#include <tesseract/kinematics/rtp_inv_kin.h>
#include <tesseract/kinematics/utils.h>
#include <tesseract/state_solver/kdl/kdl_state_solver.h>
#include <tesseract/common/types.h>
#include <tesseract/scene_graph/graph.h>
#include <tesseract/scene_graph/joint.h>
#include <tesseract/scene_graph/link.h>
#include "kinematics_test_utils.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <limits>
#include <random>

const static std::string FACTORY_NAME = "TestFactory";

namespace
{
/**
 * @brief Minimal InverseKinematics stub for driving KinematicGroup constructor validation paths.
 * @details Allows the test to supply arbitrary joint_ids / working_frame / tip_links so the
 * KinematicGroup constructor throws can be exercised directly.
 */
class FakeInvKin : public tesseract::kinematics::InverseKinematics
{
public:
  FakeInvKin(std::vector<tesseract::common::JointId> joint_ids,
             tesseract::common::LinkId working_frame,
             std::vector<tesseract::common::LinkId> tip_links,
             tesseract::common::LinkId base_link = "base_link")
    : joint_ids_(std::move(joint_ids))
    , working_frame_(std::move(working_frame))
    , base_link_(std::move(base_link))
    , tip_links_(std::move(tip_links))
  {
  }

  void calcInvKin(tesseract::kinematics::IKSolutions& /*solutions*/,
                  const tesseract::common::LinkIdTransformMap& /*tip_link_poses*/,
                  const Eigen::Ref<const Eigen::VectorXd>& /*seed*/) const override
  {
  }

  std::vector<tesseract::common::JointId> getJointIds() const override { return joint_ids_; }
  Eigen::Index numJoints() const override { return static_cast<Eigen::Index>(joint_ids_.size()); }
  tesseract::common::LinkId getBaseLinkId() const override { return base_link_; }
  tesseract::common::LinkId getWorkingFrame() const override { return working_frame_; }
  std::vector<tesseract::common::LinkId> getTipLinkIds() const override { return tip_links_; }
  std::string getSolverName() const override { return "FakeInvKin"; }
  tesseract::kinematics::InverseKinematics::UPtr clone() const override { return std::make_unique<FakeInvKin>(*this); }

private:
  std::vector<tesseract::common::JointId> joint_ids_;
  tesseract::common::LinkId working_frame_;
  tesseract::common::LinkId base_link_;
  std::vector<tesseract::common::LinkId> tip_links_;
};
}  // namespace

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
  using tesseract::common::LinkId;

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
  using tesseract::common::LinkId;

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

// =============================================================================
// Integer link/joint ID tests for JointGroup
// =============================================================================

TEST(TesseractKinematicsUnit, JointGroupCalcFwdKinLinkIdUnit)  // NOLINT
{
  using tesseract::common::LinkId;

  tesseract::common::GeneralResourceLocator locator;
  auto scene_graph = tesseract::kinematics::test_suite::getSceneGraphIIWA(locator);

  // Create a JointGroup from the scene graph
  tesseract::scene_graph::KDLStateSolver ss(*scene_graph);
  auto joint_group = std::make_unique<tesseract::kinematics::JointGroup>(
      "manipulator",
      std::vector<tesseract::common::JointId>{
          "joint_a1", "joint_a2", "joint_a3", "joint_a4", "joint_a5", "joint_a6", "joint_a7" },
      *scene_graph,
      ss.getState());

  Eigen::VectorXd jvals = Eigen::VectorXd::Zero(7);
  jvals[1] = 0.5;
  jvals[3] = -0.3;

  // calcFwdKin returns LinkIdTransformMap
  tesseract::common::LinkIdTransformMap result = joint_group->calcFwdKin(jvals);

  // Verify specific LinkId keys are present
  EXPECT_TRUE(result.count("base_link") > 0);
  EXPECT_TRUE(result.count("tool0") > 0);
  EXPECT_TRUE(result.count("link_7") > 0);

  // Verify the result is non-empty and contains expected links
  EXPECT_FALSE(result.empty());
}

TEST(TesseractKinematicsUnit, JointGroupIsActiveLinkIdUnit)  // NOLINT
{
  using tesseract::common::LinkId;

  tesseract::common::GeneralResourceLocator locator;
  auto scene_graph = tesseract::kinematics::test_suite::getSceneGraphIIWA(locator);

  tesseract::scene_graph::KDLStateSolver ss(*scene_graph);
  auto joint_group = std::make_unique<tesseract::kinematics::JointGroup>(
      "manipulator",
      std::vector<tesseract::common::JointId>{
          "joint_a1", "joint_a2", "joint_a3", "joint_a4", "joint_a5", "joint_a6", "joint_a7" },
      *scene_graph,
      ss.getState());

  // Active links are those moved by the active joints
  for (const auto& name : joint_group->getActiveLinkIds())
  {
    EXPECT_TRUE(joint_group->isActiveLinkId(name));
  }

  // base_link should not be active (it's the fixed base)
  EXPECT_FALSE(joint_group->isActiveLinkId("base_link"));

  // Non-existent link should not be active
  EXPECT_FALSE(joint_group->isActiveLinkId("nonexistent_link"));
}

TEST(TesseractKinematicsUnit, JointGroupByJointIdAccessorsUnit)  // NOLINT
{
  using tesseract::common::JointId;
  using tesseract::common::LinkId;

  tesseract::common::GeneralResourceLocator locator;
  auto scene_graph = tesseract::kinematics::test_suite::getSceneGraphIIWA(locator);

  tesseract::scene_graph::KDLStateSolver ss(*scene_graph);
  const auto scene_state = ss.getState();

  std::vector<JointId> joint_ids{ "joint_a1", "joint_a2", "joint_a3", "joint_a4", "joint_a5", "joint_a6", "joint_a7" };

  // Construct JointGroup via the JointId overload.
  tesseract::kinematics::JointGroup jg("manipulator", joint_ids, *scene_graph, scene_state);

  // numJoints / getJointIds
  EXPECT_EQ(jg.numJoints(), static_cast<Eigen::Index>(joint_ids.size()));
  EXPECT_EQ(jg.getJointIds(), joint_ids);

  // getBaseLinkId — should match scene graph root.
  const LinkId base_link_id = jg.getBaseLinkId();
  EXPECT_EQ(base_link_id, scene_graph->getRoot());

  // Link id collections non-empty.
  const std::vector<LinkId>& link_ids = jg.getLinkIds();
  const std::vector<LinkId>& active_link_ids = jg.getActiveLinkIds();
  const std::vector<LinkId>& static_link_ids = jg.getStaticLinkIds();
  EXPECT_FALSE(link_ids.empty());
  EXPECT_FALSE(active_link_ids.empty());
  EXPECT_FALSE(static_link_ids.empty());
  EXPECT_EQ(link_ids.size(), active_link_ids.size() + static_link_ids.size());

  // hasLinkId / isActiveLinkId (true-case) for all links.
  for (const auto& lid : link_ids)
    EXPECT_TRUE(jg.hasLinkId(lid));
  for (const auto& lid : active_link_ids)
    EXPECT_TRUE(jg.isActiveLinkId(lid));

  // isActiveLinkId (false-case) for static links and a non-existent link.
  for (const auto& lid : static_link_ids)
    EXPECT_FALSE(jg.isActiveLinkId(lid));
  EXPECT_FALSE(jg.isActiveLinkId("nonexistent_link"));
  EXPECT_FALSE(jg.hasLinkId("nonexistent_link"));

  // calcFwdKin — both overloads.
  Eigen::VectorXd q = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(joint_ids.size()));
  q[1] = 0.25;
  q[3] = -0.4;
  tesseract::common::LinkIdTransformMap tfs1 = jg.calcFwdKin(q);
  EXPECT_FALSE(tfs1.empty());

  tesseract::common::LinkIdTransformMap tfs2;
  jg.calcFwdKin(tfs2, q);
  EXPECT_EQ(tfs1.size(), tfs2.size());
  for (const auto& kv : tfs1)
  {
    ASSERT_TRUE(tfs2.count(kv.first) > 0);
    EXPECT_TRUE(tfs2.at(kv.first).isApprox(kv.second, 1e-9));
  }

  // calcJacobian — pick an active link id to ensure a non-trivial jacobian.
  const LinkId& jac_link = active_link_ids.back();
  Eigen::MatrixXd jac = jg.calcJacobian(q, jac_link);
  EXPECT_EQ(jac.rows(), 6);
  EXPECT_EQ(jac.cols(), q.size());

  // calcJacobian with explicit base_link overload.
  Eigen::MatrixXd jac_with_base = jg.calcJacobian(q, base_link_id, jac_link);
  EXPECT_EQ(jac_with_base.rows(), 6);
  EXPECT_EQ(jac_with_base.cols(), q.size());
  EXPECT_TRUE(jac_with_base.isApprox(jac, 1e-9));

  // calcJacobian with link_point overload.
  Eigen::MatrixXd jac_at_point = jg.calcJacobian(q, jac_link, Eigen::Vector3d(0.05, 0.0, 0.0));
  EXPECT_EQ(jac_at_point.rows(), 6);
  EXPECT_EQ(jac_at_point.cols(), q.size());

  // 4-arg calcJacobian with base == root — delegates to the 3-arg link_point overload.
  Eigen::MatrixXd jac_at_point_with_base = jg.calcJacobian(q, base_link_id, jac_link, Eigen::Vector3d(0.05, 0.0, 0.0));
  EXPECT_TRUE(jac_at_point_with_base.isApprox(jac_at_point, 1e-9));

  // 4-arg calcJacobian with an active non-root base link.
  const LinkId& intermediate_base = active_link_ids.front();
  ASSERT_NE(intermediate_base, base_link_id);
  Eigen::MatrixXd jac_active_base = jg.calcJacobian(q, intermediate_base, jac_link, Eigen::Vector3d(0.05, 0.0, 0.0));
  EXPECT_EQ(jac_active_base.rows(), 6);
  EXPECT_EQ(jac_active_base.cols(), q.size());
}

TEST(TesseractKinematicsUnit, JointGroupCalcJacobian4ArgStaticBaseUnit)  // NOLINT
{
  using tesseract::common::JointId;
  using tesseract::common::LinkId;

  tesseract::common::GeneralResourceLocator locator;
  auto scene_graph = tesseract::kinematics::test_suite::getSceneGraphIIWA(locator);

  tesseract::scene_graph::KDLStateSolver ss(*scene_graph);
  const auto scene_state = ss.getState();

  // Sub-group of IIWA joints: excluding joint_a1..joint_a3 leaves link_3 as a
  // non-root static link in the group's sub-tree — the only configuration that
  // exercises the static-base branch of the 4-arg calcJacobian.
  std::vector<JointId> joint_ids{ "joint_a4", "joint_a5", "joint_a6", "joint_a7" };
  tesseract::kinematics::JointGroup jg("sub_manipulator", joint_ids, *scene_graph, scene_state);

  const LinkId static_base("link_3");
  const LinkId tip("tool0");
  ASSERT_FALSE(jg.isActiveLinkId(static_base));
  ASSERT_TRUE(jg.isActiveLinkId(tip));

  Eigen::VectorXd q = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(joint_ids.size()));
  q[0] = 0.3;
  q[2] = -0.2;

  Eigen::MatrixXd jac = jg.calcJacobian(q, static_base, tip, Eigen::Vector3d(0.05, 0.0, 0.0));
  EXPECT_EQ(jac.rows(), 6);
  EXPECT_EQ(jac.cols(), q.size());
}

TEST(TesseractKinematicsUnit, KinematicGroupByJointIdAccessorsUnit)  // NOLINT
{
  using tesseract::common::JointId;
  using tesseract::common::LinkId;

  tesseract::common::GeneralResourceLocator locator;
  auto scene_graph = tesseract::kinematics::test_suite::getSceneGraphIIWA(locator);

  tesseract::scene_graph::KDLStateSolver ss(*scene_graph);
  const auto scene_state = ss.getState();

  const LinkId base_link_id("base_link");
  const LinkId tip_link_id("tool0");
  std::vector<JointId> joint_ids{ "joint_a1", "joint_a2", "joint_a3", "joint_a4", "joint_a5", "joint_a6", "joint_a7" };

  // Build an inverse kinematics solver and construct KinematicGroup via the JointId overload.
  tesseract::kinematics::KDLInvKinChainLMA::Config config;
  auto inv_kin =
      std::make_unique<tesseract::kinematics::KDLInvKinChainLMA>(*scene_graph, base_link_id, tip_link_id, config);

  tesseract::kinematics::KinematicGroup kg("manipulator", joint_ids, std::move(inv_kin), *scene_graph, scene_state);

  // Accessors inherited from JointGroup still work through the ID-based build.
  EXPECT_EQ(kg.numJoints(), static_cast<Eigen::Index>(joint_ids.size()));
  EXPECT_EQ(kg.getJointIds(), joint_ids);
  EXPECT_EQ(kg.getBaseLinkId(), scene_graph->getRoot());
  EXPECT_FALSE(kg.getLinkIds().empty());
  EXPECT_FALSE(kg.getActiveLinkIds().empty());
  EXPECT_FALSE(kg.getStaticLinkIds().empty());

  // Target pose reachable by the IIWA at the default configuration.
  Eigen::Isometry3d target_pose = Eigen::Isometry3d::Identity();
  target_pose.translation()[2] = 1.306;

  Eigen::VectorXd seed(joint_ids.size());
  seed << -0.785398, 0.785398, -0.785398, 0.785398, -0.785398, 0.785398, -0.785398;

  // calcInvKin — exercise the main body, not just early-return.
  tesseract::kinematics::KinGroupIKInput input(target_pose, base_link_id, tip_link_id);
  tesseract::kinematics::IKSolutions solutions = kg.calcInvKin(input, seed);
  EXPECT_FALSE(solutions.empty());

  // Validate the IK solution by running it back through FK.
  for (const auto& sol : solutions)
  {
    auto result_poses = kg.calcFwdKin(sol);
    Eigen::Isometry3d result = result_poses.at(base_link_id).inverse() * result_poses.at(tip_link_id);
    EXPECT_TRUE(target_pose.translation().isApprox(result.translation(), 1e-4));
  }

  // getAllValidWorkingFrames / getAllPossibleTipLinkIds.
  const std::vector<LinkId> working_frames = kg.getAllValidWorkingFrames();
  EXPECT_FALSE(working_frames.empty());
  EXPECT_NE(std::find(working_frames.begin(), working_frames.end(), base_link_id), working_frames.end());

  const std::vector<LinkId> tip_links = kg.getAllPossibleTipLinkIds();
  EXPECT_FALSE(tip_links.empty());
  EXPECT_NE(std::find(tip_links.begin(), tip_links.end(), tip_link_id), tip_links.end());
}

TEST(TesseractKinematicsUnit, KinematicGroupConstructorThrowsUnit)  // NOLINT
{
  using tesseract::common::JointId;
  using tesseract::common::LinkId;

  tesseract::common::GeneralResourceLocator locator;
  auto scene_graph = tesseract::kinematics::test_suite::getSceneGraphIIWA(locator);

  tesseract::scene_graph::KDLStateSolver ss(*scene_graph);
  const auto scene_state = ss.getState();

  const LinkId base_link_id("base_link");
  const LinkId tip_link_id("tool0");
  const std::vector<JointId> joint_ids{ "joint_a1", "joint_a2", "joint_a3", "joint_a4",
                                        "joint_a5", "joint_a6", "joint_a7" };

  // Wrong-size joint_ids: the fake reports 7 joints but the KinematicGroup is built with 3.
  {
    auto inv = std::make_unique<FakeInvKin>(joint_ids, base_link_id, std::vector<LinkId>{ tip_link_id });
    const std::vector<JointId> short_ids{ joint_ids[0], joint_ids[1], joint_ids[2] };
    EXPECT_THROW(tesseract::kinematics::KinematicGroup("kg", short_ids, std::move(inv), *scene_graph, scene_state),
                 std::runtime_error);
  }

  // Matching size but different ids: exercises the "joint_ids does not match" throw.
  {
    std::vector<JointId> mismatched = joint_ids;
    mismatched.back() = "bogus_joint";
    auto inv = std::make_unique<FakeInvKin>(joint_ids, base_link_id, std::vector<LinkId>{ tip_link_id });
    EXPECT_THROW(tesseract::kinematics::KinematicGroup("kg", mismatched, std::move(inv), *scene_graph, scene_state),
                 std::runtime_error);
  }

  // Same ids but reversed order: exercises the reorder_required_ / inv_kin_joint_map_ branch.
  {
    std::vector<JointId> reordered(joint_ids.rbegin(), joint_ids.rend());
    auto inv = std::make_unique<FakeInvKin>(joint_ids, base_link_id, std::vector<LinkId>{ tip_link_id });
    EXPECT_NO_THROW(
        tesseract::kinematics::KinematicGroup("kg_reordered", reordered, std::move(inv), *scene_graph, scene_state));
  }

  // Unknown working frame: exercises the working-frame link-transform lookup throw.
  {
    auto inv = std::make_unique<FakeInvKin>(joint_ids, "not_a_link", std::vector<LinkId>{ tip_link_id });
    EXPECT_THROW(tesseract::kinematics::KinematicGroup("kg", joint_ids, std::move(inv), *scene_graph, scene_state),
                 std::runtime_error);
  }

  // Unknown tip link: exercises the tip-link link-transform lookup throw.
  {
    auto inv = std::make_unique<FakeInvKin>(joint_ids, base_link_id, std::vector<LinkId>{ "not_a_tip" });
    EXPECT_THROW(tesseract::kinematics::KinematicGroup("kg", joint_ids, std::move(inv), *scene_graph, scene_state),
                 std::runtime_error);
  }
}

TEST(TesseractKinematicsUnit, JointGroupCopyAssignmentUnit)  // NOLINT
{
  using tesseract::common::JointId;
  using tesseract::common::LinkId;

  tesseract::common::GeneralResourceLocator locator;
  auto scene_graph = tesseract::kinematics::test_suite::getSceneGraphIIWA(locator);

  tesseract::scene_graph::KDLStateSolver ss(*scene_graph);
  const auto scene_state = ss.getState();

  const std::vector<JointId> joint_ids{ "joint_a1", "joint_a2", "joint_a3", "joint_a4",
                                        "joint_a5", "joint_a6", "joint_a7" };

  tesseract::kinematics::JointGroup jg_a("manipulator_a", joint_ids, *scene_graph, scene_state);

  EXPECT_FALSE(jg_a.getLinkIds().empty());

  // Exercises JointGroup copy-assignment.
  const std::vector<JointId> subset{ joint_ids[0], joint_ids[1], joint_ids[2] };
  tesseract::kinematics::JointGroup jg_b("sub_manipulator", subset, *scene_graph, scene_state);
  EXPECT_EQ(jg_b.numJoints(), static_cast<Eigen::Index>(subset.size()));

  jg_b = jg_a;
  EXPECT_EQ(jg_b.numJoints(), jg_a.numJoints());
  EXPECT_EQ(jg_b.getJointIds(), jg_a.getJointIds());
  EXPECT_EQ(jg_b.getLinkIds().size(), jg_a.getLinkIds().size());

  // Self-assignment is a no-op.
#if defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wself-assign-overloaded"
#endif
  jg_b = jg_b;  // NOLINT(clang-diagnostic-self-assign-overloaded)
#if defined(__clang__)
#pragma GCC diagnostic pop
#endif
  EXPECT_EQ(jg_b.getJointIds(), joint_ids);

  // JointGroup throws when a joint id is not in the scene graph.
  std::vector<JointId> with_bogus = joint_ids;
  with_bogus.emplace_back("does_not_exist_in_graph");
  EXPECT_THROW(tesseract::kinematics::JointGroup("bad_group", with_bogus, *scene_graph, scene_state),
               std::runtime_error);
}

TEST(TesseractKinematicsUnit, KinematicGroupConstructorMismatchedIdsUnit)  // NOLINT
{
  // KinematicGroup throws "joint_ids does not match inverse kinematics object!" when its joint
  // ids disagree with the inverse kinematics object's. KinematicGroupConstructorThrowsUnit's
  // "mismatched" case cannot reach that check: it fails earlier in the JointGroup base class
  // because bogus_joint is not in the scene graph. Here every joint id is a valid scene-graph
  // joint but the FakeInvKin reports a different (same-count) joint set, reaching the inner
  // set-equality check.
  using tesseract::common::JointId;
  using tesseract::common::LinkId;

  tesseract::common::GeneralResourceLocator locator;
  auto scene_graph = tesseract::kinematics::test_suite::getSceneGraphIIWA(locator);
  tesseract::scene_graph::KDLStateSolver ss(*scene_graph);
  const auto scene_state = ss.getState();

  const LinkId base_link_id("base_link");
  const LinkId tip_link_id("tool0");
  const std::vector<JointId> real_ids{ "joint_a1", "joint_a2", "joint_a3", "joint_a4",
                                       "joint_a5", "joint_a6", "joint_a7" };

  // FakeInvKin reports a vector of the same count but containing a bogus id.
  std::vector<JointId> fake_inv_ids = real_ids;
  fake_inv_ids.back() = "not_a_real_joint";
  auto inv = std::make_unique<FakeInvKin>(fake_inv_ids, base_link_id, std::vector<LinkId>{ tip_link_id });

  EXPECT_THROW(tesseract::kinematics::KinematicGroup("kg", real_ids, std::move(inv), *scene_graph, scene_state),
               std::runtime_error);
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
  j.parent_link_id = "base";
  j.child_link_id = "tip";
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
  j1.parent_link_id = "l0";
  j1.child_link_id = "l1";
  j1.parent_to_joint_origin_transform.translation() = Eigen::Vector3d(0.0, 0.0, 1.0);
  j1.axis = Eigen::Vector3d::UnitY();
  j1.limits = std::make_shared<JointLimits>();
  j1.limits->lower = -M_PI;
  j1.limits->upper = M_PI;
  sg.addJoint(j1);

  Joint j2("j2");
  j2.type = JointType::REVOLUTE;
  j2.parent_link_id = "l1";
  j2.child_link_id = "l2";
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

TEST(TesseractKinematicsUnit, ChainReachUpperBoundDisconnectedLinkThrows)  // NOLINT
{
  auto sg = makeSingleJointSceneGraph(
      tesseract::scene_graph::JointType::REVOLUTE, Eigen::Vector3d(0.0, 0.0, 0.1), -M_PI, M_PI);
  sg->addLink(tesseract::scene_graph::Link("island"));

  // Both links are present, so this is the empty-joint-path rejection and not the missing-link one
  // above. The message assertion is what keeps the two apart.
  try
  {
    tesseract::kinematics::computeChainReachUpperBound(*sg, "base", "island");
    ADD_FAILURE() << "Expected rejection for a tip link with no path from the base link";
  }
  catch (const std::runtime_error& e)
  {
    EXPECT_NE(std::string(e.what()).find("no path from 'base' to 'island'"), std::string::npos)
        << "threw for the wrong reason: " << e.what();
  }
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

TEST(TesseractKinematicsUnit, ChainReachUpperBoundPrismaticNoLimitsThrows)  // NOLINT
{
  auto sg =
      makeSingleJointSceneGraph(tesseract::scene_graph::JointType::PRISMATIC, Eigen::Vector3d(0.0, 0.0, 0.1), 0.0, 1.0);
  auto j = std::const_pointer_cast<tesseract::scene_graph::Joint>(sg->getJoint("j0"));
  j->limits = nullptr;

  EXPECT_THROW(tesseract::kinematics::computeChainReachUpperBound(*sg, "base", "tip"), std::runtime_error);
}

TEST(TesseractKinematicsUnit, ChainReachUpperBoundPrismaticNonFiniteLimitsThrows)  // NOLINT
{
  // Lower bound is infinite — bound formula would silently produce infinity. Must reject.
  {
    auto sg = makeSingleJointSceneGraph(
        tesseract::scene_graph::JointType::PRISMATIC, Eigen::Vector3d(0.0, 0.0, 0.1), 0.0, 1.0);
    auto j = std::const_pointer_cast<tesseract::scene_graph::Joint>(sg->getJoint("j0"));
    j->limits->lower = -std::numeric_limits<double>::infinity();
    EXPECT_THROW(tesseract::kinematics::computeChainReachUpperBound(*sg, "base", "tip"), std::runtime_error);
  }
  // Upper bound is NaN.
  {
    auto sg = makeSingleJointSceneGraph(
        tesseract::scene_graph::JointType::PRISMATIC, Eigen::Vector3d(0.0, 0.0, 0.1), 0.0, 1.0);
    auto j = std::const_pointer_cast<tesseract::scene_graph::Joint>(sg->getJoint("j0"));
    j->limits->upper = std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(tesseract::kinematics::computeChainReachUpperBound(*sg, "base", "tip"), std::runtime_error);
  }
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
  const auto joint_ids = fwd->getJointIds();
  Eigen::MatrixX2d limits = tesseract::kinematics::test_suite::getTargetLimits(*scene_graph, joint_ids).joint_limits;

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

    tesseract::common::LinkIdTransformMap poses;
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

TEST(TesseractKinematicsUnit, GatherJointLimits)  // NOLINT
{
  auto sg = std::make_shared<tesseract::scene_graph::SceneGraph>();
  sg->setName("test");
  sg->addLink(tesseract::scene_graph::Link("base"));

  auto limits_a = tesseract::kinematics::test_suite::defaultTestJointLimits();
  limits_a.lower = -1.0;
  limits_a.upper = 2.0;
  tesseract::kinematics::test_suite::addRevoluteChild(
      *sg, "a", "base", "l1", Eigen::Vector3d::UnitZ(), Eigen::Isometry3d::Identity(), limits_a);

  auto limits_b = tesseract::kinematics::test_suite::defaultTestJointLimits();
  limits_b.lower = -3.0;
  limits_b.upper = 0.5;
  tesseract::kinematics::test_suite::addRevoluteChild(
      *sg, "b", "l1", "l2", Eigen::Vector3d::UnitZ(), Eigen::Isometry3d::Identity(), limits_b);

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
  jc.parent_link_id = "l2";
  jc.child_link_id = "l3";
  jc.type = tesseract::scene_graph::JointType::FIXED;  // FIXED has no limits
  sg->addLink(tesseract::scene_graph::Link("l3"));
  sg->addJoint(jc);
  EXPECT_THROW(tesseract::kinematics::gatherJointLimits(*sg, { "c" }), std::runtime_error);
}

TEST(TesseractKinematicsUnit, BuildSampleGrid)  // NOLINT
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

TEST(TesseractKinematicsUnit, BuildSampleGridRejectsMalformedInput)  // NOLINT
{
  const double inf = std::numeric_limits<double>::infinity();
  const double nan = std::numeric_limits<double>::quiet_NaN();

  const auto range = [](double lower, double upper) {
    Eigen::MatrixX2d m(1, 2);
    m << lower, upper;
    return m;
  };
  const auto res = [](double value) { return Eigen::VectorXd::Constant(1, value); };

  struct Case
  {
    const char* what;
    Eigen::MatrixX2d range;
    Eigen::VectorXd resolution;
  };

  // A joint whose limits were never set leaves an infinite bound, and narrowing the resulting
  // sample count is undefined; an inverted range would silently build a descending grid.
  const std::vector<Case> cases{
    { "joint count mismatch", range(0.0, 1.0), Eigen::VectorXd::Constant(2, 0.1) },
    { "infinite upper bound", range(0.0, inf), res(0.1) },
    { "infinite lower bound", range(-inf, 0.0), res(0.1) },
    { "NaN bound", range(0.0, nan), res(0.1) },
    { "inverted range", range(1.0, 0.0), res(0.1) },
    { "zero resolution", range(0.0, 1.0), res(0.0) },
    { "negative resolution", range(0.0, 1.0), res(-0.1) },
    { "infinite resolution", range(0.0, 1.0), res(inf) },
  };

  for (const Case& c : cases)
  {
    SCOPED_TRACE(c.what);
    EXPECT_THROW(tesseract::kinematics::buildSampleGrid(c.range, c.resolution), std::runtime_error);  // NOLINT
  }
}

TEST(TesseractKinematicsUnit, BuildSampleGridRejectsSampleCountAboveCap)  // NOLINT
{
  // A wide range at a fine resolution asks for more samples than the guard allows. Unguarded,
  // this narrowed to a garbage size that was handed straight to Eigen::VectorXd::LinSpaced.
  Eigen::MatrixX2d range(1, 2);
  range << 0.0, 1.0e6;
  Eigen::VectorXd res(1);
  res << 1.0e-6;
  EXPECT_THROW(tesseract::kinematics::buildSampleGrid(range, res), std::runtime_error);  // NOLINT

  // The guard must reject only above the cap, not before it: a legitimately fine discretisation
  // (0.1 mrad over a full revolution, ~63k samples) still has to build.
  Eigen::MatrixX2d fine_range(1, 2);
  fine_range << -M_PI, M_PI;
  Eigen::VectorXd fine_res(1);
  fine_res << 1.0e-4;
  std::vector<Eigen::VectorXd> fine_grid;
  EXPECT_NO_THROW(fine_grid = tesseract::kinematics::buildSampleGrid(fine_range, fine_res));  // NOLINT
  ASSERT_EQ(fine_grid.size(), 1U);
  EXPECT_EQ(fine_grid[0].size(), static_cast<Eigen::Index>(std::ceil(2.0 * M_PI / 1.0e-4)) + 1);
}

TEST(TesseractKinematicsUnit, AddRevoluteChildNonZeroDynamicLimitDefaults)  // NOLINT
{
  // Regression guard for kinematics_test_utils.h::defaultTestJointLimits() —
  // every dynamic band must default to a strictly positive value so
  // getTargetLimits() on graphs built via addRevoluteChild is usable.
  auto sg = std::make_shared<tesseract::scene_graph::SceneGraph>();
  sg->setName("test");
  sg->addLink(tesseract::scene_graph::Link("base"));
  sg->setRoot("base");

  namespace ts = tesseract::kinematics::test_suite;
  ts::addRevoluteChild(*sg, "j1", "base", "link1", Eigen::Vector3d::UnitZ());

  auto joint = sg->getJoint("j1");
  ASSERT_NE(joint, nullptr);
  ASSERT_NE(joint->limits, nullptr);
  EXPECT_GT(joint->limits->effort, 0.0);
  EXPECT_GT(joint->limits->velocity, 0.0);
  EXPECT_GT(joint->limits->acceleration, 0.0);
  EXPECT_GT(joint->limits->jerk, 0.0);
  EXPECT_LT(joint->limits->lower, joint->limits->upper);
}

TEST(TesseractKinematicsUnit, RTPInvKinCtorsRejectNullManipulator)  // NOLINT
{
  // All four public ctors must reject a null manipulator with std::runtime_error;
  // this test pins the exception type for every ctor variant.
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
