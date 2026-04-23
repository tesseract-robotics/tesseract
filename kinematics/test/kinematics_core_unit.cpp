#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/kinematics/kdl/kdl_fwd_kin_chain.h>
#include <tesseract/kinematics/kdl/kdl_inv_kin_chain_lma.h>
#include <tesseract/kinematics/inverse_kinematics.h>
#include <tesseract/kinematics/joint_group.h>
#include <tesseract/kinematics/kinematic_group.h>
#include <tesseract/kinematics/utils.h>
#include <tesseract/state_solver/kdl/kdl_state_solver.h>
#include <tesseract/common/types.h>
#include "kinematics_test_utils.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

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
             tesseract::common::LinkId base_link = tesseract::common::LinkId("base_link"))
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
  tesseract::kinematics::InverseKinematics::UPtr clone() const override
  {
    return std::make_unique<FakeInvKin>(*this);
  }

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
  const LinkId tool0 = LinkId("tool0");

  // First test joint 4, 5 and 6 at zero which should be in a singularity
  Eigen::VectorXd jv = Eigen::VectorXd::Zero(6);
  Eigen::MatrixXd jacobian(6, fwd_kin.numJoints());
  fwd_kin.calcJacobian(jacobian, jv, tool0);
  EXPECT_TRUE(tesseract::kinematics::isNearSingularity(jacobian, 0.001));

  // Set joint 5 angle to 1 deg and it with the default threshold it should still be in singularity
  jv[4] = 1 * M_PI / 180.0;
  fwd_kin.calcJacobian(jacobian, jv, tool0);
  EXPECT_TRUE(tesseract::kinematics::isNearSingularity(jacobian));

  // Set joint 5 angle to 2 deg and it should no longer be in a singularity
  jv[4] = 2 * M_PI / 180.0;
  fwd_kin.calcJacobian(jacobian, jv, tool0);
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
  const LinkId tool0 = LinkId("tool0");

  // First test joint 4, 5 and 6 at zero which should be in a singularity
  Eigen::VectorXd jv = Eigen::VectorXd::Zero(6);
  Eigen::MatrixXd jacobian(6, fwd_kin.numJoints());
  fwd_kin.calcJacobian(jacobian, jv, tool0);
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
// Phase 2 test additions — Integer link/joint ID tests for JointGroup
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
  EXPECT_FALSE(jg.isActiveLinkId(LinkId("nonexistent_link")));
  EXPECT_FALSE(jg.hasLinkId(LinkId("nonexistent_link")));

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
  const LinkId jac_link = active_link_ids.back();
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
  const LinkId intermediate_base = active_link_ids.front();
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
  const std::vector<JointId> joint_ids{ JointId("joint_a1"), JointId("joint_a2"), JointId("joint_a3"),
                                        JointId("joint_a4"), JointId("joint_a5"), JointId("joint_a6"),
                                        JointId("joint_a7") };

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
    mismatched.back() = JointId("bogus_joint");
    auto inv = std::make_unique<FakeInvKin>(joint_ids, base_link_id, std::vector<LinkId>{ tip_link_id });
    EXPECT_THROW(tesseract::kinematics::KinematicGroup("kg", mismatched, std::move(inv), *scene_graph, scene_state),
                 std::runtime_error);
  }

  // Same ids but reversed order: exercises the reorder_required_ / inv_kin_joint_map_ branch.
  {
    std::vector<JointId> reordered(joint_ids.rbegin(), joint_ids.rend());
    auto inv = std::make_unique<FakeInvKin>(joint_ids, base_link_id, std::vector<LinkId>{ tip_link_id });
    EXPECT_NO_THROW(tesseract::kinematics::KinematicGroup("kg_reordered",
                                                          reordered,
                                                          std::move(inv),
                                                          *scene_graph,
                                                          scene_state));
  }

  // Unknown working frame: exercises the working-frame link-transform lookup throw.
  {
    auto inv =
        std::make_unique<FakeInvKin>(joint_ids, LinkId("not_a_link"), std::vector<LinkId>{ tip_link_id });
    EXPECT_THROW(tesseract::kinematics::KinematicGroup("kg", joint_ids, std::move(inv), *scene_graph, scene_state),
                 std::runtime_error);
  }

  // Unknown tip link: exercises the tip-link link-transform lookup throw.
  {
    auto inv = std::make_unique<FakeInvKin>(joint_ids, base_link_id, std::vector<LinkId>{ LinkId("not_a_tip") });
    EXPECT_THROW(tesseract::kinematics::KinematicGroup("kg", joint_ids, std::move(inv), *scene_graph, scene_state),
                 std::runtime_error);
  }
}

TEST(TesseractKinematicsUnit, JointGroupCopyAssignmentAndNameAccessorsUnit)  // NOLINT
{
  using tesseract::common::JointId;
  using tesseract::common::LinkId;

  tesseract::common::GeneralResourceLocator locator;
  auto scene_graph = tesseract::kinematics::test_suite::getSceneGraphIIWA(locator);

  tesseract::scene_graph::KDLStateSolver ss(*scene_graph);
  const auto scene_state = ss.getState();

  const std::vector<JointId> joint_ids{ JointId("joint_a1"), JointId("joint_a2"), JointId("joint_a3"),
                                        JointId("joint_a4"), JointId("joint_a5"), JointId("joint_a6"),
                                        JointId("joint_a7") };

  tesseract::kinematics::JointGroup jg_a("manipulator_a", joint_ids, *scene_graph, scene_state);

  // String-name accessors (string overloads that go via toNames(ids)).
  const std::vector<std::string> joint_names = jg_a.getJointNames();
  EXPECT_EQ(joint_names.size(), joint_ids.size());
  for (std::size_t i = 0; i < joint_ids.size(); ++i)
    EXPECT_EQ(joint_names[i], joint_ids[i].name());

  const std::vector<std::string> link_names = jg_a.getLinkNames();
  EXPECT_FALSE(link_names.empty());
  EXPECT_EQ(link_names.size(), jg_a.getLinkIds().size());

  // Copy-assignment body (L142–147 in joint_group.cpp).
  const std::vector<JointId> subset{ joint_ids[0], joint_ids[1], joint_ids[2] };
  tesseract::kinematics::JointGroup jg_b("sub_manipulator", subset, *scene_graph, scene_state);
  EXPECT_EQ(jg_b.numJoints(), static_cast<Eigen::Index>(subset.size()));

  jg_b = jg_a;
  EXPECT_EQ(jg_b.numJoints(), jg_a.numJoints());
  EXPECT_EQ(jg_b.getJointIds(), jg_a.getJointIds());
  EXPECT_EQ(jg_b.getLinkIds().size(), jg_a.getLinkIds().size());

  // Self-assignment is a no-op.
  jg_b = jg_b;  // NOLINT(clang-diagnostic-self-assign-overloaded)
  EXPECT_EQ(jg_b.getJointIds(), joint_ids);

  // Missing-joint throw (joint_group.cpp:54).
  std::vector<JointId> with_bogus = joint_ids;
  with_bogus.push_back(JointId("does_not_exist_in_graph"));
  EXPECT_THROW(
      tesseract::kinematics::JointGroup("bad_group", with_bogus, *scene_graph, scene_state),
      std::runtime_error);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
