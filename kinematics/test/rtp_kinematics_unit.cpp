#include <tesseract/common/macros.h>
#include <set>
#include <utility>
#include <vector>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include "kinematics_test_utils.h"

#include <tesseract/kinematics/kdl/kdl_fwd_kin_chain.h>
#include <tesseract/kinematics/kinematics_plugin_factory.h>
#include <tesseract/kinematics/rtp_inv_kin.h>
#include <tesseract/kinematics/opw/opw_inv_kin.h>
#include <tesseract/kinematics/utils.h>
#include <tesseract/kinematics/kinematic_group.h>
#include <tesseract/state_solver/kdl/kdl_state_solver.h>
#include <tesseract/common/yaml_utils.h>
#include <opw_kinematics/opw_parameters.h>

using namespace tesseract::kinematics::test_suite;
using namespace tesseract::kinematics;

inline opw_kinematics::Parameters<double> getOPWKinematicsParamABB()
{
  opw_kinematics::Parameters<double> opw_params;
  opw_params.a1 = (0.100);
  opw_params.a2 = (-0.135);
  opw_params.b = (0.000);
  opw_params.c1 = (0.615);
  opw_params.c2 = (0.705);
  opw_params.c3 = (0.755);
  opw_params.c4 = (0.085);
  opw_params.offsets[2] = -M_PI / 2.0;
  return opw_params;
}

InverseKinematics::UPtr makeOPWInvKin(const tesseract::scene_graph::SceneGraph& scene_graph)
{
  auto robot_fwd_kin = std::make_unique<KDLFwdKinChain>(scene_graph, "base_link", "tool0");
  return std::make_unique<OPWInvKin>(getOPWKinematicsParamABB(),
                                     robot_fwd_kin->getBaseLinkName(),
                                     robot_fwd_kin->getTipLinkNames()[0],
                                     robot_fwd_kin->getJointNames());
}

ForwardKinematics::UPtr makeToolFwdKin(const tesseract::scene_graph::SceneGraph& scene_graph)
{
  return std::make_unique<KDLFwdKinChain>(scene_graph, "tool0", "tool_tip");
}

namespace
{
/** @brief Minimal stub IK exposing two tip link names — used to verify RTP rejects multi-tip manipulators. */
class TwoTipStubInvKin : public InverseKinematics
{
public:
  void calcInvKin(IKSolutions& /*solutions*/,
                  const tesseract::common::TransformMap& /*tip_link_poses*/,
                  const Eigen::Ref<const Eigen::VectorXd>& /*seed*/) const override
  {
  }
  std::vector<std::string> getJointNames() const override { return { "joint_1" }; }
  Eigen::Index numJoints() const override { return 1; }
  std::string getBaseLinkName() const override { return "base_link"; }
  std::string getWorkingFrame() const override { return "base_link"; }
  std::vector<std::string> getTipLinkNames() const override { return { "tip_a", "tip_b" }; }
  std::string getSolverName() const override { return "TwoTipStub"; }
  InverseKinematics::UPtr clone() const override { return std::make_unique<TwoTipStubInvKin>(*this); }
};

/** @brief Minimal stub FK with a configurable base link name — used to drive the RTP tool-base
 *         connectivity checks (base link not in scene graph; base link disconnected from manip tip).
 *         Joint name is also configurable so callers can supply a name that exists in the scene
 *         graph; otherwise gatherJointLimits would throw before init()'s connectivity check runs. */
class StubFwdKin : public ForwardKinematics
{
public:
  explicit StubFwdKin(std::string base, std::string joint = "tool_joint")
    : base_link_(std::move(base)), joint_name_(std::move(joint))
  {
  }
  void calcFwdKin(tesseract::common::TransformMap& /*transforms*/,
                  const Eigen::Ref<const Eigen::VectorXd>& /*joint_angles*/) const override
  {
  }
  void calcJacobian(Eigen::Ref<Eigen::MatrixXd> /*jacobian*/,
                    const Eigen::Ref<const Eigen::VectorXd>& /*joint_angles*/,
                    const std::string& /*link_name*/) const override
  {
  }
  std::string getBaseLinkName() const override { return base_link_; }
  std::vector<std::string> getJointNames() const override { return { joint_name_ }; }
  std::vector<std::string> getTipLinkNames() const override { return { "stub_tip" }; }
  Eigen::Index numJoints() const override { return 1; }
  std::string getSolverName() const override { return "StubFwd"; }
  ForwardKinematics::UPtr clone() const override { return std::make_unique<StubFwdKin>(*this); }

private:
  std::string base_link_;
  std::string joint_name_;
};

/** @brief Minimal stub IK that returns an empty solution set — drives the RTPInvKin::ikAt
 *         early-return when the inner manipulator IK finds no solutions. */
class EmptyInvKin : public InverseKinematics
{
public:
  void calcInvKin(IKSolutions& solutions,
                  const tesseract::common::TransformMap& /*tip_link_poses*/,
                  const Eigen::Ref<const Eigen::VectorXd>& /*seed*/) const override
  {
    solutions.clear();
  }
  std::vector<std::string> getJointNames() const override
  {
    return { "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6" };
  }
  Eigen::Index numJoints() const override { return 6; }
  std::string getBaseLinkName() const override { return "base_link"; }
  std::string getWorkingFrame() const override { return "base_link"; }
  std::vector<std::string> getTipLinkNames() const override { return { "tool0" }; }
  std::string getSolverName() const override { return "EmptyInv"; }
  InverseKinematics::UPtr clone() const override { return std::make_unique<EmptyInvKin>(*this); }
};
}  // namespace

TEST(TesseractKinematicsUnit, RTPInvKinMetadata)  // NOLINT
{
  tesseract::common::GeneralResourceLocator locator;
  auto scene_graph = getSceneGraphABBWithToolPositioner(locator);

  tesseract::scene_graph::KDLStateSolver state_solver(*scene_graph);
  tesseract::scene_graph::SceneState scene_state = state_solver.getState();

  auto opw_kin = makeOPWInvKin(*scene_graph);
  auto tool_kin = makeToolFwdKin(*scene_graph);

  Eigen::VectorXd tool_resolution = Eigen::VectorXd::Constant(1, 0.1);
  auto rtp =
      std::make_unique<RTPInvKin>(*scene_graph, scene_state, opw_kin->clone(), 2.0, tool_kin->clone(), tool_resolution);

  EXPECT_EQ(rtp->getSolverName(), DEFAULT_RTP_INV_KIN_SOLVER_NAME);
  EXPECT_EQ(rtp->numJoints(), 7);
  EXPECT_EQ(rtp->getBaseLinkName(), "base_link");
  EXPECT_EQ(rtp->getWorkingFrame(), "base_link");
  ASSERT_EQ(rtp->getTipLinkNames().size(), 1);
  EXPECT_EQ(rtp->getTipLinkNames()[0], "tool_tip");

  std::vector<std::string> expected_joints{ "joint_1", "joint_2", "joint_3",   "joint_4",
                                            "joint_5", "joint_6", "tool_joint" };
  EXPECT_EQ(rtp->getJointNames(), expected_joints);
}

TEST(TesseractKinematicsUnit, RTPInvKinConstructorValidation)  // NOLINT
{
  tesseract::common::GeneralResourceLocator locator;
  auto scene_graph = getSceneGraphABBWithToolPositioner(locator);
  tesseract::scene_graph::KDLStateSolver state_solver(*scene_graph);
  tesseract::scene_graph::SceneState scene_state = state_solver.getState();

  auto opw_kin = makeOPWInvKin(*scene_graph);
  auto tool_kin = makeToolFwdKin(*scene_graph);
  Eigen::VectorXd tool_resolution = Eigen::VectorXd::Constant(1, 0.1);

  {  // Empty scene graph
    tesseract::scene_graph::SceneGraph empty_sg;
    EXPECT_ANY_THROW(std::make_unique<RTPInvKin>(
        empty_sg, scene_state, opw_kin->clone(), 2.0, tool_kin->clone(), tool_resolution));  // NOLINT
  }
  {  // Empty solver name
    EXPECT_ANY_THROW(std::make_unique<RTPInvKin>(
        *scene_graph, scene_state, opw_kin->clone(), 2.0, tool_kin->clone(), tool_resolution, ""));  // NOLINT
  }
  {  // Null manipulator
    EXPECT_ANY_THROW(std::make_unique<RTPInvKin>(
        *scene_graph, scene_state, nullptr, 2.0, tool_kin->clone(), tool_resolution));  // NOLINT
  }
  {  // Zero reach
    EXPECT_ANY_THROW(std::make_unique<RTPInvKin>(
        *scene_graph, scene_state, opw_kin->clone(), 0.0, tool_kin->clone(), tool_resolution));  // NOLINT
  }
  {  // Null tool positioner
    EXPECT_ANY_THROW(std::make_unique<RTPInvKin>(
        *scene_graph, scene_state, opw_kin->clone(), 2.0, nullptr, tool_resolution));  // NOLINT
  }
  {  // Empty resolution
    Eigen::VectorXd bad_res;
    EXPECT_ANY_THROW(std::make_unique<RTPInvKin>(
        *scene_graph, scene_state, opw_kin->clone(), 2.0, tool_kin->clone(), bad_res));  // NOLINT
  }
  {  // Negative resolution
    Eigen::VectorXd neg_res = Eigen::VectorXd::Constant(1, -0.1);
    EXPECT_ANY_THROW(std::make_unique<RTPInvKin>(
        *scene_graph, scene_state, opw_kin->clone(), 2.0, tool_kin->clone(), neg_res));  // NOLINT
  }
  {  // Auto-reach ctor: empty scene graph
    tesseract::scene_graph::SceneGraph empty_sg;
    EXPECT_ANY_THROW(
        std::make_unique<RTPInvKin>(empty_sg, scene_state, opw_kin->clone(), tool_kin->clone(), tool_resolution));
  }
  {  // Auto-reach ctor: null manipulator
    EXPECT_ANY_THROW(
        std::make_unique<RTPInvKin>(*scene_graph, scene_state, nullptr, tool_kin->clone(), tool_resolution));
  }
  {  // Auto-reach ctor: null tool positioner
    EXPECT_ANY_THROW(
        std::make_unique<RTPInvKin>(*scene_graph, scene_state, opw_kin->clone(), nullptr, tool_resolution));
  }
  {  // Inverted tool sample range (min > max)
    Eigen::MatrixX2d bad_range(1, 2);
    bad_range << 1.0, -1.0;
    EXPECT_ANY_THROW(std::make_unique<RTPInvKin>(
        *scene_graph, scene_state, opw_kin->clone(), 2.0, tool_kin->clone(), bad_range, tool_resolution));  // NOLINT
  }
  Eigen::MatrixX2d range(1, 2);
  range << -M_PI, M_PI;
  {  // Explicit-range ctor with explicit reach: null tool positioner
    EXPECT_ANY_THROW(std::make_unique<RTPInvKin>(
        *scene_graph, scene_state, opw_kin->clone(), 2.0, nullptr, range, tool_resolution));  // NOLINT
  }
  {  // Explicit-range auto-reach ctor: null tool positioner
    EXPECT_ANY_THROW(std::make_unique<RTPInvKin>(
        *scene_graph, scene_state, opw_kin->clone(), nullptr, range, tool_resolution));  // NOLINT
  }
  {  // Explicit-range auto-reach ctor: multi-tip manipulator
    EXPECT_ANY_THROW(std::make_unique<RTPInvKin>(
        *scene_graph, scene_state, std::make_unique<TwoTipStubInvKin>(), tool_kin->clone(), range, tool_resolution));
  }
  {  // tool_sample_range row count mismatch (2 rows, 1-DOF tool positioner)
    Eigen::MatrixX2d wrong_range(2, 2);
    wrong_range << -1.0, 1.0, -1.0, 1.0;
    EXPECT_ANY_THROW(std::make_unique<RTPInvKin>(
        *scene_graph, scene_state, opw_kin->clone(), 2.0, tool_kin->clone(), wrong_range, tool_resolution));  // NOLINT
  }
  {  // Empty scene graph via the explicit-range ctor — the only public path that doesn't run
     // gatherJointLimits / computeChainReach first, so init()'s root-link check fires.
    tesseract::scene_graph::SceneGraph empty_sg;
    EXPECT_ANY_THROW(std::make_unique<RTPInvKin>(
        empty_sg, scene_state, opw_kin->clone(), 2.0, tool_kin->clone(), range, tool_resolution));  // NOLINT
  }
}

TEST(TesseractKinematicsUnit, RTPInvKinAutoReachMatchesExplicit)  // NOLINT
{
  tesseract::common::GeneralResourceLocator locator;
  auto scene_graph = getSceneGraphABBWithToolPositioner(locator);
  tesseract::scene_graph::KDLStateSolver state_solver(*scene_graph);
  tesseract::scene_graph::SceneState scene_state = state_solver.getState();

  auto opw_kin = makeOPWInvKin(*scene_graph);
  auto tool_kin = makeToolFwdKin(*scene_graph);
  Eigen::VectorXd tool_resolution = Eigen::VectorXd::Constant(1, 0.1);

  // The auto-reach ctor should succeed without throwing. We don't inspect the reach directly
  // (it's private) but we verify the RTP behaves identically to one built with an explicit reach
  // that is known-large-enough (3.0).
  auto rtp_auto =
      std::make_unique<RTPInvKin>(*scene_graph, scene_state, opw_kin->clone(), tool_kin->clone(), tool_resolution);
  auto rtp_explicit =
      std::make_unique<RTPInvKin>(*scene_graph, scene_state, opw_kin->clone(), 3.0, tool_kin->clone(), tool_resolution);

  EXPECT_EQ(rtp_auto->numJoints(), rtp_explicit->numJoints());
  EXPECT_EQ(rtp_auto->getBaseLinkName(), rtp_explicit->getBaseLinkName());
  EXPECT_EQ(rtp_auto->getTipLinkNames(), rtp_explicit->getTipLinkNames());
  EXPECT_EQ(rtp_auto->getJointNames(), rtp_explicit->getJointNames());

  // Behavioural check: feed a reachable tool_tip target and confirm both return solutions.
  // Pick a mid-workspace target via FK roundtrip.
  auto fwd_full = std::make_unique<KDLFwdKinChain>(*scene_graph, "base_link", "tool_tip");
  Eigen::VectorXd q = Eigen::VectorXd::Zero(7);
  q(1) = -0.3;  // lift joint_2 a bit to escape singular home
  tesseract::common::TransformMap poses;
  fwd_full->calcFwdKin(poses, q);
  tesseract::common::TransformMap target{ { "tool_tip", poses.at("tool_tip") } };

  IKSolutions s_auto, s_explicit;
  rtp_auto->calcInvKin(s_auto, target, Eigen::VectorXd::Zero(7));
  rtp_explicit->calcInvKin(s_explicit, target, Eigen::VectorXd::Zero(7));
  EXPECT_FALSE(s_auto.empty());
  EXPECT_EQ(s_auto.size(), s_explicit.size());

  // Bounded-reach check: an undersized explicit reach must filter the same target out via the
  // reach gate. If auto-reach silently degenerated to "always accept" this third arm would still
  // produce solutions, masking the bug.
  auto rtp_undersized =
      std::make_unique<RTPInvKin>(*scene_graph, scene_state, opw_kin->clone(), 0.3, tool_kin->clone(), tool_resolution);
  IKSolutions s_undersized;
  rtp_undersized->calcInvKin(s_undersized, target, Eigen::VectorXd::Zero(7));
  EXPECT_TRUE(s_undersized.empty());
}

TEST(TesseractKinematicsUnit, RTPInvKinSingleSampleRoundtrip)  // NOLINT
{
  tesseract::common::GeneralResourceLocator locator;
  auto scene_graph = getSceneGraphABBWithToolPositioner(locator);
  tesseract::scene_graph::KDLStateSolver state_solver(*scene_graph);
  tesseract::scene_graph::SceneState scene_state = state_solver.getState();

  // Single-sample sweep: resolution larger than the joint range -> one grid point at q=lower.
  Eigen::MatrixX2d tool_range(1, 2);
  tool_range << 0.0, 0.0;  // Lock tool joint at 0.
  Eigen::VectorXd tool_resolution = Eigen::VectorXd::Constant(1, 10.0);

  auto rtp = std::make_unique<RTPInvKin>(*scene_graph,
                                         scene_state,
                                         makeOPWInvKin(*scene_graph),
                                         2.0,
                                         makeToolFwdKin(*scene_graph),
                                         tool_range,
                                         tool_resolution);

  // Pick a joint vector with tool=0, compute FK for tool_tip, then invert.
  auto full_fwd_kin = KDLFwdKinChain(*scene_graph, "base_link", "tool_tip");
  Eigen::VectorXd q(7);
  q << 0.1, -0.2, 0.3, 0.0, 0.5, 0.0, 0.0;
  tesseract::common::TransformMap fwd_poses;
  full_fwd_kin.calcFwdKin(fwd_poses, q);
  Eigen::Isometry3d tool_tip_pose = fwd_poses.at("tool_tip");

  tesseract::common::TransformMap target;
  target["tool_tip"] = tool_tip_pose;

  IKSolutions solutions;
  Eigen::VectorXd seed = q;
  rtp->calcInvKin(solutions, target, seed);

  ASSERT_FALSE(solutions.empty());

  // At least one solution must reproduce tool_tip_pose when passed through FK.
  bool matched = false;
  const double tol = 1e-4;
  for (const auto& sol : solutions)
  {
    ASSERT_EQ(sol.size(), 7);
    EXPECT_NEAR(sol(6), 0.0, 1e-9);  // Tool joint locked at 0.

    tesseract::common::TransformMap check_poses;
    full_fwd_kin.calcFwdKin(check_poses, sol);
    Eigen::Isometry3d check = check_poses.at("tool_tip");

    if ((check.translation() - tool_tip_pose.translation()).norm() < tol &&
        Eigen::Quaterniond(check.linear()).angularDistance(Eigen::Quaterniond(tool_tip_pose.linear())) < tol)
    {
      matched = true;
      break;
    }
  }
  EXPECT_TRUE(matched);
}

TEST(TesseractKinematicsUnit, RTPInvKinMultiSampleSweep)  // NOLINT
{
  tesseract::common::GeneralResourceLocator locator;
  auto scene_graph = getSceneGraphABBWithToolPositioner(locator);
  tesseract::scene_graph::KDLStateSolver state_solver(*scene_graph);
  tesseract::scene_graph::SceneState scene_state = state_solver.getState();

  // Sweep tool joint from -pi/2 to pi/2 at 0.1 rad resolution -> ~32 samples.
  Eigen::MatrixX2d tool_range(1, 2);
  tool_range << -M_PI / 2.0, M_PI / 2.0;
  Eigen::VectorXd tool_resolution = Eigen::VectorXd::Constant(1, 0.1);

  auto rtp = std::make_unique<RTPInvKin>(*scene_graph,
                                         scene_state,
                                         makeOPWInvKin(*scene_graph),
                                         2.0,
                                         makeToolFwdKin(*scene_graph),
                                         tool_range,
                                         tool_resolution);

  // Reachable pose in the middle of the ABB workspace.
  auto full_fwd_kin = KDLFwdKinChain(*scene_graph, "base_link", "tool_tip");
  Eigen::VectorXd q(7);
  q << 0.0, 0.2, -0.3, 0.0, 0.5, 0.0, 0.4;  // tool at 0.4 rad
  tesseract::common::TransformMap fwd_poses;
  full_fwd_kin.calcFwdKin(fwd_poses, q);
  Eigen::Isometry3d target_pose = fwd_poses.at("tool_tip");

  tesseract::common::TransformMap target;
  target["tool_tip"] = target_pose;

  IKSolutions solutions;
  Eigen::VectorXd seed = Eigen::VectorXd::Zero(7);
  rtp->calcInvKin(solutions, target, seed);

  // Multiple tool samples should produce multiple valid solutions (OPW yields up to 8 branches
  // per manipulator-tip target, multiplied across tool samples that remain reachable).
  EXPECT_GT(solutions.size(), 1U);

  // Every reported solution must be a valid FK roundtrip.
  const double tol = 1e-4;
  std::size_t valid = 0;
  for (const auto& sol : solutions)
  {
    ASSERT_EQ(sol.size(), 7);
    tesseract::common::TransformMap check_poses;
    full_fwd_kin.calcFwdKin(check_poses, sol);
    Eigen::Isometry3d check = check_poses.at("tool_tip");

    if ((check.translation() - target_pose.translation()).norm() < tol &&
        Eigen::Quaterniond(check.linear()).angularDistance(Eigen::Quaterniond(target_pose.linear())) < tol)
      ++valid;
  }
  EXPECT_EQ(valid, solutions.size());

  // At least one solution's tool value should be near q(6) = 0.4.
  // LinSpaced(33, -pi/2, pi/2) has step pi/32 ~ 0.0982; nearest grid point to 0.4 is ~0.3927.
  bool found_tool_match = false;
  for (const auto& sol : solutions)
  {
    if (std::abs(sol(6) - 0.4) < 0.06)  // 0.1 resolution -> within 0.06 is plenty
    {
      found_tool_match = true;
      break;
    }
  }
  EXPECT_TRUE(found_tool_match);
}

TEST(TesseractKinematicsUnit, RTPInvKinCloneAndKinematicGroup)  // NOLINT
{
  tesseract::common::GeneralResourceLocator locator;
  auto scene_graph = getSceneGraphABBWithToolPositioner(locator);
  tesseract::scene_graph::KDLStateSolver state_solver(*scene_graph);
  tesseract::scene_graph::SceneState scene_state = state_solver.getState();

  Eigen::VectorXd tool_resolution = Eigen::VectorXd::Constant(1, 0.1);
  auto rtp = std::make_unique<RTPInvKin>(
      *scene_graph, scene_state, makeOPWInvKin(*scene_graph), 2.0, makeToolFwdKin(*scene_graph), tool_resolution);

  auto cloned = rtp->clone();
  EXPECT_EQ(cloned->getSolverName(), DEFAULT_RTP_INV_KIN_SOLVER_NAME);
  EXPECT_EQ(cloned->numJoints(), 7);
  EXPECT_EQ(cloned->getJointNames(), rtp->getJointNames());
  EXPECT_EQ(cloned->getBaseLinkName(), rtp->getBaseLinkName());
  ASSERT_EQ(cloned->getTipLinkNames().size(), 1U);
  EXPECT_EQ(cloned->getTipLinkNames()[0], rtp->getTipLinkNames()[0]);

  std::vector<std::string> joint_names{
    "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "tool_joint"
  };
  KinematicGroup kin_group("rtp_manip", joint_names, std::move(cloned), *scene_graph, scene_state);
  EXPECT_EQ(kin_group.getBaseLinkName(), scene_graph->getRoot());
  EXPECT_EQ(kin_group.getName(), "rtp_manip");
  EXPECT_EQ(kin_group.getJointNames(), joint_names);

  auto tip_names = kin_group.getAllPossibleTipLinkNames();
  EXPECT_NE(std::find(tip_names.begin(), tip_names.end(), "tool_tip"), tip_names.end());
}

TEST(TesseractKinematicsUnit, RTPInvKinCopyAssign)  // NOLINT
{
  tesseract::common::GeneralResourceLocator locator;
  auto scene_graph = getSceneGraphABBWithToolPositioner(locator);
  tesseract::scene_graph::KDLStateSolver state_solver(*scene_graph);
  tesseract::scene_graph::SceneState scene_state = state_solver.getState();

  Eigen::VectorXd tool_resolution = Eigen::VectorXd::Constant(1, 0.1);

  auto rtp_src = std::make_unique<RTPInvKin>(*scene_graph,
                                             scene_state,
                                             makeOPWInvKin(*scene_graph),
                                             2.0,
                                             makeToolFwdKin(*scene_graph),
                                             tool_resolution,
                                             "rtp_src_solver");

  Eigen::VectorXd alt_resolution = Eigen::VectorXd::Constant(1, 0.5);
  auto rtp_dst = std::make_unique<RTPInvKin>(*scene_graph,
                                             scene_state,
                                             makeOPWInvKin(*scene_graph),
                                             1.5,
                                             makeToolFwdKin(*scene_graph),
                                             alt_resolution,
                                             "rtp_dst_solver");
  ASSERT_NE(rtp_dst->getSolverName(), rtp_src->getSolverName());

  // Self-assignment must be a no-op (early-return at operator= top).
#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wself-assign-overloaded"
#endif
  *rtp_src = *rtp_src;
#if defined(__clang__)
#pragma clang diagnostic pop
#endif
  EXPECT_EQ(rtp_src->getSolverName(), "rtp_src_solver");

  *rtp_dst = *rtp_src;
  EXPECT_EQ(rtp_dst->getSolverName(), rtp_src->getSolverName());
  EXPECT_EQ(rtp_dst->getJointNames(), rtp_src->getJointNames());
  EXPECT_EQ(rtp_dst->getBaseLinkName(), rtp_src->getBaseLinkName());
  EXPECT_EQ(rtp_dst->getTipLinkNames(), rtp_src->getTipLinkNames());
  EXPECT_EQ(rtp_dst->numJoints(), rtp_src->numJoints());
}

TEST(TesseractKinematicsUnit, RTPInvKinFactoryYaml)  // NOLINT
{
  tesseract::common::GeneralResourceLocator locator;
  auto scene_graph = getSceneGraphABBWithToolPositioner(locator);
  tesseract::scene_graph::KDLStateSolver state_solver(*scene_graph);
  tesseract::scene_graph::SceneState scene_state = state_solver.getState();

  const std::string yaml_str = R"(
kinematic_plugins:
  search_libraries:
    - tesseract_kinematics_factories
  inv_kin_plugins:
    rtp_manipulator:
      default: RTPInvKin
      plugins:
        RTPInvKin:
          class: RTPInvKinFactory
          config:
            manipulator_reach: 2.0
            tool_sample_resolution:
              - name: tool_joint
                value: 0.1
            tool_positioner:
              class: KDLFwdKinChainFactory
              config:
                base_link: tool0
                tip_link: tool_tip
            manipulator:
              class: OPWInvKinFactory
              config:
                base_link: base_link
                tip_link: tool0
                params:
                  a1: 0.100
                  a2: -0.135
                  b: 0.00
                  c1: 0.615
                  c2: 0.705
                  c3: 0.755
                  c4: 0.086
                  offsets: [0, 0, -1.57079632679, 0, 0, 0]
                  sign_corrections: [1, 1, 1, 1, 1, 1]
)";

  KinematicsPluginFactory factory(YAML::Load(yaml_str), locator);
  auto loaded = factory.createInvKin("rtp_manipulator", "RTPInvKin", *scene_graph, scene_state);
  ASSERT_NE(loaded, nullptr);
  EXPECT_EQ(loaded->numJoints(), 7);
  ASSERT_EQ(loaded->getTipLinkNames().size(), 1U);
  EXPECT_EQ(loaded->getTipLinkNames()[0], "tool_tip");
  EXPECT_EQ(loaded->getBaseLinkName(), "base_link");
}

TEST(TesseractKinematicsUnit, RTPInvKinFactoryAutoReach)  // NOLINT
{
  tesseract::common::GeneralResourceLocator locator;
  auto scene_graph = getSceneGraphABBWithToolPositioner(locator);
  tesseract::scene_graph::KDLStateSolver state_solver(*scene_graph);
  tesseract::scene_graph::SceneState scene_state = state_solver.getState();

  // Note: no `manipulator_reach` - factory must auto-derive it.
  const std::string yaml_str = R"(
kinematic_plugins:
  search_libraries:
    - tesseract_kinematics_factories
  inv_kin_plugins:
    rtp_manipulator:
      default: RTPInvKin
      plugins:
        RTPInvKin:
          class: RTPInvKinFactory
          config:
            tool_sample_resolution:
              - name: tool_joint
                value: 0.1
            tool_positioner:
              class: KDLFwdKinChainFactory
              config:
                base_link: tool0
                tip_link: tool_tip
            manipulator:
              class: OPWInvKinFactory
              config:
                base_link: base_link
                tip_link: tool0
                params:
                  a1: 0.100
                  a2: -0.135
                  b: 0.00
                  c1: 0.615
                  c2: 0.705
                  c3: 0.755
                  c4: 0.086
                  offsets: [0, 0, -1.57079632679, 0, 0, 0]
                  sign_corrections: [1, 1, 1, 1, 1, 1]
)";

  KinematicsPluginFactory factory(YAML::Load(yaml_str), locator);
  auto loaded = factory.createInvKin("rtp_manipulator", "RTPInvKin", *scene_graph, scene_state);
  ASSERT_NE(loaded, nullptr);
  EXPECT_EQ(loaded->numJoints(), 7);
}

TEST(TesseractKinematicsUnit, RTPInvKinFactoryFailureMatrix)  // NOLINT
{
  tesseract::common::GeneralResourceLocator locator;
  auto scene_graph = getSceneGraphABBWithToolPositioner(locator);
  tesseract::scene_graph::KDLStateSolver state_solver(*scene_graph);
  tesseract::scene_graph::SceneState scene_state = state_solver.getState();

  const std::string yaml_str = R"(
kinematic_plugins:
  search_libraries:
    - tesseract_kinematics_factories
  inv_kin_plugins:
    rtp_manipulator:
      default: RTPInvKin
      plugins:
        RTPInvKin:
          class: RTPInvKinFactory
          config:
            manipulator_reach: 2.0
            tool_sample_resolution:
              - name: tool_joint
                value: 0.1
            tool_positioner:
              class: KDLFwdKinChainFactory
              config:
                base_link: tool0
                tip_link: tool_tip
            manipulator:
              class: OPWInvKinFactory
              config:
                base_link: base_link
                tip_link: tool0
                params:
                  a1: 0.100
                  a2: -0.135
                  b: 0.00
                  c1: 0.615
                  c2: 0.705
                  c3: 0.755
                  c4: 0.086
                  offsets: [0, 0, -1.57079632679, 0, 0, 0]
                  sign_corrections: [1, 1, 1, 1, 1, 1]
)";

  // Sanity: the unmodified yaml must produce a valid solver.
  {
    KinematicsPluginFactory factory(YAML::Load(yaml_str), locator);
    EXPECT_NE(factory.createInvKin("rtp_manipulator", "RTPInvKin", *scene_graph, scene_state), nullptr);
  }

  auto load_failure_expected = [&](const YAML::Node& config) {
    KinematicsPluginFactory factory(config, locator);
    EXPECT_EQ(factory.createInvKin("rtp_manipulator", "RTPInvKin", *scene_graph, scene_state), nullptr);
  };

  {  // Missing config block
    YAML::Node config = tesseract::common::loadYamlString(yaml_str, locator);
    auto plugin = config["kinematic_plugins"]["inv_kin_plugins"]["rtp_manipulator"]["plugins"]["RTPInvKin"];
    plugin.remove("config");
    load_failure_expected(config);
  }
  {  // Non-positive manipulator_reach
    YAML::Node config = tesseract::common::loadYamlString(yaml_str, locator);
    config["kinematic_plugins"]["inv_kin_plugins"]["rtp_manipulator"]["plugins"]["RTPInvKin"]["config"]["manipulator_"
                                                                                                        "reach"] = -1.0;
    load_failure_expected(config);
  }
  {  // Missing tool_sample_resolution
    YAML::Node config = tesseract::common::loadYamlString(yaml_str, locator);
    auto cfg = config["kinematic_plugins"]["inv_kin_plugins"]["rtp_manipulator"]["plugins"]["RTPInvKin"]["config"];
    cfg.remove("tool_sample_resolution");
    load_failure_expected(config);
  }
  {  // tool_sample_resolution entry missing 'name'
    YAML::Node config = tesseract::common::loadYamlString(yaml_str, locator);
    config["kinematic_plugins"]["inv_kin_plugins"]["rtp_manipulator"]["plugins"]["RTPInvKin"]["config"]["tool_sample_"
                                                                                                        "resolution"][0]
        .remove("name");
    load_failure_expected(config);
  }
  {  // tool_sample_resolution entry missing 'value'
    YAML::Node config = tesseract::common::loadYamlString(yaml_str, locator);
    config["kinematic_plugins"]["inv_kin_plugins"]["rtp_manipulator"]["plugins"]["RTPInvKin"]["config"]["tool_sample_"
                                                                                                        "resolution"][0]
        .remove("value");
    load_failure_expected(config);
  }
  {  // tool_sample_resolution joint name not in scene graph
    YAML::Node config = tesseract::common::loadYamlString(yaml_str, locator);
    config["kinematic_plugins"]["inv_kin_plugins"]["rtp_manipulator"]["plugins"]["RTPInvKin"]["config"]
          ["tool_sample_resolution"][0]["name"] = "joint_does_not_exist";
    load_failure_expected(config);
  }
  {  // tool_sample_resolution min below joint lower limit
    YAML::Node config = tesseract::common::loadYamlString(yaml_str, locator);
    config["kinematic_plugins"]["inv_kin_plugins"]["rtp_manipulator"]["plugins"]["RTPInvKin"]["config"]
          ["tool_sample_resolution"][0]["min"] = -10000.0;
    load_failure_expected(config);
  }
  {  // tool_sample_resolution max above joint upper limit
    YAML::Node config = tesseract::common::loadYamlString(yaml_str, locator);
    config["kinematic_plugins"]["inv_kin_plugins"]["rtp_manipulator"]["plugins"]["RTPInvKin"]["config"]
          ["tool_sample_resolution"][0]["max"] = 10000.0;
    load_failure_expected(config);
  }
  {  // tool_sample_resolution min greater than max
    YAML::Node config = tesseract::common::loadYamlString(yaml_str, locator);
    auto entry =
        config["kinematic_plugins"]["inv_kin_plugins"]["rtp_manipulator"]["plugins"]["RTPInvKin"]["config"]["tool_"
                                                                                                            "sample_"
                                                                                                            "resolutio"
                                                                                                            "n"][0];
    entry["min"] = 0.5;
    entry["max"] = 0.1;
    load_failure_expected(config);
  }
  {  // Missing tool_positioner
    YAML::Node config = tesseract::common::loadYamlString(yaml_str, locator);
    auto cfg = config["kinematic_plugins"]["inv_kin_plugins"]["rtp_manipulator"]["plugins"]["RTPInvKin"]["config"];
    cfg.remove("tool_positioner");
    load_failure_expected(config);
  }
  {  // tool_positioner missing class entry
    YAML::Node config = tesseract::common::loadYamlString(yaml_str, locator);
    auto pos = config["kinematic_plugins"]["inv_kin_plugins"]["rtp_manipulator"]["plugins"]["RTPInvKin"]["config"]["too"
                                                                                                                   "l_"
                                                                                                                   "pos"
                                                                                                                   "iti"
                                                                                                                   "one"
                                                                                                                   "r"];
    pos.remove("class");
    load_failure_expected(config);
  }
  {  // tool_positioner with unregistered class
    YAML::Node config = tesseract::common::loadYamlString(yaml_str, locator);
    config["kinematic_plugins"]["inv_kin_plugins"]["rtp_manipulator"]["plugins"]["RTPInvKin"]["config"]
          ["tool_positioner"]["class"] = "DoesNotExistFactory";
    load_failure_expected(config);
  }
  {  // Missing manipulator
    YAML::Node config = tesseract::common::loadYamlString(yaml_str, locator);
    auto cfg = config["kinematic_plugins"]["inv_kin_plugins"]["rtp_manipulator"]["plugins"]["RTPInvKin"]["config"];
    cfg.remove("manipulator");
    load_failure_expected(config);
  }
  {  // manipulator missing class entry
    YAML::Node config = tesseract::common::loadYamlString(yaml_str, locator);
    auto manip =
        config["kinematic_plugins"]["inv_kin_plugins"]["rtp_manipulator"]["plugins"]["RTPInvKin"]["config"]["manipulato"
                                                                                                            "r"];
    manip.remove("class");
    load_failure_expected(config);
  }
  {  // manipulator with unregistered class
    YAML::Node config = tesseract::common::loadYamlString(yaml_str, locator);
    config["kinematic_plugins"]["inv_kin_plugins"]["rtp_manipulator"]["plugins"]["RTPInvKin"]["config"]["manipulator"]
          ["class"] = "DoesNotExistFactory";
    load_failure_expected(config);
  }
  {  // tool_sample_resolution has more entries than tool positioner has joints
    YAML::Node config = tesseract::common::loadYamlString(yaml_str, locator);
    YAML::Node extra;
    extra["name"] = "joint_1";  // exists in scene graph and has limits, but not in the tool chain
    extra["value"] = 0.1;
    config["kinematic_plugins"]["inv_kin_plugins"]["rtp_manipulator"]["plugins"]["RTPInvKin"]["config"]["tool_sample_"
                                                                                                        "resolution"]
        .push_back(extra);
    load_failure_expected(config);
  }
  {  // tool_sample_resolution names a non-tool-chain joint (size matches but lookup misses)
    YAML::Node config = tesseract::common::loadYamlString(yaml_str, locator);
    config["kinematic_plugins"]["inv_kin_plugins"]["rtp_manipulator"]["plugins"]["RTPInvKin"]["config"]
          ["tool_sample_resolution"][0]["name"] = "joint_1";
    load_failure_expected(config);
  }
}

TEST(TesseractKinematicsUnit, RTPInvKinFactoryRejectsBadReach)  // NOLINT
{
  // Regression: a non-positive manipulator_reach must be reported as a load
  // failure, not propagate as an uncaught ctor exception out of the factory.
  tesseract::common::GeneralResourceLocator locator;
  auto scene_graph = getSceneGraphABBWithToolPositioner(locator);
  tesseract::scene_graph::KDLStateSolver state_solver(*scene_graph);
  tesseract::scene_graph::SceneState scene_state = state_solver.getState();

  const std::string yaml_str = R"(
kinematic_plugins:
  search_libraries:
    - tesseract_kinematics_factories
  inv_kin_plugins:
    rtp_manipulator:
      default: RTPInvKin
      plugins:
        RTPInvKin:
          class: RTPInvKinFactory
          config:
            manipulator_reach: -1.0
            tool_sample_resolution:
              - name: tool_joint
                value: 0.1
            tool_positioner:
              class: KDLFwdKinChainFactory
              config:
                base_link: tool0
                tip_link: tool_tip
            manipulator:
              class: OPWInvKinFactory
              config:
                base_link: base_link
                tip_link: tool0
                params:
                  a1: 0.100
                  a2: -0.135
                  b: 0.00
                  c1: 0.615
                  c2: 0.705
                  c3: 0.755
                  c4: 0.086
                  offsets: [0, 0, -1.57079632679, 0, 0, 0]
                  sign_corrections: [1, 1, 1, 1, 1, 1]
)";

  KinematicsPluginFactory factory(YAML::Load(yaml_str), locator);
  auto loaded = factory.createInvKin("rtp_manipulator", "RTPInvKin", *scene_graph, scene_state);
  EXPECT_EQ(loaded, nullptr);
}

TEST(TesseractKinematicsUnit, RTPInvKinFactoryRejectsJointWithoutLimits)  // NOLINT
{
  // Exercises parseSampleResolutionMap's "joint has no limits" branch by mutating the scene graph
  // post-URDF-parse to drop the limits on tool_joint, then asking the factory to load against it.
  tesseract::common::GeneralResourceLocator locator;
  auto scene_graph = getSceneGraphABBWithToolPositioner(locator);
  // Build state solver BEFORE dropping limits — KDLStateSolver dereferences joint limits during
  // construction, so mutating must happen after.
  tesseract::scene_graph::KDLStateSolver state_solver(*scene_graph);
  tesseract::scene_graph::SceneState scene_state = state_solver.getState();
  auto j = std::const_pointer_cast<tesseract::scene_graph::Joint>(scene_graph->getJoint("tool_joint"));
  j->limits = nullptr;

  const std::string yaml_str = R"(
kinematic_plugins:
  search_libraries:
    - tesseract_kinematics_factories
  inv_kin_plugins:
    rtp_manipulator:
      default: RTPInvKin
      plugins:
        RTPInvKin:
          class: RTPInvKinFactory
          config:
            manipulator_reach: 2.0
            tool_sample_resolution:
              - name: tool_joint
                value: 0.1
            tool_positioner:
              class: KDLFwdKinChainFactory
              config:
                base_link: tool0
                tip_link: tool_tip
            manipulator:
              class: OPWInvKinFactory
              config:
                base_link: base_link
                tip_link: tool0
                params:
                  a1: 0.100
                  a2: -0.135
                  b: 0.00
                  c1: 0.615
                  c2: 0.705
                  c3: 0.755
                  c4: 0.086
                  offsets: [0, 0, -1.57079632679, 0, 0, 0]
                  sign_corrections: [1, 1, 1, 1, 1, 1]
)";

  KinematicsPluginFactory factory(YAML::Load(yaml_str), locator);
  EXPECT_EQ(factory.createInvKin("rtp_manipulator", "RTPInvKin", *scene_graph, scene_state), nullptr);
}

TEST(TesseractKinematicsUnit, RTPInvKinRejectsMultiTipManipulator)  // NOLINT
{
  // RTP's static-offset model requires a single manipulator tip; a multi-tip manipulator would
  // silently use only the first tip and discard the rest. The ctor must reject this.
  tesseract::common::GeneralResourceLocator locator;
  auto scene_graph = getSceneGraphABBWithToolPositioner(locator);
  tesseract::scene_graph::KDLStateSolver state_solver(*scene_graph);
  tesseract::scene_graph::SceneState scene_state = state_solver.getState();

  auto tool_kin = makeToolFwdKin(*scene_graph);
  Eigen::VectorXd tool_resolution = Eigen::VectorXd::Constant(1, 0.1);

  // Explicit-reach ctor.
  EXPECT_ANY_THROW(std::make_unique<RTPInvKin>(  // NOLINT
      *scene_graph,
      scene_state,
      std::make_unique<TwoTipStubInvKin>(),
      2.0,
      tool_kin->clone(),
      tool_resolution));

  // Auto-reach ctor.
  EXPECT_ANY_THROW(std::make_unique<RTPInvKin>(  // NOLINT
      *scene_graph,
      scene_state,
      std::make_unique<TwoTipStubInvKin>(),
      tool_kin->clone(),
      tool_resolution));
}

TEST(TesseractKinematicsUnit, RTPInvKinRejectsActiveJointBetweenManipTipAndToolBase)  // NOLINT
{
  // The manip-tip ↔ tool-base link gap is bridged by an active revolute joint, which violates the
  // static-offset assumption used internally. The ctor must reject this configuration.
  tesseract::common::GeneralResourceLocator locator;
  auto scene_graph = getSceneGraphABBWithActiveJointBeforeToolPositioner(locator);
  tesseract::scene_graph::KDLStateSolver state_solver(*scene_graph);
  tesseract::scene_graph::SceneState scene_state = state_solver.getState();

  auto opw_kin = makeOPWInvKin(*scene_graph);
  // Tool positioner is on the far side of the bad active joint.
  auto tool_kin = std::make_unique<KDLFwdKinChain>(*scene_graph, "tool_pivot", "tool_tip");
  Eigen::VectorXd tool_resolution = Eigen::VectorXd::Constant(1, 0.1);

  EXPECT_ANY_THROW(std::make_unique<RTPInvKin>(  // NOLINT
      *scene_graph,
      scene_state,
      std::move(opw_kin),
      2.0,
      std::move(tool_kin),
      tool_resolution));
}

TEST(TesseractKinematicsUnit, RTPInvKinRejectsToolBaseNotInSceneGraph)  // NOLINT
{
  tesseract::common::GeneralResourceLocator locator;
  auto scene_graph = getSceneGraphABBWithToolPositioner(locator);
  tesseract::scene_graph::KDLStateSolver state_solver(*scene_graph);
  tesseract::scene_graph::SceneState scene_state = state_solver.getState();

  auto opw_kin = makeOPWInvKin(*scene_graph);
  auto bad_tool = std::make_unique<StubFwdKin>("does_not_exist_link");
  Eigen::VectorXd tool_resolution = Eigen::VectorXd::Constant(1, 0.1);

  EXPECT_ANY_THROW(std::make_unique<RTPInvKin>(  // NOLINT
      *scene_graph,
      scene_state,
      std::move(opw_kin),
      2.0,
      std::move(bad_tool),
      tool_resolution));
}

TEST(TesseractKinematicsUnit, RTPInvKinRejectsToolBaseDisconnectedFromManipTip)  // NOLINT
{
  tesseract::common::GeneralResourceLocator locator;
  auto scene_graph = getSceneGraphABBWithToolPositioner(locator);
  // Build state solver and the manipulator IK BEFORE adding the disconnected link — KDL-backed
  // helpers parse the whole graph as a tree and would throw on the multi-root layout.
  tesseract::scene_graph::KDLStateSolver state_solver(*scene_graph);
  tesseract::scene_graph::SceneState scene_state = state_solver.getState();
  auto opw_kin = makeOPWInvKin(*scene_graph);
  scene_graph->addLink(tesseract::scene_graph::Link("phantom_island"));

  auto disconnected_tool = std::make_unique<StubFwdKin>("phantom_island");
  Eigen::VectorXd tool_resolution = Eigen::VectorXd::Constant(1, 0.1);

  EXPECT_ANY_THROW(std::make_unique<RTPInvKin>(  // NOLINT
      *scene_graph,
      scene_state,
      std::move(opw_kin),
      2.0,
      std::move(disconnected_tool),
      tool_resolution));
}

TEST(TesseractKinematicsUnit, RTPInvKinReturnsNoSolutionsWhenManipIKEmpty)  // NOLINT
{
  tesseract::common::GeneralResourceLocator locator;
  auto scene_graph = getSceneGraphABBWithToolPositioner(locator);
  tesseract::scene_graph::KDLStateSolver state_solver(*scene_graph);
  tesseract::scene_graph::SceneState scene_state = state_solver.getState();

  auto empty_manip = std::make_unique<EmptyInvKin>();
  auto tool_kin = makeToolFwdKin(*scene_graph);
  Eigen::VectorXd tool_resolution = Eigen::VectorXd::Constant(1, 0.1);

  auto rtp = std::make_unique<RTPInvKin>(
      *scene_graph, scene_state, std::move(empty_manip), 2.0, std::move(tool_kin), tool_resolution);

  // Aim near the manipulator base so the per-sample reach check passes and the inner manip IK
  // is actually invoked — the stub then returns no solutions, exercising the early-return path.
  tesseract::common::TransformMap target;
  target["tool_tip"] = Eigen::Isometry3d::Identity();

  IKSolutions solutions;
  Eigen::VectorXd seed = Eigen::VectorXd::Zero(7);
  rtp->calcInvKin(solutions, target, seed);
  EXPECT_TRUE(solutions.empty());
}

TEST(TesseractKinematicsUnit, RTPInvKinMultiJointToolFKRoundtrip)  // NOLINT
{
  // Exercises the nested_ik recursion at depth >= 2 by using a 2-joint tool positioner.
  tesseract::common::GeneralResourceLocator locator;
  auto scene_graph = getSceneGraphABBWithMultiJointToolPositioner(locator);
  tesseract::scene_graph::KDLStateSolver state_solver(*scene_graph);
  tesseract::scene_graph::SceneState scene_state = state_solver.getState();

  auto opw_kin = makeOPWInvKin(*scene_graph);
  auto tool_kin = std::make_unique<KDLFwdKinChain>(*scene_graph, "tool0", "tool_tip");

  // Sweep both tool joints on a coarse grid.
  Eigen::MatrixX2d tool_range(2, 2);
  tool_range << -0.4, 0.4, -0.4, 0.4;
  Eigen::VectorXd tool_resolution = Eigen::VectorXd::Constant(2, 0.2);

  auto rtp = std::make_unique<RTPInvKin>(
      *scene_graph, scene_state, std::move(opw_kin), 2.0, std::move(tool_kin), tool_range, tool_resolution);

  EXPECT_EQ(rtp->numJoints(), 8);

  // Pick a known config whose tool joint values fall on the sweep grid.
  auto full_fwd_kin = KDLFwdKinChain(*scene_graph, "base_link", "tool_tip");
  Eigen::VectorXd q(8);
  q << 0.1, -0.2, 0.3, 0.0, 0.5, 0.0, 0.2, -0.2;
  tesseract::common::TransformMap fwd_poses;
  full_fwd_kin.calcFwdKin(fwd_poses, q);
  Eigen::Isometry3d target_pose = fwd_poses.at("tool_tip");

  tesseract::common::TransformMap target;
  target["tool_tip"] = target_pose;

  IKSolutions solutions;
  Eigen::VectorXd seed = Eigen::VectorXd::Zero(8);
  rtp->calcInvKin(solutions, target, seed);

  ASSERT_FALSE(solutions.empty());

  // Every reported solution must be a valid FK roundtrip (proves both tool joints are applied).
  const double tol = 1e-4;
  std::size_t valid = 0;
  for (const auto& sol : solutions)
  {
    ASSERT_EQ(sol.size(), 8);
    tesseract::common::TransformMap check_poses;
    full_fwd_kin.calcFwdKin(check_poses, sol);
    Eigen::Isometry3d check = check_poses.at("tool_tip");

    if ((check.translation() - target_pose.translation()).norm() < tol &&
        Eigen::Quaterniond(check.linear()).angularDistance(Eigen::Quaterniond(target_pose.linear())) < tol)
      ++valid;
  }
  EXPECT_EQ(valid, solutions.size());

  // The grid step is 0.2; the chosen tool config (0.2, -0.2) lands exactly on a grid point. At
  // least one solution should match it. This proves the recursion visits both joint dimensions.
  bool found_grid_match = false;
  for (const auto& sol : solutions)
  {
    if (std::abs(sol(6) - 0.2) < 0.05 && std::abs(sol(7) - (-0.2)) < 0.05)
    {
      found_grid_match = true;
      break;
    }
  }
  EXPECT_TRUE(found_grid_match);
}

TEST(TesseractKinematicsUnit, RTPInvKinMultiJointToolSolutionCount)  // NOLINT
{
  // The Cartesian product of grid samples is exercised: confirm the solution count is bounded
  // above by N1 * N2 * 8 (OPW max branches) and below by 1 for a reachable target.
  tesseract::common::GeneralResourceLocator locator;
  auto scene_graph = getSceneGraphABBWithMultiJointToolPositioner(locator);
  tesseract::scene_graph::KDLStateSolver state_solver(*scene_graph);
  tesseract::scene_graph::SceneState scene_state = state_solver.getState();

  Eigen::MatrixX2d tool_range(2, 2);
  tool_range << -0.4, 0.4, -0.4, 0.4;
  Eigen::VectorXd tool_resolution = Eigen::VectorXd::Constant(2, 0.2);

  auto rtp = std::make_unique<RTPInvKin>(*scene_graph,
                                         scene_state,
                                         makeOPWInvKin(*scene_graph),
                                         2.0,
                                         std::make_unique<KDLFwdKinChain>(*scene_graph, "tool0", "tool_tip"),
                                         tool_range,
                                         tool_resolution);

  // LinSpaced on [-0.4, 0.4] with step 0.2 yields ceil(0.8/0.2) + 1 = 5 samples per joint.
  const std::size_t n_per_joint = 5;
  const std::size_t opw_max_branches = 8;
  const std::size_t upper_bound = n_per_joint * n_per_joint * opw_max_branches;

  auto full_fwd_kin = KDLFwdKinChain(*scene_graph, "base_link", "tool_tip");
  Eigen::VectorXd q(8);
  q << 0.0, 0.2, -0.3, 0.0, 0.5, 0.0, 0.0, 0.0;
  tesseract::common::TransformMap fwd_poses;
  full_fwd_kin.calcFwdKin(fwd_poses, q);

  tesseract::common::TransformMap target;
  target["tool_tip"] = fwd_poses.at("tool_tip");

  IKSolutions solutions;
  rtp->calcInvKin(solutions, target, Eigen::VectorXd::Zero(8));

  EXPECT_GT(solutions.size(), 0U);
  EXPECT_LE(solutions.size(), upper_bound);

  // Confirm both tool dimensions were swept: collect distinct (q6, q7) pairs returned.
  std::set<std::pair<int, int>> distinct_grid_cells;
  for (const auto& sol : solutions)
  {
    int g1 = static_cast<int>(std::round(sol(6) / 0.2));
    int g2 = static_cast<int>(std::round(sol(7) / 0.2));
    distinct_grid_cells.emplace(g1, g2);
  }
  // If the recursion only swept one dimension, all distinct pairs would share the same value in
  // one coordinate. Require at least two unique values along each dimension.
  std::set<int> g1_vals, g2_vals;
  for (const auto& [g1, g2] : distinct_grid_cells)
  {
    g1_vals.insert(g1);
    g2_vals.insert(g2);
  }
  EXPECT_GE(g1_vals.size(), 2U);
  EXPECT_GE(g2_vals.size(), 2U);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
