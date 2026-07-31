#include <tesseract/common/macros.h>
#include <memory>
#include <set>
#include <utility>
#include <vector>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include "abb_opw_fixture.h"
#include "kinematics_test_utils.h"

#include <tesseract/kinematics/kdl/kdl_fwd_kin_chain.h>
#include <tesseract/kinematics/kinematics_plugin_factory.h>
#include <tesseract/kinematics/rtp_inv_kin.h>
#include <tesseract/kinematics/utils.h>
#include <tesseract/kinematics/kinematic_group.h>
#include <tesseract/state_solver/kdl/kdl_state_solver.h>
#include <tesseract/common/yaml_utils.h>

#include <optional>
#include <stdexcept>
#include <string>

using namespace tesseract::kinematics::test_suite;
using namespace tesseract::kinematics;

namespace
{
/**
 * @brief The RTP plugin document every factory test in this file loads.
 * @param manipulator_reach Emitted as a `manipulator_reach` entry when set. Pass std::nullopt to
 *        omit it, which is how the factory's auto-reach path is exercised.
 */
std::string rtpPluginYaml(std::optional<double> manipulator_reach = 2.0)
{
  const std::string reach_entry =
      manipulator_reach ? "            manipulator_reach: " + std::to_string(*manipulator_reach) + "\n" : "";

  return R"(
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
)" + reach_entry +
         R"(            tool_sample_resolution:
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
}

/**
 * @brief The RTPInvKin plugin node in a loaded rtpPluginYaml() document.
 * @details Taken by non-const reference on purpose: callers mutate the returned node to build
 *          failure cases, and YAML::Node's const overloads do not permit assignment.
 */
YAML::Node rtpPlugin(YAML::Node& config)
{
  return config["kinematic_plugins"]["inv_kin_plugins"]["rtp_manipulator"]["plugins"]["RTPInvKin"];
}

/** @brief That plugin's `config` block - the RTPInvKinFactory schema itself. */
YAML::Node rtpPluginConfig(YAML::Node& config) { return rtpPlugin(config)["config"]; }

/** @brief Minimal stub IK exposing two tip link names — used to verify RTP rejects multi-tip manipulators. */
class TwoTipStubInvKin : public InverseKinematics
{
public:
  void calcInvKin(IKSolutions& /*solutions*/,
                  const tesseract::common::LinkIdTransformMap& /*tip_link_poses*/,
                  const Eigen::Ref<const Eigen::VectorXd>& /*seed*/) const override
  {
  }
  std::vector<tesseract::common::JointId> getJointIds() const override { return { "joint_1" }; }
  Eigen::Index numJoints() const override { return 1; }
  tesseract::common::LinkId getBaseLinkId() const override { return "base_link"; }
  tesseract::common::LinkId getWorkingFrame() const override { return "base_link"; }
  std::vector<tesseract::common::LinkId> getTipLinkIds() const override { return { "tip_a", "tip_b" }; }
  std::string getSolverName() const override { return "TwoTipStub"; }
  InverseKinematics::UPtr clone() const override { return std::make_unique<TwoTipStubInvKin>(*this); }
};

/** @brief Minimal stub FK with a configurable base link id — used to drive the RTP tool-base
 *         connectivity checks (base link not in scene graph; base link disconnected from manip tip).
 *         The joint id is also configurable so callers can supply one that exists in the scene
 *         graph; otherwise gatherJointLimits would throw before init()'s connectivity check runs.
 *         The tip link list is configurable because no real ForwardKinematics here reports zero or
 *         several tips. */
class StubFwdKin : public ForwardKinematics
{
public:
  explicit StubFwdKin(tesseract::common::LinkId base,
                      tesseract::common::JointId joint = "tool_joint",
                      std::vector<tesseract::common::LinkId> tips = { "stub_tip" })
    : base_link_(std::move(base)), joint_id_(std::move(joint)), tip_links_(std::move(tips))
  {
  }
  void calcFwdKin(tesseract::common::LinkIdTransformMap& /*transforms*/,
                  const Eigen::Ref<const Eigen::VectorXd>& /*joint_angles*/) const override
  {
  }
  void calcJacobian(Eigen::Ref<Eigen::MatrixXd> /*jacobian*/,
                    const Eigen::Ref<const Eigen::VectorXd>& /*joint_angles*/,
                    const tesseract::common::LinkId& /*link_id*/) const override
  {
  }
  tesseract::common::LinkId getBaseLinkId() const override { return base_link_; }
  std::vector<tesseract::common::JointId> getJointIds() const override { return { joint_id_ }; }
  std::vector<tesseract::common::LinkId> getTipLinkIds() const override { return tip_links_; }
  Eigen::Index numJoints() const override { return 1; }
  std::string getSolverName() const override { return "StubFwd"; }
  ForwardKinematics::UPtr clone() const override { return std::make_unique<StubFwdKin>(*this); }

private:
  tesseract::common::LinkId base_link_;
  tesseract::common::JointId joint_id_;
  std::vector<tesseract::common::LinkId> tip_links_;
};

/** @brief True when @p a and @p b agree in both position and orientation to within @p tol. */
bool poseMatches(const Eigen::Isometry3d& a, const Eigen::Isometry3d& b, double tol)
{
  return (a.translation() - b.translation()).norm() < tol &&
         Eigen::Quaterniond(a.linear()).angularDistance(Eigen::Quaterniond(b.linear())) < tol;
}

/** @brief Minimal stub IK that returns an empty solution set, so calcInvKin contributes no
 *         solutions for any sample. */
class EmptyInvKin : public InverseKinematics
{
public:
  void calcInvKin(IKSolutions& solutions,
                  const tesseract::common::LinkIdTransformMap& /*tip_link_poses*/,
                  const Eigen::Ref<const Eigen::VectorXd>& /*seed*/) const override
  {
    solutions.clear();
  }
  std::vector<tesseract::common::JointId> getJointIds() const override
  {
    return { "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6" };
  }
  Eigen::Index numJoints() const override { return 6; }
  tesseract::common::LinkId getBaseLinkId() const override { return "base_link"; }
  tesseract::common::LinkId getWorkingFrame() const override { return "base_link"; }
  std::vector<tesseract::common::LinkId> getTipLinkIds() const override { return { "tool0" }; }
  std::string getSolverName() const override { return "EmptyInv"; }
  InverseKinematics::UPtr clone() const override { return std::make_unique<EmptyInvKin>(*this); }
};

/** @brief EmptyInvKin that counts inner solves and reports a configurable working frame. */
class CountingInvKin : public EmptyInvKin
{
public:
  explicit CountingInvKin(std::shared_ptr<int> calls, tesseract::common::LinkId working_frame = "base_link")
    : calls_(std::move(calls)), working_frame_(std::move(working_frame))
  {
  }
  void calcInvKin(IKSolutions& solutions,
                  const tesseract::common::LinkIdTransformMap& /*tip_link_poses*/,
                  const Eigen::Ref<const Eigen::VectorXd>& /*seed*/) const override
  {
    ++*calls_;
    solutions.clear();
  }
  tesseract::common::LinkId getWorkingFrame() const override { return working_frame_; }
  InverseKinematics::UPtr clone() const override { return std::make_unique<CountingInvKin>(*this); }

private:
  std::shared_ptr<int> calls_;
  tesseract::common::LinkId working_frame_;
};

/** @brief EmptyInvKin whose clone() throws, to probe the copy-assignment failure path. */
class ThrowingCloneInvKin : public EmptyInvKin
{
public:
  InverseKinematics::UPtr clone() const override { throw std::runtime_error("clone failed"); }
};

/** @brief Number of inner solves one calcInvKin() call makes for a tool-tip @p target. */
int countInnerSolves(const RTPInvKin& rtp, const std::shared_ptr<int>& calls, const Eigen::Isometry3d& target)
{
  *calls = 0;
  tesseract::common::LinkIdTransformMap poses;
  poses["tool_tip"] = target;
  IKSolutions solutions;
  rtp.calcInvKin(solutions, poses, Eigen::VectorXd::Zero(rtp.numJoints()));
  return *calls;
}
}  // namespace

TEST(TesseractKinematicsUnit, RTPInvKinMetadata)  // NOLINT
{
  tesseract::common::GeneralResourceLocator locator;
  auto scene_graph = getSceneGraphABBWithToolPositioner(locator);

  tesseract::scene_graph::KDLStateSolver state_solver(*scene_graph);
  tesseract::scene_graph::SceneState scene_state = state_solver.getState();

  auto opw_kin = makeOPWInvKinABB(*scene_graph);
  auto tool_kin = makeToolFwdKinABB(*scene_graph);

  Eigen::VectorXd tool_resolution = Eigen::VectorXd::Constant(1, 0.1);
  auto rtp =
      std::make_unique<RTPInvKin>(*scene_graph, scene_state, opw_kin->clone(), 2.0, tool_kin->clone(), tool_resolution);

  EXPECT_EQ(rtp->getSolverName(), DEFAULT_RTP_INV_KIN_SOLVER_NAME);
  EXPECT_EQ(rtp->numJoints(), 7);
  EXPECT_EQ(rtp->getBaseLinkId(), "base_link");
  EXPECT_EQ(rtp->getWorkingFrame(), "base_link");
  ASSERT_EQ(rtp->getTipLinkIds().size(), 1);
  EXPECT_EQ(rtp->getTipLinkIds()[0], "tool_tip");

  std::vector<tesseract::common::JointId> expected_joints{ "joint_1", "joint_2", "joint_3",   "joint_4",
                                                           "joint_5", "joint_6", "tool_joint" };
  EXPECT_EQ(rtp->getJointIds(), expected_joints);
}

TEST(TesseractKinematicsUnit, RTPInvKinConstructorValidation)  // NOLINT
{
  tesseract::common::GeneralResourceLocator locator;
  auto scene_graph = getSceneGraphABBWithToolPositioner(locator);
  tesseract::scene_graph::KDLStateSolver state_solver(*scene_graph);
  tesseract::scene_graph::SceneState scene_state = state_solver.getState();

  auto opw_kin = makeOPWInvKinABB(*scene_graph);
  auto tool_kin = makeToolFwdKinABB(*scene_graph);
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
  {  // Tool positioner with other than one tip link. The stub's base is the manipulator tip, so
     // connectivity passes and the message check pins which rejection fired.
    auto expect_tip_rejection = [&](const std::vector<tesseract::common::LinkId>& tips) {
      auto stub = std::make_unique<StubFwdKin>(ABB_MANIP_TIP_LINK, "tool_joint", tips);
      try
      {
        auto kin = std::make_unique<RTPInvKin>(
            *scene_graph, scene_state, opw_kin->clone(), 2.0, std::move(stub), tool_resolution);
        ADD_FAILURE() << "Expected rejection for a tool positioner with " << tips.size() << " tip link(s)";
      }
      catch (const std::runtime_error& e)
      {
        EXPECT_NE(std::string(e.what()).find("tool positioner with exactly one tip link"), std::string::npos)
            << "threw for the wrong reason: " << e.what();
      }
    };
    expect_tip_rejection({ "tip_a", "tip_b" });
    expect_tip_rejection({});
  }
  {  // Scene state that does not describe the scene graph. Asserting on runtime_error specifically:
     // an unchecked map::at would throw out_of_range, which is a logic_error and would not match.
    tesseract::scene_graph::SceneState partial_state = scene_state;
    partial_state.link_transforms.erase(ABB_MANIP_TIP_LINK);
    EXPECT_THROW(std::make_unique<RTPInvKin>(*scene_graph,  // NOLINT
                                             partial_state,
                                             opw_kin->clone(),
                                             2.0,
                                             tool_kin->clone(),
                                             tool_resolution),
                 std::runtime_error);
  }
  {  // tool_sample_range row count mismatch (2 rows, 1-DOF tool positioner)
    Eigen::MatrixX2d wrong_range(2, 2);
    wrong_range << -1.0, 1.0, -1.0, 1.0;
    EXPECT_ANY_THROW(std::make_unique<RTPInvKin>(
        *scene_graph, scene_state, opw_kin->clone(), 2.0, tool_kin->clone(), wrong_range, tool_resolution));  // NOLINT
  }
  {  // Empty scene graph via the explicit-range ctor
    tesseract::scene_graph::SceneGraph empty_sg;
    EXPECT_ANY_THROW(std::make_unique<RTPInvKin>(
        empty_sg, scene_state, opw_kin->clone(), 2.0, tool_kin->clone(), range, tool_resolution));  // NOLINT
  }
}

TEST(TesseractKinematicsUnit, RTPInvKinIsIndependentOfArmConfiguration)  // NOLINT
{
  // init() caches a per-sample transform derived from the construction-time scene state, which
  // invites the question of whether moving the arm invalidates it. It does not: the cached value is
  // tool tip -> manipulator tip, and both of its factors are relative. Two solvers built from scene
  // states with the arm in different places must answer an identical query identically.
  tesseract::common::GeneralResourceLocator locator;
  auto scene_graph = getSceneGraphABBWithToolPositioner(locator);
  tesseract::scene_graph::KDLStateSolver state_solver(*scene_graph);

  KDLFwdKinChain full_fwd_kin(*scene_graph, ABB_BASE_LINK, ABB_TOOL_TIP_LINK);
  const std::vector<tesseract::common::JointId> joint_ids = full_fwd_kin.getJointIds();

  Eigen::VectorXd q_moved(7);
  q_moved << 0.3, -0.4, 0.5, 0.2, -0.6, 0.7, 0.9;

  const tesseract::scene_graph::SceneState home = state_solver.getState();
  const tesseract::scene_graph::SceneState moved = state_solver.getState(joint_ids, q_moved);

  // Guard the premise: the two states must really place the manipulator tip differently.
  ASSERT_FALSE(home.link_transforms.at(ABB_MANIP_TIP_LINK).isApprox(moved.link_transforms.at(ABB_MANIP_TIP_LINK)));

  const Eigen::VectorXd tool_resolution = Eigen::VectorXd::Constant(1, 0.1);
  const auto make_rtp = [&](const tesseract::scene_graph::SceneState& state) {
    return std::make_unique<RTPInvKin>(
        *scene_graph, state, makeOPWInvKinABB(*scene_graph), 2.0, makeToolFwdKinABB(*scene_graph), tool_resolution);
  };

  // One absolute target pose, solved by a solver built at each state.
  Eigen::VectorXd q_target(7);
  q_target << 0.0, 0.2, -0.3, 0.0, 0.5, 0.0, 0.4;
  tesseract::common::LinkIdTransformMap fwd_poses;
  full_fwd_kin.calcFwdKin(fwd_poses, q_target);
  tesseract::common::LinkIdTransformMap target;
  target[ABB_TOOL_TIP_LINK] = fwd_poses.at(ABB_TOOL_TIP_LINK);

  const Eigen::VectorXd seed = Eigen::VectorXd::Zero(7);
  IKSolutions sols_home;
  IKSolutions sols_moved;
  make_rtp(home)->calcInvKin(sols_home, target, seed);
  make_rtp(moved)->calcInvKin(sols_moved, target, seed);

  ASSERT_FALSE(sols_home.empty());
  ASSERT_EQ(sols_home.size(), sols_moved.size());
  for (std::size_t i = 0; i < sols_home.size(); ++i)
    EXPECT_TRUE(sols_home[i].isApprox(sols_moved[i])) << "solution " << i << " differs between arm configurations";
}

TEST(TesseractKinematicsUnit, RTPInvKinAutoReachMatchesExplicit)  // NOLINT
{
  tesseract::common::GeneralResourceLocator locator;
  auto scene_graph = getSceneGraphABBWithToolPositioner(locator);
  tesseract::scene_graph::KDLStateSolver state_solver(*scene_graph);
  tesseract::scene_graph::SceneState scene_state = state_solver.getState();

  auto opw_kin = makeOPWInvKinABB(*scene_graph);
  auto tool_kin = makeToolFwdKinABB(*scene_graph);
  Eigen::VectorXd tool_resolution = Eigen::VectorXd::Constant(1, 0.1);

  // The auto-reach ctor should succeed without throwing. We don't inspect the reach directly
  // (it's private) but we verify the RTP behaves identically to one built with an explicit reach
  // that is known-large-enough (3.0).
  auto rtp_auto =
      std::make_unique<RTPInvKin>(*scene_graph, scene_state, opw_kin->clone(), tool_kin->clone(), tool_resolution);
  auto rtp_explicit =
      std::make_unique<RTPInvKin>(*scene_graph, scene_state, opw_kin->clone(), 3.0, tool_kin->clone(), tool_resolution);

  EXPECT_EQ(rtp_auto->numJoints(), rtp_explicit->numJoints());
  EXPECT_EQ(rtp_auto->getBaseLinkId(), rtp_explicit->getBaseLinkId());
  EXPECT_EQ(rtp_auto->getTipLinkIds(), rtp_explicit->getTipLinkIds());
  EXPECT_EQ(rtp_auto->getJointIds(), rtp_explicit->getJointIds());

  // Behavioural check: feed a reachable tool_tip target and confirm both return solutions.
  // Pick a mid-workspace target via FK roundtrip.
  auto fwd_full = std::make_unique<KDLFwdKinChain>(*scene_graph, "base_link", "tool_tip");
  Eigen::VectorXd q = Eigen::VectorXd::Zero(7);
  q(1) = -0.3;  // lift joint_2 a bit to escape singular home
  tesseract::common::LinkIdTransformMap poses;
  fwd_full->calcFwdKin(poses, q);
  tesseract::common::LinkIdTransformMap target{ { "tool_tip", poses.at("tool_tip") } };

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
                                         makeOPWInvKinABB(*scene_graph),
                                         2.0,
                                         makeToolFwdKinABB(*scene_graph),
                                         tool_range,
                                         tool_resolution);

  // Pick a joint vector with tool=0, compute FK for tool_tip, then invert.
  auto full_fwd_kin = KDLFwdKinChain(*scene_graph, "base_link", "tool_tip");
  Eigen::VectorXd q(7);
  q << 0.1, -0.2, 0.3, 0.0, 0.5, 0.0, 0.0;
  tesseract::common::LinkIdTransformMap fwd_poses;
  full_fwd_kin.calcFwdKin(fwd_poses, q);
  Eigen::Isometry3d tool_tip_pose = fwd_poses.at("tool_tip");

  tesseract::common::LinkIdTransformMap target;
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

    tesseract::common::LinkIdTransformMap check_poses;
    full_fwd_kin.calcFwdKin(check_poses, sol);
    Eigen::Isometry3d check = check_poses.at("tool_tip");

    if (poseMatches(check, tool_tip_pose, tol))
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
                                         makeOPWInvKinABB(*scene_graph),
                                         2.0,
                                         makeToolFwdKinABB(*scene_graph),
                                         tool_range,
                                         tool_resolution);

  // Reachable pose in the middle of the ABB workspace.
  auto full_fwd_kin = KDLFwdKinChain(*scene_graph, "base_link", "tool_tip");
  Eigen::VectorXd q(7);
  q << 0.0, 0.2, -0.3, 0.0, 0.5, 0.0, 0.4;  // tool at 0.4 rad
  tesseract::common::LinkIdTransformMap fwd_poses;
  full_fwd_kin.calcFwdKin(fwd_poses, q);
  Eigen::Isometry3d target_pose = fwd_poses.at("tool_tip");

  tesseract::common::LinkIdTransformMap target;
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
    tesseract::common::LinkIdTransformMap check_poses;
    full_fwd_kin.calcFwdKin(check_poses, sol);
    Eigen::Isometry3d check = check_poses.at("tool_tip");

    if (poseMatches(check, target_pose, tol))
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
      *scene_graph, scene_state, makeOPWInvKinABB(*scene_graph), 2.0, makeToolFwdKinABB(*scene_graph), tool_resolution);

  auto cloned = rtp->clone();
  EXPECT_EQ(cloned->getSolverName(), DEFAULT_RTP_INV_KIN_SOLVER_NAME);
  EXPECT_EQ(cloned->numJoints(), 7);
  EXPECT_EQ(cloned->getJointIds(), rtp->getJointIds());
  EXPECT_EQ(cloned->getBaseLinkId(), rtp->getBaseLinkId());
  ASSERT_EQ(cloned->getTipLinkIds().size(), 1U);
  EXPECT_EQ(cloned->getTipLinkIds()[0], rtp->getTipLinkIds()[0]);

  std::vector<tesseract::common::JointId> joint_ids{ "joint_1", "joint_2", "joint_3",   "joint_4",
                                                     "joint_5", "joint_6", "tool_joint" };
  KinematicGroup kin_group("rtp_manip", joint_ids, std::move(cloned), *scene_graph, scene_state);
  EXPECT_EQ(kin_group.getBaseLinkId(), scene_graph->getRoot());
  EXPECT_EQ(kin_group.getName(), "rtp_manip");
  EXPECT_EQ(kin_group.getJointIds(), joint_ids);

  auto tip_names = kin_group.getAllPossibleTipLinkIds();
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
                                             makeOPWInvKinABB(*scene_graph),
                                             2.0,
                                             makeToolFwdKinABB(*scene_graph),
                                             tool_resolution,
                                             "rtp_src_solver");

  Eigen::VectorXd alt_resolution = Eigen::VectorXd::Constant(1, 0.5);
  auto rtp_dst = std::make_unique<RTPInvKin>(*scene_graph,
                                             scene_state,
                                             makeOPWInvKinABB(*scene_graph),
                                             1.5,
                                             makeToolFwdKinABB(*scene_graph),
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
  EXPECT_EQ(rtp_dst->getJointIds(), rtp_src->getJointIds());
  EXPECT_EQ(rtp_dst->getBaseLinkId(), rtp_src->getBaseLinkId());
  EXPECT_EQ(rtp_dst->getTipLinkIds(), rtp_src->getTipLinkIds());
  EXPECT_EQ(rtp_dst->numJoints(), rtp_src->numJoints());
}

TEST(TesseractKinematicsUnit, RTPInvKinFactoryYaml)  // NOLINT
{
  tesseract::common::GeneralResourceLocator locator;
  auto scene_graph = getSceneGraphABBWithToolPositioner(locator);
  tesseract::scene_graph::KDLStateSolver state_solver(*scene_graph);
  tesseract::scene_graph::SceneState scene_state = state_solver.getState();

  const std::string yaml_str = rtpPluginYaml();

  KinematicsPluginFactory factory(YAML::Load(yaml_str), locator);
  auto loaded = factory.createInvKin("rtp_manipulator", "RTPInvKin", *scene_graph, scene_state);
  ASSERT_NE(loaded, nullptr);
  EXPECT_EQ(loaded->numJoints(), 7);
  ASSERT_EQ(loaded->getTipLinkIds().size(), 1U);
  EXPECT_EQ(loaded->getTipLinkIds()[0], "tool_tip");
  EXPECT_EQ(loaded->getBaseLinkId(), "base_link");
}

TEST(TesseractKinematicsUnit, RTPInvKinFactoryAutoReach)  // NOLINT
{
  tesseract::common::GeneralResourceLocator locator;
  auto scene_graph = getSceneGraphABBWithToolPositioner(locator);
  tesseract::scene_graph::KDLStateSolver state_solver(*scene_graph);
  tesseract::scene_graph::SceneState scene_state = state_solver.getState();

  // Note: no `manipulator_reach` - factory must auto-derive it.
  const std::string yaml_str = rtpPluginYaml(std::nullopt);

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

  const std::string yaml_str = rtpPluginYaml();

  // Sanity: the unmodified yaml must produce a valid solver.
  {
    KinematicsPluginFactory factory(YAML::Load(yaml_str), locator);
    EXPECT_NE(factory.createInvKin("rtp_manipulator", "RTPInvKin", *scene_graph, scene_state), nullptr);
  }

  // Whatever the schema can judge on its own - a required property missing, or a class that is not
  // a registered factory - is rejected when the plugin document is loaded, before any solver is
  // asked for.
  auto expect_schema_rejects = [&](const YAML::Node& config) {
    EXPECT_THROW(KinematicsPluginFactory(config, locator), std::runtime_error);  // NOLINT
  };

  // Whatever the schema cannot judge - joint identities, ranges measured against the scene graph's
  // limits, and the tool chain's joint count - survives loading and is caught on creation.
  auto expect_create_fails = [&](const YAML::Node& config) {
    KinematicsPluginFactory factory(config, locator);
    EXPECT_EQ(factory.createInvKin("rtp_manipulator", "RTPInvKin", *scene_graph, scene_state), nullptr);
  };

  {  // Missing config block
    YAML::Node config = tesseract::common::loadYamlString(yaml_str, locator);
    auto plugin = rtpPlugin(config);
    plugin.remove("config");
    expect_schema_rejects(config);
  }
  {  // Non-positive manipulator_reach
    YAML::Node config = tesseract::common::loadYamlString(yaml_str, locator);
    rtpPluginConfig(config)["manipulator_reach"] = -1.0;
    expect_create_fails(config);
  }
  {  // Missing tool_sample_resolution
    YAML::Node config = tesseract::common::loadYamlString(yaml_str, locator);
    auto cfg = rtpPluginConfig(config);
    cfg.remove("tool_sample_resolution");
    expect_schema_rejects(config);
  }
  {  // tool_sample_resolution entry missing 'name'
    YAML::Node config = tesseract::common::loadYamlString(yaml_str, locator);
    auto cfg = rtpPluginConfig(config);
    cfg["tool_sample_resolution"][0].remove("name");
    expect_schema_rejects(config);
  }
  {  // tool_sample_resolution entry missing 'value'
    YAML::Node config = tesseract::common::loadYamlString(yaml_str, locator);
    auto cfg = rtpPluginConfig(config);
    cfg["tool_sample_resolution"][0].remove("value");
    expect_schema_rejects(config);
  }
  {  // tool_sample_resolution joint name not in scene graph
    YAML::Node config = tesseract::common::loadYamlString(yaml_str, locator);
    rtpPluginConfig(config)["tool_sample_resolution"][0]["name"] = "joint_does_not_exist";
    expect_create_fails(config);
  }
  {  // tool_sample_resolution min below joint lower limit
    YAML::Node config = tesseract::common::loadYamlString(yaml_str, locator);
    rtpPluginConfig(config)["tool_sample_resolution"][0]["min"] = -10000.0;
    expect_create_fails(config);
  }
  {  // tool_sample_resolution max above joint upper limit
    YAML::Node config = tesseract::common::loadYamlString(yaml_str, locator);
    rtpPluginConfig(config)["tool_sample_resolution"][0]["max"] = 10000.0;
    expect_create_fails(config);
  }
  {  // tool_sample_resolution min greater than max
    YAML::Node config = tesseract::common::loadYamlString(yaml_str, locator);
    auto cfg = rtpPluginConfig(config);
    auto entry = cfg["tool_sample_resolution"][0];
    entry["min"] = 0.5;
    entry["max"] = 0.1;
    expect_create_fails(config);
  }
  {  // Missing tool_positioner
    YAML::Node config = tesseract::common::loadYamlString(yaml_str, locator);
    auto cfg = rtpPluginConfig(config);
    cfg.remove("tool_positioner");
    expect_schema_rejects(config);
  }
  {  // tool_positioner missing class entry
    YAML::Node config = tesseract::common::loadYamlString(yaml_str, locator);
    auto cfg = rtpPluginConfig(config);
    cfg["tool_positioner"].remove("class");
    expect_schema_rejects(config);
  }
  {  // tool_positioner with unregistered class
    YAML::Node config = tesseract::common::loadYamlString(yaml_str, locator);
    rtpPluginConfig(config)["tool_positioner"]["class"] = "DoesNotExistFactory";
    expect_schema_rejects(config);
  }
  {  // Missing manipulator
    YAML::Node config = tesseract::common::loadYamlString(yaml_str, locator);
    auto cfg = rtpPluginConfig(config);
    cfg.remove("manipulator");
    expect_schema_rejects(config);
  }
  {  // manipulator missing class entry
    YAML::Node config = tesseract::common::loadYamlString(yaml_str, locator);
    auto cfg = rtpPluginConfig(config);
    cfg["manipulator"].remove("class");
    expect_schema_rejects(config);
  }
  {  // manipulator with unregistered class
    YAML::Node config = tesseract::common::loadYamlString(yaml_str, locator);
    rtpPluginConfig(config)["manipulator"]["class"] = "DoesNotExistFactory";
    expect_schema_rejects(config);
  }
  {  // tool_sample_resolution has more entries than tool positioner has joints
    YAML::Node config = tesseract::common::loadYamlString(yaml_str, locator);
    YAML::Node extra;
    extra["name"] = "joint_1";  // exists in scene graph and has limits, but not in the tool chain
    extra["value"] = 0.1;
    auto cfg = rtpPluginConfig(config);
    cfg["tool_sample_resolution"].push_back(extra);
    expect_create_fails(config);
  }
  {  // tool_sample_resolution names a non-tool-chain joint (size matches but lookup misses)
    YAML::Node config = tesseract::common::loadYamlString(yaml_str, locator);
    rtpPluginConfig(config)["tool_sample_resolution"][0]["name"] = "joint_1";
    expect_create_fails(config);
  }
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

  const std::string yaml_str = rtpPluginYaml();

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

  auto tool_kin = makeToolFwdKinABB(*scene_graph);
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

  auto opw_kin = makeOPWInvKinABB(*scene_graph);
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

  auto opw_kin = makeOPWInvKinABB(*scene_graph);
  auto bad_tool = std::make_unique<StubFwdKin>("does_not_exist_link");
  Eigen::VectorXd tool_resolution = Eigen::VectorXd::Constant(1, 0.1);

  // The tool base is required in the scene state before it is looked up in the graph, so a link
  // absent from both is rejected by the earlier guard. Seeding the state makes the graph lookup
  // the check under test; the message assertion below is what pins that.
  scene_state.link_transforms["does_not_exist_link"] = Eigen::Isometry3d::Identity();

  try
  {
    auto kin = std::make_unique<RTPInvKin>(
        *scene_graph, scene_state, std::move(opw_kin), 2.0, std::move(bad_tool), tool_resolution);
    ADD_FAILURE() << "Expected rejection for a tool base link absent from the scene graph";
  }
  catch (const std::runtime_error& e)
  {
    EXPECT_NE(std::string(e.what()).find("Tool positioner base link 'does_not_exist_link' not found in scene graph"),
              std::string::npos)
        << "threw for the wrong reason: " << e.what();
  }
}

TEST(TesseractKinematicsUnit, RTPInvKinRejectsToolBaseDisconnectedFromManipTip)  // NOLINT
{
  tesseract::common::GeneralResourceLocator locator;
  auto scene_graph = getSceneGraphABBWithToolPositioner(locator);
  // Build state solver and the manipulator IK BEFORE adding the disconnected link — KDL-backed
  // helpers parse the whole graph as a tree and would throw on the multi-root layout.
  tesseract::scene_graph::KDLStateSolver state_solver(*scene_graph);
  tesseract::scene_graph::SceneState scene_state = state_solver.getState();
  auto opw_kin = makeOPWInvKinABB(*scene_graph);
  scene_graph->addLink(tesseract::scene_graph::Link("phantom_island"));

  auto disconnected_tool = std::make_unique<StubFwdKin>("phantom_island");
  Eigen::VectorXd tool_resolution = Eigen::VectorXd::Constant(1, 0.1);

  // The state was captured before the link was added, so it too must be seeded — otherwise the
  // scene-state guard fires first and the connectivity check is never reached.
  scene_state.link_transforms["phantom_island"] = Eigen::Isometry3d::Identity();

  try
  {
    auto kin = std::make_unique<RTPInvKin>(
        *scene_graph, scene_state, std::move(opw_kin), 2.0, std::move(disconnected_tool), tool_resolution);
    ADD_FAILURE() << "Expected rejection for a tool base link disconnected from the manipulator tip";
  }
  catch (const std::runtime_error& e)
  {
    EXPECT_NE(std::string(e.what()).find("is not connected to manipulator tip link"), std::string::npos)
        << "threw for the wrong reason: " << e.what();
  }
}

TEST(TesseractKinematicsUnit, RTPInvKinRejectsToolSampleGridExceedingCap)  // NOLINT
{
  // The cap is on the product across tool joints, not on any one of them: each row below is far
  // inside buildSampleGrid's per-joint limit and only their product exceeds the combined cap.
  tesseract::common::GeneralResourceLocator locator;
  auto scene_graph = getSceneGraphABBWithMultiJointToolPositioner(locator);
  tesseract::scene_graph::KDLStateSolver state_solver(*scene_graph);
  tesseract::scene_graph::SceneState scene_state = state_solver.getState();

  auto opw_kin = makeOPWInvKinABB(*scene_graph);
  auto tool_kin = std::make_unique<KDLFwdKinChain>(*scene_graph, "tool0", "tool_tip");

  Eigen::MatrixX2d tool_range(2, 2);
  tool_range << -1.0, 1.0, -1.0, 1.0;
  Eigen::VectorXd tool_resolution = Eigen::VectorXd::Constant(2, 2e-3);  // 1001 samples per joint

  try
  {
    auto kin = std::make_unique<RTPInvKin>(
        *scene_graph, scene_state, std::move(opw_kin), 2.0, std::move(tool_kin), tool_range, tool_resolution);
    ADD_FAILURE() << "Expected rejection for a tool sample grid above the combined cap";
  }
  catch (const std::runtime_error& e)
  {
    EXPECT_NE(std::string(e.what()).find("combined samples"), std::string::npos)
        << "threw for the wrong reason: " << e.what();
  }
}

TEST(TesseractKinematicsUnit, RTPInvKinReturnsNoSolutionsWhenManipIKEmpty)  // NOLINT
{
  tesseract::common::GeneralResourceLocator locator;
  auto scene_graph = getSceneGraphABBWithToolPositioner(locator);
  tesseract::scene_graph::KDLStateSolver state_solver(*scene_graph);
  tesseract::scene_graph::SceneState scene_state = state_solver.getState();

  auto empty_manip = std::make_unique<EmptyInvKin>();
  auto tool_kin = makeToolFwdKinABB(*scene_graph);
  Eigen::VectorXd tool_resolution = Eigen::VectorXd::Constant(1, 0.1);

  auto rtp = std::make_unique<RTPInvKin>(
      *scene_graph, scene_state, std::move(empty_manip), 2.0, std::move(tool_kin), tool_resolution);

  // Aim near the manipulator base so the per-sample reach check passes and the inner manip IK
  // is actually invoked — the stub then returns no solutions, exercising the early-return path.
  tesseract::common::LinkIdTransformMap target;
  target["tool_tip"] = Eigen::Isometry3d::Identity();

  IKSolutions solutions;
  Eigen::VectorXd seed = Eigen::VectorXd::Zero(7);
  rtp->calcInvKin(solutions, target, seed);
  EXPECT_TRUE(solutions.empty());
}

TEST(TesseractKinematicsUnit, RTPInvKinMultiJointToolFKRoundtrip)  // NOLINT
{
  // Exercises a two-dimensional sample grid by using a 2-joint tool positioner.
  tesseract::common::GeneralResourceLocator locator;
  auto scene_graph = getSceneGraphABBWithMultiJointToolPositioner(locator);
  tesseract::scene_graph::KDLStateSolver state_solver(*scene_graph);
  tesseract::scene_graph::SceneState scene_state = state_solver.getState();

  auto opw_kin = makeOPWInvKinABB(*scene_graph);
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
  tesseract::common::LinkIdTransformMap fwd_poses;
  full_fwd_kin.calcFwdKin(fwd_poses, q);
  Eigen::Isometry3d target_pose = fwd_poses.at("tool_tip");

  tesseract::common::LinkIdTransformMap target;
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
    tesseract::common::LinkIdTransformMap check_poses;
    full_fwd_kin.calcFwdKin(check_poses, sol);
    Eigen::Isometry3d check = check_poses.at("tool_tip");

    if (poseMatches(check, target_pose, tol))
      ++valid;
  }
  EXPECT_EQ(valid, solutions.size());

  // The grid step is 0.2; the chosen tool config (0.2, -0.2) lands exactly on a grid point. At
  // least one solution should match it. This proves the grid sweeps both joint dimensions.
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
                                         makeOPWInvKinABB(*scene_graph),
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
  tesseract::common::LinkIdTransformMap fwd_poses;
  full_fwd_kin.calcFwdKin(fwd_poses, q);

  tesseract::common::LinkIdTransformMap target;
  target["tool_tip"] = fwd_poses.at("tool_tip");

  IKSolutions solutions;
  rtp->calcInvKin(solutions, target, Eigen::VectorXd::Zero(8));

  EXPECT_GT(solutions.size(), 0U);
  EXPECT_LE(solutions.size(), upper_bound);

  // Confirm both tool dimensions were swept. If the grid only swept one dimension, every solution
  // would share the same value in one coordinate, so require at least two along each.
  std::set<int> g1_vals, g2_vals;
  for (const auto& sol : solutions)
  {
    g1_vals.insert(static_cast<int>(std::round(sol(6) / 0.2)));
    g2_vals.insert(static_cast<int>(std::round(sol(7) / 0.2)));
  }
  EXPECT_GE(g1_vals.size(), 2U);
  EXPECT_GE(g2_vals.size(), 2U);
}

TEST(TesseractKinematicsUnit, RTPInvKinReachFilterFollowsManipulatorWorkingFrame)  // NOLINT
{
  tesseract::common::GeneralResourceLocator locator;
  auto scene_graph = getSceneGraphABBWithToolPositioner(locator);

  // A working frame rigidly offset 10 m from the manipulator base.
  scene_graph->addLink(tesseract::scene_graph::Link("offset_frame"));
  tesseract::scene_graph::Joint mount("offset_mount");
  mount.type = tesseract::scene_graph::JointType::FIXED;
  mount.parent_link_id = "base_link";
  mount.child_link_id = "offset_frame";
  mount.parent_to_joint_origin_transform.translation() = Eigen::Vector3d(10.0, 0.0, 0.0);
  scene_graph->addJoint(mount);

  tesseract::scene_graph::KDLStateSolver state_solver(*scene_graph);
  tesseract::scene_graph::SceneState scene_state = state_solver.getState();
  const Eigen::VectorXd tool_resolution = Eigen::VectorXd::Constant(1, 0.1);

  const auto make = [&](const std::shared_ptr<int>& calls, const std::string& working_frame) {
    return RTPInvKin(*scene_graph,
                     scene_state,
                     std::make_unique<CountingInvKin>(calls, working_frame),
                     2.0,
                     makeToolFwdKinABB(*scene_graph),
                     tool_resolution);
  };

  const Eigen::Isometry3d at_origin = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d at_base = Eigen::Isometry3d::Identity();
  at_base.translation() = Eigen::Vector3d(-10.0, 0.0, 0.0);

  // Rigid offset: the filter is centred on the manipulator base as seen from the working frame.
  auto calls = std::make_shared<int>(0);
  const RTPInvKin rigid = make(calls, "offset_frame");
  EXPECT_EQ(rigid.getWorkingFrame(), "offset_frame");
  EXPECT_GT(countInnerSolves(rigid, calls, at_base), 0);
  EXPECT_EQ(countInnerSolves(rigid, calls, at_origin), 0);

  // A revolute joint separates link_1 from the base, so the filter is off and every sample solves.
  const RTPInvKin moving = make(calls, "link_1");
  EXPECT_EQ(moving.getWorkingFrame(), "link_1");
  EXPECT_EQ(countInnerSolves(moving, calls, at_base), countInnerSolves(moving, calls, at_origin));
  EXPECT_GT(countInnerSolves(moving, calls, at_origin), 0);
}

TEST(TesseractKinematicsUnit, RTPInvKinSamplesRotationalToolJointOverOneTurn)  // NOLINT
{
  tesseract::common::GeneralResourceLocator locator;
  auto scene_graph = getSceneGraphABBWithToolPositioner(locator);
  tesseract::scene_graph::KDLStateSolver state_solver(*scene_graph);
  tesseract::scene_graph::SceneState scene_state = state_solver.getState();

  auto calls = std::make_shared<int>(0);
  const Eigen::VectorXd tool_resolution = Eigen::VectorXd::Constant(1, 0.1);

  // tool_joint spans [-pi, pi]: LinSpaced(64, -pi, pi) whose last sample repeats the first pose.
  const RTPInvKin full_turn(*scene_graph,
                            scene_state,
                            std::make_unique<CountingInvKin>(calls),
                            2.0,
                            makeToolFwdKinABB(*scene_graph),
                            tool_resolution);
  EXPECT_EQ(countInnerSolves(full_turn, calls, Eigen::Isometry3d::Identity()), 63);

  // A range short of a full turn keeps both endpoints: LinSpaced(33, -pi/2, pi/2).
  Eigen::MatrixX2d half_turn_range(1, 2);
  half_turn_range << -M_PI / 2.0, M_PI / 2.0;
  const RTPInvKin half_turn(*scene_graph,
                            scene_state,
                            std::make_unique<CountingInvKin>(calls),
                            2.0,
                            makeToolFwdKinABB(*scene_graph),
                            half_turn_range,
                            tool_resolution);
  EXPECT_EQ(countInnerSolves(half_turn, calls, Eigen::Isometry3d::Identity()), 33);
}

TEST(TesseractKinematicsUnit, RTPInvKinRejectsFloatingJointBetweenManipTipAndToolBase)  // NOLINT
{
  // ShortestPath::active_joints omits FLOATING joints, yet their transform can still change.
  tesseract::common::GeneralResourceLocator locator;
  auto scene_graph = getSceneGraphABBWithActiveJointBeforeToolPositioner(locator);
  tesseract::scene_graph::KDLStateSolver state_solver(*scene_graph);
  tesseract::scene_graph::SceneState scene_state = state_solver.getState();
  auto opw_kin = makeOPWInvKinABB(*scene_graph);
  auto tool_kin = std::make_unique<KDLFwdKinChain>(*scene_graph, "tool_pivot", "tool_tip");

  std::const_pointer_cast<tesseract::scene_graph::Joint>(scene_graph->getJoint("bad_extra_joint"))->type =
      tesseract::scene_graph::JointType::FLOATING;

  try
  {
    auto kin = std::make_unique<RTPInvKin>(
        *scene_graph, scene_state, std::move(opw_kin), 2.0, std::move(tool_kin), Eigen::VectorXd::Constant(1, 0.1));
    ADD_FAILURE() << "Expected rejection for a floating joint between manipulator tip and tool base";
  }
  catch (const std::runtime_error& e)
  {
    EXPECT_NE(std::string(e.what()).find("found non-fixed joint 'bad_extra_joint'"), std::string::npos)
        << "threw for the wrong reason: " << e.what();
  }
}

TEST(TesseractKinematicsUnit, RTPInvKinRejectsToolFwdKinWithoutTipPose)  // NOLINT
{
  tesseract::common::GeneralResourceLocator locator;
  auto scene_graph = getSceneGraphABBWithToolPositioner(locator);
  tesseract::scene_graph::KDLStateSolver state_solver(*scene_graph);
  tesseract::scene_graph::SceneState scene_state = state_solver.getState();

  // StubFwdKin::calcFwdKin writes nothing, so its tip pose is never returned.
  try
  {
    auto kin = std::make_unique<RTPInvKin>(*scene_graph,
                                           scene_state,
                                           makeOPWInvKinABB(*scene_graph),
                                           2.0,
                                           std::make_unique<StubFwdKin>("tool0"),
                                           Eigen::VectorXd::Constant(1, 0.1));
    ADD_FAILURE() << "Expected rejection for tool forward kinematics that omit the tip link";
  }
  catch (const std::runtime_error& e)
  {
    EXPECT_NE(std::string(e.what()).find("did not return tip link 'stub_tip'"), std::string::npos)
        << "threw for the wrong reason: " << e.what();
  }
}

TEST(TesseractKinematicsUnit, RTPInvKinCopyAssignLeavesTargetOnFailedClone)  // NOLINT
{
  tesseract::common::GeneralResourceLocator locator;
  auto scene_graph = getSceneGraphABBWithToolPositioner(locator);
  tesseract::scene_graph::KDLStateSolver state_solver(*scene_graph);
  tesseract::scene_graph::SceneState scene_state = state_solver.getState();
  const Eigen::VectorXd tool_resolution = Eigen::VectorXd::Constant(1, 0.1);

  const RTPInvKin src(*scene_graph,
                      scene_state,
                      std::make_unique<ThrowingCloneInvKin>(),
                      2.0,
                      makeToolFwdKinABB(*scene_graph),
                      tool_resolution,
                      "src");
  RTPInvKin dst(*scene_graph,
                scene_state,
                makeOPWInvKinABB(*scene_graph),
                2.0,
                makeToolFwdKinABB(*scene_graph),
                tool_resolution,
                "dst");

  EXPECT_THROW(dst = src, std::runtime_error);  // NOLINT
  EXPECT_EQ(dst.getSolverName(), "dst");
  auto clone = dst.clone();
  EXPECT_EQ(clone->getSolverName(), "dst");
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
