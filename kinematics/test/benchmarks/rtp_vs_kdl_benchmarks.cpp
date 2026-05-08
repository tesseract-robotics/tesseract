/**
 * @file rtp_vs_kdl_benchmarks.cpp
 * @brief Benchmark comparing RTPInvKin (OPW + 1-DOF tool positioner) against
 *        KDLInvKinChainNR_JL on the same 7-DOF ABB IRB2400 + tool positioner
 *        fixture. Reports per-call wall time plus solution-count / success
 *        counters. Note: the two solvers produce fundamentally different
 *        solution sets (RTP enumerates, KDL-NR-JL returns at most one), so
 *        compare time-to-first-solution, not total work done.
 */

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <benchmark/benchmark.h>
#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <vector>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include "kinematics_test_utils.h"

#include <tesseract/common/resource_locator.h>
#include <tesseract/kinematics/inverse_kinematics.h>
#include <tesseract/kinematics/rtp_inv_kin.h>
#include <tesseract/kinematics/opw/opw_inv_kin.h>
#include <tesseract/kinematics/kdl/kdl_fwd_kin_chain.h>
#include <tesseract/kinematics/kdl/kdl_inv_kin_chain_nr_jl.h>
#include <tesseract/scene_graph/graph.h>
#include <tesseract/scene_graph/scene_state.h>
#include <tesseract/state_solver/kdl/kdl_state_solver.h>
#include <opw_kinematics/opw_parameters.h>

using namespace tesseract::kinematics;
using namespace tesseract::kinematics::test_suite;

namespace
{

/// Number of randomized reachable targets to sample. Chosen so even warm-KDL
/// (fastest benchmark, ~5 us/call) runs >= 10^4 iterations per repetition.
constexpr std::size_t NUM_TARGETS = 256;

/// Std-dev of Gaussian noise added to the ground-truth joint vector to form
/// the KDL warm seed, in radians. 0.1 rad ~ 5.7 deg - comparable to a single
/// planning step, so KDL sees a realistic "previous solution nearby" seed.
constexpr double NOISE_SIGMA_RAD = 0.1;

/// Deterministic RNG seed so benchmark reruns produce identical numbers.
/// Bump if you ever need to regenerate the target distribution.
constexpr std::uint64_t TARGET_RNG_SEED = 0xB1A5EDU;

/// Tool-tip link name in the ABB IRB2400 + tool positioner fixture.
constexpr const char* TOOL_TIP_LINK = "tool_tip";

opw_kinematics::Parameters<double> abbIrb2400OpwParameters()
{
  opw_kinematics::Parameters<double> p;
  p.a1 = 0.100;
  p.a2 = -0.135;
  p.b = 0.000;
  p.c1 = 0.615;
  p.c2 = 0.705;
  p.c3 = 0.755;
  p.c4 = 0.085;
  p.offsets[2] = -M_PI / 2.0;
  return p;
}

InverseKinematics::UPtr makeOpwInvKinABB(const tesseract::scene_graph::SceneGraph& sg)
{
  auto fwd = std::make_unique<KDLFwdKinChain>(sg, "base_link", "tool0");
  return std::make_unique<OPWInvKin>(
      abbIrb2400OpwParameters(), fwd->getBaseLinkName(), fwd->getTipLinkNames()[0], fwd->getJointNames());
}

std::unique_ptr<ForwardKinematics> makeToolFwdKinABB(const tesseract::scene_graph::SceneGraph& sg)
{
  return std::make_unique<KDLFwdKinChain>(sg, "tool0", TOOL_TIP_LINK);
}

/// Returns true iff `sol` is inside joint limits and FK(sol) reproduces `target_pose`
/// within position/orientation tolerance. Used by selfCheck to assert that the
/// solvers' returned sets really are all valid - the benchmarks then trust
/// sols.size() at runtime without re-validating on the timed path.
#ifndef NDEBUG
bool isValidSolution(const Eigen::VectorXd& sol,
                     ForwardKinematics& full_fk,
                     const Eigen::Isometry3d& target_pose,
                     const tesseract::common::KinematicLimits& limits,
                     double pos_tol = 1e-4,
                     double rot_tol = 1e-4)
{
  if (sol.size() != limits.joint_limits.rows())
    return false;

  for (Eigen::Index i = 0; i < sol.size(); ++i)
  {
    if (sol(i) < limits.joint_limits(i, 0) - 1e-9 || sol(i) > limits.joint_limits(i, 1) + 1e-9)
      return false;
  }

  tesseract::common::TransformMap fwd;
  full_fk.calcFwdKin(fwd, sol);
  const auto tip = fwd.at(TOOL_TIP_LINK);

  if ((tip.translation() - target_pose.translation()).norm() > pos_tol)
    return false;

  const Eigen::Matrix3d dR = tip.linear().transpose() * target_pose.linear();
  const double rot_err = Eigen::AngleAxisd(dR).angle();
  return std::abs(rot_err) <= rot_tol;
}
#endif  // NDEBUG

struct Target
{
  tesseract::common::TransformMap tip_link_poses;  ///< keyed by TOOL_TIP_LINK
  Eigen::VectorXd ground_truth;                    ///< 7-DOF joint vector that produced the target
  Eigen::VectorXd seed_warm;                       ///< ground_truth + N(0, NOISE_SIGMA_RAD), clamped to limits
};

/// Build N randomized reachable tool_tip targets via FK roundtrip on joint
/// vectors sampled uniformly within each joint's limits. Each target also
/// stores a noisy seed (ground_truth + N(0, NOISE_SIGMA_RAD), clamped to
/// limits) used by the KDL warm benchmark to reflect real planning conditions
/// where the previous-step solution is close but not exact.
std::vector<Target> buildRandomTargets(const tesseract::scene_graph::SceneGraph& sg)
{
  KDLFwdKinChain full_fk(sg, "base_link", TOOL_TIP_LINK);
  const auto limits = getTargetLimits(sg, full_fk.getJointNames());
  const Eigen::Index ndof = limits.joint_limits.rows();

  std::mt19937_64 rng(TARGET_RNG_SEED);
  std::normal_distribution<double> noise_dist(0.0, NOISE_SIGMA_RAD);

  std::vector<std::uniform_real_distribution<double>> per_joint;
  per_joint.reserve(static_cast<std::size_t>(ndof));
  for (Eigen::Index i = 0; i < ndof; ++i)
    per_joint.emplace_back(limits.joint_limits(i, 0), limits.joint_limits(i, 1));

  std::vector<Target> out;
  out.reserve(NUM_TARGETS);
  for (std::size_t k = 0; k < NUM_TARGETS; ++k)
  {
    Eigen::VectorXd q(ndof);
    for (Eigen::Index i = 0; i < ndof; ++i)
      q(i) = per_joint[static_cast<std::size_t>(i)](rng);

    Eigen::VectorXd q_noisy = q;
    for (Eigen::Index i = 0; i < ndof; ++i)
    {
      q_noisy(i) = std::clamp(q(i) + noise_dist(rng), limits.joint_limits(i, 0), limits.joint_limits(i, 1));
    }

    tesseract::common::TransformMap fwd;
    full_fk.calcFwdKin(fwd, q);

    Target t;
    t.ground_truth = q;
    t.seed_warm = q_noisy;
    t.tip_link_poses[TOOL_TIP_LINK] = fwd.at(TOOL_TIP_LINK);
    out.push_back(std::move(t));
  }
  return out;
}

/// Constructed once, shared across every registered benchmark by reference.
struct Fixture
{
  tesseract::scene_graph::SceneGraph::UPtr scene_graph;
  tesseract::scene_graph::SceneState scene_state;
  std::vector<Target> targets;
};

Fixture makeFixture()
{
  tesseract::common::GeneralResourceLocator locator;
  auto sg = getSceneGraphABBWithToolPositioner(locator);
  tesseract::scene_graph::KDLStateSolver ss(*sg);
  tesseract::scene_graph::SceneState state = ss.getState();
  auto targets = buildRandomTargets(*sg);

  Fixture f;
  f.scene_graph = std::move(sg);
  f.scene_state = std::move(state);
  f.targets = std::move(targets);
  return f;
}

RTPInvKin::UPtr makeRTP(const Fixture& f, double tool_resolution_rad)
{
  Eigen::VectorXd resolution = Eigen::VectorXd::Constant(1, tool_resolution_rad);
  // manipulator_reach is auto-derived from the manipulator's base->tip chain.
  return std::make_unique<RTPInvKin>(*f.scene_graph,
                                     f.scene_state,
                                     makeOpwInvKinABB(*f.scene_graph),
                                     makeToolFwdKinABB(*f.scene_graph),
                                     resolution);
}

KDLInvKinChainNR_JL::UPtr makeKDLNRJL(const Fixture& f)
{
  KDLInvKinChainNR_JL::Config config;  // KDL defaults
  return std::make_unique<KDLInvKinChainNR_JL>(*f.scene_graph, "base_link", TOOL_TIP_LINK, config);
}

/// Benchmarks RTPInvKin::calcInvKin over the fixed target set.
/// `state.range(0)` encodes the tool sample resolution in milliradians so the
/// benchmark label shows the resolution clearly.
///
/// The seed passed here is `t.ground_truth`, while BM_KDL_NR_JL_WARM uses
/// `t.seed_warm`. This is intentional for OPW (the inner solver is closed-form
/// and ignores the seed) but the comparison would become apples-to-oranges if
/// the inner manipulator solver were swapped to a seed-dependent one.
void BM_RTP_INV_KIN(benchmark::State& state, const Fixture* f)
{
  const double tool_res_rad = static_cast<double>(state.range(0)) * 1e-3;
  auto rtp = makeRTP(*f, tool_res_rad);

  IKSolutions sols;
  sols.reserve(1024);
  std::size_t total_calls = 0;
  std::size_t total_solutions = 0;
  std::size_t total_successes = 0;

  for (auto _ : state)  // NOLINT
  {
    for (const auto& t : f->targets)
    {
      sols.clear();
      rtp->calcInvKin(sols, t.tip_link_poses, t.ground_truth);
      benchmark::DoNotOptimize(sols);
      total_solutions += sols.size();
      total_successes += (sols.empty() ? 0U : 1U);
      ++total_calls;
    }
  }

  // Per-call averages reported via Google Benchmark counters.
  state.counters["valid_solutions_per_call"] =
      benchmark::Counter(static_cast<double>(total_solutions) / static_cast<double>(total_calls));
  state.counters["success_rate"] =
      benchmark::Counter(static_cast<double>(total_successes) / static_cast<double>(total_calls));
  // Google Benchmark divides `total_solutions` by the total elapsed real time.
  // With kInvert, the second counter flips that to seconds per valid solution,
  // so you can read the RTP-vs-KDL cost ratio directly.
  state.counters["valid_sols_per_sec"] =
      benchmark::Counter(static_cast<double>(total_solutions), benchmark::Counter::kIsRate);
  state.counters["sec_per_valid_sol"] = benchmark::Counter(
      static_cast<double>(total_solutions), benchmark::Counter::kIsRate | benchmark::Counter::kInvert);
}

/// Benchmarks KDLInvKinChainNR_JL with a perturbed seed (realistic planning conditions).
void BM_KDL_NR_JL_WARM(benchmark::State& state, const Fixture* f)
{
  auto kdl = makeKDLNRJL(*f);

  IKSolutions sols;
  sols.reserve(2);
  std::size_t total_calls = 0;
  std::size_t total_solutions = 0;
  std::size_t total_successes = 0;

  for (auto _ : state)  // NOLINT
  {
    for (const auto& t : f->targets)
    {
      sols.clear();
      kdl->calcInvKin(sols, t.tip_link_poses, t.seed_warm);
      benchmark::DoNotOptimize(sols);
      total_solutions += sols.size();
      total_successes += (sols.empty() ? 0U : 1U);
      ++total_calls;
    }
  }

  state.counters["valid_solutions_per_call"] =
      benchmark::Counter(static_cast<double>(total_solutions) / static_cast<double>(total_calls));
  state.counters["success_rate"] =
      benchmark::Counter(static_cast<double>(total_successes) / static_cast<double>(total_calls));
  state.counters["valid_sols_per_sec"] =
      benchmark::Counter(static_cast<double>(total_solutions), benchmark::Counter::kIsRate);
  state.counters["sec_per_valid_sol"] = benchmark::Counter(
      static_cast<double>(total_solutions), benchmark::Counter::kIsRate | benchmark::Counter::kInvert);
}

/// Benchmarks KDLInvKinChainNR_JL with a zero seed (worst-case: no prior knowledge).
/// Expect higher iteration counts, lower success rate, and higher time per call.
void BM_KDL_NR_JL_COLD(benchmark::State& state, const Fixture* f)
{
  auto kdl = makeKDLNRJL(*f);
  const Eigen::VectorXd zero_seed = Eigen::VectorXd::Zero(kdl->numJoints());

  IKSolutions sols;
  sols.reserve(2);
  std::size_t total_calls = 0;
  std::size_t total_solutions = 0;
  std::size_t total_successes = 0;

  for (auto _ : state)  // NOLINT
  {
    for (const auto& t : f->targets)
    {
      sols.clear();
      kdl->calcInvKin(sols, t.tip_link_poses, zero_seed);
      benchmark::DoNotOptimize(sols);
      total_solutions += sols.size();
      total_successes += (sols.empty() ? 0U : 1U);
      ++total_calls;
    }
  }

  state.counters["valid_solutions_per_call"] =
      benchmark::Counter(static_cast<double>(total_solutions) / static_cast<double>(total_calls));
  state.counters["success_rate"] =
      benchmark::Counter(static_cast<double>(total_successes) / static_cast<double>(total_calls));
  state.counters["valid_sols_per_sec"] =
      benchmark::Counter(static_cast<double>(total_solutions), benchmark::Counter::kIsRate);
  state.counters["sec_per_valid_sol"] = benchmark::Counter(
      static_cast<double>(total_solutions), benchmark::Counter::kIsRate | benchmark::Counter::kInvert);
}

/// Always-on sanity guards executed before RunSpecifiedBenchmarks.
/// Bare assert() is a no-op in Release benchmark builds, which would let
/// a refactor that breaks IK silently produce throughput numbers for broken
/// IK. These guards survive Release; the inner FK-validation loops remain
/// debug-only because they are expensive.
void selfCheck(const Fixture& f)
{
  auto fail = [](const char* msg) {
    std::fprintf(stderr, "selfCheck: %s\n", msg);
    std::abort();
  };

  if (f.targets.empty())
    fail("fixture has no targets");

  const auto& target0 = f.targets.front();

#ifndef NDEBUG
  // Full-chain FK and limits used by the per-solution validator below.
  KDLFwdKinChain full_fk(*f.scene_graph, "base_link", TOOL_TIP_LINK);
  const auto limits = getTargetLimits(*f.scene_graph, full_fk.getJointNames());
  const auto tool_tip_pose = target0.tip_link_poses.at(TOOL_TIP_LINK);
#endif  // NDEBUG

  auto rtp = makeRTP(f, 0.1);
  if (rtp->numJoints() != 7)
    fail("RTP solver reports unexpected joint count (expected 7)");
  {
    IKSolutions sols;
    rtp->calcInvKin(sols, target0.tip_link_poses, target0.ground_truth);
    if (sols.empty())
      fail("RTP returned no solutions for a reachable target");
#ifndef NDEBUG
    for (const auto& s : sols)
      assert(isValidSolution(s, full_fk, tool_tip_pose, limits) &&
             "RTP returned a solution outside limits or not matching the target pose");
#endif  // NDEBUG
  }

  auto kdl = makeKDLNRJL(f);
  if (kdl->numJoints() != 7)
    fail("KDL-NR-JL solver reports unexpected joint count (expected 7)");
  {
    IKSolutions sols;
    kdl->calcInvKin(sols, target0.tip_link_poses, target0.ground_truth);
    if (sols.empty())
      fail("KDL-NR-JL returned no solutions for a ground-truth-seeded target");
#ifndef NDEBUG
    for (const auto& s : sols)
      assert(isValidSolution(s, full_fk, tool_tip_pose, limits) &&
             "KDL-NR-JL returned a solution outside limits or not matching the target pose");
#endif  // NDEBUG
  }
  {
    IKSolutions sols;
    kdl->calcInvKin(sols, target0.tip_link_poses, target0.seed_warm);
    if (sols.empty())
      fail("KDL-NR-JL did not converge from a perturbed seed on target[0]; "
           "if this trips, lower NOISE_SIGMA_RAD");
#ifndef NDEBUG
    for (const auto& s : sols)
      assert(isValidSolution(s, full_fk, tool_tip_pose, limits) &&
             "KDL-NR-JL returned an invalid solution from the perturbed seed");
#endif  // NDEBUG
  }
}

}  // namespace

int main(int argc, char** argv)
{
  Fixture fixture = makeFixture();
  selfCheck(fixture);

  {
    // Tool resolution sweep: 0.05, 0.10, 0.20 rad (encoded as milliradians so
    // Google Benchmark's int Arg() carries the label cleanly).
    const std::vector<int> tool_res_mrad = { 50, 100, 200 };
    auto* rtp_bench = benchmark::RegisterBenchmark("BM_RTP_INV_KIN/tool_res_mrad", BM_RTP_INV_KIN, &fixture);
    for (int mrad : tool_res_mrad)
      rtp_bench->Arg(mrad);
    rtp_bench->UseRealTime()->Unit(benchmark::TimeUnit::kMicrosecond);
  }

  benchmark::RegisterBenchmark("BM_KDL_NR_JL_WARM", BM_KDL_NR_JL_WARM, &fixture)
      ->UseRealTime()
      ->Unit(benchmark::TimeUnit::kMicrosecond);

  benchmark::RegisterBenchmark("BM_KDL_NR_JL_COLD", BM_KDL_NR_JL_COLD, &fixture)
      ->UseRealTime()
      ->Unit(benchmark::TimeUnit::kMicrosecond);

  benchmark::Initialize(&argc, argv);
  benchmark::RunSpecifiedBenchmarks();
  return 0;
}
