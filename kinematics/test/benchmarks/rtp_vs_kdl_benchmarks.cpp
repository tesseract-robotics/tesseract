/**
 * @file rtp_vs_kdl_benchmarks.cpp
 * @brief Benchmark comparing RTPInvKin (OPW + 1-DOF tool positioner) against
 *        the three KDLInvKinChain{NR_JL, NR, LMA} solvers on the same 7-DOF
 *        ABB IRB2400 + tool positioner fixture. Reports per-call wall time
 *        plus solution-count / success counters. Note: the solvers produce
 *        fundamentally different solution sets (RTP enumerates, KDL returns
 *        at most one), so compare time-to-first-solution, not total work.
 *        KDLInvKinChainNR and KDLInvKinChainLMA do not enforce joint limits
 *        and may converge to out-of-limits configurations.
 */

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <benchmark/benchmark.h>
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <vector>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include "abb_opw_fixture.h"
#include "kinematics_test_utils.h"

#include <tesseract/common/kinematic_limits.h>
#include <tesseract/common/resource_locator.h>
#include <tesseract/kinematics/inverse_kinematics.h>
#include <tesseract/kinematics/rtp_inv_kin.h>
#include <tesseract/kinematics/utils.h>
#include <tesseract/kinematics/kdl/kdl_fwd_kin_chain.h>
#include <tesseract/kinematics/kdl/kdl_inv_kin_chain_nr_jl.h>
#include <tesseract/kinematics/kdl/kdl_inv_kin_chain_nr.h>
#include <tesseract/kinematics/kdl/kdl_inv_kin_chain_lma.h>
#include <tesseract/scene_graph/graph.h>
#include <tesseract/scene_graph/scene_state.h>
#include <tesseract/state_solver/kdl/kdl_state_solver.h>

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

/// Multiple of the manipulator reach bound at which unreachable targets are placed. Far enough
/// out that every tool sample is rejected regardless of tool-joint geometry.
constexpr double UNREACHABLE_REACH_SCALE = 1.5;

/// Tool-tip link name in the ABB IRB2400 + tool positioner fixture. Shared with the unit tests
/// via abb_opw_fixture.h so both exercise the same chain.
const std::string& TOOL_TIP_LINK = ABB_TOOL_TIP_LINK;  // NOLINT(cert-err58-cpp)

/// Numerical slack added to joint-limit comparisons. Joint values within
/// kJointLimitSlack of a limit are accepted as in-bounds, mirroring how
/// downstream planners typically treat clamped values.
constexpr double kJointLimitSlack = 1e-9;

/// Returns true iff `sol` is inside joint limits and FK(sol) reproduces `target_pose`
/// within position/orientation tolerance. Used by selfCheck to assert that the
/// solvers' returned sets really are all valid - the benchmarks then trust
/// sols.size() at runtime without re-validating on the timed path.
#ifndef NDEBUG
/// Position / orientation tolerance used by the solution validator below.
constexpr double kPosTol = 1e-4;
constexpr double kRotTol = 1e-4;

bool isValidSolution(const Eigen::VectorXd& sol,
                     ForwardKinematics& full_fk,
                     const Eigen::Isometry3d& target_pose,
                     const tesseract::common::KinematicLimits& limits)
{
  if (sol.size() != limits.joint_limits.rows())
    return false;

  if (!tesseract::common::satisfiesLimits<double>(sol, limits.joint_limits, kJointLimitSlack, 0.0))
    return false;

  tesseract::common::TransformMap fwd;
  full_fk.calcFwdKin(fwd, sol);
  const auto tip = fwd.at(TOOL_TIP_LINK);

  if ((tip.translation() - target_pose.translation()).norm() > kPosTol)
    return false;

  const Eigen::Matrix3d dR = tip.linear().transpose() * target_pose.linear();
  const double rot_err = Eigen::AngleAxisd(dR).angle();
  return std::abs(rot_err) <= kRotTol;
}
#endif  // NDEBUG

/// Per-call accumulators for one benchmark run, fed into emitCounters() to
/// produce the standard counter set printed by every BM_* function.
struct CallStats
{
  std::size_t calls = 0;
  std::size_t solutions = 0;
  std::size_t successes = 0;            ///< solver returned at least one solution
  std::size_t in_limits_successes = 0;  ///< solver returned at least one in-limits solution
};

/// Emits the standard counter set for a benchmark row. Values divide by total
/// elapsed real time (kIsRate) and invert (kInvert) where the meaningful
/// quantity is a period rather than a rate. See the legend printed at the
/// bottom of main() for what each column means.
void emitCounters(benchmark::State& state, const CallStats& s)
{
  const auto calls = static_cast<double>(s.calls);
  const auto sols = static_cast<double>(s.solutions);
  const auto succ = static_cast<double>(s.successes);
  const auto in_lim = static_cast<double>(s.in_limits_successes);

  // Round non-rate counters to 3 decimals so 0.976562 prints as 0.977.
  // These names are repeated verbatim in the legend printed at the end of main(); rename in both.
  const auto round3 = [](double x) { return std::round(x * 1000.0) / 1000.0; };
  state.counters["success_rate"] = benchmark::Counter(round3(succ / calls));
  state.counters["in_limits_success_rate"] = benchmark::Counter(round3(in_lim / calls));
  state.counters["sols_per_call"] = benchmark::Counter(round3(sols / calls));
  state.counters["time_per_call"] =
      benchmark::Counter(calls, benchmark::Counter::kIsRate | benchmark::Counter::kInvert);
  state.counters["time_per_valid_success"] =
      benchmark::Counter(in_lim, benchmark::Counter::kIsRate | benchmark::Counter::kInvert);
  state.counters["time_per_sol"] = benchmark::Counter(sols, benchmark::Counter::kIsRate | benchmark::Counter::kInvert);
}

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
std::vector<Target> buildRandomTargets(ForwardKinematics& full_fk, const tesseract::common::KinematicLimits& limits)
{
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

/// Same orientations and seeds as the reachable set, with each translation pushed out along its own
/// direction to UNREACHABLE_REACH_SCALE times the manipulator reach bound. This is the workload
/// that exercises RTP's reach early-exit and the KDL solvers' non-convergence path; neither is
/// reached by targets built from FK roundtrip.
///
/// `ground_truth` is carried over unchanged. For this set it is not a solution - no solution exists
/// - it is simply a plausible seed, which is all any solver here uses it as.
std::vector<Target> buildUnreachableTargets(const std::vector<Target>& reachable, double manip_reach)
{
  const double radius = UNREACHABLE_REACH_SCALE * manip_reach;

  std::vector<Target> out;
  out.reserve(reachable.size());
  for (const Target& src : reachable)
  {
    Target t = src;
    Eigen::Isometry3d& pose = t.tip_link_poses[TOOL_TIP_LINK];
    const double norm = pose.translation().norm();
    const Eigen::Vector3d dir = (norm > 1e-9) ? Eigen::Vector3d(pose.translation() / norm) : Eigen::Vector3d::UnitX();
    pose.translation() = dir * radius;
    out.push_back(std::move(t));
  }
  return out;
}

/// Constructed once, shared across every registered benchmark by reference.
struct Fixture
{
  tesseract::scene_graph::SceneGraph::UPtr scene_graph;
  tesseract::scene_graph::SceneState scene_state;
  ForwardKinematics::UPtr full_fk;            ///< manip-base -> tool-tip; generates and validates targets
  tesseract::common::KinematicLimits limits;  ///< for the manip-base -> tool-tip chain
  std::vector<Target> targets;                ///< reachable by construction (FK roundtrip)
  std::vector<Target> targets_unreachable;    ///< outside the manipulator reach bound
};

Fixture makeFixture()
{
  tesseract::common::GeneralResourceLocator locator;
  auto sg = getSceneGraphABBWithToolPositioner(locator);
  tesseract::scene_graph::KDLStateSolver ss(*sg);

  // Built once: target generation, the in-limits counter and selfCheck must all agree on which
  // chain and which limits they are talking about.
  auto full_fk = std::make_unique<KDLFwdKinChain>(*sg, ABB_BASE_LINK, TOOL_TIP_LINK);
  auto limits = getTargetLimits(*sg, full_fk->getJointNames());

  // The same bound RTP derives internally via its auto-reach constructor, so the unreachable set is
  // placed relative to the radius the solver actually filters on.
  const double manip_reach = computeChainReachUpperBound(*sg, ABB_BASE_LINK, ABB_MANIP_TIP_LINK);

  Fixture f;
  f.scene_state = ss.getState();
  f.targets = buildRandomTargets(*full_fk, limits);
  f.targets_unreachable = buildUnreachableTargets(f.targets, manip_reach);
  f.scene_graph = std::move(sg);
  f.full_fk = std::move(full_fk);
  f.limits = std::move(limits);
  return f;
}

/// Returns true iff `sols` contains at least one element whose joint values
/// all lie within the kinematic limits (with kJointLimitSlack tolerance).
/// KDLInvKinChainNR and KDLInvKinChainLMA do not enforce limits; this lets
/// the benchmark count "planner-usable" successes separately from raw solver
/// convergence. Allocation-free and dominated by calcInvKin, so safe inside
/// the timed loop.
bool anyInLimits(const IKSolutions& sols, const tesseract::common::KinematicLimits& limits)
{
  const auto& jl = limits.joint_limits;
  return std::any_of(sols.begin(), sols.end(), [&](const Eigen::VectorXd& s) {
    return s.size() == jl.rows() && tesseract::common::satisfiesLimits<double>(s, jl, kJointLimitSlack, 0.0);
  });
}

RTPInvKin::UPtr makeRTP(const Fixture& f, double tool_resolution_rad)
{
  Eigen::VectorXd resolution = Eigen::VectorXd::Constant(1, tool_resolution_rad);
  // manipulator_reach is auto-derived from the manipulator's base->tip chain.
  return std::make_unique<RTPInvKin>(
      *f.scene_graph, f.scene_state, makeOPWInvKinABB(*f.scene_graph), makeToolFwdKinABB(*f.scene_graph), resolution);
}

KDLInvKinChainNR_JL::UPtr makeKDLNRJL(const Fixture& f)
{
  KDLInvKinChainNR_JL::Config config;
  return std::make_unique<KDLInvKinChainNR_JL>(*f.scene_graph, "base_link", TOOL_TIP_LINK, config);
}

KDLInvKinChainNR::UPtr makeKDLNR(const Fixture& f)
{
  KDLInvKinChainNR::Config config;
  return std::make_unique<KDLInvKinChainNR>(*f.scene_graph, "base_link", TOOL_TIP_LINK, config);
}

KDLInvKinChainLMA::UPtr makeKDLLMA(const Fixture& f)
{
  KDLInvKinChainLMA::Config config;
  return std::make_unique<KDLInvKinChainLMA>(*f.scene_graph, "base_link", TOOL_TIP_LINK, config);
}

/// Which per-target seed a benchmark hands the solver.
enum class SeedMode : std::uint8_t
{
  GroundTruth,  ///< the exact joint vector that produced the target
  Warm,         ///< ground_truth + N(0, NOISE_SIGMA_RAD): a planner's previous step
  Cold          ///< zero vector: no prior knowledge
};

/// The entire measured region, shared by every registered benchmark. Keeping one copy is what
/// makes the rows comparable: a change here (an added counter, a moved DoNotOptimize) applies
/// to all solvers at once instead of needing seven identical edits.
void runIkBenchmark(benchmark::State& state,
                    const Fixture& f,
                    const std::vector<Target>& targets,
                    InverseKinematics& solver,
                    SeedMode seed_mode)
{
  const Eigen::VectorXd zero_seed = Eigen::VectorXd::Zero(solver.numJoints());
  const auto seedFor = [&](const Target& t) -> const Eigen::VectorXd& {
    switch (seed_mode)
    {
      case SeedMode::GroundTruth:
        return t.ground_truth;
      case SeedMode::Warm:
        return t.seed_warm;
      case SeedMode::Cold:
        break;
    }
    return zero_seed;
  };

  IKSolutions sols;
  CallStats stats;

  // Untimed warm-up: grows `sols` to this solver's working set so the first timed iteration does
  // not pay vector growth. Replaces the per-benchmark reserve() guesses this used to carry.
  solver.calcInvKin(sols, targets.front().tip_link_poses, seedFor(targets.front()));
  sols.clear();

  for (auto _ : state)  // NOLINT
  {
    for (const auto& t : targets)
    {
      sols.clear();
      solver.calcInvKin(sols, t.tip_link_poses, seedFor(t));
      benchmark::DoNotOptimize(sols);
      stats.solutions += sols.size();
      stats.successes += (sols.empty() ? 0U : 1U);
      stats.in_limits_successes += (anyInLimits(sols, f.limits) ? 1U : 0U);
      ++stats.calls;
    }
  }

  emitCounters(state, stats);
}

/// Registers one KDL row: construct the solver inside the benchmark (so construction is untimed
/// but per-repetition) and run it through the shared measured region.
template <typename MakeSolver>
void registerKdlBenchmark(const char* name,
                          const Fixture& f,
                          const std::vector<Target>& targets,
                          MakeSolver make_solver,
                          SeedMode seed_mode)
{
  benchmark::RegisterBenchmark(name,
                               [&f, &targets, make_solver, seed_mode](benchmark::State& state) {
                                 auto solver = make_solver(f);
                                 runIkBenchmark(state, f, targets, *solver, seed_mode);
                               })
      ->UseRealTime()
      ->Unit(benchmark::TimeUnit::kMicrosecond);
}

/// Registers the same KDL row over both workloads. Every solver is measured on both, so a reader
/// cannot mistake a reachable-only number for the solver's general cost.
template <typename MakeSolver>
void registerKdlBenchmarkBothWorkloads(const std::string& name,
                                       const Fixture& f,
                                       MakeSolver make_solver,
                                       SeedMode seed_mode)
{
  registerKdlBenchmark((name + "_REACHABLE").c_str(), f, f.targets, make_solver, seed_mode);
  registerKdlBenchmark((name + "_UNREACHABLE").c_str(), f, f.targets_unreachable, make_solver, seed_mode);
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

  if (f.targets_unreachable.size() != f.targets.size())
    fail("unreachable target set does not mirror the reachable one");

  const auto& target0 = f.targets.front();

#ifndef NDEBUG
  ForwardKinematics& full_fk = *f.full_fk;
  const auto& limits = f.limits;
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
      assert(isValidSolution(s, full_fk, tool_tip_pose, limits) && "RTP returned a solution outside limits or not "
                                                                   "matching the target pose");
#endif  // NDEBUG
  }
  {
    // The unreachable set is only meaningful if it really is unreachable. Checked over every
    // target, not just the first: one solvable pose would quietly turn the UNREACHABLE rows back
    // into a partial copy of the reachable ones.
    IKSolutions sols;
    for (const auto& t : f.targets_unreachable)
    {
      sols.clear();
      rtp->calcInvKin(sols, t.tip_link_poses, t.ground_truth);
      if (!sols.empty())
        fail("RTP solved a target placed outside the manipulator reach; raise UNREACHABLE_REACH_SCALE");
    }
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
      assert(isValidSolution(s, full_fk, tool_tip_pose, limits) && "KDL-NR-JL returned a solution outside limits or "
                                                                   "not matching the target pose");
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
      assert(isValidSolution(s, full_fk, tool_tip_pose, limits) && "KDL-NR-JL returned an invalid solution from the "
                                                                   "perturbed seed");
#endif  // NDEBUG
  }

  // KDLInvKinChainNR / LMA do not enforce joint limits, so we only assert
  // that the solver returns at least one solution; FK/limits validation is
  // skipped because these solvers can legitimately exit out-of-limits.
  auto kdl_nr = makeKDLNR(f);
  if (kdl_nr->numJoints() != 7)
    fail("KDL-NR solver reports unexpected joint count (expected 7)");
  {
    IKSolutions sols;
    kdl_nr->calcInvKin(sols, target0.tip_link_poses, target0.ground_truth);
    if (sols.empty())
      fail("KDL-NR returned no solutions for a ground-truth-seeded target");
  }

  auto kdl_lma = makeKDLLMA(f);
  if (kdl_lma->numJoints() != 7)
    fail("KDL-LMA solver reports unexpected joint count (expected 7)");
  {
    IKSolutions sols;
    kdl_lma->calcInvKin(sols, target0.tip_link_poses, target0.ground_truth);
    if (sols.empty())
      fail("KDL-LMA returned no solutions for a ground-truth-seeded target");
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
    //
    // RTP is seeded with ground_truth while the KDL WARM rows use seed_warm. That is intentional:
    // RTP's inner OPW solver is closed-form and ignores the seed, so any seed is equivalent for
    // it. The comparison would become apples-to-oranges if the inner solver were ever swapped for
    // a seed-dependent one.
    const std::vector<int> tool_res_mrad = { 50, 100, 200 };
    const auto register_rtp = [&fixture, &tool_res_mrad](const char* name, const std::vector<Target>& targets) {
      auto* bench = benchmark::RegisterBenchmark(name, [&fixture, &targets](benchmark::State& state) {
        auto rtp = makeRTP(fixture, static_cast<double>(state.range(0)) * 1e-3);
        runIkBenchmark(state, fixture, targets, *rtp, SeedMode::GroundTruth);
      });
      for (int mrad : tool_res_mrad)
        bench->Arg(mrad);
      bench->ArgName("tool_res_mrad")->UseRealTime()->Unit(benchmark::TimeUnit::kMicrosecond);
    };
    register_rtp("BM_RTP_INV_KIN_REACHABLE", fixture.targets);
    register_rtp("BM_RTP_INV_KIN_UNREACHABLE", fixture.targets_unreachable);
  }

  registerKdlBenchmarkBothWorkloads("BM_KDL_NR_JL_WARM", fixture, makeKDLNRJL, SeedMode::Warm);
  registerKdlBenchmarkBothWorkloads("BM_KDL_NR_JL_COLD", fixture, makeKDLNRJL, SeedMode::Cold);
  registerKdlBenchmarkBothWorkloads("BM_KDL_NR_WARM", fixture, makeKDLNR, SeedMode::Warm);
  registerKdlBenchmarkBothWorkloads("BM_KDL_NR_COLD", fixture, makeKDLNR, SeedMode::Cold);
  registerKdlBenchmarkBothWorkloads("BM_KDL_LMA_WARM", fixture, makeKDLLMA, SeedMode::Warm);
  registerKdlBenchmarkBothWorkloads("BM_KDL_LMA_COLD", fixture, makeKDLLMA, SeedMode::Cold);

  benchmark::Initialize(&argc, argv);
  benchmark::RunSpecifiedBenchmarks();

  std::printf("\n"
              "Counter legend:\n"
              "  success_rate             fraction of calls returning at least one solution\n"
              "                           (raw solver convergence; ignores joint limits)\n"
              "  in_limits_success_rate   fraction returning at least one in-limits solution\n"
              "                           (planner-relevant: NR/LMA may converge out-of-limits)\n"
              "  sols_per_call            solutions returned per call (RTP enumerates, KDL <= 1)\n"
              "  time_per_call            wall time per calcInvKin call\n"
              "  time_per_valid_success   wall time per call yielding an in-limits answer\n"
              "                           (= time_per_call / in_limits_success_rate; the\n"
              "                            number a planner actually pays per usable IK)\n"
              "  time_per_sol             amortised wall time per returned solution\n"
              "                           (for RTP, averages over the enumerated set)\n"
              "\n"
              "Note: the 'Time' / 'CPU' columns are per inner loop (%zu calcInvKin calls).\n"
              "      Use time_per_call for direct per-call comparison.\n"
              "\n"
              "Picking a solver:\n"
              "  Single-shot (RRT step, IK warm-start): compare time_per_call.\n"
              "  Batch sampling (trajectory seeding):   compare time_per_sol.\n"
              "  Realistic planner cost:                compare time_per_valid_success.\n"
              "\n"
              "If success_rate > in_limits_success_rate the solver is converging to\n"
              "configurations the planner would reject. NR_JL keeps these equal by\n"
              "construction; NR/LMA may diverge (especially from cold seeds).\n",
              NUM_TARGETS);
  return 0;
}
