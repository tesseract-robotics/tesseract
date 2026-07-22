# Kinematics Benchmarks

## RTP vs KDL-NR-JL

`tesseract_kinematics_rtp_vs_kdl_benchmark` compares `RTPInvKin`
(OPW 6-DOF manipulator + 1-DOF revolute tool positioner) against
`KDLInvKinChainNR_JL` (Newton-Raphson with joint-limit clamping) solving
the same 7-DOF IK on the ABB IRB2400 + tool positioner fixture.

### When to use which

The two solvers produce fundamentally different solution sets:
- RTP enumerates all manipulator branches at every tool-joint sample.
- KDL-NR-JL returns at most one solution, seed-dependent.

Two views on the same data:
- **Per-call time** (the main row): what a planner sees per IK query. KDL
  wins when one solution is enough.
- **`sec_per_valid_sol`**: cost amortized across all returned solutions.
  RTP wins by orders of magnitude here because it enumerates.

Neither view alone is "the" answer — pick based on whether your planner
consumes a single solution per target or wants to choose among many.

### Running

```bash
./colcon_tesseract.sh --packages-select tesseract
. install/setup.bash
./build/tesseract/kinematics/test/benchmarks/tesseract_kinematics_rtp_vs_kdl_benchmark \
  --benchmark_format=console --benchmark_repetitions=3
```

### Benchmarks

| Name | Variant |
|------|---------|
| `BM_RTP_INV_KIN/tool_res_mrad/50` | 0.050 rad tool grid |
| `BM_RTP_INV_KIN/tool_res_mrad/100` | 0.100 rad tool grid |
| `BM_RTP_INV_KIN/tool_res_mrad/200` | 0.200 rad tool grid |
| `BM_KDL_NR_JL_WARM` | seed = ground-truth + N(0, 0.1 rad), clamped |
| `BM_KDL_NR_JL_COLD` | seed = zero vector |

### Target set

Each iteration runs `NUM_TARGETS = 256` reachable tool_tip poses generated
by FK roundtrip on joint vectors sampled uniformly within each joint's
limits. The RNG is seeded deterministically (`TARGET_RNG_SEED`) so reruns
produce identical numbers.

Reported time is per iteration (all 256 calls), so divide by 256 for
per-call time — or read `sec_per_valid_sol` directly.

### Counters

- `valid_solutions_per_call` — mean number of valid IK solutions returned
  per call. Both solvers only return solutions that pass joint-limit checks;
  selfCheck additionally validates FK roundtrip on the first target's full
  solution set, so this really is the count of usable solutions. RTP scales
  with `2π / tool_resolution × (≤ 8 OPW branches)`; KDL-NR-JL is 0 or 1.
- `success_rate` — fraction of calls returning at least one solution.
- `valid_sols_per_sec` — total valid solutions divided by wall time (Google
  Benchmark rate counter).
- `sec_per_valid_sol` — inverse of the above; read the RTP-vs-KDL cost ratio
  off this counter directly.

### Warm-seed realism

`BM_KDL_NR_JL_WARM` seeds KDL at `ground_truth + N(0, NOISE_SIGMA_RAD)`
(0.1 rad ≈ 5.7°, clamped back into limits). This models planning conditions
where the previous step's solution is close to — but not exactly on — the
current target. Success rate is expected to be high but possibly < 1.0;
per-call time is expected to be higher than a ground-truth-seeded run. To
change the noise magnitude, edit `NOISE_SIGMA_RAD` and rebuild.

### Caveats

`RTPInvKin`'s `manipulator_reach` argument is an early-exit filter on
`‖target tool0 pose‖` — tool_joint samples beyond that radius are rejected
before OPW is called. This benchmark uses the auto-reach `RTPInvKin`
constructor, which derives the filter radius from the manipulator's
base→tip chain via `computeChainReachUpperBound`. For ABB IRB2400 that
bound is ~2.20 m, comfortably above the peak `‖tool0‖ ≈ 2.1 m` observed
under random joint sampling, so the filter never fires on reachable
targets.
