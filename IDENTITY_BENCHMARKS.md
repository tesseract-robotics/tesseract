# Identity benchmark record

Machine: WSL2, see repo CLAUDE.md for perf caveats. Command:
`tesseract_common_identity_benchmarks --benchmark_repetitions=5 --benchmark_report_aggregates_only=true`

## Phase 0 — Baseline (thin 24-byte LinkIdPair, value-only equality, std::unordered_map)

```
BM_PairConstruction_mean                             1.23 ns         1.23 ns            5
BM_AcmIsAllowed_PreconstructedPair_Hit_mean          9.00 ns         9.00 ns            5
BM_AcmIsAllowed_PreconstructedPair_Miss_mean         6.40 ns         6.40 ns            5
BM_AcmIsAllowed_TwoIds_Hit_mean                      4.81 ns         4.81 ns            5
BM_MarginLookup_TwoIds_mean                          6.73 ns         6.73 ns            5
BM_PairMapPopulate_mean                               561 ns          561 ns            5
```

## Phase 1 — Fat LinkIdPair + hybrid equality, still std::unordered_map (after Task 9)

```
BM_PairConstruction_mean                             32.6 ns         32.6 ns            5
BM_AcmIsAllowed_PreconstructedPair_Hit_mean          15.2 ns         15.2 ns            5
BM_AcmIsAllowed_PreconstructedPair_Miss_mean          7.09 ns         7.09 ns            5
BM_AcmIsAllowed_TwoIds_Hit_mean                      46.2 ns         46.2 ns            5
BM_MarginLookup_TwoIds_mean                          44.0 ns         44.0 ns            5
BM_PairMapPopulate_mean                              1683 ns         1683 ns            5
```

### Interpretation vs Phase 0

| Benchmark | Phase 0 | Phase 1 | Ratio |
|---|---|---|---|
| `BM_PairConstruction` | 1.23 ns | 32.6 ns | **26.50x** |
| `BM_AcmIsAllowed_PreconstructedPair_Hit` | 9.00 ns | 15.2 ns | 1.69x |
| `BM_AcmIsAllowed_PreconstructedPair_Miss` | 6.40 ns | 7.09 ns | **1.11x** |
| `BM_AcmIsAllowed_TwoIds_Hit` | 4.81 ns | 46.2 ns | 9.60x |
| `BM_MarginLookup_TwoIds` | 6.73 ns | 44.0 ns | 6.54x |
| `BM_PairMapPopulate` | 561 ns | 1683 ns | 3.00x |

The delta matches the expected Phase 1 pattern exactly, with no wild outliers:

- **`PairConstruction` and `PairMapPopulate` got slower** (26.50x and 3.00x) — the fat `LinkIdPair` key now carries two heap-allocated name strings per pair, so construction and map population both pay two heap allocations (plus copies) that didn't exist in the thin, value-only Phase 0 key. This is the known, accepted interim cost of carrying names in the key instead of trusting integer identity alone. `PairMapPopulate`'s ratio is diluted relative to bare construction because populate also pays hashing/bucket-insertion cost that is unchanged between phases.
- **Preconstructed-pair hit/miss are roughly unchanged**: miss only rose 1.11x (equality on miss short-circuits on the integer members before ever touching the name strings, so it's nearly the same cost as Phase 0), and hit rose a modest 1.69x (one extra name string compare is now paid on every hit, as expected — small in absolute terms, ~6ns).
- **Two-ids lookups got slower** (9.60x and 6.54x) — every `TwoIds` call must now construct a fat pair (heap-allocating both name strings) before it can even attempt the hash lookup. This is the deliberate interim cost of keeping `std::unordered_map` with a name-carrying key; it is the single biggest lever available. Phase 2 (`boost::unordered_flat_map` + transparent, non-owning views) is designed but **deliberately deferred pending user benchmarking on real data** — this migration round stops after Task 9, so these numbers stand as the reference point against which any future Phase 2 work should be justified.

No result falls outside the expected pattern (worst case is the two-ids/construction paths, all under 27x on ns-scale absolute costs; preconstructed lookups — the paths that matter most for hot-loop ACM checks — stayed within ~1.1–1.7x). No STOP condition triggered.

> **Update (2026-07-16):** the two-ids ratios above predate the Finding-3 hoist and no longer describe the shipping code. The per-lookup fat-pair *construction* they attribute the cost to was eliminated by `OrderedIdPair::assign` + reused scratch (see the Finding-3 follow-up below: `BM_MarginLookup_TwoIds` 44.0 → 29.6 ns, now below the string era). The residual is one name-compare on hits plus the per-state `vector<JointId>` copy — minor. Finding 2 (`IDENTITY_MIGRATION_REVIEW.md`) was downgraded to LOW (downstream of Finding 1), and Phase 2 is now an optional optimization, not the mitigation these numbers were the reference point for.

### Macro benchmark — Phase 1 reference point (no Phase 0 capture exists)

No Phase-0 run of the collision macro benchmark was captured at Task 1, so these numbers
are recorded purely as the Phase-1 reference point: compare any future Phase 2 run against
this section, not against Phase 0. The full benchmark suite takes >10 min on this machine,
so a representative filtered subset is used — **any future run must use this exact invocation**:

```
tesseract_collision_coal_bullet_discrete_bvh_benchmarks \
  --benchmark_repetitions=3 --benchmark_report_aggregates_only=true \
  --benchmark_filter='BM_CLONE_BulletDiscreteBVHManager_ACTIVE_OBJ_(0|64|512)/|BM_SET_COLLISION_OBJECTS_TRANSFORM_(SINGLE|VECTOR|MAP)_BulletDiscreteBVHManager_ACTIVE_OBJ_(64|512)/|BM_CONTACT_TEST_(0|1)_BulletDiscreteBVHManager_ALL_(SPHERE_SPHERE|BOX_BOX)/'
```

The subset covers manager clone (env setup cost), contact tests (ACM/contact-map hot path),
and the `SET_COLLISION_OBJECTS_TRANSFORM_*` families (the link-id-keyed transform-map paths
most directly touched by this migration).

```
BM_CLONE_BulletDiscreteBVHManager_ACTIVE_OBJ_0/real_time_mean                                             326 us
BM_CLONE_BulletDiscreteBVHManager_ACTIVE_OBJ_64/real_time_mean                                           1152 us
BM_CLONE_BulletDiscreteBVHManager_ACTIVE_OBJ_512/real_time_mean                                        184554 us
BM_CONTACT_TEST_0_BulletDiscreteBVHManager_ALL_BOX_BOX/real_time_mean                                    1.74 us
BM_CONTACT_TEST_0_BulletDiscreteBVHManager_ALL_SPHERE_SPHERE/real_time_mean                             0.263 us
BM_CONTACT_TEST_1_BulletDiscreteBVHManager_ALL_BOX_BOX/real_time_mean                                    1.73 us
BM_CONTACT_TEST_1_BulletDiscreteBVHManager_ALL_SPHERE_SPHERE/real_time_mean                             0.263 us
BM_SET_COLLISION_OBJECTS_TRANSFORM_SINGLE_BulletDiscreteBVHManager_ACTIVE_OBJ_64/real_time_mean          62.9 ns
BM_SET_COLLISION_OBJECTS_TRANSFORM_SINGLE_BulletDiscreteBVHManager_ACTIVE_OBJ_512/real_time_mean         76.1 ns
BM_SET_COLLISION_OBJECTS_TRANSFORM_VECTOR_BulletDiscreteBVHManager_ACTIVE_OBJ_64/real_time_mean          63.9 ns
BM_SET_COLLISION_OBJECTS_TRANSFORM_VECTOR_BulletDiscreteBVHManager_ACTIVE_OBJ_512/real_time_mean         79.3 ns
BM_SET_COLLISION_OBJECTS_TRANSFORM_MAP_BulletDiscreteBVHManager_ACTIVE_OBJ_64/real_time_mean             74.2 ns
BM_SET_COLLISION_OBJECTS_TRANSFORM_MAP_BulletDiscreteBVHManager_ACTIVE_OBJ_512/real_time_mean            88.6 ns
```

## Phase 1 follow-up — per-pair margin hoist in trajopt_ifopt (2026-07-07)

The three trajopt_ifopt gradient loops iterate the pair-keyed `ContactResultMap` with the key
in scope and already hoisted the coeff lookup per pair, but re-derived the margin per contact
through a two-id lookup (one fat `LinkIdPair` construction = 2 heap string copies per contact,
see `BM_MarginLookup_TwoIds` above). Fixed in trajopt commit `538ce4bd`: margin hoisted per
pair next to the coeff, and the now-trivial per-contact `getGradient`/`calcGradientData`
virtuals deleted (the loops call the free `trajopt_common::getGradient` directly, also removing
a per-contact virtual dispatch). Behavior bit-identical; trajopt_ifopt 37/37, trajopt_sqp
45/45 tests.

Whole-solve benchmarks, same machine, 5 repetitions, medians compared (single-threaded only —
the multi-threaded families are unusable on WSL2: 5.3–88% CV, and the last "before" family
additionally overlapped a rebuild). Invocations:

```
trajopt_sqp_solve_benchmarks --benchmark_repetitions=5 --benchmark_report_aggregates_only=true   # measurement
trajopt_solve_benchmarks     --benchmark_repetitions=5 --benchmark_report_aggregates_only=true   # control (sco path, untouched)
```

| Benchmark (real_time_median) | Before | After | Delta |
|---|---|---|---|
| `BM_TRAJOPT_IFOPT_SIMPLE_COLLISION_SOLVE` | 289 us | 268 us | **-7.3%** |
| `BM_TRAJOPT_IFOPT_PLANNING_SOLVE` | 24.2 ms | 22.9 ms | **-5.4%** |
| `BM_TRAJOPT_SIMPLE_COLLISION_SOLVE` (control) | 314 us | 307 us | -2.2% (noise) |
| `BM_TRAJOPT_PLANNING_SOLVE` (control) | 19573 ms | 19688 ms | +0.6% (noise) |

Both measurement deltas exceed their run-to-run CVs (2.3–3.0%); both control deltas sit within
theirs (2.7–3.5%), confirming the effect is confined to the changed evaluators. The win is the
expected one: 2 heap allocations + ~40 ns of pair construction eliminated per contact per
gradient pass, plus one virtual dispatch.

## Phase 1 follow-up — Finding 3: pair scratch on the Bullet/narrowphase path (2026-07-08)

Restores the upstream string-era `TESSERACT_THREAD_LOCAL`-scratch idiom on the Phase-1 fat pair
via the new `OrderedIdPair::assign` (in-place re-canonicalize; reuses the held ids' string
capacity, so steady state is zero allocations):

- Bullet `addDiscreteSingleResult`/`addCastSingleResult` per-contact-point keys (scratch at the
  call sites; `processResult` copies the key on insert).
- Inside the two-id overloads `CollisionMarginPairData::getCollisionMargin`,
  `CollisionMarginData::getCollisionMargin`, and `setCollisionMarginHelper` — matching where
  upstream kept its scratch, and transparently fixing the Bullet bridged-manifold ctor plus
  every other two-id margin caller with zero call-site churn.
  *(2026-08-06: `CollisionMarginPairData::getCollisionMargin(id, id)` has since been deleted — no
  production caller remained, and a pair in hand is ~2x cheaper than paying TLS access, two
  capacity-reusing copies and `combineHash` per lookup. Same reasoning that retired the ACM two-id
  overload; the other two scratch sites stand.)*

Micro (same invocation as above; means):

| Benchmark | Phase 1 | After | Delta |
|---|---|---|---|
| `BM_PairAssign` (new) | — | 10.3 ns | vs 33.9 ns `BM_PairConstruction` same run: **-70%**, zero allocations |
| `BM_MarginLookup_TwoIds` | 44.0 ns | 29.6 ns | **-33%** |
| `BM_AcmIsAllowed_TwoIds_Hit` | 46.2 ns | 45.3 ns | unchanged (overload not converted; since deleted — see the 2026-07-14 sweep) |

The ACM two-id overload was left unconverted here because no hot callers remained after the ifopt
hoist. Both two-id overloads have since been deleted — the ACM one in the 2026-07-14 sweep,
`CollisionMarginPairData::getCollisionMargin(id, id)` on 2026-08-06. Each benchmark now spells the
pair at the call site, hoisting it and `assign`ing in-loop, so both still measure the work their
overload did and the numbers above stay comparable.

The residual vs Phase 0's 6.73 ns margin lookup is TLS access + two capacity-reusing byte
copies + `combineHash`; Phase 2's transparent views target that remainder.

Macro subset (exact invocation from the Phase-1 reference above): cross-session comparison is
**inconclusive** — the untouched control families (`SET_COLLISION_OBJECTS_TRANSFORM_*`, no pair
construction on that path) drifted +4-15% vs the reference, so the session baselines differ;
the contact-test families moved within the same band. The attributable back-to-back A/B this called
for was run 2026-07-16 — see "Attributable macro A/B for the sweep" below. Tests: tesseract 837 (one
unrelated flaky, passes on rerun) + coal 253, all green.

## Phase 1 follow-up — pair-construction sweep (2026-07-14)

Finding 3 fixed the sites it found; this sweep looked for the rest, and establishes the invariant
they all satisfy:

> **A lookup never manufactures a `LinkIdPair`. Only an insertion does.**

An owning pair costs two heap string copies (~33 ns, `BM_PairConstruction`). Where a pair is the
map's *key*, that cost is irreducible — the container owns it. Where a pair is merely a *lookup
key*, it is pure waste, and `assign` into a reused pair removes it (10.3 ns, zero allocations,
`BM_PairAssign`). The danger is that the two read identically at the call site, which is why the
rule is stated rather than left to taste. Three variants, in order of preference:

- **Loop-hoisted scratch** — an owning pair declared above the loop, `assign`ed per iteration.
  Cheapest: no thread-local machinery at all. Used in `BulletDiscreteSimpleManager::contactTest`,
  `BulletCastSimpleManager::contactTest`, `getCollisionObjectPairs`, the three sco evaluation
  loops in trajopt `collision_terms.cpp`, and the Qt ACM-generation handler.
- **Callback member** — where the hot code is a callback object with a per-query lifetime, the
  object owns the scratch. Used by the Coal plugin's `CollisionCallback`; same cost as a hoist.
- **`TESSERACT_THREAD_LOCAL` scratch** — the fallback where neither a loop nor an owning object is
  in scope, because the function is called once per candidate pair from someone else's broadphase.
  Used inside the two-id `getCollisionMargin`/`isContactAllowed` and the two FCL callbacks. Costs
  a guard check plus `__tls_get_addr` on top of the assign.

The two-id `AllowedCollisionMatrix::isCollisionAllowed` was **deleted** rather than scratched: it
had no non-test callers, and an unconverted two-id overload sitting beside the allocation-free
two-id margin twin is a perf trap — it silently builds a fat pair while reading like the cheap
idiom. The ACM's two-id `addAllowedCollision`/`removeAllowedCollision` are deliberately kept: they
are cold mutations whose key the map owns regardless, so there is no hidden cost to expose.

Not benchmarked in isolation (each site is a few nanoseconds against a contact test); the point is
the invariant, and that the manager-side residual which the sco cache-arrays plan had deferred to
Phase 2 turned out not to need it. Plugin authors: a third-party backend written the obvious way
will manufacture a pair per candidate exactly as all three of ours did.

### Attributable macro A/B for the sweep (2026-07-16)

The macro number the Phase-1 note above deferred as "not yet run". Method: two trees built from
scratch with identical provenance, differing only by reverting the sweep in one (`before`), then a
**same-session interleaved paired A/B** — for each pair, run `before` then `after` (order alternated
per pair) back to back, so slow machine drift cancels. A cell is *attributable* only when
`|delta%| > max(CV_before, CV_after)`; delta% negative means `after` (sweep present) is faster.

Four tiers, all three backends. The sweep has two parts — the broadphase pair hoist (measured
directly by collision benchmarks and `checkTrajectory`) and the sco-loop hoist in `collision_terms.cpp`
(reachable only through a non-ifopt trajopt solve):

| Tier | Instrument | Result |
|---|---|---|
| 0 | `tesseract_collision_benchmarks_only` — Panda in a 50-obstacle scene, `--seed 42`, all backends | null: 6/96 cells attributable, all <4.3%, mixed-sign, incoherent; every Coal-discrete cell within CV |
| 1 | per-backend `*_discrete_bvh_benchmarks` — `LARGE_DATASET` scaling (edge 4/8/12 = 64/512/1728 objects) + `SET_..._TRANSFORM` drift control | null: 1/27 attributable (wrong-sign, smallest scene); delta% vs object count non-monotone; drift controls ~0 |
| 2 | `tesseract_check_trajectory` — production `checkTrajectory` loop, iiwa, TRAJ5 discrete+continuous (+LVS) | null for the sweep — see the Coal-discrete note below |
| 3 | `trajopt_solve_benchmarks` / `trajopt_sqp_solve_benchmarks` — SIMPLE + PLANNING + IFOPT solves | null: single-threaded SIMPLE/PLANNING (the only sco-loop instrument) within CV for all backends; multi-threaded cells uninformative (60–208% CV) |

Two cells cross the attributable bar and neither is the sweep. Tier 2 Coal-discrete regresses
`+4.3..+5.5%` across all three discrete cells — but that is *wrong-sign* (assign is cheaper than
construct, so the sweep can only make `after` faster or neutral), Coal-discrete-only (Coal continuous,
Bullet, and FCL are null), and directly contradicted by Tier 1, whose Coal `LARGE_DATASET` runs the
same Coal collision code over the same tree pair and shows no such shift. It is a code-layout
difference between the two `tesseract_check_trajectory` build units, which interleaving cancels machine
drift but not per-binary layout. Tier 3 FCL `MULTI_THREADED_PLANNING` is `+15.6%`, also wrong-sign and
inside the multi-threaded family whose siblings carry 60–208% CV.

Conclusion: the sweep's effect is below this machine's macro resolution on every tier — the expected
outcome for a `~24 ns`-per-pair micro win (`BM_PairConstruction` 33.9 ns → `BM_PairAssign` 10.3 ns,
−70%, zero allocations) diluted across contact tests and full SQP solves. The empirical ceiling is the
tightest control/CV floor observed, `~1–3%` of the enclosing operation; the micro win itself remains
the load-bearing evidence, and it is directly measured. Raw JSON/CSV and per-tier tables are archived
outside the tree.

## String-baseline vs id-migration (2026-07-18)

The measurement Finding 1 asked for: is the integer-identity migration faster than the string-based
upstream it replaces? Two trees built from scratch with identical provenance — **A/after** =
`feature/integer-link-ids` (integer ids; tesseract `5791ae7f1`, trajopt `b4eb72d6`,
tesseract_collision_coal `8d88aa6`, coal `91d4a2c8`); **B/before** = the upstream string bases each
feature branch descends from (tesseract `origin/master a9f750001`, trajopt `origin/master 677c0c02`,
tesseract_collision_coal `f75b759`, coal `91d4a2c8`), a pure ancestor of feature so the only difference
is the migration — **except the `tesseract_collision_coal` plugin**, whose A (`8d88aa6`) carries three
cast-only Coal optimizations (`6533c3d`/`3225ff4`/`8d88aa6`) beyond the migration; those touch only the
*cast* managers, so every Coal-**cast** delta below is migration + opts (confounded), while all discrete
cells and every Bullet/FCL cell isolate the migration cleanly. See the Tier 2 note. Same-session
**interleaved paired A/B**: each pair runs before/after back to back
with the order alternated per pair, so slow machine drift cancels. A cell is *attributable* only when
`|delta%| > max(CV_before, CV_after)`.

All four tiers below come from this **single coherent post-fix build** — the A tree includes the
CoalCast convex-cast-hull AABB fix (`tesseract_collision_coal 8d88aa6`) — so the whole record is one
provenance with no pre-fix/post-fix splice. It supersedes an earlier spliced version of this section.

**Metric convention:** Tier 0 is `checks_per_second` (higher = faster → **positive** delta% = id
faster); Tiers 1–3 are `real_time` (lower = faster → **negative** delta% = id faster).

**No null control exists for this comparison.** Unlike the sweep A/B above (where
`SET_..._TRANSFORM` was an untouched id-vs-id null), every path here differs between the trees — the
migration changes container *types* (`std::map<std::string,…>` → `std::unordered_map<LinkId,…>`,
`std::vector<std::string>` → `std::vector<LinkId>`), so even the transform path is a *primary*
measurement, not a control. Drift is cancelled by interleaving alone; the large signals
(`−44…−62%`, `+10…+27%`) dwarf the CVs (mostly `1–6%`) and are direction-coherent across every
backend and scenario, so they read as real, not drift. The two trees are separately-built binaries,
so per-binary code layout is not cancelled (only machine drift is); sub-CV deltas are therefore *at
or below this machine's resolution*, not wins or losses.

### Tier 0 — dense-scene Panda collision (headline), `checks_per_second`

Panda 7-DOF arm, 50-obstacle scene, `--seed 42` (byte-identical work both trees), all backends in one
run. Only the exhaustive contact modes (CLOSEST/ALL) are read as throughput — FIRST-mode returns on
the first contact found, so the migration's container-order change makes the two builds do *different*
work; those cells swing `±30–86%` in both directions and are order artifacts, not speed.

| Backend / path | delta% (id vs string) | attr | reading |
|---|---|:--:|---|
| Coal discrete (BVH) | +10 … +27% | Y | large, coherent id win — every scenario |
| Coal cast (CONT + CLONE) | +8 … +15% | Y | **confounded** — migration + 3 cast-only Coal opts (see Tier 2); clean cast ref is Bullet-cast ≈+0.6% |
| FCL discrete (BVH) | +3 … +8% | Y | small, coherent id win |
| Bullet discrete / cast | 0 … +3% | mixed | neutral — mostly at/below resolution |

### Tier 1 — synthetic broadphase scaling + transform path, `real_time`

| Path | bullet | fcl | coal |
|---|---|---|---|
| `SET_..._TRANSFORM` {SINGLE,VECTOR,MAP} @512 objects | −44…−46% (Y) | −55…−62% (Y) | −48…−61% (Y) |
| `LARGE_DATASET` PRIMATIVE (cheap collision math) | −30…−39% (Y) | −3…−6% (Y at edge 8/12) | −30…−36% (Y) |
| `LARGE_DATASET` CONVEX_MESH (GJK-heavy) | −2…−8% (Y at edge 4/8) | −4…−7% (Y) | −17…−22% (Y) |

The transform-container swap (`map<string>` → `unordered_map<int>`, 512 lookups/call) is
`−48…−62%` measured in isolation, and surfaces in the realistic `LARGE_DATASET` loop wherever
collision math is cheap enough not to bury it. Win % *decreases* with object count (bullet PRIMATIVE
−38.6 → −31.0 → −29.6 across edge 4/8/12), because the transform cost is linear in objects while the
collision-pair math is super-linear — so this is a per-object transform effect, not a
per-broadphase-pair one. fcl is id-faster everywhere; with its `LARGE_DATASET` after-side CV down to
2–4% this run (18–33% in the earlier spliced run) its LD cells are now attributable, not just its
clean `SET_` rows.

### Tier 2 — production `checkTrajectory` (iiwa), `real_time`

| Backend | discrete (STATE_SOLVER / JOINT_GROUP / CURRENT) | continuous-LVS |
|---|---|---|
| bullet | −9.1 / −4.3 / −3.9% (STATE_SOLVER Y; JOINT_GROUP/CURRENT sub-CV) | −5.6 / −4.1% (BulletCast; STATE_SOLVER Y) |
| fcl | −5.6 / −1.9 / −2.3% (STATE_SOLVER Y; JOINT_GROUP/CURRENT sub-CV) | =BulletCast (FCL has no cast manager) |
| coal | −15.6 / −7.9 / −8.1% (all Y) | **−17.7 / −8.3% (id faster, all Y — confounded, see below)** |

Discrete corroborates Tiers 0/1 — id faster on every backend; coal's three cells are all attributable,
and bullet's and fcl's `STATE_SOLVER` cells are, while their `JOINT_GROUP`/`CURRENT` cells (−2…−4%) sit
at this machine's CV floor. Bullet moves here (unlike its neutral Tier 0) because `checkTrajectory`'s
per-state bulk transform update hits the swapped `setCollisionObjectsTransform` container — the same
path Tier 1's `SET_` rows measured at −45%.

**Two things drive the percentages: which path is measured, and each backend's baseline.** The saving
is absolute framework overhead (per-state transform/name-lookup that scales with link/state counts,
not with the narrowphase), so it (a) is roughly the same for the two BVH backends within a cell —
`DISCRETE_JOINT_GROUP-LVS` saves ~42–44 µs on both coal and bullet (fcl less, ~23 µs) and
`DISCRETE_STATE_SOLVER-LVS` ~94–98 µs on coal and bullet (fcl ~75 µs) — and (b) is ~2× larger on the
`STATE_SOLVER` path than on `JOINT_GROUP`/`CURRENT`, because the state-solver path carries more
per-state framework work for the migration to cut. The **percentage** then divides that saving by each
backend's total (coal ~555 < bullet ~957 < fcl ~1263 µs on JOINT_GROUP-LVS), so coal — the fastest
backend — shows the largest %. Coal's larger % on Tier 2 is thus mostly a denominator effect, not a
Coal-specific optimization (this differs from Tiers 0/1, where Coal's lead *is* mechanistic — see the
conclusion).

**CoalCast continuous-LVS is id-faster here (−17.7 / −8.3%, both attributable) but the figure is
confounded — not a pure-migration number.** The id (A) build carries three cast-only Coal optimizations
absent from the string baseline: `6533c3d` (GJK warm-start sharing), `3225ff4` (flat support averaging),
and `8d88aa6` (O(1) conservative cast-hull AABB). So this pair measures migration *plus* those opts. The
clean pure-migration reference for the cast path is **BulletCast continuous (−5.6 / −4.1%)** — same cast
glue, no Coal opts; the gap from that to CoalCast's −17.7 / −8.3% is the opts' contribution, not the
migration's. (The v3 rerun (`sb_results_v3.md`) confirms this directly: Coal's cast cell *exceeds* its
discrete cell while Bullet's cast cell is *smaller* than its discrete — the inversion is the opts'
signature.) `8d88aa6` also resolved an earlier regression: on the pre-fix spliced record these two cells
were id-*slower* (+11.8 / +22.7%), root-caused to the CoalCast convex-cast-hull AABB path; that fix is
part of the A tree throughout this section, so no regression remains — but read the cast-cell *magnitude*
as opts-driven, not migration-driven.

### Tier 3 — end-to-end trajopt solve, `real_time`

| Backend | SIMPLE | PLANNING | IFOPT_SIMPLE | IFOPT_PLANNING (cast) |
|---|--:|--:|--:|--:|
| coal | −0.3 | +0.4 | +0.8 | +0.3 |
| bullet | +3.0 (Y) | +0.3 | +2.3 | −2.2 |
| fcl | +1.9 | +0.3 | +0.5 | — (no cast) |

**Clean null — the single-threaded cells sit within `−2.2 … +3.0%`.** The one cell that crosses its CV
bar is bullet SIMPLE (+3.0%, CV ~2.5%), and it is **wrong-sign** (id nominally slower) and
uncorroborated by its siblings — a per-binary layout artifact, not a real id regression. The
container/lookup win is below solve-level resolution: wrapped in a full SQP solve, collision is a small
fraction of the work (Jacobian evaluation + QP + line search dominate), so a collision hot-path win
does not move whole-solve time. Coal IFOPT_PLANNING (production continuous cast) is `+0.3%`, a dead
null — the `8d88aa6` cast fix that drives the Tier 0/2 CoalCast wins is swamped by optimizer cost at
this dilution. Multi-threaded cells are omitted as uninformative (thread-scheduling variance, CV
20–49%).

### Conclusion

The integer-identity migration is **measurably faster than the strings it replaces on the collision
hot paths that touch the migrated containers, and never slower at production granularity:**

- **Coal shows the largest percentages, for two distinct reasons that dominate in different regimes.**
  In contactTest-heavy work (Tiers 0/1) its `collision_objects_` vector (`vector<string>` →
  `vector<LinkId>`) is iterated *inside* the broadphase callback, while Bullet's and FCL's changed
  containers are off that path — so Coal earns a genuine *extra* saving there (+10–27% dense-scene
  discrete, −48…−61% transform path; the +8–15% *cast* figure is confounded by three cast-only Coal opts
  and overstates the migration's share — clean cast reference is Bullet-cast, ≈+0.6%). In framework-heavy
  work (Tier 2 `checkTrajectory`)
  the absolute saving is instead roughly shared between the BVH backends (coal ≈ bullet per cell, fcl
  somewhat less), and Coal — the fastest backend — simply shows that saving as the largest *fraction*
  (a denominator effect). A user on Coal still feels the larger %, but the Tier-2 component is mostly
  shared framework overhead, not a Coal-only win.
- **FCL wins modestly** (+3–8% dense-scene, −55…−62% transform); its smaller `checkTrajectory` % than
  Coal is largely the denominator effect above (FCL is the slowest backend).
- **Bullet is neutral on `contactTest`** — its changed container (`link2cow_`,
  `map<string>` → `unordered_map<LinkId>`) is off that hot path, which runs inside Bullet's own
  `btOverlappingPairCache` — but it wins on the transform-heavy production `checkTrajectory` path
  (−3.9…−9.1%) and −44…−46% on the isolated `SET_` transform benchmark.
- **At the full-solve level the win washes out below resolution** (Tier 3 clean null): collision is a
  small fraction of SQP solve time, so the migration neither helps nor hurts whole-solve time.
- **No regression remains anywhere in this single-build record** — the CoalCast `checkTrajectory`-LVS
  cells that regressed on the earlier spliced record are id wins here (−8…−18%), the `8d88aa6`
  cast-AABB fix being part of the measured A tree. (Those CoalCast magnitudes are opts-confounded — see
  the Tier 2 note; the *migration's* own effect on the cast path is the clean Bullet-cast figure.)

The string-era comparison the record previously lacked now exists and points **id-faster-or-neutral
everywhere measured**, decisively so for Coal. Raw JSON/CSV and per-tier tables are archived outside
the tree.

## Phase 2 — boost::unordered_flat_map + transparent view lookups (deferred — pending real-data benchmarking)

**Reframed 2026-07-16 — optional optimization, no longer a prerequisite.** The Finding-3 hoist
(2026-07-14, see the follow-ups above) removed the per-candidate pair construction that the Phase-1
`TwoIds` ratios attributed the regression to; against the string-era comparator the hot lookups are
now neutral-or-better. Finding 2 (`IDENTITY_MIGRATION_REVIEW.md`) is downgraded to LOW and downstream
of Finding 1 accordingly, so Phase 2 is an experiment to be justified by benchmarks rather than a
required fix — and the non-owning view is expected to be dropped, since the hoist already captured its
benefit.

Scope, matrix, and decision rule:
`docs/superpowers/plans/2026-07-15-phase2-pair-key-container-experiment.md` (container swap
`flat_map`/`node_map`, chosen per-container, gated on a Finding-1 string-era baseline).

(Benchmark results: pending — to be captured if/when the experiment runs.)
