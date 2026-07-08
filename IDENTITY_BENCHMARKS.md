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

Micro (same invocation as above; means):

| Benchmark | Phase 1 | After | Delta |
|---|---|---|---|
| `BM_PairAssign` (new) | — | 10.3 ns | vs 33.9 ns `BM_PairConstruction` same run: **-70%**, zero allocations |
| `BM_MarginLookup_TwoIds` | 44.0 ns | 29.6 ns | **-33%** |
| `BM_AcmIsAllowed_TwoIds_Hit` | 46.2 ns | 45.3 ns | unchanged (ACM two-id overload deliberately not converted; no hot callers remain after the ifopt hoist) |

The residual vs Phase 0's 6.73 ns margin lookup is TLS access + two capacity-reusing byte
copies + `combineHash`; Phase 2's transparent views target that remainder.

Macro subset (exact invocation from the Phase-1 reference above): cross-session comparison is
**inconclusive** — the untouched control families (`SET_COLLISION_OBJECTS_TRANSFORM_*`, no pair
construction on that path) drifted +4-15% vs the reference, so the session baselines differ;
the contact-test families moved within the same band. An attributable macro number needs a
back-to-back stash A/B in one session (not yet run). Tests: tesseract 837 (one unrelated flaky,
passes on rerun) + coal 253, all green.

## Phase 2 — boost::unordered_flat_map + transparent view lookups (deferred — pending real-data benchmarking)

(pending)
