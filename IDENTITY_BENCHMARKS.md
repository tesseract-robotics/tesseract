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

## Phase 2 — boost::unordered_flat_map + transparent view lookups (deferred — pending real-data benchmarking)

(pending)
