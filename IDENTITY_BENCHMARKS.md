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

(pending)

## Phase 2 — boost::unordered_flat_map + transparent view lookups (after Task 11)

(pending)
