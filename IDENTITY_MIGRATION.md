# Identity Migration Guide

This guide covers migrating downstream code from string link/joint names to the integer identity
types `LinkId` / `JointId`, which are now the primary currency of the Tesseract API. For the
design rationale, the collision-resolution model, and the performance model, see
`IDENTITY_DESIGN.md`; measured costs are in `IDENTITY_BENCHMARKS.md`. The single-package CMake
restructure (include paths, targets, `find_package`) is covered separately in `MIGRATION.md`.

## What stays compatible

- **On-disk formats are unchanged.** URDF and SRDF remain string-based on disk. Cereal archives
  written by pre-migration code load without conversion (ACM and margin data round-trip through
  string-pair maps), and archives written by the new code remain readable by pre-migration
  tooling. YAML encodes pair keys as name sequences and re-hashes on load.
- **String literals still compile at most call sites.** The id constructors are implicit for
  downstream code, so `env->getLink("base_link")` and `map.find("tool0")` build unchanged. Note
  the performance caveat under Migration idioms below.
- **`Environment` keeps its string conveniences**: `setState`/`getState` accept
  `std::vector<std::string>` and `std::unordered_map<std::string, double>`, group APIs stay
  keyed by group-name strings, and most name getters remain alongside the id getters.
  `JointGroup` keeps `getJointNames()` / `getLinkNames()`.

## The core surface changes

- **`ContactResult::link_names` → `link_ids`** — renamed *and* retyped to
  `std::array<LinkId, 2>` (`tesseract/collision/types.h`). Read the string as
  `link_ids[0].name()`; `shape_id` / `subshape_id` are unchanged.
- **`ContactResultMap` is keyed by `LinkIdPair`** instead of a string pair. Key lookups build a
  `LinkIdPair` from two `LinkId`s; the names are recoverable from the key
  (`key.first().name()`).
- **`IsContactAllowedFn` is removed** — there is no `std::function` shim. Subclass
  `tesseract::common::ContactAllowedValidator` and implement
  `bool operator()(const LinkIdPair&) const` (`tesseract/common/contact_allowed_validator.h`);
  ACM- and combining-validators are provided.
- **`SceneState`'s four maps are id-keyed**: `joints`
  (`std::unordered_map<JointId, double>`), `link_transforms` (`LinkIdTransformMap`),
  `joint_transforms` and `floating_joints` (`JointIdTransformMap`).
- **`StateSolver` has no string API left.** `setState`, `getState`, `getJacobian`, and every
  getter take and return ids (`getJointIds`, `getLinkIds`, `getBaseLinkId`, …).
- **`Joint` link references are ids**: `child_link_name` / `parent_link_name` →
  `child_link_id` / `parent_link_id` (`LinkId`); `JointMimic::joint_name` → `joint_id`
  (`JointId`). Cereal keys are unchanged, so old archives load.
- **Collection-returning APIs return ids**, not name vectors:
  `DiscreteContactManager::getCollisionObjects()` → `std::vector<LinkId>`,
  `JointGroup::calcFwdKin()` → `LinkIdTransformMap`,
  `KinematicGroup::getAllPossibleTipLinkIds()`, `JointGroup::getJointIds()` / `getLinkIds()`,
  and the like. Convert at your boundary with `toNames` where a string list is needed.

## Plugin authors

The contact-manager pure-virtual interface is retyped end-to-end: `addCollisionObject`,
`getCollisionObjects`, `setCollisionObjectsTransform`, `setActiveCollisionObjects`, and the
validator hook all speak ids (`tesseract/collision/discrete_contact_manager.h`,
`continuous_contact_manager.h`). A third-party collision or kinematics backend must migrate its
whole override surface at once — for scale, migrating the Coal collision backend touched roughly
1,400 added and 900 removed lines.

## Migration idioms

- **Bridge with `toIds` / `toNames`** (`tesseract/common/types.h`). `toIds<LinkId>(names)`
  converts a name vector at the boundary into Tesseract; `toNames(ids)` converts any id
  container back for display, logging, or string-based neighbors.
- **Construct ids once and reuse them.** Implicit conversion means an unmigrated call site like
  `map.find("base_link")` compiles unchanged — but it pays the string-era cost (a string
  construction plus hash per call). The migration's win is realized only when a `NameId` is
  constructed once and reused across many lookups. Hoist ids out of loops and store them in
  members instead of re-converting names per call.
- **Repeated pair-keyed lookups: `assign` into a thread-local scratch pair.** `OrderedIdPair`
  has an in-place `assign(a, b)` that re-canonicalizes while reusing the held ids' string
  capacity, so steady-state reassignment performs no allocation. Paired with a
  `TESSERACT_THREAD_LOCAL` scratch pair it is the sanctioned idiom for pair-keyed lookups in hot
  loops (it replaces the string era's `makeOrderedLinkPair` out-param pattern).

## Iteration order is unspecified (and changed vs. the string era)

`NameId::operator<` orders by the cached hash, not by name, and `std::hash<std::string>` is
implementation-defined — so ordered containers of ids (`std::map<LinkId, …>`, `std::set<LinkId>`,
sorted vectors) iterate in an order that is not alphabetical and differs across standard
libraries (libstdc++ / libc++ / MSVC). Consequences:

- Results that were alphabetically ordered in the string era no longer are, and their order is
  now **unspecified**: `SceneGraph::getJointChildrenIds` (the vector overload),
  `KinematicGroup::getAllValidWorkingFrames()`, and `Environment::getStaticLinkIds(joint_ids)`.
  Sort by `.name()` yourself where display order matters.
- The same applies to any id-keyed ordered container you write: do not rely on its iteration
  order across platforms, and never persist an order derived from it (hash values are
  runtime-only — see `IDENTITY_DESIGN.md`, Serialization).

## Entry structs and pair helpers

`ACMEntry` and `PairMarginEntry` (and, in the `trajopt` repo, `PairCoeffEntry`) lost their
`name1`/`name2` fields — the `LinkIdPair` key is now the single source of names, so callers read
`key.first().name()` / `key.second().name()` instead. `getAlphabeticalACMEntries` now returns
the new `srdf::AlphabeticalACMEntry` type rather than the old entry struct.
`checkHashCollision`, `checkPairHashCollision`, and `orderedPairNames` have been deleted, along
with the raw-value constructor of `OrderedIdPair` (construct from two `NameId`s instead).
Colliding names no longer throw at insertion time: they resolve via hybrid equality and coexist
as distinct keys (see `IDENTITY_DESIGN.md`).

In the `trajopt` repo, the per-contact `DiscreteCollisionEvaluator::getGradient` and
`ContinuousCollisionEvaluator::calcGradientData` virtuals were removed — the collision margin is
now resolved once per pair in `calcCollisions`/`calcCollisionData`; call the free
`trajopt_common::getGradient` overloads directly instead. The uncalled 4-argument two-timestep
`CollisionEvaluator::GetGradient(dofvals0, dofvals1, contact_result, isTimestep1)` convenience
wrapper was also removed; use the 6-argument overload taking `margin` and `coeff` explicitly.
