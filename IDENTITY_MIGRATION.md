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

## The governing rule

Every method kept the shape it had before — same parameters, same return type, same overload
set — just id-based instead of string-based. Where a string form and an id form existed in
parallel, the string form was removed; the id form was already there and is now the only one.
Where **only** a string form existed, it was converted to take or return ids. There is no
deprecation period for either case: a name-typed call site becomes a compile error, not a
silent behavior change, because `std::vector<std::string>` does not implicitly convert to
`std::vector<LinkId>` or `std::vector<JointId>`.

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
- **`AllowedCollisionMatrix::isCollisionAllowed(LinkId, LinkId)` is removed.** The pair form is the
  only spelling: `acm.isCollisionAllowed({ id1, id2 })`. The two-id convenience constructed a fat
  `LinkIdPair` — two heap string copies — on every call, while reading like the allocation-free
  two-id margin lookups it sat next to; spelling the pair at the call site makes that cost visible.
- **`SceneState`'s four maps are id-keyed**: `joints`
  (`std::unordered_map<JointId, double>`), `link_transforms` (`LinkIdTransformMap`),
  `joint_transforms` and `floating_joints` (`JointIdTransformMap`).
- **`StateSolver` has no string API left.** `setState`, `getState`, `getJacobian`, and every
  getter take and return ids (`getJointIds`, `getLinkIds`, `getBaseLinkId`, …).
- **`Joint` link references are ids**: `child_link_name` / `parent_link_name` →
  `child_link_id` / `parent_link_id` (`LinkId`); `JointMimic::joint_name` → `joint_id`
  (`JointId`). Cereal keys are unchanged, so old archives load.
- **`JointGroup` and `JointState` are id-only.** `JointGroup::getJointNames()` and
  `JointGroup::getLinkNames()` are gone; `JointState::getJointNames()` is gone (`JointState` never
  had a link-name getter). `getJointIds()` / `getLinkIds()` are the sole accessors. Convert with
  `toNames` at a boundary that genuinely needs strings.
- **Collection-returning APIs return ids**, not name vectors:
  `DiscreteContactManager::getCollisionObjects()` → `std::vector<LinkId>`,
  `JointGroup::calcFwdKin()` → `LinkIdTransformMap`,
  `KinematicGroup::getAllPossibleTipLinkIds()`, `JointGroup::getJointIds()` / `getLinkIds()`,
  and the like. Convert at your boundary with `toNames` where a string list is needed.
- **`checkTrajectory` is id-only.** The `const std::vector<std::string>& joint_names` overloads
  (both the discrete and the continuous contact-manager forms) are gone. What remains: the
  `const std::vector<JointId>&` overload and the `const JointGroup&` overload, for each manager
  type.
- **`getAllowedCollisions` is id-only.** The `std::vector<std::string>` overload is gone; call the
  `std::vector<LinkId>` overload instead.
- **`tesseract::common::JointState` has no joint-name constructor.** Construct from
  `std::vector<JointId>` plus a position vector. Brace-initializing that vector from string
  literals still compiles in any translation unit that does not define
  `TESSERACT_NAMEID_NO_IMPLICIT`.

## The planning surface changes (`tesseract_planning`)

- **`JointWaypoint` and `StateWaypoint` no longer construct from joint names.** The
  `std::vector<std::string>` constructors are gone; construct from `std::vector<JointId>`. The
  `std::initializer_list` constructors survive, retyped to `JointId`, so the brace-literal
  convenience still works where the implicit conversion is available:
  `JointWaypoint({ "j1", "j2" }, { 1.0, 2.0 })`. What no longer compiles is the case that mattered —
  handing a waypoint a `std::vector<std::string>` you are already holding. That is a compile error,
  not a silent per-joint hash.

  One honest caveat: the implicit conversion is gated **per translation unit**
  (`TESSERACT_NAMEID_NO_IMPLICIT`), not per argument, so in a TU where it is available a braced list
  can still mix in a `std::string` variable — `{ name_var, "j2" }` — and it will convert. C++ overload
  resolution cannot distinguish a literal from a variable of the same type. Tesseract's own libraries
  define the macro and so are held to the explicit spelling; downstream code that wants the same
  guarantee should define it too.
- **The waypoint polys (joint and state) expose `setJointIds`/`getJointIds` only.** Both
  `setNames` and `getNames` are gone from every layer of the type-erasure stack and from the
  installed `test_suite/*.hpp` conformance headers. A caller that needs names for display or
  logging converts at its own boundary: `tesseract::common::toNames(wp.getJointIds())` — this is
  what the Qt tree items now do.
- **The `command_language` free functions are id-only.** `getJointPosition`,
  `formatJointPosition`, and `checkJointPositionFormat` no longer have string delegates; call the
  id overload and convert at your own boundary if you only have names.

## The ROS surface changes (`tesseract_rosutils`)

`tesseract_rosutils` now speaks ids on both sides of the ROS message boundary:

- **`fromMsg` gained id-keyed `SceneState::JointValues` twins** — one from
  `sensor_msgs::msg::JointState`, one from a `std::vector<tesseract_msgs::msg::StringDoublePair>`.
- **`toEigen(const sensor_msgs::msg::JointState&, …)` takes `const std::vector<JointId>&`** and
  orders its output by that id list — same contract as before, id-spelled.
- **Deleted:** `toIdJointValues`, both string-keyed `toMsg` forms, and both string-keyed `fromMsg`
  forms.

The removal most likely to affect a downstream ROS consumer is
`bool fromMsg(std::unordered_map<std::string, double>&, const sensor_msgs::msg::JointState&)`.
Take the id-keyed twin, `fromMsg(SceneState::JointValues&, const sensor_msgs::msg::JointState&)`,
and read `id.name()` off its keys if you still need names.

The general path for any ROS message field that used to hand you a name is the same one that
applies everywhere else in this guide: convert at your own boundary with `toIds<JointId>` (or
take the id-keyed message conversion directly, where one exists) and hold onto the ids from
there.

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
- **Repeated pair-keyed lookups: `assign` into a reused scratch pair.** A `LinkIdPair` owns two
  names, so constructing one costs two heap string copies. `OrderedIdPair` has an in-place
  `assign(a, b)` that re-canonicalizes while reusing the held ids' string capacity, so
  steady-state reassignment performs no allocation. The rule the core now follows, and which
  a collision or kinematics backend should follow too:

  > A lookup never manufactures a `LinkIdPair`. Only an insertion does.

  Where a pair is a container's key, its cost is irreducible. Where it is only a lookup key, reuse
  one: declare it above the loop and `assign` per iteration; or, if the hot code is a callback
  object invoked per candidate pair, hold the scratch as a member. Fall back to a
  `TESSERACT_THREAD_LOCAL` scratch only when neither a loop nor an owning object is in scope —
  it works, but adds a guard check and `__tls_get_addr` per access. (This replaces the string
  era's `makeOrderedLinkPair` out-param pattern.) A backend written the obvious way manufactures
  a pair per candidate pair; all three in-tree backends did before they were swept.
- **Planner mid-layer: hoist the id vector out of the kinematics group once.** `JointGroup`
  returns ids by const reference, so binding them costs nothing:

      const std::vector<JointId>& joint_ids = manip->getJointIds();
      assignSolution(mi, joint_ids, values, format_as_input);

  The planning mid-layer in `tesseract_planning` — `assignSolution`, `assignTrajectory`,
  `getInterpolatedInstructions` — takes ids only; there is no string overload to fall back on.
  Passing a name vector is a compile error rather than a silent per-joint allocation and re-hash
  inside the callee. If names are all you hold, convert once at your own boundary with
  `toIds<JointId>` and keep the ids.

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
