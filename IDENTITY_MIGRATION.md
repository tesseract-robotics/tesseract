# Identity Migration Guide

This guide covers migrating downstream code from string link/joint names to the integer identity
types `LinkId` / `JointId`, which are now the primary currency of the Tesseract API. For the
design rationale, the collision-resolution model, and the performance model, see
`IDENTITY_DESIGN.md`. The single-package CMake restructure (include paths, targets,
`find_package`) is covered separately in `MIGRATION.md`.

It lives in `tesseract` but covers four repositories, since the retyping crosses all of them:
`tesseract`, `trajopt`, `tesseract_planning`, and `tesseract_ros2`. Each has its own section below.

This guide is temporary. It describes one transition and will be removed once downstream has
migrated; `IDENTITY_DESIGN.md` is the document meant to outlive it.

## What stays compatible

- **On-disk formats are unchanged.** URDF and SRDF remain string-based on disk. Cereal archives
  written by pre-migration code load without conversion, and archives written by the new code
  remain readable by pre-migration tooling: an id serializes as its bare name, and a `LinkIdPair`
  key serializes as its two names under the same NVPs cereal gave the old
  `std::pair<std::string, std::string>` key. YAML encodes pair keys as name sequences the same
  way. Both re-hash on load. In every case the two names are written **alphabetically**, not in
  the pair's canonical (hash-value) order — see *Reading names off a pair*, under
  Migration idioms.
- **A name still works wherever it was a single argument.** The id constructors are implicit for
  downstream code, so `env->getLink("base_link")` and `map.find("tool0")` build unchanged, and so
  does a `std::string` variable passed to a parameter that is now a `LinkId`, or assigned into a
  struct member that is now one. See *The governing rule* for the shapes that do **not** survive,
  and Migration idioms for the performance caveat.
- **Group APIs stay keyed by group-name strings.** `Environment::getJointGroup`,
  `getKinematicGroup`, and `getGroupNames` still take and return group-name strings — a group
  name is a user-facing label, not a link or joint identity. Solver names
  (`getSolverName`) are unchanged for the same reason.
- **`Link` and `Joint` keep `getName()`**, alongside a new `getId()`. Their constructors and
  `clone()` take ids, which a name still converts to. The `environment/commands/*` constructors
  are the same story — `RemoveLinkCommand("link")` compiles as it did.

## What the compiler won't catch

The port is compiler-driven, with three exceptions. These are the only places this guide asks you
to go looking rather than wait to be told.

- **Iteration order changed, silently.** Ordered containers of ids sort by hash, not
  alphabetically. Nothing breaks, builds, or warns — the order is simply different, and it is not
  stable across builds. See *Iteration order is unspecified* for the specific APIs affected.
- **Unmigrated call sites keep compiling, and keep paying.** Because the string constructors are
  implicit, `map.find("base_link")` builds fine and still costs a string construction plus a hash
  per call. It is entirely possible to finish the port, have a green build, and get none of the
  speedup. The fix is a second pass — see *Finding the sites the compiler let through*.
- **A `ContactAllowedValidator` subclass stops overriding without saying so.** Covered under
  The core surface changes, where the failure mode and the fix are spelled out.

## The governing rule

Every method kept the shape it had before — same parameters, same return type, same overload
set — just id-based instead of string-based. Where a string form and an id form existed in
parallel, the string form was removed; the id form was already there and is now the only one.
Where **only** a string form existed, it was converted to take or return ids. There is no
deprecation period for either case.

Because the id constructors are implicit for downstream code, that retyping is not uniformly a
break. What breaks is narrower than what changed:

| Change shape | Still compiles? |
|---|---|
| Parameter `const std::string&` → `const LinkId&` | **yes** |
| Struct member `std::string` → `LinkId`, written from a name | **yes** |
| Renamed symbol — `getAdjacentLinkNames` → `getAdjacentLinkIds` | no |
| Container — `std::vector<std::string>` → `std::vector<JointId>` | no |
| Return type — an id, or an id container, read into a string one | no |
| Variant alternative — `std::get<std::string>(info.tcp_offset)` | no |
| Pure virtual signature — your override no longer overrides | no *(class stays abstract)* |
| Overload removed where only the string form existed | no |

The container and return-type rows share one cause: `std::vector<std::string>` does not implicitly
convert to `std::vector<LinkId>` or `std::vector<JointId>`, in either direction. So a name-typed
*container* crossing a boundary is always a compile error rather than a silent behavior change,
while a name-typed *scalar* crosses unremarked.

That makes the compiler the porting engine, and the error you hit is the natural way into the rest
of this document. The wordings below are paraphrases — GCC, Clang and MSVC each phrase these
differently, so match on the symbol rather than on the sentence.

| What the compiler says | What it means | Fix |
|---|---|---|
| no member named `link_names` in `ContactResult` | renamed *and* retyped | `link_ids[i]`; `.name()` for the string |
| no viable conversion from `vector<string>` to `vector<JointId>` | a name-typed container crossed the boundary | `toIds<JointId>(names)` once, then keep the ids |
| no matching function for `checkTrajectory(..., vector<string>&, ...)` | string overload removed | pass `manip->getJointIds()` |
| your `ContactAllowedValidator` subclass does not override / is abstract | `operator()` takes a pair | `bool operator()(const LinkIdPair&) const override` |
| no member named `getJointNames` in `JointGroup` | id-only accessor | `getJointIds()`; `toNames()` at a display boundary |
| no member named `getAdjacentLinkNames` in `SceneGraph` | renamed | `getAdjacentLinkIds()`; see Renamed symbols |
| `LinkNamesPair` / `makeOrderedLinkPair` not declared | the pair type canonicalizes itself | `LinkIdPair(a, b)`; `pair.assign(a, b)` to reuse a scratch pair |
| no matching function for `isCollisionAllowed(id1, id2)` | pair-keyed readers take a pair | `isCollisionAllowed({ id1, id2 })` |
| no member named `getBaseLinkName` *(kinematics plugin)* | renamed pure virtual | `getBaseLinkId()`; see Plugin authors |
| `std::get<std::string>` — no such variant alternative | `ManipulatorInfo::tcp_offset` holds a `LinkId` | `std::get<LinkId>(info.tcp_offset)` |
| cannot instantiate abstract class *(your contact manager)* | the pure virtual is the set form | override `setActiveCollisionObjects(const std::unordered_set<LinkId>&)` |
| … still abstract, no `getCollisionObjectsTransform` | new pure virtual, unrelated to ids | implement it; throw `std::out_of_range` for an unknown id |

## Renamed symbols

Renames are the one break with nothing to deduce — the fix is the new spelling. This index lists
every one across the four repositories; where the rename carries more than a new name, the last
column points at the section that says so.

| Old | New | See |
|---|---|---|
| `ContactResult::link_names` | `link_ids` | Core surface |
| `Joint::child_link_name` / `parent_link_name` | `child_link_id` / `parent_link_id` | Core surface |
| `JointMimic::joint_name` | `joint_id` | Core surface |
| `SceneGraph::getAdjacentLinkNames` | `getAdjacentLinkIds` | |
| `SceneGraph::getInvAdjacentLinkNames` | `getInvAdjacentLinkIds` | |
| `SceneGraph::getLinkChildrenNames` | `getLinkChildrenIds` | |
| `SceneGraph::getJointChildrenNames` | `getJointChildrenIds` *(both overloads)* | Iteration order |
| `JointGroup::getJointNames` / `getLinkNames` | `getJointIds` / `getLinkIds` | Core surface |
| `JointState::getJointNames` | removed; read the `joint_ids` member | Core surface |
| `Environment` / `StateSolver` `get*Names` | `get*Ids` | Core surface |
| `KinematicGroup::getAllPossibleTipLinkNames` | `getAllPossibleTipLinkIds` | |
| `KinGroupIKInput::tip_link_name` | `tip_link_id` | |
| `ForwardKinematics` / `InverseKinematics` `getBaseLinkName` | `getBaseLinkId` | Plugin authors |
| … `getJointNames` / `getTipLinkNames` | `getJointIds` / `getTipLinkIds` | Plugin authors |
| `LinkNamesPair` | `LinkIdPair` | Core surface |
| `makeOrderedLinkPair(a, b)` | `LinkIdPair(a, b)`, or `pair.assign(a, b)` | Migration idioms |
| `TransformMap` | `LinkIdTransformMap` | |
| `TermInfo::link` *(trajopt)* | `link_id` | trajopt |
| `*_active_link_names_` *(trajopt evaluators)* | `*_active_link_ids_` | trajopt |
| `setNames` / `getNames` *(waypoint polys)* | `setJointIds` / `getJointIds` | tesseract_planning |
| `toIdJointValues` *(rosutils)* | removed; take the id-keyed `fromMsg` | tesseract_ros2 |

## Migration idioms

- **Bridge with `toIds` / `toNames`** (`tesseract/common/types.h`). `toIds<LinkId>(names)`
  converts a name vector at the boundary into Tesseract; `toNames(ids)` converts any id
  container back for display, logging, or string-based neighbors.
- **Construct ids once and reuse them.** Hoist them out of loops and store them in members; the win
  is realized only across repeated lookups on the same id.
- **Repeated pair-keyed lookups: `assign` into a reused scratch pair.** `assign(a, b)`
  re-canonicalizes in place and reuses the held ids' string capacity, so steady-state reassignment
  allocates nothing. The rule the core follows, and which a collision or kinematics backend should
  follow too:

  > A lookup never manufactures a `LinkIdPair`. Only an insertion does.

      // before — a fresh pair, and so two heap string copies, per candidate
      for (const auto& [id1, id2] : candidates)
        if (acm.isCollisionAllowed({ id1, id2 }))
          continue;

      // after — one pair, re-canonicalized in place, no allocation in steady state
      LinkIdPair scratch;
      for (const auto& [id1, id2] : candidates)
      {
        scratch.assign(id1, id2);
        if (acm.isCollisionAllowed(scratch))
          continue;
      }

  Hold the scratch as a member instead if the hot code is a callback invoked per candidate pair.
  This replaces the string era's `makeOrderedLinkPair` out-param pattern. Where a pair is a
  container's *key*, build it directly — see `IDENTITY_DESIGN.md`, §"Using ids".
- **Reading names off a pair: use `orderedNameView()`.** `OrderedIdPair::first()` and `second()`
  are ordered by hash value, so which name is which is not reproducible across builds.
  `orderedNameView()` returns the two names **alphabetically**:

      const auto [name1, name2] = key.orderedNameView();

  Use it anywhere the order is observable outside the process — writing a file, logging, a UI
  row, a message field, a test assertion on output. `first()` / `second()` are for when you only
  need the two ids and their order is irrelevant. For the lifetime and invalidation rules, see
  `IDENTITY_DESIGN.md`, Serialization.
- **Planner mid-layer: hoist the id vector out of the kinematics group once.** `JointGroup`
  returns ids by const reference, so binding them costs nothing:

      const std::vector<JointId>& joint_ids = manip->getJointIds();
      assignSolution(mi, joint_ids, values, format_as_input);

  The planning mid-layer in `tesseract_planning` — `assignSolution`, `assignTrajectory`,
  `getInterpolatedInstructions` — takes ids only; there is no string overload to fall back on.
  If names are all you hold, convert once at your own boundary with `toIds<JointId>` and keep
  the ids.

### Finding the sites the compiler let through

Once the build is green, the remaining work is the call sites that still hand a name to an
id-taking API — they compile, and they still pay a string construction plus a hash per call.
Rather than hunt for them, turn them into compile errors:

    target_compile_definitions(my_lib PRIVATE TESSERACT_NAMEID_NO_IMPLICIT)

This makes the id constructors `explicit` in that target's own translation units, so every
remaining implicit string→id conversion becomes an error, and the compiler hands you the worklist.
It is per translation unit and affects overload resolution only, so you can adopt it one target at
a time, and consumers of your library are unaffected. Tesseract's own production libraries are
built this way. See `IDENTITY_DESIGN.md`, §"Type safety, and implicit string conversion", for the full semantics
and the one thing the toggle does not catch.

## The core surface changes

- **`ContactResult::link_names` → `link_ids`** — renamed *and* retyped to
  `std::array<LinkId, 2>` (`tesseract/collision/types.h`). Read the string as
  `link_ids[0].name()`; `shape_id` / `subshape_id` are unchanged.
- **`ContactResultMap` is keyed by `LinkIdPair`** instead of a string pair. Key lookups build a
  `LinkIdPair` from two `LinkId`s; the names are recoverable from the key — read them with
  `key.orderedNameView()`, not `key.first()` / `key.second()`, wherever the order is visible
  to anyone.
- **`ContactAllowedValidator::operator()` takes a `LinkIdPair`** instead of two names
  (`tesseract/common/contact_allowed_validator.h`). The class itself is unchanged, so an existing
  subclass keeps compiling as a *new* overload and quietly stops overriding anything — the failure
  surfaces as the class being abstract, or, if the member carries `override`, as an error naming it.
  Retype the signature **and the members it reads** — a `std::unordered_set<std::string>` left in
  place would hash a fresh string per candidate pair:

      // before
      bool operator()(const std::string& link1, const std::string& link2) const override
      { return ignored_.count(link1) != 0 || ignored_.count(link2) != 0; }
      std::unordered_set<std::string> ignored_;

      // after
      bool operator()(const LinkIdPair& pair) const override
      { return ignored_.count(pair.first()) != 0 || ignored_.count(pair.second()) != 0; }
      std::unordered_set<LinkId> ignored_;

  `first()` / `second()` are correct here because the order is not observable. ACM- and
  combining-validators are provided.
- **Pair-keyed lookups take a pair; pair-keyed writes still take two ids.** The two-argument
  *readers* are gone across all three pair-keyed containers — `AllowedCollisionMatrix::isCollisionAllowed`,
  `CollisionMarginPairData::getCollisionMargin`, and (in `trajopt`) `CollisionCoeffData::getCollisionCoeff`
  each take a `LinkIdPair` and nothing else. Spell the pair at the call site:
  `acm.isCollisionAllowed({ id1, id2 })`. Building one costs two heap string copies, so resist
  wrapping it back up in a two-argument helper of your own. The *writers* keep the two-id spelling —
  `addAllowedCollision`, `removeAllowedCollision`, `setCollisionMargin`, `setCollisionCoeff` —
  because an insertion has to build the key regardless (the ACM writers also gained pair overloads
  for callers that already hold one). The one surviving two-id reader is
  `CollisionMarginData::getCollisionMargin(id1, id2)`.

  `{ id1, id2 }` is right at a cold call site; inside a loop, hoist a pair and `assign` per
  iteration — see Migration idioms.
- **`SceneGraph` is id-addressed throughout** — every link- and joint-addressing method, from
  `getLink` and `removeJoint` to `getShortestPath`. Its four `get*Names` traversal helpers were
  renamed as well as retyped (see Renamed symbols), and `ShortestPath`'s three members are now id
  vectors, so binding one to a `std::vector<std::string>` is a compile error.
- **`SceneState`'s four maps are id-keyed**: `joints`
  (`std::unordered_map<JointId, double>`), `link_transforms` (`LinkIdTransformMap`),
  `joint_transforms` and `floating_joints` (`JointIdTransformMap`).
- **`StateSolver` has no string API left.** `setState`, `getState`, `getJacobian`, and every
  getter take and return ids (`getJointIds`, `getLinkIds`, `getBaseLinkId`, …).
- **`Environment` has no string identity API left.** `setState`/`getState` take
  `std::vector<JointId>` / `SceneState::JointValues`; the identity getters return ids
  (`getJointIds`, `getActiveJointIds`, `getLinkIds`, `getActiveLinkIds`, `getStaticLinkIds`,
  `getRootLinkId`, `getGroupJointIds`, `getCurrentJointValues(vector<JointId>)`,
  `getLinkTransforms(vector<LinkId>)`). The string-spelled twins are gone; a `sensor_msgs::JointState`
  or other string source converts once at the caller's boundary with `toIds` / `toNames`.
  Group-name APIs are unaffected.
- **`ManipulatorInfo::tcp_offset` is a `std::variant<LinkId, Eigen::Isometry3d>`.** Assigning a
  name into it still works, but `std::get<std::string>` and `std::holds_alternative<std::string>`
  no longer name an alternative — use `LinkId`. Its `working_frame` and `tcp_frame` members are
  `LinkId` and take a name unchanged.
- **`Joint` link references are ids**: `child_link_name` / `parent_link_name` →
  `child_link_id` / `parent_link_id` (`LinkId`); `JointMimic::joint_name` → `joint_id`
  (`JointId`). Cereal keys are unchanged, so old archives load.
- **`JointGroup` and `JointState` are id-only.** The `get*Names()` accessors are gone;
  `getJointIds()` / `getLinkIds()` are the sole accessors and return by const reference.
  `JointState` also loses its joint-name constructor — build it from `std::vector<JointId>` plus a
  position vector. Convert with `toNames` at a boundary that genuinely needs strings.
- **Collection-returning APIs return ids**, not name vectors —
  `DiscreteContactManager::getCollisionObjects()` (`const std::vector<LinkId>&`),
  `JointGroup::calcFwdKin()` (`LinkIdTransformMap`), `KinematicGroup::getAllPossibleTipLinkIds()`,
  and the like.
- **`checkTrajectory` and `getAllowedCollisions` are id-only.** Their
  `std::vector<std::string>` overloads are gone. `checkTrajectory` keeps a
  `const std::vector<JointId>&` and a `const JointGroup&` overload, for each manager type.
- **`LinkNamesPair` and `makeOrderedLinkPair` are gone.** `LinkIdPair` replaces the
  `std::pair<std::string, std::string>` alias everywhere it was a container key, and canonicalizes
  itself in its constructor, so there is no separate ordering helper. The two SRDF helpers keep
  their names, retyped: `srdf::compareLinkPairAlphabetically` and `srdf::getAlphabeticalACMKeys`
  now speak `LinkIdPair`, and remain the sanctioned way to get ACM entries out in a stable,
  human-meaningful order.

## The trajopt surface changes

- **`TermInfo::link` → `link_id`** (`trajopt/problem_description.hpp`), on both the term that
  bounds link displacement and the singularity-avoidance term. `source_frame` / `target_frame` on
  the Cartesian pose terms keep their names and are now `LinkId`, so assigning a name still works.
- **Constraint constructors take ids.** `CartPosConstraint`, `CartLineConstraint` and
  `InverseKinematicsConstraint` take `LinkId source_frame, LinkId target_frame` — a name still
  converts, so these are breaks only if you were passing a name *container*.
- **`CollisionCoeffData`** (`trajopt_common/collision_types.h`) is pair-keyed: the two-name
  `setCollisionCoeff` / `getCollisionCoeff` are gone in favour of `LinkIdPair` forms,
  `getCollisionCoeffPairData()` is gone, and `PairsCollisionCoeffData` is keyed on `LinkIdPair`.
  `hasZeroCoeff` and `getPairsWithZeroCoeff` speak pairs too.
- **`trajopt/utils.hpp` free functions take id vectors** — `isSuperset`, `updateFromSubset` and
  `getSubset` each moved from `std::vector<std::string>` to `std::vector<JointId>`. Container
  parameters, so these are hard breaks.
- **Evaluator state is id-keyed.** `GetStateFn` and the transform caches use `LinkIdTransformMap`,
  and the active-link members became `std::unordered_set<LinkId>` — the same set-for-vector change
  the contact managers made; see Plugin authors for what that implies about indexing and order.

## The planning surface changes (`tesseract_planning`)

- **`JointWaypoint` and `StateWaypoint` no longer construct from joint names.** The
  `std::vector<std::string>` constructors are gone; construct from `std::vector<JointId>`. The
  brace-literal convenience survives and still works wherever the implicit conversion is available:
  `JointWaypoint({ "j1", "j2" }, { 1.0, 2.0 })`. Those overloads take the ids as a
  `std::vector<JointId>` and only the values as `std::initializer_list<double>` — a flat braced list
  cannot convert to an `Eigen::VectorXd` argument, which is the sole reason they exist — so an id
  vector you already hold works there too: `JointWaypoint(joint_ids, { 1.0, 2.0 })`. What no longer
  compiles is the case that mattered — handing a waypoint a `std::vector<std::string>` you are
  already holding. That is a compile error, not a silent per-joint hash.

  One honest caveat: the brace-literal convenience is gated per translation unit, not per argument,
  so it does not stop a `std::string` variable riding along in the braced list. Define
  `TESSERACT_NAMEID_NO_IMPLICIT` on your own targets if you want that caught — see
  `IDENTITY_DESIGN.md`, §"Type safety, and implicit string conversion".
- **The waypoint polys (joint and state) expose `setJointIds`/`getJointIds` only.** Both
  `setNames` and `getNames` are gone from every layer of the type-erasure stack and from the
  installed `test_suite/*.hpp` conformance headers. A caller that needs names for display or
  logging converts at its own boundary: `tesseract::common::toNames(wp.getJointIds())`.
- **The `command_language` free functions are id-only.** `getJointPosition`,
  `formatJointPosition`, and `checkJointPositionFormat` no longer have string delegates; call the
  id overload and convert at your own boundary if you only have names.
- **`getInterpolatedInstructions` takes `std::vector<JointId>`**
  (`motion_planners/simple/interpolation.h`), as does the `active_links` parameter in
  `motion_planners/trajopt/trajopt_utils.h`, now `std::vector<LinkId>`. Both are container
  parameters, so both are hard breaks.
- **Profile and config structs carry ids.** The trajopt move profiles' `working_frame` /
  `tcp_frame` and the `robot_config.h` free functions take `LinkId`. A name still converts, so
  these break only where you were passing or reading a name container.

## The ROS surface changes (`tesseract_ros2`)

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

`tesseract_monitoring` changed with it:

- **`CurrentStateMonitor`** — `haveCompleteState(std::vector<std::string>&)` (both the plain and
  the `rclcpp::Duration` forms) now reports missing joints as ids, `getCurrentStateValues()`
  returns `SceneState::JointValues`, and `isPassiveOrMimicDOF` takes a `JointId`.
- **`EnvironmentMonitorInterface::setEnvironmentState`** — the string-keyed map and the two
  `std::vector<std::string>` parallel-array overloads are gone; the id-keyed equivalents replace
  them.

The general path for any ROS message field that used to hand you a name is the same one that
applies everywhere else in this guide: convert at your own boundary with `toIds<JointId>` (or
take the id-keyed message conversion directly, where one exists) and hold onto the ids from
there.

## Plugin authors

### Collision backends

The contact-manager pure-virtual interface is retyped end-to-end: `addCollisionObject`,
`getCollisionObjects`, `isCollisionObjectEnabled`, `setCollisionObjectsTransform`,
`setActiveCollisionObjects`, and the validator hook all speak ids
(`tesseract/collision/discrete_contact_manager.h`, `continuous_contact_manager.h`). A third-party
backend must migrate its whole override surface at once — for scale, migrating the Coal collision
backend touched roughly 1,400 added and 900 removed lines.

The `std::string` conveniences on both managers are gone.
`setActiveCollisionObjects(const std::vector<std::string>&)` is deleted, and the member that
replaces it is keyed on a **set**: the pure virtual your backend must override is
`setActiveCollisionObjects(const std::unordered_set<LinkId>&)`. The `std::vector<LinkId>` and
`std::initializer_list<LinkId>` spellings still exist for callers, but they are non-pure
delegates to the set form — overriding one of those instead leaves the pure virtual
unimplemented, and your class stays abstract.
`getActiveCollisionObjects()` keeps its name but returns
`const std::unordered_set<LinkId>&` rather than `const std::vector<std::string>&`;
see Iteration order is unspecified below. The
parallel-array `setCollisionObjectsTransform(const std::vector<std::string>&, ...)`
overloads are replaced by `std::vector<LinkId>` equivalents, which throw
`std::runtime_error` on an id/pose size mismatch rather than reading out of bounds.
They remain `virtual` and non-pure: override them if your backend can apply a bulk
update more cheaply than one call per object. Build a `std::vector<LinkId>` at your
boundary with `toIds<LinkId>(names)`.

Both managers also gained a pure virtual that has no string-era counterpart:
`Eigen::Isometry3d getCollisionObjectsTransform(const LinkId& id) const`, which returns a single
object's world transform and throws `std::out_of_range` for an id the manager does not hold. It
is not part of the identity retyping — it rode along in the same release — but a backend that
only renames its existing overrides will still fail to compile until it implements this one.

### Kinematics backends

`ForwardKinematics` and `InverseKinematics` (`tesseract/kinematics/forward_kinematics.h`,
`inverse_kinematics.h`) are retyped the same way, and three of their pure virtuals were renamed as
well, so a backend that only changes types will still be abstract:

- `getBaseLinkName()` → `getBaseLinkId()`, returning `LinkId`
- `getJointNames()` → `getJointIds()`, returning `std::vector<JointId>`
- `getTipLinkNames()` → `getTipLinkIds()`, returning `std::vector<LinkId>`
- `calcFwdKin` fills a `LinkIdTransformMap`; `calcJacobian` takes a `LinkId`
- `calcInvKin` takes its tip-link poses as a `LinkIdTransformMap`
- `InverseKinematics::getWorkingFrame()` returns a `LinkId`

`getSolverName()` and `clone()` are unchanged. Callers building IK inputs should note that
`KinGroupIKInput::tip_link_name` is now `tip_link_id`, and both it and `working_frame` are
`LinkId`s.

## Iteration order is unspecified (and changed vs. the string era)

Ordered containers of ids (`std::map<LinkId, …>`, `std::set<LinkId>`, sorted vectors) iterate in an
order that is not alphabetical and is not stable across builds. Consequences:

- Results that were alphabetically ordered in the string era no longer are, and their order is
  now **unspecified**: `SceneGraph::getJointChildrenIds` (the vector overload),
  `KinematicGroup::getAllValidWorkingFrames()`, and `Environment::getStaticLinkIds(joint_ids)`.
  Sort by `.name()` yourself where display order matters.
- `ContactManager::getActiveCollisionObjects()` keeps its name but returns
  `const std::unordered_set<LinkId>&` instead of `const std::vector<std::string>&`. Indexing
  (`operator[]`, `at()`, `.data()`) is a compile error on the new type, so the only silent change
  is **iteration order**.
- The same applies to any id-keyed ordered container you write: do not rely on its iteration
  order across platforms, and never persist an order derived from it (hash values are
  runtime-only — see `IDENTITY_DESIGN.md`, Serialization).

## Unrelated API changes in the same release

In the `trajopt` repo, the per-contact `DiscreteCollisionEvaluator::getGradient` and
`ContinuousCollisionEvaluator::calcGradientData` virtuals were removed — the collision margin is
now resolved once per pair in `calcCollisions`/`calcCollisionData`; call the free
`trajopt_common::getGradient` overloads directly instead. The uncalled 4-argument two-timestep
`CollisionEvaluator::GetGradient(dofvals0, dofvals1, contact_result, isTimestep1)` convenience
wrapper was also removed; use the 6-argument overload taking `margin` and `coeff` explicitly.
