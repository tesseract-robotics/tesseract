# Tesseract Core Migration Guide: String-to-ID API Changes

This document is aimed at Claude Code for migrating downstream Tesseract projects
(tesseract_planning, trajopt, tesseract_ros, etc.) to work with the refactored
tesseract core. The changes span commits `8e9e9dc..c8eaf78` on `feature/integer-link-ids`.

## Overview of Breaking Changes

The core tesseract project replaced string-based identifiers with type-safe
`LinkId` / `JointId` objects throughout its public API. Key themes:

1. **`NameId::fromName()` removed** — replaced by implicit constructors
2. **String overloads removed** — all duplicate `(const std::string&)` methods deleted
3. **`ObjectPairKey` removed** — replaced by `LinkIdPair`
4. **`LinkIdPair::make()` removed** — replaced by constructor
5. **`LinkIdPair` members made private** — `.first` / `.second` become `.first()` / `.second()`
6. **`LinkIdPair` generalized** — now `OrderedIdPair<Tag>` with `using LinkIdPair = OrderedIdPair<LinkTag>`
7. **`KDLTreeData` members renamed** — string vectors become ID vectors
8. **`ChainGroup` type changed** — from `vector<pair<string,string>>` to `vector<pair<LinkId,LinkId>>`
9. **`SceneState` fields use ID-keyed maps** — `joints`, `link_transforms`, `joint_transforms` all ID-keyed

## Quick Reference: What Compiles Transparently

Because `NameId` has an **implicit** constructor from `std::string` and `const char*`,
many call sites compile without changes:

```cpp
// These all work without modification:
LinkId id = "my_link";                    // implicit conversion
LinkId id("my_link");                     // direct construction
some_function_taking_LinkId("my_link");   // implicit at call site
JointId jid = joint_name_string;          // implicit from std::string variable
```

**However**, implicit conversion does NOT help when:
- The removed overload had a different return type or behavior
- You need to construct containers (`vector<LinkId>` from `vector<string>`)
- You access map results keyed by the new ID type
- You use removed types like `ObjectPairKey`

---

## Migration Rules (Mechanical Replacements)

### Rule 1: `NameId::fromName(x)` → `NameId(x)` or just `x`

The static factory `LinkId::fromName(name)` / `JointId::fromName(name)` was removed.
Replace with the constructor, or rely on implicit conversion.

```cpp
// BEFORE
LinkId id = LinkId::fromName("base_link");
JointId jid = JointId::fromName(joint_name);

// AFTER
LinkId id("base_link");
JointId jid(joint_name);
// Or simply:
LinkId id = "base_link";
JointId jid = joint_name;
```

**Search pattern:** `LinkId::fromName\(` and `JointId::fromName\(`

### Rule 2: `ObjectPairKey` → `LinkIdPair`

The type alias `tesseract::collision::ObjectPairKey` (`std::pair<std::string, std::string>`)
was removed. It is replaced by `tesseract::common::LinkIdPair` (which is
`OrderedIdPair<LinkTag>`).

```cpp
// BEFORE
using tesseract::collision::ObjectPairKey;
ObjectPairKey key = std::make_pair(link1, link2);

// AFTER
using tesseract::common::LinkIdPair;
LinkIdPair key(link1_id, link2_id);  // auto-orders by hash value
```

**Important:** `LinkIdPair` canonically orders its elements (`first().value() <= second().value()`),
so `LinkIdPair(a, b) == LinkIdPair(b, a)`. This differs from `std::pair` which is order-sensitive.
If your code relied on `ObjectPairKey` preserving insertion order, the behavior has changed —
but all existing uses in tesseract core treated it as unordered, so this is likely fine.

**Search pattern:** `ObjectPairKey`

### Rule 3: `LinkIdPair::make(a, b)` → `LinkIdPair(a, b)`

The static factory was replaced by the constructor.

```cpp
// BEFORE
auto pair = LinkIdPair::make(id1, id2);

// AFTER
LinkIdPair pair(id1, id2);
```

**Search pattern:** `LinkIdPair::make\(`

### Rule 4: `pair.first` / `pair.second` → `pair.first()` / `pair.second()`

`LinkIdPair` members are now private with const accessor methods.

```cpp
// BEFORE
auto& link1 = pair.first;
auto& link2 = pair.second;

// AFTER
const auto& link1 = pair.first();
const auto& link2 = pair.second();
```

**Search pattern:** In the context of `LinkIdPair` variables, look for `.first` and `.second`
that are NOT on `std::pair` or `std::map::iterator`. The compiler will emit:
`error: invalid use of member function 'first' (did you forget the '()'?)`.

### Rule 5: `getCollisionObjectPairs()` signature changed

```cpp
// BEFORE
std::vector<ObjectPairKey> getCollisionObjectPairs(
    const std::vector<std::string>& active_links,
    const std::vector<std::string>& static_links,
    const std::shared_ptr<const ContactAllowedValidator>& validator = nullptr);

// AFTER
std::vector<LinkIdPair> getCollisionObjectPairs(
    const std::vector<LinkId>& active_links,
    const std::vector<LinkId>& static_links,
    const std::shared_ptr<const ContactAllowedValidator>& validator = nullptr);
```

Convert input vectors with `toIds()`:
```cpp
auto active_ids = tesseract::common::toIds<LinkId>(active_link_names);
auto static_ids = tesseract::common::toIds<LinkId>(static_link_names);
auto pairs = getCollisionObjectPairs(active_ids, static_ids, validator);
```

### Rule 6: String overloads removed from collision managers

`DiscreteContactManager` and `ContinuousContactManager` no longer have string-based
overloads. All methods now require `LinkId` parameters. The implicit constructor
handles most cases transparently:

```cpp
// BEFORE
manager->addCollisionObject("link_name", mask, shapes, poses);
manager->hasCollisionObject("link_name");
manager->enableCollisionObject("link_name");
manager->setCollisionObjectsTransform("link_name", pose);

// AFTER — these compile as-is because of implicit LinkId(const std::string&)
manager->addCollisionObject("link_name", mask, shapes, poses);  // OK
manager->hasCollisionObject("link_name");                        // OK
manager->enableCollisionObject("link_name");                     // OK
manager->setCollisionObjectsTransform("link_name", pose);        // OK
```

**But** the batch overload changed:
```cpp
// BEFORE
manager->setActiveCollisionObjects(std::vector<std::string>{"a", "b"});

// AFTER — still works, there's a string overload for setActiveCollisionObjects
manager->setActiveCollisionObjects(std::vector<std::string>{"a", "b"});  // OK
// Or use IDs directly:
manager->setActiveCollisionObjects(std::vector<LinkId>{"a", "b"});
```

### Rule 7: `AllowedCollisionMatrix` string overloads removed

```cpp
// BEFORE
acm.addAllowedCollision("link1", "link2", "Adjacent");
acm.isCollisionAllowed("link1", "link2");
acm.removeAllowedCollision("link1", "link2");
acm.removeAllowedCollision("link1");  // remove all pairs with link1

// AFTER — compiles as-is due to implicit LinkId constructor
acm.addAllowedCollision("link1", "link2", "Adjacent");  // OK
acm.isCollisionAllowed("link1", "link2");                // OK
acm.removeAllowedCollision("link1", "link2");            // OK
acm.removeAllowedCollision("link1");                     // OK — implicit LinkId("link1")
```

These will compile transparently. Only change needed if you stored the result
of methods that returned different types, or if you iterate the ACM entries
(they're now keyed by `LinkIdPair` not `pair<string,string>`).

### Rule 8: `CollisionMarginData` string overloads removed

Same pattern — implicit constructors handle most cases:

```cpp
// BEFORE
margin_data.setCollisionMargin("link1", "link2", 0.05);
margin_data.getCollisionMargin("link1", "link2");

// AFTER — compiles as-is
margin_data.setCollisionMargin("link1", "link2", 0.05);  // OK
margin_data.getCollisionMargin("link1", "link2");          // OK
```

### Rule 9: `SceneGraph` string overloads removed

All `SceneGraph` methods that accepted `const std::string&` for link/joint names
were removed. The implicit constructor handles most call sites:

```cpp
// BEFORE
auto link = scene_graph.getLink("base_link");
scene_graph.setRoot("base_link");
scene_graph.removeLink("old_link");
scene_graph.changeJointOrigin("joint1", new_origin);

// AFTER — compiles as-is
auto link = scene_graph.getLink("base_link");              // OK
scene_graph.setRoot("base_link");                           // OK
scene_graph.removeLink("old_link");                         // OK
scene_graph.changeJointOrigin("joint1", new_origin);        // OK
```

**Exception — `getRoot()` return type changed:**
```cpp
// BEFORE
const std::string& root = scene_graph.getRoot();

// AFTER
const LinkId& root = scene_graph.getRoot();
// If you need the string:
const std::string& root_name = scene_graph.getRoot().name();
```

**Search pattern:** `\.getRoot\(\)` — check if result is assigned to `std::string`

### Rule 10: `Link` and `Joint` string constructors removed

```cpp
// BEFORE
auto link = std::make_shared<Link>("my_link");
auto joint = std::make_shared<Joint>("my_joint");

// AFTER — compiles as-is due to implicit NameId constructor
auto link = std::make_shared<Link>("my_link");    // OK, implicit LinkId("my_link")
auto joint = std::make_shared<Joint>("my_joint");  // OK, implicit JointId("my_joint")
```

Same for `clone()`:
```cpp
// BEFORE
auto cloned = link->clone("new_name");

// AFTER — compiles as-is
auto cloned = link->clone("new_name");  // OK
```

### Rule 11: `Environment` string overloads removed

```cpp
// BEFORE
env->getLinkTransform("tcp_link");
env->getRelativeLinkTransform("base", "tool0");

// AFTER — compiles as-is
env->getLinkTransform("tcp_link");                    // OK
env->getRelativeLinkTransform("base", "tool0");       // OK
```

### Rule 12: `StateSolver` string overloads changed to delegating defaults

The `StateSolver` base class **retains string-based overloads** as virtual methods
with default implementations that delegate to the ID-based versions. So downstream
code calling `setState(vector<string>, VectorXd)` etc. still compiles.

**No changes needed** for code calling StateSolver methods with strings.

However, if you **subclass** `StateSolver` or `MutableStateSolver`:
- You must implement the ID-based pure virtual methods (not the string ones)
- Add `using StateSolver::setState;` etc. to prevent name hiding if you override any overload
- The string-based methods in `MutableStateSolver` were also removed — only ID-based pure virtuals remain

### Rule 13: `KDLTreeData` member names changed

```cpp
// BEFORE
data.base_link_name        → data.base_link_id
data.joint_names           → data.joint_ids
data.active_joint_names    → data.active_joint_ids
data.floating_joint_names  → data.floating_joint_ids
data.link_names            → data.link_ids
data.active_link_names     → data.active_link_ids
data.static_link_names     → data.static_link_ids
data.floating_joint_values → type changed from TransformMap to JointIdTransformMap
```

**Search pattern:** `\.base_link_name\b`, `\.joint_names\b`, `\.active_joint_names\b`,
`\.link_names\b`, `\.active_link_names\b`, `\.static_link_names\b`, `\.floating_joint_names\b`
(in the context of `KDLTreeData`)

### Rule 14: `KDLChainData` member names changed

```cpp
// BEFORE
chain_data.segment_index    // map<string, int>
chain_data.chains           // vector<pair<string, string>>

// AFTER
chain_data.segment_index    // map<LinkId, int>
chain_data.chains           // vector<pair<LinkId, LinkId>>
```

### Rule 15: `parseSceneGraph()` signatures changed

```cpp
// BEFORE
KDLTreeData parseSceneGraph(const SceneGraph& scene_graph,
                            const std::vector<std::string>& joint_names,
                            const std::unordered_map<std::string, double>& joint_values,
                            const TransformMap& floating_joint_values = {});

// AFTER
KDLTreeData parseSceneGraph(const SceneGraph& scene_graph,
                            const std::vector<JointId>& joint_ids,
                            const std::unordered_map<JointId, double>& joint_values,
                            const JointIdTransformMap& floating_joint_values = {});
```

### Rule 16: `ChainGroup` type changed

```cpp
// BEFORE
using ChainGroup = std::vector<std::pair<std::string, std::string>>;

// AFTER
using ChainGroup = std::vector<std::pair<LinkId, LinkId>>;
```

Code constructing chain groups:
```cpp
// BEFORE
ChainGroup chain = { std::make_pair("base_link", "tip_link") };

// AFTER — compiles as-is due to implicit LinkId constructor
ChainGroup chain = { std::make_pair(LinkId("base_link"), LinkId("tip_link")) };
// Or:
ChainGroup chain = { { "base_link", "tip_link" } };  // may need explicit LinkId depending on compiler
```

### Rule 17: KDL solver constructors changed

```cpp
// BEFORE
KDLFwdKinChain(scene_graph, "base_link", "tip_link", solver_name);
KDLFwdKinChain(scene_graph, vector<pair<string,string>>{{...}}, solver_name);

// AFTER — compiles as-is for single chain (implicit conversion)
KDLFwdKinChain(scene_graph, "base_link", "tip_link", solver_name);  // OK

// For multi-chain, need explicit LinkId pairs:
std::vector<std::pair<LinkId, LinkId>> chains = { {"base", "tip"} };
KDLFwdKinChain(scene_graph, chains, solver_name);
```

Same for `KDLInvKinChainLMA`, `KDLInvKinChainNR`, `KDLInvKinChainNR_JL`.

### Rule 18: `ContactResult::link_names` → `ContactResult::link_ids`

```cpp
// BEFORE
result.link_names[0]  // std::string
result.link_names[1]

// AFTER
result.link_ids[0]    // LinkId
result.link_ids[1]
// To get the string name:
result.link_ids[0].name()
```

**Search pattern:** `\.link_names\b` on `ContactResult` variables

### Rule 19: `ContactResultMap` is now keyed by `LinkIdPair`

```cpp
// BEFORE
using KeyType = std::pair<std::string, std::string>;
for (const auto& [key, results] : contact_map) {
    std::string link1 = key.first;
    std::string link2 = key.second;
}

// AFTER
using KeyType = LinkIdPair;
for (const auto& [key, results] : contact_map) {
    const std::string& link1 = key.first().name();
    const std::string& link2 = key.second().name();
}
```

### Rule 20: `JointState::joint_names` → `JointState::joint_ids`

```cpp
// BEFORE
state.joint_names  // std::vector<std::string>

// AFTER
state.joint_ids    // std::vector<JointId>
// To get names:
auto names = tesseract::common::toNames(state.joint_ids);
// Or use the method:
auto names = state.getJointNames();
```

**Search pattern:** `\.joint_names\b` on `JointState` variables

### Rule 21: `SceneState` fields are ID-keyed

```cpp
// BEFORE
scene_state.joints              // unordered_map<string, double>
scene_state.link_transforms     // TransformMap (string-keyed)
scene_state.joint_transforms    // TransformMap (string-keyed)
scene_state.floating_joints     // TransformMap (string-keyed)

// AFTER
scene_state.joints              // SceneState::JointValues = unordered_map<JointId, double>
scene_state.link_transforms     // LinkIdTransformMap
scene_state.joint_transforms    // JointIdTransformMap
scene_state.floating_joints     // JointIdTransformMap
```

Accessing by name:
```cpp
// BEFORE
double val = scene_state.joints["joint1"];
auto tf = scene_state.link_transforms["base_link"];

// AFTER
double val = scene_state.joints[JointId("joint1")];
auto tf = scene_state.link_transforms[LinkId("base_link")];
// Or with implicit conversion:
double val = scene_state.joints["joint1"];     // OK if JointId has implicit ctor
auto tf = scene_state.link_transforms["base_link"];  // OK
```

**But** iteration patterns that extract `.first` as `std::string` must change:
```cpp
// BEFORE
for (const auto& [name, value] : scene_state.joints) {
    std::cout << name << ": " << value;  // name is string
}

// AFTER
for (const auto& [id, value] : scene_state.joints) {
    std::cout << id.name() << ": " << value;  // id is JointId
}
```

### Rule 22: `TransformMap` type alias removed

The string-keyed `TransformMap` (`AlignedUnorderedMap<std::string, Eigen::Isometry3d>`)
has been removed from `eigen_types.h`. All usages must switch to
`JointIdTransformMap` or `LinkIdTransformMap` depending on whether the
keys represent joints or links.

**Search pattern:** `\bTransformMap\b`

### Rule 23: `CalibrationInfo::joints` type changed

```cpp
// BEFORE
tesseract::common::TransformMap joints;  // string-keyed
cal_info.joints["joint_name"] = transform;

// AFTER
tesseract::common::JointIdTransformMap joints;  // JointId-keyed
cal_info.joints["joint_name"] = transform;  // OK — implicit JointId ctor
```

Iteration that extracts the key as a string must use `.name()`:
```cpp
for (const auto& [id, tf] : cal_info.joints)
    some_function(id.name(), tf);  // id.name() to get string
```

### Rule 24: `EnvironmentMonitorInterface` floating joints parameter changed

All 8 `setEnvironmentState` methods changed their `floating_joints` parameter
from `TransformMap` to `JointIdTransformMap`.

```cpp
// BEFORE
bool setEnvironmentState(const std::string& ns,
    const std::unordered_map<std::string, double>& joints,
    const TransformMap& floating_joints = {}) const = 0;

// AFTER
bool setEnvironmentState(const std::string& ns,
    const std::unordered_map<std::string, double>& joints,
    const JointIdTransformMap& floating_joints = {}) const = 0;
```

**Downstream impact:** All implementations of `EnvironmentMonitorInterface`
(e.g., in `tesseract_ros`) must update their method signatures.

---

## Convenience Helpers

Two helpers were added to ease migration:

```cpp
#include <tesseract/common/types.h>

// Convert vector<string> → vector<IdT>
auto link_ids = tesseract::common::toIds<LinkId>(link_name_strings);
auto joint_ids = tesseract::common::toIds<JointId>(joint_name_strings);

// Convert any container of NameId<Tag> → vector<string>
auto names = tesseract::common::toNames(link_ids);
auto names = tesseract::common::toNames(joint_ids);
```

---

## Migration Strategy for Downstream Projects

### Step 1: Compile and Collect Errors

Build the downstream project against the updated tesseract core. The compiler
errors will pinpoint every breaking call site. Common error messages:

| Error Message | Likely Cause |
|---|---|
| `fromName is not a member of NameId` | Rule 1 |
| `ObjectPairKey was not declared` | Rule 2 |
| `make is not a member of LinkIdPair/OrderedIdPair` | Rule 3 |
| `invalid use of member function 'first' (did you forget '()'?)` | Rule 4 |
| `no matching function for call to ... (string vs LinkId)` | Rules 5-11 |
| `has no member named 'base_link_name'` | Rule 13 |
| `has no member named 'joint_names'` (on KDLTreeData) | Rule 13 |
| `has no member named 'link_names'` (on ContactResult) | Rule 18 |
| `has no member named 'joint_names'` (on JointState) | Rule 20 |

### Step 2: Apply Mechanical Replacements

Most fixes are simple search-and-replace. Process in this order:

1. **`fromName` calls** — Remove `::fromName(` wrapper, keep the argument
2. **`ObjectPairKey`** — Replace type with `LinkIdPair`, update construction
3. **`LinkIdPair::make(`** — Replace with `LinkIdPair(`
4. **`.first` / `.second` on LinkIdPair** — Add `()` to make them method calls
5. **`link_names` on ContactResult** — Change to `link_ids`, add `.name()` where string needed
6. **`joint_names` on JointState** — Change to `joint_ids`, add `.name()` where string needed
7. **KDLTreeData members** — Rename fields per Rule 13
8. **Container conversions** — Use `toIds<>()` / `toNames()` at boundaries
9. **Map iteration** — Update structured bindings to use `.name()` on ID keys

### Step 3: Verify Implicit Conversions

Many call sites will compile without changes thanks to the implicit `NameId`
constructors. After fixing compiler errors, verify that the implicit conversions
are doing what you expect — there should be no semantic changes since the
conversion is the same hash computation that `fromName()` used.

### Step 4: Build and Test

```bash
cd /path/to/build
cmake --build . -j$(nproc)
ctest --output-on-failure -j$(nproc)
```

---

## Project-Specific Notes

### tesseract_planning

Likely heavy usage of:
- `ManipulatorInfo` (tcp_offset is now `variant<LinkId, Isometry3d>` not `variant<string, Isometry3d>`)
- `JointState` / `JointTrajectory` — `joint_names` → `joint_ids`
- `ContactResultMap` iteration — keys are `LinkIdPair` not `pair<string,string>`
- `SceneState` field access — all ID-keyed maps
- `StateSolver` calls — string overloads still work (delegating defaults exist)
- `Environment` calls — string overloads for `setState`/`getState` still work

### trajopt / trajopt_ifopt

Likely heavy usage of:
- `ContactResult::link_names` → `link_ids`
- `ContactResultMap` keyed by `LinkIdPair`
- `CollisionMarginData` methods (string overloads removed but implicit conversion works)
- `DiscreteContactManager` / `ContinuousContactManager` methods
- `getCollisionObjectPairs()` — signature changed, needs `vector<LinkId>` inputs

### tesseract_ros / tesseract_ros2

Likely heavy usage of:
- `SceneState` fields — converting between ROS messages (string-based) and ID-keyed maps
- `JointState::joint_names` → `joint_ids` (need `toNames()` for ROS message population)
- `SceneGraph::getRoot()` — returns `LinkId` not `string`, need `.name()` for ROS
- `Environment` state methods — string overloads still work

### descartes_light

Likely uses:
- `StateSolver` interface — string overloads still work
- `KinematicGroup` / `JointGroup` — `getJointNames()` still works (delegates to IDs)

---

## Types Quick Reference

| Old Type/Pattern | New Type/Pattern |
|---|---|
| `std::string` (as link name) | `tesseract::common::LinkId` (implicit from string) |
| `std::string` (as joint name) | `tesseract::common::JointId` (implicit from string) |
| `LinkId::fromName("x")` | `LinkId("x")` or just `"x"` |
| `JointId::fromName("x")` | `JointId("x")` or just `"x"` |
| `ObjectPairKey` | `tesseract::common::LinkIdPair` |
| `LinkIdPair::make(a, b)` | `LinkIdPair(a, b)` |
| `pair.first` / `pair.second` (LinkIdPair) | `pair.first()` / `pair.second()` |
| `TransformMap` (removed) | `LinkIdTransformMap` / `JointIdTransformMap` |
| `CalibrationInfo::joints` (TransformMap) | `CalibrationInfo::joints` (JointIdTransformMap) |
| `ContactResult::link_names` | `ContactResult::link_ids` |
| `JointState::joint_names` | `JointState::joint_ids` |
| `KDLTreeData::joint_names` | `KDLTreeData::joint_ids` |
| `KDLTreeData::base_link_name` | `KDLTreeData::base_link_id` |
| `KDLChainData::segment_index` (string-keyed) | `KDLChainData::segment_index` (LinkId-keyed) |
| `ChainGroup = vector<pair<string,string>>` | `ChainGroup = vector<pair<LinkId,LinkId>>` |
| `SceneState::joints` (string-keyed) | `SceneState::joints` (JointId-keyed) |
| `SceneGraph::getRoot()` → `string` | `SceneGraph::getRoot()` → `LinkId` |
| `id.name()` | unchanged — returns `const std::string&` |
| `id.value()` | unchanged — returns `uint64_t` hash |
