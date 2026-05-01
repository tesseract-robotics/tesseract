# Identity Design: NameId, LinkId, JointId

Runtime identity for links and joints in Tesseract is a tagged integer hash (currently 64-bit), not a string. This document explains what that means, why it was done, how type safety and performance work out, and — most importantly — why hash collisions cannot silently corrupt data.

## The identity types

All of identity lives in `common/include/tesseract/common/types.h`:

```cpp
using NameIdValue = std::uint64_t;       // typedef so the width can be widened later

template <typename Tag>
struct NameId {
  NameId();                              // invalid (value == 0)
  NameId(const std::string&);            // hashes the name, retains it
  NameIdValue value() const noexcept;    // the hash
  const std::string& name() const;       // original string, for display / serialization
  bool isValid() const noexcept;         // value != 0
  bool operator==(const NameId&) const noexcept;  // value-based
};

using LinkId  = NameId<LinkTag>;
using JointId = NameId<JointTag>;

template <typename Tag>
struct OrderedIdPair {
  OrderedIdPair(const NameId<Tag>& a, const NameId<Tag>& b);
  // Canonicalizes so (a,b) == (b,a); caches the combined pair hash.
};

using LinkIdPair = OrderedIdPair<LinkTag>;
```

The source name is retained on the `NameId` so public APIs, error messages, and on-disk serialization continue to speak strings. The hash drives lookup and equality; the name stays available for everything that needs to address humans or files.

The numeric type behind a `NameId` is exposed as `NameIdValue` rather than spelled `uint64_t` everywhere, so a future widening only requires changing the typedef and a different hash function — *not* every downstream caller. Note that `std::hash<std::string>` itself returns `std::size_t`, so widening `NameIdValue` past 64 bits is preparation only; producing a wider hash also requires switching off `std::hash<std::string>` to something like xxh128.

## Type safety

`LinkId` and `JointId` are *distinct types* because `LinkTag` and `JointTag` are different. A link identity cannot flow into a joint-addressing API without an explicit conversion:

```cpp
graph.getLink(some_joint_id);  // compile error — JointId is not a LinkId
```

The previous string-based APIs could not distinguish — both link and joint lookups took `const std::string&`. A class of mix-up bugs (joint name flowing into link lookup, etc.) now fails at compile time.

The same tag scheme extends to `OrderedIdPair<Tag>`: `LinkIdPair` cannot be formed from `JointId`s by accident.

Note one footgun: `NameId::operator<` orders by hash value, not by name. Ordered containers (`std::map`, `std::set`) work but produce no human-meaningful key order — sort by `name()` explicitly when display order matters.

## Implicit string conversion: implicit by default, explicit inside Tesseract

`NameId(const std::string&)` and `NameId(const char*)` are convenience constructors. By default they are **implicit**, so tests, examples, and downstream user code can write `graph.getLink("base")` and let the string convert. The constructors are guarded by a `TESSERACT_NAMEID_EXPLICIT` macro in `types.h` that expands to `explicit` when `TESSERACT_NAMEID_NO_IMPLICIT` is defined and to nothing otherwise.

Each of Tesseract's own production libraries adds `TESSERACT_NAMEID_NO_IMPLICIT` as a **PRIVATE** compile definition, so the constructors are explicit only in the libraries' own translation units. In practice this means:

- Inside Tesseract source files (`*/src/*.cpp` for `common`, `geometry`, `scene_graph`, `state_solver_*`, `collision*`, `kinematics*`, `srdf`, `urdf`, `environment`, `visualization`), any string-to-id construction must be spelled out — `LinkId("base")` rather than `"base"`. The compiler surfaces every place an internal pathway is still going through strings, which is exactly what we want to migrate away from.
- Tests, examples, and downstream consumers (including templates instantiated in their TUs) keep the convenience of implicit conversion. The library's public API remains string-friendly.

`NameId` is a class template, so its constructors are inline and instantiated per TU. Differing explicit-ness across TUs is a compile-time check — the constructor bodies are identical, so there is no ABI/ODR impact.

The CMake plumbing lives in `common/cmake/tesseract_macros.cmake` (variable `TESSERACT_COMPILE_DEFINITIONS_PRIVATE`) and is applied via `target_compile_definitions(<lib> PRIVATE ${TESSERACT_COMPILE_DEFINITIONS_PRIVATE})` on each production library. New libraries added to Tesseract should follow the same pattern.

## Performance

The migration was motivated by profiling that showed string hashing and comparison dominating hot paths such as ACM queries and FK/IK state lookups. With string keys, every lookup paid:

- one `std::hash<std::string>` — `O(len(name))`, typically 20–50 chars for robot links
- one `std::string::operator==` per hash-bucket probe
- in the worst case, an SSO-miss allocation on temporary-key construction

With `NameId` keys:

- `std::hash<NameId>` is the identity function — it returns the pre-computed `NameIdValue`
- equality and probe comparisons are `NameIdValue` compares
- constructing a lookup key is free if you already hold a `NameId`

Both standard (`std::hash<NameId>`) and boost (`hash_value` ADL) hash entry points are provided, so `NameId` works as a key in both `std::unordered_map` and `boost::unordered_flat_map` without spelling a hasher.

Hash computation is front-loaded at `NameId` construction (once) and then amortized over every subsequent lookup. For structures like `SceneGraph::link_map_`, `SceneGraph::joint_map_`, `SceneState::joints`, `LinkIdTransformMap`, and the ACM, that ratio is very favorable.

Per-pair storage is also smaller. `LinkIdPair` is `2 * sizeof(NameIdValue) + sizeof(std::size_t)` bytes (24 bytes today) — two `NameIdValue` ids plus a cached `size_t` pair hash — regardless of name length. The old per-pair key was two `std::string`s with their attendant size and allocation overhead.

The migration is not a universal win. Constructing a `NameId` from a string is *slightly* more expensive than constructing a `std::string` because it hashes. Code that builds an identity and never looks anything up (rare) pays without benefit. Parse-heavy paths (URDF/SRDF/YAML load) are dominated by parsing itself, so the extra hash is in the noise.

Beyond raw speed, the migration is an API-consistency effort: every public interface that previously accepted a name string has an `Id`-accepting overload. The goal is for IDs to be the primary currency end-to-end so callers never have to retain strings solely to feed downstream string APIs.

## Serialization and interoperability

**Hashes are runtime-only.** They are deterministic within a single process, but **not** stable across builds, across libc++ versions, or across architectures. They are never persisted.

- **Cereal** stores ACM and margin data as `std::map<std::pair<std::string, std::string>, …>` and reconstructs the hash on load via the checked `add*` / `set*` entry points. See `common/include/tesseract/common/cereal_serialization.h`.
- **YAML** encodes pair keys as two-element name sequences and decodes by re-hashing, again with collision checks. See `common/include/tesseract/common/yaml_extensions.h`.
- **URDF / SRDF** remain string-based on disk.

Users must not persist `NameId::value()`. It is a cache, not a key.

## Hash collision risk and prevention

`std::hash<std::string>` produces a `std::size_t` (currently 64 bits on this platform), so collisions are mathematically possible, though astronomically rare for typical robot link/joint names. The design guarantees that a collision, if it ever occurred, would surface as a **loud, localized runtime error**, never as silent aliasing or dropped data.

There are two layers of risk and two layers of defense.

### Layer 1 — single-ID collision in the SceneGraph

All link/joint insertion routes through `SceneGraph::addLinkHelper` / `addJointHelper` in `scene_graph/src/graph.cpp`. Each does a hash-keyed lookup in `link_map_` / `joint_map_` and delegates to `common::checkHashCollision`, which throws `std::runtime_error` citing both names when the stored name differs from the name being inserted.

Because the scene graph is the authoritative identity registry, every downstream map keyed on `LinkId` / `JointId` — `LinkIdTransformMap`, `JointIdTransformMap`, `SceneState::joints`, kinematics group vectors, kdl_parser internal maps — is populated from IDs that already passed through that gate. The guarantee propagates.

The SRDF parser has an additional safeguard: every place that resolves a parsed name routes through `tesseract::srdf::isRegisteredLink` / `isRegisteredJoint` (`srdf/src/utils.cpp`), which returns false both when the name is unknown and when it hash-collides with an already-loaded scene-graph link or joint of a different name — indistinguishable by design, since both are equally disqualifying for SRDF name resolution. Call sites in `srdf/src/{groups,group_states,disabled_collisions,collision_margins,configs}.cpp` handle the false return with their own policy (throw vs warn-and-continue) and context-specific error message.

### Layer 2 — pair-ID collision in ACM and CollisionMarginPairData

`AllowedCollisionMatrix` and `CollisionMarginPairData` key on `LinkIdPair`. The stored entries carry the original names (`ACMEntry::name1/name2`, `PairMarginEntry::name1/name2`), which makes collision detection possible: when a write finds an existing entry at the same key, `common::checkPairHashCollision` compares the two incoming names against the two stored names — canonicalized in either order — and throws if they don't match.

Every mutation path routes through a single private helper per container (`insertEntryChecked`), so collision detection is not an optional add-on bolted onto some paths. The covered paths:

- `addAllowedCollision` / `setCollisionMargin` — incremental setters
- `insertAllowedCollisionMatrix` — ACM-to-ACM merge
- `CollisionMarginPairData::apply(..., MODIFY)` — margin merge
- Bulk constructors taking `AllowedCollisionEntries` / `PairsCollisionMarginData`
- YAML decoders for those two container types
- Cereal load paths — transitively, because they delegate to the incremental setters

Any future mutation path has one obvious place to route through.

### What is intentionally *not* guaranteed

- Collision detection requires both colliding entries to be *added to the same container at some point*. If only one name of a colliding pair ever appears in a given run, there is nothing to detect.
- `CollisionMarginPairData::apply(..., REPLACE)` and assignment adopt the source verbatim. The source's own write path is the trust boundary; a corrupt source becomes a corrupt target.

### Sister structures outside core tesseract

`trajopt_common::CollisionCoeffData` wraps `PairsCollisionCoeffData` (`std::unordered_map<LinkIdPair, PairCoeffEntry>`) and mirrors the same threat model. Its single write path, `CollisionCoeffData::setCollisionCoeff`, uses `try_emplace` + `checkPairHashCollision("CollisionCoeffData", ...)` — the same idiom as ACM/margin. All other population routes into the container funnel through that setter:

- YAML decoder (`trajopt_common/include/trajopt_common/yaml_extensions.h`)
- Cereal load (`trajopt_common/include/trajopt_common/cereal_serialization.h`)
- JSON problem-description loader (`trajopt/src/problem_description.cpp`)

There is no bulk constructor from `PairsCollisionCoeffData`, no `apply(..., MODIFY)`, and no direct `lookup_table_` write outside the setter. The source of populated data is never the ACM or SceneGraph — it is user-supplied JSON/YAML/code — so the check is applied exactly where it matters.

Net: trajopt_common's pair-keyed container inherits the same single-point-of-enforcement guarantee as the two containers in `tesseract::common`, just via the incremental-setter route rather than a private `insertEntryChecked` helper. Future sister structures added to planning/ifopt/sco should follow the same pattern.

## Trade-offs

**Gained**

- Hot-path lookups (ACM, FK, IK, state queries) are faster — integer hash + integer compare instead of string hash + string compare.
- `LinkIdPair` storage is a fixed `2 * sizeof(NameIdValue) + sizeof(std::size_t)` bytes (24 today) regardless of name length.
- Compile-time separation of link and joint identity via tag types catches a class of mix-up bugs that the old string API could not.
- Centralized collision detection — each ACM/margin container has exactly one private helper that every mutation path goes through. Future additions get safety by default.
- End-to-end API consistency — the ID is the primary currency; strings are needed only at parse/serialize boundaries and for display.

**Given up**

- `sizeof(NameId)` is larger than `sizeof(std::string)` on typical standard libraries (an extra `NameIdValue` alongside the string). For storage of a `NameId` outside a map, this is a small loss.
- Hash cost is paid at `NameId` construction rather than per lookup. Code that constructs a `NameId` and never looks anything up pays without benefit.
- The on-disk format is still string-based, so entries carry both the `LinkIdPair` key *and* the names — a small amount of redundancy. This redundancy is also what makes collision detection possible, so it is a deliberate cost.
- A new failure mode — `std::runtime_error` on hash collision during insertion — did not exist with string keys. Extremely unlikely to trigger in practice, but possible.

## Quick reference

| Concern | Where |
|---|---|
| Type definitions, helpers | `common/include/tesseract/common/types.h`, `common/src/types.cpp` |
| Single-ID collision enforcement | `common/src/types.cpp` (`checkHashCollision`); `scene_graph/src/graph.cpp` (`addLinkHelper`, `addJointHelper`) |
| ACM write paths | `common/src/allowed_collision_matrix.cpp` (`insertEntryChecked`) |
| Margin write paths | `common/src/collision_margin_data.cpp` (`insertEntryChecked`) |
| YAML decoders | `common/include/tesseract/common/yaml_extensions.h` |
| Cereal string-compat format | `common/include/tesseract/common/cereal_serialization.h` |
| SRDF name verification | `srdf/src/utils.cpp` (`isRegisteredLink`, `isRegisteredJoint`); call sites in `srdf/src/{groups,group_states,disabled_collisions,collision_margins,configs}.cpp` |
