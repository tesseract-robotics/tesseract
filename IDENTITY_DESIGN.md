# Identity Design: NameId, LinkId, JointId

Runtime identity for links and joints in Tesseract is a type-tagged integer hash paired with the name it was computed from. The hash drives container lookup; the name confirms equality and serves display and serialization. This document explains what that means, why it was done, and how hash collisions are resolved — deterministically, with the same semantics as a string-keyed hash map.

## The identity types

All of identity lives in `common/include/tesseract/common/types.h`:

```cpp
using NameIdValue = std::uint64_t;       // the hash value type

template <typename Tag>
struct NameId {
  NameId();                              // invalid (value == 0, empty name)
  NameId(const std::string&);            // hashes the name, retains it
  NameId(const char*);                   // ditto; nullptr yields the invalid id
  NameIdValue value() const noexcept;    // the hash; 0 means invalid
  const std::string& name() const;       // original string; empty for invalid ids
  bool isValid() const noexcept;         // value != 0
  bool operator==(const NameId&) const;  // hybrid: value first, name confirms
  bool operator<(const NameId&) const;   // by value; name breaks value ties
  static NameId createWithValueForTesting(NameIdValue, std::string);  // tests only
};

using LinkId  = NameId<LinkTag>;
using JointId = NameId<JointTag>;
inline const LinkId  INVALID_LINK_ID{};
inline const JointId INVALID_JOINT_ID{};

template <typename Tag>
struct OrderedIdPair {              // two full NameIds + cached combined hash
  OrderedIdPair(const NameId<Tag>& a, const NameId<Tag>& b);  // canonicalizes: (a,b) == (b,a)
  const NameId<Tag>& first() const;
  const NameId<Tag>& second() const;
  std::size_t hash() const;         // computed once at construction
  static std::size_t combineHash(NameIdValue, NameIdValue);   // the mixing function
};

using LinkIdPair = OrderedIdPair<LinkTag>;
```

The source name is retained on the `NameId` so public APIs, error messages, and on-disk serialization continue to speak strings. The hash drives lookup; the name stays available for everything that needs to address humans or files — and for the confirming compare in `operator==` (see "Equality and hash-collision resolution").

The numeric type behind a `NameId` is exposed as `NameIdValue` rather than spelled `uint64_t` everywhere, so the value type is named once at API boundaries and defined in one place.

`NameId` exposes no mutating methods after construction; instances are safe to share across threads.

**The invalid id.** A default-constructed `NameId`, an empty string, and a `nullptr` `const char*` all yield the invalid id: `value() == INVALID_NAME_ID_VALUE` (0), `name()` empty, `isValid() == false`. All invalid ids compare equal to each other and unequal to every valid id. The value is reserved for this state — in the astronomically unlikely event a real name hashes onto it, its value is remapped off it at construction (harmless: equality confirms names anyway). Use `INVALID_LINK_ID` / `INVALID_JOINT_ID` when an explicit sentinel reads better than `{}`.

**Test-only escape hatch.** `createWithValueForTesting` constructs an id with an explicit value/name combination so unit tests can manufacture hash collisions on demand. Never use it outside tests: an id whose value is not a function of its name breaks the invariant every container in the framework relies on.

## Type safety

`LinkId` and `JointId` are *distinct types* because `LinkTag` and `JointTag` are different. A link identity cannot flow into a joint-addressing API without an explicit conversion:

```cpp
graph.getLink(some_joint_id);  // compile error — JointId is not a LinkId
```

The previous string-based APIs could not distinguish — both link and joint lookups took `const std::string&`. A class of mix-up bugs (joint name flowing into link lookup, etc.) now fails at compile time.

This compile-time check fires where the implicit string-to-ID conversion is disabled — inside Tesseract's own production libraries (see "Implicit string conversion" below). Downstream code that uses the convenient implicit form retains the same bug-shape as the old string API; the migration neither tightens nor loosens safety at the public boundary.

The same tag scheme extends to `OrderedIdPair<Tag>`: `LinkIdPair` cannot be formed from `JointId`s by accident.

Note one footgun: `NameId::operator<` orders by hash value (name only breaks value ties), not by name. Ordered containers (`std::map`, `std::set`) work but produce no human-meaningful key order — sort by `name()` explicitly when display order matters.

## Implicit string conversion: implicit by default, explicit inside Tesseract

`NameId(const std::string&)` and `NameId(const char*)` are convenience constructors. By default they are **implicit**, so tests, examples, and downstream user code can write `graph.getLink("base")` and let the string convert. The constructors are guarded by a `TESSERACT_NAMEID_EXPLICIT` macro in `types.h` that expands to `explicit` when `TESSERACT_NAMEID_NO_IMPLICIT` is defined and to nothing otherwise.

Each of Tesseract's own production libraries adds `TESSERACT_NAMEID_NO_IMPLICIT` as a **PRIVATE** compile definition, so the constructors are explicit only in the libraries' own translation units. In practice this means:

- Inside Tesseract source files (`*/src/*.cpp` for `common`, `geometry`, `scene_graph`, `state_solver_*`, `collision*`, `kinematics*`, `srdf`, `urdf`, `environment`, `visualization`), any string-to-ID construction must be spelled out — `LinkId("base")` rather than `"base"`. The compiler surfaces every place an internal pathway is still going through strings, which is exactly what we want to migrate away from.
- Tests, examples, and downstream consumers (including templates instantiated in their TUs) keep the convenience of implicit conversion. The library's public API remains string-friendly.

`NameId` is a class template, so its constructors are inline and instantiated per TU. Differing explicit-ness across TUs is a compile-time check — the constructor bodies are identical, so there is no ABI/ODR impact.

The CMake plumbing lives in `common/cmake/tesseract_macros.cmake` (variable `TESSERACT_COMPILE_DEFINITIONS_PRIVATE`) and is applied via `target_compile_definitions(<lib> PRIVATE ${TESSERACT_COMPILE_DEFINITIONS_PRIVATE})` on each production library. New libraries added to Tesseract should follow the same pattern.

## Equality and hash-collision resolution

`std::hash<std::string>` produces a `std::size_t` (64 bits on this platform), so two distinct names *can* hash to the same value. The design resolves this the way a string-keyed hash map does, rather than trying to detect it:

- **Hybrid equality.** `NameId::operator==` compares the cached values first; only when the values match does it confirm with the name strings. Two colliding names therefore compare **unequal**, so every hash-keyed container keeps them as distinct keys — they land in the same bucket and are separated by the equality probe, which is precisely how `std::unordered_map<std::string, …>` handles a bucket collision. There is nothing to detect, no error path, and no way for a collision to silently alias two links.
- **Load-bearing invariant:** every valid `NameId` carries the name it was constructed from. The string constructors are the only public way to obtain a valid id, so the confirming compare always has both names available. (This is exactly the invariant `createWithValueForTesting` exists to violate — in tests only.)
- **Pairs are watertight the same way.** `OrderedIdPair` stores the two full `NameId`s, so pair equality is hybrid via the components. Canonicalization orders by value and breaks value ties by name, so `LinkIdPair(a, b) == LinkIdPair(b, a)` holds even when `a` and `b` collide.
- **Ordering is consistent.** `operator<` on both types breaks value ties by name, giving a strict weak ordering whose equivalence classes are exactly hybrid equality — safe for `std::map`/`std::set` and for sorting.

The practical consequence of a collision is a performance footnote, not a correctness event: the colliding ids share a bucket and pay a string compare to disambiguate, exactly as colliding strings would. At realistic scene sizes (fewer than 10^4 names) the 64-bit birthday bound puts the probability of even one collision around 10^-12 — and equality already pays one string compare on every *genuine* match anyway (see Performance), so the marginal cost is nil.

The SRDF parser's name verification (`tesseract::srdf::isRegisteredLink` / `isRegisteredJoint` in `srdf/src/utils.cpp`) is typo protection, not collision handling: it checks that a parsed name exists in the scene graph, and under hybrid equality that check is name-exact. Structural entries (groups, group states, calibration) abort SRDF load on an unknown name; advisory entries (disabled collisions, per-pair margins) warn and skip.

## Performance

The migration was motivated by profiling that showed string hashing and comparison dominating hot paths such as ACM queries and FK/IK state lookups. With string keys, every lookup paid:

- one `std::hash<std::string>` — `O(len(name))`, typically 20–50 chars for robot links
- one `std::string::operator==` per hash-bucket probe
- in the worst case, an SSO-miss allocation on temporary-key construction

With `NameId` keys:

- `std::hash<NameId>` is the identity function — it returns the pre-computed `NameIdValue`
- a probe that *misses* short-circuits on the value compare and never touches the strings
- a probe that *hits* pays one confirming string compare (the price of watertight equality)
- constructing a lookup key is free if you already hold a `NameId`

Both standard (`std::hash<NameId>`) and boost (`hash_value` ADL) hash entry points are provided, so `NameId` works as a key in both `std::unordered_map` and boost's hashed containers without spelling a hasher.

Hash computation is front-loaded at `NameId` construction (once) and then amortized over every subsequent lookup. For structures like `SceneGraph::link_map_`, `SceneGraph::joint_map_`, `SceneState::joints`, `LinkIdTransformMap`, and the ACM, that ratio is very favorable.

The corollary: inline construction at lookup sites (`map.find(LinkId("base"))`, or the implicit `map.find("base")`) is discouraged. It pays the same `std::string` + hash cost as the old string-keyed API, and the precomputed-hash win is only realized when the same `NameId` is constructed once and reused across many lookups.

**Pair keys are heavier than single ids.** `LinkIdPair` stores two full `NameId`s plus the cached combined hash (`sizeof(LinkIdPair) == 2 * sizeof(NameId) + sizeof(std::size_t)`, ~88 bytes on libstdc++) — the names are what make pair equality watertight. Constructing a pair copies both name strings (two heap allocations for non-SSO names), so the single-id corollary applies with more force: in a hot loop, construct the pair once per pair and reuse it across lookups rather than calling the two-id convenience overloads per element. The pair hash itself is computed once at construction by `OrderedIdPair::combineHash`, a boost-style mixer over the two id values using the 64-bit golden-ratio constant `0x9e3779b97f4a7c15`; lookups just return the cached result. Measured costs for all of these paths are recorded in `IDENTITY_BENCHMARKS.md`.

The migration is not a universal win. Constructing a `NameId` from a string is *slightly* more expensive than constructing a `std::string` because it hashes. Code that builds an identity and never looks anything up (rare) pays without benefit. Parse-heavy paths (URDF/SRDF/YAML load) are dominated by parsing itself, so the extra hash is in the noise.

Beyond raw speed, the migration is an API-consistency effort: every public interface that previously accepted a name string has an ID-accepting overload. IDs are the primary currency end-to-end, so callers never have to retain strings solely to feed downstream string APIs.

## Display and formatting

The name is the display representation. `NameId` has an `operator<<` that writes exactly `id.name()` — nothing more — so ids compose into log lines and error messages the same way the raw string used to. Debugging output that cares about bucket behavior prints `id.value()` explicitly; there is deliberately no `to_string` or formatter beyond the stream inserter.

`OrderedIdPair` has no inserter: pair formatting varies by context (separator, ordering, surrounding text), so print the two names explicitly.

Helpers `toIds` / `toNames` (`types.h`) convert name lists to id vectors and any container of ids back to names at API boundaries.

## Serialization and interoperability

**Hash values are runtime-only.** They are deterministic within a single process, but **not** stable across builds, standard-library versions, or architectures. They are never persisted.

The on-disk schemas were not changed by the migration. Existing serialized data loads on the new code without conversion, and files written by the new code remain readable by pre-migration tooling.

- **Cereal** stores ACM and margin data as `std::map<std::pair<std::string, std::string>, …>` — names from the pair key — and reconstructs the ids on load via the plain `add*` / `set*` entry points. See `common/include/tesseract/common/cereal_serialization.h`.
- **YAML** encodes pair keys as two-element name sequences and decodes by re-hashing. See `common/include/tesseract/common/yaml_extensions.h`.
- **URDF / SRDF** remain string-based on disk.

Users must not persist `NameId::value()`. It is a cache, not a key.

## Alternatives considered

### Detect-and-throw at insertion (the original design, superseded)

The first iteration of this migration kept value-only equality and instead tried to *detect* collisions at container-insertion time: scene-graph inserts compared stored vs incoming names and threw on mismatch, and the pair-keyed containers (ACM, margins, coefficients) carried redundant `name1`/`name2` fields in their values so checked write paths could do the same. Superseded by hybrid equality because detection is inherently partial and resolution is total:

- Detection only fires when both colliding names reach the *same container* through a *checked path*; bulk-replace and assignment adopted data unverified, and the checked-write idiom in each container was a convention new code could silently skip.
- It added a failure mode (throw on insert) that string keys never had, and the redundant name fields bloated every entry.
- Hybrid equality has none of these gaps: it is a property of the key type itself, so every container — present and future, core or downstream — is watertight by construction, with no error path and no coverage to audit.

### 128-bit UUIDs

Two variants. **Randomly-generated UUIDs** would require a registry to assign and persist a name → UUID mapping. That defeats the decoupled-construction property the design centers on: `LinkId("base")` could not be a free-standing value computable in a test, parser, or offline tool without consulting that registry.

**Name-based UUIDs** (UUIDv5: SHA-1 of namespace + name) are essentially a 128-bit hash and would push the birthday bound out to ~2^64 distinct names. However:

- A 128-bit value cannot be used directly as the `std::unordered_map` hash. `std::hash` returns `std::size_t` (64-bit), so a 128-bit UUID would be reduced to a 64-bit bucket index for the actual lookup. Bucket-level distribution is therefore identical to plain 64-bit hashing.
- `value()` would no longer be identity-hashable, per-key storage grows, and SHA-1 at construction is materially more expensive than `std::hash<std::string>`.
- Most importantly, a wider hash buys nothing correctness-wise: hybrid equality already makes collisions harmless, so the stronger hash would only shave the (already negligible) chance of an extra confirming string compare.

A related idea, a **construction-time interning registry** (probe on collision, guaranteeing process-wide value↔name injectivity), was rejected for the same registry reason plus unbounded growth: entries can never be evicted without reference-counting every live `NameId`.

### `std::shared_ptr<std::string>` for the name slot

Replacing the inline `std::string` with a `std::shared_ptr<std::string>` would shrink `sizeof(NameId)` (one pointer + control-block pointer ≈ 16 bytes vs a typical libstdc++ `std::string` of ~32 bytes) and make `NameId` copies cheap. Rejected because:

- Atomic reference-counting on every copy is expensive on the hot paths that move IDs around — exactly where the original perf motivation lives.
- An extra indirection on every `name()` access — now including the confirming compare in `operator==`.
- To get the deduplication benefit (shared storage across multiple `NameId`s carrying the same name) you need a global intern table, which loses the decoupled-construction property again.
- Without dedup, every `NameId` still allocates its own string, so the optimization only pays off on the rare copy-heavy case. In practice IDs land in a map once and rarely move; the target is small.

## Trade-offs

**Gained**

- Hot-path lookups (ACM, FK, IK, state queries) are faster — integer hash, and integer compare on every probe miss — instead of string hash + string compare throughout.
- Deterministic, watertight identity: hash collisions resolve like a string-keyed map, as a property of the key type. No detection machinery, no throw-on-insert failure mode, nothing for new containers to opt into.
- Compile-time separation of link and joint identity via tag types catches a class of mix-up bugs that the old string API could not.
- End-to-end API consistency — the ID is the primary currency; strings are needed only at parse/serialize boundaries and for display.

**Given up**

- `sizeof(NameId)` is larger than `sizeof(std::string)` (an extra `NameIdValue` alongside the string), and `LinkIdPair` is ~88 bytes — it carries both names so pair equality can confirm them. Pair construction copies two strings; hot loops should construct pairs once and reuse them.
- Equality *hits* pay one string compare (misses don't) — the price of watertight semantics.
- Hash cost is paid at `NameId` construction rather than per lookup. Code that constructs a `NameId` and never looks anything up pays without benefit.

## Quick reference

| Concern | Where |
|---|---|
| Type definitions, helpers, hash hooks | `common/include/tesseract/common/types.h` (header-only) |
| Scene-graph identity registry | `scene_graph/src/graph.cpp` (`addLinkHelper`, `addJointHelper`) |
| ACM write paths | `common/src/allowed_collision_matrix.cpp` |
| Margin write paths | `common/src/collision_margin_data.cpp` |
| YAML decoders | `common/include/tesseract/common/yaml_extensions.h` |
| Cereal string-compat format | `common/include/tesseract/common/cereal_serialization.h` |
| SRDF name verification | `srdf/src/utils.cpp` (`isRegisteredLink`, `isRegisteredJoint`); call sites in `srdf/src/{groups,group_states,disabled_collisions,collision_margins,configs}.cpp` |
| Benchmark record | `IDENTITY_BENCHMARKS.md` |
