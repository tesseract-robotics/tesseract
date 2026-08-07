# Identity Design: NameId, LinkId, JointId

Runtime identity for links and joints in Tesseract is a type-tagged integer hash paired with the name it was computed from. The hash drives container lookup; the name confirms equality and serves display and serialization. This document explains why the design exists, the requirements that shaped it, how it works — including how hash collisions are resolved, deterministically and with the same semantics as a string-keyed hash map — what migration means for downstream code, and why the main alternatives were rejected.

## Motivation

String-based identity had three problems:

1. **Hot-path cost.** Profiling showed string hashing and comparison dominating hot paths such as ACM queries and FK/IK state lookups. Every string-keyed lookup pays an `O(len)` hash plus a string compare per bucket probe, and pair-keyed maps (ACM, collision margins, coefficients) paid it twice over — see Performance for the full accounting.
2. **No way to amortize.** The lookup cost had no caller-side remedy: a name is its own representation, so there was no cheaper token to compute once and hold. Every container boundary re-derived identity from the string, no matter how many times the same caller had crossed it before.
3. **No type distinction.** Link and joint lookups both took `const std::string&`, so a joint name flowing into a link-addressing API compiled cleanly and failed at runtime, if it failed visibly at all.

The second problem dictates the migration's scope. A precomputed hash only pays off if ids are accepted end-to-end — any interface still taking only strings forces callers back to the string and forfeits the hash at that boundary. Hence ids as the primary currency: every public interface that accepted a name string has an ID-accepting overload, so identity computed once flows through the whole stack without ever being re-hashed.

## Requirements

The replacement had to preserve properties that string identity provided for free. These requirements are the yardstick against which the alternatives at the end of this document were judged:

- **Free-standing construction.** An identity must be computable from a name alone — in a test, a parser, an offline tool — before any `Environment` exists and without consulting a registry.
- **Stability across model boundaries.** Environments are cloned per planning thread, contact results produced against one state are inspected against a later one, and ACM/margin entries are authored against names in SRDF before a model is loaded. Identity must survive all of these boundaries unchanged.
- **A mutable model.** Links and joints are added and removed at runtime via Commands. Identity held by callers must not be invalidated by mutation of the scene graph.
- **Watertight correctness.** Two distinct names must never alias, under any input — the same guarantee string keys give, with no probabilistic caveat.
- **A string-friendly boundary.** Public APIs stay callable with strings, error messages stay readable, and on-disk formats stay unchanged — existing serialized data must load without conversion.

A name-derived hash satisfies the first three by construction: it is a pure function of the name, so it can be computed anywhere, agrees across models, and is untouched by scene mutation. The rest of the design — hybrid equality, the retained name — exists to satisfy the last two.

## The identity types

All of identity lives in `common/include/tesseract/common/types.h`:

```cpp
using NameIdValue = std::size_t;         // the hash value type
NameIdValue nameIdHash(const std::string&);                 // hashes a name into a NameIdValue

template <typename Tag>
struct NameId {
  NameId();                              // invalid (value == 0, empty name)
  NameId(std::string);                   // hashes the name, retains it
  NameId(const char*);                   // ditto; nullptr yields the invalid id
  NameIdValue value() const noexcept;    // the hash; 0 for an invalid id, but 0 is not reserved
  const std::string& name() const;       // original string; empty for invalid ids
  bool isValid() const noexcept;         // carries a name
  bool operator==(const NameId&) const;  // hybrid: value first, name confirms
  bool operator<(const NameId&) const;   // by value; name breaks value ties
  friend struct NameIdTestAccess;        // test-only backdoor, defined only in test headers
};

using LinkId  = NameId<LinkTag>;
using JointId = NameId<JointTag>;
inline const LinkId  INVALID_LINK_ID{};
inline const JointId INVALID_JOINT_ID{};

std::size_t combineNameIdHash(NameIdValue, NameIdValue);    // the mixing function

template <typename Tag>
struct OrderedIdPair {              // two full NameIds + cached combined hash
  OrderedIdPair(const NameId<Tag>& a, const NameId<Tag>& b);  // canonicalizes: (a,b) == (b,a)
  void assign(const NameId<Tag>& a, const NameId<Tag>& b);    // re-canonicalize in place, reusing string capacity
  const NameId<Tag>& first() const;
  const NameId<Tag>& second() const;
  std::size_t hash() const;         // combineNameIdHash of the two values, cached
  std::pair<const std::string&, const std::string&> orderedNameView() const&;  // alphabetical, for observable output
};

using LinkIdPair = OrderedIdPair<LinkTag>;
```

The source name is retained on the `NameId` so public APIs, error messages, and on-disk serialization continue to speak strings. The hash drives lookup; the name stays available for everything that needs to address humans or files — and for the confirming compare in `operator==` (see "Equality and hash-collision resolution").

The numeric type behind a `NameId` is exposed as `NameIdValue` rather than spelled `std::size_t` everywhere. It is `std::size_t` — the width every standard and boost hashed container reduces a hash to — and will not become anything else; the name exists to mark which values are raw name hashes, since those are runtime-only and must never be persisted.

`NameId` exposes no mutating methods after construction; instances are safe to share across threads.

**The invalid id.** A default-constructed `NameId`, an empty string, and a `nullptr` `const char*` all yield the invalid id: `name()` empty, `isValid() == false`, `value() == 0`. Validity is carried by the name, not by the value: an id is valid exactly when it has a name, and the string constructors are the only way to obtain one. Zero is *not* a reserved value — a real name that hashed onto it would give a perfectly valid id, which hybrid equality keeps distinct from the invalid one because the names differ. All invalid ids compare equal to each other and unequal to every valid id. Use `INVALID_LINK_ID` / `INVALID_JOINT_ID` when an explicit sentinel reads better than `{}`.

**Test-only escape hatch.** `NameIdTestAccess::create<IdT>` constructs an id with an explicit value/name combination so unit tests can manufacture hash collisions on demand. The struct is a friend of `NameId` defined only in `name_id_testing.h`, which ships as the opt-in `tesseract::common_test_suite` component rather than from `types.h`, so production code cannot name the operation — an id whose value is not a function of its name would break the invariant every container in the framework relies on.

## Equality and hash-collision resolution

`boost::hash<std::string>` produces a `std::size_t` (64 bits on this platform), so two distinct names *can* hash to the same value. The design resolves this the way a string-keyed hash map does, rather than trying to detect it:

- **Hybrid equality.** `NameId::operator==` compares the cached values first; only when the values match does it confirm with the name strings. Two colliding names therefore compare **unequal**, so every hash-keyed container keeps them as distinct keys — they land in the same bucket and are separated by the equality probe, which is precisely how `std::unordered_map<std::string, …>` handles a bucket collision. There is nothing to detect, no error path, and no way for a collision to silently alias two links.
- **Load-bearing invariant:** every valid `NameId` carries the name it was constructed from. The string constructors are the only public way to obtain a valid id, so the confirming compare always has both names available. (This is exactly the invariant `NameIdTestAccess` exists to violate — in tests only.)
- **Pairs are watertight the same way.** `OrderedIdPair` stores the two full `NameId`s, so pair equality is hybrid via the components. Canonicalization orders by value and breaks value ties by name, so `LinkIdPair(a, b) == LinkIdPair(b, a)` holds even when `a` and `b` collide.
- **Ordering is consistent.** `operator<` on both types breaks value ties by name, giving a strict weak ordering whose equivalence classes are exactly hybrid equality — safe for `std::map`/`std::set` and for sorting.

The practical consequence of a collision is a performance footnote, not a correctness event: the colliding ids share a bucket and pay a string compare to disambiguate, exactly as colliding strings would. At realistic scene sizes (fewer than 10^4 names) the 64-bit birthday bound puts the probability of even one collision around 10^-12 — and equality already pays one string compare on every *genuine* match anyway (see Performance), so the marginal cost is nil. Those odds are the 64-bit ones: on a 32-bit target the same 10^4 names collide with probability near 1%, which makes collisions occasional rather than effectively never — correctness is unaffected either way, and the cost is still one confirming string compare.

## Type safety

`LinkId` and `JointId` are *distinct types* because `LinkTag` and `JointTag` are different. A link identity cannot flow into a joint-addressing API without an explicit conversion:

```cpp
graph.getLink(some_joint_id);  // compile error — JointId is not a LinkId
```

This closes the mix-up class described under Motivation: a joint name flowing into a link-addressing API now fails at compile time instead of at runtime.

This compile-time check fires where the implicit string-to-ID conversion is disabled — inside Tesseract's own production libraries, and in any downstream library that opts in the same way (see "Implicit string conversion" below). Downstream code that keeps the convenient implicit form retains the same bug-shape as the old string API; the migration neither tightens nor loosens safety at the public boundary.

The same tag scheme extends to `OrderedIdPair<Tag>`: `LinkIdPair` cannot be formed from `JointId`s by accident.

Note one footgun: `NameId::operator<` orders by hash value (name only breaks value ties), not by name. Ordered containers (`std::map`, `std::set`) work but produce no human-meaningful key order — sort by `name()` explicitly when display order matters.

## Implicit string conversion: implicit by default, explicit inside Tesseract

`NameId(std::string)` and `NameId(const char*)` are convenience constructors. By default they are **implicit**, so tests, examples, and downstream user code can write `graph.getLink("base")` and let the string convert. The constructors are guarded by a `TESSERACT_NAMEID_EXPLICIT` macro in `types.h` that expands to `explicit` when `TESSERACT_NAMEID_NO_IMPLICIT` is defined and to nothing otherwise.

Each of Tesseract's own production libraries adds `TESSERACT_NAMEID_NO_IMPLICIT` as a **PRIVATE** compile definition, so the constructors are explicit only in the libraries' own translation units. In practice this means:

- Inside Tesseract source files (`*/src/*.cpp` for `common`, `geometry`, `scene_graph`, `state_solver_*`, `collision*`, `kinematics*`, `srdf`, `urdf`, `environment`, `visualization`), any string-to-ID construction must be spelled out — `LinkId("base")` rather than `"base"`. The compiler surfaces every place an internal pathway is still going through strings, which is exactly what we want to migrate away from.
- Tests, examples, benchmarks, and downstream consumers (including templates instantiated in their TUs) keep the convenience of implicit conversion. The library's public API remains string-friendly.

That convenience is meant to be used. In Tesseract's own tests, examples and benchmarks, construct ids implicitly from literals — `std::vector<JointId> joint_ids = { "joint_1", "joint_2" };` — rather than spelling out `JointId("joint_1")`. The explicit form is the *diagnostic* the production libraries opt into, not a house style; writing it in a TU that does not define `TESSERACT_NAMEID_NO_IMPLICIT` only adds noise.

The flip side of that convenience: existing downstream string call sites keep compiling unchanged — and keep paying the string-era cost of a string copy + hash per call. Upgrading and recompiling makes nothing faster by itself; the performance win requires constructing ids once and reusing them (see Performance).

`NameId` is a class template, so its constructors are inline and instantiated per TU and the check is compile-time only. Formally the differing token sequences across TUs are an ODR violation (ill-formed, no diagnostic required) — a deliberate, benign one: `explicit` affects overload resolution only, not layout or codegen, so all instantiations produce identical object code. C++20 modules would reject the scheme, so the toggle would have to be retired if Tesseract ever adopts them.

The CMake plumbing lives in `common/cmake/tesseract_macros.cmake` (variable `TESSERACT_COMPILE_DEFINITIONS_PRIVATE`) and is applied via `target_compile_definitions(<lib> PRIVATE ${TESSERACT_COMPILE_DEFINITIONS_PRIVATE})` on each production library. New libraries added to Tesseract should follow the same pattern.

**Downstream libraries can opt in, and should if they want the type check.** The macro is not Tesseract-private. A downstream library that defines `TESSERACT_NAMEID_NO_IMPLICIT` as a PRIVATE compile definition on its own targets gets the same diagnostic in its own translation units: every string-to-id construction has to be spelled out, and the link/joint separation described under "Type safety" is actually enforced rather than bypassed by the implicit conversion. Because the toggle is per translation unit and affects overload resolution only, it can be adopted one target at a time and consumers of that library are unaffected — including consumers that do not define it themselves.

## Performance

The cost claim from Motivation, itemized. With string keys, every lookup paid:

- one `boost::hash<std::string>` — `O(len(name))`; link names in the shipped models average 9 characters and reach 20
- one `std::string::operator==` per hash-bucket probe
- in the worst case, an SSO-miss allocation on temporary-key construction

With `NameId` keys:

- `std::hash<NameId>` returns the cached `NameIdValue` — a string hash computed once instead of per lookup
- a probe that *misses* short-circuits on the value compare and never touches the strings
- a probe that *hits* pays one confirming string compare (the price of watertight equality)
- constructing a lookup key is free if you already hold a `NameId`

`std::hash` is specialized for both id types in `types.h`, so they work as keys in `std::unordered_map`/`set` without spelling a hasher. Boost hashing lives in a separate opt-in header, `common/include/tesseract/common/types_boost_hash.h`: include it to hash an id with `boost::hash`, `boost::hash_combine`, or boost's unordered containers. Keeping it separate is what lets `types.h` — the most widely included header in the framework — stay free of boost includes.

The split is also load-bearing for correctness, which is why the `hash_value` ADL hooks live in that header rather than in `types.h`. Boost's open-addressing containers consult `hash_is_avalanching` at instantiation to decide whether to remix the hash before using it. The pair hash *is* avalanching — it is `boost::hash_combine` output regardless of what went in — so the header declares it and the containers skip a redundant mix. Were the hooks reachable without that declaration, one translation unit could instantiate a container with the remix and another instantiate the same type without it, and the two would disagree about which bucket a key belongs in. Bundling the hooks with the trait makes that unreachable: boost hashing and the declaration arrive together or not at all. (The trait reached boost in 1.81 and is compiled in conditionally; on older boost the containers that consult it do not exist yet.)

`NameId` is declared alongside it. Its value is `boost::hash<std::string>` output, which boost itself declares avalanching, and which — unlike `std::hash<std::string>` — is boost's own implementation and therefore identical on every platform rather than Murmur-derived on libstdc++ and FNV-1a on MSVC.

Hash computation is front-loaded at `NameId` construction (once) and then amortized over every subsequent lookup. For structures like `SceneGraph::link_map_`, `SceneGraph::joint_map_`, `SceneState::joints`, `LinkIdTransformMap`, and the ACM, that ratio is very favorable.

The corollary: inline construction at lookup sites (`map.find(LinkId("base"))`, or the implicit `map.find("base")`) is discouraged. It pays the same `std::string` + hash cost as the old string-keyed API, and the precomputed-hash win is only realized when the same `NameId` is constructed once and reused across many lookups.

**Pair keys.** The pair-keyed maps (ACM, collision margins, coefficients) need a pair key regardless of representation, so the only question is what it is made of. The string era used `std::pair<std::string, std::string>`, canonicalized by `makeOrderedLinkPair` — a string compare plus two string copies per construction — and hashed by running `boost::hash_combine` over both names: two `O(len)` string hashes on **every** lookup, plus two `std::string` compares per bucket probe. That is the single-id string cost above, doubled. `LinkIdPair` carries the same two names — they are what make pair equality watertight, confirming colliding pairs by name exactly as `NameId` does — but moves the hashing off the lookup path: canonicalization compares integer values (name only on a value tie), the combined hash is computed by `combineNameIdHash` when the pair is built — the same `boost::hash_combine`, now run over the two cached values instead of the two names — and every lookup just returns that cached `std::size_t`, short-circuiting on the two integer compares and touching the strings only on a genuine hit. So pairs realize the same precomputed-hash win as single ids — more of it, since the string version hashed twice. Note that "built once" describes the key, not necessarily the query: callers holding a long-lived pair (or `assign`ing into a scratch one) pay the mix once, but callers that construct a pair per lookup — `isContactAllowed`, `getCollisionMargin`, coal's `contactTest` — pay it per lookup. What those still save over the string era is the two `O(len)` name hashes and the per-probe string compares.

The residual cost is the two name copies at construction (`sizeof(LinkIdPair)` ~88 bytes on libstdc++, two heap allocations for non-SSO names) — the price of watertight equality, and copies the string pair paid too. The single-id corollary therefore applies with more force: in a hot loop, construct the pair once and reuse it across lookups rather than calling the two-id convenience overloads per element; `OrderedIdPair::assign` re-canonicalizes a long-lived pair in place, reusing its string capacity so reassignment allocates nothing once capacity suffices.

The migration is not a universal win. Constructing a `NameId` from a string is *slightly* more expensive than constructing a `std::string` because it hashes. Code that builds an identity and never looks anything up (rare) pays without benefit. Parse-heavy paths (URDF/SRDF/YAML load) are dominated by parsing itself, so the extra hash is in the noise.

**What to expect end-to-end.** The win scales with how identity-lookup-bound the workload is. Collision-heavy query paths (ACM checks, contact-map and transform-map access, state lookups) improve measurably; end-to-end planning times are typically dominated by planner and collision math rather than identity lookups, so a workload that was never bottlenecked on string keys will see little overall change. Id-based identity makes identity stop showing up in profiles — it does not make the rest of the pipeline faster.

## Display and formatting

The name is the display representation. `NameId` has an `operator<<` that writes exactly `id.name()` — nothing more — so ids compose into log lines and error messages the same way the raw string used to. Debugging output that cares about bucket behavior prints `id.value()` explicitly; there is deliberately no `to_string` or formatter beyond the stream inserter.

`OrderedIdPair` has no inserter: pair formatting varies by context (separator, ordering, surrounding text), so print the two names explicitly.

Helpers `toIds` / `toNames` (`types.h`) convert name lists to id vectors and any container of ids back to names at API boundaries.

## Serialization and interoperability

**Hash values are runtime-only.** They are deterministic within a single process, but **not** stable across builds, boost versions, or architectures. They are never persisted.

The on-disk schemas were not changed by the migration. Existing serialized data loads on the new code without conversion, and files written by the new code remain readable by pre-migration tooling.

- **Cereal** stores ACM and margin data as `std::map<std::pair<std::string, std::string>, …>` — names from the pair key — and reconstructs the ids on load via the plain `add*` / `set*` entry points. See `common/include/tesseract/common/cereal_serialization.h`.
- **YAML** encodes pair keys as two-element name sequences and decodes by re-hashing. See `common/include/tesseract/common/yaml_extensions.h`.
- **URDF / SRDF** remain string-based on disk.

Users must not persist `NameId::value()`. It is a cache, not a key.

## Migrating downstream code

Two properties bound what adopting ids costs a caller, and both are permanent features of the design rather than transition artifacts:

- **Data outlives the change.** The on-disk schemas are unchanged (see Serialization), so archives, URDF and SRDF load without conversion in either direction. String *literal* call sites also keep compiling, because the id constructors are implicit outside Tesseract's own libraries.
- **What breaks, breaks loudly.** Name-*typed* code does not survive: `std::vector<std::string>` has no conversion to an id vector, and getters that returned names return ids. Every such site is a compile error rather than a silent behavior change, and the fix is mechanical — convert once at the boundary with `toIds` / `toNames`, then hold ids from there rather than re-deriving them per call.

`IDENTITY_MIGRATION.md` catalogues the individual API changes for callers coming from the string era.

## Alternatives considered

Each alternative below either fails one of the Requirements above or fails to deliver the improvements that motivated the migration.

### Dense per-model integer indices

The approach used by MoveIt, Pinocchio, and RBDL: the model assigns each link and joint a small contiguous index at load time (link → `0..N-1`), and per-link data lives in plain vectors indexed directly. Where it applies, this is the strongest design in the space — no hashing, no hash maps, no collisions, pair keys are two ints, and dense vector access beats even an identity-hash bucket probe. It fails three requirements at once, because it presupposes a fixed model and Tesseract's central object is a mutable one:

- **Free-standing construction:** an index is meaningless without the model that assigned it. `LinkId("base")` is a value computable in a test, a parser, or an offline tool before any `Environment` exists; a dense index can only be obtained by asking a specific loaded model, which couples every identity to a registry.
- **A mutable model:** links and joints are added and removed via Commands. Removing a link either invalidates every index above it — silently corrupting ids held by callers, contact results, and cached maps — or forces tombstones and a free-list, at which point the indices are no longer dense and the vectors lose the properties that made them the win.
- **Stability across model boundaries:** per-model indices do not survive cloning, contact-result inspection against a later state, or SRDF authored before load. Every such boundary would need an index↔name translation layer — the string maps this design removes.

MoveIt-style indices work because the robot model there is immutable after load. For a fixed-topology inner loop built on top of Tesseract, dense indices remain a valid caller-side optimization; they could not be the identity type of the core API.

### 128-bit UUIDs

Two variants. **Randomly-generated UUIDs** require a registry to assign and persist a name → UUID mapping, failing free-standing construction outright: `LinkId("base")` could not be a value computable in a test, parser, or offline tool without consulting that registry.

**Name-based UUIDs** (UUIDv5: SHA-1 of namespace + name) are essentially a 128-bit hash and would push the birthday bound out to ~2^64 distinct names. However:

- A 128-bit value cannot be used directly as the `std::unordered_map` hash. `std::hash` returns `std::size_t` (64-bit), so a 128-bit UUID would be reduced to a 64-bit bucket index for the actual lookup. Bucket-level distribution is therefore identical to plain 64-bit hashing.
- `value()` would no longer be identity-hashable, per-key storage grows, and SHA-1 at construction is materially more expensive than `boost::hash<std::string>`.
- Most importantly, a wider hash buys nothing correctness-wise: hybrid equality already makes collisions harmless, so the stronger hash would only shave the (already negligible) chance of an extra confirming string compare.

A related idea, a **construction-time interning registry** (probe on collision, guaranteeing process-wide value↔name injectivity), fails the same registry test and adds unbounded growth: entries can never be evicted without reference-counting every live `NameId`. Two further hazards would remain even where a registry were acceptable. The table has to be exactly one instance across every shared library, and two copies — static linking, a plugin loaded `RTLD_LOCAL`, differing symbol visibility — hand out distinct pointers for the same name; that is merely wasteful until the confirming compare is reduced to a pointer compare, which is precisely the optimization interning exists to enable, and from then on ids that should be equal compare unequal on some build configurations only. Construction also stops being lock-free, mutating shared state on a path parallel planning enters concurrently, trading a cheap copy for a contended construction.

### `std::shared_ptr<std::string>` for the name slot

Replacing the inline `std::string` with a `std::shared_ptr<std::string>` would shrink `sizeof(NameId)` (one pointer + control-block pointer ≈ 16 bytes vs a typical libstdc++ `std::string` of ~32 bytes) and make `NameId` copies cheap — an atomic reference-count bump instead of a potentially allocating string copy. Rejected because:

- An extra indirection on every `name()` access — now including the confirming compare in `operator==`: two pointer hops to reach the characters instead of at most one, on the path every equality hit takes.
- To get the deduplication benefit (shared storage across multiple `NameId`s carrying the same name) you need a global intern table, which fails free-standing construction again.
- Without dedup, every `NameId` still allocates its own string, so the only remaining win is the cheap copy — and copy-heavy paths are rare: in practice IDs land in a map once and rarely move. The target is small, and lookups pay the indirection every time.

### Heterogeneous string lookup (transparent keys)

The cheap, orthogonal fix for one specific string-era cost: keep string keys but give the containers transparent comparators/hashers (`is_transparent` heterogeneous lookup), so lookups accept `std::string_view` and never allocate a temporary `std::string`. A few lines of container plumbing, no API change.

Rejected first on availability: Tesseract is C++17, and heterogeneous lookup for the *unordered* containers — the ones on the hot paths — arrived in C++20 (the ordered containers have had `std::less<>` transparency since C++14, but they are not where the cost is). Boost's hashed containers offer it under C++17, at the price of swapping container types throughout. Even where available, it removes only the temporary-key allocation, not the lookup cost itself: every probe still pays the `O(len)` string hash plus a string compare per bucket candidate — the costs profiling flagged — and it offers nothing for link/joint type safety or for pair keys. It is complementary rather than competing: the same machinery works with `NameId` keys wherever a string-keyed boundary remains.

### Collision detection instead of resolution

An id could keep value-only equality — no retained name on the compare path — and handle collisions by *detecting* them where ids enter containers: scene-graph inserts compare the stored name against the incoming one and throw on mismatch, and pair-keyed containers carry redundant name fields in their values so checked write paths can do the same. Rejected because detection is inherently partial where resolution is total:

- Detection only fires when both colliding names reach the *same container* through a *checked path*; bulk-replace and assignment adopt data unverified, and the checked-write idiom in each container is a convention new code can silently skip.
- It adds a failure mode (throw on insert) that string keys never had, and the redundant name fields bloat every entry.
- Hybrid equality has none of these gaps: it is a property of the key type itself, so every container — present and future, core or downstream — is watertight by construction, with no error path and no coverage to audit.

## Trade-offs

**Gained**

- Hot-path lookups (ACM, FK, IK, state queries) are faster — integer hash, and integer compare on every probe miss — instead of string hash + string compare throughout.
- Deterministic, watertight identity: hash collisions resolve like a string-keyed map, as a property of the key type. No detection machinery, no throw-on-insert failure mode, nothing for new containers to opt into.
- Compile-time separation of link and joint identity via tag types catches a class of mix-up bugs that the old string API could not.
- End-to-end API consistency — the ID is the primary currency; strings are needed only at parse/serialize boundaries and for display.

**Given up**

- `sizeof(NameId)` is larger than `sizeof(std::string)` (an extra `NameIdValue` alongside the string), and `LinkIdPair` is ~88 bytes — it carries both names so pair equality can confirm them. Pair construction copies two strings; hot loops should construct pairs once and reuse them. In aggregate the footprint growth is small: `ContactResult` goes from 768 to 784 bytes (+2.1%) versus the string era, and no allocation is added — the retained name is the same heap allocation the old `link_names` made, so the only new cost is the inline 8-byte hash per id. Pair keys are the wider growth (88 vs 64 bytes) but scale with colliding link *pairs*, not contact points: `ContactResultMap` stores a `ContactResultVector` per key. Sizes are libstdc++/x86-64.
- Hash cost (minimal) is paid at `NameId` construction rather than per lookup. Code that constructs a `NameId` and never looks anything up pays without benefit.

## Quick reference

| Concern | Where |
|---|---|
| Type definitions, helpers, `std::hash` | `common/include/tesseract/common/types.h` (header-only) |
| Boost hashing support (opt-in) | `common/include/tesseract/common/types_boost_hash.h` |
| Scene-graph identity registry | `scene_graph/src/graph.cpp` (`addLinkHelper`, `addJointHelper`) |
| ACM write paths | `common/src/allowed_collision_matrix.cpp` |
| Margin write paths | `common/src/collision_margin_data.cpp` |
| YAML decoders | `common/include/tesseract/common/yaml_extensions.h` |
| Cereal string-compat format | `common/include/tesseract/common/cereal_serialization.h` |
