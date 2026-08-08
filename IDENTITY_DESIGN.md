# Identity Design: NameId, LinkId, JointId

Runtime identity for links and joints in Tesseract is a type-tagged hash of the name, paired with the name it was computed from. The hash drives container lookup; the name confirms equality and serves display and serialization. An id is a pure function of its name, so it can be constructed anywhere — in a test, a parser, an offline tool — with no model and no registry to consult.

```cpp
const LinkId base("base_link");                               // hashes the name once, and retains it
const LinkId tool("tool0");

const Eigen::Isometry3d& tf = state.link_transforms.at(base); // integer hash; a probe miss touches no string
const bool allowed = acm.isCollisionAllowed({ base, tool });  // pair key: canonicalized and hashed on construction
std::cout << base << " -> " << tool << '\n';                  // writes "base_link -> tool0"
```

## Why not strings

1. **Hot-path cost.** Profiling showed string hashing and comparison dominating hot paths such as ACM queries and FK/IK state lookups. A string-keyed lookup pays an `O(len)` hash plus a string compare per bucket probe, and pair-keyed maps (ACM, collision margins, coefficients) pay it twice over.
2. **No way to amortize.** A name is its own representation, so there is no cheaper token to compute once and hold. Every container boundary re-derives identity from the string, no matter how many times the same caller has crossed it.
3. **No type distinction.** A name carries no indication of what it names, so link and joint lookups have the same signature: a joint name flowing into a link-addressing API compiles cleanly and fails at runtime, if it fails visibly at all.

The second problem is why ids are the API's primary currency rather than an optional fast path: a precomputed hash only pays off if ids are accepted end-to-end, since any interface still taking only strings forces callers back to the string and forfeits the hash at that boundary. Every public interface that addresses a link or a joint therefore takes an id, and identity computed once flows through the whole stack without being re-derived.

## The identity types

The identity types live in `common/include/tesseract/common/types.h`. The sketch is abridged — it omits the remaining comparison operators, the `std::hash` specializations, the stream inserter and the `toIds`/`toNames` helpers; the boost hooks live in `types_boost_hash.h`, and the cereal and YAML serializers in their format headers.

```cpp
template <typename Tag>
struct NameId {
  NameId();                              // invalid: empty name, value 0
  NameId(std::string);                   // hashes the name, retains it
  std::size_t value() const noexcept;    // the cached hash
  const std::string& name() const;       // original string; empty for invalid ids
  bool isValid() const noexcept;         // true exactly when it has a name
  bool operator==(const NameId&) const;  // hybrid: value first, name confirms
  bool operator<(const NameId&) const;   // by value; name breaks value ties
};

using LinkId  = NameId<LinkTag>;
using JointId = NameId<JointTag>;

template <typename Tag>
struct OrderedIdPair {                                        // two full NameIds + cached combined hash
  OrderedIdPair(const NameId<Tag>& a, const NameId<Tag>& b);  // canonicalizes: (a,b) == (b,a)
  void assign(const NameId<Tag>& a, const NameId<Tag>& b);    // re-canonicalize in place, reusing string capacity
  const NameId<Tag>& first() const;                           // by hash value, not alphabetical
  const NameId<Tag>& second() const;
  std::size_t hash() const;                                   // combineNameIdHash of the two values, cached
  std::pair<const std::string&, const std::string&> orderedNameView() const&;  // alphabetical, for observable output
};

using LinkIdPair = OrderedIdPair<LinkTag>;
```

The source name is retained on the `NameId` so public APIs, error messages, and on-disk serialization continue to speak strings. The hash drives lookup; the name stays available for everything that needs to address humans or files — and for the confirming compare in `operator==`.

The value is a plain `std::size_t` — the width every standard and boost hashed container reduces a hash to. Widening it would buy nothing: hybrid equality already resolves collisions, so a wider hash would only trade memory for a slightly rarer confirming string compare.

`NameId` has no setters and no lazily-computed state, so concurrent `const` access needs no synchronization — which is what matters for environments cloned per planning thread. It is copy- and move-assignable like any value type, so a shared instance still must not be assigned to while another thread reads it. `OrderedIdPair` additionally has `assign`, an explicit mutator: a scratch pair reused across lookups belongs to a single thread.

**The invalid id.** A default-constructed `NameId`, an empty string, and a `nullptr` `const char*` all yield the invalid id: `name()` empty, `isValid() == false`, `value() == 0`. Validity is carried by the name, not by the value: an id is valid exactly when it has a name, and the string constructors are the only way to obtain one. Zero is *not* a reserved value — a real name that hashed onto it would give a perfectly valid id, which hybrid equality keeps distinct from the invalid one because the names differ. All invalid ids compare equal to each other and unequal to every valid id.

No Tesseract API returns an invalid id, and there are no named sentinel constants. Lookups signal not-found the way string-keyed ones did — `getLink` / `getJoint` return `nullptr`, container lookups return `end()`, and accessors that cannot fail gracefully throw (`getCollisionObjectsTransform`, for one, throws `std::out_of_range`). Spell an "unset" id of your own as `LinkId{}` and test it with `isValid()`.

**Test-only escape hatch.** `NameIdTestAccess::create<IdT>` constructs an id with an explicit value/name combination so unit tests can manufacture hash collisions on demand; `createReverseCanonical<IdT>` builds the pair of ids whose canonical order is the reverse of their alphabetical order, which is how a test tells the two orderings apart without depending on what real names happen to hash to. The struct is a friend of `NameId` defined only in `name_id_testing.h`, which ships as the opt-in `tesseract::common_test_suite` component rather than from `types.h`, so production code cannot name the operation — an id whose value is not a function of its name would break the invariant every container in the framework relies on.

## Using ids

- **Construct an id once and hold it.** Hoist it out of the loop, or keep it as a member. Inline construction at a lookup site — `map.find(LinkId("base"))`, or the implicit `map.find("base")` — pays a string construction plus a hash per call, which is the cost ids exist to avoid. The precomputed-hash win is realized only across repeated lookups on the same id.
- **Convert at the boundary with `toIds` / `toNames`** (`types.h`): `toIds<LinkId>(names)` on the way in, `toNames(ids)` on the way out for display, logging, or string-based neighbors. Convert once, then hold ids rather than re-deriving them per call.
- **Reuse one pair for repeated pair-keyed lookups.** A `LinkIdPair` owns two names, so constructing one copies two strings. `assign(a, b)` re-canonicalizes in place and reuses the held ids' string capacity, so steady-state reassignment allocates nothing. The rule the core follows, and which a collision or kinematics backend should follow too: **a lookup never manufactures a `LinkIdPair`; only an insertion does.** Declare the pair above the loop and `assign` per iteration, or hold it as a member of a per-candidate callback object.

  The dividing line is whether the container *keeps* the key. `find` and `erase` take it by reference and discard it, so a reused pair is pure win. `operator[]` and `insert` store it, so a scratch forces the container to copy both names out, whereas a temporary — `lookup_table_[LinkIdPair(id1, id2)] = margin;` — lets them move in. Reaching for the scratch there is slower than the obvious spelling, not faster.
- **Read names off a pair with `orderedNameView()`**, not `first()` / `second()`, wherever the order is observable. See Serialization.
- **Iteration order of id-keyed containers is unspecified.** `operator<` orders by the cached hash, so ordered containers of ids (`std::map<LinkId, …>`, `std::set<LinkId>`, sorted vectors) work but iterate in an order that is neither alphabetical nor stable across builds. Sort by `.name()` explicitly where display order matters, and never persist an order derived from id ordering.

## Equality and hash-collision resolution

`boost::hash<std::string>` produces a `std::size_t`, so two distinct names *can* hash to the same value. The design resolves this the way a string-keyed hash map does, rather than trying to detect it:

- **Hybrid equality.** `NameId::operator==` compares the cached values first; only when the values match does it confirm with the name strings. Two colliding names therefore compare **unequal**, so every hash-keyed container keeps them as distinct keys — they land in the same bucket and are separated by the equality probe, which is precisely how `std::unordered_map<std::string, …>` handles a bucket collision. There is nothing to detect, no error path, and no way for a collision to silently alias two links.
- **Load-bearing invariant:** every valid `NameId` carries the name it was constructed from. The string constructors are the only public way to obtain a valid id, so the confirming compare always has both names available. (This is exactly the invariant `NameIdTestAccess` exists to violate — in tests only.)
- **Pairs are watertight the same way.** `OrderedIdPair` stores the two full `NameId`s, so pair equality is hybrid via the components. Canonicalization orders by value and breaks value ties by name, so `LinkIdPair(a, b) == LinkIdPair(b, a)` holds even when `a` and `b` collide.
- **Ordering is consistent.** `operator<` on both types breaks value ties by name, giving a strict weak ordering whose equivalence classes are exactly hybrid equality — safe for `std::map`/`std::set` and for sorting.

The practical consequence of a collision is a performance footnote: the colliding ids share a bucket and pay a string compare to disambiguate, exactly as colliding strings would. At realistic scene sizes (fewer than 10^4 names) the 64-bit birthday bound puts the probability of even one collision around 10^-12; on a 32-bit target the same 10^4 names collide with probability near 1%, which makes collisions occasional rather than effectively never. Correctness is unaffected either way, and the cost is one confirming string compare — which equality already pays on every genuine match anyway.

## Type safety, and implicit string conversion

`LinkId` and `JointId` are *distinct types* because `LinkTag` and `JointTag` are different. A link identity cannot flow into a joint-addressing API without an explicit conversion:

```cpp
graph.getLink(some_joint_id);  // compile error — JointId is not a LinkId
```

The same tag scheme extends to `OrderedIdPair<Tag>`, so a `LinkIdPair` cannot be formed from `JointId`s by accident.

**Implicit string conversion.** `NameId(std::string)` and `NameId(const char*)` are convenience constructors. By default they are **implicit**, so tests, examples, and downstream user code can write `graph.getLink("base")` and let the string convert. The constructors are guarded by a `TESSERACT_NAMEID_EXPLICIT` macro in `types.h` that expands to `explicit` when `TESSERACT_NAMEID_NO_IMPLICIT` is defined and to nothing otherwise.

Each of Tesseract's own production libraries adds `TESSERACT_NAMEID_NO_IMPLICIT` as a **PRIVATE** compile definition, so the constructors are explicit only in the libraries' own translation units. Inside Tesseract source files, any string-to-id construction must be spelled out — `LinkId("base")` rather than `"base"` — which surfaces every internal pathway still going through strings, and is what makes the link/joint separation above bite rather than be bypassed by the implicit conversion. Tests, examples, benchmarks, and downstream consumers keep the convenience, and the public API remains string-friendly.

That convenience is meant to be used. In Tesseract's own tests, examples and benchmarks, construct ids implicitly from literals — `std::vector<JointId> joint_ids = { "joint_1", "joint_2" };` — rather than spelling out `JointId("joint_1")`. The explicit form is the *diagnostic* the production libraries opt into, not a house style; writing it in a TU that does not define `TESSERACT_NAMEID_NO_IMPLICIT` only adds noise.

**Downstream libraries can opt in, and should if they want the type check.** The macro is not Tesseract-private: a downstream library that defines `TESSERACT_NAMEID_NO_IMPLICIT` as a PRIVATE compile definition on its own targets gets the same diagnostic in its own translation units. Because the toggle is per translation unit and affects overload resolution only, it can be adopted one target at a time, and consumers of that library are unaffected — including consumers that do not define it themselves.

**What the toggle does not buy, in either setting.** The gate is per translation unit, not per argument, so in a permissive TU a braced list can mix a `std::string` variable in among literals — `{ name_var, "j2" }` — and it converts. Overload resolution cannot distinguish a literal from a variable of the same type, which is why the macro is a diagnostic a TU opts into wholesale rather than a property of the id types: turning it on catches the variable case along with everything else, and leaving it off catches neither.

`NameId` is a class template, so its constructors are inline and instantiated per TU and the check is compile-time only. Formally the differing token sequences across TUs are an ODR violation (ill-formed, no diagnostic required) — a deliberate one, benign for as long as `explicit` affects overload resolution only and not layout or codegen, so that all instantiations produce identical object code. The condition that keeps it benign is that no header may branch on whether the constructors are convertible; a trait that detects convertibility would make the difference observable and the violation real. C++20 modules would reject the scheme outright, so the toggle would have to be retired if Tesseract ever adopts them.

The CMake plumbing lives in `common/cmake/tesseract_macros.cmake` (variable `TESSERACT_COMPILE_DEFINITIONS_PRIVATE`) and is applied via `target_compile_definitions(<lib> PRIVATE ${TESSERACT_COMPILE_DEFINITIONS_PRIVATE})` on each production library. New libraries added to Tesseract should follow the same pattern.

## Serialization and interoperability

**Hash values are runtime-only.** They are deterministic within a single process, but **not** stable across builds, boost versions, or architectures. They are never persisted, and never leave the process. A value in a ROS message, a shared-memory segment, or a cache shared between processes fails the same way one on disk does — the far side may have hashed the same name differently. `NameId::value()` is a cache, not a key; `tesseract_rosutils` converts to names at the message boundary for exactly this reason.

The on-disk schemas are unchanged, and existing serialized data loads without conversion. This holds because the id types serialize *as their names*, once per format, rather than each container converting itself back to strings:

- **Cereal** — `NameId` has `save_minimal` / `load_minimal` returning and taking the bare name, so an id occupies exactly the string field its name occupied. `OrderedIdPair` has `save` / `load` that hand the two names to cereal's own `std::pair` serializer, so a pair key writes `"first"` / `"second"` NVPs. Id-keyed containers are therefore archived directly, with no string-keyed intermediate, and the ids are recomputed from the names on load. See `common/include/tesseract/common/cereal_serialization.h`.
- **YAML** — `NameId` converts to and from a bare name scalar; `OrderedIdPair` converts to and from a two-element sequence of names. See `common/include/tesseract/common/yaml_extensions.h`.
- **URDF / SRDF** remain string-based on disk.

**Where a pair's order is observable, it is alphabetical — never canonical.** Canonicalization orders a pair by hash value, which is a runtime property: which of the two names lands in `first()` is a function of the hash, so it can change with the boost version and is not reproducible across builds. Every path where the order leaves the process — archives, YAML, SRDF output, log lines, table rows — therefore reads the names through `OrderedIdPair::orderedNameView()`, which returns them alphabetically. That makes each pair render reproducibly across builds. It does not make a whole file byte-identical: the *order of entries* within a container still follows hash-keyed iteration, which is unspecified. Sort explicitly where a stable file order matters, as SRDF's `<disable_collisions>` output does.

For the on-disk formats the invariant is owned in one place per format rather than by each caller: the cereal `save` and the YAML `encode` above call `orderedNameView()` once, so every container keyed on a pair inherits the ordering without restating it. Everywhere else — logs, UI models, table builders — it is a call-site rule with nothing to enforce it, since those sites have no shared choke point to own it. That is the cost of ordering by value: the accessor makes the correct spelling available and obvious, but a new writer can still reach for `first()` and get output that is merely arbitrary rather than wrong.

`orderedNameView()` returns references into the pair and is deleted on rvalues; `assign()` invalidates a view taken before it.

## Hashing, and the cost model

**Hashing.** `std::hash` is specialized for both id types in `types.h`, so they work as keys in `std::unordered_map`/`set` without spelling a hasher. Boost hashing lives in a separate opt-in header, `common/include/tesseract/common/types_boost_hash.h`: include it to hash an id with `boost::hash`, `boost::hash_combine`, or boost's unordered containers. Keeping it separate is what lets `types.h` — the most widely included header in the framework — stay free of boost includes.

The split is also load-bearing for correctness, which is why the `hash_value` ADL hooks live in that header rather than in `types.h`. Boost's open-addressing containers consult `hash_is_avalanching` at instantiation to decide whether to remix the hash before using it. The pair hash *is* avalanching — it is `boost::hash_combine` output regardless of what went in — so the header declares it and the containers skip a redundant mix. Were the hooks reachable without that declaration, one translation unit could instantiate a container with the remix and another instantiate the same type without it, and the two would disagree about which bucket a key belongs in. Bundling the hooks with the trait makes that unreachable: boost hashing and the declaration arrive together or not at all. (The trait reached boost in 1.81 and is compiled in conditionally; on older boost the containers that consult it do not exist yet.)

`NameId` is declared alongside it. Its value is `boost::hash<std::string>` output, which boost itself declares avalanching, and which — unlike `std::hash<std::string>` — is boost's own implementation and therefore identical on every platform rather than Murmur-derived on libstdc++ and FNV-1a on MSVC.

**What a lookup costs.** With `NameId` keys:

- `std::hash<NameId>` returns the cached value — a string hash computed once instead of per lookup
- a probe that *misses* short-circuits on the value compare and never touches the strings
- a probe that *hits* pays one confirming string compare, the price of watertight equality
- constructing a lookup key is free if you already hold a `NameId`

Hash computation is front-loaded at construction and then amortized over every subsequent lookup. For structures like `SceneGraph::link_map_`, `SceneGraph::joint_map_`, `SceneState::joints`, `LinkIdTransformMap`, and the ACM, that ratio is very favorable.

**Pair keys** need a pair key regardless of representation, so the only question is what it is made of. `LinkIdPair` carries the two full names — they are what make pair equality watertight — but moves the hashing off the lookup path: canonicalization compares integer values (name only on a value tie), the combined hash is computed by `combineNameIdHash` when the pair is built, and every lookup just returns that cached `std::size_t`, short-circuiting on the two integer compares and touching the strings only on a genuine hit. Note that "built once" describes the **key**, not necessarily the **query**: callers holding a long-lived pair, or `assign`ing into a scratch one, pay the mix once, but callers that construct a pair per lookup — `isContactAllowed`, `getCollisionMargin`, coal's `contactTest` — pay it per lookup, and what those save is the two `O(len)` name hashes and the per-probe string compares.

**What it costs.** `sizeof(NameId)` is larger than `sizeof(std::string)` by the inline 8-byte hash, and `LinkIdPair` is ~88 bytes because it carries both names. No allocation is added — the retained name is the same heap allocation a name string always made — and in aggregate the footprint growth is small: `ContactResult` goes from 768 to 784 bytes (+2.1%). Pair keys are the wider growth (88 vs 64 bytes) but scale with colliding link *pairs*, not contact points, since `ContactResultMap` stores a `ContactResultVector` per key. Pair construction copies two strings, which is why hot loops should construct a pair once and `assign` into it. Sizes are libstdc++/x86-64.

Ids are not a universal win. Constructing a `NameId` is *slightly* more expensive than constructing a `std::string` because it hashes, so code that builds an identity and never looks anything up pays without benefit. Parse-heavy paths (URDF/SRDF/YAML load) are dominated by parsing itself, so the extra hash is in the noise.

**What to expect end-to-end.** The win scales with how identity-lookup-bound the workload is. Collision-heavy query paths improve measurably — an isolated transform-lookup path by 44–62%, dense-scene discrete collision by 10–27% on Coal — while end-to-end planning is unchanged, because collision is a small fraction of SQP time. Ids make identity stop showing up in profiles; they do not make the rest of the pipeline faster.

## Requirements

The design is easiest to judge against the properties string identity provided for free. These are what any replacement had to preserve, and the yardstick for the alternatives below:

- **Free-standing construction.** An identity must be computable from a name alone — in a test, a parser, an offline tool — before any `Environment` exists and without consulting a registry.
- **Stability across model boundaries.** Environments are cloned per planning thread, contact results produced against one state are inspected against a later one, and ACM/margin entries are authored against names in SRDF before a model is loaded. Identity must survive all of these boundaries unchanged.
- **A mutable model.** Links and joints are added and removed at runtime via Commands. Identity held by callers must not be invalidated by mutation of the scene graph.
- **Watertight correctness.** Two distinct names must never alias, under any input — the same guarantee string keys give, with no probabilistic caveat.
- **A string-friendly boundary.** Public APIs stay callable with strings, error messages stay readable, and on-disk formats stay unchanged.

A name-derived hash satisfies the first three by construction: it is a pure function of the name, so it can be computed anywhere, agrees across models, and is untouched by scene mutation. The rest of the design — hybrid equality, the retained name — exists to satisfy the last two.

## Alternatives considered

Each candidate is scored against the requirements above, plus whether it **delivers the win** that motivated the change at all: amortized lookup cost and link/joint type safety. Two candidates fail no requirement and are rejected on other grounds; they are the last two rows.

| Alternative | Free-standing | Mutable model | Cross-model | Watertight | Delivers the win |
|---|:---:|:---:|:---:|:---:|:---:|
| **Name hash + retained name** (chosen) | ✓ | ✓ | ✓ | ✓ | ✓ |
| Dense per-model integer indices | ✗ | ✗ | ✗ | ✓ | ✓ |
| Randomly-generated 128-bit UUIDs | ✗ | ✓ | ✓ | ✓ | ✓ |
| Name-based UUIDs (v5) | ✓ | ✓ | ✓ | ✓ | ✓ |
| Construction-time interning registry | ✗ | ✓ | ✗ | ✓ | ✓ |
| Value-only equality + collision *detection* | ✓ | ✓ | ✓ | ✗ | ✓ |
| A pinned hash function | ✓ | ✓ | ✓ | ✓ | ✓ |
| Heterogeneous string lookup (`is_transparent`) | ✓ | ✓ | ✓ | ✓ | ✗ |

**Dense per-model integer indices** — the approach used by MoveIt, Pinocchio, and RBDL: the model assigns each link and joint a small contiguous index at load time and per-link data lives in plain vectors. Where it applies this is the strongest design in the space — no hashing, no hash maps, no collisions, pair keys are two ints — but it presupposes a fixed model, and Tesseract's central object is a mutable one. An index is meaningless without the model that assigned it; removing a link either invalidates every index above it or forces tombstones and a free-list, at which point the indices are no longer dense and the vectors lose the properties that made them the win; and per-model indices survive neither cloning nor SRDF authored before load. For a fixed-topology inner loop built on top of Tesseract, dense indices remain a valid caller-side optimization; they could not be the identity type of the core API.

**Randomly-generated 128-bit UUIDs** require a registry to assign and persist a name → UUID mapping, so `LinkId("base")` could not be a value computable in a test, parser, or offline tool.

**Name-based UUIDs** (UUIDv5: SHA-1 of namespace + name) are essentially a 128-bit hash and pass every requirement, but buy nothing for it. `std::hash` returns `std::size_t`, so a 128-bit value is reduced to a 64-bit bucket index for the actual lookup and bucket-level distribution is identical to plain 64-bit hashing; `value()` is no longer identity-hashable; per-key storage grows; and SHA-1 at construction is materially more expensive than `boost::hash<std::string>`. Most importantly, a wider hash buys nothing correctness-wise, because hybrid equality already makes collisions harmless.

**A construction-time interning registry** (probe on collision, guaranteeing process-wide value↔name injectivity) fails the same registry test and adds unbounded growth, since entries can never be evicted without reference-counting every live `NameId`. Two further hazards would remain even where a registry were acceptable. The table has to be exactly one instance across every shared library, and two copies — static linking, a plugin loaded `RTLD_LOCAL`, differing symbol visibility — hand out distinct pointers for the same name; that is merely wasteful until the confirming compare is reduced to a pointer compare, which is precisely the optimization interning exists to enable, and from then on ids that should be equal compare unequal on some build configurations only. Construction also stops being lock-free, mutating shared state on a path parallel planning enters concurrently.

**Collision detection instead of resolution** — keep value-only equality and handle collisions by *detecting* them where ids enter containers, with scene-graph inserts throwing on a name mismatch. Detection is inherently partial where resolution is total: it fires only when both colliding names reach the same container through a checked path, so bulk-replace and assignment adopt data unverified and the checked-write idiom is a convention new code can silently skip. It also adds a failure mode (throw on insert) that string keys never had. Hybrid equality is a property of the key type itself, so every container — present and future, core or downstream — is watertight by construction, with no error path and no coverage to audit.

**A pinned hash function** — FNV-1a, xxHash, or a vendored mixer — fails no requirement. Half of what it offers is already in hand, since boost's hash is boost's own implementation and therefore identical across platforms and standard libraries; what pinning adds is stability across boost *versions*, which replaced their string hash wholesale in 1.81. Rejected because that stability is a hazard rather than a feature: nothing in the design needs a persistable value — names are the wire format and every serializer writes them — while a value guaranteed stable would invite exactly the coupling this design avoids, with on-disk data, message fields, or cache keys addressed by hash instead of by name, unpicked afterwards only by a format migration.

**Heterogeneous string lookup** (transparent comparators/hashers, so lookups accept `std::string_view` without allocating a temporary key) is the cheap, orthogonal fix for one specific cost, and fails no requirement — but it does not deliver. Every probe still pays the `O(len)` string hash plus a string compare per bucket candidate, which are the costs profiling flagged, and it offers nothing for link/joint type safety or for pair keys. It is also unavailable here: Tesseract is C++17, and heterogeneous lookup for the *unordered* containers arrived in C++20. It is complementary rather than competing — the same machinery works with `NameId` keys wherever a string-keyed boundary remains.

### Sub-decisions

**Alphabetical pair canonicalization.** `OrderedIdPair` could order by name instead of by hash value, which would retire `orderedNameView()`, its rvalue-delete, and its invalidation caveat — every call site could then read `first()` / `second()` and be correct by default. Rejected because it puts an unconditional `std::string` compare in every construction *and* every `assign()`, which is the per-contact-point path `assign()` exists to keep allocation-free. Ordering by value with a name tiebreak keeps the comparison integral in the overwhelming majority of cases; the residual cost is that the ordering rule under Serialization has no choke point outside the on-disk formats.

## Quick reference

| Concern | Where |
|---|---|
| Type definitions, helpers, `std::hash` | `common/include/tesseract/common/types.h` (header-only) |
| Boost hashing support (opt-in) | `common/include/tesseract/common/types_boost_hash.h` |
| Where names become ids in the core | `scene_graph/src/graph.cpp` (`addLinkHelper`, `addJointHelper`) |
| YAML converters | `common/include/tesseract/common/yaml_extensions.h` |
| Cereal serializers (ids archive as names) | `common/include/tesseract/common/cereal_serialization.h` |
| Alphabetical order for observable pair output | `OrderedIdPair::orderedNameView` (`types.h`) |
| The `TESSERACT_NAMEID_NO_IMPLICIT` plumbing | `common/cmake/tesseract_macros.cmake` |
| Test-only id construction | `common/test_suite/include/tesseract/common/test_suite/name_id_testing.h` |

Porting call sites from the string-based API is covered separately in `IDENTITY_MIGRATION.md`.
