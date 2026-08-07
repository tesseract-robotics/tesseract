/**
 * @file name_id_unit.cpp
 * @brief Unit tests for NameId<Tag>, LinkId, JointId, and LinkIdPair types
 *
 * @author Roelof Oomen
 * @date April 9, 2026
 */

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <unordered_set>
#include <unordered_map>
#include <map>
#include <cereal/archives/xml.hpp>
#include <sstream>
#include <boost/container_hash/hash.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/common/types.h>
#include <tesseract/common/types_boost_hash.h>
#include <tesseract/common/cereal_serialization.h>
#include <tesseract/common/serialization.h>
#include <tesseract/common/test_suite/name_id_testing.h>

using namespace tesseract::common;

// ======================== NameId basics ========================

TEST(NameIdTest, FromNameDeterministic)  // NOLINT
{
  const LinkId a1 = LinkId("link_a");
  const LinkId a2 = LinkId("link_a");
  EXPECT_EQ(a1, a2);
  EXPECT_EQ(a1.value(), a2.value());
}

TEST(NameIdTest, DifferentNamesDifferentIds)  // NOLINT
{
  const LinkId a = LinkId("link_a");
  const LinkId b = LinkId("link_b");
  EXPECT_NE(a, b);
}

TEST(NameIdTest, IsValid)  // NOLINT
{
  const LinkId valid = LinkId("something");
  EXPECT_TRUE(valid.isValid());

  const LinkId invalid{};
  EXPECT_FALSE(invalid.isValid());
  EXPECT_EQ(invalid.value(), 0U);
}

TEST(NameIdTest, SentinelIsInvalid)  // NOLINT
{
  EXPECT_FALSE(INVALID_LINK_ID.isValid());
  EXPECT_EQ(INVALID_LINK_ID.value(), 0U);
  EXPECT_FALSE(INVALID_JOINT_ID.isValid());
  EXPECT_EQ(INVALID_JOINT_ID.value(), 0U);
}

TEST(NameIdTest, ValidityIsCarriedByTheName)  // NOLINT
{
  // An empty name yields the invalid id; it is not hashed.
  const LinkId empty_id = LinkId("");
  EXPECT_FALSE(empty_id.isValid());
  EXPECT_EQ(empty_id.value(), 0U);
  EXPECT_TRUE(empty_id.name().empty());

  // Zero is not a reserved value. A name hashing onto it still gives a valid id, and that id stays
  // distinct from the invalid one because equality confirms the names.
  const auto zero_valued = NameIdTestAccess::create<LinkId>(0, "hashes_to_zero");
  EXPECT_TRUE(zero_valued.isValid());
  EXPECT_EQ(zero_valued.value(), 0U);
  EXPECT_NE(zero_valued, INVALID_LINK_ID);
}

// ======================== Name accessor ========================

TEST(NameIdTest, NameAccessor)  // NOLINT
{
  const LinkId id = LinkId("test_link");
  EXPECT_EQ(id, "test_link");
}

TEST(NameIdTest, DefaultConstructedHasEmptyName)  // NOLINT
{
  const LinkId id{};
  EXPECT_TRUE(id.name().empty());
  EXPECT_FALSE(id.isValid());
}

TEST(NameIdTest, InvalidIdHasEmptyName)  // NOLINT
{
  EXPECT_TRUE(INVALID_LINK_ID.name().empty());
  EXPECT_TRUE(INVALID_JOINT_ID.name().empty());
}

// ======================== Type safety ========================

TEST(NameIdTest, LinkIdAndJointIdAreDistinctTypes)  // NOLINT
{
  // Static asserts verify compile-time type safety
  static_assert(!std::is_same_v<LinkId, JointId>, "LinkId and JointId must be distinct types");
  static_assert(!std::is_same_v<LinkTag, JointTag>, "LinkTag and JointTag must be distinct types");

  // Same name produces same numeric value but different types
  const LinkId link = LinkId("foo");
  const JointId joint = JointId("foo");
  EXPECT_EQ(link.value(), joint.value());
}

TEST(NameIdTest, NothrowOperations)  // NOLINT
{
  // Move operations must stay nothrow or std::vector falls back to copying on growth.
  static_assert(std::is_nothrow_move_constructible_v<LinkId>, "LinkId move must not throw");
  static_assert(std::is_nothrow_move_assignable_v<LinkId>, "LinkId move-assign must not throw");
  static_assert(std::is_nothrow_move_constructible_v<LinkIdPair>, "LinkIdPair move must not throw");
  static_assert(std::is_nothrow_move_assignable_v<LinkIdPair>, "LinkIdPair move-assign must not throw");

  // Hashing a name and taking ownership of it cannot throw; building the string can.
  static_assert(std::is_nothrow_constructible_v<LinkId, std::string>, "hashing an owned name must not throw");
}

// ======================== Hash ========================

TEST(NameIdTest, StdHashSpecialization)  // NOLINT
{
  const LinkId id = LinkId("test_link");
  std::hash<LinkId> hasher;
  EXPECT_EQ(hasher(id), id.value());
}

TEST(NameIdTest, BoostHashAdlHook)  // NOLINT
{
  // Calling hash_value unqualified exercises ADL — the same lookup boost::hash and
  // boost::unordered_flat_map use to find the hook for NameId.
  const LinkId id = LinkId("test_link");
  EXPECT_EQ(hash_value(id), id.value());
}

TEST(NameIdTest, WorksInUnorderedSet)  // NOLINT
{
  std::unordered_set<LinkId> ids;
  ids.insert(LinkId("a"));
  ids.insert(LinkId("b"));
  ids.insert(LinkId("a"));  // duplicate
  EXPECT_EQ(ids.size(), 2U);
  EXPECT_TRUE(ids.count(LinkId("a")) == 1);
  EXPECT_TRUE(ids.count(LinkId("b")) == 1);
  EXPECT_TRUE(ids.count(LinkId("c")) == 0);
}

// ======================== Ordering ========================

TEST(NameIdTest, LessThan)  // NOLINT
{
  const LinkId a = LinkId("aaa");
  const LinkId b = LinkId("bbb");
  // One must be less than the other (they shouldn't be equal)
  EXPECT_TRUE((a < b) || (b < a));
  EXPECT_FALSE(a < a);
}

// ======================== Display ========================

TEST(NameIdTest, StreamInsertionWritesName)  // NOLINT
{
  std::ostringstream oss;
  oss << "link " << LinkId("base_link") << " and joint " << JointId("joint_1");
  EXPECT_EQ(oss.str(), "link base_link and joint joint_1");
}

TEST(NameIdTest, StreamInsertionInvalidIdWritesNothing)  // NOLINT
{
  std::ostringstream oss;
  oss << INVALID_LINK_ID;
  EXPECT_TRUE(oss.str().empty());
}

// ======================== LinkIdPair ========================

TEST(LinkIdPairTest, MakeCanonical)  // NOLINT
{
  const LinkId a = LinkId("link_a");
  const LinkId b = LinkId("link_b");
  const LinkIdPair ab = LinkIdPair(a, b);
  const LinkIdPair ba = LinkIdPair(b, a);
  EXPECT_EQ(ab, ba);
  // Canonical ordering: first <= second by value
  EXPECT_LE(ab.first().value(), ab.second().value());
}

TEST(LinkIdPairTest, Equality)  // NOLINT
{
  const LinkIdPair p1 = LinkIdPair(LinkId("x"), LinkId("y"));
  const LinkIdPair p2 = LinkIdPair(LinkId("y"), LinkId("x"));
  const LinkIdPair p3 = LinkIdPair(LinkId("x"), LinkId("z"));
  EXPECT_EQ(p1, p2);
  EXPECT_NE(p1, p3);
}

TEST(LinkIdPairTest, HashWorksInUnorderedMap)  // NOLINT
{
  std::unordered_map<LinkIdPair, int> map;
  const LinkIdPair key = LinkIdPair(LinkId("a"), LinkId("b"));
  map[key] = 42;

  // Lookup with reversed order should find same entry
  const LinkIdPair reversed = LinkIdPair(LinkId("b"), LinkId("a"));
  EXPECT_EQ(map.at(reversed), 42);
}

TEST(LinkIdPairTest, BoostHashAdlHook)  // NOLINT
{
  // Calling hash_value unqualified exercises ADL — the same lookup boost::hash and
  // boost::unordered_flat_map use to find the hook for OrderedIdPair.
  const LinkIdPair pair = LinkIdPair(LinkId("a"), LinkId("b"));
  EXPECT_EQ(hash_value(pair), pair.hash());
}

#ifdef TESSERACT_COMMON_HAS_HASH_IS_AVALANCHING
TEST(LinkIdPairTest, BoostHashIsAvalanching)  // NOLINT
{
  // boost's open-addressing containers consult this trait to decide whether to remix the hash.
  static_assert(boost::unordered::hash_is_avalanching<boost::hash<LinkIdPair>>::value,
                "LinkIdPair's boost hash is declared avalanching");
  static_assert(boost::unordered::hash_is_avalanching<boost::hash<LinkId>>::value,
                "LinkId's boost hash is declared avalanching");

  const LinkIdPair pair = LinkIdPair(LinkId("a"), LinkId("b"));
  EXPECT_EQ(boost::hash<LinkIdPair>{}(pair), pair.hash());
}
#endif

TEST(LinkIdPairTest, SameLinkPair)  // NOLINT
{
  const LinkId a = LinkId("self");
  const LinkIdPair pair = LinkIdPair(a, a);
  EXPECT_EQ(pair.first(), pair.second());
}

TEST(LinkIdPairTest, CanonicalOrdering)  // NOLINT
{
  const LinkId a = LinkId("link_a");
  const LinkId b = LinkId("link_b");
  const LinkIdPair ab = LinkIdPair(a, b);
  const LinkIdPair ba = LinkIdPair(b, a);
  EXPECT_EQ(ab.first(), ba.first());
  EXPECT_EQ(ab.second(), ba.second());
  EXPECT_LE(ab.first().value(), ab.second().value());
}

TEST(LinkIdPairTest, AssignMatchesConstructor)  // NOLINT
{
  const LinkId a = LinkId("link_a");
  const LinkId b = LinkId("link_b");
  const LinkIdPair constructed = LinkIdPair(a, b);

  LinkIdPair assigned;
  assigned.assign(a, b);
  EXPECT_EQ(assigned, constructed);
  EXPECT_EQ(assigned.hash(), constructed.hash());

  assigned.assign(b, a);
  EXPECT_EQ(assigned, constructed);
  EXPECT_EQ(assigned.hash(), constructed.hash());
}

TEST(LinkIdPairTest, AssignReusesScratch)  // NOLINT
{
  // One scratch pair reassigned repeatedly, including shorter names over longer ones
  LinkIdPair scratch;
  scratch.assign(LinkId("a_rather_long_link_name_beyond_sso"), LinkId("another_rather_long_link_name"));
  EXPECT_EQ(scratch, LinkIdPair(LinkId("a_rather_long_link_name_beyond_sso"), LinkId("another_rather_long_link_name")));

  scratch.assign(LinkId("x"), LinkId("y"));
  const LinkIdPair expected = LinkIdPair(LinkId("x"), LinkId("y"));
  EXPECT_EQ(scratch, expected);
  EXPECT_EQ(scratch.hash(), expected.hash());
}

TEST(LinkIdPairTest, AssignCanonicalizesCollisionTies)  // NOLINT
{
  // Two distinct names sharing one hash value: ordering must fall back to the names
  const auto a = NameIdTestAccess::create<LinkId>(42, "link_a");
  const auto b = NameIdTestAccess::create<LinkId>(42, "link_b");

  LinkIdPair assigned;
  assigned.assign(b, a);
  EXPECT_EQ(assigned.first().name(), "link_a");
  EXPECT_EQ(assigned.second().name(), "link_b");
  EXPECT_EQ(assigned, LinkIdPair(a, b));
}

TEST(LinkIdPairTest, AssignAliasSafe)  // NOLINT
{
  const LinkId a = LinkId("link_a");
  const LinkId b = LinkId("link_b");
  const LinkId c = LinkId("link_c");

  LinkIdPair pair = LinkIdPair(a, b);
  pair.assign(pair.first(), pair.second());
  EXPECT_EQ(pair, LinkIdPair(a, b));

  pair.assign(pair.second(), pair.first());
  EXPECT_EQ(pair, LinkIdPair(a, b));

  // One-sided alias: one argument references a member, the other does not
  const LinkIdPair expected = LinkIdPair(c, pair.first());
  pair.assign(c, pair.first());
  EXPECT_EQ(pair, expected);
}

TEST(LinkIdPairTest, AssignInvalidIds)  // NOLINT
{
  LinkIdPair pair = LinkIdPair(LinkId("a"), LinkId("b"));
  pair.assign(LinkId(), LinkId());
  EXPECT_EQ(pair, LinkIdPair());
  EXPECT_EQ(pair.hash(), LinkIdPair().hash());
}

// ======================== Cereal serialization ========================

TEST(NameIdTest, CerealArchivesTheNameOnly)  // NOLINT
{
  // A NameId's entire wire form is its name. save_minimal returns a std::string, and cereal rejects
  // a type carrying both a minimal pair and a serialize/save function, so no second field can be
  // added without first removing this one. The hash value is runtime-only and must never be
  // archived: it is not stable across builds, so a reader would rebuild different ids.
  static_assert(cereal::traits::has_non_member_save_minimal<LinkId, cereal::XMLOutputArchive>::value,
                "LinkId must serialize through save_minimal");
  static_assert(!cereal::traits::has_non_member_serialize<LinkId, cereal::XMLOutputArchive>::value,
                "LinkId must not gain a serialize function; it would open room for a second field");

  const LinkId id("test_link");
  const std::string xml = Serialization::toArchiveStringXML(id, "id");
  EXPECT_NE(xml.find("test_link"), std::string::npos) << xml;
  EXPECT_EQ(xml.find(std::to_string(id.value())), std::string::npos) << "Hash value leaked into wire output: " << xml;
}

TEST(NameIdTest, CerealRoundTripValid)  // NOLINT
{
  const LinkId original = LinkId("test_link");

  const std::string xml = Serialization::toArchiveStringXML(original, "id");
  const auto loaded = Serialization::fromArchiveStringXML<LinkId>(xml, "id");

  EXPECT_EQ(original, loaded);
  EXPECT_EQ(original.name(), loaded.name());
  EXPECT_TRUE(loaded.isValid());
}

TEST(NameIdTest, CerealRoundTripInvalidSentinel)  // NOLINT
{
  const LinkId original{};  // default-constructed = invalid
  EXPECT_FALSE(original.isValid());

  const std::string xml = Serialization::toArchiveStringXML(original, "id");

  LinkId loaded = LinkId("placeholder");  // start non-default to prove overwrite
  {
    std::istringstream iss(xml);
    cereal::XMLInputArchive ar(iss);
    ar(cereal::make_nvp("id", loaded));
  }

  EXPECT_FALSE(loaded.isValid());
  EXPECT_EQ(original, loaded);
  EXPECT_TRUE(loaded.name().empty());
}

TEST(LinkIdPairTest, CerealArchivesTwoNamesLikeStdPair)  // NOLINT
{
  // A pair archives exactly as the std::pair<std::string, std::string> it replaced, so containers
  // keyed on it keep the wire format they had when keyed on names. The archive must not inherit
  // canonical order, so the pair is built with its canonical order the reverse of alphabetical.
  const auto [smaller, larger] = NameIdTestAccess::createReverseCanonical<LinkId>("link_a", "link_b");
  const LinkIdPair pair(smaller, larger);
  ASSERT_EQ(pair.first().name(), larger.name());  // the guarantee createReverseCanonical() makes

  const std::string xml = Serialization::toArchiveStringXML(pair, "pair");
  EXPECT_EQ(xml, Serialization::toArchiveStringXML(std::pair<std::string, std::string>{ "link_a", "link_b" }, "pair"));

  // Loading rebuilds the ids from the archived names, so the recovered pair carries real hash
  // values rather than the injected ones — equality against `pair` would not hold.
  const LinkIdPair rebuilt(LinkId("link_a"), LinkId("link_b"));
  EXPECT_EQ(Serialization::fromArchiveStringXML<LinkIdPair>(xml, "pair"), rebuilt);
}

TEST(LinkIdPairTest, CerealPairKeyedContainerNeedsNoHandWrittenSupport)  // NOLINT
{
  using Entries = std::unordered_map<LinkIdPair, std::string>;
  Entries original;
  original[LinkIdPair(LinkId("link_a"), LinkId("link_b"))] = "reason";

  const std::string xml = Serialization::toArchiveStringXML(original, "entries");
  EXPECT_EQ(xml.find(std::to_string(original.begin()->first.hash())), std::string::npos)
      << "Pair hash leaked into wire output: " << xml;

  EXPECT_EQ(Serialization::fromArchiveStringXML<Entries>(xml, "entries"), original);
}

// ======================== Implicit constructor tests ========================

TEST(NameIdTest, ConstructorFromString)  // NOLINT
{
  auto id = LinkId(std::string("test_link"));
  EXPECT_TRUE(id.isValid());
  EXPECT_EQ(id, "test_link");
  EXPECT_NE(id.value(), 0U);
}

TEST(NameIdTest, ConstructorFromCharStar)  // NOLINT
{
  auto id = LinkId("literal_link");
  EXPECT_TRUE(id.isValid());
  EXPECT_EQ(id, "literal_link");
  EXPECT_EQ(id, LinkId(std::string("literal_link")));
}

TEST(NameIdTest, ConstructorFromNullCharStarIsInvalid)  // NOLINT
{
  const char* null_name = nullptr;
  auto id = LinkId(null_name);
  EXPECT_FALSE(id.isValid());
  EXPECT_EQ(id.value(), 0U);
  EXPECT_TRUE(id.name().empty());
}

TEST(NameIdTest, ConstructorEmptyStringIsInvalid)  // NOLINT
{
  auto id = LinkId(std::string(""));
  EXPECT_FALSE(id.isValid());
  EXPECT_EQ(id.value(), 0U);
  EXPECT_TRUE(id.name().empty());
}

TEST(NameIdTest, ConstructorIsImplicit)  // NOLINT
{
  // Implicit conversion from std::string and const char* must work
  static_assert(std::is_convertible_v<std::string, LinkId>);
  static_assert(std::is_convertible_v<const char*, LinkId>);
  static_assert(std::is_convertible_v<std::string, JointId>);
  static_assert(std::is_convertible_v<const char*, JointId>);
}

TEST(NameIdTest, ConstructorJointId)  // NOLINT
{
  auto jid = JointId("test_joint");
  EXPECT_TRUE(jid.isValid());
  EXPECT_EQ(jid, "test_joint");
  // JointId and LinkId with same name should have same hash but different types
  auto lid = LinkId("test_joint");
  EXPECT_EQ(jid.value(), lid.value());
}

// ======================== Hybrid equality ========================

TEST(NameIdHybridEquality, CollidingNamesCompareUnequal)  // NOLINT
{
  const auto a = NameIdTestAccess::create<LinkId>(42, "link_a");
  const auto b = NameIdTestAccess::create<LinkId>(42, "link_b");
  EXPECT_EQ(a.value(), b.value());
  EXPECT_FALSE(a == b);
  EXPECT_TRUE(a != b);
  // Strict weak ordering must separate value-equal, name-different ids exactly one way.
  EXPECT_TRUE((a < b) != (b < a));
}

TEST(NameIdHybridEquality, SameNameCompareEqual)  // NOLINT
{
  const tesseract::common::LinkId a("base_link");
  const tesseract::common::LinkId b("base_link");
  EXPECT_TRUE(a == b);
  EXPECT_FALSE(a != b);
  EXPECT_FALSE(a < b);
  EXPECT_FALSE(b < a);
}

TEST(NameIdHybridEquality, CollidingIdsCoexistInUnorderedMap)  // NOLINT
{
  const auto a = NameIdTestAccess::create<LinkId>(42, "link_a");
  const auto b = NameIdTestAccess::create<LinkId>(42, "link_b");
  std::unordered_map<tesseract::common::LinkId, int> map;
  map[a] = 1;
  map[b] = 2;
  EXPECT_EQ(map.size(), 2U);
  EXPECT_EQ(map.at(a), 1);
  EXPECT_EQ(map.at(b), 2);
  EXPECT_EQ(map.erase(a), 1U);
  EXPECT_EQ(map.count(b), 1U);  // erasing one colliding key must not disturb the other
}

TEST(NameIdHybridEquality, CollidingIdsCoexistInOrderedMap)  // NOLINT
{
  const auto a = NameIdTestAccess::create<LinkId>(42, "link_a");
  const auto b = NameIdTestAccess::create<LinkId>(42, "link_b");
  std::map<tesseract::common::LinkId, int> map;  // exercises hybrid operator<
  map[a] = 1;
  map[b] = 2;
  EXPECT_EQ(map.size(), 2U);
  EXPECT_EQ(map.at(a), 1);
  EXPECT_EQ(map.at(b), 2);
}

// ======================== OrderedIdPair hybrid equality ========================

TEST(OrderedIdPairHybrid, CollidingPairsCompareUnequalAndCoexist)  // NOLINT
{
  const auto a = NameIdTestAccess::create<LinkId>(42, "link_a");
  const auto b = NameIdTestAccess::create<LinkId>(42, "link_b");
  const tesseract::common::LinkId x("some_other_link");
  const tesseract::common::LinkIdPair pa(a, x);
  const tesseract::common::LinkIdPair pb(b, x);
  EXPECT_EQ(pa.hash(), pb.hash());  // same bucket...
  EXPECT_FALSE(pa == pb);           // ...but distinguished at equality, like a hash map
  std::unordered_map<tesseract::common::LinkIdPair, int> map;
  map[pa] = 1;
  map[pb] = 2;
  EXPECT_EQ(map.size(), 2U);
  EXPECT_EQ(map.at(pa), 1);
  EXPECT_EQ(map.at(pb), 2);
}

TEST(OrderedIdPairHybrid, CanonicalizationIsArgumentOrderIndependentUnderCollision)  // NOLINT
{
  // Colliding values tie-break on name, so (a,b) and (b,a) still canonicalize identically.
  const auto a = NameIdTestAccess::create<LinkId>(42, "link_a");
  const auto b = NameIdTestAccess::create<LinkId>(42, "link_b");
  EXPECT_TRUE(tesseract::common::LinkIdPair(a, b) == tesseract::common::LinkIdPair(b, a));
  EXPECT_EQ(tesseract::common::LinkIdPair(a, b).first().name(), "link_a");
  EXPECT_EQ(tesseract::common::LinkIdPair(b, a).first().name(), "link_a");
}

TEST(OrderedIdPairHybrid, KeyCarriesNames)  // NOLINT
{
  const tesseract::common::LinkId c("link_c");
  const tesseract::common::LinkId d("link_d");
  const tesseract::common::LinkIdPair p(c, d);
  EXPECT_TRUE(p == tesseract::common::LinkIdPair(d, c));
  // The canonical slot is decided by hash value, which is not stable across builds; assert both
  // names are carried without assuming which slot each lands in.
  const std::string f = p.first().name();
  const std::string s = p.second().name();
  EXPECT_TRUE((f == "link_c" && s == "link_d") || (f == "link_d" && s == "link_c")) << "first=" << f << " second=" << s;
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
