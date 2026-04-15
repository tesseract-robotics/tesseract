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
#include <cereal/archives/xml.hpp>
#include <sstream>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/common/types.h>
#include <tesseract/common/cereal_serialization.h>

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

TEST(NameIdTest, ZeroGuard)  // NOLINT
{
  // Empty name returns the invalid sentinel, not a hashed ID
  const LinkId empty_id = LinkId("");
  EXPECT_FALSE(empty_id.isValid());
  EXPECT_EQ(empty_id.value(), 0U);
  EXPECT_TRUE(empty_id.name().empty());

  // Non-empty names always produce a valid (non-zero) ID even if hash happens to be 0
  const LinkId valid_id = LinkId("any_link");
  EXPECT_TRUE(valid_id.isValid());
  EXPECT_NE(valid_id.value(), 0U);
}

// ======================== Name accessor ========================

TEST(NameIdTest, NameAccessor)  // NOLINT
{
  const LinkId id = LinkId("test_link");
  EXPECT_EQ(id.name(), "test_link");
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

// ======================== Hash ========================

TEST(NameIdTest, IdentityHash)  // NOLINT
{
  const LinkId id = LinkId("test_link");
  LinkId::Hash hasher;
  EXPECT_EQ(hasher(id), id.value());
}

TEST(NameIdTest, StdHashSpecialization)  // NOLINT
{
  const LinkId id = LinkId("test_link");
  std::hash<LinkId> hasher;
  EXPECT_EQ(hasher(id), id.value());
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

// ======================== LinkIdPair ========================

TEST(LinkIdPairTest, MakeCanonical)  // NOLINT
{
  const LinkId a = LinkId("link_a");
  const LinkId b = LinkId("link_b");
  const LinkIdPair ab = LinkIdPair::make(a, b);
  const LinkIdPair ba = LinkIdPair::make(b, a);
  EXPECT_EQ(ab, ba);
  // Canonical ordering: first.value <= second.value
  EXPECT_LE(ab.first.value(), ab.second.value());
}

TEST(LinkIdPairTest, Equality)  // NOLINT
{
  const LinkIdPair p1 = LinkIdPair::make(LinkId("x"), LinkId("y"));
  const LinkIdPair p2 = LinkIdPair::make(LinkId("y"), LinkId("x"));
  const LinkIdPair p3 = LinkIdPair::make(LinkId("x"), LinkId("z"));
  EXPECT_EQ(p1, p2);
  EXPECT_NE(p1, p3);
}

TEST(LinkIdPairTest, HashWorksInUnorderedMap)  // NOLINT
{
  std::unordered_map<LinkIdPair, int, LinkIdPair::Hash> map;
  const LinkIdPair key = LinkIdPair::make(LinkId("a"), LinkId("b"));
  map[key] = 42;

  // Lookup with reversed order should find same entry
  const LinkIdPair reversed = LinkIdPair::make(LinkId("b"), LinkId("a"));
  EXPECT_EQ(map.at(reversed), 42);
}

TEST(LinkIdPairTest, SameLinkPair)  // NOLINT
{
  const LinkId a = LinkId("self");
  const LinkIdPair pair = LinkIdPair::make(a, a);
  EXPECT_EQ(pair.first, pair.second);
}

TEST(NameIdTest, LinkIdPairPreservesNames)  // NOLINT
{
  const LinkId a = LinkId("link_a");
  const LinkId b = LinkId("link_b");
  const LinkIdPair pair = LinkIdPair::make(b, a);
  // After canonical ordering, names should be preserved
  EXPECT_FALSE(pair.first.name().empty());
  EXPECT_FALSE(pair.second.name().empty());
}

// ======================== Cereal serialization ========================

TEST(NameIdTest, CerealRoundTripValid)  // NOLINT
{
  const LinkId original = LinkId("test_link");

  std::string xml;
  {
    std::ostringstream oss;
    {
      cereal::XMLOutputArchive ar(oss);
      ar(cereal::make_nvp("id", original));
    }
    xml = oss.str();
  }

  LinkId loaded{};
  {
    std::istringstream iss(xml);
    cereal::XMLInputArchive ar(iss);
    ar(cereal::make_nvp("id", loaded));
  }

  EXPECT_EQ(original, loaded);
  EXPECT_EQ(original.name(), loaded.name());
  EXPECT_TRUE(loaded.isValid());
}

TEST(NameIdTest, CerealRoundTripInvalidSentinel)  // NOLINT
{
  const LinkId original{};  // default-constructed = invalid
  EXPECT_FALSE(original.isValid());

  std::string xml;
  {
    std::ostringstream oss;
    {
      cereal::XMLOutputArchive ar(oss);
      ar(cereal::make_nvp("id", original));
    }
    xml = oss.str();
  }

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

// ======================== Implicit constructor tests ========================

TEST(NameIdTest, ConstructorFromString)  // NOLINT
{
  auto id = LinkId(std::string("test_link"));
  EXPECT_TRUE(id.isValid());
  EXPECT_EQ(id.name(), "test_link");
  EXPECT_NE(id.value(), 0U);
}

TEST(NameIdTest, ConstructorFromCharStar)  // NOLINT
{
  auto id = LinkId("literal_link");
  EXPECT_TRUE(id.isValid());
  EXPECT_EQ(id.name(), "literal_link");
  EXPECT_EQ(id, LinkId(std::string("literal_link")));
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
  EXPECT_EQ(jid.name(), "test_joint");
  // JointId and LinkId with same name should have same hash but different types
  auto lid = LinkId("test_joint");
  EXPECT_EQ(jid.value(), lid.value());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
