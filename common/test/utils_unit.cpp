#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <iostream>
#include <fstream>
TESSERACT_COMMON_IGNORE_WARNINGS_POP
#include <tesseract/common/utils.h>

/// Testing getAllowedCollisions
TEST(TesseractCommonUtilsUnit, TestGetAllowedCollisions)  // NOLINT
{
  tesseract::common::AllowedCollisionMatrix acm;

  acm.addAllowedCollision("link1", "link2", "test");
  acm.addAllowedCollision("link4", "link3", "test");
  acm.addAllowedCollision("link1", "link3", "test");
  acm.addAllowedCollision("link1", "cause_duplicate", "test");
  acm.addAllowedCollision("link10", "link11", "test");
  acm.addAllowedCollision("dummy", "dummy", "link1");

  std::vector<std::string> link_names{ "link1", "link3", "cause_duplicate" };

  // Removing duplicates: Expect link1, link2, link3, link4, cause_duplicate
  {
    auto results = tesseract::common::getAllowedCollisions(link_names, acm.getAllAllowedCollisions());
    EXPECT_TRUE(std::find(results.begin(), results.end(), "link1") != results.end());
    EXPECT_TRUE(std::find(results.begin(), results.end(), "link2") != results.end());
    EXPECT_TRUE(std::find(results.begin(), results.end(), "link3") != results.end());
    EXPECT_TRUE(std::find(results.begin(), results.end(), "link4") != results.end());
    EXPECT_TRUE(std::find(results.begin(), results.end(), "cause_duplicate") != results.end());
    EXPECT_FALSE(std::find(results.begin(), results.end(), "link10") != results.end());
    EXPECT_FALSE(std::find(results.begin(), results.end(), "link11") != results.end());
    EXPECT_FALSE(std::find(results.begin(), results.end(), "dummy") != results.end());
    EXPECT_EQ(results.size(), 5);
  }
  // Not removing duplicates: Expect link1, link2, link3, link4, cause_duplicate, link1
  {
    auto results = tesseract::common::getAllowedCollisions(link_names, acm.getAllAllowedCollisions(), false);
    EXPECT_TRUE(std::find(results.begin(), results.end(), "link1") != results.end());
    EXPECT_TRUE(std::find(results.begin(), results.end(), "link2") != results.end());
    EXPECT_TRUE(std::find(results.begin(), results.end(), "link3") != results.end());
    EXPECT_TRUE(std::find(results.begin(), results.end(), "link4") != results.end());
    EXPECT_TRUE(std::find(results.begin(), results.end(), "cause_duplicate") != results.end());
    EXPECT_FALSE(std::find(results.begin(), results.end(), "link10") != results.end());
    EXPECT_FALSE(std::find(results.begin(), results.end(), "link11") != results.end());
    EXPECT_FALSE(std::find(results.begin(), results.end(), "dummy") != results.end());
    EXPECT_EQ(results.size(), 6);
  }
}

/// Testing the LinkId-based getAllowedCollisions overload (Id-primary; string overload delegates).
TEST(TesseractCommonUtilsUnit, GetAllowedCollisionsLinkIdOverload)  // NOLINT
{
  using namespace tesseract::common;
  AllowedCollisionMatrix acm;
  acm.addAllowedCollision("a", "b", "test");
  acm.addAllowedCollision("a", "c", "test");
  acm.addAllowedCollision("d", "e", "test");

  // Query for "a": expect {b, c}.
  {
    std::vector<LinkId> query{ LinkId("a") };
    auto results = getAllowedCollisions(query, acm.getAllAllowedCollisions(), /*remove_duplicates=*/true);
    ASSERT_EQ(results.size(), 2U);
    std::set<std::string> result_names;
    for (const auto& id : results)
      result_names.insert(id.name());
    EXPECT_EQ(result_names, (std::set<std::string>{ "b", "c" }));

    // String overload returns the same set of names (delegation).
    std::vector<std::string> str_query{ "a" };
    auto str_results = getAllowedCollisions(str_query, acm.getAllAllowedCollisions(), true);
    std::set<std::string> str_set(str_results.begin(), str_results.end());
    EXPECT_EQ(str_set, result_names);
  }

  // Query both endpoints of the same entry: with dedup, neither should be reported twice.
  // For entry (a,b) with query {a,b}: first branch emits b (via a), second branch emits a (via b).
  // Both are unique, so both appear; but neither appears more than once.
  {
    std::vector<LinkId> query{ LinkId("a"), LinkId("b") };
    auto results = getAllowedCollisions(query, acm.getAllAllowedCollisions(), /*remove_duplicates=*/true);
    std::set<std::string> result_names;
    for (const auto& id : results)
      result_names.insert(id.name());
    // Entries (a,b) yields {a,b}; (a,c) yields {c} (only a is in query); (d,e) yields {} → set {a,b,c}.
    EXPECT_EQ(result_names, (std::set<std::string>{ "a", "b", "c" }));
    EXPECT_EQ(results.size(), result_names.size());  // No duplicates.

    // Without dedup the count must equal raw branch hits: (a,b)→2, (a,c)→1, (d,e)→0 = 3.
    auto results_no_dedup = getAllowedCollisions(query, acm.getAllAllowedCollisions(), /*remove_duplicates=*/false);
    EXPECT_EQ(results_no_dedup.size(), 3U);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
