#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <iostream>
#include <fstream>
TESSERACT_COMMON_IGNORE_WARNINGS_POP
#include <tesseract_scene_graph/utils.h>

/// Testing getAllowedCollisions
TEST(TesseractSceneGraphUnit, TestGetAllowedCollisions)  // NOLINT
{
  tesseract_scene_graph::AllowedCollisionMatrix acm;

  acm.addAllowedCollision("link1", "link2", "test");
  acm.addAllowedCollision("link4", "link3", "test");
  acm.addAllowedCollision("link1", "link3", "test");
  acm.addAllowedCollision("link1", "cause_duplicate", "test");
  acm.addAllowedCollision("link10", "link11", "test");
  acm.addAllowedCollision("dummy", "dummy", "link1");

  std::vector<std::string> link_names{ "link1", "link3", "cause_duplicate" };

  // Removing duplicates: Expect link1, link2, link3, link4, cause_duplicate
  {
    auto results = tesseract_scene_graph::getAllowedCollisions(link_names, acm.getAllAllowedCollisions());
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
    auto results = tesseract_scene_graph::getAllowedCollisions(link_names, acm.getAllAllowedCollisions(), false);
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

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
