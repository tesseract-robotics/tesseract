#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <numeric>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_collision/core/types.h>

using namespace tesseract_collision;

TEST(TesseractCollisionUnit, CollisionMarginDataUnit)  // NOLINT
{
  double tol = std::numeric_limits<double>::epsilon();

  {  // Test Default Constructor
    CollisionMarginData data;
    EXPECT_NEAR(data.getDefaultCollisionMargin(), 0, tol);
    EXPECT_NEAR(data.getMaxCollisionMargin(), 0, tol);
    EXPECT_NEAR(data.getCollisionMargin("link_1", "link_2"), 0, tol);
    EXPECT_EQ(data.getCollisionMarginPairData().getCollisionMargins().size(), 0);
  }

  {  // Test construction with non zero default distance
    double default_margin = 0.0254;
    CollisionMarginData data(default_margin);
    EXPECT_NEAR(data.getDefaultCollisionMargin(), default_margin, tol);
    EXPECT_NEAR(data.getMaxCollisionMargin(), default_margin, tol);
    EXPECT_NEAR(data.getCollisionMargin("link_1", "link_2"), default_margin, tol);
    EXPECT_EQ(data.getCollisionMarginPairData().getCollisionMargins().size(), 0);
  }

  {  // Test changing default margin
    double default_margin = 0.0254;
    CollisionMarginData data;
    data.setDefaultCollisionMargin(default_margin);
    EXPECT_NEAR(data.getDefaultCollisionMargin(), default_margin, tol);
    EXPECT_NEAR(data.getMaxCollisionMargin(), default_margin, tol);
    EXPECT_NEAR(data.getCollisionMargin("link_1", "link_2"), default_margin, tol);
    EXPECT_EQ(data.getCollisionMarginPairData().getCollisionMargins().size(), 0);
  }

  {  // Test adding pair margin larger than default
    double default_margin = 0.0254;
    double pair_margin = 0.5;
    CollisionMarginData data(default_margin);
    data.setCollisionMargin("link_1", "link_2", pair_margin);
    EXPECT_NEAR(data.getDefaultCollisionMargin(), default_margin, tol);
    EXPECT_NEAR(data.getMaxCollisionMargin(), std::max(default_margin, pair_margin), tol);
    EXPECT_NEAR(data.getCollisionMargin("link_1", "link_2"), pair_margin, tol);
    EXPECT_EQ(data.getCollisionMarginPairData().getCollisionMargins().size(), 1);
  }

  {  // Test adding pair margin less than default
    double default_margin = 0.0254;
    double pair_margin = 0.01;
    CollisionMarginData data(default_margin);
    data.setCollisionMargin("link_1", "link_2", pair_margin);
    EXPECT_NEAR(data.getDefaultCollisionMargin(), default_margin, tol);
    EXPECT_NEAR(data.getMaxCollisionMargin(), std::max(default_margin, pair_margin), tol);
    EXPECT_NEAR(data.getCollisionMargin("link_1", "link_2"), pair_margin, tol);
    EXPECT_EQ(data.getCollisionMarginPairData().getCollisionMargins().size(), 1);
  }

  {  // Test setting default larger than the current max margin
    double default_margin = 0.0254;
    double pair_margin = 0.5;
    CollisionMarginData data(default_margin);
    data.setCollisionMargin("link_1", "link_2", pair_margin);

    default_margin = 2 * pair_margin;
    data.setDefaultCollisionMargin(default_margin);
    EXPECT_NEAR(data.getDefaultCollisionMargin(), default_margin, tol);
    EXPECT_NEAR(data.getMaxCollisionMargin(), std::max(default_margin, pair_margin), tol);
    EXPECT_NEAR(data.getCollisionMargin("link_1", "link_2"), pair_margin, tol);
    EXPECT_EQ(data.getCollisionMarginPairData().getCollisionMargins().size(), 1);
  }

  {  // Test setting pair_margin larger than default and then set it lower so the max should be the default
    double default_margin = 0.0254;
    double pair_margin = 0.5;
    CollisionMarginData data(default_margin);
    data.setCollisionMargin("link_1", "link_2", pair_margin);
    data.setCollisionMargin("link_1", "link_2", default_margin);
    EXPECT_NEAR(data.getDefaultCollisionMargin(), default_margin, tol);
    EXPECT_NEAR(data.getMaxCollisionMargin(), default_margin, tol);
    EXPECT_NEAR(data.getCollisionMargin("link_1", "link_2"), default_margin, tol);
    EXPECT_EQ(data.getCollisionMarginPairData().getCollisionMargins().size(), 1);
  }

  {  // Test setting default larger than pair the change to lower than pair and the max should be the pair
    double default_margin = 0.05;
    double pair_margin = 0.0254;
    CollisionMarginData data(default_margin);
    data.setCollisionMargin("link_1", "link_2", pair_margin);

    default_margin = 0.0;
    data.setDefaultCollisionMargin(default_margin);
    EXPECT_NEAR(data.getDefaultCollisionMargin(), default_margin, tol);
    EXPECT_NEAR(data.getMaxCollisionMargin(), std::max(default_margin, pair_margin), tol);
    EXPECT_NEAR(data.getCollisionMargin("link_1", "link_2"), pair_margin, tol);
    EXPECT_EQ(data.getCollisionMarginPairData().getCollisionMargins().size(), 1);
  }

  {  // Test increment positive
    double default_margin = 0.0254;
    double pair_margin = 0.5;
    double increment = 0.01;
    CollisionMarginData data(default_margin);
    data.setCollisionMargin("link_1", "link_2", pair_margin);
    data.incrementMargins(increment);
    EXPECT_NEAR(data.getDefaultCollisionMargin(), default_margin + increment, tol);
    EXPECT_NEAR(data.getMaxCollisionMargin(), std::max(default_margin + increment, pair_margin + increment), tol);
    EXPECT_NEAR(data.getCollisionMargin("link_1", "link_2"), pair_margin + increment, tol);
    EXPECT_EQ(data.getCollisionMarginPairData().getCollisionMargins().size(), 1);
  }

  {  // Test increment negative
    double default_margin = 0.0254;
    double pair_margin = 0.5;
    double increment = -0.01;
    CollisionMarginData data(default_margin);
    data.setCollisionMargin("link_1", "link_2", pair_margin);
    data.incrementMargins(increment);
    EXPECT_NEAR(data.getDefaultCollisionMargin(), default_margin + increment, tol);
    EXPECT_NEAR(data.getMaxCollisionMargin(), std::max(default_margin + increment, pair_margin + increment), tol);
    EXPECT_NEAR(data.getCollisionMargin("link_1", "link_2"), pair_margin + increment, tol);
    EXPECT_EQ(data.getCollisionMarginPairData().getCollisionMargins().size(), 1);
  }

  {  // Test scale > 1
    double default_margin = 0.0254;
    double pair_margin = 0.5;
    double scale = 1.5;
    CollisionMarginData data(default_margin);
    data.setCollisionMargin("link_1", "link_2", pair_margin);
    data.scaleMargins(scale);
    EXPECT_NEAR(data.getDefaultCollisionMargin(), default_margin * scale, tol);
    EXPECT_NEAR(data.getMaxCollisionMargin(), std::max(default_margin * scale, pair_margin * scale), tol);
    EXPECT_NEAR(data.getCollisionMargin("link_1", "link_2"), pair_margin * scale, tol);
    EXPECT_EQ(data.getCollisionMarginPairData().getCollisionMargins().size(), 1);
  }

  {  // Test scale < 1
    double default_margin = 0.0254;
    double pair_margin = 0.5;
    double scale = 0.5;
    CollisionMarginData data(default_margin);
    data.setCollisionMargin("link_1", "link_2", pair_margin);
    data.scaleMargins(scale);
    EXPECT_NEAR(data.getDefaultCollisionMargin(), default_margin * scale, tol);
    EXPECT_NEAR(data.getMaxCollisionMargin(), std::max(default_margin * scale, pair_margin * scale), tol);
    EXPECT_NEAR(data.getCollisionMargin("link_1", "link_2"), pair_margin * scale, tol);
    EXPECT_EQ(data.getCollisionMarginPairData().getCollisionMargins().size(), 1);
  }

  {  // Test Apply Override Default
    double default_margin = 0.0254;
    double pair_margin = 0.5;
    CollisionMarginData data(default_margin);
    data.setCollisionMargin("link_1", "link_2", pair_margin);

    default_margin = default_margin * 3;
    data.setDefaultCollisionMargin(default_margin);
    EXPECT_NEAR(data.getDefaultCollisionMargin(), default_margin, tol);
    EXPECT_NEAR(data.getMaxCollisionMargin(), std::max(default_margin, pair_margin), tol);
    EXPECT_NEAR(data.getCollisionMargin("link_1", "link_2"), pair_margin, tol);
    EXPECT_EQ(data.getCollisionMarginPairData().getCollisionMargins().size(), 1);
  }

  {  // Test Apply Override Link Pair
    double default_margin = 0.0254;
    double pair_margin = 0.5;
    CollisionMarginData data(default_margin);
    data.setCollisionMargin("link_1", "link_2", pair_margin);

    default_margin = default_margin * 3;
    pair_margin = pair_margin * 3;
    CollisionMarginPairData override_pair_margins;
    override_pair_margins.setCollisionMargin("link_1", "link_2", pair_margin);
    data.setDefaultCollisionMargin(default_margin);
    data.apply(override_pair_margins, CollisionMarginPairOverrideType::MODIFY);
    EXPECT_NEAR(data.getDefaultCollisionMargin(), default_margin, tol);
    EXPECT_NEAR(data.getMaxCollisionMargin(), std::max(default_margin, pair_margin), tol);
    EXPECT_NEAR(data.getCollisionMargin("link_1", "link_2"), pair_margin, tol);
    EXPECT_EQ(data.getCollisionMarginPairData().getCollisionMargins().size(), 1);
  }

  {  // Test Apply Override Replace
    double default_margin = 0.0254;
    double pair_margin = 0.5;
    CollisionMarginData data(default_margin);
    data.setCollisionMargin("link_1", "link_2", pair_margin);

    default_margin = default_margin * 3;
    pair_margin = pair_margin * 3;
    CollisionMarginPairData override_pair_margins;
    override_pair_margins.setCollisionMargin("link_1", "link_2", pair_margin);
    data.setDefaultCollisionMargin(default_margin);
    data.apply(override_pair_margins, CollisionMarginPairOverrideType::REPLACE);
    EXPECT_NEAR(data.getDefaultCollisionMargin(), default_margin, tol);
    EXPECT_NEAR(data.getMaxCollisionMargin(), std::max(default_margin, pair_margin), tol);
    EXPECT_NEAR(data.getCollisionMargin("link_1", "link_2"), pair_margin, tol);
    EXPECT_EQ(data.getCollisionMarginPairData().getCollisionMargins().size(), 1);
  }

  {  // Test Apply Override None
    double default_margin = 0.0254;
    double pair_margin = 0.5;
    CollisionMarginData data(default_margin);
    data.setCollisionMargin("link_1", "link_2", pair_margin);

    default_margin = default_margin * 3;
    CollisionMarginPairData override_pair_margins;
    override_pair_margins.setCollisionMargin("link_1", "link_2", pair_margin * 3);
    data.setDefaultCollisionMargin(default_margin);
    data.apply(override_pair_margins, CollisionMarginPairOverrideType::NONE);
    EXPECT_NEAR(data.getDefaultCollisionMargin(), default_margin, tol);
    EXPECT_NEAR(data.getMaxCollisionMargin(), std::max(default_margin, pair_margin), tol);
    EXPECT_NEAR(data.getCollisionMargin("link_1", "link_2"), pair_margin, tol);
    EXPECT_EQ(data.getCollisionMarginPairData().getCollisionMargins().size(), 1);
  }

  {  // Test Apply Override Modify
    double default_margin = 0.0254;
    double pair_margin = 0.5;
    CollisionMarginData data(default_margin);
    data.setCollisionMargin("link_1", "link_2", pair_margin);

    default_margin = default_margin * 3;
    CollisionMarginPairData override_pair_margins;
    override_pair_margins.setCollisionMargin("link_1", "link_3", pair_margin * 3);
    data.setDefaultCollisionMargin(default_margin);
    data.apply(override_pair_margins, CollisionMarginPairOverrideType::MODIFY);
    EXPECT_NEAR(data.getDefaultCollisionMargin(), default_margin, tol);
    EXPECT_NEAR(data.getMaxCollisionMargin(), std::max(default_margin, pair_margin * 3), tol);
    EXPECT_NEAR(data.getCollisionMargin("link_1", "link_2"), pair_margin, tol);
    EXPECT_NEAR(data.getCollisionMargin("link_1", "link_3"), pair_margin * 3, tol);
    EXPECT_EQ(data.getCollisionMarginPairData().getCollisionMargins().size(), 2);
  }

  {  // Test Apply Override Modify with pair that already exists
    double default_margin = 0.0254;
    double pair_margin = 0.5;
    CollisionMarginData data(default_margin);
    data.setCollisionMargin("link_1", "link_2", pair_margin);

    pair_margin = pair_margin * 3;
    default_margin = default_margin * 3;
    CollisionMarginPairData override_pair_margins;
    override_pair_margins.setCollisionMargin("link_1", "link_2", pair_margin);
    override_pair_margins.setCollisionMargin("link_1", "link_3", pair_margin * 3);
    data.setDefaultCollisionMargin(default_margin);
    data.apply(override_pair_margins, CollisionMarginPairOverrideType::MODIFY);
    EXPECT_NEAR(data.getDefaultCollisionMargin(), default_margin, tol);
    EXPECT_NEAR(data.getMaxCollisionMargin(), std::max(default_margin, pair_margin * 3), tol);
    EXPECT_NEAR(data.getCollisionMargin("link_1", "link_2"), pair_margin, tol);
    EXPECT_NEAR(data.getCollisionMargin("link_1", "link_3"), pair_margin * 3, tol);
    EXPECT_EQ(data.getCollisionMarginPairData().getCollisionMargins().size(), 2);
  }

  {  // Test Apply Override Modify Pair
    double default_margin = 0.0254;
    double pair_margin = 0.5;
    CollisionMarginData data(default_margin);
    data.setCollisionMargin("link_1", "link_2", pair_margin);

    default_margin = default_margin * 3;
    CollisionMarginPairData override_pair_margins;
    override_pair_margins.setCollisionMargin("link_1", "link_3", pair_margin * 3);
    data.setDefaultCollisionMargin(default_margin);
    data.apply(override_pair_margins, CollisionMarginPairOverrideType::MODIFY);
    EXPECT_NEAR(data.getDefaultCollisionMargin(), default_margin, tol);
    EXPECT_NEAR(data.getMaxCollisionMargin(), std::max(default_margin, pair_margin * 3), tol);
    EXPECT_NEAR(data.getCollisionMargin("link_1", "link_2"), pair_margin, tol);
    EXPECT_NEAR(data.getCollisionMargin("link_1", "link_3"), pair_margin * 3, tol);
    EXPECT_EQ(data.getCollisionMarginPairData().getCollisionMargins().size(), 2);
  }

  {  // Test Apply Override Modify Pair that already exists
    double default_margin = 0.0254;
    double pair_margin = 0.5;
    CollisionMarginData data(default_margin);
    data.setCollisionMargin("link_1", "link_2", pair_margin);

    pair_margin = pair_margin * 3;
    default_margin = default_margin * 3;
    CollisionMarginPairData override_pair_margins;
    override_pair_margins.setCollisionMargin("link_1", "link_2", pair_margin);
    override_pair_margins.setCollisionMargin("link_1", "link_3", pair_margin * 3);
    data.setDefaultCollisionMargin(default_margin);
    data.apply(override_pair_margins, CollisionMarginPairOverrideType::MODIFY);
    EXPECT_NEAR(data.getDefaultCollisionMargin(), default_margin, tol);
    EXPECT_NEAR(data.getMaxCollisionMargin(), std::max(default_margin, pair_margin * 3), tol);
    EXPECT_NEAR(data.getCollisionMargin("link_1", "link_2"), pair_margin, tol);
    EXPECT_NEAR(data.getCollisionMargin("link_1", "link_3"), pair_margin * 3, tol);
    EXPECT_EQ(data.getCollisionMarginPairData().getCollisionMargins().size(), 2);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
