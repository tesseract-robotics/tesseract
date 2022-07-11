#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <vector>
#include <string>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_collision/core/common.h>
#include <tesseract_common/utils.h>

TEST(TesseractCoreUnit, getCollisionObjectPairsUnit)  // NOLINT
{
  std::vector<std::string> active_links{ "link_1", "link_2", "link_3" };
  std::vector<std::string> static_links{ "base_link", "part_link" };

  std::vector<tesseract_collision::ObjectPairKey> check_pairs;
  check_pairs.push_back(tesseract_collision::getObjectPairKey("link_1", "link_2"));
  check_pairs.push_back(tesseract_collision::getObjectPairKey("link_1", "link_3"));
  check_pairs.push_back(tesseract_collision::getObjectPairKey("link_2", "link_3"));
  check_pairs.push_back(tesseract_collision::getObjectPairKey("base_link", "link_1"));
  check_pairs.push_back(tesseract_collision::getObjectPairKey("base_link", "link_2"));
  check_pairs.push_back(tesseract_collision::getObjectPairKey("base_link", "link_3"));
  check_pairs.push_back(tesseract_collision::getObjectPairKey("part_link", "link_1"));
  check_pairs.push_back(tesseract_collision::getObjectPairKey("part_link", "link_2"));
  check_pairs.push_back(tesseract_collision::getObjectPairKey("part_link", "link_3"));

  std::vector<tesseract_collision::ObjectPairKey> pairs =
      tesseract_collision::getCollisionObjectPairs(active_links, static_links);

  EXPECT_TRUE(tesseract_common::isIdentical<tesseract_collision::ObjectPairKey>(pairs, check_pairs, false));

  // Now check provided a is contact allowed function
  auto acm = [](const std::string& s1, const std::string& s2) {
    return (tesseract_collision::getObjectPairKey("base_link", "link_1") ==
            tesseract_collision::getObjectPairKey(s1, s2));
  };

  check_pairs.clear();
  check_pairs.push_back(tesseract_collision::getObjectPairKey("link_1", "link_2"));
  check_pairs.push_back(tesseract_collision::getObjectPairKey("link_1", "link_3"));
  check_pairs.push_back(tesseract_collision::getObjectPairKey("link_2", "link_3"));
  check_pairs.push_back(tesseract_collision::getObjectPairKey("base_link", "link_2"));
  check_pairs.push_back(tesseract_collision::getObjectPairKey("base_link", "link_3"));
  check_pairs.push_back(tesseract_collision::getObjectPairKey("part_link", "link_1"));
  check_pairs.push_back(tesseract_collision::getObjectPairKey("part_link", "link_2"));
  check_pairs.push_back(tesseract_collision::getObjectPairKey("part_link", "link_3"));

  pairs = tesseract_collision::getCollisionObjectPairs(active_links, static_links, acm);

  EXPECT_TRUE(tesseract_common::isIdentical<tesseract_collision::ObjectPairKey>(pairs, check_pairs, false));
}

TEST(TesseractCoreUnit, isContactAllowedUnit)  // NOLINT
{
  auto acm = [](const std::string& s1, const std::string& s2) {
    return (tesseract_collision::getObjectPairKey("base_link", "link_1") ==
            tesseract_collision::getObjectPairKey(s1, s2));
  };

  EXPECT_TRUE(tesseract_collision::isContactAllowed("base_link", "base_link", acm, false));
  EXPECT_FALSE(tesseract_collision::isContactAllowed("base_link", "link_2", acm, false));
  EXPECT_TRUE(tesseract_collision::isContactAllowed("base_link", "link_1", acm, true));
}

TEST(TesseractCoreUnit, scaleVerticesUnit)  // NOLINT
{
  tesseract_common::VectorVector3d base_vertices{};
  base_vertices.push_back(Eigen::Vector3d(0, 0, 0));
  base_vertices.push_back(Eigen::Vector3d(0, 0, 1));
  base_vertices.push_back(Eigen::Vector3d(0, 1, 1));
  base_vertices.push_back(Eigen::Vector3d(0, 1, 0));
  base_vertices.push_back(Eigen::Vector3d(1, 0, 0));
  base_vertices.push_back(Eigen::Vector3d(1, 0, 1));
  base_vertices.push_back(Eigen::Vector3d(1, 1, 1));
  base_vertices.push_back(Eigen::Vector3d(1, 1, 0));

  // Test identity scale
  // NOLINTNEXTLINE(cppcoreguidelines-init-variables)
  tesseract_common::VectorVector3d test_vertices{ base_vertices };
  tesseract_collision::scaleVertices(test_vertices, Eigen::Vector3d(1, 1, 1));
  for (std::size_t i = 0; i < 8; ++i)
  {
    EXPECT_TRUE(test_vertices[i].isApprox(base_vertices[i]));
  }

  // Test scale by 10
  test_vertices = base_vertices;
  tesseract_collision::scaleVertices(test_vertices, Eigen::Vector3d(10, 10, 10));
  EXPECT_TRUE(test_vertices[0].isApprox(Eigen::Vector3d(-4.5, -4.5, -4.5)));
  EXPECT_TRUE(test_vertices[1].isApprox(Eigen::Vector3d(-4.5, -4.5, 5.5)));
  EXPECT_TRUE(test_vertices[2].isApprox(Eigen::Vector3d(-4.5, 5.5, 5.5)));
  EXPECT_TRUE(test_vertices[3].isApprox(Eigen::Vector3d(-4.5, 5.5, -4.5)));
  EXPECT_TRUE(test_vertices[4].isApprox(Eigen::Vector3d(5.5, -4.5, -4.5)));
  EXPECT_TRUE(test_vertices[5].isApprox(Eigen::Vector3d(5.5, -4.5, 5.5)));
  EXPECT_TRUE(test_vertices[6].isApprox(Eigen::Vector3d(5.5, 5.5, 5.5)));
  EXPECT_TRUE(test_vertices[7].isApprox(Eigen::Vector3d(5.5, 5.5, -4.5)));

  // Test scale by 10 with center (0, 0, 0)
  test_vertices = base_vertices;
  tesseract_collision::scaleVertices(test_vertices, Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(10, 10, 10));
  EXPECT_TRUE(test_vertices[0].isApprox(Eigen::Vector3d(0, 0, 0)));
  EXPECT_TRUE(test_vertices[1].isApprox(Eigen::Vector3d(0, 0, 10)));
  EXPECT_TRUE(test_vertices[2].isApprox(Eigen::Vector3d(0, 10, 10)));
  EXPECT_TRUE(test_vertices[3].isApprox(Eigen::Vector3d(0, 10, 0)));
  EXPECT_TRUE(test_vertices[4].isApprox(Eigen::Vector3d(10, 0, 0)));
  EXPECT_TRUE(test_vertices[5].isApprox(Eigen::Vector3d(10, 0, 10)));
  EXPECT_TRUE(test_vertices[6].isApprox(Eigen::Vector3d(10, 10, 10)));
  EXPECT_TRUE(test_vertices[7].isApprox(Eigen::Vector3d(10, 10, 0)));
}

TEST(TesseractCoreUnit, ContactResultsUnit)  // NOLINT
{
  tesseract_collision::ContactResult results;

  EXPECT_EQ(results.distance, std::numeric_limits<double>::max());
  EXPECT_TRUE(results.nearest_points[0].isApprox(Eigen::Vector3d::Zero()));
  EXPECT_TRUE(results.nearest_points[1].isApprox(Eigen::Vector3d::Zero()));
  EXPECT_TRUE(results.nearest_points_local[0].isApprox(Eigen::Vector3d::Zero()));
  EXPECT_TRUE(results.nearest_points_local[1].isApprox(Eigen::Vector3d::Zero()));
  EXPECT_TRUE(results.transform[0].isApprox(Eigen::Isometry3d::Identity()));
  EXPECT_TRUE(results.transform[1].isApprox(Eigen::Isometry3d::Identity()));
  EXPECT_TRUE(results.link_names[0].empty());
  EXPECT_TRUE(results.link_names[1].empty());
  EXPECT_EQ(results.shape_id[0], -1);
  EXPECT_EQ(results.shape_id[1], -1);
  EXPECT_EQ(results.subshape_id[0], -1);
  EXPECT_EQ(results.subshape_id[1], -1);
  EXPECT_EQ(results.type_id[0], 0);
  EXPECT_EQ(results.type_id[1], 0);
  EXPECT_TRUE(results.normal.isApprox(Eigen::Vector3d::Zero()));
  EXPECT_EQ(results.cc_time[0], -1);
  EXPECT_EQ(results.cc_time[1], -1);
  EXPECT_EQ(results.cc_type[0], tesseract_collision::ContinuousCollisionType::CCType_None);
  EXPECT_EQ(results.cc_type[1], tesseract_collision::ContinuousCollisionType::CCType_None);
  EXPECT_TRUE(results.cc_transform[0].isApprox(Eigen::Isometry3d::Identity()));
  EXPECT_TRUE(results.cc_transform[1].isApprox(Eigen::Isometry3d::Identity()));
  EXPECT_EQ(results.single_contact_point, false);

  results.distance = 10;
  results.nearest_points[0] = Eigen::Vector3d(1, 2, 3);
  results.nearest_points[1] = Eigen::Vector3d(1, 2, 3);
  results.nearest_points_local[0] = Eigen::Vector3d(1, 2, 3);
  results.nearest_points_local[1] = Eigen::Vector3d(1, 2, 3);
  results.transform[0] = Eigen::Isometry3d::Identity() * Eigen::Translation3d(1, 2, 3);
  results.transform[1] = Eigen::Isometry3d::Identity() * Eigen::Translation3d(1, 2, 3);
  results.link_names[0] = "notempty";
  results.link_names[1] = "notempty";
  results.shape_id[0] = 5;
  results.shape_id[1] = 5;
  results.subshape_id[0] = 10;
  results.subshape_id[1] = 10;
  results.type_id[0] = 3;
  results.type_id[1] = 3;
  results.normal = Eigen::Vector3d(1, 2, 3);
  results.cc_time[0] = 7;
  results.cc_time[1] = 7;
  results.cc_type[0] = tesseract_collision::ContinuousCollisionType::CCType_Between;
  results.cc_type[1] = tesseract_collision::ContinuousCollisionType::CCType_Time0;
  results.cc_transform[0] = Eigen::Isometry3d::Identity() * Eigen::Translation3d(1, 2, 3);
  results.cc_transform[1] = Eigen::Isometry3d::Identity() * Eigen::Translation3d(1, 2, 3);
  results.single_contact_point = true;

  results.clear();

  EXPECT_EQ(results.distance, std::numeric_limits<double>::max());
  EXPECT_TRUE(results.nearest_points[0].isApprox(Eigen::Vector3d::Zero()));
  EXPECT_TRUE(results.nearest_points[1].isApprox(Eigen::Vector3d::Zero()));
  EXPECT_TRUE(results.nearest_points_local[0].isApprox(Eigen::Vector3d::Zero()));
  EXPECT_TRUE(results.nearest_points_local[1].isApprox(Eigen::Vector3d::Zero()));
  EXPECT_TRUE(results.transform[0].isApprox(Eigen::Isometry3d::Identity()));
  EXPECT_TRUE(results.transform[1].isApprox(Eigen::Isometry3d::Identity()));
  EXPECT_TRUE(results.link_names[0].empty());
  EXPECT_TRUE(results.link_names[1].empty());
  EXPECT_EQ(results.shape_id[0], -1);
  EXPECT_EQ(results.shape_id[1], -1);
  EXPECT_EQ(results.subshape_id[0], -1);
  EXPECT_EQ(results.subshape_id[1], -1);
  EXPECT_EQ(results.type_id[0], 0);
  EXPECT_EQ(results.type_id[1], 0);
  EXPECT_TRUE(results.normal.isApprox(Eigen::Vector3d::Zero()));
  EXPECT_EQ(results.cc_time[0], -1);
  EXPECT_EQ(results.cc_time[1], -1);
  EXPECT_EQ(results.cc_type[0], tesseract_collision::ContinuousCollisionType::CCType_None);
  EXPECT_EQ(results.cc_type[1], tesseract_collision::ContinuousCollisionType::CCType_None);
  EXPECT_TRUE(results.cc_transform[0].isApprox(Eigen::Isometry3d::Identity()));
  EXPECT_TRUE(results.cc_transform[1].isApprox(Eigen::Isometry3d::Identity()));
  EXPECT_EQ(results.single_contact_point, false);
}

TEST(TesseractCoreUnit, CollisionCheckConfigUnit)  // NOLINT
{
  tesseract_collision::ContactRequest request;
  tesseract_collision::CollisionCheckConfig config(
      5, request, tesseract_collision::CollisionEvaluatorType::LVS_DISCRETE, 0.5);
  EXPECT_NEAR(config.contact_manager_config.margin_data.getDefaultCollisionMargin(), 5, 1e-6);
  EXPECT_EQ(config.type, tesseract_collision::CollisionEvaluatorType::LVS_DISCRETE);
  EXPECT_NEAR(config.longest_valid_segment_length, 0.5, 1e-6);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
