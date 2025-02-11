#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <vector>
#include <string>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_collision/core/common.h>
#include <tesseract_common/contact_allowed_validator.h>
#include <tesseract_common/utils.h>

class TestContactAllowedValidator : public tesseract_common::ContactAllowedValidator
{
public:
  bool operator()(const std::string& s1, const std::string& s2) const override
  {
    return (tesseract_common::makeOrderedLinkPair("base_link", "link_1") ==
            tesseract_common::makeOrderedLinkPair(s1, s2));
  }
};

TEST(TesseractCoreUnit, getCollisionObjectPairsUnit)  // NOLINT
{
  std::vector<std::string> active_links{ "link_1", "link_2", "link_3" };
  std::vector<std::string> static_links{ "base_link", "part_link" };

  std::vector<tesseract_collision::ObjectPairKey> check_pairs;
  check_pairs.push_back(tesseract_common::makeOrderedLinkPair("link_1", "link_2"));
  check_pairs.push_back(tesseract_common::makeOrderedLinkPair("link_1", "link_3"));
  check_pairs.push_back(tesseract_common::makeOrderedLinkPair("link_2", "link_3"));
  check_pairs.push_back(tesseract_common::makeOrderedLinkPair("base_link", "link_1"));
  check_pairs.push_back(tesseract_common::makeOrderedLinkPair("base_link", "link_2"));
  check_pairs.push_back(tesseract_common::makeOrderedLinkPair("base_link", "link_3"));
  check_pairs.push_back(tesseract_common::makeOrderedLinkPair("part_link", "link_1"));
  check_pairs.push_back(tesseract_common::makeOrderedLinkPair("part_link", "link_2"));
  check_pairs.push_back(tesseract_common::makeOrderedLinkPair("part_link", "link_3"));

  std::vector<tesseract_collision::ObjectPairKey> pairs =
      tesseract_collision::getCollisionObjectPairs(active_links, static_links);

  EXPECT_TRUE(tesseract_common::isIdentical<tesseract_collision::ObjectPairKey>(pairs, check_pairs, false));

  // Now check provided a is contact allowed function
  auto validator = std::make_shared<TestContactAllowedValidator>();

  check_pairs.clear();
  check_pairs.push_back(tesseract_common::makeOrderedLinkPair("link_1", "link_2"));
  check_pairs.push_back(tesseract_common::makeOrderedLinkPair("link_1", "link_3"));
  check_pairs.push_back(tesseract_common::makeOrderedLinkPair("link_2", "link_3"));
  check_pairs.push_back(tesseract_common::makeOrderedLinkPair("base_link", "link_2"));
  check_pairs.push_back(tesseract_common::makeOrderedLinkPair("base_link", "link_3"));
  check_pairs.push_back(tesseract_common::makeOrderedLinkPair("part_link", "link_1"));
  check_pairs.push_back(tesseract_common::makeOrderedLinkPair("part_link", "link_2"));
  check_pairs.push_back(tesseract_common::makeOrderedLinkPair("part_link", "link_3"));

  pairs = tesseract_collision::getCollisionObjectPairs(active_links, static_links, validator);

  EXPECT_TRUE(tesseract_common::isIdentical<tesseract_collision::ObjectPairKey>(pairs, check_pairs, false));
}

TEST(TesseractCoreUnit, isContactAllowedUnit)  // NOLINT
{
  auto validator = std::make_shared<TestContactAllowedValidator>();

  EXPECT_TRUE(tesseract_collision::isContactAllowed("base_link", "base_link", validator, false));
  EXPECT_FALSE(tesseract_collision::isContactAllowed("base_link", "link_2", validator, false));
  EXPECT_TRUE(tesseract_collision::isContactAllowed("base_link", "link_1", validator, true));
}

TEST(TesseractCoreUnit, scaleVerticesUnit)  // NOLINT
{
  tesseract_common::VectorVector3d base_vertices{};
  base_vertices.emplace_back(0, 0, 0);
  base_vertices.emplace_back(0, 0, 1);
  base_vertices.emplace_back(0, 1, 1);
  base_vertices.emplace_back(0, 1, 0);
  base_vertices.emplace_back(1, 0, 0);
  base_vertices.emplace_back(1, 0, 1);
  base_vertices.emplace_back(1, 1, 1);
  base_vertices.emplace_back(1, 1, 0);

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

TEST(TesseractCoreUnit, ContactResultMapUnit)  // NOLINT
{
  {  // Test construction state
    tesseract_collision::ContactResultMap result_map;
    EXPECT_EQ(result_map.count(), 0);
    EXPECT_EQ(result_map.size(), 0);
    EXPECT_EQ(result_map.getContainer().size(), 0);
  }

  auto key1 = tesseract_common::makeOrderedLinkPair("link1", "link2");
  auto key2 = tesseract_common::makeOrderedLinkPair("link2", "link3");

  {  // Test addContactResult single method
    tesseract_collision::ContactResultMap result_map;
    result_map.addContactResult(key1, tesseract_collision::ContactResult{});
    EXPECT_EQ(result_map.count(), 1);
    EXPECT_EQ(result_map.size(), 1);
    EXPECT_FALSE(result_map.empty());
    EXPECT_TRUE(result_map.begin() == result_map.getContainer().begin());
    EXPECT_TRUE(result_map.end() == result_map.getContainer().end());
    EXPECT_TRUE(result_map.cbegin() == result_map.getContainer().cbegin());
    EXPECT_TRUE(result_map.cend() == result_map.getContainer().cend());
    EXPECT_EQ(result_map.getContainer().size(), 1);
    auto it = result_map.find(key1);
    EXPECT_TRUE(it != result_map.end());
    EXPECT_EQ(it->second.size(), 1);

    result_map.addContactResult(key1, tesseract_collision::ContactResult{});
    EXPECT_EQ(result_map.count(), 2);
    EXPECT_EQ(result_map.size(), 1);
    EXPECT_FALSE(result_map.empty());
    EXPECT_TRUE(result_map.begin() == result_map.getContainer().begin());
    EXPECT_TRUE(result_map.end() == result_map.getContainer().end());
    EXPECT_TRUE(result_map.cbegin() == result_map.getContainer().cbegin());
    EXPECT_TRUE(result_map.cend() == result_map.getContainer().cend());
    EXPECT_EQ(result_map.getContainer().size(), 1);
    it = result_map.find(key1);
    EXPECT_TRUE(it != result_map.end());
    EXPECT_EQ(it->second.size(), 2);

    result_map.addContactResult(key2, tesseract_collision::ContactResult{});
    EXPECT_EQ(result_map.count(), 3);
    EXPECT_EQ(result_map.size(), 2);
    EXPECT_FALSE(result_map.empty());
    EXPECT_TRUE(result_map.begin() == result_map.getContainer().begin());
    EXPECT_TRUE(result_map.end() == result_map.getContainer().end());
    EXPECT_TRUE(result_map.cbegin() == result_map.getContainer().cbegin());
    EXPECT_TRUE(result_map.cend() == result_map.getContainer().cend());
    EXPECT_EQ(result_map.getContainer().size(), 2);
    it = result_map.find(key1);
    EXPECT_TRUE(it != result_map.end());
    EXPECT_EQ(it->second.size(), 2);

    it = result_map.find(key2);
    EXPECT_TRUE(it != result_map.end());
    EXPECT_EQ(it->second.size(), 1);

    // test clear
    result_map.clear();
    EXPECT_EQ(result_map.count(), 0);
    EXPECT_EQ(result_map.size(), 0);
    EXPECT_TRUE(result_map.empty());
    EXPECT_TRUE(result_map.begin() == result_map.getContainer().begin());
    EXPECT_TRUE(result_map.end() == result_map.getContainer().end());
    EXPECT_TRUE(result_map.cbegin() == result_map.getContainer().cbegin());
    EXPECT_TRUE(result_map.cend() == result_map.getContainer().cend());
    EXPECT_EQ(result_map.getContainer().size(), 2);
    it = result_map.find(key1);
    EXPECT_TRUE(it != result_map.end());
    EXPECT_EQ(it->second.size(), 0);
    EXPECT_TRUE(it->second.capacity() > 0);

    it = result_map.find(key2);
    EXPECT_TRUE(it != result_map.end());
    EXPECT_EQ(it->second.size(), 0);
    EXPECT_TRUE(it->second.capacity() > 0);

    // test release
    result_map.release();
    EXPECT_EQ(result_map.count(), 0);
    EXPECT_EQ(result_map.size(), 0);
    EXPECT_TRUE(result_map.empty());
    EXPECT_TRUE(result_map.begin() == result_map.getContainer().begin());
    EXPECT_TRUE(result_map.end() == result_map.getContainer().end());
    EXPECT_TRUE(result_map.cbegin() == result_map.getContainer().cbegin());
    EXPECT_TRUE(result_map.cend() == result_map.getContainer().cend());
    EXPECT_TRUE(result_map.getContainer().empty());
  }

  {  // Test addContactResult vector method
    tesseract_collision::ContactResultMap result_map;
    result_map.addContactResult(key1, { tesseract_collision::ContactResult{}, tesseract_collision::ContactResult{} });
    EXPECT_EQ(result_map.count(), 2);
    EXPECT_EQ(result_map.size(), 1);
    EXPECT_FALSE(result_map.empty());
    EXPECT_TRUE(result_map.begin() == result_map.getContainer().begin());
    EXPECT_TRUE(result_map.end() == result_map.getContainer().end());
    EXPECT_TRUE(result_map.cbegin() == result_map.getContainer().cbegin());
    EXPECT_TRUE(result_map.cend() == result_map.getContainer().cend());
    EXPECT_EQ(result_map.getContainer().size(), 1);
    auto it = result_map.find(key1);
    EXPECT_TRUE(it != result_map.end());
    EXPECT_EQ(it->second.size(), 2);

    result_map.addContactResult(key1, { tesseract_collision::ContactResult{}, tesseract_collision::ContactResult{} });
    EXPECT_EQ(result_map.count(), 4);
    EXPECT_EQ(result_map.size(), 1);
    EXPECT_FALSE(result_map.empty());
    EXPECT_TRUE(result_map.begin() == result_map.getContainer().begin());
    EXPECT_TRUE(result_map.end() == result_map.getContainer().end());
    EXPECT_TRUE(result_map.cbegin() == result_map.getContainer().cbegin());
    EXPECT_TRUE(result_map.cend() == result_map.getContainer().cend());
    EXPECT_EQ(result_map.getContainer().size(), 1);
    it = result_map.find(key1);
    EXPECT_TRUE(it != result_map.end());
    EXPECT_EQ(it->second.size(), 4);

    result_map.addContactResult(key2, { tesseract_collision::ContactResult{}, tesseract_collision::ContactResult{} });
    EXPECT_EQ(result_map.count(), 6);
    EXPECT_EQ(result_map.size(), 2);
    EXPECT_FALSE(result_map.empty());
    EXPECT_TRUE(result_map.begin() == result_map.getContainer().begin());
    EXPECT_TRUE(result_map.end() == result_map.getContainer().end());
    EXPECT_TRUE(result_map.cbegin() == result_map.getContainer().cbegin());
    EXPECT_TRUE(result_map.cend() == result_map.getContainer().cend());
    EXPECT_EQ(result_map.getContainer().size(), 2);
    it = result_map.find(key1);
    EXPECT_TRUE(it != result_map.end());
    EXPECT_EQ(it->second.size(), 4);

    it = result_map.find(key2);
    EXPECT_TRUE(it != result_map.end());
    EXPECT_EQ(it->second.size(), 2);

    // test release
    result_map.release();
    EXPECT_EQ(result_map.count(), 0);
    EXPECT_EQ(result_map.size(), 0);
    EXPECT_TRUE(result_map.empty());
    EXPECT_TRUE(result_map.begin() == result_map.getContainer().begin());
    EXPECT_TRUE(result_map.end() == result_map.getContainer().end());
    EXPECT_TRUE(result_map.cbegin() == result_map.getContainer().cbegin());
    EXPECT_TRUE(result_map.cend() == result_map.getContainer().cend());
    EXPECT_TRUE(result_map.getContainer().empty());
  }

  {  // Test setContactResult single method
    tesseract_collision::ContactResultMap result_map;
    result_map.setContactResult(key1, tesseract_collision::ContactResult{});
    EXPECT_EQ(result_map.count(), 1);
    EXPECT_EQ(result_map.size(), 1);
    EXPECT_FALSE(result_map.empty());
    EXPECT_TRUE(result_map.begin() == result_map.getContainer().begin());
    EXPECT_TRUE(result_map.end() == result_map.getContainer().end());
    EXPECT_TRUE(result_map.cbegin() == result_map.getContainer().cbegin());
    EXPECT_TRUE(result_map.cend() == result_map.getContainer().cend());
    EXPECT_EQ(result_map.getContainer().size(), 1);
    auto it = result_map.find(key1);
    EXPECT_TRUE(it != result_map.end());
    EXPECT_EQ(it->second.size(), 1);

    result_map.addContactResult(key1, tesseract_collision::ContactResult{});
    EXPECT_EQ(result_map.count(), 2);
    EXPECT_EQ(result_map.size(), 1);
    EXPECT_FALSE(result_map.empty());
    EXPECT_TRUE(result_map.begin() == result_map.getContainer().begin());
    EXPECT_TRUE(result_map.end() == result_map.getContainer().end());
    EXPECT_TRUE(result_map.cbegin() == result_map.getContainer().cbegin());
    EXPECT_TRUE(result_map.cend() == result_map.getContainer().cend());
    EXPECT_EQ(result_map.getContainer().size(), 1);
    it = result_map.find(key1);
    EXPECT_TRUE(it != result_map.end());
    EXPECT_EQ(it->second.size(), 2);

    result_map.setContactResult(key1, tesseract_collision::ContactResult{});
    EXPECT_EQ(result_map.count(), 1);
    EXPECT_EQ(result_map.size(), 1);
    EXPECT_FALSE(result_map.empty());
    EXPECT_TRUE(result_map.begin() == result_map.getContainer().begin());
    EXPECT_TRUE(result_map.end() == result_map.getContainer().end());
    EXPECT_TRUE(result_map.cbegin() == result_map.getContainer().cbegin());
    EXPECT_TRUE(result_map.cend() == result_map.getContainer().cend());
    EXPECT_EQ(result_map.getContainer().size(), 1);
    it = result_map.find(key1);
    EXPECT_TRUE(it != result_map.end());
    EXPECT_EQ(it->second.size(), 1);

    result_map.setContactResult(key2, tesseract_collision::ContactResult{});
    EXPECT_EQ(result_map.count(), 2);
    EXPECT_EQ(result_map.size(), 2);
    EXPECT_FALSE(result_map.empty());
    EXPECT_TRUE(result_map.begin() == result_map.getContainer().begin());
    EXPECT_TRUE(result_map.end() == result_map.getContainer().end());
    EXPECT_TRUE(result_map.cbegin() == result_map.getContainer().cbegin());
    EXPECT_TRUE(result_map.cend() == result_map.getContainer().cend());
    EXPECT_EQ(result_map.getContainer().size(), 2);
    it = result_map.find(key1);
    EXPECT_TRUE(it != result_map.end());
    EXPECT_EQ(it->second.size(), 1);

    it = result_map.find(key2);
    EXPECT_TRUE(it != result_map.end());
    EXPECT_EQ(it->second.size(), 1);

    // test clear
    result_map.clear();
    EXPECT_EQ(result_map.count(), 0);
    EXPECT_EQ(result_map.size(), 0);
    EXPECT_EQ(result_map.getContainer().size(), 2);
    it = result_map.find(key1);
    EXPECT_TRUE(it != result_map.end());
    EXPECT_EQ(it->second.size(), 0);
    EXPECT_TRUE(it->second.capacity() > 0);

    it = result_map.find(key2);
    EXPECT_TRUE(it != result_map.end());
    EXPECT_EQ(it->second.size(), 0);
    EXPECT_TRUE(it->second.capacity() > 0);

    // test shrink to fit
    result_map.shrinkToFit();
    EXPECT_EQ(result_map.count(), 0);
    EXPECT_EQ(result_map.size(), 0);
    EXPECT_TRUE(result_map.empty());
    EXPECT_TRUE(result_map.begin() == result_map.getContainer().begin());
    EXPECT_TRUE(result_map.end() == result_map.getContainer().end());
    EXPECT_TRUE(result_map.cbegin() == result_map.getContainer().cbegin());
    EXPECT_TRUE(result_map.cend() == result_map.getContainer().cend());
    EXPECT_TRUE(result_map.getContainer().empty());
  }

  {  // Test setContactResult vector method
    tesseract_collision::ContactResultMap result_map;
    result_map.setContactResult(key1, { tesseract_collision::ContactResult{}, tesseract_collision::ContactResult{} });
    EXPECT_EQ(result_map.count(), 2);
    EXPECT_EQ(result_map.size(), 1);
    EXPECT_FALSE(result_map.empty());
    EXPECT_TRUE(result_map.begin() == result_map.getContainer().begin());
    EXPECT_TRUE(result_map.end() == result_map.getContainer().end());
    EXPECT_TRUE(result_map.cbegin() == result_map.getContainer().cbegin());
    EXPECT_TRUE(result_map.cend() == result_map.getContainer().cend());
    EXPECT_EQ(result_map.getContainer().size(), 1);
    auto it = result_map.find(key1);
    EXPECT_TRUE(it != result_map.end());
    EXPECT_EQ(it->second.size(), 2);

    result_map.addContactResult(key1, { tesseract_collision::ContactResult{}, tesseract_collision::ContactResult{} });
    EXPECT_EQ(result_map.count(), 4);
    EXPECT_EQ(result_map.size(), 1);
    EXPECT_FALSE(result_map.empty());
    EXPECT_TRUE(result_map.begin() == result_map.getContainer().begin());
    EXPECT_TRUE(result_map.end() == result_map.getContainer().end());
    EXPECT_TRUE(result_map.cbegin() == result_map.getContainer().cbegin());
    EXPECT_TRUE(result_map.cend() == result_map.getContainer().cend());
    EXPECT_EQ(result_map.getContainer().size(), 1);
    it = result_map.find(key1);
    EXPECT_TRUE(it != result_map.end());
    EXPECT_EQ(it->second.size(), 4);

    result_map.setContactResult(key1, { tesseract_collision::ContactResult{}, tesseract_collision::ContactResult{} });
    EXPECT_EQ(result_map.count(), 2);
    EXPECT_EQ(result_map.size(), 1);
    EXPECT_FALSE(result_map.empty());
    EXPECT_TRUE(result_map.begin() == result_map.getContainer().begin());
    EXPECT_TRUE(result_map.end() == result_map.getContainer().end());
    EXPECT_TRUE(result_map.cbegin() == result_map.getContainer().cbegin());
    EXPECT_TRUE(result_map.cend() == result_map.getContainer().cend());
    EXPECT_EQ(result_map.getContainer().size(), 1);
    it = result_map.find(key1);
    EXPECT_TRUE(it != result_map.end());
    EXPECT_EQ(it->second.size(), 2);

    result_map.setContactResult(key2, { tesseract_collision::ContactResult{}, tesseract_collision::ContactResult{} });
    EXPECT_EQ(result_map.count(), 4);
    EXPECT_EQ(result_map.size(), 2);
    EXPECT_FALSE(result_map.empty());
    EXPECT_TRUE(result_map.begin() == result_map.getContainer().begin());
    EXPECT_TRUE(result_map.end() == result_map.getContainer().end());
    EXPECT_TRUE(result_map.cbegin() == result_map.getContainer().cbegin());
    EXPECT_TRUE(result_map.cend() == result_map.getContainer().cend());
    EXPECT_EQ(result_map.getContainer().size(), 2);
    it = result_map.find(key1);
    EXPECT_TRUE(it != result_map.end());
    EXPECT_EQ(it->second.size(), 2);

    it = result_map.find(key2);
    EXPECT_TRUE(it != result_map.end());
    EXPECT_EQ(it->second.size(), 2);

    // test release
    result_map.release();
    EXPECT_EQ(result_map.count(), 0);
    EXPECT_EQ(result_map.size(), 0);
    EXPECT_TRUE(result_map.empty());
    EXPECT_TRUE(result_map.begin() == result_map.getContainer().begin());
    EXPECT_TRUE(result_map.end() == result_map.getContainer().end());
    EXPECT_TRUE(result_map.cbegin() == result_map.getContainer().cbegin());
    EXPECT_TRUE(result_map.cend() == result_map.getContainer().cend());
    EXPECT_TRUE(result_map.getContainer().empty());
  }

  {  // flatten move
    tesseract_collision::ContactResultMap result_map;
    result_map.setContactResult(key1, { tesseract_collision::ContactResult{}, tesseract_collision::ContactResult{} });
    result_map.addContactResult(key2, tesseract_collision::ContactResult{});
    EXPECT_EQ(result_map.count(), 3);
    EXPECT_EQ(result_map.size(), 2);
    EXPECT_FALSE(result_map.empty());
    EXPECT_TRUE(result_map.begin() == result_map.getContainer().begin());
    EXPECT_TRUE(result_map.end() == result_map.getContainer().end());
    EXPECT_TRUE(result_map.cbegin() == result_map.getContainer().cbegin());
    EXPECT_TRUE(result_map.cend() == result_map.getContainer().cend());
    EXPECT_EQ(result_map.getContainer().size(), 2);
    auto it = result_map.find(key1);
    EXPECT_TRUE(it != result_map.end());
    EXPECT_EQ(it->second.size(), 2);
    it = result_map.find(key2);
    EXPECT_TRUE(it != result_map.end());
    EXPECT_EQ(it->second.size(), 1);

    tesseract_collision::ContactResultVector result_vector;
    result_map.flattenMoveResults(result_vector);
    EXPECT_EQ(result_vector.size(), 3);

    EXPECT_EQ(result_map.count(), 0);
    EXPECT_EQ(result_map.size(), 0);
    EXPECT_TRUE(result_map.empty());
    EXPECT_TRUE(result_map.begin() == result_map.getContainer().begin());
    EXPECT_TRUE(result_map.end() == result_map.getContainer().end());
    EXPECT_TRUE(result_map.cbegin() == result_map.getContainer().cbegin());
    EXPECT_TRUE(result_map.cend() == result_map.getContainer().cend());
    EXPECT_EQ(result_map.getContainer().size(), 2);
    it = result_map.find(key1);
    EXPECT_TRUE(it != result_map.end());
    EXPECT_EQ(it->second.size(), 0);
    EXPECT_TRUE(it->second.capacity() > 0);

    it = result_map.find(key2);
    EXPECT_TRUE(it != result_map.end());
    EXPECT_EQ(it->second.size(), 0);
    EXPECT_TRUE(it->second.capacity() > 0);
  }

  {  // flatten copy
    tesseract_collision::ContactResultMap result_map;
    result_map.addContactResult(key1, { tesseract_collision::ContactResult{}, tesseract_collision::ContactResult{} });
    result_map.setContactResult(key2, tesseract_collision::ContactResult{});
    EXPECT_EQ(result_map.count(), 3);
    EXPECT_EQ(result_map.size(), 2);
    EXPECT_FALSE(result_map.empty());
    EXPECT_TRUE(result_map.begin() == result_map.getContainer().begin());
    EXPECT_TRUE(result_map.end() == result_map.getContainer().end());
    EXPECT_TRUE(result_map.cbegin() == result_map.getContainer().cbegin());
    EXPECT_TRUE(result_map.cend() == result_map.getContainer().cend());
    EXPECT_EQ(result_map.getContainer().size(), 2);
    auto it = result_map.find(key1);
    EXPECT_TRUE(it != result_map.end());
    EXPECT_EQ(it->second.size(), 2);
    it = result_map.find(key2);
    EXPECT_TRUE(it != result_map.end());
    EXPECT_EQ(it->second.size(), 1);

    tesseract_collision::ContactResultVector result_vector;
    result_map.flattenCopyResults(result_vector);
    EXPECT_EQ(result_vector.size(), 3);

    EXPECT_EQ(result_map.count(), 3);
    EXPECT_EQ(result_map.size(), 2);
    EXPECT_FALSE(result_map.empty());
    EXPECT_TRUE(result_map.begin() == result_map.getContainer().begin());
    EXPECT_TRUE(result_map.end() == result_map.getContainer().end());
    EXPECT_TRUE(result_map.cbegin() == result_map.getContainer().cbegin());
    EXPECT_TRUE(result_map.cend() == result_map.getContainer().cend());
    EXPECT_EQ(result_map.getContainer().size(), 2);
    it = result_map.find(key1);
    EXPECT_TRUE(it != result_map.end());
    EXPECT_EQ(it->second.size(), 2);

    it = result_map.find(key2);
    EXPECT_TRUE(it != result_map.end());
    EXPECT_EQ(it->second.size(), 1);
  }

  {  // flatten reference wrapper
    tesseract_collision::ContactResultMap result_map;
    result_map.setContactResult(key1, { tesseract_collision::ContactResult{}, tesseract_collision::ContactResult{} });
    result_map.addContactResult(key2, tesseract_collision::ContactResult{});
    EXPECT_EQ(result_map.count(), 3);
    EXPECT_EQ(result_map.size(), 2);
    EXPECT_FALSE(result_map.empty());
    EXPECT_TRUE(result_map.begin() == result_map.getContainer().begin());
    EXPECT_TRUE(result_map.end() == result_map.getContainer().end());
    EXPECT_TRUE(result_map.cbegin() == result_map.getContainer().cbegin());
    EXPECT_TRUE(result_map.cend() == result_map.getContainer().cend());
    EXPECT_EQ(result_map.getContainer().size(), 2);
    auto it = result_map.find(key1);
    EXPECT_TRUE(it != result_map.end());
    EXPECT_EQ(it->second.size(), 2);
    it = result_map.find(key2);
    EXPECT_TRUE(it != result_map.end());
    EXPECT_EQ(it->second.size(), 1);

    std::vector<std::reference_wrapper<tesseract_collision::ContactResult>> result_vector;
    result_map.flattenWrapperResults(result_vector);
    EXPECT_EQ(result_vector.size(), 3);

    EXPECT_EQ(result_map.count(), 3);
    EXPECT_EQ(result_map.size(), 2);
    EXPECT_FALSE(result_map.empty());
    EXPECT_TRUE(result_map.begin() == result_map.getContainer().begin());
    EXPECT_TRUE(result_map.end() == result_map.getContainer().end());
    EXPECT_TRUE(result_map.cbegin() == result_map.getContainer().cbegin());
    EXPECT_TRUE(result_map.cend() == result_map.getContainer().cend());
    EXPECT_EQ(result_map.getContainer().size(), 2);
    it = result_map.find(key1);
    EXPECT_TRUE(it != result_map.end());
    EXPECT_EQ(it->second.size(), 2);

    it = result_map.find(key2);
    EXPECT_TRUE(it != result_map.end());
    EXPECT_EQ(it->second.size(), 1);
  }

  {  // flatten reference wrapper const
    tesseract_collision::ContactResultMap result_map;
    result_map.addContactResult(key1, { tesseract_collision::ContactResult{}, tesseract_collision::ContactResult{} });
    result_map.setContactResult(key2, tesseract_collision::ContactResult{});
    EXPECT_EQ(result_map.count(), 3);
    EXPECT_EQ(result_map.size(), 2);
    EXPECT_FALSE(result_map.empty());
    EXPECT_TRUE(result_map.begin() == result_map.getContainer().begin());
    EXPECT_TRUE(result_map.end() == result_map.getContainer().end());
    EXPECT_TRUE(result_map.cbegin() == result_map.getContainer().cbegin());
    EXPECT_TRUE(result_map.cend() == result_map.getContainer().cend());
    EXPECT_EQ(result_map.getContainer().size(), 2);
    auto it = result_map.find(key1);
    EXPECT_TRUE(it != result_map.end());
    EXPECT_EQ(it->second.size(), 2);
    it = result_map.find(key2);
    EXPECT_TRUE(it != result_map.end());
    EXPECT_EQ(it->second.size(), 1);

    std::vector<std::reference_wrapper<const tesseract_collision::ContactResult>> result_vector;
    result_map.flattenWrapperResults(result_vector);
    EXPECT_EQ(result_vector.size(), 3);

    EXPECT_EQ(result_map.count(), 3);
    EXPECT_EQ(result_map.size(), 2);
    EXPECT_FALSE(result_map.empty());
    EXPECT_TRUE(result_map.begin() == result_map.getContainer().begin());
    EXPECT_TRUE(result_map.end() == result_map.getContainer().end());
    EXPECT_TRUE(result_map.cbegin() == result_map.getContainer().cbegin());
    EXPECT_TRUE(result_map.cend() == result_map.getContainer().cend());
    EXPECT_EQ(result_map.getContainer().size(), 2);
    it = result_map.find(key1);
    EXPECT_TRUE(it != result_map.end());
    EXPECT_EQ(it->second.size(), 2);

    it = result_map.find(key2);
    EXPECT_TRUE(it != result_map.end());
    EXPECT_EQ(it->second.size(), 1);
  }

  {  // filter
    tesseract_collision::ContactResultMap result_map;
    result_map.setContactResult(key1, { tesseract_collision::ContactResult{}, tesseract_collision::ContactResult{} });
    result_map.addContactResult(key2, tesseract_collision::ContactResult{});
    EXPECT_EQ(result_map.count(), 3);
    EXPECT_EQ(result_map.size(), 2);
    EXPECT_FALSE(result_map.empty());
    EXPECT_TRUE(result_map.begin() == result_map.getContainer().begin());
    EXPECT_TRUE(result_map.end() == result_map.getContainer().end());
    EXPECT_TRUE(result_map.cbegin() == result_map.getContainer().cbegin());
    EXPECT_TRUE(result_map.cend() == result_map.getContainer().cend());
    EXPECT_EQ(result_map.getContainer().size(), 2);
    auto it = result_map.find(key1);
    EXPECT_TRUE(it != result_map.end());
    EXPECT_EQ(it->second.size(), 2);
    it = result_map.find(key2);
    EXPECT_TRUE(it != result_map.end());
    EXPECT_EQ(it->second.size(), 1);

    auto filter = [key1](tesseract_collision::ContactResultMap::PairType& pair) {
      if (key1 == pair.first)
        pair.second.clear();
    };
    result_map.filter(filter);

    EXPECT_EQ(result_map.count(), 1);
    EXPECT_EQ(result_map.size(), 1);
    EXPECT_FALSE(result_map.empty());
    EXPECT_TRUE(result_map.begin() == result_map.getContainer().begin());
    EXPECT_TRUE(result_map.end() == result_map.getContainer().end());
    EXPECT_TRUE(result_map.cbegin() == result_map.getContainer().cbegin());
    EXPECT_TRUE(result_map.cend() == result_map.getContainer().cend());
    EXPECT_EQ(result_map.getContainer().size(), 2);
    it = result_map.find(key1);
    EXPECT_TRUE(it != result_map.end());
    EXPECT_EQ(it->second.size(), 0);
    EXPECT_TRUE(it->second.capacity() > 0);

    it = result_map.find(key2);
    EXPECT_TRUE(it != result_map.end());
    EXPECT_EQ(it->second.size(), 1);
  }
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
