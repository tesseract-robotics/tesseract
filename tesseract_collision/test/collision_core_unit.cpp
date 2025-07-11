#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <vector>
#include <string>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_collision/core/common.h>
#include <tesseract_collision/core/types.h>
#include <tesseract_collision/core/serialization.h>
#include <tesseract_collision/core/yaml_extensions.h>
#include <tesseract_common/contact_allowed_validator.h>
#include <tesseract_common/unit_test_utils.h>
#include <tesseract_common/utils.h>
#include <tesseract_common/yaml_utils.h>

class TestContactAllowedValidator : public tesseract_common::ContactAllowedValidator
{
public:
  bool operator()(const std::string& s1, const std::string& s2) const override
  {
    return (tesseract_common::makeOrderedLinkPair("base_link", "link_1") ==
            tesseract_common::makeOrderedLinkPair(s1, s2));
  }
};

TEST(TesseractCoreUnit, ContactManagerConfigTest)  // NOLINT
{
  {  // Default Construction
    tesseract_collision::ContactManagerConfig config;
    EXPECT_FALSE(config.default_margin.has_value());
    EXPECT_EQ(config.pair_margin_override_type, tesseract_collision::CollisionMarginPairOverrideType::NONE);
    EXPECT_TRUE(config.pair_margin_data.empty());
    EXPECT_EQ(config.acm_override_type, tesseract_collision::ACMOverrideType::NONE);
    EXPECT_TRUE(config.acm.getAllAllowedCollisions().empty());
    EXPECT_TRUE(config.modify_object_enabled.empty());

    tesseract_common::testSerialization<tesseract_collision::ContactManagerConfig>(config, "ContactManagerConfig");

    config.incrementMargins(0.025);
    EXPECT_FALSE(config.default_margin.has_value());
    EXPECT_EQ(config.pair_margin_override_type, tesseract_collision::CollisionMarginPairOverrideType::NONE);
    EXPECT_TRUE(config.pair_margin_data.empty());
    EXPECT_EQ(config.acm_override_type, tesseract_collision::ACMOverrideType::NONE);
    EXPECT_TRUE(config.acm.getAllAllowedCollisions().empty());
    EXPECT_TRUE(config.modify_object_enabled.empty());

    config.scaleMargins(2.0);
    EXPECT_FALSE(config.default_margin.has_value());
    EXPECT_EQ(config.pair_margin_override_type, tesseract_collision::CollisionMarginPairOverrideType::NONE);
    EXPECT_TRUE(config.pair_margin_data.empty());
    EXPECT_EQ(config.acm_override_type, tesseract_collision::ACMOverrideType::NONE);
    EXPECT_TRUE(config.acm.getAllAllowedCollisions().empty());
    EXPECT_TRUE(config.modify_object_enabled.empty());
  }

  {  // Construction
    tesseract_collision::ContactManagerConfig config(0.025);
    EXPECT_TRUE(config.default_margin.has_value());
    EXPECT_TRUE(tesseract_common::almostEqualRelativeAndAbs(config.default_margin.value(), 0.025));  // NOLINT
    EXPECT_EQ(config.pair_margin_override_type, tesseract_collision::CollisionMarginPairOverrideType::NONE);
    EXPECT_TRUE(config.pair_margin_data.empty());
    EXPECT_EQ(config.acm_override_type, tesseract_collision::ACMOverrideType::NONE);
    EXPECT_TRUE(config.acm.getAllAllowedCollisions().empty());
    EXPECT_TRUE(config.modify_object_enabled.empty());

    tesseract_common::testSerialization<tesseract_collision::ContactManagerConfig>(config, "ContactManagerConfig");

    config.incrementMargins(0.025);
    EXPECT_TRUE(config.default_margin.has_value());
    EXPECT_TRUE(tesseract_common::almostEqualRelativeAndAbs(config.default_margin.value(), 0.05));  // NOLINT
    EXPECT_EQ(config.pair_margin_override_type, tesseract_collision::CollisionMarginPairOverrideType::NONE);
    EXPECT_TRUE(config.pair_margin_data.empty());
    EXPECT_EQ(config.acm_override_type, tesseract_collision::ACMOverrideType::NONE);
    EXPECT_TRUE(config.acm.getAllAllowedCollisions().empty());
    EXPECT_TRUE(config.modify_object_enabled.empty());

    config.scaleMargins(2.0);
    EXPECT_TRUE(config.default_margin.has_value());
    EXPECT_TRUE(tesseract_common::almostEqualRelativeAndAbs(config.default_margin.value(), 2.0 * 0.05));  // NOLINT
    EXPECT_EQ(config.pair_margin_override_type, tesseract_collision::CollisionMarginPairOverrideType::NONE);
    EXPECT_TRUE(config.pair_margin_data.empty());
    EXPECT_EQ(config.acm_override_type, tesseract_collision::ACMOverrideType::NONE);
    EXPECT_TRUE(config.acm.getAllAllowedCollisions().empty());
    EXPECT_TRUE(config.modify_object_enabled.empty());
  }

  {  // Construction
    tesseract_collision::ContactManagerConfig config(0.025);
    config.pair_margin_data.setCollisionMargin("link1", "link2", 0.05);
    config.pair_margin_override_type = tesseract_collision::CollisionMarginPairOverrideType::MODIFY;

    tesseract_common::testSerialization<tesseract_collision::ContactManagerConfig>(config, "ContactManagerConfig");

    config.incrementMargins(0.025);
    EXPECT_TRUE(config.default_margin.has_value());
    EXPECT_TRUE(tesseract_common::almostEqualRelativeAndAbs(config.default_margin.value(), 0.05));  // NOLINT
    EXPECT_TRUE(tesseract_common::almostEqualRelativeAndAbs(
        config.pair_margin_data.getCollisionMargin("link1", "link2").value(), 0.05 + 0.025));  // NOLINT

    config.scaleMargins(2.0);
    EXPECT_TRUE(config.default_margin.has_value());
    EXPECT_TRUE(tesseract_common::almostEqualRelativeAndAbs(config.default_margin.value(), 2.0 * 0.05));  // NOLINT
    EXPECT_TRUE(tesseract_common::almostEqualRelativeAndAbs(
        config.pair_margin_data.getCollisionMargin("link1", "link2").value(), 2.0 * (0.05 + 0.025)));  // NOLINT
  }

  {
    tesseract_collision::ContactManagerConfig config;
    EXPECT_NO_THROW(config.validate());  // NOLINT
  }

  {
    tesseract_collision::ContactManagerConfig config(0.1);
    EXPECT_NO_THROW(config.validate());  // NOLINT
  }

  {
    tesseract_collision::ContactManagerConfig config;
    config.default_margin = 0.1;
    EXPECT_NO_THROW(config.validate());  // NOLINT
  }

  {
    tesseract_collision::ContactManagerConfig config;
    config.pair_margin_data.setCollisionMargin("a", "b", 0.1);
    EXPECT_ANY_THROW(config.validate());  // NOLINT
  }

  {
    tesseract_collision::ContactManagerConfig config;
    config.pair_margin_override_type = tesseract_collision::CollisionMarginPairOverrideType::MODIFY;
    config.pair_margin_data.setCollisionMargin("a", "b", 0.1);
    EXPECT_NO_THROW(config.validate());  // NOLINT
  }

  {
    tesseract_collision::ContactManagerConfig config;
    config.acm.addAllowedCollision("a", "b", "never");
    EXPECT_ANY_THROW(config.validate());  // NOLINT
  }

  {
    tesseract_collision::ContactManagerConfig config;
    config.acm_override_type = tesseract_collision::ACMOverrideType::OR;
    config.acm.addAllowedCollision("a", "b", "never");
    EXPECT_NO_THROW(config.validate());  // NOLINT
  }
}

TEST(TesseractCoreUnit, ContactManagerConfigYamlUnit)  // NOLINT
{
  const std::string yaml_string = R"(
    pair_margin_override_type: NONE
    acm_override_type: NONE
  )";

  tesseract_collision::ContactManagerConfig data_original;

  {  // decode
    tesseract_collision::ContactManagerConfig cm;
    YAML::Node n = YAML::Load(yaml_string);
    auto success = YAML::convert<tesseract_collision::ContactManagerConfig>::decode(n, cm);
    EXPECT_TRUE(success);
    EXPECT_EQ(cm.pair_margin_override_type, data_original.pair_margin_override_type);
    EXPECT_EQ(cm.acm_override_type, data_original.acm_override_type);
  }

  {  // encode
    tesseract_collision::ContactManagerConfig cm;
    YAML::Node n = YAML::Load(yaml_string);
    YAML::Node output_n = YAML::convert<tesseract_collision::ContactManagerConfig>::encode(cm);
    EXPECT_EQ(cm.pair_margin_override_type, data_original.pair_margin_override_type);
    EXPECT_EQ(cm.acm_override_type, data_original.acm_override_type);
  }
}

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

  tesseract_common::testSerialization<tesseract_collision::ContactResult>(results, "ContactResult");

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

  tesseract_common::testSerialization<tesseract_collision::ContactResult>(results, "ContactResult");

  tesseract_collision::ContactResult copy_results(results);
  EXPECT_TRUE(copy_results == results);

  results.clear();
  EXPECT_TRUE(copy_results != results);
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

  tesseract_common::testSerialization<tesseract_collision::ContactResult>(results, "ContactResult");
}

TEST(TesseractCoreUnit, ContactResultMapUnit)  // NOLINT
{
  {  // Test construction state
    tesseract_collision::ContactResultMap result_map;
    EXPECT_EQ(result_map.count(), 0);
    EXPECT_EQ(result_map.size(), 0);
    EXPECT_EQ(result_map.getContainer().size(), 0);
    EXPECT_FALSE(result_map.getSummary().empty());

    tesseract_common::testSerialization<tesseract_collision::ContactResultMap>(result_map, "ContactResultMap");
  }

  auto key1 = tesseract_common::makeOrderedLinkPair("link1", "link2");
  auto key2 = tesseract_common::makeOrderedLinkPair("link2", "link3");

  {  // Test addContactResult single method
    tesseract_collision::ContactResultMap result_map;
    result_map.addContactResult(key1, tesseract_collision::ContactResult{});
    EXPECT_FALSE(result_map.getSummary().empty());
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

    tesseract_common::testSerialization<tesseract_collision::ContactResultMap>(result_map, "ContactResultMap");

    result_map.addContactResult(key1, tesseract_collision::ContactResult{});
    EXPECT_FALSE(result_map.getSummary().empty());
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

    tesseract_common::testSerialization<tesseract_collision::ContactResultMap>(result_map, "ContactResultMap");

    result_map.addContactResult(key2, tesseract_collision::ContactResult{});
    EXPECT_FALSE(result_map.getSummary().empty());
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

    tesseract_common::testSerialization<tesseract_collision::ContactResultMap>(result_map, "ContactResultMap");

    tesseract_collision::ContactResultMap copy_result_map(result_map);
    EXPECT_TRUE(copy_result_map == result_map);

    // test clear
    result_map.clear();
    EXPECT_TRUE(copy_result_map != result_map);
    EXPECT_FALSE(result_map.getSummary().empty());
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

    tesseract_common::testSerialization<tesseract_collision::ContactResultMap>(result_map, "ContactResultMap");
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

    tesseract_common::testSerialization<tesseract_collision::ContactResultMap>(result_map, "ContactResultMap");
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

TEST(TesseractCoreUnit, ContactRequestUnit)  // NOLINT
{
  {
    tesseract_collision::ContactRequest request;
    EXPECT_EQ(request.type, tesseract_collision::ContactTestType::ALL);
    EXPECT_TRUE(request.calculate_penetration);
    EXPECT_TRUE(request.calculate_distance);
    EXPECT_EQ(request.contact_limit, 0);
    EXPECT_TRUE(request.is_valid == nullptr);

    tesseract_common::testSerialization<tesseract_collision::ContactRequest>(request, "ContactRequest");
  }

  {
    tesseract_collision::ContactRequest request(tesseract_collision::ContactTestType::FIRST);
    EXPECT_EQ(request.type, tesseract_collision::ContactTestType::FIRST);
    EXPECT_TRUE(request.calculate_penetration);
    EXPECT_TRUE(request.calculate_distance);
    EXPECT_EQ(request.contact_limit, 0);
    EXPECT_TRUE(request.is_valid == nullptr);

    tesseract_common::testSerialization<tesseract_collision::ContactRequest>(request, "ContactRequest");
  }
}

TEST(TesseractCoreUnit, ContactRequestYamlUnit)  // NOLINT
{
  const std::string yaml_string = R"(
    type: ALL
    calculate_penetration: true
    calculate_distance: true
    contact_limit: 0
  )";

  tesseract_collision::ContactRequest data_original;
  data_original.type = tesseract_collision::ContactTestType::ALL;
  data_original.calculate_penetration = true;
  data_original.calculate_distance = true;
  data_original.contact_limit = 0;

  {  // decode
    tesseract_collision::ContactRequest cr;
    YAML::Node n = YAML::Load(yaml_string);
    auto success = YAML::convert<tesseract_collision::ContactRequest>::decode(n, cr);
    EXPECT_TRUE(success);
    EXPECT_EQ(cr.type, data_original.type);
    EXPECT_EQ(cr.calculate_penetration, data_original.calculate_penetration);
    EXPECT_EQ(cr.calculate_distance, data_original.calculate_distance);
    EXPECT_EQ(cr.contact_limit, data_original.contact_limit);
  }

  {  // encode
    tesseract_collision::ContactRequest cr;
    YAML::Node n = YAML::Load(yaml_string);
    YAML::Node output_n = YAML::convert<tesseract_collision::ContactRequest>::encode(cr);
    EXPECT_EQ(cr.type, data_original.type);
    EXPECT_EQ(cr.calculate_penetration, data_original.calculate_penetration);
    EXPECT_EQ(cr.calculate_distance, data_original.calculate_distance);
    EXPECT_EQ(cr.contact_limit, data_original.contact_limit);
  }
}

TEST(TesseractCoreUnit, CollisionCheckConfigUnit)  // NOLINT
{
  {  // Default Constructor
    tesseract_collision::CollisionCheckConfig config;
    EXPECT_EQ(config.contact_request, tesseract_collision::ContactRequest());
    EXPECT_EQ(config.type, tesseract_collision::CollisionEvaluatorType::DISCRETE);
    EXPECT_TRUE(tesseract_common::almostEqualRelativeAndAbs(config.longest_valid_segment_length, 0.005));
    EXPECT_EQ(config.check_program_mode, tesseract_collision::CollisionCheckProgramType::ALL);

    tesseract_common::testSerialization<tesseract_collision::CollisionCheckConfig>(config, "CollisionCheckConfig");
  }
  {
    tesseract_collision::ContactRequest request;
    tesseract_collision::CollisionCheckConfig config(request,
                                                     tesseract_collision::CollisionEvaluatorType::LVS_DISCRETE,
                                                     0.5,
                                                     tesseract_collision::CollisionCheckProgramType::ALL_EXCEPT_START);

    EXPECT_EQ(config.contact_request, request);
    EXPECT_EQ(config.type, tesseract_collision::CollisionEvaluatorType::LVS_DISCRETE);
    EXPECT_TRUE(tesseract_common::almostEqualRelativeAndAbs(config.longest_valid_segment_length, 0.5));
    EXPECT_EQ(config.check_program_mode, tesseract_collision::CollisionCheckProgramType::ALL_EXCEPT_START);

    tesseract_common::testSerialization<tesseract_collision::CollisionCheckConfig>(config, "CollisionCheckConfig");
  }
}

TEST(TesseractCoreUnit, CollisionCheckConfigYamlUnit)  // NOLINT
{
  const std::string contact_request_yaml_string = R"(
    type: ALL
    calculate_penetration: true
    calculate_distance: true
    contact_limit: 0
  )";

  const std::string yaml_string = R"(
    type: DISCRETE
    longest_valid_segment_length: 0.005
    check_program_mode: ALL
  )";

  tesseract_collision::ContactRequest cr_original;
  cr_original.type = tesseract_collision::ContactTestType::ALL;
  cr_original.calculate_penetration = true;
  cr_original.calculate_distance = true;
  cr_original.contact_limit = 0;

  tesseract_collision::CollisionCheckConfig data_original;
  data_original.contact_request = cr_original;
  data_original.type = tesseract_collision::CollisionEvaluatorType::DISCRETE;
  data_original.longest_valid_segment_length = 0.005;
  data_original.check_program_mode = tesseract_collision::CollisionCheckProgramType::ALL;

  {  // decode
    tesseract_collision::CollisionCheckConfig cr;
    YAML::Node n = YAML::Load(yaml_string);
    auto success = YAML::convert<tesseract_collision::CollisionCheckConfig>::decode(n, cr);
    EXPECT_TRUE(success);
    EXPECT_EQ(cr.contact_request, data_original.contact_request);
    EXPECT_EQ(cr.type, data_original.type);
    EXPECT_EQ(cr.longest_valid_segment_length, data_original.longest_valid_segment_length);
    EXPECT_EQ(cr.check_program_mode, data_original.check_program_mode);
  }

  {  // encode
    tesseract_collision::CollisionCheckConfig cr;
    YAML::Node n = YAML::Load(yaml_string);
    n["contact_request"] = YAML::Load(contact_request_yaml_string);
    YAML::Node output_n = YAML::convert<tesseract_collision::CollisionCheckConfig>::encode(cr);
    EXPECT_EQ(cr.type, data_original.type);
    EXPECT_EQ(cr.longest_valid_segment_length, data_original.longest_valid_segment_length);
    EXPECT_EQ(cr.check_program_mode, data_original.check_program_mode);
  }
}

TEST(TesseractCoreUnit, ContactTrajectorySubstepResultsUnit)  // NOLINT
{
  // Test constructor with start and end states
  {
    int substep_number = 5;
    Eigen::VectorXd start_state(3);
    Eigen::VectorXd end_state(3);
    start_state << 1.0, 2.0, 3.0;
    end_state << 4.0, 5.0, 6.0;

    tesseract_collision::ContactTrajectorySubstepResults results(substep_number, start_state, end_state);

    EXPECT_EQ(results.substep, substep_number);
    EXPECT_TRUE(results.state0.isApprox(start_state));
    EXPECT_TRUE(results.state1.isApprox(end_state));
    EXPECT_EQ(results.numContacts(), 0);
  }

  // Test constructor with single state
  {
    int substep_number = 3;
    Eigen::VectorXd state(2);
    state << 1.5, 2.5;

    tesseract_collision::ContactTrajectorySubstepResults results(substep_number, state);

    EXPECT_EQ(results.substep, substep_number);
    EXPECT_TRUE(results.state0.isApprox(state));
    EXPECT_TRUE(results.state1.isApprox(state));
    EXPECT_EQ(results.numContacts(), 0);
  }

  // Test with contacts
  {
    int substep_number = 2;
    Eigen::VectorXd state(1);
    state << 1.0;

    tesseract_collision::ContactTrajectorySubstepResults results(substep_number, state);

    // Create contact results
    auto key1 = tesseract_common::makeOrderedLinkPair("link1", "link2");
    auto key2 = tesseract_common::makeOrderedLinkPair("link3", "link4");

    tesseract_collision::ContactResult cr1;
    cr1.distance = -0.1;
    cr1.link_names[0] = "link1";
    cr1.link_names[1] = "link2";

    tesseract_collision::ContactResult cr2;
    cr2.distance = -0.2;
    cr2.link_names[0] = "link3";
    cr2.link_names[1] = "link4";

    tesseract_collision::ContactResultVector crv1 = { cr1 };
    tesseract_collision::ContactResultVector crv2 = { cr2 };

    results.contacts.addContactResult(key1, crv1);
    results.contacts.addContactResult(key2, crv2);

    EXPECT_EQ(results.numContacts(), 2);

    // Test worstCollision
    tesseract_collision::ContactResultVector worst = results.worstCollision();
    EXPECT_EQ(worst.size(), 1);
    EXPECT_EQ(worst[0].distance, -0.2);
    EXPECT_EQ(worst[0].link_names[0], "link3");
    EXPECT_EQ(worst[0].link_names[1], "link4");
  }

  // Test with no contacts
  {
    tesseract_collision::ContactTrajectorySubstepResults results;
    tesseract_collision::ContactResultVector worst = results.worstCollision();
    EXPECT_EQ(worst.size(), 0);
  }
}
TEST(TesseractCoreUnit, ContactTrajectoryStepResultsUnit)  // NOLINT
{
  // Test constructor with start and end states and num_substeps
  {
    int step_number = 3;
    Eigen::VectorXd start_state(3);
    Eigen::VectorXd end_state(3);
    start_state << 1.0, 2.0, 3.0;
    end_state << 4.0, 5.0, 6.0;
    int num_substeps = 5;

    tesseract_collision::ContactTrajectoryStepResults results(step_number, start_state, end_state, num_substeps);

    EXPECT_EQ(results.step, step_number);
    EXPECT_TRUE(results.state0.isApprox(start_state));
    EXPECT_TRUE(results.state1.isApprox(end_state));
    EXPECT_EQ(results.total_substeps, num_substeps);
    EXPECT_EQ(results.substeps.size(), num_substeps);
    EXPECT_EQ(results.numSubsteps(), num_substeps);
    EXPECT_EQ(results.numContacts(), 0);
  }

  // Test constructor with single state
  {
    int step_number = 2;
    Eigen::VectorXd state(2);
    state << 1.5, 2.5;

    tesseract_collision::ContactTrajectoryStepResults results(step_number, state);
    // The constructor with a single state defaults to 2 substeps
    int expected_substeps = 2;

    EXPECT_EQ(results.step, step_number);
    EXPECT_TRUE(results.state0.isApprox(state));
    EXPECT_TRUE(results.state1.isApprox(state));
    EXPECT_EQ(results.total_substeps, expected_substeps);
    EXPECT_EQ(results.substeps.size(), expected_substeps);
    EXPECT_EQ(results.numSubsteps(), expected_substeps);
    EXPECT_EQ(results.numContacts(), 0);
  }

  // Test resize method
  {
    int step_number = 1;
    Eigen::VectorXd state(1);
    state << 1.0;

    tesseract_collision::ContactTrajectoryStepResults results(step_number, state);
    EXPECT_EQ(results.substeps.size(), 2);  // Default is 2

    int new_size = 4;
    results.resize(new_size);

    EXPECT_EQ(results.total_substeps, new_size);
    EXPECT_EQ(results.substeps.size(), new_size);
    EXPECT_EQ(results.numSubsteps(), new_size);
  }

  // Test with contacts in substeps
  {
    int step_number = 1;
    Eigen::VectorXd start_state(2);
    Eigen::VectorXd end_state(2);
    start_state << 0.0, 0.0;
    end_state << 1.0, 1.0;
    int num_substeps = 3;

    tesseract_collision::ContactTrajectoryStepResults results(step_number, start_state, end_state, num_substeps);

    // Initialize substep states and indices
    for (size_t i = 0; i < results.substeps.size(); ++i)
    {
      auto& substep = results.substeps[i];
      if (substep.state0.size() == 0)
        substep.state0 = start_state;
      if (substep.state1.size() == 0)
        substep.state1 = end_state;
      if (substep.substep < 0)
        substep.substep = static_cast<int>(i);
    }

    // Create contact results for first substep
    auto key1 = tesseract_common::makeOrderedLinkPair("link1", "link2");
    auto key2 = tesseract_common::makeOrderedLinkPair("link3", "link4");

    tesseract_collision::ContactResult cr1;
    cr1.distance = -0.1;
    cr1.link_names[0] = "link1";
    cr1.link_names[1] = "link2";

    tesseract_collision::ContactResult cr2;
    cr2.distance = -0.2;
    cr2.link_names[0] = "link3";
    cr2.link_names[1] = "link4";

    tesseract_collision::ContactResultVector crv1 = { cr1 };
    tesseract_collision::ContactResultVector crv2 = { cr2 };

    results.substeps[0].contacts.addContactResult(key1, crv1);

    // Create contact results for second substep
    tesseract_collision::ContactResult cr3;
    cr3.distance = -0.3;
    cr3.link_names[0] = "link1";
    cr3.link_names[1] = "link2";

    tesseract_collision::ContactResultVector crv3 = { cr3 };

    results.substeps[1].contacts.addContactResult(key1, crv3);
    results.substeps[1].contacts.addContactResult(key2, crv2);

    // Test numContacts
    EXPECT_EQ(results.numContacts(), 3);

    // Test worstSubstep
    tesseract_collision::ContactTrajectorySubstepResults worst_substep = results.worstSubstep();
    EXPECT_EQ(worst_substep.substep, 1);

    // Test worstCollision
    tesseract_collision::ContactResultVector worst_collision = results.worstCollision();
    EXPECT_EQ(worst_collision.size(), 1);
    if (!worst_collision.empty())
    {
      EXPECT_EQ(worst_collision[0].distance, -0.3);
    }

    // Test mostCollisionsSubstep
    tesseract_collision::ContactTrajectorySubstepResults most_collisions = results.mostCollisionsSubstep();
    EXPECT_EQ(most_collisions.substep, 1);
    EXPECT_EQ(most_collisions.numContacts(), 2);
  }

  // Test with no contacts
  {
    int step_number = 0;
    Eigen::VectorXd state(1);
    state << 0.0;

    tesseract_collision::ContactTrajectoryStepResults results(step_number, state);

    // Explicitly initialize substep states and indices
    for (size_t i = 0; i < results.substeps.size(); ++i)
    {
      auto& substep = results.substeps[i];
      if (substep.state0.size() == 0)
        substep.state0 = state;
      if (substep.state1.size() == 0)
        substep.state1 = state;
      if (substep.substep < 0)
        substep.substep = static_cast<int>(i);
    }

    EXPECT_EQ(results.numContacts(), 0);

    tesseract_collision::ContactTrajectorySubstepResults worst_substep = results.worstSubstep();
    EXPECT_EQ(worst_substep.substep, -1);  // Expect default index when no contacts

    tesseract_collision::ContactResultVector worst_collision = results.worstCollision();
    EXPECT_EQ(worst_collision.size(), 0);

    tesseract_collision::ContactTrajectorySubstepResults most_collisions = results.mostCollisionsSubstep();
    EXPECT_EQ(most_collisions.substep, -1);  // Expect default index when no contacts
    EXPECT_EQ(most_collisions.numContacts(), 0);
  }
}

TEST(TesseractCoreUnit, ContactTrajectoryResultsUnit)  // NOLINT
{
  // Test constructor with joint names
  {
    std::vector<std::string> joint_names = { "joint1", "joint2", "joint3" };
    tesseract_collision::ContactTrajectoryResults results(joint_names);

    EXPECT_EQ(results.joint_names, joint_names);
    EXPECT_EQ(results.total_steps, 0);
    EXPECT_EQ(results.steps.size(), 0);
    EXPECT_EQ(results.numSteps(), 0);
    EXPECT_EQ(results.numContacts(), 0);
  }

  // Test constructor with joint names and num_steps
  {
    std::vector<std::string> joint_names = { "joint1", "joint2" };
    int num_steps = 3;

    tesseract_collision::ContactTrajectoryResults results(joint_names, num_steps);

    EXPECT_EQ(results.joint_names, joint_names);
    EXPECT_EQ(results.total_steps, num_steps);
    EXPECT_EQ(results.steps.size(), num_steps);
    EXPECT_EQ(results.numSteps(), num_steps);
    EXPECT_EQ(results.numContacts(), 0);
  }

  // Test resize method
  {
    std::vector<std::string> joint_names = { "joint1" };

    tesseract_collision::ContactTrajectoryResults results(joint_names);
    EXPECT_EQ(results.steps.size(), 0);

    results.resize(5);
    EXPECT_EQ(results.total_steps, 5);
    EXPECT_EQ(results.steps.size(), 5);
    EXPECT_EQ(results.numSteps(), 5);
  }

  // Test with contacts in steps
  {
    std::vector<std::string> joint_names = { "joint1", "joint2" };
    int num_steps = 2;

    tesseract_collision::ContactTrajectoryResults results(joint_names, num_steps);

    // Setup Step 1
    Eigen::VectorXd step1_start(2);
    Eigen::VectorXd step1_end(2);
    step1_start << 0.0, 0.0;
    step1_end << 1.0, 1.0;

    results.steps[0].step = 0;
    results.steps[0].state0 = step1_start;
    results.steps[0].state1 = step1_end;
    results.steps[0].resize(2);  // 2 substeps for first step

    // Create contact results for step 1, substep 0
    auto key1 = tesseract_common::makeOrderedLinkPair("link1", "link2");

    tesseract_collision::ContactResult cr1;
    cr1.distance = -0.1;
    cr1.link_names[0] = "link1";
    cr1.link_names[1] = "link2";

    tesseract_collision::ContactResultVector crv1 = { cr1 };

    results.steps[0].substeps[0].substep = 0;
    results.steps[0].substeps[0].contacts.addContactResult(key1, crv1);

    // Setup Step 2
    Eigen::VectorXd step2_start(2);
    Eigen::VectorXd step2_end(2);
    step2_start << 1.0, 1.0;
    step2_end << 2.0, 2.0;

    results.steps[1].step = 1;
    results.steps[1].state0 = step2_start;
    results.steps[1].state1 = step2_end;
    results.steps[1].resize(3);  // 3 substeps for second step

    // Create contact results for step 2, substep 1
    auto key2 = tesseract_common::makeOrderedLinkPair("link3", "link4");

    tesseract_collision::ContactResult cr2;
    cr2.distance = -0.3;
    cr2.link_names[0] = "link3";
    cr2.link_names[1] = "link4";

    tesseract_collision::ContactResultVector crv2 = { cr2 };

    results.steps[1].substeps[1].substep = 1;
    results.steps[1].substeps[1].contacts.addContactResult(key2, crv2);

    // Create another contact for step 2, substep 2
    tesseract_collision::ContactResult cr3;
    cr3.distance = -0.2;
    cr3.link_names[0] = "link3";
    cr3.link_names[1] = "link4";

    tesseract_collision::ContactResultVector crv3 = { cr3 };

    results.steps[1].substeps[2].substep = 2;
    results.steps[1].substeps[2].contacts.addContactResult(key2, crv3);

    // Test numContacts
    EXPECT_EQ(results.numContacts(), 3);

    // Test worstStep
    tesseract_collision::ContactTrajectoryStepResults worst_step = results.worstStep();
    EXPECT_EQ(worst_step.step, 1);

    // Test worstCollision
    tesseract_collision::ContactResultVector worst_collision = results.worstCollision();
    EXPECT_EQ(worst_collision.size(), 1);
    EXPECT_EQ(worst_collision[0].distance, -0.3);
    EXPECT_EQ(worst_collision[0].link_names[0], "link3");
    EXPECT_EQ(worst_collision[0].link_names[1], "link4");

    // Test mostCollisionsStep
    tesseract_collision::ContactTrajectoryStepResults most_collisions = results.mostCollisionsStep();
    EXPECT_EQ(most_collisions.step, 1);
    EXPECT_EQ(most_collisions.numContacts(), 2);

    // Test table output methods - only checking that they don't crash
    EXPECT_NO_THROW(results.trajectoryCollisionResultsTable());

    std::stringstream ss = results.trajectoryCollisionResultsTable();
    EXPECT_FALSE(ss.str().empty());

    // Test frequency output method
    EXPECT_NO_THROW(results.collisionFrequencyPerLink());

    std::stringstream freq_ss = results.collisionFrequencyPerLink();
    EXPECT_FALSE(freq_ss.str().empty());
  }

  // Test with no contacts
  {
    std::vector<std::string> joint_names = { "joint1" };
    tesseract_collision::ContactTrajectoryResults results(joint_names);

    EXPECT_EQ(results.numContacts(), 0);

    tesseract_collision::ContactTrajectoryStepResults worst_step = results.worstStep();
    EXPECT_EQ(worst_step.step, -1);

    tesseract_collision::ContactResultVector worst_collision = results.worstCollision();
    EXPECT_EQ(worst_collision.size(), 0);

    tesseract_collision::ContactTrajectoryStepResults most_collisions = results.mostCollisionsStep();
    EXPECT_EQ(most_collisions.step, -1);
    EXPECT_EQ(most_collisions.numContacts(), 0);

    // Empty results should still produce output without crashing
    EXPECT_NO_THROW(results.trajectoryCollisionResultsTable());
    EXPECT_NO_THROW(results.collisionFrequencyPerLink());

    std::stringstream ss = results.trajectoryCollisionResultsTable();
    EXPECT_FALSE(ss.str().empty());
    EXPECT_TRUE(ss.str().find("No contacts detected") != std::string::npos);

    std::stringstream freq_ss = results.collisionFrequencyPerLink();
    EXPECT_FALSE(freq_ss.str().empty());
    EXPECT_TRUE(freq_ss.str().find("No contacts detected") != std::string::npos);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
