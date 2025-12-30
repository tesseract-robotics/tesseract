#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <vector>
#include <string>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/contact_allowed_validator.h>
#include <tesseract_common/unit_test_utils.h>
#include <tesseract_common/utils.h>
#include <tesseract_common/yaml_utils.h>
#include <tesseract_common/cereal_serialization.h>

#include <tesseract_collision/core/common.h>
#include <tesseract_collision/core/types.h>
#include <tesseract_collision/core/yaml_extensions.h>
#include <tesseract_collision/core/cereal_serialization.h>

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
  // Use non-default values for all fields
  const std::string yaml_string = R"(
    default_margin: 0.123
    pair_margin_override_type: MODIFY
    pair_margin_data:
      [linkA, linkB]: 0.456
    acm_override_type: OR
    acm:
      [linkA, linkB]: "always"
    modify_object_enabled:
      object1: true
      object2: false
  )";

  tesseract_collision::ContactManagerConfig data_original;
  data_original.default_margin = 0.123;
  data_original.pair_margin_override_type = tesseract_collision::CollisionMarginPairOverrideType::MODIFY;
  data_original.pair_margin_data.setCollisionMargin("linkA", "linkB", 0.456);
  data_original.acm_override_type = tesseract_collision::ACMOverrideType::OR;
  data_original.acm.addAllowedCollision("linkA", "linkB", "always");
  data_original.modify_object_enabled["object1"] = true;
  data_original.modify_object_enabled["object2"] = false;

  // Decode test
  {
    tesseract_collision::ContactManagerConfig cm;
    YAML::Node n = YAML::Load(yaml_string);
    auto success = YAML::convert<tesseract_collision::ContactManagerConfig>::decode(n, cm);
    EXPECT_TRUE(success);
    EXPECT_EQ(cm, data_original);
  }

  // Encode test: compare YAML output to expected YAML
  {
    YAML::Node output_n = YAML::convert<tesseract_collision::ContactManagerConfig>::encode(data_original);

    // Check that the YAML node contains the expected values
    EXPECT_TRUE(output_n["default_margin"]);
    EXPECT_NEAR(output_n["default_margin"].as<double>(), 0.123, 1e-8);

    EXPECT_TRUE(output_n["pair_margin_override_type"]);
    EXPECT_EQ(output_n["pair_margin_override_type"].as<std::string>(), "MODIFY");

    EXPECT_TRUE(output_n["pair_margin_data"]);
    auto margins = output_n["pair_margin_data"];
    ASSERT_TRUE(margins && margins.IsMap());
    bool found_margin = false;
    for (const auto& margin : margins)
    {
      // Keys are encoded as a YAML flow sequence [linkA, linkB]
      const YAML::Node& key = margin.first;
      if (key.IsSequence() && key.size() == 2)
      {
        const auto a = key[0].as<std::string>();
        const auto b = key[1].as<std::string>();
        if ((a == "linkA" && b == "linkB") || (a == "linkB" && b == "linkA"))
        {
          EXPECT_NEAR(margin.second.as<double>(), 0.456, 1e-8);
          found_margin = true;
          break;
        }
      }
    }
    EXPECT_TRUE(found_margin);

    EXPECT_TRUE(output_n["acm_override_type"]);
    EXPECT_EQ(output_n["acm_override_type"].as<std::string>(), "OR");

    EXPECT_TRUE(output_n["acm"]);
    auto allowed_collisions = output_n["acm"];
    ASSERT_TRUE(allowed_collisions && allowed_collisions.IsMap());
    bool found_acm = false;
    for (const auto& entry : allowed_collisions)
    {
      const YAML::Node& key = entry.first;
      if (key.IsSequence() && key.size() == 2)
      {
        const auto a = key[0].as<std::string>();
        const auto b = key[1].as<std::string>();
        bool links_match = (a == "linkA" && b == "linkB") || (a == "linkB" && b == "linkA");
        if (links_match && entry.second.as<std::string>() == "always")
        {
          found_acm = true;
          break;
        }
      }
    }
    EXPECT_TRUE(found_acm);

    EXPECT_TRUE(output_n["modify_object_enabled"]);
    EXPECT_EQ(output_n["modify_object_enabled"]["object1"].as<bool>(), true);
    EXPECT_EQ(output_n["modify_object_enabled"]["object2"].as<bool>(), false);
  }

  // Encode-decode cycle test
  {
    YAML::Node output_n = YAML::convert<tesseract_collision::ContactManagerConfig>::encode(data_original);
    tesseract_collision::ContactManagerConfig roundtrip;
    auto success = YAML::convert<tesseract_collision::ContactManagerConfig>::decode(output_n, roundtrip);
    EXPECT_TRUE(success);
    EXPECT_EQ(roundtrip, data_original);
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
  // Use non-default values for all fields
  const std::string yaml_string = R"(
    type: LIMITED
    calculate_penetration: false
    calculate_distance: false
    contact_limit: 42
  )";

  tesseract_collision::ContactRequest data_original;
  data_original.type = tesseract_collision::ContactTestType::LIMITED;
  data_original.calculate_penetration = false;
  data_original.calculate_distance = false;
  data_original.contact_limit = 42;

  // Decode test
  {
    tesseract_collision::ContactRequest cr;
    YAML::Node n = YAML::Load(yaml_string);
    auto success = YAML::convert<tesseract_collision::ContactRequest>::decode(n, cr);
    EXPECT_TRUE(success);
    EXPECT_EQ(cr, data_original);
  }

  // Encode test: compare YAML output to expected YAML
  {
    YAML::Node output_n = YAML::convert<tesseract_collision::ContactRequest>::encode(data_original);

    // Check that the YAML node contains the expected values
    EXPECT_TRUE(output_n["type"]);
    EXPECT_EQ(output_n["type"].as<std::string>(), "LIMITED");

    EXPECT_TRUE(output_n["calculate_penetration"]);
    EXPECT_EQ(output_n["calculate_penetration"].as<bool>(), false);

    EXPECT_TRUE(output_n["calculate_distance"]);
    EXPECT_EQ(output_n["calculate_distance"].as<bool>(), false);

    EXPECT_TRUE(output_n["contact_limit"]);
    EXPECT_EQ(output_n["contact_limit"].as<long>(), 42);
  }

  // Encode-decode cycle test
  {
    YAML::Node output_n = YAML::convert<tesseract_collision::ContactRequest>::encode(data_original);
    tesseract_collision::ContactRequest roundtrip;
    auto success = YAML::convert<tesseract_collision::ContactRequest>::decode(output_n, roundtrip);
    EXPECT_TRUE(success);
    EXPECT_EQ(roundtrip, data_original);
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
    EXPECT_EQ(config.exit_condition, tesseract_collision::CollisionCheckExitType::FIRST);

    tesseract_common::testSerialization<tesseract_collision::CollisionCheckConfig>(config, "CollisionCheckConfig");
  }
  {
    tesseract_collision::ContactRequest request;
    tesseract_collision::CollisionCheckConfig config(request,
                                                     tesseract_collision::CollisionEvaluatorType::LVS_DISCRETE,
                                                     0.5,
                                                     tesseract_collision::CollisionCheckProgramType::ALL_EXCEPT_START,
                                                     tesseract_collision::CollisionCheckExitType::ONE_PER_STEP);

    EXPECT_EQ(config.contact_request, request);
    EXPECT_EQ(config.type, tesseract_collision::CollisionEvaluatorType::LVS_DISCRETE);
    EXPECT_TRUE(tesseract_common::almostEqualRelativeAndAbs(config.longest_valid_segment_length, 0.5));
    EXPECT_EQ(config.check_program_mode, tesseract_collision::CollisionCheckProgramType::ALL_EXCEPT_START);
    EXPECT_EQ(config.exit_condition, tesseract_collision::CollisionCheckExitType::ONE_PER_STEP);

    tesseract_common::testSerialization<tesseract_collision::CollisionCheckConfig>(config, "CollisionCheckConfig");
  }
}

TEST(TesseractCoreUnit, CollisionCheckConfigYamlUnit)  // NOLINT
{
  // Use non-default values for all fields
  const std::string yaml_string = R"(
    contact_request:
      type: CLOSEST
      calculate_penetration: false
      calculate_distance: false
      contact_limit: 10
    type: LVS_CONTINUOUS
    longest_valid_segment_length: 0.123
    check_program_mode: ALL_EXCEPT_END
    exit_condition: ONE_PER_STEP
  )";

  tesseract_collision::ContactRequest cr_original;
  cr_original.type = tesseract_collision::ContactTestType::CLOSEST;
  cr_original.calculate_penetration = false;
  cr_original.calculate_distance = false;
  cr_original.contact_limit = 10;

  tesseract_collision::CollisionCheckConfig data_original;
  data_original.contact_request = cr_original;
  data_original.type = tesseract_collision::CollisionEvaluatorType::LVS_CONTINUOUS;
  data_original.longest_valid_segment_length = 0.123;
  data_original.check_program_mode = tesseract_collision::CollisionCheckProgramType::ALL_EXCEPT_END;
  data_original.exit_condition = tesseract_collision::CollisionCheckExitType::ONE_PER_STEP;

  // Decode test
  {
    tesseract_collision::CollisionCheckConfig ccc;
    YAML::Node n = YAML::Load(yaml_string);
    auto success = YAML::convert<tesseract_collision::CollisionCheckConfig>::decode(n, ccc);
    EXPECT_TRUE(success);
    EXPECT_EQ(ccc, data_original);
  }

  // Encode test: compare YAML output to expected YAML
  {
    YAML::Node output_n = YAML::convert<tesseract_collision::CollisionCheckConfig>::encode(data_original);

    // Check that the YAML node contains the expected values
    EXPECT_TRUE(output_n["contact_request"]);
    auto contact_request_node = output_n["contact_request"];
    EXPECT_TRUE(contact_request_node["type"]);
    EXPECT_EQ(contact_request_node["type"].as<std::string>(), "CLOSEST");
    EXPECT_TRUE(contact_request_node["calculate_penetration"]);
    EXPECT_EQ(contact_request_node["calculate_penetration"].as<bool>(), false);
    EXPECT_TRUE(contact_request_node["calculate_distance"]);
    EXPECT_EQ(contact_request_node["calculate_distance"].as<bool>(), false);
    EXPECT_TRUE(contact_request_node["contact_limit"]);
    EXPECT_EQ(contact_request_node["contact_limit"].as<long>(), 10);

    EXPECT_TRUE(output_n["type"]);
    EXPECT_EQ(output_n["type"].as<std::string>(), "LVS_CONTINUOUS");

    EXPECT_TRUE(output_n["longest_valid_segment_length"]);
    EXPECT_NEAR(output_n["longest_valid_segment_length"].as<double>(), 0.123, 1e-8);

    EXPECT_TRUE(output_n["check_program_mode"]);
    EXPECT_EQ(output_n["check_program_mode"].as<std::string>(), "ALL_EXCEPT_END");

    EXPECT_TRUE(output_n["exit_condition"]);
    EXPECT_EQ(output_n["exit_condition"].as<std::string>(), "ONE_PER_STEP");
  }

  // Encode-decode cycle test
  {
    YAML::Node output_n = YAML::convert<tesseract_collision::CollisionCheckConfig>::encode(data_original);
    tesseract_collision::CollisionCheckConfig roundtrip;
    auto success = YAML::convert<tesseract_collision::CollisionCheckConfig>::decode(output_n, roundtrip);
    EXPECT_TRUE(success);
    EXPECT_EQ(roundtrip, data_original);
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
    EXPECT_FALSE(ss.str().find("No contacts detected") != std::string::npos);
    std::cout << ss.str();

    // Test frequency output method
    EXPECT_NO_THROW(results.collisionFrequencyPerLink());

    std::stringstream freq_ss = results.collisionFrequencyPerLink();
    EXPECT_FALSE(freq_ss.str().empty());

    EXPECT_TRUE(static_cast<bool>(results));
    EXPECT_FALSE(freq_ss.str().find("No contacts detected") != std::string::npos);

    // Test condensed summary method
    EXPECT_NO_THROW(results.condensedSummary());

    std::stringstream condensed_ss = results.condensedSummary();
    EXPECT_FALSE(condensed_ss.str().empty());
    EXPECT_FALSE(condensed_ss.str().find("No contacts detected") != std::string::npos);

    // Check that the condensed summary contains expected format: "step.substep: [link1, link2]->distance"
    std::string condensed_output = condensed_ss.str();
    std::cout << "Condensed Summary Output:\n" << condensed_output;

    // Should contain "0.0: [link1, link2]" for first collision at step 0, substep 0
    EXPECT_TRUE(condensed_output.find("0.0: [link1, link2]") != std::string::npos);

    // Should contain "1.3: [link3, link4]" for first collision at step 1, substep 1 (1/3 = 0.3, so 1.3)
    EXPECT_TRUE(condensed_output.find("1.3: [link3, link4]") != std::string::npos);
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

    // Test condensed summary method with no contacts
    EXPECT_NO_THROW(results.condensedSummary());

    std::stringstream condensed_ss = results.condensedSummary();
    EXPECT_FALSE(condensed_ss.str().empty());
    EXPECT_TRUE(condensed_ss.str().find("No contacts detected") != std::string::npos);

    EXPECT_FALSE(static_cast<bool>(results));
  }

  // Test addContact methods
  {
    std::vector<std::string> joint_names = { "joint1", "joint2", "joint3" };
    int num_steps = 3;

    tesseract_collision::ContactTrajectoryResults results(joint_names, num_steps);

    // Test basic addContact functionality - ensure higher level calls properly initialize lower levels
    {
      int step_number = 0;
      int substep_number = 1;
      int num_substeps = 3;

      Eigen::VectorXd start_state(3);
      Eigen::VectorXd end_state(3);
      start_state << 1.0, 2.0, 3.0;
      end_state << 4.0, 5.0, 6.0;

      tesseract_common::TrajArray traj_array_with_substates(3, start_state.size());
      traj_array_with_substates.row(0) = start_state;
      traj_array_with_substates.row(1) = (start_state + end_state) / 2.0;  // Intermediate state
      traj_array_with_substates.row(2) = end_state;

      // Create contact results to add
      tesseract_collision::ContactResultMap contacts;
      auto key1 = tesseract_common::makeOrderedLinkPair("link1", "link2");
      auto key2 = tesseract_common::makeOrderedLinkPair("link3", "link4");

      tesseract_collision::ContactResult cr1;
      cr1.distance = -0.15;
      cr1.link_names[0] = "link1";
      cr1.link_names[1] = "link2";

      tesseract_collision::ContactResult cr2;
      cr2.distance = -0.25;
      cr2.link_names[0] = "link3";
      cr2.link_names[1] = "link4";

      contacts.addContactResult(key1, cr1);
      contacts.addContactResult(key2, cr2);

      // Verify step is initially empty/uninitialized
      EXPECT_EQ(results.steps[static_cast<std::size_t>(step_number)].step, -1);  // Default uninitialized step value
      EXPECT_EQ(results.steps[static_cast<std::size_t>(step_number)].total_substeps, 0);
      EXPECT_EQ(results.steps[static_cast<std::size_t>(step_number)].substeps.size(), 0);

      // Add the contact - this should automatically initialize the step with the right number of substeps
      results.addContact(step_number,
                         substep_number,
                         num_substeps,
                         start_state,
                         end_state,
                         traj_array_with_substates.row(1),
                         traj_array_with_substates.row(2),
                         contacts);

      // Verify the step was properly initialized by the addContact call
      EXPECT_EQ(results.steps[static_cast<std::size_t>(step_number)].step, step_number);
      EXPECT_EQ(results.steps[static_cast<std::size_t>(step_number)].total_substeps, num_substeps);
      EXPECT_EQ(results.steps[static_cast<std::size_t>(step_number)].substeps.size(), num_substeps);
      EXPECT_TRUE(results.steps[static_cast<std::size_t>(step_number)].state0.isApprox(start_state));
      EXPECT_TRUE(results.steps[static_cast<std::size_t>(step_number)].state1.isApprox(end_state));

      // Verify the contact was added correctly to the specific substep
      EXPECT_EQ(results.steps[static_cast<std::size_t>(step_number)]
                    .substeps[static_cast<std::size_t>(substep_number)]
                    .substep,
                substep_number);
      EXPECT_TRUE(results.steps[static_cast<std::size_t>(step_number)]
                      .substeps[static_cast<std::size_t>(substep_number)]
                      .state0.isApprox(traj_array_with_substates.row(1).transpose()));
      EXPECT_TRUE(results.steps[static_cast<std::size_t>(step_number)]
                      .substeps[static_cast<std::size_t>(substep_number)]
                      .state1.isApprox(traj_array_with_substates.row(2).transpose()));
      EXPECT_EQ(results.steps[static_cast<std::size_t>(step_number)]
                    .substeps[static_cast<std::size_t>(substep_number)]
                    .numContacts(),
                2);

      // Check that the contacts were copied correctly
      const auto& substep_contacts = results.steps[static_cast<std::size_t>(step_number)]
                                         .substeps[static_cast<std::size_t>(substep_number)]
                                         .contacts;
      EXPECT_EQ(substep_contacts.count(), 2);

      auto it1 = substep_contacts.find(key1);
      auto it2 = substep_contacts.find(key2);
      EXPECT_NE(it1, substep_contacts.end());
      EXPECT_NE(it2, substep_contacts.end());

      if (it1 != substep_contacts.end())
      {
        EXPECT_EQ(it1->second.size(), 1);
        EXPECT_EQ(it1->second[0].distance, -0.15);
        EXPECT_EQ(it1->second[0].link_names[0], "link1");
        EXPECT_EQ(it1->second[0].link_names[1], "link2");
      }

      if (it2 != substep_contacts.end())
      {
        EXPECT_EQ(it2->second.size(), 1);
        EXPECT_EQ(it2->second[0].distance, -0.25);
        EXPECT_EQ(it2->second[0].link_names[0], "link3");
        EXPECT_EQ(it2->second[0].link_names[1], "link4");
      }
    }

    // Test multiple contacts to the same step/substep - verify automatic initialization
    {
      int step_number = 1;
      int substep_number = 0;
      int num_substeps = 2;

      Eigen::VectorXd start_state(2);
      Eigen::VectorXd end_state(2);
      start_state << 0.5, 1.5;
      end_state << 2.5, 3.5;

      // Verify step is initially uninitialized
      EXPECT_EQ(results.steps[static_cast<std::size_t>(step_number)].step, -1);
      EXPECT_EQ(results.steps[static_cast<std::size_t>(step_number)].total_substeps, 0);

      // Add first set of contacts - this should automatically initialize the step
      tesseract_collision::ContactResultMap contacts1;
      auto key1 = tesseract_common::makeOrderedLinkPair("linkA", "linkB");

      tesseract_collision::ContactResult cr1;
      cr1.distance = -0.1;
      cr1.link_names[0] = "linkA";
      cr1.link_names[1] = "linkB";

      contacts1.addContactResult(key1, cr1);

      results.addContact(
          step_number, substep_number, num_substeps, start_state, end_state, start_state, end_state, contacts1);

      // Verify step was automatically initialized
      EXPECT_EQ(results.steps[static_cast<std::size_t>(step_number)].step, step_number);
      EXPECT_EQ(results.steps[static_cast<std::size_t>(step_number)].total_substeps, num_substeps);
      EXPECT_EQ(results.steps[static_cast<std::size_t>(step_number)].substeps.size(), num_substeps);
      EXPECT_TRUE(results.steps[static_cast<std::size_t>(step_number)].state0.isApprox(start_state));
      EXPECT_TRUE(results.steps[static_cast<std::size_t>(step_number)].state1.isApprox(end_state));

      // Verify first contact was added
      EXPECT_EQ(results.steps[static_cast<std::size_t>(step_number)]
                    .substeps[static_cast<std::size_t>(substep_number)]
                    .numContacts(),
                1);

      // Add second set of contacts to same substep (this should replace the previous ones)
      Eigen::VectorXd new_start_state(2);
      Eigen::VectorXd new_end_state(2);
      new_start_state << 1.0, 2.0;
      new_end_state << 3.0, 4.0;

      tesseract_collision::ContactResultMap contacts2;
      auto key2 = tesseract_common::makeOrderedLinkPair("linkC", "linkD");

      tesseract_collision::ContactResult cr2;
      cr2.distance = -0.3;
      cr2.link_names[0] = "linkC";
      cr2.link_names[1] = "linkD";

      contacts2.addContactResult(key2, cr2);

      results.addContact(step_number,
                         substep_number,
                         num_substeps,
                         new_start_state,
                         new_end_state,
                         new_start_state,
                         new_end_state,
                         contacts2);

      // Verify the substep was updated with new states and contacts
      EXPECT_TRUE(results.steps[static_cast<std::size_t>(step_number)]
                      .substeps[static_cast<std::size_t>(substep_number)]
                      .state0.isApprox(new_start_state));
      EXPECT_TRUE(results.steps[static_cast<std::size_t>(step_number)]
                      .substeps[static_cast<std::size_t>(substep_number)]
                      .state1.isApprox(new_end_state));
      EXPECT_EQ(results.steps[static_cast<std::size_t>(step_number)]
                    .substeps[static_cast<std::size_t>(substep_number)]
                    .numContacts(),
                1);

      // Verify it's the new contact
      const auto& substep_contacts = results.steps[static_cast<std::size_t>(step_number)]
                                         .substeps[static_cast<std::size_t>(substep_number)]
                                         .contacts;
      auto it = substep_contacts.find(key2);
      EXPECT_NE(it, substep_contacts.end());
      if (it != substep_contacts.end())
      {
        EXPECT_EQ(it->second[0].distance, -0.3);
        EXPECT_EQ(it->second[0].link_names[0], "linkC");
        EXPECT_EQ(it->second[0].link_names[1], "linkD");
      }
    }

    // Test error cases - out of range step number
    {
      int invalid_step_number = 5;  // Out of range (we have 3 steps: 0, 1, 2)
      int substep_number = 0;
      int num_substeps = 1;

      Eigen::VectorXd state(1);
      state << 1.0;

      tesseract_collision::ContactResultMap contacts;

      EXPECT_THROW(
          results.addContact(invalid_step_number, substep_number, num_substeps, state, state, state, state, contacts),
          std::out_of_range);
    }

    // Test error cases - negative step number
    {
      int invalid_step_number = -1;
      int substep_number = 0;
      int num_substeps = 1;

      Eigen::VectorXd state(1);
      state << 1.0;

      tesseract_collision::ContactResultMap contacts;

      EXPECT_THROW(
          results.addContact(invalid_step_number, substep_number, num_substeps, state, state, state, state, contacts),
          std::out_of_range);
    }

    // Test total contact counting after adding contacts
    EXPECT_GT(results.numContacts(), 0);
    EXPECT_TRUE(static_cast<bool>(results));
  }

  // Test ContactTrajectoryStepResults addContact method directly
  {
    int step_number = 2;
    Eigen::VectorXd start_state(2);
    Eigen::VectorXd end_state(2);
    start_state << 1.0, 2.0;
    end_state << 3.0, 4.0;
    int initial_substeps = 2;

    tesseract_collision::ContactTrajectoryStepResults step_results(
        step_number, start_state, end_state, initial_substeps);

    // Test addContact to a valid substep within existing range
    {
      int substep_number = 1;  // Within the current 2 substeps
      int same_substeps = 2;   // Same number of substeps

      Eigen::VectorXd substep_start(2);
      Eigen::VectorXd substep_end(2);
      substep_start << 1.5, 2.0;
      substep_end << 2.0, 2.5;

      tesseract_collision::ContactResultMap contacts;
      auto key = tesseract_common::makeOrderedLinkPair("testLink1", "testLink2");

      tesseract_collision::ContactResult cr;
      cr.distance = -0.05;
      cr.link_names[0] = "testLink1";
      cr.link_names[1] = "testLink2";

      contacts.addContactResult(key, cr);

      // This should work since substep is within range and no resize is needed
      step_results.addContact(
          step_number, substep_number, same_substeps, start_state, end_state, substep_start, substep_end, contacts);

      // Verify no resize occurred
      EXPECT_EQ(step_results.total_substeps, initial_substeps);
      EXPECT_EQ(step_results.substeps.size(), initial_substeps);

      // Verify the contact was added correctly
      EXPECT_EQ(step_results.substeps[static_cast<std::size_t>(substep_number)].substep, substep_number);
      EXPECT_TRUE(step_results.substeps[static_cast<std::size_t>(substep_number)].state0.isApprox(substep_start));
      EXPECT_TRUE(step_results.substeps[static_cast<std::size_t>(substep_number)].state1.isApprox(substep_end));
      EXPECT_EQ(step_results.substeps[static_cast<std::size_t>(substep_number)].numContacts(), 1);
      EXPECT_EQ(step_results.numContacts(), 1);
    }

    // Test addContact that requires automatic resizing to accommodate more substeps
    {
      int substep_number = 3;     // This is beyond the current 2 substeps
      int required_substeps = 5;  // Need to resize to 5 substeps

      Eigen::VectorXd substep_start(2);
      Eigen::VectorXd substep_end(2);
      substep_start << 2.0, 2.5;
      substep_end << 2.5, 3.0;

      tesseract_collision::ContactResultMap contacts;
      auto key = tesseract_common::makeOrderedLinkPair("anotherLink1", "anotherLink2");

      tesseract_collision::ContactResult cr;
      cr.distance = -0.03;
      cr.link_names[0] = "anotherLink1";
      cr.link_names[1] = "anotherLink2";

      contacts.addContactResult(key, cr);

      // Verify initial state
      EXPECT_EQ(step_results.total_substeps, initial_substeps);
      EXPECT_EQ(step_results.substeps.size(), initial_substeps);

      // This should automatically resize the step to accommodate the required substeps
      step_results.addContact(
          step_number, substep_number, required_substeps, start_state, end_state, substep_start, substep_end, contacts);

      // Verify the step was resized to accommodate the new requirements
      EXPECT_EQ(step_results.total_substeps, required_substeps);
      EXPECT_EQ(step_results.substeps.size(), required_substeps);

      // Verify the contact was added correctly
      EXPECT_EQ(step_results.substeps[static_cast<std::size_t>(substep_number)].substep, substep_number);
      EXPECT_TRUE(step_results.substeps[static_cast<std::size_t>(substep_number)].state0.isApprox(substep_start));
      EXPECT_TRUE(step_results.substeps[static_cast<std::size_t>(substep_number)].state1.isApprox(substep_end));
      EXPECT_EQ(step_results.substeps[static_cast<std::size_t>(substep_number)].numContacts(), 1);
      EXPECT_EQ(step_results.numContacts(), 2);  // Now should have 2 total contacts
    }

    // Test that calling addContact with the same number of substeps doesn't cause unnecessary resize
    {
      int substep_number = 4;  // Within the current 5 substeps
      int same_substeps = 5;   // Same number of substeps

      Eigen::VectorXd substep_start(2);
      Eigen::VectorXd substep_end(2);
      substep_start << 3.0, 3.5;
      substep_end << 3.5, 4.0;

      tesseract_collision::ContactResultMap contacts;
      auto key = tesseract_common::makeOrderedLinkPair("thirdLink1", "thirdLink2");

      tesseract_collision::ContactResult cr;
      cr.distance = -0.01;
      cr.link_names[0] = "thirdLink1";
      cr.link_names[1] = "thirdLink2";

      contacts.addContactResult(key, cr);

      // This should not resize since substeps already matches num_substeps
      step_results.addContact(
          step_number, substep_number, same_substeps, start_state, end_state, substep_start, substep_end, contacts);

      // Verify no resize occurred
      EXPECT_EQ(step_results.total_substeps, 5);
      EXPECT_EQ(step_results.substeps.size(), 5);

      // Verify the contact was added correctly
      EXPECT_EQ(step_results.substeps[static_cast<std::size_t>(substep_number)].substep, substep_number);
      EXPECT_TRUE(step_results.substeps[static_cast<std::size_t>(substep_number)].state0.isApprox(substep_start));
      EXPECT_TRUE(step_results.substeps[static_cast<std::size_t>(substep_number)].state1.isApprox(substep_end));
      EXPECT_EQ(step_results.substeps[static_cast<std::size_t>(substep_number)].numContacts(), 1);
      EXPECT_EQ(step_results.numContacts(), 3);  // Now should have 3 total contacts
    }

    // Test addContact with out of range substep number
    {
      int invalid_substep_number = 10;  // Out of range
      int same_substeps = 5;

      Eigen::VectorXd state(1);
      state << 1.0;

      tesseract_collision::ContactResultMap contacts;

      EXPECT_THROW(step_results.addContact(
                       step_number, invalid_substep_number, same_substeps, state, state, state, state, contacts),
                   std::out_of_range);
    }

    // Test addContact with substep_number equal to total_substeps (boundary condition - should throw)
    // This catches the off-by-one error where substep index N with N total substeps would try to access
    // index [N] in a vector of size N (which only has valid indices 0 to N-1)
    {
      int boundary_substep_number = 5;  // Equal to total_substeps (5), should be out of range
      int same_substeps = 5;

      Eigen::VectorXd state(1);
      state << 1.0;

      tesseract_collision::ContactResultMap contacts;

      EXPECT_THROW(step_results.addContact(
                       step_number, boundary_substep_number, same_substeps, state, state, state, state, contacts),
                   std::out_of_range);
    }

    // Test addContact with substep_number equal to num_substeps when resizing (boundary condition)
    // This tests the case where we pass substep index 1 with 1 total substep - should throw
    {
      tesseract_collision::ContactTrajectoryStepResults fresh_step_results;
      int fresh_step_number = 0;
      int boundary_substep = 1;  // Index 1
      int total = 1;             // Only 1 substep total (valid indices: 0 only)

      Eigen::VectorXd state(1);
      state << 1.0;

      tesseract_collision::ContactResultMap contacts;

      // This should throw because after resizing to 1, valid indices are only [0], not [1]
      EXPECT_THROW(fresh_step_results.addContact(
                       fresh_step_number, boundary_substep, total, state, state, state, state, contacts),
                   std::out_of_range);
    }

    // Test addContact with valid boundary (substep N-1 with N total substeps)
    {
      tesseract_collision::ContactTrajectoryStepResults valid_step_results;
      int valid_step_number = 0;
      int valid_substep = 0;  // Index 0
      int total = 1;          // 1 substep total (valid indices: 0 only)

      Eigen::VectorXd state(2);
      state << 1.0, 2.0;

      tesseract_collision::ContactResultMap contacts;
      auto key = tesseract_common::makeOrderedLinkPair("validLink1", "validLink2");
      tesseract_collision::ContactResult cr;
      cr.distance = -0.05;
      cr.link_names[0] = "validLink1";
      cr.link_names[1] = "validLink2";
      contacts.addContactResult(key, cr);

      // This should NOT throw - index 0 with 1 total is valid
      EXPECT_NO_THROW(
          valid_step_results.addContact(valid_step_number, valid_substep, total, state, state, state, state, contacts));

      EXPECT_EQ(valid_step_results.total_substeps, 1);
      EXPECT_EQ(valid_step_results.substeps.size(), 1);
      EXPECT_EQ(valid_step_results.substeps[0].numContacts(), 1);
    }

    // Test addContact with substep index 1 and 2 total substeps (valid case after fix)
    {
      tesseract_collision::ContactTrajectoryStepResults two_step_results;
      int step_num = 0;
      int substep_idx = 1;  // Index 1
      int total = 2;        // 2 substeps total (valid indices: 0 and 1)

      Eigen::VectorXd state(2);
      state << 1.0, 2.0;

      tesseract_collision::ContactResultMap contacts;
      auto key = tesseract_common::makeOrderedLinkPair("twoStepLink1", "twoStepLink2");
      tesseract_collision::ContactResult cr;
      cr.distance = -0.03;
      cr.link_names[0] = "twoStepLink1";
      cr.link_names[1] = "twoStepLink2";
      contacts.addContactResult(key, cr);

      // This should NOT throw - index 1 with 2 total is valid
      EXPECT_NO_THROW(two_step_results.addContact(step_num, substep_idx, total, state, state, state, state, contacts));

      EXPECT_EQ(two_step_results.total_substeps, 2);
      EXPECT_EQ(two_step_results.substeps.size(), 2);
      EXPECT_EQ(two_step_results.substeps[1].numContacts(), 1);
    }

    // Test addContact with negative substep number
    {
      int invalid_substep_number = -1;
      int same_substeps = 5;

      Eigen::VectorXd state(1);
      state << 1.0;

      tesseract_collision::ContactResultMap contacts;

      EXPECT_THROW(step_results.addContact(
                       step_number, invalid_substep_number, same_substeps, state, state, state, state, contacts),
                   std::runtime_error);
    }
  }

  // Test ContactTrajectorySubstepResults addContact method directly
  {
    tesseract_collision::ContactTrajectorySubstepResults substep_results;

    int substep_number = 5;
    Eigen::VectorXd start_state(3);
    Eigen::VectorXd end_state(3);
    start_state << 1.0, 2.0, 3.0;
    end_state << 4.0, 5.0, 6.0;

    // Create contacts to add
    tesseract_collision::ContactResultMap contacts;
    auto key1 = tesseract_common::makeOrderedLinkPair("substepLink1", "substepLink2");
    auto key2 = tesseract_common::makeOrderedLinkPair("substepLink3", "substepLink4");

    tesseract_collision::ContactResult cr1;
    cr1.distance = -0.12;
    cr1.link_names[0] = "substepLink1";
    cr1.link_names[1] = "substepLink2";

    tesseract_collision::ContactResult cr2;
    cr2.distance = -0.08;
    cr2.link_names[0] = "substepLink3";
    cr2.link_names[1] = "substepLink4";

    contacts.addContactResult(key1, cr1);
    contacts.addContactResult(key2, cr2);

    // Add the contacts
    substep_results.addContact(substep_number, start_state, end_state, contacts);

    // Verify everything was set correctly
    EXPECT_EQ(substep_results.substep, substep_number);
    EXPECT_TRUE(substep_results.state0.isApprox(start_state));
    EXPECT_TRUE(substep_results.state1.isApprox(end_state));
    EXPECT_EQ(substep_results.numContacts(), 2);

    // Verify the contacts were copied correctly
    EXPECT_EQ(substep_results.contacts.count(), 2);

    auto it1 = substep_results.contacts.find(key1);
    auto it2 = substep_results.contacts.find(key2);
    EXPECT_NE(it1, substep_results.contacts.end());
    EXPECT_NE(it2, substep_results.contacts.end());

    if (it1 != substep_results.contacts.end())
    {
      EXPECT_EQ(it1->second.size(), 1);
      EXPECT_EQ(it1->second[0].distance, -0.12);
      EXPECT_EQ(it1->second[0].link_names[0], "substepLink1");
      EXPECT_EQ(it1->second[0].link_names[1], "substepLink2");
    }

    if (it2 != substep_results.contacts.end())
    {
      EXPECT_EQ(it2->second.size(), 1);
      EXPECT_EQ(it2->second[0].distance, -0.08);
      EXPECT_EQ(it2->second[0].link_names[0], "substepLink3");
      EXPECT_EQ(it2->second[0].link_names[1], "substepLink4");
    }

    // Test that calling addContact again replaces the previous data
    int new_substep_number = 8;
    Eigen::VectorXd new_start_state(2);
    Eigen::VectorXd new_end_state(2);
    new_start_state << 10.0, 20.0;
    new_end_state << 30.0, 40.0;

    tesseract_collision::ContactResultMap new_contacts;
    auto new_key = tesseract_common::makeOrderedLinkPair("newLink1", "newLink2");

    tesseract_collision::ContactResult new_cr;
    new_cr.distance = -0.99;
    new_cr.link_names[0] = "newLink1";
    new_cr.link_names[1] = "newLink2";

    new_contacts.addContactResult(new_key, new_cr);

    substep_results.addContact(new_substep_number, new_start_state, new_end_state, new_contacts);

    // Verify the data was replaced
    EXPECT_EQ(substep_results.substep, new_substep_number);
    EXPECT_TRUE(substep_results.state0.isApprox(new_start_state));
    EXPECT_TRUE(substep_results.state1.isApprox(new_end_state));
    EXPECT_EQ(substep_results.numContacts(), 1);

    // Verify the old contacts are gone and new contact is present
    EXPECT_EQ(substep_results.contacts.count(), 1);
    auto new_it = substep_results.contacts.find(new_key);
    EXPECT_NE(new_it, substep_results.contacts.end());
    if (new_it != substep_results.contacts.end())
    {
      EXPECT_EQ(new_it->second[0].distance, -0.99);
      EXPECT_EQ(new_it->second[0].link_names[0], "newLink1");
      EXPECT_EQ(new_it->second[0].link_names[1], "newLink2");
    }
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
