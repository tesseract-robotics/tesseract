/**
 * @file factory_utils_unit.cpp
 * @brief Unit tests for the shared kinematics plugin-factory YAML helpers.
 *
 * These helpers are exercised indirectly by the ROP/REP/RTP factory tests, but those
 * route through a full plugin load (URDF fixture, YAML schema, inner solver plugins),
 * so a failure there does not localise to the parsing logic. These tests call the
 * helpers directly against a two-link scene graph.
 */

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <memory>
#include <yaml-cpp/yaml.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/kinematics/factory_utils.h>
#include <tesseract/scene_graph/graph.h>
#include <tesseract/scene_graph/joint.h>
#include <tesseract/scene_graph/link.h>

namespace
{
/** @brief Minimal scene graph: base -> j1 -> l1, with j1 limited to [-1, 1]. */
std::unique_ptr<tesseract::scene_graph::SceneGraph> buildSceneGraph()
{
  auto sg = std::make_unique<tesseract::scene_graph::SceneGraph>();
  sg->setName("factory_utils_test");
  sg->addLink(tesseract::scene_graph::Link("base"));
  sg->setRoot("base");
  sg->addLink(tesseract::scene_graph::Link("l1"));

  tesseract::scene_graph::Joint j("j1");
  j.parent_link_id = "base";
  j.child_link_id = "l1";
  j.type = tesseract::scene_graph::JointType::REVOLUTE;
  j.axis = Eigen::Vector3d::UnitZ();
  j.limits = std::make_shared<tesseract::scene_graph::JointLimits>();
  j.limits->lower = -1.0;
  j.limits->upper = 1.0;
  j.limits->velocity = 1.0;
  j.limits->acceleration = 1.0;
  j.limits->jerk = 1.0;
  sg->addJoint(j);

  return sg;
}

/** @brief Extends buildSceneGraph() with l1 -> j2 -> l2, where j2 is limited to [-2, 2]. */
void addSecondJoint(tesseract::scene_graph::SceneGraph& sg)
{
  sg.addLink(tesseract::scene_graph::Link("l2"));

  tesseract::scene_graph::Joint j("j2");
  j.parent_link_id = "l1";
  j.child_link_id = "l2";
  j.type = tesseract::scene_graph::JointType::REVOLUTE;
  j.axis = Eigen::Vector3d::UnitZ();
  j.limits = std::make_shared<tesseract::scene_graph::JointLimits>();
  j.limits->lower = -2.0;
  j.limits->upper = 2.0;
  j.limits->velocity = 1.0;
  j.limits->acceleration = 1.0;
  j.limits->jerk = 1.0;
  sg.addJoint(j);
}

/** @brief Parse @p yaml against the fixture graph and reorder it onto @p joint_ids. */
tesseract::kinematics::SampleGridConfig parse(const tesseract::scene_graph::SceneGraph& sg,
                                              const std::string& yaml,
                                              const std::vector<tesseract::common::JointId>& joint_ids = { "j1" })
{
  const auto map =
      tesseract::kinematics::parseSampleResolutionMap(YAML::Load(yaml), sg, "TestFactory", "sample_resolution");
  return tesseract::kinematics::toSampleGridConfig(map, joint_ids, "TestFactory", "sample_resolution");
}
}  // namespace

TEST(KinematicsFactoryUtils, ParsePluginInfoReadsClassAndConfig)  // NOLINT
{
  const YAML::Node node = YAML::Load(R"(
class: SomeFactory
config:
  base_link: base
)");
  const auto info = tesseract::kinematics::parsePluginInfo(node, "TestFactory", "positioner");
  EXPECT_EQ(info.class_name, "SomeFactory");
  ASSERT_TRUE(info.config);
  EXPECT_EQ(info.config["base_link"].as<std::string>(), "base");
}

TEST(KinematicsFactoryUtils, ParsePluginInfoConfigIsOptional)  // NOLINT
{
  const YAML::Node node = YAML::Load("class: SomeFactory");
  const auto info = tesseract::kinematics::parsePluginInfo(node, "TestFactory", "positioner");
  EXPECT_EQ(info.class_name, "SomeFactory");
}

TEST(KinematicsFactoryUtils, ParsePluginInfoMissingClassThrows)  // NOLINT
{
  const YAML::Node node = YAML::Load("config: { base_link: base }");
  EXPECT_THROW(tesseract::kinematics::parsePluginInfo(node, "TestFactory", "positioner"),  // NOLINT
               std::runtime_error);
}

TEST(KinematicsFactoryUtils, SampleResolutionDefaultsToJointLimits)  // NOLINT
{
  const auto sg = buildSceneGraph();
  const auto grid = parse(*sg, "- { name: j1, value: 0.1 }");

  ASSERT_EQ(grid.resolution.size(), 1);
  ASSERT_EQ(grid.range.rows(), 1);
  EXPECT_DOUBLE_EQ(grid.resolution(0), 0.1);
  EXPECT_DOUBLE_EQ(grid.range(0, 0), -1.0);
  EXPECT_DOUBLE_EQ(grid.range(0, 1), 1.0);
}

TEST(KinematicsFactoryUtils, SampleResolutionHonoursMinMaxOverrides)  // NOLINT
{
  const auto sg = buildSceneGraph();
  const auto grid = parse(*sg, "- { name: j1, value: 0.1, min: -0.5, max: 0.5 }");

  EXPECT_DOUBLE_EQ(grid.range(0, 0), -0.5);
  EXPECT_DOUBLE_EQ(grid.range(0, 1), 0.5);
}

TEST(KinematicsFactoryUtils, SampleResolutionRejectsOverridesOutsideJointLimits)  // NOLINT
{
  const auto sg = buildSceneGraph();
  // min below the joint's lower limit.
  EXPECT_THROW(parse(*sg, "- { name: j1, value: 0.1, min: -2.0 }"), std::runtime_error);  // NOLINT
  // max above the joint's upper limit.
  EXPECT_THROW(parse(*sg, "- { name: j1, value: 0.1, max: 2.0 }"), std::runtime_error);  // NOLINT
  // Inverted range that is still inside the joint's limits.
  EXPECT_THROW(parse(*sg, "- { name: j1, value: 0.1, min: 0.5, max: -0.5 }"), std::runtime_error);  // NOLINT
}

TEST(KinematicsFactoryUtils, SampleResolutionRequiresNameAndValue)  // NOLINT
{
  const auto sg = buildSceneGraph();
  EXPECT_THROW(parse(*sg, "- { value: 0.1 }"), std::runtime_error);  // NOLINT
  EXPECT_THROW(parse(*sg, "- { name: j1 }"), std::runtime_error);    // NOLINT
}

TEST(KinematicsFactoryUtils, SampleResolutionRejectsUnknownJoint)  // NOLINT
{
  const auto sg = buildSceneGraph();
  EXPECT_THROW(parse(*sg, "- { name: no_such_joint, value: 0.1 }"), std::runtime_error);  // NOLINT
}

TEST(KinematicsFactoryUtils, SampleResolutionRejectsJointWithoutLimits)  // NOLINT
{
  const auto sg = buildSceneGraph();
  // A joint's limits member is a default-null shared_ptr; reading lower/upper without a
  // guard would dereference null rather than report a configuration error.
  std::const_pointer_cast<tesseract::scene_graph::Joint>(sg->getJoint("j1"))->limits = nullptr;
  EXPECT_THROW(parse(*sg, "- { name: j1, value: 0.1 }"), std::runtime_error);  // NOLINT
}

TEST(KinematicsFactoryUtils, SampleResolutionAcceptsEmptySequence)  // NOLINT
{
  const auto sg = buildSceneGraph();
  const auto grid = parse(*sg, "[]", {});
  EXPECT_EQ(grid.resolution.size(), 0);
  EXPECT_EQ(grid.range.rows(), 0);
}

TEST(KinematicsFactoryUtils, SampleResolutionRejectsJointCountMismatch)  // NOLINT
{
  const auto sg = buildSceneGraph();
  // One entry, but the chain reports two joints.
  EXPECT_THROW(parse(*sg, "- { name: j1, value: 0.1 }", { "j1", "j2" }), std::runtime_error);  // NOLINT
  // Right number of entries, but none of them names the joint the chain asked for.
  EXPECT_THROW(parse(*sg, "- { name: j1, value: 0.1 }", { "other" }), std::runtime_error);  // NOLINT
}

TEST(KinematicsFactoryUtils, SampleResolutionOrdersOutputByJointNames)  // NOLINT
{
  auto sg = buildSceneGraph();
  addSecondJoint(*sg);

  // Sequence order is j2 then j1; the output must follow the requested joint order instead.
  const auto grid = parse(*sg,
                          "- { name: j2, value: 0.2 }\n"
                          "- { name: j1, value: 0.1 }",
                          { "j1", "j2" });

  ASSERT_EQ(grid.resolution.size(), 2);
  EXPECT_DOUBLE_EQ(grid.resolution(0), 0.1);
  EXPECT_DOUBLE_EQ(grid.resolution(1), 0.2);
  EXPECT_DOUBLE_EQ(grid.range(0, 0), -1.0);
  EXPECT_DOUBLE_EQ(grid.range(1, 0), -2.0);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
