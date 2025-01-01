#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <tinyxml2.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/utils.h>
#include <tesseract_common/resource_locator.h>
#include <tesseract_common/collision_margin_data.h>
#include <tesseract_common/yaml_utils.h>
#include <tesseract_common/yaml_extenstions.h>
#include <tesseract_scene_graph/graph.h>
#include <tesseract_srdf/collision_margins.h>
#include <tesseract_srdf/configs.h>
#include <tesseract_srdf/disabled_collisions.h>
#include <tesseract_srdf/group_states.h>
#include <tesseract_srdf/group_tool_center_points.h>
#include <tesseract_srdf/groups.h>
#include <tesseract_srdf/srdf_model.h>
#include <tesseract_srdf/utils.h>
#include <tesseract_scene_graph/graph.h>
#include <tesseract_scene_graph/link.h>
#include <tesseract_scene_graph/joint.h>

enum class ABBConfig
{
  ROBOT_ONLY,
  ROBOT_ON_RAIL,
  ROBOT_WITH_POSITIONER
};

tesseract_scene_graph::SceneGraph::Ptr getABBSceneGraph(ABBConfig config = ABBConfig::ROBOT_ONLY)
{
  using namespace tesseract_scene_graph;
  using namespace tesseract_srdf;

  auto g = std::make_shared<SceneGraph>();

  g->setName("abb_irb2400");

  Link base_link("base_link");
  Link link_1("link_1");
  Link link_2("link_2");
  Link link_3("link_3");
  Link link_4("link_4");
  Link link_5("link_5");
  Link link_6("link_6");
  Link tool0("tool0");

  EXPECT_TRUE(g->addLink(base_link));
  EXPECT_TRUE(g->addLink(link_1));
  EXPECT_TRUE(g->addLink(link_2));
  EXPECT_TRUE(g->addLink(link_3));
  EXPECT_TRUE(g->addLink(link_4));
  EXPECT_TRUE(g->addLink(link_5));
  EXPECT_TRUE(g->addLink(link_6));
  EXPECT_TRUE(g->addLink(tool0));

  if (config == ABBConfig::ROBOT_ON_RAIL)
  {
    EXPECT_TRUE(g->addLink(Link("world")));
    EXPECT_TRUE(g->addLink(Link("axis_1")));
    EXPECT_TRUE(g->addLink(Link("axis_2")));

    Joint joint_a("joint_axis_1");
    joint_a.axis = Eigen::Vector3d(0, 1, 0);
    joint_a.parent_link_name = "world";
    joint_a.child_link_name = "axis_1";
    joint_a.type = JointType::PRISMATIC;
    joint_a.limits = std::make_shared<JointLimits>(-10, 10, 0, 5, 10, 20);
    EXPECT_TRUE(g->addJoint(joint_a));

    Joint joint_b("joint_axis_2");
    joint_b.axis = Eigen::Vector3d(1, 0, 0);
    joint_b.parent_link_name = "axis_1";
    joint_b.child_link_name = "axis_2";
    joint_b.type = JointType::PRISMATIC;
    joint_b.limits = std::make_shared<JointLimits>(-10, 10, 0, 5, 10, 20);
    EXPECT_TRUE(g->addJoint(joint_b));

    Joint joint_c("joint_base_link");
    joint_c.axis = Eigen::Vector3d(0, 1, 0);
    joint_c.parent_link_name = "axis_1";
    joint_c.child_link_name = "base_link";
    joint_c.type = JointType::FIXED;
    EXPECT_TRUE(g->addJoint(joint_c));
  }
  else if (config == ABBConfig::ROBOT_WITH_POSITIONER)
  {
    EXPECT_TRUE(g->addLink(Link("world")));
    EXPECT_TRUE(g->addLink(Link("axis_1")));
    EXPECT_TRUE(g->addLink(Link("axis_2")));

    Joint joint_a("joint_axis_1");
    joint_a.parent_to_joint_origin_transform.translation() = Eigen::Vector3d(1, 0, 0);
    joint_a.axis = Eigen::Vector3d(0, 1, 0);
    joint_a.parent_link_name = "world";
    joint_a.child_link_name = "axis_1";
    joint_a.type = JointType::PRISMATIC;
    joint_a.limits = std::make_shared<JointLimits>(-10, 10, 0, 5, 10, 20);
    EXPECT_TRUE(g->addJoint(joint_a));

    Joint joint_b("joint_axis_2");
    joint_b.parent_to_joint_origin_transform.translation() = Eigen::Vector3d(1, 0, 0);
    joint_b.axis = Eigen::Vector3d(1, 0, 0);
    joint_b.parent_link_name = "axis_1";
    joint_b.child_link_name = "axis_2";
    joint_b.type = JointType::PRISMATIC;
    joint_b.limits = std::make_shared<JointLimits>(-10, 10, 0, 5, 10, 20);
    EXPECT_TRUE(g->addJoint(joint_b));

    Joint joint_c("joint_base_link");
    joint_c.parent_link_name = "world";
    joint_c.child_link_name = "base_link";
    joint_c.type = JointType::FIXED;
    EXPECT_TRUE(g->addJoint(joint_c));
  }

  Joint joint_1("joint_1");
  joint_1.parent_link_name = "base_link";
  joint_1.child_link_name = "link_1";
  joint_1.type = JointType::FIXED;
  EXPECT_TRUE(g->addJoint(joint_1));

  Joint joint_2("joint_2");
  joint_2.parent_link_name = "link_1";
  joint_2.child_link_name = "link_2";
  joint_2.type = JointType::REVOLUTE;
  joint_2.limits = std::make_shared<JointLimits>(-7, 7, 0, 5, 10, 20);
  EXPECT_TRUE(g->addJoint(joint_2));

  Joint joint_3("joint_3");
  joint_3.parent_to_joint_origin_transform.translation()(0) = 1.25;
  joint_3.parent_link_name = "link_2";
  joint_3.child_link_name = "link_3";
  joint_3.type = JointType::REVOLUTE;
  joint_3.limits = std::make_shared<JointLimits>(-7, 7, 0, 5, 10, 20);
  EXPECT_TRUE(g->addJoint(joint_3));

  Joint joint_4("joint_4");
  joint_4.parent_to_joint_origin_transform.translation()(0) = 1.25;
  joint_4.parent_link_name = "link_3";
  joint_4.child_link_name = "link_4";
  joint_4.type = JointType::REVOLUTE;
  joint_4.limits = std::make_shared<JointLimits>(-7, 7, 0, 5, 10, 20);
  EXPECT_TRUE(g->addJoint(joint_4));

  Joint joint_5("joint_5");
  joint_5.parent_to_joint_origin_transform.translation()(1) = 1.25;
  joint_5.parent_link_name = "link_4";
  joint_5.child_link_name = "link_5";
  joint_5.type = JointType::REVOLUTE;
  joint_5.limits = std::make_shared<JointLimits>(-7, 7, 0, 5, 10, 20);
  EXPECT_TRUE(g->addJoint(joint_5));

  Joint joint_6("joint_6");
  joint_6.parent_to_joint_origin_transform.translation()(1) = 1.25;
  joint_6.parent_link_name = "link_5";
  joint_6.child_link_name = "link_6";
  joint_6.type = JointType::REVOLUTE;
  joint_6.limits = std::make_shared<JointLimits>(-7, 7, 0, 5, 10, 20);
  EXPECT_TRUE(g->addJoint(joint_6));

  Joint joint_tool0("joint_tool0");
  joint_tool0.parent_link_name = "link_6";
  joint_tool0.child_link_name = "tool0";
  joint_tool0.type = JointType::FIXED;
  EXPECT_TRUE(g->addJoint(joint_tool0));

  return g;
}

tesseract_scene_graph::SceneGraph buildTestSceneGraph()
{
  using namespace tesseract_scene_graph;
  SceneGraph g;

  Link base_link("base_link");
  Link link_1("link_1");
  Link link_2("link_2");
  Link link_3("link_3");
  Link link_4("link_4");
  Link link_5("link_5");

  EXPECT_TRUE(g.addLink(base_link));
  EXPECT_TRUE(g.addLink(link_1));
  EXPECT_TRUE(g.addLink(link_2));
  EXPECT_TRUE(g.addLink(link_3));
  EXPECT_TRUE(g.addLink(link_4));
  EXPECT_TRUE(g.addLink(link_5));

  Joint base_joint("base_joint");
  base_joint.parent_link_name = "base_link";
  base_joint.child_link_name = "link_1";
  base_joint.type = JointType::FIXED;
  EXPECT_TRUE(g.addJoint(base_joint));

  Joint joint_1("joint_1");
  joint_1.parent_link_name = "link_1";
  joint_1.child_link_name = "link_2";
  joint_1.type = JointType::REVOLUTE;
  joint_1.limits = std::make_shared<JointLimits>(-7, 7, 0, 5, 10, 20);
  EXPECT_TRUE(g.addJoint(joint_1));

  Joint joint_2("joint_2");
  joint_2.parent_to_joint_origin_transform.translation()(0) = 1.25;
  joint_2.parent_link_name = "link_2";
  joint_2.child_link_name = "link_3";
  joint_2.type = JointType::REVOLUTE;
  joint_2.limits = std::make_shared<JointLimits>(-7, 7, 0, 5, 10, 20);
  EXPECT_TRUE(g.addJoint(joint_2));

  Joint joint_3("joint_3");
  joint_3.parent_to_joint_origin_transform.translation()(0) = 1.25;
  joint_3.parent_link_name = "link_3";
  joint_3.child_link_name = "link_4";
  joint_3.type = JointType::REVOLUTE;
  joint_3.limits = std::make_shared<JointLimits>(-7, 7, 0, 5, 10, 20);
  EXPECT_TRUE(g.addJoint(joint_3));

  Joint joint_4("joint_4");
  joint_4.parent_to_joint_origin_transform.translation()(1) = 1.25;
  joint_4.parent_link_name = "link_2";
  joint_4.child_link_name = "link_5";
  joint_4.type = JointType::REVOLUTE;
  joint_4.limits = std::make_shared<JointLimits>(-7, 7, 0, 5, 10, 20);
  EXPECT_TRUE(g.addJoint(joint_4));

  return g;
}

TEST(TesseractSRDFUnit, LoadSRDFFileUnit)  // NOLINT
{
  using namespace tesseract_scene_graph;
  using namespace tesseract_srdf;
  using namespace tesseract_common;

  GeneralResourceLocator locator;
  std::string srdf_file =
      locator.locateResource("package://tesseract_support/urdf/lbr_iiwa_14_r820.srdf")->getFilePath();

  SceneGraph g;

  g.setName("kuka_lbr_iiwa_14_r820");

  Link base_link("base_link");
  Link link_1("link_1");
  Link link_2("link_2");
  Link link_3("link_3");
  Link link_4("link_4");
  Link link_5("link_5");
  Link link_6("link_6");
  Link link_7("link_7");
  Link tool0("tool0");

  EXPECT_TRUE(g.addLink(base_link));
  EXPECT_TRUE(g.addLink(link_1));
  EXPECT_TRUE(g.addLink(link_2));
  EXPECT_TRUE(g.addLink(link_3));
  EXPECT_TRUE(g.addLink(link_4));
  EXPECT_TRUE(g.addLink(link_5));
  EXPECT_TRUE(g.addLink(link_6));
  EXPECT_TRUE(g.addLink(link_7));
  EXPECT_TRUE(g.addLink(tool0));

  Joint joint_1("joint_a1");
  joint_1.parent_link_name = "base_link";
  joint_1.child_link_name = "link_1";
  joint_1.type = JointType::FIXED;
  EXPECT_TRUE(g.addJoint(joint_1));

  Joint joint_2("joint_a2");
  joint_2.parent_link_name = "link_1";
  joint_2.child_link_name = "link_2";
  joint_2.type = JointType::REVOLUTE;
  joint_2.limits = std::make_shared<JointLimits>(-7, 7, 0, 5, 10, 20);
  EXPECT_TRUE(g.addJoint(joint_2));

  Joint joint_3("joint_a3");
  joint_3.parent_to_joint_origin_transform.translation()(0) = 1.25;
  joint_3.parent_link_name = "link_2";
  joint_3.child_link_name = "link_3";
  joint_3.type = JointType::REVOLUTE;
  joint_3.limits = std::make_shared<JointLimits>(-7, 7, 0, 5, 10, 20);
  EXPECT_TRUE(g.addJoint(joint_3));

  Joint joint_4("joint_a4");
  joint_4.parent_to_joint_origin_transform.translation()(0) = 1.25;
  joint_4.parent_link_name = "link_3";
  joint_4.child_link_name = "link_4";
  joint_4.type = JointType::REVOLUTE;
  joint_4.limits = std::make_shared<JointLimits>(-7, 7, 0, 5, 10, 20);
  EXPECT_TRUE(g.addJoint(joint_4));

  Joint joint_5("joint_a5");
  joint_5.parent_to_joint_origin_transform.translation()(1) = 1.25;
  joint_5.parent_link_name = "link_4";
  joint_5.child_link_name = "link_5";
  joint_5.type = JointType::REVOLUTE;
  joint_5.limits = std::make_shared<JointLimits>(-7, 7, 0, 5, 10, 20);
  EXPECT_TRUE(g.addJoint(joint_5));

  Joint joint_6("joint_a6");
  joint_6.parent_to_joint_origin_transform.translation()(1) = 1.25;
  joint_6.parent_link_name = "link_5";
  joint_6.child_link_name = "link_6";
  joint_6.type = JointType::REVOLUTE;
  joint_6.limits = std::make_shared<JointLimits>(-7, 7, 0, 5, 10, 20);
  EXPECT_TRUE(g.addJoint(joint_6));

  Joint joint_7("joint_a7");
  joint_7.parent_to_joint_origin_transform.translation()(1) = 1.25;
  joint_7.parent_link_name = "link_6";
  joint_7.child_link_name = "link_7";
  joint_7.type = JointType::REVOLUTE;
  joint_7.limits = std::make_shared<JointLimits>(-7, 7, 0, 5, 10, 20);
  EXPECT_TRUE(g.addJoint(joint_7));

  Joint joint_tool0("joint_tool0");
  joint_tool0.parent_link_name = "link_7";
  joint_tool0.child_link_name = "tool0";
  joint_tool0.type = JointType::FIXED;
  EXPECT_TRUE(g.addJoint(joint_tool0));

  SRDFModel srdf;
  srdf.initFile(g, srdf_file, locator);
  EXPECT_EQ(srdf.name, "kuka_lbr_iiwa_14_r820");
  EXPECT_EQ(srdf.version[0], 1);
  EXPECT_EQ(srdf.version[1], 0);
  EXPECT_EQ(srdf.version[2], 0);

  processSRDFAllowedCollisions(g, srdf);

  tesseract_common::AllowedCollisionMatrix::ConstPtr acm = g.getAllowedCollisionMatrix();

  // collision between link1 and link2 should be allowed
  EXPECT_TRUE(acm->isCollisionAllowed("link_1", "link_2"));

  // collision between link1 and link2 should be allowed
  EXPECT_FALSE(acm->isCollisionAllowed("base_link", "link_5"));

  g.removeAllowedCollision("link_1", "link_2");
  // now collision link1 and link2 is not allowed anymore
  EXPECT_FALSE(acm->isCollisionAllowed("link_1", "link_2"));

  g.clearAllowedCollisions();
  EXPECT_EQ(acm->getAllAllowedCollisions().size(), 0);
}

TEST(TesseractSRDFUnit, TesseractSRDFModelUnit)  // NOLINT
{
  using namespace tesseract_scene_graph;
  using namespace tesseract_srdf;
  using namespace tesseract_common;

  GeneralResourceLocator locator;
  SRDFModel srdf;

  // Set Name
  srdf.name = "test_srdf";
  EXPECT_TRUE(srdf.name == "test_srdf");

  // Add chain groups
  auto& chain_groups = srdf.kinematics_information.chain_groups;
  EXPECT_TRUE(chain_groups.empty());

  chain_groups["manipulator_chain"] = { std::make_pair("base_link", "link_5") };
  EXPECT_FALSE(srdf.kinematics_information.chain_groups.empty());

  // Add joint groups
  auto& joint_groups = srdf.kinematics_information.joint_groups;
  EXPECT_TRUE(joint_groups.empty());

  joint_groups["manipulator_joint"] = { "joint_1", "joint_2", "joint_3", "joint_4" };
  EXPECT_FALSE(srdf.kinematics_information.joint_groups.empty());

  // Add link groups
  auto& link_groups = srdf.kinematics_information.link_groups;
  EXPECT_TRUE(link_groups.empty());
  link_groups["manipulator_link"] = { "base_link", "link_1", "link_2", "link_3", "link_4", "link_5" };
  EXPECT_FALSE(srdf.kinematics_information.link_groups.empty());

  // Add group states
  auto& group_state = srdf.kinematics_information.group_states;
  EXPECT_TRUE(group_state.empty());
  GroupsJointState joint_state;
  joint_state["joint_1"] = 0;
  joint_state["joint_2"] = 0;
  joint_state["joint_3"] = 0;
  joint_state["joint_4"] = 0;
  group_state["manipulator_chain"]["All Zeros"] = joint_state;
  group_state["manipulator_joint"]["All Zeros"] = joint_state;
  group_state["manipulator_link"]["All Zeros"] = joint_state;
  EXPECT_EQ(srdf.kinematics_information.group_states.size(), 3);

  // Add Tool Center Points
  auto& group_tcps = srdf.kinematics_information.group_tcps;
  EXPECT_TRUE(group_tcps.empty());
  group_tcps["manipulator_chain"]["laser"] = Eigen::Isometry3d::Identity();
  group_tcps["manipulator_joint"]["laser"] = Eigen::Isometry3d::Identity();
  group_tcps["manipulator_link"]["laser"] = Eigen::Isometry3d::Identity();
  EXPECT_FALSE(srdf.kinematics_information.group_tcps.empty());

  // Add disabled collisions
  auto& acm = srdf.acm;
  EXPECT_TRUE(acm.getAllAllowedCollisions().empty());
  acm.addAllowedCollision("link_1", "link_3", "Adjacent");
  acm.addAllowedCollision("link_1", "link_2", "Adjacent");
  acm.addAllowedCollision("link_2", "link_3", "Adjacent");
  acm.addAllowedCollision("link_3", "link_4", "Adjacent");
  acm.addAllowedCollision("link_4", "link_5", "Adjacent");
  acm.addAllowedCollision("base_link", "link_1", "Adjacent");
  EXPECT_FALSE(acm.getAllAllowedCollisions().empty());
  srdf.saveToFile(tesseract_common::getTempPath() + "test.srdf");

  SceneGraph g = buildTestSceneGraph();

  SRDFModel srdf_reload;
  srdf_reload.initFile(g, tesseract_common::getTempPath() + "test.srdf", locator);
  EXPECT_TRUE(srdf_reload.name == "test_srdf");
  EXPECT_FALSE(srdf_reload.kinematics_information.chain_groups.empty());
  EXPECT_FALSE(srdf_reload.kinematics_information.joint_groups.empty());
  EXPECT_FALSE(srdf_reload.kinematics_information.link_groups.empty());
  EXPECT_EQ(srdf_reload.kinematics_information.group_states.size(), 3);
  EXPECT_TRUE(srdf_reload.kinematics_information.group_states["manipulator_chain"].find("All Zeros") !=
              srdf_reload.kinematics_information.group_states["manipulator_chain"].end());
  EXPECT_TRUE(srdf_reload.kinematics_information.group_states["manipulator_joint"].find("All Zeros") !=
              srdf_reload.kinematics_information.group_states["manipulator_joint"].end());
  EXPECT_TRUE(srdf_reload.kinematics_information.group_states["manipulator_link"].find("All Zeros") !=
              srdf_reload.kinematics_information.group_states["manipulator_link"].end());
  EXPECT_FALSE(srdf_reload.kinematics_information.group_tcps.empty());
  EXPECT_TRUE(srdf_reload.kinematics_information.group_tcps["manipulator_chain"].find("laser") !=
              srdf_reload.kinematics_information.group_tcps["manipulator_chain"].end());
  EXPECT_TRUE(srdf_reload.kinematics_information.group_tcps["manipulator_joint"].find("laser") !=
              srdf_reload.kinematics_information.group_tcps["manipulator_joint"].end());
  EXPECT_TRUE(srdf_reload.kinematics_information.group_tcps["manipulator_link"].find("laser") !=
              srdf_reload.kinematics_information.group_tcps["manipulator_link"].end());
  EXPECT_FALSE(srdf_reload.acm.getAllAllowedCollisions().empty());
  srdf_reload.saveToFile(tesseract_common::getTempPath() + "test_reload.srdf");
}

TEST(TesseractSRDFUnit, LoadSRDFFailureCasesUnit)  // NOLINT
{
  using namespace tesseract_scene_graph;
  using namespace tesseract_srdf;
  using namespace tesseract_common;

  GeneralResourceLocator locator;
  SceneGraph::Ptr g = getABBSceneGraph();

  {  // Success
    std::string xml_string =
        R"(<robot name="abb_irb2400" version="1.0.0">
             <group name="manipulator">
               <chain base_link="base_link" tip_link="tool0" />
             </group>
           </robot>)";

    SRDFModel srdf;
    srdf.initString(*g, xml_string, locator);
    EXPECT_EQ(srdf.name, "abb_irb2400");
    EXPECT_EQ(srdf.version[0], 1);
    EXPECT_EQ(srdf.version[1], 0);
    EXPECT_EQ(srdf.version[2], 0);
  }

  {  // Success version with no patch
    std::string xml_string =
        R"(<robot name="abb_irb2400" version="1.0">
             <group name="manipulator">
               <chain base_link="base_link" tip_link="tool0" />
             </group>
           </robot>)";

    SRDFModel srdf;
    srdf.initString(*g, xml_string, locator);
    EXPECT_EQ(srdf.name, "abb_irb2400");
    EXPECT_EQ(srdf.version[0], 1);
    EXPECT_EQ(srdf.version[1], 0);
    EXPECT_EQ(srdf.version[2], 0);
  }

  // Now test failures
  {  // missing name
    std::string xml_string =
        R"(<robot version="1.0.0">
             <group name="manipulator">
               <chain base_link="base_link" tip_link="tool0" />
             </group>
           </robot>)";

    SRDFModel srdf;
    EXPECT_ANY_THROW(srdf.initString(*g, xml_string, locator));  // NOLINT
  }
  {  // invalid version
    std::string xml_string =
        R"(<robot name="abb_irb2400" version="1 0 0">
             <group name="manipulator">
               <chain base_link="base_link" tip_link="tool0" />
             </group>
           </robot>)";

    SRDFModel srdf;
    EXPECT_ANY_THROW(srdf.initString(*g, xml_string, locator));  // NOLINT
  }
  {  // initXml missing name element
    std::string xml_string =
        R"(<robot version="1.0.0">
             <group name="manipulator">
               <chain base_link="base_link" tip_link="tool0" />
             </group>
           </missing_robot>)";

    SRDFModel srdf;
    EXPECT_ANY_THROW(srdf.initString(*g, xml_string, locator));  // NOLINT
  }
  {  // invalid xml
    std::string xml_string =
        R"(<robot name="abb_irb2400" version="1 0 0">
             <group name="manipulator">
               <chain base_link="base_link" tip_link="tool0" />
             </group>
           </robot_invalid>)";

    SRDFModel srdf;
    EXPECT_ANY_THROW(srdf.initString(*g, xml_string, locator));  // NOLINT
  }
  {  // initXml missing robot element
    std::string xml_string =
        R"(<missing_robot name="abb_irb2400" version="1.0.0">
             <group name="manipulator">
               <chain base_link="base_link" tip_link="tool0" />
             </group>
           </missing_robot>)";

    SRDFModel srdf;
    EXPECT_ANY_THROW(srdf.initString(*g, xml_string, locator));  // NOLINT
  }
  {  // initFile file path does not exist
    SRDFModel srdf;
    EXPECT_ANY_THROW(srdf.initFile(*g, "/tmp/file_does_not_exist.srdf", locator));  // NOLINT
  }
}

class TempResourceLocator : public tesseract_common::ResourceLocator
{
public:
  std::shared_ptr<tesseract_common::Resource> locateResource(const std::string& url) const override final
  {
    tesseract_common::fs::path mod_url(url);
    if (!mod_url.is_absolute())
    {
      mod_url = tesseract_common::fs::path(tesseract_common::getTempPath() + url);
    }

    return std::make_shared<tesseract_common::SimpleLocatedResource>(
        url, mod_url.string(), std::make_shared<TempResourceLocator>(*this));
  }
};

TEST(TesseractSRDFUnit, LoadSRDFSaveUnit)  // NOLINT
{
  using namespace tesseract_scene_graph;
  using namespace tesseract_srdf;
  using namespace tesseract_common;

  SceneGraph::Ptr g = getABBSceneGraph(ABBConfig::ROBOT_ON_RAIL);
  TempResourceLocator locator;

  std::string xml_string =
      R"(<robot name="abb_irb2400" version="1.0.0">
           <group name="manipulator">
             <chain base_link="base_link" tip_link="tool0" />
           </group>
           <group name="positioner">
             <chain base_link="world" tip_link="base_link" />
           </group>
           <group name="gantry">
             <chain base_link="world" tip_link="tool0" />
           </group>

           <group name="manipulator_joint">
             <joint name="joint_1"/>
             <joint name="joint_2"/>
             <joint name="joint_3"/>
             <joint name="joint_4"/>
             <joint name="joint_5"/>
             <joint name="joint_6"/>
             <joint name="joint_tool0"/>
           </group>

           <group_state name="all-zeros" group="manipulator">
             <joint name="joint_1" value="0"/>
             <joint name="joint_2" value="0"/>
             <joint name="joint_3" value="0"/>
             <joint name="joint_4" value="0"/>
             <joint name="joint_5" value="0"/>
             <joint name="joint_6" value="0"/>
           </group_state>

           <group_tcps group="gantry">
             <tcp name="laser" xyz="1 .1 1" rpy="0 1.57 0" />
             <tcp name="welder" xyz=".1 1 .2" wxyz="1 0 0 0" />
           </group_tcps>

           <disable_collisions link1="base_link" link2="link_1" reason="Adjacent" />
           <disable_collisions link1="base_link" link2="link_2" reason="Never" />
           <disable_collisions link1="base_link" link2="link_3" reason="Never" />

           <collision_margins default_margin="0.025">
             <pair_margin link1="link_6" link2="link_5" margin="0.01"/>
             <pair_margin link1="link_5" link2="link_4" margin="0.015"/>
           </collision_margins>
         </robot>)";

  std::string yaml_kin_plugins_string =
      R"(kinematic_plugins:
           fwd_kin_plugins:
             manipulator:
               default: KDLFwdKinChain
               plugins:
                 KDLFwdKinChain:
                   class: KDLFwdKinChainFactory
                   default: true
                   config:
                     base_link: base_link
                     tip_link: tool0
           inv_kin_plugins:
             manipulator:
               default: KDLInvKinChainLMA
               plugins:
                 KDLInvKinChainLMA:
                   class: KDLInvKinChainLMAFactory
                   default: true
                   config:
                     base_link: base_link
                     tip_link: tool0
                 KDLInvKinChainNR:
                   class: KDLInvKinChainNRFactory
                   config:
                     base_link: base_link
                     tip_link: tool0)";

  std::string yaml_cm_plugins_string =
      R"(contact_manager_plugins:
           search_paths:
             - /usr/local/lib
           search_libraries:
             - tesseract_collision_bullet_factories
             - tesseract_collision_fcl_factories
           discrete_plugins:
             default: BulletDiscreteBVHManager
             plugins:
               BulletDiscreteBVHManager:
                 class: BulletDiscreteBVHManagerFactory
                 default: true
               BulletDiscreteSimpleManager:
                 class: BulletDiscreteSimpleManagerFactory
               FCLDiscreteBVHManager:
                 class: FCLDiscreteBVHManagerFactory
           continuous_plugins:
             default: BulletCastBVHManager
             plugins:
               BulletCastBVHManager:
                 class: BulletCastBVHManagerFactory
                 default: true
               BulletCastSimpleManager:
                 class: BulletCastSimpleManagerFactory)";

  std::string yaml_calibration_string =
      R"(calibration:
           joints:
             joint_1:
               position:
                 x: 1
                 y: 2
                 z: 3
               orientation:
                 x: 0
                 y: 0
                 z: 0
                 w: 1
             joint_2:
               position:
                 x: 4
                 y: 5
                 z: 6
               orientation:
                 x: 0
                 y: 0
                 z: 0
                 w: 1)";

  SRDFModel srdf_save;
  srdf_save.initString(*g, xml_string, locator);

  YAML::Node kinematics_plugin_config = YAML::Load(yaml_kin_plugins_string);
  srdf_save.kinematics_information.kinematics_plugin_info =
      kinematics_plugin_config[KinematicsPluginInfo::CONFIG_KEY].as<KinematicsPluginInfo>();

  YAML::Node contact_managers_plugin_config = YAML::Load(yaml_cm_plugins_string);
  srdf_save.contact_managers_plugin_info =
      contact_managers_plugin_config[ContactManagersPluginInfo::CONFIG_KEY].as<ContactManagersPluginInfo>();

  YAML::Node calibration_config = YAML::Load(yaml_calibration_string);
  srdf_save.calibration_info = calibration_config[CalibrationInfo::CONFIG_KEY].as<CalibrationInfo>();

  std::string save_path = tesseract_common::getTempPath() + "unit_test_save_srdf.srdf";
  EXPECT_TRUE(srdf_save.saveToFile(save_path));

  SRDFModel srdf;
  srdf.initFile(*g, save_path, locator);
  EXPECT_EQ(srdf.name, "abb_irb2400");
  EXPECT_EQ(srdf.version[0], 1);
  EXPECT_EQ(srdf.version[1], 0);
  EXPECT_EQ(srdf.version[2], 0);

  EXPECT_FALSE(srdf.kinematics_information.kinematics_plugin_info.empty());
  EXPECT_FALSE(srdf.contact_managers_plugin_info.empty());
  EXPECT_FALSE(srdf.calibration_info.empty());

  processSRDFAllowedCollisions(*g, srdf);

  KinematicsInformation& kin_info = srdf.kinematics_information;

  // Check for tcp information
  EXPECT_EQ(kin_info.group_tcps.size(), 1);
  auto tcp_it = kin_info.group_tcps.find("gantry");
  EXPECT_TRUE(tcp_it != kin_info.group_tcps.end());
  EXPECT_EQ(tcp_it->second.size(), 2);
  EXPECT_TRUE(tcp_it->second.find("laser") != tcp_it->second.end());
  EXPECT_TRUE(tcp_it->second.find("welder") != tcp_it->second.end());

  // Check for chain group information
  EXPECT_EQ(kin_info.chain_groups.size(), 3);
  auto chain_gantry_it = kin_info.chain_groups.find("gantry");
  auto chain_manipulator_it = kin_info.chain_groups.find("manipulator");
  auto chain_positioner_it = kin_info.chain_groups.find("positioner");
  EXPECT_TRUE(chain_gantry_it != kin_info.chain_groups.end());
  EXPECT_TRUE(chain_manipulator_it != kin_info.chain_groups.end());
  EXPECT_TRUE(chain_positioner_it != kin_info.chain_groups.end());

  // Check for joint group information
  EXPECT_EQ(kin_info.joint_groups.size(), 1);
  auto joint_manipulator_it = kin_info.joint_groups.find("manipulator_joint");
  EXPECT_TRUE(joint_manipulator_it != kin_info.joint_groups.end());

  // Check for group states information
  EXPECT_EQ(kin_info.group_states.size(), 1);
  auto group_state_it = kin_info.group_states.find("manipulator");
  EXPECT_TRUE(group_state_it != kin_info.group_states.end());
  EXPECT_EQ(group_state_it->second.size(), 1);
  EXPECT_TRUE(group_state_it->second.find("all-zeros") != group_state_it->second.end());

  tesseract_common::AllowedCollisionMatrix::ConstPtr acm = g->getAllowedCollisionMatrix();
  EXPECT_TRUE(acm->isCollisionAllowed("base_link", "link_1"));
  EXPECT_TRUE(acm->isCollisionAllowed("base_link", "link_2"));
  EXPECT_TRUE(acm->isCollisionAllowed("base_link", "link_3"));

  EXPECT_TRUE(srdf.collision_margin_data != nullptr);
  EXPECT_NEAR(srdf.collision_margin_data->getDefaultCollisionMargin(), 0.025, 1e-6);
  EXPECT_NEAR(srdf.collision_margin_data->getMaxCollisionMargin(), 0.025, 1e-6);
  EXPECT_EQ(srdf.collision_margin_data->getPairCollisionMargins().size(), 2);
  EXPECT_NEAR(srdf.collision_margin_data->getPairCollisionMargin("link_5", "link_6"), 0.01, 1e-6);
  EXPECT_NEAR(srdf.collision_margin_data->getPairCollisionMargin("link_5", "link_4"), 0.015, 1e-6);

  // Calibration failure joint does not exist
  yaml_calibration_string =
      R"(calibration:
           joints:
             does_not_exist:
               position:
                 x: 1
                 y: 2
                 z: 3
               orientation:
                 x: 0
                 y: 0
                 z: 0
                 w: 1
             joint_2:
               position:
                 x: 4
                 y: 5
                 z: 6
               orientation:
                 x: 0
                 y: 0
                 z: 0
                 w: 1)";
  YAML::Node bad_calibration_config = YAML::Load(yaml_calibration_string);
  srdf_save.calibration_info = bad_calibration_config[CalibrationInfo::CONFIG_KEY].as<CalibrationInfo>();

  save_path = tesseract_common::getTempPath() + "unit_test_save_bad_srdf.srdf";
  EXPECT_TRUE(srdf_save.saveToFile(save_path));

  SRDFModel bad_srdf;
  EXPECT_ANY_THROW(bad_srdf.initFile(*g, save_path, locator));  // NOLINT
}

TEST(TesseractSRDFUnit, LoadSRDFAllowedCollisionMatrixUnit)  // NOLINT
{
  using namespace tesseract_scene_graph;
  using namespace tesseract_srdf;

  SceneGraph::Ptr g = getABBSceneGraph();
  TempResourceLocator locator;

  std::string xml_string =
      R"(<robot name="abb_irb2400">
           <disable_collisions link1="base_link" link2="link_1" reason="Adjacent" />
           <disable_collisions link1="base_link" link2="link_2" reason="Never" />
           <disable_collisions link1="base_link" link2="link_3" reason="Never" />
         </robot>)";
  tinyxml2::XMLDocument xml_doc;
  EXPECT_TRUE(xml_doc.Parse(xml_string.c_str()) == tinyxml2::XML_SUCCESS);

  tinyxml2::XMLElement* element = xml_doc.FirstChildElement("robot");
  EXPECT_TRUE(element != nullptr);

  tesseract_common::AllowedCollisionMatrix acm = parseDisabledCollisions(*g, element, std::array<int, 3>({ 1, 0, 0 }));
  EXPECT_TRUE(acm.isCollisionAllowed("base_link", "link_1"));
  EXPECT_TRUE(acm.isCollisionAllowed("base_link", "link_2"));
  EXPECT_TRUE(acm.isCollisionAllowed("base_link", "link_3"));

  // Now test failures
  auto is_failure = [g](const std::string& xml_string) {
    tinyxml2::XMLDocument xml_doc;
    EXPECT_TRUE(xml_doc.Parse(xml_string.c_str()) == tinyxml2::XML_SUCCESS);

    tinyxml2::XMLElement* element = xml_doc.FirstChildElement("robot");
    EXPECT_TRUE(element != nullptr);

    try
    {
      parseDisabledCollisions(*g, element, std::array<int, 3>({ 1, 0, 0 }));
    }
    catch (const std::exception& e)
    {
      tesseract_common::printNestedException(e);
      return true;
    }
    return false;
  };

  {  // missing link1
    std::string xml_string =
        R"(<robot name="abb_irb2400">
             <disable_collisions link2="link_1" reason="Adjacent" />
             <disable_collisions link1="base_link" link2="link_2" reason="Never" />
             <disable_collisions link1="base_link" link2="link_3" reason="Never" />
           </robot>)";
    EXPECT_TRUE(is_failure(xml_string));

    tesseract_srdf::SRDFModel srdf_model;
    EXPECT_ANY_THROW(srdf_model.initString(*g, xml_string, locator));  // NOLINT
  }
  {  // missing link2
    std::string xml_string =
        R"(<robot name="abb_irb2400">
             <disable_collisions link1="base_link" link2="link_1" reason="Adjacent" />
             <disable_collisions link1="base_link" reason="Never" />
             <disable_collisions link1="base_link" link2="link_3" reason="Never" />
           </robot>)";
    EXPECT_TRUE(is_failure(xml_string));

    tesseract_srdf::SRDFModel srdf_model;
    EXPECT_ANY_THROW(srdf_model.initString(*g, xml_string, locator));  // NOLINT
  }
  {  // missing reason but should not fail
    std::string xml_string =
        R"(<robot name="abb_irb2400">
             <disable_collisions link1="base_link" link2="link_1" reason="Adjacent" />
             <disable_collisions link1="base_link" link2="link_2" reason="Never" />
             <disable_collisions link1="base_link" link2="link_3" />
           </robot>)";
    EXPECT_FALSE(is_failure(xml_string));

    tesseract_srdf::SRDFModel srdf_model;
    EXPECT_NO_THROW(srdf_model.initString(*g, xml_string, locator));  // NOLINT
  }
  {  // invalid link1 but should not fail
    std::string xml_string =
        R"(<robot name="abb_irb2400">
             <disable_collisions link1="missing_link" link2="link_1" reason="Adjacent" />
             <disable_collisions link1="base_link" link2="link_2" reason="Never" />
             <disable_collisions link1="base_link" link2="link_3" reason="Never" />
           </robot>)";
    EXPECT_FALSE(is_failure(xml_string));

    tesseract_srdf::SRDFModel srdf_model;
    EXPECT_NO_THROW(srdf_model.initString(*g, xml_string, locator));  // NOLINT
  }
  {  // invalid link2 but should not fail
    std::string xml_string =
        R"(<robot name="abb_irb2400">
             <disable_collisions link1="base_link" link2="link_1" reason="Adjacent" />
             <disable_collisions link1="base_link" link2="missing_link" reason="Never" />
             <disable_collisions link1="base_link" link2="link_3" reason="Never" />
           </robot>)";
    EXPECT_FALSE(is_failure(xml_string));

    tesseract_srdf::SRDFModel srdf_model;
    EXPECT_NO_THROW(srdf_model.initString(*g, xml_string, locator));  // NOLINT
  }
  {  // The reason is numeric but still a valid string so should not fail
    std::string xml_string =
        R"(<robot name="abb_irb2400">
             <disable_collisions link1="base_link" link2="link_1" reason="Adjacent" />
             <disable_collisions link1="base_link" link2="missing_link" reason="Never" />
             <disable_collisions link1="base_link" link2="link_3" reason="2.335" />
           </robot>)";
    EXPECT_FALSE(is_failure(xml_string));

    tesseract_srdf::SRDFModel srdf_model;
    EXPECT_NO_THROW(srdf_model.initString(*g, xml_string, locator));  // NOLINT
  }
}

TEST(TesseractSRDFUnit, SRDFChainGroupUnit)  // NOLINT
{
  using namespace tesseract_scene_graph;
  using namespace tesseract_srdf;

  SceneGraph::Ptr g = getABBSceneGraph();
  TempResourceLocator locator;

  std::string str = R"(<robot name="abb_irb2400">
                         <group name="manipulator">
                           <chain base_link="base_link" tip_link="tool0" />
                         </group>
                       </robot>)";

  tinyxml2::XMLDocument xml_doc;
  EXPECT_TRUE(xml_doc.Parse(str.c_str()) == tinyxml2::XML_SUCCESS);

  tinyxml2::XMLElement* element = xml_doc.FirstChildElement("robot");
  EXPECT_TRUE(element != nullptr);

  auto [group_names, chain_groups, joint_groups, link_groups] =
      parseGroups(*g, element, std::array<int, 3>({ 1, 0, 0 }));

  EXPECT_EQ(group_names.size(), 1);
  EXPECT_EQ(*group_names.begin(), "manipulator");
  EXPECT_EQ(chain_groups.size(), 1);
  EXPECT_TRUE(joint_groups.empty());
  EXPECT_TRUE(link_groups.empty());
  EXPECT_EQ(chain_groups["manipulator"].size(), 1);
  EXPECT_EQ(chain_groups["manipulator"][0].first, "base_link");
  EXPECT_EQ(chain_groups["manipulator"][0].second, "tool0");

  // Now test failures
  auto is_failure = [g](const std::string& xml_string) {
    tinyxml2::XMLDocument xml_doc;
    EXPECT_TRUE(xml_doc.Parse(xml_string.c_str()) == tinyxml2::XML_SUCCESS);

    tinyxml2::XMLElement* element = xml_doc.FirstChildElement("robot");
    EXPECT_TRUE(element != nullptr);

    try
    {
      parseGroups(*g, element, std::array<int, 3>({ 1, 0, 0 }));
    }
    catch (const std::exception& e)
    {
      tesseract_common::printNestedException(e);
      return true;
    }
    return false;
  };

  {  // missing name
    std::string str = R"(<robot name="abb_irb2400">
                           <group>
                             <chain base_link="base_link" tip_link="tool0" />
                           </group>
                         </robot>)";
    EXPECT_TRUE(is_failure(str));

    tesseract_srdf::SRDFModel srdf_model;
    EXPECT_ANY_THROW(srdf_model.initString(*g, str, locator));  // NOLINT
  }
  {  // missing chains
    std::string str = R"(<robot name="abb_irb2400">
                           <group name="manipulator"/>
                         </robot>)";
    EXPECT_TRUE(is_failure(str));

    tesseract_srdf::SRDFModel srdf_model;
    EXPECT_ANY_THROW(srdf_model.initString(*g, str, locator));  // NOLINT
  }
  {  // missing chain base_link
    std::string str = R"(<robot name="abb_irb2400">
                           <group name="manipulator">
                             <chain tip_link="tool0" />
                           </group>
                         </robot>)";
    EXPECT_TRUE(is_failure(str));

    tesseract_srdf::SRDFModel srdf_model;
    EXPECT_ANY_THROW(srdf_model.initString(*g, str, locator));  // NOLINT
  }
  {  // missing chain tip_link
    std::string str = R"(<robot name="abb_irb2400">
                           <group name="manipulator">
                             <chain base_link="base_link" />
                           </group>
                         </robot>)";
    EXPECT_TRUE(is_failure(str));

    tesseract_srdf::SRDFModel srdf_model;
    EXPECT_ANY_THROW(srdf_model.initString(*g, str, locator));  // NOLINT
  }
  {  // invalid chain base_link
    std::string str = R"(<robot name="abb_irb2400">
                           <group name="manipulator">
                             <chain base_link="missing_link" tip_link="tool0" />
                           </group>
                         </robot>)";
    EXPECT_TRUE(is_failure(str));

    tesseract_srdf::SRDFModel srdf_model;
    EXPECT_ANY_THROW(srdf_model.initString(*g, str, locator));  // NOLINT
  }
  {  // invalid chain tip_link
    std::string str = R"(<robot name="abb_irb2400">
                           <group name="manipulator">
                             <chain base_link="base_link" tip_link="missing_link" />
                           </group>
                         </robot>)";
    EXPECT_TRUE(is_failure(str));

    tesseract_srdf::SRDFModel srdf_model;
    EXPECT_ANY_THROW(srdf_model.initString(*g, str, locator));  // NOLINT
  }
}

TEST(TesseractSRDFUnit, SRDFJointGroupUnit)  // NOLINT
{
  using namespace tesseract_scene_graph;
  using namespace tesseract_srdf;

  SceneGraph::Ptr g = getABBSceneGraph();
  TempResourceLocator locator;

  std::string str = R"(<robot name="abb_irb2400">
                         <group name="manipulator">
                           <joint name="joint_1"/>
                           <joint name="joint_2"/>
                           <joint name="joint_3"/>
                           <joint name="joint_4"/>
                           <joint name="joint_5"/>
                           <joint name="joint_6"/>
                           <joint name="joint_tool0"/>
                         </group>
                       </robot>)";

  tinyxml2::XMLDocument xml_doc;
  EXPECT_TRUE(xml_doc.Parse(str.c_str()) == tinyxml2::XML_SUCCESS);

  tinyxml2::XMLElement* element = xml_doc.FirstChildElement("robot");
  EXPECT_TRUE(element != nullptr);

  auto [group_names, chain_groups, joint_groups, link_groups] =
      parseGroups(*g, element, std::array<int, 3>({ 1, 0, 0 }));

  EXPECT_EQ(group_names.size(), 1);
  EXPECT_EQ(*group_names.begin(), "manipulator");
  EXPECT_TRUE(chain_groups.empty());
  EXPECT_EQ(joint_groups.size(), 1);
  EXPECT_TRUE(link_groups.empty());
  EXPECT_EQ(joint_groups["manipulator"].size(), 7);

  // Now test failures
  auto is_failure = [g](const std::string& xml_string) {
    tinyxml2::XMLDocument xml_doc;
    EXPECT_TRUE(xml_doc.Parse(xml_string.c_str()) == tinyxml2::XML_SUCCESS);

    tinyxml2::XMLElement* element = xml_doc.FirstChildElement("robot");
    EXPECT_TRUE(element != nullptr);

    try
    {
      parseGroups(*g, element, std::array<int, 3>({ 1, 0, 0 }));
    }
    catch (const std::exception& e)
    {
      tesseract_common::printNestedException(e);
      return true;
    }
    return false;
  };

  {  // missing name
    std::string str = R"(<robot name="abb_irb2400">
                           <group>
                             <joint name="joint_1"/>
                           </group>
                         </robot>)";

    EXPECT_TRUE(is_failure(str));

    tesseract_srdf::SRDFModel srdf_model;
    EXPECT_ANY_THROW(srdf_model.initString(*g, str, locator));  // NOLINT
  }
  {  // missing joints
    std::string str = R"(<robot name="abb_irb2400">
                           <group name="manipulator"/>
                         </robot>)";

    EXPECT_TRUE(is_failure(str));

    tesseract_srdf::SRDFModel srdf_model;
    EXPECT_ANY_THROW(srdf_model.initString(*g, str, locator));  // NOLINT
  }
  {  // missing joint name
    std::string str = R"(<robot name="abb_irb2400">
                           <group name="manipulator">
                             <joint/>
                           </group>
                         </robot>)";

    EXPECT_TRUE(is_failure(str));

    tesseract_srdf::SRDFModel srdf_model;
    EXPECT_ANY_THROW(srdf_model.initString(*g, str, locator));  // NOLINT
  }

  {  // joint does not exist
    std::string str = R"(<robot name="abb_irb2400">
                           <group name="manipulator">
                             <joint name="missing_joint"/>
                           </group>
                         </robot>)";

    EXPECT_TRUE(is_failure(str));

    tesseract_srdf::SRDFModel srdf_model;
    EXPECT_ANY_THROW(srdf_model.initString(*g, str, locator));  // NOLINT
  }
}

TEST(TesseractSRDFUnit, SRDFLinkGroupUnit)  // NOLINT
{
  using namespace tesseract_scene_graph;
  using namespace tesseract_srdf;

  SceneGraph::Ptr g = getABBSceneGraph();
  TempResourceLocator locator;

  std::string str = R"(<robot name="abb_irb2400">
                         <group name="manipulator">
                           <link name="base_link"/>
                           <link name="link_1"/>
                           <link name="link_2"/>
                           <link name="link_3"/>
                           <link name="link_4"/>
                           <link name="link_5"/>
                           <link name="link_6"/>
                           <link name="tool0"/>
                         </group>
                       </robot>)";

  tinyxml2::XMLDocument xml_doc;
  EXPECT_TRUE(xml_doc.Parse(str.c_str()) == tinyxml2::XML_SUCCESS);

  tinyxml2::XMLElement* element = xml_doc.FirstChildElement("robot");
  EXPECT_TRUE(element != nullptr);

  auto [group_names, chain_groups, joint_groups, link_groups] =
      parseGroups(*g, element, std::array<int, 3>({ 1, 0, 0 }));

  EXPECT_EQ(group_names.size(), 1);
  EXPECT_EQ(*group_names.begin(), "manipulator");
  EXPECT_TRUE(chain_groups.empty());
  EXPECT_TRUE(joint_groups.empty());
  EXPECT_EQ(link_groups.size(), 1);
  EXPECT_EQ(link_groups["manipulator"].size(), 8);

  // Now test failures
  auto is_failure = [g](const std::string& xml_string) {
    tinyxml2::XMLDocument xml_doc;
    EXPECT_TRUE(xml_doc.Parse(xml_string.c_str()) == tinyxml2::XML_SUCCESS);

    tinyxml2::XMLElement* element = xml_doc.FirstChildElement("robot");
    EXPECT_TRUE(element != nullptr);

    try
    {
      parseGroups(*g, element, std::array<int, 3>({ 1, 0, 0 }));
    }
    catch (const std::exception& e)
    {
      tesseract_common::printNestedException(e);
      return true;
    }
    return false;
  };

  {  // missing name
    std::string str = R"(<robot name="abb_irb2400">
                           <group>
                             <link name="joint_1"/>
                           </group>
                         </robot>)";

    EXPECT_TRUE(is_failure(str));

    tesseract_srdf::SRDFModel srdf_model;
    EXPECT_ANY_THROW(srdf_model.initString(*g, str, locator));  // NOLINT
  }
  {  // missing joints
    std::string str = R"(<robot name="abb_irb2400">
                           <group name="manipulator"/>
                         </robot>)";

    EXPECT_TRUE(is_failure(str));
  }
  {  // missing joint name
    std::string str = R"(<robot name="abb_irb2400">
                           <group name="manipulator">
                             <link/>
                           </group>
                         </robot>)";

    EXPECT_TRUE(is_failure(str));

    tesseract_srdf::SRDFModel srdf_model;
    EXPECT_ANY_THROW(srdf_model.initString(*g, str, locator));  // NOLINT
  }
  {  // link does not exist
    std::string str = R"(<robot name="abb_irb2400">
                           <group name="manipulator">
                             <link name="missing_link"/>
                           </group>
                         </robot>)";

    EXPECT_TRUE(is_failure(str));

    tesseract_srdf::SRDFModel srdf_model;
    EXPECT_ANY_THROW(srdf_model.initString(*g, str, locator));  // NOLINT
  }
}

TEST(TesseractSRDFUnit, LoadSRDFGroupStatesUnit)  // NOLINT
{
  using namespace tesseract_scene_graph;
  using namespace tesseract_srdf;

  SceneGraph::Ptr g = getABBSceneGraph();
  TempResourceLocator locator;

  std::string xml_string =
      R"(<robot name="abb_irb2400">
           <group_state name="all-zeros" group="manipulator">
             <joint name="joint_1" value="0"/>
             <joint name="joint_2" value="0"/>
             <joint name="joint_3" value="0"/>
             <joint name="joint_4" value="0"/>
             <joint name="joint_5" value="0"/>
             <joint name="joint_6" value="0"/>
           </group_state>
         </robot>)";
  tinyxml2::XMLDocument xml_doc;
  EXPECT_TRUE(xml_doc.Parse(xml_string.c_str()) == tinyxml2::XML_SUCCESS);

  tinyxml2::XMLElement* element = xml_doc.FirstChildElement("robot");
  EXPECT_TRUE(element != nullptr);

  GroupNames group_names = { "manipulator" };
  GroupJointStates group_states = parseGroupStates(*g, group_names, element, std::array<int, 3>({ 1, 0, 0 }));
  EXPECT_EQ(group_states.size(), 1);

  auto it = group_states.find("manipulator");
  EXPECT_TRUE(it != group_states.end());

  auto it2 = it->second.find("all-zeros");
  EXPECT_TRUE(it2 != it->second.end());
  EXPECT_EQ(it2->second.size(), 6);
  EXPECT_DOUBLE_EQ(it2->second["joint_1"], 0);
  EXPECT_DOUBLE_EQ(it2->second["joint_2"], 0);
  EXPECT_DOUBLE_EQ(it2->second["joint_3"], 0);
  EXPECT_DOUBLE_EQ(it2->second["joint_4"], 0);
  EXPECT_DOUBLE_EQ(it2->second["joint_5"], 0);
  EXPECT_DOUBLE_EQ(it2->second["joint_6"], 0);

  // Now test failures
  auto is_failure = [g, &group_names](const std::string& xml_string) {
    tinyxml2::XMLDocument xml_doc;
    EXPECT_TRUE(xml_doc.Parse(xml_string.c_str()) == tinyxml2::XML_SUCCESS);

    tinyxml2::XMLElement* element = xml_doc.FirstChildElement("robot");
    EXPECT_TRUE(element != nullptr);

    try
    {
      parseGroupStates(*g, group_names, element, std::array<int, 3>({ 1, 0, 0 }));
    }
    catch (const std::exception& e)
    {
      tesseract_common::printNestedException(e);
      return true;
    }
    return false;
  };

  {  // missing name
    std::string xml_string =
        R"(<robot name="abb_irb2400">
             <group_state group="manipulator">
               <joint name="joint_1" value="0"/>
             </group_state>
           </robot>)";
    EXPECT_TRUE(is_failure(xml_string));

    tesseract_srdf::SRDFModel srdf_model;
    EXPECT_ANY_THROW(srdf_model.initString(*g, xml_string, locator));  // NOLINT
  }
  {  // missing group
    std::string xml_string =
        R"(<robot name="abb_irb2400">
             <group_state name="all-zeros">
               <joint name="joint_1" value="0"/>
             </group_state>
           </robot>)";
    EXPECT_TRUE(is_failure(xml_string));

    tesseract_srdf::SRDFModel srdf_model;
    EXPECT_ANY_THROW(srdf_model.initString(*g, xml_string, locator));  // NOLINT
  }
  {  // invalid group
    std::string xml_string =
        R"(<robot name="abb_irb2400">
             <group_state name="all-zeros" group="missing_group">
               <joint name="joint_1" value="0"/>
             </group_state>
           </robot>)";
    EXPECT_TRUE(is_failure(xml_string));

    tesseract_srdf::SRDFModel srdf_model;
    EXPECT_ANY_THROW(srdf_model.initString(*g, xml_string, locator));  // NOLINT
  }
  {  // no joints
    std::string xml_string =
        R"(<robot name="abb_irb2400">
             <group_state name="all-zeros" group="manipulator"/>
           </robot>)";
    EXPECT_TRUE(is_failure(xml_string));

    tesseract_srdf::SRDFModel srdf_model;
    EXPECT_ANY_THROW(srdf_model.initString(*g, xml_string, locator));  // NOLINT
  }
  {  // missing joint name
    std::string xml_string =
        R"(<robot name="abb_irb2400">
             <group_state name="all-zeros" group="manipulator">
               <joint value="0"/>
             </group_state>
           </robot>)";
    EXPECT_TRUE(is_failure(xml_string));

    tesseract_srdf::SRDFModel srdf_model;
    EXPECT_ANY_THROW(srdf_model.initString(*g, xml_string, locator));  // NOLINT
  }
  {  // missing joint value
    std::string xml_string =
        R"(<robot name="abb_irb2400">
             <group_state name="all-zeros" group="manipulator">
               <joint name="joint_1"/>
             </group_state>
           </robot>)";
    EXPECT_TRUE(is_failure(xml_string));

    tesseract_srdf::SRDFModel srdf_model;
    EXPECT_ANY_THROW(srdf_model.initString(*g, xml_string, locator));  // NOLINT
  }
  {  // invalid joint value
    std::string xml_string =
        R"(<robot name="abb_irb2400">
             <group_state name="all-zeros" group="manipulator">
               <joint name="joint_1" value="abc"/>
             </group_state>
           </robot>)";
    EXPECT_TRUE(is_failure(xml_string));

    tesseract_srdf::SRDFModel srdf_model;
    EXPECT_ANY_THROW(srdf_model.initString(*g, xml_string, locator));  // NOLINT
  }
  {  // invalid joint name
    std::string xml_string =
        R"(<robot name="abb_irb2400">
             <group_state name="all-zeros" group="manipulator">
               <joint name="missing_joint" value="0"/>
             </group_state>
           </robot>)";
    EXPECT_TRUE(is_failure(xml_string));

    tesseract_srdf::SRDFModel srdf_model;
    EXPECT_ANY_THROW(srdf_model.initString(*g, xml_string, locator));  // NOLINT
  }
}

TEST(TesseractSRDFUnit, SRDFGroupTCPsUnit)  // NOLINT
{
  using namespace tesseract_scene_graph;
  using namespace tesseract_srdf;

  SceneGraph::Ptr g = getABBSceneGraph();
  TempResourceLocator locator;

  std::string str = R"(<robot name="abb_irb2400">
                         <group_tcps group="manipulator">
                           <tcp name="laser" xyz="1 .1 1" rpy="0 1.57 0" />
                           <tcp name="welder" xyz=".1 1 .2" wxyz="1 0 0 0" />
                         </group_tcps>
                       </robot>)";

  tinyxml2::XMLDocument xml_doc;
  EXPECT_TRUE(xml_doc.Parse(str.c_str()) == tinyxml2::XML_SUCCESS);

  tinyxml2::XMLElement* element = xml_doc.FirstChildElement("robot");
  EXPECT_TRUE(element != nullptr);

  GroupTCPs group_tcps = parseGroupTCPs(*g, element, std::array<int, 3>({ 1, 0, 0 }));

  EXPECT_EQ(group_tcps.size(), 1);

  auto it = group_tcps.find("manipulator");
  EXPECT_TRUE(it != group_tcps.end());
  EXPECT_EQ(it->second.size(), 2);

  auto it2 = it->second.find("laser");
  EXPECT_TRUE(it2 != it->second.end());

  auto it3 = it->second.find("welder");
  EXPECT_TRUE(it3 != it->second.end());

  // Now test failures
  auto is_failure = [g](const std::string& xml_string) {
    tinyxml2::XMLDocument xml_doc;
    EXPECT_TRUE(xml_doc.Parse(xml_string.c_str()) == tinyxml2::XML_SUCCESS);

    tinyxml2::XMLElement* element = xml_doc.FirstChildElement("robot");
    EXPECT_TRUE(element != nullptr);

    try
    {
      parseGroupTCPs(*g, element, std::array<int, 3>({ 1, 0, 0 }));
    }
    catch (const std::exception& e)
    {
      tesseract_common::printNestedException(e);
      return true;
    }
    return false;
  };

  {  // missing group
    std::string str = R"(<robot name="abb_irb2400">
                           <group_tcps>
                             <tcp name="laser" xyz="1 .1 1" rpy="0 1.57 0" />
                           </group_tcps>
                         </robot>)";
    EXPECT_TRUE(is_failure(str));

    tesseract_srdf::SRDFModel srdf_model;
    EXPECT_ANY_THROW(srdf_model.initString(*g, str, locator));  // NOLINT
  }
  {  // missing tcp element
    std::string str = R"(<robot name="abb_irb2400">
                           <group_tcps group="manipulator"/>
                         </robot>)";
    EXPECT_TRUE(is_failure(str));

    tesseract_srdf::SRDFModel srdf_model;
    EXPECT_ANY_THROW(srdf_model.initString(*g, str, locator));  // NOLINT
  }
  {  // missing tcp name
    std::string str = R"(<robot name="abb_irb2400">
                           <group_tcps group="manipulator">
                             <tcp xyz="1 .1 1" rpy="0 1.57 0" />
                           </group_tcps>
                         </robot>)";
    EXPECT_TRUE(is_failure(str));

    tesseract_srdf::SRDFModel srdf_model;
    EXPECT_ANY_THROW(srdf_model.initString(*g, str, locator));  // NOLINT
  }
  {  // missing tcp xyz
    std::string str = R"(<robot name="abb_irb2400">
                           <group_tcps group="manipulator">
                             <tcp name="laser" rpy="0 1.57 0" />
                           </group_tcps>
                         </robot>)";
    EXPECT_TRUE(is_failure(str));

    tesseract_srdf::SRDFModel srdf_model;
    EXPECT_ANY_THROW(srdf_model.initString(*g, str, locator));  // NOLINT
  }
  {  // missing tcp orientation
    std::string str = R"(<robot name="abb_irb2400">
                           <group_tcps group="manipulator">
                             <tcp name="laser" xyz="1 .1 1" />
                           </group_tcps>
                         </robot>)";
    EXPECT_TRUE(is_failure(str));

    tesseract_srdf::SRDFModel srdf_model;
    EXPECT_ANY_THROW(srdf_model.initString(*g, str, locator));  // NOLINT
  }
  {  // invalid tcp xyz
    std::string str = R"(<robot name="abb_irb2400">
                           <group_tcps group="manipulator">
                             <tcp name="laser" xyz="a .1 1" rpy="0 1.57 0" />
                           </group_tcps>
                         </robot>)";
    EXPECT_TRUE(is_failure(str));

    tesseract_srdf::SRDFModel srdf_model;
    EXPECT_ANY_THROW(srdf_model.initString(*g, str, locator));  // NOLINT
  }
  {  // invalid orientation
    std::string str = R"(<robot name="abb_irb2400">
                           <group_tcps group="manipulator">
                             <tcp name="laser" xyz="1 .1 1" rpy="a 1.57 0" />
                           </group_tcps>
                         </robot>)";
    EXPECT_TRUE(is_failure(str));

    tesseract_srdf::SRDFModel srdf_model;
    EXPECT_ANY_THROW(srdf_model.initString(*g, str, locator));  // NOLINT
  }
  {  // invalid orientation
    std::string str = R"(<robot name="abb_irb2400">
                           <group_tcps group="manipulator">
                             <tcp name="laser" xyz="1 .1 1" wxyz="a 1.57 0 0" />
                           </group_tcps>
                         </robot>)";
    EXPECT_TRUE(is_failure(str));

    tesseract_srdf::SRDFModel srdf_model;
    EXPECT_ANY_THROW(srdf_model.initString(*g, str, locator));  // NOLINT
  }
}

TEST(TesseractSRDFUnit, SRDFCollisionMarginsUnit)  // NOLINT
{
  using namespace tesseract_scene_graph;
  using namespace tesseract_srdf;

  SceneGraph::Ptr g = getABBSceneGraph();
  TempResourceLocator locator;

  {  // Testing having default margin and pair margin
    std::string str = R"(<robot name="abb_irb2400">
                           <collision_margins default_margin="0.025">
                             <pair_margin link1="link_6" link2="link_5" margin="0.01"/>
                             <pair_margin link1="link_5" link2="link_4" margin="0.015"/>
                           </collision_margins>
                         </robot>)";

    tinyxml2::XMLDocument xml_doc;
    EXPECT_TRUE(xml_doc.Parse(str.c_str()) == tinyxml2::XML_SUCCESS);

    tinyxml2::XMLElement* element = xml_doc.FirstChildElement("robot");
    EXPECT_TRUE(element != nullptr);

    tesseract_common::CollisionMarginData::Ptr margin_data =
        parseCollisionMargins(*g, element, std::array<int, 3>({ 1, 0, 0 }));

    EXPECT_TRUE(margin_data != nullptr);
    EXPECT_NEAR(margin_data->getDefaultCollisionMargin(), 0.025, 1e-6);
    EXPECT_NEAR(margin_data->getMaxCollisionMargin(), 0.025, 1e-6);
    EXPECT_EQ(margin_data->getPairCollisionMargins().size(), 2);
    EXPECT_NEAR(margin_data->getPairCollisionMargin("link_5", "link_6"), 0.01, 1e-6);
    EXPECT_NEAR(margin_data->getPairCollisionMargin("link_5", "link_4"), 0.015, 1e-6);
  }

  {  // Test only having default margin
    std::string str = R"(<robot name="abb_irb2400">
                           <collision_margins default_margin="0.025"/>
                         </robot>)";

    tinyxml2::XMLDocument xml_doc;
    EXPECT_TRUE(xml_doc.Parse(str.c_str()) == tinyxml2::XML_SUCCESS);

    tinyxml2::XMLElement* element = xml_doc.FirstChildElement("robot");
    EXPECT_TRUE(element != nullptr);

    tesseract_common::CollisionMarginData::Ptr margin_data =
        parseCollisionMargins(*g, element, std::array<int, 3>({ 1, 0, 0 }));

    EXPECT_TRUE(margin_data != nullptr);
    EXPECT_NEAR(margin_data->getDefaultCollisionMargin(), 0.025, 1e-6);
    EXPECT_NEAR(margin_data->getMaxCollisionMargin(), 0.025, 1e-6);
    EXPECT_EQ(margin_data->getPairCollisionMargins().size(), 0);
  }

  {  // Testing having negative default margin and pair margin
    std::string str = R"(<robot name="abb_irb2400">
                           <collision_margins default_margin="-0.025">
                             <pair_margin link1="link_6" link2="link_5" margin="-0.01"/>
                             <pair_margin link1="link_5" link2="link_4" margin="-0.015"/>
                           </collision_margins>
                         </robot>)";

    tinyxml2::XMLDocument xml_doc;
    EXPECT_TRUE(xml_doc.Parse(str.c_str()) == tinyxml2::XML_SUCCESS);

    tinyxml2::XMLElement* element = xml_doc.FirstChildElement("robot");
    EXPECT_TRUE(element != nullptr);

    tesseract_common::CollisionMarginData::Ptr margin_data =
        parseCollisionMargins(*g, element, std::array<int, 3>({ 1, 0, 0 }));

    EXPECT_TRUE(margin_data != nullptr);
    EXPECT_NEAR(margin_data->getDefaultCollisionMargin(), -0.025, 1e-6);
    EXPECT_NEAR(margin_data->getMaxCollisionMargin(), -0.01, 1e-6);
    EXPECT_EQ(margin_data->getPairCollisionMargins().size(), 2);
    EXPECT_NEAR(margin_data->getPairCollisionMargin("link_5", "link_6"), -0.01, 1e-6);
    EXPECT_NEAR(margin_data->getPairCollisionMargin("link_5", "link_4"), -0.015, 1e-6);
  }

  {  // Test not having collision margin data
    std::string str = R"(<robot name="abb_irb2400">
                         </robot>)";

    tinyxml2::XMLDocument xml_doc;
    EXPECT_TRUE(xml_doc.Parse(str.c_str()) == tinyxml2::XML_SUCCESS);

    tinyxml2::XMLElement* element = xml_doc.FirstChildElement("robot");
    EXPECT_TRUE(element != nullptr);

    tesseract_common::CollisionMarginData::Ptr margin_data =
        parseCollisionMargins(*g, element, std::array<int, 3>({ 1, 0, 0 }));

    EXPECT_TRUE(margin_data == nullptr);
  }

  // Now test failures
  auto is_failure = [g](const std::string& xml_string) {
    tinyxml2::XMLDocument xml_doc;
    EXPECT_TRUE(xml_doc.Parse(xml_string.c_str()) == tinyxml2::XML_SUCCESS);

    tinyxml2::XMLElement* element = xml_doc.FirstChildElement("robot");
    EXPECT_TRUE(element != nullptr);

    try
    {
      parseCollisionMargins(*g, element, std::array<int, 3>({ 1, 0, 0 }));
    }
    catch (const std::exception& e)
    {
      tesseract_common::printNestedException(e);
      return true;
    }
    return false;
  };

  {  // missing default_margin
    std::string str = R"(<robot name="abb_irb2400">
                           <collision_margins>
                             <pair_margin link1="link_6" link2="link_5" margin="0.01"/>
                             <pair_margin link1="link_5" link2="link_4" margin="0.015"/>
                           </collision_margins>
                         </robot>)";
    EXPECT_TRUE(is_failure(str));

    tesseract_srdf::SRDFModel srdf_model;
    EXPECT_ANY_THROW(srdf_model.initString(*g, str, locator));  // NOLINT
  }

  {  // missing pair link1
    std::string str = R"(<robot name="abb_irb2400">
                           <collision_margins default_margin="0.025">
                             <pair_margin link2="link_5" margin="0.01"/>
                             <pair_margin link1="link_5" link2="link_4" margin="0.015"/>
                           </collision_margins>
                         </robot>)";
    EXPECT_TRUE(is_failure(str));

    tesseract_srdf::SRDFModel srdf_model;
    EXPECT_ANY_THROW(srdf_model.initString(*g, str, locator));  // NOLINT
  }

  {  // missing pair link2
    std::string str = R"(<robot name="abb_irb2400">
                           <collision_margins default_margin="0.025">
                             <pair_margin link1="link_6" link2="link_5" margin="0.01"/>
                             <pair_margin link1="link_5" margin="0.015"/>
                           </collision_margins>
                         </robot>)";
    EXPECT_TRUE(is_failure(str));

    tesseract_srdf::SRDFModel srdf_model;
    EXPECT_ANY_THROW(srdf_model.initString(*g, str, locator));  // NOLINT
  }

  {  // missing pair margin
    std::string str = R"(<robot name="abb_irb2400">
                           <collision_margins default_margin="0.025">
                             <pair_margin link1="link_6" link2="link_5" margin="0.01"/>
                             <pair_margin link1="link_5" link2="link_4"/>
                           </collision_margins>
                         </robot>)";
    EXPECT_TRUE(is_failure(str));

    tesseract_srdf::SRDFModel srdf_model;
    EXPECT_ANY_THROW(srdf_model.initString(*g, str, locator));  // NOLINT
  }

  {  // empty default margin
    std::string str = R"(<robot name="abb_irb2400">
                           <collision_margins default_margin="">
                             <pair_margin link1="link_6" link2="link_5" margin="0.01"/>
                             <pair_margin link1="link_5" link2="link_4" margin="0.01"/>
                           </collision_margins>
                         </robot>)";
    EXPECT_TRUE(is_failure(str));

    tesseract_srdf::SRDFModel srdf_model;
    EXPECT_ANY_THROW(srdf_model.initString(*g, str, locator));  // NOLINT
  }

  {  // empty pair margin
    std::string str = R"(<robot name="abb_irb2400">
                           <collision_margins default_margin="0.025">
                             <pair_margin link1="link_6" link2="link_5" margin=""/>
                             <pair_margin link1="link_5" link2="link_4" margin="0.01"/>
                           </collision_margins>
                         </robot>)";
    EXPECT_TRUE(is_failure(str));

    tesseract_srdf::SRDFModel srdf_model;
    EXPECT_ANY_THROW(srdf_model.initString(*g, str, locator));  // NOLINT
  }

  {  // invalid link name 1, but should not fail
    std::string str = R"(<robot name="abb_irb2400">
                           <collision_margins default_margin="-0.025">
                             <pair_margin link1="missing_link" link2="link_5" margin="0.01"/>
                             <pair_margin link1="link_5" link2="link_4" margin="0.015"/>
                           </collision_margins>
                         </robot>)";
    EXPECT_FALSE(is_failure(str));

    tesseract_srdf::SRDFModel srdf_model;
    EXPECT_NO_THROW(srdf_model.initString(*g, str, locator));  // NOLINT
  }

  {  // invalid link name 2, but should not fail
    std::string str = R"(<robot name="abb_irb2400">
                           <collision_margins default_margin="-0.025">
                             <pair_margin link1="link_6" link2="missing_link" margin="0.01"/>
                             <pair_margin link1="link_5" link2="link_4" margin="0.015"/>
                           </collision_margins>
                         </robot>)";
    EXPECT_FALSE(is_failure(str));

    tesseract_srdf::SRDFModel srdf_model;
    EXPECT_NO_THROW(srdf_model.initString(*g, str, locator));  // NOLINT
  }
}

TEST(TesseractSRDFUnit, AddRemoveChainGroupUnit)  // NOLINT
{
  using namespace tesseract_srdf;
  KinematicsInformation info;

  // ADD
  ChainGroup chain_group;
  chain_group.push_back(std::make_pair("base_link", "tool0"));
  info.addChainGroup("manipulator", chain_group);
  EXPECT_TRUE(info.hasChainGroup("manipulator"));
  EXPECT_TRUE(info.chain_groups.at("manipulator") == chain_group);
  EXPECT_EQ(info.chain_groups.size(), 1);
  EXPECT_EQ(info.group_names.size(), 1);
  EXPECT_TRUE(info.hasGroup("manipulator"));

  // Copy Equal
  KinematicsInformation info1_copy = info;
  EXPECT_EQ(info1_copy, info);

  // Not equal
  chain_group = ChainGroup();
  chain_group.push_back(std::make_pair("tool0", "base_link"));
  info1_copy.addChainGroup("manipulator", chain_group);
  EXPECT_NE(info1_copy, info);

  // Insert
  info1_copy.insert(info);
  EXPECT_EQ(info1_copy, info);

  // Remove
  info.removeChainGroup("manipulator");
  EXPECT_FALSE(info.hasChainGroup("manipulator"));
  EXPECT_FALSE(info.hasGroup("manipulator"));
  EXPECT_EQ(info.chain_groups.size(), 0);
  EXPECT_EQ(info.group_names.size(), 0);
}

TEST(TesseractSRDFUnit, AddRemoveJointGroupUnit)  // NOLINT
{
  using namespace tesseract_srdf;
  KinematicsInformation info;

  // ADD
  JointGroup joint_group = { "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6" };
  info.addJointGroup("manipulator", joint_group);
  EXPECT_TRUE(info.hasJointGroup("manipulator"));
  EXPECT_TRUE(info.hasGroup("manipulator"));
  EXPECT_TRUE(info.joint_groups.at("manipulator") == joint_group);
  EXPECT_EQ(info.joint_groups.size(), 1);
  EXPECT_EQ(info.group_names.size(), 1);

  // Copy Equal
  KinematicsInformation info1_copy = info;
  EXPECT_EQ(info1_copy, info);

  // Different order equal
  joint_group = { "joint_6", "joint_5", "joint_4", "joint_3", "joint_2", "joint_1" };
  info1_copy.addJointGroup("manipulator", joint_group);
  EXPECT_EQ(info1_copy, info);

  // Not Equal
  joint_group = { "joint_6", "joint_5", "joint_4", "joint_3", "joint_2", "joint_0" };
  info1_copy.addJointGroup("manipulator", joint_group);
  EXPECT_NE(info1_copy, info);

  // Insert
  info1_copy.insert(info);
  EXPECT_EQ(info1_copy, info);

  // Remove
  info.removeJointGroup("manipulator");
  EXPECT_FALSE(info.hasJointGroup("manipulator"));
  EXPECT_FALSE(info.hasGroup("manipulator"));
  EXPECT_EQ(info.joint_groups.size(), 0);
  EXPECT_EQ(info.group_names.size(), 0);
}

TEST(TesseractSRDFUnit, AddRemoveLinkGroupUnit)  // NOLINT
{
  using namespace tesseract_srdf;
  KinematicsInformation info;

  // ADD
  LinkGroup link_group = { "link_1", "link_2", "link_3", "link_4", "link_5", "link_6" };
  info.addLinkGroup("manipulator", link_group);
  EXPECT_TRUE(info.hasLinkGroup("manipulator"));
  EXPECT_TRUE(info.hasGroup("manipulator"));
  EXPECT_EQ(info.link_groups.size(), 1);
  EXPECT_EQ(info.group_names.size(), 1);

  // Copy Equal
  KinematicsInformation info1_copy = info;
  EXPECT_EQ(info1_copy, info);

  // Different order equal
  link_group = { "link_6", "link_5", "link_4", "link_3", "link_2", "link_1" };
  info1_copy.addLinkGroup("manipulator", link_group);
  EXPECT_EQ(info1_copy, info);

  // Not Equal
  link_group = { "link_6", "link_5", "link_4", "link_3", "link_2", "link_0" };
  info1_copy.addLinkGroup("manipulator", link_group);
  EXPECT_NE(info1_copy, info);

  // Insert
  info1_copy.insert(info);
  EXPECT_EQ(info1_copy, info);

  // Remove
  info.removeLinkGroup("manipulator");
  EXPECT_FALSE(info.hasLinkGroup("manipulator"));
  EXPECT_FALSE(info.hasGroup("manipulator"));
  EXPECT_EQ(info.link_groups.size(), 0);
  EXPECT_EQ(info.group_names.size(), 0);
}

TEST(TesseractSRDFUnit, AddRemoveGroupJointStateUnit)  // NOLINT
{
  using namespace tesseract_srdf;
  KinematicsInformation info;

  // ADD
  GroupsJointState group_states;
  group_states["joint_1"] = 0;
  group_states["joint_2"] = 0;
  group_states["joint_3"] = 0;
  group_states["joint_4"] = 0;
  group_states["joint_5"] = 0;
  group_states["joint_6"] = 0;

  info.addGroupJointState("manipulator", "all-zeros", group_states);
  EXPECT_TRUE(info.hasGroupJointState("manipulator", "all-zeros"));
  EXPECT_TRUE(info.group_states.at("manipulator").at("all-zeros") == group_states);
  EXPECT_EQ(info.group_states.at("manipulator").size(), 1);
  EXPECT_EQ(info.group_states.size(), 1);

  // Copy Equal
  KinematicsInformation info1_copy = info;
  EXPECT_EQ(info1_copy, info);

  // Not Equal
  group_states["joint_1"] = 1;
  group_states["joint_2"] = 2;
  group_states["joint_3"] = 3;
  group_states["joint_4"] = 4;
  group_states["joint_5"] = 5;
  group_states["joint_6"] = 6;

  info1_copy.addGroupJointState("manipulator", "all-zeros", group_states);
  EXPECT_NE(info1_copy, info);

  // Insert
  info1_copy.insert(info);
  EXPECT_EQ(info1_copy, info);

  // Remove
  info.removeGroupJointState("manipulator", "all-zeros");
  EXPECT_FALSE(info.hasGroupJointState("manipulator", "all-zeros"));
  EXPECT_EQ(info.group_states.size(), 0);
}

TEST(TesseractSRDFUnit, AddRemoveGroupTCPUnit)  // NOLINT
{
  using namespace tesseract_srdf;
  KinematicsInformation info;

  // ADD
  Eigen::Isometry3d tcp_laser = Eigen::Isometry3d::Identity();
  tcp_laser.translation() = Eigen::Vector3d(1, 0.1, 1);

  Eigen::Isometry3d tcp_welder = Eigen::Isometry3d::Identity();
  tcp_welder.translation() = Eigen::Vector3d(0.1, 1, 0.2);

  info.addGroupTCP("manipulator", "laser", tcp_laser);
  info.addGroupTCP("manipulator", "welder", tcp_welder);
  EXPECT_TRUE(info.hasGroupTCP("manipulator", "laser"));
  EXPECT_TRUE(info.hasGroupTCP("manipulator", "welder"));
  EXPECT_TRUE(info.group_tcps.at("manipulator").at("laser").isApprox(tcp_laser, 1e-6));
  EXPECT_TRUE(info.group_tcps.at("manipulator").at("welder").isApprox(tcp_welder, 1e-6));
  EXPECT_EQ(info.group_tcps.at("manipulator").size(), 2);
  EXPECT_EQ(info.group_tcps.size(), 1);

  // Copy Equal
  KinematicsInformation info1_copy = info;
  EXPECT_EQ(info1_copy, info);

  // Not Equal
  tcp_laser = Eigen::Isometry3d::Identity();
  tcp_laser.translation() = Eigen::Vector3d(0.1, 1, 0.2);

  tcp_welder = Eigen::Isometry3d::Identity();
  tcp_welder.translation() = Eigen::Vector3d(1, 0.1, 1);

  info1_copy.addGroupTCP("manipulator", "laser", tcp_laser);
  info1_copy.addGroupTCP("manipulator", "welder", tcp_welder);

  EXPECT_NE(info1_copy, info);

  // Insert
  info1_copy.insert(info);
  EXPECT_EQ(info1_copy, info);

  // Remove
  info.removeGroupTCP("manipulator", "laser");
  info.removeGroupTCP("manipulator", "welder");
  EXPECT_FALSE(info.hasGroupTCP("manipulator", "laser"));
  EXPECT_FALSE(info.hasGroupTCP("manipulator", "welder"));
  EXPECT_EQ(info.group_tcps.size(), 0);
}

TEST(TesseractSRDFUnit, ParseConfigFilePathUnit)  // NOLINT
{
  std::array<int, 3> version{ 1, 0, 0 };
  tesseract_common::GeneralResourceLocator locator;
  tesseract_scene_graph::SceneGraph::Ptr g = getABBSceneGraph();

  {  // valid
    std::string str = R"(<robot name="abb_irb2400">
                           <contact_managers_plugin_config filename="package://tesseract_support/urdf/contact_manager_plugins.yaml"/>
                         </robot>)";

    tinyxml2::XMLDocument xml_doc;
    EXPECT_TRUE(xml_doc.Parse(str.c_str()) == tinyxml2::XML_SUCCESS);

    tinyxml2::XMLElement* robot_element = xml_doc.FirstChildElement("robot");
    EXPECT_TRUE(robot_element != nullptr);

    tinyxml2::XMLElement* element = robot_element->FirstChildElement("contact_managers_plugin_config");
    EXPECT_TRUE(element != nullptr);

    tesseract_common::fs::path path = tesseract_srdf::parseConfigFilePath(locator, element, version);
    EXPECT_TRUE(tesseract_common::fs::exists(path));
  }

  {  // failures (incorrect attribute)
    std::string str = R"(<robot name="abb_irb2400">
                           <contact_managers_plugin_config incorrect_attribute="package://tesseract_support/urdf/contact_manager_plugins.yaml"/>
                         </robot>)";

    tinyxml2::XMLDocument xml_doc;
    EXPECT_TRUE(xml_doc.Parse(str.c_str()) == tinyxml2::XML_SUCCESS);

    tinyxml2::XMLElement* robot_element = xml_doc.FirstChildElement("robot");
    EXPECT_TRUE(robot_element != nullptr);

    tinyxml2::XMLElement* element = robot_element->FirstChildElement("contact_managers_plugin_config");
    EXPECT_TRUE(element != nullptr);

    EXPECT_ANY_THROW(tesseract_srdf::parseConfigFilePath(locator, element, version));  // NOLINT

    tesseract_srdf::SRDFModel srdf_model;
    EXPECT_ANY_THROW(srdf_model.initString(*g, str, locator));  // NOLINT
  }

  {  // failures (resource does not exist)
    std::string str = R"(<robot name="abb_irb2400">
                           <contact_managers_plugin_config filename="package://tesseract_support/urdf/does_not_exist.yaml"/>
                         </robot>)";

    tinyxml2::XMLDocument xml_doc;
    EXPECT_TRUE(xml_doc.Parse(str.c_str()) == tinyxml2::XML_SUCCESS);

    tinyxml2::XMLElement* robot_element = xml_doc.FirstChildElement("robot");
    EXPECT_TRUE(robot_element != nullptr);

    tinyxml2::XMLElement* element = robot_element->FirstChildElement("contact_managers_plugin_config");
    EXPECT_TRUE(element != nullptr);

    EXPECT_ANY_THROW(tesseract_srdf::parseConfigFilePath(locator, element, version));  // NOLINT

    tesseract_srdf::SRDFModel srdf_model;
    EXPECT_ANY_THROW(srdf_model.initString(*g, str, locator));  // NOLINT
  }

  {  // failures (resource not found)
    std::string str = R"(<robot name="abb_irb2400">
                           <contact_managers_plugin_config filename="does_not_exist.yaml"/>
                         </robot>)";

    tinyxml2::XMLDocument xml_doc;
    EXPECT_TRUE(xml_doc.Parse(str.c_str()) == tinyxml2::XML_SUCCESS);

    tinyxml2::XMLElement* robot_element = xml_doc.FirstChildElement("robot");
    EXPECT_TRUE(robot_element != nullptr);

    tinyxml2::XMLElement* element = robot_element->FirstChildElement("contact_managers_plugin_config");
    EXPECT_TRUE(element != nullptr);

    EXPECT_ANY_THROW(tesseract_srdf::parseConfigFilePath(locator, element, version));  // NOLINT

    tesseract_srdf::SRDFModel srdf_model;
    EXPECT_ANY_THROW(srdf_model.initString(*g, str, locator));  // NOLINT
  }
}

TEST(TesseractSRDFUnit, ParseContactManagersPluginConfigUnit)  // NOLINT
{
  std::array<int, 3> version{ 1, 0, 0 };
  tesseract_common::GeneralResourceLocator locator;
  tesseract_scene_graph::SceneGraph::Ptr g = getABBSceneGraph();

  {  // valid
    std::string str = R"(<robot name="abb_irb2400">
                           <contact_managers_plugin_config filename="package://tesseract_support/urdf/contact_manager_plugins.yaml"/>
                         </robot>)";

    tinyxml2::XMLDocument xml_doc;
    EXPECT_TRUE(xml_doc.Parse(str.c_str()) == tinyxml2::XML_SUCCESS);

    tinyxml2::XMLElement* robot_element = xml_doc.FirstChildElement("robot");
    EXPECT_TRUE(robot_element != nullptr);

    tinyxml2::XMLElement* element = robot_element->FirstChildElement("contact_managers_plugin_config");
    EXPECT_TRUE(element != nullptr);

    tesseract_common::ContactManagersPluginInfo info =
        tesseract_srdf::parseContactManagersPluginConfig(locator, element, version);
    EXPECT_FALSE(info.empty());
  }

  {  // failure
    std::string str = R"(<robot name="abb_irb2400">
                           <contact_managers_plugin_config filename="package://tesseract_support/urdf/does_not_exist.yaml"/>
                         </robot>)";

    tinyxml2::XMLDocument xml_doc;
    EXPECT_TRUE(xml_doc.Parse(str.c_str()) == tinyxml2::XML_SUCCESS);

    tinyxml2::XMLElement* robot_element = xml_doc.FirstChildElement("robot");
    EXPECT_TRUE(robot_element != nullptr);

    tinyxml2::XMLElement* element = robot_element->FirstChildElement("contact_managers_plugin_config");
    EXPECT_TRUE(element != nullptr);

    EXPECT_ANY_THROW(tesseract_srdf::parseContactManagersPluginConfig(locator, element, version));  // NOLINT

    tesseract_srdf::SRDFModel srdf_model;
    EXPECT_ANY_THROW(srdf_model.initString(*g, str, locator));  // NOLINT
  }

  {  // failure
    std::string str = R"(<robot name="abb_irb2400">
                           <contact_managers_plugin_config filename="package://tesseract_support/urdf/malformed_config.yaml"/>
                         </robot>)";

    tinyxml2::XMLDocument xml_doc;
    EXPECT_TRUE(xml_doc.Parse(str.c_str()) == tinyxml2::XML_SUCCESS);

    tinyxml2::XMLElement* robot_element = xml_doc.FirstChildElement("robot");
    EXPECT_TRUE(robot_element != nullptr);

    tinyxml2::XMLElement* element = robot_element->FirstChildElement("contact_managers_plugin_config");
    EXPECT_TRUE(element != nullptr);

    EXPECT_ANY_THROW(tesseract_srdf::parseContactManagersPluginConfig(locator, element, version));  // NOLINT

    tesseract_srdf::SRDFModel srdf_model;
    EXPECT_ANY_THROW(srdf_model.initString(*g, str, locator));  // NOLINT
  }
}

TEST(TesseractSRDFUnit, ParseKinematicsPluginConfigUnit)  // NOLINT
{
  std::array<int, 3> version{ 1, 0, 0 };
  tesseract_common::GeneralResourceLocator locator;
  tesseract_scene_graph::SceneGraph::Ptr g = getABBSceneGraph();

  {  // valid
    std::string str = R"(<robot name="abb_irb2400">
                           <kinematics_plugin_config filename="package://tesseract_support/urdf/abb_irb2400_plugins.yaml"/>
                         </robot>)";

    tinyxml2::XMLDocument xml_doc;
    EXPECT_TRUE(xml_doc.Parse(str.c_str()) == tinyxml2::XML_SUCCESS);

    tinyxml2::XMLElement* robot_element = xml_doc.FirstChildElement("robot");
    EXPECT_TRUE(robot_element != nullptr);

    tinyxml2::XMLElement* element = robot_element->FirstChildElement("kinematics_plugin_config");
    EXPECT_TRUE(element != nullptr);

    tesseract_common::KinematicsPluginInfo info =
        tesseract_srdf::parseKinematicsPluginConfig(locator, element, version);
    EXPECT_FALSE(info.empty());
  }

  {  // failure
    std::string str = R"(<robot name="abb_irb2400">
                           <kinematics_plugin_config/>
                         </robot>)";

    tinyxml2::XMLDocument xml_doc;
    EXPECT_TRUE(xml_doc.Parse(str.c_str()) == tinyxml2::XML_SUCCESS);

    tinyxml2::XMLElement* robot_element = xml_doc.FirstChildElement("robot");
    EXPECT_TRUE(robot_element != nullptr);

    tinyxml2::XMLElement* element = robot_element->FirstChildElement("kinematics_plugin_config");
    EXPECT_TRUE(element != nullptr);

    EXPECT_ANY_THROW(tesseract_srdf::parseKinematicsPluginConfig(locator, element, version));  // NOLINT

    tesseract_srdf::SRDFModel srdf_model;
    EXPECT_ANY_THROW(srdf_model.initString(*g, str, locator));  // NOLINT
  }

  {  // failure
    std::string str = R"(<robot name="abb_irb2400">
                           <kinematics_plugin_config filename="package://tesseract_support/urdf/does_not_exist.yaml"/>
                         </robot>)";

    tinyxml2::XMLDocument xml_doc;
    EXPECT_TRUE(xml_doc.Parse(str.c_str()) == tinyxml2::XML_SUCCESS);

    tinyxml2::XMLElement* robot_element = xml_doc.FirstChildElement("robot");
    EXPECT_TRUE(robot_element != nullptr);

    tinyxml2::XMLElement* element = robot_element->FirstChildElement("kinematics_plugin_config");
    EXPECT_TRUE(element != nullptr);

    EXPECT_ANY_THROW(tesseract_srdf::parseKinematicsPluginConfig(locator, element, version));  // NOLINT

    tesseract_srdf::SRDFModel srdf_model;
    EXPECT_ANY_THROW(srdf_model.initString(*g, str, locator));  // NOLINT
  }

  {  // failure
    std::string str = R"(<robot name="abb_irb2400">
                           <kinematics_plugin_config filename="package://tesseract_support/urdf/malformed_config.yaml"/>
                         </robot>)";

    tinyxml2::XMLDocument xml_doc;
    EXPECT_TRUE(xml_doc.Parse(str.c_str()) == tinyxml2::XML_SUCCESS);

    tinyxml2::XMLElement* robot_element = xml_doc.FirstChildElement("robot");
    EXPECT_TRUE(robot_element != nullptr);

    tinyxml2::XMLElement* element = robot_element->FirstChildElement("kinematics_plugin_config");
    EXPECT_TRUE(element != nullptr);

    EXPECT_ANY_THROW(tesseract_srdf::parseKinematicsPluginConfig(locator, element, version));  // NOLINT

    tesseract_srdf::SRDFModel srdf_model;
    EXPECT_ANY_THROW(srdf_model.initString(*g, str, locator));  // NOLINT
  }
}

TEST(TesseractSRDFUnit, ParseCalibrationConfigUnit)  // NOLINT
{
  std::array<int, 3> version{ 1, 0, 0 };
  tesseract_common::GeneralResourceLocator locator;
  tesseract_scene_graph::SceneGraph::Ptr g = getABBSceneGraph();

  {  // valid
    std::string str = R"(<robot name="abb_irb2400">
                           <calibration_config filename="package://tesseract_support/urdf/abb_irb2400_calibration.yaml"/>
                         </robot>)";

    tinyxml2::XMLDocument xml_doc;
    EXPECT_TRUE(xml_doc.Parse(str.c_str()) == tinyxml2::XML_SUCCESS);

    tinyxml2::XMLElement* robot_element = xml_doc.FirstChildElement("robot");
    EXPECT_TRUE(robot_element != nullptr);

    tinyxml2::XMLElement* element = robot_element->FirstChildElement("calibration_config");
    EXPECT_TRUE(element != nullptr);

    tesseract_common::CalibrationInfo info = tesseract_srdf::parseCalibrationConfig(*g, locator, element, version);
    EXPECT_FALSE(info.empty());
  }

  {  // failure
    std::string str = R"(<robot name="abb_irb2400">
                           <calibration_config filename="package://tesseract_support/urdf/does_not_exist.yaml"/>
                         </robot>)";

    tinyxml2::XMLDocument xml_doc;
    EXPECT_TRUE(xml_doc.Parse(str.c_str()) == tinyxml2::XML_SUCCESS);

    tinyxml2::XMLElement* robot_element = xml_doc.FirstChildElement("robot");
    EXPECT_TRUE(robot_element != nullptr);

    tinyxml2::XMLElement* element = robot_element->FirstChildElement("calibration_config");
    EXPECT_TRUE(element != nullptr);

    EXPECT_ANY_THROW(tesseract_srdf::parseCalibrationConfig(*g, locator, element, version));  // NOLINT

    tesseract_srdf::SRDFModel srdf_model;
    EXPECT_ANY_THROW(srdf_model.initString(*g, str, locator));  // NOLINT
  }

  {  // failure
    std::string str = R"(<robot name="abb_irb2400">
                           <calibration_config/>
                         </robot>)";

    tinyxml2::XMLDocument xml_doc;
    EXPECT_TRUE(xml_doc.Parse(str.c_str()) == tinyxml2::XML_SUCCESS);

    tinyxml2::XMLElement* robot_element = xml_doc.FirstChildElement("robot");
    EXPECT_TRUE(robot_element != nullptr);

    tinyxml2::XMLElement* element = robot_element->FirstChildElement("calibration_config");
    EXPECT_TRUE(element != nullptr);

    EXPECT_ANY_THROW(tesseract_srdf::parseCalibrationConfig(*g, locator, element, version));  // NOLINT

    tesseract_srdf::SRDFModel srdf_model;
    EXPECT_ANY_THROW(srdf_model.initString(*g, str, locator));  // NOLINT
  }

  {  // failure
    std::string str = R"(<robot name="abb_irb2400">
                           <calibration_config filename="package://tesseract_support/urdf/malformed_config.yaml"/>
                         </robot>)";

    tinyxml2::XMLDocument xml_doc;
    EXPECT_TRUE(xml_doc.Parse(str.c_str()) == tinyxml2::XML_SUCCESS);

    tinyxml2::XMLElement* robot_element = xml_doc.FirstChildElement("robot");
    EXPECT_TRUE(robot_element != nullptr);

    tinyxml2::XMLElement* element = robot_element->FirstChildElement("calibration_config");
    EXPECT_TRUE(element != nullptr);

    EXPECT_ANY_THROW(tesseract_srdf::parseCalibrationConfig(*g, locator, element, version));  // NOLINT

    tesseract_srdf::SRDFModel srdf_model;
    EXPECT_ANY_THROW(srdf_model.initString(*g, str, locator));  // NOLINT
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
