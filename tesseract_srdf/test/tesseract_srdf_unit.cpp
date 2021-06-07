#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <iostream>
#include <fstream>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/utils.h>
#include <tesseract_scene_graph/graph.h>
#include <tesseract_scene_graph/utils.h>
#include <tesseract_scene_graph/resource_locator.h>
#include <tesseract_srdf/collision_margins.h>
#include <tesseract_srdf/disabled_collisions.h>
#include <tesseract_srdf/group_opw_kinematics.h>
#include <tesseract_srdf/group_rep_kinematics.h>
#include <tesseract_srdf/group_rop_kinematics.h>
#include <tesseract_srdf/group_states.h>
#include <tesseract_srdf/group_tool_center_points.h>
#include <tesseract_srdf/groups.h>
#include <tesseract_srdf/srdf_model.h>
#include <tesseract_srdf/utils.h>

std::string locateResource(const std::string& url)
{
  std::string mod_url = url;
  if (url.find("package://tesseract_support") == 0)
  {
    mod_url.erase(0, strlen("package://tesseract_support"));
    size_t pos = mod_url.find('/');
    if (pos == std::string::npos)
    {
      return std::string();
    }

    std::string package = mod_url.substr(0, pos);
    mod_url.erase(0, pos);
    std::string package_path = std::string(TESSERACT_SUPPORT_DIR);

    if (package_path.empty())
    {
      return std::string();
    }

    mod_url = package_path + mod_url;
  }

  return mod_url;
}

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

  g->addLink(base_link);
  g->addLink(link_1);
  g->addLink(link_2);
  g->addLink(link_3);
  g->addLink(link_4);
  g->addLink(link_5);
  g->addLink(link_6);
  g->addLink(tool0);

  if (config == ABBConfig::ROBOT_ON_RAIL)
  {
    g->addLink(Link("world"));
    g->addLink(Link("axis_1"));
    g->addLink(Link("axis_2"));

    Joint joint_a("joint_axis_1");
    joint_a.axis = Eigen::Vector3d(0, 1, 0);
    joint_a.parent_link_name = "world";
    joint_a.child_link_name = "axis_1";
    joint_a.type = JointType::PRISMATIC;
    g->addJoint(joint_a);

    Joint joint_b("joint_axis_2");
    joint_b.axis = Eigen::Vector3d(1, 0, 0);
    joint_b.parent_link_name = "axis_1";
    joint_b.child_link_name = "axis_2";
    joint_b.type = JointType::PRISMATIC;
    g->addJoint(joint_b);

    Joint joint_c("joint_base_link");
    joint_c.axis = Eigen::Vector3d(0, 1, 0);
    joint_c.parent_link_name = "axis_1";
    joint_c.child_link_name = "base_link";
    joint_c.type = JointType::FIXED;
    g->addJoint(joint_c);
  }
  else if (config == ABBConfig::ROBOT_WITH_POSITIONER)
  {
    g->addLink(Link("world"));
    g->addLink(Link("axis_1"));
    g->addLink(Link("axis_2"));

    Joint joint_a("joint_axis_1");
    joint_a.parent_to_joint_origin_transform.translation() = Eigen::Vector3d(1, 0, 0);
    joint_a.axis = Eigen::Vector3d(0, 1, 0);
    joint_a.parent_link_name = "world";
    joint_a.child_link_name = "axis_1";
    joint_a.type = JointType::PRISMATIC;
    g->addJoint(joint_a);

    Joint joint_b("joint_axis_2");
    joint_b.parent_to_joint_origin_transform.translation() = Eigen::Vector3d(1, 0, 0);
    joint_b.axis = Eigen::Vector3d(1, 0, 0);
    joint_b.parent_link_name = "axis_1";
    joint_b.child_link_name = "axis_2";
    joint_b.type = JointType::PRISMATIC;
    g->addJoint(joint_b);

    Joint joint_c("joint_base_link");
    joint_c.parent_link_name = "world";
    joint_c.child_link_name = "base_link";
    joint_c.type = JointType::FIXED;
    g->addJoint(joint_c);
  }

  Joint joint_1("joint_1");
  joint_1.parent_link_name = "base_link";
  joint_1.child_link_name = "link_1";
  joint_1.type = JointType::FIXED;
  g->addJoint(joint_1);

  Joint joint_2("joint_2");
  joint_2.parent_link_name = "link_1";
  joint_2.child_link_name = "link_2";
  joint_2.type = JointType::REVOLUTE;
  g->addJoint(joint_2);

  Joint joint_3("joint_3");
  joint_3.parent_to_joint_origin_transform.translation()(0) = 1.25;
  joint_3.parent_link_name = "link_2";
  joint_3.child_link_name = "link_3";
  joint_3.type = JointType::REVOLUTE;
  g->addJoint(joint_3);

  Joint joint_4("joint_4");
  joint_4.parent_to_joint_origin_transform.translation()(0) = 1.25;
  joint_4.parent_link_name = "link_3";
  joint_4.child_link_name = "link_4";
  joint_4.type = JointType::REVOLUTE;
  g->addJoint(joint_4);

  Joint joint_5("joint_5");
  joint_5.parent_to_joint_origin_transform.translation()(1) = 1.25;
  joint_5.parent_link_name = "link_4";
  joint_5.child_link_name = "link_5";
  joint_5.type = JointType::REVOLUTE;
  g->addJoint(joint_5);

  Joint joint_6("joint_6");
  joint_6.parent_to_joint_origin_transform.translation()(1) = 1.25;
  joint_6.parent_link_name = "link_5";
  joint_6.child_link_name = "link_6";
  joint_6.type = JointType::REVOLUTE;
  g->addJoint(joint_6);

  Joint joint_tool0("joint_tool0");
  joint_tool0.parent_link_name = "link_6";
  joint_tool0.child_link_name = "tool0";
  joint_tool0.type = JointType::FIXED;
  g->addJoint(joint_tool0);

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

  g.addLink(base_link);
  g.addLink(link_1);
  g.addLink(link_2);
  g.addLink(link_3);
  g.addLink(link_4);
  g.addLink(link_5);

  Joint base_joint("base_joint");
  base_joint.parent_link_name = "base_link";
  base_joint.child_link_name = "link_1";
  base_joint.type = JointType::FIXED;
  g.addJoint(base_joint);

  Joint joint_1("joint_1");
  joint_1.parent_link_name = "link_1";
  joint_1.child_link_name = "link_2";
  joint_1.type = JointType::REVOLUTE;
  g.addJoint(joint_1);

  Joint joint_2("joint_2");
  joint_2.parent_to_joint_origin_transform.translation()(0) = 1.25;
  joint_2.parent_link_name = "link_2";
  joint_2.child_link_name = "link_3";
  joint_2.type = JointType::REVOLUTE;
  g.addJoint(joint_2);

  Joint joint_3("joint_3");
  joint_3.parent_to_joint_origin_transform.translation()(0) = 1.25;
  joint_3.parent_link_name = "link_3";
  joint_3.child_link_name = "link_4";
  joint_3.type = JointType::REVOLUTE;
  g.addJoint(joint_3);

  Joint joint_4("joint_4");
  joint_4.parent_to_joint_origin_transform.translation()(1) = 1.25;
  joint_4.parent_link_name = "link_2";
  joint_4.child_link_name = "link_5";
  joint_4.type = JointType::REVOLUTE;
  g.addJoint(joint_4);

  return g;
}

TEST(TesseractSRDFUnit, LoadSRDFFileUnit)  // NOLINT
{
  using namespace tesseract_scene_graph;
  using namespace tesseract_srdf;

  std::string srdf_file = std::string(TESSERACT_SUPPORT_DIR) + "/urdf/lbr_iiwa_14_r820.srdf";

  ResourceLocator::Ptr locator = std::make_shared<SimpleResourceLocator>(locateResource);
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

  g.addLink(base_link);
  g.addLink(link_1);
  g.addLink(link_2);
  g.addLink(link_3);
  g.addLink(link_4);
  g.addLink(link_5);
  g.addLink(link_6);
  g.addLink(link_7);
  g.addLink(tool0);

  Joint joint_1("joint_a1");
  joint_1.parent_link_name = "base_link";
  joint_1.child_link_name = "link_1";
  joint_1.type = JointType::FIXED;
  g.addJoint(joint_1);

  Joint joint_2("joint_a2");
  joint_2.parent_link_name = "link_1";
  joint_2.child_link_name = "link_2";
  joint_2.type = JointType::REVOLUTE;
  g.addJoint(joint_2);

  Joint joint_3("joint_a3");
  joint_3.parent_to_joint_origin_transform.translation()(0) = 1.25;
  joint_3.parent_link_name = "link_2";
  joint_3.child_link_name = "link_3";
  joint_3.type = JointType::REVOLUTE;
  g.addJoint(joint_3);

  Joint joint_4("joint_a4");
  joint_4.parent_to_joint_origin_transform.translation()(0) = 1.25;
  joint_4.parent_link_name = "link_3";
  joint_4.child_link_name = "link_4";
  joint_4.type = JointType::REVOLUTE;
  g.addJoint(joint_4);

  Joint joint_5("joint_a5");
  joint_5.parent_to_joint_origin_transform.translation()(1) = 1.25;
  joint_5.parent_link_name = "link_4";
  joint_5.child_link_name = "link_5";
  joint_5.type = JointType::REVOLUTE;
  g.addJoint(joint_5);

  Joint joint_6("joint_a6");
  joint_6.parent_to_joint_origin_transform.translation()(1) = 1.25;
  joint_6.parent_link_name = "link_5";
  joint_6.child_link_name = "link_6";
  joint_6.type = JointType::REVOLUTE;
  g.addJoint(joint_6);

  Joint joint_7("joint_a7");
  joint_7.parent_to_joint_origin_transform.translation()(1) = 1.25;
  joint_7.parent_link_name = "link_6";
  joint_7.child_link_name = "link_7";
  joint_7.type = JointType::REVOLUTE;
  g.addJoint(joint_7);

  Joint joint_tool0("joint_tool0");
  joint_tool0.parent_link_name = "link_7";
  joint_tool0.child_link_name = "tool0";
  joint_tool0.type = JointType::FIXED;
  g.addJoint(joint_tool0);

  SRDFModel srdf;
  srdf.initFile(g, srdf_file);
  EXPECT_EQ(srdf.name, "kuka_lbr_iiwa_14_r820");
  EXPECT_EQ(srdf.version[0], 1);
  EXPECT_EQ(srdf.version[1], 0);
  EXPECT_EQ(srdf.version[2], 0);

  processSRDFAllowedCollisions(g, srdf);

  AllowedCollisionMatrix::ConstPtr acm = g.getAllowedCollisionMatrix();

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
  acm.addAllowedCollision("base_link", "link_1", "Adjacent");
  acm.addAllowedCollision("link_1", "link_2", "Adjacent");
  acm.addAllowedCollision("link_2", "link_3", "Adjacent");
  acm.addAllowedCollision("link_3", "link_4", "Adjacent");
  acm.addAllowedCollision("link_4", "link_5", "Adjacent");
  EXPECT_FALSE(srdf.acm.getAllAllowedCollisions().empty());
  srdf.saveToFile(tesseract_common::getTempPath() + "test.srdf");

  SceneGraph g = buildTestSceneGraph();

  SRDFModel srdf_reload;
  srdf_reload.initFile(g, tesseract_common::getTempPath() + "test.srdf");
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

  SceneGraph::Ptr g = getABBSceneGraph();

  std::string xml_string =
      R"(<robot name="abb_irb2400" version="1.0.0">
           <group name="manipulator">
             <chain base_link="base_link" tip_link="tool0" />
           </group>
         </robot>)";

  SRDFModel srdf;
  srdf.initString(*g, xml_string);
  EXPECT_EQ(srdf.name, "abb_irb2400");
  EXPECT_EQ(srdf.version[0], 1);
  EXPECT_EQ(srdf.version[1], 0);
  EXPECT_EQ(srdf.version[2], 0);

  // Now test failures
  {  // missing name
    std::string xml_string =
        R"(<robot version="1.0.0">
             <group name="manipulator">
               <chain base_link="base_link" tip_link="tool0" />
             </group>
           </robot>)";

    SRDFModel srdf;
    EXPECT_ANY_THROW(srdf.initString(*g, xml_string));
  }
  {  // invalid version
    std::string xml_string =
        R"(<robot name="abb_irb2400" version="1 0 0">
             <group name="manipulator">
               <chain base_link="base_link" tip_link="tool0" />
             </group>
           </robot>)";

    SRDFModel srdf;
    EXPECT_ANY_THROW(srdf.initString(*g, xml_string));
  }
  {  // invalid xml
    std::string xml_string =
        R"(<robot name="abb_irb2400" version="1 0 0">
             <group name="manipulator">
               <chain base_link="base_link" tip_link="tool0" />
             </group>
           </robot_invalid>)";

    SRDFModel srdf;
    EXPECT_ANY_THROW(srdf.initString(*g, xml_string));
  }
  {  // initXml missing robot element
    std::string xml_string =
        R"(<missing_robot name="abb_irb2400" version="1.0.0">
             <group name="manipulator">
               <chain base_link="base_link" tip_link="tool0" />
             </group>
           </missing_robot>)";

    SRDFModel srdf;
    EXPECT_ANY_THROW(srdf.initString(*g, xml_string));
  }
  {  // initFile file path does not exist
    SRDFModel srdf;
    EXPECT_ANY_THROW(srdf.initFile(*g, "/tmp/file_does_not_exist.srdf"));
  }
}
TEST(TesseractSRDFUnit, LoadSRDFSaveUnit)  // NOLINT
{
  using namespace tesseract_scene_graph;
  using namespace tesseract_srdf;

  SceneGraph::Ptr g = getABBSceneGraph(ABBConfig::ROBOT_ON_RAIL);
  ResourceLocator::Ptr locator = std::make_shared<SimpleResourceLocator>(locateResource);

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

           <group_rop group="gantry">
             <manipulator group="manipulator" ik_solver="OPWInvKin" reach="2.3"/>
             <positioner group="positioner" fk_solver="KDLFwdKin">
               <joint name="joint_axis_1" resolution="0.1"/>
               <joint name="joint_axis_2" resolution="0.2"/>
             </positioner>
           </group_rop>

           <group_tcps group="gantry">
             <tcp name="laser" xyz="1 .1 1" rpy="0 1.57 0" />
             <tcp name="welder" xyz=".1 1 .2" wxyz="1 0 0 0" />
           </group_tcps>

           <group_opw group="manipulator" a1="0.1" a2="-0.135" b="0" c1="0.615" c2="0.705" c3="0.755" c4="0.085" offsets="0.0 0.0 -1.570796 0.0 0.0 0.0" sign_corrections="1 1 1 -1 1 1"/>

           <disable_collisions link1="base_link" link2="link_1" reason="Adjacent" />
           <disable_collisions link1="base_link" link2="link_2" reason="Never" />
           <disable_collisions link1="base_link" link2="link_3" reason="Never" />

           <collision_margins default_margin="0.025">
             <pair_margin link1="link_6" link2="link_5" margin="0.01"/>
             <pair_margin link1="link_5" link2="link_4" margin="0.015"/>
           </collision_margins>
         </robot>)";

  SRDFModel srdf_save;
  srdf_save.initString(*g, xml_string);
  std::string save_path = tesseract_common::getTempPath() + "unit_test_save_srdf.srdf";
  EXPECT_TRUE(srdf_save.saveToFile(save_path));

  SRDFModel srdf;
  srdf.initFile(*g, save_path);
  EXPECT_EQ(srdf.name, "abb_irb2400");
  EXPECT_EQ(srdf.version[0], 1);
  EXPECT_EQ(srdf.version[1], 0);
  EXPECT_EQ(srdf.version[2], 0);

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

  // Check for rop group information
  EXPECT_EQ(kin_info.group_rop_kinematics.size(), 1);
  auto rop_solver_it = kin_info.group_rop_kinematics.find("gantry");
  EXPECT_TRUE(rop_solver_it != kin_info.group_rop_kinematics.end());

  // Check for opw group information
  EXPECT_EQ(kin_info.group_opw_kinematics.size(), 1);
  auto opw_solver_it = kin_info.group_opw_kinematics.find("manipulator");
  EXPECT_TRUE(opw_solver_it != kin_info.group_opw_kinematics.end());

  // Check for group states information
  EXPECT_EQ(kin_info.group_states.size(), 1);
  auto group_state_it = kin_info.group_states.find("manipulator");
  EXPECT_TRUE(group_state_it != kin_info.group_states.end());
  EXPECT_EQ(group_state_it->second.size(), 1);
  EXPECT_TRUE(group_state_it->second.find("all-zeros") != group_state_it->second.end());

  AllowedCollisionMatrix::ConstPtr acm = g->getAllowedCollisionMatrix();
  EXPECT_TRUE(acm->isCollisionAllowed("base_link", "link_1"));
  EXPECT_TRUE(acm->isCollisionAllowed("base_link", "link_2"));
  EXPECT_TRUE(acm->isCollisionAllowed("base_link", "link_3"));

  EXPECT_TRUE(srdf.collision_margin_data != nullptr);
  EXPECT_NEAR(srdf.collision_margin_data->getDefaultCollisionMargin(), 0.025, 1e-6);
  EXPECT_NEAR(srdf.collision_margin_data->getMaxCollisionMargin(), 0.025, 1e-6);
  EXPECT_EQ(srdf.collision_margin_data->getPairCollisionMargins().size(), 2);
  EXPECT_NEAR(srdf.collision_margin_data->getPairCollisionMargin("link_5", "link_6"), 0.01, 1e-6);
  EXPECT_NEAR(srdf.collision_margin_data->getPairCollisionMargin("link_5", "link_4"), 0.015, 1e-6);
}

TEST(TesseractSRDFUnit, LoadSRDFSave2Unit)  // NOLINT
{
  using namespace tesseract_scene_graph;
  using namespace tesseract_srdf;

  SceneGraph::Ptr g = getABBSceneGraph(ABBConfig::ROBOT_WITH_POSITIONER);
  ResourceLocator::Ptr locator = std::make_shared<SimpleResourceLocator>(locateResource);

  std::string xml_string =
      R"(<robot name="abb_irb2400" version="1.0.0">
           <group name="manipulator">
             <chain base_link="base_link" tip_link="tool0" />
           </group>
           <group name="positioner">
             <chain base_link="world" tip_link="axis_2" />
           </group>
           <group name="gantry">
             <joint name="joint_axis_1"/>
             <joint name="joint_axis_2"/>
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

           <group_rep group="gantry">
             <manipulator group="manipulator" ik_solver="OPWInvKin" reach="2.3"/>
             <positioner group="positioner" fk_solver="KDLFwdKin">
               <joint name="joint_axis_1" resolution="0.1"/>
               <joint name="joint_axis_2" resolution="0.2"/>
             </positioner>
           </group_rep>

           <group_tcps group="gantry">
             <tcp name="laser" xyz="1 .1 1" rpy="0 1.57 0" />
             <tcp name="welder" xyz=".1 1 .2" wxyz="1 0 0 0" />
           </group_tcps>

           <group_opw group="manipulator" a1="0.1" a2="-0.135" b="0" c1="0.615" c2="0.705" c3="0.755" c4="0.085" offsets="0.0 0.0 -1.570796 0.0 0.0 0.0" sign_corrections="1 1 1 -1 1 1"/>

           <disable_collisions link1="base_link" link2="link_1" reason="Adjacent" />
           <disable_collisions link1="base_link" link2="link_2" reason="Never" />
           <disable_collisions link1="base_link" link2="link_3" reason="Never" />

           <collision_margins default_margin="0.025">
             <pair_margin link1="link_6" link2="link_5" margin="0.01"/>
             <pair_margin link1="link_5" link2="link_4" margin="0.015"/>
           </collision_margins>
         </robot>)";

  SRDFModel srdf_save;

  try
  {
    srdf_save.initString(*g, xml_string);
  }
  catch (const std::exception& e)
  {
    tesseract_common::printNestedException(e);
    throw e;
  }

  std::string save_path = tesseract_common::getTempPath() + "unit_test_save2_srdf.srdf";
  EXPECT_TRUE(srdf_save.saveToFile(save_path));

  SRDFModel srdf;

  try
  {
    srdf.initFile(*g, save_path);
  }
  catch (const std::exception& e)
  {
    tesseract_common::printNestedException(e);
    throw e;
  }

  EXPECT_EQ(srdf.name, "abb_irb2400");
  EXPECT_EQ(srdf.version[0], 1);
  EXPECT_EQ(srdf.version[1], 0);
  EXPECT_EQ(srdf.version[2], 0);

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
  EXPECT_EQ(kin_info.chain_groups.size(), 2);
  auto chain_manipulator_it = kin_info.chain_groups.find("manipulator");
  auto chain_positioner_it = kin_info.chain_groups.find("positioner");
  EXPECT_TRUE(chain_manipulator_it != kin_info.chain_groups.end());
  EXPECT_TRUE(chain_positioner_it != kin_info.chain_groups.end());

  // Check for joint group information
  EXPECT_EQ(kin_info.joint_groups.size(), 1);
  auto joint_manipulator_it = kin_info.joint_groups.find("gantry");
  EXPECT_TRUE(joint_manipulator_it != kin_info.joint_groups.end());

  // Check for rop group information
  EXPECT_EQ(kin_info.group_rep_kinematics.size(), 1);
  auto rep_solver_it = kin_info.group_rep_kinematics.find("gantry");
  EXPECT_TRUE(rep_solver_it != kin_info.group_rep_kinematics.end());

  // Check for opw group information
  EXPECT_EQ(kin_info.group_opw_kinematics.size(), 1);
  auto opw_solver_it = kin_info.group_opw_kinematics.find("manipulator");
  EXPECT_TRUE(opw_solver_it != kin_info.group_opw_kinematics.end());

  // Check for group states information
  EXPECT_EQ(kin_info.group_states.size(), 1);
  auto group_state_it = kin_info.group_states.find("manipulator");
  EXPECT_TRUE(group_state_it != kin_info.group_states.end());
  EXPECT_EQ(group_state_it->second.size(), 1);
  EXPECT_TRUE(group_state_it->second.find("all-zeros") != group_state_it->second.end());

  AllowedCollisionMatrix::ConstPtr acm = g->getAllowedCollisionMatrix();
  EXPECT_TRUE(acm->isCollisionAllowed("base_link", "link_1"));
  EXPECT_TRUE(acm->isCollisionAllowed("base_link", "link_2"));
  EXPECT_TRUE(acm->isCollisionAllowed("base_link", "link_3"));

  EXPECT_TRUE(srdf.collision_margin_data != nullptr);
  EXPECT_NEAR(srdf.collision_margin_data->getDefaultCollisionMargin(), 0.025, 1e-6);
  EXPECT_NEAR(srdf.collision_margin_data->getMaxCollisionMargin(), 0.025, 1e-6);
  EXPECT_EQ(srdf.collision_margin_data->getPairCollisionMargins().size(), 2);
  EXPECT_NEAR(srdf.collision_margin_data->getPairCollisionMargin("link_5", "link_6"), 0.01, 1e-6);
  EXPECT_NEAR(srdf.collision_margin_data->getPairCollisionMargin("link_5", "link_4"), 0.015, 1e-6);
}

TEST(TesseractSRDFUnit, LoadSRDFROPUnit)  // NOLINT
{
  using namespace tesseract_scene_graph;
  using namespace tesseract_srdf;

  SceneGraph::Ptr g = getABBSceneGraph(ABBConfig::ROBOT_ON_RAIL);
  ResourceLocator::Ptr locator = std::make_shared<SimpleResourceLocator>(locateResource);

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

           <group_rop group="gantry">
             <manipulator group="manipulator" ik_solver="OPWInvKin" reach="2.3"/>
             <positioner group="positioner" fk_solver="KDLFwdKin">
               <joint name="joint_axis_1" resolution="0.1"/>
               <joint name="joint_axis_2" resolution="0.2"/>
             </positioner>
           </group_rop>

           <group_tcps group="gantry">
             <tcp name="laser" xyz="1 .1 1" rpy="0 1.57 0" />
             <tcp name="welder" xyz=".1 1 .2" wxyz="1 0 0 0" />
           </group_tcps>

           <group_opw group="manipulator" a1="0.1" a2="-0.135" b="0" c1="0.615" c2="0.705" c3="0.755" c4="0.085" offsets="0.0 0.0 -1.570796 0.0 0.0 0.0" sign_corrections="1 1 1 -1 1 1"/>

           <disable_collisions link1="base_link" link2="link_1" reason="Adjacent" />
           <disable_collisions link1="base_link" link2="link_2" reason="Never" />
           <disable_collisions link1="base_link" link2="link_3" reason="Never" />
         </robot>)";

  SRDFModel srdf;
  const SRDFModel& srdf_const = srdf;
  try
  {
    srdf.initString(*g, xml_string);
  }
  catch (const std::exception& e)
  {
    tesseract_common::printNestedException(e);
    throw e;
  }

  EXPECT_EQ(srdf.name, "abb_irb2400");
  EXPECT_EQ(srdf_const.name, "abb_irb2400");
  EXPECT_EQ(srdf.version[0], 1);
  EXPECT_EQ(srdf.version[1], 0);
  EXPECT_EQ(srdf.version[2], 0);

  processSRDFAllowedCollisions(*g, srdf);

  KinematicsInformation& kin_info = srdf.kinematics_information;
  const KinematicsInformation& kin_info_const = srdf_const.kinematics_information;
  EXPECT_TRUE(&kin_info == &kin_info_const);

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

  // Check for rop group information
  EXPECT_EQ(kin_info.group_rop_kinematics.size(), 1);
  auto rop_solver_it = kin_info.group_rop_kinematics.find("gantry");
  EXPECT_TRUE(rop_solver_it != kin_info.group_rop_kinematics.end());

  // Check for opw group information
  EXPECT_EQ(kin_info.group_opw_kinematics.size(), 1);
  auto opw_solver_it = kin_info.group_opw_kinematics.find("manipulator");
  EXPECT_TRUE(opw_solver_it != kin_info.group_opw_kinematics.end());

  // Check for group states information
  EXPECT_EQ(kin_info.group_states.size(), 1);
  auto group_state_it = kin_info.group_states.find("manipulator");
  EXPECT_TRUE(group_state_it != kin_info.group_states.end());
  EXPECT_EQ(group_state_it->second.size(), 1);
  EXPECT_TRUE(group_state_it->second.find("all-zeros") != group_state_it->second.end());

  AllowedCollisionMatrix::ConstPtr acm = g->getAllowedCollisionMatrix();
  EXPECT_TRUE(acm->isCollisionAllowed("base_link", "link_1"));
  EXPECT_TRUE(acm->isCollisionAllowed("base_link", "link_2"));
  EXPECT_TRUE(acm->isCollisionAllowed("base_link", "link_3"));
}

TEST(TesseractSRDFUnit, LoadSRDFREPUnit)  // NOLINT
{
  using namespace tesseract_scene_graph;
  using namespace tesseract_srdf;

  SceneGraph::Ptr g = getABBSceneGraph(ABBConfig::ROBOT_WITH_POSITIONER);
  ResourceLocator::Ptr locator = std::make_shared<SimpleResourceLocator>(locateResource);

  std::string xml_string =
      R"(<robot name="abb_irb2400" version="1.0.0">
           <group name="manipulator">
             <chain base_link="base_link" tip_link="tool0" />
           </group>
           <group name="positioner">
             <chain base_link="world" tip_link="axis_1" />
           </group>
           <group name="gantry">
             <joint name="joint_axis_1"/>
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

           <group_rep group="gantry">
             <manipulator group="manipulator" ik_solver="OPWInvKin" reach="2.3"/>
             <positioner group="positioner" fk_solver="KDLFwdKin">
               <joint name="joint_axis_1" resolution="0.1"/>
               <joint name="joint_axis_2" resolution="0.2"/>
             </positioner>
           </group_rep>

           <group_tcps group="gantry">
             <tcp name="laser" xyz="1 .1 1" rpy="0 1.57 0" />
             <tcp name="welder" xyz=".1 1 .2" wxyz="1 0 0 0" />
           </group_tcps>

           <group_opw group="manipulator" a1="0.1" a2="-0.135" b="0" c1="0.615" c2="0.705" c3="0.755" c4="0.085" offsets="0.0 0.0 -1.570796 0.0 0.0 0.0" sign_corrections="1 1 1 -1 1 1"/>

           <disable_collisions link1="base_link" link2="link_1" reason="Adjacent" />
           <disable_collisions link1="base_link" link2="link_2" reason="Never" />
           <disable_collisions link1="base_link" link2="link_3" reason="Never" />
         </robot>)";

  SRDFModel srdf;
  try
  {
    srdf.initString(*g, xml_string);
  }
  catch (const std::exception& e)
  {
    tesseract_common::printNestedException(e);
    throw e;
  }

  EXPECT_EQ(srdf.name, "abb_irb2400");
  EXPECT_EQ(srdf.version[0], 1);
  EXPECT_EQ(srdf.version[1], 0);
  EXPECT_EQ(srdf.version[2], 0);

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
  EXPECT_EQ(kin_info.chain_groups.size(), 2);
  auto chain_manipulator_it = kin_info.chain_groups.find("manipulator");
  auto chain_positioner_it = kin_info.chain_groups.find("positioner");
  EXPECT_TRUE(chain_manipulator_it != kin_info.chain_groups.end());
  EXPECT_TRUE(chain_positioner_it != kin_info.chain_groups.end());

  // Check for joint group information
  EXPECT_EQ(kin_info.joint_groups.size(), 1);
  auto joint_manipulator_it = kin_info.joint_groups.find("gantry");
  EXPECT_TRUE(joint_manipulator_it != kin_info.joint_groups.end());

  // Check for rop group information
  EXPECT_EQ(kin_info.group_rep_kinematics.size(), 1);
  auto rep_solver_it = kin_info.group_rep_kinematics.find("gantry");
  EXPECT_TRUE(rep_solver_it != kin_info.group_rep_kinematics.end());

  // Check for opw group information
  EXPECT_EQ(kin_info.group_opw_kinematics.size(), 1);
  auto opw_solver_it = kin_info.group_opw_kinematics.find("manipulator");
  EXPECT_TRUE(opw_solver_it != kin_info.group_opw_kinematics.end());

  // Check for group states information
  EXPECT_EQ(kin_info.group_states.size(), 1);
  auto group_state_it = kin_info.group_states.find("manipulator");
  EXPECT_TRUE(group_state_it != kin_info.group_states.end());
  EXPECT_EQ(group_state_it->second.size(), 1);
  EXPECT_TRUE(group_state_it->second.find("all-zeros") != group_state_it->second.end());

  AllowedCollisionMatrix::ConstPtr acm = g->getAllowedCollisionMatrix();
  EXPECT_TRUE(acm->isCollisionAllowed("base_link", "link_1"));
  EXPECT_TRUE(acm->isCollisionAllowed("base_link", "link_2"));
  EXPECT_TRUE(acm->isCollisionAllowed("base_link", "link_3"));
}

TEST(TesseractSRDFUnit, LoadSRDFAllowedCollisionMatrixUnit)  // NOLINT
{
  using namespace tesseract_scene_graph;
  using namespace tesseract_srdf;

  SceneGraph::Ptr g = getABBSceneGraph();

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

  AllowedCollisionMatrix acm = parseDisabledCollisions(*g, element, std::array<int, 3>({ 1, 0, 0 }));
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
  }
  {  // missing link2
    std::string xml_string =
        R"(<robot name="abb_irb2400">
             <disable_collisions link1="base_link" link2="link_1" reason="Adjacent" />
             <disable_collisions link1="base_link" reason="Never" />
             <disable_collisions link1="base_link" link2="link_3" reason="Never" />
           </robot>)";
    EXPECT_TRUE(is_failure(xml_string));
  }
  {  // missing reason but should not fail
    std::string xml_string =
        R"(<robot name="abb_irb2400">
             <disable_collisions link1="base_link" link2="link_1" reason="Adjacent" />
             <disable_collisions link1="base_link" link2="link_2" reason="Never" />
             <disable_collisions link1="base_link" link2="link_3" />
           </robot>)";
    EXPECT_FALSE(is_failure(xml_string));
  }
  {  // invalid link1 but should not fail
    std::string xml_string =
        R"(<robot name="abb_irb2400">
             <disable_collisions link1="missing_link" link2="link_1" reason="Adjacent" />
             <disable_collisions link1="base_link" link2="link_2" reason="Never" />
             <disable_collisions link1="base_link" link2="link_3" reason="Never" />
           </robot>)";
    EXPECT_FALSE(is_failure(xml_string));
  }
  {  // invalid link2 but should not fail
    std::string xml_string =
        R"(<robot name="abb_irb2400">
             <disable_collisions link1="base_link" link2="link_1" reason="Adjacent" />
             <disable_collisions link1="base_link" link2="missing_link" reason="Never" />
             <disable_collisions link1="base_link" link2="link_3" reason="Never" />
           </robot>)";
    EXPECT_FALSE(is_failure(xml_string));
  }
  {  // The reason is numeric but still a valid string so should not fail
    std::string xml_string =
        R"(<robot name="abb_irb2400">
             <disable_collisions link1="base_link" link2="link_1" reason="Adjacent" />
             <disable_collisions link1="base_link" link2="missing_link" reason="Never" />
             <disable_collisions link1="base_link" link2="link_3" reason="2.335" />
           </robot>)";
    EXPECT_FALSE(is_failure(xml_string));
  }
}

TEST(TesseractSRDFUnit, LoadSRDFOPWKinematicsUnit)  // NOLINT
{
  using namespace tesseract_scene_graph;
  using namespace tesseract_srdf;

  SceneGraph::Ptr g = getABBSceneGraph();

  std::string xml_string =
      R"(<robot name="abb_irb2400">
           <group_opw group="manipulator" a1="0.1" a2="-0.135" b="0" c1="0.615" c2="0.705" c3="0.755" c4="0.085" offsets="0.0 0.0 -1.570796 0.0 0.0 0.0" sign_corrections="1 1 1 -1 1 1"/>
         </robot>)";
  tinyxml2::XMLDocument xml_doc;
  EXPECT_TRUE(xml_doc.Parse(xml_string.c_str()) == tinyxml2::XML_SUCCESS);

  tinyxml2::XMLElement* element = xml_doc.FirstChildElement("robot");
  EXPECT_TRUE(element != nullptr);

  GroupOPWKinematics opw_groups = parseGroupOPWKinematics(*g, element, std::array<int, 3>({ 1, 0, 0 }));
  OPWKinematicParameters opw = opw_groups["manipulator"];
  EXPECT_NEAR(opw.a1, 0.1, 1e-8);
  EXPECT_NEAR(opw.a2, -0.135, 1e-8);
  EXPECT_NEAR(opw.b, 0.0, 1e-8);
  EXPECT_NEAR(opw.c1, 0.615, 1e-8);
  EXPECT_NEAR(opw.c2, 0.705, 1e-8);
  EXPECT_NEAR(opw.c3, 0.755, 1e-8);
  EXPECT_NEAR(opw.c4, 0.085, 1e-8);
  EXPECT_NEAR(opw.offsets[0], 0.0, 1e-8);
  EXPECT_NEAR(opw.offsets[1], 0.0, 1e-8);
  EXPECT_NEAR(opw.offsets[2], -1.570796, 1e-8);
  EXPECT_NEAR(opw.offsets[3], 0.0, 1e-8);
  EXPECT_NEAR(opw.offsets[4], 0.0, 1e-8);
  EXPECT_NEAR(opw.offsets[5], 0.0, 1e-8);
  EXPECT_EQ(opw.sign_corrections[0], 1);
  EXPECT_EQ(opw.sign_corrections[1], 1);
  EXPECT_EQ(opw.sign_corrections[2], 1);
  EXPECT_EQ(opw.sign_corrections[3], -1);
  EXPECT_EQ(opw.sign_corrections[4], 1);
  EXPECT_EQ(opw.sign_corrections[5], 1);

  // Now test failures
  auto is_failure = [g](const std::string& xml_string) {
    tinyxml2::XMLDocument xml_doc;
    EXPECT_TRUE(xml_doc.Parse(xml_string.c_str()) == tinyxml2::XML_SUCCESS);

    tinyxml2::XMLElement* element = xml_doc.FirstChildElement("robot");
    EXPECT_TRUE(element != nullptr);

    try
    {
      parseGroupOPWKinematics(*g, element, std::array<int, 3>({ 1, 0, 0 }));
    }
    catch (const std::exception& e)
    {
      tesseract_common::printNestedException(e);
      return true;
    }
    return false;
  };

  {  // missing a1
    std::string xml_string =
        R"(<robot name="abb_irb2400">
             <group_opw group="manipulator" a2="-0.135" b="0" c1="0.615" c2="0.705" c3="0.755" c4="0.085" offsets="0.0 0.0 -1.570796 0.0 0.0 0.0" sign_corrections="1 1 1 -1 1 1"/>
           </robot>)";
    EXPECT_TRUE(is_failure(xml_string));
  }
  {  // missing a2
    std::string xml_string =
        R"(<robot name="abb_irb2400">
             <group_opw group="manipulator" a1="0.1" b="0" c1="0.615" c2="0.705" c3="0.755" c4="0.085" offsets="0.0 0.0 -1.570796 0.0 0.0 0.0" sign_corrections="1 1 1 -1 1 1"/>
           </robot>)";
    EXPECT_TRUE(is_failure(xml_string));
  }
  {  // missing b
    std::string xml_string =
        R"(<robot name="abb_irb2400">
             <group_opw group="manipulator" a1="0.1" a2="-0.135" c1="0.615" c2="0.705" c3="0.755" c4="0.085" offsets="0.0 0.0 -1.570796 0.0 0.0 0.0" sign_corrections="1 1 1 -1 1 1"/>
           </robot>)";
    EXPECT_TRUE(is_failure(xml_string));
  }
  {  // missing c1
    std::string xml_string =
        R"(<robot name="abb_irb2400">
             <group_opw group="manipulator" a1="0.1" a2="-0.135" b="0" c2="0.705" c3="0.755" c4="0.085" offsets="0.0 0.0 -1.570796 0.0 0.0 0.0" sign_corrections="1 1 1 -1 1 1"/>
           </robot>)";
    EXPECT_TRUE(is_failure(xml_string));
  }
  {  // missing c2
    std::string xml_string =
        R"(<robot name="abb_irb2400">
             <group_opw group="manipulator" a1="0.1" a2="-0.135" b="0" c1="0.615" c3="0.755" c4="0.085" offsets="0.0 0.0 -1.570796 0.0 0.0 0.0" sign_corrections="1 1 1 -1 1 1"/>
           </robot>)";
    EXPECT_TRUE(is_failure(xml_string));
  }
  {  // missing c3
    std::string xml_string =
        R"(<robot name="abb_irb2400">
             <group_opw group="manipulator" a1="0.1" a2="-0.135" b="0" c1="0.615" c2="0.705" c4="0.085" offsets="0.0 0.0 -1.570796 0.0 0.0 0.0" sign_corrections="1 1 1 -1 1 1"/>
           </robot>)";
    EXPECT_TRUE(is_failure(xml_string));
  }
  {  // missing c4
    std::string xml_string =
        R"(<robot name="abb_irb2400">
             <group_opw group="manipulator" a1="0.1" a2="-0.135" b="0" c1="0.615" c2="0.705" c3="0.755" offsets="0.0 0.0 -1.570796 0.0 0.0 0.0" sign_corrections="1 1 1 -1 1 1"/>
           </robot>)";
    EXPECT_TRUE(is_failure(xml_string));
  }
  {  // missing offset is allowed
    std::string xml_string =
        R"(<robot name="abb_irb2400">
             <group_opw group="manipulator" a1="0.1" a2="-0.135" b="0" c1="0.615" c2="0.705" c3="0.755" c4="0.085" sign_corrections="1 1 1 -1 1 1"/>
           </robot>)";
    EXPECT_FALSE(is_failure(xml_string));
  }
  {  // missing sign_corrections is allowed
    std::string xml_string =
        R"(<robot name="abb_irb2400">
             <group_opw group="manipulator" a1="0.1" a2="-0.135" b="0" c1="0.615" c2="0.705" c3="0.755" c4="0.085" offsets="0.0 0.0 -1.570796 0.0 0.0 0.0"/>
           </robot>)";
    EXPECT_FALSE(is_failure(xml_string));
  }

  {  // invalid a1
    std::string xml_string =
        R"(<robot name="abb_irb2400">
             <group_opw group="manipulator" a1="adfa" a2="-0.135" b="0" c1="0.615" c2="0.705" c3="0.755" c4="0.085" offsets="0.0 0.0 -1.570796 0.0 0.0 0.0" sign_corrections="1 1 1 -1 1 1"/>
           </robot>)";
    EXPECT_TRUE(is_failure(xml_string));
  }
  {  // invalid a2
    std::string xml_string =
        R"(<robot name="abb_irb2400">
             <group_opw group="manipulator" a1="0.1" a2="adfas" b="0" c1="0.615" c2="0.705" c3="0.755" c4="0.085" offsets="0.0 0.0 -1.570796 0.0 0.0 0.0" sign_corrections="1 1 1 -1 1 1"/>
           </robot>)";
    EXPECT_TRUE(is_failure(xml_string));
  }
  {  // invalid b
    std::string xml_string =
        R"(<robot name="abb_irb2400">
             <group_opw group="manipulator" a1="0.1" a2="-0.135" b="adfas" c1="0.615" c2="0.705" c3="0.755" c4="0.085" offsets="0.0 0.0 -1.570796 0.0 0.0 0.0" sign_corrections="1 1 1 -1 1 1"/>
           </robot>)";
    EXPECT_TRUE(is_failure(xml_string));
  }
  {  // invalid c1
    std::string xml_string =
        R"(<robot name="abb_irb2400">
             <group_opw group="manipulator" a1="0.1" a2="-0.135" b="0" c1="adfsa" c2="0.705" c3="0.755" c4="0.085" offsets="0.0 0.0 -1.570796 0.0 0.0 0.0" sign_corrections="1 1 1 -1 1 1"/>
           </robot>)";
    EXPECT_TRUE(is_failure(xml_string));
  }
  {  // invalid c2
    std::string xml_string =
        R"(<robot name="abb_irb2400">
             <group_opw group="manipulator" a1="0.1" a2="-0.135" b="0" c1="0.615" c2="asdfasd" c3="0.755" c4="0.085" offsets="0.0 0.0 -1.570796 0.0 0.0 0.0" sign_corrections="1 1 1 -1 1 1"/>
           </robot>)";
    EXPECT_TRUE(is_failure(xml_string));
  }
  {  // invalide c3
    std::string xml_string =
        R"(<robot name="abb_irb2400">
             <group_opw group="manipulator" a1="0.1" a2="-0.135" b="0" c1="0.615" c2="0.705" c3="adfas" c4="0.085" offsets="0.0 0.0 -1.570796 0.0 0.0 0.0" sign_corrections="1 1 1 -1 1 1"/>
           </robot>)";
    EXPECT_TRUE(is_failure(xml_string));
  }
  {  // invalid c4
    std::string xml_string =
        R"(<robot name="abb_irb2400">
             <group_opw group="manipulator" a1="0.1" a2="-0.135" b="0" c1="0.615" c2="0.705" c3="0.755" c4="asdfa" offsets="0.0 0.0 -1.570796 0.0 0.0 0.0" sign_corrections="1 1 1 -1 1 1"/>
           </robot>)";
    EXPECT_TRUE(is_failure(xml_string));
  }
  {  // invalid offset
    std::string xml_string =
        R"(<robot name="abb_irb2400">
             <group_opw group="manipulator" a1="0.1" a2="-0.135" b="0" c1="0.615" c2="0.705" c3="0.755" c4="0.085" offsets="avd 0.0 -1.570796 0.0 0.0 0.0" sign_corrections="1 1 1 -1 1 1"/>
           </robot>)";
    EXPECT_TRUE(is_failure(xml_string));
  }
  {  // invalid sign_corrections
    std::string xml_string =
        R"(<robot name="abb_irb2400">
             <group_opw group="manipulator" a1="0.1" a2="-0.135" b="0" c1="0.615" c2="0.705" c3="0.755" c4="0.085" offsets="0.0 0.0 -1.570796 0.0 0.0 0.0" sign_corrections="a 1 1 -1 1 1"/>
           </robot>)";
    EXPECT_TRUE(is_failure(xml_string));
  }
  {  // invalid sign_corrections
    std::string xml_string =
        R"(<robot name="abb_irb2400">
             <group_opw group="manipulator" a1="0.1" a2="-0.135" b="0" c1="0.615" c2="0.705" c3="0.755" c4="0.085" offsets="0.0 0.0 -1.570796 0.0 0.0 0.0" sign_corrections="5 1 1 -1 1 1"/>
           </robot>)";
    EXPECT_TRUE(is_failure(xml_string));
  }
}

TEST(TesseractSRDFUnit, SRDFChainGroupUnit)  // NOLINT
{
  using namespace tesseract_scene_graph;
  using namespace tesseract_srdf;

  SceneGraph::Ptr g = getABBSceneGraph();

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
  EXPECT_EQ(group_names[0], "manipulator");
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
  }
  {  // missing chains
    std::string str = R"(<robot name="abb_irb2400">
                           <group name="manipulator"/>
                         </robot>)";
    EXPECT_TRUE(is_failure(str));
  }
  {  // missing chain base_link
    std::string str = R"(<robot name="abb_irb2400">
                           <group name="manipulator">
                             <chain tip_link="tool0" />
                           </group>
                         </robot>)";
    EXPECT_TRUE(is_failure(str));
  }
  {  // missing chain tip_link
    std::string str = R"(<robot name="abb_irb2400">
                           <group name="manipulator">
                             <chain base_link="base_link" />
                           </group>
                         </robot>)";
    EXPECT_TRUE(is_failure(str));
  }
  {  // invalid chain base_link
    std::string str = R"(<robot name="abb_irb2400">
                           <group name="manipulator">
                             <chain base_link="missing_link" tip_link="tool0" />
                           </group>
                         </robot>)";
    EXPECT_TRUE(is_failure(str));
  }
  {  // invalid chain tip_link
    std::string str = R"(<robot name="abb_irb2400">
                           <group name="manipulator">
                             <chain base_link="base_link" tip_link="missing_link" />
                           </group>
                         </robot>)";
    EXPECT_TRUE(is_failure(str));
  }
}

TEST(TesseractSRDFUnit, SRDFJointGroupUnit)  // NOLINT
{
  using namespace tesseract_scene_graph;
  using namespace tesseract_srdf;

  SceneGraph::Ptr g = getABBSceneGraph();

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
  EXPECT_EQ(group_names[0], "manipulator");
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
                             <joint/>
                           </group>
                         </robot>)";

    EXPECT_TRUE(is_failure(str));
  }
}

TEST(TesseractSRDFUnit, SRDFLinkGroupUnit)  // NOLINT
{
  using namespace tesseract_scene_graph;
  using namespace tesseract_srdf;

  SceneGraph::Ptr g = getABBSceneGraph();

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
  EXPECT_EQ(group_names[0], "manipulator");
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
  }
}

TEST(TesseractSRDFUnit, LoadSRDFREPKinematicsUnit)  // NOLINT
{
  using namespace tesseract_scene_graph;
  using namespace tesseract_srdf;

  SceneGraph::Ptr g = getABBSceneGraph();

  std::string xml_string =
      R"(<robot name="abb_irb2400">
           <group_rep group="gantry">
             <manipulator group="manipulator" ik_solver="OPWInvKin" reach="2.3"/>
             <positioner group="positioner" fk_solver="KDLFwdKin">
               <joint name="axis_1" resolution="0.1"/>
               <joint name="axis_2" resolution="0.2"/>
             </positioner>
           </group_rep>
         </robot>)";
  tinyxml2::XMLDocument xml_doc;
  EXPECT_TRUE(xml_doc.Parse(xml_string.c_str()) == tinyxml2::XML_SUCCESS);

  tinyxml2::XMLElement* element = xml_doc.FirstChildElement("robot");
  EXPECT_TRUE(element != nullptr);

  GroupREPKinematics rep_groups = parseGroupREPKinematics(*g, element, std::array<int, 3>({ 1, 0, 0 }));
  EXPECT_EQ(rep_groups.size(), 1);

  REPKinematicParameters params = rep_groups["gantry"];
  EXPECT_TRUE(params.solver_name.empty());
  EXPECT_EQ(params.manipulator_group, "manipulator");
  EXPECT_EQ(params.manipulator_ik_solver, "OPWInvKin");
  EXPECT_DOUBLE_EQ(params.manipulator_reach, 2.3);
  EXPECT_EQ(params.positioner_group, "positioner");
  EXPECT_EQ(params.positioner_fk_solver, "KDLFwdKin");
  EXPECT_EQ(params.positioner_sample_resolution.size(), 2);
  EXPECT_TRUE(params.positioner_sample_resolution.find("axis_1") != params.positioner_sample_resolution.end());
  EXPECT_DOUBLE_EQ(params.positioner_sample_resolution["axis_1"], 0.1);
  EXPECT_TRUE(params.positioner_sample_resolution.find("axis_2") != params.positioner_sample_resolution.end());
  EXPECT_DOUBLE_EQ(params.positioner_sample_resolution["axis_2"], 0.2);

  {  // Test provided name
    std::string xml_string =
        R"(<robot name="abb_irb2400">
             <group_rep group="gantry" solver_name="REPSolver1">
               <manipulator group="manipulator" ik_solver="OPWInvKin" reach="2.3"/>
               <positioner group="positioner" fk_solver="KDLFwdKin">
                 <joint name="axis_1" resolution="0.1"/>
                 <joint name="axis_2" resolution="0.2"/>
               </positioner>
             </group_rep>
           </robot>)";
    tinyxml2::XMLDocument xml_doc;
    EXPECT_TRUE(xml_doc.Parse(xml_string.c_str()) == tinyxml2::XML_SUCCESS);

    tinyxml2::XMLElement* element = xml_doc.FirstChildElement("robot");
    EXPECT_TRUE(element != nullptr);

    GroupREPKinematics rep_groups = parseGroupREPKinematics(*g, element, std::array<int, 3>({ 1, 0, 0 }));
    EXPECT_EQ(rep_groups.size(), 1);

    REPKinematicParameters params = rep_groups["gantry"];
    EXPECT_EQ(params.solver_name, "REPSolver1");
    EXPECT_EQ(params.manipulator_group, "manipulator");
    EXPECT_EQ(params.manipulator_ik_solver, "OPWInvKin");
    EXPECT_DOUBLE_EQ(params.manipulator_reach, 2.3);
    EXPECT_EQ(params.positioner_group, "positioner");
    EXPECT_EQ(params.positioner_fk_solver, "KDLFwdKin");
    EXPECT_EQ(params.positioner_sample_resolution.size(), 2);
    EXPECT_TRUE(params.positioner_sample_resolution.find("axis_1") != params.positioner_sample_resolution.end());
    EXPECT_DOUBLE_EQ(params.positioner_sample_resolution["axis_1"], 0.1);
    EXPECT_TRUE(params.positioner_sample_resolution.find("axis_2") != params.positioner_sample_resolution.end());
    EXPECT_DOUBLE_EQ(params.positioner_sample_resolution["axis_2"], 0.2);
  }

  // Now test failures
  auto is_failure = [g](const std::string& xml_string) {
    tinyxml2::XMLDocument xml_doc;
    EXPECT_TRUE(xml_doc.Parse(xml_string.c_str()) == tinyxml2::XML_SUCCESS);

    tinyxml2::XMLElement* element = xml_doc.FirstChildElement("robot");
    EXPECT_TRUE(element != nullptr);

    try
    {
      parseGroupREPKinematics(*g, element, std::array<int, 3>({ 1, 0, 0 }));
    }
    catch (const std::exception& e)
    {
      tesseract_common::printNestedException(e);
      return true;
    }
    return false;
  };

  {  // missing group
    std::string xml_string =
        R"(<robot name="abb_irb2400">
             <group_rep>
               <manipulator group="manipulator" ik_solver="OPWInvKin" reach="2.3"/>
               <positioner group="positioner" fk_solver="KDLFwdKin">
                 <joint name="axis_1" resolution="0.1"/>
                 <joint name="axis_2" resolution="0.2"/>
               </positioner>
             </group_rep>
           </robot>)";

    EXPECT_TRUE(is_failure(xml_string));
  }
  {  // missing manipulator element
    std::string xml_string =
        R"(<robot name="abb_irb2400">
             <group_rep group="gantry">
               <positioner group="positioner" fk_solver="KDLFwdKin">
                 <joint name="axis_1" resolution="0.1"/>
                 <joint name="axis_2" resolution="0.2"/>
               </positioner>
             </group_rep>
           </robot>)";

    EXPECT_TRUE(is_failure(xml_string));
  }
  {  // missing manipulator group
    std::string xml_string =
        R"(<robot name="abb_irb2400">
             <group_rep group="gantry">
               <manipulator ik_solver="OPWInvKin" reach="2.3"/>
               <positioner group="positioner" fk_solver="KDLFwdKin">
                 <joint name="axis_1" resolution="0.1"/>
                 <joint name="axis_2" resolution="0.2"/>
               </positioner>
             </group_rep>
           </robot>)";

    EXPECT_TRUE(is_failure(xml_string));
  }
  {  // missing manipulator ik_solver
    std::string xml_string =
        R"(<robot name="abb_irb2400">
             <group_rep group="gantry">
               <manipulator group="manipulator" reach="2.3"/>
               <positioner group="positioner" fk_solver="KDLFwdKin">
                 <joint name="axis_1" resolution="0.1"/>
                 <joint name="axis_2" resolution="0.2"/>
               </positioner>
             </group_rep>
           </robot>)";

    EXPECT_TRUE(is_failure(xml_string));
  }
  {  // missing manipulator reach
    std::string xml_string =
        R"(<robot name="abb_irb2400">
             <group_rep group="gantry">
               <manipulator group="manipulator" ik_solver="OPWInvKin"/>
               <positioner group="positioner" fk_solver="KDLFwdKin">
                 <joint name="axis_1" resolution="0.1"/>
                 <joint name="axis_2" resolution="0.2"/>
               </positioner>
             </group_rep>
           </robot>)";

    EXPECT_TRUE(is_failure(xml_string));
  }
  {  // missing positioner element
    std::string xml_string =
        R"(<robot name="abb_irb2400">
             <group_rep group="gantry">
               <manipulator group="manipulator" ik_solver="OPWInvKin" reach="2.3"/>
             </group_rep>
           </robot>)";

    EXPECT_TRUE(is_failure(xml_string));
  }
  {  // missing positioner group
    std::string xml_string =
        R"(<robot name="abb_irb2400">
             <group_rep group="gantry">
               <manipulator group="manipulator" ik_solver="OPWInvKin" reach="2.3"/>
               <positioner fk_solver="KDLFwdKin">
                 <joint name="axis_1" resolution="0.1"/>
                 <joint name="axis_2" resolution="0.2"/>
               </positioner>
             </group_rep>
           </robot>)";

    EXPECT_TRUE(is_failure(xml_string));
  }
  {  // missing positioner fk_solver is allowed
    std::string xml_string =
        R"(<robot name="abb_irb2400">
             <group_rep group="gantry">
               <manipulator group="manipulator" ik_solver="OPWInvKin" reach="2.3"/>
               <positioner group="positioner">
                 <joint name="axis_1" resolution="0.1"/>
                 <joint name="axis_2" resolution="0.2"/>
               </positioner>
             </group_rep>
           </robot>)";

    EXPECT_FALSE(is_failure(xml_string));
  }
  {  // missing positioner no joints
    std::string xml_string =
        R"(<robot name="abb_irb2400">
             <group_rep group="gantry">
               <manipulator group="manipulator" ik_solver="OPWInvKin" reach="2.3"/>
               <positioner group="positioner" fk_solver="KDLFwdKin"/>
             </group_rep>
           </robot>)";

    EXPECT_TRUE(is_failure(xml_string));
  }
  {  // missing positioner joint name
    std::string xml_string =
        R"(<robot name="abb_irb2400">
             <group_rep group="gantry">
               <manipulator group="manipulator" ik_solver="OPWInvKin" reach="2.3"/>
               <positioner group="positioner" fk_solver="KDLFwdKin">
                 <joint resolution="0.1"/>
                 <joint name="axis_2" resolution="0.2"/>
               </positioner>
             </group_rep>
           </robot>)";

    EXPECT_TRUE(is_failure(xml_string));
  }
  {  // missing positioner joint resolution
    std::string xml_string =
        R"(<robot name="abb_irb2400">
             <group_rep group="gantry">
               <manipulator group="manipulator" ik_solver="OPWInvKin" reach="2.3"/>
               <positioner group="positioner" fk_solver="KDLFwdKin">
                 <joint name="axis_1"/>
                 <joint name="axis_2" resolution="0.2"/>
               </positioner>
             </group_rep>
           </robot>)";

    EXPECT_TRUE(is_failure(xml_string));
  }
  {  // missing positioner joint but will pass here, but gets caught at a later stage
    std::string xml_string =
        R"(<robot name="abb_irb2400">
             <group_rep group="gantry">
               <manipulator group="manipulator" ik_solver="OPWInvKin" reach="2.3"/>
               <positioner group="positioner" fk_solver="KDLFwdKin">
                 <joint name="axis_1" resolution="0.1"/>
               </positioner>
             </group_rep>
           </robot>)";

    EXPECT_FALSE(is_failure(xml_string));
  }
}

TEST(TesseractSRDFUnit, LoadSRDFROPKinematicsUnit)  // NOLINT
{
  using namespace tesseract_scene_graph;
  using namespace tesseract_srdf;

  SceneGraph::Ptr g = getABBSceneGraph();

  std::string xml_string =
      R"(<robot name="abb_irb2400">
           <group_rop group="gantry">
             <manipulator group="manipulator" ik_solver="OPWInvKin" reach="2.3"/>
             <positioner group="positioner" fk_solver="KDLFwdKin">
               <joint name="axis_1" resolution="0.1"/>
               <joint name="axis_2" resolution="0.2"/>
             </positioner>
           </group_rop>
         </robot>)";
  tinyxml2::XMLDocument xml_doc;
  EXPECT_TRUE(xml_doc.Parse(xml_string.c_str()) == tinyxml2::XML_SUCCESS);

  tinyxml2::XMLElement* element = xml_doc.FirstChildElement("robot");
  EXPECT_TRUE(element != nullptr);

  GroupROPKinematics rop_groups = parseGroupROPKinematics(*g, element, std::array<int, 3>({ 1, 0, 0 }));
  EXPECT_EQ(rop_groups.size(), 1);

  ROPKinematicParameters params = rop_groups["gantry"];
  EXPECT_TRUE(params.solver_name.empty());
  EXPECT_EQ(params.manipulator_group, "manipulator");
  EXPECT_EQ(params.manipulator_ik_solver, "OPWInvKin");
  EXPECT_DOUBLE_EQ(params.manipulator_reach, 2.3);
  EXPECT_EQ(params.positioner_group, "positioner");
  EXPECT_EQ(params.positioner_fk_solver, "KDLFwdKin");
  EXPECT_EQ(params.positioner_sample_resolution.size(), 2);
  EXPECT_TRUE(params.positioner_sample_resolution.find("axis_1") != params.positioner_sample_resolution.end());
  EXPECT_DOUBLE_EQ(params.positioner_sample_resolution["axis_1"], 0.1);
  EXPECT_TRUE(params.positioner_sample_resolution.find("axis_2") != params.positioner_sample_resolution.end());
  EXPECT_DOUBLE_EQ(params.positioner_sample_resolution["axis_2"], 0.2);

  {  // Now test with name provided
    std::string xml_string =
        R"(<robot name="abb_irb2400">
             <group_rop group="gantry" solver_name="ROPSolver2">
               <manipulator group="manipulator" ik_solver="OPWInvKin" reach="2.3"/>
               <positioner group="positioner" fk_solver="KDLFwdKin">
                 <joint name="axis_1" resolution="0.1"/>
                 <joint name="axis_2" resolution="0.2"/>
               </positioner>
             </group_rop>
           </robot>)";
    tinyxml2::XMLDocument xml_doc;
    EXPECT_TRUE(xml_doc.Parse(xml_string.c_str()) == tinyxml2::XML_SUCCESS);

    tinyxml2::XMLElement* element = xml_doc.FirstChildElement("robot");
    EXPECT_TRUE(element != nullptr);

    GroupROPKinematics rop_groups = parseGroupROPKinematics(*g, element, std::array<int, 3>({ 1, 0, 0 }));
    EXPECT_EQ(rop_groups.size(), 1);

    ROPKinematicParameters params = rop_groups["gantry"];
    EXPECT_EQ(params.solver_name, "ROPSolver2");
    EXPECT_EQ(params.manipulator_group, "manipulator");
    EXPECT_EQ(params.manipulator_ik_solver, "OPWInvKin");
    EXPECT_DOUBLE_EQ(params.manipulator_reach, 2.3);
    EXPECT_EQ(params.positioner_group, "positioner");
    EXPECT_EQ(params.positioner_fk_solver, "KDLFwdKin");
    EXPECT_EQ(params.positioner_sample_resolution.size(), 2);
    EXPECT_TRUE(params.positioner_sample_resolution.find("axis_1") != params.positioner_sample_resolution.end());
    EXPECT_DOUBLE_EQ(params.positioner_sample_resolution["axis_1"], 0.1);
    EXPECT_TRUE(params.positioner_sample_resolution.find("axis_2") != params.positioner_sample_resolution.end());
    EXPECT_DOUBLE_EQ(params.positioner_sample_resolution["axis_2"], 0.2);
  }

  // Now test failures
  auto is_failure = [g](const std::string& xml_string) {
    tinyxml2::XMLDocument xml_doc;
    EXPECT_TRUE(xml_doc.Parse(xml_string.c_str()) == tinyxml2::XML_SUCCESS);

    tinyxml2::XMLElement* element = xml_doc.FirstChildElement("robot");
    EXPECT_TRUE(element != nullptr);

    try
    {
      parseGroupROPKinematics(*g, element, std::array<int, 3>({ 1, 0, 0 }));
    }
    catch (const std::exception& e)
    {
      tesseract_common::printNestedException(e);
      return true;
    }
    return false;
  };

  {  // missing group
    std::string xml_string =
        R"(<robot name="abb_irb2400">
             <group_rop>
               <manipulator group="manipulator" ik_solver="OPWInvKin" reach="2.3"/>
               <positioner group="positioner" fk_solver="KDLFwdKin">
                 <joint name="axis_1" resolution="0.1"/>
                 <joint name="axis_2" resolution="0.2"/>
               </positioner>
             </group_rop>
           </robot>)";

    EXPECT_TRUE(is_failure(xml_string));
  }
  {  // missing manipulator element
    std::string xml_string =
        R"(<robot name="abb_irb2400">
             <group_rop group="gantry">
               <positioner group="positioner" fk_solver="KDLFwdKin">
                 <joint name="axis_1" resolution="0.1"/>
                 <joint name="axis_2" resolution="0.2"/>
               </positioner>
             </group_rop>
           </robot>)";

    EXPECT_TRUE(is_failure(xml_string));
  }
  {  // missing manipulator group
    std::string xml_string =
        R"(<robot name="abb_irb2400">
             <group_rop group="gantry">
               <manipulator ik_solver="OPWInvKin" reach="2.3"/>
               <positioner group="positioner" fk_solver="KDLFwdKin">
                 <joint name="axis_1" resolution="0.1"/>
                 <joint name="axis_2" resolution="0.2"/>
               </positioner>
             </group_rop>
           </robot>)";

    EXPECT_TRUE(is_failure(xml_string));
  }
  {  // missing manipulator ik_solver
    std::string xml_string =
        R"(<robot name="abb_irb2400">
             <group_rop group="gantry">
               <manipulator group="manipulator" reach="2.3"/>
               <positioner group="positioner" fk_solver="KDLFwdKin">
                 <joint name="axis_1" resolution="0.1"/>
                 <joint name="axis_2" resolution="0.2"/>
               </positioner>
             </group_rop>
           </robot>)";

    EXPECT_TRUE(is_failure(xml_string));
  }
  {  // missing manipulator reach
    std::string xml_string =
        R"(<robot name="abb_irb2400">
             <group_rop group="gantry">
               <manipulator group="manipulator" ik_solver="OPWInvKin"/>
               <positioner group="positioner" fk_solver="KDLFwdKin">
                 <joint name="axis_1" resolution="0.1"/>
                 <joint name="axis_2" resolution="0.2"/>
               </positioner>
             </group_rop>
           </robot>)";

    EXPECT_TRUE(is_failure(xml_string));
  }
  {  // missing positioner element
    std::string xml_string =
        R"(<robot name="abb_irb2400">
             <group_rop group="gantry">
               <manipulator group="manipulator" ik_solver="OPWInvKin" reach="2.3"/>
             </group_rop>
           </robot>)";

    EXPECT_TRUE(is_failure(xml_string));
  }
  {  // missing positioner group
    std::string xml_string =
        R"(<robot name="abb_irb2400">
             <group_rop group="gantry">
               <manipulator group="manipulator" ik_solver="OPWInvKin" reach="2.3"/>
               <positioner fk_solver="KDLFwdKin">
                 <joint name="axis_1" resolution="0.1"/>
                 <joint name="axis_2" resolution="0.2"/>
               </positioner>
             </group_rop>
           </robot>)";

    EXPECT_TRUE(is_failure(xml_string));
  }
  {  // missing positioner fk_solver is allowed
    std::string xml_string =
        R"(<robot name="abb_irb2400">
             <group_rop group="gantry">
               <manipulator group="manipulator" ik_solver="OPWInvKin" reach="2.3"/>
               <positioner group="positioner">
                 <joint name="axis_1" resolution="0.1"/>
                 <joint name="axis_2" resolution="0.2"/>
               </positioner>
             </group_rop>
           </robot>)";

    EXPECT_FALSE(is_failure(xml_string));
  }
  {  // missing positioner no joints
    std::string xml_string =
        R"(<robot name="abb_irb2400">
             <group_rop group="gantry">
               <manipulator group="manipulator" ik_solver="OPWInvKin" reach="2.3"/>
               <positioner group="positioner" fk_solver="KDLFwdKin"/>
             </group_rop>
           </robot>)";

    EXPECT_TRUE(is_failure(xml_string));
  }
  {  // missing positioner joint name
    std::string xml_string =
        R"(<robot name="abb_irb2400">
             <group_rop group="gantry">
               <manipulator group="manipulator" ik_solver="OPWInvKin" reach="2.3"/>
               <positioner group="positioner" fk_solver="KDLFwdKin">
                 <joint resolution="0.1"/>
                 <joint name="axis_2" resolution="0.2"/>
               </positioner>
             </group_rop>
           </robot>)";

    EXPECT_TRUE(is_failure(xml_string));
  }
  {  // missing positioner joint resolution
    std::string xml_string =
        R"(<robot name="abb_irb2400">
             <group_rop group="gantry">
               <manipulator group="manipulator" ik_solver="OPWInvKin" reach="2.3"/>
               <positioner group="positioner" fk_solver="KDLFwdKin">
                 <joint name="axis_1"/>
                 <joint name="axis_2" resolution="0.2"/>
               </positioner>
             </group_rop>
           </robot>)";

    EXPECT_TRUE(is_failure(xml_string));
  }
  {  // missing positioner joint but will pass here, but gets caught at a later stage
    std::string xml_string =
        R"(<robot name="abb_irb2400">
             <group_rop group="gantry">
               <manipulator group="manipulator" ik_solver="OPWInvKin" reach="2.3"/>
               <positioner group="positioner" fk_solver="KDLFwdKin">
                 <joint name="axis_2" resolution="0.2"/>
               </positioner>
             </group_rop>
           </robot>)";

    EXPECT_FALSE(is_failure(xml_string));
  }
}

TEST(TesseractSRDFUnit, LoadSRDFGroupStatesUnit)  // NOLINT
{
  using namespace tesseract_scene_graph;
  using namespace tesseract_srdf;

  SceneGraph::Ptr g = getABBSceneGraph();

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
  }
  {  // missing group
    std::string xml_string =
        R"(<robot name="abb_irb2400">
             <group_state name="all-zeros">
               <joint name="joint_1" value="0"/>
             </group_state>
           </robot>)";
    EXPECT_TRUE(is_failure(xml_string));
  }
  {  // invalid group
    std::string xml_string =
        R"(<robot name="abb_irb2400">
             <group_state name="all-zeros" group="missing_group">
               <joint name="joint_1" value="0"/>
             </group_state>
           </robot>)";
    EXPECT_TRUE(is_failure(xml_string));
  }
  {  // no joints
    std::string xml_string =
        R"(<robot name="abb_irb2400">
             <group_state name="all-zeros" group="manipulator"/>
           </robot>)";
    EXPECT_TRUE(is_failure(xml_string));
  }
  {  // missing joint name
    std::string xml_string =
        R"(<robot name="abb_irb2400">
             <group_state name="all-zeros" group="manipulator">
               <joint value="0"/>
             </group_state>
           </robot>)";
    EXPECT_TRUE(is_failure(xml_string));
  }
  {  // missing joint value
    std::string xml_string =
        R"(<robot name="abb_irb2400">
             <group_state name="all-zeros" group="manipulator">
               <joint name="joint_1"/>
             </group_state>
           </robot>)";
    EXPECT_TRUE(is_failure(xml_string));
  }
  {  // invalid joint value
    std::string xml_string =
        R"(<robot name="abb_irb2400">
             <group_state name="all-zeros" group="manipulator">
               <joint name="joint_1" value="abc"/>
             </group_state>
           </robot>)";
    EXPECT_TRUE(is_failure(xml_string));
  }
}

TEST(TesseractSRDFUnit, SRDFGroupTCPsUnit)  // NOLINT
{
  using namespace tesseract_scene_graph;
  using namespace tesseract_srdf;

  SceneGraph::Ptr g = getABBSceneGraph();

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
  }
  {  // missing tcp element
    std::string str = R"(<robot name="abb_irb2400">
                           <group_tcps group="manipulator"/>
                         </robot>)";
    EXPECT_TRUE(is_failure(str));
  }
  {  // missing tcp name
    std::string str = R"(<robot name="abb_irb2400">
                           <group_tcps group="manipulator">
                             <tcp xyz="1 .1 1" rpy="0 1.57 0" />
                           </group_tcps>
                         </robot>)";
    EXPECT_TRUE(is_failure(str));
  }
  {  // missing tcp xyz
    std::string str = R"(<robot name="abb_irb2400">
                           <group_tcps group="manipulator">
                             <tcp name="laser" rpy="0 1.57 0" />
                           </group_tcps>
                         </robot>)";
    EXPECT_TRUE(is_failure(str));
  }
  {  // missing tcp orientation
    std::string str = R"(<robot name="abb_irb2400">
                           <group_tcps group="manipulator">
                             <tcp name="laser" xyz="1 .1 1" />
                           </group_tcps>
                         </robot>)";
    EXPECT_TRUE(is_failure(str));
  }
  {  // invalid tcp xyz
    std::string str = R"(<robot name="abb_irb2400">
                           <group_tcps group="manipulator">
                             <tcp name="laser" xyz="a .1 1" rpy="0 1.57 0" />
                           </group_tcps>
                         </robot>)";
    EXPECT_TRUE(is_failure(str));
  }
  {  // invalid orientation
    std::string str = R"(<robot name="abb_irb2400">
                           <group_tcps group="manipulator">
                             <tcp name="laser" xyz="1 .1 1" rpy="a 1.57 0" />
                           </group_tcps>
                         </robot>)";
    EXPECT_TRUE(is_failure(str));
  }
  {  // invalid orientation
    std::string str = R"(<robot name="abb_irb2400">
                           <group_tcps group="manipulator">
                             <tcp name="laser" xyz="1 .1 1" wxyz="a 1.57 0 0" />
                           </group_tcps>
                         </robot>)";
    EXPECT_TRUE(is_failure(str));
  }
}

TEST(TesseractSRDFUnit, SRDFCollisionMarginsUnit)  // NOLINT
{
  using namespace tesseract_scene_graph;
  using namespace tesseract_srdf;

  SceneGraph::Ptr g = getABBSceneGraph();

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
  }

  {  // missing pair link1
    std::string str = R"(<robot name="abb_irb2400">
                           <collision_margins default_margin="0.025">
                             <pair_margin link2="link_5" margin="0.01"/>
                             <pair_margin link1="link_5" link2="link_4" margin="0.015"/>
                           </collision_margins>
                         </robot>)";
    EXPECT_TRUE(is_failure(str));
  }

  {  // missing pair link2
    std::string str = R"(<robot name="abb_irb2400">
                           <collision_margins default_margin="0.025">
                             <pair_margin link1="link_6" link2="link_5" margin="0.01"/>
                             <pair_margin link1="link_5" margin="0.015"/>
                           </collision_margins>
                         </robot>)";
    EXPECT_TRUE(is_failure(str));
  }

  {  // missing pair margin
    std::string str = R"(<robot name="abb_irb2400">
                           <collision_margins default_margin="0.025">
                             <pair_margin link1="link_6" link2="link_5" margin="0.01"/>
                             <pair_margin link1="link_5" link2="link_4"/>
                           </collision_margins>
                         </robot>)";
    EXPECT_TRUE(is_failure(str));
  }

  {  // empty default margin
    std::string str = R"(<robot name="abb_irb2400">
                           <collision_margins default_margin="">
                             <pair_margin link1="link_6" link2="link_5" margin="0.01"/>
                             <pair_margin link1="link_5" link2="link_4" margin="0.01"/>
                           </collision_margins>
                         </robot>)";
    EXPECT_TRUE(is_failure(str));
  }

  {  // empty pair margin
    std::string str = R"(<robot name="abb_irb2400">
                           <collision_margins default_margin="0.025">
                             <pair_margin link1="link_6" link2="link_5" margin=""/>
                             <pair_margin link1="link_5" link2="link_4" margin="0.01"/>
                           </collision_margins>
                         </robot>)";
    EXPECT_TRUE(is_failure(str));
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
