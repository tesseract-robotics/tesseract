#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <iostream>
#include <fstream>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_scene_graph/graph.h>
#include <tesseract_scene_graph/utils.h>
#include <tesseract_scene_graph/resource_locator.h>
#include <tesseract_scene_graph/srdf/disabled_collisions.h>
#include <tesseract_scene_graph/srdf/group_opw_kinematics.h>
#include <tesseract_scene_graph/srdf/group_rep_kinematics.h>
#include <tesseract_scene_graph/srdf/group_rop_kinematics.h>
#include <tesseract_scene_graph/srdf/group_states.h>
#include <tesseract_scene_graph/srdf/group_tool_center_points.h>
#include <tesseract_scene_graph/srdf/groups.h>

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

tesseract_scene_graph::SceneGraph::Ptr getABBSceneGraph()
{
  using namespace tesseract_scene_graph;
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

  g->addLink(std::move(base_link));
  g->addLink(std::move(link_1));
  g->addLink(std::move(link_2));
  g->addLink(std::move(link_3));
  g->addLink(std::move(link_4));
  g->addLink(std::move(link_5));
  g->addLink(std::move(link_6));
  g->addLink(std::move(tool0));

  Joint joint_1("joint_1");
  joint_1.parent_link_name = "base_link";
  joint_1.child_link_name = "link_1";
  joint_1.type = JointType::FIXED;
  g->addJoint(std::move(joint_1));

  Joint joint_2("joint_2");
  joint_2.parent_link_name = "link_1";
  joint_2.child_link_name = "link_2";
  joint_2.type = JointType::REVOLUTE;
  g->addJoint(std::move(joint_2));

  Joint joint_3("joint_3");
  joint_3.parent_to_joint_origin_transform.translation()(0) = 1.25;
  joint_3.parent_link_name = "link_2";
  joint_3.child_link_name = "link_3";
  joint_3.type = JointType::REVOLUTE;
  g->addJoint(std::move(joint_3));

  Joint joint_4("joint_4");
  joint_4.parent_to_joint_origin_transform.translation()(0) = 1.25;
  joint_4.parent_link_name = "link_3";
  joint_4.child_link_name = "link_4";
  joint_4.type = JointType::REVOLUTE;
  g->addJoint(std::move(joint_4));

  Joint joint_5("joint_5");
  joint_5.parent_to_joint_origin_transform.translation()(1) = 1.25;
  joint_5.parent_link_name = "link_4";
  joint_5.child_link_name = "link_5";
  joint_5.type = JointType::REVOLUTE;
  g->addJoint(std::move(joint_5));

  Joint joint_6("joint_6");
  joint_6.parent_to_joint_origin_transform.translation()(1) = 1.25;
  joint_6.parent_link_name = "link_5";
  joint_6.child_link_name = "link_6";
  joint_6.type = JointType::REVOLUTE;
  g->addJoint(std::move(joint_6));

  Joint joint_tool0("joint_tool0");
  joint_tool0.parent_link_name = "link_6";
  joint_tool0.child_link_name = "tool0";
  joint_tool0.type = JointType::FIXED;
  g->addJoint(std::move(joint_tool0));

  return g;
}

TEST(TesseractSceneGraphSRDFUnit, LoadSRDFUnit)  // NOLINT
{
  using namespace tesseract_scene_graph;

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

  g.addLink(std::move(base_link));
  g.addLink(std::move(link_1));
  g.addLink(std::move(link_2));
  g.addLink(std::move(link_3));
  g.addLink(std::move(link_4));
  g.addLink(std::move(link_5));
  g.addLink(std::move(link_6));
  g.addLink(std::move(link_7));
  g.addLink(std::move(tool0));

  Joint joint_1("joint_a1");
  joint_1.parent_link_name = "base_link";
  joint_1.child_link_name = "link_1";
  joint_1.type = JointType::FIXED;
  g.addJoint(std::move(joint_1));

  Joint joint_2("joint_a2");
  joint_2.parent_link_name = "link_1";
  joint_2.child_link_name = "link_2";
  joint_2.type = JointType::REVOLUTE;
  g.addJoint(std::move(joint_2));

  Joint joint_3("joint_a3");
  joint_3.parent_to_joint_origin_transform.translation()(0) = 1.25;
  joint_3.parent_link_name = "link_2";
  joint_3.child_link_name = "link_3";
  joint_3.type = JointType::REVOLUTE;
  g.addJoint(std::move(joint_3));

  Joint joint_4("joint_a4");
  joint_4.parent_to_joint_origin_transform.translation()(0) = 1.25;
  joint_4.parent_link_name = "link_3";
  joint_4.child_link_name = "link_4";
  joint_4.type = JointType::REVOLUTE;
  g.addJoint(std::move(joint_4));

  Joint joint_5("joint_a5");
  joint_5.parent_to_joint_origin_transform.translation()(1) = 1.25;
  joint_5.parent_link_name = "link_4";
  joint_5.child_link_name = "link_5";
  joint_5.type = JointType::REVOLUTE;
  g.addJoint(std::move(joint_5));

  Joint joint_6("joint_a6");
  joint_6.parent_to_joint_origin_transform.translation()(1) = 1.25;
  joint_6.parent_link_name = "link_5";
  joint_6.child_link_name = "link_6";
  joint_6.type = JointType::REVOLUTE;
  g.addJoint(std::move(joint_6));

  Joint joint_7("joint_a7");
  joint_7.parent_to_joint_origin_transform.translation()(1) = 1.25;
  joint_7.parent_link_name = "link_6";
  joint_7.child_link_name = "link_7";
  joint_7.type = JointType::REVOLUTE;
  g.addJoint(std::move(joint_7));

  Joint joint_tool0("joint_tool0");
  joint_tool0.parent_link_name = "link_7";
  joint_tool0.child_link_name = "tool0";
  joint_tool0.type = JointType::FIXED;
  g.addJoint(std::move(joint_tool0));

  SRDFModel srdf;
  EXPECT_TRUE(srdf.initFile(g, srdf_file));

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

TEST(TesseractSceneGraphSRDFUnit, LoadSRDFOPWKinematicsUnit)  // NOLINT
{
  using namespace tesseract_scene_graph;
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
}

TEST(TesseractSceneGraphSRDFUnit, SRDFChainGroupUnit)  // NOLINT
{
  using namespace tesseract_scene_graph;
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
}

TEST(TesseractSceneGraphSRDFUnit, SRDFJointGroupUnit)  // NOLINT
{
  using namespace tesseract_scene_graph;
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
}

TEST(TesseractSceneGraphSRDFUnit, SRDFLinkGroupUnit)  // NOLINT
{
  using namespace tesseract_scene_graph;
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
}

TEST(TesseractSceneGraphSRDFUnit, LoadSRDFREPKinematicsUnit)  // NOLINT
{
  using namespace tesseract_scene_graph;
  SceneGraph::Ptr g = getABBSceneGraph();

  std::string xml_string =
      R"(<robot name="abb_irb2400">
           <group_rep group="gantry">
             <manipulator group="manipulator" ik_solver="OPWInvKin" reach="2.3"/>
             <positioner group="positioner" fk_solver="KDLFwdKin">
               <joint name="axis_1" resolution="0.1"/>
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
  EXPECT_EQ(params.manipulator_group, "manipulator");
  EXPECT_EQ(params.manipulator_ik_solver, "OPWInvKin");
  EXPECT_DOUBLE_EQ(params.manipulator_reach, 2.3);
  EXPECT_EQ(params.positioner_group, "positioner");
  EXPECT_EQ(params.positioner_fk_solver, "KDLFwdKin");
  EXPECT_EQ(params.positioner_sample_resolution.size(), 1);
  EXPECT_TRUE(params.positioner_sample_resolution.find("axis_1") != params.positioner_sample_resolution.end());
  EXPECT_DOUBLE_EQ(params.positioner_sample_resolution["axis_1"], 0.1);
}

TEST(TesseractSceneGraphSRDFUnit, LoadSRDFROPKinematicsUnit)  // NOLINT
{
  using namespace tesseract_scene_graph;
  SceneGraph::Ptr g = getABBSceneGraph();

  std::string xml_string =
      R"(<robot name="abb_irb2400">
           <group_rop group="gantry">
             <manipulator group="manipulator" ik_solver="OPWInvKin" reach="2.3"/>
             <positioner group="positioner" fk_solver="KDLFwdKin">
               <joint name="axis_1" resolution="0.1"/>
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
  EXPECT_EQ(params.manipulator_group, "manipulator");
  EXPECT_EQ(params.manipulator_ik_solver, "OPWInvKin");
  EXPECT_DOUBLE_EQ(params.manipulator_reach, 2.3);
  EXPECT_EQ(params.positioner_group, "positioner");
  EXPECT_EQ(params.positioner_fk_solver, "KDLFwdKin");
  EXPECT_EQ(params.positioner_sample_resolution.size(), 1);
  EXPECT_TRUE(params.positioner_sample_resolution.find("axis_1") != params.positioner_sample_resolution.end());
  EXPECT_DOUBLE_EQ(params.positioner_sample_resolution["axis_1"], 0.1);
}

TEST(TesseractSceneGraphSRDFUnit, LoadSRDFGroupStatesUnit)  // NOLINT
{
  using namespace tesseract_scene_graph;
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
}

TEST(TesseractSceneGraphSRDFUnit, SRDFGroupTCPsUnit)  // NOLINT
{
  using namespace tesseract_scene_graph;
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
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
