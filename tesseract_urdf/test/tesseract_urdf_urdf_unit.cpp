#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <Eigen/Geometry>
#include <tesseract_common/utils.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_urdf/urdf_parser.h>
#include "tesseract_urdf_common_unit.h"

TEST(TesseractURDFUnit, parse_urdf)  // NOLINT
{
  tesseract_common::SimpleResourceLocator resource_locator(locateResource);
  {
    std::string str =
        R"(<robot name="test" extra="0 0 0">
             <joint name="j1" type="fixed">
               <parent link="l1"/>
               <child link="l2"/>
               <origin xyz="0 0 0" rpy="0 0 0"/>
               <dynamics damping="87.098" friction="3.1290"/>
               <limit lower="12.34" upper="22.999" effort="99.0" velocity="23.0"/>
               <safety_controller soft_lower_limit="8.765" soft_upper_limit="9.003" k_position="7.0034" k_velocity="9.998"/>
               <calibration rising="8.654" falling="0.0445"/>
               <mimic joint="j2" multiplier="9.87" offset="0.098"/>
             </joint>
             <link name="l1"/>
             <link name="l2"/>
           </robot>)";
    tesseract_scene_graph::SceneGraph::Ptr sg = tesseract_urdf::parseURDFString(str, resource_locator);
    EXPECT_TRUE(sg != nullptr);
    EXPECT_TRUE(sg->getName() == "test");
    EXPECT_TRUE(sg->isTree());
    EXPECT_TRUE(sg->isAcyclic());
    EXPECT_TRUE(sg->getJoints().size() == 1);
    EXPECT_TRUE(sg->getLinks().size() == 2);
  }

  {
    std::string str =
        R"(<robot name="test" extra="0 0 0">
             <joint name="j1" type="fixed">
               <parent link="l1"/>
               <child link="l2"/>
               <origin xyz="0 0 0" rpy="0 0 0"/>
               <dynamics damping="87.098" friction="3.1290"/>
               <limit lower="12.34" upper="22.999" effort="99.0" velocity="23.0"/>
               <safety_controller soft_lower_limit="8.765" soft_upper_limit="9.003" k_position="7.0034" k_velocity="9.998"/>
               <calibration rising="8.654" falling="0.0445"/>
               <mimic joint="j2" multiplier="9.87" offset="0.098"/>
             </joint>
             <link name="l1"/>
             <link name="l2"/>
           </robot>)";
    tesseract_scene_graph::SceneGraph::Ptr sg = tesseract_urdf::parseURDFString(str, resource_locator);
    EXPECT_TRUE(sg != nullptr);
    EXPECT_TRUE(sg->getName() == "test");
    EXPECT_TRUE(sg->isTree());
    EXPECT_TRUE(sg->isAcyclic());
    EXPECT_TRUE(sg->getJoints().size() == 1);
    EXPECT_TRUE(sg->getLinks().size() == 2);
  }

  {
    std::string str =
        R"(<robot name="test">
             <joint name="j1" type="fixed">
               <parent link="l1"/>
               <child link="l2"/>
               <origin xyz="0 0 0" rpy="0 0 0"/>
               <dynamics damping="87.098" friction="3.1290"/>
               <limit lower="12.34" upper="22.999" effort="99.0" velocity="23.0"/>
               <safety_controller soft_lower_limit="8.765" soft_upper_limit="9.003" k_position="7.0034" k_velocity="9.998"/>
               <calibration rising="8.654" falling="0.0445"/>
               <mimic joint="j2" multiplier="9.87" offset="0.098"/>
             </joint>
             <joint name="j2" type="fixed">
               <parent link="l1"/>
               <child link="l2"/>
             </joint>
             <link name="l1"/>
             <link name="l2"/>
           </robot>)";
    EXPECT_ANY_THROW(tesseract_urdf::parseURDFString(str, resource_locator));  // NOLINT
  }

  {
    std::string str =
        R"(<robot name="test">
             <joint name="j1" type="fixed">
               <parent link="l1"/>
               <child link="l2"/>
               <origin xyz="0 0 0" rpy="0 0 0"/>
               <dynamics damping="87.098" friction="3.1290"/>
               <limit lower="12.34" upper="22.999" effort="99.0" velocity="23.0"/>
               <safety_controller soft_lower_limit="8.765" soft_upper_limit="9.003" k_position="7.0034" k_velocity="9.998"/>
               <calibration rising="8.654" falling="0.0445"/>
               <mimic joint="j2" multiplier="9.87" offset="0.098"/>
             </joint>
             <joint name="j2" type="fixed">
               <parent link="l1"/>
               <child link="l2"/>
             </joint>
             <link name="l1"/>
             <link name="l2"/>
           </robot>)";
    EXPECT_ANY_THROW(tesseract_urdf::parseURDFString(str, resource_locator));  // NOLINT
  }

  {
    std::string str =
        R"(<robot name="test">
             <joint name="j1" type="fixed">
               <parent link="l1"/>
               <child link="l2"/>
               <origin xyz="0 0 0" rpy="0 0 0"/>
               <dynamics damping="87.098" friction="3.1290"/>
               <limit lower="12.34" upper="22.999" effort="99.0" velocity="23.0"/>
               <safety_controller soft_lower_limit="8.765" soft_upper_limit="9.003" k_position="7.0034" k_velocity="9.998"/>
               <calibration rising="8.654" falling="0.0445"/>
               <mimic joint="j2" multiplier="9.87" offset="0.098"/>
             </joint>
             <link name="l1"/>
             <link name="l3"/>
           </robot>)";
    EXPECT_ANY_THROW(tesseract_urdf::parseURDFString(str, resource_locator));  // NOLINT
  }

  {
    std::string str =
        R"(<robot name="test">
             <joint name="j1" type="fixed">
               <parent link="l1"/>
               <child link="l2"/>
               <origin xyz="0 0 0" rpy="0 0 0"/>
               <dynamics damping="87.098" friction="3.1290"/>
               <limit lower="12.34" upper="22.999" effort="99.0" velocity="23.0"/>
               <safety_controller soft_lower_limit="8.765" soft_upper_limit="9.003" k_position="7.0034" k_velocity="9.998"/>
               <calibration rising="8.654" falling="0.0445"/>
               <mimic joint="j2" multiplier="9.87" offset="0.098"/>
             </joint>
             <link name="l1"/>
             <link name="l2"/>
             <link name="l3"/>
           </robot>)";
    EXPECT_ANY_THROW(tesseract_urdf::parseURDFString(str, resource_locator));  // NOLINT
  }

  {
    std::string str =
        R"(<robot name="test">
             <joint name="j1" type="fixed">
               <parent link="l1"/>
               <child link="l3"/>
               <origin xyz="0 0 0" rpy="0 0 0"/>
               <dynamics damping="87.098" friction="3.1290"/>
               <limit lower="12.34" upper="22.999" effort="99.0" velocity="23.0"/>
               <safety_controller soft_lower_limit="8.765" soft_upper_limit="9.003" k_position="7.0034" k_velocity="9.998"/>
               <calibration rising="8.654" falling="0.0445"/>
               <mimic joint="j2" multiplier="9.87" offset="0.098"/>
             </joint>
             <link name="l1"/>
             <link name="l2"/>
           </robot>)";
    EXPECT_ANY_THROW(tesseract_urdf::parseURDFString(str, resource_locator));  // NOLINT
  }

  {
    std::string str =
        R"(<robot name="test">
             <joint name="j1" type="fixed">
               <parent link="l1"/>
               <child link="l2"/>
               <origin xyz="0 0 0" rpy="0 0 0"/>
               <dynamics damping="87.098" friction="3.1290"/>
               <limit lower="12.34" upper="22.999" effort="99.0" velocity="23.0"/>
               <safety_controller soft_lower_limit="8.765" soft_upper_limit="9.003" k_position="7.0034" k_velocity="9.998"/>
               <calibration rising="8.654" falling="0.0445"/>
               <mimic joint="j2" multiplier="9.87" offset="0.098"/>
             </joint>
             <link name="l1"/>
             <link name="l2"/>
             <link name="l1"/>
           </robot>)";
    EXPECT_ANY_THROW(tesseract_urdf::parseURDFString(str, resource_locator));  // NOLINT
  }

  {
    std::string str =
        R"(<robot>
             <joint name="j1" type="fixed">
               <parent link="l1"/>
               <child link="l2"/>
               <origin xyz="0 0 0" rpy="0 0 0"/>
               <dynamics damping="87.098" friction="3.1290"/>
               <limit lower="12.34" upper="22.999" effort="99.0" velocity="23.0"/>
               <safety_controller soft_lower_limit="8.765" soft_upper_limit="9.003" k_position="7.0034" k_velocity="9.998"/>
               <calibration rising="8.654" falling="0.0445"/>
               <mimic joint="j2" multiplier="9.87" offset="0.098"/>
             </joint>
             <link name="l1"/>
             <link name="l2"/>
           </robot>)";
    EXPECT_ANY_THROW(tesseract_urdf::parseURDFString(str, resource_locator));  // NOLINT
  }

  {
    std::string str =
        R"(<robot name="test">
             <joint name="j1" type="fixed">
               <parent link="l2"/>
               <child link="l3"/>
               <origin xyz="0 0 0" rpy="0 0 0"/>
               <dynamics damping="87.098" friction="3.1290"/>
               <limit lower="12.34" upper="22.999" effort="99.0" velocity="23.0"/>
               <safety_controller soft_lower_limit="8.765" soft_upper_limit="9.003" k_position="7.0034" k_velocity="9.998"/>
               <calibration rising="8.654" falling="0.0445"/>
               <mimic joint="j2" multiplier="9.87" offset="0.098"/>
             </joint>
             <joint name="j1" type="fixed">
               <parent link="l1"/>
               <child link="l2"/>
             </joint>
             <link name="l1"/>
             <link name="l2"/>
             <link name="l3"/>
           </robot>)";
    EXPECT_ANY_THROW(tesseract_urdf::parseURDFString(str, resource_locator));  // NOLINT
  }

  {
    tesseract_scene_graph::SceneGraph::Ptr sg = tesseract_urdf::parseURDFFile(std::string(TESSERACT_SUPPORT_DIR) + "/ur"
                                                                                                                   "df/"
                                                                                                                   "lbr"
                                                                                                                   "_ii"
                                                                                                                   "wa_"
                                                                                                                   "14_"
                                                                                                                   "r82"
                                                                                                                   "0."
                                                                                                                   "urd"
                                                                                                                   "f",
                                                                              resource_locator);
    EXPECT_TRUE(sg != nullptr);
  }
}

TEST(TesseractURDFUnit, parse_urdf_with_available_materials)  // NOLINT
{
  tesseract_common::SimpleResourceLocator resource_locator(locateResource);
  {
    std::string str =
        R"(<robot name="test" extra="0 0 0">
             <material name="test_material" extra="0 0 0">
               <color rgba="1 .5 .5 1" extra="0 0 0"/>
             </material>
             <joint name="j1" type="fixed">
               <parent link="l1"/>
               <child link="l2"/>
               <origin xyz="0 0 0" rpy="0 0 0"/>
               <dynamics damping="87.098" friction="3.1290"/>
               <limit lower="12.34" upper="22.999" effort="99.0" velocity="23.0"/>
               <safety_controller soft_lower_limit="8.765" soft_upper_limit="9.003" k_position="7.0034" k_velocity="9.998"/>
               <calibration rising="8.654" falling="0.0445"/>
               <mimic joint="j2" multiplier="9.87" offset="0.098"/>
             </joint>
             <link name="l1">
               <visual>
                 <origin xyz="0 0 0" rpy="0 0 0" />
                 <geometry>
                   <box size="1 1 1" />
                 </geometry>
                 <material name="test_material"/>
               </visual>
             </link>
             <link name="l2"/>
           </robot>)";
    tesseract_scene_graph::SceneGraph::Ptr sg = tesseract_urdf::parseURDFString(str, resource_locator);
    EXPECT_TRUE(sg != nullptr);
    EXPECT_TRUE(sg->getName() == "test");
    EXPECT_TRUE(sg->isTree());
    EXPECT_TRUE(sg->isAcyclic());
    EXPECT_TRUE(sg->getJoints().size() == 1);
    EXPECT_TRUE(sg->getLinks().size() == 2);
    EXPECT_EQ(sg->getLink("l1")->visual[0]->material->getName(), "test_material");
  }

  {
    std::string str =
        R"(<robot name="test" extra="0 0 0">
             <joint name="j1" type="fixed">
               <parent link="l1"/>
               <child link="l2"/>
               <origin xyz="0 0 0" rpy="0 0 0"/>
               <dynamics damping="87.098" friction="3.1290"/>
               <limit lower="12.34" upper="22.999" effort="99.0" velocity="23.0"/>
               <safety_controller soft_lower_limit="8.765" soft_upper_limit="9.003" k_position="7.0034" k_velocity="9.998"/>
               <calibration rising="8.654" falling="0.0445"/>
               <mimic joint="j2" multiplier="9.87" offset="0.098"/>
             </joint>
             <link name="l1">
               <visual>
                 <origin xyz="0 0 0" rpy="0 0 0" />
                 <geometry>
                   <box size="1 1 1" />
                 </geometry>
                 <material name="test_material" extra="0 0 0">
                   <color rgba="1 .5 .5 1" extra="0 0 0"/>
                 </material>
               </visual>
             </link>
             <link name="l2">
               <visual>
                 <origin xyz="0 0 0" rpy="0 0 0" />
                 <geometry>
                   <box size="1 1 1" />
                 </geometry>
                 <material name="test_material" />
               </visual>
             </link>
           </robot>)";
    tesseract_scene_graph::SceneGraph::Ptr sg = tesseract_urdf::parseURDFString(str, resource_locator);
    EXPECT_TRUE(sg != nullptr);
    EXPECT_TRUE(sg->getName() == "test");
    EXPECT_TRUE(sg->isTree());
    EXPECT_TRUE(sg->isAcyclic());
    EXPECT_TRUE(sg->getJoints().size() == 1);
    EXPECT_TRUE(sg->getLinks().size() == 2);
    EXPECT_EQ(sg->getLink("l1")->visual[0]->material->getName(), "test_material");
  }

  {
    std::string str =
        R"(<robot name="test" extra="0 0 0">
             <joint name="j1" type="fixed">
               <parent link="l1"/>
               <child link="l2"/>
               <origin xyz="0 0 0" rpy="0 0 0"/>
               <dynamics damping="87.098" friction="3.1290"/>
               <limit lower="12.34" upper="22.999" effort="99.0" velocity="23.0"/>
               <safety_controller soft_lower_limit="8.765" soft_upper_limit="9.003" k_position="7.0034" k_velocity="9.998"/>
               <calibration rising="8.654" falling="0.0445"/>
               <mimic joint="j2" multiplier="9.87" offset="0.098"/>
             </joint>
             <link name="l1">
               <visual>
                 <origin xyz="0 0 0" rpy="0 0 0" />
                 <geometry>
                   <box size="1 1 1" />
                 </geometry>
                 <material name="test_material" />
               </visual>
             </link>
             <link name="l2">
               <visual>
                 <origin xyz="0 0 0" rpy="0 0 0" />
                 <geometry>
                   <box size="1 1 1" />
                 </geometry>
                 <material name="test_material" extra="0 0 0">
                   <color rgba="1 .5 .5 1" extra="0 0 0"/>
                 </material>
               </visual>
             </link>
           </robot>)";
    EXPECT_ANY_THROW(tesseract_urdf::parseURDFString(str, resource_locator));  // NOLINT
  }

  {
    std::string str =
        R"(<robot name="test" extra="0 0 0">
             <joint name="j1" type="fixed">
               <parent link="l1"/>
               <child link="l2"/>
               <origin xyz="0 0 0" rpy="0 0 0"/>
               <dynamics damping="87.098" friction="3.1290"/>
               <limit lower="12.34" upper="22.999" effort="99.0" velocity="23.0"/>
               <safety_controller soft_lower_limit="8.765" soft_upper_limit="9.003" k_position="7.0034" k_velocity="9.998"/>
               <calibration rising="8.654" falling="0.0445"/>
               <mimic joint="j2" multiplier="9.87" offset="0.098"/>
             </joint>
             <link name="l1">
               <visual>
                 <origin xyz="0 0 0" rpy="0 0 0" />
                 <geometry>
                   <box size="1 1 1" />
                 </geometry>
                 <material name="test_material" extra="0 0 0">
                   <color rgba="1 .5 .5 1" extra="0 0 0"/>
                 </material>
               </visual>
             </link>
             <link name="l2">
               <visual>
                 <origin xyz="0 0 0" rpy="0 0 0" />
                 <geometry>
                   <box size="1 1 1" />
                 </geometry>
                 <material name="test_material" extra="0 0 0">
                   <color rgba="1 .5 .5 1" extra="0 0 0"/>
                 </material>
               </visual>
             </link>
           </robot>)";
    tesseract_scene_graph::SceneGraph::Ptr sg = tesseract_urdf::parseURDFString(str, resource_locator);
    EXPECT_TRUE(sg != nullptr);
  }
}

TEST(TesseractURDFUnit, LoadURDFUnit)  // NOLINT
{
  using namespace tesseract_scene_graph;

  std::string urdf_file = std::string(TESSERACT_SUPPORT_DIR) + "/urdf/lbr_iiwa_14_r820.urdf";

  tesseract_common::SimpleResourceLocator locator(locateResource);
  auto g = tesseract_urdf::parseURDFFile(urdf_file, locator);

  EXPECT_TRUE(g->getJoints().size() == 9);
  EXPECT_TRUE(g->getLinks().size() == 10);
  EXPECT_TRUE(g->isTree());
  EXPECT_TRUE(g->isAcyclic());

  // Save Graph
  g->saveDOT(tesseract_common::getTempPath() + "tesseract_urdf_import.dot");

  // Get Shortest Path
  auto path = g->getShortestPath("link_1", "link_4");

  std::cout << path << std::endl;
  EXPECT_TRUE(path.links.size() == 4);
  EXPECT_TRUE(std::find(path.links.begin(), path.links.end(), "link_1") != path.links.end());
  EXPECT_TRUE(std::find(path.links.begin(), path.links.end(), "link_2") != path.links.end());
  EXPECT_TRUE(std::find(path.links.begin(), path.links.end(), "link_3") != path.links.end());
  EXPECT_TRUE(std::find(path.links.begin(), path.links.end(), "link_4") != path.links.end());
  EXPECT_TRUE(path.joints.size() == 3);
  EXPECT_TRUE(std::find(path.joints.begin(), path.joints.end(), "joint_a2") != path.joints.end());
  EXPECT_TRUE(std::find(path.joints.begin(), path.joints.end(), "joint_a3") != path.joints.end());
  EXPECT_TRUE(std::find(path.joints.begin(), path.joints.end(), "joint_a4") != path.joints.end());
}

TEST(TesseractURDFUnit, write_urdf)  // NOLINT
{
  {  // trigger nullptr input
    tesseract_scene_graph::SceneGraph::Ptr sg = nullptr;
    bool success = true;
    try
    {
      tesseract_urdf::writeURDFFile(sg, "/tmp/", "urdf0.urdf");
    }
    catch (...)
    {
      success = false;
    }

    EXPECT_FALSE(success);
  }

  {  // Successful run
    tesseract_scene_graph::SceneGraph::Ptr sg = std::make_shared<tesseract_scene_graph::SceneGraph>();

    // Add 2 links
    tesseract_scene_graph::Link::Ptr link_0 = std::make_shared<tesseract_scene_graph::Link>("link_0");
    tesseract_scene_graph::Link::Ptr link_1 = std::make_shared<tesseract_scene_graph::Link>("link_1");
    sg->addLink(*link_0);
    sg->addLink(*link_1);

    // Add joint
    tesseract_scene_graph::Joint::Ptr joint_0 = std::make_shared<tesseract_scene_graph::Joint>("joint_0");
    joint_0->type = tesseract_scene_graph::JointType::FIXED;
    joint_0->parent_link_name = link_0->getName();
    joint_0->child_link_name = link_1->getName();
    sg->addJoint(*joint_0);

    bool success = true;
    try
    {
      tesseract_urdf::writeURDFFile(sg, "/tmp/", "urdf1.urdf");
    }
    catch (...)
    {
      success = false;
    }

    EXPECT_TRUE(success);
  }

  /* Triggering a bad link is actually very difficult.  The addLink function uses Link::clone(), which
   * dereferences all the collision & visual pointers, causing a segfault if the link was ill-formed.
   * This should probably get changed to throwing an exception if it is nullptr.
  { // Trigger Bad Link
    tesseract_scene_graph::SceneGraph::Ptr sg = std::make_shared<tesseract_scene_graph::SceneGraph>();

    // Add 2 links
    tesseract_scene_graph::Link::Ptr link_0 = std::make_shared<tesseract_scene_graph::Link>("link_0");
    link_0->visual.resize(1);
    link_0->visual[0] = nullptr;  // Bad visual geometry to cause write failure in link
    tesseract_scene_graph::Link::Ptr link_1 = std::make_shared<tesseract_scene_graph::Link>("link_1");
    sg->addLink(*link_0);
    sg->addLink(*link_1);

    // Add joint
    tesseract_scene_graph::Joint::Ptr joint_0 = std::make_shared<tesseract_scene_graph::Joint>("joint_0");
    joint_0->type = tesseract_scene_graph::JointType::FIXED;
    joint_0->parent_link_name = link_0->getName();
    joint_0->child_link_name = link_1->getName();
    sg->addJoint(*joint_0);

    bool success = true;
    try
    {
      tesseract_urdf::writeURDFFile(sg, "/tmp/", "urdf3.urdf");
    }
    catch (...)
    {
      success = false;
    }

    EXPECT_FALSE(success);
  }
  */
}
