#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_urdf/urdf_parser.h>
#include "tesseract_urdf_common_unit.h"

TEST(TesseractURDFUnit, parse_urdf)  // NOLINT
{
  std::shared_ptr<tesseract_scene_graph::SimpleResourceLocator> resource_locator =
      std::make_shared<tesseract_scene_graph::SimpleResourceLocator>(locateResource);
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
    tesseract_scene_graph::SceneGraph::Ptr sg;
    auto status = tesseract_urdf::parseURDFString(sg, str, resource_locator);
    EXPECT_TRUE(*status);
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
    tesseract_scene_graph::SceneGraph::Ptr sg;
    auto status = tesseract_urdf::parseURDFString(sg, str, resource_locator);
    EXPECT_FALSE(*status);
    EXPECT_TRUE(sg == nullptr);
    CONSOLE_BRIDGE_logError(status->message().c_str());
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
    tesseract_scene_graph::SceneGraph::Ptr sg;
    auto status = tesseract_urdf::parseURDFString(sg, str, resource_locator);
    EXPECT_FALSE(*status);
    EXPECT_TRUE(sg == nullptr);
    CONSOLE_BRIDGE_logError(status->message().c_str());
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
    tesseract_scene_graph::SceneGraph::Ptr sg;
    auto status = tesseract_urdf::parseURDFString(sg, str, resource_locator);
    EXPECT_FALSE(*status);
    EXPECT_TRUE(sg == nullptr);
    CONSOLE_BRIDGE_logError(status->message().c_str());
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
    tesseract_scene_graph::SceneGraph::Ptr sg;
    auto status = tesseract_urdf::parseURDFString(sg, str, resource_locator);
    EXPECT_FALSE(*status);
    EXPECT_TRUE(sg == nullptr);
    CONSOLE_BRIDGE_logError(status->message().c_str());
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
    tesseract_scene_graph::SceneGraph::Ptr sg;
    auto status = tesseract_urdf::parseURDFString(sg, str, resource_locator);
    EXPECT_FALSE(*status);
    EXPECT_TRUE(sg == nullptr);
    CONSOLE_BRIDGE_logError(status->message().c_str());
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
    tesseract_scene_graph::SceneGraph::Ptr sg;
    auto status = tesseract_urdf::parseURDFString(sg, str, resource_locator);
    EXPECT_FALSE(*status);
    EXPECT_TRUE(sg == nullptr);
    CONSOLE_BRIDGE_logError(status->message().c_str());
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
    tesseract_scene_graph::SceneGraph::Ptr sg;
    auto status = tesseract_urdf::parseURDFString(sg, str, resource_locator);
    EXPECT_FALSE(*status);
    EXPECT_TRUE(sg == nullptr);
    CONSOLE_BRIDGE_logError(status->message().c_str());
  }

  {
    tesseract_scene_graph::SceneGraph::Ptr sg;
    auto status = tesseract_urdf::parseURDFFile(
        sg, std::string(TESSERACT_SUPPORT_DIR) + "/urdf/lbr_iiwa_14_r820.urdf", resource_locator);
    EXPECT_TRUE(*status);
  }
}

TEST(TesseractURDFUnit, parse_urdf_with_available_materials)  // NOLINT
{
  std::shared_ptr<tesseract_scene_graph::SimpleResourceLocator> resource_locator =
      std::make_shared<tesseract_scene_graph::SimpleResourceLocator>(locateResource);
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
    tesseract_scene_graph::SceneGraph::Ptr sg;
    auto status = tesseract_urdf::parseURDFString(sg, str, resource_locator);
    EXPECT_TRUE(*status);
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
    tesseract_scene_graph::SceneGraph::Ptr sg;
    auto status = tesseract_urdf::parseURDFString(sg, str, resource_locator);
    EXPECT_TRUE(*status);
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
    tesseract_scene_graph::SceneGraph::Ptr sg;
    auto status = tesseract_urdf::parseURDFString(sg, str, resource_locator);
    EXPECT_FALSE(*status);
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
    tesseract_scene_graph::SceneGraph::Ptr sg;
    auto status = tesseract_urdf::parseURDFString(sg, str, resource_locator);
    EXPECT_TRUE(*status);
  }
}

TEST(TesseractURDFUnit, LoadURDFUnit)  // NOLINT
{
  using namespace tesseract_scene_graph;

  std::string urdf_file = std::string(TESSERACT_SUPPORT_DIR) + "/urdf/lbr_iiwa_14_r820.urdf";

  tesseract_scene_graph::ResourceLocator::Ptr locator =
      std::make_shared<tesseract_scene_graph::SimpleResourceLocator>(locateResource);
  SceneGraph::Ptr g = tesseract_urdf::parseURDFFile(urdf_file, locator);

  EXPECT_TRUE(g->getJoints().size() == 9);
  EXPECT_TRUE(g->getLinks().size() == 10);
  EXPECT_TRUE(g->isTree());
  EXPECT_TRUE(g->isAcyclic());

  // Save Graph
  g->saveDOT("/tmp/tesseract_urdf_import.dot");

  // Get Shortest Path
  SceneGraph::Path path = g->getShortestPath("link_1", "link_4");

  std::cout << path << std::endl;
  EXPECT_TRUE(path.first.size() == 4);
  EXPECT_TRUE(std::find(path.first.begin(), path.first.end(), "link_1") != path.first.end());
  EXPECT_TRUE(std::find(path.first.begin(), path.first.end(), "link_2") != path.first.end());
  EXPECT_TRUE(std::find(path.first.begin(), path.first.end(), "link_3") != path.first.end());
  EXPECT_TRUE(std::find(path.first.begin(), path.first.end(), "link_4") != path.first.end());
  EXPECT_TRUE(path.second.size() == 3);
  EXPECT_TRUE(std::find(path.second.begin(), path.second.end(), "joint_a2") != path.second.end());
  EXPECT_TRUE(std::find(path.second.begin(), path.second.end(), "joint_a3") != path.second.end());
  EXPECT_TRUE(std::find(path.second.begin(), path.second.end(), "joint_a4") != path.second.end());
}
