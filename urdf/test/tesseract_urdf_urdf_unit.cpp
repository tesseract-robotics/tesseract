#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <Eigen/Geometry>
#include <tesseract/common/utils.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/scene_graph/graph.h>
#include <tesseract/scene_graph/joint.h>
#include <tesseract/scene_graph/link.h>
#include <tesseract/geometry/geometries.h>
#include <tesseract/urdf/urdf_parser.h>
#include <tesseract/common/resource_locator.h>
#include "tesseract_urdf_common_unit.h"

TEST(TesseractURDFUnit, parse_urdf)  // NOLINT
{
  tesseract::common::GeneralResourceLocator resource_locator;
  {
    std::string str =
        R"(<robot name="test" extra="0 0 0" tesseract:make_convex="true">
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
    tesseract::scene_graph::SceneGraph::Ptr sg = tesseract::urdf::parseURDFString(str, resource_locator);
    EXPECT_TRUE(sg != nullptr);
    EXPECT_TRUE(sg->getName() == "test");
    EXPECT_TRUE(sg->isTree());
    EXPECT_TRUE(sg->isAcyclic());
    EXPECT_TRUE(sg->getJoints().size() == 1);
    EXPECT_TRUE(sg->getLinks().size() == 2);
  }

  {
    std::string str =
        R"(<robot name="test" extra="0 0 0" tesseract:make_convex="true">
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
    tesseract::scene_graph::SceneGraph::Ptr sg = tesseract::urdf::parseURDFString(str, resource_locator);
    EXPECT_TRUE(sg != nullptr);
    EXPECT_TRUE(sg->getName() == "test");
    EXPECT_TRUE(sg->isTree());
    EXPECT_TRUE(sg->isAcyclic());
    EXPECT_TRUE(sg->getJoints().size() == 1);
    EXPECT_TRUE(sg->getLinks().size() == 2);
  }

  {
    std::string str =
        R"(<robot name="test" tesseract:make_convex="true">
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
    EXPECT_ANY_THROW(tesseract::urdf::parseURDFString(str, resource_locator));  // NOLINT
  }

  {
    std::string str =
        R"(<robot name="test" tesseract:make_convex="true">
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
    EXPECT_ANY_THROW(tesseract::urdf::parseURDFString(str, resource_locator));  // NOLINT
  }

  {
    std::string str =
        R"(<robot name="test" tesseract:make_convex="true">
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
    EXPECT_ANY_THROW(tesseract::urdf::parseURDFString(str, resource_locator));  // NOLINT
  }

  {
    std::string str =
        R"(<robot name="test" tesseract:make_convex="true">
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
    EXPECT_ANY_THROW(tesseract::urdf::parseURDFString(str, resource_locator));  // NOLINT
  }

  {
    std::string str =
        R"(<robot name="test" tesseract:make_convex="true">
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
    EXPECT_ANY_THROW(tesseract::urdf::parseURDFString(str, resource_locator));  // NOLINT
  }

  {
    std::string str =
        R"(<robot name="test" tesseract:make_convex="true">
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
    EXPECT_ANY_THROW(tesseract::urdf::parseURDFString(str, resource_locator));  // NOLINT
  }

  {
    std::string str =
        R"(<robot tesseract:make_convex="true">
             <joint name="j1" type="fixed" >
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
    EXPECT_ANY_THROW(tesseract::urdf::parseURDFString(str, resource_locator));  // NOLINT
  }

  {
    std::string str =
        R"(<robot name="test" tesseract:make_convex="true">
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
    EXPECT_ANY_THROW(tesseract::urdf::parseURDFString(str, resource_locator));  // NOLINT
  }

  // Not a urdf
  {
    std::string str = R"(this is not a urdf)";
    EXPECT_ANY_THROW(tesseract::urdf::parseURDFString(str, resource_locator));  // NOLINT
  }

  // Missing robot element
  {
    std::string str = R"(<abcd name="test"></abcd>)";
    EXPECT_ANY_THROW(tesseract::urdf::parseURDFString(str, resource_locator));  // NOLINT
  }

  // Invalid version
  {
    std::string str = R"(<robot name="test" tesseract:make_convex="true" version="a"></robot>)";
    EXPECT_ANY_THROW(tesseract::urdf::parseURDFString(str, resource_locator));  // NOLINT
  }

  // Version not equal 1
  {
    std::string str = R"(<robot name="test" tesseract:make_convex="true" version="2"></robot>)";
    EXPECT_ANY_THROW(tesseract::urdf::parseURDFString(str, resource_locator));  // NOLINT
  }

  // Missing tesseract:make_convex attribute
  {
    std::string str = R"(<robot name="test"></robot>)";
    EXPECT_ANY_THROW(tesseract::urdf::parseURDFString(str, resource_locator));  // NOLINT
  }

  // The tesseract:make_convex attribute is not a bool
  {
    std::string str = R"(<robot name="test" tesseract:make_convex="1.5"></robot>)";
    EXPECT_ANY_THROW(tesseract::urdf::parseURDFString(str, resource_locator));  // NOLINT
  }

  {
    const std::string path =
        resource_locator.locateResource("package://tesseract/support/urdf/lbr_iiwa_14_r820.urdf")->getFilePath();
    tesseract::scene_graph::SceneGraph::Ptr sg = tesseract::urdf::parseURDFFile(path, resource_locator);
    EXPECT_TRUE(sg != nullptr);
  }
}

TEST(TesseractURDFUnit, parse_urdf_with_available_materials)  // NOLINT
{
  tesseract::common::GeneralResourceLocator resource_locator;
  {
    std::string str =
        R"(<robot name="test" extra="0 0 0" tesseract:make_convex="true">
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
    tesseract::scene_graph::SceneGraph::Ptr sg = tesseract::urdf::parseURDFString(str, resource_locator);
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
        R"(<robot name="test" extra="0 0 0" tesseract:make_convex="true">
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
    tesseract::scene_graph::SceneGraph::Ptr sg = tesseract::urdf::parseURDFString(str, resource_locator);
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
        R"(<robot name="test" extra="0 0 0" tesseract:make_convex="true">
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
    EXPECT_ANY_THROW(tesseract::urdf::parseURDFString(str, resource_locator));  // NOLINT
  }

  {
    std::string str =
        R"(<robot name="test" extra="0 0 0" tesseract:make_convex="true">
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
    tesseract::scene_graph::SceneGraph::Ptr sg = tesseract::urdf::parseURDFString(str, resource_locator);
    EXPECT_TRUE(sg != nullptr);
  }
}

TEST(TesseractURDFUnit, LoadURDFUnit)  // NOLINT
{
  using namespace tesseract::scene_graph;

  tesseract::common::GeneralResourceLocator locator;
  std::string urdf_file =
      locator.locateResource("package://tesseract/support/urdf/lbr_iiwa_14_r820.urdf")->getFilePath();

  auto g = tesseract::urdf::parseURDFFile(urdf_file, locator);

  EXPECT_TRUE(g->getJoints().size() == 9);
  EXPECT_TRUE(g->getLinks().size() == 10);
  EXPECT_TRUE(g->isTree());
  EXPECT_TRUE(g->isAcyclic());

  // Save Graph
  g->saveDOT(tesseract::common::getTempPath() + "tesseract_urdf_import.dot");

  // Get Shortest Path
  auto path = g->getShortestPath("link_1", "link_4");

  std::cout << path << "\n";
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
    tesseract::scene_graph::SceneGraph::Ptr sg = nullptr;
    bool success = true;
    try
    {
      tesseract::urdf::writeURDFFile(sg, "/tmp/", "urdf0.urdf");
    }
    catch (...)
    {
      success = false;
    }

    EXPECT_FALSE(success);
  }

  {  // Successful run
    tesseract::scene_graph::SceneGraph::Ptr sg = std::make_shared<tesseract::scene_graph::SceneGraph>();

    // Add 2 links
    tesseract::scene_graph::Link::Ptr link_0 = std::make_shared<tesseract::scene_graph::Link>("link_0");
    tesseract::scene_graph::Link::Ptr link_1 = std::make_shared<tesseract::scene_graph::Link>("link_1");
    sg->addLink(*link_0);
    sg->addLink(*link_1);

    // Add joint
    tesseract::scene_graph::Joint::Ptr joint_0 = std::make_shared<tesseract::scene_graph::Joint>("joint_0");
    joint_0->type = tesseract::scene_graph::JointType::FIXED;
    joint_0->parent_link_id = link_0->getName();
    joint_0->child_link_id = link_1->getName();
    sg->addJoint(*joint_0);

    bool success = true;
    try
    {
      tesseract::urdf::writeURDFFile(sg, "/tmp/", "urdf1.urdf");
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
    tesseract::scene_graph::SceneGraph::Ptr sg = std::make_shared<tesseract::scene_graph::SceneGraph>();

    // Add 2 links
    tesseract::scene_graph::Link::Ptr link_0 = std::make_shared<tesseract::scene_graph::Link>("link_0");
    link_0->visual.resize(1);
    link_0->visual[0] = nullptr;  // Bad visual geometry to cause write failure in link
    tesseract::scene_graph::Link::Ptr link_1 = std::make_shared<tesseract::scene_graph::Link>("link_1");
    sg->addLink(*link_0);
    sg->addLink(*link_1);

    // Add joint
    tesseract::scene_graph::Joint::Ptrros-industrial joint_0 =
  std::make_shared<tesseract::scene_graph::Joint>("joint_0"); joint_0->type = tesseract::scene_graph::JointType::FIXED;
    joint_0->parent_link_id = link_0->getName();
    joint_0->child_link_id = link_1->getName();
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

namespace
{
// A package directory, so the package:// URLs writeURDFFile writes resolve when the file is parsed back. The
// packages share a parent of their own, because a resource locator searches every directory in its package path, and
// the temp directory can hold directories this process may not read, such as systemd's private /tmp directories.
std::filesystem::path makeTempPackage(const std::string& name)
{
  const std::filesystem::path package_path =
      std::filesystem::path(tesseract::common::getTempPath()) / "tesseract_urdf_unit_packages" / name;
  std::filesystem::create_directories(package_path);
  writeTextToFile((package_path / "package.xml").string(), "<package/>");
  return package_path;
}
}  // namespace

TEST(TesseractURDFUnit, write_urdf_convex_mesh)  // NOLINT
{
  const std::filesystem::path package_path = makeTempPackage("convex_pkg");

  auto vertices = std::make_shared<tesseract::common::VectorVector3d>();
  vertices->emplace_back(0, 0, 0);
  vertices->emplace_back(1, 0, 0);
  vertices->emplace_back(0, 1, 0);
  vertices->emplace_back(0, 0, 1);
  auto faces = std::make_shared<Eigen::VectorXi>(16);
  *faces << 3, 0, 2, 1, 3, 0, 1, 3, 3, 0, 3, 2, 3, 1, 2, 3;

  tesseract::scene_graph::Link link("link_0");
  auto collision = std::make_shared<tesseract::scene_graph::Collision>();
  collision->geometry = std::make_shared<tesseract::geometry::ConvexMesh>(vertices, faces);
  link.collision.push_back(collision);

  auto sg = std::make_shared<tesseract::scene_graph::SceneGraph>("convex");
  sg->addLink(link);
  sg->addLink(tesseract::scene_graph::Link("link_1"));

  // The parser requires at least one joint
  tesseract::scene_graph::Joint joint("joint_0");
  joint.type = tesseract::scene_graph::JointType::FIXED;
  joint.parent_link_id = tesseract::common::LinkId("link_0");
  joint.child_link_id = tesseract::common::LinkId("link_1");
  sg->addJoint(joint);

  ASSERT_NO_THROW(tesseract::urdf::writeURDFFile(sg, package_path.string(), "convex"));  // NOLINT

  // Reads back as a ConvexMesh
  const std::vector<std::filesystem::path> package_dirs{ package_path.parent_path() };
  tesseract::common::GeneralResourceLocator locator(package_dirs);
  auto parsed = tesseract::urdf::parseURDFFile((package_path / "urdf" / "convex.urdf").string(), locator);
  ASSERT_NE(parsed, nullptr);
  const auto parsed_link = parsed->getLink(tesseract::common::LinkId("link_0"));
  ASSERT_NE(parsed_link, nullptr);
  ASSERT_EQ(parsed_link->collision.size(), 1);
  EXPECT_EQ(parsed_link->collision[0]->geometry->getType(), tesseract::geometry::GeometryType::CONVEX_MESH);
}

TEST(TesseractURDFUnit, write_urdf_make_convex_robot)  // NOLINT
{
  // abb_irb2400.urdf sets tesseract:make_convex, so its collision meshes load as ConvexMesh
  tesseract::common::GeneralResourceLocator locator;
  const std::string urdf_file =
      locator.locateResource("package://tesseract/support/urdf/abb_irb2400.urdf")->getFilePath();
  std::shared_ptr<const tesseract::scene_graph::SceneGraph> sg = tesseract::urdf::parseURDFFile(urdf_file, locator);
  ASSERT_NE(sg, nullptr);
  EXPECT_NO_THROW(  // NOLINT
      tesseract::urdf::writeURDFFile(sg, makeTempPackage("abb_irb2400_pkg").string(), "abb_irb2400"));
}

TEST(TesseractURDFUnit, write_urdf_save_failure)  // NOLINT
{
  const std::filesystem::path package_path = makeTempPackage("unsaved_pkg");
  auto sg = std::make_shared<tesseract::scene_graph::SceneGraph>("unsaved");
  sg->addLink(tesseract::scene_graph::Link("link_0"));

  // The target path is a directory, so the file cannot be saved
  std::filesystem::create_directories(package_path / "urdf" / "unsaved.urdf");
  EXPECT_ANY_THROW(tesseract::urdf::writeURDFFile(sg, package_path.string(), "unsaved"));  // NOLINT
}
