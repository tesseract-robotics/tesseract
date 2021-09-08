#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_geometry/impl/box.h>
#include <tesseract_urdf/box.h>
#include <tesseract_urdf/link.h>
#include "tesseract_urdf_common_unit.h"

TEST(TesseractURDFUnit, parse_link)  // NOLINT
{
  tesseract_common::SimpleResourceLocator resource_locator(locateResource);

  {
    std::unordered_map<std::string, tesseract_scene_graph::Material::Ptr> empty_available_materials;
    std::string str = R"(<link name="my_link" extra="0 0 0">
                           <inertial>
                             <origin xyz="0 0 0.5" rpy="0 0 0"/>
                             <mass value="1" />
                             <inertia ixx="100" ixy="0" ixz="0" iyy="100" iyz="0" izz="100"  />
                           </inertial>
                           <visual>
                             <origin xyz="0 0 0" rpy="0 0 0"  />
                             <geometry>
                               <box size="1 1 1"  />
                             </geometry>
                             <material name="Cyan">
                               <color rgba="0 1.0 1.0 1.0" />
                             </material>
                           </visual>
                           <collision>
                             <origin xyz="0 0 0" rpy="0 0 0" />
                             <geometry>
                               <cylinder radius="1" length="0.5" />
                             </geometry>
                           </collision>
                         </link>)";
    tesseract_scene_graph::Link::Ptr elem;
    EXPECT_TRUE(runTest<tesseract_scene_graph::Link::Ptr>(
        elem, &tesseract_urdf::parseLink, str, "link", resource_locator, empty_available_materials, 2));
    EXPECT_TRUE(elem->getName() == "my_link");
    EXPECT_TRUE(elem->inertial != nullptr);
    EXPECT_TRUE(elem->visual.size() == 1);
    EXPECT_TRUE(elem->collision.size() == 1);
    EXPECT_TRUE(empty_available_materials.size() == 1);
  }

  {
    std::unordered_map<std::string, tesseract_scene_graph::Material::Ptr> empty_available_materials;
    std::string str = R"(<link name="my_link" extra="0 0 0">
                           <inertial>
                             <origin xyz="0 0 0.5" rpy="0 0 0"/>
                             <mass value="1" />
                             <inertia ixx="100" ixy="0" ixz="0" iyy="100" iyz="0" izz="100"  />
                           </inertial>
                           <visual>
                             <origin xyz="0 0 0" rpy="0 0 0"  />
                             <geometry>
                               <box size="1 1 1"  />
                             </geometry>
                             <material name="Cyan">
                               <color rgba="0 1.0 1.0 1.0" />
                             </material>
                           </visual>
                           <visual>
                             <origin xyz="0 0 0" rpy="0 0 0"  />
                             <geometry>
                               <box size="1 1 1"  />
                             </geometry>
                             <material name="Cyan">
                               <color rgba="0 1.0 1.0 1.0" />
                             </material>
                           </visual>
                           <collision>
                             <origin xyz="0 0 0" rpy="0 0 0" />
                             <geometry>
                               <cylinder radius="1" length="0.5" />
                             </geometry>
                           </collision>
                           <collision>
                             <origin xyz="0 0 0" rpy="0 0 0" />
                             <geometry>
                               <cylinder radius="1" length="0.5" />
                             </geometry>
                           </collision>
                         </link>)";
    tesseract_scene_graph::Link::Ptr elem;
    EXPECT_TRUE(runTest<tesseract_scene_graph::Link::Ptr>(
        elem, &tesseract_urdf::parseLink, str, "link", resource_locator, empty_available_materials, 2));
    EXPECT_TRUE(elem->getName() == "my_link");
    EXPECT_TRUE(elem->inertial != nullptr);
    EXPECT_TRUE(elem->visual.size() == 2);
    EXPECT_TRUE(elem->collision.size() == 2);
    EXPECT_TRUE(empty_available_materials.size() == 1);
  }

  {
    std::unordered_map<std::string, tesseract_scene_graph::Material::Ptr> empty_available_materials;
    std::string str = R"(<link name="my_link">
                           <visual>
                             <origin xyz="0 0 0" rpy="0 0 0"  />
                             <geometry>
                               <box size="1 1 1"  />
                             </geometry>
                             <material name="Cyan">
                               <color rgba="0 1.0 1.0 1.0" />
                             </material>
                           </visual>
                           <collision>
                             <origin xyz="0 0 0" rpy="0 0 0" />
                             <geometry>
                               <cylinder radius="1" length="0.5" />
                             </geometry>
                           </collision>
                         </link>)";
    tesseract_scene_graph::Link::Ptr elem;
    EXPECT_TRUE(runTest<tesseract_scene_graph::Link::Ptr>(
        elem, &tesseract_urdf::parseLink, str, "link", resource_locator, empty_available_materials, 2));
    EXPECT_TRUE(elem->getName() == "my_link");
    EXPECT_TRUE(elem->inertial == nullptr);
    EXPECT_TRUE(elem->visual.size() == 1);
    EXPECT_TRUE(elem->collision.size() == 1);
    EXPECT_TRUE(empty_available_materials.size() == 1);
  }

  {
    std::unordered_map<std::string, tesseract_scene_graph::Material::Ptr> empty_available_materials;
    std::string str = R"(<link name="my_link">
                           <visual>
                             <origin xyz="0 0 0" rpy="0 0 0"  />
                             <geometry>
                               <box size="1 1 1"  />
                             </geometry>
                             <material name="Cyan">
                               <color rgba="0 1.0 1.0 1.0" />
                             </material>
                           </visual>
                           <visual>
                             <origin xyz="0 0 0" rpy="0 0 0"  />
                             <geometry>
                               <box size="1 1 1"  />
                             </geometry>
                             <material name="Cyan" />>
                           </visual>
                           <collision>
                             <origin xyz="0 0 0" rpy="0 0 0" />
                             <geometry>
                               <cylinder radius="1" length="0.5" />
                             </geometry>
                           </collision>
                           <collision>
                             <origin xyz="0 0 0" rpy="0 0 0" />
                             <geometry>
                               <cylinder radius="1" length="0.5" />
                             </geometry>
                           </collision>
                         </link>)";
    tesseract_scene_graph::Link::Ptr elem;
    EXPECT_TRUE(runTest<tesseract_scene_graph::Link::Ptr>(
        elem, &tesseract_urdf::parseLink, str, "link", resource_locator, empty_available_materials, 2));
    EXPECT_TRUE(elem->getName() == "my_link");
    EXPECT_TRUE(elem->inertial == nullptr);
    EXPECT_TRUE(elem->visual.size() == 2);
    EXPECT_TRUE(elem->collision.size() == 2);
    EXPECT_TRUE(empty_available_materials.size() == 1);
  }

  {
    std::unordered_map<std::string, tesseract_scene_graph::Material::Ptr> empty_available_materials;
    std::string str = R"(<link name="my_link">
                           <visual>
                             <origin xyz="0 0 0" rpy="0 0 0"  />
                             <geometry>
                               <box size="1 1 1"  />
                             </geometry>
                             <material name="Cyan">
                               <color rgba="0 1.0 1.0 1.0" />
                             </material>
                           </visual>
                         </link>)";
    tesseract_scene_graph::Link::Ptr elem;
    EXPECT_TRUE(runTest<tesseract_scene_graph::Link::Ptr>(
        elem, &tesseract_urdf::parseLink, str, "link", resource_locator, empty_available_materials, 2));
    EXPECT_TRUE(elem->getName() == "my_link");
    EXPECT_TRUE(elem->inertial == nullptr);
    EXPECT_TRUE(elem->visual.size() == 1);
    EXPECT_TRUE(elem->collision.empty());
    EXPECT_TRUE(empty_available_materials.size() == 1);
  }

  {
    std::unordered_map<std::string, tesseract_scene_graph::Material::Ptr> empty_available_materials;
    std::string str = R"(<link name="my_link">
                           <visual>
                             <origin xyz="0 0 0" rpy="0 0 0"  />
                             <geometry>
                               <box size="1 1 1"  />
                             </geometry>
                             <material name="Cyan">
                               <color rgba="0 1.0 1.0 1.0" />
                             </material>
                           </visual>
                           <visual>
                             <origin xyz="0 0 0" rpy="0 0 0" />
                             <geometry>
                               <box size="1 1 1" />
                             </geometry>
                             <material name="Cyan" />
                           </visual>
                         </link>)";
    tesseract_scene_graph::Link::Ptr elem;
    EXPECT_TRUE(runTest<tesseract_scene_graph::Link::Ptr>(
        elem, &tesseract_urdf::parseLink, str, "link", resource_locator, empty_available_materials, 2));
    EXPECT_TRUE(elem->getName() == "my_link");
    EXPECT_TRUE(elem->inertial == nullptr);
    EXPECT_TRUE(elem->visual.size() == 2);
    EXPECT_TRUE(elem->collision.empty());
    EXPECT_TRUE(empty_available_materials.size() == 1);
  }

  {
    std::unordered_map<std::string, tesseract_scene_graph::Material::Ptr> empty_available_materials;
    std::string str = R"(<link name="my_link">
                           <collision>
                             <origin xyz="0 0 0" rpy="0 0 0" />
                             <geometry>
                               <cylinder radius="1" length="0.5" />
                             </geometry>
                           </collision>
                         </link>)";
    tesseract_scene_graph::Link::Ptr elem;
    EXPECT_TRUE(runTest<tesseract_scene_graph::Link::Ptr>(
        elem, &tesseract_urdf::parseLink, str, "link", resource_locator, empty_available_materials, 2));
    EXPECT_TRUE(elem->getName() == "my_link");
    EXPECT_TRUE(elem->inertial == nullptr);
    EXPECT_TRUE(elem->visual.empty());
    EXPECT_TRUE(elem->collision.size() == 1);
    EXPECT_TRUE(empty_available_materials.empty());
  }

  {
    std::unordered_map<std::string, tesseract_scene_graph::Material::Ptr> empty_available_materials;
    std::string str = R"(<link name="my_link">
                           <collision>
                             <origin xyz="0 0 0" rpy="0 0 0" />
                             <geometry>
                               <cylinder radius="1" length="0.5" />
                             </geometry>
                           </collision>
                           <collision>
                             <origin xyz="0 0 0" rpy="0 0 0" />
                             <geometry>
                               <cylinder radius="1" length="0.5" />
                             </geometry>
                           </collision>
                         </link>)";
    tesseract_scene_graph::Link::Ptr elem;
    EXPECT_TRUE(runTest<tesseract_scene_graph::Link::Ptr>(
        elem, &tesseract_urdf::parseLink, str, "link", resource_locator, empty_available_materials, 2));
    EXPECT_TRUE(elem->getName() == "my_link");
    EXPECT_TRUE(elem->inertial == nullptr);
    EXPECT_TRUE(elem->visual.empty());
    EXPECT_TRUE(elem->collision.size() == 2);
    EXPECT_TRUE(empty_available_materials.empty());
  }

  {
    std::unordered_map<std::string, tesseract_scene_graph::Material::Ptr> empty_available_materials;
    std::string str = R"(<link name="my_link"/>)";
    tesseract_scene_graph::Link::Ptr elem;
    EXPECT_TRUE(runTest<tesseract_scene_graph::Link::Ptr>(
        elem, &tesseract_urdf::parseLink, str, "link", resource_locator, empty_available_materials, 2));
    EXPECT_TRUE(elem->getName() == "my_link");
    EXPECT_TRUE(elem->inertial == nullptr);
    EXPECT_TRUE(elem->visual.empty());
    EXPECT_TRUE(elem->collision.empty());
  }

  {
    std::unordered_map<std::string, tesseract_scene_graph::Material::Ptr> empty_available_materials;
    std::string str = R"(<link>
                           <visual>
                             <origin xyz="0 0 0" rpy="0 0 0" />
                             <geometry>
                               <box size="1 1 1"  />
                             </geometry>
                             <material name="Cyan">
                               <color rgba="0 1.0 1.0 1.0" />
                             </material>
                           </visual>
                         </link>)";
    tesseract_scene_graph::Link::Ptr elem;
    EXPECT_FALSE(runTest<tesseract_scene_graph::Link::Ptr>(
        elem, &tesseract_urdf::parseLink, str, "link", resource_locator, empty_available_materials, 2));
  }

  {
    std::unordered_map<std::string, tesseract_scene_graph::Material::Ptr> empty_available_materials;
    std::string str = R"(<link name="my_link">
                           <visual>
                             <origin xyz="0 0 0" rpy="0 0 a" />
                             <geometry>
                               <box size="1 1 1"  />
                             </geometry>
                             <material name="Cyan">
                               <color rgba="0 1.0 1.0 1.0" />
                             </material>
                           </visual>
                         </link>)";
    tesseract_scene_graph::Link::Ptr elem;
    EXPECT_FALSE(runTest<tesseract_scene_graph::Link::Ptr>(
        elem, &tesseract_urdf::parseLink, str, "link", resource_locator, empty_available_materials, 2));
  }

  {
    std::unordered_map<std::string, tesseract_scene_graph::Material::Ptr> empty_available_materials;
    std::string str = R"(<link name="my_link">
                           <inertial>
                             <mass value="a"/>
                             <inertia ixx="1.0" ixy="2.0" ixz="3.0" iyy="4.0" iyz="5.0" izz="6.0"/>
                           </inertial>
                           <collision>
                             <origin xyz="0 0 0" rpy="0 0 0" />
                             <geometry>
                               <cylinder radius="1" length="0.5" />
                             </geometry>
                           </collision>
                         </link>)";
    tesseract_scene_graph::Link::Ptr elem;
    EXPECT_FALSE(runTest<tesseract_scene_graph::Link::Ptr>(
        elem, &tesseract_urdf::parseLink, str, "link", resource_locator, empty_available_materials, 2));
  }

  {
    std::unordered_map<std::string, tesseract_scene_graph::Material::Ptr> empty_available_materials;
    std::string str = R"(<link name="my_link">
                           <collision>
                             <origin xyz="a 0 0" rpy="0 0 0" />
                             <geometry>
                               <cylinder radius="1" length="0.5" />
                             </geometry>
                           </collision>
                         </link>)";
    tesseract_scene_graph::Link::Ptr elem;
    EXPECT_FALSE(runTest<tesseract_scene_graph::Link::Ptr>(
        elem, &tesseract_urdf::parseLink, str, "link", resource_locator, empty_available_materials, 2));
  }
}

TEST(TesseractURDFUnit, write_link)  // NOLINT
{
  {  // Trigger id adjustments and inertial
    tesseract_scene_graph::Link::Ptr link = std::make_shared<tesseract_scene_graph::Link>("link");
    link->inertial = std::make_shared<tesseract_scene_graph::Inertial>();

    tesseract_scene_graph::Collision::Ptr collision = std::make_shared<tesseract_scene_graph::Collision>();
    collision->name = "test";
    collision->origin = Eigen::Isometry3d::Identity();
    collision->geometry = std::make_shared<tesseract_geometry::Box>(1.0, 1.0, 1.0);
    link->collision.push_back(collision);
    link->collision.push_back(collision);

    tesseract_scene_graph::Visual::Ptr visual = std::make_shared<tesseract_scene_graph::Visual>();
    visual->name = "test";
    visual->origin = Eigen::Isometry3d::Identity();
    visual->geometry = std::make_shared<tesseract_geometry::Box>(1.0, 1.0, 1.0);
    link->visual.push_back(visual);
    link->visual.push_back(visual);

    std::string text;
    EXPECT_EQ(
        0, writeTest<tesseract_scene_graph::Link::Ptr>(link, &tesseract_urdf::writeLink, text, std::string("/tmp/")));
    EXPECT_NE(text, "");
  }

  {  // Trigger nullptr collision
    tesseract_scene_graph::Link::Ptr link = std::make_shared<tesseract_scene_graph::Link>("link");
    link->inertial = std::make_shared<tesseract_scene_graph::Inertial>();

    link->collision.push_back(nullptr);

    tesseract_scene_graph::Visual::Ptr visual = std::make_shared<tesseract_scene_graph::Visual>();
    visual->name = "test";
    visual->origin = Eigen::Isometry3d::Identity();
    visual->geometry = std::make_shared<tesseract_geometry::Box>(1.0, 1.0, 1.0);
    link->visual.push_back(visual);

    std::string text;
    EXPECT_EQ(
        1, writeTest<tesseract_scene_graph::Link::Ptr>(link, &tesseract_urdf::writeLink, text, std::string("/tmp/")));
    EXPECT_EQ(text, "");
  }

  {  // Trigger nullptr visual
    tesseract_scene_graph::Link::Ptr link = std::make_shared<tesseract_scene_graph::Link>("link");
    link->inertial = std::make_shared<tesseract_scene_graph::Inertial>();

    tesseract_scene_graph::Collision::Ptr collision = std::make_shared<tesseract_scene_graph::Collision>();
    collision->name = "test";
    collision->origin = Eigen::Isometry3d::Identity();
    collision->geometry = std::make_shared<tesseract_geometry::Box>(1.0, 1.0, 1.0);
    link->collision.push_back(collision);

    link->visual.push_back(nullptr);

    std::string text;
    EXPECT_EQ(
        1, writeTest<tesseract_scene_graph::Link::Ptr>(link, &tesseract_urdf::writeLink, text, std::string("/tmp/")));
    EXPECT_EQ(text, "");
  }

  {
    tesseract_scene_graph::Link::Ptr link = nullptr;
    std::string text;
    EXPECT_EQ(
        1, writeTest<tesseract_scene_graph::Link::Ptr>(link, &tesseract_urdf::writeLink, text, std::string("/tmp/")));
    EXPECT_EQ(text, "");
  }
}
