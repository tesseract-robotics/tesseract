#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_urdf/link.h>
#include "tesseract_urdf_common_unit.h"

TEST(TesseractURDFUnit, parse_link)  // NOLINT
{
  std::shared_ptr<tesseract_scene_graph::SimpleResourceLocator> resource_locator =
      std::make_shared<tesseract_scene_graph::SimpleResourceLocator>(locateResource);

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
    auto status =
        runTest<tesseract_scene_graph::Link::Ptr>(elem, str, "link", resource_locator, empty_available_materials, 2);
    EXPECT_TRUE(*status);
    EXPECT_TRUE(elem->getName() == "my_link");
    EXPECT_TRUE(elem->inertial != nullptr);
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
                           <collision>
                             <origin xyz="0 0 0" rpy="0 0 0" />
                             <geometry>
                               <cylinder radius="1" length="0.5" />
                             </geometry>
                           </collision>
                         </link>)";
    tesseract_scene_graph::Link::Ptr elem;
    auto status =
        runTest<tesseract_scene_graph::Link::Ptr>(elem, str, "link", resource_locator, empty_available_materials, 2);
    EXPECT_TRUE(*status);
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
                         </link>)";
    tesseract_scene_graph::Link::Ptr elem;
    auto status =
        runTest<tesseract_scene_graph::Link::Ptr>(elem, str, "link", resource_locator, empty_available_materials, 2);
    EXPECT_TRUE(*status);
    EXPECT_TRUE(elem->getName() == "my_link");
    EXPECT_TRUE(elem->inertial == nullptr);
    EXPECT_TRUE(elem->visual.size() == 1);
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
    auto status =
        runTest<tesseract_scene_graph::Link::Ptr>(elem, str, "link", resource_locator, empty_available_materials, 2);
    EXPECT_TRUE(*status);
    EXPECT_TRUE(elem->getName() == "my_link");
    EXPECT_TRUE(elem->inertial == nullptr);
    EXPECT_TRUE(elem->visual.empty());
    EXPECT_TRUE(elem->collision.size() == 1);
    EXPECT_TRUE(empty_available_materials.empty());
  }

  {
    std::unordered_map<std::string, tesseract_scene_graph::Material::Ptr> empty_available_materials;
    std::string str = R"(<link name="my_link"/>)";
    tesseract_scene_graph::Link::Ptr elem;
    auto status =
        runTest<tesseract_scene_graph::Link::Ptr>(elem, str, "link", resource_locator, empty_available_materials, 2);
    EXPECT_TRUE(*status);
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
    auto status =
        runTest<tesseract_scene_graph::Link::Ptr>(elem, str, "link", resource_locator, empty_available_materials, 2);
    EXPECT_FALSE(*status);
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
    auto status =
        runTest<tesseract_scene_graph::Link::Ptr>(elem, str, "link", resource_locator, empty_available_materials, 2);
    EXPECT_FALSE(*status);
  }
}
