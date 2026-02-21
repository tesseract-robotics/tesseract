#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_urdf/collision.h>
#include <tesseract_geometry/impl/box.h>
#include <tesseract_geometry/impl/compound_mesh.h>
#include <tesseract_common/resource_locator.h>
#include "tesseract_urdf_common_unit.h"

TEST(TesseractURDFUnit, parse_collision)  // NOLINT
{
  tesseract::common::GeneralResourceLocator resource_locator;
  const bool global_make_convex = false;
  const auto parse_collision_fn = [&](const tinyxml2::XMLElement* xml_element,
                                      const tesseract::common::ResourceLocator& locator) {
    return tesseract::urdf::parseCollision(xml_element, locator, global_make_convex);
  };

  {
    std::string str = R"(<collision extra="0 0 0">
                           <origin xyz="1 2 3" rpy="0 0 0" />
                           <geometry>
                             <box size="1 2 3" />
                           </geometry>
                         </collision>)";
    tesseract::scene_graph::Collision::Ptr elem;
    EXPECT_TRUE(runTest<tesseract::scene_graph::Collision::Ptr>(
        elem, parse_collision_fn, str, tesseract::urdf::COLLISION_ELEMENT_NAME.data(), resource_locator));
    EXPECT_TRUE(elem->geometry != nullptr);
    EXPECT_FALSE(elem->origin.isApprox(Eigen::Isometry3d::Identity(), 1e-8));
  }

  {
    std::string str = R"(<collision>
                           <geometry>
                             <box size="1 2 3" />
                           </geometry>"
                         </collision>)";
    tesseract::scene_graph::Collision::Ptr elem;
    EXPECT_TRUE(runTest<tesseract::scene_graph::Collision::Ptr>(
        elem, parse_collision_fn, str, tesseract::urdf::COLLISION_ELEMENT_NAME.data(), resource_locator));
    EXPECT_TRUE(elem->geometry != nullptr);
    EXPECT_TRUE(elem->origin.isApprox(Eigen::Isometry3d::Identity(), 1e-8));
  }

  {
    std::string str = R"(<collision>
                           <geometry>
                             <mesh filename="package://tesseract_support/meshes/box_box.dae"/>
                           </geometry>"
                         </collision>)";
    tesseract::scene_graph::Collision::Ptr elem;
    EXPECT_TRUE(runTest<tesseract::scene_graph::Collision::Ptr>(
        elem, parse_collision_fn, str, tesseract::urdf::COLLISION_ELEMENT_NAME.data(), resource_locator));
    EXPECT_TRUE(elem->geometry != nullptr);
    EXPECT_TRUE(elem->origin.isApprox(Eigen::Isometry3d::Identity(), 1e-8));
    EXPECT_TRUE(elem->geometry->getType() == tesseract::geometry::GeometryType::COMPOUND_MESH);
    EXPECT_EQ(std::dynamic_pointer_cast<const tesseract::geometry::CompoundMesh>(elem->geometry)->getMeshes().size(),
              2);
  }

  {
    std::string str = R"(<collision extra="0 0 0">
                           <origin xyz="1 2 3 5" rpy="0 0 0" />
                           <geometry>
                             <box size="1 2 3" />
                           </geometry>
                         </collision>)";
    tesseract::scene_graph::Collision::Ptr elem;
    EXPECT_FALSE(runTest<tesseract::scene_graph::Collision::Ptr>(
        elem, parse_collision_fn, str, tesseract::urdf::COLLISION_ELEMENT_NAME.data(), resource_locator));
    EXPECT_TRUE(elem == nullptr);
  }

  {
    std::string str = R"(<collision>
                           <geometry>
                             <box size="1 2 3 4" />
                           </geometry>"
                         </collision>)";
    tesseract::scene_graph::Collision::Ptr elem;
    EXPECT_FALSE(runTest<tesseract::scene_graph::Collision::Ptr>(
        elem, parse_collision_fn, str, tesseract::urdf::COLLISION_ELEMENT_NAME.data(), resource_locator));
    EXPECT_TRUE(elem == nullptr);
  }

  {
    std::string str = R"(<collision>
                         </collision>)";
    tesseract::scene_graph::Collision::Ptr elem;
    EXPECT_FALSE(runTest<tesseract::scene_graph::Collision::Ptr>(
        elem, parse_collision_fn, str, tesseract::urdf::COLLISION_ELEMENT_NAME.data(), resource_locator));
    EXPECT_TRUE(elem == nullptr);
  }
}

TEST(TesseractURDFUnit, write_collision)  // NOLINT
{
  {  // trigger check for an assigned name and check for specified ID
    tesseract::scene_graph::Collision::Ptr collision = std::make_shared<tesseract::scene_graph::Collision>();
    collision->name = "test";
    collision->origin = Eigen::Isometry3d::Identity();
    collision->geometry = std::make_shared<tesseract::geometry::Box>(1.0, 1.0, 1.0);
    std::string text;
    EXPECT_EQ(0,
              writeTest<tesseract::scene_graph::Collision::Ptr>(collision,
                                                                &tesseract::urdf::writeCollision,
                                                                text,
                                                                tesseract::common::getTempPath(),
                                                                std::string("test"),
                                                                0));
    EXPECT_NE(text, "");
  }

  {  // trigger check for an unassigned name and check for specified ID
    tesseract::scene_graph::Collision::Ptr collision = std::make_shared<tesseract::scene_graph::Collision>();
    collision->origin = Eigen::Isometry3d::Identity();
    collision->geometry = std::make_shared<tesseract::geometry::Box>(1.0, 1.0, 1.0);
    std::string text;
    EXPECT_EQ(0,
              writeTest<tesseract::scene_graph::Collision::Ptr>(collision,
                                                                &tesseract::urdf::writeCollision,
                                                                text,
                                                                tesseract::common::getTempPath(),
                                                                std::string("test"),
                                                                0));
    EXPECT_NE(text, "");
  }

  {  // trigger check for an orgin not identity and check for specified ID
    tesseract::scene_graph::Collision::Ptr collision = std::make_shared<tesseract::scene_graph::Collision>();
    collision->origin = Eigen::Isometry3d::Identity();
    collision->origin.translation() = Eigen::Vector3d(1.0, 2.0, 3.0);
    collision->geometry = std::make_shared<tesseract::geometry::Box>(1.0, 1.0, 1.0);
    std::string text;
    EXPECT_EQ(0,
              writeTest<tesseract::scene_graph::Collision::Ptr>(collision,
                                                                &tesseract::urdf::writeCollision,
                                                                text,
                                                                tesseract::common::getTempPath(),
                                                                std::string("test"),
                                                                0));
    EXPECT_NE(text, "");
  }

  {  // trigger check for nullptr input
    tesseract::scene_graph::Collision::Ptr collision = nullptr;
    std::string text;
    EXPECT_EQ(1,
              writeTest<tesseract::scene_graph::Collision::Ptr>(collision,
                                                                &tesseract::urdf::writeCollision,
                                                                text,
                                                                tesseract::common::getTempPath(),
                                                                std::string("test"),
                                                                -1));
    EXPECT_EQ(text, "");
  }

  {  // trigger check for bad geometry
    tesseract::scene_graph::Collision::Ptr collision = std::make_shared<tesseract::scene_graph::Collision>();
    collision->name = "test";
    collision->origin = Eigen::Isometry3d::Identity();
    collision->geometry = nullptr;
    std::string text;
    EXPECT_EQ(1,
              writeTest<tesseract::scene_graph::Collision::Ptr>(collision,
                                                                &tesseract::urdf::writeCollision,
                                                                text,
                                                                tesseract::common::getTempPath(),
                                                                std::string("test"),
                                                                -1));
    EXPECT_EQ(text, "");
  }
}
