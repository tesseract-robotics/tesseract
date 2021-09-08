#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_urdf/collision.h>
#include <tesseract_geometry/impl/box.h>
#include "tesseract_urdf_common_unit.h"

TEST(TesseractURDFUnit, parse_collision)  // NOLINT
{
  tesseract_common::SimpleResourceLocator resource_locator(locateResource);

  {
    std::string str = R"(<collision extra="0 0 0">
                           <origin xyz="1 2 3" rpy="0 0 0" />
                           <geometry>
                             <box size="1 2 3" />
                           </geometry>
                         </collision>)";
    std::vector<tesseract_scene_graph::Collision::Ptr> elem;
    EXPECT_TRUE(runTest<std::vector<tesseract_scene_graph::Collision::Ptr>>(
        elem, &tesseract_urdf::parseCollision, str, "collision", resource_locator, 2));
    EXPECT_TRUE(elem.size() == 1);
    EXPECT_TRUE(elem[0]->geometry != nullptr);
    EXPECT_FALSE(elem[0]->origin.isApprox(Eigen::Isometry3d::Identity(), 1e-8));
  }

  {
    std::string str = R"(<collision>
                           <geometry>
                             <box size="1 2 3" />
                           </geometry>"
                         </collision>)";
    std::vector<tesseract_scene_graph::Collision::Ptr> elem;
    EXPECT_TRUE(runTest<std::vector<tesseract_scene_graph::Collision::Ptr>>(
        elem, &tesseract_urdf::parseCollision, str, "collision", resource_locator, 2));
    EXPECT_TRUE(elem.size() == 1);
    EXPECT_TRUE(elem[0]->geometry != nullptr);
    EXPECT_TRUE(elem[0]->origin.isApprox(Eigen::Isometry3d::Identity(), 1e-8));
  }

  {
    std::string str = R"(<collision>
                           <geometry>
                             <mesh filename="package://tesseract_support/meshes/box_box.dae"/>
                           </geometry>"
                         </collision>)";
    std::vector<tesseract_scene_graph::Collision::Ptr> elem;
    EXPECT_TRUE(runTest<std::vector<tesseract_scene_graph::Collision::Ptr>>(
        elem, &tesseract_urdf::parseCollision, str, "collision", resource_locator, 2));
    EXPECT_TRUE(elem.size() == 2);
    EXPECT_TRUE(elem[0]->geometry != nullptr);
    EXPECT_TRUE(elem[0]->origin.isApprox(Eigen::Isometry3d::Identity(), 1e-8));
    EXPECT_TRUE(elem[1]->geometry != nullptr);
    EXPECT_TRUE(elem[1]->origin.isApprox(Eigen::Isometry3d::Identity(), 1e-8));
  }

  {
    std::string str = R"(<collision extra="0 0 0">
                           <origin xyz="1 2 3 5" rpy="0 0 0" />
                           <geometry>
                             <box size="1 2 3" />
                           </geometry>
                         </collision>)";
    std::vector<tesseract_scene_graph::Collision::Ptr> elem;
    EXPECT_FALSE(runTest<std::vector<tesseract_scene_graph::Collision::Ptr>>(
        elem, &tesseract_urdf::parseCollision, str, "collision", resource_locator, 2));
    EXPECT_TRUE(elem.empty());
  }

  {
    std::string str = R"(<collision>
                           <geometry>
                             <box size="1 2 3 4" />
                           </geometry>"
                         </collision>)";
    std::vector<tesseract_scene_graph::Collision::Ptr> elem;
    EXPECT_FALSE(runTest<std::vector<tesseract_scene_graph::Collision::Ptr>>(
        elem, &tesseract_urdf::parseCollision, str, "collision", resource_locator, 2));
    EXPECT_TRUE(elem.empty());
  }

  {
    std::string str = R"(<collision>
                         </collision>)";
    std::vector<tesseract_scene_graph::Collision::Ptr> elem;
    EXPECT_FALSE(runTest<std::vector<tesseract_scene_graph::Collision::Ptr>>(
        elem, &tesseract_urdf::parseCollision, str, "collision", resource_locator, 2));
    EXPECT_TRUE(elem.empty());
  }
}

TEST(TesseractURDFUnit, write_collision)  // NOLINT
{
  {  // trigger check for an assigned name and check for specified ID
    tesseract_scene_graph::Collision::Ptr collision = std::make_shared<tesseract_scene_graph::Collision>();
    collision->name = "test";
    collision->origin = Eigen::Isometry3d::Identity();
    collision->geometry = std::make_shared<tesseract_geometry::Box>(1.0, 1.0, 1.0);
    std::string text;
    EXPECT_EQ(0,
              writeTest<tesseract_scene_graph::Collision::Ptr>(
                  collision, &tesseract_urdf::writeCollision, text, std::string("/tmp/"), std::string("test"), 0));
    EXPECT_NE(text, "");
  }

  {  // trigger check for nullptr input
    tesseract_scene_graph::Collision::Ptr collision = nullptr;
    std::string text;
    EXPECT_EQ(1,
              writeTest<tesseract_scene_graph::Collision::Ptr>(
                  collision, &tesseract_urdf::writeCollision, text, std::string("/tmp/"), std::string("test"), -1));
    EXPECT_EQ(text, "");
  }

  {  // trigger check for bad geometry
    tesseract_scene_graph::Collision::Ptr collision = std::make_shared<tesseract_scene_graph::Collision>();
    collision->name = "test";
    collision->origin = Eigen::Isometry3d::Identity();
    collision->geometry = nullptr;
    std::string text;
    EXPECT_EQ(1,
              writeTest<tesseract_scene_graph::Collision::Ptr>(
                  collision, &tesseract_urdf::writeCollision, text, std::string("/tmp/"), std::string("test"), -1));
    EXPECT_EQ(text, "");
  }
}
