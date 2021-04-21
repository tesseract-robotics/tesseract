#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <iostream>
#include <fstream>
#include <tesseract_geometry/geometries.h>
#include <tesseract_common/utils.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_scene_graph/graph.h>
#include <tesseract_scene_graph/utils.h>
#include <tesseract_scene_graph/resource_locator.h>

TEST(TesseractSceneGraphUnit, TesseractSceneGraphLinkMaterialUnit)  // NOLINT
{
  using namespace tesseract_scene_graph;
  Material m("test_material");

  EXPECT_EQ(m.getName(), "test_material");
  EXPECT_TRUE(m.texture_filename.empty());
  EXPECT_TRUE(m.color.isApprox(Eigen::Vector4d(0.5, 0.5, 0.5, 1.0)));

  m.texture_filename = "test.png";
  m.color = Eigen::Vector4d(1.0, 1.0, 1.0, 1.0);
  m.clear();

  EXPECT_EQ(m.getName(), "test_material");
  EXPECT_TRUE(m.texture_filename.empty());
  EXPECT_TRUE(m.color.isApprox(Eigen::Vector4d(0.5, 0.5, 0.5, 1.0)));
}

TEST(TesseractSceneGraphUnit, TesseractSceneGraphLinkInertiaUnit)  // NOLINT
{
  using namespace tesseract_scene_graph;
  Inertial m;

  EXPECT_TRUE(m.origin.isApprox(Eigen::Isometry3d::Identity()));
  EXPECT_NEAR(m.mass, 0, 1e-6);
  EXPECT_NEAR(m.ixx, 0, 1e-6);
  EXPECT_NEAR(m.ixy, 0, 1e-6);
  EXPECT_NEAR(m.ixz, 0, 1e-6);
  EXPECT_NEAR(m.iyy, 0, 1e-6);
  EXPECT_NEAR(m.iyz, 0, 1e-6);
  EXPECT_NEAR(m.izz, 0, 1e-6);

  m.origin.translation() = Eigen::Vector3d(1, 2, 3);
  m.mass = 1;
  m.ixx = 5;
  m.ixy = 5;
  m.ixz = 5;
  m.iyy = 5;
  m.iyz = 5;
  m.izz = 5;
  m.clear();

  EXPECT_TRUE(m.origin.isApprox(Eigen::Isometry3d::Identity()));
  EXPECT_NEAR(m.mass, 0, 1e-6);
  EXPECT_NEAR(m.ixx, 0, 1e-6);
  EXPECT_NEAR(m.ixy, 0, 1e-6);
  EXPECT_NEAR(m.ixz, 0, 1e-6);
  EXPECT_NEAR(m.iyy, 0, 1e-6);
  EXPECT_NEAR(m.iyz, 0, 1e-6);
  EXPECT_NEAR(m.izz, 0, 1e-6);
}

TEST(TesseractSceneGraphUnit, TesseractSceneGraphLinkVisualUnit)  // NOLINT
{
  using namespace tesseract_scene_graph;
  Visual m;

  EXPECT_TRUE(m.origin.isApprox(Eigen::Isometry3d::Identity()));
  EXPECT_TRUE(m.material == DEFAULT_TESSERACT_MATERIAL);
  EXPECT_TRUE(m.geometry == nullptr);
  EXPECT_TRUE(m.name.empty());

  m.origin.translation() = Eigen::Vector3d(1, 2, 3);
  m.material = std::make_shared<Material>("test_material");
  m.material->color = Eigen::Vector4d(1, 1, 1, 1);
  m.geometry = std::make_shared<tesseract_geometry::Box>(1, 1, 1);
  m.name = "test_visual";
  m.clear();

  EXPECT_TRUE(m.origin.isApprox(Eigen::Isometry3d::Identity()));
  EXPECT_TRUE(m.material == DEFAULT_TESSERACT_MATERIAL);
  EXPECT_TRUE(m.geometry == nullptr);
  EXPECT_TRUE(m.name.empty());
}

TEST(TesseractSceneGraphUnit, TesseractSceneGraphLinkCollisionUnit)  // NOLINT
{
  using namespace tesseract_scene_graph;
  Collision m;

  EXPECT_TRUE(m.origin.isApprox(Eigen::Isometry3d::Identity()));
  EXPECT_TRUE(m.geometry == nullptr);
  EXPECT_TRUE(m.name.empty());

  m.origin.translation() = Eigen::Vector3d(1, 2, 3);
  m.geometry = std::make_shared<tesseract_geometry::Box>(1, 1, 1);
  m.name = "test_collision";
  m.clear();

  EXPECT_TRUE(m.origin.isApprox(Eigen::Isometry3d::Identity()));
  EXPECT_TRUE(m.geometry == nullptr);
  EXPECT_TRUE(m.name.empty());
}

TEST(TesseractSceneGraphUnit, TesseractSceneGraphLinkUnit)  // NOLINT
{
  using namespace tesseract_scene_graph;
  Link l("test_link");

  EXPECT_EQ(l.getName(), "test_link");

  l.inertial = std::make_shared<Inertial>();
  l.inertial->origin.translation() = Eigen::Vector3d(1, 2, 3);
  l.inertial->mass = 1;
  l.inertial->ixx = 5;
  l.inertial->ixy = 5;
  l.inertial->ixz = 5;
  l.inertial->iyy = 5;
  l.inertial->iyz = 5;
  l.inertial->izz = 5;

  Visual::Ptr v = std::make_shared<Visual>();
  v->origin.translation() = Eigen::Vector3d(1, 2, 3);
  v->material = std::make_shared<Material>("test_material");
  v->material->color = Eigen::Vector4d(1, 1, 1, 1);
  v->geometry = std::make_shared<tesseract_geometry::Box>(1, 1, 1);
  v->name = "test_visual";
  l.visual.push_back(v);

  Collision::Ptr c = std::make_shared<Collision>();
  c->origin.translation() = Eigen::Vector3d(1, 2, 3);
  c->geometry = std::make_shared<tesseract_geometry::Box>(1, 1, 1);
  c->name = "test_collision";
  l.collision.push_back(c);

  Link l_clone = l.clone();
  EXPECT_EQ(l_clone.getName(), "test_link");
  EXPECT_TRUE(l_clone.inertial != l.inertial);
  EXPECT_TRUE(l_clone.inertial != nullptr);
  EXPECT_TRUE(l_clone.inertial->origin.isApprox(l.inertial->origin));
  EXPECT_NEAR(l_clone.inertial->mass, 1, 1e-6);
  EXPECT_NEAR(l_clone.inertial->ixx, 5, 1e-6);
  EXPECT_NEAR(l_clone.inertial->ixy, 5, 1e-6);
  EXPECT_NEAR(l_clone.inertial->ixz, 5, 1e-6);
  EXPECT_NEAR(l_clone.inertial->iyy, 5, 1e-6);
  EXPECT_NEAR(l_clone.inertial->iyz, 5, 1e-6);
  EXPECT_NEAR(l_clone.inertial->izz, 5, 1e-6);
  EXPECT_EQ(l_clone.visual.size(), 1);
  EXPECT_TRUE(l_clone.visual.front() != v);
  EXPECT_TRUE(l_clone.visual.front() != nullptr);
  EXPECT_TRUE(l_clone.visual.front()->origin.isApprox(l.visual.front()->origin));
  // Pointers should be copied, no deep copy for materials and geometries
  EXPECT_TRUE(l_clone.visual.front()->material == l.visual.front()->material);
  EXPECT_TRUE(l_clone.visual.front()->geometry == l.visual.front()->geometry);
  EXPECT_TRUE(l_clone.visual.front()->name == l.visual.front()->name);
  EXPECT_EQ(l_clone.collision.size(), 1);
  EXPECT_TRUE(l_clone.collision.front() != c);
  EXPECT_TRUE(l_clone.collision.front() != nullptr);
  EXPECT_TRUE(l_clone.collision.front()->origin.isApprox(l.collision.front()->origin));
  // Pointers should be copied, no deep copy for materials and geometries
  EXPECT_TRUE(l_clone.collision.front()->geometry == l.collision.front()->geometry);
  EXPECT_TRUE(l_clone.collision.front()->name == l.collision.front()->name);

  l.clear();
  EXPECT_EQ(l.getName(), "test_link");
  EXPECT_TRUE(l.visual.empty());
  EXPECT_TRUE(l.collision.empty());
  EXPECT_TRUE(l.inertial == nullptr);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
