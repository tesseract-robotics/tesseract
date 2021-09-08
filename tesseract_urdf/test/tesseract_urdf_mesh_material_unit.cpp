#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_urdf/mesh.h>
#include <tesseract_geometry/impl/mesh.h>
#include "tesseract_urdf_common_unit.h"

TEST(TesseractURDFUnit, parse_mesh_material_dae)  // NOLINT
{
  tesseract_common::SimpleResourceLocator resource_locator(locateResource);
  {
    std::string str = R"(<mesh filename="package://tesseract_support/meshes/tesseract_material_mesh.dae"/>)";
    std::vector<tesseract_geometry::Mesh::Ptr> meshes;
    EXPECT_TRUE(runTest<std::vector<tesseract_geometry::Mesh::Ptr>>(
        meshes, &tesseract_urdf::parseMesh, str, "mesh", resource_locator, 2, true));
    EXPECT_TRUE(meshes.size() == 4);
    auto& mesh0 = meshes[1];
    auto& mesh1 = meshes[2];
    auto& mesh2 = meshes[3];
    auto& mesh3 = meshes[0];

    EXPECT_EQ(mesh0->getFaceCount(), 34);
    EXPECT_EQ(mesh0->getVertexCount(), 68);
    EXPECT_EQ(mesh1->getFaceCount(), 15);
    EXPECT_EQ(mesh1->getVertexCount(), 17);
    EXPECT_EQ(mesh2->getFaceCount(), 15);
    EXPECT_EQ(mesh2->getVertexCount(), 17);
    EXPECT_EQ(mesh3->getFaceCount(), 2);
    EXPECT_EQ(mesh3->getVertexCount(), 4);

    auto mesh0_normals = mesh0->getNormals();
    ASSERT_TRUE(mesh0_normals != nullptr);
    EXPECT_EQ(mesh0_normals->size(), 68);
    auto mesh1_normals = mesh1->getNormals();
    ASSERT_TRUE(mesh1_normals != nullptr);
    EXPECT_EQ(mesh1_normals->size(), 17);
    auto mesh2_normals = mesh2->getNormals();
    ASSERT_TRUE(mesh2_normals != nullptr);
    EXPECT_EQ(mesh2_normals->size(), 17);
    auto mesh3_normals = mesh3->getNormals();
    ASSERT_TRUE(mesh3_normals != nullptr);
    EXPECT_EQ(mesh3_normals->size(), 4);

    auto mesh0_material = mesh0->getMaterial();
    EXPECT_TRUE(mesh0_material->getBaseColorFactor().isApprox(Eigen::Vector4d(0.7, 0.7, 0.7, 1), 0.01));
    EXPECT_NEAR(mesh0_material->getMetallicFactor(), 0.0, 0.01);
    EXPECT_NEAR(mesh0_material->getRoughnessFactor(), 0.5, 0.01);
    EXPECT_TRUE(mesh0_material->getEmissiveFactor().isApprox(Eigen::Vector4d(0, 0, 0, 1), 0.01));

    auto mesh1_material = mesh1->getMaterial();
    EXPECT_TRUE(mesh1_material->getBaseColorFactor().isApprox(Eigen::Vector4d(0.8, 0, 0, 1), 0.01));
    EXPECT_NEAR(mesh1_material->getMetallicFactor(), 0.0, 0.01);
    EXPECT_NEAR(mesh1_material->getRoughnessFactor(), 0.5, 0.01);
    EXPECT_TRUE(mesh1_material->getEmissiveFactor().isApprox(Eigen::Vector4d(0, 0, 0, 1), 0.01));

    auto mesh2_material = mesh2->getMaterial();
    EXPECT_TRUE(mesh2_material->getBaseColorFactor().isApprox(Eigen::Vector4d(0.05, 0.8, 0.05, 1), 0.01));
    EXPECT_NEAR(mesh2_material->getMetallicFactor(), 0.0, 0.01);
    EXPECT_NEAR(mesh2_material->getRoughnessFactor(), 0.5, 0.01);
    EXPECT_TRUE(mesh2_material->getEmissiveFactor().isApprox(Eigen::Vector4d(0.1, 0.1, 0.5, 1), 0.01));

    auto mesh3_material = mesh3->getMaterial();
    EXPECT_TRUE(mesh3_material->getBaseColorFactor().isApprox(Eigen::Vector4d(1, 1, 1, 1), 0.01));
    EXPECT_NEAR(mesh3_material->getMetallicFactor(), 0, 0.01);
    EXPECT_NEAR(mesh3_material->getRoughnessFactor(), 0.5, 0.01);
    EXPECT_TRUE(mesh3_material->getEmissiveFactor().isApprox(Eigen::Vector4d(0, 0, 0, 1), 0.01));

    EXPECT_TRUE(mesh0->getTextures() == nullptr);
    EXPECT_TRUE(mesh1->getTextures() == nullptr);
    EXPECT_TRUE(mesh2->getTextures() == nullptr);

    ASSERT_TRUE(mesh3->getTextures() != nullptr);
    ASSERT_EQ(mesh3->getTextures()->size(), 1);

    auto texture = mesh3->getTextures()->at(0);
    EXPECT_EQ(texture->getTextureImage()->getResourceContents().size(), 38212);
    EXPECT_EQ(texture->getUVs()->size(), 4);
  }
}

#ifdef TESSERACT_ASSIMP_USE_PBRMATERIAL
TEST(TesseractURDFUnit, parse_mesh_material_gltf2)  // NOLINT
{
  tesseract_common::SimpleResourceLocator resource_locator(locateResource);
  {
    std::string str = R"(<mesh filename="package://tesseract_support/meshes/tesseract_material_mesh.glb"/>)";
    std::vector<tesseract_geometry::Mesh::Ptr> meshes;
    EXPECT_TRUE(runTest<std::vector<tesseract_geometry::Mesh::Ptr>>(
        meshes, &tesseract_urdf::parseMesh, str, "mesh", resource_locator, 2, true));
    EXPECT_TRUE(meshes.size() == 4);
    auto& mesh0 = meshes[0];
    auto& mesh1 = meshes[1];
    auto& mesh2 = meshes[2];
    auto& mesh3 = meshes[3];

    EXPECT_EQ(mesh0->getFaceCount(), 34);
    EXPECT_EQ(mesh0->getVertexCount(), 68);
    EXPECT_EQ(mesh1->getFaceCount(), 15);
    EXPECT_EQ(mesh1->getVertexCount(), 17);
    EXPECT_EQ(mesh2->getFaceCount(), 15);
    EXPECT_EQ(mesh2->getVertexCount(), 17);
    EXPECT_EQ(mesh3->getFaceCount(), 2);
    EXPECT_EQ(mesh3->getVertexCount(), 4);

    auto mesh0_normals = mesh0->getNormals();
    ASSERT_TRUE(mesh0_normals != nullptr);
    EXPECT_EQ(mesh0_normals->size(), 68);
    auto mesh1_normals = mesh1->getNormals();
    ASSERT_TRUE(mesh1_normals != nullptr);
    EXPECT_EQ(mesh1_normals->size(), 17);
    auto mesh2_normals = mesh2->getNormals();
    ASSERT_TRUE(mesh2_normals != nullptr);
    EXPECT_EQ(mesh2_normals->size(), 17);
    auto mesh3_normals = mesh3->getNormals();
    ASSERT_TRUE(mesh3_normals != nullptr);
    EXPECT_EQ(mesh3_normals->size(), 4);

    auto mesh0_material = mesh0->getMaterial();
    EXPECT_TRUE(mesh0_material->getBaseColorFactor().isApprox(Eigen::Vector4d(0.7, 0.7, 0.7, 1), 0.01));
    EXPECT_NEAR(mesh0_material->getMetallicFactor(), 0.0, 0.01);
    EXPECT_NEAR(mesh0_material->getRoughnessFactor(), 0.5, 0.01);
    EXPECT_TRUE(mesh0_material->getEmissiveFactor().isApprox(Eigen::Vector4d(0, 0, 0, 1), 0.01));

    auto mesh1_material = mesh1->getMaterial();
    EXPECT_TRUE(mesh1_material->getBaseColorFactor().isApprox(Eigen::Vector4d(0.8, 0, 0, 1), 0.01));
    EXPECT_NEAR(mesh1_material->getMetallicFactor(), 0.8, 0.01);
    EXPECT_NEAR(mesh1_material->getRoughnessFactor(), 0.1, 0.01);
    EXPECT_TRUE(mesh1_material->getEmissiveFactor().isApprox(Eigen::Vector4d(0, 0, 0, 1), 0.01));

    auto mesh2_material = mesh2->getMaterial();
    EXPECT_TRUE(mesh2_material->getBaseColorFactor().isApprox(Eigen::Vector4d(0.05, 0.8, 0.05, 1), 0.01));
    EXPECT_NEAR(mesh2_material->getMetallicFactor(), 0.9, 0.01);
    EXPECT_NEAR(mesh2_material->getRoughnessFactor(), 0.7, 0.01);
    EXPECT_TRUE(mesh2_material->getEmissiveFactor().isApprox(Eigen::Vector4d(0.1, 0.1, 0.5, 1), 0.01));

    auto mesh3_material = mesh3->getMaterial();
    EXPECT_TRUE(mesh3_material->getBaseColorFactor().isApprox(Eigen::Vector4d(1, 1, 1, 1), 0.01));
    EXPECT_NEAR(mesh3_material->getMetallicFactor(), 0, 0.01);
    EXPECT_NEAR(mesh3_material->getRoughnessFactor(), 0.5, 0.01);
    EXPECT_TRUE(mesh3_material->getEmissiveFactor().isApprox(Eigen::Vector4d(0, 0, 0, 1), 0.01));

    EXPECT_TRUE(mesh0->getTextures() == nullptr);
    EXPECT_TRUE(mesh1->getTextures() == nullptr);
    EXPECT_TRUE(mesh2->getTextures() == nullptr);

    ASSERT_TRUE(mesh3->getTextures() != nullptr);
    ASSERT_EQ(mesh3->getTextures()->size(), 1);

    auto texture = mesh3->getTextures()->at(0);
    EXPECT_EQ(texture->getTextureImage()->getResourceContents().size(), 38212);
    EXPECT_EQ(texture->getUVs()->size(), 4);
  }
}
#endif
