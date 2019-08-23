#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <vector>
#include <tinyxml2.h>
#include <console_bridge/console.h>
#include <unordered_map>
#include <pcl/io/pcd_io.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_geometry/geometries.h>
#include <tesseract_scene_graph/utils.h>
#include <tesseract_urdf/origin.h>
#include <tesseract_urdf/box.h>
#include <tesseract_urdf/sphere.h>
#include <tesseract_urdf/cone.h>
#include <tesseract_urdf/cylinder.h>
#include <tesseract_urdf/capsule.h>
#include <tesseract_urdf/mesh.h>
#include <tesseract_urdf/convex_mesh.h>
#include <tesseract_urdf/sdf_mesh.h>
#include <tesseract_urdf/octree.h>
#include <tesseract_urdf/calibration.h>
#include <tesseract_urdf/dynamics.h>
#include <tesseract_urdf/inertial.h>
#include <tesseract_urdf/limits.h>
#include <tesseract_urdf/material.h>
#include <tesseract_urdf/mimic.h>
#include <tesseract_urdf/safety_controller.h>
#include <tesseract_urdf/geometry.h>
#include <tesseract_urdf/visual.h>
#include <tesseract_urdf/collision.h>
#include <tesseract_urdf/link.h>
#include <tesseract_urdf/joint.h>
#include <tesseract_urdf/urdf_parser.h>

using namespace tesseract_urdf;

std::string locateResource(const std::string& url)
{
  std::string mod_url = url;
  if (url.find("package://tesseract_support") == 0)
  {
    mod_url.erase(0, strlen("package://tesseract_support"));
    size_t pos = mod_url.find("/");
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

template <typename ElementType>
tesseract_common::StatusCode::Ptr runTest(ElementType& type,
                                          const std::string& xml_string,
                                          const std::string element_name)
{
  tinyxml2::XMLDocument xml_doc;
  EXPECT_TRUE(xml_doc.Parse(xml_string.c_str()) == tinyxml2::XML_SUCCESS);

  tinyxml2::XMLElement *element = xml_doc.FirstChildElement(element_name.c_str());
  EXPECT_TRUE(element != nullptr);

  auto status = parse(type, element);
  if (!(*status))
  {
    CONSOLE_BRIDGE_logError(status->message().c_str());
  }

  return status;
}

template <typename ElementType>
tesseract_common::StatusCode::Ptr runTest(ElementType& type,
                                          const std::string& xml_string,
                                          const std::string element_name,
                                          tesseract_scene_graph::ResourceLocatorFn locator,
                                          bool visual)
{
  tinyxml2::XMLDocument xml_doc;
  EXPECT_TRUE(xml_doc.Parse(xml_string.c_str()) == tinyxml2::XML_SUCCESS);

  tinyxml2::XMLElement *element = xml_doc.FirstChildElement(element_name.c_str());
  EXPECT_TRUE(element != nullptr);

  auto status = parse(type, element, locator, visual);
  if (!(*status))
  {
    CONSOLE_BRIDGE_logError(status->message().c_str());
  }

  return status;
}

template <typename ElementType>
tesseract_common::StatusCode::Ptr runTest(ElementType& type,
                                          const std::string& xml_string,
                                          const std::string element_name,
                                          tesseract_scene_graph::ResourceLocatorFn locator)
{
  tinyxml2::XMLDocument xml_doc;
  EXPECT_TRUE(xml_doc.Parse(xml_string.c_str()) == tinyxml2::XML_SUCCESS);

  tinyxml2::XMLElement *element = xml_doc.FirstChildElement(element_name.c_str());
  EXPECT_TRUE(element != nullptr);

  auto status = parse(type, element, locator);
  if (!(*status))
  {
    CONSOLE_BRIDGE_logError(status->message().c_str());
  }

  return status;
}

template <typename ElementType>
tesseract_common::StatusCode::Ptr runTest(ElementType& type,
                                          const std::string& xml_string,
                                          const std::string element_name,
                                          tesseract_scene_graph::ResourceLocatorFn locator,
                                          const std::unordered_map<std::string, tesseract_scene_graph::Material::Ptr>& available_materials)
{
  tinyxml2::XMLDocument xml_doc;
  EXPECT_TRUE(xml_doc.Parse(xml_string.c_str()) == tinyxml2::XML_SUCCESS);

  tinyxml2::XMLElement *element = xml_doc.FirstChildElement(element_name.c_str());
  EXPECT_TRUE(element != nullptr);

  auto status = parse(type, element, locator, available_materials);
  if (!(*status))
  {
    CONSOLE_BRIDGE_logError(status->message().c_str());
  }

  return status;
}

template <typename ElementType>
tesseract_common::StatusCode::Ptr runTest(ElementType& type,
                                          const std::string& xml_string,
                                          const std::string element_name,
                                          const std::unordered_map<std::string, tesseract_scene_graph::Material::Ptr>& available_materials)
{
  tinyxml2::XMLDocument xml_doc;
  EXPECT_TRUE(xml_doc.Parse(xml_string.c_str()) == tinyxml2::XML_SUCCESS);

  tinyxml2::XMLElement *element = xml_doc.FirstChildElement(element_name.c_str());
  EXPECT_TRUE(element != nullptr);

  auto status = parse(type, element, available_materials);
  if (!(*status))
  {
    CONSOLE_BRIDGE_logError(status->message().c_str());
  }

  return status;
}

TEST(TesseractURDFUnit, parse_origin)
{
  {
    std::string str = "<origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>";
    Eigen::Isometry3d origin;
    auto status = runTest<Eigen::Isometry3d>(origin, str, "origin");
    EXPECT_TRUE(*status);
    EXPECT_TRUE(origin.isApprox(Eigen::Isometry3d::Identity(), 1e-8));
  }

  {
    std::string str = "<origin xyz=\"0 0 0\" q=\"1 0 0 0\"/>";
    Eigen::Isometry3d origin;
    auto status = runTest<Eigen::Isometry3d>(origin, str, "origin");
    EXPECT_TRUE(*status);
    EXPECT_TRUE(origin.isApprox(Eigen::Isometry3d::Identity(), 1e-8));
  }

  {
    std::string str = "<origin xyz=\"0 2.5 0\" rpy=\"3.14 0 -4.5\"/>";
    Eigen::Isometry3d origin;
    auto status = runTest<Eigen::Isometry3d>(origin, str, "origin");
    EXPECT_TRUE(*status);
    EXPECT_TRUE(origin.translation().isApprox(Eigen::Vector3d(0, 2.5, 0), 1e-8));
  }

  {
    std::string str = "<origin xyz=\"0 0 0\"/>";
    Eigen::Isometry3d origin;
    auto status = runTest<Eigen::Isometry3d>(origin, str, "origin");
    EXPECT_TRUE(*status);
    EXPECT_TRUE(origin.isApprox(Eigen::Isometry3d::Identity(), 1e-8));
  }

  {
    std::string str = "<origin rpy=\"0 0 0\"/>";
    Eigen::Isometry3d origin;
    auto status = runTest<Eigen::Isometry3d>(origin, str, "origin");
    EXPECT_TRUE(*status);
    EXPECT_TRUE(origin.isApprox(Eigen::Isometry3d::Identity(), 1e-8));
  }

  {
    std::string str = "<origin />";
    Eigen::Isometry3d origin;
    auto status = runTest<Eigen::Isometry3d>(origin, str, "origin");
    EXPECT_FALSE(*status);
  }
}

TEST(TesseractURDFUnit, parse_box)
{
  {
    std::string str = "<box size=\"1 2 3\"/>";
    tesseract_geometry::Box::Ptr geom;
    auto status = runTest<tesseract_geometry::Box::Ptr>(geom, str, "box");
    EXPECT_TRUE(*status);

    EXPECT_NEAR(geom->getX(), 1, 1e-8);
    EXPECT_NEAR(geom->getY(), 2, 1e-8);
    EXPECT_NEAR(geom->getZ(), 3, 1e-8);
  }

  {
    std::string str = "<box size=\"1 2 a\"/>";
    tesseract_geometry::Box::Ptr geom;
    auto status = runTest<tesseract_geometry::Box::Ptr>(geom, str, "box");
    EXPECT_FALSE(*status);
  }

  {
    std::string str = "<box size=\"1 2\"/>";
    tesseract_geometry::Box::Ptr geom;
    auto status = runTest<tesseract_geometry::Box::Ptr>(geom, str, "box");
    EXPECT_FALSE(*status);
  }

  {
    std::string str = "<box />";
    tesseract_geometry::Box::Ptr geom;
    auto status = runTest<tesseract_geometry::Box::Ptr>(geom, str, "box");
    EXPECT_FALSE(*status);
  }
}

TEST(TesseractURDFUnit, parse_sphere)
{
  {
    std::string str = "<sphere radius=\"1\"/>";
    tesseract_geometry::Sphere::Ptr geom;
    auto status = runTest<tesseract_geometry::Sphere::Ptr>(geom, str, "sphere");
    EXPECT_TRUE(*status);
    EXPECT_NEAR(geom->getRadius(), 1, 1e-8);
  }

  {
    std::string str = "<sphere radius=\"a\"/>";
    tesseract_geometry::Sphere::Ptr geom;
    auto status = runTest<tesseract_geometry::Sphere::Ptr>(geom, str, "sphere");
    EXPECT_FALSE(*status);
  }

// TODO: I would expect this to fail but tinyxml2 still parses it so need to create an issue.
//  {
//    std::string str = "<sphere radius=\"1 2\"/>";
//    tesseract_geometry::Sphere::Ptr geom;
//    auto status = runTest<tesseract_geometry::Sphere::Ptr>(geom, str, "sphere");
//    EXPECT_FALSE(*status);
//  }

  {
    std::string str = "<sphere />";
    tesseract_geometry::Sphere::Ptr geom;
    auto status = runTest<tesseract_geometry::Sphere::Ptr>(geom, str, "sphere");
    EXPECT_FALSE(*status);
  }
}

TEST(TesseractURDFUnit, parse_cone)
{
  {
    std::string str = "<cone radius=\"1\" length=\"2\"/>";
    tesseract_geometry::Cone::Ptr geom;
    auto status = runTest<tesseract_geometry::Cone::Ptr>(geom, str, "cone");
    EXPECT_TRUE(*status);
    EXPECT_NEAR(geom->getRadius(), 1, 1e-8);
    EXPECT_NEAR(geom->getLength(), 2, 1e-8);
  }

  {
    std::string str = "<cone radius=\"a\" length=\"2\"/>";
    tesseract_geometry::Cone::Ptr geom;
    auto status = runTest<tesseract_geometry::Cone::Ptr>(geom, str, "cone");
    EXPECT_FALSE(*status);
  }

  {
    std::string str = "<cone radius=\"1\" length=\"a\"/>";
    tesseract_geometry::Cone::Ptr geom;
    auto status = runTest<tesseract_geometry::Cone::Ptr>(geom, str, "cone");
    EXPECT_FALSE(*status);
  }

  // TODO: I would expect this to fail but tinyxml2 still parses it so need to create an issue.
//  {
//    std::string str = "<cone radius=\"1 2\" length=\"2 3\"/>";
//    tesseract_geometry::Cone::Ptr geom;
//    auto status = runTest<tesseract_geometry::Cone::Ptr>(geom, str, "cone");
//    EXPECT_FALSE(*status);
//  }

  {
    std::string str = "<cone radius=\"1\"/>";
    tesseract_geometry::Cone::Ptr geom;
    auto status = runTest<tesseract_geometry::Cone::Ptr>(geom, str, "cone");
    EXPECT_FALSE(*status);
  }

  {
    std::string str = "<cone length=\"2\"/>";
    tesseract_geometry::Cone::Ptr geom;
    auto status = runTest<tesseract_geometry::Cone::Ptr>(geom, str, "cone");
    EXPECT_FALSE(*status);
  }

  {
    std::string str = "<cone />";
    tesseract_geometry::Cone::Ptr geom;
    auto status = runTest<tesseract_geometry::Cone::Ptr>(geom, str, "cone");
    EXPECT_FALSE(*status);
  }
}

TEST(TesseractURDFUnit, parse_cylinder)
{
  {
    std::string str = "<cylinder radius=\"1\" length=\"2\"/>";
    tesseract_geometry::Cylinder::Ptr geom;
    auto status = runTest<tesseract_geometry::Cylinder::Ptr>(geom, str, "cylinder");
    EXPECT_TRUE(*status);
    EXPECT_NEAR(geom->getRadius(), 1, 1e-8);
    EXPECT_NEAR(geom->getLength(), 2, 1e-8);
  }

  {
    std::string str = "<cylinder radius=\"a\" length=\"2\"/>";
    tesseract_geometry::Cylinder::Ptr geom;
    auto status = runTest<tesseract_geometry::Cylinder::Ptr>(geom, str, "cylinder");
    EXPECT_FALSE(*status);
  }

  {
    std::string str = "<cylinder radius=\"1\" length=\"a\"/>";
    tesseract_geometry::Cylinder::Ptr geom;
    auto status = runTest<tesseract_geometry::Cylinder::Ptr>(geom, str, "cylinder");
    EXPECT_FALSE(*status);
  }

  // TODO: I would expect this to fail but tinyxml2 still parses it so need to create an issue.
//  {
//    std::string str = "<cylinder radius=\"1 2\" length=\"2 3\"/>";
//    tesseract_geometry::Cylinder::Ptr geom;
//    auto status = runTest<tesseract_geometry::Cylinder::Ptr>(geom, str, "cylinder");
//    EXPECT_FALSE(*status);
//  }

  {
    std::string str = "<cylinder radius=\"1\"/>";
    tesseract_geometry::Cylinder::Ptr geom;
    auto status = runTest<tesseract_geometry::Cylinder::Ptr>(geom, str, "cylinder");
    EXPECT_FALSE(*status);
  }

  {
    std::string str = "<cylinder length=\"2\"/>";
    tesseract_geometry::Cylinder::Ptr geom;
    auto status = runTest<tesseract_geometry::Cylinder::Ptr>(geom, str, "cylinder");
    EXPECT_FALSE(*status);
  }

  {
    std::string str = "<cylinder />";
    tesseract_geometry::Cylinder::Ptr geom;
    auto status = runTest<tesseract_geometry::Cylinder::Ptr>(geom, str, "cylinder");
    EXPECT_FALSE(*status);
  }
}

TEST(TesseractURDFUnit, parse_capsule)
{
  {
    std::string str = "<capsule radius=\"1\" length=\"2\"/>";
    tesseract_geometry::Capsule::Ptr geom;
    auto status = runTest<tesseract_geometry::Capsule::Ptr>(geom, str, "capsule");
    EXPECT_TRUE(*status);
    EXPECT_NEAR(geom->getRadius(), 1, 1e-8);
    EXPECT_NEAR(geom->getLength(), 2, 1e-8);
  }

  {
    std::string str = "<capsule radius=\"a\" length=\"2\"/>";
    tesseract_geometry::Capsule::Ptr geom;
    auto status = runTest<tesseract_geometry::Capsule::Ptr>(geom, str, "capsule");
    EXPECT_FALSE(*status);
  }

  {
    std::string str = "<capsule radius=\"1\" length=\"a\"/>";
    tesseract_geometry::Capsule::Ptr geom;
    auto status = runTest<tesseract_geometry::Capsule::Ptr>(geom, str, "capsule");
    EXPECT_FALSE(*status);
  }

  // TODO: I would expect this to fail but tinyxml2 still parses it so need to create an issue.
//  {
//    std::string str = "<capsule radius=\"1 2\" length=\"2 3\"/>";
//    tesseract_geometry::Capsule::Ptr geom;
//    auto status = runTest<tesseract_geometry::Capsule::Ptr>(geom, str, "capsule");
//    EXPECT_FALSE(*status);
//  }

  {
    std::string str = "<capsule radius=\"1\"/>";
    tesseract_geometry::Capsule::Ptr geom;
    auto status = runTest<tesseract_geometry::Capsule::Ptr>(geom, str, "capsule");
    EXPECT_FALSE(*status);
  }

  {
    std::string str = "<capsule length=\"2\"/>";
    tesseract_geometry::Capsule::Ptr geom;
    auto status = runTest<tesseract_geometry::Capsule::Ptr>(geom, str, "capsule");
    EXPECT_FALSE(*status);
  }

  {
    std::string str = "<capsule />";
    tesseract_geometry::Capsule::Ptr geom;
    auto status = runTest<tesseract_geometry::Capsule::Ptr>(geom, str, "capsule");
    EXPECT_FALSE(*status);
  }
}

TEST(TesseractURDFUnit, parse_mesh)
{
  {
    std::string str = "<mesh filename=\"package://tesseract_support/meshes/sphere_p25m.stl\" scale=\"1 2 1\"/>";
    std::vector<tesseract_geometry::Mesh::Ptr> geom;
    auto status = runTest<std::vector<tesseract_geometry::Mesh::Ptr>>(geom, str, "mesh", locateResource, true);
    EXPECT_TRUE(*status);
    EXPECT_TRUE(geom.size() == 1);
    EXPECT_TRUE(geom[0]->getTriangleCount() == 80);
    EXPECT_TRUE(geom[0]->getVerticeCount() == 42);
  }

  {
    std::string str = "<mesh filename=\"package://tesseract_support/meshes/sphere_p25m.stl\"/>";
    std::vector<tesseract_geometry::Mesh::Ptr> geom;
    auto status = runTest<std::vector<tesseract_geometry::Mesh::Ptr>>(geom, str, "mesh", locateResource, true);
    EXPECT_TRUE(*status);
    EXPECT_TRUE(geom.size() == 1);
    EXPECT_TRUE(geom[0]->getTriangleCount() == 80);
    EXPECT_TRUE(geom[0]->getVerticeCount() == 42);
  }

  {
    std::string str = "<mesh filename=\"abc\" scale=\"1 2 1\"/>";
    std::vector<tesseract_geometry::Mesh::Ptr> geom;
    auto status = runTest<std::vector<tesseract_geometry::Mesh::Ptr>>(geom, str, "mesh", locateResource, true);
    EXPECT_FALSE(*status);
  }

  {
    std::string str = "<mesh filename=\"package://tesseract_support/meshes/sphere_p25m.stl\" scale=\"1 a 1\"/>";
    std::vector<tesseract_geometry::Mesh::Ptr> geom;
    auto status = runTest<std::vector<tesseract_geometry::Mesh::Ptr>>(geom, str, "mesh", locateResource, true);
    EXPECT_FALSE(*status);
  }

  {
    std::string str = "<mesh filename=\"package://tesseract_support/meshes/sphere_p25m.stl\" scale=\"1 2 1 3\"/>";
    std::vector<tesseract_geometry::Mesh::Ptr> geom;
    auto status = runTest<std::vector<tesseract_geometry::Mesh::Ptr>>(geom, str, "mesh", locateResource, true);
    EXPECT_FALSE(*status);
  }

  {
    std::string str = "<mesh scale=\"1 2 1\"/>";
    std::vector<tesseract_geometry::Mesh::Ptr> geom;
    auto status = runTest<std::vector<tesseract_geometry::Mesh::Ptr>>(geom, str, "mesh", locateResource, true);
    EXPECT_FALSE(*status);
  }

  {
    std::string str = "<mesh />";
    std::vector<tesseract_geometry::Mesh::Ptr> geom;
    auto status = runTest<std::vector<tesseract_geometry::Mesh::Ptr>>(geom, str, "mesh", locateResource, true);
    EXPECT_FALSE(*status);
  }
}


TEST(TesseractURDFUnit, parse_sdf_mesh)
{
  {
    std::string str = "<sdf_mesh filename=\"package://tesseract_support/meshes/sphere_p25m.stl\" scale=\"1 2 1\"/>";
    std::vector<tesseract_geometry::SDFMesh::Ptr> geom;
    auto status = runTest<std::vector<tesseract_geometry::SDFMesh::Ptr>>(geom, str, "sdf_mesh", locateResource, true);
    EXPECT_TRUE(*status);
    EXPECT_TRUE(geom.size() == 1);
    EXPECT_TRUE(geom[0]->getTriangleCount() == 80);
    EXPECT_TRUE(geom[0]->getVerticeCount() == 42);
  }

  {
    std::string str = "<sdf_mesh filename=\"package://tesseract_support/meshes/sphere_p25m.stl\"/>";
    std::vector<tesseract_geometry::SDFMesh::Ptr> geom;
    auto status = runTest<std::vector<tesseract_geometry::SDFMesh::Ptr>>(geom, str, "sdf_mesh", locateResource, true);
    EXPECT_TRUE(*status);
    EXPECT_TRUE(geom.size() == 1);
    EXPECT_TRUE(geom[0]->getTriangleCount() == 80);
    EXPECT_TRUE(geom[0]->getVerticeCount() == 42);
  }

  {
    std::string str = "<sdf_mesh filename=\"abc\" scale=\"1 2 1\"/>";
    std::vector<tesseract_geometry::SDFMesh::Ptr> geom;
    auto status = runTest<std::vector<tesseract_geometry::SDFMesh::Ptr>>(geom, str, "sdf_mesh", locateResource, true);
    EXPECT_FALSE(*status);
  }

  {
    std::string str = "<sdf_mesh filename=\"package://tesseract_support/meshes/sphere_p25m.stl\" scale=\"1 a 1\"/>";
    std::vector<tesseract_geometry::SDFMesh::Ptr> geom;
    auto status = runTest<std::vector<tesseract_geometry::SDFMesh::Ptr>>(geom, str, "sdf_mesh", locateResource, true);
    EXPECT_FALSE(*status);
  }

  {
    std::string str = "<sdf_mesh filename=\"package://tesseract_support/meshes/sphere_p25m.stl\" scale=\"1 2 1 3\"/>";
    std::vector<tesseract_geometry::SDFMesh::Ptr> geom;
    auto status = runTest<std::vector<tesseract_geometry::SDFMesh::Ptr>>(geom, str, "sdf_mesh", locateResource, true);
    EXPECT_FALSE(*status);
  }

  {
    std::string str = "<sdf_mesh scale=\"1 2 1\"/>";
    std::vector<tesseract_geometry::SDFMesh::Ptr> geom;
    auto status = runTest<std::vector<tesseract_geometry::SDFMesh::Ptr>>(geom, str, "sdf_mesh", locateResource, true);
    EXPECT_FALSE(*status);
  }

  {
    std::string str = "<sdf_mesh />";
    std::vector<tesseract_geometry::SDFMesh::Ptr> geom;
    auto status = runTest<std::vector<tesseract_geometry::SDFMesh::Ptr>>(geom, str, "sdf_mesh", locateResource, true);
    EXPECT_FALSE(*status);
  }
}

TEST(TesseractURDFUnit, parse_convex_mesh)
{
  {
    std::string str = "<convex_mesh filename=\"package://tesseract_support/meshes/box_2m.ply\" scale=\"1 2 1\"/>";
    std::vector<tesseract_geometry::ConvexMesh::Ptr> geom;
    auto status = runTest<std::vector<tesseract_geometry::ConvexMesh::Ptr>>(geom, str, "convex_mesh", locateResource, false);
    EXPECT_TRUE(*status);
    EXPECT_TRUE(geom.size() == 1);
    EXPECT_TRUE(geom[0]->getFaceCount() == 6);
    EXPECT_TRUE(geom[0]->getVerticeCount() == 8);
  }

  {
    std::string str = "<convex_mesh filename=\"package://tesseract_support/meshes/box_2m.ply\" scale=\"1 2 1\" convert=\"true\"/>";
    std::vector<tesseract_geometry::ConvexMesh::Ptr> geom;
    auto status = runTest<std::vector<tesseract_geometry::ConvexMesh::Ptr>>(geom, str, "convex_mesh", locateResource, false);
    EXPECT_TRUE(*status);
    EXPECT_TRUE(geom.size() == 1);
    EXPECT_TRUE(geom[0]->getFaceCount() >= 6); // Because we are converting due to numerical variance you could end up with additional faces.
    EXPECT_TRUE(geom[0]->getVerticeCount() == 8);
  }

  {
    std::string str = "<convex_mesh filename=\"package://tesseract_support/meshes/box_2m.ply\"/>";
    std::vector<tesseract_geometry::ConvexMesh::Ptr> geom;
    auto status = runTest<std::vector<tesseract_geometry::ConvexMesh::Ptr>>(geom, str, "convex_mesh", locateResource, false);
    EXPECT_TRUE(*status);
    EXPECT_TRUE(geom.size() == 1);
    EXPECT_TRUE(geom[0]->getFaceCount() == 6);
    EXPECT_TRUE(geom[0]->getVerticeCount() == 8);
  }

  {
    std::string str = "<convex_mesh filename=\"abc\" scale=\"1 2 1\"/>";
    std::vector<tesseract_geometry::ConvexMesh::Ptr> geom;
    auto status = runTest<std::vector<tesseract_geometry::ConvexMesh::Ptr>>(geom, str, "convex_mesh", locateResource, false);
    EXPECT_FALSE(*status);
  }

  {
    std::string str = "<convex_mesh filename=\"package://tesseract_support/meshes/box_2m.ply\" scale=\"1 a 1\"/>";
    std::vector<tesseract_geometry::ConvexMesh::Ptr> geom;
    auto status = runTest<std::vector<tesseract_geometry::ConvexMesh::Ptr>>(geom, str, "convex_mesh", locateResource, false);
    EXPECT_FALSE(*status);
  }

  {
    std::string str = "<convex_mesh filename=\"package://tesseract_support/meshes/box_2m.ply\" scale=\"1 2 1 3\"/>";
    std::vector<tesseract_geometry::ConvexMesh::Ptr> geom;
    auto status = runTest<std::vector<tesseract_geometry::ConvexMesh::Ptr>>(geom, str, "convex_mesh", locateResource, false);
    EXPECT_FALSE(*status);
  }

  {
    std::string str = "<convex_mesh scale=\"1 2 1\"/>";
    std::vector<tesseract_geometry::ConvexMesh::Ptr> geom;
    auto status = runTest<std::vector<tesseract_geometry::ConvexMesh::Ptr>>(geom, str, "convex_mesh", locateResource, false);
    EXPECT_FALSE(*status);
  }

  {
    std::string str = "<convex_mesh />";
    std::vector<tesseract_geometry::ConvexMesh::Ptr> geom;
    auto status = runTest<std::vector<tesseract_geometry::ConvexMesh::Ptr>>(geom, str, "convex_mesh", locateResource, false);
    EXPECT_FALSE(*status);
  }
}

//bool isNodeCollapsible(octomap::OcTree& octree, octomap::OcTreeNode* node)
//{
//  if (!octree.nodeChildExists(node, 0))
//    return false;

//  double occupancy_threshold = octree.getOccupancyThres();

//  const octomap::OcTreeNode* firstChild = octree.getNodeChild(node, 0);
//  if (octree.nodeHasChildren(firstChild) || firstChild->getOccupancy() < occupancy_threshold)
//    return false;

//  for (unsigned int i = 1; i<8; i++)
//  {
//    // comparison via getChild so that casts of derived classes ensure
//    // that the right == operator gets called
//    if (!octree.nodeChildExists(node, i))
//      return false;

//    if (octree.nodeHasChildren(octree.getNodeChild(node, i)))
//      return false;

//    if (octree.getNodeChild(node, i)->getOccupancy() < occupancy_threshold)
//    {
//      CONSOLE_BRIDGE_logError("Occupancy: %f", octree.getNodeChild(node, i)->getOccupancy());
//      return false;
//    }

//  }

//  return true;
//}

//bool pruneNode(octomap::OcTree& octree, octomap::OcTreeNode* node)
//{
//  if (!isNodeCollapsible(octree, node))
//    return false;

//  // set value to children's values (all assumed equal)
//  node->copyData(*(octree.getNodeChild(node, 0)));

//  // delete children (known to be leafs at this point!)
//  for (unsigned int i=0;i<8;i++)
//  {
//    octree.deleteNodeChild(node, i);
//  }

//  return true;
//}

//void pruneRecurs(octomap::OcTree& octree, octomap::OcTreeNode* node,
//                 unsigned int depth, unsigned int max_depth, unsigned int& num_pruned)
//{
//  assert(node);

//  if (depth < max_depth)
//  {
//    for (unsigned int i=0; i<8; i++)
//    {
//      if (octree.nodeChildExists(node, i))
//      {
//        pruneRecurs(octree, octree.getNodeChild(node, i), depth+1, max_depth, num_pruned);
//      }
//    }
//  } // end if depth

//  else {
//    // max level reached
//    if (pruneNode(octree, node))
//    {
//      num_pruned++;
//    }
//  }
//}

//void prune(octomap::OcTree& octree)
//{
//  if (octree.getRoot() == nullptr)
//    return;

//  for (unsigned int depth = octree.getTreeDepth()-1; depth > 0; --depth)
//  {
//    unsigned int num_pruned = 0;
//    pruneRecurs(octree, octree.getRoot(), 0, depth, num_pruned);
//    if (num_pruned == 0)
//      break;
//  }
//}

TEST(TesseractURDFUnit, parse_octree)
{
//  {
//    // Create octomap and add save it
//    pcl::PointCloud<pcl::PointXYZ> full_cloud;
//    double delta = 0.05;
//    int length = static_cast<int>(1 / delta);

//    for (int x = 0; x < length; ++x)
//      for (int y = 0; y < length; ++y)
//        for (int z = 0; z < length; ++z)
//          full_cloud.push_back(pcl::PointXYZ(-0.5f + static_cast<float>(x * delta),
//                                             -0.5f + static_cast<float>(y * delta),
//                                             -0.5f + static_cast<float>(z * delta)));


//    pcl::io::savePCDFile("/tmp/box_pcd.pcd", full_cloud, true);
//  }

  {
    std::string str = "<octomap shape_type=\"box\">"
                      "  <octree filename=\"package://tesseract_support/meshes/box_2m.bt\"/>"
                      "</octomap>";
    tesseract_geometry::Octree::Ptr geom;
    auto status = runTest<tesseract_geometry::Octree::Ptr>(geom, str, "octomap", locateResource, true);
    EXPECT_TRUE(*status);
    EXPECT_TRUE(geom->getSubType() == geom->BOX);
    EXPECT_TRUE(geom->getOctree() != nullptr);
    EXPECT_EQ(geom->calcNumSubShapes(), 8);
  }

  {
    std::string str = "<octomap shape_type=\"box\" prune=\"true\">"
                      "  <octree filename=\"package://tesseract_support/meshes/box_2m.bt\"/>"
                      "</octomap>";
    tesseract_geometry::Octree::Ptr geom;
    auto status = runTest<tesseract_geometry::Octree::Ptr>(geom, str, "octomap", locateResource, true);
    EXPECT_TRUE(*status);
    EXPECT_TRUE(geom->getSubType() == geom->BOX);
    EXPECT_TRUE(geom->getOctree() != nullptr);
    EXPECT_EQ(geom->calcNumSubShapes(), 8);
  }

  {
    std::string str = "<octomap shape_type=\"sphere_inside\" prune=\"true\">"
                      "  <octree filename=\"package://tesseract_support/meshes/box_2m.bt\"/>"
                      "</octomap>";
    tesseract_geometry::Octree::Ptr geom;
    auto status = runTest<tesseract_geometry::Octree::Ptr>(geom, str, "octomap", locateResource, true);
    EXPECT_TRUE(*status);
    EXPECT_TRUE(geom->getSubType() == geom->SPHERE_INSIDE);
    EXPECT_TRUE(geom->getOctree() != nullptr);
    EXPECT_EQ(geom->calcNumSubShapes(), 8);
  }

  {
    std::string str = "<octomap shape_type=\"box\">"
                      "  <point_cloud filename=\"package://tesseract_support/meshes/box_pcd.pcd\" resolution=\"0.1\"/>"
                      "</octomap>";
    tesseract_geometry::Octree::Ptr geom;
    auto status = runTest<tesseract_geometry::Octree::Ptr>(geom, str, "octomap", locateResource, true);
    EXPECT_TRUE(*status);
    EXPECT_TRUE(geom->getSubType() == geom->BOX);
    EXPECT_TRUE(geom->getOctree() != nullptr);
    EXPECT_EQ(geom->calcNumSubShapes(), 944);
  }

  {
    std::string str = "<octomap shape_type=\"box\" prune=\"true\">"
                      "  <point_cloud filename=\"package://tesseract_support/meshes/box_pcd.pcd\" resolution=\"0.1\"/>"
                      "</octomap>";
    tesseract_geometry::Octree::Ptr geom;
    auto status = runTest<tesseract_geometry::Octree::Ptr>(geom, str, "octomap", locateResource, true);
    EXPECT_TRUE(*status);
    EXPECT_TRUE(geom->getSubType() == geom->BOX);
    EXPECT_TRUE(geom->getOctree() != nullptr);
    EXPECT_EQ(geom->calcNumSubShapes(), 496);
  }

  {
    std::string str = "<octomap shape_type=\"sphere_outside\" prune=\"true\">"
                      "  <octree filename=\"package://tesseract_support/meshes/box_2m.bt\"/>"
                      "</octomap>";
    tesseract_geometry::Octree::Ptr geom;
    auto status = runTest<tesseract_geometry::Octree::Ptr>(geom, str, "octomap", locateResource, true);
    EXPECT_TRUE(*status);
    EXPECT_TRUE(geom->getSubType() == geom->SPHERE_OUTSIDE);
    EXPECT_TRUE(geom->getOctree() != nullptr);
    EXPECT_EQ(geom->calcNumSubShapes(), 8);
  }

  {
    std::string str = "<octomap shape_type=\"sphere_outside\" prune=\"true\">"
                      "  <octree filename=\"abc\"/>"
                      "</octomap>";
    tesseract_geometry::Octree::Ptr geom;
    auto status = runTest<tesseract_geometry::Octree::Ptr>(geom, str, "octomap", locateResource, true);
    EXPECT_FALSE(*status);
  }

  {
    std::string str = "<octomap shape_type=\"star\" prune=\"true\">"
                      "  <octree filename=\"package://tesseract_support/meshes/box_2m.bt\"/>"
                      "</octomap>";
    tesseract_geometry::Octree::Ptr geom;
    auto status = runTest<tesseract_geometry::Octree::Ptr>(geom, str, "octomap", locateResource, true);
    EXPECT_FALSE(*status);
  }

  {
    std::string str = "<octomap shape_type=\"star\" prune=\"true\">"
                      "  <octree/>"
                      "</octomap>";
    tesseract_geometry::Octree::Ptr geom;
    auto status = runTest<tesseract_geometry::Octree::Ptr>(geom, str, "octomap", locateResource, true);
    EXPECT_FALSE(*status);
  }

  {
    std::string str = "<octomap />";
    tesseract_geometry::Octree::Ptr geom;
    auto status = runTest<tesseract_geometry::Octree::Ptr>(geom, str, "octomap", locateResource, true);
    EXPECT_FALSE(*status);
  }
}


TEST(TesseractURDFUnit, parse_calibration)
{
  {
    std::string str = "<calibration rising=\"1\" falling=\"2\"/>";
    tesseract_scene_graph::JointCalibration::Ptr elem;
    auto status = runTest<tesseract_scene_graph::JointCalibration::Ptr>(elem, str, "calibration");
    EXPECT_TRUE(*status);
    EXPECT_NEAR(elem->rising, 1, 1e-8);
    EXPECT_NEAR(elem->falling, 2, 1e-8);
  }

  {
    std::string str = "<calibration rising=\"1\"/>";
    tesseract_scene_graph::JointCalibration::Ptr elem;
    auto status = runTest<tesseract_scene_graph::JointCalibration::Ptr>(elem, str, "calibration");
    EXPECT_TRUE(*status);
    EXPECT_NEAR(elem->rising, 1, 1e-8);
    EXPECT_NEAR(elem->falling, 0, 1e-8);
  }

  {
    std::string str = "<calibration falling=\"2\"/>";
    tesseract_scene_graph::JointCalibration::Ptr elem;
    auto status = runTest<tesseract_scene_graph::JointCalibration::Ptr>(elem, str, "calibration");
    EXPECT_TRUE(*status);
    EXPECT_NEAR(elem->rising, 0, 1e-8);
    EXPECT_NEAR(elem->falling, 2, 1e-8);
  }

  {
    std::string str = "<calibration rising=\"a\" falling=\"2\"/>";
    tesseract_scene_graph::JointCalibration::Ptr elem;
    auto status = runTest<tesseract_scene_graph::JointCalibration::Ptr>(elem, str, "calibration");
    EXPECT_FALSE(*status);
  }

  {
    std::string str = "<calibration rising=\"1\" falling=\"b\"/>";
    tesseract_scene_graph::JointCalibration::Ptr elem;
    auto status = runTest<tesseract_scene_graph::JointCalibration::Ptr>(elem, str, "calibration");
    EXPECT_FALSE(*status);
  }

  {
    std::string str = "<calibration/>";
    tesseract_scene_graph::JointCalibration::Ptr elem;
    auto status = runTest<tesseract_scene_graph::JointCalibration::Ptr>(elem, str, "calibration");
    EXPECT_FALSE(*status);
  }
}

TEST(TesseractURDFUnit, parse_dynamics)
{
  {
    std::string str = "<dynamics damping=\"1\" friction=\"2\"/>";
    tesseract_scene_graph::JointDynamics::Ptr elem;
    auto status = runTest<tesseract_scene_graph::JointDynamics::Ptr>(elem, str, "dynamics");
    EXPECT_TRUE(*status);
    EXPECT_NEAR(elem->damping, 1, 1e-8);
    EXPECT_NEAR(elem->friction, 2, 1e-8);
  }

  {
    std::string str = "<dynamics damping=\"1\"/>";
    tesseract_scene_graph::JointDynamics::Ptr elem;
    auto status = runTest<tesseract_scene_graph::JointDynamics::Ptr>(elem, str, "dynamics");
    EXPECT_TRUE(*status);
    EXPECT_NEAR(elem->damping, 1, 1e-8);
    EXPECT_NEAR(elem->friction, 0, 1e-8);
  }

  {
    std::string str = "<dynamics friction=\"2\"/>";
    tesseract_scene_graph::JointDynamics::Ptr elem;
    auto status = runTest<tesseract_scene_graph::JointDynamics::Ptr>(elem, str, "dynamics");
    EXPECT_TRUE(*status);
    EXPECT_NEAR(elem->damping, 0, 1e-8);
    EXPECT_NEAR(elem->friction, 2, 1e-8);
  }

  {
    std::string str = "<dynamics damping=\"a\" friction=\"2\"/>";
    tesseract_scene_graph::JointDynamics::Ptr elem;
    auto status = runTest<tesseract_scene_graph::JointDynamics::Ptr>(elem, str, "dynamics");
    EXPECT_FALSE(*status);
  }

  {
    std::string str = "<dynamics damping=\"1\" friction=\"b\"/>";
    tesseract_scene_graph::JointDynamics::Ptr elem;
    auto status = runTest<tesseract_scene_graph::JointDynamics::Ptr>(elem, str, "dynamics");
    EXPECT_FALSE(*status);
  }

  {
    std::string str = "<dynamics/>";
    tesseract_scene_graph::JointDynamics::Ptr elem;
    auto status = runTest<tesseract_scene_graph::JointDynamics::Ptr>(elem, str, "dynamics");
    EXPECT_FALSE(*status);
  }
}

TEST(TesseractURDFUnit, parse_inertial)
{
  {
    std::string str = "<inertial>"
        "  <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>"
        "  <mass value=\"1.0\"/>"
        "  <inertia ixx=\"1.0\" ixy=\"2.0\" ixz=\"3.0\" iyy=\"4.0\" iyz=\"5.0\" izz=\"6.0\"/>"
        "</inertial>";
    tesseract_scene_graph::Inertial::Ptr elem;
    auto status = runTest<tesseract_scene_graph::Inertial::Ptr>(elem, str, "inertial");
    EXPECT_TRUE(*status);
    EXPECT_NEAR(elem->mass, 1, 1e-8);
    EXPECT_NEAR(elem->ixx, 1, 1e-8);
    EXPECT_NEAR(elem->ixy, 2, 1e-8);
    EXPECT_NEAR(elem->ixz, 3, 1e-8);
    EXPECT_NEAR(elem->iyy, 4, 1e-8);
    EXPECT_NEAR(elem->iyz, 5, 1e-8);
    EXPECT_NEAR(elem->izz, 6, 1e-8);
  }

  {
    std::string str = "<inertial>"
        "  <mass value=\"1.0\"/>"
        "  <inertia ixx=\"1.0\" ixy=\"2.0\" ixz=\"3.0\" iyy=\"4.0\" iyz=\"5.0\" izz=\"6.0\"/>"
        "</inertial>";
    tesseract_scene_graph::Inertial::Ptr elem;
    auto status = runTest<tesseract_scene_graph::Inertial::Ptr>(elem, str, "inertial");
    EXPECT_TRUE(*status);
    EXPECT_NEAR(elem->mass, 1, 1e-8);
    EXPECT_NEAR(elem->ixx, 1, 1e-8);
    EXPECT_NEAR(elem->ixy, 2, 1e-8);
    EXPECT_NEAR(elem->ixz, 3, 1e-8);
    EXPECT_NEAR(elem->iyy, 4, 1e-8);
    EXPECT_NEAR(elem->iyz, 5, 1e-8);
    EXPECT_NEAR(elem->izz, 6, 1e-8);
  }

  {
    std::string str = "<inertial>"
        "  <inertia ixx=\"1.0\" ixy=\"2.0\" ixz=\"3.0\" iyy=\"4.0\" iyz=\"5.0\" izz=\"6.0\"/>"
        "</inertial>";
    tesseract_scene_graph::Inertial::Ptr elem;
    auto status = runTest<tesseract_scene_graph::Inertial::Ptr>(elem, str, "inertial");
    EXPECT_FALSE(*status);
  }

  {
    std::string str = "<inertial>"
        "  <mass value=\"1.0\"/>"
        "</inertial>";
    tesseract_scene_graph::Inertial::Ptr elem;
    auto status = runTest<tesseract_scene_graph::Inertial::Ptr>(elem, str, "inertial");
    EXPECT_FALSE(*status);
  }

  {
    std::string str = "<inertial>"
        "  <mass value=\"a\"/>"
        "  <inertia ixx=\"1.0\" ixy=\"2.0\" ixz=\"3.0\" iyy=\"4.0\" iyz=\"5.0\" izz=\"6.0\"/>"
        "</inertial>";
    tesseract_scene_graph::Inertial::Ptr elem;
    auto status = runTest<tesseract_scene_graph::Inertial::Ptr>(elem, str, "inertial");
    EXPECT_FALSE(*status);
  }

  {
    std::string str = "<inertial>"
        "  <mass />"
        "  <inertia ixx=\"1.0\" ixy=\"2.0\" ixz=\"3.0\" iyy=\"4.0\" iyz=\"5.0\" izz=\"6.0\"/>"
        "</inertial>";
    tesseract_scene_graph::Inertial::Ptr elem;
    auto status = runTest<tesseract_scene_graph::Inertial::Ptr>(elem, str, "inertial");
    EXPECT_FALSE(*status);
  }

  {
    std::string str = "<inertial>"
        "  <mass value=\"1.0\"/>"
        "  <inertia ixx=\"a\" ixy=\"2.0\" ixz=\"3.0\" iyy=\"4.0\" iyz=\"5.0\" izz=\"6.0\"/>"
        "</inertial>";
    tesseract_scene_graph::Inertial::Ptr elem;
    auto status = runTest<tesseract_scene_graph::Inertial::Ptr>(elem, str, "inertial");
    EXPECT_FALSE(*status);
  }

  {
    std::string str = "<inertial>"
        "  <mass value=\"1.0\"/>"
        "  <inertia ixx=\"1.0\" ixy=\"a\" ixz=\"3.0\" iyy=\"4.0\" iyz=\"5.0\" izz=\"6.0\"/>"
        "</inertial>";
    tesseract_scene_graph::Inertial::Ptr elem;
    auto status = runTest<tesseract_scene_graph::Inertial::Ptr>(elem, str, "inertial");
    EXPECT_FALSE(*status);
  }

  {
    std::string str = "<inertial>"
        "  <mass value=\"1.0\"/>"
        "  <inertia ixx=\"1.0\" ixy=\"2.0\" ixz=\"a\" iyy=\"4.0\" iyz=\"5.0\" izz=\"6.0\"/>"
        "</inertial>";
    tesseract_scene_graph::Inertial::Ptr elem;
    auto status = runTest<tesseract_scene_graph::Inertial::Ptr>(elem, str, "inertial");
    EXPECT_FALSE(*status);
  }

  {
    std::string str = "<inertial>"
        "  <mass value=\"1.0\"/>"
        "  <inertia ixx=\"1.0\" ixy=\"2.0\" ixz=\"3.0\" iyy=\"a\" iyz=\"5.0\" izz=\"6.0\"/>"
        "</inertial>";
    tesseract_scene_graph::Inertial::Ptr elem;
    auto status = runTest<tesseract_scene_graph::Inertial::Ptr>(elem, str, "inertial");
    EXPECT_FALSE(*status);
  }

  {
    std::string str = "<inertial>"
        "  <mass value=\"1.0\"/>"
        "  <inertia ixx=\"1.0\" ixy=\"2.0\" ixz=\"3.0\" iyy=\"4.0\" iyz=\"a\" izz=\"6.0\"/>"
        "</inertial>";
    tesseract_scene_graph::Inertial::Ptr elem;
    auto status = runTest<tesseract_scene_graph::Inertial::Ptr>(elem, str, "inertial");
    EXPECT_FALSE(*status);
  }

  {
    std::string str = "<inertial>"
        "  <mass value=\"1.0\"/>"
        "  <inertia ixx=\"1.0\" ixy=\"2.0\" ixz=\"3.0\" iyy=\"4.0\" iyz=\"5.0\" izz=\"a\"/>"
        "</inertial>";
    tesseract_scene_graph::Inertial::Ptr elem;
    auto status = runTest<tesseract_scene_graph::Inertial::Ptr>(elem, str, "inertial");
    EXPECT_FALSE(*status);
  }

  {
    std::string str = "<inertial>"
        "  <mass value=\"1.0\"/>"
        "  <inertia ixy=\"2.0\" ixz=\"3.0\" iyy=\"4.0\" iyz=\"5.0\" izz=\"6.0\"/>"
        "</inertial>";
    tesseract_scene_graph::Inertial::Ptr elem;
    auto status = runTest<tesseract_scene_graph::Inertial::Ptr>(elem, str, "inertial");
    EXPECT_FALSE(*status);
  }

  {
    std::string str = "<inertial>"
        "  <mass value=\"1.0\"/>"
        "  <inertia ixx=\"1.0\" ixz=\"3.0\" iyy=\"4.0\" iyz=\"5.0\" izz=\"6.0\"/>"
        "</inertial>";
    tesseract_scene_graph::Inertial::Ptr elem;
    auto status = runTest<tesseract_scene_graph::Inertial::Ptr>(elem, str, "inertial");
    EXPECT_FALSE(*status);
  }

  {
    std::string str = "<inertial>"
        "  <mass value=\"1.0\"/>"
        "  <inertia ixx=\"1.0\" ixy=\"2.0\" iyy=\"4.0\" iyz=\"5.0\" izz=\"6.0\"/>"
        "</inertial>";
    tesseract_scene_graph::Inertial::Ptr elem;
    auto status = runTest<tesseract_scene_graph::Inertial::Ptr>(elem, str, "inertial");
    EXPECT_FALSE(*status);
  }

  {
    std::string str = "<inertial>"
        "  <mass value=\"1.0\"/>"
        "  <inertia ixx=\"1.0\" ixy=\"2.0\" ixz=\"3.0\" iyz=\"5.0\" izz=\"6.0\"/>"
        "</inertial>";
    tesseract_scene_graph::Inertial::Ptr elem;
    auto status = runTest<tesseract_scene_graph::Inertial::Ptr>(elem, str, "inertial");
    EXPECT_FALSE(*status);
  }

  {
    std::string str = "<inertial>"
        "  <mass value=\"1.0\"/>"
        "  <inertia ixx=\"1.0\" ixy=\"2.0\" ixz=\"3.0\" iyy=\"4.0\" izz=\"6.0\"/>"
        "</inertial>";
    tesseract_scene_graph::Inertial::Ptr elem;
    auto status = runTest<tesseract_scene_graph::Inertial::Ptr>(elem, str, "inertial");
    EXPECT_FALSE(*status);
  }

  {
    std::string str = "<inertial>"
        "  <mass value=\"1.0\"/>"
        "  <inertia ixx=\"1.0\" ixy=\"2.0\" ixz=\"3.0\" iyy=\"4.0\" iyz=\"5.0\"/>"
        "</inertial>";
    tesseract_scene_graph::Inertial::Ptr elem;
    auto status = runTest<tesseract_scene_graph::Inertial::Ptr>(elem, str, "inertial");
    EXPECT_FALSE(*status);
  }

  {
    std::string str = "<inertial>"
        "  <mass value=\"1.0\"/>"
        "  <inertia />"
        "</inertial>";
    tesseract_scene_graph::Inertial::Ptr elem;
    auto status = runTest<tesseract_scene_graph::Inertial::Ptr>(elem, str, "inertial");
    EXPECT_FALSE(*status);
  }
}

TEST(TesseractURDFUnit, parse_limits)
{
  {
    std::string str = "<limit lower=\"1\" upper=\"2\" effort=\"3\" velocity=\"4\"/>";
    tesseract_scene_graph::JointLimits::Ptr elem;
    auto status = runTest<tesseract_scene_graph::JointLimits::Ptr>(elem, str, "limit");
    EXPECT_TRUE(*status);
    EXPECT_NEAR(elem->lower, 1, 1e-8);
    EXPECT_NEAR(elem->upper, 2, 1e-8);
    EXPECT_NEAR(elem->effort, 3, 1e-8);
    EXPECT_NEAR(elem->velocity, 4, 1e-8);
  }

  {
    std::string str = "<limit upper=\"2\" effort=\"3\" velocity=\"4\"/>";
    tesseract_scene_graph::JointLimits::Ptr elem;
    auto status = runTest<tesseract_scene_graph::JointLimits::Ptr>(elem, str, "limit");
    EXPECT_TRUE(*status);
    EXPECT_NEAR(elem->lower, 0, 1e-8);
    EXPECT_NEAR(elem->upper, 2, 1e-8);
    EXPECT_NEAR(elem->effort, 3, 1e-8);
    EXPECT_NEAR(elem->velocity, 4, 1e-8);
  }

  {
    std::string str = "<limit lower=\"1\" effort=\"3\" velocity=\"4\"/>";
    tesseract_scene_graph::JointLimits::Ptr elem;
    auto status = runTest<tesseract_scene_graph::JointLimits::Ptr>(elem, str, "limit");
    EXPECT_TRUE(*status);
    EXPECT_NEAR(elem->lower, 1, 1e-8);
    EXPECT_NEAR(elem->upper, 0, 1e-8);
    EXPECT_NEAR(elem->effort, 3, 1e-8);
    EXPECT_NEAR(elem->velocity, 4, 1e-8);
  }

  {
    std::string str = "<limit effort=\"3\" velocity=\"4\"/>";
    tesseract_scene_graph::JointLimits::Ptr elem;
    auto status = runTest<tesseract_scene_graph::JointLimits::Ptr>(elem, str, "limit");
    EXPECT_TRUE(*status);
    EXPECT_NEAR(elem->lower, 0, 1e-8);
    EXPECT_NEAR(elem->upper, 0, 1e-8);
    EXPECT_NEAR(elem->effort, 3, 1e-8);
    EXPECT_NEAR(elem->velocity, 4, 1e-8);
  }

  {
    std::string str = "<limit lower=\"a\" upper=\"2\" effort=\"3\" velocity=\"4\"/>";
    tesseract_scene_graph::JointLimits::Ptr elem;
    auto status = runTest<tesseract_scene_graph::JointLimits::Ptr>(elem, str, "limit");
    EXPECT_FALSE(*status);
  }

  {
    std::string str = "<limit lower=\"1\" upper=\"a\" effort=\"3\" velocity=\"4\"/>";
    tesseract_scene_graph::JointLimits::Ptr elem;
    auto status = runTest<tesseract_scene_graph::JointLimits::Ptr>(elem, str, "limit");
    EXPECT_FALSE(*status);
  }

  {
    std::string str = "<limit lower=\"1\" upper=\"2\" effort=\"a\" velocity=\"4\"/>";
    tesseract_scene_graph::JointLimits::Ptr elem;
    auto status = runTest<tesseract_scene_graph::JointLimits::Ptr>(elem, str, "limit");
    EXPECT_FALSE(*status);
  }

  {
    std::string str = "<limit lower=\"1\" upper=\"2\" effort=\"3\" velocity=\"a\"/>";
    tesseract_scene_graph::JointLimits::Ptr elem;
    auto status = runTest<tesseract_scene_graph::JointLimits::Ptr>(elem, str, "limit");
    EXPECT_FALSE(*status);
  }

  {
    std::string str = "<limit velocity=\"4\"/>";
    tesseract_scene_graph::JointLimits::Ptr elem;
    auto status = runTest<tesseract_scene_graph::JointLimits::Ptr>(elem, str, "limit");
    EXPECT_FALSE(*status);
  }

  {
    std::string str = "<limit effort=\"3\"/>";
    tesseract_scene_graph::JointLimits::Ptr elem;
    auto status = runTest<tesseract_scene_graph::JointLimits::Ptr>(elem, str, "limit");
    EXPECT_FALSE(*status);
  }

  {
    std::string str = "<limit />";
    tesseract_scene_graph::JointLimits::Ptr elem;
    auto status = runTest<tesseract_scene_graph::JointLimits::Ptr>(elem, str, "limit");
    EXPECT_FALSE(*status);
  }
}

TEST(TesseractURDFUnit, parse_material)
{
  std::unordered_map<std::string, tesseract_scene_graph::Material::Ptr> empty_available_materials;
  std::unordered_map<std::string, tesseract_scene_graph::Material::Ptr> available_materials;
  auto m = std::make_shared<tesseract_scene_graph::Material>("test_material");
  m->color = Eigen::Vector4d(1, .5, .5, 1);
  m->texture_filename = "/tmp/texture.txt";
  available_materials["test_material"] = m;

  {
    std::string str = "<material name=\"test_material\">"
        "  <color rgba=\"1 .5 .5 1\"/>"
        "  <texture filename=\"/tmp/texture.txt\"/>"
        "</material>";
    tesseract_scene_graph::Material::Ptr elem;
    auto status = runTest<tesseract_scene_graph::Material::Ptr>(elem, str, "material", empty_available_materials);
    EXPECT_TRUE(*status);
    EXPECT_TRUE(elem->getName() == "test_material");
    EXPECT_TRUE(elem->color.isApprox(Eigen::Vector4d(1, .5, .5, 1), 1e-8));
    EXPECT_TRUE(elem->texture_filename == "/tmp/texture.txt");
  }

  {
    std::string str = "<material name=\"test_material\">"
        "  <color rgba=\"1 .5 .5 1\"/>"
        "</material>";
    tesseract_scene_graph::Material::Ptr elem;
    auto status = runTest<tesseract_scene_graph::Material::Ptr>(elem, str, "material", empty_available_materials);
    EXPECT_TRUE(*status);
    EXPECT_TRUE(elem->getName() == "test_material");
    EXPECT_TRUE(elem->color.isApprox(Eigen::Vector4d(1, .5, .5, 1), 1e-8));
    EXPECT_TRUE(elem->texture_filename == "");
  }

  {
    std::string str = "<material name=\"test_material\"/>";
    tesseract_scene_graph::Material::Ptr elem;
    auto status = runTest<tesseract_scene_graph::Material::Ptr>(elem, str, "material", available_materials);
    EXPECT_TRUE(*status);
    EXPECT_TRUE(elem->getName() == "test_material");
    EXPECT_TRUE(elem->color.isApprox(Eigen::Vector4d(1, .5, .5, 1), 1e-8));
    EXPECT_TRUE(elem->texture_filename == "/tmp/texture.txt");
  }

  {
    std::string str = "<material name=\"test_material\"/>";
    tesseract_scene_graph::Material::Ptr elem;
    auto status = runTest<tesseract_scene_graph::Material::Ptr>(elem, str, "material", empty_available_materials);
    EXPECT_FALSE(*status);
  }

  {
    std::string str = "<material name=\"test_material\">"
        "  <color rgba=\"1 .5 .5 a\"/>"
        "  <texture filename=\"/tmp/texture.txt\"/>"
        "</material>";
    tesseract_scene_graph::Material::Ptr elem;
    auto status = runTest<tesseract_scene_graph::Material::Ptr>(elem, str, "material", empty_available_materials);
    EXPECT_FALSE(*status);
  }

  {
    std::string str = "<material name=\"test_material\">"
        "  <color rgba=\"1 .5 .5 1 1\"/>"
        "  <texture filename=\"/tmp/texture.txt\"/>"
        "</material>";
    tesseract_scene_graph::Material::Ptr elem;
    auto status = runTest<tesseract_scene_graph::Material::Ptr>(elem, str, "material", empty_available_materials);
    EXPECT_FALSE(*status);
  }
}

TEST(TesseractURDFUnit, parse_mimic)
{
  {
    std::string str = "<mimic joint=\"joint_1\" multiplier=\"1\" offset=\"2\"/>";
    tesseract_scene_graph::JointMimic::Ptr elem;
    auto status = runTest<tesseract_scene_graph::JointMimic::Ptr>(elem, str, "mimic");
    EXPECT_TRUE(*status);
    EXPECT_TRUE(elem->joint_name == "joint_1");
    EXPECT_NEAR(elem->multiplier, 1, 1e-8);
    EXPECT_NEAR(elem->offset, 2, 1e-8);
  }

  {
    std::string str = "<mimic joint=\"joint_1\" multiplier=\"1\"/>";
    tesseract_scene_graph::JointMimic::Ptr elem;
    auto status = runTest<tesseract_scene_graph::JointMimic::Ptr>(elem, str, "mimic");
    EXPECT_TRUE(*status);
    EXPECT_TRUE(elem->joint_name == "joint_1");
    EXPECT_NEAR(elem->multiplier, 1, 1e-8);
    EXPECT_NEAR(elem->offset, 0, 1e-8);
  }

  {
    std::string str = "<mimic joint=\"joint_1\" offset=\"2\"/>";
    tesseract_scene_graph::JointMimic::Ptr elem;
    auto status = runTest<tesseract_scene_graph::JointMimic::Ptr>(elem, str, "mimic");
    EXPECT_TRUE(*status);
    EXPECT_TRUE(elem->joint_name == "joint_1");
    EXPECT_NEAR(elem->multiplier, 1, 1e-8);
    EXPECT_NEAR(elem->offset, 2, 1e-8);
  }

  {
    std::string str = "<mimic joint=\"joint_1\"/>";
    tesseract_scene_graph::JointMimic::Ptr elem;
    auto status = runTest<tesseract_scene_graph::JointMimic::Ptr>(elem, str, "mimic");
    EXPECT_TRUE(*status);
    EXPECT_TRUE(elem->joint_name == "joint_1");
    EXPECT_NEAR(elem->multiplier, 1, 1e-8);
    EXPECT_NEAR(elem->offset, 0, 1e-8);
  }

  {
    std::string str = "<mimic joint=\"joint_1\" multiplier=\"a\" offset=\"2\"/>";
    tesseract_scene_graph::JointMimic::Ptr elem;
    auto status = runTest<tesseract_scene_graph::JointMimic::Ptr>(elem, str, "mimic");
    EXPECT_FALSE(*status);
  }

  {
    std::string str = "<mimic joint=\"joint_1\" multiplier=\"1\" offset=\"a\"/>";
    tesseract_scene_graph::JointMimic::Ptr elem;
    auto status = runTest<tesseract_scene_graph::JointMimic::Ptr>(elem, str, "mimic");
    EXPECT_FALSE(*status);
  }

  {
    std::string str = "<mimic />";
    tesseract_scene_graph::JointMimic::Ptr elem;
    auto status = runTest<tesseract_scene_graph::JointMimic::Ptr>(elem, str, "mimic");
    EXPECT_FALSE(*status);
  }
}

TEST(TesseractURDFUnit, parse_safety_controller)
{
  {
    std::string str = "<safety_controller soft_lower_limit=\"1\" soft_upper_limit=\"2\" k_position=\"3\" k_velocity=\"4\"/>";
    tesseract_scene_graph::JointSafety::Ptr elem;
    auto status = runTest<tesseract_scene_graph::JointSafety::Ptr>(elem, str, "safety_controller");
    EXPECT_TRUE(*status);
    EXPECT_NEAR(elem->soft_lower_limit, 1, 1e-8);
    EXPECT_NEAR(elem->soft_upper_limit, 2, 1e-8);
    EXPECT_NEAR(elem->k_position, 3, 1e-8);
    EXPECT_NEAR(elem->k_velocity, 4, 1e-8);
  }

  {
    std::string str = "<safety_controller soft_upper_limit=\"2\" k_position=\"3\" k_velocity=\"4\"/>";
    tesseract_scene_graph::JointSafety::Ptr elem;
    auto status = runTest<tesseract_scene_graph::JointSafety::Ptr>(elem, str, "safety_controller");
    EXPECT_TRUE(*status);
    EXPECT_NEAR(elem->soft_lower_limit, 0, 1e-8);
    EXPECT_NEAR(elem->soft_upper_limit, 2, 1e-8);
    EXPECT_NEAR(elem->k_position, 3, 1e-8);
    EXPECT_NEAR(elem->k_velocity, 4, 1e-8);
  }

  {
    std::string str = "<safety_controller soft_lower_limit=\"1\" k_position=\"3\" k_velocity=\"4\"/>";
    tesseract_scene_graph::JointSafety::Ptr elem;
    auto status = runTest<tesseract_scene_graph::JointSafety::Ptr>(elem, str, "safety_controller");
    EXPECT_TRUE(*status);
    EXPECT_NEAR(elem->soft_lower_limit, 1, 1e-8);
    EXPECT_NEAR(elem->soft_upper_limit, 0, 1e-8);
    EXPECT_NEAR(elem->k_position, 3, 1e-8);
    EXPECT_NEAR(elem->k_velocity, 4, 1e-8);
  }

  {
    std::string str = "<safety_controller soft_lower_limit=\"1\" soft_upper_limit=\"2\" k_velocity=\"4\"/>";
    tesseract_scene_graph::JointSafety::Ptr elem;
    auto status = runTest<tesseract_scene_graph::JointSafety::Ptr>(elem, str, "safety_controller");
    EXPECT_TRUE(*status);
    EXPECT_NEAR(elem->soft_lower_limit, 1, 1e-8);
    EXPECT_NEAR(elem->soft_upper_limit, 2, 1e-8);
    EXPECT_NEAR(elem->k_position, 0, 1e-8);
    EXPECT_NEAR(elem->k_velocity, 4, 1e-8);
  }

  {
    std::string str = "<safety_controller k_velocity=\"4\"/>";
    tesseract_scene_graph::JointSafety::Ptr elem;
    auto status = runTest<tesseract_scene_graph::JointSafety::Ptr>(elem, str, "safety_controller");
    EXPECT_TRUE(*status);
    EXPECT_NEAR(elem->soft_lower_limit, 0, 1e-8);
    EXPECT_NEAR(elem->soft_upper_limit, 0, 1e-8);
    EXPECT_NEAR(elem->k_position, 0, 1e-8);
    EXPECT_NEAR(elem->k_velocity, 4, 1e-8);
  }

  {
    std::string str = "<safety_controller soft_lower_limit=\"1\" soft_upper_limit=\"2\" k_position=\"3\"/>";
    tesseract_scene_graph::JointSafety::Ptr elem;
    auto status = runTest<tesseract_scene_graph::JointSafety::Ptr>(elem, str, "safety_controller");
    EXPECT_FALSE(*status);
  }

  {
    std::string str = "<safety_controller />";
    tesseract_scene_graph::JointSafety::Ptr elem;
    auto status = runTest<tesseract_scene_graph::JointSafety::Ptr>(elem, str, "safety_controller");
    EXPECT_FALSE(*status);
  }
}

TEST(TesseractURDFUnit, parse_geometry)
{
  {
    std::string str = "<geometry>"
        "  <box size=\"1 1 1\" />"
        "</geometry>";
    std::vector<tesseract_geometry::Geometry::Ptr> elem;
    auto status = runTest<std::vector<tesseract_geometry::Geometry::Ptr>>(elem, str, "geometry", locateResource, true);
    EXPECT_TRUE(*status);
    EXPECT_TRUE(elem.size() == 1);
    EXPECT_TRUE(elem[0]->getType() == tesseract_geometry::GeometryType::BOX);
  }

  {
    std::string str = "<geometry>"
        "</geometry>";
    std::vector<tesseract_geometry::Geometry::Ptr> elem;
    auto status = runTest<std::vector<tesseract_geometry::Geometry::Ptr>>(elem, str, "geometry", locateResource, true);
    EXPECT_FALSE(*status);
  }

  {
    std::string str = "<geometry>"
        "  <box size=\"1 1 a\" />"
        "</geometry>";
    std::vector<tesseract_geometry::Geometry::Ptr> elem;
    auto status = runTest<std::vector<tesseract_geometry::Geometry::Ptr>>(elem, str, "geometry", locateResource, true);
    EXPECT_FALSE(*status);
  }
}

TEST(TesseractURDFUnit, parse_visual)
{
  std::unordered_map<std::string, tesseract_scene_graph::Material::Ptr> empty_available_materials;
  {
    std::string str = "<visual>"
        "  <origin xyz=\"1 2 3\" rpy=\"0 0 0\" />"
        "  <geometry>"
        "    <box size=\"1 2 3\" />"
        "  </geometry>"
        "  <material name=\"Cyan\">"
        "    <color rgba=\"0 1.0 1.0 1.0\"/>"
        "  </material>"
        "</visual>";
    std::vector<tesseract_scene_graph::Visual::Ptr> elem;
    auto status = runTest<std::vector<tesseract_scene_graph::Visual::Ptr>>(elem, str, "visual", locateResource, empty_available_materials);
    EXPECT_TRUE(*status);
    EXPECT_TRUE(elem.size() == 1);
    EXPECT_TRUE(elem[0]->geometry != nullptr);
    EXPECT_TRUE(elem[0]->material != nullptr);
    EXPECT_FALSE(elem[0]->origin.isApprox(Eigen::Isometry3d::Identity(), 1e-8));
  }

  {
    std::string str = "<visual>"
        "  <geometry>"
        "    <box size=\"1 2 3\" />"
        "  </geometry>"
        "  <material name=\"Cyan\">"
        "    <color rgba=\"0 1.0 1.0 1.0\"/>"
        "  </material>"
        "</visual>";
    std::vector<tesseract_scene_graph::Visual::Ptr> elem;
    auto status = runTest<std::vector<tesseract_scene_graph::Visual::Ptr>>(elem, str, "visual", locateResource, empty_available_materials);
    EXPECT_TRUE(*status);
    EXPECT_TRUE(elem.size() == 1);
    EXPECT_TRUE(elem[0]->geometry != nullptr);
    EXPECT_TRUE(elem[0]->material != nullptr);
    EXPECT_TRUE(elem[0]->origin.isApprox(Eigen::Isometry3d::Identity(), 1e-8));
  }

  {
    std::string str = "<visual>"
        "  <geometry>"
        "    <box size=\"1 2 3\" />"
        "  </geometry>"
        "</visual>";
    std::vector<tesseract_scene_graph::Visual::Ptr> elem;
    auto status = runTest<std::vector<tesseract_scene_graph::Visual::Ptr>>(elem, str, "visual", locateResource, empty_available_materials);
    EXPECT_TRUE(*status);
    EXPECT_TRUE(elem.size() == 1);
    EXPECT_TRUE(elem[0]->geometry != nullptr);
    EXPECT_TRUE(elem[0]->material != nullptr);
    EXPECT_TRUE(elem[0]->origin.isApprox(Eigen::Isometry3d::Identity(), 1e-8));
  }

  {
    std::string str = "<visual>"
        "  <material name=\"Cyan\">"
        "    <color rgba=\"0 1.0 1.0 1.0\"/>"
        "  </material>"
        "</visual>";
    std::vector<tesseract_scene_graph::Visual::Ptr> elem;
    auto status = runTest<std::vector<tesseract_scene_graph::Visual::Ptr>>(elem, str, "visual", locateResource, empty_available_materials);
    EXPECT_FALSE(*status);
  }
}

TEST(TesseractURDFUnit, parse_collision)
{
  {
    std::string str = "<collision>"
        "  <origin xyz=\"1 2 3\" rpy=\"0 0 0\" />"
        "  <geometry>"
        "    <box size=\"1 2 3\" />"
        "  </geometry>"
        "</collision>";
    std::vector<tesseract_scene_graph::Collision::Ptr> elem;
    auto status = runTest<std::vector<tesseract_scene_graph::Collision::Ptr>>(elem, str, "collision", locateResource);
    EXPECT_TRUE(*status);
    EXPECT_TRUE(elem.size() == 1);
    EXPECT_TRUE(elem[0]->geometry != nullptr);
    EXPECT_FALSE(elem[0]->origin.isApprox(Eigen::Isometry3d::Identity(), 1e-8));
  }

  {
    std::string str = "<collision>"
        "  <geometry>"
        "    <box size=\"1 2 3\" />"
        "  </geometry>"
        "</collision>";
    std::vector<tesseract_scene_graph::Collision::Ptr> elem;
    auto status = runTest<std::vector<tesseract_scene_graph::Collision::Ptr>>(elem, str, "collision", locateResource);
    EXPECT_TRUE(*status);
    EXPECT_TRUE(elem.size() == 1);
    EXPECT_TRUE(elem[0]->geometry != nullptr);
    EXPECT_TRUE(elem[0]->origin.isApprox(Eigen::Isometry3d::Identity(), 1e-8));
  }

  {
    std::string str = "<collision>"
        "</collision>";
    std::vector<tesseract_scene_graph::Collision::Ptr> elem;
    auto status = runTest<std::vector<tesseract_scene_graph::Collision::Ptr>>(elem, str, "collision", locateResource);
    EXPECT_FALSE(*status);
  }
}

TEST(TesseractURDFUnit, parse_link)
{
  std::unordered_map<std::string, tesseract_scene_graph::Material::Ptr> empty_available_materials;
  {
    std::string str = "<link name=\"my_link\">"
                      "  <inertial>"
                      "    <origin xyz=\"0 0 0.5\" rpy=\"0 0 0\"/>"
                      "    <mass value=\"1\"/>"
                      "    <inertia ixx=\"100\" ixy=\"0\" ixz=\"0\" iyy=\"100\" iyz=\"0\" izz=\"100\" />"
                      "  </inertial>"
                      "  <visual>"
                      "    <origin xyz=\"0 0 0\" rpy=\"0 0 0\" />"
                      "    <geometry>"
                      "      <box size=\"1 1 1\" />"
                      "    </geometry>"
                      "    <material name=\"Cyan\">"
                      "      <color rgba=\"0 1.0 1.0 1.0\"/>"
                      "    </material>"
                      "  </visual>"
                      "  <collision>"
                      "    <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>"
                      "    <geometry>"
                      "      <cylinder radius=\"1\" length=\"0.5\"/>"
                      "    </geometry>"
                      "  </collision>"
                      "</link>";
    tesseract_scene_graph::Link::Ptr elem;
    auto status = runTest<tesseract_scene_graph::Link::Ptr>(elem, str, "link", locateResource, empty_available_materials);
    EXPECT_TRUE(*status);
    EXPECT_TRUE(elem->getName() == "my_link");
    EXPECT_TRUE(elem->inertial != nullptr);
    EXPECT_TRUE(elem->visual.size() == 1);
    EXPECT_TRUE(elem->collision.size() == 1);
  }

  {
    std::string str = "<link name=\"my_link\">"
                      "  <visual>"
                      "    <origin xyz=\"0 0 0\" rpy=\"0 0 0\" />"
                      "    <geometry>"
                      "      <box size=\"1 1 1\" />"
                      "    </geometry>"
                      "    <material name=\"Cyan\">"
                      "      <color rgba=\"0 1.0 1.0 1.0\"/>"
                      "    </material>"
                      "  </visual>"
                      "  <collision>"
                      "    <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>"
                      "    <geometry>"
                      "      <cylinder radius=\"1\" length=\"0.5\"/>"
                      "    </geometry>"
                      "  </collision>"
                      "</link>";
    tesseract_scene_graph::Link::Ptr elem;
    auto status = runTest<tesseract_scene_graph::Link::Ptr>(elem, str, "link", locateResource, empty_available_materials);
    EXPECT_TRUE(*status);
    EXPECT_TRUE(elem->getName() == "my_link");
    EXPECT_TRUE(elem->inertial == nullptr);
    EXPECT_TRUE(elem->visual.size() == 1);
    EXPECT_TRUE(elem->collision.size() == 1);
  }

  {
    std::string str = "<link name=\"my_link\">"
                      "  <visual>"
                      "    <origin xyz=\"0 0 0\" rpy=\"0 0 0\" />"
                      "    <geometry>"
                      "      <box size=\"1 1 1\" />"
                      "    </geometry>"
                      "    <material name=\"Cyan\">"
                      "      <color rgba=\"0 1.0 1.0 1.0\"/>"
                      "    </material>"
                      "  </visual>"
                      "</link>";
    tesseract_scene_graph::Link::Ptr elem;
    auto status = runTest<tesseract_scene_graph::Link::Ptr>(elem, str, "link", locateResource, empty_available_materials);
    EXPECT_TRUE(*status);
    EXPECT_TRUE(elem->getName() == "my_link");
    EXPECT_TRUE(elem->inertial == nullptr);
    EXPECT_TRUE(elem->visual.size() == 1);
    EXPECT_TRUE(elem->collision.size() == 0);
  }

  {
    std::string str = "<link name=\"my_link\">"
                      "  <collision>"
                      "    <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>"
                      "    <geometry>"
                      "      <cylinder radius=\"1\" length=\"0.5\"/>"
                      "    </geometry>"
                      "  </collision>"
                      "</link>";
    tesseract_scene_graph::Link::Ptr elem;
    auto status = runTest<tesseract_scene_graph::Link::Ptr>(elem, str, "link", locateResource, empty_available_materials);
    EXPECT_TRUE(*status);
    EXPECT_TRUE(elem->getName() == "my_link");
    EXPECT_TRUE(elem->inertial == nullptr);
    EXPECT_TRUE(elem->visual.size() == 0);
    EXPECT_TRUE(elem->collision.size() == 1);
  }

  {
    std::string str = "<link name=\"my_link\"/>";
    tesseract_scene_graph::Link::Ptr elem;
    auto status = runTest<tesseract_scene_graph::Link::Ptr>(elem, str, "link", locateResource, empty_available_materials);
    EXPECT_TRUE(*status);
    EXPECT_TRUE(elem->getName() == "my_link");
    EXPECT_TRUE(elem->inertial == nullptr);
    EXPECT_TRUE(elem->visual.size() == 0);
    EXPECT_TRUE(elem->collision.size() == 0);
  }

  {
    std::string str = "<link >"
                      "  <visual>"
                      "    <origin xyz=\"0 0 0\" rpy=\"0 0 0\" />"
                      "    <geometry>"
                      "      <box size=\"1 1 1\" />"
                      "    </geometry>"
                      "    <material name=\"Cyan\">"
                      "      <color rgba=\"0 1.0 1.0 1.0\"/>"
                      "    </material>"
                      "  </visual>"
                      "</link>";
    tesseract_scene_graph::Link::Ptr elem;
    auto status = runTest<tesseract_scene_graph::Link::Ptr>(elem, str, "link", locateResource, empty_available_materials);
    EXPECT_FALSE(*status);
  }

  {
    std::string str = "<link name=\"my_link\">"
                      "  <visual>"
                      "    <origin xyz=\"0 0 0\" rpy=\"0 0 a\" />"
                      "    <geometry>"
                      "      <box size=\"1 1 1\" />"
                      "    </geometry>"
                      "    <material name=\"Cyan\">"
                      "      <color rgba=\"0 1.0 1.0 1.0\"/>"
                      "    </material>"
                      "  </visual>"
                      "</link>";
    tesseract_scene_graph::Link::Ptr elem;
    auto status = runTest<tesseract_scene_graph::Link::Ptr>(elem, str, "link", locateResource, empty_available_materials);
    EXPECT_FALSE(*status);
  }
}

TEST(TesseractURDFUnit, parse_joint)
{
  {
    std::string str = "<joint name=\"my_joint\" type=\"floating\">"
                      "  <origin xyz=\"0 0 1\" rpy=\"0 0 3.1416\"/>"
                      "  <parent link=\"link1\"/>"
                      "  <child link=\"link2\"/>"
                      "  <calibration rising=\"0.0\"/>"
                      "  <dynamics damping=\"0.0\" friction=\"0.0\"/>"
                      "  <limit effort=\"30\" velocity=\"1.0\" lower=\"-2.2\" upper=\"0.7\" />"
                      "  <safety_controller k_velocity=\"10\" k_position=\"15\" soft_lower_limit=\"-2.0\" soft_upper_limit=\"0.5\" />"
                      "</joint>";
    tesseract_scene_graph::Joint::Ptr elem;
    auto status = runTest<tesseract_scene_graph::Joint::Ptr>(elem, str, "joint");
    EXPECT_TRUE(*status);
    EXPECT_TRUE(elem->getName() == "my_joint");
    EXPECT_TRUE(elem->type == tesseract_scene_graph::JointType::FLOATING);
    EXPECT_FALSE(elem->parent_to_joint_origin_transform.isApprox(Eigen::Isometry3d::Identity(), 1e-8));
    EXPECT_TRUE(elem->axis.isApprox(Eigen::Vector3d(1,0,0), 1e-8));
    EXPECT_TRUE(elem->parent_link_name == "link1");
    EXPECT_TRUE(elem->child_link_name == "link2");
    EXPECT_TRUE(elem->calibration != nullptr);
    EXPECT_TRUE(elem->dynamics != nullptr);
    EXPECT_TRUE(elem->limits == nullptr);
    EXPECT_TRUE(elem->safety != nullptr);
    EXPECT_TRUE(elem->mimic == nullptr);
  }

  {
    std::string str = "<joint name=\"my_joint\" type=\"revolute\">"
                      "  <origin xyz=\"0 0 1\" rpy=\"0 0 3.1416\"/>"
                      "  <parent link=\"link1\"/>"
                      "  <child link=\"link2\"/>"
                      "  <calibration rising=\"0.0\"/>"
                      "  <dynamics damping=\"0.0\" friction=\"0.0\"/>"
                      "  <limit effort=\"30\" velocity=\"1.0\" lower=\"-2.2\" upper=\"0.7\" />"
                      "  <safety_controller k_velocity=\"10\" k_position=\"15\" soft_lower_limit=\"-2.0\" soft_upper_limit=\"0.5\" />"
                      "</joint>";
    tesseract_scene_graph::Joint::Ptr elem;
    auto status = runTest<tesseract_scene_graph::Joint::Ptr>(elem, str, "joint");
    EXPECT_TRUE(*status);
    EXPECT_TRUE(elem->getName() == "my_joint");
    EXPECT_TRUE(elem->type == tesseract_scene_graph::JointType::REVOLUTE);
    EXPECT_FALSE(elem->parent_to_joint_origin_transform.isApprox(Eigen::Isometry3d::Identity(), 1e-8));
    EXPECT_TRUE(elem->axis.isApprox(Eigen::Vector3d(1,0,0), 1e-8));
    EXPECT_TRUE(elem->parent_link_name == "link1");
    EXPECT_TRUE(elem->child_link_name == "link2");
    EXPECT_TRUE(elem->calibration != nullptr);
    EXPECT_TRUE(elem->dynamics != nullptr);
    EXPECT_TRUE(elem->limits != nullptr);
    EXPECT_TRUE(elem->safety != nullptr);
    EXPECT_TRUE(elem->mimic == nullptr);
  }

  {
    std::string str = "<joint name=\"my_joint\" type=\"revolute\">"
                      "  <axis xyz=\"0 0 1\"/>"
                      "  <parent link=\"link1\"/>"
                      "  <child link=\"link2\"/>"
                      "  <limit effort=\"30\" velocity=\"1.0\" lower=\"-2.2\" upper=\"0.7\" />"
                      "</joint>";
    tesseract_scene_graph::Joint::Ptr elem;
    auto status = runTest<tesseract_scene_graph::Joint::Ptr>(elem, str, "joint");
    EXPECT_TRUE(*status);
    EXPECT_TRUE(elem->getName() == "my_joint");
    EXPECT_TRUE(elem->type == tesseract_scene_graph::JointType::REVOLUTE);
    EXPECT_TRUE(elem->parent_to_joint_origin_transform.isApprox(Eigen::Isometry3d::Identity(), 1e-8));
    EXPECT_TRUE(elem->axis.isApprox(Eigen::Vector3d(0,0,1), 1e-8));
    EXPECT_TRUE(elem->parent_link_name == "link1");
    EXPECT_TRUE(elem->child_link_name == "link2");
    EXPECT_TRUE(elem->calibration == nullptr);
    EXPECT_TRUE(elem->dynamics == nullptr);
    EXPECT_TRUE(elem->limits != nullptr);
    EXPECT_TRUE(elem->safety == nullptr);
    EXPECT_TRUE(elem->mimic == nullptr);
  }

  {
    std::string str = "<joint name=\"my_joint\" type=\"continuous\">"
                      "  <axis xyz=\"0 0 1\"/>"
                      "  <parent link=\"link1\"/>"
                      "  <child link=\"link2\"/>"
                      "  <limit effort=\"30\" velocity=\"1.0\"/>"
                      "</joint>";
    tesseract_scene_graph::Joint::Ptr elem;
    auto status = runTest<tesseract_scene_graph::Joint::Ptr>(elem, str, "joint");
    EXPECT_TRUE(*status);
    EXPECT_TRUE(elem->getName() == "my_joint");
    EXPECT_TRUE(elem->type == tesseract_scene_graph::JointType::CONTINUOUS);
    EXPECT_TRUE(elem->parent_to_joint_origin_transform.isApprox(Eigen::Isometry3d::Identity(), 1e-8));
    EXPECT_TRUE(elem->axis.isApprox(Eigen::Vector3d(0,0,1), 1e-8));
    EXPECT_TRUE(elem->parent_link_name == "link1");
    EXPECT_TRUE(elem->child_link_name == "link2");
    EXPECT_TRUE(elem->calibration == nullptr);
    EXPECT_TRUE(elem->dynamics == nullptr);
    EXPECT_TRUE(elem->limits != nullptr);
    EXPECT_TRUE(elem->safety == nullptr);
    EXPECT_TRUE(elem->mimic == nullptr);
  }

  {
    std::string str = "<joint name=\"my_joint\" type=\"fixed\">"
                      "  <axis xyz=\"0 0 1\"/>"
                      "  <parent link=\"link1\"/>"
                      "  <child link=\"link2\"/>"
                      "  <limit effort=\"30\" velocity=\"1.0\" lower=\"-2.2\" upper=\"0.7\" />"
                      "</joint>";
    tesseract_scene_graph::Joint::Ptr elem;
    auto status = runTest<tesseract_scene_graph::Joint::Ptr>(elem, str, "joint");
    EXPECT_TRUE(*status);
    EXPECT_TRUE(elem->getName() == "my_joint");
    EXPECT_TRUE(elem->type == tesseract_scene_graph::JointType::FIXED);
    EXPECT_TRUE(elem->parent_to_joint_origin_transform.isApprox(Eigen::Isometry3d::Identity(), 1e-8));
    EXPECT_TRUE(elem->axis.isApprox(Eigen::Vector3d(1,0,0), 1e-8));
    EXPECT_TRUE(elem->parent_link_name == "link1");
    EXPECT_TRUE(elem->child_link_name == "link2");
    EXPECT_TRUE(elem->calibration == nullptr);
    EXPECT_TRUE(elem->dynamics == nullptr);
    EXPECT_TRUE(elem->limits == nullptr);
    EXPECT_TRUE(elem->safety == nullptr);
    EXPECT_TRUE(elem->mimic == nullptr);
  }

  {
    std::string str = "<joint name=\"my_joint\" type=\"prismatic\">"
                      "  <axis xyz=\"0 0 1\"/>"
                      "  <parent link=\"link1\"/>"
                      "  <child link=\"link2\"/>"
                      "  <limit effort=\"30\" velocity=\"1.0\" lower=\"-2.2\" upper=\"0.7\" />"
                      "</joint>";
    tesseract_scene_graph::Joint::Ptr elem;
    auto status = runTest<tesseract_scene_graph::Joint::Ptr>(elem, str, "joint");
    EXPECT_TRUE(*status);
    EXPECT_TRUE(elem->getName() == "my_joint");
    EXPECT_TRUE(elem->type == tesseract_scene_graph::JointType::PRISMATIC);
    EXPECT_TRUE(elem->parent_to_joint_origin_transform.isApprox(Eigen::Isometry3d::Identity(), 1e-8));
    EXPECT_TRUE(elem->axis.isApprox(Eigen::Vector3d(0,0,1), 1e-8));
    EXPECT_TRUE(elem->parent_link_name == "link1");
    EXPECT_TRUE(elem->child_link_name == "link2");
    EXPECT_TRUE(elem->calibration == nullptr);
    EXPECT_TRUE(elem->dynamics == nullptr);
    EXPECT_TRUE(elem->limits != nullptr);
    EXPECT_TRUE(elem->safety == nullptr);
    EXPECT_TRUE(elem->mimic == nullptr);
  }

  {
    std::string str = "<joint name=\"my_joint\" type=\"planar\">"
                      "  <axis xyz=\"0 0 1\"/>"
                      "  <parent link=\"link1\"/>"
                      "  <child link=\"link2\"/>"
                      "  <limit effort=\"30\" velocity=\"1.0\" lower=\"-2.2\" upper=\"0.7\" />"
                      "</joint>";
    tesseract_scene_graph::Joint::Ptr elem;
    auto status = runTest<tesseract_scene_graph::Joint::Ptr>(elem, str, "joint");
    EXPECT_TRUE(*status);
    EXPECT_TRUE(elem->getName() == "my_joint");
    EXPECT_TRUE(elem->type == tesseract_scene_graph::JointType::PLANAR);
    EXPECT_TRUE(elem->parent_to_joint_origin_transform.isApprox(Eigen::Isometry3d::Identity(), 1e-8));
    EXPECT_TRUE(elem->axis.isApprox(Eigen::Vector3d(0,0,1), 1e-8));
    EXPECT_TRUE(elem->parent_link_name == "link1");
    EXPECT_TRUE(elem->child_link_name == "link2");
    EXPECT_TRUE(elem->calibration == nullptr);
    EXPECT_TRUE(elem->dynamics == nullptr);
    EXPECT_TRUE(elem->limits == nullptr);
    EXPECT_TRUE(elem->safety == nullptr);
    EXPECT_TRUE(elem->mimic == nullptr);
  }

  {
    std::string str = "<joint name=\"my_joint\">"
                      "  <parent link=\"link1\"/>"
                      "  <child link=\"link2\"/>"
                      "</joint>";
    tesseract_scene_graph::Joint::Ptr elem;
    auto status = runTest<tesseract_scene_graph::Joint::Ptr>(elem, str, "joint");
    EXPECT_FALSE(*status);
  }

  {
    std::string str = "<joint type=\"planar\">"
                      "  <parent link=\"link1\"/>"
                      "  <child link=\"link2\"/>"
                      "</joint>";
    tesseract_scene_graph::Joint::Ptr elem;
    auto status = runTest<tesseract_scene_graph::Joint::Ptr>(elem, str, "joint");
    EXPECT_FALSE(*status);
  }

  {
    std::string str = "<joint name=\"my_joint\" type=\"planar\">"
                      "  <child link=\"link2\"/>"
                      "</joint>";
    tesseract_scene_graph::Joint::Ptr elem;
    auto status = runTest<tesseract_scene_graph::Joint::Ptr>(elem, str, "joint");
    EXPECT_FALSE(*status);
  }

  {
    std::string str = "<joint name=\"my_joint\" type=\"planar\">"
                      "  <parent link=\"link1\"/>"
                      "</joint>";
    tesseract_scene_graph::Joint::Ptr elem;
    auto status = runTest<tesseract_scene_graph::Joint::Ptr>(elem, str, "joint");
    EXPECT_FALSE(*status);
  }
}

TEST(TesseractURDFUnit, parse_urdf)
{
  {
    std::string str =
        "<robot name=\"test\">"
        "  <joint name=\"j1\" type=\"fixed\">"
        "    <parent link=\"l1\"/>"
        "    <child link=\"l2\"/>"
        "    <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>"
        "    <dynamics damping=\"87.098\" friction=\"3.1290\"/>"
        "    <limit lower=\"12.34\" upper=\"22.999\" effort=\"99.0\" velocity=\"23.0\"/>"
        "    <safety_controller soft_lower_limit=\"8.765\" soft_upper_limit=\"9.003\" k_position=\"7.0034\" k_velocity=\"9.998\"/>"
        "    <calibration rising=\"8.654\" falling=\"0.0445\"/>"
        "    <mimic joint=\"j2\" multiplier=\"9.87\" offset=\"0.098\"/>"
        "  </joint>"
        "  <link name=\"l1\"/>"
        "  <link name=\"l2\"/>"
        "</robot>";
    tesseract_scene_graph::SceneGraph::Ptr sg;
    auto status = parseURDFString(sg, str, locateResource);
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
        "<robot name=\"test\">"
        "  <joint name=\"j1\" type=\"fixed\">"
        "    <parent link=\"l1\"/>"
        "    <child link=\"l2\"/>"
        "    <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>"
        "    <dynamics damping=\"87.098\" friction=\"3.1290\"/>"
        "    <limit lower=\"12.34\" upper=\"22.999\" effort=\"99.0\" velocity=\"23.0\"/>"
        "    <safety_controller soft_lower_limit=\"8.765\" soft_upper_limit=\"9.003\" k_position=\"7.0034\" k_velocity=\"9.998\"/>"
        "    <calibration rising=\"8.654\" falling=\"0.0445\"/>"
        "    <mimic joint=\"j2\" multiplier=\"9.87\" offset=\"0.098\"/>"
        "  </joint>"
        "  <joint name=\"j2\" type=\"fixed\">"
        "    <parent link=\"l1\"/>"
        "    <child link=\"l2\"/>"
        "  </joint>"
        "  <link name=\"l1\"/>"
        "  <link name=\"l2\"/>"
        "</robot>";
    tesseract_scene_graph::SceneGraph::Ptr sg;
    auto status = parseURDFString(sg, str, locateResource);
    EXPECT_FALSE(*status);
    EXPECT_TRUE(sg == nullptr);
    CONSOLE_BRIDGE_logError(status->message().c_str());
  }

  {
    std::string str =
        "<robot name=\"test\">"
        "  <joint name=\"j1\" type=\"fixed\">"
        "    <parent link=\"l1\"/>"
        "    <child link=\"l2\"/>"
        "    <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>"
        "    <dynamics damping=\"87.098\" friction=\"3.1290\"/>"
        "    <limit lower=\"12.34\" upper=\"22.999\" effort=\"99.0\" velocity=\"23.0\"/>"
        "    <safety_controller soft_lower_limit=\"8.765\" soft_upper_limit=\"9.003\" k_position=\"7.0034\" k_velocity=\"9.998\"/>"
        "    <calibration rising=\"8.654\" falling=\"0.0445\"/>"
        "    <mimic joint=\"j2\" multiplier=\"9.87\" offset=\"0.098\"/>"
        "  </joint>"
        "  <link name=\"l1\"/>"
        "  <link name=\"l3\"/>"
        "</robot>";
    tesseract_scene_graph::SceneGraph::Ptr sg;
    auto status = parseURDFString(sg, str, locateResource);
    EXPECT_FALSE(*status);
    EXPECT_TRUE(sg == nullptr);
    CONSOLE_BRIDGE_logError(status->message().c_str());
  }

  {
    std::string str =
        "<robot name=\"test\">"
        "  <joint name=\"j1\" type=\"fixed\">"
        "    <parent link=\"l1\"/>"
        "    <child link=\"l2\"/>"
        "    <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>"
        "    <dynamics damping=\"87.098\" friction=\"3.1290\"/>"
        "    <limit lower=\"12.34\" upper=\"22.999\" effort=\"99.0\" velocity=\"23.0\"/>"
        "    <safety_controller soft_lower_limit=\"8.765\" soft_upper_limit=\"9.003\" k_position=\"7.0034\" k_velocity=\"9.998\"/>"
        "    <calibration rising=\"8.654\" falling=\"0.0445\"/>"
        "    <mimic joint=\"j2\" multiplier=\"9.87\" offset=\"0.098\"/>"
        "  </joint>"
        "  <link name=\"l1\"/>"
        "  <link name=\"l2\"/>"
        "  <link name=\"l3\"/>"
        "</robot>";
    tesseract_scene_graph::SceneGraph::Ptr sg;
    auto status = parseURDFString(sg, str, locateResource);
    EXPECT_FALSE(*status);
    EXPECT_TRUE(sg == nullptr);
    CONSOLE_BRIDGE_logError(status->message().c_str());
  }

  {
    std::string str =
        "<robot name=\"test\">"
        "  <joint name=\"j1\" type=\"fixed\">"
        "    <parent link=\"l1\"/>"
        "    <child link=\"l3\"/>"
        "    <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>"
        "    <dynamics damping=\"87.098\" friction=\"3.1290\"/>"
        "    <limit lower=\"12.34\" upper=\"22.999\" effort=\"99.0\" velocity=\"23.0\"/>"
        "    <safety_controller soft_lower_limit=\"8.765\" soft_upper_limit=\"9.003\" k_position=\"7.0034\" k_velocity=\"9.998\"/>"
        "    <calibration rising=\"8.654\" falling=\"0.0445\"/>"
        "    <mimic joint=\"j2\" multiplier=\"9.87\" offset=\"0.098\"/>"
        "  </joint>"
        "  <link name=\"l1\"/>"
        "  <link name=\"l2\"/>"
        "</robot>";
    tesseract_scene_graph::SceneGraph::Ptr sg;
    auto status = parseURDFString(sg, str, locateResource);
    EXPECT_FALSE(*status);
    EXPECT_TRUE(sg == nullptr);
    CONSOLE_BRIDGE_logError(status->message().c_str());
  }

  {
    std::string str =
        "<robot name=\"test\">"
        "  <joint name=\"j1\" type=\"fixed\">"
        "    <parent link=\"l1\"/>"
        "    <child link=\"l2\"/>"
        "    <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>"
        "    <dynamics damping=\"87.098\" friction=\"3.1290\"/>"
        "    <limit lower=\"12.34\" upper=\"22.999\" effort=\"99.0\" velocity=\"23.0\"/>"
        "    <safety_controller soft_lower_limit=\"8.765\" soft_upper_limit=\"9.003\" k_position=\"7.0034\" k_velocity=\"9.998\"/>"
        "    <calibration rising=\"8.654\" falling=\"0.0445\"/>"
        "    <mimic joint=\"j2\" multiplier=\"9.87\" offset=\"0.098\"/>"
        "  </joint>"
        "  <link name=\"l1\"/>"
        "  <link name=\"l2\"/>"
        "  <link name=\"l1\"/>"
        "</robot>";
    tesseract_scene_graph::SceneGraph::Ptr sg;
    auto status = parseURDFString(sg, str, locateResource);
    EXPECT_FALSE(*status);
    EXPECT_TRUE(sg == nullptr);
    CONSOLE_BRIDGE_logError(status->message().c_str());
  }

  {
    std::string str =
        "<robot>"
        "  <joint name=\"j1\" type=\"fixed\">"
        "    <parent link=\"l1\"/>"
        "    <child link=\"l2\"/>"
        "    <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>"
        "    <dynamics damping=\"87.098\" friction=\"3.1290\"/>"
        "    <limit lower=\"12.34\" upper=\"22.999\" effort=\"99.0\" velocity=\"23.0\"/>"
        "    <safety_controller soft_lower_limit=\"8.765\" soft_upper_limit=\"9.003\" k_position=\"7.0034\" k_velocity=\"9.998\"/>"
        "    <calibration rising=\"8.654\" falling=\"0.0445\"/>"
        "    <mimic joint=\"j2\" multiplier=\"9.87\" offset=\"0.098\"/>"
        "  </joint>"
        "  <link name=\"l1\"/>"
        "  <link name=\"l2\"/>"
        "</robot>";
    tesseract_scene_graph::SceneGraph::Ptr sg;
    auto status = parseURDFString(sg, str, locateResource);
    EXPECT_FALSE(*status);
    EXPECT_TRUE(sg == nullptr);
    CONSOLE_BRIDGE_logError(status->message().c_str());
  }

  {
    std::string str =
        "<robot name=\"test\">"
        "  <joint name=\"j1\" type=\"fixed\">"
        "    <parent link=\"l2\"/>"
        "    <child link=\"l3\"/>"
        "    <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>"
        "    <dynamics damping=\"87.098\" friction=\"3.1290\"/>"
        "    <limit lower=\"12.34\" upper=\"22.999\" effort=\"99.0\" velocity=\"23.0\"/>"
        "    <safety_controller soft_lower_limit=\"8.765\" soft_upper_limit=\"9.003\" k_position=\"7.0034\" k_velocity=\"9.998\"/>"
        "    <calibration rising=\"8.654\" falling=\"0.0445\"/>"
        "    <mimic joint=\"j2\" multiplier=\"9.87\" offset=\"0.098\"/>"
        "  </joint>"
        "  <joint name=\"j1\" type=\"fixed\">"
        "    <parent link=\"l1\"/>"
        "    <child link=\"l2\"/>"
        "  </joint>"
        "  <link name=\"l1\"/>"
        "  <link name=\"l2\"/>"
        "  <link name=\"l3\"/>"
        "</robot>";
    tesseract_scene_graph::SceneGraph::Ptr sg;
    auto status = parseURDFString(sg, str, locateResource);
    EXPECT_FALSE(*status);
    EXPECT_TRUE(sg == nullptr);
    CONSOLE_BRIDGE_logError(status->message().c_str());
  }

  {
    tesseract_scene_graph::SceneGraph::Ptr sg;
    auto status = parseURDFFile(sg, std::string(TESSERACT_SUPPORT_DIR) + "/urdf/lbr_iiwa_14_r820.urdf", locateResource);
    EXPECT_TRUE(*status);
  }
}

TEST(TesseractURDFUnit, LoadURDFUnit)
{
  using namespace tesseract_scene_graph;

  std::string urdf_file = std::string(TESSERACT_SUPPORT_DIR) + "/urdf/lbr_iiwa_14_r820.urdf";

  ResourceLocatorFn locator = locateResource;
  SceneGraph::Ptr g = parseURDFFile(urdf_file, locator);

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

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  // use the environment locale so that the unit test can be repeated with various locales easily
  setlocale(LC_ALL, "");

  return RUN_ALL_TESTS();
}
