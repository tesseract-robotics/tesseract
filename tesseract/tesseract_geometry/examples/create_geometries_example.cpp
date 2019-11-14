#include <console_bridge/console.h>
#include <tesseract_geometry/geometries.h>
#include <iostream>

using namespace tesseract_geometry;

int main(int /*argc*/, char** /*argv*/)
{
  // Shape Box
  auto box = std::make_shared<tesseract_geometry::Box>(1, 1, 1);
  // Shape Cone
  auto cone = std::make_shared<tesseract_geometry::Cone>(1, 1);
  // Shape Capsule
  auto capsule = std::make_shared<tesseract_geometry::Capsule>(1, 1);
  // Shape Cylinder
  auto cylinder = std::make_shared<tesseract_geometry::Cylinder>(1, 1);
  // Shape Plane
  auto plane = std::make_shared<tesseract_geometry::Plane>(1, 1, 1, 1);
  // Shape Sphere
  auto sphere = std::make_shared<tesseract_geometry::Sphere>(1);

  // Manually create mesh
  std::shared_ptr<const tesseract_common::VectorVector3d> mesh_vertices =
      std::make_shared<const tesseract_common::VectorVector3d>();
  std::shared_ptr<const Eigen::VectorXi> mesh_faces = std::make_shared<const Eigen::VectorXi>();
  // Next fill out vertices and triangles
  auto mesh = std::make_shared<tesseract_geometry::Mesh>(mesh_vertices, mesh_faces);

  // Manually create signed distance field mesh
  std::shared_ptr<const tesseract_common::VectorVector3d> sdf_vertices =
      std::make_shared<const tesseract_common::VectorVector3d>();
  std::shared_ptr<const Eigen::VectorXi> sdf_faces = std::make_shared<const Eigen::VectorXi>();
  // Next fill out vertices and triangles
  auto sdf_mesh = std::make_shared<tesseract_geometry::SDFMesh>(sdf_vertices, sdf_faces);

  // Manually create convex mesh
  std::shared_ptr<const tesseract_common::VectorVector3d> convex_vertices =
      std::make_shared<const tesseract_common::VectorVector3d>();
  std::shared_ptr<const Eigen::VectorXi> convex_faces = std::make_shared<const Eigen::VectorXi>();
  // Next fill out vertices and triangles
  auto convex_mesh = std::make_shared<tesseract_geometry::ConvexMesh>(convex_vertices, convex_faces);

  // Create an octree
  std::shared_ptr<const octomap::OcTree> octree;
  auto octree_t = std::make_shared<tesseract_geometry::Octree>(octree, tesseract_geometry::Octree::SubType::BOX);
}
