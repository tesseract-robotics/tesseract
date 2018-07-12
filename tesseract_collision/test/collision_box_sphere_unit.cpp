
#include "tesseract_collision/bullet/bullet_discrete_managers.h"
#include <gtest/gtest.h>
#include <ros/ros.h>

//void createConvexHull(const shapes::Shape* shape)
//{

//  const shapes::Mesh* mesh = static_cast<const shapes::Mesh*>(shape);

//  double maxX = -std::numeric_limits<double>::infinity(), maxY = -std::numeric_limits<double>::infinity(),
//         maxZ = -std::numeric_limits<double>::infinity();
//  double minX = std::numeric_limits<double>::infinity(), minY = std::numeric_limits<double>::infinity(),
//         minZ = std::numeric_limits<double>::infinity();

//  for (unsigned int i = 0; i < mesh->vertex_count; ++i)
//  {
//    double vx = mesh->vertices[3 * i];
//    double vy = mesh->vertices[3 * i + 1];
//    double vz = mesh->vertices[3 * i + 2];

//    if (maxX < vx)
//      maxX = vx;
//    if (maxY < vy)
//      maxY = vy;
//    if (maxZ < vz)
//      maxZ = vz;

//    if (minX > vx)
//      minX = vx;
//    if (minY > vy)
//      minY = vy;
//    if (minZ > vz)
//      minZ = vz;
//  }

//  if (maxX < minX)
//    maxX = minX = 0.0;
//  if (maxY < minY)
//    maxY = minY = 0.0;
//  if (maxZ < minZ)
//    maxZ = minZ = 0.0;

//  mesh_data_->box_size_ = Eigen::Vector3d(maxX - minX, maxY - minY, maxZ - minZ);

//  mesh_data_->box_offset_ = Eigen::Vector3d((minX + maxX) / 2.0, (minY + maxY) / 2.0, (minZ + maxZ) / 2.0);

//  mesh_data_->planes_.clear();
//  mesh_data_->triangles_.clear();
//  mesh_data_->vertices_.clear();
//  mesh_data_->mesh_radiusB_ = 0.0;
//  mesh_data_->mesh_center_ = Eigen::Vector3d();

//  double xdim = maxX - minX;
//  double ydim = maxY - minY;
//  double zdim = maxZ - minZ;

//  double pose1;
//  double pose2;

//  unsigned int off1;
//  unsigned int off2;

//  /* compute bounding cylinder */
//  double cyl_length;
//  double maxdist = -std::numeric_limits<double>::infinity();
//  if (xdim > ydim && xdim > zdim)
//  {
//    off1 = 1;
//    off2 = 2;
//    pose1 = mesh_data_->box_offset_.y();
//    pose2 = mesh_data_->box_offset_.z();
//    cyl_length = xdim;
//  }
//  else if (ydim > zdim)
//  {
//    off1 = 0;
//    off2 = 2;
//    pose1 = mesh_data_->box_offset_.x();
//    pose2 = mesh_data_->box_offset_.z();
//    cyl_length = ydim;
//  }
//  else
//  {
//    off1 = 0;
//    off2 = 1;
//    pose1 = mesh_data_->box_offset_.x();
//    pose2 = mesh_data_->box_offset_.y();
//    cyl_length = zdim;
//  }

//  /* compute convex hull */
//  coordT* points = (coordT*)calloc(mesh->vertex_count * 3, sizeof(coordT));
//  for (unsigned int i = 0; i < mesh->vertex_count; ++i)
//  {
//    points[3 * i + 0] = (coordT)mesh->vertices[3 * i + 0];
//    points[3 * i + 1] = (coordT)mesh->vertices[3 * i + 1];
//    points[3 * i + 2] = (coordT)mesh->vertices[3 * i + 2];

//    double dista = mesh->vertices[3 * i + off1] - pose1;
//    double distb = mesh->vertices[3 * i + off2] - pose2;
//    double dist = sqrt(((dista * dista) + (distb * distb)));
//    if (dist > maxdist)
//      maxdist = dist;
//  }
//  mesh_data_->bounding_cylinder_.radius = maxdist;
//  mesh_data_->bounding_cylinder_.length = cyl_length;

//  static FILE* null = fopen("/dev/null", "w");

//  char flags[] = "qhull Tv Qt";
//  int exitcode = qh_new_qhull(3, mesh->vertex_count, points, true, flags, null, null);

//  if (exitcode != 0)
//  {
//    CONSOLE_BRIDGE_logWarn("Convex hull creation failed");
//    qh_freeqhull(!qh_ALL);
//    int curlong, totlong;
//    qh_memfreeshort(&curlong, &totlong);
//    return;
//  }

//  int num_facets = qh num_facets;

//  int num_vertices = qh num_vertices;
//  mesh_data_->vertices_.reserve(num_vertices);
//  Eigen::Vector3d sum(0, 0, 0);

//  // necessary for FORALLvertices
//  std::map<unsigned int, unsigned int> qhull_vertex_table;
//  vertexT* vertex;
//  FORALLvertices
//  {
//    Eigen::Vector3d vert(vertex->point[0], vertex->point[1], vertex->point[2]);
//    qhull_vertex_table[vertex->id] = mesh_data_->vertices_.size();
//    sum += vert;
//    mesh_data_->vertices_.push_back(vert);
//  }

//  mesh_data_->mesh_center_ = sum / (double)(num_vertices);
//  for (unsigned int j = 0; j < mesh_data_->vertices_.size(); ++j)
//  {
//    double dist = (mesh_data_->vertices_[j] - mesh_data_->mesh_center_).squaredNorm();
//    if (dist > mesh_data_->mesh_radiusB_)
//      mesh_data_->mesh_radiusB_ = dist;
//  }

//  mesh_data_->mesh_radiusB_ = sqrt(mesh_data_->mesh_radiusB_);
//  mesh_data_->triangles_.reserve(num_facets);

//  // neccessary for qhull macro
//  facetT* facet;
//  FORALLfacets
//  {
//    Eigen::Vector4d planeEquation(facet->normal[0], facet->normal[1], facet->normal[2], facet->offset);
//    if (!mesh_data_->planes_.empty())
//    {
//      // filter equal planes - assuming same ones follow each other
//      if ((planeEquation - mesh_data_->planes_.back()).cwiseAbs().maxCoeff() > 1e-6)  // max diff to last
//        mesh_data_->planes_.push_back(planeEquation);
//    }
//    else
//    {
//      mesh_data_->planes_.push_back(planeEquation);
//    }

//    // Needed by FOREACHvertex_i_
//    int vertex_n, vertex_i;
//    FOREACHvertex_i_((*facet).vertices)
//    {
//      mesh_data_->triangles_.push_back(qhull_vertex_table[vertex->id]);
//    }

//    mesh_data_->plane_for_triangle_[(mesh_data_->triangles_.size() - 1) / 3] = mesh_data_->planes_.size() - 1;
//  }
//  qh_freeqhull(!qh_ALL);
//  int curlong, totlong;
//  qh_memfreeshort(&curlong, &totlong);
//}

void addCollisionObjects(tesseract::DiscreteContactManagerBase &checker, bool use_convex_mesh = false)
{
  //////////////////////
  // Add box to checker
  //////////////////////
  shapes::ShapePtr box(new shapes::Box(1, 1, 1));
  Eigen::Affine3d box_pose;
  box_pose.setIdentity();

  std::vector<shapes::ShapeConstPtr> obj1_shapes;
  EigenSTL::vector_Affine3d obj1_poses;
  tesseract::CollisionObjectTypeVector obj1_types;
  obj1_shapes.push_back(box);
  obj1_poses.push_back(box_pose);
  obj1_types.push_back(tesseract::CollisionObjectType::UseShapeType);

  checker.addCollisionObject("box_link", 0, obj1_shapes, obj1_poses, obj1_types);

  /////////////////////////////////////////////
  // Add thin box to checker which is disabled
  /////////////////////////////////////////////
  shapes::ShapePtr thin_box(new shapes::Box(0.1, 1, 1));
  Eigen::Affine3d thin_box_pose;
  thin_box_pose.setIdentity();

  std::vector<shapes::ShapeConstPtr> obj2_shapes;
  EigenSTL::vector_Affine3d obj2_poses;
  tesseract::CollisionObjectTypeVector obj2_types;
  obj2_shapes.push_back(thin_box);
  obj2_poses.push_back(thin_box_pose);
  obj2_types.push_back(tesseract::CollisionObjectType::UseShapeType);

  checker.addCollisionObject("thin_box_link", 0, obj2_shapes, obj2_poses, obj2_types, false);

  /////////////////////////////////////////////////////////////////
  // Add sphere to checker. If use_convex_mesh = true then this
  // sphere will be added as a convex hull mesh.
  /////////////////////////////////////////////////////////////////
  shapes::ShapePtr sphere;
  if (use_convex_mesh)
    sphere.reset(shapes::createMeshFromResource("package://tesseract_collision/test/sphere_p25m.stl"));
  else
    sphere.reset(new shapes::Sphere(0.25));

  Eigen::Affine3d sphere_pose;
  sphere_pose.setIdentity();

  std::vector<shapes::ShapeConstPtr> obj3_shapes;
  EigenSTL::vector_Affine3d obj3_poses;
  tesseract::CollisionObjectTypeVector obj3_types;
  obj3_shapes.push_back(sphere);
  obj3_poses.push_back(sphere_pose);

  if (use_convex_mesh)
    obj3_types.push_back(tesseract::CollisionObjectType::ConvexHull);
  else
    obj3_types.push_back(tesseract::CollisionObjectType::UseShapeType);

  checker.addCollisionObject("sphere_link", 0, obj3_shapes, obj3_poses, obj3_types);
}

void runTest(tesseract::DiscreteContactManagerBase &checker)
{
  //////////////////////////////////////
  // Test when object is in collision
  //////////////////////////////////////
  tesseract::ContactRequest req;
  req.link_names.push_back("box_link");
  req.link_names.push_back("sphere_link");
  req.contact_distance = 0.1;
  req.type = tesseract::ContactRequestType::CLOSEST;
  checker.setContactRequest(req);

  // Set the collision object transforms
  tesseract::TransformMap location;
  location["box_link"] = Eigen::Affine3d::Identity();
  location["sphere_link"] = Eigen::Affine3d::Identity();
  location["sphere_link"].translation()(0) = 0.2;
  checker.setCollisionObjectsTransform(location);

  // Perform collision check
  tesseract::ContactResultMap result;
  checker.contactTest(result);

  tesseract::ContactResultVector result_vector;
  tesseract::moveContactResultsMapToContactResultsVector(result, result_vector);

  EXPECT_TRUE(!result_vector.empty());
  EXPECT_NEAR(result_vector[0].distance, -0.55, 0.0001);

  std::vector<int> idx = {0, 1, 1};
  if (result_vector[0].link_names[0] != "box_link")
    idx = {1, 0, -1};

  EXPECT_NEAR(result_vector[0].nearest_points[idx[0]][0], 0.5, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[idx[0]][1], 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[idx[0]][2], 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[idx[1]][0], -0.05, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[idx[1]][1], 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[idx[1]][2], 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].normal[0], idx[2] * 1.0, 0.001);
  EXPECT_NEAR(result_vector[0].normal[1], idx[2] * 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].normal[2], idx[2] * 0.0, 0.001);

  ////////////////////////////////////////////////
  // Test object is out side the contact distance
  ////////////////////////////////////////////////
  location["sphere_link"].translation() = Eigen::Vector3d(1, 0, 0);
  result.clear();
  result_vector.clear();
  checker.setCollisionObjectsTransform(location);

  checker.contactTest(result);
  tesseract::moveContactResultsMapToContactResultsVector(result, result_vector);

  EXPECT_TRUE(result_vector.empty());

  /////////////////////////////////////////////
  // Test object inside the contact distance
  /////////////////////////////////////////////
  result.clear();
  result_vector.clear();
  req.contact_distance = 0.27; //0.251;
  checker.setContactRequest(req);

  checker.contactTest(result);
  tesseract::moveContactResultsMapToContactResultsVector(result, result_vector);

  EXPECT_TRUE(!result_vector.empty());
  EXPECT_NEAR(result_vector[0].distance, 0.25, 0.0001);

  idx = {0, 1, 1};
  if (result_vector[0].link_names[0] != "box_link")
    idx = {1, 0, -1};

  EXPECT_NEAR(result_vector[0].nearest_points[idx[0]][0], 0.5, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[idx[0]][1], 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[idx[0]][2], 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[idx[1]][0], 0.75, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[idx[1]][1], 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[idx[1]][2], 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].normal[0], idx[2] * 1.0, 0.001);
  EXPECT_NEAR(result_vector[0].normal[1], idx[2] * 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].normal[2], idx[2] * 0.0, 0.001);
}

TEST(TesseractCollisionUnit, BulletDiscreteSimpleCollisionBoxSphereUnit)
{
  tesseract::BulletDiscreteSimpleManager checker;
  addCollisionObjects(checker);
  runTest(checker);
}

//TEST(TesseractCollisionUnit, BulletDiscreteSimpleCollisionBoxSphereConvexHullUnit)
//{
//  tesseract::BulletDiscreteSimpleManager checker;
//  addCollisionObjects(checker, true);
//  runTest(checker);
//}

TEST(TesseractCollisionUnit, BulletDiscreteBVHCollisionBoxSphereUnit)
{
  tesseract::BulletDiscreteBVHManager checker;
  addCollisionObjects(checker);
  runTest(checker);
}

//TEST(TesseractCollisionUnit, BulletDiscreteBVHCollisionBoxSphereConvexHullUnit)
//{
//  tesseract::BulletDiscreteBVHManager checker;
//  addCollisionObjects(checker, true);
//  runTest(checker);
//}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
