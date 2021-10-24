#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <random>
#include <chrono>
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_collision/bullet/bullet_discrete_simple_manager.h>
#include <tesseract_collision/bullet/bullet_discrete_bvh_manager.h>
#include <tesseract_collision/bullet/bullet_cast_simple_manager.h>
#include <tesseract_collision/bullet/bullet_cast_bvh_manager.h>
#include <tesseract_collision/bullet/convex_hull_utils.h>
#include <tesseract_collision/fcl/fcl_discrete_managers.h>

static const std::size_t DIM = 10;

using namespace tesseract_collision;
using namespace tesseract_geometry;

void addCollisionObjects(DiscreteContactManager& checker, bool use_single_link, bool use_convex_mesh)
{
  /////////////////////////////////////////////////////////////////
  // Add sphere to checker. If use_convex_mesh = true then this
  // sphere will be added as a convex hull mesh.
  /////////////////////////////////////////////////////////////////
  CollisionShapePtr sphere;

  if (use_convex_mesh)
  {
    tesseract_common::VectorVector3d mesh_vertices;
    Eigen::VectorXi mesh_faces;
    loadSimplePlyFile(std::string(TESSERACT_SUPPORT_DIR) + "/meshes/sphere_p25m.ply", mesh_vertices, mesh_faces);

    // This is required because convex hull cannot have multiple faces on the same plane.
    auto ch_verticies = std::make_shared<tesseract_common::VectorVector3d>();
    auto ch_faces = std::make_shared<Eigen::VectorXi>();
    int ch_num_faces = createConvexHull(*ch_verticies, *ch_faces, mesh_vertices);
    sphere = std::make_shared<ConvexMesh>(ch_verticies, ch_faces, ch_num_faces);
  }
  else
  {
    sphere = std::make_shared<Sphere>(0.25);
  }

  Eigen::Isometry3d sphere_pose;
  sphere_pose.setIdentity();

  CollisionShapesConst obj3_shapes;
  tesseract_common::VectorIsometry3d obj3_poses;
  obj3_shapes.push_back(sphere);
  obj3_poses.push_back(sphere_pose);
  checker.addCollisionObject("move_link", 0, obj3_shapes, obj3_poses);

  CollisionShapesConst link_shapes;
  tesseract_common::VectorIsometry3d link_poses;
  for (std::size_t x = 0; x < DIM; ++x)
  {
    for (std::size_t y = 0; y < DIM; ++y)
    {
      for (std::size_t z = 0; z < DIM; ++z)
      {
        Eigen::Isometry3d sphere_pose;
        sphere_pose.setIdentity();
        sphere_pose.translation() = Eigen::Vector3d(double(x), double(y), double(z));

        if (use_single_link)
        {
          link_shapes.push_back(sphere);
          link_poses.push_back(sphere_pose);
        }
        else
        {
          CollisionShapesConst shapes;
          tesseract_common::VectorIsometry3d poses;
          shapes.push_back(sphere);
          poses.push_back(sphere_pose);
          std::string link_name = "sphere_link_" + std::to_string(x) + std::to_string(y) + std::to_string(z);
          checker.addCollisionObject(link_name, 0, shapes, poses);
        }
      }
    }
  }
  if (use_single_link)
    checker.addCollisionObject("sphere_link", 0, link_shapes, link_poses);

  checker.setActiveCollisionObjects({ "move_link" });
}

void addCollisionObjects(ContinuousContactManager& checker, bool use_single_link, bool use_convex_mesh)
{
  /////////////////////////////////////////////////////////////////
  // Add sphere to checker. If use_convex_mesh = true then this
  // sphere will be added as a convex hull mesh.
  /////////////////////////////////////////////////////////////////
  CollisionShapePtr sphere;

  if (use_convex_mesh)
  {
    tesseract_common::VectorVector3d mesh_vertices;
    Eigen::VectorXi mesh_faces;
    loadSimplePlyFile(std::string(TESSERACT_SUPPORT_DIR) + "/meshes/sphere_p25m.ply", mesh_vertices, mesh_faces);

    // This is required because convex hull cannot have multiple faces on the same plane.
    auto ch_verticies = std::make_shared<tesseract_common::VectorVector3d>();
    auto ch_faces = std::make_shared<Eigen::VectorXi>();
    int ch_num_faces = createConvexHull(*ch_verticies, *ch_faces, mesh_vertices);
    sphere = std::make_shared<ConvexMesh>(ch_verticies, ch_faces, ch_num_faces);
  }
  else
  {
    sphere = std::make_shared<Sphere>(0.25);
  }

  Eigen::Isometry3d sphere_pose;
  sphere_pose.setIdentity();

  CollisionShapesConst obj3_shapes;
  tesseract_common::VectorIsometry3d obj3_poses;
  obj3_shapes.push_back(sphere);
  obj3_poses.push_back(sphere_pose);
  checker.addCollisionObject("move_link", 0, obj3_shapes, obj3_poses);

  CollisionShapesConst link_shapes;
  tesseract_common::VectorIsometry3d link_poses;
  for (std::size_t x = 0; x < DIM; ++x)
  {
    for (std::size_t y = 0; y < DIM; ++y)
    {
      for (std::size_t z = 0; z < DIM; ++z)
      {
        Eigen::Isometry3d sphere_pose;
        sphere_pose.setIdentity();
        sphere_pose.translation() = Eigen::Vector3d(double(x), double(y), double(z));

        if (use_single_link)
        {
          link_shapes.push_back(sphere);
          link_poses.push_back(sphere_pose);
        }
        else
        {
          CollisionShapesConst shapes;
          tesseract_common::VectorIsometry3d poses;
          shapes.push_back(sphere);
          poses.push_back(sphere_pose);
          std::string link_name = "sphere_link_" + std::to_string(x) + std::to_string(y) + std::to_string(z);
          checker.addCollisionObject(link_name, 0, shapes, poses);
        }
      }
    }
  }
  if (use_single_link)
    checker.addCollisionObject("sphere_link", 0, link_shapes, link_poses);

  checker.setActiveCollisionObjects({ "move_link" });
}

std::vector<Eigen::Isometry3d> getTransforms(std::size_t num_poses)
{
  std::vector<Eigen::Isometry3d> poses(num_poses);
  for (std::size_t i = 0; i < num_poses; ++i)
  {
    double x = (double(rand()) / RAND_MAX) * double(DIM);
    double y = (double(rand()) / RAND_MAX) * double(DIM);
    double z = (double(rand()) / RAND_MAX) * double(DIM);
    poses[i] = Eigen::Isometry3d::Identity();
    poses[i].translation() = Eigen::Vector3d(x, y, z);
  }
  return poses;
}

void runDiscreteProfile(bool use_single_link, bool use_convex_mesh, double contact_distance)
{
  auto bt_simple_checker = std::make_shared<tesseract_collision_bullet::BulletDiscreteSimpleManager>();
  auto bt_bvh_checker = std::make_shared<tesseract_collision_bullet::BulletDiscreteBVHManager>();
  auto fcl_bvh_checker = std::make_shared<tesseract_collision_fcl::FCLDiscreteBVHManager>();

  std::vector<Eigen::Isometry3d> poses = getTransforms(50);
  std::vector<DiscreteContactManager::Ptr> checkers = { bt_simple_checker, bt_bvh_checker, fcl_bvh_checker };
  std::vector<std::string> checker_names = { "BtSimple", "BtBVH", "FCLBVH" };
  std::vector<std::size_t> checker_contacts = { 0, 0, 0 };

  std::printf("Total number of shape: %d\n", int(DIM * DIM * DIM));
  //  for (std::size_t i = 0; i < checkers.size(); ++i)
  for (std::size_t i = 0; i < 2; ++i)
  {
    addCollisionObjects(*checkers[i], use_single_link, use_convex_mesh);
    checkers[i]->setCollisionMarginData(CollisionMarginData(contact_distance));

    auto start_time = std::chrono::high_resolution_clock::now();
    for (auto& pose : poses)
    {
      checkers[i]->setCollisionObjectsTransform("move_link", pose);

      // Perform collision check
      ContactResultMap result;
      checkers[i]->contactTest(result, ContactTestType::FIRST);
      checker_contacts[i] += result.size();
    }
    auto end_time = std::chrono::high_resolution_clock::now();
    double total_time = std::chrono::duration<double, std::milli>(end_time - start_time).count();

    std::printf(
        "%15s | DT: %15.5f ms | Contacts: %d\n", checker_names[i].c_str(), total_time, int(checker_contacts[i]));
  }
}

void runContinuousProfile(bool use_single_link, bool use_convex_mesh, double contact_distance)
{
  auto bt_simple_checker = std::make_shared<tesseract_collision_bullet::BulletCastSimpleManager>();
  auto bt_bvh_checker = std::make_shared<tesseract_collision_bullet::BulletCastBVHManager>();

  std::vector<Eigen::Isometry3d> poses = getTransforms(50);
  std::vector<ContinuousContactManager::Ptr> checkers = { bt_simple_checker, bt_bvh_checker };
  std::vector<std::string> checker_names = { "BtCastSimple", "BtCastBVH" };
  std::vector<std::size_t> checker_contacts = { 0, 0, 0 };

  Eigen::Isometry3d delta_pose;
  delta_pose.setIdentity();
  delta_pose.translation() = Eigen::Vector3d(0.5, 0.5, 0.5);

  std::printf("Total number of shape: %d\n", int(DIM * DIM * DIM));
  //  for (std::size_t i = 0; i < checkers.size(); ++i)
  for (std::size_t i = 0; i < 2; ++i)
  {
    addCollisionObjects(*checkers[i], use_single_link, use_convex_mesh);
    checkers[i]->setCollisionMarginData(CollisionMarginData(contact_distance));

    auto start_time = std::chrono::high_resolution_clock::now();
    for (auto& pose : poses)
    {
      checkers[i]->setCollisionObjectsTransform("move_link", pose, pose * delta_pose);

      // Perform collision check
      ContactResultMap result;
      checkers[i]->contactTest(result, ContactTestType::FIRST);
      checker_contacts[i] += result.size();
    }
    auto end_time = std::chrono::high_resolution_clock::now();
    double total_time = std::chrono::duration<double, std::milli>(end_time - start_time).count();

    std::printf(
        "%15s | DT: %15.5f ms | Contacts: %d\n", checker_names[i].c_str(), total_time, int(checker_contacts[i]));
  }
}

int main(int /*argc*/, char** /*argv*/)
{
  std::printf("Discrete: One Primitive Shape per link\n");
  runDiscreteProfile(false, false, 0.0);

  std::printf("Discrete: All Primitive Shape in single link\n");
  runDiscreteProfile(true, false, 0.0);

  std::printf("Discrete: One Convex Shape per link\n");
  runDiscreteProfile(false, true, 0.0);

  std::printf("Discrete: All Convex Shape in single link\n");
  runDiscreteProfile(true, true, 0.0);

  std::printf("Continuous: One Primitive Shape per link\n");
  runContinuousProfile(false, false, 0.0);

  std::printf("Continuous: All Primitive Shape in single link\n");
  runContinuousProfile(true, false, 0.0);

  std::printf("Continuous: One Convex Shape per link\n");
  runContinuousProfile(false, true, 0.0);

  std::printf("Continuous: All Convex Shape in single link\n");
  runContinuousProfile(true, true, 0.0);
  return 0;
}
