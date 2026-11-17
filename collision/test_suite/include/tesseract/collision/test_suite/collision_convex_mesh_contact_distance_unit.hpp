#ifndef TESSERACT_COLLISION_COLLISION_CONVEX_MESH_CONTACT_DISTANCE_UNIT_HPP
#define TESSERACT_COLLISION_COLLISION_CONVEX_MESH_CONTACT_DISTANCE_UNIT_HPP

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <cstddef>
#include <memory>
#include <vector>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/discrete_contact_manager.h>
#include <tesseract/collision/common.h>
#include <tesseract/geometry/impl/convex_mesh.h>

namespace tesseract::collision::test_suite
{
namespace detail
{
/** @brief Unit cube [-0.5, 0.5]^3 as a ConvexMesh */
inline tesseract::geometry::ConvexMesh::Ptr makeUnitCubeConvexMesh()
{
  auto vertices = std::make_shared<tesseract::common::VectorVector3d>();
  for (int k = 0; k < 8; ++k)
    vertices->emplace_back(((k & 1) != 0 ? 0.5 : -0.5), ((k & 2) != 0 ? 0.5 : -0.5), ((k & 4) != 0 ? 0.5 : -0.5));
  auto faces = std::make_shared<Eigen::VectorXi>(30);
  *faces << 4, 0, 2, 3, 1,  // -z
      4, 4, 5, 7, 6,        // +z
      4, 0, 1, 5, 4,        // -y
      4, 2, 6, 7, 3,        // +y
      4, 0, 4, 6, 2,        // -x
      4, 1, 3, 7, 5;        // +x
  return std::make_shared<tesseract::geometry::ConvexMesh>(vertices, faces, 6);
}
}  // namespace detail

/**
 * @brief Two ConvexMesh unit cubes face to face, just inside and just outside the contact distance.
 * @details The broadphase AABB of a ConvexMesh is its hull bounds grown by half the contact distance, with no slack.
 * A contact 0.1 mm inside the contact distance must be found, along every axis, and one 0.1 mm outside must not.
 * An AABB that is too small (for example, one cached with a negative margin) loses the first.
 */
inline void runTest(DiscreteContactManager& checker)
{
  const tesseract::common::LinkId a("cube_a");
  const tesseract::common::LinkId b("cube_b");
  const CollisionShapesConst shape{ detail::makeUnitCubeConvexMesh() };
  const tesseract::common::VectorIsometry3d pose{ Eigen::Isometry3d::Identity() };
  checker.addCollisionObject(a, 0, shape, pose);
  checker.addCollisionObject(b, 0, shape, pose);
  checker.setActiveCollisionObjects(std::vector<tesseract::common::LinkId>{ a, b });

  constexpr double eps = 1e-4;  // 0.1 mm either side of the contact distance
  for (const double contact_distance : { 0.01, 0.03, 0.1 })
  {
    checker.setCollisionMarginData(CollisionMarginData(contact_distance));
    for (int axis = 0; axis < 3; ++axis)
    {
      for (const double gap : { contact_distance - eps, contact_distance + eps })
      {
        tesseract::common::LinkIdTransformMap location;
        location[a] = Eigen::Isometry3d::Identity();
        location[b] = Eigen::Isometry3d::Identity();
        location[b].translation()(axis) = 1.0 + gap;
        checker.setCollisionObjectsTransform(location);

        ContactResultMap result;
        checker.contactTest(result, ContactRequest(ContactTestType::ALL));
        ContactResultVector result_vector;
        result.flattenMoveResults(result_vector);

        if (gap < contact_distance)
        {
          ASSERT_EQ(result_vector.size(), std::size_t{ 1 })
              << "contact_distance " << contact_distance << ", axis " << axis << ", gap " << gap;
          EXPECT_NEAR(result_vector[0].distance, gap, 1e-6);
        }
        else
        {
          EXPECT_TRUE(result_vector.empty())
              << "contact_distance " << contact_distance << ", axis " << axis << ", gap " << gap;
        }
      }
    }
  }
}
}  // namespace tesseract::collision::test_suite

#endif  // TESSERACT_COLLISION_COLLISION_CONVEX_MESH_CONTACT_DISTANCE_UNIT_HPP
