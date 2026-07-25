/**
 * @file fcl_utils.cpp
 * @brief Tesseract ROS FCL Utility Functions.
 *
 * @author Levi Armstrong
 * @date Dec 18, 2017
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (BSD)
 * @par
 * All rights reserved.
 * @par
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * @par
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 * @par
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <cassert>
#include <fcl/geometry/bvh/BVH_model-inl.h>
#include <fcl/geometry/shape/box-inl.h>
#include <fcl/geometry/shape/cylinder-inl.h>
#include <fcl/geometry/shape/convex-inl.h>
#include <fcl/geometry/shape/plane-inl.h>
#include <fcl/geometry/shape/sphere-inl.h>
#include <fcl/geometry/shape/cone-inl.h>
#include <fcl/geometry/shape/capsule-inl.h>
#include <fcl/geometry/octree/octree-inl.h>
#include <memory>
#include <stdexcept>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/fcl/fcl_utils.h>
#include <tesseract/collision/fcl/fcl_collision_geometry_cache.h>
#include <tesseract/collision/implicit_sdf_collision_solver.h>
#include <tesseract/geometry/geometries.h>
#include <tesseract/geometry/impl/signed_distance_field.h>

namespace tesseract::collision::fcl_internal
{
CollisionGeometryPtr createShapePrimitive(const tesseract::geometry::Plane::ConstPtr& geom)
{
  return std::make_shared<fcl::Planed>(geom->getA(), geom->getB(), geom->getC(), geom->getD());
}

CollisionGeometryPtr createShapePrimitive(const tesseract::geometry::Box::ConstPtr& geom)
{
  return std::make_shared<fcl::Boxd>(geom->getX(), geom->getY(), geom->getZ());
}

CollisionGeometryPtr createShapePrimitive(const tesseract::geometry::Sphere::ConstPtr& geom)
{
  return std::make_shared<fcl::Sphered>(geom->getRadius());
}

CollisionGeometryPtr createShapePrimitive(const tesseract::geometry::Cylinder::ConstPtr& geom)
{
  return std::make_shared<fcl::Cylinderd>(geom->getRadius(), geom->getLength());
}

CollisionGeometryPtr createShapePrimitive(const tesseract::geometry::Cone::ConstPtr& geom)
{
  return std::make_shared<fcl::Coned>(geom->getRadius(), geom->getLength());
}

CollisionGeometryPtr createShapePrimitive(const tesseract::geometry::Capsule::ConstPtr& geom)
{
  return std::make_shared<fcl::Capsuled>(geom->getRadius(), geom->getLength());
}

CollisionGeometryPtr createShapePrimitive(const tesseract::geometry::SignedDistanceField::ConstPtr& geom)
{
  if (!makeImplicitSDFShape(*geom, Eigen::Isometry3d::Identity()).isValid())
    throw std::invalid_argument("FCL signed distance field geometry is invalid");

  // FCL has no native SDF geometry. Register a conservative box solely for broadphase dispatch;
  // SDF pairs are intercepted below and evaluated by the backend-neutral implicit solver.
  const Eigen::Vector3d scaled_min = geom->getDomain().min().cwiseProduct(geom->getScale());
  const Eigen::Vector3d scaled_max = geom->getDomain().max().cwiseProduct(geom->getScale());
  const Eigen::Vector3d half_extents = scaled_min.cwiseAbs().cwiseMax(scaled_max.cwiseAbs());
  return std::make_shared<fcl::Boxd>(2.0 * half_extents.x(), 2.0 * half_extents.y(), 2.0 * half_extents.z());
}

CollisionGeometryPtr createShapePrimitive(const tesseract::geometry::Mesh::ConstPtr& geom)
{
  int vertice_count = geom->getVertexCount();
  int triangle_count = geom->getFaceCount();
  const tesseract::common::VectorVector3d& vertices = *(geom->getVertices());
  const Eigen::VectorXi& triangles = *(geom->getFaces());

  auto g = std::make_shared<fcl::BVHModel<fcl::OBBRSSd>>();
  if (vertice_count > 0 && triangle_count > 0)
  {
    std::vector<fcl::Triangle> tri_indices(static_cast<size_t>(triangle_count));
    for (int i = 0; i < triangle_count; ++i)
    {
      assert(triangles[4L * i] == 3);
      tri_indices[static_cast<size_t>(i)] = fcl::Triangle(static_cast<size_t>(triangles[(4 * i) + 1]),
                                                          static_cast<size_t>(triangles[(4 * i) + 2]),
                                                          static_cast<size_t>(triangles[(4 * i) + 3]));
    }

    g->beginModel();
    g->addSubModel(vertices, tri_indices);
    g->endModel();

    return g;
  }

  CONSOLE_BRIDGE_logError("The mesh is empty!");
  return nullptr;
}

CollisionGeometryPtr createShapePrimitive(const tesseract::geometry::ConvexMesh::ConstPtr& geom)
{
  int vertice_count = geom->getVertexCount();
  int face_count = geom->getFaceCount();

  if (vertice_count > 0 && face_count > 0)
  {
    auto faces = std::make_shared<const std::vector<int>>(geom->getFaces()->data(),
                                                          geom->getFaces()->data() + geom->getFaces()->size());
    return std::make_shared<fcl::Convexd>(geom->getVertices(), face_count, faces);
  }

  CONSOLE_BRIDGE_logError("The mesh is empty!");
  return nullptr;
}

CollisionGeometryPtr createShapePrimitive(const tesseract::geometry::Octree::ConstPtr& geom)
{
  switch (geom->getSubType())
  {
    case tesseract::geometry::OctreeSubType::BOX:
    {
      return std::make_shared<fcl::OcTreed>(geom->getOctree());
    }
    default:
    {
      CONSOLE_BRIDGE_logError("This fcl octree sub shape type (%d) is not supported for geometry octree",
                              static_cast<int>(geom->getSubType()));
      return nullptr;
    }
  }
}

CollisionGeometryPtr createShapePrimitiveHelper(const CollisionShapeConstPtr& geom)
{
  switch (geom->getType())
  {
    case tesseract::geometry::GeometryType::PLANE:
    {
      return createShapePrimitive(std::static_pointer_cast<const tesseract::geometry::Plane>(geom));
    }
    case tesseract::geometry::GeometryType::BOX:
    {
      return createShapePrimitive(std::static_pointer_cast<const tesseract::geometry::Box>(geom));
    }
    case tesseract::geometry::GeometryType::SPHERE:
    {
      return createShapePrimitive(std::static_pointer_cast<const tesseract::geometry::Sphere>(geom));
    }
    case tesseract::geometry::GeometryType::CYLINDER:
    {
      return createShapePrimitive(std::static_pointer_cast<const tesseract::geometry::Cylinder>(geom));
    }
    case tesseract::geometry::GeometryType::CONE:
    {
      return createShapePrimitive(std::static_pointer_cast<const tesseract::geometry::Cone>(geom));
    }
    case tesseract::geometry::GeometryType::CAPSULE:
    {
      return createShapePrimitive(std::static_pointer_cast<const tesseract::geometry::Capsule>(geom));
    }
    case tesseract::geometry::GeometryType::SIGNED_DISTANCE_FIELD:
    {
      return createShapePrimitive(std::static_pointer_cast<const tesseract::geometry::SignedDistanceField>(geom));
    }
    case tesseract::geometry::GeometryType::MESH:
    {
      return createShapePrimitive(std::static_pointer_cast<const tesseract::geometry::Mesh>(geom));
    }
    case tesseract::geometry::GeometryType::CONVEX_MESH:
    {
      return createShapePrimitive(std::static_pointer_cast<const tesseract::geometry::ConvexMesh>(geom));
    }
    case tesseract::geometry::GeometryType::OCTREE:
    {
      return createShapePrimitive(std::static_pointer_cast<const tesseract::geometry::Octree>(geom));
    }
    case tesseract::geometry::GeometryType::COMPOUND_MESH:
    {
      throw std::runtime_error("CompundMesh type should not be passed to this function!");
    }
    default:
    {
      CONSOLE_BRIDGE_logError("This geometric shape type (%d) is not supported using fcl yet",
                              static_cast<int>(geom->getType()));
      return nullptr;
    }
  }
}

CollisionGeometryPtr createShapePrimitive(const CollisionShapeConstPtr& geom)
{
  CollisionGeometryPtr shape = FCLCollisionGeometryCache::get(geom);
  if (shape != nullptr)
    return shape;

  shape = createShapePrimitiveHelper(geom);
  FCLCollisionGeometryCache::insert(geom, shape);
  return shape;
}

bool needsCollisionCheck(const CollisionObjectWrapper* cd1,
                         const CollisionObjectWrapper* cd2,
                         const std::shared_ptr<const tesseract::common::ContactAllowedValidator>& validator,
                         bool verbose)
{
  return cd1->m_enabled && cd2->m_enabled && (cd2->m_collisionFilterGroup & cd1->m_collisionFilterMask) &&  // NOLINT
         (cd1->m_collisionFilterGroup & cd2->m_collisionFilterMask) &&                                      // NOLINT
         !isContactAllowed(cd1->getName(), cd2->getName(), validator, verbose);
}

namespace
{
bool isImplicitSDFPair(const CollisionObjectWrapper& cow1,
                       const fcl::CollisionObjectd* object1,
                       const CollisionObjectWrapper& cow2,
                       const fcl::CollisionObjectd* object2)
{
  const int shape_index1 = CollisionObjectWrapper::getShapeIndex(object1);
  const int shape_index2 = CollisionObjectWrapper::getShapeIndex(object2);
  const auto& geometry1 = cow1.getCollisionGeometries()[static_cast<std::size_t>(shape_index1)];
  const auto& geometry2 = cow2.getCollisionGeometries()[static_cast<std::size_t>(shape_index2)];
  return geometry1->getType() == tesseract::geometry::GeometryType::SIGNED_DISTANCE_FIELD ||
         geometry2->getType() == tesseract::geometry::GeometryType::SIGNED_DISTANCE_FIELD;
}

bool processImplicitSDFPair(const CollisionObjectWrapper& cow1,
                            const fcl::CollisionObjectd* object1,
                            const CollisionObjectWrapper& cow2,
                            const fcl::CollisionObjectd* object2,
                            ContactTestData& cdata)
{
  const int shape_index1 = CollisionObjectWrapper::getShapeIndex(object1);
  const int shape_index2 = CollisionObjectWrapper::getShapeIndex(object2);
  const auto& geometry1 = cow1.getCollisionGeometries()[static_cast<std::size_t>(shape_index1)];
  const auto& geometry2 = cow2.getCollisionGeometries()[static_cast<std::size_t>(shape_index2)];
  const Eigen::Isometry3d geometry_pose1 =
      cow1.getCollisionObjectsTransform() *
      cow1.getCollisionGeometriesTransforms()[static_cast<std::size_t>(shape_index1)];
  const Eigen::Isometry3d geometry_pose2 =
      cow2.getCollisionObjectsTransform() *
      cow2.getCollisionGeometriesTransforms()[static_cast<std::size_t>(shape_index2)];

  const ImplicitSDFShape shape1 = makeImplicitSDFShape(*geometry1, geometry_pose1);
  const ImplicitSDFShape shape2 = makeImplicitSDFShape(*geometry2, geometry_pose2);
  if (!shape1.isValid() || !shape2.isValid())
    throw std::runtime_error("FCL signed distance field collision does not support geometry pair types " +
                             std::to_string(static_cast<int>(geometry1->getType())) + " and " +
                             std::to_string(static_cast<int>(geometry2->getType())));

  ImplicitSDFCollisionConfig config;
  config.contact_margin = cdata.collision_margin_data.getCollisionMargin(cow1.getName(), cow2.getName());
  config.max_contacts =
      (cdata.req.contact_limit > 0) ?
          static_cast<int>(std::min<std::int64_t>(cdata.req.contact_limit, std::numeric_limits<int>::max())) :
          4;
  if (cdata.req.type == ContactTestType::FIRST || cdata.req.type == ContactTestType::CLOSEST)
    config.max_contacts = 1;

  const std::vector<ImplicitSDFContact> implicit_contacts = collideImplicitSDF(shape1, shape2, config);
  if (implicit_contacts.empty())
    return false;

  const Eigen::Isometry3d& link_pose1 = cow1.getCollisionObjectsTransform();
  const Eigen::Isometry3d& link_pose2 = cow2.getCollisionObjectsTransform();
  const Eigen::Isometry3d link_pose1_inv = link_pose1.inverse();
  const Eigen::Isometry3d link_pose2_inv = link_pose2.inverse();
  TESSERACT_THREAD_LOCAL tesseract::common::LinkNamesPair link_pair;
  tesseract::common::makeOrderedLinkPair(link_pair, cow1.getName(), cow2.getName());

  for (const auto& implicit_contact : implicit_contacts)
  {
    ContactResult contact;
    contact.link_names = { cow1.getName(), cow2.getName() };
    contact.shape_id = { shape_index1, shape_index2 };
    contact.subshape_id = { -1, -1 };
    contact.nearest_points = implicit_contact.nearest_points;
    contact.nearest_points_local = { link_pose1_inv * contact.nearest_points[0],
                                     link_pose2_inv * contact.nearest_points[1] };
    contact.transform = { link_pose1, link_pose2 };
    contact.type_id = { cow1.getTypeID(), cow2.getTypeID() };
    contact.distance = implicit_contact.distance;
    contact.normal = implicit_contact.normal;

    const auto existing = cdata.res->find(link_pair);
    const bool found = (existing != cdata.res->end() && !existing->second.empty());
    processResult(cdata, contact, link_pair, found);
    if (cdata.done)
      break;
  }
  return cdata.done;
}
}  // namespace

bool collisionCallback(fcl::CollisionObjectd* o1, fcl::CollisionObjectd* o2, void* data)
{
  auto* cdata = reinterpret_cast<ContactTestData*>(data);  // NOLINT

  if (cdata->done)
    return true;

  const auto* cd1 = static_cast<const CollisionObjectWrapper*>(o1->getUserData());
  const auto* cd2 = static_cast<const CollisionObjectWrapper*>(o2->getUserData());

  if (!needsCollisionCheck(cd1, cd2, cdata->validator, false))
    return false;

  if (isImplicitSDFPair(*cd1, o1, *cd2, o2))
    return processImplicitSDFPair(*cd1, o1, *cd2, o2, *cdata);

  std::size_t num_contacts = (cdata->req.contact_limit > 0) ? static_cast<std::size_t>(cdata->req.contact_limit) :
                                                              std::numeric_limits<std::size_t>::max();
  if (cdata->req.type == ContactTestType::FIRST)
    num_contacts = 1;

  fcl::CollisionResultd col_result;
  fcl::collide(o1, o2, fcl::CollisionRequestd(num_contacts, cdata->req.calculate_penetration, 1, false), col_result);

  if (!col_result.isCollision())
    return false;

  TESSERACT_THREAD_LOCAL tesseract::common::LinkNamesPair link_pair;
  tesseract::common::makeOrderedLinkPair(link_pair, cd1->getName(), cd2->getName());

  const Eigen::Isometry3d& tf1 = cd1->getCollisionObjectsTransform();
  const Eigen::Isometry3d& tf2 = cd2->getCollisionObjectsTransform();
  Eigen::Isometry3d tf1_inv = tf1.inverse();
  Eigen::Isometry3d tf2_inv = tf2.inverse();

  for (size_t i = 0; i < col_result.numContacts(); ++i)
  {
    const fcl::Contactd& fcl_contact = col_result.getContact(i);
    ContactResult contact;
    contact.link_names[0] = cd1->getName();
    contact.link_names[1] = cd2->getName();
    contact.shape_id[0] = CollisionObjectWrapper::getShapeIndex(o1);
    contact.shape_id[1] = CollisionObjectWrapper::getShapeIndex(o2);
    contact.subshape_id[0] = static_cast<int>(fcl_contact.b1);
    contact.subshape_id[1] = static_cast<int>(fcl_contact.b2);
    contact.nearest_points[0] = fcl_contact.pos;
    contact.nearest_points[1] = fcl_contact.pos;
    contact.nearest_points_local[0] = tf1_inv * contact.nearest_points[0];
    contact.nearest_points_local[1] = tf2_inv * contact.nearest_points[1];
    contact.transform[0] = tf1;
    contact.transform[1] = tf2;
    contact.type_id[0] = cd1->getTypeID();
    contact.type_id[1] = cd2->getTypeID();
    contact.distance = -1.0 * fcl_contact.penetration_depth;
    contact.normal = fcl_contact.normal;

    const auto it = cdata->res->find(link_pair);
    bool found = (it != cdata->res->end() && !it->second.empty());

    processResult(*cdata, contact, link_pair, found);
  }

  return cdata->done;
}

bool distanceCallback(fcl::CollisionObjectd* o1, fcl::CollisionObjectd* o2, void* data)
{
  auto* cdata = reinterpret_cast<ContactTestData*>(data);  // NOLINT

  if (cdata->done)
    return true;

  const auto* cd1 = static_cast<const CollisionObjectWrapper*>(o1->getUserData());
  const auto* cd2 = static_cast<const CollisionObjectWrapper*>(o2->getUserData());

  if (!needsCollisionCheck(cd1, cd2, cdata->validator, false))
    return false;

  if (isImplicitSDFPair(*cd1, o1, *cd2, o2))
    return processImplicitSDFPair(*cd1, o1, *cd2, o2, *cdata);

  fcl::DistanceResultd fcl_result;
  fcl::DistanceRequestd fcl_request(true, true);
  double d = fcl::distance(o1, o2, fcl_request, fcl_result);

  if (d > cdata->collision_margin_data.getCollisionMargin(cd1->getName(), cd2->getName()))
    return false;

  const Eigen::Isometry3d& tf1 = cd1->getCollisionObjectsTransform();
  const Eigen::Isometry3d& tf2 = cd2->getCollisionObjectsTransform();
  Eigen::Isometry3d tf1_inv = tf1.inverse();
  Eigen::Isometry3d tf2_inv = tf2.inverse();

  ContactResult contact;
  contact.link_names[0] = cd1->getName();
  contact.link_names[1] = cd2->getName();
  contact.shape_id[0] = CollisionObjectWrapper::getShapeIndex(o1);
  contact.shape_id[1] = CollisionObjectWrapper::getShapeIndex(o2);
  contact.subshape_id[0] = static_cast<int>(fcl_result.b1);
  contact.subshape_id[1] = static_cast<int>(fcl_result.b2);
  contact.nearest_points[0] = fcl_result.nearest_points[0];
  contact.nearest_points[1] = fcl_result.nearest_points[1];
  contact.nearest_points_local[0] = tf1_inv * contact.nearest_points[0];
  contact.nearest_points_local[1] = tf2_inv * contact.nearest_points[1];
  contact.transform[0] = tf1;
  contact.transform[1] = tf2;
  contact.type_id[0] = cd1->getTypeID();
  contact.type_id[1] = cd2->getTypeID();
  contact.distance = fcl_result.min_distance;
  contact.normal =
      (std::copysign(1.0, fcl_result.min_distance) * (contact.nearest_points[1] - contact.nearest_points[0]))
          .normalized();

  // TODO: There is an issue with FCL need to track down
  assert(!std::isnan(contact.nearest_points[0](0)));

  TESSERACT_THREAD_LOCAL tesseract::common::LinkNamesPair link_pair;
  tesseract::common::makeOrderedLinkPair(link_pair, cd1->getName(), cd2->getName());
  const auto it = cdata->res->find(link_pair);
  bool found = (it != cdata->res->end() && !it->second.empty());

  processResult(*cdata, contact, link_pair, found);

  return cdata->done;
}

CollisionObjectWrapper::CollisionObjectWrapper(std::string name,
                                               const int& type_id,
                                               CollisionShapesConst shapes,
                                               tesseract::common::VectorIsometry3d shape_poses)
  : name_(std::move(name)), type_id_(type_id), shapes_(std::move(shapes)), shape_poses_(std::move(shape_poses))
{
  assert(!shapes_.empty());                       // NOLINT
  assert(!shape_poses_.empty());                  // NOLINT
  assert(!name_.empty());                         // NOLINT
  assert(shapes_.size() == shape_poses_.size());  // NOLINT

  m_collisionFilterGroup = CollisionFilterGroups::KinematicFilter;
  m_collisionFilterMask = CollisionFilterGroups::StaticFilter | CollisionFilterGroups::KinematicFilter;

  collision_geometries_.reserve(shapes_.size());
  collision_objects_.reserve(shapes_.size());
  collision_objects_raw_.reserve(shapes_.size());
  for (std::size_t i = 0; i < shapes_.size(); ++i)  // NOLINT
  {
    if (shapes_[i]->getType() == tesseract::geometry::GeometryType::COMPOUND_MESH)
    {
      const auto& meshes = std::static_pointer_cast<const tesseract::geometry::CompoundMesh>(shapes_[i])->getMeshes();
      for (const auto& mesh : meshes)
      {
        CollisionGeometryPtr subshape = createShapePrimitive(mesh);
        if (subshape != nullptr)
        {
          collision_geometries_.push_back(subshape);
          auto co = std::make_shared<FCLCollisionObjectWrapper>(subshape);
          co->setUserData(this);
          co->setShapeIndex(static_cast<int>(i));
          co->setTransform(shape_poses_[i]);
          co->updateAABB();
          collision_objects_.push_back(co);
          collision_objects_raw_.push_back(co.get());
        }
      }
    }
    else
    {
      CollisionGeometryPtr subshape = createShapePrimitive(shapes_[i]);
      if (subshape != nullptr)
      {
        collision_geometries_.push_back(subshape);
        auto co = std::make_shared<FCLCollisionObjectWrapper>(subshape);
        co->setUserData(this);
        co->setShapeIndex(static_cast<int>(i));
        co->setTransform(shape_poses_[i]);
        co->updateAABB();
        collision_objects_.push_back(co);
        collision_objects_raw_.push_back(co.get());
      }
    }
  }
}

int CollisionObjectWrapper::getShapeIndex(const fcl::CollisionObjectd* co)
{
  return static_cast<const FCLCollisionObjectWrapper*>(co)->getShapeIndex();
}

}  // namespace tesseract::collision::fcl_internal
