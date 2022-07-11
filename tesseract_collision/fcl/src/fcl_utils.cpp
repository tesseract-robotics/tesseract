/**
 * @file fcl_utils.cpp
 * @brief Tesseract ROS FCL Utility Functions.
 *
 * @author Levi Armstrong
 * @date Dec 18, 2017
 * @version TODO
 * @bug No known bugs
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
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
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
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_collision/fcl/fcl_utils.h>

namespace tesseract_collision::tesseract_collision_fcl
{
CollisionGeometryPtr createShapePrimitive(const tesseract_geometry::Plane::ConstPtr& geom)
{
  return std::make_shared<fcl::Planed>(geom->getA(), geom->getB(), geom->getC(), geom->getD());
}

CollisionGeometryPtr createShapePrimitive(const tesseract_geometry::Box::ConstPtr& geom)
{
  return std::make_shared<fcl::Boxd>(geom->getX(), geom->getY(), geom->getZ());
}

CollisionGeometryPtr createShapePrimitive(const tesseract_geometry::Sphere::ConstPtr& geom)
{
  return std::make_shared<fcl::Sphered>(geom->getRadius());
}

CollisionGeometryPtr createShapePrimitive(const tesseract_geometry::Cylinder::ConstPtr& geom)
{
  return std::make_shared<fcl::Cylinderd>(geom->getRadius(), geom->getLength());
}

CollisionGeometryPtr createShapePrimitive(const tesseract_geometry::Cone::ConstPtr& geom)
{
  return std::make_shared<fcl::Coned>(geom->getRadius(), geom->getLength());
}

CollisionGeometryPtr createShapePrimitive(const tesseract_geometry::Capsule::ConstPtr& geom)
{
  return std::make_shared<fcl::Capsuled>(geom->getRadius(), geom->getLength());
}

CollisionGeometryPtr createShapePrimitive(const tesseract_geometry::Mesh::ConstPtr& geom)
{
  int vertice_count = geom->getVertexCount();
  int triangle_count = geom->getFaceCount();
  const tesseract_common::VectorVector3d& vertices = *(geom->getVertices());
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

CollisionGeometryPtr createShapePrimitive(const tesseract_geometry::ConvexMesh::ConstPtr& geom)
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

CollisionGeometryPtr createShapePrimitive(const tesseract_geometry::Octree::ConstPtr& geom)
{
  switch (geom->getSubType())
  {
    case tesseract_geometry::Octree::SubType::BOX:
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

CollisionGeometryPtr createShapePrimitive(const CollisionShapeConstPtr& geom)
{
  switch (geom->getType())
  {
    case tesseract_geometry::GeometryType::PLANE:
    {
      return createShapePrimitive(std::static_pointer_cast<const tesseract_geometry::Plane>(geom));
    }
    case tesseract_geometry::GeometryType::BOX:
    {
      return createShapePrimitive(std::static_pointer_cast<const tesseract_geometry::Box>(geom));
    }
    case tesseract_geometry::GeometryType::SPHERE:
    {
      return createShapePrimitive(std::static_pointer_cast<const tesseract_geometry::Sphere>(geom));
    }
    case tesseract_geometry::GeometryType::CYLINDER:
    {
      return createShapePrimitive(std::static_pointer_cast<const tesseract_geometry::Cylinder>(geom));
    }
    case tesseract_geometry::GeometryType::CONE:
    {
      return createShapePrimitive(std::static_pointer_cast<const tesseract_geometry::Cone>(geom));
    }
    case tesseract_geometry::GeometryType::CAPSULE:
    {
      return createShapePrimitive(std::static_pointer_cast<const tesseract_geometry::Capsule>(geom));
    }
    case tesseract_geometry::GeometryType::MESH:
    {
      return createShapePrimitive(std::static_pointer_cast<const tesseract_geometry::Mesh>(geom));
    }
    case tesseract_geometry::GeometryType::CONVEX_MESH:
    {
      return createShapePrimitive(std::static_pointer_cast<const tesseract_geometry::ConvexMesh>(geom));
    }
    case tesseract_geometry::GeometryType::OCTREE:
    {
      return createShapePrimitive(std::static_pointer_cast<const tesseract_geometry::Octree>(geom));
    }
    default:
    {
      CONSOLE_BRIDGE_logError("This geometric shape type (%d) is not supported using fcl yet",
                              static_cast<int>(geom->getType()));
      return nullptr;
    }
  }
}

bool collisionCallback(fcl::CollisionObjectd* o1, fcl::CollisionObjectd* o2, void* data)
{
  auto* cdata = reinterpret_cast<ContactTestData*>(data);  // NOLINT

  if (cdata->done)
    return true;

  const auto* cd1 = static_cast<const CollisionObjectWrapper*>(o1->getUserData());
  const auto* cd2 = static_cast<const CollisionObjectWrapper*>(o2->getUserData());
  assert(cd1->getName() != cd2->getName());

  bool needs_collision = cd1->m_enabled && cd2->m_enabled &&
                         (cd1->m_collisionFilterGroup & cd2->m_collisionFilterMask) &&  // NOLINT
                         (cd2->m_collisionFilterGroup & cd1->m_collisionFilterMask) &&  // NOLINT
                         !isContactAllowed(cd1->getName(), cd2->getName(), cdata->fn, false);

  assert(std::find(cdata->active->begin(), cdata->active->end(), cd1->getName()) != cdata->active->end() ||
         std::find(cdata->active->begin(), cdata->active->end(), cd2->getName()) != cdata->active->end());

  if (!needs_collision)
    return false;

  std::size_t num_contacts = (cdata->req.contact_limit > 0) ? static_cast<std::size_t>(cdata->req.contact_limit) :
                                                              std::numeric_limits<std::size_t>::max();
  if (cdata->req.type == ContactTestType::FIRST)
    num_contacts = 1;

  fcl::CollisionResultd col_result;
  fcl::collide(o1, o2, fcl::CollisionRequestd(num_contacts, cdata->req.calculate_penetration, 1, false), col_result);

  if (col_result.isCollision())
  {
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
      contact.shape_id[0] = static_cast<int>(cd1->getShapeIndex(o1));
      contact.shape_id[1] = static_cast<int>(cd2->getShapeIndex(o2));
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

      ObjectPairKey pc = getObjectPairKey(cd1->getName(), cd2->getName());
      const auto& it = cdata->res->find(pc);
      bool found = (it != cdata->res->end());

      processResult(*cdata, contact, pc, found);
    }
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
  assert(cd1->getName() != cd2->getName());

  bool needs_collision = cd1->m_enabled && cd2->m_enabled &&
                         (cd1->m_collisionFilterGroup & cd2->m_collisionFilterMask) &&  // NOLINT
                         (cd2->m_collisionFilterGroup & cd1->m_collisionFilterMask) &&  // NOLINT
                         !isContactAllowed(cd1->getName(), cd2->getName(), cdata->fn, false);

  assert(std::find(cdata->active->begin(), cdata->active->end(), cd1->getName()) != cdata->active->end() ||
         std::find(cdata->active->begin(), cdata->active->end(), cd2->getName()) != cdata->active->end());

  if (!needs_collision)
    return false;

  fcl::DistanceResultd fcl_result;
  fcl::DistanceRequestd fcl_request(true, true);
  double d = fcl::distance(o1, o2, fcl_request, fcl_result);

  if (d < cdata->collision_margin_data.getMaxCollisionMargin())
  {
    const Eigen::Isometry3d& tf1 = cd1->getCollisionObjectsTransform();
    const Eigen::Isometry3d& tf2 = cd2->getCollisionObjectsTransform();
    Eigen::Isometry3d tf1_inv = tf1.inverse();
    Eigen::Isometry3d tf2_inv = tf2.inverse();

    ContactResult contact;
    contact.link_names[0] = cd1->getName();
    contact.link_names[1] = cd2->getName();
    contact.shape_id[0] = cd1->getShapeIndex(o1);
    contact.shape_id[1] = cd2->getShapeIndex(o2);
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
    contact.normal = (fcl_result.min_distance * (contact.nearest_points[1] - contact.nearest_points[0])).normalized();

    // TODO: There is an issue with FCL need to track down
    assert(!std::isnan(contact.nearest_points[0](0)));

    ObjectPairKey pc = getObjectPairKey(cd1->getName(), cd2->getName());
    const auto& it = cdata->res->find(pc);
    bool found = (it != cdata->res->end());

    processResult(*cdata, contact, pc, found);
  }

  return cdata->done;
}

CollisionObjectWrapper::CollisionObjectWrapper(std::string name,
                                               const int& type_id,
                                               CollisionShapesConst shapes,
                                               tesseract_common::VectorIsometry3d shape_poses)
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
  for (std::size_t i = 0; i < shapes_.size(); ++i)
  {
    CollisionGeometryPtr subshape = createShapePrimitive(shapes_[i]);
    if (subshape != nullptr)
    {
      collision_geometries_.push_back(subshape);
      auto co = std::make_shared<FCLCollisionObjectWrapper>(subshape);
      co->setUserData(this);
      co->setTransform(shape_poses_[i]);
      co->updateAABB();
      collision_objects_.push_back(co);
      collision_objects_raw_.push_back(co.get());
    }
  }
}

int CollisionObjectWrapper::getShapeIndex(const fcl::CollisionObjectd* co) const
{
  auto it = std::find_if(collision_objects_.begin(), collision_objects_.end(), [&co](const CollisionObjectPtr& c) {
    return c.get() == co;
  });

  if (it != collision_objects_.end())
    return static_cast<int>(std::distance(collision_objects_.begin(), it));

  return -1;
}

}  // namespace tesseract_collision::tesseract_collision_fcl
