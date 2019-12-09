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
#include <fcl/geometry/bvh/BVH_model.h>
#include <fcl/geometry/shape/box.h>
#include <fcl/geometry/shape/cylinder.h>
#include <fcl/geometry/shape/convex.h>
#include <fcl/geometry/shape/plane.h>
#include <fcl/geometry/shape/sphere.h>
#include <fcl/geometry/shape/cone.h>
#include <fcl/geometry/shape/capsule.h>
#include <fcl/geometry/octree/octree.h>
#include <boost/thread/mutex.hpp>
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_collision/fcl/fcl_utils.h>

namespace tesseract_collision
{
namespace tesseract_collision_fcl
{
CollisionGeometryPtr createShapePrimitive(const tesseract_geometry::Plane::ConstPtr& geom)
{
  return CollisionGeometryPtr(new fcl::Planed(geom->getA(), geom->getB(), geom->getC(), geom->getD()));
}

CollisionGeometryPtr createShapePrimitive(const tesseract_geometry::Box::ConstPtr& geom)
{
  return CollisionGeometryPtr(new fcl::Boxd(geom->getX(), geom->getY(), geom->getZ()));
}

CollisionGeometryPtr createShapePrimitive(const tesseract_geometry::Sphere::ConstPtr& geom)
{
  return CollisionGeometryPtr(new fcl::Sphered(geom->getRadius()));
}

CollisionGeometryPtr createShapePrimitive(const tesseract_geometry::Cylinder::ConstPtr& geom)
{
  return CollisionGeometryPtr(new fcl::Cylinderd(geom->getRadius(), geom->getLength()));
}

CollisionGeometryPtr createShapePrimitive(const tesseract_geometry::Cone::ConstPtr& geom)
{
  return CollisionGeometryPtr(new fcl::Coned(geom->getRadius(), geom->getLength()));
}

CollisionGeometryPtr createShapePrimitive(const tesseract_geometry::Capsule::ConstPtr& geom)
{
  return CollisionGeometryPtr(new fcl::Capsuled(geom->getRadius(), geom->getLength()));
}

CollisionGeometryPtr createShapePrimitive(const tesseract_geometry::Mesh::ConstPtr& geom)
{
  int vertice_count = geom->getVerticeCount();
  int triangle_count = geom->getTriangleCount();
  const tesseract_common::VectorVector3d& vertices = *(geom->getVertices());
  const Eigen::VectorXi& triangles = *(geom->getTriangles());

  auto g = new fcl::BVHModel<fcl::OBBRSSd>();
  if (vertice_count > 0 && triangle_count > 0)
  {
    std::vector<fcl::Triangle> tri_indices(static_cast<size_t>(triangle_count));
    for (int i = 0; i < triangle_count; ++i)
    {
      assert(triangles[4 * i] == 3);
      tri_indices[static_cast<size_t>(i)] = fcl::Triangle(static_cast<size_t>(triangles[(4 * i) + 1]),
                                                          static_cast<size_t>(triangles[(4 * i) + 2]),
                                                          static_cast<size_t>(triangles[(4 * i) + 3]));
    }

    g->beginModel();
    g->addSubModel(vertices, tri_indices);
    g->endModel();

    return CollisionGeometryPtr(g);
  }

  CONSOLE_BRIDGE_logError("The mesh is empty!");
  return nullptr;
}

CollisionGeometryPtr createShapePrimitive(const tesseract_geometry::ConvexMesh::ConstPtr& geom)
{
  int vertice_count = geom->getVerticeCount();
  int face_count = geom->getFaceCount();

  if (vertice_count > 0 && face_count > 0)
  {
    std::shared_ptr<std::vector<int>> faces(
        new std::vector<int>(geom->getFaces()->data(), geom->getFaces()->data() + geom->getFaces()->size()));
    return CollisionGeometryPtr(new fcl::Convexd(geom->getVertices(), face_count, faces));
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
      return CollisionGeometryPtr(new fcl::OcTreed(geom->getOctree()));
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
  auto* cdata = reinterpret_cast<ContactTestData*>(data);

  if (cdata->done)
    return true;

  const auto* cd1 = static_cast<const CollisionObjectWrapper*>(o1->getUserData());
  const auto* cd2 = static_cast<const CollisionObjectWrapper*>(o2->getUserData());

  bool needs_collision =
      cd1->m_enabled && cd2->m_enabled && (cd1->m_collisionFilterGroup & cd2->m_collisionFilterMask) &&
      (cd2->m_collisionFilterGroup & cd1->m_collisionFilterMask) &&
      !isContactAllowed(cd1->getName(), cd2->getName(), cdata->fn, false) &&
      (std::find(cdata->active.begin(), cdata->active.end(), cd1->getName()) != cdata->active.end() ||
       std::find(cdata->active.begin(), cdata->active.end(), cd2->getName()) != cdata->active.end());

  if (!needs_collision)
    return false;

  size_t num_contacts = std::numeric_limits<size_t>::max();
  if (cdata->type == ContactTestType::FIRST)
    num_contacts = 1;

  fcl::CollisionResultd col_result;
  fcl::collide(o1, o2, fcl::CollisionRequestd(num_contacts, true, 1, false), col_result);

  if (col_result.isCollision())
  {
    for (size_t i = 0; i < col_result.numContacts(); ++i)
    {
      const fcl::Contactd& fcl_contact = col_result.getContact(i);
      ContactResult contact;
      contact.link_names[0] = cd1->getName();
      contact.link_names[1] = cd2->getName();
      contact.shape_id[0] = cd1->getShapeIndex(o1);
      contact.shape_id[1] = cd2->getShapeIndex(o2);
      contact.subshape_id[0] = fcl_contact.b1;
      contact.subshape_id[1] = fcl_contact.b2;
      contact.nearest_points[0] = fcl_contact.pos;
      contact.nearest_points[1] = fcl_contact.pos;
      contact.type_id[0] = cd1->getTypeID();
      contact.type_id[1] = cd2->getTypeID();
      contact.distance = -1.0 * fcl_contact.penetration_depth;
      contact.normal = fcl_contact.normal;

      ObjectPairKey pc = getObjectPairKey(cd1->getName(), cd2->getName());
      const auto& it = cdata->res.find(pc);
      bool found = (it != cdata->res.end());

      processResult(*cdata, contact, pc, found);
    }
  }

  return cdata->done;
}

bool distanceCallback(fcl::CollisionObjectd* o1, fcl::CollisionObjectd* o2, void* data, double& min_dist)
{
  auto* cdata = reinterpret_cast<ContactTestData*>(data);
  min_dist = cdata->contact_distance;

  if (cdata->done)
    return true;

  const auto* cd1 = static_cast<const CollisionObjectWrapper*>(o1->getUserData());
  const auto* cd2 = static_cast<const CollisionObjectWrapper*>(o2->getUserData());

  bool needs_collision =
      cd1->m_enabled && cd2->m_enabled && (cd1->m_collisionFilterGroup & cd2->m_collisionFilterMask) &&
      (cd2->m_collisionFilterGroup & cd1->m_collisionFilterMask) &&
      !isContactAllowed(cd1->getName(), cd2->getName(), cdata->fn, false) &&
      (std::find(cdata->active.begin(), cdata->active.end(), cd1->getName()) != cdata->active.end() ||
       std::find(cdata->active.begin(), cdata->active.end(), cd2->getName()) != cdata->active.end());

  if (!needs_collision)
    return false;

  fcl::DistanceResultd fcl_result;
  fcl::DistanceRequestd fcl_request(true, true);
  double d = fcl::distance(o1, o2, fcl_request, fcl_result);

  if (d < cdata->contact_distance)
  {
    ContactResult contact;
    contact.link_names[0] = cd1->getName();
    contact.link_names[1] = cd2->getName();
    contact.shape_id[0] = cd1->getShapeIndex(o1);
    contact.shape_id[1] = cd2->getShapeIndex(o2);
    contact.subshape_id[0] = fcl_result.b1;
    contact.subshape_id[1] = fcl_result.b2;
    contact.nearest_points[0] = fcl_result.nearest_points[0];
    contact.nearest_points[1] = fcl_result.nearest_points[1];
    contact.type_id[0] = cd1->getTypeID();
    contact.type_id[1] = cd2->getTypeID();
    contact.distance = fcl_result.min_distance;
    contact.normal = (fcl_result.min_distance * (contact.nearest_points[1] - contact.nearest_points[0])).normalized();

    // TODO: There is an issue with FCL need to track down
    if (std::isnan(contact.nearest_points[0](0)))
    {
      CONSOLE_BRIDGE_logError("Nearest Points are NAN's");
    }

    ObjectPairKey pc = getObjectPairKey(cd1->getName(), cd2->getName());
    const auto& it = cdata->res.find(pc);
    bool found = (it != cdata->res.end());

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
  assert(!shapes_.empty());
  assert(!shape_poses_.empty());
  assert(!name_.empty());
  assert(shapes_.size() == shape_poses_.size());

  collision_geometries_.reserve(shapes_.size());
  collision_objects_.reserve(shapes_.size());
  for (const auto& shape : shapes_)
  {
    CollisionGeometryPtr subshape = createShapePrimitive(shape);
    if (subshape != nullptr)
    {
      collision_geometries_.push_back(subshape);
      CollisionObjectPtr co(new fcl::CollisionObjectd(subshape));
      co->setUserData(this);
      collision_objects_.push_back(co);
    }
  }
}

CollisionObjectWrapper::CollisionObjectWrapper(std::string name,
                                               const int& type_id,
                                               CollisionShapesConst shapes,
                                               tesseract_common::VectorIsometry3d shape_poses,
                                               std::vector<CollisionGeometryPtr> collision_geometries,
                                               const std::vector<CollisionObjectPtr>& collision_objects)
  : name_(std::move(name))
  , type_id_(type_id)
  , shapes_(std::move(shapes))
  , shape_poses_(std::move(shape_poses))
  , collision_geometries_(std::move(collision_geometries))
{
  collision_objects_.reserve(collision_objects.size());
  for (const auto& co : collision_objects)
  {
    CollisionObjectPtr collObj(new fcl::CollisionObjectd(*co));
    collObj->setUserData(this);
    collision_objects_.push_back(collObj);
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

}  // namespace tesseract_collision_fcl
}  // namespace tesseract_collision
