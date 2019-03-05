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
#include <tesseract_collision/core/macros.h>
TESSERACT_COLLISION_IGNORE_WARNINGS_PUSH
#include <fcl/geometry/bvh/BVH_model.h>
#include <fcl/geometry/shape/box.h>
#include <fcl/geometry/shape/cylinder.h>
#include <fcl/geometry/shape/convex.h>
#include <fcl/geometry/shape/plane.h>
#include <fcl/geometry/shape/sphere.h>
#include <fcl/geometry/shape/cone.h>
#include <fcl/geometry/octree/octree.h>
#include <boost/thread/mutex.hpp>
#include <memory>
TESSERACT_COLLISION_IGNORE_WARNINGS_POP

#include <tesseract_collision/fcl/fcl_utils.h>

namespace tesseract_collision
{
namespace tesseract_collision_fcl
{
CollisionGeometryPtr createShapePrimitive(const tesseract_geometry::PlaneConstPtr& geom)
{
  return CollisionGeometryPtr(new fcl::Planed(geom->getA(), geom->getB(), geom->getC(), geom->getD()));
}

CollisionGeometryPtr createShapePrimitive(const tesseract_geometry::BoxConstPtr& geom)
{
  return CollisionGeometryPtr(new fcl::Boxd(geom->getX(), geom->getY(), geom->getZ()));
}

CollisionGeometryPtr createShapePrimitive(const tesseract_geometry::SphereConstPtr& geom)
{
  return CollisionGeometryPtr(new fcl::Sphered(geom->getRadius()));
}

CollisionGeometryPtr createShapePrimitive(const tesseract_geometry::CylinderConstPtr& geom)
{
  return CollisionGeometryPtr(new fcl::Cylinderd(geom->getRadius(), geom->getLength()));
}

CollisionGeometryPtr createShapePrimitive(const tesseract_geometry::ConeConstPtr& geom)
{
  return CollisionGeometryPtr(new fcl::Coned(geom->getRadius(), geom->getLength()));
}

CollisionGeometryPtr createShapePrimitive(const tesseract_geometry::MeshConstPtr& geom)
{
  int vertice_count = geom->getVerticeCount();
  int triangle_count = geom->getTriangleCount();
  const VectorVector3d& vertices = *(geom->getVertices());
  const std::vector<int>& triangles = *(geom->getTriangles());

  auto g = new fcl::BVHModel<fcl::OBBRSSd>();
  if (vertice_count > 0 && triangle_count > 0)
  {
    std::vector<fcl::Triangle> tri_indices(static_cast<size_t>(triangle_count));
    for (int i = 0; i < triangle_count; ++i)
    {
      assert(triangles[static_cast<size_t>(4 * i)] == 3);
      tri_indices[static_cast<size_t>(i)] = fcl::Triangle(triangles[static_cast<size_t>((4 * i) + 1)],
                                                          triangles[static_cast<size_t>((4 * i) + 2)],
                                                          triangles[static_cast<size_t>((4 * i) + 3)]);
    }

    g->beginModel();
    g->addSubModel(vertices, tri_indices);
    g->endModel();

    return CollisionGeometryPtr(g);
  }

  CONSOLE_BRIDGE_logError("The mesh is empty!");
  return nullptr;
}

CollisionGeometryPtr createShapePrimitive(const tesseract_geometry::ConvexMeshConstPtr& geom)
{
  int vertice_count = geom->getVerticeCount();
  int face_count = geom->getFaceCount();

  if (vertice_count > 0 && face_count > 0)
    return CollisionGeometryPtr(new fcl::Convexd(geom->getVertices(), face_count, geom->getFaces()));

  CONSOLE_BRIDGE_logError("The mesh is empty!");
  return nullptr;
}

CollisionGeometryPtr createShapePrimitive(const tesseract_geometry::OctreeConstPtr& geom)
{
  switch (geom->getSubType())
  {
    case tesseract_geometry::Octree::SubType::BOX:
    {
      return CollisionGeometryPtr(new fcl::OcTreed(geom->getOctree()));
    }
    default:
    {
      CONSOLE_BRIDGE_logError("This fcl octree sub shape type (%d) is not supported for geometry octree", static_cast<int>(geom->getSubType()));
      return nullptr;
    }
  }
}

CollisionGeometryPtr createShapePrimitive(const CollisionShapeConstPtr &geom)
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
      CONSOLE_BRIDGE_logError("This geometric shape type (%d) is not supported using fcl yet", static_cast<int>(geom->getType()));
      return nullptr;
    }
  }
}

bool collisionCallback(fcl::CollisionObjectd* o1, fcl::CollisionObjectd* o2, void* data)
{
  ContactTestData* cdata = reinterpret_cast<ContactTestData*>(data);

  if (cdata->done)
    return true;

  const CollisionObjectWrapper* cd1 = static_cast<const CollisionObjectWrapper*>(o1->getUserData());
  const CollisionObjectWrapper* cd2 = static_cast<const CollisionObjectWrapper*>(o2->getUserData());

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
  ContactTestData* cdata = reinterpret_cast<ContactTestData*>(data);
  min_dist = cdata->contact_distance;

  if (cdata->done)
    return true;

  const CollisionObjectWrapper* cd1 = static_cast<const CollisionObjectWrapper*>(o1->getUserData());
  const CollisionObjectWrapper* cd2 = static_cast<const CollisionObjectWrapper*>(o2->getUserData());

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

CollisionObjectWrapper::CollisionObjectWrapper(const std::string& name,
                                               const int& type_id,
                                               const CollisionShapesConst &shapes,
                                               const VectorIsometry3d& shape_poses)
  : name_(name)
  , type_id_(type_id)
  , shapes_(shapes)
  , shape_poses_(shape_poses)
{
  assert(!shapes.empty());
  assert(!shape_poses.empty());
  assert(!name.empty());
  assert(shapes.size() == shape_poses.size());

  collision_geometries_.reserve(shapes_.size());
  collision_objects_.reserve(shapes_.size());
  for (std::size_t j = 0; j < shapes_.size(); ++j)
  {
    CollisionGeometryPtr subshape = createShapePrimitive(shapes_[j]);
    if (subshape != nullptr)
    {
      collision_geometries_.push_back(subshape);
      CollisionObjectPtr co(new fcl::CollisionObjectd(subshape));
      co->setUserData(this);
      collision_objects_.push_back(co);
    }
  }
}

CollisionObjectWrapper::CollisionObjectWrapper(const std::string& name,
                                               const int& type_id,
                                               const CollisionShapesConst &shapes,
                                               const VectorIsometry3d& shape_poses,
                                               const std::vector<CollisionGeometryPtr>& collision_geometries,
                                               const std::vector<CollisionObjectPtr>& collision_objects)
  : name_(name)
  , type_id_(type_id)
  , shapes_(shapes)
  , shape_poses_(shape_poses)
  , collision_geometries_(collision_geometries)
{
  collision_objects_.reserve(collision_objects.size());
  for (const auto& co : collision_objects)
  {
    CollisionObjectPtr collObj(new fcl::CollisionObjectd(*co));
    collObj->setUserData(this);
    collision_objects_.push_back(collObj);
  }
}
}
}
