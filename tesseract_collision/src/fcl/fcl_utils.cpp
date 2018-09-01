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

#include <tesseract_collision/fcl/fcl_utils.h>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/bodies.h>
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

namespace tesseract
{
FCLCollisionGeometryPtr createShapePrimitive(const shapes::ShapeConstPtr& geom,
                                             const CollisionObjectType& collision_object_type)
{

  switch (geom->type)
  {
    case shapes::PLANE:
    {
      assert(collision_object_type == CollisionObjectType::UseShapeType);
      const shapes::Plane* p = static_cast<const shapes::Plane*>(geom.get());
      return FCLCollisionGeometryPtr(new fcl::Planed(p->a, p->b, p->c, p->d));
    }
    case shapes::BOX:
    {
      assert(collision_object_type == CollisionObjectType::UseShapeType);
      const shapes::Box* s = static_cast<const shapes::Box*>(geom.get());
      const double* size = s->size;
      return FCLCollisionGeometryPtr(new fcl::Boxd(size[0], size[1], size[2]));
    }
    case shapes::SPHERE:
    {
      assert(collision_object_type == CollisionObjectType::UseShapeType);
      const shapes::Sphere* s = static_cast<const shapes::Sphere*>(geom.get());
      return FCLCollisionGeometryPtr(new fcl::Sphered(s->radius));
    }
    case shapes::CYLINDER:
    {
      assert(collision_object_type == CollisionObjectType::UseShapeType);
      const shapes::Cylinder* s = static_cast<const shapes::Cylinder*>(geom.get());
      return FCLCollisionGeometryPtr(new fcl::Cylinderd(s->radius, s->length));
    }
    case shapes::CONE:
    {
      assert(collision_object_type == CollisionObjectType::UseShapeType);
      const shapes::Cone* s = static_cast<const shapes::Cone*>(geom.get());
      return FCLCollisionGeometryPtr(new fcl::Coned(s->radius, s->length));
    }
    case shapes::MESH:
    {
      assert(collision_object_type == CollisionObjectType::UseShapeType ||
             collision_object_type == CollisionObjectType::ConvexHull ||
             collision_object_type == CollisionObjectType::SDF);

      const shapes::Mesh* mesh = static_cast<const shapes::Mesh*>(geom.get());
      bodies::ConvexMesh convex(geom.get());
      convex.correctVertexOrderFromPlanes();
      bool is_convex = true;
      for (unsigned i = 0; i < mesh->vertex_count; ++i)
      {
        Eigen::Vector3d pt(mesh->vertices[3 * i], mesh->vertices[3 * i + 1], mesh->vertices[3 * i + 2]);
        if (!convex.containsPoint(pt))
        {
          is_convex = false;
          break;
        }
      }

      // convert the mesh to the assigned collision object type
      switch (collision_object_type)
      {
        case CollisionObjectType::ConvexHull:
        {
          const std::vector<unsigned int>& triangles = convex.getTriangles();
          const EigenSTL::vector_Vector3d& vertices = convex.getVertices();
          const EigenSTL::vector_Vector4d& planes = convex.getPlanes();

          int triangle_count = triangles.size()/3;
          Eigen::Vector3d* fcl_vertices = new Eigen::Vector3d[vertices.size()];
          Eigen::Vector3d* fcl_plane_normals = new Eigen::Vector3d[planes.size()];
          double* fcl_plane_dis = new double[planes.size()];
          int* polygons = new int[4 * triangle_count];

          for (unsigned i = 0; i < vertices.size(); ++i)
          {
            fcl_vertices[i] = vertices[i];
          }

          for (unsigned i = 0; i < planes.size(); ++i)
          {
            fcl_plane_normals[i] = planes[i].head<3>();
            fcl_plane_dis[i] = planes[i][3];
          }

          for(auto i = 0; i < triangle_count; ++i)
          {
            int i1 = triangles[3*i + 0];
            int i2 = triangles[3*i + 1];
            int i3 = triangles[3*i + 2];

            polygons[4*i + 0] = 3;
            polygons[4*i + 1] = i1;
            polygons[4*i + 2] = i2;
            polygons[4*i + 3] = i3;
          }

          return FCLCollisionGeometryPtr(new fcl::Convexd(fcl_plane_normals, fcl_plane_dis, planes.size(), fcl_vertices, vertices.size(), polygons));
        }
        case CollisionObjectType::UseShapeType:
        {
          auto g = new fcl::BVHModel<fcl::OBBRSSd>();
          if (mesh->vertex_count > 0 && mesh->triangle_count > 0)
          {
            std::vector<fcl::Triangle> tri_indices(mesh->triangle_count);
            for (unsigned int i = 0; i < mesh->triangle_count; ++i)
              tri_indices[i] =
                  fcl::Triangle(mesh->triangles[3 * i], mesh->triangles[3 * i + 1], mesh->triangles[3 * i + 2]);

            std::vector<Eigen::Vector3d> points(mesh->vertex_count);
            for (unsigned int i = 0; i < mesh->vertex_count; ++i)
              points[i] = Eigen::Vector3d(mesh->vertices[3 * i], mesh->vertices[3 * i + 1], mesh->vertices[3 * i + 2]);

            g->beginModel();
            g->addSubModel(points, tri_indices);
            g->endModel();
          }

          return FCLCollisionGeometryPtr(g);
        }
        default:
        {
          ROS_ERROR("This fcl shape type (%d) is not supported for geometry meshes", (int)collision_object_type);
          return nullptr;
        }
      }
      break;
    }
    case shapes::OCTREE:
    {
      assert(collision_object_type == CollisionObjectType::UseShapeType ||
             collision_object_type == CollisionObjectType::ConvexHull ||
             collision_object_type == CollisionObjectType::SDF ||
             collision_object_type == CollisionObjectType::MultiSphere);
      const shapes::OcTree* g = static_cast<const shapes::OcTree*>(geom.get());

      // convert the mesh to the assigned collision object type
      switch (collision_object_type)
      {
        case CollisionObjectType::UseShapeType:
        {
          return FCLCollisionGeometryPtr(new fcl::OcTreed(g->octree));
        }

        default:
        {
          ROS_ERROR("This fcl shape type (%d) is not supported for geometry octree", (int)collision_object_type);
          return nullptr;
        }
      }
    }
    default:
      ROS_ERROR("This geometric shape type (%d) is not supported using fcl yet", (int)geom->type);
      return nullptr;
  }

  return nullptr;
}

bool collisionCallback(fcl::CollisionObjectd* o1, fcl::CollisionObjectd* o2, void* data)
{
  ContactDistanceData* cdata = reinterpret_cast<ContactDistanceData*>(data);

  if (cdata->done)
    return true;

  const FCLCollisionObjectWrapper* cd1 = static_cast<const FCLCollisionObjectWrapper*>(o1->getUserData());
  const FCLCollisionObjectWrapper* cd2 = static_cast<const FCLCollisionObjectWrapper*>(o2->getUserData());

  bool needs_collision = cd1->m_enabled && cd2->m_enabled &&
      (cd1->m_collisionFilterGroup & cd2->m_collisionFilterMask) &&
      (cd2->m_collisionFilterGroup & cd1->m_collisionFilterMask) &&
      !isContactAllowed(cd1->getName(), cd2->getName(), cdata->req->isContactAllowed, false) &&
      (std::find(cdata->req->link_names.begin(), cdata->req->link_names.end(), cd1->getName()) != cdata->req->link_names.end() ||
       std::find(cdata->req->link_names.begin(), cdata->req->link_names.end(), cd2->getName()) != cdata->req->link_names.end());

  if (!needs_collision)
    return false;

  fcl::CollisionResultd col_result;

  int num_contacts = fcl::collide(
      o1, o2, fcl::CollisionRequestd(1, true, 1, false),
      col_result);

  if (col_result.isCollision())
  {
    ContactResult contact;
    contact.link_names[0] = cd1->getName();
    contact.link_names[1] = cd2->getName();
    contact.nearest_points[0] = Eigen::Vector3d(-1, -1, -1);
    contact.nearest_points[1] = Eigen::Vector3d(-1, -1, -1);
    contact.type_id[0] = cd1->getTypeID();
    contact.type_id[1] = cd2->getTypeID();
    contact.distance = 0;
    contact.normal = Eigen::Vector3d(-1, -1, -1);

    ObjectPairKey pc = getObjectPairKey(cd1->getName(), cd2->getName());
    const auto& it = cdata->res->find(pc);
    bool found = (it != cdata->res->end());

    processResult(*cdata, contact, pc, found);
  }

  return cdata->done;
}

bool distanceCallback(fcl::CollisionObjectd* o1, fcl::CollisionObjectd* o2, void* data, double& min_dist)
{
  ContactDistanceData* cdata = reinterpret_cast<ContactDistanceData*>(data);
  min_dist = cdata->req->contact_distance;

  if (cdata->done)
    return true;

  const FCLCollisionObjectWrapper* cd1 = static_cast<const FCLCollisionObjectWrapper*>(o1->getUserData());
  const FCLCollisionObjectWrapper* cd2 = static_cast<const FCLCollisionObjectWrapper*>(o2->getUserData());

  bool needs_collision = cd1->m_enabled && cd2->m_enabled &&
      (cd1->m_collisionFilterGroup & cd2->m_collisionFilterMask) &&
      (cd2->m_collisionFilterGroup & cd1->m_collisionFilterMask) &&
      !isContactAllowed(cd1->getName(), cd2->getName(), cdata->req->isContactAllowed, false) &&
      (std::find(cdata->req->link_names.begin(), cdata->req->link_names.end(), cd1->getName()) != cdata->req->link_names.end() ||
       std::find(cdata->req->link_names.begin(), cdata->req->link_names.end(), cd2->getName()) != cdata->req->link_names.end());

  if (!needs_collision)
    return false;

  fcl::DistanceResultd fcl_result;
  fcl::DistanceRequestd fcl_request(true, true);
  double d = fcl::distance(o1, o2, fcl_request, fcl_result);

  if (d < cdata->req->contact_distance)
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
      ROS_ERROR("Nearest Points are NAN's");
    }

    ObjectPairKey pc = getObjectPairKey(cd1->getName(), cd2->getName());
    const auto& it = cdata->res->find(pc);
    bool found = (it != cdata->res->end());

    processResult(*cdata, contact, pc, found);
  }

  return cdata->done;
}

FCLCollisionObjectWrapper::FCLCollisionObjectWrapper(const std::string& name,
                                                     const int& type_id,
                                                     const std::vector<shapes::ShapeConstPtr>& shapes,
                                                     const VectorIsometry3d& shape_poses,
                                                     const CollisionObjectTypeVector& collision_object_types)
  : name_(name)
  , type_id_(type_id)
  , shapes_(shapes)
  , shape_poses_(shape_poses)
  , collision_object_types_(collision_object_types)
{
  collision_geometries_.reserve(shapes_.size());
  collision_objects_.reserve(shapes_.size());
  for (std::size_t j = 0; j < shapes_.size(); ++j)
  {
    FCLCollisionGeometryPtr subshape = createShapePrimitive(shapes_[j], collision_object_types_[j]);
    if (subshape != NULL)
    {
      collision_geometries_.push_back(subshape);
      FCLCollisionObjectPtr co(new fcl::CollisionObjectd(subshape));
      co->setUserData(this);
      collision_objects_.push_back(co);
    }
  }
}

FCLCollisionObjectWrapper::FCLCollisionObjectWrapper(const std::string& name,
                                                     const int& type_id,
                                                     const std::vector<shapes::ShapeConstPtr>& shapes,
                                                     const VectorIsometry3d& shape_poses,
                                                     const CollisionObjectTypeVector& collision_object_types,
                                                     const std::vector<FCLCollisionGeometryPtr>& collision_geometries,
                                                     const std::vector<FCLCollisionObjectPtr>& collision_objects)
  : name_(name)
  , type_id_(type_id)
  , shapes_(shapes)
  , shape_poses_(shape_poses)
  , collision_object_types_(collision_object_types)
  , collision_geometries_(collision_geometries)
{
  collision_objects_.reserve(collision_objects.size());
  for (const auto& co : collision_objects)
  {
    FCLCollisionObjectPtr collObj(new fcl::CollisionObjectd(*co));
    collObj->setUserData(this);
    collision_objects_.push_back(collObj);
  }
}

}
