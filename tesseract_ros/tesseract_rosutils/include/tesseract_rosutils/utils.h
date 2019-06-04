/**
 * @file ros_tesseract_utils.h
 * @brief Tesseract ROS utility functions.
 *
 * @author Levi Armstrong
 * @date April 15, 2018
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2013, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef TESSERACT_ROS_UTILS_H
#define TESSERACT_ROS_UTILS_H

#include <tesseract_collision/core/macros.h>
TESSERACT_COLLISION_IGNORE_WARNINGS_PUSH
#include <octomap_msgs/conversions.h>
#include <std_msgs/Int32.h>
#include <eigen_conversions/eigen_msg.h>
#include <ros/console.h>
#include <ros/package.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <tesseract_msgs/TesseractState.h>
#include <tesseract_msgs/EnvironmentCommand.h>
#include <tesseract_msgs/ContactResultVector.h>
#include <tesseract_msgs/Mesh.h>
#include <tesseract_msgs/Link.h>
#include <tesseract_msgs/Geometry.h>
#include <tesseract_msgs/Material.h>
#include <tesseract_msgs/Inertial.h>
#include <tesseract_msgs/VisualGeometry.h>
#include <tesseract_msgs/CollisionGeometry.h>

#include <tesseract_msgs/Joint.h>
#include <tesseract_msgs/JointCalibration.h>
#include <tesseract_msgs/JointDynamics.h>
#include <tesseract_msgs/JointLimits.h>
#include <tesseract_msgs/JointMimic.h>
#include <tesseract_msgs/JointSafety.h>

#include <tesseract_environment/core/environment.h>

#include <Eigen/Geometry>
TESSERACT_COLLISION_IGNORE_WARNINGS_POP

#include <tesseract_scene_graph/link.h>
#include <tesseract_geometry/geometries.h>
#include <tesseract_collision/core/common.h>

namespace tesseract_rosutils
{

static inline std::string locateResource(const std::string& url)
{
  std::string mod_url = url;
  if (url.find("package://") == 0)
  {
    mod_url.erase(0, strlen("package://"));
    size_t pos = mod_url.find("/");
    if (pos == std::string::npos)
    {
      return std::string();
    }

    std::string package = mod_url.substr(0, pos);
    mod_url.erase(0, pos);
    std::string package_path = ros::package::getPath(package);

    if (package_path.empty())
    {
      return std::string();
    }

    mod_url = package_path + mod_url;
  }

  return mod_url;
}

static inline bool isMsgEmpty(const sensor_msgs::JointState& msg)
{
  return msg.name.empty() && msg.position.empty() && msg.velocity.empty() && msg.effort.empty();
}

static inline bool isMsgEmpty(const sensor_msgs::MultiDOFJointState& msg)
{
  return msg.joint_names.empty() && msg.transforms.empty() && msg.twist.empty() && msg.wrench.empty();
}

static inline bool isIdentical(const tesseract_geometry::Geometry& shape1, const tesseract_geometry::Geometry& shape2)
{
  if (shape1.getType() != shape2.getType())
    return false;

  switch (shape1.getType())
  {
    case tesseract_geometry::GeometryType::BOX:
    {
      const tesseract_geometry::Box& s1 = static_cast<const tesseract_geometry::Box&>(shape1);
      const tesseract_geometry::Box& s2 = static_cast<const tesseract_geometry::Box&>(shape2);

      if (std::abs(s1.getX() - s2.getX()) > std::numeric_limits<double>::epsilon())
        return false;

      if (std::abs(s1.getY() - s2.getY()) > std::numeric_limits<double>::epsilon())
        return false;

      if (std::abs(s1.getZ() - s2.getZ()) > std::numeric_limits<double>::epsilon())
        return false;

      break;
    }
    case tesseract_geometry::GeometryType::SPHERE:
    {
      const tesseract_geometry::Sphere& s1 = static_cast<const tesseract_geometry::Sphere&>(shape1);
      const tesseract_geometry::Sphere& s2 = static_cast<const tesseract_geometry::Sphere&>(shape2);

      if (std::abs(s1.getRadius() - s2.getRadius()) > std::numeric_limits<double>::epsilon())
        return false;

      break;
    }
    case tesseract_geometry::GeometryType::CYLINDER:
    {
      const tesseract_geometry::Cylinder& s1 = static_cast<const tesseract_geometry::Cylinder&>(shape1);
      const tesseract_geometry::Cylinder& s2 = static_cast<const tesseract_geometry::Cylinder&>(shape2);

      if (std::abs(s1.getRadius() - s2.getRadius()) > std::numeric_limits<double>::epsilon())
        return false;

      if (std::abs(s1.getLength() - s2.getLength()) > std::numeric_limits<double>::epsilon())
        return false;

      break;
    }
    case tesseract_geometry::GeometryType::CONE:
    {
      const tesseract_geometry::Cone& s1 = static_cast<const tesseract_geometry::Cone&>(shape1);
      const tesseract_geometry::Cone& s2 = static_cast<const tesseract_geometry::Cone&>(shape2);

      if (std::abs(s1.getRadius() - s2.getRadius()) > std::numeric_limits<double>::epsilon())
        return false;

      if (std::abs(s1.getLength() - s2.getLength()) > std::numeric_limits<double>::epsilon())
        return false;

      break;
    }
    case tesseract_geometry::GeometryType::MESH:
    {
      const tesseract_geometry::Mesh& s1 = static_cast<const tesseract_geometry::Mesh&>(shape1);
      const tesseract_geometry::Mesh& s2 = static_cast<const tesseract_geometry::Mesh&>(shape2);

      if (s1.getVerticeCount() != s2.getVerticeCount())
        return false;

      if (s1.getTriangleCount() != s2.getTriangleCount())
        return false;

      if (s1.getTriangles() != s2.getTriangles())
        return false;

      if (s1.getVertices() != s2.getVertices())
        return false;

      break;
    }
    case tesseract_geometry::GeometryType::CONVEX_MESH:
    {
      const tesseract_geometry::ConvexMesh& s1 = static_cast<const tesseract_geometry::ConvexMesh&>(shape1);
      const tesseract_geometry::ConvexMesh& s2 = static_cast<const tesseract_geometry::ConvexMesh&>(shape2);

      if (s1.getVerticeCount() != s2.getVerticeCount())
        return false;

      if (s1.getFaceCount() != s2.getFaceCount())
        return false;

      if (s1.getFaces() != s2.getFaces())
        return false;

      if (s1.getVertices() != s2.getVertices())
        return false;

      break;
    }
    case tesseract_geometry::GeometryType::SDF_MESH:
    {
      const tesseract_geometry::Mesh& s1 = static_cast<const tesseract_geometry::Mesh&>(shape1);
      const tesseract_geometry::Mesh& s2 = static_cast<const tesseract_geometry::Mesh&>(shape2);

      if (s1.getVerticeCount() != s2.getVerticeCount())
        return false;

      if (s1.getTriangleCount() != s2.getTriangleCount())
        return false;

      if (s1.getTriangles() != s2.getTriangles())
        return false;

      if (s1.getVertices() != s2.getVertices())
        return false;

      break;
    }
    case tesseract_geometry::GeometryType::OCTREE:
    {
      const tesseract_geometry::Octree& s1 = static_cast<const tesseract_geometry::Octree&>(shape1);
      const tesseract_geometry::Octree& s2 = static_cast<const tesseract_geometry::Octree&>(shape2);

      if (s1.getOctree()->getTreeType() != s2.getOctree()->getTreeType())
        return false;

      if (s1.getOctree()->size() != s2.getOctree()->size())
        return false;

      if (s1.getOctree()->getTreeDepth() != s2.getOctree()->getTreeDepth())
        return false;

      if (s1.getOctree()->memoryUsage() != s2.getOctree()->memoryUsage())
        return false;

      if (s1.getOctree()->memoryFullGrid() != s2.getOctree()->memoryFullGrid())
        return false;

      break;
    }
    default:
      ROS_ERROR("This geometric shape type (%d) is not supported", static_cast<int>(shape1.getType()));
      return false;
  }

  return true;
}

static inline bool isIdentical(const tesseract_scene_graph::Visual& visual1, const tesseract_scene_graph::Visual& visual2)
{
  assert(false);
}

static inline bool isIdentical(const tesseract_scene_graph::Collision& collision1, const tesseract_scene_graph::Collision& collision2)
{
  assert(false);
}

static inline bool isIdentical(const tesseract_scene_graph::Link& link1, const tesseract_scene_graph::Link& link2)
{
  if (link1.getName() != link2.getName())
    return false;

  if (link1.collision.size() != link2.collision.size())
    return false;

  for (unsigned i = 0; i < link1.collision.size(); ++i)
  {
    if (!isIdentical(*link1.collision[i], *link2.collision[i]))
      return false;
  }

  // Check Visual
  if (link1.visual.size() != link2.visual.size())
    return false;

  for (unsigned i = 0; i < link1.visual.size(); ++i)
  {
    if (!isIdentical(*link1.visual[i], *link2.visual[i]))
      return false;
  }

  return true;
}

/** \brief Construct the message that corresponds to the shape. Return false on failure. */
static inline bool toMsg(tesseract_msgs::Geometry& geometry_msgs, const tesseract_geometry::Geometry& geometry)
{
  switch (geometry.getType())
  {
    case tesseract_geometry::GeometryType::SPHERE:
    {
      const tesseract_geometry::Sphere& sphere = static_cast<const tesseract_geometry::Sphere&>(geometry);

      geometry_msgs.type = tesseract_msgs::Geometry::SPHERE;
      geometry_msgs.sphere_radius = sphere.getRadius();
      break;
    }
    case tesseract_geometry::GeometryType::BOX:
    {
      const tesseract_geometry::Box& box = static_cast<const tesseract_geometry::Box&>(geometry);

      geometry_msgs.type = tesseract_msgs::Geometry::BOX;
      geometry_msgs.box_dimensions[0] = box.getX();
      geometry_msgs.box_dimensions[1] = box.getY();
      geometry_msgs.box_dimensions[2] = box.getZ();
      break;
    }
    case tesseract_geometry::GeometryType::CYLINDER:
    {
      const tesseract_geometry::Cylinder& cylinder = static_cast<const tesseract_geometry::Cylinder&>(geometry);

      geometry_msgs.type = tesseract_msgs::Geometry::CYLINDER;
      geometry_msgs.cylinder_dimensions[0] = cylinder.getRadius();
      geometry_msgs.cylinder_dimensions[1] = cylinder.getLength();
      break;
    }
    case tesseract_geometry::GeometryType::CONE:
    {
      const tesseract_geometry::Cone& cone = static_cast<const tesseract_geometry::Cone&>(geometry);

      geometry_msgs.type = tesseract_msgs::Geometry::CONE;
      geometry_msgs.cone_dimensions[0] = cone.getRadius();
      geometry_msgs.cone_dimensions[1] = cone.getLength();
      break;
    }
    case tesseract_geometry::GeometryType::PLANE:
    {
      const tesseract_geometry::Plane& plane = static_cast<const tesseract_geometry::Plane&>(geometry);

      geometry_msgs.type = tesseract_msgs::Geometry::PLANE;
      geometry_msgs.plane_coeff[0] = plane.getA();
      geometry_msgs.plane_coeff[1] = plane.getB();
      geometry_msgs.plane_coeff[2] = plane.getC();
      geometry_msgs.plane_coeff[3] = plane.getD();
      break;
    }
    case tesseract_geometry::GeometryType::OCTREE:
    {
      const tesseract_geometry::Octree& octree = static_cast<const tesseract_geometry::Octree&>(geometry);

      geometry_msgs.type = tesseract_msgs::Geometry::OCTREE;
      octomap_msgs::fullMapToMsg(*(octree.getOctree()), geometry_msgs.octomap);
      break;
    }
    case tesseract_geometry::GeometryType::MESH:
    {
      const tesseract_geometry::Mesh& mesh = static_cast<const tesseract_geometry::Mesh&>(geometry);

      geometry_msgs.type = tesseract_msgs::Geometry::MESH;

      const tesseract_geometry::VectorVector3d& vertices = *(mesh.getVertices());
      geometry_msgs.mesh.vertices.resize(vertices.size());
      for (size_t i = 0; i < vertices.size(); ++i)
      {
        geometry_msgs.mesh.vertices[i].x = vertices[i](0);
        geometry_msgs.mesh.vertices[i].y = vertices[i](1);
        geometry_msgs.mesh.vertices[i].z = vertices[i](2);
      }

      const Eigen::VectorXi& faces = *(mesh.getTriangles());
      geometry_msgs.mesh.faces.resize(faces.size());
      for (size_t i = 0; i < faces.size(); ++i)
        geometry_msgs.mesh.faces[i] = faces[i];

      geometry_msgs.mesh.file_path = mesh.getFilePath();
      if (geometry_msgs.mesh.file_path.empty())
      {
        geometry_msgs.mesh.scale[0] = 1;
        geometry_msgs.mesh.scale[1] = 1;
        geometry_msgs.mesh.scale[2] = 1;
      }
      else
      {
        const Eigen::Vector3d& scale = mesh.getScale();
        geometry_msgs.mesh.scale[0] = scale.x();
        geometry_msgs.mesh.scale[1] = scale.y();
        geometry_msgs.mesh.scale[2] = scale.z();
      }

      break;
    }
    case tesseract_geometry::GeometryType::CONVEX_MESH:
    {
      const tesseract_geometry::ConvexMesh& mesh = static_cast<const tesseract_geometry::ConvexMesh&>(geometry);

      geometry_msgs.type = tesseract_msgs::Geometry::CONVEX_MESH;

      const tesseract_geometry::VectorVector3d& vertices = *(mesh.getVertices());
      geometry_msgs.mesh.vertices.resize(vertices.size());
      for (size_t i = 0; i < vertices.size(); ++i)
      {
        geometry_msgs.mesh.vertices[i].x = vertices[i](0);
        geometry_msgs.mesh.vertices[i].y = vertices[i](1);
        geometry_msgs.mesh.vertices[i].z = vertices[i](2);
      }

      const Eigen::VectorXi& faces = *(mesh.getFaces());
      geometry_msgs.mesh.faces.resize(faces.size());
      for (size_t i = 0; i < faces.size(); ++i)
        geometry_msgs.mesh.faces[i] = faces[i];

      geometry_msgs.mesh.file_path = mesh.getFilePath();
      if (geometry_msgs.mesh.file_path.empty())
      {
        geometry_msgs.mesh.scale[0] = 1;
        geometry_msgs.mesh.scale[1] = 1;
        geometry_msgs.mesh.scale[2] = 1;
      }
      else
      {
        const Eigen::Vector3d& scale = mesh.getScale();
        geometry_msgs.mesh.scale[0] = scale.x();
        geometry_msgs.mesh.scale[1] = scale.y();
        geometry_msgs.mesh.scale[2] = scale.z();
      }

      break;
    }
    case tesseract_geometry::GeometryType::SDF_MESH:
    {
      const tesseract_geometry::SDFMesh& mesh = static_cast<const tesseract_geometry::SDFMesh&>(geometry);

      geometry_msgs.type = tesseract_msgs::Geometry::SDF_MESH;

      const tesseract_geometry::VectorVector3d& vertices = *(mesh.getVertices());
      geometry_msgs.mesh.vertices.resize(vertices.size());
      for (size_t i = 0; i < vertices.size(); ++i)
      {
        geometry_msgs.mesh.vertices[i].x = vertices[i](0);
        geometry_msgs.mesh.vertices[i].y = vertices[i](1);
        geometry_msgs.mesh.vertices[i].z = vertices[i](2);
      }

      const Eigen::VectorXi& faces = *(mesh.getTriangles());
      geometry_msgs.mesh.faces.resize(faces.size());
      for (size_t i = 0; i < faces.size(); ++i)
        geometry_msgs.mesh.faces[i] = faces[i];

      geometry_msgs.mesh.file_path = mesh.getFilePath();
      if (geometry_msgs.mesh.file_path.empty())
      {
        geometry_msgs.mesh.scale[0] = 1;
        geometry_msgs.mesh.scale[1] = 1;
        geometry_msgs.mesh.scale[2] = 1;
      }
      else
      {
        const Eigen::Vector3d& scale = mesh.getScale();
        geometry_msgs.mesh.scale[0] = scale.x();
        geometry_msgs.mesh.scale[1] = scale.y();
        geometry_msgs.mesh.scale[2] = scale.z();
      }

      break;
    }
    default:
    {
      ROS_ERROR("Unable to construct primitive shape message for shape of type %d", static_cast<int>(geometry.getType()));
      return false;
    }
  }

  return true;
}

static inline bool fromMsg(tesseract_geometry::GeometryPtr& geometry, const tesseract_msgs::Geometry& geometry_msg)
{
  geometry = nullptr;
  if (geometry_msg.type == tesseract_msgs::Geometry::SPHERE)
  {
    geometry = tesseract_geometry::SpherePtr(new tesseract_geometry::Sphere(geometry_msg.sphere_radius));
  }
  else if (geometry_msg.type == tesseract_msgs::Geometry::BOX)
  {
    geometry = tesseract_geometry::BoxPtr(new tesseract_geometry::Box(geometry_msg.box_dimensions[0], geometry_msg.box_dimensions[1], geometry_msg.box_dimensions[2]));
  }
  else if (geometry_msg.type == tesseract_msgs::Geometry::CYLINDER)
  {
    geometry = tesseract_geometry::CylinderPtr(new tesseract_geometry::Cylinder(geometry_msg.cylinder_dimensions[0], geometry_msg.cylinder_dimensions[1]));
  }
  else if (geometry_msg.type == tesseract_msgs::Geometry::CONE)
  {
    geometry = tesseract_geometry::ConePtr(new tesseract_geometry::Cone(geometry_msg.cone_dimensions[0], geometry_msg.cone_dimensions[1]));
  }
  else if (geometry_msg.type == tesseract_msgs::Geometry::PLANE)
  {
    geometry = tesseract_geometry::PlanePtr(new tesseract_geometry::Plane(geometry_msg.plane_coeff[0], geometry_msg.plane_coeff[1], geometry_msg.plane_coeff[2], geometry_msg.plane_coeff[3]));
  }
  else if (geometry_msg.type == tesseract_msgs::Geometry::MESH)
  {
    std::shared_ptr<tesseract_geometry::VectorVector3d> vertices(new tesseract_geometry::VectorVector3d(geometry_msg.mesh.vertices.size()));
    std::shared_ptr<Eigen::VectorXi> faces(new Eigen::VectorXi(geometry_msg.mesh.faces.size()));

    for (unsigned int i = 0; i < geometry_msg.mesh.vertices.size(); ++i)
      (*vertices)[i] = Eigen::Vector3d(geometry_msg.mesh.vertices[i].x,
                                       geometry_msg.mesh.vertices[i].y,
                                       geometry_msg.mesh.vertices[i].z);

    for (unsigned int i = 0; i < geometry_msg.mesh.faces.size(); ++i)
      (*faces)[i] = geometry_msg.mesh.faces[i];

    if (!geometry_msg.mesh.file_path.empty())
      geometry = tesseract_geometry::MeshPtr(new tesseract_geometry::Mesh(vertices, faces, geometry_msg.mesh.file_path, Eigen::Vector3d(geometry_msg.mesh.scale[0],geometry_msg.mesh.scale[1],geometry_msg.mesh.scale[2])));
    else
      geometry = tesseract_geometry::MeshPtr(new tesseract_geometry::Mesh(vertices, faces));
  }
  else if (geometry_msg.type == tesseract_msgs::Geometry::CONVEX_MESH)
  {
    std::shared_ptr<tesseract_geometry::VectorVector3d> vertices(new tesseract_geometry::VectorVector3d(geometry_msg.mesh.vertices.size()));
    std::shared_ptr<Eigen::VectorXi> faces(new Eigen::VectorXi(geometry_msg.mesh.faces.size()));

    for (unsigned int i = 0; i < geometry_msg.mesh.vertices.size(); ++i)
      (*vertices)[i] = Eigen::Vector3d(geometry_msg.mesh.vertices[i].x,
                                       geometry_msg.mesh.vertices[i].y,
                                       geometry_msg.mesh.vertices[i].z);

    for (unsigned int i = 0; i < geometry_msg.mesh.faces.size(); ++i)
      (*faces)[i] = geometry_msg.mesh.faces[i];

    if (!geometry_msg.mesh.file_path.empty())
      geometry = tesseract_geometry::ConvexMeshPtr(new tesseract_geometry::ConvexMesh(vertices, faces, geometry_msg.mesh.file_path, Eigen::Vector3d(geometry_msg.mesh.scale[0],geometry_msg.mesh.scale[1],geometry_msg.mesh.scale[2])));
    else
      geometry = tesseract_geometry::ConvexMeshPtr(new tesseract_geometry::ConvexMesh(vertices, faces));
  }
  else if (geometry_msg.type == tesseract_msgs::Geometry::SDF_MESH)
  {
    std::shared_ptr<tesseract_geometry::VectorVector3d> vertices(new tesseract_geometry::VectorVector3d(geometry_msg.mesh.vertices.size()));
    std::shared_ptr<Eigen::VectorXi> faces(new Eigen::VectorXi(geometry_msg.mesh.faces.size()));

    for (unsigned int i = 0; i < geometry_msg.mesh.vertices.size(); ++i)
      (*vertices)[i] = Eigen::Vector3d(geometry_msg.mesh.vertices[i].x,
                                       geometry_msg.mesh.vertices[i].y,
                                       geometry_msg.mesh.vertices[i].z);

    for (unsigned int i = 0; i < geometry_msg.mesh.faces.size(); ++i)
      (*faces)[i] = geometry_msg.mesh.faces[i];

    if (!geometry_msg.mesh.file_path.empty())
      geometry = tesseract_geometry::SDFMeshPtr(new tesseract_geometry::SDFMesh(vertices, faces, geometry_msg.mesh.file_path, Eigen::Vector3d(geometry_msg.mesh.scale[0],geometry_msg.mesh.scale[1],geometry_msg.mesh.scale[2])));
    else
      geometry = tesseract_geometry::SDFMeshPtr(new tesseract_geometry::SDFMesh(vertices, faces));
  }
  else if (geometry_msg.type == tesseract_msgs::Geometry::OCTREE)
  {
    std::shared_ptr<octomap::OcTree> om(static_cast<octomap::OcTree*>(octomap_msgs::msgToMap(geometry_msg.octomap)));
    geometry = tesseract_geometry::GeometryPtr(new tesseract_geometry::Octree(om, tesseract_geometry::Octree::SubType::BOX)); // TODO: Need to include SubShapeType in message
  }

  if (geometry == nullptr)
    ROS_ERROR("Unable to construct shape corresponding to shape_msg of type %d", static_cast<int>(geometry_msg.type));
}


static inline bool toMsg(tesseract_msgs::Material& material_msg, const tesseract_scene_graph::MaterialPtr& material)
{
  if (material == nullptr)
  {
    material_msg.empty = true;
    return true;
  }

  material_msg.name = material->getName();
  material_msg.texture_filename = material->texture_filename;
  material_msg.color.r = material->color(0);
  material_msg.color.g = material->color(1);
  material_msg.color.b = material->color(2);
  material_msg.color.a = material->color(3);
  return true;
}

static inline bool fromMsg(tesseract_scene_graph::MaterialPtr& material, const tesseract_msgs::Material& material_msg)
{
  if (material_msg.empty)
  {
    material = nullptr;
    return true;
  }

  material = std::make_shared<tesseract_scene_graph::Material>(material_msg.name);
  material->texture_filename = material_msg.texture_filename;
  material->color(0) = material_msg.color.r;
  material->color(1) = material_msg.color.g;
  material->color(2) = material_msg.color.b;
  material->color(3) = material_msg.color.a;
  return true;
}

static inline bool toMsg(tesseract_msgs::Inertial& inertial_msg, const tesseract_scene_graph::InertialPtr& inertial)
{
  if (inertial == nullptr)
  {
    inertial_msg.empty = true;
    return true;
  }

  tf::poseEigenToMsg(inertial->origin, inertial_msg.origin);

  inertial_msg.mass = inertial->mass;
  inertial_msg.ixx = inertial->ixx;
  inertial_msg.ixy = inertial->ixy;
  inertial_msg.ixz = inertial->ixz;
  inertial_msg.iyy = inertial->iyy;
  inertial_msg.iyz = inertial->iyz;
  inertial_msg.izz = inertial->izz;

  return true;
}

static inline bool fromMsg(tesseract_scene_graph::InertialPtr& inertial, const tesseract_msgs::Inertial& inertial_msg)
{
  if (inertial_msg.empty)
  {
    inertial = nullptr;
    return true;
  }

  inertial = std::make_shared<tesseract_scene_graph::Inertial>();

  tf::poseMsgToEigen(inertial_msg.origin, inertial->origin);

  inertial->mass = inertial_msg.mass;
  inertial->ixx = inertial_msg.ixx;
  inertial->ixy = inertial_msg.ixy;
  inertial->ixz = inertial_msg.ixz;
  inertial->iyy = inertial_msg.iyy;
  inertial->iyz = inertial_msg.iyz;
  inertial->izz = inertial_msg.izz;

  return true;
}


static inline bool toMsg(tesseract_msgs::VisualGeometry& visual_msg, const tesseract_scene_graph::Visual& visual)
{
  visual_msg.name = visual.name;
  tf::poseEigenToMsg(visual.origin, visual_msg.origin);
  toMsg(visual_msg.geometry, *(visual.geometry));
  toMsg(visual_msg.material, visual.material);
  return true;
}

static inline bool fromMsg(tesseract_scene_graph::VisualPtr& visual, const tesseract_msgs::VisualGeometry& visual_msg)
{
  visual = std::make_shared<tesseract_scene_graph::Visual>();
  visual->name = visual_msg.name;
  tf::poseMsgToEigen(visual_msg.origin, visual->origin);
  fromMsg(visual->geometry, visual_msg.geometry);
  fromMsg(visual->material, visual_msg.material);
  return true;
}

static inline bool toMsg(tesseract_msgs::CollisionGeometry& collision_msg, const tesseract_scene_graph::Collision& collision)
{
  collision_msg.name = collision.name;
  tf::poseEigenToMsg(collision.origin, collision_msg.origin);
  toMsg(collision_msg.geometry, *(collision.geometry));
  return true;
}

static inline bool fromMsg(tesseract_scene_graph::CollisionPtr& collision, const tesseract_msgs::CollisionGeometry& collision_msg)
{
  collision = std::make_shared<tesseract_scene_graph::Collision>();
  collision->name = collision_msg.name;
  tf::poseMsgToEigen(collision_msg.origin, collision->origin);
  fromMsg(collision->geometry, collision_msg.geometry);
  return true;
}

static inline bool toMsg(tesseract_msgs::Link& link_msg, const tesseract_scene_graph::Link& link)
{
  link_msg.name = link.getName();

  toMsg(link_msg.inertial, link.inertial);

  link_msg.collision.resize(link.collision.size());
  for (size_t i = 0; i < link.collision.size(); ++i)
    toMsg(link_msg.collision[i], *(link.collision[i]));

  link_msg.visual.resize(link.visual.size());
  for (size_t i = 0; i < link.visual.size(); ++i)
    toMsg(link_msg.visual[i], *(link.visual[i]));

  return true;
}

static inline tesseract_scene_graph::Link fromMsg(const tesseract_msgs::Link& link_msg)
{
  tesseract_scene_graph::Link link(link_msg.name);

  fromMsg(link.inertial, link_msg.inertial);

  link.collision.resize(link_msg.collision.size());
  for (size_t i = 0; i < link_msg.collision.size(); ++i)
    fromMsg(link.collision[i], link_msg.collision[i]);

  link.visual.resize(link_msg.visual.size());
  for (size_t i = 0; i < link_msg.visual.size(); ++i)
    fromMsg(link.visual[i], link_msg.visual[i]);

  return link;
}

static inline bool toMsg(tesseract_msgs::JointCalibration& joint_calibration_msg, const tesseract_scene_graph::JointCalibrationPtr& joint_calibration)
{
  if (joint_calibration == nullptr)
  {
    joint_calibration_msg.empty = true;
    return true;
  }

  joint_calibration_msg.reference_position = joint_calibration->reference_position;

  joint_calibration_msg.rising = joint_calibration->rising;

  joint_calibration_msg.falling = joint_calibration->falling;

  return true;
}

static inline bool fromMsg(tesseract_scene_graph::JointCalibrationPtr& joint_calibration, const tesseract_msgs::JointCalibration& joint_calibration_msg)
{
  if (joint_calibration_msg.empty)
  {
    joint_calibration = nullptr;
    return true;
  }
  joint_calibration = std::make_shared<tesseract_scene_graph::JointCalibration>();

  joint_calibration->reference_position = joint_calibration_msg.reference_position;

  joint_calibration->rising = joint_calibration_msg.rising;

  joint_calibration->falling = joint_calibration_msg.falling;

  return true;
}

static inline bool toMsg(tesseract_msgs::JointDynamics& joint_dynamics_msg, const tesseract_scene_graph::JointDynamicsPtr& joint_dynamics)
{
  if (joint_dynamics == nullptr)
  {
    joint_dynamics_msg.empty = true;
    return true;
  }

  joint_dynamics_msg.damping = joint_dynamics->damping;

  joint_dynamics_msg.friction = joint_dynamics->friction;

  return true;
}

static inline bool fromMsg(tesseract_scene_graph::JointDynamicsPtr& joint_dynamics, const tesseract_msgs::JointDynamics& joint_dynamics_msg)
{
  if (joint_dynamics_msg.empty)
  {
    joint_dynamics = nullptr;
    return true;
  }

  joint_dynamics = std::make_shared<tesseract_scene_graph::JointDynamics>();

  joint_dynamics->damping = joint_dynamics_msg.damping;

  joint_dynamics->friction = joint_dynamics_msg.friction;

  return true;
}

static inline bool toMsg(tesseract_msgs::JointLimits& joint_limits_msg, const tesseract_scene_graph::JointLimitsPtr& joint_limits)
{

  if (joint_limits == nullptr)
  {
    joint_limits_msg.empty = true;
    return true;
  }

  joint_limits_msg.lower = joint_limits->lower;

  joint_limits_msg.upper = joint_limits->upper;

  joint_limits_msg.effort = joint_limits->effort;

  joint_limits_msg.velocity = joint_limits->velocity;

  return true;
}

static inline bool fromMsg(tesseract_scene_graph::JointLimitsPtr& joint_limits, const tesseract_msgs::JointLimits& joint_limits_msg)
{
  if (joint_limits_msg.empty)
  {
    joint_limits = nullptr;
    return true;
  }

  joint_limits = std::make_shared<tesseract_scene_graph::JointLimits>();

  joint_limits->lower = joint_limits_msg.lower;

  joint_limits->upper = joint_limits_msg.upper;

  joint_limits->effort = joint_limits_msg.effort;

  joint_limits->velocity = joint_limits_msg.velocity;

  return true;
}

static inline bool toMsg(tesseract_msgs::JointMimic& joint_mimic_msg, const tesseract_scene_graph::JointMimicPtr& joint_mimic)
{
  if (joint_mimic == nullptr)
  {
    joint_mimic_msg.empty = true;
    return true;
  }

  joint_mimic_msg.offset = joint_mimic->offset;

  joint_mimic_msg.multiplier = joint_mimic->multiplier;

  joint_mimic_msg.joint_name = joint_mimic->joint_name;

  return true;
}

static inline bool fromMsg(tesseract_scene_graph::JointMimicPtr& joint_mimic, const tesseract_msgs::JointMimic& joint_mimic_msg)
{
  if (joint_mimic_msg.empty)
  {
    joint_mimic = nullptr;
    return true;
  }

  joint_mimic = std::make_shared<tesseract_scene_graph::JointMimic>();

  joint_mimic->offset = joint_mimic_msg.offset;

  joint_mimic->multiplier = joint_mimic_msg.multiplier;

  joint_mimic->joint_name = joint_mimic_msg.joint_name;

  return true;
}

static inline bool toMsg(tesseract_msgs::JointSafety& joint_safety_msg, const tesseract_scene_graph::JointSafetyPtr& joint_safety)
{
  if (joint_safety == nullptr)
  {
    joint_safety_msg.empty = true;
    return true;
  }

  joint_safety_msg.soft_upper_limit = joint_safety->soft_upper_limit;

  joint_safety_msg.soft_lower_limit = joint_safety->soft_lower_limit;

  joint_safety_msg.k_position = joint_safety->k_position;

  joint_safety_msg.k_velocity = joint_safety->k_velocity;

  return true;
}

static inline bool fromMsg(tesseract_scene_graph::JointSafetyPtr& joint_safety, const tesseract_msgs::JointSafety& joint_safety_msg)
{
  if (joint_safety_msg.empty)
  {
    joint_safety = nullptr;
    return true;
  }

  joint_safety = std::make_shared<tesseract_scene_graph::JointSafety>();

  joint_safety->soft_upper_limit = joint_safety_msg.soft_upper_limit;

  joint_safety->soft_lower_limit = joint_safety_msg.soft_lower_limit;

  joint_safety->k_position = joint_safety_msg.k_position;

  joint_safety->k_velocity = joint_safety_msg.k_velocity;

  return true;
}

static inline bool toMsg(tesseract_msgs::Joint& joint_msg, const tesseract_scene_graph::Joint& joint)
{
  joint_msg.name = joint.getName();
  joint_msg.type = static_cast<int>(joint.type);

  joint_msg.axis[0] = joint.axis[0];
  joint_msg.axis[1] = joint.axis[1];
  joint_msg.axis[2] = joint.axis[2];

  joint_msg.child_link_name = joint.child_link_name;
  joint_msg.parent_link_name = joint.parent_link_name;

  tf::poseEigenToMsg(joint.parent_to_joint_origin_transform, joint_msg.parent_to_joint_origin_transform);
  toMsg(joint_msg.limits, joint.limits);
  toMsg(joint_msg.dynamics, joint.dynamics);
  toMsg(joint_msg.safety, joint.safety);
  toMsg(joint_msg.calibration, joint.calibration);
  toMsg(joint_msg.mimic, joint.mimic);
}

static inline tesseract_scene_graph::Joint fromMsg(const tesseract_msgs::Joint& joint_msg)
{
  tesseract_scene_graph::Joint joint(joint_msg.name);

  joint.type = static_cast<tesseract_scene_graph::JointType>(joint_msg.type);

  joint.axis[0] = joint_msg.axis[0];
  joint.axis[1] = joint_msg.axis[1];
  joint.axis[2] = joint_msg.axis[2];

  joint.child_link_name = joint_msg.child_link_name;
  joint.parent_link_name = joint_msg.parent_link_name;

  tf::poseMsgToEigen(joint_msg.parent_to_joint_origin_transform, joint.parent_to_joint_origin_transform);
  fromMsg(joint.limits, joint_msg.limits);
  fromMsg(joint.dynamics, joint_msg.dynamics);
  fromMsg(joint.safety, joint_msg.safety);
  fromMsg(joint.calibration, joint_msg.calibration);
  fromMsg(joint.mimic, joint_msg.mimic);

  return joint;
}

static inline void toMsg(sensor_msgs::JointState& joint_state, const tesseract_environment::EnvState& state)
{
  joint_state.header.stamp = ros::Time::now();
  for (const auto& joint : state.joints)
  {
    joint_state.name.push_back(joint.first);
    joint_state.position.push_back(joint.second);
  }
}

static inline bool toMsg(tesseract_msgs::EnvironmentCommand& command_msg, const tesseract_environment::Command& command)
{
  switch (command.getType())
  {
    case tesseract_environment::CommandType::ADD:
    {
      command_msg.command = tesseract_msgs::EnvironmentCommand::ADD;
      const tesseract_environment::AddCommand& cmd = static_cast<const tesseract_environment::AddCommand&>(command);
      tesseract_rosutils::toMsg(command_msg.add_link, *(cmd.getLink()));
      tesseract_rosutils::toMsg(command_msg.add_joint, *(cmd.getJoint()));
      return true;
    }
    case tesseract_environment::CommandType::MOVE_LINK:
    {
      command_msg.command = tesseract_msgs::EnvironmentCommand::MOVE_LINK;
      const tesseract_environment::MoveLinkCommand& cmd = static_cast<const tesseract_environment::MoveLinkCommand&>(command);
      tesseract_rosutils::toMsg(command_msg.move_link_joint, *(cmd.getJoint()));
      return true;
    }
    case tesseract_environment::CommandType::MOVE_JOINT:
    {
      command_msg.command = tesseract_msgs::EnvironmentCommand::MOVE_JOINT;
      const tesseract_environment::MoveJointCommand& cmd = static_cast<const tesseract_environment::MoveJointCommand&>(command);
      command_msg.move_joint_name = cmd.getJointName();
      command_msg.move_joint_parent_link = cmd.getParentLink();
      return true;
    }
    case tesseract_environment::CommandType::REMOVE_LINK:
    {
      command_msg.command = tesseract_msgs::EnvironmentCommand::REMOVE_LINK;
      const tesseract_environment::RemoveLinkCommand& cmd = static_cast<const tesseract_environment::RemoveLinkCommand&>(command);

      command_msg.remove_link = cmd.getLinkName();
      return true;
    }
    case tesseract_environment::CommandType::REMOVE_JOINT:
    {
      command_msg.command = tesseract_msgs::EnvironmentCommand::REMOVE_JOINT;
      const tesseract_environment::RemoveJointCommand& cmd = static_cast<const tesseract_environment::RemoveJointCommand&>(command);
      command_msg.remove_joint = cmd.getJointName();
      return true;
    }
    case tesseract_environment::CommandType::CHANGE_LINK_ORIGIN:
    {
      assert(false);
    }
    case tesseract_environment::CommandType::CHANGE_JOINT_ORIGIN:
    {
      command_msg.command = tesseract_msgs::EnvironmentCommand::CHANGE_JOINT_ORIGIN;
      const tesseract_environment::ChangeJointOriginCommand& cmd = static_cast<const tesseract_environment::ChangeJointOriginCommand&>(command);
      command_msg.change_joint_orgin_name = cmd.getJointName();
      tf::poseEigenToMsg(cmd.getOrigin(), command_msg.change_joint_orgin_pose);
      return true;
    }
    case tesseract_environment::CommandType::CHANGE_LINK_COLLISION_ENABLED:
    {
      command_msg.command = tesseract_msgs::EnvironmentCommand::CHANGE_LINK_COLLISION_ENABLED;
      const tesseract_environment::ChangeLinkCollisionEnabledCommand& cmd = static_cast<const tesseract_environment::ChangeLinkCollisionEnabledCommand&>(command);

      command_msg.change_link_collision_enabled_name = cmd.getLinkName();
      command_msg.change_link_collision_enabled_value = cmd.getEnabled();
      return true;
    }
    case tesseract_environment::CommandType::CHANGE_LINK_VISIBILITY:
    {
      command_msg.command = tesseract_msgs::EnvironmentCommand::CHANGE_LINK_VISIBILITY;
      const tesseract_environment::ChangeLinkVisibilityCommand& cmd = static_cast<const tesseract_environment::ChangeLinkVisibilityCommand&>(command);

      command_msg.change_link_visibility_name = cmd.getLinkName();
      command_msg.change_link_visibility_value = cmd.getEnabled();
      return true;
    }
    case tesseract_environment::CommandType::ADD_ALLOWED_COLLISION:
    {
      command_msg.command = tesseract_msgs::EnvironmentCommand::ADD_ALLOWED_COLLISION;
      const tesseract_environment::AddAllowedCollisionCommand& cmd = static_cast<const tesseract_environment::AddAllowedCollisionCommand&>(command);

      command_msg.add_allowed_collision.link_1 = cmd.getLinkName1();
      command_msg.add_allowed_collision.link_2 = cmd.getLinkName2();
      command_msg.add_allowed_collision.reason = cmd.getReason();
      return true;
    }
    case tesseract_environment::CommandType::REMOVE_ALLOWED_COLLISION:
    {
      command_msg.command = tesseract_msgs::EnvironmentCommand::REMOVE_ALLOWED_COLLISION;
      const tesseract_environment::RemoveAllowedCollisionCommand& cmd = static_cast<const tesseract_environment::RemoveAllowedCollisionCommand&>(command);

      command_msg.add_allowed_collision.link_1 = cmd.getLinkName1();
      command_msg.add_allowed_collision.link_2 = cmd.getLinkName2();
      return true;
    }
    case tesseract_environment::CommandType::REMOVE_ALLOWED_COLLISION_LINK:
    {
      command_msg.command = tesseract_msgs::EnvironmentCommand::REMOVE_ALLOWED_COLLISION_LINK;
      const tesseract_environment::RemoveAllowedCollisionLinkCommand& cmd = static_cast<const tesseract_environment::RemoveAllowedCollisionLinkCommand&>(command);
      command_msg.remove_allowed_collision_link = cmd.getLinkName();
      return true;
    }
  }

  return false;
}

static inline bool toMsg(std::vector<tesseract_msgs::EnvironmentCommand>& commands_msg, const tesseract_environment::Commands& commands, const int past_revision)
{
  for (int i = past_revision; i < commands.size(); ++i)
  {
    tesseract_msgs::EnvironmentCommand command_msg;
    if (!tesseract_rosutils::toMsg(command_msg, *(commands[i])))
      return false;

    commands_msg.push_back(command_msg);
  }

  return true;
}

static inline void toMsg(const sensor_msgs::JointStatePtr& joint_state, const tesseract_environment::EnvState& state)
{
  toMsg(*joint_state, state);
}

static inline void toMsg(tesseract_msgs::TesseractState& state_msg, const tesseract_environment::Environment& env)
{
  state_msg.id = env.getName();
  state_msg.revision = env.getRevision();
  toMsg(state_msg.commands, env.getCommandHistory(), 0);

  tesseract_environment::EnvStateConstPtr state = env.getCurrentState();
  toMsg(state_msg.joint_state, *state);
}

static inline void toMsg(const tesseract_msgs::TesseractStatePtr& state_msg, const tesseract_environment::Environment& env)
{
  toMsg(*state_msg, env);
}

/**
 * @brief Generate a JointTrajectory Message that contains all joints in the environment
 * @param traj_msg The output JointTrajectory Message
 * @param start_state The Environment start/current state
 * @param joint_names The joint names corresponding to the trajectory
 * @param traj The joint trajectory
 */
static inline void toMsg(trajectory_msgs::JointTrajectory& traj_msg,
                         const tesseract_environment::EnvState& start_state,
                         const std::vector<std::string>& joint_names,
                         const Eigen::Ref<const tesseract_environment::TrajArray>& traj)
{
  assert(joint_names.size() == static_cast<unsigned>(traj.cols()));

  // Initialze the whole traject with the current state.
  std::map<std::string, int> jn_to_index;
  traj_msg.joint_names.resize(start_state.joints.size());
  traj_msg.points.resize(static_cast<size_t>(traj.rows()));
  for (int i = 0; i < traj.rows(); ++i)
  {
    trajectory_msgs::JointTrajectoryPoint jtp;
    jtp.positions.resize(start_state.joints.size());

    int j = 0;
    for (const auto& joint : start_state.joints)
    {
      if (i == 0)
      {
        traj_msg.joint_names[static_cast<size_t>(j)] = joint.first;
        jn_to_index[joint.first] = j;
      }
      jtp.positions[static_cast<size_t>(j)] = joint.second;

      ++j;
    }
    jtp.time_from_start = ros::Duration(i);
    traj_msg.points[static_cast<size_t>(i)] = jtp;
  }

  // Update only the joints which were provided.
  for (int i = 0; i < traj.rows(); ++i)
  {
    for (int j = 0; j < traj.cols(); ++j)
    {
      traj_msg.points[static_cast<size_t>(i)]
          .positions[static_cast<size_t>(jn_to_index[joint_names[static_cast<size_t>(j)]])] = traj(i, j);
    }
  }
}

/**
 * @brief Generate a JointTrajectory Message that contains all joints in the environment
 * @param traj_msg The output JointTrajectory Message
 * @param start_state The Environment start/current state
 * @param joint_names The joint names corresponding to the trajectory
 * @param traj The joint trajectory
 */
static inline void toMsg(const trajectory_msgs::JointTrajectoryPtr& traj_msg,
                         const tesseract_environment::EnvState& start_state,
                         const std::vector<std::string>& joint_names,
                         const Eigen::Ref<const tesseract_environment::TrajArray>& traj)
{
  toMsg(*traj_msg, start_state, joint_names, traj);
}

/**
 * @brief Generate a JointTrajectory Message that contains only trajectory joints
 * @param traj_msg The output JointTrajectory Message
 * @param joint_names The joint names corresponding to the trajectory
 * @param traj The joint trajectory
 */
static inline void toMsg(trajectory_msgs::JointTrajectory& traj_msg,
                         const std::vector<std::string>& joint_names,
                         const Eigen::Ref<const tesseract_environment::TrajArray>& traj)
{
  assert(joint_names.size() == static_cast<unsigned>(traj.cols()));

  // Initialze the whole traject with the current state.
  std::map<std::string, int> jn_to_index;
  traj_msg.joint_names.resize(joint_names.size());
  traj_msg.points.resize(static_cast<size_t>(traj.rows()));

  for (int i = 0; i < traj.rows(); ++i)
  {
    trajectory_msgs::JointTrajectoryPoint jtp;
    jtp.positions.resize(static_cast<size_t>(traj.cols()));

    for (int j = 0; j < traj.cols(); ++j)
    {
      if (i == 0)
        traj_msg.joint_names[static_cast<size_t>(j)] = joint_names[static_cast<size_t>(j)];

      jtp.positions[static_cast<size_t>(j)] = traj(i, j);
    }

    jtp.time_from_start = ros::Duration(i);
    traj_msg.points[static_cast<size_t>(i)] = jtp;
  }
}

/**
 * @brief Generate a JointTrajectory Message that contains only trajectory joints
 * @param traj_msg The output JointTrajectory Message
 * @param joint_names The joint names corresponding to the trajectory
 * @param traj The joint trajectory
 */
static inline void toMsg(const trajectory_msgs::JointTrajectoryPtr& traj_msg,
                         const std::vector<std::string>& joint_names,
                         const Eigen::Ref<const tesseract_environment::TrajArray>& traj)
{
  toMsg(*traj_msg, joint_names, traj);
}

static inline bool processMsg(tesseract_environment::Environment& env, const sensor_msgs::JointState& joint_state_msg)
{
  if (!isMsgEmpty(joint_state_msg))
  {
    std::unordered_map<std::string, double> joints;
    for (auto i = 0u; i < joint_state_msg.name.size(); ++i)
    {
      joints[joint_state_msg.name[i]] = joint_state_msg.position[i];
    }
    env.setState(joints);
    return true;
  }
  return false;
}

static inline bool processMsg(tesseract_environment::Environment& env, const std::vector<tesseract_msgs::EnvironmentCommand>& env_command_msg)
{
  for (const auto& command : env_command_msg)
  {
    switch (command.command)
    {
      case tesseract_msgs::EnvironmentCommand::ADD:
      {
        tesseract_scene_graph::Link link = tesseract_rosutils::fromMsg(command.add_link);
        tesseract_scene_graph::Joint joint = tesseract_rosutils::fromMsg(command.add_joint);
        return env.addLink(link, joint);
      }
      case tesseract_msgs::EnvironmentCommand::MOVE_LINK:
      {
        tesseract_scene_graph::Joint joint = tesseract_rosutils::fromMsg(command.move_link_joint);
        return env.moveLink(joint);
      }
      case tesseract_msgs::EnvironmentCommand::MOVE_JOINT:
      {
        return env.moveJoint(command.move_joint_name, command.move_joint_parent_link);
      }
      case tesseract_msgs::EnvironmentCommand::REMOVE_LINK:
      {
        return env.removeLink(command.remove_link);
      }
      case tesseract_msgs::EnvironmentCommand::REMOVE_JOINT:
      {
        return env.removeJoint(command.remove_joint);
      }
      case tesseract_msgs::EnvironmentCommand::CHANGE_LINK_ORIGIN:
      {
        assert(false);
      }
      case tesseract_msgs::EnvironmentCommand::CHANGE_JOINT_ORIGIN:
      {
        Eigen::Isometry3d pose;
        tf::poseMsgToEigen(command.change_joint_orgin_pose, pose);
        return env.changeJointOrigin(command.change_joint_orgin_name, pose);
      }
      case tesseract_msgs::EnvironmentCommand::CHANGE_LINK_COLLISION_ENABLED:
      {
        env.setLinkCollisionEnabled(command.change_link_collision_enabled_name, command.change_link_collision_enabled_value);
        return true;
      }
      case tesseract_msgs::EnvironmentCommand::CHANGE_LINK_VISIBILITY:
      {
        env.setLinkVisibility(command.change_link_visibility_name, command.change_link_visibility_value);
        return true;
      }
      case tesseract_msgs::EnvironmentCommand::ADD_ALLOWED_COLLISION:
      {
        env.addAllowedCollision(command.add_allowed_collision.link_1, command.add_allowed_collision.link_2, command.add_allowed_collision.reason);
        return true;
      }
      case tesseract_msgs::EnvironmentCommand::REMOVE_ALLOWED_COLLISION:
      {
        env.removeAllowedCollision(command.add_allowed_collision.link_1, command.add_allowed_collision.link_2);
        return true;
      }
      case tesseract_msgs::EnvironmentCommand::REMOVE_ALLOWED_COLLISION_LINK:
      {
        env.removeAllowedCollision(command.remove_allowed_collision_link);
        return true;
      }
      case tesseract_msgs::EnvironmentCommand::UPDATE_JOINT_STATE:
      {
        return processMsg(env, command.joint_state);
      }
    }
  }

  return false;
}

static inline bool processMsg(const tesseract_environment::EnvironmentPtr& env,
                              const sensor_msgs::JointState& joint_state_msg)
{
  return processMsg(*env, joint_state_msg);
}

static inline bool processMsg(tesseract_environment::Environment& env,
                              const tesseract_msgs::TesseractState& state_msg)
{
  if (state_msg.id != env.getName() || env.getRevision() > state_msg.revision)
    return false;

  if (!processMsg(env, state_msg.commands))
    return false;

  if (!processMsg(env, state_msg.joint_state))
    return false;

  return true;
}

static inline bool processMsg(const tesseract_environment::EnvironmentPtr& env,
                              const tesseract_msgs::TesseractState& state_msg)
{
  return processMsg(*env, state_msg);
}

static inline void toMsg(tesseract_msgs::ContactResult& contact_result_msg,
                         const tesseract_collision::ContactResult& contact_result,
                         const ros::Time& stamp = ros::Time::now())
{
  contact_result_msg.stamp = stamp;
  contact_result_msg.distance = contact_result.distance;
  contact_result_msg.link_names[0] = contact_result.link_names[0];
  contact_result_msg.link_names[1] = contact_result.link_names[1];
  contact_result_msg.normal.x = contact_result.normal[0];
  contact_result_msg.normal.y = contact_result.normal[1];
  contact_result_msg.normal.z = contact_result.normal[2];
  contact_result_msg.nearest_points[0].x = contact_result.nearest_points[0][0];
  contact_result_msg.nearest_points[0].y = contact_result.nearest_points[0][1];
  contact_result_msg.nearest_points[0].z = contact_result.nearest_points[0][2];
  contact_result_msg.nearest_points[1].x = contact_result.nearest_points[1][0];
  contact_result_msg.nearest_points[1].y = contact_result.nearest_points[1][1];
  contact_result_msg.nearest_points[1].z = contact_result.nearest_points[1][2];
  contact_result_msg.cc_time = contact_result.cc_time;
  contact_result_msg.cc_nearest_points[0].x = contact_result.cc_nearest_points[0][0];
  contact_result_msg.cc_nearest_points[0].y = contact_result.cc_nearest_points[0][1];
  contact_result_msg.cc_nearest_points[0].z = contact_result.cc_nearest_points[0][2];
  contact_result_msg.cc_nearest_points[1].x = contact_result.cc_nearest_points[1][0];
  contact_result_msg.cc_nearest_points[1].y = contact_result.cc_nearest_points[1][1];
  contact_result_msg.cc_nearest_points[1].z = contact_result.cc_nearest_points[1][2];

  contact_result_msg.type_id[0] = static_cast<char>(contact_result.type_id[0]);
  contact_result_msg.type_id[1] = static_cast<char>(contact_result.type_id[1]);

  if (contact_result.cc_type == tesseract_collision::ContinouseCollisionTypes::CCType_Time0)
    contact_result_msg.cc_type = 1;
  else if (contact_result.cc_type == tesseract_collision::ContinouseCollisionTypes::CCType_Time1)
    contact_result_msg.cc_type = 2;
  else if (contact_result.cc_type == tesseract_collision::ContinouseCollisionTypes::CCType_Between)
    contact_result_msg.cc_type = 3;
  else
    contact_result_msg.cc_type = 0;
}

static inline void toMsg(const tesseract_msgs::ContactResultPtr& contact_result_msg,
                         const tesseract_collision::ContactResult& contact_result,
                         const ros::Time& stamp = ros::Time::now())
{
  toMsg(*contact_result_msg, contact_result, stamp);
}

}

#endif
