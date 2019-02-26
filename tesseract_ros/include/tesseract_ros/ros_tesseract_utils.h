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

#include <tesseract_core/macros.h>
TESSERACT_IGNORE_WARNINGS_PUSH
#include <octomap_msgs/conversions.h>
#include <geometric_shapes/shape_messages.h>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/solid_primitive_dims.h>
#include <std_msgs/Int32.h>
#include <eigen_conversions/eigen_msg.h>
#include <ros/console.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <tesseract_msgs/TesseractState.h>
#include <tesseract_msgs/ContactResultVector.h>
#include <tesseract_msgs/Mesh.h>
TESSERACT_IGNORE_WARNINGS_POP

#include <tesseract_ros/ros_basic_env.h>
#include <tesseract_core/basic_types.h>
#include <tesseract_collision/core/collision_shapes.h>
#include <tesseract_collision/core/common.h>

namespace tesseract
{
namespace tesseract_ros
{
static inline bool isMsgEmpty(const sensor_msgs::JointState& msg)
{
  return msg.name.empty() && msg.position.empty() && msg.velocity.empty() && msg.effort.empty();
}

static inline bool isMsgEmpty(const sensor_msgs::MultiDOFJointState& msg)
{
  return msg.joint_names.empty() && msg.transforms.empty() && msg.twist.empty() && msg.wrench.empty();
}

static inline bool isIdentical(const shapes::Shape& shape1, const shapes::Shape& shape2)
{
  if (shape1.type != shape2.type)
    return false;

  switch (shape1.type)
  {
    case shapes::BOX:
    {
      const shapes::Box& s1 = static_cast<const shapes::Box&>(shape1);
      const shapes::Box& s2 = static_cast<const shapes::Box&>(shape2);

      for (unsigned i = 0; i < 3; ++i)
        if (std::abs(s1.size[i] - s2.size[i]) > std::numeric_limits<double>::epsilon())
          return false;

      break;
    }
    case shapes::SPHERE:
    {
      const shapes::Sphere& s1 = static_cast<const shapes::Sphere&>(shape1);
      const shapes::Sphere& s2 = static_cast<const shapes::Sphere&>(shape2);

      if (std::abs(s1.radius - s2.radius) > std::numeric_limits<double>::epsilon())
        return false;

      break;
    }
    case shapes::CYLINDER:
    {
      const shapes::Cylinder& s1 = static_cast<const shapes::Cylinder&>(shape1);
      const shapes::Cylinder& s2 = static_cast<const shapes::Cylinder&>(shape2);

      if (std::abs(s1.radius - s2.radius) > std::numeric_limits<double>::epsilon())
        return false;

      if (std::abs(s1.length - s2.length) > std::numeric_limits<double>::epsilon())
        return false;

      break;
    }
    case shapes::CONE:
    {
      const shapes::Cone& s1 = static_cast<const shapes::Cone&>(shape1);
      const shapes::Cone& s2 = static_cast<const shapes::Cone&>(shape2);

      if (std::abs(s1.radius - s2.radius) > std::numeric_limits<double>::epsilon())
        return false;

      if (std::abs(s1.length - s2.length) > std::numeric_limits<double>::epsilon())
        return false;

      break;
    }
    case shapes::MESH:
    {
      const shapes::Mesh& s1 = static_cast<const shapes::Mesh&>(shape1);
      const shapes::Mesh& s2 = static_cast<const shapes::Mesh&>(shape2);

      if (s1.vertex_count != s2.vertex_count)
        return false;

      if (s1.triangle_count != s2.triangle_count)
        return false;

      break;
    }
    case shapes::OCTREE:
    {
      const shapes::OcTree& s1 = static_cast<const shapes::OcTree&>(shape1);
      const shapes::OcTree& s2 = static_cast<const shapes::OcTree&>(shape2);

      if (s1.octree->getTreeType() != s2.octree->getTreeType())
        return false;

      if (s1.octree->size() != s2.octree->size())
        return false;

      if (s1.octree->getTreeDepth() != s2.octree->getTreeDepth())
        return false;

      if (s1.octree->memoryUsage() != s2.octree->memoryUsage())
        return false;

      if (s1.octree->memoryFullGrid() != s2.octree->memoryFullGrid())
        return false;

      break;
    }
    default:
      ROS_ERROR("This geometric shape type (%d) is not supported", static_cast<int>(shape1.type));
      return false;
  }

  return true;
}

static inline bool isIdentical(const AttachableObject& ao1, const AttachableObject& ao2)
{
  if (ao1.name != ao2.name)
    return false;

  if (ao1.collision.shapes.size() != ao2.collision.shapes.size())
    return false;

  for (unsigned i = 0; i < ao1.collision.shapes.size(); ++i)
  {
    if (!tesseract_collision::CollisionShape::isIdentical(*ao1.collision.shapes[i], *ao2.collision.shapes[i]))
      return false;
  }

  if (ao1.collision.shape_colors.size() != ao2.collision.shape_colors.size())
    return false;

  for (unsigned i = 0; i < ao1.collision.shape_colors.size(); ++i)
  {
    for (unsigned j = 0; j < 4; ++j)
    {
      if (std::abs(ao1.collision.shape_colors[i][j] - ao2.collision.shape_colors[i][j]) >
          std::numeric_limits<double>::epsilon())
        return false;
    }
  }

  // Check Visual
  if (ao1.visual.shapes.size() != ao2.visual.shapes.size())
    return false;

  for (unsigned i = 0; i < ao1.visual.shapes.size(); ++i)
  {
    if (!isIdentical(*ao1.visual.shapes[i], *ao2.visual.shapes[i]))
      return false;
  }

  if (ao1.visual.shape_colors.size() != ao2.visual.shape_colors.size())
    return false;

  for (unsigned i = 0; i < ao1.visual.shape_colors.size(); ++i)
  {
    for (unsigned j = 0; j < 4; ++j)
    {
      if (std::abs(ao1.visual.shape_colors[i][j] - ao2.visual.shape_colors[i][j]) >
          std::numeric_limits<double>::epsilon())
        return false;
    }
  }

  return true;
}

/** \brief Construct the message that corresponds to the shape. Return false on failure. */
inline bool constructMsgFromCollisionShape(const tesseract_collision::CollisionShape& shape, shape_msgs::SolidPrimitive& shape_msgs)
{
  if (shape.getType() == tesseract_collision::CollisionShapeType::SPHERE)
  {
    const tesseract_collision::SphereCollisionShape& sphere = static_cast<const tesseract_collision::SphereCollisionShape&>(shape);

    shape_msgs.type = shape_msgs::SolidPrimitive::SPHERE;
    shape_msgs.dimensions.resize(geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::SPHERE>::value);
    shape_msgs.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS] = sphere.getRadius();
  }
  else if (shape.getType() == tesseract_collision::CollisionShapeType::BOX)
  {
    const tesseract_collision::BoxCollisionShape& box = static_cast<const tesseract_collision::BoxCollisionShape&>(shape);

    shape_msgs.type = shape_msgs::SolidPrimitive::BOX;
    shape_msgs.dimensions.resize(geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
    shape_msgs.dimensions[shape_msgs::SolidPrimitive::BOX_X] = box.getX();
    shape_msgs.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = box.getY();
    shape_msgs.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = box.getZ();
  }
  else if (shape.getType() == tesseract_collision::CollisionShapeType::CYLINDER)
  {
    const tesseract_collision::CylinderCollisionShape& cylinder = static_cast<const tesseract_collision::CylinderCollisionShape&>(shape);

    shape_msgs.type = shape_msgs::SolidPrimitive::CYLINDER;
    shape_msgs.dimensions.resize(geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::CYLINDER>::value);
    shape_msgs.dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] = cylinder.getRadius();
    shape_msgs.dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT] = cylinder.getLength();
  }
  else if (shape.getType() == tesseract_collision::CollisionShapeType::CONE)
  {
    const tesseract_collision::ConeCollisionShape& cone = static_cast<const tesseract_collision::ConeCollisionShape&>(shape);

    shape_msgs.type = shape_msgs::SolidPrimitive::CONE;
    shape_msgs.dimensions.resize(geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::CONE>::value);
    shape_msgs.dimensions[shape_msgs::SolidPrimitive::CONE_RADIUS] = cone.getRadius();
    shape_msgs.dimensions[shape_msgs::SolidPrimitive::CONE_HEIGHT] = cone.getLength();
  }
  else
  {
    ROS_ERROR("Unable to construct primitive shape message for shape of type %d", (int)shape.getType());
    return false;
  }

  return true;
}

inline tesseract_collision::CollisionShapePtr constructCollisionShapeFromMsg(const shape_msgs::SolidPrimitive& shape_msg)
{
  tesseract_collision::CollisionShapePtr shape = nullptr;
  if (shape_msg.type == shape_msgs::SolidPrimitive::SPHERE)
  {
    if (shape_msg.dimensions.size() > shape_msgs::SolidPrimitive::SPHERE_RADIUS)
      shape = tesseract_collision::SphereCollisionShapePtr(new tesseract_collision::SphereCollisionShape(shape_msg.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS]));
  }
  else if (shape_msg.type == shape_msgs::SolidPrimitive::BOX)
  {
    if (shape_msg.dimensions.size() > shape_msgs::SolidPrimitive::BOX_X &&
        shape_msg.dimensions.size() > shape_msgs::SolidPrimitive::BOX_Y &&
        shape_msg.dimensions.size() > shape_msgs::SolidPrimitive::BOX_Z)
      shape = tesseract_collision::BoxCollisionShapePtr(new tesseract_collision::BoxCollisionShape(shape_msg.dimensions[shape_msgs::SolidPrimitive::BOX_X],
                                                                                                   shape_msg.dimensions[shape_msgs::SolidPrimitive::BOX_Y],
                                                                                                   shape_msg.dimensions[shape_msgs::SolidPrimitive::BOX_Z]));
  }
  else if (shape_msg.type == shape_msgs::SolidPrimitive::CYLINDER)
  {
    if (shape_msg.dimensions.size() > shape_msgs::SolidPrimitive::CYLINDER_RADIUS &&
        shape_msg.dimensions.size() > shape_msgs::SolidPrimitive::CYLINDER_HEIGHT)
      shape = tesseract_collision::CylinderCollisionShapePtr(new tesseract_collision::CylinderCollisionShape(shape_msg.dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS],
                                                                                                             shape_msg.dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT]));
  }
  else if (shape_msg.type == shape_msgs::SolidPrimitive::CONE)
  {
    if (shape_msg.dimensions.size() > shape_msgs::SolidPrimitive::CONE_RADIUS &&
        shape_msg.dimensions.size() > shape_msgs::SolidPrimitive::CONE_HEIGHT)
      shape = tesseract_collision::ConeCollisionShapePtr(new tesseract_collision::ConeCollisionShape(shape_msg.dimensions[shape_msgs::SolidPrimitive::CONE_RADIUS],
                                                                                                     shape_msg.dimensions[shape_msgs::SolidPrimitive::CONE_HEIGHT]));
  }
  if (shape == nullptr)
    ROS_ERROR("Unable to construct shape corresponding to shape_msg of type %d", (int)shape_msg.type);

  return shape;
}

static inline void attachableObjectToAttachableObjectMsg(tesseract_msgs::AttachableObject& ao_msg,
                                                         const AttachableObject& ao)
{
  ao_msg.operation = tesseract_msgs::AttachableObject::ADD;
  ao_msg.name = ao.name;

  // Visual Geometry
  for (auto i = 0u; i < ao.visual.shapes.size(); ++i)
  {
    if (ao.visual.shapes[i]->type == shapes::MESH)
    {
      shapes::ShapeMsg shape_msg;
      shapes::constructMsgFromShape(ao.visual.shapes[i].get(), shape_msg);
      ao_msg.visual.meshes.push_back(boost::get<shape_msgs::Mesh>(shape_msg));

      geometry_msgs::Pose pose;
      tf::poseEigenToMsg(ao.visual.shape_poses[i], pose);
      ao_msg.visual.mesh_poses.push_back(pose);

      if (!ao.visual.shape_colors.empty())
      {
        std_msgs::ColorRGBA color;
        color.r = static_cast<float>(ao.visual.shape_colors[i](0));
        color.g = static_cast<float>(ao.visual.shape_colors[i](1));
        color.b = static_cast<float>(ao.visual.shape_colors[i](2));
        color.a = static_cast<float>(ao.visual.shape_colors[i](3));
        ao_msg.visual.mesh_colors.push_back(color);
      }
    }
    else if (ao.visual.shapes[i]->type == shapes::OCTREE)
    {
      octomap_msgs::Octomap octomap_msg;
      const shapes::OcTree* o = static_cast<const shapes::OcTree*>(ao.visual.shapes[i].get());
      octomap_msgs::fullMapToMsg(*o->octree, octomap_msg);
      ao_msg.visual.octomaps.push_back(octomap_msg);

      geometry_msgs::Pose pose;
      tf::poseEigenToMsg(ao.visual.shape_poses[i], pose);
      ao_msg.visual.octomap_poses.push_back(pose);

      if (!ao.visual.shape_colors.empty())
      {
        std_msgs::ColorRGBA color;
        color.r = static_cast<float>(ao.visual.shape_colors[i](0));
        color.g = static_cast<float>(ao.visual.shape_colors[i](1));
        color.b = static_cast<float>(ao.visual.shape_colors[i](2));
        color.a = static_cast<float>(ao.visual.shape_colors[i](3));
        ao_msg.visual.octomap_colors.push_back(color);
      }
    }
    else if (ao.visual.shapes[i]->type == shapes::PLANE)
    {
      shapes::ShapeMsg shape_msg;
      shapes::constructMsgFromShape(ao.visual.shapes[i].get(), shape_msg);
      ao_msg.visual.planes.push_back(boost::get<shape_msgs::Plane>(shape_msg));

      geometry_msgs::Pose pose;
      tf::poseEigenToMsg(ao.visual.shape_poses[i], pose);
      ao_msg.visual.plane_poses.push_back(pose);

      if (!ao.visual.shape_colors.empty())
      {
        std_msgs::ColorRGBA color;
        color.r = static_cast<float>(ao.visual.shape_colors[i](0));
        color.g = static_cast<float>(ao.visual.shape_colors[i](1));
        color.b = static_cast<float>(ao.visual.shape_colors[i](2));
        color.a = static_cast<float>(ao.visual.shape_colors[i](3));
        ao_msg.visual.plane_colors.push_back(color);
      }
    }
    else
    {
      shapes::ShapeMsg shape_msg;
      shapes::constructMsgFromShape(ao.visual.shapes[i].get(), shape_msg);
      ao_msg.visual.primitives.push_back(boost::get<shape_msgs::SolidPrimitive>(shape_msg));

      geometry_msgs::Pose pose;
      tf::poseEigenToMsg(ao.visual.shape_poses[i], pose);
      ao_msg.visual.primitive_poses.push_back(pose);

      if (!ao.visual.shape_colors.empty())
      {
        std_msgs::ColorRGBA color;
        color.r = static_cast<float>(ao.visual.shape_colors[i](0));
        color.g = static_cast<float>(ao.visual.shape_colors[i](1));
        color.b = static_cast<float>(ao.visual.shape_colors[i](2));
        color.a = static_cast<float>(ao.visual.shape_colors[i](3));
        ao_msg.visual.primitive_colors.push_back(color);
      }
    }
  }

  // Collision Geometry
  for (auto i = 0u; i < ao.collision.shapes.size(); ++i)
  {
    if (ao.collision.shapes[i]->getType() == tesseract_collision::CollisionShapeType::MESH)
    {
      const tesseract_collision::MeshCollisionShape& mesh = static_cast<const tesseract_collision::MeshCollisionShape&>(*(ao.collision.shapes[i]));

      tesseract_msgs::Mesh shape_msg;

      const VectorVector3d& vertices = *(mesh.getVertices());
      shape_msg.vertices.resize(vertices.size());
      for (size_t i = 0; i < vertices.size(); ++i)
      {
        shape_msg.vertices[i].x = vertices[i](0);
        shape_msg.vertices[i].y = vertices[i](1);
        shape_msg.vertices[i].z = vertices[i](2);
      }

      const std::vector<int>& faces = *(mesh.getTriangles());
      shape_msg.faces = std::vector<uint32_t>(faces.begin(), faces.end());

      ao_msg.collision.meshes.push_back(shape_msg);

      geometry_msgs::Pose pose;
      tf::poseEigenToMsg(ao.collision.shape_poses[i], pose);
      ao_msg.collision.mesh_poses.push_back(pose);

      if (!ao.collision.shape_colors.empty())
      {
        std_msgs::ColorRGBA color;
        color.r = static_cast<float>(ao.collision.shape_colors[i](0));
        color.g = static_cast<float>(ao.collision.shape_colors[i](1));
        color.b = static_cast<float>(ao.collision.shape_colors[i](2));
        color.a = static_cast<float>(ao.collision.shape_colors[i](3));
        ao_msg.collision.mesh_colors.push_back(color);
      }
    }
    else if (ao.collision.shapes[i]->getType() == tesseract_collision::CollisionShapeType::CONVEX_MESH)
    {
      tesseract_msgs::Mesh shape_msg;

      ao_msg.collision.convex_meshes.push_back(shape_msg);

      geometry_msgs::Pose pose;
      tf::poseEigenToMsg(ao.collision.shape_poses[i], pose);
      ao_msg.collision.convex_mesh_poses.push_back(pose);

      if (!ao.collision.shape_colors.empty())
      {
        std_msgs::ColorRGBA color;
        color.r = static_cast<float>(ao.collision.shape_colors[i](0));
        color.g = static_cast<float>(ao.collision.shape_colors[i](1));
        color.b = static_cast<float>(ao.collision.shape_colors[i](2));
        color.a = static_cast<float>(ao.collision.shape_colors[i](3));
        ao_msg.collision.convex_mesh_colors.push_back(color);
      }
    }
    else if (ao.collision.shapes[i]->getType() == tesseract_collision::CollisionShapeType::SDF_MESH)
    {
      tesseract_msgs::Mesh shape_msg;

      ao_msg.collision.sdf_meshes.push_back(shape_msg);

      geometry_msgs::Pose pose;
      tf::poseEigenToMsg(ao.collision.shape_poses[i], pose);
      ao_msg.collision.sdf_mesh_poses.push_back(pose);

      if (!ao.collision.shape_colors.empty())
      {
        std_msgs::ColorRGBA color;
        color.r = static_cast<float>(ao.collision.shape_colors[i](0));
        color.g = static_cast<float>(ao.collision.shape_colors[i](1));
        color.b = static_cast<float>(ao.collision.shape_colors[i](2));
        color.a = static_cast<float>(ao.collision.shape_colors[i](3));
        ao_msg.collision.sdf_mesh_colors.push_back(color);
      }
    }
    else if (ao.collision.shapes[i]->getType() == tesseract_collision::CollisionShapeType::OCTREE)
    {
      const tesseract_collision::OctreeCollisionShape& octree = static_cast<const tesseract_collision::OctreeCollisionShape&>(*(ao.collision.shapes[i]));

      octomap_msgs::Octomap octomap_msg;
      octomap_msgs::fullMapToMsg(*(octree.getOctree()), octomap_msg);
      ao_msg.collision.octomaps.push_back(octomap_msg);

      geometry_msgs::Pose pose;
      tf::poseEigenToMsg(ao.collision.shape_poses[i], pose);
      ao_msg.collision.octomap_poses.push_back(pose);

      if (!ao.collision.shape_colors.empty())
      {
        std_msgs::ColorRGBA color;
        color.r = static_cast<float>(ao.collision.shape_colors[i](0));
        color.g = static_cast<float>(ao.collision.shape_colors[i](1));
        color.b = static_cast<float>(ao.collision.shape_colors[i](2));
        color.a = static_cast<float>(ao.collision.shape_colors[i](3));
        ao_msg.collision.octomap_colors.push_back(color);
      }
    }
    else if (ao.collision.shapes[i]->getType() == tesseract_collision::CollisionShapeType::PLANE)
    {    
      const tesseract_collision::PlaneCollisionShape& plane = static_cast<const tesseract_collision::PlaneCollisionShape&>(*ao.collision.shapes[i]);

      shape_msgs::Plane shape_msg;
      shape_msg.coef[0] = plane.getA();
      shape_msg.coef[1] = plane.getB();
      shape_msg.coef[2] = plane.getC();
      shape_msg.coef[3] = plane.getD();
      ao_msg.collision.planes.push_back(shape_msg);

      geometry_msgs::Pose pose;
      tf::poseEigenToMsg(ao.collision.shape_poses[i], pose);
      ao_msg.collision.plane_poses.push_back(pose);

      if (!ao.collision.shape_colors.empty())
      {
        std_msgs::ColorRGBA color;
        color.r = static_cast<float>(ao.collision.shape_colors[i](0));
        color.g = static_cast<float>(ao.collision.shape_colors[i](1));
        color.b = static_cast<float>(ao.collision.shape_colors[i](2));
        color.a = static_cast<float>(ao.collision.shape_colors[i](3));
        ao_msg.collision.plane_colors.push_back(color);
      }
    }
    else
    {
      shape_msgs::SolidPrimitive shape_msg;
      constructMsgFromCollisionShape(*(ao.collision.shapes[i]), shape_msg);
      ao_msg.collision.primitives.push_back(shape_msg);

      geometry_msgs::Pose pose;
      tf::poseEigenToMsg(ao.collision.shape_poses[i], pose);
      ao_msg.collision.primitive_poses.push_back(pose);

      if (!ao.collision.shape_colors.empty())
      {
        std_msgs::ColorRGBA color;
        color.r = static_cast<float>(ao.collision.shape_colors[i](0));
        color.g = static_cast<float>(ao.collision.shape_colors[i](1));
        color.b = static_cast<float>(ao.collision.shape_colors[i](2));
        color.a = static_cast<float>(ao.collision.shape_colors[i](3));
        ao_msg.collision.primitive_colors.push_back(color);
      }
    }
  }
}

static inline void attachableObjectToAttachableObjectMsg(tesseract_msgs::AttachableObjectPtr ao_msg,
                                                         const AttachableObject& ao)
{
  attachableObjectToAttachableObjectMsg(*ao_msg, ao);
}

static inline void attachableObjectMsgToAttachableObject(AttachableObject& ao,
                                                         const tesseract_msgs::AttachableObject& ao_msg)
{
  ao.name = ao_msg.name;

  // Visual Geometry
  for (auto i = 0u; i < ao_msg.visual.primitives.size(); ++i)
  {
    shapes::ShapePtr shape(shapes::constructShapeFromMsg(ao_msg.visual.primitives[i]));
    ao.visual.shapes.push_back(shape);

    Eigen::Isometry3d pose;
    tf::poseMsgToEigen(ao_msg.visual.primitive_poses[i], pose);
    ao.visual.shape_poses.push_back(pose);

    if (!ao_msg.visual.primitive_colors.empty())
    {
      const std_msgs::ColorRGBA& c = ao_msg.visual.primitive_colors[i];
      ao.visual.shape_colors.push_back(Eigen::Vector4d(c.r, c.g, c.b, c.a));
    }
  }

  for (auto i = 0u; i < ao_msg.visual.meshes.size(); ++i)
  {
    shapes::ShapePtr shape(shapes::constructShapeFromMsg(ao_msg.visual.meshes[i]));
    ao.visual.shapes.push_back(shape);

    Eigen::Isometry3d pose;
    tf::poseMsgToEigen(ao_msg.visual.mesh_poses[i], pose);
    ao.visual.shape_poses.push_back(pose);

    if (!ao_msg.visual.mesh_colors.empty())
    {
      const std_msgs::ColorRGBA& c = ao_msg.visual.mesh_colors[i];
      ao.visual.shape_colors.push_back(Eigen::Vector4d(c.r, c.g, c.b, c.a));
    }
  }

  for (auto i = 0u; i < ao_msg.visual.planes.size(); ++i)
  {
    shapes::ShapePtr shape(shapes::constructShapeFromMsg(ao_msg.visual.planes[i]));
    ao.visual.shapes.push_back(shape);

    Eigen::Isometry3d pose;
    tf::poseMsgToEigen(ao_msg.visual.plane_poses[i], pose);
    ao.visual.shape_poses.push_back(pose);

    if (!ao_msg.visual.plane_colors.empty())
    {
      const std_msgs::ColorRGBA& c = ao_msg.visual.plane_colors[i];
      ao.visual.shape_colors.push_back(Eigen::Vector4d(c.r, c.g, c.b, c.a));
    }
  }

  for (auto i = 0u; i < ao_msg.visual.octomaps.size(); ++i)
  {
    std::shared_ptr<octomap::OcTree> om(
        static_cast<octomap::OcTree*>(octomap_msgs::msgToMap(ao_msg.visual.octomaps[i])));
    shapes::ShapePtr shape(new shapes::OcTree(om));
    ao.visual.shapes.push_back(shape);

    Eigen::Isometry3d pose;
    tf::poseMsgToEigen(ao_msg.visual.octomap_poses[i], pose);
    ao.visual.shape_poses.push_back(pose);

    if (!ao_msg.visual.octomap_colors.empty())
    {
      const std_msgs::ColorRGBA& c = ao_msg.visual.octomap_colors[i];
      ao.visual.shape_colors.push_back(Eigen::Vector4d(c.r, c.g, c.b, c.a));
    }
  }

  // Collision Geometry
  for (auto i = 0u; i < ao_msg.collision.primitives.size(); ++i)
  {
    tesseract_collision::CollisionShapePtr shape = constructCollisionShapeFromMsg(ao_msg.collision.primitives[i]);
    ao.collision.shapes.push_back(shape);

    Eigen::Isometry3d pose;
    tf::poseMsgToEigen(ao_msg.collision.primitive_poses[i], pose);
    ao.collision.shape_poses.push_back(pose);

    if (!ao_msg.collision.primitive_colors.empty())
    {
      const std_msgs::ColorRGBA& c = ao_msg.collision.primitive_colors[i];
      ao.collision.shape_colors.push_back(Eigen::Vector4d(c.r, c.g, c.b, c.a));
    }
  }

  for (auto i = 0u; i < ao_msg.collision.meshes.size(); ++i)
  { 
    std::shared_ptr<tesseract::VectorVector3d> vertices(new tesseract::VectorVector3d(ao_msg.collision.meshes[i].vertices.size()));
    std::shared_ptr<std::vector<int>> faces(new std::vector<int>(ao_msg.collision.meshes[i].faces.begin(), ao_msg.collision.meshes[i].faces.end()));
    for (unsigned int i = 0; i < ao_msg.collision.meshes[i].vertices.size(); ++i)
      (*vertices)[i] = Eigen::Vector3d(ao_msg.collision.meshes[i].vertices[i].x,
                                       ao_msg.collision.meshes[i].vertices[i].y,
                                       ao_msg.collision.meshes[i].vertices[i].z);

    ao.collision.shapes.push_back(tesseract_collision::MeshCollisionShapePtr(new tesseract_collision::MeshCollisionShape(vertices, faces)));

    Eigen::Isometry3d pose;
    tf::poseMsgToEigen(ao_msg.collision.mesh_poses[i], pose);
    ao.collision.shape_poses.push_back(pose);

    if (!ao_msg.collision.mesh_colors.empty())
    {
      const std_msgs::ColorRGBA& c = ao_msg.collision.mesh_colors[i];
      ao.collision.shape_colors.push_back(Eigen::Vector4d(c.r, c.g, c.b, c.a));
    }
  }

  for (auto i = 0u; i < ao_msg.collision.convex_meshes.size(); ++i)
  {
    std::shared_ptr<tesseract::VectorVector3d> vertices(new tesseract::VectorVector3d(ao_msg.collision.convex_meshes[i].vertices.size()));
    std::shared_ptr<std::vector<int>> faces(new std::vector<int>(ao_msg.collision.convex_meshes[i].faces.begin(), ao_msg.collision.convex_meshes[i].faces.end()));
    for (unsigned int i = 0; i < ao_msg.collision.convex_meshes[i].vertices.size(); ++i)
      (*vertices)[i] = Eigen::Vector3d(ao_msg.collision.convex_meshes[i].vertices[i].x,
                                       ao_msg.collision.convex_meshes[i].vertices[i].y,
                                       ao_msg.collision.convex_meshes[i].vertices[i].z);



    ao.collision.shapes.push_back(tesseract_collision::ConvexMeshCollisionShapePtr(new tesseract_collision::ConvexMeshCollisionShape(vertices, faces)));

    Eigen::Isometry3d pose;
    tf::poseMsgToEigen(ao_msg.collision.convex_mesh_poses[i], pose);
    ao.collision.shape_poses.push_back(pose);

    if (!ao_msg.collision.convex_mesh_colors.empty())
    {
      const std_msgs::ColorRGBA& c = ao_msg.collision.convex_mesh_colors[i];
      ao.collision.shape_colors.push_back(Eigen::Vector4d(c.r, c.g, c.b, c.a));
    }
  }

  for (auto i = 0u; i < ao_msg.collision.sdf_meshes.size(); ++i)
  {
    std::shared_ptr<tesseract::VectorVector3d> vertices(new tesseract::VectorVector3d(ao_msg.collision.sdf_meshes[i].vertices.size()));
    std::shared_ptr<std::vector<int>> faces(new std::vector<int>(ao_msg.collision.sdf_meshes[i].faces.begin(), ao_msg.collision.sdf_meshes[i].faces.end()));
    for (unsigned int i = 0; i < ao_msg.collision.sdf_meshes[i].vertices.size(); ++i)
      (*vertices)[i] = Eigen::Vector3d(ao_msg.collision.sdf_meshes[i].vertices[i].x,
                                       ao_msg.collision.sdf_meshes[i].vertices[i].y,
                                       ao_msg.collision.sdf_meshes[i].vertices[i].z);

    ao.collision.shapes.push_back(tesseract_collision::SDFMeshCollisionShapePtr(new tesseract_collision::SDFMeshCollisionShape(vertices, faces)));

    Eigen::Isometry3d pose;
    tf::poseMsgToEigen(ao_msg.collision.sdf_mesh_poses[i], pose);
    ao.collision.shape_poses.push_back(pose);

    if (!ao_msg.collision.sdf_mesh_colors.empty())
    {
      const std_msgs::ColorRGBA& c = ao_msg.collision.sdf_mesh_colors[i];
      ao.collision.shape_colors.push_back(Eigen::Vector4d(c.r, c.g, c.b, c.a));
    }
  }

  for (auto i = 0u; i < ao_msg.collision.planes.size(); ++i)
  {
    tesseract_collision::CollisionShapePtr shape(new tesseract_collision::PlaneCollisionShape(ao_msg.collision.planes[i].coef[0],
                                                                                              ao_msg.collision.planes[i].coef[1],
                                                                                              ao_msg.collision.planes[i].coef[2],
                                                                                              ao_msg.collision.planes[i].coef[3]));
    ao.collision.shapes.push_back(shape);

    Eigen::Isometry3d pose;
    tf::poseMsgToEigen(ao_msg.collision.plane_poses[i], pose);
    ao.collision.shape_poses.push_back(pose);

    if (!ao_msg.collision.plane_colors.empty())
    {
      const std_msgs::ColorRGBA& c = ao_msg.collision.plane_colors[i];
      ao.collision.shape_colors.push_back(Eigen::Vector4d(c.r, c.g, c.b, c.a));
    }
  }

  for (auto i = 0u; i < ao_msg.collision.octomaps.size(); ++i)
  {
    std::shared_ptr<octomap::OcTree> om(
        static_cast<octomap::OcTree*>(octomap_msgs::msgToMap(ao_msg.collision.octomaps[i])));
    tesseract_collision::CollisionShapePtr shape(new tesseract_collision::OctreeCollisionShape(om, tesseract_collision::OctreeCollisionShape::SubShapeType::BOX)); // TODO: Need to include SubShapeType in message
    ao.collision.shapes.push_back(shape);

    Eigen::Isometry3d pose;
    tf::poseMsgToEigen(ao_msg.collision.octomap_poses[i], pose);
    ao.collision.shape_poses.push_back(pose);

    if (!ao_msg.collision.octomap_colors.empty())
    {
      const std_msgs::ColorRGBA& c = ao_msg.collision.octomap_colors[i];
      ao.collision.shape_colors.push_back(Eigen::Vector4d(c.r, c.g, c.b, c.a));
    }
  }
}

static inline void attachableObjectMsgToAttachableObject(AttachableObjectPtr ao,
                                                         const tesseract_msgs::AttachableObject& ao_msg)
{
  attachableObjectMsgToAttachableObject(*ao, ao_msg);
}

static inline void attachedBodyInfoToAttachedBodyInfoMsg(tesseract_msgs::AttachedBodyInfo& ab_info_msg,
                                                         const AttachedBodyInfo& ab_info)
{
  ab_info_msg.operation = tesseract_msgs::AttachedBodyInfo::ADD;
  ab_info_msg.object_name = ab_info.object_name;
  ab_info_msg.parent_link_name = ab_info.parent_link_name;
  tf::poseEigenToMsg(ab_info.transform, ab_info_msg.transform);
  ab_info_msg.touch_links = ab_info.touch_links;
}

static inline void attachedBodyInfoToAttachedBodyInfoMsg(tesseract_msgs::AttachedBodyInfoPtr ab_info_msg,
                                                         const AttachedBodyInfo& ab_info)
{
  attachedBodyInfoToAttachedBodyInfoMsg(*ab_info_msg, ab_info);
}

static inline void attachedBodyInfoMsgToAttachedBodyInfo(AttachedBodyInfo& ab_info,
                                                         const tesseract_msgs::AttachedBodyInfo& body)
{
  ab_info.object_name = body.object_name;
  ab_info.parent_link_name = body.parent_link_name;
  tf::poseMsgToEigen(body.transform, ab_info.transform);
  ab_info.touch_links = body.touch_links;
}

static inline void tesseractEnvStateToJointStateMsg(sensor_msgs::JointState& joint_state, const EnvState& state)
{
  joint_state.header.stamp = ros::Time::now();
  for (const auto& joint : state.joints)
  {
    joint_state.name.push_back(joint.first);
    joint_state.position.push_back(joint.second);
  }
}

static inline void tesseractEnvStateToJointStateMsg(sensor_msgs::JointStatePtr joint_state, const EnvState& state)
{
  tesseractEnvStateToJointStateMsg(*joint_state, state);
}

static inline void tesseractToTesseractStateMsg(tesseract_msgs::TesseractState& state_msg,
                                                const tesseract_ros::ROSBasicEnv& env)
{
  state_msg.name = env.getName();
  state_msg.urdf_name = env.getURDF()->getName();
  state_msg.is_diff = false;

  for (const auto& entry : env.getAllowedCollisionMatrix()->getAllAllowedCollisions())
  {
    const auto& link_names = entry.first;
    const auto& reason = entry.second;
    tesseract_msgs::AllowedCollisionEntry collision_entry;
    collision_entry.link_1 = link_names.first;
    collision_entry.link_2 = link_names.second;
    collision_entry.reason = reason;
    state_msg.allowed_collisions.push_back(collision_entry);
  }

  for (const auto& ao : env.getAttachableObjects())
  {
    tesseract_msgs::AttachableObject ao_msg;
    attachableObjectToAttachableObjectMsg(ao_msg, *ao.second);
    state_msg.attachable_objects.push_back(ao_msg);
  }

  for (const auto& ab : env.getAttachedBodies())
  {
    tesseract_msgs::AttachedBodyInfo ab_msg;
    attachedBodyInfoToAttachedBodyInfoMsg(ab_msg, ab.second);
    state_msg.attached_bodies.push_back(ab_msg);
  }

  EnvStateConstPtr state = env.getState();
  tesseractEnvStateToJointStateMsg(state_msg.joint_state, *state);
}

static inline void tesseractToTesseractStateMsg(tesseract_msgs::TesseractStatePtr state_msg,
                                                const tesseract_ros::ROSBasicEnv& env)
{
  tesseractToTesseractStateMsg(*state_msg, env);
}

/**
 * @brief Generate a JointTrajectory Message that contains all joints in the environment
 * @param traj_msg The output JointTrajectory Message
 * @param start_state The Environment start/current state
 * @param joint_names The joint names corresponding to the trajectory
 * @param traj The joint trajectory
 */
static inline void tesseractTrajectoryToJointTrajectoryMsg(trajectory_msgs::JointTrajectory& traj_msg,
                                                           const tesseract::EnvState& start_state,
                                                           const std::vector<std::string>& joint_names,
                                                           const Eigen::Ref<const TrajArray>& traj)
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
static inline void tesseractTrajectoryToJointTrajectoryMsg(trajectory_msgs::JointTrajectoryPtr traj_msg,
                                                           const tesseract::EnvState& start_state,
                                                           const std::vector<std::string>& joint_names,
                                                           const Eigen::Ref<const TrajArray>& traj)
{
  tesseractTrajectoryToJointTrajectoryMsg(*traj_msg, start_state, joint_names, traj);
}

/**
 * @brief Generate a JointTrajectory Message that contains only trajectory joints
 * @param traj_msg The output JointTrajectory Message
 * @param joint_names The joint names corresponding to the trajectory
 * @param traj The joint trajectory
 */
static inline void tesseractTrajectoryToJointTrajectoryMsg(trajectory_msgs::JointTrajectory& traj_msg,
                                                           const std::vector<std::string>& joint_names,
                                                           const Eigen::Ref<const TrajArray>& traj)
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
static inline void tesseractTrajectoryToJointTrajectoryMsg(trajectory_msgs::JointTrajectoryPtr traj_msg,
                                                           const std::vector<std::string>& joint_names,
                                                           const Eigen::Ref<const TrajArray>& traj)
{
  tesseractTrajectoryToJointTrajectoryMsg(*traj_msg, joint_names, traj);
}

static inline bool processAttachableObjectMsg(tesseract_ros::ROSBasicEnv& env,
                                              const tesseract_msgs::AttachableObject& ao_msg)
{
  if (ao_msg.operation == tesseract_msgs::AttachableObject::REMOVE)
  {
    env.removeAttachableObject(ao_msg.name);
  }
  else if (ao_msg.operation == tesseract_msgs::AttachableObject::ADD)
  {
    AttachableObjectPtr ao(new AttachableObject());
    attachableObjectMsgToAttachableObject(ao, ao_msg);
    env.addAttachableObject(ao);
  }
  else
  {
    ROS_ERROR("AttachableObject, Unknown operation.");
    return false;
  }

  return true;
}

static inline bool processAttachableObjectMsg(tesseract_ros::ROSBasicEnvPtr env,
                                              const tesseract_msgs::AttachableObject& ao_msg)
{
  return processAttachableObjectMsg(*env, ao_msg);
}

static inline bool processAttachedBodyInfoMsg(tesseract_ros::ROSBasicEnv& env,
                                              const tesseract_msgs::AttachedBodyInfo& ab_msg)
{
  if (ab_msg.operation == tesseract_msgs::AttachedBodyInfo::REMOVE)
  {
    env.detachBody(ab_msg.object_name);
  }
  else if (ab_msg.operation == tesseract_msgs::AttachedBodyInfo::ADD)
  {
    AttachedBodyInfo ab_info;
    tesseract_ros::attachedBodyInfoMsgToAttachedBodyInfo(ab_info, ab_msg);
    env.attachBody(ab_info);
  }
  else if (ab_msg.operation == tesseract_msgs::AttachedBodyInfo::MOVE)
  {
    env.detachBody(ab_msg.object_name);
    AttachedBodyInfo ab_info;
    tesseract_ros::attachedBodyInfoMsgToAttachedBodyInfo(ab_info, ab_msg);
    env.attachBody(ab_info);
  }

  return true;
}

static inline bool processAttachedBodyInfoMsg(tesseract_ros::ROSBasicEnvPtr env,
                                              const tesseract_msgs::AttachedBodyInfo& ab_msg)
{
  return processAttachedBodyInfoMsg(*env, ab_msg);
}

static inline bool processJointStateMsg(tesseract_ros::ROSBasicEnv& env, const sensor_msgs::JointState& joint_state_msg)
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

static inline bool processJointStateMsg(tesseract_ros::ROSBasicEnvPtr env,
                                        const sensor_msgs::JointState& joint_state_msg)
{
  return processJointStateMsg(*env, joint_state_msg);
}

static inline bool processTesseractStateMsg(tesseract_ros::ROSBasicEnv& env,
                                            const tesseract_msgs::TesseractState& state_msg)
{
  bool success = true;

  if (!state_msg.is_diff)
  {
    env.clearAttachedBodies();
    env.clearAttachableObjects();
    env.clearKnownObjectColors();
    env.getAllowedCollisionMatrixNonConst()->clearAllowedCollisions();
  }

  processJointStateMsg(env, state_msg.joint_state);

  for (const auto& ao_msg : state_msg.attachable_objects)
  {
    if (!processAttachableObjectMsg(env, ao_msg))
      success = false;
  }

  for (const auto& ab_msg : state_msg.attached_bodies)
  {
    if (!processAttachedBodyInfoMsg(env, ab_msg))
      success = false;
  }

  for (const auto& ce_msg : state_msg.allowed_collisions)
  {
    auto acm = env.getAllowedCollisionMatrixNonConst();
    acm->addAllowedCollision(ce_msg.link_1, ce_msg.link_2, ce_msg.reason);
  }

  return success;
}

static inline bool processTesseractStateMsg(tesseract_ros::ROSBasicEnvPtr env,
                                            const tesseract_msgs::TesseractState& state_msg)
{
  return processTesseractStateMsg(*env, state_msg);
}

static inline void tesseractContactResultToContactResultMsg(tesseract_msgs::ContactResult& contact_result_msg,
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

static inline void tesseractContactResultToContactResultMsg(tesseract_msgs::ContactResultPtr contact_result_msg,
                                                            const tesseract_collision::ContactResult& contact_result,
                                                            const ros::Time& stamp = ros::Time::now())
{
  tesseractContactResultToContactResultMsg(*contact_result_msg, contact_result, stamp);
}

}
}
#endif  // TESSERACT_ROS_UTILS_H
