/**
 * @file kdl_utils.h
 * @brief Tesseract Environment KDL utility functions.
 *
 * @author Levi Armstrong
 * @date May 27, 2018
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
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
#ifndef TESSERACT_ENVIRONMENT_KDL_UTILS_H
#define TESSERACT_ENVIRONMENT_KDL_UTILS_H

#include <tesseract_environment/core/macros.h>
TESSERACT_ENVIRONMENT_IGNORE_WARNINGS_PUSH
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <Eigen/Eigen>
#include <console_bridge/console.h>
TESSERACT_ENVIRONMENT_IGNORE_WARNINGS_POP

#include <tesseract_collision/core/collision_shapes.h>
#include <tesseract_collision/core/common.h>

namespace tesseract_environment
{
/**
 * @brief Convert KDL::Frame to Eigen::Isometry3d
 * @param frame Input KDL Frame
 * @param transform Output Eigen transform (Isometry3d)
 */
inline void KDLToEigen(const KDL::Frame& frame, Eigen::Isometry3d& transform)
{
  transform.setIdentity();

  // translation
  for (int i = 0; i < 3; ++i)
    transform(i, 3) = frame.p[i];

  // rotation matrix
  for (int i = 0; i < 9; ++i)
    transform(i / 3, i % 3) = frame.M.data[i];
}

/**
 * @brief Convert Eigen::Isometry3d to KDL::Frame
 * @param transform Input Eigen transform (Isometry3d)
 * @param frame Output KDL Frame
 */
inline void EigenToKDL(const Eigen::Isometry3d& transform, KDL::Frame& frame)
{
  frame.Identity();

  for (int i = 0; i < 3; ++i)
    frame.p[i] = transform(i, 3);

  for (int i = 0; i < 9; ++i)
    frame.M.data[i] = transform(i / 3, i % 3);
}

/**
 * @brief Convert KDL::Jacobian to Eigen::Matrix
 * @param jacobian Input KDL Jacobian
 * @param matrix Output Eigen MatrixXd
 */
inline void KDLToEigen(const KDL::Jacobian& jacobian, Eigen::Ref<Eigen::MatrixXd> matrix)
{
  assert(matrix.rows() == jacobian.rows());
  assert(matrix.cols() == jacobian.columns());

  for (unsigned i = 0; i < jacobian.rows(); ++i)
    for (unsigned j = 0; j < jacobian.columns(); ++j)
      matrix(i, j) = jacobian(i, j);
}

/**
 * @brief Convert a subset of KDL::Jacobian to Eigen::Matrix
 * @param jacobian Input KDL Jacobian
 * @param q_nrs Input the columns to use
 * @param matrix Output Eigen MatrixXd
 */
inline void KDLToEigen(const KDL::Jacobian& jacobian, const std::vector<int>& q_nrs, Eigen::Ref<Eigen::MatrixXd> matrix)
{
  assert(matrix.rows() == jacobian.rows());
  assert(static_cast<unsigned>(matrix.cols()) == q_nrs.size());

  for (int i = 0; i < static_cast<int>(jacobian.rows()); ++i)
    for (int j = 0; j < static_cast<int>(q_nrs.size()); ++j)
      matrix(i, j) = jacobian(static_cast<unsigned>(i), static_cast<unsigned>(q_nrs[static_cast<size_t>(j)]));
}

/**
 * @brief Convert Eigen::Vector to KDL::JntArray
 * @param vec Input Eigen vector
 * @param joints Output KDL joint array
 */
inline void EigenToKDL(const Eigen::Ref<const Eigen::VectorXd>& vec, KDL::JntArray& joints) { joints.data = vec; }

inline void getActiveLinkNamesRecursive(std::vector<std::string>& active_links,
                                        const urdf::LinkConstSharedPtr urdf_link,
                                        bool active)
{
  // recursively get all active links
  if (active)
  {
    active_links.push_back(urdf_link->name);
    for (std::size_t i = 0; i < urdf_link->child_links.size(); ++i)
    {
      getActiveLinkNamesRecursive(active_links, urdf_link->child_links[i], active);
    }
  }
  else
  {
    for (std::size_t i = 0; i < urdf_link->child_links.size(); ++i)
    {
      if ((urdf_link->parent_joint) && (urdf_link->parent_joint->type != urdf::Joint::FIXED))
        getActiveLinkNamesRecursive(active_links, urdf_link->child_links[i], true);
      else
        getActiveLinkNamesRecursive(active_links, urdf_link->child_links[i], active);
    }
  }
}

inline tesseract_collision::CollisionShapePtr constructShape(const urdf::Geometry* geom)
{
  tesseract_collision::CollisionShape* result = nullptr;
  switch (geom->type)
  {
    case urdf::Geometry::SPHERE:
      result = new tesseract_collision::SphereCollisionShape(static_cast<const urdf::Sphere*>(geom)->radius);
      break;
    case urdf::Geometry::BOX:
    {
      urdf::Vector3 dim = static_cast<const urdf::Box*>(geom)->dim;
      result = new tesseract_collision::BoxCollisionShape(dim.x, dim.y, dim.z);
    }
    break;
    case urdf::Geometry::CYLINDER:
      result = new tesseract_collision::CylinderCollisionShape(static_cast<const urdf::Cylinder*>(geom)->radius,
                                                               static_cast<const urdf::Cylinder*>(geom)->length);
      break;
    case urdf::Geometry::MESH:
    {
      const urdf::Mesh* mesh = static_cast<const urdf::Mesh*>(geom);
      if (!mesh->filename.empty())
      {
        Eigen::Vector3d scale(mesh->scale.x, mesh->scale.y, mesh->scale.z);
        shapes::Mesh* m = shapes::createMeshFromResource(mesh->filename, scale);

        tesseract_collision::VectorVector3d mesh_vertices;
        mesh_vertices.reserve(m->vertex_count);
        for (unsigned int i = 0; i < m->vertex_count; ++i)
          mesh_vertices.push_back(Eigen::Vector3d(m->vertices[3 * i + 0], m->vertices[3 * i + 1], m->vertices[3 * i + 2]));

        std::shared_ptr<tesseract_collision::VectorVector3d> ch_vertices(new tesseract_collision::VectorVector3d());
        std::shared_ptr<std::vector<int>> ch_faces(new std::vector<int>());
        int num_faces = tesseract_collision::createConvexHull(*ch_vertices, *ch_faces, mesh_vertices);

        result = new tesseract_collision::ConvexMeshCollisionShape(ch_vertices, ch_faces, num_faces);
      }
    }
    break;
    default:
      CONSOLE_BRIDGE_logError("Unknown geometry type: %d", static_cast<int>(geom->type));
      break;
  }

  return tesseract_collision::CollisionShapePtr(result);
}

inline Eigen::Isometry3d urdfPose2Eigen(const urdf::Pose& pose)
{
  Eigen::Quaterniond q(pose.rotation.w, pose.rotation.x, pose.rotation.y, pose.rotation.z);
  Eigen::Isometry3d result;
  result.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  result.linear() = q.toRotationMatrix();
  return result;
}
}
#endif  // TESSERACT_ENVIRONMENT_KDL_UTILS_H
