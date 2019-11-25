/**
 * @file geometry.h
 * @brief Parse geometry from xml string
 *
 * @author Levi Armstrong
 * @date September 1, 2019
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2019, Southwest Research Institute
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
#ifndef TESSERACT_URDF_GEOMETRY_H
#define TESSERACT_URDF_GEOMETRY_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <tesseract_common/status_code.h>
#include <Eigen/Geometry>
#include <tinyxml2.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_collision/core/common.h>
#include <tesseract_geometry/geometries.h>
#include <tesseract_scene_graph/utils.h>
#include <tesseract_scene_graph/resource_locator.h>
#include <tesseract_urdf/sphere.h>
#include <tesseract_urdf/box.h>
#include <tesseract_urdf/cylinder.h>
#include <tesseract_urdf/cone.h>
#include <tesseract_urdf/capsule.h>
#include <tesseract_urdf/mesh.h>
#include <tesseract_urdf/convex_mesh.h>
#include <tesseract_urdf/sdf_mesh.h>
#include <tesseract_urdf/octomap.h>

namespace tesseract_urdf
{
class GeometryStatusCategory : public tesseract_common::StatusCategory
{
public:
  GeometryStatusCategory() : name_("GeometryStatusCategory") {}
  const std::string& name() const noexcept override { return name_; }
  std::string message(int code) const override
  {
    switch (code)
    {
      case Success:
        return "Sucessfully parsed geomety!";
      case ErrorParsingSphere:
        return "Failed parsing geometry type sphere!";
      case ErrorParsingBox:
        return "Failed parsing geometry type box!";
      case ErrorParsingCylinder:
        return "Failed parsing geometry type cylinder!";
      case ErrorParsingCone:
        return "Failed parsing geometry type cone!";
      case ErrorParsingCapsule:
        return "Failed parsing geometry type capsule!";
      case ErrorParsingOctomap:
        return "Failed parsing geometry type octomap!";
      case ErrorParsingMesh:
        return "Failed parsing geometry type mesh!";
      case ErrorParsingConvexMesh:
        return "Failed parsing geometry type convex_mesh!";
      case ErrorParsingSDFMesh:
        return "Failed parsing geometry type sdf_mesh!";
      case ErrorInvalidGeometryType:
        return "Error parsing geometry, invalid geometry type!";
      default:
        return "Invalid error code for " + name_ + "!";
    }
  }

  enum
  {
    Success = 0,
    ErrorParsingGeoemtryElement = -1,
    ErrorParsingSphere = -2,
    ErrorParsingBox = -3,
    ErrorParsingCylinder = -4,
    ErrorParsingCone = -5,
    ErrorParsingCapsule = -6,
    ErrorParsingOctomap = -7,
    ErrorParsingMesh = -8,
    ErrorParsingConvexMesh = -9,
    ErrorParsingSDFMesh = -10,
    ErrorInvalidGeometryType = -11
  };

private:
  std::string name_;
};

inline tesseract_common::StatusCode::Ptr parse(std::vector<tesseract_geometry::Geometry::Ptr>& geometries,
                                               const tinyxml2::XMLElement* xml_element,
                                               const tesseract_scene_graph::ResourceLocator::Ptr& locator,
                                               const bool visual,
                                               const int version)
{
  geometries.clear();
  auto status_cat = std::make_shared<GeometryStatusCategory>();

  const tinyxml2::XMLElement* geometry = xml_element->FirstChildElement();
  if (geometry == nullptr)
    return std::make_shared<tesseract_common::StatusCode>(GeometryStatusCategory::ErrorParsingGeoemtryElement,
                                                          status_cat);

  std::string geometry_type;
  tinyxml2::XMLError status = QueryStringValue(geometry, geometry_type);
  if (status != tinyxml2::XML_SUCCESS)
    return std::make_shared<tesseract_common::StatusCode>(GeometryStatusCategory::ErrorInvalidGeometryType, status_cat);

  if (geometry_type == "sphere")
  {
    tesseract_geometry::Sphere::Ptr sphere;
    tesseract_common::StatusCode::Ptr status = parse(sphere, geometry, version);
    if (!(*status))
      return std::make_shared<tesseract_common::StatusCode>(
          GeometryStatusCategory::ErrorParsingSphere, status_cat, status);

    geometries = { sphere };
  }
  else if (geometry_type == "box")
  {
    tesseract_geometry::Box::Ptr box;
    tesseract_common::StatusCode::Ptr status = parse(box, geometry, version);
    if (!(*status))
      return std::make_shared<tesseract_common::StatusCode>(
          GeometryStatusCategory::ErrorParsingBox, status_cat, status);

    geometries = { box };
  }
  else if (geometry_type == "cylinder")
  {
    tesseract_geometry::Cylinder::Ptr cylinder;
    tesseract_common::StatusCode::Ptr status = parse(cylinder, geometry, version);
    if (!(*status))
      return std::make_shared<tesseract_common::StatusCode>(
          GeometryStatusCategory::ErrorParsingCylinder, status_cat, status);

    geometries = { cylinder };
  }
  else if (geometry_type == "cone")
  {
    tesseract_geometry::Cone::Ptr cone;
    tesseract_common::StatusCode::Ptr status = parse(cone, geometry, version);
    if (!(*status))
      return std::make_shared<tesseract_common::StatusCode>(
          GeometryStatusCategory::ErrorParsingCone, status_cat, status);

    geometries = { cone };
  }
  else if (geometry_type == "capsule")
  {
    tesseract_geometry::Capsule::Ptr capsule;
    tesseract_common::StatusCode::Ptr status = parse(capsule, geometry, version);
    if (!(*status))
      return std::make_shared<tesseract_common::StatusCode>(
          GeometryStatusCategory::ErrorParsingCapsule, status_cat, status);

    geometries = { capsule };
  }
  else if (geometry_type == "octomap")
  {
    tesseract_geometry::Octree::Ptr octree;
    tesseract_common::StatusCode::Ptr status = parse(octree, geometry, locator, visual, version);
    if (!(*status))
      return std::make_shared<tesseract_common::StatusCode>(
          GeometryStatusCategory::ErrorParsingOctomap, status_cat, status);

    geometries = { octree };
  }
  else if (geometry_type == "mesh")
  {
    std::vector<tesseract_geometry::Mesh::Ptr> meshes;
    tesseract_common::StatusCode::Ptr status = parse(meshes, geometry, locator, visual, version);
    if (!(*status))
      return std::make_shared<tesseract_common::StatusCode>(
          GeometryStatusCategory::ErrorParsingMesh, status_cat, status);

    if (version < 2 && !visual)
    {
      for (const auto& mesh : meshes)
      {
        auto ch_vertices = std::make_shared<tesseract_common::VectorVector3d>();
        auto ch_faces = std::make_shared<Eigen::VectorXi>();
        int ch_num_faces = tesseract_collision::createConvexHull(*ch_vertices, *ch_faces, *(mesh->getVertices()));
        geometries.push_back(std::make_shared<tesseract_geometry::ConvexMesh>(ch_vertices, ch_faces, ch_num_faces));
      }
    }
    else
    {
      geometries = std::vector<tesseract_geometry::Geometry::Ptr>(meshes.begin(), meshes.end());
    }
  }
  else if (geometry_type == "convex_mesh")
  {
    std::vector<tesseract_geometry::ConvexMesh::Ptr> meshes;
    tesseract_common::StatusCode::Ptr status = parse(meshes, geometry, locator, visual, version);
    if (!(*status))
      return std::make_shared<tesseract_common::StatusCode>(
          GeometryStatusCategory::ErrorParsingConvexMesh, status_cat, status);

    geometries = std::vector<tesseract_geometry::Geometry::Ptr>(meshes.begin(), meshes.end());
  }
  else if (geometry_type == "sdf_mesh")
  {
    std::vector<tesseract_geometry::SDFMesh::Ptr> meshes;
    tesseract_common::StatusCode::Ptr status = parse(meshes, geometry, locator, visual, version);
    if (!(*status))
      return std::make_shared<tesseract_common::StatusCode>(
          GeometryStatusCategory::ErrorParsingSDFMesh, status_cat, status);

    geometries = std::vector<tesseract_geometry::Geometry::Ptr>(meshes.begin(), meshes.end());
  }
  else
  {
    return std::make_shared<tesseract_common::StatusCode>(GeometryStatusCategory::ErrorInvalidGeometryType, status_cat);
  }

  return std::make_shared<tesseract_common::StatusCode>(GeometryStatusCategory::Success, status_cat);
}

}  // namespace tesseract_urdf

#endif  // TESSERACT_URDF_GEOMETRY_H
