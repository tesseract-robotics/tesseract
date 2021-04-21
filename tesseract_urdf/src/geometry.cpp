/**
 * @file geometry.cpp
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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <stdexcept>
#include <Eigen/Geometry>
#include <tinyxml2.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_collision/core/common.h>
#include <tesseract_geometry/geometries.h>
#include <tesseract_scene_graph/utils.h>
#include <tesseract_scene_graph/resource_locator.h>
#include <tesseract_urdf/geometry.h>
#include <tesseract_urdf/sphere.h>
#include <tesseract_urdf/box.h>
#include <tesseract_urdf/cylinder.h>
#include <tesseract_urdf/cone.h>
#include <tesseract_urdf/capsule.h>
#include <tesseract_urdf/mesh.h>
#include <tesseract_urdf/convex_mesh.h>
#include <tesseract_urdf/sdf_mesh.h>
#include <tesseract_urdf/octomap.h>

std::vector<tesseract_geometry::Geometry::Ptr>
tesseract_urdf::parseGeometry(const tinyxml2::XMLElement* xml_element,
                              const tesseract_scene_graph::ResourceLocator::Ptr& locator,
                              bool visual,
                              int version)
{
  std::vector<tesseract_geometry::Geometry::Ptr> geometries;

  const tinyxml2::XMLElement* geometry = xml_element->FirstChildElement();
  if (geometry == nullptr)
    std::throw_with_nested(std::runtime_error("Geometry: Error missing 'geometry' elemment!"));

  std::string geometry_type;
  tinyxml2::XMLError status = tesseract_common::QueryStringValue(geometry, geometry_type);
  if (status != tinyxml2::XML_SUCCESS)
    std::throw_with_nested(std::runtime_error("Geometry: Error parsing 'geometry' element, invalid geometry type!"));

  if (geometry_type == "sphere")
  {
    tesseract_geometry::Sphere::Ptr sphere;
    try
    {
      sphere = parseSphere(geometry, version);
    }
    catch (...)
    {
      std::throw_with_nested(std::runtime_error("Geometry: Failed parsing geometry type 'sphere'!"));
    }

    geometries = { sphere };
  }
  else if (geometry_type == "box")
  {
    tesseract_geometry::Box::Ptr box;
    try
    {
      box = parseBox(geometry, version);
    }
    catch (...)
    {
      std::throw_with_nested(std::runtime_error("Geometry: Failed parsing geometry type 'box'!"));
    }

    geometries = { box };
  }
  else if (geometry_type == "cylinder")
  {
    tesseract_geometry::Cylinder::Ptr cylinder;
    try
    {
      cylinder = parseCylinder(geometry, version);
    }
    catch (...)
    {
      std::throw_with_nested(std::runtime_error("Geometry: Failed parsing geometry type 'cylinder'!"));
    }

    geometries = { cylinder };
  }
  else if (geometry_type == "cone")
  {
    tesseract_geometry::Cone::Ptr cone;
    try
    {
      cone = parseCone(geometry, version);
    }
    catch (...)
    {
      std::throw_with_nested(std::runtime_error("Geometry: Failed parsing geometry type 'cone'!"));
    }

    geometries = { cone };
  }
  else if (geometry_type == "capsule")
  {
    tesseract_geometry::Capsule::Ptr capsule;
    try
    {
      capsule = parseCapsule(geometry, version);
    }
    catch (...)
    {
      std::throw_with_nested(std::runtime_error("Geometry: Failed parsing geometry type 'capsule'!"));
    }

    geometries = { capsule };
  }
  else if (geometry_type == "octomap")
  {
    tesseract_geometry::Octree::Ptr octree;
    try
    {
      octree = parseOctomap(geometry, locator, visual, version);
    }
    catch (...)
    {
      std::throw_with_nested(std::runtime_error("Geometry: Failed parsing geometry type 'octomap'!"));
    }

    geometries = { octree };
  }
  else if (geometry_type == "mesh")
  {
    std::vector<tesseract_geometry::Mesh::Ptr> meshes;
    try
    {
      meshes = parseMesh(geometry, locator, visual, version);
    }
    catch (...)
    {
      std::throw_with_nested(std::runtime_error("Geometry: Failed parsing geometry type 'mesh'!"));
    }

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
    try
    {
      meshes = parseConvexMesh(geometry, locator, visual, version);
    }
    catch (...)
    {
      std::throw_with_nested(std::runtime_error("Geometry: Failed parsing geometry type 'convex_mesh'!"));
    }

    geometries = std::vector<tesseract_geometry::Geometry::Ptr>(meshes.begin(), meshes.end());
  }
  else if (geometry_type == "sdf_mesh")
  {
    std::vector<tesseract_geometry::SDFMesh::Ptr> meshes;
    try
    {
      meshes = parseSDFMesh(geometry, locator, visual, version);
    }
    catch (...)
    {
      std::throw_with_nested(std::runtime_error("Geometry: Failed parsing geometry type 'sdf_mesh'!"));
    }

    geometries = std::vector<tesseract_geometry::Geometry::Ptr>(meshes.begin(), meshes.end());
  }
  else
  {
    std::throw_with_nested(std::runtime_error("Geometry: Invalid geometry type '" + geometry_type + "'!"));
  }

  return geometries;
}
