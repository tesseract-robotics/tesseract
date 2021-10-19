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
#include <tesseract_collision/bullet/convex_hull_utils.h>
#include <tesseract_geometry/geometries.h>
#include <tesseract_scene_graph/utils.h>
#include <tesseract_common/resource_locator.h>
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
                              const tesseract_common::ResourceLocator& locator,
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

tinyxml2::XMLElement* tesseract_urdf::writeGeometry(const std::shared_ptr<const tesseract_geometry::Geometry>& geometry,
                                                    tinyxml2::XMLDocument& doc,
                                                    const std::string& directory,
                                                    const std::string& filename)
{
  if (geometry == nullptr)
    std::throw_with_nested(std::runtime_error("Geometry is nullptr and cannot be converted to XML"));
  tinyxml2::XMLElement* xml_element = doc.NewElement("geometry");

  tesseract_geometry::GeometryType type = geometry->getType();

  if (type == tesseract_geometry::GeometryType::SPHERE)
  {
    try
    {
      tinyxml2::XMLElement* xml_sphere =
          writeSphere(std::static_pointer_cast<const tesseract_geometry::Sphere>(geometry), doc);
      xml_element->InsertEndChild(xml_sphere);
    }
    catch (...)
    {
      std::throw_with_nested(std::runtime_error("Could not write geometry marked as sphere!"));
    }
  }
  else if (type == tesseract_geometry::GeometryType::CYLINDER)
  {
    try
    {
      tinyxml2::XMLElement* xml_cylinder =
          writeCylinder(std::static_pointer_cast<const tesseract_geometry::Cylinder>(geometry), doc);
      xml_element->InsertEndChild(xml_cylinder);
    }
    catch (...)
    {
      std::throw_with_nested(std::runtime_error("Could not write geometry marked as cylinder!"));
    }
  }
  else if (type == tesseract_geometry::GeometryType::CAPSULE)
  {
    try
    {
      tinyxml2::XMLElement* xml_capsule =
          writeCapsule(std::static_pointer_cast<const tesseract_geometry::Capsule>(geometry), doc);
      xml_element->InsertEndChild(xml_capsule);
    }
    catch (...)
    {
      std::throw_with_nested(std::runtime_error("Could not write geometry marked as capsule!"));
    }
  }
  else if (type == tesseract_geometry::GeometryType::CONE)
  {
    try
    {
      tinyxml2::XMLElement* xml_cone =
          writeCone(std::static_pointer_cast<const tesseract_geometry::Cone>(geometry), doc);
      xml_element->InsertEndChild(xml_cone);
    }
    catch (...)
    {
      std::throw_with_nested(std::runtime_error("Could not write geometry marked as cone!"));
    }
  }
  else if (type == tesseract_geometry::GeometryType::BOX)
  {
    try
    {
      tinyxml2::XMLElement* xml_box = writeBox(std::static_pointer_cast<const tesseract_geometry::Box>(geometry), doc);
      xml_element->InsertEndChild(xml_box);
    }
    catch (...)
    {
      std::throw_with_nested(std::runtime_error("Could not write geometry marked as box!"));
    }
  }
  else if (type == tesseract_geometry::GeometryType::PLANE)
  {
    std::throw_with_nested(std::runtime_error("Cannot write geometry of type PLANE to XML!  Consider using box."));
  }
  else if (type == tesseract_geometry::GeometryType::MESH)
  {
    try
    {
      tinyxml2::XMLElement* xml_mesh = writeMesh(
          std::static_pointer_cast<const tesseract_geometry::Mesh>(geometry), doc, directory, filename + ".ply");
      xml_element->InsertEndChild(xml_mesh);
    }
    catch (...)
    {
      std::throw_with_nested(std::runtime_error("Could not write geometry marked as mesh!"));
    }
  }
  else if (type == tesseract_geometry::GeometryType::CONVEX_MESH)
  {
    try
    {
      tinyxml2::XMLElement* xml_convex_mesh = writeConvexMesh(
          std::static_pointer_cast<const tesseract_geometry::ConvexMesh>(geometry), doc, directory, filename + ".ply");
      xml_element->InsertEndChild(xml_convex_mesh);
    }
    catch (...)
    {
      std::throw_with_nested(std::runtime_error("Could not write geometry marked as convex mesh!"));
    }
  }
  else if (type == tesseract_geometry::GeometryType::SDF_MESH)
  {
    try
    {
      tinyxml2::XMLElement* xml_sdf_mesh = writeSDFMesh(
          std::static_pointer_cast<const tesseract_geometry::SDFMesh>(geometry), doc, directory, filename + ".ply");
      xml_element->InsertEndChild(xml_sdf_mesh);
    }
    catch (...)
    {
      std::throw_with_nested(std::runtime_error("Could not write geometry marked as SDF mesh!"));
    }
  }
  else if (type == tesseract_geometry::GeometryType::OCTREE)
  {
    try
    {
      tinyxml2::XMLElement* xml_octree = writeOctomap(
          std::static_pointer_cast<const tesseract_geometry::Octree>(geometry), doc, directory, filename + ".bt");
      xml_element->InsertEndChild(xml_octree);
    }
    catch (...)
    {
      std::throw_with_nested(std::runtime_error("Could not write geometry marked as octree!"));
    }
  }
  else
  {
    std::throw_with_nested(std::runtime_error("Unknown geometry type, cannot write to XML!"));
  }

  return xml_element;
}
