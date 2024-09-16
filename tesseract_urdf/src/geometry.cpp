/**
 * @file geometry.cpp
 * @brief Parse geometry from XML string
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
#include <tesseract_common/resource_locator.h>
#include <tesseract_urdf/box.h>
#include <tesseract_urdf/cylinder.h>
#include <tesseract_urdf/cone.h>
#include <tesseract_urdf/convex_mesh.h>
#include <tesseract_urdf/capsule.h>
#include <tesseract_urdf/geometry.h>
#include <tesseract_urdf/mesh.h>
#include <tesseract_urdf/octomap.h>
#include <tesseract_urdf/sdf_mesh.h>
#include <tesseract_urdf/sphere.h>

tesseract_geometry::Geometry::Ptr tesseract_urdf::parseGeometry(const tinyxml2::XMLElement* xml_element,
                                                                const tesseract_common::ResourceLocator& locator,
                                                                bool visual,
                                                                int version)
{
  const tinyxml2::XMLElement* geometry = xml_element->FirstChildElement();
  if (geometry == nullptr)
    std::throw_with_nested(std::runtime_error("Geometry: Error missing 'geometry' element!"));

  std::string geometry_type;
  int status = tesseract_common::QueryStringValue(geometry, geometry_type);
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

    return sphere;
  }

  if (geometry_type == "box")
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

    return box;
  }

  if (geometry_type == "cylinder")
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

    return cylinder;
  }

  if (geometry_type == "cone")
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

    return cone;
  }

  if (geometry_type == "capsule")
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

    return capsule;
  }

  if (geometry_type == "octomap")
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

    return octree;
  }

  if (geometry_type == "mesh")
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

    if (meshes.size() > 1)
    {
      std::vector<std::shared_ptr<tesseract_geometry::PolygonMesh>> poly_meshes;
      poly_meshes.reserve(meshes.size());

      if (version < 2 && !visual)
      {
        for (const auto& mesh : meshes)
          poly_meshes.push_back(tesseract_collision::makeConvexMesh(*mesh));
      }
      else
      {
        for (const auto& mesh : meshes)
          poly_meshes.push_back(mesh);
      }
      return std::make_shared<tesseract_geometry::CompoundMesh>(poly_meshes);
    }

    if (version < 2 && !visual)
      return tesseract_collision::makeConvexMesh(*meshes.front());

    return meshes.front();
  }

  if (geometry_type == "convex_mesh")
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

    if (meshes.size() > 1)
      return std::make_shared<tesseract_geometry::CompoundMesh>(meshes);

    return meshes.front();
  }

  if (geometry_type == "sdf_mesh")
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

    if (meshes.size() > 1)
      return std::make_shared<tesseract_geometry::CompoundMesh>(meshes);

    return meshes.front();
  }

  std::throw_with_nested(std::runtime_error("Geometry: Invalid geometry type '" + geometry_type + "'!"));
}

tinyxml2::XMLElement* tesseract_urdf::writeGeometry(const std::shared_ptr<const tesseract_geometry::Geometry>& geometry,
                                                    tinyxml2::XMLDocument& doc,
                                                    const std::string& package_path,
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
          std::static_pointer_cast<const tesseract_geometry::Mesh>(geometry), doc, package_path, filename + ".ply");
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
      tinyxml2::XMLElement* xml_convex_mesh =
          writeConvexMesh(std::static_pointer_cast<const tesseract_geometry::ConvexMesh>(geometry),
                          doc,
                          package_path,
                          filename + ".ply");
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
          std::static_pointer_cast<const tesseract_geometry::SDFMesh>(geometry), doc, package_path, filename + ".ply");
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
          std::static_pointer_cast<const tesseract_geometry::Octree>(geometry), doc, package_path, filename + ".bt");
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
