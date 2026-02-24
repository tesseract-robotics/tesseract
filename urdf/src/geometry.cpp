/**
 * @file geometry.cpp
 * @brief Parse geometry from XML string
 *
 * @author Levi Armstrong
 * @date September 1, 2019
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

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <stdexcept>

#include <Eigen/Geometry>
#include <tinyxml2.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/common.h>
#include <tesseract/collision/bullet/convex_hull_utils.h>
#include <tesseract/geometry/geometries.h>
#include <tesseract/common/resource_locator.h>
#include <tesseract/urdf/box.h>
#include <tesseract/urdf/cylinder.h>
#include <tesseract/urdf/cone.h>
#include <tesseract/urdf/capsule.h>
#include <tesseract/urdf/geometry.h>
#include <tesseract/urdf/mesh.h>
#include <tesseract/urdf/octomap.h>
#include <tesseract/urdf/sdf_mesh.h>
#include <tesseract/urdf/sphere.h>

namespace tesseract::urdf
{
tesseract::geometry::Geometry::Ptr parseGeometry(const tinyxml2::XMLElement* xml_element,
                                                 const tesseract::common::ResourceLocator& locator,
                                                 bool visual,
                                                 bool make_convex_meshes)
{
  const tinyxml2::XMLElement* geometry = xml_element->FirstChildElement();
  if (geometry == nullptr)
    std::throw_with_nested(std::runtime_error("Geometry: Error missing 'geometry' element!"));

  std::string geometry_type;
  int status = tesseract::common::QueryStringValue(geometry, geometry_type);
  if (status != tinyxml2::XML_SUCCESS)
    std::throw_with_nested(std::runtime_error("Geometry: Error parsing 'geometry' element, invalid geometry type!"));

  // URDF-supported elements
  if (geometry_type == SPHERE_ELEMENT_NAME)
  {
    tesseract::geometry::Sphere::Ptr sphere;
    try
    {
      sphere = parseSphere(geometry);
    }
    catch (...)
    {
      std::throw_with_nested(std::runtime_error("Geometry: Failed parsing geometry type 'sphere'!"));
    }

    return sphere;
  }

  if (geometry_type == BOX_ELEMENT_NAME)
  {
    tesseract::geometry::Box::Ptr box;
    try
    {
      box = parseBox(geometry);
    }
    catch (...)
    {
      std::throw_with_nested(std::runtime_error("Geometry: Failed parsing geometry type 'box'!"));
    }

    return box;
  }

  if (geometry_type == CYLINDER_ELEMENT_NAME)
  {
    tesseract::geometry::Cylinder::Ptr cylinder;
    try
    {
      cylinder = parseCylinder(geometry);
    }
    catch (...)
    {
      std::throw_with_nested(std::runtime_error("Geometry: Failed parsing geometry type 'cylinder'!"));
    }

    return cylinder;
  }

  if (geometry_type == MESH_ELEMENT_NAME)
  {
    std::vector<tesseract::geometry::PolygonMesh::Ptr> meshes;
    try
    {
      meshes = parseMesh(geometry, locator, visual, make_convex_meshes);
    }
    catch (...)
    {
      std::throw_with_nested(std::runtime_error("Geometry: Failed parsing geometry type 'mesh'!"));
    }

    if (meshes.size() > 1)
      return std::make_shared<tesseract::geometry::CompoundMesh>(meshes);

    return meshes.front();
  }

  // Custom Tesseract elements
  if (geometry_type == CONE_ELEMENT_NAME)
  {
    tesseract::geometry::Cone::Ptr cone;
    try
    {
      cone = parseCone(geometry);
    }
    catch (...)
    {
      std::throw_with_nested(std::runtime_error("Geometry: Failed parsing geometry type 'cone'!"));
    }

    return cone;
  }

  if (geometry_type == CAPSULE_ELEMENT_NAME)
  {
    tesseract::geometry::Capsule::Ptr capsule;
    try
    {
      capsule = parseCapsule(geometry);
    }
    catch (...)
    {
      std::throw_with_nested(std::runtime_error("Geometry: Failed parsing geometry type 'capsule'!"));
    }

    return capsule;
  }

  if (geometry_type == OCTOMAP_ELEMENT_NAME)
  {
    tesseract::geometry::Octree::Ptr octree;
    try
    {
      octree = parseOctomap(geometry, locator, visual);
    }
    catch (...)
    {
      std::throw_with_nested(std::runtime_error("Geometry: Failed parsing geometry type 'octomap'!"));
    }

    return octree;
  }

  if (geometry_type == SDF_MESH_ELEMENT_NAME)
  {
    std::vector<tesseract::geometry::SDFMesh::Ptr> meshes;
    try
    {
      meshes = parseSDFMesh(geometry, locator, visual);
    }
    catch (...)
    {
      std::throw_with_nested(std::runtime_error("Geometry: Failed parsing geometry type 'sdf_mesh'!"));
    }

    if (meshes.size() > 1)
      return std::make_shared<tesseract::geometry::CompoundMesh>(meshes);

    return meshes.front();
  }

  std::throw_with_nested(std::runtime_error("Geometry: Invalid geometry type '" + geometry_type + "'!"));
}

tinyxml2::XMLElement* writeGeometry(const std::shared_ptr<const tesseract::geometry::Geometry>& geometry,
                                    tinyxml2::XMLDocument& doc,
                                    const std::string& package_path,
                                    const std::string& filename)
{
  if (geometry == nullptr)
    std::throw_with_nested(std::runtime_error("Geometry is nullptr and cannot be converted to XML"));
  tinyxml2::XMLElement* xml_element = doc.NewElement(GEOMETRY_ELEMENT_NAME.data());

  tesseract::geometry::GeometryType type = geometry->getType();

  if (type == tesseract::geometry::GeometryType::SPHERE)
  {
    try
    {
      tinyxml2::XMLElement* xml_sphere =
          writeSphere(std::static_pointer_cast<const tesseract::geometry::Sphere>(geometry), doc);
      xml_element->InsertEndChild(xml_sphere);
    }
    catch (...)
    {
      std::throw_with_nested(std::runtime_error("Could not write geometry marked as sphere!"));
    }
  }
  else if (type == tesseract::geometry::GeometryType::CYLINDER)
  {
    try
    {
      tinyxml2::XMLElement* xml_cylinder =
          writeCylinder(std::static_pointer_cast<const tesseract::geometry::Cylinder>(geometry), doc);
      xml_element->InsertEndChild(xml_cylinder);
    }
    catch (...)
    {
      std::throw_with_nested(std::runtime_error("Could not write geometry marked as cylinder!"));
    }
  }
  else if (type == tesseract::geometry::GeometryType::CAPSULE)
  {
    try
    {
      tinyxml2::XMLElement* xml_capsule =
          writeCapsule(std::static_pointer_cast<const tesseract::geometry::Capsule>(geometry), doc);
      xml_element->InsertEndChild(xml_capsule);
    }
    catch (...)
    {
      std::throw_with_nested(std::runtime_error("Could not write geometry marked as capsule!"));
    }
  }
  else if (type == tesseract::geometry::GeometryType::CONE)
  {
    try
    {
      tinyxml2::XMLElement* xml_cone =
          writeCone(std::static_pointer_cast<const tesseract::geometry::Cone>(geometry), doc);
      xml_element->InsertEndChild(xml_cone);
    }
    catch (...)
    {
      std::throw_with_nested(std::runtime_error("Could not write geometry marked as cone!"));
    }
  }
  else if (type == tesseract::geometry::GeometryType::BOX)
  {
    try
    {
      tinyxml2::XMLElement* xml_box = writeBox(std::static_pointer_cast<const tesseract::geometry::Box>(geometry), doc);
      xml_element->InsertEndChild(xml_box);
    }
    catch (...)
    {
      std::throw_with_nested(std::runtime_error("Could not write geometry marked as box!"));
    }
  }
  else if (type == tesseract::geometry::GeometryType::PLANE)
  {
    std::throw_with_nested(std::runtime_error("Cannot write geometry of type PLANE to XML!  Consider using box."));
  }
  else if (type == tesseract::geometry::GeometryType::MESH)
  {
    try
    {
      tinyxml2::XMLElement* xml_mesh =
          writeMesh(std::static_pointer_cast<const tesseract::geometry::PolygonMesh>(geometry),
                    doc,
                    package_path,
                    filename + ".ply");
      xml_element->InsertEndChild(xml_mesh);
    }
    catch (...)
    {
      std::throw_with_nested(std::runtime_error("Could not write geometry marked as mesh!"));
    }
  }
  else if (type == tesseract::geometry::GeometryType::SDF_MESH)
  {
    try
    {
      tinyxml2::XMLElement* xml_sdf_mesh = writeSDFMesh(
          std::static_pointer_cast<const tesseract::geometry::SDFMesh>(geometry), doc, package_path, filename + ".ply");
      xml_element->InsertEndChild(xml_sdf_mesh);
    }
    catch (...)
    {
      std::throw_with_nested(std::runtime_error("Could not write geometry marked as SDF mesh!"));
    }
  }
  else if (type == tesseract::geometry::GeometryType::OCTREE)
  {
    try
    {
      tinyxml2::XMLElement* xml_octree = writeOctomap(
          std::static_pointer_cast<const tesseract::geometry::Octree>(geometry), doc, package_path, filename + ".bt");
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

}  // namespace tesseract::urdf
