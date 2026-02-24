/**
 * @file mesh.cpp
 * @brief Parse mesh from xml string
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

#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <Eigen/Geometry>
#include <tesseract/common/utils.h>
#include <tinyxml2.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/bullet/convex_hull_utils.h>
#include <tesseract/geometry/impl/mesh.h>
#include <tesseract/geometry/mesh_parser.h>
#include <tesseract/urdf/mesh.h>
#include <tesseract/common/resource_locator.h>
#include <tesseract/urdf/utils.h>

namespace tesseract::urdf
{
std::vector<tesseract::geometry::PolygonMesh::Ptr> parseMesh(const tinyxml2::XMLElement* xml_element,
                                                             const tesseract::common::ResourceLocator& locator,
                                                             bool visual,
                                                             bool make_convex)
{
  std::string filename;
  if (tesseract::common::QueryStringAttribute(xml_element, "filename", filename) != tinyxml2::XML_SUCCESS)
    std::throw_with_nested(std::runtime_error("Mesh: Missing or failed parsing attribute 'filename'!"));

  std::string scale_string;
  Eigen::Vector3d scale(1, 1, 1);
  if (tesseract::common::QueryStringAttribute(xml_element, "scale", scale_string) == tinyxml2::XML_SUCCESS)
  {
    std::vector<std::string> tokens;
    boost::split(tokens, scale_string, boost::is_any_of(" "), boost::token_compress_on);
    if (tokens.size() != 3 || !tesseract::common::isNumeric(tokens))
      std::throw_with_nested(std::runtime_error("Mesh: Failed parsing attribute 'scale'!"));

    double sx{ 0 }, sy{ 0 }, sz{ 0 };
    // No need to check return values because the tokens are verified above
    tesseract::common::toNumeric<double>(tokens[0], sx);
    tesseract::common::toNumeric<double>(tokens[1], sy);
    tesseract::common::toNumeric<double>(tokens[2], sz);

    if (!(sx > 0))
      std::throw_with_nested(std::runtime_error("Mesh: Scale x value is not greater than zero!"));

    if (!(sy > 0))
      std::throw_with_nested(std::runtime_error("Mesh: Scale y value is not greater than zero!"));

    if (!(sz > 0))
      std::throw_with_nested(std::runtime_error("Mesh: Scale z value is not greater than zero!"));

    scale = Eigen::Vector3d(sx, sy, sz);
  }

  std::vector<tesseract::geometry::Mesh::Ptr> meshes;

  if (visual)
    meshes = tesseract::geometry::createMeshFromResource<tesseract::geometry::Mesh>(
        locator.locateResource(filename), scale, true, true, true, true, true);
  else
    meshes = tesseract::geometry::createMeshFromResource<tesseract::geometry::Mesh>(
        locator.locateResource(filename), scale, true, false);

  if (meshes.empty())
    std::throw_with_nested(std::runtime_error("Mesh: Error importing meshes from filename: '" + filename + "'!"));

  bool make_convex_override = false;
  auto make_convex_override_status = xml_element->QueryBoolAttribute("tesseract:make_convex", &make_convex_override);
  if (make_convex_override_status != tinyxml2::XML_NO_ATTRIBUTE)
  {
    // Make convex override attribute is specified
    // Check that it was loaded successfully
    if (make_convex_override_status != tinyxml2::XML_SUCCESS)
      std::throw_with_nested(std::runtime_error("Mesh: Failed to parse attribute 'tesseract:make_convex'"));

    // Override the global make_convex flag with the value from the attribute
    make_convex = make_convex_override;
  }

  if (make_convex)
  {
    std::vector<tesseract::geometry::PolygonMesh::Ptr> convex_meshes;
    convex_meshes.reserve(meshes.size());

    for (const auto& mesh : meshes)
    {
      tesseract::geometry::ConvexMesh::Ptr convex_mesh = tesseract::collision::makeConvexMesh(*mesh);
      convex_mesh->setCreationMethod(tesseract::geometry::ConvexMesh::CONVERTED);
      convex_meshes.push_back(convex_mesh);
    }

    return convex_meshes;
  }

  // Convert to base class for output
  std::vector<tesseract::geometry::PolygonMesh::Ptr> output;
  output.reserve(meshes.size());
  std::copy(meshes.begin(), meshes.end(), std::back_inserter(output));

  return output;
}

tinyxml2::XMLElement* writeMesh(const std::shared_ptr<const tesseract::geometry::PolygonMesh>& mesh,
                                tinyxml2::XMLDocument& doc,
                                const std::string& package_path,
                                const std::string& filename)
{
  if (mesh == nullptr)
    std::throw_with_nested(std::runtime_error("Mesh is nullptr and cannot be converted to XML"));
  tinyxml2::XMLElement* xml_element = doc.NewElement(MESH_ELEMENT_NAME.data());
  Eigen::IOFormat eigen_format(Eigen::StreamPrecision, Eigen::DontAlignCols, " ", " ");

  try
  {
    writeMeshToFile(mesh, trailingSlash(package_path) + noLeadingSlash(filename));
  }
  catch (...)
  {
    std::throw_with_nested(std::runtime_error("Failed to write mesh to file: " + package_path + filename));
  }
  xml_element->SetAttribute("filename", makeURDFFilePath(package_path, filename).c_str());

  if (!mesh->getScale().isOnes(std::numeric_limits<double>::epsilon()))
  {
    std::stringstream scale_string;
    scale_string << mesh->getScale().format(eigen_format);
    xml_element->SetAttribute("scale", scale_string.str().c_str());
  }

  // If the mesh is actually a convex mesh, set the `tesseract:make_convex` attribute true.
  // The geometry itself is already convex, so telling Tesseract to make it convex won't change the geometry.
  // However, it will make sure it gets added to the environment as a `ConvexMesh` shape instead of a `Mesh` shape.
  if (std::dynamic_pointer_cast<const tesseract::geometry::ConvexMesh>(mesh))
  {
    xml_element->SetAttribute("tesseract:make_convex", true);
  }

  return xml_element;
}

}  // namespace tesseract::urdf
