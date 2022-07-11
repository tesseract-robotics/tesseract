/**
 * @file octree.cpp
 * @brief Parse octree from xml string
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

#include <boost/iostreams/device/array.hpp>
#include <boost/iostreams/stream.hpp>
#include <tesseract_common/utils.h>
#include <tinyxml2.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/resource_locator.h>
#include <tesseract_urdf/octree.h>
#include <tesseract_urdf/utils.h>

tesseract_geometry::Octree::Ptr tesseract_urdf::parseOctree(const tinyxml2::XMLElement* xml_element,
                                                            const tesseract_common::ResourceLocator& locator,
                                                            tesseract_geometry::Octree::SubType shape_type,
                                                            bool prune,
                                                            int /*version*/)
{
  std::string filename;
  if (tesseract_common::QueryStringAttribute(xml_element, "filename", filename) != tinyxml2::XML_SUCCESS)
    std::throw_with_nested(std::runtime_error("Octree: Missing or failed parsing attribute 'filename'!"));

  tesseract_common::Resource::Ptr resource = locator.locateResource(filename);
  if (!resource || !resource->isFile())
    std::throw_with_nested(std::runtime_error("Octree: Missing resource '" + filename + "'!"));

  auto ot = std::make_shared<octomap::OcTree>(resource->getFilePath());

  if (ot == nullptr || ot->size() == 0)
    std::throw_with_nested(std::runtime_error("Octree: Error importing from '" + filename + "'!"));

  if (prune)
    tesseract_geometry::Octree::prune(*ot);

  auto geom = std::make_shared<tesseract_geometry::Octree>(ot, shape_type);
  if (geom == nullptr)
    std::throw_with_nested(std::runtime_error("Octree: Error creating octree geometry type from octomap::octree!"));

  return geom;
}

tinyxml2::XMLElement* tesseract_urdf::writeOctree(const tesseract_geometry::Octree::ConstPtr& octree,
                                                  tinyxml2::XMLDocument& doc,
                                                  const std::string& package_path,
                                                  const std::string& filename)
{
  if (octree == nullptr)
    std::throw_with_nested(std::runtime_error("Octree is nullptr and cannot be converted to XML"));
  tinyxml2::XMLElement* xml_element = doc.NewElement("octree");

  std::string filepath = trailingSlash(package_path) + noLeadingSlash(filename);

  // This copy is unfortunate, but avoiding the copy requires flowing mutability up to a lot of
  // functions and their arguments. Don't know why writeBinary is non-const anyway, but we'll live.
  std::shared_ptr<octomap::OcTree> underlying_tree = std::make_shared<octomap::OcTree>(*(octree->getOctree()));

  if (!underlying_tree->writeBinary(filepath))
    std::throw_with_nested(std::runtime_error("Could not write octree to file `" + filepath + "`!"));

  xml_element->SetAttribute("filename", makeURDFFilePath(package_path, filename).c_str());

  return xml_element;
}
