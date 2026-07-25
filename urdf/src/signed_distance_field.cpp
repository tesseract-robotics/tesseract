/**
 * @file signed_distance_field.cpp
 * @brief Parse signed distance field from xml string
 *
 * @author Joel Kang
 * @date June 26, 2026
 *
 * @copyright Copyright (c) 2026, Tesseract
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
#include <algorithm>
#include <cctype>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <vector>

#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <Eigen/Geometry>
#include <tesseract/common/utils.h>
#include <tinyxml2.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/geometry/impl/signed_distance_field.h>
#include <tesseract/geometry/impl/signed_distance_field_utils.h>
#include <tesseract/common/resource_locator.h>
#include <tesseract/urdf/signed_distance_field.h>
#include <tesseract/urdf/utils.h>

namespace tesseract::urdf
{
namespace
{
bool hasExtension(const std::string& filename, const std::string& extension)
{
  std::string actual = std::filesystem::path(filename).extension().string();
  std::transform(actual.begin(), actual.end(), actual.begin(), [](unsigned char value) {
    return static_cast<char>(std::tolower(value));
  });
  return actual == extension;
}
}  // namespace

tesseract::geometry::SignedDistanceField::Ptr
parseSignedDistanceField(const tinyxml2::XMLElement* xml_element, const tesseract::common::ResourceLocator& locator)
{
  std::string filename;
  if (tesseract::common::QueryStringAttribute(xml_element, "filename", filename) != tinyxml2::XML_SUCCESS)
    std::throw_with_nested(std::runtime_error("SignedDistanceField: Missing or failed parsing attribute 'filename'!"));

  std::string scale_string;
  Eigen::Vector3d scale(1, 1, 1);
  if (tesseract::common::QueryStringAttribute(xml_element, "scale", scale_string) == tinyxml2::XML_SUCCESS)
  {
    std::vector<std::string> tokens;
    boost::split(tokens, scale_string, boost::is_any_of(" "), boost::token_compress_on);
    if (tokens.size() != 3 || !tesseract::common::isNumeric(tokens))
      std::throw_with_nested(std::runtime_error("SignedDistanceField: Failed parsing attribute 'scale'!"));

    double sx{ 0 }, sy{ 0 }, sz{ 0 };
    // No need to check return values because the tokens are verified above
    tesseract::common::toNumeric<double>(tokens[0], sx);
    tesseract::common::toNumeric<double>(tokens[1], sy);
    tesseract::common::toNumeric<double>(tokens[2], sz);

    if (!(sx > 0))
      std::throw_with_nested(std::runtime_error("SignedDistanceField: Scale x is not greater than zero!"));

    if (!(sy > 0))
      std::throw_with_nested(std::runtime_error("SignedDistanceField: Scale y is not greater than zero!"));

    if (!(sz > 0))
      std::throw_with_nested(std::runtime_error("SignedDistanceField: Scale z is not greater than zero!"));

    scale = Eigen::Vector3d(sx, sy, sz);
  }

  tesseract::common::Resource::Ptr resource = locator.locateResource(filename);
  if (resource == nullptr)
    std::throw_with_nested(std::runtime_error("SignedDistanceField: Missing resource '" + filename + "'!"));

  const std::vector<std::uint8_t> data = resource->getResourceContents();
  if (data.empty())
    std::throw_with_nested(std::runtime_error("SignedDistanceField: Error importing from '" + filename + "'!"));

  tesseract::geometry::SignedDistanceField::Ptr geom;
  try
  {
    if (hasExtension(filename, ".vdb"))
      geom = tesseract::geometry::readSignedDistanceFieldVDB(data, scale);
    else if (hasExtension(filename, ".nvdb"))
      geom = tesseract::geometry::readSignedDistanceFieldNVDB(data, scale);
    else
      throw std::runtime_error("unsupported file extension");
  }
  catch (...)
  {
    std::throw_with_nested(
        std::runtime_error("SignedDistanceField: Failed parsing field data from '" + filename + "'!"));
  }

  return geom;
}

tinyxml2::XMLElement* writeSignedDistanceField(const tesseract::geometry::SignedDistanceField::ConstPtr& sdf,
                                               tinyxml2::XMLDocument& doc,
                                               const std::string& package_path,
                                               const std::string& filename)
{
  if (sdf == nullptr)
    std::throw_with_nested(std::runtime_error("Signed distance field is nullptr and cannot be converted to XML"));
  tinyxml2::XMLElement* xml_element = doc.NewElement(SIGNED_DISTANCE_FIELD_ELEMENT_NAME.data());
  Eigen::IOFormat eigen_format(Eigen::StreamPrecision, Eigen::DontAlignCols, " ", " ");

  std::string filepath = trailingSlash(package_path) + noLeadingSlash(filename);
  std::vector<std::uint8_t> data;
  if (hasExtension(filename, ".vdb"))
    data = tesseract::geometry::writeSignedDistanceFieldVDB(*sdf);
  else if (hasExtension(filename, ".nvdb"))
    data = tesseract::geometry::writeSignedDistanceFieldNVDB(*sdf);
  else
    throw std::runtime_error("SignedDistanceField: unsupported file extension for '" + filename + "'");
  std::ofstream out(filepath, std::ios::out | std::ios::binary | std::ios::trunc);
  if (!out.is_open())
    std::throw_with_nested(std::runtime_error("Could not open signed distance field file `" + filepath + "`!"));
  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast) byte blob -> char buffer for binary write
  out.write(reinterpret_cast<const char*>(data.data()), static_cast<std::streamsize>(data.size()));
  if (out.fail())
    std::throw_with_nested(std::runtime_error("Could not write signed distance field to file `" + filepath + "`!"));
  out.close();

  xml_element->SetAttribute("filename", makeURDFFilePath(package_path, filename).c_str());

  if (!sdf->getScale().isOnes())
  {
    std::stringstream scale_string;
    scale_string << sdf->getScale().format(eigen_format);
    xml_element->SetAttribute("scale", scale_string.str().c_str());
  }

  return xml_element;
}

}  // namespace tesseract::urdf
